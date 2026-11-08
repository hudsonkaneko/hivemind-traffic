"""One frozen pretrained actor in I-110 with reactive SUMO traffic; no trainer."""
from __future__ import annotations
import argparse
from copy import deepcopy
import gzip
import hashlib
import json
import math
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import numpy as np
import torch
import traci
from traffic.gpudrive_sumo_bridge import RoadMap, bicycle_step, center_from_front, front_from_center, overlaps
from traffic.sumo_backend import find_sumo_binary


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def save(path, value): Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')


def states(conn):
    result=[]
    for identity in sorted(conn.vehicle.getIDList()):
        api=conn.vehicle
        x,y,z=api.getPosition3D(identity); length=api.getLength(identity)
        cx,cy,yaw=center_from_front(x,y,api.getAngle(identity),length)
        result.append(dict(id=identity,x=cx,y=cy,z=z,yaw=yaw,speed=api.getSpeed(identity),
            acceleration=api.getAcceleration(identity),length=length,width=api.getWidth(identity),
            lane_id=api.getLaneID(identity),edge_id=api.getRoadID(identity),distance_m=api.getDistance(identity),
            lane_position=api.getLanePosition(identity),route=api.getRoute(identity),
            allowed_speed=api.getAllowedSpeed(identity)))
    return result


def frame(cars, stamp, ego, traveled, merged):
    vehicles=[]
    for original in cars:
        car=ego if ego is not None and original['id']=='agent_0' else original
        x,y,heading=front_from_center(car)
        lane=original['lane_id']; edge=lane.rsplit('_',1)[0]
        kind='ramp' if edge=='mlk_on' else 'auxiliary' if lane=='sb2_0' else 'through'
        vehicles.append(dict(id=original['id'],x=x,y=y,z=original['z'],heading=heading,
            speed=car['speed'],acceleration=car['acceleration'],length=car['length'],width=car['width'],
            lane_id=lane,lane_identity=dict(kind=kind),controlled=original['id']=='agent_0',
            distance_m=traveled if ego is not None and original['id']=='agent_0' else original['distance_m'],
            merge_complete=merged if original['id']=='agent_0' else False))
    return dict(time=stamp,vehicles=vehicles)


def run(config, output, mode):
    if output.exists(): raise ValueError('Preserve existing run outputs')
    output.mkdir(parents=True)
    network=ROOT/config['network']; routes=ROOT/config['routes']
    for path,digest in ((network,config['network_sha256']),(routes,config['routes_sha256'])):
        if sha(path)!=digest: raise ValueError('Input checksum mismatch')
    shutil.copyfile(network,output/'network.net.xml');shutil.copyfile(routes,output/'routes.rou.xml')
    road=RoadMap(network,config['road_segment_length_m'])
    guarded=config.get('continuous_safety',False) and mode=='gpudrive'
    repair_route=config.get('repair_destination_route',False) and mode=='gpudrive'
    if repair_route and not guarded:
        raise ValueError('Continuous route repair requires the explicit safety guards')
    if guarded:
        from traffic.gpudrive_continuous_safety import (guarded_step, current_constraints,
            recover_lane_state, restore_destination_route)
        from traffic.highway_merge_context import Stage1LaneGraph
        graph=Stage1LaneGraph(network)
    goal=tuple(road.lanes[config['goal_lane']]['points'][-1])
    destination_edge=road.lanes[config['goal_lane']]['edge']
    policy=None
    if mode=='gpudrive':
        from traffic.gpudrive_pretrained import FrozenGPUDrivePolicy, encode_observation
        policy=FrozenGPUDrivePolicy.from_pretrained(ROOT/config['policy_directory'])
    torch.set_num_threads(1);torch.manual_seed(config['seed']);np.random.seed(config['seed'])
    torch.use_deterministic_algorithms(True)
    command=[str(find_sumo_binary()),'-n',str(network),'-r',str(routes),'--seed',str(config['seed']),
        '--step-length','.1','--collision.action','warn','--collision.check-junctions','true',
        '--time-to-teleport','-1','--no-step-log','true','--duration-log.disable','true',
        '--log',str(output/'sumo.log'),'--tripinfo-output',str(output/'tripinfo.xml')]
    started=time.monotonic();conn=None;frames=[];actions=[];observations=[];events=[];error=None
    evaluation=dict(mode=mode,training=False,optimizer_updates=0,seed=config['seed'],
        requested_horizon_s=config['horizon_s'],policy_safety_override_fraction=None,
        safety_override_note='No existing MAPPO safety shield is applied to externally owned continuous ego motion.',
        contacts=[],native_collision_events=[],teleports=[],completed_trips=0,
        speed_limit_exceedance_steps=0,reverse_speed_steps=0,pose_readback_max_error_m=0.,merge_completed=False,
        terminal_reason='horizon',merge_time_s=None)
    ego=None;traveled=0.;merge_since=None
    source_names=['scripts/evaluate_gpudrive_i110.py','traffic/gpudrive_sumo_bridge.py','traffic/gpudrive_pretrained.py']
    if guarded:
        source_names+=['traffic/gpudrive_continuous_safety.py','traffic/highway_merge_context.py',
                       'traffic/i110_policy_interface.py','environments/i110_merge_env.py',
                       'environments/highway_stage1_env.py']
        evaluation.update(policy_safety_override_fraction=0., safety_rejections=[],
            geometric_lane_recovery_steps=0,
            safety_override_note='Continuous steering preserved; shared longitudinal bounds, hard speed limits, '
            'live minor-link yield checks and 20 ms footprint sampling. Unsafe proposals terminate this test. '
            'Not the complete native SUMO/MAPPO safety stack; background prediction is constant velocity.')
    source_hashes={p:sha(ROOT/p) for p in source_names}
    if repair_route:
        evaluation.update(destination_edge=destination_edge,route_repairs=[],route_repair_failures=[],
                          move_to_xy_keep_route=2)
    try:
        traci.start(command,label=output.name)
        conn=traci.getConnection(output.name)
        evaluation['sumo_version']=conn.getVersion()
        for lane in conn.lane.getIDList():
            conn.lane.setMaxSpeed(lane,min(conn.lane.getMaxSpeed(lane),config['scenario_speed_ceiling_m_s']))
        conn.simulationStep();cars=states(conn);ego=deepcopy(next(c for c in cars if c['id']=='agent_0'))
        if mode=='gpudrive':
            conn.vehicle.setSpeedMode('agent_0',0);conn.vehicle.setLaneChangeMode('agent_0',0)
        frames.append(frame(cars,conn.simulation.getTime(),ego if policy else None,traveled,False))
        while conn.simulation.getTime()<config['horizon_s']:
            if time.monotonic()-started>config['max_wall_seconds']: raise TimeoutError('Bounded evaluation deadline')
            if policy:
                others=[c for c in cars if c['id']!='agent_0']
                others.sort(key=lambda c:((c['x']-ego['x'])**2+(c['y']-ego['y'])**2,c['id']))
                observation=encode_observation(ego,others,road.visible_roads(ego),goal)
                accel,steering,index=policy.infer(observation)
                audit=None
                if guarded:
                    next_ego,audit=guarded_step(ego,float(accel),float(steering),road,others,
                        **current_constraints(conn,graph,ego,cars))
                else:
                    next_ego=bicycle_step(ego,float(accel),float(steering))
                observations.append(observation)
                actions.append(dict(time=conn.simulation.getTime(),index=int(index),acceleration=float(accel),
                    steering_rad=float(steering),ego_before=deepcopy(ego),
                    ego_after=deepcopy(next_ego) if not audit or audit['accepted'] else None,
                    proposed_ego=deepcopy(next_ego),safety=audit))
                if audit and not audit['accepted']:
                    evaluation['safety_rejections'].append(dict(time=conn.simulation.getTime(),**audit))
                    evaluation['terminal_reason']='unsafe_proposal_rejected'
                    break
                traveled+=math.hypot(next_ego['x']-ego['x'],next_ego['y']-ego['y'])
                fx,fy,heading=front_from_center(next_ego)
                conn.vehicle.setSpeed('agent_0',max(0.,next_ego['speed']))
                conn.vehicle.moveToXY('agent_0','',-1,fx,fy,angle=heading,
                    keepRoute=2 if repair_route else 3,matchThreshold=100.)
                ego=next_ego
            conn.simulationStep();stamp=conn.simulation.getTime();cars=states(conn)
            evaluation['completed_trips']+=len(conn.simulation.getArrivedIDList())
            evaluation['native_collision_events']+=list(conn.simulation.getCollidingVehiclesIDList())
            evaluation['teleports']+=list(conn.simulation.getStartingTeleportIDList())
            actual=next((c for c in cars if c['id']=='agent_0'),None)
            if actual is None:
                evaluation['terminal_reason']='native_arrival' if 'agent_0' in conn.simulation.getArrivedIDList() else 'ego_missing'
                if repair_route and evaluation['terminal_reason']=='native_arrival' and (
                    math.hypot(ego['x']-goal[0],ego['y']-goal[1])>=config['goal_tolerance_m']):
                    evaluation['terminal_reason']='premature_native_arrival'
                break
            if policy:
                err=math.hypot(actual['x']-ego['x'],actual['y']-ego['y'])
                evaluation['pose_readback_max_error_m']=max(evaluation['pose_readback_max_error_m'],err)
                if err>1e-3: raise RuntimeError('SUMO moved or snapped externally owned ego pose')
                if repair_route:
                    try:
                        repaired,event=restore_destination_route(conn,actual,destination_edge)
                        actual.update(repaired)
                        if event:
                            evaluation['route_repairs'].append(dict(time=stamp,**event))
                    except ValueError as exc:
                        evaluation['route_repair_failures'].append(dict(time=stamp,error=str(exc),
                            edge=actual['edge_id'],lane=actual['lane_id'],route=list(actual['route'])))
                        evaluation['terminal_reason']='destination_route_unavailable'
                        frames.append(frame(cars,stamp,ego,traveled,evaluation['merge_completed']))
                        break
                for key in ('lane_id','edge_id','lane_position','route','allowed_speed'):
                    ego[key]=actual[key]
                if guarded and (ego['lane_id'] not in graph.lanes or ego['lane_position']<0):
                    try:
                        ego=recover_lane_state(ego,road,graph)
                        evaluation['geometric_lane_recovery_steps']+=1
                    except ValueError:
                        evaluation['terminal_reason']='lane_association_unavailable'
                        frames.append(frame(cars,stamp,ego,traveled,evaluation['merge_completed']))
                        break
            else: ego=deepcopy(actual)
            lane=ego['lane_id']; in_main=lane.startswith(('sb2_','sb3_','sb4_','sb5_')) and lane!='sb2_0'
            if in_main and abs(ego['speed'])>1.:
                merge_since=stamp if merge_since is None else merge_since
                if stamp-merge_since>=3. and not evaluation['merge_completed']:
                    evaluation['merge_completed']=True;evaluation['merge_time_s']=stamp
                    events.append(dict(agent='agent_0',time=stamp,kind='three_second_mainline_occupancy'))
            else: merge_since=None
            frames.append(frame(cars,stamp,ego if policy else None,traveled,evaluation['merge_completed']))
            if ego.get('signed_step_speed',ego['speed'])<0: evaluation['reverse_speed_steps']+=1
            if lane in road.lanes and ego['speed']>min(road.lanes[lane]['speed'],config['scenario_speed_ceiling_m_s'])+.1:
                evaluation['speed_limit_exceedance_steps']+=1
            contact=[c['id'] for c in cars if c['id']!='agent_0' and overlaps(ego,c)]
            if contact:
                evaluation['contacts'].append(dict(time=stamp,vehicles=contact));evaluation['terminal_reason']='contact';break
            if road.offroad(ego): evaluation['terminal_reason']='offroad';break
            if math.hypot(ego['x']-goal[0],ego['y']-goal[1])<config['goal_tolerance_m']:
                evaluation['terminal_reason']='goal_reached';break
    except Exception as exc:
        error=str(exc);evaluation['terminal_reason']='runtime_error';evaluation['error']=error
    finally:
        if conn: conn.close()
    trace=[v for f in frames for v in f['vehicles'] if v['id']=='agent_0']
    speeds=[v['speed'] for v in trace];accels=[v['acceleration'] for v in trace]
    if guarded:
        changed=sum(a['safety']['acceleration_changed'] or not a['safety']['accepted'] for a in actions)
        evaluation.update(policy_safety_override_fraction=changed/max(1,len(actions)),
            safety_changed_decisions=changed,accepted_decisions=sum(a['safety']['accepted'] for a in actions),
            steering_reinterpretations=0)
    evaluation.update(simulated_duration_s=frames[-1]['time'] if frames else 0.,wall_seconds=time.monotonic()-started,
        goal_reached=evaluation['terminal_reason'] in ('goal_reached','native_arrival'),
        minimum_speed_m_s=min(speeds,default=None),mean_speed_m_s=float(np.mean(speeds)) if speeds else None,
        maximum_speed_m_s=max(speeds,default=None),
        distance_m=trace[-1]['distance_m'] if trace else 0.,
        stopped_exposure_s=.1*sum(abs(s)<.1 for s in speeds[1:]),
        peak_abs_jerk_m_s3=max((abs(b-a)/.1 for a,b in zip(accels,accels[1:])),default=0.),
        actor_decisions=len(actions),sampling='deterministic argmax',
        collision_geometry='Full vehicle footprint; GPUDrive training used reduced collision boxes.',
        baseline_comparison='Native traffic baseline; historical eight-controller runs are not paired comparisons.')
    save(output/'evaluation.json',evaluation)
    recording=dict(schema='gpudrive_i110_transfer_v01',controller=mode,training=False,optimizer_steps=0,
        policy_mode='frozen GPUDrive continuous-action transfer' if policy else 'SUMO native baseline',
        policy_sha256=config['policy_sha256'] if policy else None,checkpoint_sha256=config['policy_sha256'] if policy else None,
        source_timestep=.1,frames=frames,merge_events=events,controlled_agents=['agent_0'],
        seed=config['seed'],network_sha256=sha(network),source_sha256=source_hashes,
        motion_owner='GPUDrive classic bicycle ego / reactive SUMO background' if policy else 'SUMO all cars')
    save(output/'recording.json',recording);save(output/'resolved-config.json',config)
    with gzip.open(output/'actions.jsonl.gz','wt',encoding='utf-8') as f:
        for action in actions:f.write(json.dumps(action)+'\n')
    if observations:np.savez_compressed(output/'observations.npz',observations=np.stack(observations))
    for name in source_names:
        snapshot=output/'source'/name;snapshot.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(ROOT/name,snapshot)
    save(output/'manifest.json',dict(training=False,optimizer_steps=0,config=config,command=command,
        git_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        git_diff_sha256=hashlib.sha256(subprocess.check_output(['git','diff','HEAD'])).hexdigest(),
        source_sha256=source_hashes,python=sys.version,torch=torch.__version__,numpy=np.__version__,
        platform=platform.platform(),checkpoint_sha256=config['policy_sha256'],
        outputs={p.name:sha(p) for p in output.iterdir() if p.is_file()}))
    print(json.dumps(evaluation))
    return 1 if error else 0


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--mode',choices=['native','gpudrive'],required=True)
    a=p.parse_args()
    if Path.cwd().resolve()!=ROOT or not a.output.resolve().is_relative_to(ROOT/'outputs'):
        raise ValueError('Use canonical workspace and an output beneath outputs')
    cfg=json.loads(a.config.read_text(encoding='utf-8'))
    if cfg['training'] is not False:raise ValueError('This test cannot train')
    return run(cfg,a.output.resolve(),a.mode)


if __name__=='__main__':raise SystemExit(main())
