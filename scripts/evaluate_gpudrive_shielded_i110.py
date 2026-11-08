"""Frozen GPUDrive through the existing native SUMO safety stack; no trainer."""
from __future__ import annotations
import argparse
from collections import Counter
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import numpy as np
import torch
from environments.highway_environment_v05 import HighwayEnvironmentV05Env
from scripts.evaluate_gpudrive_i110 import states,sha,save
from scripts.prepare_highway_flow import capture_frame,route_metrics
from traffic.gpudrive_sumo_bridge import RoadMap
from traffic.gpudrive_pretrained import FrozenGPUDrivePolicy,encode_observation
from traffic.gpudrive_native_adapter import native_request


def validate_fixed_study(config):
    expected=dict(speed_ceiling_m_s=26.8224,horizon_s=80,controlled_vehicles=1,
        total_initial_vehicles=16,traffic_mode='sustained',adapter_lookahead_s=1.,lane_change_duration_s=3.)
    if config.get('training') is not False or any(config.get(k)!=v for k,v in expected.items()):
        raise ValueError('Configuration differs from the fixed no-training v02 study')


def run(config,output,mode):
    validate_fixed_study(config)
    if output.exists():raise ValueError('Fresh output required')
    output.mkdir(parents=True)
    network=ROOT/config['network']
    if sha(network)!=config['network_sha256']:raise ValueError('Network hash mismatch')
    if config['speed_ceiling_m_s']!=26.8224 or config['training'] is not False:
        raise ValueError('Fixed 60mph, no-training evaluation only')
    shutil.copyfile(network,output/'network.net.xml');save(output/'resolved-config.json',config)
    source_names=['scripts/evaluate_gpudrive_shielded_i110.py','scripts/evaluate_gpudrive_i110.py',
        'scripts/prepare_highway_flow.py','scripts/record_highway_environment_v05.py',
        'traffic/gpudrive_native_adapter.py','traffic/gpudrive_pretrained.py','traffic/gpudrive_sumo_bridge.py',
        'environments/highway_environment_v05.py','environments/highway_stage1_env.py',
        'environments/i110_merge_env.py','environments/sumo_mappo_env.py',
        'traffic/highway_merge_context.py','traffic/highway_stage1_teacher.py',
        'traffic/highway_v05_demand.py','traffic/highway_v05_metrics.py']
    hashes={p:sha(ROOT/p) for p in source_names}
    for p in source_names:
        target=output/'source'/p;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/p,target)
    torch.set_num_threads(1);torch.manual_seed(config['seed']);torch.use_deterministic_algorithms(True)
    model=FrozenGPUDrivePolicy.from_pretrained(ROOT/config['policy_directory']) if mode=='gpudrive' else None
    road=RoadMap(network,config['road_segment_length_m']);goal=tuple(road.lanes[config['goal_lane']]['points'][-1])
    env=HighwayEnvironmentV05Env(network,output/'simulation',controlled_count=1,total_vehicles=16,
        horizon=800,through_lanes=4,task='merge',density='medium',difficulty=2,
        speed_limit_m_s=config['speed_ceiling_m_s'],traffic_mode='sustained',
        communication='none',packet_loss=0.,control_target_version='v3',
        lane_change_duration=3.,stabilization_seconds=8.)
    begun=time.monotonic();frames=[];actions=[];observations=[];events=[];causes=Counter();error=None
    interventions=0;range_clips=0;native_changes=0;lane_rejections=0;report={}
    try:
        env.reset(seed=config['seed']);env.control_mode='policy' if model else 'native'
        assert env.agents==['agent_0']
        env.conn.vehicle.setMaxSpeed('agent_0',config['speed_ceiling_m_s'])
        env.conn.vehicle.setSpeedMode('agent_0',31)
        env.conn.vehicle.setLaneChangeMode('agent_0',512 if model else 1621)
        if not model:env.conn.vehicle.setSpeed('agent_0',-1)
        frames.append(capture_frame(env))
        while env.agents:
            if time.monotonic()-begun>config['max_wall_seconds']:raise TimeoutError('Evaluation time budget')
            if model:
                cars=states(env.conn);ego=next(c for c in cars if c['id']=='agent_0')
                others=sorted((c for c in cars if c['id']!='agent_0'),
                    key=lambda c:((c['x']-ego['x'])**2+(c['y']-ego['y'])**2,c['id']))
                observation=encode_observation(ego,others,road.visible_roads(ego),goal)
                accel,steer,index=model.infer(observation)
                action,detail=native_request(float(accel),float(steer),ego['speed'],env.states['agent_0']['lane_width'])
                detail.update(index=int(index),time=env.time)
                observations.append(observation)
            else:action=np.asarray([0.,0.],np.float32)
            _,_,_,_,infos=env.step({'agent_0':action})
            info=infos['agent_0'];frames.append(capture_frame(env))
            if info.get('merge_completed'):events.append(dict(agent='agent_0',time=env.time,kind='native_merge'))
            if model:
                changed=bool(detail['acceleration_range_clipped'] or info['override'])
                interventions+=changed;range_clips+=detail['acceleration_range_clipped']
                native_changes+=info.get('native_realization_changed',False);lane_rejections+=info.get('lane_rejected',False)
                causes.update(info.get('override_causes',[]))
                if detail['acceleration_range_clipped']:causes['acceleration_range']+=1
                actions.append(dict(detail,combined_intervention=changed,native_result=info))
        # Retain equal primary horizons; no native post-horizon drain credit.
        while env.time<env.start_time+80.-1e-8:
            if time.monotonic()-begun>config['max_wall_seconds']:raise TimeoutError('Accounting time budget')
            env.step({});frames.append(capture_frame(env))
        report=env.report()
    except Exception as exc:
        error=str(exc);report=env.report() if env.conn else {}
    finally:env.close()
    focal=[v for f in frames for v in f['vehicles'] if v['id']=='agent_0']
    accounting=report.get('accounting',{});metric=route_metrics(frames,{'agent_0'})
    report.update(training=False,optimizer_updates=0,controller=mode,seed=config['seed'],error=error,
        wall_seconds=time.monotonic()-begun,global_speed_ceiling_m_s=26.8224,
        maximum_ego_speed_m_s=max((v['speed'] for v in focal),default=0.),
        global_ceiling_exceedance_steps=sum(v['speed']>26.8224+1e-6 for v in focal),
        raw_actor_decisions=len(actions),combined_policy_intervention_fraction=interventions/max(1,len(actions)) if model else None,
        combined_policy_intervention_steps=interventions,acceleration_range_clips=range_clips,
        native_realization_change_steps=native_changes,rejected_lane_requests=lane_rejections,
        combined_intervention_causes=dict(causes),route_measurements=metric,
        motion_owner='SUMO for every vehicle; no external pose writes',
        adapter='One-second lateral-slip estimate; half-lane threshold -> lane intent; native 3-second maneuver',
        comparison_scope='Matched v02 native background/demand; v01 uses different dynamics and demand and is historical only')
    save(output/'evaluation.json',report)
    save(output/'recording.json',dict(schema='gpudrive_i110_shielded_v02',controller=mode,training=False,
        optimizer_steps=0,policy_mode='frozen GPUDrive with native safety and lane-intent adapter' if model else 'native baseline',
        policy_sha256=config['policy_sha256'] if model else None,source_timestep=.1,
        frames=frames,merge_events=events,controlled_agents=['agent_0'],seed=config['seed'],
        network_sha256=sha(network),source_sha256=hashes,motion_owner='SUMO native'))
    with gzip.open(output/'actor-actions.jsonl.gz','wt',encoding='utf-8') as f:
        for a in actions:f.write(json.dumps(a,allow_nan=False)+'\n')
    if observations:np.savez_compressed(output/'actor-observations.npz',observations=np.stack(observations))
    demand=list((output/'simulation').rglob('routes.rou.xml'))
    save(output/'manifest.json',dict(config=config,training=False,optimizer_steps=0,
        git_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        source_sha256=hashes,source_unchanged=all(sha(ROOT/p)==h for p,h in hashes.items()),
        demand_sha256={p.relative_to(output).as_posix():sha(p) for p in demand},
        python=sys.version,torch=torch.__version__,numpy=np.__version__,
        checkpoint_sha256=sha(ROOT/config['policy_directory']/'model.safetensors'),
        recording_sha256=sha(output/'recording.json'),evaluation_sha256=sha(output/'evaluation.json')))
    print(json.dumps({k:report.get(k) for k in ('error','wall_seconds','successful_merges','collisions',
        'teleports','maximum_ego_speed_m_s','global_ceiling_exceedance_steps','intervention_fraction',
        'combined_policy_intervention_fraction','rejected_lane_requests','completed_trips')}))
    return int(error is not None)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--mode',choices=['native','gpudrive'],required=True)
    a=p.parse_args()
    if Path.cwd().resolve()!=ROOT or not a.output.resolve().is_relative_to(ROOT/'outputs'):
        raise ValueError('Canonical root and project output required')
    return run(json.loads(a.config.read_text(encoding='utf-8')),a.output.resolve(),a.mode)


if __name__=='__main__':raise SystemExit(main())
