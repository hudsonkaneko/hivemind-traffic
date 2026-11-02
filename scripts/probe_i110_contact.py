"""Bounded physical road-contact checks; no training or kinematic vehicle motion.

PhysX owns the car after spawn. Map-based pure pursuit applies only steering and
wheel torque. The factory's infinite plane is disabled and tire hit identities
must point to the imported road. This is a surrogate-car engineering test.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import traceback

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import numpy as np


def paths_from_geometry(data):
    from usd.highway_event_layer import route_pose
    road=data['meshes']['Southbound']
    paths={}
    for lane in range(4):
        positions=[]
        for s,w in zip(road['station_m'],road['widths_m']):
            p,_=route_pose(road,s,w/2-1.6-3.7*(lane+.5));positions.append(p)
        paths['main_'+str(lane)]=np.array(positions)
    for name,key in [('MlkEntrance','entrance'),('VernonExit','exit')]:
        paths[key]=np.array(data['meshes'][name]['center_xyz'])
    # Route stitching is explicit and occurs inside the common pavement.
    main=paths['main_3']
    entrance=paths['entrance'];tail=main[main[:,1]<entrance[-1,1]-10]
    paths['entrance_join']=np.concatenate([entrance,tail])
    exit_path=paths['exit'];head=main[main[:,1]>exit_path[0,1]+10]
    paths['exit_join']=np.concatenate([head,exit_path])
    return paths


def run_case(case,paths,scene,output,started,deadline):
    from pxr import Gf,Usd,UsdGeom,UsdPhysics,UsdShade,PhysxSchema
    from traffic.physics_session import PhysicsSession
    from traffic.physx_vehicle import PhysxVehicle
    from traffic.driver_control import AppliedControl
    from traffic.vehicle_contacts import VehicleContactMonitor
    from usd.physical_scene import PATHS
    from omni.physx.bindings import _physx as constants
    name=case['id'];out=output/name;out.mkdir()
    route=paths[case['route']]; chain=np.r_[0,np.cumsum(np.linalg.norm(np.diff(route[:,:2],axis=0),axis=1))]
    start=float(case.get('start_m',10));end=float(min(case.get('end_m',chain[-1]-10),chain[-1]-10))
    p=np.array([np.interp(start,chain,route[:,k]) for k in range(3)])
    i=np.clip(np.searchsorted(chain,start),1,len(route)-1);tangent=route[i,:2]-route[i-1,:2]
    yaw=math.atan2(tangent[1],tangent[0]); rows=[]; contacts=[];vehicle=None;monitor=None
    session=PhysicsSession(120);last=None;max_error=0;steer=0
    support='/World/Environment/Highway/Surface'
    try:
        vehicle=PhysxVehicle(session.stage,120,scene_directory=out/'fixture')
        stage=session.stage
        stage.SetEditTarget(stage.GetSessionLayer())
        stage.GetPrimAtPath(PATHS.ground).SetActive(False)
        road=stage.DefinePrim('/World/Environment/Highway')
        road.GetReferences().AddReference(str((scene/'road-support.usda').resolve()))
        UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(support)).Bind(
            UsdShade.Material(stage.GetPrimAtPath('/World/Physics/Resources/TarmacMaterial')),
            UsdShade.Tokens.weakerThanDescendants,'physics')
        group=UsdPhysics.CollisionGroup(stage.GetPrimAtPath('/World/Physics/Resources/GroundSurfaceCollisionGroup'))
        group.GetCollidersCollectionAPI().GetIncludesRel().AddTarget(support)
        if case.get('negative_control'):road.SetActive(False)
        chassis=stage.GetPrimAtPath(vehicle.path);xf=UsdGeom.Xformable(chassis);xf.ClearXformOpOrder()
        transform=Gf.Matrix4d(1);transform.SetRotate(Gf.Rotation(Gf.Vec3d(0,0,1),math.degrees(yaw)))
        transform.SetTranslateOnly(Gf.Vec3d(float(p[0]),float(p[1]),float(p[2]+1.05)))
        xf.AddTransformOp().Set(transform)
        PhysxSchema.PhysxRigidBodyAPI.Apply(chassis).CreateSleepThresholdAttr(0.)
        monitor=VehicleContactMonitor(stage,vehicle.path)
        if stage.GetCompositionErrors():raise RuntimeError(str(stage.GetCompositionErrors()))
        (out/'vehicle-model.json').write_text(json.dumps(vehicle.metadata,indent=2))
        session.attach();dt=1/120
        max_seconds=case.get('seconds',2+(end-start)/max(.5,case['speed'])+35)
        progress=start
        for tick in range(math.ceil(max_seconds/dt)):
            if time.perf_counter()-started>deadline-12:raise TimeoutError('Probe deadline')
            if last is not None:
                point=np.array(last['position_m'][:2]);delta=route[:,:2]-point
                near=int(np.argmin(np.sum(delta*delta,axis=1)))
                progress=float(chain[near]);max_error=max(max_error,float(np.min(np.linalg.norm(delta,axis=1))))
                lookahead=max(5.,last['speed_m_s']*.9)
                goal=np.array([np.interp(min(end,progress+lookahead),chain,route[:,k]) for k in range(2)])
                d=goal-point;angle=math.atan2(d[1],d[0])-last['yaw_rad']
                requested=math.atan2(2*3.2*math.sin(angle),max(1.,np.linalg.norm(d)))
                steer+=float(np.clip(requested-steer,-.5*dt,.5*dt))
            target=0 if tick<240 else case['speed']
            speed=0 if last is None else last['speed_m_s']
            throttle=float(np.clip((target-speed)*.35,0,1));brake=float(np.clip((speed-target)*.35,0,1))
            if target==0:throttle=0;brake=1
            vehicle.apply(AppliedControl(float(np.clip(steer,-.45,.45)),throttle,brake,'bounded_map_contact_probe',tick,False),700,1500)
            session.step(dt,tick*dt);last=vehicle.state()
            hits=[];jounce=[];ground_points=[]
            for path in vehicle.wheel_paths:
                state=vehicle.physx.get_wheel_state(path)
                hits.append(state[constants.VEHICLE_WHEEL_STATE_GROUND_SHAPE] if state[constants.VEHICLE_WHEEL_STATE_IS_ON_GROUND] else None)
                jounce.append(float(state[constants.VEHICLE_WHEEL_STATE_SUSPENSION_JOUNCE]))
                ground_points.append(list(state[constants.VEHICLE_WHEEL_STATE_GROUND_HIT_POSITION]) if hits[-1] else None)
            found=monitor.sample(session.sim,tick+1);contacts.extend(found)
            row=dict(tick=tick+1,time_s=(tick+1)*dt,progress_m=progress,ground_shapes=hits,
                     suspension_jounce_m=jounce,ground_points=ground_points,**last)
            rows.append(row)
            if case.get('negative_control'):continue
            expected_road_z=float(np.interp(progress,chain,route[:,2]))
            if tick>240 and (last['upright_z']<.9 or last['position_m'][2]<expected_road_z-2):
                raise RuntimeError('Unstable car or fall-through')
            if case['speed']>0 and tick>240 and progress>=end:break
        active=rows[240:] if len(rows)>240 else rows
        grounded=sum(all(r['wheel_on_ground']) for r in active)/max(1,len(active))
        hit_ids=sorted({h for r in active for h in r['ground_shapes'] if h})
        chassis_contacts=[r for r in contacts if r.get('contact_count',0)>0]
        gates=dict(all_wheels_grounded=grounded>=.995,correct_surface=hit_ids==[support],
                   upright=min(r['upright_z'] for r in active)>=.95,
                   completion=case['speed']==0 or progress>=end,
                   no_chassis_contacts=not chassis_contacts)
        if case.get('negative_control'):
            gates=dict(no_hidden_ground=not hit_ids and grounded==0,
                       falls_without_surface=rows[-1]['position_m'][2]<rows[0]['position_m'][2]-1)
        result=dict(case=case,passed=all(gates.values()),gates=gates,grounded_fraction=grounded,
                    contact_shapes=hit_ids,final_progress_m=progress,requested_end_m=end,
                    peak_speed_m_s=max(r['speed_m_s'] for r in rows),
                    minimum_upright_z=min(r['upright_z'] for r in active),chassis_contacts=len(chassis_contacts),
                    samples=len(rows),negative_control=bool(case.get('negative_control')))
        return result
    finally:
        (out/'trajectory.json').write_text(json.dumps(rows,separators=(',',':')))
        (out/'contacts.json').write_text(json.dumps(contacts,indent=2))
        monitor=None;vehicle=None;stage=None;chassis=None;xf=None;road=None;group=None
        session.close()
        (out/'lifecycle.json').write_text(json.dumps(session.lifecycle,indent=2))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('geometry','scene','config','output'):p.add_argument('--'+key,type=Path,required=True)
    args=p.parse_args();args.output.mkdir(exist_ok=False)
    config=json.loads(args.config.read_text());(args.output/'resolved-config.json').write_text(json.dumps(config,indent=2))
    started=time.perf_counter();app=None;result=dict(passed=False,cases=[],no_training=True)
    paths_to_hash=[args.geometry,args.config]+list(args.scene.glob('*.usda'))
    hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths_to_hash}
    try:
        from isaacsim import SimulationApp
        app=SimulationApp({'headless':True,'disable_viewport_updates':True})
        from traffic.physics_session import prepare_vehicle_runtime
        result['runtime']=prepare_vehicle_runtime()
        data=json.loads(args.geometry.read_text());paths=paths_from_geometry(data)
        (args.output/'routes.json').write_text(json.dumps({k:v.tolist() for k,v in paths.items()}))
        for case in config['cases']:
            report=run_case(case,paths,args.scene,args.output,started,config['max_wall_s'])
            result['cases'].append(report)
            print('I110_CASE='+json.dumps(report),flush=True)
            (args.output/'result.json').write_text(json.dumps(result,indent=2))
        result['passed']=all(c['passed'] for c in result['cases'])
    except Exception as e:
        result['error']=repr(e);result['traceback']=traceback.format_exc();traceback.print_exc()
    finally:
        result['source_sha256']=hashes
        result['sources_unchanged']=all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in hashes.items())
        result['elapsed_s']=time.perf_counter()-started
        (args.output/'result.json').write_text(json.dumps(result,indent=2))
        if app:app.close(skip_cleanup=False)
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
