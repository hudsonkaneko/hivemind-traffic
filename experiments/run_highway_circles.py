"""Three native cars on V02 highway: CIRCLES ego and frozen PPO lane traffic."""
from copy import deepcopy
from datetime import datetime,timezone
from pathlib import Path
from types import SimpleNamespace
import argparse
import gc
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import time
import traceback
import uuid

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from experiments.run_loop_hazard_rsl import atomic_json,source_hashes,policy_state_hash
from scripts.fetch_circles_controllers import REVISION


def sources():
    values=source_hashes()
    for name in ('experiments/run_highway_circles.py','experiments/highway-circles-60mph-v1.json',
                 'traffic/highway_circles_probe.py','traffic/circles_control.py',
                 'traffic/namespaced_physx_vehicle.py','usd/physical_fleet_scene.py',
                 'traffic/rendered_physics_session.py','usd/highway_circles_scene.py'):
        values[name]=hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
    return values


def reset_fleet(vehicles,poses,gates,episode):
    import numpy as np
    def array(value):
        if hasattr(value,'detach'): value=value.detach().cpu()
        return np.asarray(value.numpy() if hasattr(value,'numpy') else value)
    reports=[]
    for index,(vehicle,pose,gate) in enumerate(zip(vehicles,poses,gates)):
        before=[array(v._native_state_view.get_transforms()).copy() for v in vehicles]
        view=vehicle._native_state_view
        sample=view.get_transforms()
        if hasattr(sample,'ptr') and hasattr(sample,'device'):
            import warp as wp
            floats=lambda v:wp.array(v,dtype=wp.float32,device=sample.device)
            indices=wp.array([0],dtype=wp.uint32,device=sample.device)
        elif hasattr(sample,'detach'):
            import torch
            floats=lambda v:torch.tensor(v,dtype=torch.float32,device=sample.device)
            indices=torch.tensor([0],dtype=torch.int32,device=sample.device)
        else:
            floats=lambda v:np.asarray(v,dtype=np.float32)
            indices=np.asarray([0],dtype=np.uint32)
        vehicle.physx.set_vehicle_to_rest_state(vehicle.path)
        for wheel in vehicle.wheel_paths:vehicle.physx.set_wheel_rotation_angle(wheel,0.)
        view.set_transforms(floats([pose]),indices)
        view.set_velocities(floats([[0.]*6]),indices)
        gate.reset(episode_id=episode,vehicle_id=vehicle.binding_paths.vehicle_id)
        actual=array(view.get_transforms())[0]
        error=float(np.max(np.abs(actual[:3]-pose[:3])))
        orientation=float(min(np.max(np.abs(actual[3:]-pose[3:])),np.max(np.abs(actual[3:]+pose[3:]))))
        unchanged=all(np.array_equal(before[j],array(v._native_state_view.get_transforms()))
                      for j,v in enumerate(vehicles) if j!=index)
        passed=error<1e-4 and orientation<1e-4 and unchanged and np.max(np.abs(array(view.get_velocities())))<1e-6
        reports.append(dict(vehicle=vehicle.binding_paths.vehicle_id,passed=bool(passed),position_error_m=error,
                            quaternion_error=orientation,other_bodies_unchanged=unchanged))
    if not all(r['passed'] for r in reports):raise RuntimeError('Fleet reset isolation failed')
    return reports


def capture(session,path):
    from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_file
    viewport=get_active_viewport()
    if viewport is None:raise RuntimeError('Native viewport unavailable for evidence')
    viewport.camera_path='/World/Cameras/HighwayProbe'
    viewport.resolution=(1280,720)
    session.publish_transforms()
    for _ in range(12):session.render()
    helper=capture_viewport_to_file(viewport,str(path))
    for _ in range(120):
        session.render()
        if path.is_file() and path.stat().st_size>1000:break
    if not path.is_file():raise RuntimeError('Native screenshot did not complete')
    return dict(file=path.name,physics_tick=session.physics_steps,bytes=path.stat().st_size)


def native(output,cfg):
    result=dict(completed=False,training=False,optimizer_steps=0,cases=[],screenshots=[])
    app=session=settings=None
    vehicles=[];monitors=[];gates=[];policies=[]
    previous_contact=None
    started=time.monotonic()
    try:
        from isaacsim import SimulationApp
        app=SimulationApp({'headless':True,'width':1280,'height':720,'disable_viewport_updates':False})
        import carb
        import torch
        import numpy as np
        sys.path.insert(0,str(ROOT/'runtimes/circles_onnx312'))
        import onnxruntime as ort
        from pxr import Gf,Sdf,Usd,UsdGeom
        from traffic.physics_session import prepare_vehicle_runtime
        from traffic.rendered_physics_session import RenderedPhysicsSession
        from traffic.physx_vehicle import PhysxVehicle
        from traffic.namespaced_physx_vehicle import bind_existing_vehicle
        from traffic.driver_control import AppliedControl,DriverCommand,DriverControlGate
        from traffic.vehicle_contacts import VehicleContactMonitor
        from traffic.path_following import PathFollower,FollowerConfig,PathReference,VehicleState
        from traffic.rsl_loop_policy import RslLoopPolicy
        from traffic.circles_control import acceleration_to_pedals
        from traffic.highway_circles_probe import CEILING,lane_route,lane_only_observation,circles_inputs,guard
        from usd.physical_fleet_scene import author_extra_vehicle,validate_fleet_scene
        from usd.highway_loop_scene import install_highway_loop,spawn_vehicle_on_loop,validate_highway_loop_scene
        from usd.physical_scene import PATHS,asset_hashes
        torch.set_num_threads(4);torch.manual_seed(cfg['seed'])
        result['runtime_setup']=prepare_vehicle_runtime(True)
        result['versions']=dict(python=sys.version,torch=torch.__version__,numpy=np.__version__,onnxruntime=ort.__version__)
        settings=carb.settings.get_settings()
        previous_contact=settings.get('/physics/disableContactProcessing')
        if type(previous_contact) is not bool:raise RuntimeError('Invalid contact-processing setting')
        settings.set_bool('/physics/disableContactProcessing',False)
        session=RenderedPhysicsSession(120,30,physics_scene_path=PATHS.physics)
        template=PhysxVehicle(session.stage,120,scene_directory=output/'scene')
        result['highway']=install_highway_loop(session.stage,output/'scene',ROOT/'highway_usd/_v02')
        initial=spawn_vehicle_on_loop(session.stage,0.)
        z=initial['position_m'][2]
        ids=cfg['vehicle_ids']
        radii=cfg['lane_radii_m']; angles=cfg['start_angles_rad']
        routes=[lane_route(identity,radius,angle,1000.) for identity,radius,angle in zip(ids,radii,angles)]
        poses=[]
        for route in routes:
            p=route.evaluate(0.)
            poses.append([*p.position_xy,z,0.,0.,math.sin(p.yaw_rad/2),math.cos(p.yaw_rad/2)])
        result['fleet_authoring']=[author_extra_vehicle(session.stage,PATHS.vehicle,ids[i],poses[i],output/'scene') for i in (1,2)]
        vehicles=[bind_existing_vehicle(session.stage,'/World/Vehicles/vehicle_'+identity,template.metadata) for identity in ids]
        template=None
        monitors=[VehicleContactMonitor(session.stage,v.path) for v in vehicles]
        gates=[DriverControlGate('initial',identity) for identity in ids]
        result['scene_contract']=validate_fleet_scene(session.stage,[v.binding_paths.vehicle for v in vehicles])
        UsdGeom.Scope.Define(session.stage,'/World/Cameras')
        camera=UsdGeom.Camera.Define(session.stage,'/World/Cameras/HighwayProbe')
        camera.CreateClippingRangeAttr(Gf.Vec2f(.1,3000.))
        matrix=Gf.Matrix4d().SetLookAt(Gf.Vec3d(560,-65,75),Gf.Vec3d(503,10,0),Gf.Vec3d(0,0,1)).GetInverse()
        UsdGeom.Xformable(camera).AddTransformOp().Set(matrix)
        # Export a composed, portable entry before runtime control opinions.
        from usd.highway_circles_scene import export_initial_scene
        entry=export_initial_scene(session.stage,output/'scene')
        reopened=Usd.Stage.Open(str(entry))
        result['saved_scene_contract']=validate_fleet_scene(reopened,[v.binding_paths.vehicle for v in vehicles])
        result['saved_highway_contract']=validate_highway_loop_scene(reopened)
        reopened=None
        static_hashes=asset_hashes(output/'scene')
        runtime_layer=Sdf.Layer.CreateAnonymous('highway-traffic-controls.usda')
        session.stage.GetRootLayer().subLayerPaths=[runtime_layer.identifier]+list(session.stage.GetRootLayer().subLayerPaths)
        session.stage.SetEditTarget(runtime_layer)
        for v in vehicles:v.apply(AppliedControl(0,0,1,'initial',None,False),700,1500)
        result['startup']=session.start()
        for v in vehicles:v.bind_native_state(session.manager.get_physics_simulation_view())
        policies=[RslLoopPolicy(ROOT/cfg['policy'],cfg['policy_sha256'],episode_id='highway',vehicle_id=identity) for identity in ids[1:]]
        result['policy_states_before']=[policy_state_hash(p._actor) for p in policies]
        print('LAB_LANE_TASK_READY',flush=True)
        opts=ort.SessionOptions();opts.intra_op_num_threads=1;opts.inter_op_num_threads=1
        for name in ['scripted_baseline',*cfg['circles_models']]:
            baseline=name=='scripted_baseline'
            net=None if baseline else ort.InferenceSession(str(output/'models'/(name+'.onnx')),sess_options=opts,providers=['CPUExecutionProvider'])
            resets=reset_fleet(vehicles,poses,gates,name)
            followers=[PathFollower({route.route_id:route},name,identity,FollowerConfig()) for route,identity in zip(routes,ids)]
            applied=[AppliedControl(0,0,1,'reset',None,False) for _ in vehicles]
            states=[v.state(refresh_usd=False) for v in vehicles]
            sequence=[0]*3;unsupported=[0]*3;acceleration=0.;observations=[None]*3;contexts=[None]*3
            metrics=[dict(vehicle_id=i,peak_speed_m_s=0.,max_lateral_error_m=0.,progress_m=0.,contacts=0,
                          ceiling_interventions=0,positive_throttle_ticks=0,brake_ticks=0) for i in ids]
            case=dict(name=name,background_controller='scripted' if baseline else 'frozen_PPO_lane_only',
                      resets=resets,complete=False,failure=None,cars=metrics)
            with (output/(name+'-steps.jsonl')).open('x',encoding='utf-8') as stream:
                for tick in range(round(cfg['duration_s']*120)):
                    if time.monotonic()-started>cfg['max_process_seconds']-30:raise TimeoutError('Native operation deadline')
                    if settings.get('/physics/disableContactProcessing') is not False:raise RuntimeError('Contact processing disabled')
                    commands=[None]*3
                    if tick%2==0:
                        model_input,input_context=circles_inputs(states,routes[0])
                        if not baseline and tick%6==0:
                            raw=net.run(None,{net.get_inputs()[0].name:np.asarray([model_input],dtype=np.float32)})[0]
                            if raw.size!=1 or not np.isfinite(raw).all():raise ValueError('Invalid ONNX output')
                            acceleration=float(raw.reshape(-1)[0])
                        for i,(identity,route) in enumerate(zip(ids,routes)):
                            state=VehicleState.from_physics(states[i],episode_id=name,vehicle_id=identity,tick=session.physics_steps)
                            ref=PathReference(name,identity,route.route_id,session.physics_steps,session.physics_steps+24,3.,1000.)
                            scripted=followers[i].command(state,session.physics_steps,ref)
                            if i==0:
                                if baseline:
                                    error=CEILING-state.speed_m_s
                                    throttle,brake=min(1,max(0,.7*error)),min(1,max(0,-.35*error))
                                else:throttle,brake,_=acceleration_to_pedals(model_input[0],acceleration,speed_ceiling=CEILING)
                                action=[scripted.steering_rad,throttle,brake]
                                observations[i]=model_input
                                contexts[i]=dict(**input_context,acceleration=acceleration,steering=followers[i].last_diagnostics)
                            elif baseline:
                                action=[scripted.steering_rad,scripted.throttle,scripted.brake]
                            else:
                                observations[i],contexts[i]=lane_only_observation(states[i],applied[i],radius=radii[i],start_angle=angles[i],tick=tick)
                                action=list(policies[i-1].predict(observations[i]))
                                contexts[i]['policy']=deepcopy(policies[i-1].diagnostics)
                            if states[i]['speed_m_s']>=CEILING:
                                action[1:]=[0.,1.];metrics[i]['ceiling_interventions']+=1
                            if tick<240:action=[0.,0.,1.]
                            commands[i]=DriverCommand(name,identity,sequence[i],session.physics_steps,session.physics_steps+12,*action)
                            sequence[i]+=1
                    controls=[]
                    for i,v in enumerate(vehicles):
                        applied[i]=gates[i].step(tick=session.physics_steps,dt_s=1/120,command=commands[i])
                        controls.append(v.apply(applied[i],700,1500))
                    session.step(publish_usd=False)
                    buffers=session.sim.get_contact_report()
                    report_source=SimpleNamespace(get_contact_report=lambda:buffers)
                    events=[m.sample(report_source,session.physics_steps) for m in monitors]
                    states=[v.state(refresh_usd=False) for v in vehicles]
                    checks=[]
                    for i,state in enumerate(states):
                        unsupported[i]=unsupported[i]+1 if tick>=240 and not all(state['wheel_on_ground']) else 0
                        check=guard(state,routes[i],events[i],unsupported[i],tick+1)
                        if applied[i].is_fallback:check['reason']='command_fallback'
                        checks.append(check)
                        metric=metrics[i]
                        metric['peak_speed_m_s']=max(metric['peak_speed_m_s'],state['speed_m_s'])
                        metric['max_lateral_error_m']=max(metric['max_lateral_error_m'],abs(check['lateral_error_m']))
                        metric['progress_m']=check['progress_m'];metric['final_speed_m_s']=state['speed_m_s']
                        metric['contacts']+=len(events[i])
                        if tick>=240:
                            metric['positive_throttle_ticks']+=int(applied[i].throttle>1e-5)
                            metric['brake_ticks']+=int(applied[i].brake>1e-5)
                    stream.write(json.dumps(dict(tick=tick+1,global_tick=session.physics_steps,states=states,
                        observations=observations,contexts=contexts,controls=controls,contacts=events,checks=checks),allow_nan=False)+'\n')
                    failures=[dict(vehicle_id=ids[i],**c) for i,c in enumerate(checks) if c['reason']]
                    if tick==240 and name=='cra':
                        result['screenshots'].append(capture(session,output/'highway-three-cars.png'))
                    if failures:
                        case['failure']=failures
                        break
                case['ticks']=tick+1
                case['complete']=tick+1==round(cfg['duration_s']*120)
                case['passed']=case['complete'] and not case['failure'] and all(m['progress_m']>5 for m in metrics)
                result['cases'].append(case)
                atomic_json(output/'result.json',result)
                print('HIGHWAY_CASE '+json.dumps(case),flush=True)
        result['policy_states_after']=[policy_state_hash(p._actor) for p in policies]
        result['policies_unchanged']=result['policy_states_after']==result['policy_states_before']
        result['static_assets_unchanged']=asset_hashes(output/'scene')==static_hashes
        result['final_scene_contract']=validate_fleet_scene(session.stage,[v.binding_paths.vehicle for v in vehicles])
        result['completed']=True
    except BaseException as exc:
        result.update(error=f'{type(exc).__name__}: {exc}',traceback=traceback.format_exc())
        traceback.print_exc()
    finally:
        camera=runtime_layer=reopened=template=v=m=vehicle=buffers=report_source=None
        vehicles.clear();monitors.clear();gates.clear();policies.clear();gc.collect()
        if session is not None:
            try:
                session.close()
                result['cleanup']=dict(passed=session.closed and not session.lifecycle[-1].get('context_has_stage')
                    and not session.lifecycle[-1].get('old_stage_cached_after_erase'),lifecycle=session.lifecycle)
            except Exception as exc:result['cleanup']=dict(passed=False,error=repr(exc))
        if settings is not None and type(previous_contact) is bool:
            settings.set_bool('/physics/disableContactProcessing',previous_contact)
            result['contact_setting_restored']=settings.get('/physics/disableContactProcessing') is previous_contact
        result['wall_seconds']=time.monotonic()-started
        atomic_json(output/'result.json',result)
        if app is not None:app.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native',type=Path,help=argparse.SUPPRESS)
    args=parser.parse_args()
    if Path.cwd().resolve()!=ROOT:parser.error('Run from checkout root')
    cfg=json.loads((ROOT/'experiments/highway-circles-60mph-v1.json').read_text())
    from traffic.highway_circles_probe import validate_protocol
    validate_protocol(cfg)
    if args.native:
        output=args.native.resolve()
        if output.parent!=ROOT/'outputs/highway_circles' or not output.is_dir():raise ValueError('Invalid run directory')
        manifest=json.loads((output/'manifest.json').read_text())
        if sources()!=manifest['sources'] or cfg!=manifest['config']:raise ValueError('Changed source or protocol')
        for relative,digest in manifest['inputs'].items():
            if hashlib.sha256((ROOT/relative).read_bytes()).hexdigest()!=digest:raise ValueError('Changed model or highway input')
        (output/'started').open('x').close()
        native(output,cfg)
        return
    from experiments.verify_lane_following_lab import kit_process_snapshot,classify_runtime_log,KNOWN_STARTUP_ERRORS
    from experiments.verify_physics_vehicle import stop_child
    from experiments.probe_support import GpuMonitor,gpu_sample
    from hivemind.launcher import isaac_runtime,load_config
    before=kit_process_snapshot()
    if not before['available'] or before['count']:raise RuntimeError('Existing Kit job; serialize native runs')
    gpu=gpu_sample()
    if gpu['used_mib']/gpu['total_mib']>=.9:raise RuntimeError('GPU memory guard')
    output=ROOT/'outputs/highway_circles'/('v01-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:8])
    output.mkdir(parents=True,exist_ok=False)
    logs=ROOT/'logs/highway_circles'/output.name;logs.mkdir(parents=True,exist_ok=False)
    shutil.copytree(ROOT/'outputs/circles_control/models'/REVISION,output/'models')
    files=[ROOT/cfg['policy'],*sorted((output/'models').glob('*.onnx')),*sorted((ROOT/'highway_usd/_v02').glob('*'))]
    inputs={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files if p.is_file()}
    hashes=sources()
    for relative in hashes:
        dest=output/'source'/relative;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/relative,dest)
    command=[str(isaac_runtime(ROOT,load_config(ROOT))),str(Path(__file__).resolve()),'--native',str(output)]
    atomic_json(output/'manifest.json',dict(config=cfg,sources=hashes,inputs=inputs,command=command,
        git_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True,cwd=ROOT).strip(),
        git_status=subprocess.check_output(['git','status','--short'],text=True,cwd=ROOT)))
    print('RUN_DIRECTORY='+str(output),flush=True)
    started=time.monotonic();stop_reason=None
    with (logs/'runtime.log').open('w') as stream,GpuMonitor() as monitor:
        process=subprocess.Popen(command,cwd=ROOT,env=dict(os.environ,PYTHONUNBUFFERED='1'),stdout=stream,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
        try:
            while process.poll() is None:
                if time.monotonic()-started>cfg['max_process_seconds']:stop_reason='wall_time'
                elif monitor.errors:stop_reason='gpu_monitor'
                elif monitor.rows and monitor.rows[-1]['used_mib']/monitor.rows[-1]['total_mib']>=.9:stop_reason='gpu_memory'
                if stop_reason:stop_child(process);break
                time.sleep(1)
        finally:
            if process.poll() is None:stop_child(process)
    r=json.loads((output/'result.json').read_text()) if (output/'result.json').exists() else {}
    log=classify_runtime_log((logs/'runtime.log').read_text(errors='replace'),KNOWN_STARTUP_ERRORS)
    after=kit_process_snapshot()
    valid=bool(process.returncode==0 and not stop_reason and r.get('completed') and r.get('cleanup',{}).get('passed')
        and r.get('contact_setting_restored') and r.get('policies_unchanged') and r.get('static_assets_unchanged')
        and hashes==sources() and after['available'] and not after['count'] and not log['unexpected_runtime_errors']
        and not log['usd_reference_warning_count'])
    atomic_json(output/'gpu-samples.json',monitor.rows)
    atomic_json(output/'exit.json',dict(evidence_valid=valid,returncode=process.returncode,stop_reason=stop_reason,
        sources_unchanged=hashes==sources(),kit_after=after,log_assessment=log,wall_seconds=time.monotonic()-started))
    print(json.dumps(dict(output=str(output),evidence_valid=valid)),flush=True)
    return 0 if valid else 1


if __name__=='__main__':raise SystemExit(main())
