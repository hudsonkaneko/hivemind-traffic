"""Frozen SUMO trajectory replay with the local v14 sensor vehicle; no training."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recording', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=int, default=60)
    args = parser.parse_args()
    output = args.output.resolve()
    if args.seconds <= 0:
        parser.error('seconds must be positive')
    if Path.cwd().resolve() != ROOT or not output.is_relative_to(ROOT/'outputs') or output.exists():
        parser.error('Use checkout root and a fresh output directory')
    output.mkdir(parents=True)
    source = ROOT/'vehicles/sim_ready/americano-i7-ev/v14'
    from usd.showcase_vehicle import package_hashes
    hashes = package_hashes(source)
    data = json.loads((args.recording/'recording.json').read_text())
    model = json.loads((source/'model.json').read_text())
    frames = data['frames'][:args.seconds*10]
    if len(frames) != args.seconds*10: raise ValueError('Recording is shorter than requested replay')
    scene = output/'scene'; scene.mkdir()
    shutil.copytree(source/'assets',scene/'prepared-assets')
    snapshot=output/'source';snapshot.mkdir()
    source_hashes={}
    for name in ('scripts/mappo_sensor_replay.py','scripts/replay_export.py','traffic/native_sensor_capture.py','traffic/physical_lidar.py','traffic/rendered_physics_session.py'):
        source_file=ROOT/name
        target=snapshot/name;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(source_file,target)
        source_hashes[name]=hashlib.sha256(source_file.read_bytes()).hexdigest()
    for name in ('recording.json', 'network.net.xml'):
        shutil.copyfile(args.recording/name, scene/name)
    config = dict(source_vehicle=str(source.relative_to(ROOT)),recording=str(args.recording),
        seconds=args.seconds,render_hz=20,camera_save_hz=2,lidar_save_hz=2,
        camera_resolution=[640,360],motion_owner='recorded SUMO',training=False,
        source_hashes=hashes,code_sha256=source_hashes,
        git_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        git_status=subprocess.check_output(['git','status','--short'],text=True),
        trajectory_sha256=hashlib.sha256((args.recording/'recording.json').read_bytes()).hexdigest(),
        pose_sampling='10 Hz recorded poses held between updates; no deskew',
        policy_sha256=data['checkpoint_sha256'],seed=data['seed'])
    (output/'resolved-config.json').write_text(json.dumps(config,indent=2))
    app=session=None; cameras={}; lidars={}; records=[]; timings=[]
    result=dict(passed=False,training=False,physical_control=False)
    started=time.perf_counter()
    try:
        import numpy as np
        from PIL import Image
        from isaacsim import SimulationApp
        app=SimulationApp(dict(headless=True,width=1280,height=720,enable_motion_bvh=True,
            renderer='RealTimePathTracing',disable_viewport_updates=False,
            create_new_stage=False,limit_cpu_threads=8))
        from pxr import Usd,UsdGeom,UsdPhysics,Gf
        from traffic.physics_session import prepare_vehicle_runtime
        from traffic.rendered_physics_session import RenderedPhysicsSession
        from traffic.native_sensor_capture import NativeCamera
        from traffic.physical_lidar import PhysicalLidar
        from replay_export import export_replay,vehicle_path,pose
        from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_file
        result['runtime']=prepare_vehicle_runtime()
        exported=export_replay(scene,structured=True)
        session=RenderedPhysicsSession(120,20,physics_scene_path='/World/Physics/Scene')
        stage=session.stage
        stage.GetRootLayer().subLayerPaths.append(str(scene/'world.usda'))
        stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
        UsdGeom.SetStageMetersPerUnit(stage,1);UsdGeom.SetStageUpAxis(stage,'Z')
        UsdGeom.Scope.Define(stage,'/World/Physics')
        UsdPhysics.Scene.Define(stage,'/World/Physics/Scene')
        ego=vehicle_path('agent_0')
        stage.GetPrimAtPath(ego+'/Model').SetActive(False)
        # Reference only visual and sensor layers: no PhysX vehicle can compete with SUMO.
        prepared=UsdGeom.Xform.Define(stage,ego+'/Prepared')
        for name in ('vehicle-sensors.usda','vehicle-geometry.usda'):
            prepared.GetPrim().GetReferences().AddReference(str(scene/'prepared-assets'/name),'/Vehicle')
        front_offset=model['body_collision_center_m'][0]+model['body_collision_dimensions_m'][0]/2
        prepared.AddTranslateOp().Set(Gf.Vec3d(-front_offset,0,0))
        chassis=ego+'/Prepared/Chassis'
        assert not any(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in stage.Traverse())
        UsdGeom.Scope.Define(stage,'/World/Cameras')
        camera=UsdGeom.Camera.Define(stage,'/World/Cameras/Follow')
        camera.GetFocalLengthAttr().Set(18.)
        camop=camera.AddTransformOp()
        get_active_viewport().camera_path=str(camera.GetPath())
        from isaacsim.core.rendering_manager import ViewportManager
        if not ViewportManager.wait_for_viewport(max_frames=120,sleep_time=.02)[0]:
            raise RuntimeError('Viewport not ready')
        assert not stage.GetCompositionErrors()
        stage.GetRootLayer().Export(str(scene/'sensor-scene.usda'))
        saved=Usd.Stage.Open(str(scene/'sensor-scene.usda'))
        saved.GetRootLayer().subLayerPaths=['./world.usda']
        refs=saved.GetPrimAtPath(ego+'/Prepared').GetReferences();refs.ClearReferences()
        for name in ('vehicle-sensors.usda','vehicle-geometry.usda'):
            refs.AddReference('./prepared-assets/'+name,'/Vehicle')
        saved.GetRootLayer().Save();saved=None
        static_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in scene.glob('*.usda')}
        stage.SetEditTarget(stage.GetSessionLayer())
        ops={}
        for vid in exported['ids']:
            xf=UsdGeom.Xformable(stage.GetPrimAtPath(vehicle_path(vid)))
            ops[vid]=(xf.GetOrderedXformOps(),UsdGeom.Imageable(xf.GetPrim()).GetVisibilityAttr())
        def place(frame):
            present={v['id']:v for v in frame['vehicles']}
            for vid,(xops,vis) in ops.items():
                vis.Set('inherited' if vid in present else 'invisible')
                if vid in present:
                    pos,yaw=pose(present[vid])
                    for op in xops:
                        if op.GetOpType()==UsdGeom.XformOp.TypeTranslate:op.Set(Gf.Vec3d(*pos))
                        elif op.GetOpType()==UsdGeom.XformOp.TypeRotateZ:op.Set(yaw)
            v=present['agent_0']
            camop.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(v['x']-25,v['y']-36,30),Gf.Vec3d(v['x']-front_offset,v['y'],1),Gf.Vec3d(0,0,1)).GetInverse())
        place(frames[0])
        for spec in model['multi_camera_rig']['cameras']:
            name=spec['name']
            cameras[name]=NativeCamera(chassis+'/Sensors/'+name,resolution=(640,360),
                episode_id=output.name,sensor_id=name,calibration_id='v14/'+name)
            (output/name).mkdir()
        specs=[dict(name='LidarRoof',runtime_profile=model['roof_lidar']['runtime_profile'])]+model['perimeter_lidars']
        for spec in specs:
            name=spec['name']
            lidars[name]=PhysicalLidar(chassis,episode_id=output.name,vehicle_id=name,
                tick_source=lambda:session.physics_steps,sensor_path=chassis+'/Sensors/'+name+'/Scan',
                mount_translation_m=(0,0,0),scan_profile=spec['runtime_profile'])
        session.start()
        ready=False; camera_error=None
        for tick in range(360):
            session.step()
            if tick%6==5:session.render()
            if tick>120 and tick%6==5 and all(l.latest is not None and l.latest['scan_complete'] for l in lidars.values()):
                try:
                    for c in cameras.values(): c.read(session.time_s)
                    ready=True;break
                except RuntimeError as exc:camera_error=repr(exc)
        if not ready:
            result['warmup_diagnostics']={n:dict(packets=l.packet_count,errors=l.errors,complete=l.latest['scan_complete'] if l.latest else None) for n,l in lidars.items()}
            result['camera_error']=camera_error
            raise RuntimeError('Sensors failed bounded warmup')
        result['startup_seconds']=time.perf_counter()-started
        result['replay_clock_origin_s']=session.time_s
        old_stamps={}; max_error=0.; replay_start=time.perf_counter()
        for index,frame in enumerate(frames):
            begin=time.perf_counter();place(frame)
            for _ in range(12):
                session.step()
                if session.physics_steps%6==0:session.render()
            expected=next(v for v in frame['vehicles'] if v['id']=='agent_0')
            actual=UsdGeom.Xformable(stage.GetPrimAtPath(ego)).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
            max_error=max(max_error,float(np.linalg.norm(np.array(actual)-np.array(pose(expected)[0]))))
            if index%5==4:
                stem=f'{index:04d}'; row=dict(index=index,source_time_s=frame['time'],sensor_time_s=session.time_s,cameras={},lidars={})
                for name,c in cameras.items():
                    rgb,_=c.read(session.time_s)
                    Image.fromarray(rgb.rgb).save(output/name/(stem+'.png'))
                    row['cameras'][name]=dict(rgb_std=float(rgb.rgb.std()),transform=np.asarray(UsdGeom.Xformable(stage.GetPrimAtPath(c.path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).tolist())
                arrays={}
                for name,lidar in lidars.items():
                    p=lidar.latest
                    if lidar.errors or not p or not p['scan_complete'] or p['timestamp_ns']<=old_stamps.get(name,-1):
                        raise RuntimeError('Invalid or stale LiDAR '+name+str(lidar.errors))
                    old_stamps[name]=p['timestamp_ns'];hits=(p['flags'].astype(np.uint32)&64)!=0
                    if not hits.any() or not np.isfinite(p['xyz'][hits]).all():raise RuntimeError('Invalid returns '+name)
                    for key in ('xyz','flags','offset_ns'):arrays[name+'_'+key]=p[key]
                    age=session.absolute_time_s-p['frame_end_ns']*1e-9
                    if not -1e-5<=age<=.2:raise RuntimeError('LiDAR clock mismatch '+name+str(age))
                    row['lidars'][name]=dict(timestamp_ns=p['timestamp_ns'],frame_end_ns=p['frame_end_ns'],delivery_tick=p['delivery_tick'],age_s=age,hits=int(hits.sum()),sensor_position_world_m=p['sensor_position_world_m'].tolist())
                np.savez_compressed(output/(stem+'-lidar.npz'),**arrays)
                records.append(row)
                (output/'captures.json').write_text(json.dumps(records,indent=2))
            if index in (4,len(frames)//2-1,len(frames)-1):
                capture_viewport_to_file(get_active_viewport(),str(output/f'overview-{index:04d}.png'))
                session.render()
            timings.append(time.perf_counter()-begin)
        result.update(passed=True,steps=len(frames),capture_sets=len(records),camera_frames=len(records)*len(cameras),
            saved_scans=len(records)*len(lidars),position_error_m=max_error,wall_seconds=time.perf_counter()-replay_start,
            step_ms=dict(mean=float(np.mean(timings)*1000),p95=float(np.percentile(timings,95)*1000),max=float(max(timings)*1000)),
            lidar_packets={n:l.packet_count for n,l in lidars.items()},clock=session.snapshot(),
            camera_calibration=model['multi_camera_rig'],lidar_metadata={n:l.metadata for n,l in lidars.items()},
            static_layers_unchanged=all(hashlib.sha256((scene/n).read_bytes()).hexdigest()==h for n,h in static_hashes.items()))
        result['passed']=max_error<1e-6 and result['static_layers_unchanged'] and len(records)==len(frames)//5
    except Exception as exc:
        result.update(passed=False,error=repr(exc),traceback=traceback.format_exc());traceback.print_exc()
    finally:
        result['source_unchanged']=package_hashes(source)==hashes
        result['passed']=result['passed'] and result['source_unchanged']
        for sensor in [*cameras.values(),*lidars.values()]:
            try:sensor.close()
            except Exception as exc:result.update(passed=False,cleanup_error=repr(exc))
        cameras.clear();lidars.clear()
        if session:
            try:session.close()
            except Exception as exc:result.update(passed=False,cleanup_error=repr(exc))
        (output/'result.json').write_text(json.dumps(result,indent=2))
        print('SENSOR_REPLAY_RESULT='+json.dumps(result),flush=True)
        if app:app.close()
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
