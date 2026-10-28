"""Display a frozen MAPPO SUMO recording in Isaac; retain replay/timing evidence."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--recording', type=Path, required=True)
parser.add_argument('--gui', action='store_true')
args, _ = parser.parse_known_args()
output = args.recording.resolve()
if Path.cwd().resolve() != ROOT or not output.is_relative_to(ROOT / 'outputs'):
    parser.error('Run in checkout root with a recording under outputs/')
if (output / 'replay_report.json').exists() or list(output.glob('preview-*.png')):
    parser.error('Existing replay evidence found; preserve it and use a fresh recording directory')
data = json.loads((output / 'recording.json').read_text())
from isaacsim import SimulationApp
started = time.perf_counter()
app = SimulationApp({'headless': not args.gui, 'width': 1280, 'height': 720, 'extra_args': ['--/app/useFabricSceneDelegate=false']})
try:
    import omni.usd
    import omni.timeline
    from pxr import UsdGeom, Usd, Gf
    from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
    from replay_export import export_replay, vehicle_path, pose
    exported = export_replay(output, structured=True)
    names = ['world.usda', 'road.usda', 'motion.usda', 'vehicle.usda']
    hashes = {n:hashlib.sha256((output/n).read_bytes()).hexdigest() for n in names}
    omni.usd.get_context().open_stage(str(output/'world.usda'))
    for _ in range(15): app.update()
    stage = omni.usd.get_context().get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    UsdGeom.Scope.Define(stage, '/World/Cameras')
    camera_path = '/World/Cameras/Follow'
    camera = UsdGeom.Camera.Define(stage, camera_path)
    camera.GetFocalLengthAttr().Set(18.)
    camera_transform = camera.AddTransformOp()
    get_active_viewport().camera_path = camera_path
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    timeline.set_auto_update(False)
    startup = time.perf_counter()-started
    durations=[]
    geometry_diagnostics=[]
    checks=0
    maximum_error=0.
    captures=[]
    t0=data['frames'][0]['time']
    playback_started=time.perf_counter()
    for index,frame in enumerate(data['frames']):
        tick=time.perf_counter()
        elapsed=frame['time']-t0
        timeline.set_current_time(elapsed)
        # Explicit session-layer playback makes renderer state independent of paused timeline evaluation.
        present={v['id']:v for v in frame['vehicles']}
        for vid in exported['ids']:
            xf=UsdGeom.Xformable(stage.GetPrimAtPath(vehicle_path(vid)))
            UsdGeom.Imageable(xf.GetPrim()).GetVisibilityAttr().Set('inherited' if vid in present else 'invisible')
            if vid in present:
                position,yaw=pose(present[vid])
                for op in xf.GetOrderedXformOps():
                    if op.GetOpType()==UsdGeom.XformOp.TypeTranslate: op.Set(Gf.Vec3d(*position))
                    elif op.GetOpType()==UsdGeom.XformOp.TypeRotateZ: op.Set(yaw)
        ego=next(v for v in frame['vehicles'] if v['id']=='agent_0')
        eye=Gf.Vec3d(ego['x']-25,ego['y']-36,30)
        target=Gf.Vec3d(ego['x'],ego['y'],.6)
        camera_transform.Set(Gf.Matrix4d().SetLookAt(eye,target,Gf.Vec3d(0,0,1)).GetInverse())
        app.update()
        for v in frame['vehicles']:
            matrix=UsdGeom.Xformable(stage.GetPrimAtPath(vehicle_path(v['id']))).ComputeLocalToWorldTransform(elapsed*60)
            error=float(np.linalg.norm(np.array(matrix.ExtractTranslation())-np.array(pose(v)[0])))
            default_matrix=UsdGeom.Xformable(stage.GetPrimAtPath(vehicle_path(v['id']))).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            default_error=float(np.linalg.norm(np.array(default_matrix.ExtractTranslation())-np.array(pose(v)[0])))
            maximum_error=max(maximum_error,error,default_error)
            checks+=1
        if index in (0, len(data['frames'])//2, len(data['frames'])-1):
            bbox=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_]).ComputeWorldBound(stage.GetPrimAtPath(vehicle_path('agent_0'))).ComputeAlignedRange()
            geometry_diagnostics.append(dict(index=index,camera_position=list(UsdGeom.Xformable(camera).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()),ego_bounds_min=list(bbox.GetMin()),ego_bounds_max=list(bbox.GetMax()),active_camera=str(get_active_viewport().camera_path)))
            (output/'geometry-diagnostics.json').write_text(json.dumps(geometry_diagnostics,indent=2))
            for _ in range(3): app.update()
            target=output/f'preview-{index:04d}.png'
            captures.append(capture_viewport_to_file(get_active_viewport(),str(target)))
        # Pace visual replay only. Policy inference already ran independently in SUMO.
        if args.gui:
            remaining=playback_started+elapsed+.1-time.perf_counter()
            if remaining>0: time.sleep(remaining)
        durations.append(time.perf_counter()-tick)
    capture_deadline=time.perf_counter()+10
    while len(list(output.glob('preview-*.png')))<3 and time.perf_counter()<capture_deadline:
        app.update()
        time.sleep(.02)
    for _ in range(5): app.update()
    wall=time.perf_counter()-playback_started
    unchanged=all(hashlib.sha256((output/n).read_bytes()).hexdigest()==h for n,h in hashes.items())
    images=list(output.glob('preview-*.png'))
    passed=maximum_error<1e-6 and unchanged and not stage.GetCompositionErrors() and len(images)==3
    report=dict(passed=passed,training=False,optimizer_steps=0,motion_owner='SUMO recorded policy rollout',
        mode='kinematic 3D replay; not physical policy validation',seed=data['seed'],policy_sha256=data['checkpoint_sha256'],
        samples=len(data['frames']),vehicle_count=len(exported['ids']),position_checks=checks,
        maximum_position_error_m=maximum_error,endpoint_checks=exported['checks'],
        static_and_motion_layers_unchanged=unchanged,layer_sha256=hashes,
        simulation_seconds=data['frames'][-1]['time']-t0,source_timestep=data['source_timestep'],
        startup_seconds=startup,playback_wall_seconds=wall,
        playback_real_time_factor=(data['frames'][-1]['time']-t0)/wall,
        paced_frame_ms=dict(mean=float(np.mean(durations)*1000),p95=float(np.percentile(durations,95)*1000),maximum=float(max(durations)*1000)),
        geometry_diagnostics=geometry_diagnostics, screenshots=[p.name for p in images],root_children=[p.GetPath().pathString for p in stage.GetPrimAtPath('/World').GetChildren()],
        runtime_owned_namespaces=['/Render','/OmniverseKit_Persp'],
        layers=[Path(layer.identifier).name for layer in stage.GetLayerStack() if not layer.anonymous])
    (output/'replay_report.json').write_text(json.dumps(report,indent=2))
    print('MAPPO_REPLAY_RESULT='+json.dumps(report),flush=True)
    if not passed: raise RuntimeError('Replay acceptance failed; inspect retained report')
finally:
    app.close()
