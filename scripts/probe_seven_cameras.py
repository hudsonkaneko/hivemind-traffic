"""Bounded seven-camera capture/geometry test, not a policy or training run."""
import argparse
import json
import math
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    output = parser.parse_args().output.resolve()
    if Path.cwd().resolve() != ROOT or not output.is_relative_to(ROOT/'outputs') or not output.is_dir():
        parser.error('Use supervisor from project root')
    cfg = json.loads((output/'resolved-config.json').read_text())
    if cfg['training'] or not 0 < cfg['duration_s'] <= 6 or (output/'probe-result.json').exists():
        parser.error('Only fresh bounded validation is supported')
    from usd.showcase_vehicle import package_hashes
    from usd.seven_camera_rig import camera_specs
    source = ROOT/cfg['source_vehicle']
    hashes = package_hashes(source)
    result = dict(passed=False, training=False)
    started = time.perf_counter()
    app = session = vehicle = view = monitor = None
    cameras = {}
    rows = []
    try:
        import numpy as np
        from isaacsim import SimulationApp
        app = SimulationApp(dict(headless=True, renderer='RealTimePathTracing', width=960,
            height=540, enable_motion_bvh=True, disable_viewport_updates=False,
            create_new_stage=False, limit_cpu_threads=8))
        from pxr import Gf, Sdf, Usd, UsdGeom
        from PIL import Image, ImageDraw
        from traffic.physics_session import prepare_vehicle_runtime
        from traffic.rendered_physics_session import RenderedPhysicsSession
        from traffic.native_sensor_capture import NativeCamera
        from traffic.driver_control import AppliedControl
        from traffic.vehicle_contacts import VehicleContactMonitor
        from traffic.lane_geometry import LaneRoute, RouteSegment
        from traffic.bypass_validation import OrientedBox
        from usd.showcase_vehicle import compose_prepared_vehicle
        from usd.physical_scene import PATHS, author_scene_details, validate_scene
        from visualization.physics_road_view import author_physics_road
        result['runtime'] = prepare_vehicle_runtime()
        session = RenderedPhysicsSession(120, 20, physics_scene_path=PATHS.physics)
        stage = session.stage
        scene = output/'scene'
        vehicle = compose_prepared_vehicle(stage, scene, source, 120)
        view = author_physics_road(stage, LaneRoute('camera_test',3.6,(RouteSegment(80.),)),
            clean_layout=True, show_reference_centerline=False, asset_directory=scene/'assets')
        author_scene_details(stage, scene, OrientedBox(45.,0.,0.,1.,3.))
        UsdGeom.Imageable(stage.GetPrimAtPath('/World/Debug')).CreateVisibilityAttr().Set('invisible')
        # Reusable calibration target geometry; instance placements are evaluation only.
        target_asset = Usd.Stage.CreateNew(str(scene/'assets/calibration-target.usda'))
        target_root = UsdGeom.Xform.Define(target_asset,'/CalibrationTarget')
        target_asset.SetDefaultPrim(target_root.GetPrim())
        UsdGeom.SetStageMetersPerUnit(target_asset,1.)
        UsdGeom.SetStageUpAxis(target_asset,UsdGeom.Tokens.z)
        cube = UsdGeom.Cube.Define(target_asset,'/CalibrationTarget/Panel')
        cube.CreateSizeAttr().Set(1.)
        cube.AddScaleOp().Set(Gf.Vec3d(1.6,1.6,3.))
        target_asset.GetRootLayer().Save()
        target_asset = cube = target_root = None
        targets = {}
        colors = [(1,.15,.1),(.1,1,.2),(.1,.3,1),(1,.8,.1),(.8,.1,1),(.1,1,1)]
        with Usd.EditContext(stage, Sdf.Layer.FindOrOpen(str(scene/'layout.usda'))):
            UsdGeom.Xform.Define(stage, '/World/Environment/CalibrationTargets')
            for index, heading in enumerate((0,65,-65,135,-135,180)):
                angle = math.radians(heading)
                pos = (14*math.cos(angle),14*math.sin(angle),1.5)
                targets[heading] = pos
                path = '/World/Environment/CalibrationTargets/Target'+str(index)
                prim = UsdGeom.Xform.Define(stage,path)
                prim.GetPrim().GetReferences().AddReference('./assets/calibration-target.usda')
                prim.AddTranslateOp().Set(Gf.Vec3d(*pos))
                UsdGeom.Gprim(stage.GetPrimAtPath(path+'/Panel')).CreateDisplayColorAttr().Set([Gf.Vec3f(*colors[index])])
        Sdf.Layer.FindOrOpen(str(scene/'layout.usda')).Save()
        result['scene_structure'] = validate_scene(stage,require_sensor=False)
        view.dynamic_layer.Export(str(scene/'debug.usda'))
        stage.GetRootLayer().Export(str(scene/'world.usda'))
        saved = Sdf.Layer.FindOrOpen(str(scene/'world.usda'))
        saved.subLayerPaths = ['./debug.usda' if p == view.dynamic_layer.identifier else './'+Path(p).name for p in saved.subLayerPaths]
        saved.Save()
        reopened = Usd.Stage.Open(str(scene/'world.usda'))
        result['composition_errors'] = [str(e) for e in reopened.GetCompositionErrors()]
        result['hierarchy'] = [dict(path=str(p.GetPath()),type=p.GetTypeName(),references=p.HasAuthoredReferences(),payloads=p.HasAuthoredPayloads()) for p in reopened.Traverse()]
        reopened = saved = None
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=np.array([7.,-9.,7.]), target=np.array([0.,0.,1.]),
                        camera_prim_path=PATHS.overview_camera)
        from isaacsim.core.rendering_manager import ViewportManager
        ready, frames = ViewportManager.wait_for_viewport(max_frames=120,sleep_time=.02)
        if not ready: raise RuntimeError('Viewport not ready')
        specs = camera_specs()
        local_before = {}
        for spec in specs:
            name = spec['name']; path = vehicle.path+'/Sensors/'+name
            local_before[name] = UsdGeom.Xformable(stage.GetPrimAtPath(path)).GetLocalTransformation()
            cameras[name] = NativeCamera(path,resolution=(640,360),episode_id=output.name,
                sensor_id=name,frame_id=name+'_optical',calibration_id='seven-camera-v01/'+name)
            if not Gf.IsClose(local_before[name],UsdGeom.Xformable(stage.GetPrimAtPath(path)).GetLocalTransformation(),1e-6):
                raise RuntimeError('RTX changed authored camera mount: '+name)
        stage.GetPrimAtPath(PATHS.overview_camera).ApplyAPI('OmniSensorAPI')
        cameras['overview'] = NativeCamera(PATHS.overview_camera,resolution=(960,540),episode_id=output.name)
        xform = UsdGeom.Xformable(stage.GetPrimAtPath(PATHS.overview_camera))
        xform.ClearXformOpOrder()
        xform.AddTransformOp(opSuffix='evaluationView').Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(7,-9,7),Gf.Vec3d(0,0,1),Gf.Vec3d(0,0,1)).GetInverse())
        xform = None
        from omni.kit.viewport.utility import get_active_viewport
        viewport = get_active_viewport()
        if viewport:
            viewport.updates_enabled=True
            viewport.camera_path=PATHS.overview_camera
        viewport=None
        monitor = VehicleContactMonitor(stage,vehicle.path)
        vehicle.apply(AppliedControl(0.,0.,1.,'capture_warmup',None,True),700.,1500.)
        session.start()
        initial = vehicle.state()
        contacts = []
        counts = {s['name']:0 for s in specs}
        captured = False
        for tick in range(1,int(cfg['duration_s']*120)+1):
            if time.perf_counter()-started > cfg['max_process_seconds']-20: raise TimeoutError('Capture deadline')
            session.step()
            now = session.time_s
            truth = vehicle.state()
            contacts.extend(monitor.sample(session.sim,tick))
            if tick%6 == 0: session.render()
            if tick%12: continue
            row = dict(time_s=now,cameras={},speed_m_s=truth['speed_m_s'],upright_z=truth['upright_z'])
            rgb_frames = {}
            for spec in specs:
                name=spec['name']
                try:
                    frame,depth = cameras[name].read(now)
                except RuntimeError:
                    if now < 2.: continue
                    raise
                rgb_frames[name]=frame.rgb
                good=frame.rgb.shape==(360,640,3) and bool(np.isfinite(frame.rgb).all()) and float(frame.rgb.std())>2
                counts[name]+=int(good)
                mount=UsdGeom.Xformable(stage.GetPrimAtPath(vehicle.path+'/Sensors/'+name))
                local_ok=Gf.IsClose(mount.GetLocalTransformation(),local_before[name],1e-6)
                # Ground truth projection is used only to evaluate view direction/depth.
                point=mount.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).GetInverse().Transform(Gf.Vec3d(*targets[spec['heading_deg']]))
                fx=640*spec['focal_length_mm']/36
                u,v=320+fx*point[0]/(-point[2]),180-fx*point[1]/(-point[2])
                in_view=point[2]<0 and 4<=u<636 and 4<=v<356
                observed=float(np.median(depth[int(v)-3:int(v)+4,int(u)-3:int(u)+4])) if in_view else None
                target_ok=bool(in_view and math.isfinite(observed) and abs(observed+point[2])<1.8)
                row['cameras'][name]=dict(valid=good,mount_unchanged=local_ok,target_visible=target_ok,
                    target_pixel=[u,v],depth_m=observed,expected_center_depth_m=-point[2])
            rows.append(row)
            if now>=2. and not captured and len(rgb_frames)==7:
                sheet=Image.new('RGB',(1280,4*390),(24,26,30));draw=ImageDraw.Draw(sheet)
                for i,spec in enumerate(specs):
                    name=spec['name'];im=Image.fromarray(rgb_frames[name]);im.save(output/(name+'.png'))
                    x=(i%2)*640;y=(i//2)*390
                    sheet.paste(im,(x,y+30));draw.text((x+12,y+8),name+' | '+str(spec['horizontal_fov_deg'])+' deg',fill='white')
                overview,_=cameras['overview'].read(now)
                Image.fromarray(overview.rgb).save(output/'overview.png')
                sheet.save(output/'seven-camera-contact-sheet.png')
                captured=True
            # Scripted low-speed motion proves chassis inheritance; no policy consumes these frames.
            throttle=.16 if 2.<=now<4. and truth['speed_m_s']<1.5 else 0.
            brake=1. if now<2. or now>=4. else 0.
            vehicle.apply(AppliedControl(0.,throttle,brake,'bounded_capture_test',None,False),700.,1500.)
        active=[r for r in rows if r['time_s']>=2.]
        gates=dict(all_seven_capture=all(c>=30 for c in counts.values()),
            stable_mounts=all(len(r['cameras'])==7 and all(c['mount_unchanged'] for c in r['cameras'].values()) for r in active),
            targets_visible=all(any(r['cameras'].get(s['name'],{}).get('target_visible',False) for r in active) for s in specs),
            moved=truth['position_m'][0]-initial['position_m'][0]>.2,
            stable=all(r['upright_z']>.8 for r in rows),no_contacts=not contacts,
            screenshots=captured,composition=not result['composition_errors'])
        result.update(gates=gates,passed=all(gates.values()),valid_frames=counts,
            progress_m=truth['position_m'][0]-initial['position_m'][0],clock=session.snapshot(),contacts=len(contacts),
            camera_specs=specs,limitations=['Synthetic perspective cameras, no fisheye distortion or hardware noise',
                'Caller simulation timestamps; hardware freshness not proven','No learned fusion, Cosmos, active LiDAR concurrency or policy training',
                'Angular coverage and six static targets do not establish obstacle-free visibility everywhere'])
    except Exception as error:
        result.update(error=repr(error),traceback=traceback.format_exc())
        traceback.print_exc()
    finally:
        result.update(source_unchanged=package_hashes(source)==hashes,source_hashes=hashes,wall_seconds=time.perf_counter()-started)
        result['passed']=bool(result.get('passed') and result['source_unchanged'])
        (output/'camera-measurements.json').write_text(json.dumps(rows,indent=2))
        for camera in cameras.values():
            try: camera.close()
            except Exception as error: result.setdefault('cleanup_errors',[]).append(repr(error))
        cameras.clear()
        camera=vehicle=view=monitor=stage=mount=prim=None
        if session:
            try: session.close()
            except Exception as error: result.setdefault('cleanup_errors',[]).append(repr(error))
        if result.get('cleanup_errors'): result['passed']=False
        (output/'probe-result.json').write_text(json.dumps(result,indent=2))
        if app: app.close()
    return 0 if result['passed'] else 1


if __name__=='__main__': raise SystemExit(main())
