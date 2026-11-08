"""Bounded RGB/semantic/RTX capture for static and moving fusion validation.

No learning is performed. The ego is PhysX-owned and braked. Reusable background
vehicle visuals are scripted kinematic poses owned solely by this fixture.
Object-ID classes and camera depth are saved separately as evaluation oracles.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
CLASSES = dict(background=0, drivable=1, lane=2, vehicle=3, pedestrian=4, obstacle=5)


def canonical_classes(names):
    """The prepared asset's 'car' semantic is a member of vehicle class 3."""
    return {'vehicle' if name == 'car' else name for name in names}


def aligned_capture_schedule(origin_steps, origin_time_s, warmup_s, stride_ticks, frame_count):
    """Align native 20 Hz LiDAR and 10 Hz cameras after manager startup ticks."""
    if (not math.isclose(origin_steps/120., origin_time_s, rel_tol=0, abs_tol=1e-6)
            or stride_ticks % 12):
        raise ValueError('Native origin/capture stride is incompatible with the 10 Hz sensor grid')
    first_absolute_tick = math.ceil((origin_time_s+warmup_s)*10.-1e-9)*12
    first_relative_tick = first_absolute_tick-origin_steps
    ticks = [first_relative_tick+stride_ticks*i for i in range(frame_count)]
    return ticks, dict(requested_warmup_s=warmup_s, origin_physics_steps=origin_steps,
        origin_time_s=origin_time_s, render_absolute_tick_modulus=6,
        camera_absolute_tick_modulus=12, first_relative_tick=ticks[0], last_relative_tick=ticks[-1],
        actual_warmup_s=ticks[0]/120., first_capture_absolute_s=(origin_steps+ticks[0])/120.,
        last_capture_absolute_s=(origin_steps+ticks[-1])/120.)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    output = parser.parse_args().output.resolve()
    if Path.cwd().resolve() != ROOT or not output.is_relative_to(ROOT/'outputs') or (output/'capture-result.json').exists():
        parser.error('Use a fresh project output directory')
    cfg = json.loads((output/'resolved-config.json').read_text())
    mode = cfg['mode']
    if mode not in ('static', 'moving') or cfg.get('training', False): raise ValueError('Capture only; static or moving mode required')
    frames_requested = cfg.get('frames', 8)
    speed = float(cfg.get('moving_speed_m_s', 2.)) if mode == 'moving' else 0.
    if type(frames_requested) is not int or not 2 <= frames_requested <= 12 or not 0 <= speed <= 10:
        raise ValueError('Capture bound exceeded')
    start_s = cfg.get('capture_start_s', 1.2)
    stride_s = cfg.get('frame_stride_s', .1)
    if (isinstance(start_s, bool) or isinstance(stride_s, bool) or
            not isinstance(start_s, (int,float)) or not isinstance(stride_s, (int,float)) or
            not math.isfinite(start_s) or not math.isfinite(stride_s) or
            not .5 <= start_s <= 2. or not .05 <= stride_s <= .2):
        raise ValueError('Capture start/stride must be bounded finite seconds')
    start_tick, stride_ticks = round(start_s*120), round(stride_s*120)
    if (abs(start_tick-start_s*120)>1e-7 or abs(stride_ticks-stride_s*120)>1e-7 or
            start_tick%6 or stride_ticks%12 or start_s+(frames_requested-1)*stride_s>4.):
        raise ValueError('Capture stride must align with 10Hz cameras; requested window must finish within four seconds')
    scene_id = cfg.get('scene_id', 'fusion-'+mode)
    source = (ROOT/cfg.get('source_vehicle', 'vehicles/sim_ready/americano-i7-ev/v14')).resolve()
    if not source.is_relative_to(ROOT/'vehicles/sim_ready'): raise ValueError('Prepared asset must be inside workspace')
    from usd.showcase_vehicle import package_hashes
    source_hashes = package_hashes(source)
    app = session = camera = vehicle = view = stage = None
    lidars, frames, ops = [], [], []
    result = dict(passed=False, training=False, mode=mode, moving_speed_m_s=speed,
                  capture_start_s=start_s, frame_stride_s=stride_s, frames_requested=frames_requested)
    try:
        import numpy as np
        from isaacsim import SimulationApp
        app = SimulationApp(dict(headless=True, renderer='RealTimePathTracing', width=640, height=360,
            enable_motion_bvh=True, disable_viewport_updates=False, create_new_stage=False, limit_cpu_threads=8,
            extra_args=['--/rtx-transient/stableIds/enabled=true']))
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdSemantics, UsdUtils
        from PIL import Image, ImageDraw
        from traffic.physics_session import prepare_vehicle_runtime
        from traffic.rendered_physics_session import RenderedPhysicsSession
        from traffic.native_sensor_capture import NativeCamera
        from traffic.semantic_lidar_capture import SemanticLidarCapture
        from traffic.driver_control import AppliedControl
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
        view = author_physics_road(stage, LaneRoute('fusion-road', 10.8, (RouteSegment(80., 0.),)),
            clean_layout=True, show_reference_centerline=False, asset_directory=scene/'assets')
        author_scene_details(stage, scene, OrientedBox(10., -3., 0., 1.4, 1.2))
        UsdGeom.Imageable(stage.GetPrimAtPath('/World/Debug')).CreateVisibilityAttr().Set('invisible')

        def label(path, name):
            prim = stage.GetPrimAtPath(path)
            if not prim: raise RuntimeError('Cannot label missing prim '+path)
            UsdSemantics.LabelsAPI.Apply(prim, 'class').CreateLabelsAttr().Set([name])

        def class_for_path(path):
            prim = stage.GetPrimAtPath(path)
            while prim and not prim.IsPseudoRoot():
                api = UsdSemantics.LabelsAPI(prim, 'class')
                values = api.GetLabelsAttr().Get() if api else None
                if values:
                    names = canonical_classes(map(str, values))
                    if len(names) != 1 or not names.issubset(CLASSES): raise RuntimeError('Unknown/ambiguous oracle labels '+str(names))
                    return CLASSES[next(iter(names))]
                prim = prim.GetParent()
            return 0

        label('/World/Environment/Highway/Geometry/RoadSurface', 'drivable')
        for name in ('LeftEdge', 'RightEdge'): label('/World/Environment/Highway/Geometry/'+name, 'lane')
        label(PATHS.barrier, 'obstacle')
        layout = Sdf.Layer.FindOrOpen(str(scene/'layout.usda'))
        traffic_paths = ['/World/Vehicles/vehicle_fusion_lead', '/World/Vehicles/vehicle_fusion_adjacent']
        initial_positions = [(16., 0., 0.), (20., 3.6, 0.)]
        with Usd.EditContext(stage, layout):
            for path, position in zip(traffic_paths, initial_positions):
                lead = UsdGeom.Xform.Define(stage, path)
                lead.GetPrim().GetReferences().AddReference('./assets/vehicle-geometry.usda')
                Usd.ModelAPI(lead.GetPrim()).SetKind('component')
                lead.GetPrim().SetCustomDataByKey('motion_authority', 'scripted_kinematic_capture_fixture')
                lead.GetPrim().SetCustomDataByKey('vehicle_id', path.rsplit('/',1)[1])
                ops.append(lead.AddTranslateOp())
                ops[-1].Set(Gf.Vec3d(*position))
                label(path, 'vehicle')
        layout.Save()
        motion = Sdf.Layer.CreateAnonymous('fusion-traffic-motion')
        stage.GetRootLayer().subLayerPaths.insert(0, motion.identifier)
        overview = UsdGeom.Xformable(stage.GetPrimAtPath(PATHS.overview_camera))
        overview.MakeMatrixXform().Set(Gf.Matrix4d().SetLookAt(
            Gf.Vec3d(7., -12., 9.), Gf.Vec3d(10., 0., 1.), Gf.Vec3d(0., 0., 1.)).GetInverse())
        result['scene_structure'] = validate_scene(stage, require_sensor=False)
        # Save the composed, initial authored scene before transient RTX graph creation.
        view.dynamic_layer.Export(str(scene/'debug.usda'))
        motion.Export(str(scene/'traffic-initial.usda'))
        stage.GetRootLayer().Export(str(scene/'world.usda'))
        saved = Sdf.Layer.FindOrOpen(str(scene/'world.usda'))
        replacements = {view.dynamic_layer.identifier:'./debug.usda', motion.identifier:'./traffic-initial.usda'}
        portable_layers = []
        for original in saved.subLayerPaths:
            if original in replacements:
                portable = replacements[original]
            else:
                layer_path = Path(original).resolve()
                if not layer_path.is_relative_to(scene):
                    raise RuntimeError('Saved scene sublayer escapes its package: '+original)
                portable = './'+layer_path.relative_to(scene).as_posix()
            if not (scene/portable).is_file():
                raise RuntimeError('Saved scene sublayer is missing: '+portable)
            portable_layers.append(portable)
        saved.subLayerPaths = portable_layers
        saved.Save(); saved = None
        reopened = Usd.Stage.Open(str(scene/'world.usda'))
        if reopened.GetCompositionErrors() or not reopened.GetDefaultPrim():
            raise RuntimeError('Saved composed scene failed to reopen')
        dependency_layers, dependency_assets, missing = UsdUtils.ComputeAllDependencies(Sdf.AssetPath(str(scene/'world.usda')))
        dependencies = [Path(layer.realPath).resolve() for layer in dependency_layers if layer.realPath]
        dependencies += [Path(asset).resolve() for asset in dependency_assets]
        if missing or any(not path.is_relative_to(scene) for path in dependencies):
            raise RuntimeError('Saved scene has missing or external dependencies: '+repr(list(missing)))
        result['saved_scene_composition'] = dict(sublayers=portable_layers,
            errors=[], default_prim=str(reopened.GetDefaultPrim().GetPath()),
            dependency_layers=len(dependency_layers), missing_dependencies=[], all_dependencies_local=True,
            runtime_namespace_exceptions=['/Render', '/OmniverseKit_Persp'])
        reopened = None

        # Let Hydra consume the composed stage before attaching six sensor
        # products. Headless execution has no viewport readiness handle.
        for _ in range(40):
            session.renderer.render()
            time.sleep(.02)
        if not session.timeline.is_stopped():
            raise RuntimeError('Render warmup unexpectedly started the timeline')
        result['renderer_warmup'] = dict(stopped_timeline_updates=40,
            sleep_per_update_s=.02, viewport_required=False)
        camera_path = vehicle.path+'/Sensors/CameraFront'
        camera = NativeCamera(camera_path, resolution=(640,360), episode_id=scene_id,
                              calibration_id=vehicle.metadata['multi_camera_rig']['version']+'/CameraFront')
        camera.sensor.attach_annotators(['semantic_segmentation'])
        usd_camera = UsdGeom.Camera(stage.GetPrimAtPath(camera_path))
        if not math.isclose(float(usd_camera.GetPrim().GetAttribute('omni:sensor:tickRate').Get()),10.):
            raise RuntimeError('This capture schedule requires the prepared camera tick rate of 10 Hz')
        if str(usd_camera.GetProjectionAttr().Get()) != 'perspective': raise RuntimeError('Only calibrated perspective capture is supported')
        focal = float(usd_camera.GetFocalLengthAttr().Get())
        horizontal = float(usd_camera.GetHorizontalApertureAttr().Get())
        vertical = float(usd_camera.GetVerticalApertureAttr().Get())
        if usd_camera.GetHorizontalApertureOffsetAttr().Get() or usd_camera.GetVerticalApertureOffsetAttr().Get():
            raise RuntimeError('Nonzero aperture offsets require explicit calibration support')
        clipping = usd_camera.GetClippingRangeAttr().Get()
        calibration = dict(model='pinhole', width=640, height=360, fx=640*focal/horizontal,
            fy=360*focal/vertical, cx=320., cy=180., near_m=float(clipping[0]), far_m=float(clipping[1]),
            focal_length=focal, horizontal_aperture=horizontal, vertical_aperture=vertical)
        specs = [dict(name='LidarRoof',runtime_profile=vehicle.metadata['roof_lidar']['runtime_profile'])]+vehicle.metadata['perimeter_lidars']
        for spec in specs:
            lidars.append(SemanticLidarCapture(vehicle.path, class_for_path=class_for_path,
                episode_id=scene_id, vehicle_id=spec['name'], tick_source=lambda:session.physics_steps,
                sensor_path=vehicle.path+'/Sensors/'+spec['name']+'/Scan', mount_translation_m=(0,0,0),
                scan_profile=spec['runtime_profile']))

        def sensor_status(now_s=None):
            channels = {}
            for name in ('rgb', 'distance_to_image_plane', 'semantic_segmentation'):
                value, _ = camera.sensor.get_data(name)
                channels[name] = value is not None
            lidar_rows = []
            for x in lidars:
                packet = x.latest
                row = dict(path=x.path, packet_count=x.packet_count,
                    timestamp_ns=packet['timestamp_ns'] if packet else None,
                    frame_end_ns=packet['frame_end_ns'] if packet else None,
                    delivery_tick=packet['delivery_tick'] if packet else None,
                    errors=x.errors+x.base.errors)
                if packet is not None and now_s is not None:
                    hits = (packet['flags'].astype(np.uint32)&64) != 0
                    age = now_s-packet['timestamp_s']
                    row.update(frame_end_age_s=now_s-packet['frame_end_ns']*1e-9,
                        newest_point_age_s=float(age.min()), oldest_point_age_s=float(age.max()),
                        eligible_hits_within_50ms=int(np.count_nonzero(hits & (age >= -1e-5) & (age <= .05))))
                lidar_rows.append(row)
            return dict(camera_channels=channels,
                camera_render_product=str(camera.sensor.render_product.GetPath()),
                camera_tick_rate_hz=float(stage.GetPrimAtPath(camera_path).GetAttribute('omni:sensor:tickRate').Get()),
                lidars=lidar_rows)

        vehicle.apply(AppliedControl(0,0,1,'fusion_capture_parked',None,True),700,1500)
        session.start()
        origin_steps = session.startup['origin_steps']
        capture_tick_list, result['capture_schedule'] = aligned_capture_schedule(
            origin_steps, session.startup['origin_time_s'], start_s, stride_ticks, frames_requested)
        capture_ticks = set(capture_tick_list)
        window_midpoint_s = (capture_tick_list[0]+capture_tick_list[-1])/240.
        captured_lidar_stamps = {}
        palette = np.array([[30,35,45],[70,180,90],[250,230,60],[60,130,240],[230,80,180],[240,120,40]],dtype=np.uint8)
        for tick in range(1, max(capture_ticks)+1):
            with Usd.EditContext(stage, motion):
                elapsed = tick/120.-window_midpoint_s
                positions = [(16.+speed*elapsed,0.,0.), (20.-speed*.4*elapsed,3.6,0.)]
                for op, position in zip(ops, positions): op.Set(Gf.Vec3d(*position))
            session.step()
            if (origin_steps+tick)%6 == 0:
                session.render()
                # Allow the renderer's asynchronous product work to progress;
                # this never advances simulation or changes the capture times.
                time.sleep(.01)
            if tick not in capture_ticks: continue
            absolute_time = session.absolute_time_s
            # Delivery at this render tick is not acquisition freshness: RTX can
            # deliver an older in-flight scan. Drain at unchanged simulation time
            # until native return timestamps support the fixed 50 ms fusion gate.
            ready_deadline = time.monotonic()+(8. if not frames else 2.)
            extra_renders = 0
            while True:
                status = sensor_status(absolute_time)
                ready = (all(status['camera_channels'].values()) and all(
                    row.get('eligible_hits_within_50ms', 0) >= 20 and
                    abs(row.get('frame_end_age_s', float('inf'))) <= 2e-6 and
                    row['delivery_tick'] == session.physics_steps
                    for row in status['lidars']))
                if ready or time.monotonic() >= ready_deadline: break
                session.render()
                extra_renders += 1
                time.sleep(.02)
            barrier = dict(ready=ready, max_pairing_skew_s=.05, frame_end_tolerance_s=2e-6,
                extra_render_only_updates=extra_renders, status=status,
                physics_steps=session.physics_steps, absolute_time_s=session.absolute_time_s)
            if not frames: result['first_capture_readiness'] = barrier
            if not ready:
                raise RuntimeError('Native sensors could not satisfy the 50 ms pairing barrier: '+json.dumps(status))
            frame, depth = camera.read(absolute_time)
            raw, info = camera.sensor.get_data('semantic_segmentation')
            if raw is None: raise RuntimeError('Missing semantic image')
            raw = raw.numpy().squeeze()
            mask = np.zeros(raw.shape,dtype=np.uint8)
            for key, values in info['idToLabels'].items():
                names = canonical_classes(set(values.get('class','').split(','))-{''})
                known = names & CLASSES.keys()
                if len(known)>1 or names-CLASSES.keys()-{'BACKGROUND','UNLABELLED'}: raise RuntimeError('Unknown semantic labels '+str(names))
                if known: mask[raw==int(key)] = CLASSES[next(iter(known))]
            if mask.shape != (360,640): raise RuntimeError('Semantic mask shape mismatch')
            pieces = []
            timing = []
            for i, lidar in enumerate(lidars):
                packet = lidar.latest
                if lidar.errors or lidar.base.errors or packet is None or packet['scan_complete'] != 1:
                    raise RuntimeError('Invalid semantic LiDAR packet '+str(lidar.errors+lidar.base.errors))
                if packet['delivery_tick'] != session.physics_steps:
                    raise RuntimeError('LiDAR packet was not delivered at the current render barrier: '+lidar.path)
                if packet['timestamp_ns'] <= captured_lidar_stamps.get(lidar.path, -1):
                    raise RuntimeError('LiDAR packet was repeated or time moved backwards: '+lidar.path)
                captured_lidar_stamps[lidar.path] = packet['timestamp_ns']
                hits = (packet['flags'].astype(np.uint32)&64) != 0
                if hits.sum()<20 or not np.isfinite(packet['xyz_world'][hits]).all(): raise RuntimeError('Insufficient or invalid LiDAR hits')
                if np.any(packet['timestamp_s'] > absolute_time+1e-5) or absolute_time-packet['timestamp_s'].min()>.2:
                    raise RuntimeError('LiDAR/camera absolute clock contract failed')
                pieces.append(packet)
                timing.append(dict(sensor=lidar.path, timestamp_ns=packet['timestamp_ns'],frame_end_ns=packet['frame_end_ns'],
                    delivery_tick=packet['delivery_tick'],fresh_at_render_barrier=True,
                    sensor_position_world_m=packet['sensor_position_world_m'].tolist(),
                    successful_hits=int(hits.sum()),unresolved_oracle_hits=int(np.count_nonzero(hits&(packet['oracle_class']<0)))))
            stem=f'{len(frames):04d}'
            Image.fromarray(frame.rgb).save(output/(stem+'-rgb.png'))
            np.save(output/(stem+'-semantic.npy'),mask)
            np.save(output/(stem+'-depth-evaluation-only.npy'),depth)
            arrays={key:np.concatenate([p[key] for p in pieces]) for key in ('xyz_world','flags','timestamp_s','oracle_class','object_id_bytes')}
            arrays['point_times_s']=arrays.pop('timestamp_s')
            arrays['sensor_index']=np.concatenate([np.full(len(p['flags']),i,dtype=np.uint8) for i,p in enumerate(pieces)])
            np.savez_compressed(output/(stem+'-lidar.npz'),**arrays)
            transform=np.asarray(UsdGeom.Xformable(stage.GetPrimAtPath(camera_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).tolist()
            metadata=dict(scene_id=scene_id,mode=mode,frame_index=len(frames),camera_to_world=transform,calibration=calibration,
                image_time_s=absolute_time,point_time_clock='SimulationManager absolute seconds; RTX timestampNs+timeOffsetNs',
                transform_convention='USD row-vector local-to-world; camera -Z forward +Y up; world metres Z-up',
                lidar_frame='WORLD NONCOMPENSATED',camera_timestamp_source='synchronous render barrier caller stamp; native annotator does not guarantee acquisition timestamp',
                camera_depth_role='evaluation-only; never fusion input',oracle_class_role='evaluation-only per-return RTX object IDs; never fusion input',
                semantic_mask_role='native simulator ground truth used as prototype mask; no trained perception model',
                scan_deskewed=False,render_count=session.render_count,physics_steps=session.physics_steps,
                pairing_barrier=barrier,
                actor_positions={path:list(UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()) for path in traffic_paths},lidar_timing=timing,
                rgb_sha256=hashlib.sha256(frame.rgb.tobytes()).hexdigest(),semantic_sha256=hashlib.sha256(mask.tobytes()).hexdigest())
            (output/(stem+'-metadata.json')).write_text(json.dumps(metadata,indent=2))
            frames.append(dict(stem=stem,rgb_path=stem+'-rgb.png',semantic_path=stem+'-semantic.npy',lidar_path=stem+'-lidar.npz',
                               labels_path=stem+'-metadata.json',depth_path=stem+'-depth-evaluation-only.npy',
                               image_time_s=absolute_time,rgb_sha256=metadata['rgb_sha256'],semantic_sha256=metadata['semantic_sha256']))
            if len(frames) in (1,frames_requested):
                panel=Image.new('RGB',(1280,390));panel.paste(Image.fromarray(frame.rgb),(0,30));panel.paste(Image.fromarray(palette[mask]),(640,30))
                draw=ImageDraw.Draw(panel);draw.text((10,8),scene_id+' | actual front-camera RGB',fill='white');draw.text((650,8),'Native semantic labels (ground truth)',fill='white')
                panel.save(output/(stem+'-capture.png'))
        different_rgb=len({f['rgb_sha256'] for f in frames});different_mask=len({f['semantic_sha256'] for f in frames})
        if mode=='moving' and (different_rgb<2 or different_mask<2): raise RuntimeError('Moving traffic did not produce changing camera/mask buffers')
        result.update(passed=len(frames)==frames_requested,frames=frames,calibration=calibration,
            motion_authority=dict(ego='Isaac PhysX, braked',background='scripted kinematic visual references'),
            moving_buffer_changes=dict(unique_rgb=different_rgb,unique_semantic_masks=different_mask),
            class_ids=CLASSES,lidar_metadata=[x.metadata for x in lidars],clock=session.snapshot())
        mapping={str(key):dict(path=str(path),semantic_class=class_for_path(str(path))) for lidar in lidars for key,path in lidar.object_map.items()}
        (output/'object-id-oracle-map.json').write_text(json.dumps(mapping,indent=2))
    except Exception as error:
        result.update(error=repr(error),traceback=traceback.format_exc(),frames=frames)
        if camera and lidars:
            try: result['failure_sensor_status'] = sensor_status()
            except Exception as diagnostic_error: result['diagnostic_error'] = repr(diagnostic_error)
        traceback.print_exc()
    finally:
        result['source_vehicle_unchanged']=package_hashes(source)==source_hashes
        result['source_vehicle_hashes']=source_hashes
        result['passed']=bool(result.get('passed') and result['source_vehicle_unchanged'])
        for lidar in lidars:
            try: lidar.close()
            except Exception as error: result.update(passed=False,cleanup_error=repr(error))
        if camera:
            try: camera.sensor.detach_annotators(['semantic_segmentation']);camera.close()
            except Exception as error: result.update(passed=False,cleanup_error=repr(error))
        lidars.clear();ops.clear()
        # Remove all USD-backed runtime handles before closing the scene.
        lidar=camera=vehicle=view=stage=lead=layout=motion=usd_camera=op=overview=reopened=None
        if session:
            try: session.close()
            except Exception as error: result.update(passed=False,cleanup_error=repr(error))
        (output/'capture-result.json').write_text(json.dumps(result,indent=2))
        if app: app.close()
    return 0 if result['passed'] else 1


if __name__=='__main__': raise SystemExit(main())
