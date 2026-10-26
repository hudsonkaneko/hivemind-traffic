"""Bounded native sensor-driven prototype; launch through the supervisor."""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def perception_control(result, target_speed_m_s=2.):
    """Controller accepts estimated perception only; no scene or truth handles."""
    from traffic.driver_control import AppliedControl
    if result is None or result.stop_required or not result.valid:
        return AppliedControl(0., 0., 1., 'sensor_fail_closed_or_obstacle', None, True)
    steer = max(-.08, min(.08, -float(result.lane.center_offset_pixels)*.0004))
    speed_error = target_speed_m_s-result.ego.speed_m_s
    return AppliedControl(steer, min(.5, max(0., speed_error*.35)),
                          min(.5, max(0., -speed_error*.3)), 'measured_lane_and_speed', None, False)


def assess(rows, contacts, source_unchanged, screenshots):
    active = [r for r in rows if r['time_s'] >= 2. and not r.get('injected_camera_fault')]
    valid = [r for r in active if r['perception']['valid']]
    tail = [r for r in rows if rows and r['time_s'] >= rows[-1]['time_s']-1.]
    fault = [r for r in rows if r.get('injected_camera_fault')]
    gates = dict(sensor_frames=len(valid)>=20 and len(valid)/max(1,len(active))>=.8, obstacle_observed=any(
        r['perception']['nearest_forward_gap_m'] is not None for r in valid),
        moved=bool(rows and rows[-1]['evaluation']['progress_m'] > 2.),
        stopped_for_obstacle=len(tail)>=8 and all(r['perception']['valid'] and
            r['perception']['stop_required'] and r['perception']['nearest_forward_gap_m'] is not None
            and r['evaluation']['speed_m_s']<.3 and 0<r['evaluation']['true_barrier_gap_m']<2.5 for r in tail),
        injected_fault_fails_closed=bool(fault) and all(r['control']['throttle']==0 and r['control']['brake']==1 for r in fault),
        no_contacts=not contacts, source_unchanged=source_unchanged,
        screenshots=any(s.startswith('camera-front-') for s in screenshots) and any(s.startswith('overview-') for s in screenshots),
        lane_containment=bool(rows and all(r['evaluation']['footprint_within_lane'] for r in rows)),
        stable=bool(rows and all(r['evaluation']['upright_z']>.8 for r in rows)))
    return dict(passed=all(gates.values()), gates=gates, valid_control_frames=len(valid),
                control_frames=len(rows), final_progress_m=rows[-1]['evaluation']['progress_m'] if rows else 0.,
                final_speed_m_s=rows[-1]['evaluation']['speed_m_s'] if rows else None)


def _json(value):
    if hasattr(value, 'tolist'):
        return value.tolist()
    if hasattr(value, '__dataclass_fields__'):
        return asdict(value)
    raise TypeError(type(value).__name__)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    output = parser.parse_args().output.resolve()
    if Path.cwd().resolve() != ROOT or not output.is_relative_to(ROOT/'outputs') or not output.is_dir():
        parser.error('Run supervisor from project root with fresh output directory')
    if (output/'probe-result.json').exists():
        parser.error('Completed output cannot be overwritten')
    cfg = json.loads((output/'resolved-config.json').read_text())
    if cfg['training'] is not False or not 0 < cfg['duration_s'] <= 15 or not 0 < cfg['target_speed_m_s'] <= 2:
        parser.error('Only bounded low-speed validation is supported')
    source = (ROOT/cfg['source_vehicle']).resolve()
    if not source.is_relative_to(ROOT/'vehicles/sim_ready'):
        parser.error('Source must be a local prepared vehicle')
    from usd.showcase_vehicle import package_hashes
    source_before = package_hashes(source)
    started = time.perf_counter()
    app = session = vehicle = camera = lidar = overview = view = monitor = None
    rows, contacts, screenshots, history, estimates = [], [], [], {}, {}
    history_times = {0: 0.}
    result = dict(passed=False, no_training=True, ground_truth_in_controller=False,
                  scope='Low-speed static barrier; RGB lane heuristic plus RTX geometry and noisy odometry')
    try:
        import numpy as np
        from isaacsim import SimulationApp
        app = SimulationApp(dict(headless=True, renderer='RealTimePathTracing', width=1280,
                                 height=720, enable_motion_bvh=True, disable_viewport_updates=False,
                                 limit_cpu_threads=8))
        from pxr import Sdf, Usd, UsdGeom
        from PIL import Image
        from traffic.physics_session import prepare_vehicle_runtime
        from traffic.rendered_physics_session import RenderedPhysicsSession
        from traffic.vehicle_contacts import VehicleContactMonitor
        from traffic.physical_lidar import PhysicalLidar
        from traffic.native_sensor_capture import NativeCamera, lidar_measurement
        from traffic.sensor_pipeline import (SeededSensorModel, EgoEstimator, SensorPipeline,
                                              CameraFrame, SensorHeader)
        from traffic.lane_geometry import LaneRoute, RouteSegment
        from traffic.bypass_validation import OrientedBox
        from usd.showcase_vehicle import compose_prepared_vehicle
        from usd.vehicle_sensor_rig import sensor_paths
        from usd.physical_scene import PATHS, author_scene_details, validate_scene
        from visualization.physics_road_view import author_physics_road
        from isaacsim.core.utils.viewports import set_camera_view
        result['runtime'] = prepare_vehicle_runtime()
        session = RenderedPhysicsSession(120, 20, physics_scene_path=PATHS.physics)
        scene = output/'scene'
        vehicle = compose_prepared_vehicle(session.stage, scene, source, 120)
        road = LaneRoute('sensor_straight', 3.6, (RouteSegment(80.),), origin_xy_m=(0.,0.))
        view = author_physics_road(session.stage, road, show_reference_centerline=False,
                                  clean_layout=True, asset_directory=scene/'assets')
        barrier_x = cfg.get('barrier_x_m', 14.)
        author_scene_details(session.stage, scene, OrientedBox(barrier_x,0.,0.,1.,3.))
        # Remove decorative future-path geometry from sensor imagery.
        debug = session.stage.GetPrimAtPath('/World/Debug')
        if debug:
            UsdGeom.Imageable(debug).CreateVisibilityAttr().Set('invisible')
        set_camera_view(eye=np.array([6.,-12.,8.]), target=np.array([9.,0.,1.]),
                        camera_prim_path=PATHS.overview_camera)
        result['scene_structure'] = validate_scene(session.stage, require_sensor=False)
        paths = sensor_paths(vehicle.path)
        # Save initial source composition before creating runtime-owned RTX graphs.
        view.dynamic_layer.Export(str(scene/'debug.usda'))
        session.stage.GetRootLayer().Export(str(scene/'world.usda'))
        saved = Sdf.Layer.FindOrOpen(str(scene/'world.usda'))
        saved.subLayerPaths = ['./debug.usda' if p == view.dynamic_layer.identifier else
                               './'+Path(p).name for p in saved.subLayerPaths]
        saved.Save()
        (output/'hierarchy.json').write_text(json.dumps([
            dict(path=str(p.GetPath()), type=p.GetTypeName(), references=p.HasAuthoredReferences(),
                 payloads=p.HasAuthoredPayloads()) for p in session.stage.Traverse()], indent=2))
        monitor = VehicleContactMonitor(session.stage, vehicle.path)
        camera = NativeCamera(paths['camera'], resolution=tuple(cfg['camera_resolution']), episode_id=output.name)
        overview = NativeCamera(PATHS.overview_camera, resolution=(960,540), episode_id=output.name)
        lidar = PhysicalLidar(vehicle.path, episode_id=output.name, vehicle_id='ego',
                              tick_source=lambda:session.physics_steps, sensor_path=paths['lidar'],
                              mount_translation_m=(0.,0.,0.))
        body_size = vehicle.metadata['body_collision_dimensions_m']
        body_center = vehicle.metadata['body_collision_center_m']
        half_length = abs(body_center[0])+body_size[0]/2
        half_width = abs(body_center[1])+body_size[1]/2
        pipeline = SensorPipeline(episode_id=output.name, ego_half_length_m=half_length,
                                  corridor_half_width_m=half_width+.1)
        result['body_envelope_m'] = dict(dimensions=body_size,center=body_center,
                                          sensing_half_length=half_length,sensing_half_width=half_width+.1)
        measurement = SeededSensorModel(cfg['seed'], episode_id=output.name)
        estimator = EgoEstimator(episode_id=output.name)
        rng = np.random.default_rng(cfg['seed']+1)
        control = perception_control(None)
        vehicle.apply(control, 700., 1500.)
        session.start()
        epoch = session.absolute_time_s
        initial = vehicle.state()
        estimator.initialize(measurement.localization(*initial['position_m'][:2], initial['yaw_rad']))
        history[0] = initial
        estimates[0] = estimator.estimate
        previous_yaw = initial['yaw_rad']
        last_camera = last_depth = None
        captured = set()
        for tick in range(1, int(cfg['duration_s']*120)+1):
            if time.perf_counter()-started > cfg['max_process_seconds']-20:
                raise TimeoutError('Native sensor probe deadline')
            session.step()
            truth = vehicle.state()
            now = session.time_s
            history[tick] = truth
            history_times[tick] = now
            contacts.extend(monitor.sample(session.sim, tick))
            if tick % 2 == 0:
                delta = math.atan2(math.sin(truth['yaw_rad']-previous_yaw), math.cos(truth['yaw_rad']-previous_yaw))
                estimator.update(measurement.telemetry(truth['speed_m_s'], delta*60., now))
                estimates[tick] = estimator.estimate
                previous_yaw = truth['yaw_rad']
            if tick % 6 == 0:
                session.render()
                if lidar.errors:
                    raise RuntimeError('RTX packet errors: '+repr(lidar.errors))
            if tick % 12 == 0:
                try:
                    last_camera, last_depth = camera.read(now)
                except RuntimeError as error:
                    if now > 2.:
                        raise
                    result.setdefault('camera_warmup', []).append(dict(time_s=now, error=str(error)))
                    last_camera = CameraFrame(SensorHeader('front_camera','camera_optical',now,now,
                        'front-camera-v1',False,output.name), np.empty((0,0,3),dtype=np.uint8))
                packet = lidar.latest
                acquisition = (float(packet['timestamp_ns'])+float(packet['offset_ns'].min()))*1e-9-epoch if packet else now
                reference_tick = max(0,min(tick,round(acquisition*120)))
                estimated_tick = min(estimates, key=lambda k:abs(k-reference_tick))
                scan = lidar_measurement(packet, history[reference_tick], now_s=now,
                                         epoch_s=epoch, noise_rng=rng, episode_id=output.name,
                                         delivery_s=history_times[packet['delivery_tick']] if packet else now)
                injected_fault = 3. <= now < 3.5
                controller_camera = CameraFrame(last_camera.header,np.zeros_like(last_camera.rgb)) if injected_fault else last_camera
                perception = pipeline.process(controller_camera, scan, estimates[estimated_tick], now)
                control = perception_control(perception, cfg['target_speed_m_s']) if now >= 2. else perception_control(None)
                vehicle.apply(control, 700., 1500.)
                c,s = math.cos(truth['yaw_rad']),math.sin(truth['yaw_rad'])
                footprint = OrientedBox(truth['position_m'][0]+c*body_center[0]-s*body_center[1],
                    truth['position_m'][1]+s*body_center[0]+c*body_center[1],truth['yaw_rad'],body_size[0],body_size[1])
                corners = footprint.corners()
                row = dict(time_s=now, perception=asdict(perception), control=asdict(control),
                           injected_camera_fault=injected_fault,
                           lidar_points=len(scan.points_chassis),
                           raw_lidar_timing=None if not packet else dict(timestamp_ns=int(packet['timestamp_ns']),
                               minimum_offset_ns=int(packet['offset_ns'].min()),maximum_offset_ns=int(packet['offset_ns'].max()),
                               delivery_tick=packet['delivery_tick']),
                           evaluation=dict(progress_m=truth['position_m'][0]-initial['position_m'][0],
                               speed_m_s=truth['speed_m_s'], upright_z=truth['upright_z'],
                               true_barrier_gap_m=barrier_x-.5-max(x for x,y in corners),
                               footprint_within_lane=all(abs(y)<=1.8 for x,y in corners),
                               position_error_m=math.dist(
                                   (estimator.estimate.x_m,estimator.estimate.y_m),truth['position_m'][:2])))
                rows.append(row)
                for second in (2,8,14):
                    if now >= second and second not in captured and last_camera.header.valid:
                        front = output/f'camera-front-{second:02d}s.png'
                        Image.fromarray(last_camera.rgb).save(front)
                        frame, _ = overview.read(now)
                        overview_file = output/f'overview-{second:02d}s.png'
                        Image.fromarray(frame.rgb).save(overview_file)
                        np.save(output/f'evaluation-depth-{second:02d}s.npy', last_depth)
                        np.save(output/f'lidar-local-{second:02d}s.npy',scan.points_chassis)
                        screenshots.extend([front.name, overview_file.name]);captured.add(second)
                if truth['upright_z'] < .8 or truth['position_m'][2] < 0:
                    raise RuntimeError('Vehicle stability failure')
        result.update(assess(rows, contacts, package_hashes(source)==source_before, screenshots))
        result.update(screenshots=screenshots, lidar_packets=lidar.packet_count, clock=session.snapshot(),
                      source_vehicle=cfg['source_vehicle'], source_hashes=source_before,
                      limitations=['RGB brightness lane heuristic is not learned semantic perception',
                          'Static barrier only; WORLD noncompensated LiDAR converted at sensor boundary',
                          'Wheel speed/gyro measurements use synthetic prototype noise',
                          'Camera timestamps use synchronous render barrier, not hardware clock metadata',
                          'No Cosmos, radar, surround perception or policy training'])
    except Exception as error:
        result.update(error=repr(error), traceback=traceback.format_exc())
        traceback.print_exc()
    finally:
        result['source_unchanged'] = package_hashes(source)==source_before
        result['passed'] = bool(result.get('passed') and result['source_unchanged'])
        result['wall_seconds'] = time.perf_counter()-started
        (output/'sensor-trajectory.json').write_text(json.dumps(rows,default=_json,indent=2))
        (output/'evaluation-contacts.json').write_text(json.dumps(contacts,default=_json,indent=2))
        for owned in (camera, overview, lidar):
            if owned:
                try: owned.close()
                except Exception as error: result.setdefault('cleanup_errors',[]).append(repr(error))
        camera=overview=lidar=monitor=view=vehicle=None
        debug=saved=None
        if session:
            try: session.close()
            except Exception as error: result.setdefault('cleanup_errors',[]).append(repr(error))
        if result.get('cleanup_errors'): result['passed']=False
        (output/'probe-result.json').write_text(json.dumps(result,default=_json,indent=2))
        print('SENSOR_PERCEPTION_RESULT='+json.dumps(result,default=_json),flush=True)
        if app: app.close()
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
