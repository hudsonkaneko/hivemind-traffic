"""One physical car follows a LiDAR-triggered detour; launch with its supervisor."""
import argparse
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import sys
import time
import traceback
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from experiments.probe_support import process_sample, write_json
from experiments.obstacle_bypass_config import validate_config
from traffic.bypass_validation import (OrientedBox, RouteSafetyBrake, assess_bypass,
    box_clearance, footprint_within_road, predicted_route_poses)
from traffic.driver_control import AppliedControl, DriverControlGate
from traffic.evidence_chunks import EvidenceChunkWriter
from traffic.lane_geometry import LaneRoute, RouteSegment
from traffic.lidar_braking import LidarBrakeConfig
from traffic.obstacle_bypass import BypassConfig, ObstacleBypassPlanner
from traffic.path_following import FollowerConfig, PathFollower, PathReference, VehicleState, SENSOR_PATH_SOURCE
from traffic.physical_lidar import PhysicalLidar, packet_to_scan, rotation_matrix
from traffic.runtime_profile import RuntimeProfiler
from traffic.wheel_geometry import assess_wheel_geometry, sample_wheel_geometry
from visualization.preview_timing import PreviewClock, rendering_profile


def route_data(route):
    return dict(route_id=route.route_id, width_m=route.width_m, origin_xy_m=route.origin_xy_m,
                initial_yaw_rad=route.initial_yaw_rad, segments=[asdict(s) for s in route.segments])


def author_barrier(stage, box):
    from pxr import Gf, UsdGeom, UsdPhysics, UsdLux
    cube = UsdGeom.Cube.Define(stage, '/World/Barrier')
    cube.CreateSizeAttr(1.)
    cube.AddTranslateOp().Set(Gf.Vec3d(box.x_m, box.y_m, .75))
    cube.AddScaleOp().Set(Gf.Vec3f(box.length_m, box.width_m, 1.5))
    cube.CreateDisplayColorAttr([Gf.Vec3f(.95, .23, .06)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdLux.DomeLight.Define(stage, '/World/DemoLight').CreateIntensityAttr(1000)


def controls(ui_state):
    import omni.ui as ui
    window = ui.Window('Highway Sim | Adaptive obstacle path', width=510, height=250)
    def camera(mode): ui_state['camera'] = mode
    def toggle(key): ui_state[key] = not ui_state[key]
    with window.frame:
        with ui.VStack(spacing=6):
            ui.Label('PHYSX MOVEMENT | LIDAR-TRIGGERED SCRIPTED BYPASS', height=22)
            with ui.HStack(height=30, spacing=5):
                ui.Button('Overview', clicked_fn=lambda:camera('overview'))
                ui.Button('Follow car', clicked_fn=lambda:camera('follow'))
                ui.Button('Pause / resume', clicked_fn=lambda:toggle('paused'))
            with ui.HStack(height=30, spacing=5):
                ui.Button('Toggle planned path', clicked_fn=lambda:toggle('path'))
                ui.Button('Toggle LiDAR points', clicked_fn=lambda:toggle('points'))
            label = ui.Label('Warming up...', word_wrap=True)
            ui.Label('Cyan: actual driver path | Pink: steering target | Green: LiDAR\nOne static-obstacle fixture; not learned driving. Use these pause controls.', word_wrap=True)
    return window, label


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    output = parser.parse_args().output.resolve()
    if not output.is_relative_to(ROOT/'outputs') or not output.is_dir() or (output/'probe-result.json').exists():
        parser.error('Use the supervisor with a fresh output directory')
    cfg = json.loads((output/'resolved-config.json').read_text())
    validate_config(cfg, resolved=True)
    nominal = LaneRoute('nominal', 3.6, (RouteSegment(90),))
    road = LaneRoute('two_lane_test_road', 7.2, (RouteSegment(100),), origin_xy_m=(0,1.8))
    obstacle = OrientedBox(45., 1.8 if cfg['mode']=='blocked' else 0., 0., 2., 7.2 if cfg['mode']=='blocked' else 1.6)
    episode = 'bypass-'+output.name
    app = session = vehicle = lidar = view = monitor = window = label = None
    rows, wheel_rows, frames, contacts, render_checks, plans, history = [], [], [], [], [], [], {}
    captures, captured, saved = [], set(), set()
    profile = RuntimeProfiler()
    profile_overhead = profile.calibrate_overhead()
    chunk_writer = preview_clock = None
    previous_stamp = frozen_packet = None
    result = dict(passed=False, no_sumo=True, no_training=True,
        controller_source='LiDAR XYZ bounds + known straight road + privileged odometry + explicit maximum obstacle size prior',
        obstacle_ground_truth_in_control=False)
    started = time.perf_counter()
    try:
        chunk_writer = EvidenceChunkWriter(output)
        from isaacsim import SimulationApp
        graphics = rendering_profile(cfg.get('real_time', False))
        result['rendering_profile'] = graphics
        app = SimulationApp({**graphics, 'headless':not cfg['gui'],
                             'enable_motion_bvh':True, 'disable_viewport_updates':False})
        import carb
        settings = carb.settings.get_settings()
        result['observed_render_settings'] = {key:settings.get(key) for key in (
            '/rtx/rendermode', '/rtx/minimal/mode', '/app/vsync', '/app/window/hideUi',
            '/plugins/carb.tasking.plugin/threadCount', '/plugins/omni.tbb.globalcontrol/maxThreadCount',
            '/app/runLoops/main/syncToPresent', '/app/runLoops/main/rateLimitEnabled',
            '/app/runLoopsGlobal/syncToPresent', '/app/runLoops/rendering_0/syncToPresent',
            '/app/runLoops/rendering_1/syncToPresent', '/app/runLoops/present/rateLimitEnabled',
            '/app/runLoops/rendering_0/rateLimitEnabled',
            '/app/asyncRendering', '/omni/replicator/asyncRendering')}
        from traffic.physics_session import prepare_vehicle_runtime
        from traffic.rendered_physics_session import RenderedPhysicsSession
        from traffic.physx_vehicle import PhysxVehicle
        from traffic.vehicle_contacts import VehicleContactMonitor
        from visualization.physics_road_view import author_physics_road
        from isaacsim.core.utils.viewports import set_camera_view
        from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
        result['runtime_setup'] = prepare_vehicle_runtime()
        session = RenderedPhysicsSession(120,cfg['render_hz'])
        vehicle = PhysxVehicle(session.stage,120)
        view = author_physics_road(session.stage,road,lane_dividers_m=(0.,),show_reference_centerline=False)
        author_barrier(session.stage, obstacle)
        monitor = VehicleContactMonitor(session.stage, vehicle.path)
        lidar = PhysicalLidar(vehicle.path, episode_id=episode, vehicle_id='ego',
                              tick_source=lambda:session.physics_steps, points=cfg['points'])
        ui_state = dict(camera=cfg['camera'],path=True,points=cfg['points'],paused=False)
        if cfg['gui']: window,label = controls(ui_state)
        viewport = get_active_viewport()
        if viewport is None: raise RuntimeError('No viewport available')
        viewport.resolution = (graphics['width'],graphics['height'])
        static_before = view.static_layer.ExportToString()
        view.static_layer.Export(str(output/'road-static.usda'))
        session.stage.Flatten().Export(str(output/'scene-initial.usda'))
        write_json(output/'scene-contract.json',dict(vehicle=vehicle.metadata,sensor=lidar.metadata,
            view=view.metadata,nominal=route_data(nominal),road=route_data(road),
            evaluation_only_obstacle=asdict(obstacle),physics_authority='PhysX'))
        planner = ObstacleBypassPlanner(episode,'ego',nominal,BypassConfig(**cfg['planner']))
        follower = PathFollower({'nominal':nominal},episode,'ego',FollowerConfig(**cfg['follower']))
        gate = DriverControlGate(episode,'ego')
        safety = RouteSafetyBrake(episode,'ego',LidarBrakeConfig(**cfg['braking']))
        vehicle.apply(AppliedControl(0,0,1,'initial_hold',None,False),700,1500)
        session.start()
        result['observed_render_settings_after_start'] = {
            key:settings.get(key) for key in result['observed_render_settings']}
        epoch = session.absolute_time_s
        result['clock_start'] = session.snapshot()
        state = vehicle.state(); history[0] = state
        decision = plan = reference = None
        driver = {}; plan_tick = 0; previous_plan_id = 'nominal'
        loop_started = time.perf_counter(); paused_wall = 0
        preview_clock = PreviewClock(paced=cfg['paced'])
        result['startup_wall_s'] = loop_started-started
        for tick in range(6000):
            profile.begin_iteration()
            if time.perf_counter()-started>cfg['max_process_seconds']-20:
                raise TimeoutError('Bounded demo deadline')
            if not app.is_running(): raise RuntimeError('Demo closed before completion')
            while ui_state['paused']:
                if not app.is_running() or time.perf_counter()-started>cfg['max_process_seconds']-20:
                    raise RuntimeError('Paused demo closed or timed out')
                pause_start=time.perf_counter();session.render();time.sleep(.02)
                paused_wall+=time.perf_counter()-pause_start
            phase = 'settle' if tick<240 else ('hold' if tick>=5400 else 'drive')
            command = None
            if tick%2==0:
                packet = lidar.latest
                if cfg['mode']=='dropout' and tick>=1440:
                    if frozen_packet is None: frozen_packet=packet
                    packet=frozen_packet
                with profile.measure('scan'):
                    scan = packet_to_scan(packet,state,episode_id=episode,vehicle_id='ego',tick=tick,epoch_s=epoch)
                if tick%12==0:
                    plan_tick=tick
                    with profile.measure('planner'):
                        plan=planner.update(scan,VehicleState.from_physics(state,episode_id=episode,vehicle_id='ego',tick=tick),tick)
                    follower.routes[plan.route.route_id]=plan.route
                    plan_record=dict(tick=tick,status=plan.status,reason=plan.reason,path_id=plan.route.route_id,
                        target_speed_m_s=plan.target_speed_m_s,sensed_bounds=plan.sensed_bounds,
                        sensed_acquisition_ticks=plan.sensed_acquisition_ticks,extent_prior_m=plan.extent_prior_m,
                        source=plan.source)
                    plans.append(plan_record)
                    # Retain every raw scan used to accumulate a candidate,
                    # not just its first observation or the adoption scan.
                    if packet is not None and plan.sensed_bounds and plan.status != 'stop' and previous_plan_id == 'nominal':
                        evidence_name = f'planner-scan-{tick:05d}'
                        with profile.measure('evidence'):
                            np.savez_compressed(output/(evidence_name+'.npz'), xyz=packet['xyz'],
                                                flags=packet['flags'], offset_ns=packet['offset_ns'])
                            write_json(output/(evidence_name+'.json'), dict(
                                episode_id=episode, vehicle_id='ego', timestamp_ns=packet['timestamp_ns'],
                                scan_complete=packet['scan_complete'], delivery_tick=packet['delivery_tick'],
                                tick=tick, epoch_s=epoch, state=state, plan=plan_record))
                    if plan.route.route_id!=previous_plan_id:
                        with profile.measure('evidence'):
                            write_json(output/'adopted-path.json',dict(**plan_record,route=route_data(plan.route),
                                samples=[asdict(p) for p in plan.route.sample()],state_at_adoption=state))
                        previous_plan_id=plan.route.route_id
                reference=PathReference(episode,'ego',plan.route.route_id,plan_tick,plan_tick+24,
                    0. if phase=='settle' else plan.target_speed_m_s,plan.stop_s_m,source=SENSOR_PATH_SOURCE)
                with profile.measure('safety'):
                    preview=predicted_route_poses(plan.route,state,state['speed_m_s'])
                    decision=safety.evaluate(scan,tick=tick,speed_m_s=state['speed_m_s'],predicted_route=preview,
                        requested_target_speed_m_s=reference.target_speed_m_s)
                with profile.measure('control'):
                    command=follower.command(VehicleState.from_physics(state,episode_id=episode,vehicle_id='ego',tick=tick),tick,reference)
                    driver=dict(follower.last_diagnostics)
                    if decision.brake_override or plan.status in ('stop','stale_invalid'):
                        command=replace(command,throttle=0.,brake=1.)
                event=('path-adopted' if plan.route.route_id!='nominal' else 'obstacle-observed' if plan.sensed_bounds else None)
                if packet is not None and event is not None and event not in saved:
                    with profile.measure('evidence'):
                        np.savez_compressed(output/f'{event}.npz',xyz=packet['xyz'],flags=packet['flags'],offset_ns=packet['offset_ns'])
                        write_json(output/f'{event}.json',dict(episode_id=episode,vehicle_id='ego',
                            timestamp_ns=packet['timestamp_ns'],scan_complete=packet['scan_complete'],delivery_tick=packet['delivery_tick'],
                            tick=tick,epoch_s=epoch,state=state,plan=plan_record,decision=asdict(decision)))
                    saved.add(event)
            with profile.measure('physics'):
                applied=gate.step(tick=tick,dt_s=1/120,command=command)
                actuator=vehicle.apply(applied,700,1500)
                clock=session.step()
            with profile.measure('state'):
                state=vehicle.state();history[tick+1]=state
            with profile.measure('collision_geometry'):
                contacts.extend(monitor.sample(session.sim,tick+1))
                car_box=OrientedBox(*state['position_m'][:2],state['yaw_rad'])
                clearance=box_clearance(car_box,obstacle)
                road_inside=footprint_within_road(car_box)
                projected=plan.route.project(*state['position_m'][:2])
            safety_status='no_safe_route' if plan.status=='stop' else ('stale_invalid' if plan.status=='stale_invalid' else decision.status)
            row=dict(tick=tick+1,sim_time_s=(tick+1)/120,absolute_sim_time_s=clock['absolute_time_s'],
                episode_id=episode,vehicle_id='ego',phase=phase,control_tick=tick%2==0,control=actuator,driver=driver,
                planner=plan_record,lidar=asdict(decision),safety_status=safety_status,
                route_adopted_from_lidar=plan.route.route_id!='nominal',obstacle_contacts=len(contacts),
                clearance_m=clearance['clearance_m'],road_inside=road_inside,progress_m=state['position_m'][0],
                tracking_error_m=projected.lateral_error_m,**state)
            rows.append(row)
            if contacts or clearance['overlap'] or not road_inside or state['upright_z']<.99:
                raise RuntimeError('Contact, footprint departure or instability: stopped')
            if abs(projected.lateral_error_m)>cfg['tracking_error_max_m']:
                raise RuntimeError('Tracking error outside tested prediction envelope')
            if (tick+1)%4==0:
                with profile.measure('wheel_geometry'):
                    wheel_rows.extend(sample_wheel_geometry(vehicle,tick+1))
            if (tick+1)%session.render_every_steps==0:
                with profile.measure('view_update'):
                    view.update(state,target_xy=driver.get('target_xy_m'),show_reference=ui_state['path'],
                                planned_route=plan.route,preview_distance_m=35.,
                                full_plan=cfg.get('real_time', False))
                    camera=view.camera_pose(ui_state['camera'],state)
                    set_camera_view(eye=np.array(camera.eye),target=np.array(camera.target))
                    lidar.set_points(ui_state['points'])
                    if label:
                        timing = preview_clock.frames[-1] if preview_clock.frames else None
                        pace_text = (f"{timing['rtf']:.2f}x | lag {timing['lag_s']:.2f}s"
                                     if timing else 'Measuring playback rate...')
                        label.text=(f"{ui_state['camera'].upper()} | {row['sim_time_s']:.1f}s | {state['speed_m_s']:.2f} m/s\n"
                            f"Planner: {plan.status.upper()} | Safety: {safety_status.upper()}\n"
                            f"Path error {projected.lateral_error_m:+.2f}m | progress {state['position_m'][0]:.1f}/90m\n"
                            f"Playback: {pace_text}")
                with profile.measure('render'):
                    before=session.snapshot();session.render();after=session.snapshot()
                render_checks.append(before['physics_steps']==after['physics_steps'] and before['absolute_time_s']==after['absolute_time_s'])
                with profile.measure('sensor_evidence'):
                    packet=lidar.latest
                    if packet is not None and packet['timestamp_ns']!=previous_stamp:
                        previous_stamp=packet['timestamp_ns']
                        frame_tick=round((packet['frame_end_ns']*1e-9-epoch)*120)
                        frame_state=history.get(frame_tick)
                        pose_error=None
                        if frame_state is not None:
                            expected=np.array(frame_state['position_m'])+rotation_matrix(frame_state['quaternion_xyzw'])@np.array(lidar.mount_translation_m)
                            pose_error=float(np.linalg.norm(expected-packet['sensor_position_world_m']))
                        times=(packet['timestamp_ns']+packet['offset_ns'].astype(np.int64))*1e-9-epoch
                        frames.append(dict(timestamp_ns=packet['timestamp_ns'],frame_tick=frame_tick,
                            acquisition_start_tick=math.floor(float(times.min())*120+1e-6),
                            acquisition_end_tick=math.ceil(float(times.max())*120-1e-6),
                            delivery_tick=packet['delivery_tick'],sensor_pose_error_m=pose_error))
                if lidar.errors: raise RuntimeError('RTX capture error: '+repr(lidar.errors))
                if cfg['capture']:
                    with profile.measure('evidence'):
                        for second in (1,9,14,18,23,30,40):
                            if row['sim_time_s']>=second and second not in captured:
                                captures.append(capture_viewport_to_file(viewport,str(output/f'preview-{second:02d}s-{ui_state["camera"]}.png')))
                                if cfg['gui'] and second == 14:
                                    import omni.kit.renderer_capture
                                    omni.kit.renderer_capture.acquire_renderer_capture_interface().capture_next_frame_swapchain(
                                        str(output/'preview-window.png'))
                                captured.add(second)
            if (tick+1)%120==0:
                with profile.measure('evidence'):
                    chunk_writer.write_checkpoint(rows,plans)
                if (tick+1)%600==0:
                    with profile.measure('reporting'):
                        print(f"BYPASS_TIME={row['sim_time_s']:.1f} X={state['position_m'][0]:.2f} Y={state['position_m'][1]:.2f} PLAN={plan.status} SAFETY={safety_status}",flush=True)
            if (tick+1)%session.render_every_steps==0:
                with profile.measure('pace_wait'):
                    preview_clock.finish_frame(row['sim_time_s'],paused_wall)
            profile.finish_iteration()
        result['observed_render_settings_after_episode'] = {
            key:settings.get(key) for key in result['observed_render_settings']}
        result.update(assess_bypass(rows,mode=cfg['mode'],obstacle=obstacle))
        post_frames=[f for f in frames if f['delivery_tick']>240]
        active=[r for r in rows if r['tick']>240]
        wheel_result=assess_wheel_geometry(wheel_rows,vehicle.wheel_paths,6000)
        extra=dict(complete=len(rows)==6000,render_clock=bool(render_checks) and all(render_checks),
            no_sensor_errors=not lidar.errors,static_road_unchanged=view.static_layer.ExportToString()==static_before,
            driver_valid=all(not r['driver']['fallback'] and not r['control']['is_fallback'] for r in active),
            tracking_error=max(abs(r['tracking_error_m']) for r in rows)<=cfg['tracking_error_max_m'],
            sensor_pose_coverage=len(post_frames)>=100 and all(f['sensor_pose_error_m'] is not None for f in post_frames),
            sensor_pose=all(f['sensor_pose_error_m'] is not None and f['sensor_pose_error_m']<=.05 for f in post_frames),
            wheel_geometry=wheel_result['passed'])
        result['gates'].update(extra)
        result.update(passed=bool(result['passed'] and all(extra.values())),wheel_geometry=wheel_result,
            loop_wall_s=time.perf_counter()-loop_started,clock_end=session.snapshot(),
            preview_files=[p.name for p in output.glob('preview-*.png')],sensor_errors=lidar.errors,
            packet_count=lidar.packet_count,sensor_frames_after_settle=len(post_frames),
            max_sensor_pose_error_m=max(f['sensor_pose_error_m'] for f in post_frames if f['sensor_pose_error_m'] is not None),
            max_tracking_error_m=max(abs(r['tracking_error_m']) for r in rows))
        for _ in range(3):session.render()
    except Exception as error:
        result.update(passed=False,error=repr(error),traceback=traceback.format_exc())
        traceback.print_exc()
    finally:
        if profile.iteration_active:
            profile.finish_iteration()
        simulated_time_s=(rows[-1]['sim_time_s'] if rows else 0.0)
        result['runtime_profile']=dict(
            **profile.summary(simulation_time_s=simulated_time_s),
            instrumentation_overhead=profile.estimate_overhead_for_run(profile_overhead))
        if preview_clock:
            result['preview_timing']=preview_clock.summary(
                expected_simulation_time_s=cfg['duration_s'],
                expected_frames=cfg['duration_s']*cfg['render_hz'])
            try:write_json(output/'preview-timing.json',preview_clock.frames)
            except Exception as error:result.update(passed=False,timing_write_error=repr(error))
            # A completed physics experiment and a real-time preview are distinct
            # outcomes. Never mark the preview verified solely on physical safety.
            if cfg.get('real_time', False):
                timing_pass = result['preview_timing']['soft_realtime_passed']
                result.setdefault('gates', {})['soft_realtime'] = timing_pass
                result['passed'] = bool(result['passed'] and timing_pass)
        if chunk_writer:
            result['checkpoint_evidence']=chunk_writer.summary()
        try:
            for name,value in [('trajectory',rows),('planner',plans),('sensor-frames',frames),('contacts',contacts),('wheel-geometry',wheel_rows)]:
                write_json(output/(name+'.json'),value)
        except Exception as error:result.update(passed=False,evidence_write_error=repr(error))
        if lidar:
            try:lidar.close()
            except Exception as error:result.update(passed=False,sensor_close_error=repr(error))
        if window:window.visible=False
        window=label=lidar=view=monitor=vehicle=None
        if session:
            try:session.close()
            except Exception as error:result.update(passed=False,session_close_error=repr(error))
            try:write_json(output/'lifecycle.json',session.lifecycle)
            except Exception as error:result.update(passed=False,lifecycle_write_error=repr(error))
        result.update(total_wall_s=time.perf_counter()-started,process=process_sample(),
            limitations=['One low-speed physical car; empty adjacent corridor and static obstacle with explicit maximum size prior',
                'Known road and privileged odometry; no learned policy, moving-obstacle tracking, free-space proof, or traffic-rule negotiation',
                'Geometric path-envelope braking, not a calibrated reachable-set guarantee; no highway-speed claim',
                'Real-time status is the measured preview_timing gate for this run, not a hard-deadline or cross-machine guarantee'])
        try:
            write_json(output/'probe-result.json',result)
            print('BYPASS_RESULT='+json.dumps(result),flush=True)
        finally:
            if app:app.close()
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
