"""Actual PhysX loop run; use demo_highway_loop.py to supervise and record it."""
import argparse
from dataclasses import asdict
import json
import hashlib
import math
from pathlib import Path
import sys
import time
import traceback

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from experiments.highway_loop_config import assess, footprint_lane_error, validate_config
from experiments.probe_support import process_sample, write_json
from traffic.driver_control import AppliedControl, DriverCommand, DriverControlGate
from traffic.evidence_chunks import EvidenceChunkWriter
from traffic.loop_driving import CircularLoop, LoopFollower, LoopProgress, LoopReference
from traffic.path_following import VehicleState
from visualization.preview_timing import PreviewClock, rendering_profile
from usd.physical_scene import PATHS


def package_hashes(directory):
    return {str(p.relative_to(directory)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in directory.rglob('*') if p.is_file()}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    output=parser.parse_args().output.resolve()
    if not output.is_relative_to(ROOT/'outputs') or not output.is_dir() or (output/'probe-result.json').exists():
        parser.error('Use the supervisor with a fresh output directory')
    cfg=json.loads((output/'resolved-config.json').read_text())
    validate_config(cfg)
    route=CircularLoop.from_navigation(ROOT/cfg['source_directory']/'navigation.json',cfg['lane_id'])
    episode='loop-'+output.name
    app=session=vehicle=monitor=view=window=label=viewport=None
    preview=writer=None
    rows=[]; contacts=[]; captures=[]; captured=set(); plans=[]; camera_checks=[]
    result=dict(passed=False,no_sumo=True,no_training=True,no_lidar=True,
                observation_source=cfg['observations'],physics_authority='Isaac PhysX')
    started=time.perf_counter()
    try:
        from isaacsim import SimulationApp
        graphics=rendering_profile(True)
        app=SimulationApp({**graphics,'headless':not cfg['gui'],'disable_viewport_updates':False})
        from traffic.physics_session import prepare_vehicle_runtime
        from traffic.rendered_physics_session import RenderedPhysicsSession
        from traffic.physx_vehicle import PhysxVehicle
        from traffic.vehicle_contacts import VehicleContactMonitor
        from usd.highway_loop_scene import (install_highway_loop,spawn_vehicle_on_loop,
            validate_highway_loop_scene,save_highway_loop_scene)
        from visualization.highway_loop_view import HighwayLoopView, make_controls
        from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_file
        result['runtime_setup']=prepare_vehicle_runtime()
        scene_directory=output/'scene'
        session=RenderedPhysicsSession(cfg['physics_hz'],cfg['render_hz'],physics_scene_path=PATHS.physics)
        vehicle=PhysxVehicle(session.stage,cfg['physics_hz'],scene_directory=scene_directory)
        result['environment']=install_highway_loop(session.stage,scene_directory,ROOT/cfg['source_directory'])
        result['spawn']=spawn_vehicle_on_loop(session.stage,angle_rad=cfg['start_angle_rad'],radius_m=route.radius_m)
        # Correct the Factory adapter's default-ground description for THIS fixture.
        vehicle.metadata['ground']='V02 shared finite road triangle mesh; prototype infinite plane disabled'
        monitor=VehicleContactMonitor(session.stage,vehicle.path)
        view=HighwayLoopView(session.stage,route)
        ui_state=dict(camera=cfg['camera'],paused=False)
        if cfg['gui']: window,label=make_controls(ui_state)
        viewport=get_active_viewport()
        if viewport is None:raise RuntimeError('No active viewport')
        viewport.resolution=(graphics['width'],graphics['height'])
        initial=dict(position_m=[route.radius_m*math.cos(cfg['start_angle_rad']),
                                 route.radius_m*math.sin(cfg['start_angle_rad']),1.],
                     yaw_rad=cfg['start_angle_rad']+math.pi/2)
        viewport.camera_path=view.update(initial,cfg['start_angle_rad']*route.radius_m,cfg['camera'])
        result['scene_structure']=validate_highway_loop_scene(session.stage)
        vehicle.apply(AppliedControl(0,0,1,'initial_hold',None,False),700,1500)
        result['composed_scene']=save_highway_loop_scene(session.stage,scene_directory,view.dynamic_layer)
        static_hashes=package_hashes(scene_directory)
        static_layers=[layer for layer in session.stage.GetUsedLayers() if not layer.anonymous]
        static_text={layer.identifier:layer.ExportToString() for layer in static_layers}
        write_json(output/'scene-contract.json',dict(route=asdict(route),vehicle=vehicle.metadata,
            environment=result['environment'],spawn=result['spawn'],scene_structure=result['scene_structure']))
        gate=DriverControlGate(episode,'ego')
        follower=LoopFollower(route,episode,'ego')
        progress=LoopProgress(route)
        session.start()
        result['clock_start']=session.snapshot()
        state=vehicle.state()
        progress.update(route.project(*state['position_m'][:2]).s_m)
        writer=EvidenceChunkWriter(output)
        preview=PreviewClock(paced=cfg['paced'])
        paused_wall=0.; loop_started=time.perf_counter()
        result['startup_wall_s']=loop_started-started
        hz=cfg['physics_hz']; settle_ticks=cfg['settle_s']*hz
        brake_tick=(cfg['settle_s']+cfg['drive_s'])*hz
        total_ticks=cfg['duration_s']*hz
        diagnostics={}
        traveled_distance=0.; command_expires=None
        for tick in range(total_ticks):
            if not app.is_running() or time.perf_counter()-started>cfg['max_process_seconds']-20:
                raise RuntimeError('Run closed or reached its deadline')
            while ui_state['paused']:
                if not app.is_running() or time.perf_counter()-started>cfg['max_process_seconds']-20:
                    raise RuntimeError('Paused run closed or timed out')
                before=time.perf_counter();session.render();time.sleep(.02)
                paused_wall+=time.perf_counter()-before
            phase='settle' if tick<settle_ticks else ('brake' if tick>=brake_tick else 'drive')
            if tick==settle_ticks:
                progress=LoopProgress(route)
                progress.update(route.project(*state['position_m'][:2]).s_m)
            command=None
            if phase=='settle':
                if tick%2==0:
                    command=DriverCommand(episode,'ego',tick,tick,tick+12,0.,0.,1.)
            elif not (phase=='brake' and cfg['dropout']) and tick%2==0:
                reference=LoopReference(episode,'ego',route.route_id,tick,tick+12,
                                        cfg['target_speed_m_s'],stop_requested=phase=='brake')
                command=follower.command(VehicleState.from_physics(state,episode_id=episode,
                    vehicle_id='ego',tick=tick),tick,reference)
                # Gate sequence belongs to the whole physical episode, including settle.
                command=DriverCommand(command.episode_id,command.vehicle_id,tick,
                    command.issued_tick,command.expires_tick,command.steering_rad,command.throttle,command.brake)
                diagnostics=dict(follower.last_diagnostics)
            if command is not None:command_expires=command.expires_tick
            control=gate.step(tick=tick,dt_s=1/hz,command=command)
            vehicle.apply(control,cfg['drive_torque_per_front_wheel_nm'],cfg['brake_torque_per_wheel_nm'])
            session.step()
            previous_position=state['position_m']
            state=vehicle.state()
            traveled_distance+=math.dist(previous_position[:2],state['position_m'][:2])
            events=monitor.sample(session.sim,tick+1);contacts.extend(events)
            projection=route.project(*state['position_m'][:2])
            tracking=progress.update(projection.s_m)
            row=dict(tick=tick+1,applied_tick=tick,sim_time_s=(tick+1)/hz,phase=phase,
                traveled_distance_m=traveled_distance,command_expires_tick=command_expires,
                position_m=state['position_m'],yaw_rad=state['yaw_rad'],speed_m_s=state['speed_m_s'],
                upright_z=state['upright_z'],wheel_on_ground=state['wheel_on_ground'],
                lane_error_m=projection.lateral_error_m,
                footprint_error_m=footprint_lane_error(state,route.radius_m),
                **tracking,throttle=control.throttle,brake=control.brake,steering_rad=control.steering_rad,
                fallback=control.is_fallback,follower_fallback=diagnostics.get('is_fallback',False),
                follower_stop_latched=diagnostics.get('stop_latched',False),
                control_reason=control.reason,follower_reason=diagnostics.get('reason','settle'),
                command_sequence=control.command_sequence)
            rows.append(row)
            if phase!='settle' and (abs(row['lane_error_m'])>1 or row['upright_z']<.9 or not .4<state['position_m'][2]<1.8):
                raise RuntimeError('Physical lane/attitude/height safety guard')
            if events:raise RuntimeError('Unexpected vehicle rigid contact')
            if (tick+1)%session.render_every_steps==0:
                viewport.camera_path=view.update(state,projection.s_m,ui_state['camera'])
                if label:
                    label.text=(f"{cfg['profile']} | {phase} | {state['speed_m_s']/0.44704:.1f} mph\n"
                        f"Lap distance {tracking['progress_m']:.1f} / {route.length_m:.1f} m | "
                        f"completed {tracking['completed_laps']} | lane error {row['lane_error_m']:.3f} m")
                session.render()
                camera_check=view.camera_check()
                camera_checks.append(dict(tick=tick+1,**camera_check))
                if not camera_check['passed']:
                    raise RuntimeError('Composed follow/overview camera differs from requested pose')
                if cfg['capture']:
                    for second in (10,cfg['settle_s']+cfg['drive_s']-1):
                        if row['sim_time_s']>=second and second not in captured:
                            captures.append(capture_viewport_to_file(viewport,str(output/f'preview-{second:03d}s-{ui_state["camera"]}.png')))
                            captured.add(second)
                preview.finish_frame(row['sim_time_s'],paused_wall)
            if (tick+1)%120==0:writer.write_checkpoint(rows,plans)
            if (tick+1)%1200==0:
                print(f"LOOP_TIME={row['sim_time_s']:.0f} SPEED={row['speed_m_s']:.3f} DISTANCE={tracking['progress_m']:.2f} LANE_ERROR={row['lane_error_m']:.4f}",flush=True)
        result.update(assess(rows,cfg,contact_count=len(contacts),route_length_m=route.length_m))
        extra=dict(follower_valid=all(not r['follower_fallback'] for r in rows),
            camera_tracking=bool(camera_checks) and all(c['passed'] for c in camera_checks),
            scene_structure=validate_highway_loop_scene(session.stage)['passed'],
            asset_files_unchanged=package_hashes(scene_directory)==static_hashes,
            static_layers_unchanged=all(layer.ExportToString()==static_text[layer.identifier] for layer in static_layers))
        result['gates'].update(extra)
        result['passed']=bool(result['passed'] and all(extra.values()))
        result.update(clock_end=session.snapshot(),loop_wall_s=time.perf_counter()-loop_started,
                      final_position_m=state['position_m'])
        for _ in range(3):session.render()
    except Exception as error:
        result.update(passed=False,error=repr(error),traceback=traceback.format_exc())
        traceback.print_exc()
    finally:
        if preview:
            result['preview_timing']=preview.summary(expected_simulation_time_s=cfg['duration_s'],
                expected_frames=cfg['duration_s']*cfg['render_hz'])
            write_json(output/'preview-timing.json',preview.frames)
        if writer:result['checkpoint_evidence']=writer.summary()
        write_json(output/'trajectory.json',rows)
        write_json(output/'contacts.json',contacts)
        write_json(output/'camera-checks.json',camera_checks)
        write_json(output/'pre-close-result.json',dict(result,cleanup_verified=False))
        if session and session.started and viewport:
            try:
                viewport.camera_path='/OmniverseKit_Persp';session.render()
            except Exception as error:result.update(passed=False,camera_close_error=repr(error))
        if window:window.visible=False
        window=label=view=monitor=vehicle=None
        static_layers=None
        if session:
            try:session.close()
            except Exception as error:result.update(passed=False,session_close_error=repr(error))
            write_json(output/'lifecycle.json',session.lifecycle)
        result.update(total_wall_s=time.perf_counter()-started,process=process_sample(),
            preview_files=[p.name for p in output.glob('preview-*.png')],
            limitations=['One Factory physics car and known-map scripted control; not calibrated highway handling',
                'No LiDAR, learned driving, traffic spawning, lane changes, V2V or AV-ratio comparison',
                'Saved USD is an initial-state assembly, not an animation or a runnable controller',
                'Soft real-time result is reported separately from physical acceptance; no hard deadline guarantee'])
        try:
            write_json(output/'probe-result.json',result)
            print('HIGHWAY_LOOP_RESULT='+json.dumps(dict(passed=result['passed'],
                  summary_path=str(output/'probe-result.json'),error=result.get('error'))),flush=True)
        finally:
            if app:app.close()
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
