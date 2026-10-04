"""One visible physics-driven car, known-map steering and RTX emergency braking.

Launch via scripts/demo_physics_lidar.py. No pose replay, SUMO or learning.
The road map and obstacle geometry are separate from LiDAR controller inputs.
"""
import argparse
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import traceback
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from experiments.probe_support import process_sample,write_json
from traffic.driver_control import AppliedControl,DriverControlGate
from traffic.lane_geometry import load_routes
from traffic.lane_validation import footprint_lateral_bound
from traffic.lidar_braking import LidarBrakeConfig,LidarEmergencyBrake
from traffic.path_following import FollowerConfig,PathFollower,PathPlanner,ScriptedCruiseBehavior,VehicleState
from traffic.physical_lidar import PhysicalLidar,packet_to_scan,rotation_matrix
from traffic.visual_lidar_validation import assess_visual_lidar,barrier_plane_gap,validate_config


def author_barrier(stage,route,config):
    from pxr import Gf,UsdGeom,UsdPhysics,UsdLux
    point=route.evaluate(config['barrier_station_m'])
    cube=UsdGeom.Cube.Define(stage,'/World/Barrier')
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*point.position_xy,.75))
    cube.AddRotateZOp().Set(math.degrees(point.yaw_rad))
    cube.AddScaleOp().Set(Gf.Vec3f(*config['barrier_dimensions_m']))
    cube.CreateDisplayColorAttr([Gf.Vec3f(.95,.23,.06)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdLux.DomeLight.Define(stage,'/World/DemoLight').CreateIntensityAttr(1000)


def make_controls(ui_state):
    import omni.ui as ui
    window=ui.Window('Highway Sim | Physics + LiDAR',width=500,height=225)
    def set_camera(mode):ui_state['camera']=mode
    def toggle(name):ui_state[name]=not ui_state[name]
    with window.frame:
        with ui.VStack(spacing=6):
            ui.Label('PHYSX MOVEMENT  |  SCRIPTED DRIVER  |  NO TRAINING',height=22)
            with ui.HStack(height=30,spacing=5):
                ui.Button('Overview',clicked_fn=lambda:set_camera('overview'))
                ui.Button('Follow car',clicked_fn=lambda:set_camera('follow'))
                ui.Button('Pause / resume',clicked_fn=lambda:toggle('paused'))
            with ui.HStack(height=30,spacing=5):
                ui.Button('Toggle planned path',clicked_fn=lambda:toggle('reference'))
                ui.Button('Toggle LiDAR points',clicked_fn=lambda:toggle('points'))
            label=ui.Label('Warming up physics and sensor...',word_wrap=True)
            ui.Label('Cyan: planned path | Pink: steering target | Green: LiDAR\nUse these controls, not the Kit timeline. Demo closes at its bounded horizon.',word_wrap=True,height=40)
    return window,label


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    output=args.output.resolve()
    if not output.is_relative_to(ROOT/'outputs') or not output.is_dir() or (output/'probe-result.json').exists():
        parser.error('Use the supervised portable launcher and a fresh run directory')
    config=validate_config(json.loads((output/'resolved-config.json').read_text()))
    route=load_routes(ROOT/config['route_file'])[config['route_id']]
    barrier=route.evaluate(config['barrier_station_m'])
    maximum_curvature=max(abs(p.curvature_rad_m) for p in route.sample())
    episode='visual-'+output.name
    app=session=vehicle=lidar=view=monitor=window=label=None
    rows=[];render_checks=[];pose_errors=[];scan_records=[];contact_events=[];history={}
    captures=[];captured=set();saved_scans=set();last_packet_stamp=None;frozen_packet=None
    result=dict(passed=False,no_sumo=True,no_training=True,observation_source='Known map and odometry for steering; RTX returns for emergency braking')
    started=time.perf_counter()
    try:
        from isaacsim import SimulationApp
        app=SimulationApp({'headless':not config['gui'],'width':1280,'height':800,
                          'enable_motion_bvh':True,'disable_viewport_updates':False})
        from traffic.physics_session import prepare_vehicle_runtime
        from traffic.rendered_physics_session import RenderedPhysicsSession
        from traffic.physx_vehicle import PhysxVehicle
        from traffic.vehicle_contacts import VehicleContactMonitor
        from visualization.physics_road_view import author_physics_road
        from isaacsim.core.utils.viewports import set_camera_view
        from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_file
        result['runtime_setup']=prepare_vehicle_runtime()
        session=RenderedPhysicsSession(config['physics_hz'],config['render_hz'])
        vehicle=PhysxVehicle(session.stage,config['physics_hz'])
        view=author_physics_road(session.stage,route)
        author_barrier(session.stage,route,config)
        monitor=VehicleContactMonitor(session.stage,vehicle.path)
        lidar=PhysicalLidar(vehicle.path,episode_id=episode,vehicle_id='ego',
                            tick_source=lambda:session.physics_steps,points=config['points'])
        ui_state=dict(camera=config['camera'],reference=True,points=config['points'],paused=False)
        if config['gui']:window,label=make_controls(ui_state)
        viewport=get_active_viewport()
        if viewport is None:raise RuntimeError('No viewport available for the visible demonstration')
        viewport.resolution=(960,600)
        # Ground-level overlays are visible geometry, not sensor-excluded claims.
        static_before=view.static_layer.ExportToString()
        view.static_layer.Export(str(output/'road-static.usda'))
        write_json(output/'scene-contract.json',dict(vehicle=vehicle.metadata,view=view.metadata,sensor=lidar.metadata,
            physics_owner='PhysX',barrier_evaluation_only=asdict(barrier),braking=config['braking']))
        session.stage.Flatten().Export(str(output/'scene-initial.usda'))
        vehicle.apply(AppliedControl(0,0,1,'initial_hold',None,False),700,1500)
        session.start()
        epoch=session.absolute_time_s
        result['clock_start']=session.snapshot()
        state=vehicle.state();history[0]=state
        planner=PathPlanner({route.route_id:route},episode,'ego')
        follower=PathFollower({route.route_id:route},episode,'ego',FollowerConfig(**config['follower']))
        behavior=ScriptedCruiseBehavior(episode,'ego',route.route_id,target_speed_m_s=3)
        stop_behavior=ScriptedCruiseBehavior(episode,'ego',route.route_id,target_speed_m_s=0)
        gate=DriverControlGate(episode,'ego')
        brake=LidarEmergencyBrake(episode,'ego',LidarBrakeConfig(**config['braking']))
        reference=None;decision=None;driver={};loop_started=time.perf_counter();paused_wall=0
        result['startup_wall_s']=loop_started-started
        for tick in range(round(config['duration_s']*config['physics_hz'])):
            if time.perf_counter()-started>config['max_process_seconds']-20:
                raise TimeoutError('Demo reached its bounded process deadline')
            if not app.is_running():raise RuntimeError('User closed the demo before completion')
            while ui_state['paused']:
                if not app.is_running():raise RuntimeError('User closed the paused demo')
                if time.perf_counter()-started>config['max_process_seconds']-20:raise TimeoutError('Paused demo deadline')
                before=time.perf_counter();session.render();time.sleep(.02);paused_wall+=time.perf_counter()-before
            phase='settle' if tick<round(config['settle_s']*config['physics_hz']) else 'drive'
            if tick%12==0:reference=planner.plan((stop_behavior if phase=='settle' else behavior).decide(tick),tick)
            command=None
            if tick%2==0:
                packet=lidar.latest
                if config['mode']=='dropout' and tick/config['physics_hz']>=config['dropout_at_s']:
                    if frozen_packet is None:frozen_packet=packet
                    packet=frozen_packet
                scan=packet_to_scan(packet,state,episode_id=episode,vehicle_id='ego',tick=tick,epoch_s=epoch)
                decision=brake.evaluate(scan,tick=tick,speed_m_s=state['speed_m_s'])
                # A few raw clouds make key decisions independently auditable,
                # without turning the demonstration into a bulk scan recorder.
                event=('obstacle-latch' if decision.stop_latched else
                       'dropout-expiry' if config['mode']=='dropout' and tick>=1440
                            and decision.status=='stale_invalid' else
                       'moving-clear' if tick==600 else None)
                if packet is not None and event is not None and event not in saved_scans:
                    np.savez_compressed(output/f'{event}.npz',xyz=packet['xyz'],
                                        flags=packet['flags'],offset_ns=packet['offset_ns'])
                    write_json(output/f'{event}.json',dict(episode_id=episode,vehicle_id='ego',
                        timestamp_ns=packet['timestamp_ns'],scan_complete=packet['scan_complete'],
                        delivery_tick=packet['delivery_tick'],tick=tick,epoch_s=epoch,
                        state=state,decision=asdict(decision)))
                    saved_scans.add(event)
                command=follower.command(VehicleState.from_physics(state,episode_id=episode,vehicle_id='ego',tick=tick),tick,reference)
                driver=dict(follower.last_diagnostics)
                if decision.brake_override:
                    command=replace(command,throttle=0.0,brake=decision.brake_override)
            applied=gate.step(tick=tick,dt_s=1/config['physics_hz'],command=command)
            controls=vehicle.apply(applied,700,1500)
            step_started=time.perf_counter();clock=session.step();state=vehicle.state();history[tick+1]=state
            contact_events.extend(monitor.sample(session.sim,tick+1))
            projection=route.project(*state['position_m'][:2])
            row=dict(tick=tick+1,sim_time_s=(tick+1)/config['physics_hz'],absolute_sim_time_s=clock['absolute_time_s'],
                episode_id=episode,vehicle_id='ego',phase=phase,control_tick=tick%2==0,control=controls,
                driver=driver,lidar=asdict(decision),progress_m=projection.s_m,lateral_error_m=projection.lateral_error_m,
                footprint_lateral_bound_m=footprint_lateral_bound(state,projection,[4.8,1.8,1.4],maximum_curvature),
                barrier_gap_m=barrier_plane_gap(state,barrier),physics_wall_ms=(time.perf_counter()-step_started)*1000,**state)
            rows.append(row)
            if contact_events or row['footprint_lateral_bound_m']>1.8 or state['upright_z']<.99:
                raise RuntimeError('Contact, road departure or instability: demo stopped')
            if (tick+1)%4==0:
                view.update(state,target_xy=driver.get('target_xy_m'),show_reference=ui_state['reference'])
                camera=view.camera_pose(ui_state['camera'],state)
                set_camera_view(eye=np.array(camera.eye),target=np.array(camera.target))
                lidar.set_points(ui_state['points'])
                if label:
                    gap=decision.nearest_gap_m
                    label.text=(f"{ui_state['camera'].upper()} | {row['sim_time_s']:.1f} s | speed {state['speed_m_s']:.2f} m/s\n"
                        f"Lane error {projection.lateral_error_m:+.3f} m | progress {projection.s_m:.1f} /100m\n"
                        f"LiDAR: {decision.status.upper()} | gap {'--' if gap is None else f'{gap:.2f} m'}")
                before=session.snapshot();session.render();after=session.snapshot()
                render_checks.append(before['physics_steps']==after['physics_steps'] and before['absolute_time_s']==after['absolute_time_s'])
                packet=lidar.latest
                if packet is not None and packet['timestamp_ns']!=last_packet_stamp:
                    last_packet_stamp=packet['timestamp_ns']
                    frame_tick=round((packet['frame_end_ns']*1e-9-epoch)*config['physics_hz'])
                    frame_state=history.get(frame_tick)
                    error=None
                    if frame_state is not None:
                        expected=np.array(frame_state['position_m'])+rotation_matrix(frame_state['quaternion_xyzw'])@np.array(lidar.mount_translation_m)
                        error=float(np.linalg.norm(expected-packet['sensor_position_world_m']))
                        pose_errors.append(error)
                    acquisition_times=(packet['timestamp_ns']+packet['offset_ns'].astype(np.int64))*1e-9-epoch
                    scan_records.append(dict(timestamp_ns=packet['timestamp_ns'],frame_end_ns=packet['frame_end_ns'],
                        acquisition_start_tick=math.floor(float(acquisition_times.min())*config['physics_hz']+1e-6),
                        acquisition_end_tick=math.ceil(float(acquisition_times.max())*config['physics_hz']-1e-6),
                        delivery_tick=packet['delivery_tick'],scan_complete=packet['scan_complete'],
                        elements=len(packet['xyz']),valid_hits=int(((packet['flags']&64)!=0).sum()),
                        offset_min_ns=int(packet['offset_ns'].min()),offset_max_ns=int(packet['offset_ns'].max()),
                        frame_tick=frame_tick,sensor_pose_error_m=error,sensor_position_world_m=packet['sensor_position_world_m'].tolist()))
                if lidar.errors:raise RuntimeError('RTX capture error: '+repr(lidar.errors))
                if config['capture']:
                    targets=[1,6] if config['mode']=='smoke' else [1,12,28]
                    for second in targets:
                        if row['sim_time_s']>=second and second not in captured:
                            name=f"preview-{second:02d}s-{ui_state['camera']}.png"
                            captures.append(capture_viewport_to_file(viewport,str(output/name)))
                            captured.add(second)
                if config['paced']:
                    delay=loop_started+paused_wall+row['sim_time_s']-time.perf_counter()
                    if delay>0:time.sleep(min(delay,1/config['render_hz']))
            if (tick+1)%120==0:
                write_json(output/'trajectory.json',rows)
                write_json(output/'sensor-frames.json',scan_records)
                if (tick+1)%600==0:print(f"PHYSICS_LIDAR_TIME={row['sim_time_s']:.1f} PROGRESS={projection.s_m:.2f} STATUS={decision.status}",flush=True)
        result.update(assess_visual_lidar(rows,config,contacts=contact_events,errors=lidar.errors,
            render_clock_checks=render_checks,sensor_pose_errors=pose_errors,sensor_frames=scan_records))
        result.update(loop_wall_s=time.perf_counter()-loop_started,clock_end=session.snapshot(),
            packet_count=lidar.packet_count,static_road_unchanged=view.static_layer.ExportToString()==static_before,
            sensor_errors=lidar.errors,preview_files=[p.name for p in output.glob('preview-*.png')])
        result['passed']=bool(result['passed'] and result['static_road_unchanged'])
        # Drain pending captures without advancing physics.
        for _ in range(3):session.render()
    except Exception as error:
        result.update(passed=False,error=repr(error),traceback=traceback.format_exc())
        traceback.print_exc()
    finally:
        try:
            write_json(output/'trajectory.json',rows)
            write_json(output/'sensor-frames.json',scan_records)
            write_json(output/'contacts.json',contact_events)
        except Exception as error:
            result.update(passed=False,evidence_write_error=repr(error))
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
            limitations=['One scripted physical car, static barrier, gentle known-map curve; not learned or general LiDAR autonomy',
                         'WORLD scans reprojected using privileged current odometry; no moving-object deskew/tracking claim',
                         'Bounded demonstration; not the full60s sensor/reset/communication/endurance roadmap gate'])
        try:
            write_json(output/'probe-result.json',result)
            print('PHYSICS_LIDAR_RESULT='+json.dumps(result),flush=True)
        finally:
            if app:app.close()
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
