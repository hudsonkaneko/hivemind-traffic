"""Supervised live showcase; launch through demo_loop_showcase.py."""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
import time
import traceback

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from experiments.loop_showcase_config import assess,fleet_specs,validate_config
from experiments.probe_support import process_sample,write_json
from traffic.bypass_validation import OrientedBox,box_clearance
from traffic.driver_control import AppliedControl,DriverCommand,DriverControlGate
from traffic.evidence_chunks import EvidenceChunkWriter
from traffic.loop_driving import CircularLoop,LoopProgress
from traffic.loop_showcase_control import ShowcaseConfig,ShowcaseController
from traffic.path_following import VehicleState
from visualization.preview_timing import PreviewClock,rendering_profile
from usd.physical_scene import PATHS
from usd.showcase_vehicle import package_hashes


def body_box(state,metadata):
    x,y=state['position_m'][:2];a=state['yaw_rad'];c,s=math.cos(a),math.sin(a)
    ox,oy=metadata['chassis_collision_center_m'][:2]
    length,width=metadata['chassis_collision_dimensions_m'][:2]
    return OrientedBox(x+c*ox-s*oy,y+s*ox+c*oy,a,length,width)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    output=parser.parse_args().output.resolve()
    if not output.is_relative_to(ROOT/'outputs') or not output.is_dir() or (output/'probe-result.json').exists():
        parser.error('Fresh supervisor-created output directory required')
    cfg=json.loads((output/'resolved-config.json').read_text());validate_config(cfg)
    route=CircularLoop();episode='showcase-'+output.name
    app=session=vehicle=fleet=monitor=view=window=label=viewport=writer=preview=None
    static_layers=None;rows=[];plans=[];contacts=[];camera_checks=[];captures=[];captured=set()
    passed=set();passed_records=[];seen_ahead=set();peers={};last_peer_stations={}
    started=time.perf_counter();result=dict(passed=False,no_training=True,no_lidar=True,no_sumo=True,
        observations=cfg['observations'],main_motion='dynamic_physx',background_motion='scripted_kinematic_physx')
    try:
        from isaacsim import SimulationApp
        graphics=rendering_profile(True)
        app=SimulationApp({**graphics,'headless':not cfg['gui'],'disable_viewport_updates':False})
        from traffic.physics_session import prepare_vehicle_runtime
        from traffic.rendered_physics_session import RenderedPhysicsSession
        from traffic.vehicle_contacts import VehicleContactMonitor
        from usd.showcase_vehicle import compose_prepared_vehicle
        from usd.showcase_fleet import KinematicFleet
        from usd.highway_loop_scene import install_highway_loop,spawn_vehicle_on_loop,validate_highway_loop_scene,save_highway_loop_scene
        from visualization.loop_showcase_view import LoopShowcaseView,make_controls
        from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_file
        from pxr import Sdf,Usd
        result['runtime_setup']=prepare_vehicle_runtime()
        source_hashes=package_hashes(ROOT/cfg['vehicle_directory'])
        scene_directory=output/'scene'
        session=RenderedPhysicsSession(cfg['physics_hz'],cfg['render_hz'],physics_scene_path=PATHS.physics)
        vehicle=compose_prepared_vehicle(session.stage,scene_directory,ROOT/cfg['vehicle_directory'],cfg['physics_hz'])
        result['environment']=install_highway_loop(session.stage,scene_directory,ROOT/cfg['source_directory'])
        result['spawn']=spawn_vehicle_on_loop(session.stage,0.,radius_m=500+3.7*cfg['initial_lane'])
        # Local publication's point light was for an asset inspection, not a 1 km road.
        session.stage.GetPrimAtPath('/World/Lighting/Key').SetActive(False)
        fleet=KinematicFleet(session.stage,scene_directory,fleet_specs(cfg),episode_id=episode)
        monitor=VehicleContactMonitor(session.stage,vehicle.path)
        view=LoopShowcaseView(session.stage,route)
        ui_state=dict(camera=cfg['camera'],paused=False)
        if cfg['gui']:window,label=make_controls(ui_state)
        viewport=get_active_viewport()
        if viewport is None:raise RuntimeError('No viewport is available')
        viewport.resolution=(graphics['width'],graphics['height'])
        model=vehicle.metadata
        controller_config=ShowcaseConfig(target_speed_m_s=cfg['target_speed_m_s'],
            wheelbase_m=vehicle.wheelbase_m,rear_axle_offset_m=vehicle.wheelbase_m/2,
            ego_length_m=model['chassis_collision_dimensions_m'][0],
            ego_width_m=model['chassis_collision_dimensions_m'][1],
            ego_collision_offset_x_m=model['chassis_collision_center_m'][0])
        controller=ShowcaseController(episode,'ego',controller_config,initial_lane=cfg['initial_lane'])
        initial=dict(position_m=result['spawn']['position_m'],yaw_rad=math.pi/2,speed_m_s=0.)
        initial_state=VehicleState.from_physics(initial,episode_id=episode,vehicle_id='ego',tick=0)
        viewport.camera_path=view.update_showcase(initial,0.,cfg['camera'],controller.path_points(initial_state))
        vehicle.apply(AppliedControl(0,0,1,'initial_hold',None,False),cfg['drive_torque_nm'],cfg['brake_torque_nm'])
        result['scene_structure']=validate_highway_loop_scene(session.stage)
        result['composed_scene']=save_highway_loop_scene(session.stage,scene_directory,view.dynamic_layer)
        saved=Sdf.Layer.FindOrOpen(result['composed_scene'])
        saved.subLayerPaths.insert(2,'./showcase-fleet-layout.usda');saved.Save();saved=None
        static_hashes=package_hashes(scene_directory)
        static_layers=[layer for layer in session.stage.GetUsedLayers() if not layer.anonymous]
        static_text={layer.identifier:layer.ExportToString() for layer in static_layers}
        write_json(output/'scene-contract.json',dict(vehicle=model,source_vehicle_hashes=source_hashes,
            fleet=fleet.metadata,controller=asdict(controller_config),environment=result['environment'],spawn=result['spawn']))
        gate=DriverControlGate(episode,'ego');progress=LoopProgress(route)
        session.start();fleet.bind(session.manager.get_physics_simulation_view())
        result['clock_start']=session.snapshot();state=vehicle.state()
        progress.update(route.project(*state['position_m'][:2]).s_m % route.length_m)
        for t in fleet.tracks(0):
            station=route.project(t.x_m,t.y_m).s_m % route.length_m
            peers[t.vehicle_id]=station
            last_peer_stations[t.vehicle_id]=station
        # Compile/load the first visible frame before the live clock begins.
        # Each render verifies zero physics advancement; report readiness cost.
        warmup_started=time.perf_counter()
        for _ in range(4):session.render()
        result['renderer_warmup_s']=time.perf_counter()-warmup_started
        writer=EvidenceChunkWriter(output);preview=PreviewClock(paced=cfg['paced'])
        result['startup_wall_s']=time.perf_counter()-started
        hz=cfg['physics_hz'];brake_tick=(cfg['settle_s']+cfg['drive_s'])*hz
        traveled=0.;paused_wall=0.;expires=None;diagnostics={};max_lanes=0
        for tick in range(cfg['duration_s']*hz):
            if not app.is_running() or time.perf_counter()-started>cfg['max_process_seconds']-20:
                raise RuntimeError('Preview closed or bounded deadline reached')
            while ui_state['paused']:
                if not app.is_running() or time.perf_counter()-started>cfg['max_process_seconds']-20:
                    raise RuntimeError('Paused preview closed or bounded deadline reached')
                before=time.perf_counter();session.render();time.sleep(.02);paused_wall+=time.perf_counter()-before
            phase='settle' if tick<cfg['settle_s']*hz else 'brake' if tick>=brake_tick else 'drive'
            tracks=fleet.tracks(tick)
            command=None
            physical_state=VehicleState.from_physics(state,episode_id=episode,vehicle_id='ego',tick=tick)
            if tick%2==0:
                if phase=='settle':command=DriverCommand(episode,'ego',tick,tick,tick+12,0,0,1)
                elif not (phase=='brake' and cfg['mode']=='dropout'):
                    command=controller.command(physical_state,tick,tracks,stop=phase=='brake')
                    if cfg['mode']=='contact-check' and phase=='drive':
                        # Intentional positive control only: ignore objects but retain lane steering.
                        reference_angle=math.atan2(2*vehicle.wheelbase_m*math.sin(.01),10.)
                        command=DriverCommand(episode,'ego',tick,tick,tick+12,reference_angle,.8,0)
                    else:
                        command=DriverCommand(command.episode_id,command.vehicle_id,tick,command.issued_tick,
                            command.expires_tick,command.steering_rad,command.throttle,command.brake)
                    diagnostics=dict(controller.diagnostics)
            if command:expires=command.expires_tick
            applied=gate.step(tick=tick,dt_s=1/hz,command=command)
            vehicle.apply(applied,cfg['drive_torque_nm'],cfg['brake_torque_nm'])
            fleet.update((tick+1)/hz)
            session.step();old=state;state=vehicle.state()
            traveled+=math.dist(old['position_m'][:2],state['position_m'][:2])
            events=monitor.sample(session.sim,tick+1);contacts.extend(events)
            tracks=fleet.tracks(tick+1);pose_check=fleet.pose_check((tick+1)/hz)
            projection=route.project(*state['position_m'][:2]);tracking=progress.update(projection.s_m % route.length_m)
            ego_box=body_box(state,model)
            gaps=[box_clearance(ego_box,OrientedBox(t.x_m,t.y_m,t.yaw_rad,t.length_m,t.width_m)) for t in tracks]
            for t in tracks:
                station=route.project(t.x_m,t.y_m).s_m % route.length_m
                previous=last_peer_stations.get(t.vehicle_id)
                if previous is not None:peers[t.vehicle_id]+=(station-previous+route.length_m/2)%route.length_m-route.length_m/2
                last_peer_stations[t.vehicle_id]=station
                relative=peers[t.vehicle_id]-tracking['progress_m']
                if relative>=(ego_box.length_m+t.length_m)/2+2:seen_ahead.add(t.vehicle_id)
                if t.vehicle_id in seen_ahead and t.vehicle_id not in passed and relative<=-((ego_box.length_m+t.length_m)/2+2):
                    passed.add(t.vehicle_id);passed_records.append(dict(vehicle_id=t.vehicle_id,tick=tick+1,sim_time_s=(tick+1)/hz))
            max_lanes=max(max_lanes,int(diagnostics.get('lane_changes',0)))
            # Positive radial boundary uses every corner of the true local collision envelope.
            in_road=all(498.15<=math.hypot(x,y)<=512.95 for x,y in ego_box.corners())
            pose_error=pose_check['max_position_error_m']
            row=dict(tick=tick+1,applied_tick=tick,sim_time_s=(tick+1)/hz,phase=phase,
                position_m=state['position_m'],yaw_rad=state['yaw_rad'],speed_m_s=state['speed_m_s'],
                upright_z=state['upright_z'],wheel_on_ground=state['wheel_on_ground'],
                traveled_distance_m=traveled,progress_m=tracking['progress_m'],
                clearance_m=min(g['clearance_m'] for g in gaps),overlap=any(g['overlap'] for g in gaps),
                footprint_in_road=in_road,background_pose_error_m=pose_error,background_native_passed=pose_check['passed'],
                throttle=applied.throttle,brake=applied.brake,steering_rad=applied.steering_rad,
                fallback=applied.is_fallback,command_expires_tick=expires,
                controller_fallback=diagnostics.get('is_fallback',False),
                controller_status=('changing_lane' if diagnostics.get('maneuver_active') else 'following'),controller_reason=diagnostics.get('reason','settle'),
                lane=diagnostics.get('lane',cfg['initial_lane']),completed_lane_changes=max_lanes,
                completed_passes=len(passed))
            rows.append(row)
            if (tick+1)%12==0:plans.append(dict(tick=tick+1,diagnostics=diagnostics,objects=[asdict(t) for t in tracks]))
            if cfg['mode']=='contact-check' and events:break
            if cfg['mode']!='contact-check' and (events or row['overlap'] or (phase!='settle' and (not in_road or state['upright_z']<.9))):
                raise RuntimeError('Contact/overlap/road/attitude safety guard')
            if (tick+1)%session.render_every_steps==0:
                physical_state=VehicleState.from_physics(state,episode_id=episode,vehicle_id='ego',tick=tick+1)
                viewport.camera_path=view.update_showcase(state,projection.s_m,ui_state['camera'],controller.path_points(physical_state))
                if label:label.text=(f"{cfg['version']} | {phase} | {state['speed_m_s']/0.44704:.1f} mph | traffic 15–19 mph\n"
                    f"Passes {len(passed)} | lane changes {max_lanes} | clearance {row['clearance_m']:.2f} m\n"
                    f"{row['controller_status']}: {row['controller_reason']} | {row['sim_time_s']:.0f}s")
                session.render();check=view.camera_check();camera_checks.append(dict(tick=tick+1,**check))
                if not check['passed']:raise RuntimeError('Camera pose ownership failed')
                if cfg['capture']:
                    for second in (10,20,35,55,cfg['settle_s']+cfg['drive_s']-1):
                        if row['sim_time_s']>=second and second not in captured:
                            captures.append(capture_viewport_to_file(viewport,str(output/f'preview-{second:03d}s-{ui_state["camera"]}.png')))
                            captured.add(second)
                preview.finish_frame(row['sim_time_s'],paused_wall)
            if (tick+1)%hz==0:writer.write_checkpoint(rows,plans)
            if (tick+1)%(hz*10)==0:
                print(f"SHOWCASE t={row['sim_time_s']:.0f} speed={row['speed_m_s']:.2f} passes={len(passed)} lane_changes={max_lanes} gap={row['clearance_m']:.2f} state={row['controller_status']} {row['controller_reason']}",flush=True)
        result.update(assess(rows,cfg,len(contacts),passed,max_lanes))
        extra=dict(camera=bool(camera_checks) and all(c['passed'] for c in camera_checks),
            background_native_contract=bool(rows) and all(r['background_native_passed'] for r in rows),
            controller_valid=all(not r['controller_fallback'] for r in rows if r['phase']=='drive') if cfg['mode']!='contact-check' else True,
            scene_structure=validate_highway_loop_scene(session.stage)['passed'],
            static_files_unchanged=package_hashes(scene_directory)==static_hashes,
            static_layers_unchanged=all(layer.ExportToString()==static_text[layer.identifier] for layer in static_layers),
            source_vehicle_unchanged=package_hashes(ROOT/cfg['vehicle_directory'])==source_hashes)
        result['gates'].update(extra);result['passed']=all(result['gates'].values())
        result.update(clock_end=session.snapshot(),passes=passed_records)
        for _ in range(3):session.render()
    except Exception as error:
        result.update(passed=False,error=repr(error),traceback=traceback.format_exc());traceback.print_exc()
    finally:
        if preview:
            result['preview_timing']=preview.summary(expected_simulation_time_s=cfg['duration_s'],expected_frames=cfg['duration_s']*cfg['render_hz'])
            write_json(output/'preview-timing.json',preview.frames)
        if writer:result['checkpoint_evidence']=writer.summary()
        write_json(output/'trajectory.json',rows);write_json(output/'planner.json',plans)
        write_json(output/'contacts.json',contacts);write_json(output/'camera-checks.json',camera_checks)
        write_json(output/'pre-close-result.json',dict(result,cleanup_verified=False))
        if session and session.started and viewport:
            try:viewport.camera_path='/OmniverseKit_Persp';session.render()
            except Exception as error:result.update(passed=False,camera_close_error=repr(error))
        if window:window.visible=False
        if fleet:fleet.release()
        window=label=view=monitor=vehicle=fleet=None;static_layers=None
        if session:
            try:session.close()
            except Exception as error:result.update(passed=False,session_close_error=repr(error))
            write_json(output/'lifecycle.json',session.lifecycle)
        result.update(total_wall_s=time.perf_counter()-started,process=process_sample(),preview_files=[p.name for p in output.glob('preview-*.png')])
        write_json(output/'probe-result.json',result)
        print('SHOWCASE_RESULT='+json.dumps(dict(passed=result['passed'],output=str(output),error=result.get('error'))),flush=True)
        if app:app.close()
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
