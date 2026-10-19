"""Supervised live showcase; launch through demo_loop_showcase.py."""
import argparse
from dataclasses import asdict, replace
import json
import math
import os
from pathlib import Path
import sys
import time
import traceback

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from experiments.loop_showcase_config import assess,fleet_specs,validate_config
from experiments.probe_support import process_sample,write_json
from traffic.bypass_validation import OrientedBox
from traffic.showcase_clearance import nearest_clearance
from traffic.driver_control import AppliedControl,DriverCommand,DriverControlGate
from traffic.evidence_chunks import EvidenceChunkWriter
from traffic.loop_driving import CircularLoop,LoopProgress
from traffic.loop_showcase_control import ShowcaseConfig,ShowcaseController,circular_station
from traffic.path_following import VehicleState
from traffic.showcase_performance import SequentialProfiler,audit_schedule
from visualization.preview_timing import PreviewClock,rendering_profile
from usd.physical_scene import PATHS
from usd.showcase_vehicle import package_hashes


def body_box(state,metadata):
    x,y=state['position_m'][:2];a=state['yaw_rad'];c,s=math.cos(a),math.sin(a)
    ox,oy=metadata['chassis_collision_center_m'][:2]
    length,width=metadata['chassis_collision_dimensions_m'][:2]
    return OrientedBox(x+c*ox-s*oy,y+s*ox+c*oy,a,length,width)


def render_frame(session,render_probe=None):
    """Service both active and paused frames through the same bounded probe."""
    session.render()
    if render_probe:render_probe.after_render()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    output=parser.parse_args().output.resolve()
    if not output.is_relative_to(ROOT/'outputs') or not output.is_dir() or (output/'probe-result.json').exists():
        parser.error('Fresh supervisor-created output directory required')
    cfg=json.loads((output/'resolved-config.json').read_text());validate_config(cfg)
    sidebar_enabled=cfg['version'] in ('v08','v09')
    live_driving=cfg['version']=='v09';live=None
    responsive=cfg['version'] in ('v07','v08','v09')
    optimized=cfg['version'] in ('v06','v07','v08','v09');dense=cfg['version'] in ('v05','v06','v07','v08','v09')
    presentation=cfg.get('evidence_profile')=='presentation'
    profile=SequentialProfiler()
    calibration=profile.calibrate_overhead()
    route=CircularLoop();episode='showcase-'+output.name
    app=session=vehicle=fleet=monitor=view=window=label=viewport=writer=preview=ui_probe=render_probe=None
    sidebar=None;restart_config=None;restart_values=None;smoke_done=False;next_stats_tick=0
    control_validation=None;metrics=None;metric_latest={}
    live_settings=cfg.get('controls',{})
    static_layers=None;rows=[];plans=[];contacts=[];camera_checks=[];visual_checks=[];captures=[];captured=set()
    passed=set();passed_records=[];seen_ahead=set();peers={};last_peer_stations={}
    native_audit_ticks=[];last_native_audit_tick=0;paused_wall=0.
    started=time.perf_counter();result=dict(passed=False,no_training=True,no_lidar=True,no_sumo=True,
        observations=cfg['observations'],main_motion='dynamic_physx',background_motion='scripted_kinematic_physx')
    try:
        from isaacsim import SimulationApp
        graphics=rendering_profile(True)
        if sidebar_enabled:
            scale=cfg['controls']['resolution_percent']/100
            graphics.update(width=round(960*scale),height=round(600*scale))
        if responsive:graphics['limit_cpu_threads']=cfg['cpu_threads']
        if cfg.get('profile_render'):
            graphics['extra_args']=[*graphics['extra_args'],'--enable','omni.hydra.engine.stats']
        if responsive and cfg['ui_profile']=='showcase':
            # Runtime-owned libraries stay outside the project; never hardcode
            # the workstation path or resolve them relative to this .kit file.
            graphics['extra_args']=[*graphics['extra_args'],'--ext-folder',str(Path(os.environ['ISAAC_PATH'])/'extscache'),
                                   '--ext-folder',str(Path(os.environ['ISAAC_PATH'])/'extsDeprecated')]
        if dense:
            # Process-local fallback for this runtime's dense-Fabric crash.
            graphics['extra_args']=[*graphics['extra_args'],'--/app/useFabricSceneDelegate=false']
        if responsive:
            from visualization.showcase_startup import window_launch_config
            graphics=window_launch_config(graphics,gui=cfg['gui'])
        result['graphics_profile']=graphics
        experience=str(ROOT/'visualization/highwaysim.showcase.kit') if responsive and cfg['ui_profile']=='showcase' else ''
        app=SimulationApp({**graphics,'headless':not cfg['gui'],'disable_viewport_updates':False},experience=experience)
        from traffic.physics_session import prepare_vehicle_runtime
        from traffic.rendered_physics_session import RenderedPhysicsSession
        from traffic.vehicle_contacts import VehicleContactMonitor
        from usd.showcase_vehicle import compose_prepared_vehicle
        from usd.showcase_fleet import KinematicFleet
        from usd.highway_loop_scene import install_highway_loop,spawn_vehicle_on_loop,validate_highway_loop_scene,save_highway_loop_scene
        from visualization.loop_showcase_view import LoopShowcaseView,make_controls
        from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_file,get_num_viewports
        from pxr import Sdf,Usd
        public_vehicle=cfg.get('vehicle_profile')=='public'
        if not responsive or public_vehicle:result['runtime_setup']=prepare_vehicle_runtime()
        # No embedded USD behaviors are part of this explicitly controlled
        # showcase. Isolate the installed optional scripting stage-close
        # consumer before it can attach to our long-lived owned stage.
        from traffic.showcase_runtime import (inspect_behavior_free_stage,
            disable_unused_behavior_scripting,assert_behavior_scripting_disabled,
            disable_unused_sensor_icons,assert_sensor_icons_disabled)
        import omni.kit.app
        extension_manager=omni.kit.app.get_app().get_extension_manager()
        if responsive:
            from visualization.showcase_responsiveness import (KitResponsiveness,assess_responsiveness,
                prepared_runtime_inventory,display_qualification_exclusions)
            result['runtime_setup']=prepared_runtime_inventory(extension_manager)
            if public_vehicle:
                result['runtime_setup']['prepared_asset_runtime']=False
        behavior_sources={}
        if public_vehicle:
            # Inspect the actual generated Factory source on an unattached stage
            # BEFORE disabling the optional behavior consumer. The composed live
            # scene is independently inspected again before physics starts.
            from traffic.physics_session import vehicle_factory
            from pxr import UsdGeom
            factory=vehicle_factory()
            inspected_stage=Usd.Stage.CreateInMemory('public-vehicle-behavior-check.usda')
            inspected_stage.SetDefaultPrim(UsdGeom.Xform.Define(inspected_stage,'/World').GetPrim())
            factory.create4WheeledCarsScenario(inspected_stage,1.,1,driveMode=factory.DRIVE_NONE,
                axes=factory.AxesIndices(2,0,1),timeStepsPerSecond=cfg['physics_hz'],createCollisionShapesForWheels=True)
            behavior_sources['vehicle']=inspect_behavior_free_stage(inspected_stage)
            inspected_stage=None
        input_stages=[('highway',ROOT/cfg['source_directory']/'highway_v02.usda')]
        if not public_vehicle:input_stages.append(('vehicle',ROOT/cfg['vehicle_directory']/'world.usda'))
        for name,path in input_stages:
            inspected_stage=Usd.Stage.Open(str(path))
            behavior_sources[name]=inspect_behavior_free_stage(inspected_stage)
            inspected_stage=None
        result['behavior_runtime']=disable_unused_behavior_scripting(extension_manager,behavior_sources)
        if dense:
            result['sensor_icon_runtime']=disable_unused_sensor_icons(extension_manager)
        source_hashes={} if public_vehicle else package_hashes(ROOT/cfg['vehicle_directory'])
        scene_directory=output/'scene'
        session=RenderedPhysicsSession(cfg['physics_hz'],cfg['render_hz'],physics_scene_path=PATHS.physics)
        if public_vehicle:
            from traffic.physx_vehicle import PhysxVehicle
            vehicle=PhysxVehicle(session.stage,cfg['physics_hz'],scene_directory=scene_directory)
            vehicle.metadata['chassis_collision_center_m']=[0.,0.,0.]
            vehicle.metadata['asset_profile']='public procedural block car; installed NVIDIA Factory sample'
        else:
            vehicle=compose_prepared_vehicle(session.stage,scene_directory,ROOT/cfg['vehicle_directory'],cfg['physics_hz'])
        result['environment']=install_highway_loop(session.stage,scene_directory,ROOT/cfg['source_directory'])
        result['spawn']=spawn_vehicle_on_loop(session.stage,0.,radius_m=500+3.7*cfg['initial_lane'])
        # Local publication's point light was for an asset inspection, not a 1 km road.
        session.stage.GetPrimAtPath('/World/Lighting/Key').SetActive(False)
        fleet_class=KinematicFleet;fleet_extra={}
        if live_driving:
            from traffic.live_showcase import LiveTraffic,LiveShowcaseController
            from usd.live_showcase_fleet import LiveKinematicFleet
            live=LiveTraffic(fleet_specs(cfg),cfg['controls'])
            fleet_class=LiveKinematicFleet;fleet_extra={'live':live}
        fleet=fleet_class(session.stage,scene_directory,fleet_specs(cfg),episode_id=episode,**fleet_extra,
            wheel_update_every=cfg['physics_hz']//cfg['render_hz'] if cfg['version'] in ('v03','v04','v05','v06','v07','v08','v09') else 1,
            batch_visual_updates=dense,separate_visuals=dense,cache_readback=optimized,
            instance_visuals=cfg.get('instance_visuals',False))
        monitor=VehicleContactMonitor(session.stage,vehicle.path)
        view=LoopShowcaseView(session.stage,route,optimized=optimized,responsive=responsive)
        ui_state=dict(camera=cfg['camera'],paused=False,show_path=True)
        # Hidden native-model smoke still needs real widgets/subscriptions;
        # ordinary headless runs avoid constructing the interactive panel.
        if cfg['gui'] or (sidebar_enabled and (cfg['control_smoke'] or cfg.get('control_validation'))):
            if sidebar_enabled:
                from visualization.showcase_sidebar import ShowcaseSidebar
                sidebar=ShowcaseSidebar(ui_state,cfg['controls'],visible=not cfg['panel_hidden'],live_driving=live_driving,
                    statistics=cfg.get('traffic_statistics',False))
                label=sidebar.telemetry
            else:window,label=make_controls(ui_state,view=view if optimized else None)
        if sidebar_enabled:
            view.follow_distance_m=live_settings['camera_distance_m']
            view.follow_height_m=live_settings['camera_height_m']
        if cfg.get('control_validation'):
            from experiments.showcase_control_validation import ControlValidationScenario,validation_snapshot
            control_validation=ControlValidationScenario(cfg['control_validation'],cfg['physics_hz'])
        viewport=get_active_viewport()
        if viewport is None:raise RuntimeError('No viewport is available')
        if responsive:
            import carb
            result['viewport_count']=get_num_viewports()
            if not carb.settings.get_settings().get('/app/gatherRenderResults') and result['viewport_count']!=1:
                raise RuntimeError('No-gather showcase requires exactly one viewport')
            result['effective_render_settings']={key:carb.settings.get_settings().get(key) for key in (
                '/app/gatherRenderResults','/app/hydraEngine/waitIdle','/app/asyncRendering',
                '/app/runLoops/main/rateLimitEnabled','/app/renderer/sleepMsOnFocus',
                '/app/renderer/sleepMsOutOfFocus','/app/useFabricSceneDelegate','/rtx-transient/dlssg/enabled')}
        viewport.resolution=(graphics['width'],graphics['height'])
        model=vehicle.metadata
        controller_config=ShowcaseConfig(target_speed_m_s=cfg['target_speed_m_s'],
            desired_time_gap_s=cfg['controls']['following_s'] if sidebar_enabled else 1.8,
            wheelbase_m=vehicle.wheelbase_m,rear_axle_offset_m=vehicle.wheelbase_m/2,
            ego_length_m=model['chassis_collision_dimensions_m'][0],
            ego_width_m=model['chassis_collision_dimensions_m'][1],
            ego_collision_offset_x_m=model['chassis_collision_center_m'][0])
        controller_class=LiveShowcaseController if live_driving else ShowcaseController
        controller=controller_class(episode,'ego',controller_config,initial_lane=cfg['initial_lane'],optimized=optimized)
        peer_station=circular_station if optimized else lambda x,y:route.project(x,y).s_m % route.length_m
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
        result['behavior_runtime']['composed_scene_check']=inspect_behavior_free_stage(session.stage)
        result['behavior_runtime']['pre_play_state']=assert_behavior_scripting_disabled(extension_manager)
        if dense:
            result['sensor_icon_runtime']['pre_play_state']=assert_sensor_icons_disabled(extension_manager)
        session.start();fleet.bind(session.manager.get_physics_simulation_view())
        if dense:
            result['ego_state_readback']=vehicle.bind_native_state(session.manager.get_physics_simulation_view())
        result['clock_start']=session.snapshot();state=vehicle.state(refresh_usd=not dense)
        progress.update(route.project(*state['position_m'][:2]).s_m % route.length_m)
        for t in fleet.tracks(0):
            station=peer_station(t.x_m,t.y_m)
            peers[t.vehicle_id]=station
            last_peer_stations[t.vehicle_id]=station
        # Compile/load the first visible frame before the live clock begins.
        # Each render verifies zero physics advancement; report readiness cost.
        warmup_started=time.perf_counter()
        fleet.publish_visuals()
        for _ in range(4):session.render()
        if responsive:
            from visualization.showcase_startup import wait_for_native_showcase_ready,ShowcaseStartupError
            try:
                result['startup_readiness']=wait_for_native_showcase_ready(
                    pump=session.render,is_running=app.is_running,viewport=viewport,sidebar=sidebar,
                    gui=cfg['gui'],window_size=(graphics.get('window_width',1440),graphics.get('window_height',900)))
            except ShowcaseStartupError as error:
                result['startup_readiness']=error.evidence
                raise
            finally:
                if 'startup_readiness' in result:write_json(output/'startup-readiness.json',result['startup_readiness'])
        result['renderer_warmup_s']=time.perf_counter()-warmup_started
        if responsive:
            ui_probe=KitResponsiveness();ui_probe.start()
            if cfg['profile_render']:
                from visualization.showcase_render_profile import RenderProfile
                render_probe=RenderProfile(viewport)
        writer=None if presentation else EvidenceChunkWriter(output)
        preview=PreviewClock(paced=cfg['paced'])
        result['startup_wall_s']=time.perf_counter()-started
        if cfg.get('traffic_statistics'):
            from traffic.showcase_metrics import ShowcaseMetrics,VehicleSample
            metrics=ShowcaseMetrics()
            nominal_background_speeds={spec['vehicle_id']:spec['speed_m_s'] for spec in fleet_specs(cfg)}
        hz=cfg['physics_hz'];brake_tick=(cfg['settle_s']+cfg['drive_s'])*hz
        traveled=0.;paused_wall=0.;expires=None;diagnostics={};max_lanes=0
        pose_check=fleet.pose_check(0.)
        for tick in range(cfg['duration_s']*hz):
            if not app.is_running() or time.perf_counter()-started>cfg['max_process_seconds']-20:
                raise RuntimeError('Preview closed or bounded deadline reached')
            if control_validation:
                control_validation.before_tick(sidebar,tick,validation_snapshot(tick,state,live,session.snapshot()))
            while ui_state['paused']:
                if not app.is_running() or time.perf_counter()-started>cfg['max_process_seconds']-20:
                    raise RuntimeError('Paused preview closed or bounded deadline reached')
                before=time.perf_counter();render_frame(session,render_probe)
                time.sleep(.02);paused_wall+=time.perf_counter()-before
                if control_validation:
                    paused_state=vehicle.state(refresh_usd=not dense)
                    control_validation.paused_frame(sidebar,tick,validation_snapshot(tick,paused_state,live,session.snapshot()))
            if sidebar:
                # In-memory only; widget callbacks never step physics or author USD.
                if cfg['control_smoke'] and live_driving:
                    sidebar.live_smoke_action(tick,hz);smoke_done=sidebar.smoke_stage
                elif cfg['control_smoke'] and not smoke_done and tick>=5*hz:
                    sidebar.smoke_action();smoke_done=True
                changes=sidebar.settings.consume_live(tick)
                live_settings=sidebar.settings.active
                if live and changes:live.configure(live_settings)
                if sidebar.settings.restart_request is not None and restart_config is None:
                    from experiments.showcase_controls import restart_assessment_config
                    drive_s=min(cfg['drive_s'],max(15,math.ceil(tick/hz)-cfg['settle_s']))
                    restart_config=restart_assessment_config(cfg,drive_s)
                    restart_values=dict(sidebar.settings.restart_request)
                    brake_tick=(cfg['settle_s']+drive_s)*hz
                    result['restart_requested_at_tick']=tick
                if view.path_visible!=ui_state['show_path']:view.set_path_visible(ui_state['show_path'])
                view.follow_distance_m=live_settings['camera_distance_m']
                view.follow_height_m=live_settings['camera_height_m']
            profile.begin_iteration()
            schedule=audit_schedule(cfg,tick+1)
            phase='settle' if tick<cfg['settle_s']*hz else 'brake' if tick>=brake_tick else 'drive'
            if dense and tick==brake_tick:
                result['fleet_rundown']=fleet.begin_stop()
            tracks=fleet.tracks(tick)
            profile.mark('fleet_pre_read')
            command=None
            physical_state=VehicleState.from_physics(state,episode_id=episode,vehicle_id='ego',tick=tick)
            if tick%2==0:
                if live:
                    controller.accept_lifecycle(tracks,live.pending_lifecycle)
                    live.pending_lifecycle.clear()
                    live.prepare_control(2/hz,maneuver=controller._maneuver)
                    controller.config=replace(controller.config,target_speed_m_s=live.ego_target_m_s,desired_time_gap_s=live.following_s)
                    controller.changing_traffic_speed=live.ramping
                    controller.traffic_transition_pending=len(live.active)!=live.settings['traffic_count']
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
            profile.mark('controller')
            if command:expires=command.expires_tick
            applied=gate.step(tick=tick,dt_s=1/hz,command=command)
            vehicle.apply(applied,cfg['drive_torque_nm'],cfg['brake_torque_nm'])
            profile.mark('actuation')
            if live:
                fleet.maneuver=controller._maneuver
                if phase=='drive':fleet.reconcile(tick,state['position_m'][:2],maneuver=controller._maneuver)
            fleet.update((tick+1)/hz)
            profile.mark('fleet_targets')
            session.step(publish_usd=not dense)
            profile.mark('physics')
            old=state;state=vehicle.state(refresh_usd=not dense)
            profile.mark('ego_state')
            traveled+=math.dist(old['position_m'][:2],state['position_m'][:2])
            events=monitor.sample(session.sim,tick+1);contacts.extend(events)
            tracks=fleet.tracks(tick+1)
            profile.mark('contacts_and_fleet_read')
            if schedule['native']:
                pose_check=fleet.pose_check((tick+1)/hz)
                last_native_audit_tick=tick+1;native_audit_ticks.append(tick+1)
                if not pose_check['passed']:raise RuntimeError('Native background motion contract failed')
            profile.mark('native_pose_audit')
            projection=route.project(*state['position_m'][:2]);tracking=progress.update(projection.s_m % route.length_m)
            ego_box=body_box(state,model)
            clearance=nearest_clearance(ego_box,[OrientedBox(t.x_m,t.y_m,t.yaw_rad,t.length_m,t.width_m) for t in tracks])
            for t in tracks:
                station=peer_station(t.x_m,t.y_m)
                previous=last_peer_stations.get(t.vehicle_id)
                if previous is not None:peers[t.vehicle_id]+=(station-previous+route.length_m/2)%route.length_m-route.length_m/2
                else:peers[t.vehicle_id]=tracking['progress_m']+(station-projection.s_m+route.length_m/2)%route.length_m-route.length_m/2
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
                clearance_m=clearance['clearance_m'],overlap=clearance['overlap'],
                clearance_exact_pairs=clearance['exact_pairs'],
                nearby_traffic_100m=sum(math.hypot(t.x_m-ego_box.x_m,t.y_m-ego_box.y_m)<=100 for t in tracks),
                background_max_speed_m_s=max(abs(t.speed_m_s) for t in tracks),
                footprint_in_road=in_road,background_pose_error_m=pose_error,background_native_passed=pose_check['passed'],
                throttle=applied.throttle,brake=applied.brake,steering_rad=applied.steering_rad,
                fallback=applied.is_fallback,command_expires_tick=expires,
                controller_fallback=diagnostics.get('is_fallback',False),
                controller_status=('changing_lane' if diagnostics.get('maneuver_active') else 'following'),controller_reason=diagnostics.get('reason','settle'),
                lane=diagnostics.get('lane',cfg['initial_lane']),completed_lane_changes=max_lanes,
                completed_passes=len(passed))
            if optimized:row['background_native_audit_tick']=last_native_audit_tick
            if live:row.update(active_background_count=len(tracks),requested_background_count=live.settings['traffic_count'],
                               applied_ego_target_m_s=live.ego_target_m_s,applied_traffic_speed_m_s=live.speed_m_s,
                               applied_following_s=live.following_s)
            if metrics:
                observed=[VehicleSample('ego',ego_box.x_m,ego_box.y_m,ego_box.yaw_rad,
                    state['speed_m_s'],ego_box.length_m,ego_box.width_m,
                    live.ego_target_m_s if live else cfg['target_speed_m_s'],role='ego')]
                for track in tracks:
                    reference=(max(live.speed_m_s,.001)*math.hypot(track.x_m,track.y_m)/500 if live
                        else nominal_background_speeds[track.vehicle_id])
                    observed.append(VehicleSample(track.vehicle_id,track.x_m,track.y_m,track.yaw_rad,
                        track.speed_m_s if phase=='drive' else abs(track.speed_m_s),
                        track.length_m,track.width_m,max(reference,.001)))
                metric_latest=metrics.update(row['sim_time_s'],phase,observed,
                    nearest_clearance_m=row['clearance_m'])
                row['traffic_statistics']=metric_latest
            rows.append(row)
            profile.mark('safety_and_metrics')
            if schedule['objects']:plans.append(dict(tick=tick+1,diagnostics=diagnostics,objects=[asdict(t) for t in tracks]))
            profile.mark('object_recording')
            if cfg['mode']=='contact-check' and events:
                profile.finish_iteration();break
            if cfg['mode']!='contact-check' and (events or row['overlap'] or (phase!='settle' and (not in_road or state['upright_z']<.9))):
                raise RuntimeError('Contact/overlap/road/attitude safety guard')
            if (tick+1)%session.render_every_steps==0:
                if dense:session.publish_transforms()
                if optimized:
                    fleet.publish_visuals(verify=not presentation,
                        focus_position_xy=state['position_m'][:2],wheel_distance_m=live_settings['wheel_distance_m'] if sidebar_enabled else cfg['decorative_wheel_distance_m'])
                else:fleet.publish_visuals()
                profile.mark('visual_publication')
                physical_state=VehicleState.from_physics(state,episode_id=episode,vehicle_id='ego',tick=tick+1)
                path_points=controller.path_points(physical_state,**({'horizon_m':live_settings['path_horizon_m']} if sidebar_enabled else {})) if not optimized or view.path_visible else None
                camera_path=view.update_showcase(state,projection.s_m,ui_state['camera'],path_points)
                if not responsive or viewport.camera_path!=camera_path:viewport.camera_path=camera_path
                refresh_stats=(tick+1>=next_stats_tick) if sidebar_enabled else (not responsive or tick+1==session.render_every_steps or (tick+1)%(hz//cfg['statistics_hz'])==0)
                if label and refresh_stats:label.text=(f"{cfg['version']} | {phase} | {state['speed_m_s']/0.44704:.1f} mph | {len(tracks)} traffic cars\n"
                    f"Passes {len(passed)} | lane changes {max_lanes} | clearance {row['clearance_m']:.2f} m\n"
                    f"{row['controller_status']}: {row['controller_reason']} | {row['sim_time_s']:.0f}s")
                if label and refresh_stats and metrics and sidebar:
                    headway=metric_latest.get('net_headway_s')
                    headway_text='--' if headway is None else f'{headway:.2f}s'
                    label.text+=(f"\nMeasured gap {headway_text} | time loss {metric_latest.get('ego_time_loss_s',0):.1f}s"
                        f"\nNear-miss proxy events {metric_latest.get('near_miss_events',0)} | ground-truth stats")
                if sidebar_enabled and refresh_stats:next_stats_tick=tick+1+hz/live_settings['statistics_hz']
                if live and sidebar and refresh_stats:
                    sidebar.live_status=f'Actual: {len(tracks)} cars | traffic {live.speed_m_s/.44704:.1f} mph'
                    sidebar.refresh_status()
                profile.mark('camera_overlay_ui')
                render_frame(session,render_probe)
                profile.mark('render')
                check=view.camera_check();camera_checks.append(dict(tick=tick+1,**check))
                if not check['passed']:raise RuntimeError('Camera pose ownership failed')
                if schedule['visual']:
                    visual_check=fleet.visual_pose_check();visual_checks.append(dict(tick=tick+1,**visual_check))
                    if not visual_check['passed']:raise RuntimeError('Rendered background poses differ from native physics poses')
                profile.mark('visual_audit')
                if cfg['capture']:
                    for second in (10,20,35,55,cfg['settle_s']+cfg['drive_s']-1):
                        if row['sim_time_s']>=second and second not in captured:
                            captures.append(capture_viewport_to_file(viewport,str(output/f'preview-{second:03d}s-{ui_state["camera"]}.png')))
                            captured.add(second)
                profile.mark('capture')
                preview.finish_frame(row['sim_time_s'],paused_wall)
                profile.mark('pace_wait')
            if schedule['checkpoint']:writer.write_checkpoint(rows,plans)
            profile.mark('checkpoint_and_tail')
            if (tick+1)%(hz*10)==0:
                print(f"SHOWCASE t={row['sim_time_s']:.0f} speed={row['speed_m_s']:.2f} passes={len(passed)} lane_changes={max_lanes} gap={row['clearance_m']:.2f} state={row['controller_status']} {row['controller_reason']}",flush=True)
            profile.finish_iteration()
            if restart_config and tick+1>=restart_config['duration_s']*hz:break
        if ui_probe:ui_probe.stop()
        if render_probe:render_probe.close()
        result.update(assess(rows,restart_config or cfg,len(contacts),passed,max_lanes))
        extra=dict(camera=bool(camera_checks) and all(c['passed'] for c in camera_checks),
            background_visual_sync=bool(visual_checks) and all(c['passed'] for c in visual_checks),
            background_native_contract=bool(rows) and all(r['background_native_passed'] for r in rows),
            controller_valid=all(not r['controller_fallback'] for r in rows if r['phase']=='drive') if cfg['mode']!='contact-check' else True,
            scene_structure=validate_highway_loop_scene(session.stage)['passed'],
            static_files_unchanged=package_hashes(scene_directory)==static_hashes,
            static_layers_unchanged=all(layer.ExportToString()==static_text[layer.identifier] for layer in static_layers),
            source_vehicle_unchanged=public_vehicle or package_hashes(ROOT/cfg['vehicle_directory'])==source_hashes)
        if cfg['mode']!='contact-check':
            extra['optional_audit_coverage']=(native_audit_ticks==[t for t in range(1,len(rows)+1) if audit_schedule(cfg,t)['native']]
                and [r['tick'] for r in visual_checks]==[t for t in range(1,len(rows)+1) if audit_schedule(cfg,t)['visual']])
        result['gates'].update(extra);result['passed']=all(result['gates'].values())
        if responsive:
            result['gates']['startup_readiness']=result.get('startup_readiness',{}).get('passed',False)
            result['passed']=all(result['gates'].values())
        if restart_config:
            result.update(restart_ready=result['passed'],passed=False,
                run_status='stopped_for_requested_restart',full_scenario_completed=False,
                restart_settings=restart_values,stop_assessment_config=restart_config,
                qualification_eligible=False)
        result['behavior_runtime']['post_run_state']=assert_behavior_scripting_disabled(extension_manager)
        if dense:
            result['sensor_icon_runtime']['post_run_state']=assert_sensor_icons_disabled(extension_manager)
        result.update(clock_end=session.snapshot(),passes=passed_records)
        for _ in range(3):session.render()
    except Exception as error:
        result.update(passed=False,error=repr(error),traceback=traceback.format_exc());traceback.print_exc()
    finally:
        if metrics:
            result['traffic_statistics']=metrics.summary()
            result['traffic_statistics']['integration']=dict(sample_hz=cfg['physics_hz'],
                ego_speed_source='native chassis planar velocity magnitude',
                background_speed_source='native velocity projected onto body forward axis; reverse drive samples rejected',
                configuration_mode=cfg['mode'],
                full_trajectory_completed=len(rows)==cfg['duration_s']*cfg['physics_hz'],
                run_error=result.get('error'),acceptance_source='summary.json after process cleanup')
            write_json(output/'traffic-statistics.json',result['traffic_statistics'])
        if profile.iteration_active:profile.finish_iteration()
        if ui_probe:ui_probe.stop()
        if render_probe:
            try:render_probe.close()
            except Exception as error:result.update(passed=False,render_profiler_close_error=repr(error))
            result['render_diagnosis']=render_probe.summary()
            write_json(output/'render-diagnosis.json',result['render_diagnosis'])
        if sidebar:
            result['sidebar']=dict(active=sidebar.settings.active,pending=sidebar.settings.pending,
                display_events=sidebar.settings.events,event_count=len(sidebar.settings.events),
                future_controls_enabled=any(w.enabled for w in sidebar.future_widgets),
                future_control_count=len(sidebar.future_widgets),docked=sidebar.docked,
                widget_model_smoke=smoke_done,mouse_latency_qualified=False)
            panel_ok=not result['sidebar']['future_controls_enabled'] and (cfg['panel_hidden'] or sidebar.docked)
            result.setdefault('gates',{})['sidebar_contract']=panel_ok
            result['passed']=bool(result.get('passed') and panel_ok)
            if result.get('restart_ready'):result['restart_ready']=panel_ok
            write_json(output/'control-events.json',sidebar.settings.events)
        if live:
            result['live_traffic']=dict(capacity=live.capacity,active_count=len(live.active),lifecycle_events=live.events,
                no_process_restart=restart_config is None,final_settings=live.settings)
            write_json(output/'traffic-lifecycle.json',live.events)
            if control_validation:
                result['control_validation']=control_validation.finish(rows,live.events,sidebar.settings.events,
                    restarted=restart_config is not None)
                write_json(output/'control-validation.json',result['control_validation'])
                result.setdefault('gates',{})['control_validation']=result['control_validation']['passed']
                result['passed']=bool(result.get('passed') and result['control_validation']['passed'])
            if cfg['control_smoke']:
                from traffic.live_showcase import assess_live_smoke
                result['live_smoke']=assess_live_smoke(rows,live.events,smoke_done,restart_config is not None)
                result.setdefault('gates',{})['live_smoke']=result['live_smoke']['passed']
                result['passed']=bool(result.get('passed') and result['live_smoke']['passed'])
        result['runtime_profile']=profile.summary(simulation_time_s=len(rows)/cfg['physics_hz'])
        result['profiling_overhead']=profile.estimate_overhead_for_run(calibration)
        write_json(output/'runtime-profile.json',result['runtime_profile'])
        result['evidence_policy']=dict(profile=cfg.get('evidence_profile','full'),
            native_audit_count=len(native_audit_ticks),visual_audit_count=len(visual_checks),
            native_audit_ticks=native_audit_ticks,full_object_records=len(plans),
            trajectory_hz=cfg['physics_hz'],durable_per_second_checkpoints=not presentation,
            limitation='Presentation audits are sampled; native-crash trajectory recovery is not guaranteed' if presentation else None)
        result['gate_scope']=dict(
            background_native_contract='sampled 1 Hz plus first tick' if presentation else 'every physics tick',
            background_pose='last scheduled native audit, not a new audit on every row' if presentation else 'every physics tick',
            background_visual_sync='sampled 1 Hz plus first display' if presentation else 'every displayed frame',
            safety='contact, clearance, road, attitude and measured body state every physics tick',
            passed='physical/scene/process acceptance; does not imply real-time display',
            smooth_display_passed='separate pacing and frame-interval gate; must be true for display qualification')
        if fleet:result['fleet_work_counts']=dict(native_readbacks=fleet.readback_count,track_constructions=fleet.track_construction_count)
        if preview:
            result['preview_timing']=preview.summary(expected_simulation_time_s=cfg['duration_s'],expected_frames=cfg['duration_s']*cfg['render_hz'])
            result['smooth_display_passed']=bool(result['preview_timing']['soft_realtime_passed']
                and result['preview_timing']['p95_frame_interval_s']<=1.5/cfg['render_hz']
                and result['preview_timing']['max_frame_interval_s']<=.2)
            write_json(output/'preview-timing.json',preview.frames)
            if ui_probe:
                result['responsiveness']=assess_responsiveness(ui_probe.frames,ui_probe.completed,
                    wall_s=result['preview_timing']['active_wall_s'],simulation_s=len(rows)/cfg['physics_hz'],
                    render_hz=cfg['render_hz'],complete=len(rows)==cfg['duration_s']*cfg['physics_hz'])
                result['display_qualification_exclusions']=display_qualification_exclusions(cfg,paused_wall)
                if sidebar and (sidebar.settings.events or restart_config or cfg['control_smoke'] or control_validation):
                    result['display_qualification_exclusions'].append('interactive_display_changes_or_restart')
                result['smooth_display_passed']=bool(result['smooth_display_passed'] and result['responsiveness']['passed']
                    and not result['display_qualification_exclusions'])
                write_json(output/'ui-event-timing.json',dict(application_frames=ui_probe.frames,native_completions=ui_probe.completed))
        if writer:result['checkpoint_evidence']=writer.summary()
        write_json(output/'trajectory.json',rows);write_json(output/'planner.json',plans)
        write_json(output/'contacts.json',contacts);write_json(output/'camera-checks.json',camera_checks)
        write_json(output/'background-visual-checks.json',visual_checks)
        write_json(output/'pre-close-result.json',dict(result,cleanup_verified=False))
        if ui_probe:ui_probe.close();ui_probe=None
        if session and session.started and viewport:
            try:viewport.camera_path='/OmniverseKit_Persp';session.render()
            except Exception as error:result.update(passed=False,camera_close_error=repr(error))
        if window:
            window.visible=False
            window.destroy()  # Release callbacks holding the USD-backed view.
        if sidebar:sidebar.close();sidebar=None
        if fleet:fleet.release()
        window=label=view=monitor=vehicle=fleet=None;static_layers=None
        if session:
            try:session.close()
            except Exception as error:result.update(passed=False,session_close_error=repr(error))
            write_json(output/'lifecycle.json',session.lifecycle)
        if responsive:
            result['cleanup_verified']=bool(session and session.lifecycle[-1]['event']=='closed'
                and not any(v for k,v in result.items() if k=='error' or k.endswith('_error')))
        result.update(total_wall_s=time.perf_counter()-started,process=process_sample(),preview_files=[p.name for p in output.glob('preview-*.png')])
        write_json(output/'probe-result.json',result)
        print('SHOWCASE_RESULT='+json.dumps(dict(passed=result['passed'],output=str(output),error=result.get('error'))),flush=True)
        if app:app.close()
    return 0 if result['passed'] or (result.get('restart_ready') and result.get('cleanup_verified')) else 1


if __name__=='__main__':raise SystemExit(main())
