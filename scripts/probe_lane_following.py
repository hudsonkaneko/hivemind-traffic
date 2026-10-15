"""Bounded map-based PhysX lane following; no SUMO, LiDAR, Lab or learning.

Use experiments.verify_vehicle_stage so each attempt has source hashes, a
deadline, GPU guard and preserved evidence. Rigid-contact reporting must first
detect an intentional barrier collision in a separate positive-control scene.
"""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import random
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from experiments.probe_support import process_sample, write_json
from traffic.driver_control import AppliedControl, DriverCommand, DriverControlGate
from traffic.dynamics_validation import compare_repeats
from traffic.lane_geometry import load_routes
from traffic.lane_validation import assess_lane, footprint_lateral_bound, validate_config
from traffic.path_following import FollowerConfig, PathFollower, PathPlanner, ScriptedCruiseBehavior, VehicleState


def add_barrier(stage):
    """Static collision fixture, authored before attach; no retained USD handles."""
    from pxr import Gf, UsdGeom, UsdPhysics
    cube = UsdGeom.Cube.Define(stage, '/World/Barrier')
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(5, 0, 1))
    cube.AddScaleOp().Set(Gf.Vec3d(1, 3, 2))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


def run_scene(output, config, *, route, repeat, started):
    from traffic.physics_session import PhysicsSession
    from traffic.physx_vehicle import PhysxVehicle
    from traffic.vehicle_contacts import VehicleContactMonitor

    positive = route is None
    name = 'contact-positive-control' if positive else f'{route.route_id}-{repeat}'
    episode_id = f'lane-{name}'
    dt = 1/config['physics_hz']
    seconds = config['contact_positive_control_s'] if positive else config['settle_s']+config['drive_s']
    rows, vehicle, monitor = [], None, None
    last_state = None
    session = PhysicsSession(config['physics_hz'])
    reference = None
    sampled = [] if positive else route.sample(max_spacing_m=.5)
    maximum_curvature = max((abs(p.curvature_rad_m) for p in sampled), default=0.0)
    gate = DriverControlGate(episode_id, 'ego')
    routes = {} if positive else {route.route_id: route}
    follower = None if positive else PathFollower(routes, episode_id, 'ego', config=FollowerConfig(**config['follower']))
    planner = None if positive else PathPlanner(routes, episode_id, 'ego')
    behaviors = {} if positive else {
        phase: ScriptedCruiseBehavior(episode_id, 'ego', route.route_id, target_speed_m_s=speed)
        for phase, speed in [('settle', 0.0), ('drive', config['target_speed_m_s'])]}
    try:
        with session:
            try:
                vehicle = PhysxVehicle(session.stage, config['physics_hz'])
                metadata = vehicle.metadata
                write_json(output/f'vehicle-model-{name}.json', metadata)
                if positive:
                    add_barrier(session.stage)
                else:
                    if vehicle.wheelbase_m != config['follower']['wheelbase_m']:
                        raise RuntimeError('Vehicle/controller wheelbase mismatch')
                    write_json(output/f'route-{name}.json', dict(
                        centerline=[asdict(p) for p in sampled],
                        left=[asdict(p) for p in route.sample(max_spacing_m=.5, lateral_offset_m=route.width_m/2)],
                        right=[asdict(p) for p in route.sample(max_spacing_m=.5, lateral_offset_m=-route.width_m/2)],
                        frame='xy_m_z_up_yaw_ccw_rad', endpoint_tangent_padding_m=5.0))
                monitor = VehicleContactMonitor(session.stage, vehicle.path)
                session.stage.Export(str(output/f'initial-scene-{name}.usda'))
                session.attach()
                for tick in range(round(seconds/dt)):
                    if time.perf_counter()-started > config['max_process_seconds']-15:
                        raise TimeoutError('Bounded lane probe deadline reached')
                    phase = 'settle' if tick < round(config['settle_s']/dt) else 'drive'
                    cmd = None
                    if positive:
                        if tick % 2 == 0:
                            speed = 0 if last_state is None else last_state['speed_m_s']
                            cmd = DriverCommand(episode_id, 'ego', tick//2, tick, tick+12, 0,
                                0 if phase == 'settle' else min(1, max(0, (1-speed)*.7)),
                                1 if phase == 'settle' else 0)
                        applied = gate.step(tick=tick, dt_s=dt, command=cmd)
                        diagnostics = dict(reason='contact_detector_positive_control', fallback=False)
                    else:
                        if tick % 12 == 0:
                            reference = planner.plan(behaviors[phase].decide(tick), tick)
                        # The first two startup ticks only hold the brake while
                        # native physical state becomes available. No invented
                        # observation or runtime pose assignment is used.
                        if tick < 2:
                            applied = AppliedControl(0, 0, 1, 'initialization_hold', None, False)
                            diagnostics = dict(reason='initialization_hold', fallback=False)
                        else:
                            if tick % 2 == 0:
                                state = VehicleState.from_physics(last_state, episode_id=episode_id,
                                                                 vehicle_id='ego', tick=tick)
                                cmd = follower.command(state, tick, reference)
                            applied = gate.step(tick=tick, dt_s=dt, command=cmd)
                            diagnostics = dict(follower.last_diagnostics)
                    controls = vehicle.apply(applied, config['drive_torque_per_front_wheel_nm'],
                                             config['brake_torque_per_wheel_nm'])
                    before = time.perf_counter()
                    session.step(dt, tick*dt)
                    last_state = vehicle.state()
                    contacts = monitor.sample(session.sim, tick+1)
                    row = dict(episode_id=episode_id, vehicle_id='ego', tick=tick+1,
                               sim_time_s=(tick+1)*dt, phase=phase, control=controls,
                               driver_diagnostics=diagnostics, **last_state)
                    if not positive:
                        projection = route.project(*last_state['position_m'][:2])
                        row.update(progress_m=projection.s_m, lateral_error_m=projection.lateral_error_m,
                            heading_error_rad=math.atan2(math.sin(last_state['yaw_rad']-projection.point.yaw_rad),
                                                         math.cos(last_state['yaw_rad']-projection.point.yaw_rad)),
                            footprint_lateral_bound_m=footprint_lateral_bound(last_state, projection,
                                metadata['chassis_collision_dimensions_m'], maximum_curvature))
                    row['step_wall_ms'] = (time.perf_counter()-before)*1000
                    rows.append(row)
                    # Flush bounded partial evidence before any safety exception.
                    if (tick+1) % 600 == 0 or contacts:
                        write_json(output/f'trajectory-{name}.json', rows)
                        write_json(output/f'contacts-{name}.json', monitor.events)
                    if positive and any('/World/Barrier' in [e[k] for k in ('actor0','actor1','collider0','collider1')]
                                        and e['event_type'] == 'found' and e['contact_count'] > 0 for e in contacts):
                        break
                    if last_state['upright_z'] < .8 or last_state['position_m'][2] < 0:
                        raise RuntimeError(f'Unsafe physical pose in {name}')
                    if not positive and (contacts or row['footprint_lateral_bound_m'] > route.width_m/2):
                        raise RuntimeError(f'Contact or lane departure in {name}; remaining cases aborted')
                if positive:
                    found = [e for e in monitor.events if e['event_type'] == 'found' and e['contact_count'] > 0
                             and '/World/Barrier' in [e[k] for k in ('actor0','actor1','collider0','collider1')]]
                    report = dict(passed=bool(found), barrier_found_events=len(found),
                                  first_contact_tick=found[0]['tick'] if found else None, samples=len(rows))
                else:
                    report = assess_lane(rows, config, route_length_m=route.length_m,
                                         lane_width_m=route.width_m, contact_events=monitor.events)
                report.update(episode_id=episode_id, route=None if positive else route.route_id,
                              repeat=repeat, trace=f'trajectory-{name}.json')
                return rows, report, metadata
            finally:
                vehicle = None
    finally:
        write_json(output/f'trajectory-{name}.json', rows)
        write_json(output/f'contacts-{name}.json', [] if monitor is None else monitor.events)
        write_json(output/f'lifecycle-{name}.json', session.lifecycle)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'outputs') or not output.is_dir():
        parser.error('Use the supervisor to create a unique output directory')
    if (output/'probe-result.json').exists() or list(output.glob('trajectory-*.json')):
        parser.error('Refusing to overwrite an existing attempt')
    config = validate_config(json.loads((output/'resolved-config.json').read_text()))
    routes = load_routes(ROOT/config['routes_file'])
    random.seed(config['seed'])
    result = dict(passed=False, no_sumo=True, no_lidar=True, no_training=True,
                  observation_source='privileged_simulator_state_and_known_map')
    reports, traces, comparisons = [], {}, {}
    app = None
    started = time.perf_counter()
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({'headless': True, 'width': 960, 'height': 540, 'disable_viewport_updates': True})
        from traffic.physics_session import prepare_vehicle_runtime
        result['runtime_setup'] = prepare_vehicle_runtime()
        result['startup_wall_s'] = time.perf_counter()-started
        _, positive, model = run_scene(output, config, route=None, repeat=0, started=started)
        result['contact_positive_control'] = positive
        if not positive['passed']:
            raise RuntimeError('Contact detector did not detect the intentional barrier collision')
        for repeat in range(config['repeats']):
            for route_id in config['route_ids']:
                rows, report, metadata = run_scene(output, config, route=routes[route_id], repeat=repeat, started=started)
                if metadata != model:
                    raise RuntimeError('Vehicle model changed across episodes')
                traces[(route_id, repeat)] = rows
                reports.append(report)
                write_json(output/'case-reports.json', reports)
                print(f"LANE_CASE={route_id} REPEAT={repeat} PASSED={report['passed']}", flush=True)
        for route_id in config['route_ids']:
            comparisons[route_id] = compare_repeats(traces[(route_id, 0)], traces[(route_id, 1)], config['acceptance'])
        result.update(passed=all(r['passed'] for r in reports) and all(r['passed'] for r in comparisons.values()),
                      limitations=['One car; 3 m/s known-map scripted baseline, not LiDAR autonomy or trained control',
                                   'Analytic lane corridors on an infinite collision plane; no road mesh or new GUI',
                                   'No SUMO map-correspondence validation, traffic interaction or highway-speed calibration',
                                   'Local repeated traces do not establish cross-platform determinism'])
    except Exception as error:
        result.update(passed=False, error=repr(error), traceback=traceback.format_exc())
        traceback.print_exc()
    finally:
        result.update(cases=reports, repeatability=comparisons, completed_cases=len(reports), expected_cases=6,
                      total_wall_s=time.perf_counter()-started, process=process_sample())
        write_json(output/'probe-result.json', result)
        print('LANE_FOLLOWING_RESULT='+json.dumps(result), flush=True)
        if app:
            try:
                app.close()
            except Exception as error:
                result.update(passed=False, close_error=repr(error))
                write_json(output/'probe-result.json', result)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
