"""Bounded headless physical-vehicle dynamics cases; no SUMO, LiDAR or training.

Invoke through the supervisor, which supplies a unique output folder and a
resolved, source-hashed configuration. Every case uses a fresh physics stage;
completed and partial traces are retained even if an acceptance gate fails.
"""
import argparse
import json
from pathlib import Path
import platform
import random
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from experiments.probe_support import process_sample, write_json
from traffic.driver_control import DriverCommand, DriverControlGate
from traffic.dynamics_validation import assess_case, compare_repeats, validate_config


def phase_command(*, episode_id, tick, sequence, phase, speed_m_s, case, config):
    """Simple speed servo for testing actuators, not an autonomous road driver."""
    driving = phase in ('accelerate', 'turn_warmup', 'circle')
    throttle = min(1.0, max(0.0, (case['target_speed_m_s'] - speed_m_s) * config['speed_proportional_gain'])) if driving else 0.0
    brake = 1.0 if phase in ('settle', 'brake', 'hold') else 0.0
    steering = case['steering_rad'] if phase in ('turn_warmup', 'circle') else 0.0
    return DriverCommand(episode_id, 'ego', sequence, tick,
                         tick + config['command_ttl_ticks'], steering, throttle, brake)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT / 'outputs') or not output.is_dir():
        parser.error('Use the supervisor to create a unique project outputs directory')
    if (output / 'probe-result.json').exists() or list(output.glob('trajectory-*.json')):
        parser.error('Refusing to overwrite an existing dynamics attempt')
    config = validate_config(json.loads((output / 'resolved-config.json').read_text()))
    random.seed(config['seed'])
    app = vehicle = session = None
    rows, reports, comparisons, lifecycles, traces = [], [], {}, [], {}
    trace_name = None
    result = dict(passed=False, no_sumo=True, no_lidar=True, isaac_lab_validated=False)
    started = time.perf_counter()
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({'headless': True, 'width': 960, 'height': 540,
                             'disable_viewport_updates': True})
        from traffic.physics_session import PhysicsSession, prepare_vehicle_runtime
        result['runtime_setup'] = prepare_vehicle_runtime()
        from traffic.physx_vehicle import PhysxVehicle
        startup_wall_s = time.perf_counter() - started
        dt = 1 / config['physics_hz']
        stride = config['physics_hz'] // config['control_hz']
        baseline_metadata = None
        for repeat in range(config['repeats']):
            for case in config['cases']:
                rows = []
                trace_name = f"trajectory-{case['name']}-{repeat}.json"
                episode_id = f"dynamics-{case['name']}-{repeat}"
                session = PhysicsSession(physics_hz=config['physics_hz'])
                try:
                    with session:
                        vehicle = PhysxVehicle(session.stage, config['physics_hz'])
                        if baseline_metadata is not None and vehicle.metadata != baseline_metadata:
                            raise RuntimeError('Vehicle model metadata changed across reset cases')
                        baseline_metadata = vehicle.metadata
                        write_json(output / 'vehicle-model.json', baseline_metadata)
                        session.stage.Export(str(output / f"initial-scene-{case['name']}-{repeat}.usda"))
                        session.attach()
                        gate = DriverControlGate(episode_id, 'ego')
                        tick, sequence, last_state = 0, 0, None
                        try:
                            for phase, seconds in case['phases']:
                                for _ in range(round(seconds / dt)):
                                    if time.perf_counter() - started > config['max_process_seconds'] - 15:
                                        raise TimeoutError('Dynamics probe reached its bounded deadline')
                                    cmd = None
                                    if tick % stride == 0:
                                        cmd = phase_command(episode_id=episode_id, tick=tick, sequence=sequence,
                                                            phase=phase, speed_m_s=0.0 if last_state is None else last_state['speed_m_s'],
                                                            case=case, config=config)
                                        sequence += 1
                                    applied = gate.step(tick=tick, dt_s=dt, command=cmd)
                                    controls = vehicle.apply(applied, config['drive_torque_per_front_wheel_nm'],
                                                             config['brake_torque_per_wheel_nm'])
                                    step_started = time.perf_counter()
                                    session.step(dt, tick * dt)
                                    last_state = vehicle.state()
                                    rows.append(dict(episode_id=episode_id, vehicle_id='ego',
                                                     tick=tick + 1, sim_time_s=(tick + 1) * dt, phase=phase,
                                                     step_wall_ms=(time.perf_counter() - step_started) * 1000,
                                                     control=controls, **last_state))
                                    if last_state['upright_z'] < 0.8 or last_state['position_m'][2] < 0:
                                        raise RuntimeError(f'Unsafe vehicle state in {episode_id}; aborting remaining cases')
                                    tick += 1
                                # Also preserve completed phases if the external deadline kills this process.
                                write_json(output / trace_name, rows)
                            report = assess_case(rows, case, config, wheelbase_m=vehicle.wheelbase_m)
                            report.update(repeat=repeat, episode_id=episode_id, trace=trace_name)
                            reports.append(report)
                            traces[(case['name'], repeat)] = rows
                            write_json(output / 'case-reports.json', reports)
                        finally:
                            # Do not retain USD/vehicle handles past the session's stage close.
                            vehicle = None
                finally:
                    if trace_name:
                        write_json(output / trace_name, rows)
                    lifecycles.append(dict(episode_id=episode_id, events=session.lifecycle))
                    write_json(output / 'lifecycle.json', lifecycles)
                print(f"DYNAMICS_CASE={case['name']} REPEAT={repeat} PASSED={reports[-1]['passed']}", flush=True)
        for case in config['cases']:
            comparisons[case['name']] = compare_repeats(traces[(case['name'], 0)], traces[(case['name'], 1)], config['acceptance'])
        result.update(passed=all(r['passed'] for r in reports) and all(r['passed'] for r in comparisons.values()),
                      cases=reports, repeatability=comparisons, repeats=config['repeats'],
                      completed_cases=len(reports), expected_cases=len(config['cases']) * config['repeats'],
                      startup_wall_s=startup_wall_s, python=platform.python_version(),
                      limitations=['Same-version local sample vehicle; not calibrated highway dynamics',
                                   'Low-slip bicycle comparison is not a real-world tire-model validation',
                                   'Coast telemetry does not establish realistic aerodynamic/rolling resistance',
                                   'No lane following, sensors, V2V, traffic bridge or trained policy'])
    except Exception as error:
        result.update(passed=False, error=repr(error), traceback=traceback.format_exc(),
                      cases=reports, repeatability=comparisons, completed_cases=len(reports),
                      expected_cases=len(config['cases']) * config['repeats'])
        traceback.print_exc()
    finally:
        if trace_name is not None:
            write_json(output / trace_name, rows)
        result.update(total_wall_s=time.perf_counter() - started, process=process_sample())
        # Installed Kit fast shutdown may call os._exit; persist before close.
        # The external supervisor separately checks shutdown logs and exit code.
        write_json(output / 'probe-result.json', result)
        print('PHYSICS_DYNAMICS_RESULT=' + json.dumps(result), flush=True)
        if app:
            try:
                app.close()
            except Exception as error:
                result.update(passed=False, close_error=repr(error))
                write_json(output / 'probe-result.json', result)
                print('PHYSICS_DYNAMICS_CLOSE_ERROR=' + json.dumps(result), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
