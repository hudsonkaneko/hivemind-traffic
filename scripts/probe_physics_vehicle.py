"""Bounded, headless one-car wheel-command compatibility probe, no SUMO."""
import argparse
import json
import math
from pathlib import Path
import platform
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from experiments.probe_support import process_sample, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'outputs') or not output.is_dir():
        parser.error('Use the supervisor to create a unique project outputs directory')
    if (output/'probe-result.json').exists():
        parser.error('Refusing to overwrite a completed probe')
    config = json.loads((output/'resolved-config.json').read_text())
    app = sim = None
    result = {'passed': False}
    rows, repeat = [], 0
    started = time.perf_counter()
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({'headless': True, 'width': 960, 'height': 540,
                             'disable_viewport_updates': True})
        import omni.kit.app
        import omni.physx
        import omni.usd
        from pxr import UsdUtils
        from traffic.driver_control import DriverCommand, DriverControlGate
        manager = omni.kit.app.get_app().get_extension_manager()
        manager.set_extension_enabled_immediate('omni.physx.vehicle', True)
        from traffic.physx_vehicle import PhysxVehicle
        sim = omni.physx.get_physx_simulation_interface()
        dt = 1/config['physics_hz']
        control_stride = config['physics_hz']//config['control_hz']
        traces, reports = [], []
        for repeat in range(config['repeats']):
            if repeat:
                sim.detach_stage()
                vehicle = None
                stage = None
                omni.usd.get_context().new_stage()
            stage = omni.usd.get_context().get_stage()
            vehicle = PhysxVehicle(stage, config['physics_hz'])
            stage.Export(str(output/f'initial-scene-{repeat}.usda'))
            write_json(output/'vehicle-model.json', vehicle.metadata)
            sim.attach_stage(UsdUtils.StageCache.Get().GetId(stage).ToLongInt())
            gate = DriverControlGate(f'probe-{repeat}', 'ego')
            tick, sequence, rows, bounds = 0, 0, [], {}
            last_state = None
            for phase, seconds in config['phases']:
                begin = len(rows)
                for _ in range(round(seconds/dt)):
                    if time.perf_counter()-started > config['max_process_seconds']-10:
                        raise TimeoutError('Probe deadline reached')
                    speed = 0 if last_state is None else last_state['speed_m_s']
                    cmd = None
                    if tick % control_stride == 0 and phase != 'dropout':
                        throttle = min(1.0, max(0.0, (config['target_speed_m_s']-speed)*0.7)) if phase in ('straight','turn') else 0.0
                        brake = 1.0 if phase in ('settle','brake','hold') else 0.0
                        steering = config['steering_rad'] if phase == 'turn' else 0.0
                        cmd = DriverCommand(f'probe-{repeat}', 'ego', sequence, tick,
                                            tick+config['command_ttl_ticks'], steering, throttle, brake)
                        sequence += 1
                    applied = gate.step(tick=tick, dt_s=dt, command=cmd)
                    control = vehicle.apply(applied, config['drive_torque_per_front_wheel_nm'],
                                            config['brake_torque_per_wheel_nm'])
                    wall = time.perf_counter()
                    sim.simulate(dt, tick*dt)
                    sim.fetch_results()
                    last_state = vehicle.state()
                    rows.append(dict(tick=tick+1, sim_time_s=(tick+1)*dt, phase=phase,
                                     step_wall_ms=(time.perf_counter()-wall)*1000,
                                     control=control, **last_state))
                    if last_state['upright_z'] < 0.8 or last_state['position_m'][2] < 0:
                        raise RuntimeError('Vehicle unstable; stopping probe')
                    tick += 1
                bounds[phase] = (begin, len(rows))
            write_json(output/f'trajectory-{repeat}.json', rows)
            report = assess(rows, bounds, config['acceptance'])
            reports.append(report); traces.append(rows)
        tolerance = config['acceptance']
        max_position = max(math.dist(a['position_m'], b['position_m']) for a,b in zip(*traces))
        max_speed = max(abs(a['speed_m_s']-b['speed_m_s']) for a,b in zip(*traces))
        repeatable = max_position <= tolerance['repeat_position_tolerance_m'] and max_speed <= tolerance['repeat_speed_tolerance_m_s']
        result = dict(passed=all(r['passed'] for r in reports) and repeatable,
                      episodes=reports, repeats=config['repeats'], repeatable=repeatable,
                      maximum_repeat_position_difference_m=max_position,
                      maximum_repeat_speed_difference_m_s=max_speed,
                      python=platform.python_version(), total_wall_s=time.perf_counter()-started,
                      process=process_sample(), no_sumo=True, no_lidar=True,
                      limitations=['Low-speed exploratory fixture, not highway calibration',
                                   'Isaac Lab reset/vectorization not yet exercised',
                                   'No lane following, dynamic obstacles or sensor synchronization'])
    except Exception as error:
        result.update(error=repr(error), traceback=traceback.format_exc())
        traceback.print_exc()
    finally:
        if rows:
            write_json(output/f'trajectory-{repeat}.json', rows)
        write_json(output/'probe-result.json', result)
        print('PHYSICS_VEHICLE_RESULT='+json.dumps(result), flush=True)
        if sim:
            sim.detach_stage()
        if app:
            app.close()
    return 0 if result['passed'] else 1


def assess(rows, bounds, acceptance):
    """Predeclared smoke gates. All telemetry retained even when a gate fails."""
    phase = lambda name: rows[bounds[name][0]:bounds[name][1]]
    straight, brake, hold, turn, dropout = [phase(p) for p in ('straight','brake','hold','turn','dropout')]
    start_brake = rows[bounds['brake'][0]-1]
    stop = next((r for r in brake if r['speed_m_s'] <= acceptance['stopped_speed_max_m_s']), brake[-1])
    stopping_rows = [start_brake] + [r for r in brake if r['tick'] <= stop['tick']]
    distance = sum(math.dist(a['position_m'][:2], b['position_m'][:2]) for a,b in zip(stopping_rows, stopping_rows[1:]))
    drift = max(abs(r['position_m'][1]-straight[0]['position_m'][1]) for r in straight)
    hold_drift = max(math.dist(r['position_m'][:2], hold[0]['position_m'][:2]) for r in hold)
    delta_yaw = math.atan2(math.sin(turn[-1]['yaw_rad']-turn[0]['yaw_rad']), math.cos(turn[-1]['yaw_rad']-turn[0]['yaw_rad']))
    active = [r for r in rows if r['phase'] != 'settle']
    grounded_fraction = sum(all(r['wheel_on_ground']) for r in active)/len(active)
    gates = dict(
        target_speed=acceptance['straight_speed_min_m_s'] <= straight[-1]['speed_m_s'] <= acceptance['straight_speed_max_m_s'],
        straight_drift=drift <= acceptance['straight_lateral_drift_max_m'],
        forward=straight[-1]['position_m'][0]-straight[0]['position_m'][0] >= acceptance['straight_forward_progress_min_m'] and straight[-1]['velocity_m_s'][0] > 0,
        braking=start_brake['speed_m_s'] >= acceptance['straight_speed_min_m_s'] and distance <= acceptance['braking_distance_max_m'] and brake[-1]['speed_m_s'] <= acceptance['stopped_speed_max_m_s'],
        hold=hold_drift <= acceptance['hold_drift_max_m'] and max(r['speed_m_s'] for r in hold) <= acceptance['stopped_speed_max_m_s'],
        turn_left=delta_yaw >= acceptance['turn_yaw_min_rad'],
        upright=min(r['upright_z'] for r in rows) >= acceptance['upright_min_z'],
        height=all(acceptance['root_height_min_m'] <= r['position_m'][2] <= acceptance['root_height_max_m'] for r in rows),
        grounded=grounded_fraction >= acceptance['grounded_fraction_min'],
        expiry_stop=dropout[-1]['speed_m_s'] <= acceptance['stopped_speed_max_m_s'] and dropout[-1]['control']['throttle'] == 0 and dropout[-1]['control']['brake'] == 1)
    return dict(passed=all(gates.values()), gates=gates, straight_speed_m_s=straight[-1]['speed_m_s'],
                straight_lateral_drift_m=drift, brake_distance_m=distance,
                brake_entry_speed_m_s=start_brake['speed_m_s'],
                all_wheels_grounded_fraction=grounded_fraction,
                brake_final_speed_m_s=brake[-1]['speed_m_s'], hold_drift_m=hold_drift,
                turn_yaw_rad=delta_yaw, dropout_final_speed_m_s=dropout[-1]['speed_m_s'],
                minimum_upright_z=min(r['upright_z'] for r in rows))


if __name__ == '__main__':
    raise SystemExit(main())
