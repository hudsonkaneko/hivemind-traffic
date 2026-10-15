"""Repeated fresh-stage vehicle resets with bounded telemetry and lifecycle evidence."""
import argparse
import gc
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


def validate_config(config):
    """Reject unbounded/malformed fixture changes before importing Isaac."""
    if not isinstance(config, dict):
        raise ValueError('Reset configuration must be an object')

    def number(container, key, low, high, integer=False):
        value = container.get(key)
        expected = int if integer else (int, float)
        if isinstance(value, bool) or not isinstance(value, expected) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f'{key} must be a finite {"integer" if integer else "number"} in [{low}, {high}]')
        return value

    number(config, 'schema_version', 1, 1, True)
    number(config, 'seed', 0, 2**32-1, True)
    if config.get('study') not in ('physics_vehicle_resets', 'physics_vehicle_reset_control'):
        raise ValueError('Unknown reset study')
    number(config, 'physics_hz', 120, 120, True)
    number(config, 'control_hz', 60, 60, True)
    repeats = number(config, 'repeats', 2, 10, True)
    if not isinstance(config.get('disable_authoring_tools'), bool):
        raise ValueError('disable_authoring_tools must be boolean')
    number(config, 'target_speed_m_s', .5, 1.0)
    number(config, 'drive_torque_per_front_wheel_nm', 1, 700)
    number(config, 'brake_torque_per_wheel_nm', 1, 1500)
    number(config, 'command_ttl_ticks', 1, 12, True)
    number(config, 'max_process_seconds', 30, 180)
    phases = config.get('phases')
    if not isinstance(phases, list) or len(phases) != 3:
        raise ValueError('Expected settle, straight and brake phases exactly once')
    duration = 0.0
    for phase, expected in zip(phases, ('settle', 'straight', 'brake')):
        if not isinstance(phase, list) or len(phase) != 2 or phase[0] != expected:
            raise ValueError('Expected ordered settle, straight and brake phases')
        duration += number({'duration': phase[1]}, 'duration', .5, 10.0)
        if not math.isclose(phase[1]*config['physics_hz'], round(phase[1]*config['physics_hz']), abs_tol=1e-9):
            raise ValueError('Phase duration must be a whole number of physics ticks')
    if duration > 10 or duration*repeats > 60:
        raise ValueError('Reset study limited to 10 simulated seconds per episode and 60 total')
    acceptance = config.get('acceptance')
    if not isinstance(acceptance, dict):
        raise ValueError('Acceptance thresholds must be an object')
    minimum = number(acceptance, 'entry_speed_min_m_s', .1, 1.0)
    maximum = number(acceptance, 'entry_speed_max_m_s', .5, 1.5)
    if not minimum <= config['target_speed_m_s'] <= maximum:
        raise ValueError('Entry-speed bounds must enclose the target')
    number(acceptance, 'stopped_speed_max_m_s', .001, .05)
    number(acceptance, 'minimum_upright_z', .95, 1.0)
    number(acceptance, 'repeat_position_tolerance_m', .000001, .1)
    number(acceptance, 'repeat_speed_tolerance_m_s', .000001, .05)
    number(acceptance, 'memory_warmup_episodes', 1, repeats-1, True)
    number(acceptance, 'post_warmup_rss_growth_max_mib', 1, 256)
    return config


def assess_resets(episodes, config):
    """Evaluate bounded smoke evidence, not a proof of long-run leak freedom."""
    acceptance = config['acceptance']
    baseline = episodes[0]['final_state']
    position_difference = max(math.dist(baseline['position_m'], e['final_state']['position_m']) for e in episodes)
    speed_difference = max(abs(baseline['speed_m_s']-e['final_state']['speed_m_s']) for e in episodes)
    postwarm = episodes[acceptance['memory_warmup_episodes']-1:]
    memory_available = bool(postwarm) and all(e['process_after_discard'].get('available') for e in postwarm)
    rss = [e['process_after_discard']['rss_bytes']/2**20 for e in postwarm] if memory_available else []
    growth = max(0.0, max(rss)-rss[0]) if rss else None
    caches = [e['closed_lifecycle']['cache_count'] for e in episodes]
    gates = dict(
        complete=len(episodes) == config['repeats'],
        dynamics=all(e['passed'] for e in episodes),
        repeat_position=position_difference <= acceptance['repeat_position_tolerance_m'],
        repeat_speed=speed_difference <= acceptance['repeat_speed_tolerance_m_s'],
        stage_cache_bounded=all(n <= caches[0] for n in caches),
        closed_stage_absent=all(not e['closed_lifecycle']['old_stage_cached_after_erase'] and
                                not e['closed_lifecycle']['context_has_stage'] for e in episodes),
        memory_available=memory_available,
        provisional_rss_bound=memory_available and growth <= acceptance['post_warmup_rss_growth_max_mib'])
    return dict(passed=all(gates.values()), gates=gates,
                maximum_repeat_position_difference_m=position_difference,
                maximum_repeat_speed_difference_m_s=speed_difference,
                post_warmup_rss_growth_mib=growth, post_close_cache_counts=caches)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'outputs') or not output.is_dir():
        parser.error('Use the supervisor to create a unique project output directory')
    if (output/'probe-result.json').exists() or list(output.glob('trajectory-*.json')):
        parser.error('Refusing to overwrite an existing probe attempt')
    config = validate_config(json.loads((output/'resolved-config.json').read_text()))
    app = session = vehicle = None
    rows, episodes = [], []
    repeat = 0
    started = time.perf_counter()
    result = {'passed': False}
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({'headless': True, 'width': 960, 'height': 540,
                             'disable_viewport_updates': True})
        from traffic.physics_session import PhysicsSession, prepare_vehicle_runtime
        from traffic.physx_vehicle import PhysxVehicle
        from traffic.driver_control import DriverCommand, DriverControlGate
        runtime = prepare_vehicle_runtime(config['disable_authoring_tools'])
        write_json(output/'vehicle-runtime.json', runtime)
        dt = 1/config['physics_hz']
        stride = config['physics_hz']//config['control_hz']
        for repeat in range(config['repeats']):
            print('RESET_EPISODE_BEGIN='+str(repeat), flush=True)
            session = PhysicsSession(config['physics_hz'])
            vehicle = PhysxVehicle(session.stage, config['physics_hz'])
            if repeat == 0:
                write_json(output/'vehicle-model.json', vehicle.metadata)
                session.stage.Export(str(output/'initial-scene.usda'))
            session.attach()
            gate = DriverControlGate(f'reset-{repeat}', 'ego')
            tick, sequence, rows = 0, 0, []
            state = None
            entry_speed, minimum_upright = None, 1.0
            for phase, seconds in config['phases']:
                if phase == 'brake':
                    entry_speed = state['speed_m_s']
                for _ in range(round(seconds/dt)):
                    if time.perf_counter()-started > config['max_process_seconds']-10:
                        raise TimeoutError('Reset probe deadline reached')
                    command = None
                    if tick % stride == 0:
                        speed = state['speed_m_s'] if state else 0.0
                        throttle = min(1.0, max(0.0, (config['target_speed_m_s']-speed)*0.7)) if phase == 'straight' else 0.0
                        command = DriverCommand(f'reset-{repeat}', 'ego', sequence, tick,
                                                tick+config['command_ttl_ticks'], 0.0, throttle,
                                                0.0 if phase == 'straight' else 1.0)
                        sequence += 1
                    applied = gate.step(tick=tick, dt_s=dt, command=command)
                    control = vehicle.apply(applied, config['drive_torque_per_front_wheel_nm'],
                                            config['brake_torque_per_wheel_nm'])
                    session.step(dt, tick*dt)
                    state = vehicle.state()
                    minimum_upright = min(minimum_upright, state['upright_z'])
                    rows.append(dict(tick=tick+1, sim_time_s=(tick+1)*dt, phase=phase, control=control, **state))
                    if state['upright_z'] < .8 or state['position_m'][2] < 0:
                        raise RuntimeError('Unstable vehicle during reset probe')
                    tick += 1
            a = config['acceptance']
            episode = dict(episode=repeat, final_state=state, brake_entry_speed_m_s=entry_speed,
                           minimum_upright_z=minimum_upright,
                           passed=a['entry_speed_min_m_s'] <= entry_speed <= a['entry_speed_max_m_s'] and
                                  state['speed_m_s'] <= a['stopped_speed_max_m_s'] and
                                  minimum_upright >= a['minimum_upright_z'])
            write_json(output/f'trajectory-{repeat}.json', rows)
            # Retain only small summary dictionaries before measuring teardown.
            rows = []
            vehicle = None
            session.close()
            episode['closed_lifecycle'] = session.lifecycle[-1]
            write_json(output/f'lifecycle-{repeat}.json', session.lifecycle)
            session = None
            gc.collect()
            episode['process_after_discard'] = process_sample()
            episodes.append(episode)
            print('RESET_EPISODE_END='+json.dumps(episode), flush=True)
            write_json(output/'episodes.json', episodes)
        result = dict(**assess_resets(episodes, config), episodes=episodes,
                      repeats=config['repeats'], runtime=runtime, python=platform.python_version(),
                      total_wall_s=time.perf_counter()-started, no_sumo=True, no_lidar=True,
                      limitations=['Ten short fresh-stage episodes are not a long-run memory-leak proof',
                                   'RSS gate is a provisional 256 MiB post-warmup budget, not calibrated scaling',
                                   'No Isaac Lab reset/vectorization, sensor lifecycle or GUI endurance validation'])
    except Exception as error:
        result.update(error=repr(error), traceback=traceback.format_exc(), episodes=episodes)
        traceback.print_exc()
    finally:
        if rows:
            write_json(output/f'trajectory-{repeat}.json', rows)
        vehicle = None
        if session is not None:
            try:
                session.close()
                write_json(output/f'lifecycle-{repeat}.json', session.lifecycle)
            except Exception as error:
                result.update(passed=False, cleanup_error=repr(error))
        write_json(output/'probe-result.json', result)
        print('PHYSICS_RESET_RESULT='+json.dumps(result), flush=True)
        if app is not None:
            app.close()
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
