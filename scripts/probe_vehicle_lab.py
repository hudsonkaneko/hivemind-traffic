"""Bounded single-car Isaac Lab reset/step probe; launched by its supervisor."""
import argparse
import json
import math
from pathlib import Path
import platform
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from experiments.probe_support import process_sample, write_json


def lab_runtime_provenance(lab_module, torch_module):
    """Discover the imported runtime, never guess a local installation path."""
    source = Path(lab_module.__file__).resolve()
    metadata = dict(isaaclab_module_file=str(source), python_version=platform.python_version(),
                    torch_version=str(torch_module.__version__), isaaclab_git_commit=None)
    checkout = next((parent for parent in source.parents if (parent / '.git').exists()), None)
    if checkout is None:
        metadata['isaaclab_git_status'] = 'No parent Git checkout found for imported module'
        return metadata
    metadata['isaaclab_checkout'] = str(checkout)
    try:
        process = subprocess.run(['git', '-C', str(checkout), 'rev-parse', 'HEAD'],
                                 capture_output=True, text=True, timeout=5, check=True)
        metadata['isaaclab_git_commit'] = process.stdout.strip()
        dirty = subprocess.run(['git', '-C', str(checkout), 'status', '--porcelain'],
                               capture_output=True, text=True, timeout=5, check=True)
        metadata['isaaclab_git_dirty'] = bool(dirty.stdout.strip())
    except (OSError, subprocess.SubprocessError) as error:
        metadata['isaaclab_git_status'] = repr(error)
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT / 'outputs') or not output.is_dir():
        parser.error('Use the supervisor to create a unique project outputs directory')
    if (output / 'probe-result.json').exists():
        parser.error('Refusing to overwrite a completed probe')
    config = json.loads((output / 'resolved-config.json').read_text())
    from environments.physics_vehicle_contract import validate_config, require_finite_telemetry
    validate_config(config)
    app = env = None
    result = {'passed': False}
    trajectories, reset_states, report = [], [], []
    rows, repeat = [], 0
    started = time.perf_counter()
    try:
        from isaaclab.app import AppLauncher
        launcher = AppLauncher({'headless': True, 'device': 'cpu', 'visualizer': ['none']})
        app = launcher.app
        from traffic.physics_session import prepare_vehicle_runtime
        runtime_setup = prepare_vehicle_runtime()
        import isaaclab
        import torch
        runtime = lab_runtime_provenance(isaaclab, torch)
        result['runtime'] = runtime
        from omni.physx.bindings._physx import VEHICLE_WHEEL_STATE_ROTATION_SPEED
        from environments.physics_vehicle_lab import PhysicsVehicleLabCfg, PhysicsVehicleLabEnv
        from environments.physics_vehicle_contract import ACTION_NAMES, OBSERVATION_NAMES
        cfg = PhysicsVehicleLabCfg()
        cfg.seed = config['seed']
        cfg.sim.dt = 1 / config['physics_hz']
        cfg.decimation = config['physics_hz'] // config['control_hz']
        cfg.sim.render_interval = cfg.decimation
        cfg.sim.log_dir = str(output / 'lab-logs')
        cfg.episode_length_s = sum(seconds for _, seconds in config['phases'])
        cfg.drive_torque_per_front_wheel_nm = config['drive_torque_per_front_wheel_nm']
        cfg.brake_torque_per_wheel_nm = config['brake_torque_per_wheel_nm']
        cfg.command_ttl_ticks = config['command_ttl_ticks']
        env = PhysicsVehicleLabEnv(cfg)
        write_json(output / 'vehicle-model.json', env.vehicle.metadata)
        write_json(output / 'lab-interface.json', {
            'environment_class': 'isaaclab.envs.DirectRLEnv', 'num_envs': 1,
            'chassis_class': 'isaaclab.assets.RigidObject', 'articulation': False,
            'observations': list(OBSERVATION_NAMES), 'observation_source': 'simulator_state',
            'actions': list(ACTION_NAMES), 'action_shape': [1, 3],
            'action_limits_low': [-0.5, 0, 0], 'action_limits_high': [0.5, 1, 1],
            'action_behavior': 'Finite actions clipped; braking suppresses throttle; nonfinite aborts before stepping',
            'observation_shape': [1, len(OBSERVATION_NAMES)], 'reward': 'zero; interface probe, no RL objective',
            'physics_dt_s': cfg.sim.dt, 'control_dt_s': env.step_dt,
            'movement_authority': 'isaac_physx', 'reset': 'Lab root pose/velocity tensor writes + native vehicle rest state',
            'runtime_setup': runtime_setup,
            'runtime': runtime,
        })
        obs, _ = env.reset(seed=config['seed'])
        initial_position = list(env.initial_position)

        def capture_reset(observation):
            state = env.vehicle.state()
            wheel_speeds = [env.vehicle.physx.get_wheel_state(path)[VEHICLE_WHEEL_STATE_ROTATION_SPEED]
                            for path in env.vehicle.wheel_paths]
            reset = dict(**state, wheel_rotation_speed_rad_s=wheel_speeds,
                         observation=observation['policy'].detach().cpu().tolist(),
                         position_error_m=math.dist(state['position_m'], initial_position),
                         episode_tick=env.episode_tick, reset_count=env.reset_count)
            require_finite_telemetry(reset, 'reset')
            return reset

        reset_states.append(capture_reset(obs))
        for repeat in range(config['repeats']):
            rows = []
            for phase, seconds in config['phases']:
                for _ in range(round(seconds / env.step_dt)):
                    if time.perf_counter() - started > config['max_process_seconds'] - 10:
                        raise TimeoutError('Isaac Lab probe deadline reached')
                    speed = env.last_state['speed_m_s']
                    throttle = min(1.0, max(0.0, (config['target_speed_m_s'] - speed) * 0.7)) if phase == 'straight' else 0.0
                    brake = 1.0 if phase in ('settle', 'brake') else 0.0
                    action = torch.tensor([[0.0, throttle, brake]], dtype=torch.float32, device='cpu')
                    wall = time.perf_counter()
                    obs, reward, terminated, truncated, extras = env.step(action)
                    state = extras['physics_state_before_reset']
                    tick = extras['episode_tick_before_reset']
                    row = dict(phase=phase, tick=tick, sim_time_s=tick * cfg.sim.dt,
                               step_wall_ms=(time.perf_counter() - wall) * 1000,
                               observation=obs['policy'].detach().cpu().tolist(),
                               reward=reward.tolist(), terminated=terminated.tolist(), truncated=truncated.tolist(),
                               control=extras['control_before_reset'], **state)
                    rows.append(row)
                    require_finite_telemetry(row)
                    if not torch.isfinite(obs['policy']).all() or tuple(obs['policy'].shape) != (1, len(OBSERVATION_NAMES)):
                        raise RuntimeError('Invalid Isaac Lab observation contract')
                    if tuple(reward.shape) != (1,) or not torch.isfinite(reward).all():
                        raise RuntimeError('Invalid Isaac Lab reward contract')
                    if terminated.any():
                        raise RuntimeError('Vehicle stability termination during compatibility probe')
                    expected_terminal_tick = round(cfg.episode_length_s / cfg.sim.dt)
                    if bool(truncated.item()) != (tick == expected_terminal_tick):
                        raise RuntimeError('Unexpected episode timing or missing auto-reset')
            write_json(output / f'trajectory-{repeat}.json', rows)
            trajectories.append(rows)
            reset_states.append(capture_reset(obs))
            straight = [r for r in rows if r['phase'] == 'straight']
            progress = straight[-1]['position_m'][0] - straight[0]['position_m'][0]
            gates = dict(
                physical_progress=progress >= config['acceptance']['minimum_forward_progress_m'],
                stopped=rows[-1]['speed_m_s'] <= config['acceptance']['stopped_speed_max_m_s'],
                finite_reward=all(r['reward'] == [0.0] for r in rows),
                automatic_reset=bool(rows[-1]['truncated'][0]) and env.episode_tick == 0,
                reset_count=env.reset_count == repeat + 2,
            )
            report.append(dict(passed=all(gates.values()), gates=gates,
                               forward_progress_m=progress, final_speed_m_s=rows[-1]['speed_m_s']))
        reference = trajectories[0]
        reference_ticks = [row['tick'] for row in reference]
        if any(len(other) != len(reference) or [row['tick'] for row in other] != reference_ticks
               for other in trajectories[1:]):
            raise RuntimeError('Cannot compare unequal-length or misaligned episode traces')
        position_difference = max(math.dist(a['position_m'], b['position_m'])
            for other in trajectories[1:] for a, b in zip(reference, other))
        speed_difference = max(abs(a['speed_m_s'] - b['speed_m_s'])
            for other in trajectories[1:] for a, b in zip(reference, other))
        acceptance = config['acceptance']
        reset_ok = all(s['position_error_m'] <= acceptance['reset_position_tolerance_m']
                       and s['speed_m_s'] <= acceptance['reset_speed_tolerance_m_s']
                       and max(abs(v) for v in s['wheel_rotation_speed_rad_s']) <= 1e-6
                       and s['episode_tick'] == 0 for s in reset_states)
        repeatable = (position_difference <= acceptance['repeat_position_tolerance_m']
                      and speed_difference <= acceptance['repeat_speed_tolerance_m_s'])
        result = dict(passed=all(r['passed'] for r in report) and reset_ok and repeatable,
            isaac_lab_direct_rl_env=True, episodes=report, resets=reset_states,
            reset_ok=reset_ok, repeatable=repeatable,
            all_trace_values_finite=True, runtime=runtime,
            maximum_repeat_position_difference_m=position_difference,
            maximum_repeat_speed_difference_m_s=speed_difference,
            python=platform.python_version(), total_wall_s=time.perf_counter() - started,
            process=process_sample(), no_sumo=True, no_lidar=True, no_training=True,
            limitations=['One CPU-physics environment only; no replication or multi-agent claim',
                         'Simulator-state observations, not sensor-derived observations',
                         'No training objective, policy training or highway-speed calibration'])
    except Exception as error:
        result.update(error=repr(error), traceback=traceback.format_exc())
        traceback.print_exc()
    finally:
        if rows:
            write_json(output / f'trajectory-{repeat}.json', rows)
        # Include teardown in the recorded outcome: a close failure is not a pass.
        try:
            if env is not None:
                env.close()
                env = None
        except Exception as error:
            result.update(passed=False, close_error=repr(error))
            traceback.print_exc()
        write_json(output / 'probe-result.json', result)
        print('PHYSICS_VEHICLE_LAB_RESULT=' + json.dumps(result), flush=True)
        if app is not None:
            app.close()
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
