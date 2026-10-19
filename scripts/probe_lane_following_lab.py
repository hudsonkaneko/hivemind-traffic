"""Native scripted task probe. No policy, optimizer or training loop is created."""
from copy import deepcopy
import argparse
import json
import math
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from experiments.probe_support import process_sample, write_json
from experiments.verify_lane_following_lab import validate_config
from scripts.probe_vehicle_lab import lab_runtime_provenance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT / 'outputs') or not output.is_dir():
        parser.error('Use the supervisor to create a unique outputs package')
    if (output / 'probe-result.json').exists():
        parser.error('Refusing to replace a completed probe')
    config = validate_config(json.loads((output / 'resolved-config.json').read_text(encoding='utf-8')))
    visible = config['profile'] == 'visible'
    app = env = viewport = wrapper = None
    result = {'passed': False, 'training_started': False, 'optimizer_steps': 0,
              'route_id': config['route_id'], 'profile': config['profile']}
    reports, captures, capture_metadata, all_resets = [], [], [], []
    started = time.perf_counter()
    try:
        from isaaclab.app import AppLauncher
        launcher_args = {'headless': not visible, 'device': 'cpu', 'visualizer': ['kit'] if visible else ['none']}
        if visible:
            launcher_args['kit_args'] = ('--/app/window/width=1440 --/app/window/height=900 '
                '--/app/runLoops/main/rateLimitEnabled=false --/app/vsync=false')
        launcher = AppLauncher(launcher_args)
        app = launcher.app
        from traffic.physics_session import prepare_vehicle_runtime
        setup = prepare_vehicle_runtime()
        import isaaclab
        import numpy as np
        import torch
        from environments.lane_following_contract import OBSERVATION_NAMES
        from environments.lane_following_lab import LaneFollowingLabCfg, LaneFollowingLabEnv
        from environments.lane_follow_sb3 import make_lane_follow_sb3_wrapper
        from experiments.lane_following_lab_validation import assess_episode
        cfg = LaneFollowingLabCfg()
        cfg.route_id = config['route_id']
        cfg.routes_file = str(ROOT / config['routes_file'])
        cfg.scene_directory = str(output / 'scene')
        cfg.seed = config['validation_seed']
        cfg.render_during_step = visible
        cfg.diagnostic_contact_barrier = config['profile'] == 'contact-control'
        cfg.sim.log_dir = str(output / 'lab-logs')
        env = LaneFollowingLabEnv(cfg)
        result['runtime'] = lab_runtime_provenance(isaaclab, torch)
        result['runtime_setup'] = setup
        write_json(output / 'vehicle-model.json', env.vehicle.metadata)
        write_json(output / 'lab-interface.json', {
            'environment': 'LaneFollowingLabEnv(isaaclab.envs.DirectRLEnv)',
            'observations': list(OBSERVATION_NAMES), 'observation_shape': [1, 22],
            'observation_source': 'privileged simulator state and known analytic road',
            'actions': ['steering_rad', 'throttle', 'brake'],
            'action_low': [-0.5, 0, 0], 'action_high': [0.5, 1, 1],
            'num_envs': 1, 'physics_device': 'cpu', 'physics_hz': 120, 'control_hz': 60,
            'movement_authority': 'isaac_physx', 'training_started': False,
            'sb3_wrapper': 'project-local pre-reset terminal-observation correction',
        })
        if visible:
            from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
            viewport = get_active_viewport()
            if viewport is None:
                raise RuntimeError('Visible validation requires an active viewport')
            viewport.resolution = (config['visible']['viewport_width'], config['visible']['viewport_height'])
        print('LAB_LANE_TASK_READY', flush=True)

        def check_deadline():
            if time.perf_counter() - started > config['max_process_seconds'] - 10:
                raise TimeoutError('Bounded native task deadline reached')

        def reset(seed=101, options=None):
            nonlocal wrapper
            observation, extras = env.reset(seed=seed, options=options)
            wrapper = make_lane_follow_sb3_wrapper(env)
            record = deepcopy(extras['reset'])
            record['returned_observation'] = observation['policy'][0].detach().cpu().tolist()
            all_resets.append(record)
            return record

        def step(action):
            wall = time.perf_counter()
            observation, reward, done, infos = wrapper.step(action.detach().cpu().numpy())
            extras = env.extras
            row = deepcopy(extras['lane_transition'])
            row.update(returned_observation=observation[0].tolist(), returned_reward=float(reward[0]),
                returned_terminated=bool(extras['final_terminated'][0]),
                returned_truncated=bool(extras['final_truncated'][0]),
                returned_done=bool(done[0]), final_observation=extras['final_observation']['policy'][0].cpu().tolist(),
                sb3_time_limit_truncated=bool(infos[0]['TimeLimit.truncated']),
                step_wall_seconds=time.perf_counter() - wall)
            if done[0]:
                row['sb3_terminal_observation'] = infos[0]['terminal_observation'].tolist()
                row['auto_reset'] = deepcopy(extras['reset'])
                all_resets.append(deepcopy(extras['reset']))
            return row

        def episode(label, seed=101, options=None, controller='scripted', expected_reason='success'):
            first = reset(seed, options)
            rows = []
            captured = set()
            loop_started = time.perf_counter()
            try:
                for _ in range(round(config['task']['episode_seconds'] * config['control_hz']) + 1):
                    check_deadline()
                    if controller == 'timeout':
                        action = torch.tensor([[0., 0., 1.]], device='cpu')
                    elif controller == 'departure':
                        action = torch.tensor([[0.5, 0.45, 0.]], device='cpu')
                    else:
                        action = env.scripted_action()
                    row = step(action)
                    rows.append(row)
                    if visible:
                        camera = env.render_snapshot('follow', render=False)
                        viewport.camera_path = camera['camera_path']
                        for second in config['visible']['capture_sim_seconds']:
                            if row['tick'] >= round(second * 120) and second not in captured:
                                captured.add(second)
                                camera = env.render_snapshot('follow', render=True)
                                name = f'preview-{int(second):02d}s-follow.png'
                                captures.append(capture_viewport_to_file(viewport, str(output / name)))
                                capture_metadata.append(dict(file=name, **camera))
                    if row['terminated'] or row['truncated']:
                        break
                report = assess_episode(rows, first, env.route, config['acceptance'], expected_reason=expected_reason)
                report.update(label=label, seed=seed, expected_reason=expected_reason,
                    trajectory_file=f'{label}-trajectory.json', reset_file=f'{label}-reset.json',
                    loop_wall_seconds=time.perf_counter() - loop_started,
                    step_wall_seconds=sum(r['step_wall_seconds'] for r in rows),
                    control_steps=len(rows))
                reports.append(report)
                print('LAB_LANE_EPISODE=' + json.dumps(report), flush=True)
            finally:
                write_json(output / f'{label}-trajectory.json', rows)
                write_json(output / f'{label}-reset.json', first)

        profile = config['profile']
        if profile in ('nominal', 'visible'):
            episode('nominal')
        elif profile == 'perturbed':
            for index, (seed, corner) in enumerate(zip(config['perturbation_seeds'], config['perturbation_corners'])):
                episode(f'corner-{index}', seed, {'lateral_offset_m': corner[0], 'yaw_error_rad': corner[1]})
        elif profile == 'contact-control':
            episode('contact-positive-control', expected_reason='rigid_contact')
        elif profile == 'diagnostics':
            diagnostic_checks = {}
            # Signed native resets, same-seed repeat, and fresh wheel/velocity state.
            resets_a = [reset(seed, {'perturb': True}) for seed in (1101, 1102, 1101)]
            reset_values = lambda r: (r['reset_spec'], r['physics_state'], r['wheel_rotation_speed_rad_s'])
            diagnostic_checks['same_seed_native_reset'] = reset_values(resets_a[0]) == reset_values(resets_a[2])
            diagnostic_checks['different_seed_reset'] = resets_a[0]['reset_spec'] != resets_a[1]['reset_spec']
            signed_resets = [reset(101, {'lateral_offset_m': lateral, 'yaw_error_rad': yaw})
                             for lateral, yaw in config['perturbation_corners']]
            diagnostic_checks['signed_native_pose_reset'] = all(
                math.dist(r['physics_state']['position_m'][:2], [r['reset_spec']['x_m'], r['reset_spec']['y_m']]) < 1e-6
                and abs(r['physics_state']['yaw_rad'] - r['reset_spec']['yaw_rad']) < 1e-6
                and r['physics_state']['speed_m_s'] < 1e-6
                and max(abs(v) for v in r['wheel_rotation_speed_rad_s']) < 1e-6 for r in signed_resets)
            reset()
            rejected = []
            for action in (torch.zeros((3,)), torch.zeros((2, 3)),
                           torch.tensor([[float('nan'), 0., 0.]]), torch.tensor([[0., float('inf'), 0.]])):
                before = (env.episode_tick, env._total_physics_ticks, deepcopy(env.vehicle.state(refresh_usd=False)))
                try:
                    env.step(action)
                    rejected.append(False)
                except ValueError:
                    after = (env.episode_tick, env._total_physics_ticks, env.vehicle.state(refresh_usd=False))
                    rejected.append(before == after)
            diagnostic_checks['invalid_actions_rejected_without_physics'] = all(rejected)
            for _ in range(120):
                step(torch.tensor([[0., 0., 1.]]))
            clipped = step(torch.tensor([[2., 2., 2.]]))
            diagnostic_checks['finite_clipping_and_brake_priority'] = (
                clipped['requested_action'] == [0.5, 0.0, 1.0]
                and clipped['control']['throttle'] == 0 and clipped['control']['brake'] == 1)
            result['diagnostic_checks'] = diagnostic_checks
            write_json(output / 'diagnostics.json', {'checks': diagnostic_checks, 'invalid_rejections': rejected,
                                                     'clipped_transition': clipped, 'signed_resets': signed_resets})
            episode('zero-drive-timeout', controller='timeout', expected_reason='time_limit')
            episode('hard-steer-departure', controller='departure', expected_reason='lane_departure')

        result['scene_contract'] = env.scene_contract()
        write_json(output / 'scene-contract.json', result['scene_contract'])
        if visible:
            # Inspect the real composed stage with rendering only; no extra physics.
            camera = env.render_snapshot('overview', render=True)
            viewport.camera_path = camera['camera_path']
            for _ in range(8):
                env.sim.render()
            captures.append(capture_viewport_to_file(viewport, str(output / 'preview-scene-overview.png')))
            capture_metadata.append(dict(file='preview-scene-overview.png', **camera))
            write_json(output / 'inspection-ready.json', {'stage': 'live composed Lab stage', 'seconds': 45,
                                                        'episode_tick': env.episode_tick})
            print('LAB_LANE_INSPECTION_READY=' + str(output), flush=True)
            until = time.perf_counter() + config['visible']['inspection_hold_wall_seconds']
            while time.perf_counter() < until:
                check_deadline()
                env.sim.render()
                time.sleep(0.025)
            for _ in range(8):
                env.sim.render()
            result['preview_files'] = [p.name for p in output.glob('preview-*.png')]
            result['preview_complete'] = len(result['preview_files']) == 4 and all(
                (output / row['file']).stat().st_size > 10000 for row in capture_metadata)
        result.update(episodes=reports, passed=bool(reports) and all(r['passed'] for r in reports)
            and all(result.get('diagnostic_checks', {}).values()) and result.get('preview_complete', True)
            and result['scene_contract']['passed'] and result['scene_contract']['static_hashes_unchanged'])
    except Exception as error:
        result.update(passed=False, error=repr(error), traceback=traceback.format_exc())
        traceback.print_exc()
    finally:
        write_json(output / 'resets.json', all_resets)
        write_json(output / 'captures.json', capture_metadata)
        try:
            if viewport is not None and env is not None:
                viewport.camera_path = '/OmniverseKit_Persp'
                env.sim.render()
            captures.clear()
            viewport = wrapper = None
            if env is not None:
                env.close()
                result['viewport_cleanup'] = deepcopy(env.viewport_cleanup)
                result['contact_reporting'] = deepcopy(env.contact_reporting)
                if not env.viewport_cleanup['passed'] or not env.contact_reporting['cleanup']['passed']:
                    result['passed'] = False
                env = None
        except Exception as error:
            result.update(passed=False, close_error=repr(error))
            traceback.print_exc()
        result.update(total_wall_seconds=time.perf_counter() - started, process=process_sample(),
            training_started=False, optimizer_steps=0,
            limitations=['One CPU PhysX car; no cloning or vectorized throughput claim',
                         'Privileged state and analytic map; no perception or traffic',
                         'Scripted task validation only; PPO proposal requires user approval'])
        write_json(output / 'probe-result.json', result)
        print('LAB_LANE_PROBE_RESULT=' + json.dumps(result), flush=True)
        if app is not None:
            app.close()
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
