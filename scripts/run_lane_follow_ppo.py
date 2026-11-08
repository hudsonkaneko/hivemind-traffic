"""Native worker for the approved pilot; only the bounded supervisor launches it."""
from copy import deepcopy
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from experiments.probe_support import process_sample, write_json
from experiments.lane_follow_ppo_protocol import (
    APPROVAL_PATH, PILOT_ID, checkpoint_identity, digest, load_protocol, source_files, verify_sources,
)
from scripts.probe_vehicle_lab import lab_runtime_provenance


def transition_record(env, returned, policy_action, env_action, wall_seconds=0.0):
    """Copy the raw SB3 boundary before normalization or timeout bootstrapping."""
    observation, reward, done, infos = returned
    extras = env.extras
    row = deepcopy(extras['lane_transition'])
    row.update(returned_observation=observation[0].tolist(), returned_reward=float(reward[0]),
        returned_terminated=bool(extras['final_terminated'][0]),
        returned_truncated=bool(extras['final_truncated'][0]), returned_done=bool(done[0]),
        final_observation=extras['final_observation']['policy'][0].cpu().tolist(),
        sb3_time_limit_truncated=bool(infos[0]['TimeLimit.truncated']),
        step_wall_seconds=wall_seconds,
        policy_action_raw=[float(x) for x in policy_action[0]],
        env_action=[float(x) for x in env_action[0]])
    if done[0]:
        row['sb3_terminal_observation'] = infos[0]['terminal_observation'].tolist()
        row['auto_reset'] = deepcopy(extras['reset'])
    return row


def normalization_state(normalizer):
    """Stable content digest, independent of pickle internals or live wrappers."""
    rms = normalizer.obs_rms
    return {'training': normalizer.training, 'norm_obs': normalizer.norm_obs,
        'norm_reward': normalizer.norm_reward, 'clip_obs': normalizer.clip_obs,
        'epsilon': normalizer.epsilon, 'mean': rms.mean.tolist(), 'var': rms.var.tolist(),
        'count': float(rms.count)}


def deterministic_action(model, normalizer, raw_observation):
    """Record the actual Gaussian mean before the Box clip used by PPO.predict."""
    import numpy as np
    import torch
    observation = normalizer.normalize_obs(raw_observation)
    with torch.no_grad():
        tensor, _ = model.policy.obs_to_tensor(observation)
        raw = model.policy.get_distribution(tensor).get_actions(deterministic=True).cpu().numpy()
    if raw.shape != (1, 3) or not np.isfinite(raw).all():
        raise RuntimeError('Policy emitted malformed/nonfinite deterministic action')
    clipped = np.clip(raw, model.action_space.low, model.action_space.high)
    ordinary, _ = model.predict(observation, deterministic=True)
    if not np.array_equal(clipped, ordinary):
        raise RuntimeError('Recorded action differs from ordinary PPO prediction')
    return raw, clipped


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT / 'outputs/lab_lane_ppo') or not output.is_dir():
        parser.error('Use the bounded project supervisor')
    if (output / 'native-result.json').exists():
        parser.error('A completed native package is immutable')
    job = json.loads((output / 'resolved-config.json').read_text(encoding='utf-8'))
    protocol = load_protocol()
    if job['protocol'] != protocol:
        raise ValueError('Resolved proposal differs from the approved source')
    manifest = json.loads((output / 'manifest.json').read_text(encoding='utf-8'))
    if job != manifest['config'] or manifest['status'] != 'running':
        raise ValueError('Worker requires its original running supervisor package')
    if set(manifest['source_hashes']) != set(source_files()):
        raise ValueError('Worker source inventory differs from the approved runner')
    verify_sources(manifest)
    from experiments.run_lane_follow_ppo import resolve_job, verify_smoke
    expected_job = resolve_job(job['mode'], route=job['route_id'], stage=job['stage'],
        controller=job['controller'], checkpoint=None if job['checkpoint_path'] is None else ROOT / job['checkpoint_path'])
    if job['mode'] == 'train':
        claim_path = ROOT / 'outputs/lab_lane_ppo' / (PILOT_ID + '.claim.json')
        claim = json.loads(claim_path.read_text(encoding='utf-8'))
        expected_claim = {'pilot_id': PILOT_ID, 'output': output.relative_to(ROOT).as_posix(),
            'smoke': job['smoke_evidence'], 'approval_sha256': digest(ROOT / APPROVAL_PATH)}
        if claim != expected_claim or manifest.get('training_claim') != claim_path.relative_to(ROOT).as_posix():
            raise ValueError('Worker requires its matching durable single-training-run claim')
        expected_job['smoke_evidence'] = verify_smoke(ROOT / claim['smoke']['run'])
        # Exclusive consumption also prevents directly relaunching this native worker.
        with (output / 'worker-started.json').open('x', encoding='utf-8') as stream:
            json.dump({'claim': expected_claim}, stream)
    if job != expected_job:
        raise ValueError('Worker parameters differ from the fixed supervisor job')
    started = time.perf_counter()
    deadline = time.monotonic() + job['max_process_seconds'] - 15
    result = {'passed': False, 'mode': job['mode'], 'training_started': False, 'optimizer_steps': 0}
    app = env = model = normalizer = raw = recorder = tap = None
    reset_records, episode_summaries, current_episode = {}, [], []
    try:
        from isaaclab.app import AppLauncher
        launcher = AppLauncher({'headless': True, 'device': 'cpu', 'visualizer': ['none']})
        app = launcher.app
        from traffic.physics_session import prepare_vehicle_runtime
        result['runtime_setup'] = prepare_vehicle_runtime()
        import isaaclab
        import numpy as np
        import torch
        from stable_baselines3 import PPO
        from stable_baselines3.common.vec_env import VecEnvWrapper, VecNormalize
        from environments.lane_following_lab import LaneFollowingLabCfg, LaneFollowingLabEnv
        from environments.lane_follow_sb3 import make_lane_follow_sb3_wrapper
        from environments.lane_follow_ppo import create_model
        from experiments.lane_follow_ppo_evaluation import case_label, reduce_episode
        torch.set_num_threads(4)
        result['runtime'] = lab_runtime_provenance(isaaclab, torch)
        versions = {name: importlib.metadata.version(name) for name in
                    ('stable-baselines3', 'gymnasium', 'torch', 'numpy', 'isaaclab', 'isaaclab_rl')}
        result['versions'] = versions
        expected = protocol['read_only_runtime_inventory']
        for package, key in [('stable-baselines3', 'stable_baselines3'), ('gymnasium', 'gymnasium'),
                             ('torch', 'torch'), ('numpy', 'numpy'), ('isaaclab', 'isaaclab'), ('isaaclab_rl', 'isaaclab_rl')]:
            if versions[package] != expected[key]:
                raise RuntimeError('Pinned runtime version changed: ' + package)
        if platform.python_version() != expected['python'] or result['runtime']['isaaclab_git_commit'] != expected['isaaclab_git']:
            raise RuntimeError('Pinned Python/Isaac Lab source changed')
        if result['runtime'].get('isaaclab_git_dirty') is not False:
            raise RuntimeError('External Isaac Lab checkout must remain unchanged')
        cfg = LaneFollowingLabCfg()
        cfg.route_id = job['route_id']
        cfg.routes_file = str(ROOT / protocol['task']['route_file'])
        cfg.scene_directory = str(output / 'scene')
        cfg.seed = 2101
        cfg.reset_perturb = True
        cfg.sim.log_dir = str(ROOT / 'logs/lab_lane_ppo' / output.name / 'isaac')
        env = LaneFollowingLabEnv(cfg)
        write_json(output / 'vehicle-model.json', env.vehicle.metadata)
        initial_scene = env.scene_contract()
        write_json(output / 'initial-scene-contract.json', initial_scene)

        def check_deadline():
            if (output / 'STOP').exists():
                raise RuntimeError('Supervisor requested stop: ' + (output / 'STOP').read_text(encoding='utf-8'))
            if time.monotonic() >= deadline:
                raise TimeoutError('Native process deadline reached')

        def reset_snapshot():
            return {'last_reset': deepcopy(env.last_reset), 'episode_tick': env.episode_tick,
                    'episode_serial': env.episode_serial, 'reset_count': env.reset_count,
                    'next_reset_index': env._next_reset_index}

        class RawTap(VecEnvWrapper):
            """Observation-only tap; no step/reset/normalization semantics change."""
            def reset(self):
                return self.venv.reset()

            def step_async(self, actions):
                self.before_reset = deepcopy(env.last_reset)
                self.step_started = time.perf_counter()
                self.venv.step_async(actions)

            def step_wait(self):
                returned = self.venv.step_wait()
                self.last = deepcopy(returned)
                self.step_wall_seconds = time.perf_counter()-self.step_started
                return returned

        raw = make_lane_follow_sb3_wrapper(env)
        print('LAB_LANE_TASK_READY', flush=True)
        provenance = {'proposal_sha256': protocol['historical_proposal_sha256'],
            'source_hashes': manifest['source_hashes'], 'runtime': result['runtime'],
            'versions': versions, 'asset_hashes': initial_scene['static_hashes'],
            'git_commit': manifest['git_commit'], 'git_status': manifest['git_status']}

        if job['mode'] in ('smoke', 'train'):
            tap = RawTap(raw)
            normalizer = VecNormalize(tap, training=True, norm_obs=True, norm_reward=False,
                                      clip_obs=10, epsilon=1e-8, gamma=.999)
            reset_records, episode_summaries, current_episode = {}, [], []
            recorder = (output / 'training-transitions.jsonl').open('x', encoding='utf-8') if job['mode'] == 'train' else None

            def training_step(local, policy):
                check_deadline()
                record = transition_record(env, tap.last, local['actions'], local['clipped_actions'], tap.step_wall_seconds)
                local['lane_transition'] = deepcopy(env.extras['lane_transition'])
                if not np.allclose(local['new_obs'], normalizer.normalize_obs(tap.last[0]), atol=1e-6, rtol=1e-6):
                    raise RuntimeError('Normalized observation boundary mismatch')
                if record['returned_done']:
                    expected_terminal = normalizer.normalize_obs(tap.last[3][0]['terminal_observation'])
                    if not np.allclose(local['infos'][0]['terminal_observation'], expected_terminal, atol=1e-6, rtol=1e-6):
                        raise RuntimeError('Terminal observation normalized incorrectly')
                record['normalized_returned_observation'] = local['new_obs'][0].tolist()
                record['global_control_step'] = int(policy.num_timesteps)
                serial = str(record['episode_serial'])
                if serial not in reset_records:
                    reset_records[serial] = deepcopy(tap.before_reset)
                    write_json(output / 'training-resets.json', reset_records)
                recorder.write(json.dumps(record, allow_nan=False, separators=(',', ':'))+'\n')
                current_episode.append(record)
                if record['returned_done']:
                    assessment = reduce_episode(current_episode, reset_records[serial], env.route, job['acceptance'])
                    if not assessment['evidence_valid']:
                        raise RuntimeError('Training episode evidence failed independent validation: '+json.dumps(assessment))
                    episode_summaries.append({'episode_serial': int(serial),
                        'global_control_step': int(policy.num_timesteps), **assessment})
                    current_episode.clear()
                    write_json(output / 'training-episodes.json', episode_summaries)
                    write_json(output / 'training-resets.json', reset_records)
                return True

            def after_update(policy, record):
                recorder.flush()
                write_json(output / 'training-progress.json', policy.training_state())
                print('LAB_LANE_PPO_PROGRESS='+json.dumps(policy.training_state()), flush=True)

            model = create_model(normalizer, protocol, output_dir=output,
                deadline_monotonic=deadline, provenance=provenance, reset_state=reset_snapshot,
                on_step=training_step if job['mode'] == 'train' else None,
                after_update=after_update if job['mode'] == 'train' else None)
            result['policy_constructed'] = True

            if job['mode'] == 'train':
                result['training_started'] = True
                write_json(output / 'training-progress.json', model.training_state())
                # A fresh native process: this first learn reset consumes episode index0.
                model.learn(total_timesteps=65536, progress_bar=False)
                result['training_state'] = model.training_state()
                result['optimizer_steps'] = result['training_state']['optimizer_steps']
                result['completed_episodes'] = len(episode_summaries)
                result['partial_episode_control_steps'] = len(current_episode)
                write_json(output / 'training-episodes.json', episode_summaries)
                write_json(output / 'training-resets.json', reset_records)
                write_json(output / 'partial-episode.json', {'complete': False, 'records': current_episode})
                result['checkpoints'] = [dict(path=p.relative_to(ROOT).as_posix(),
                    identity=checkpoint_identity(p, require_completed_run=False)) for p in sorted((output / 'checkpoints').glob('update-*'))]
                state = result['training_state']
                result['passed'] = (state['status'] == 'completed' and state['control_steps'] == 65536 and state['completed_updates'] == 32
                    and 0 < state['optimizer_steps'] <= 5120
                    and [p['identity']['update'] for p in result['checkpoints']] == [8, 16, 24, 32])
            else:
                before = {key: value.detach().cpu().clone() for key, value in model.policy.state_dict().items()}
                observation = normalizer.reset()
                policy_rows = []
                for _ in range(300):
                    check_deadline()
                    action, _ = model.predict(observation, deterministic=True)
                    observation, rewards, done, _ = normalizer.step(action)
                    if not np.isfinite(observation).all() or not np.isfinite(rewards).all():
                        raise RuntimeError('Nonfinite smoke policy boundary')
                    policy_rows.append({'tick': env.extras['lane_transition']['tick'],
                                        'action': action.tolist(), 'done': bool(done[0])})
                observation, extras = env.reset(seed=2101, options={'perturb': True})
                initial_reset = deepcopy(extras['reset'])
                scripted_rows = []
                for _ in range(2100):
                    check_deadline()
                    action = env.scripted_action().cpu().numpy()
                    observation, reward, done, info = normalizer.step(action)
                    row = transition_record(env, tap.last, action, action, tap.step_wall_seconds)
                    scripted_rows.append(row)
                    if done[0]:
                        wanted = normalizer.normalize_obs(tap.last[3][0]['terminal_observation'])
                        if not np.array_equal(info[0]['terminal_observation'], wanted):
                            raise RuntimeError('Smoke terminal normalization mismatch')
                        break
                assessment = reduce_episode(scripted_rows, initial_reset, env.route, job['acceptance'])
                if not assessment['evidence_valid'] or not assessment['driving_success']:
                    raise RuntimeError('Smoke scripted completion failed: '+json.dumps(assessment))
                write_json(output / 'smoke-policy-forward.json', policy_rows)
                write_json(output / 'smoke-scripted-trajectory.json', scripted_rows)
                write_json(output / 'smoke-scripted-reset.json', initial_reset)
                normalizer.training = False
                model.save(output / 'smoke-model.zip')
                normalizer.save(output / 'smoke-vecnormalize.pkl')
                restored = PPO.load(output / 'smoke-model.zip', device='cpu')
                restored_norm = VecNormalize.load(output / 'smoke-vecnormalize.pkl', tap)
                restored_norm.training, restored_norm.norm_reward = False, False
                raw_observation = tap.last[0]
                a = deterministic_action(model, normalizer, raw_observation)
                b = deterministic_action(restored, restored_norm, raw_observation)
                checks = {'scripted_completion': assessment['driving_success'],
                    'deterministic_roundtrip': all(np.array_equal(x,y) for x,y in zip(a,b)),
                    'normalizer_roundtrip': normalization_state(normalizer) == normalization_state(restored_norm),
                    'parameters_unchanged': all(torch.equal(before[k],v.cpu()) for k,v in model.policy.state_dict().items()),
                    'no_optimizer_state': len(model.policy.optimizer.state) == 0,
                    'no_optimizer_steps': model.training_state()['optimizer_steps'] == 0,
                    'no_learning_transitions': model.num_timesteps == 0 and model._n_updates == 0,
                    'loaded_parameters_equal': all(torch.equal(before[k],v.cpu()) for k,v in restored.policy.state_dict().items())}
                result.update(smoke_checks=checks, scripted_assessment=assessment, passed=all(checks.values()))
                restored = restored_norm = None

        else:
            checkpoint = None if job['checkpoint_path'] is None else ROOT / job['checkpoint_path']
            if checkpoint is not None:
                if checkpoint_identity(checkpoint) != job['checkpoint_identity']:
                    raise RuntimeError('Checkpoint identity changed before load')
                model = PPO.load(checkpoint / 'model.zip', device='cpu')
                normalizer = VecNormalize.load(checkpoint / 'vecnormalize.pkl', raw)
                normalizer.training, normalizer.norm_reward = False, False
                before_norm = normalization_state(normalizer)
                parameter_hash = hashlib.sha256(b''.join(v.detach().cpu().numpy().tobytes() for v in model.policy.state_dict().values())).hexdigest()
            reports = []
            for seed in job['seeds']:
                check_deadline()
                observation, extras = env.reset(seed=seed, options={'perturb': True})
                first = deepcopy(extras['reset'])
                raw_observation = observation['policy'].cpu().numpy()
                records = []
                loop_started = time.perf_counter()
                label = case_label(job['stage'], job['controller'], job['route_id'], seed,
                    None if checkpoint is None else job['checkpoint_identity']['update'])
                for _ in range(2100):
                    check_deadline()
                    if model is not None:
                        requested, action = deterministic_action(model, normalizer, raw_observation)
                    else:
                        requested = action = env.scripted_action().cpu().numpy()
                    wall = time.perf_counter()
                    returned = raw.step(action)
                    records.append(transition_record(env, returned, requested, action, time.perf_counter()-wall))
                    raw_observation = returned[0]
                    if returned[2][0]:
                        break
                assessment = reduce_episode(records, first, env.route, job['acceptance'])
                name = f'seed-{seed}'
                write_json(output / (name+'-trajectory.json'), records)
                write_json(output / (name+'-reset.json'), first)
                reports.append({'label': label, 'policy': job['controller'],
                    'checkpoint': job['checkpoint_identity'], 'route_id': job['route_id'], 'seed': seed,
                    'trajectory_file': name+'-trajectory.json', 'reset_file': name+'-reset.json',
                    'loop_wall_seconds': time.perf_counter()-loop_started, 'assessment': assessment})
                write_json(output / 'evaluation-cases.json', reports)
                print('LAB_LANE_PPO_EVALUATION='+json.dumps(reports[-1]), flush=True)
            if model is not None:
                after_hash = hashlib.sha256(b''.join(v.detach().cpu().numpy().tobytes() for v in model.policy.state_dict().values())).hexdigest()
                result['evaluation_frozen'] = before_norm == normalization_state(normalizer) and parameter_hash == after_hash
            else:
                result['evaluation_frozen'] = True
            result.update(cases=reports, passed=len(reports) == len(job['seeds'])
                and all(case['assessment']['evidence_valid'] for case in reports) and result['evaluation_frozen'])
        result['scene_contract'] = env.scene_contract()
        write_json(output / 'scene-contract.json', result['scene_contract'])
        result['passed'] = bool(result.get('passed') and result['scene_contract']['passed']
                                and result['scene_contract']['static_hashes_unchanged'])
    except Exception as error:
        result.update(passed=False, error=repr(error), traceback=traceback.format_exc())
        traceback.print_exc()
    finally:
        if recorder is not None:
            recorder.flush()
            recorder.close()
        if job['mode'] == 'train' and model is not None:
            result['training_state'] = model.training_state()
            result['optimizer_steps'] = result['training_state']['optimizer_steps']
            result['training_started'] = result['training_state']['control_steps'] > 0
            write_json(output / 'training-progress.json', result['training_state'])
            write_json(output / 'training-resets.json', reset_records)
            write_json(output / 'training-episodes.json', episode_summaries)
            write_json(output / 'partial-episode.json', {'complete': False, 'records': current_episode})
            result['completed_episodes'] = len(episode_summaries)
            result['partial_episode_control_steps'] = len(current_episode)
            if env is not None:
                result['native_state_at_close'] = reset_snapshot()
        model = normalizer = raw = tap = None
        try:
            if env is not None:
                env.close()
                result['viewport_cleanup'] = deepcopy(env.viewport_cleanup)
                result['contact_reporting'] = deepcopy(env.contact_reporting)
                result['passed'] = bool(result.get('passed') and env.viewport_cleanup['passed']
                                        and env.contact_reporting['cleanup']['passed'])
                env = None
        except Exception as error:
            result.update(passed=False, close_error=repr(error))
            traceback.print_exc()
        result.update(total_wall_seconds=time.perf_counter()-started, process=process_sample())
        write_json(output / 'native-result.json', result)
        print('LAB_LANE_PPO_RESULT='+json.dumps(result), flush=True)
        if app is not None:
            app.close()
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
