"""Console-only fresh PPO training; does not consume or edit the original pilot."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import time
import uuid

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def read_config(path):
    path = Path(path).resolve()
    if not path.is_relative_to(ROOT):
        raise ValueError('Keep training configurations inside this checkout')
    config = json.loads(path.read_text(encoding='utf-8'))
    required = {'schema_version', 'run_name', 'seed', 'route', 'total_timesteps',
                'checkpoint_every', 'max_seconds', 'action_mapping', 'ppo', 'policy'}
    if set(config) != required or config['schema_version'] != 1:
        raise ValueError('Unsupported training configuration')
    if not re.fullmatch(r'[a-z][a-z0-9-]{0,63}', config['run_name']):
        raise ValueError('Invalid run_name')
    if config['route'] not in ('straight100', 'left_r60', 'right_r60'):
        raise ValueError('Unsupported route')
    if config['action_mapping'] != 'normalized_steering_signed_longitudinal':
        raise ValueError('Unsupported policy action mapping')
    for key in ('seed', 'total_timesteps', 'checkpoint_every', 'max_seconds'):
        if type(config[key]) is not int or config[key] < (0 if key == 'seed' else 1):
            raise ValueError('Invalid integer: ' + key)
    if config['seed'] >= 2**32:
        raise ValueError('Seed exceeds the task range')
    ppo = config['ppo']
    if set(ppo) != {'learning_rate', 'n_steps', 'batch_size', 'n_epochs', 'gamma',
                   'gae_lambda', 'clip_range', 'ent_coef', 'vf_coef', 'max_grad_norm', 'target_kl'}:
        raise ValueError('Unexpected PPO parameters')
    for key, value in ppo.items():
        if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
            raise ValueError('Invalid PPO value: ' + key)
    for key in ('n_steps', 'batch_size', 'n_epochs'):
        if type(ppo[key]) is not int or ppo[key] < 2:
            raise ValueError('Invalid PPO batch/rollout/epoch count')
    if (ppo['n_steps'] % ppo['batch_size'] or config['total_timesteps'] % ppo['n_steps']
            or config['checkpoint_every'] % ppo['n_steps']):
        raise ValueError('Budgets/checkpoints must align to complete rollouts and minibatches')
    if not (0 < ppo['gamma'] <= 1 and 0 < ppo['gae_lambda'] <= 1
            and 0 < ppo['clip_range'] < 1 and ppo['learning_rate'] > 0
            and ppo['max_grad_norm'] > 0 and ppo['target_kl'] > 0):
        raise ValueError('Invalid PPO range')
    policy = config['policy']
    if set(policy) != {'hidden_sizes', 'log_std_init'}:
        raise ValueError('Unexpected policy parameters')
    if (not isinstance(policy['hidden_sizes'], list) or not policy['hidden_sizes']
            or any(type(n) is not int or n <= 0 for n in policy['hidden_sizes'])
            or type(policy['log_std_init']) not in (int, float)
            or not math.isfinite(policy['log_std_init'])):
        raise ValueError('Invalid policy network')
    return config


def vehicle_actions(actions):
    """[-1,1] steering/longitudinal -> radians, throttle, brake; no overlap."""
    values = np.asarray(actions, dtype=np.float32)
    if values.ndim != 2 or values.shape[1] != 2 or not np.isfinite(values).all():
        raise ValueError('Expected finite (num_envs, 2) policy actions')
    values = np.clip(values, -1, 1)
    return np.column_stack((values[:, 0] * .5, np.maximum(values[:, 1], 0),
                            np.maximum(-values[:, 1], 0))).astype(np.float32)


def wrap_actions(raw):
    # Import only after AppLauncher, or explicitly in CPU-only wrapper tests.
    from gymnasium.spaces import Box
    from stable_baselines3.common.vec_env import VecEnvWrapper

    class SignedActions(VecEnvWrapper):
        def __init__(self, env):
            super().__init__(env, action_space=Box(-1., 1., shape=(2,), dtype=np.float32))

        def reset(self):
            return self.venv.reset()

        def step_async(self, actions):
            self.venv.step_async(vehicle_actions(actions))

        def step_wait(self):
            return self.venv.step_wait()

    return SignedActions(raw)


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def save_policy(model, normalizer, target, config):
    """Publish a complete model/normalizer pair atomically within a fresh run."""
    target = Path(target)
    temporary = target.with_name('.' + target.name + '-' + uuid.uuid4().hex)
    temporary.mkdir(parents=True, exist_ok=False)
    model.save(temporary / 'model.zip')
    normalizer.save(temporary / 'vecnormalize.pkl')
    write_json(temporary / 'policy.json', {
        'config': config, 'timesteps': int(model.num_timesteps),
        'action_order': ['normalized_steering', 'signed_longitudinal'],
        'positive_longitudinal': 'throttle', 'negative_longitudinal': 'brake',
        'steering_scale_rad': .5,
        'inference_normalization': 'Load vecnormalize.pkl with training=False and norm_reward=False',
        'sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                   for p in temporary.iterdir() if p.is_file()},
    })
    temporary.rename(target)


def train(config, output):
    from isaaclab.app import AppLauncher
    app = AppLauncher({'headless': True, 'device': 'cpu', 'visualizer': ['none']}).app
    env = normalizer = None
    try:
        from traffic.physics_session import prepare_vehicle_runtime
        prepare_vehicle_runtime()
        import torch
        from stable_baselines3 import PPO
        from stable_baselines3.common.callbacks import BaseCallback
        from stable_baselines3.common.logger import configure
        from stable_baselines3.common.vec_env import VecNormalize
        from environments.lane_following_lab import LaneFollowingLabCfg, LaneFollowingLabEnv
        from environments.lane_follow_sb3 import make_lane_follow_sb3_wrapper

        torch.set_num_threads(4)
        cfg = LaneFollowingLabCfg()
        cfg.route_id = config['route']
        cfg.routes_file = str(ROOT / 'scenarios/physics-road/routes.json')
        cfg.scene_directory = str(output / 'scene')
        cfg.seed = config['seed']
        cfg.reset_perturb = True
        log_dir = ROOT / 'logs/lane_follow_local' / output.name
        log_dir.mkdir(parents=True, exist_ok=False)
        cfg.sim.log_dir = str(log_dir / 'isaac')
        env = LaneFollowingLabEnv(cfg)
        contract = env.scene_contract()
        if contract.get('passed') is not True:
            raise RuntimeError('Native scene contract failed')
        normalizer = VecNormalize(wrap_actions(make_lane_follow_sb3_wrapper(env)),
            norm_obs=True, norm_reward=False, clip_obs=10., epsilon=1e-8,
            gamma=config['ppo']['gamma'])
        policy = config['policy']
        deadline = time.monotonic() + config['max_seconds']

        class ConsolePPO(PPO):
            def train(self):
                super().train()
                if not all(bool(torch.isfinite(p).all()) for p in self.policy.parameters()):
                    raise RuntimeError('Nonfinite policy parameters')
                if self.num_timesteps % config['checkpoint_every'] == 0:
                    save_policy(self, normalizer,
                                output / f'checkpoint-{self.num_timesteps:09d}', config)

        class Deadline(BaseCallback):
            def _on_step(self):
                if time.monotonic() > deadline:
                    raise TimeoutError('Configured training time limit reached; retain completed checkpoints')
                return True

        model = ConsolePPO('MlpPolicy', normalizer, device='cpu', seed=config['seed'],
            policy_kwargs={'net_arch': {'pi': policy['hidden_sizes'], 'vf': policy['hidden_sizes']},
                           'activation_fn': torch.nn.Tanh, 'ortho_init': True,
                           'log_std_init': policy['log_std_init']},
            verbose=1, **config['ppo'])
        model.set_logger(configure(str(log_dir), ['stdout', 'csv']))
        print(f'TRAINING {config["total_timesteps"]} steps; policy directory: {output}', flush=True)
        model.learn(total_timesteps=config['total_timesteps'], callback=Deadline(), progress_bar=False)
        if model.num_timesteps != config['total_timesteps']:
            raise RuntimeError('Unexpected training step count')
        save_policy(model, normalizer, output / 'final', config)
        print('POLICY=' + str(output / 'final/model.zip'), flush=True)
    finally:
        try:
            if normalizer is not None:
                normalizer.close()
            elif env is not None:
                env.close()
        finally:
            app.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'experiments/lane-follow-local-v2.json')
    parser.add_argument('--check', action='store_true', help='Validate only; do not launch Isaac or train')
    parser.add_argument('--native', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--output', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if Path.cwd().resolve() != ROOT:
        parser.error('Run from the Highway Sim repository root')
    config = read_config(args.config)
    if args.native:
        if args.check or args.output is None:
            parser.error('Native worker requires a fresh output directory')
        output = args.output.resolve()
        if output.parent != ROOT / 'outputs/lane_follow_local' or not output.is_dir():
            parser.error('Invalid native output path')
        if read_config(output / 'config.json') != config:
            parser.error('Native worker configuration changed')
        with (output / 'started').open('x', encoding='utf-8') as stream:
            stream.write(datetime.now(timezone.utc).isoformat())
        train(config, output)
        return 0
    if args.output is not None:
        parser.error('Output directories are allocated automatically')
    from hivemind.launcher import isaac_runtime, load_config
    runtime = isaac_runtime(ROOT, load_config(ROOT))
    if args.check:
        print(json.dumps({'config': config, 'runtime': str(runtime), 'training_started': False}, indent=2))
        return 0
    from experiments.verify_lane_following_lab import kit_process_snapshot
    processes = kit_process_snapshot()
    if not processes.get('available') or processes['count']:
        raise RuntimeError('Close existing Isaac/Kit processes before starting training')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    output = ROOT / 'outputs/lane_follow_local' / f'{config["run_name"]}-{stamp}-{uuid.uuid4().hex[:8]}'
    output.mkdir(parents=True, exist_ok=False)
    write_json(output / 'config.json', config)
    print('RUN_DIRECTORY=' + str(output), flush=True)
    return subprocess.call([str(runtime), str(Path(__file__).resolve()), '--native',
                            '--config', str(output / 'config.json'), '--output', str(output)], cwd=ROOT)


if __name__ == '__main__':
    raise SystemExit(main())
