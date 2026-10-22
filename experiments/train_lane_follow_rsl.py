"""Fresh console training with project-local RSL-RL; no automatic launch on import."""
from __future__ import annotations
import argparse
from copy import deepcopy
from datetime import datetime, timezone
import importlib.metadata
import json
import os
from pathlib import Path
import random
import re
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
DEPENDENCIES = ROOT / 'runtimes/rsl_rl'


def read_config(path):
    path = Path(path).resolve()
    if not path.is_relative_to(ROOT):
        raise ValueError('Configuration must be inside this checkout')
    cfg = json.loads(path.read_text(encoding='utf-8'))
    if (cfg['schema_version'] != 1 or cfg['rsl_rl_version'] != '5.0.0'
            or cfg['action_mapping'] != 'normalized_steering_signed_longitudinal'
            or cfg['route'] not in ('straight100', 'left_r60', 'right_r60')
            or not re.fullmatch(r'[a-z][a-z0-9-]{0,63}', cfg['run_name'])):
        raise ValueError('Unsupported training configuration')
    for value in (cfg['seed'], cfg['total_timesteps'], cfg['max_seconds']):
        if type(value) is not int or value < 0:
            raise ValueError('Invalid training integer')
    if cfg['seed'] >= 2**32 or not cfg['max_seconds']:
        raise ValueError('Invalid seed/deadline')
    runner = cfg['runner']
    for name in ('num_steps_per_env', 'max_iterations', 'save_interval'):
        if type(runner[name]) is not int or runner[name] <= 0:
            raise ValueError('Invalid rollout/update/checkpoint count')
    if runner['num_steps_per_env'] * runner['max_iterations'] != cfg['total_timesteps']:
        raise ValueError('Training budget must match complete rollouts')
    algorithm = runner['algorithm']
    batches = algorithm['num_mini_batches']
    if type(batches) is not int or batches <= 0 or runner['num_steps_per_env'] % batches:
        raise ValueError('Minibatches must divide each rollout')
    if (algorithm['class_name'] != 'environments.lane_follow_rsl:TerminalObservationPPO'
            or runner['logger'] != 'tensorboard' or runner['check_for_nan'] is not True
            or runner['obs_groups'] != {'actor': ['policy'], 'critic': ['policy']}
            or algorithm['rnd_cfg'] is not None or algorithm['symmetry_cfg'] is not None):
        raise ValueError('Unsupported algorithm, logging or observation boundary')
    for name in ('actor', 'critic'):
        if runner[name]['class_name'] != 'MLPModel' or runner[name]['obs_normalization'] is not True:
            raise ValueError('Expected observation-normalized MLP models')
    json.dumps(cfg, allow_nan=False)
    return cfg


def dependency_check():
    installed = {d.metadata['Name'].lower().replace('_', '-'): d.version
                 for d in importlib.metadata.distributions(path=[str(DEPENDENCIES)])}
    for line in (ROOT / 'requirements-rsl.txt').read_text(encoding='utf-8').splitlines():
        if line and not line.startswith('#'):
            name, version = line.split('==')
            if installed.get(name.lower().replace('_', '-')) != version:
                raise RuntimeError('Install project dependencies: .venv-traffic/Scripts/python.exe -m pip '
                    'install --no-deps --only-binary=:all: --python-version 3.12 '
                    '--target runtimes/rsl_rl -r requirements-rsl.txt')


def train(cfg, output):
    sys.path.insert(0, str(DEPENDENCIES))
    from isaaclab.app import AppLauncher
    app = AppLauncher({'headless': True, 'device': 'cpu', 'visualizer': ['none']}).app
    env = wrapped = runner = None
    try:
        from traffic.physics_session import prepare_vehicle_runtime
        prepare_vehicle_runtime()
        import numpy as np
        import torch
        from rsl_rl.runners import OnPolicyRunner
        from environments.lane_following_lab import LaneFollowingLabCfg, LaneFollowingLabEnv
        from environments.lane_follow_rsl import wrap_lane_rsl
        torch.set_num_threads(4)
        random.seed(cfg['seed'])
        np.random.seed(cfg['seed'])
        torch.manual_seed(cfg['seed'])
        log_dir = ROOT / 'logs/lane_follow_rsl' / output.name
        log_dir.mkdir(parents=True, exist_ok=False)
        task = LaneFollowingLabCfg()
        task.route_id = cfg['route']
        task.routes_file = str(ROOT / 'scenarios/physics-road/routes.json')
        task.scene_directory = str(output / 'scene')
        task.sim.log_dir = str(log_dir / 'isaac')
        task.seed = cfg['seed']
        task.reset_perturb = True
        env = LaneFollowingLabEnv(task)
        if env.scene_contract().get('passed') is not True:
            raise RuntimeError('Native scene contract failed')
        wrapped = wrap_lane_rsl(env, deadline=time.monotonic() + cfg['max_seconds'])
        (output / 'checkpoints').mkdir()

        class LaneRunner(OnPolicyRunner):
            def save(self, path, infos=None):
                name = Path(path).name
                destination = output / name if name == 'model_final.pt' else output / 'checkpoints' / name
                super().save(str(destination), infos)

        runner = LaneRunner(wrapped, deepcopy(cfg['runner']), log_dir=str(log_dir), device='cpu')
        print(f'TRAINING RSL-RL: {cfg["total_timesteps"]} steps; output: {output}', flush=True)
        runner.learn(cfg['runner']['max_iterations'], init_at_random_ep_len=False)
        if wrapped.control_steps != cfg['total_timesteps']:
            raise RuntimeError('Unexpected control step count')
        runner.save(str(output / 'model_final.pt'), infos={'config': cfg, 'steps': wrapped.control_steps})
        runner.get_inference_policy(device='cpu')
        runner.export_policy_to_jit(str(output), filename='policy.pt')
        print('POLICY=' + str(output / 'policy.pt'), flush=True)
    finally:
        try:
            if runner is not None:
                writer = getattr(runner.logger, 'writer', None)
                if writer is not None:
                    writer.flush()
                    writer.close()
        finally:
            try:
                if wrapped is not None:
                    wrapped.close()
                elif env is not None:
                    env.close()
            finally:
                app.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'experiments/lane-follow-rsl-v1.json')
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--native', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--output', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if Path.cwd().resolve() != ROOT:
        parser.error('Run from the Highway Sim checkout root')
    cfg = read_config(args.config)
    dependency_check()
    if args.native:
        if args.check or args.output is None:
            parser.error('Native worker requires its output directory')
        output = args.output.resolve()
        if output.parent != ROOT / 'outputs/lane_follow_rsl' or not output.is_dir():
            parser.error('Invalid native output directory')
        if read_config(output / 'config.json') != cfg:
            parser.error('Native worker configuration changed')
        with (output / 'started').open('x', encoding='utf-8') as stream:
            stream.write(datetime.now(timezone.utc).isoformat())
        train(cfg, output)
        return 0
    if args.output is not None:
        parser.error('Output directories are allocated automatically')
    from hivemind.launcher import isaac_runtime, load_config
    runtime = isaac_runtime(ROOT, load_config(ROOT))
    if args.check:
        print(json.dumps({'config': cfg, 'runtime': str(runtime), 'dependencies_ready': True,
                          'training_started': False}, indent=2))
        return 0
    from experiments.verify_lane_following_lab import kit_process_snapshot
    processes = kit_process_snapshot()
    if not processes.get('available') or processes['count']:
        raise RuntimeError('Close existing Isaac/Kit processes before starting another training run')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    output = ROOT / 'outputs/lane_follow_rsl' / f'{cfg["run_name"]}-{stamp}-{uuid.uuid4().hex[:8]}'
    output.mkdir(parents=True, exist_ok=False)
    config_path = output / 'config.json'
    config_path.write_text(json.dumps(cfg, indent=2) + '\n', encoding='utf-8')
    process_env = dict(os.environ)
    process_env['PYTHONPATH'] = str(DEPENDENCIES) + os.pathsep + process_env.get('PYTHONPATH', '')
    print('RUN_DIRECTORY=' + str(output), flush=True)
    return subprocess.call([str(runtime), str(Path(__file__).resolve()), '--native',
                            '--config', str(config_path), '--output', str(output)], cwd=ROOT, env=process_env)


if __name__ == '__main__':
    raise SystemExit(main())
