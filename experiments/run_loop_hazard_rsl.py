"""Bounded native hazard probes, demonstrations, RSL-RL training and evaluation."""
from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
DEPS = ROOT / 'runtimes/rsl_rl'


def atomic_json(path, data):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    os.replace(temporary, path)


def source_hashes():
    from experiments.verify_lane_following_lab import source_files
    names = set(source_files()) | {
        'experiments/run_loop_hazard_rsl.py', 'environments/loop_hazard_lab.py',
        'environments/loop_hazard_contract.py', 'traffic/road_hazards.py',
        'environments/loop_hazard_rsl.py',
        'experiments/loop_hazard_training.py', 'experiments/loop_hazard_support.py',
        'experiments/loop-hazard-rsl-v1.json',
        'usd/loop_hazard_scene.py', 'traffic/rsl_loop_policy.py',
        'traffic/bypass_validation.py',
        'environments/lane_follow_rsl.py', 'requirements-rsl.txt',
        'scenarios/physics-road/loop-hazard-routes-v1.json',
    }
    return {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in sorted(names)}


def episode_case(args, index):
    if args.stage != 'mixed':
        return args.stage, args.side
    return [('clear', None), ('static', None), ('crossing', 'left'), ('crossing', 'right')][index % 4]


def resolved_arguments(args):
    return {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()
            if k not in ('native', 'output')}


def policy_state_hash(policy):
    digest = hashlib.sha256()
    for name, tensor in sorted(policy.state_dict().items()):
        digest.update(name.encode('utf-8'))
        digest.update(str((tensor.dtype, tuple(tensor.shape))).encode('utf-8'))
        digest.update(tensor.detach().cpu().contiguous().numpy().tobytes())
    return digest.hexdigest()


def native(args, output):
    sys.path.insert(0, str(DEPS))
    from isaaclab.app import AppLauncher
    app = AppLauncher({'headless': True, 'device': 'cpu', 'visualizer': ['none']}).app
    env = None
    result = {'schema_version': 1, 'mode': args.mode, 'episodes': [], 'completed': False,
              'observation_source': 'privileged_current_native_state_and_known_map',
              'emergency_shield': False, 'optimizer_steps': 0}
    started = time.monotonic()
    try:
        from traffic.physics_session import prepare_vehicle_runtime
        prepare_vehicle_runtime()
        import numpy as np
        import torch
        import importlib.metadata
        import platform
        result['runtime_versions'] = {'python': platform.python_version(),
                                      'torch': torch.__version__, 'numpy': np.__version__}
        for package in ('isaacsim', 'isaaclab', 'isaaclab-rl', 'rsl-rl-lib', 'tensordict'):
            try:
                result['runtime_versions'][package] = importlib.metadata.version(package)
            except importlib.metadata.PackageNotFoundError:
                result['runtime_versions'][package] = 'distribution metadata unavailable'
        from environments.loop_hazard_lab import LoopHazardLabCfg, LoopHazardLabEnv
        from environments.lane_follow_rsl import signed_vehicle_actions
        from experiments.loop_hazard_support import EpisodeAccumulator
        torch.set_num_threads(4)
        torch.manual_seed(args.seeds[0])
        cfg = LoopHazardLabCfg()
        cfg.scene_directory = str(output / 'scene')
        cfg.sim.log_dir = str(ROOT / 'logs/rsl_integration' / output.name / 'isaac')
        cfg.hazard_stage, cfg.hazard_side = episode_case(args, 0)
        cfg.seed = args.seeds[0]
        cfg.reset_perturb = True
        env = LoopHazardLabEnv(cfg)
        atomic_json(output / 'scene-contract.json', env.scene_contract())
        policy = None
        if args.policy:
            policy = torch.jit.load(str(args.policy), map_location='cpu').eval()
            result['policy_state_before'] = policy_state_hash(policy)
            result['policy_sha256'] = hashlib.sha256(args.policy.read_bytes()).hexdigest()
            result['policy'] = str(args.policy.relative_to(ROOT))
        if args.mode == 'train':
            result.update(train_native(args, output, env))
        else:
            observations, expert_actions = [], []
            with (output / 'steps.jsonl').open('x', encoding='utf-8') as records:
                for index, seed in enumerate(args.seeds):
                    stage, side = episode_case(args, index)
                    env.cfg.hazard_stage, env.cfg.hazard_side = stage, side
                    env.task.stage = stage
                    obs, _ = env.reset(seed=seed)
                    episode = {'seed': seed, 'stage': stage, 'side': side,
                               'reset': deepcopy(env.last_reset), 'steps': 0, 'return': 0.0,
                               'contacts': 0, 'clipped_actions': 0, 'max_lateral_m': 0.0}
                    accumulator = EpisodeAccumulator(env.last_reset, route=env.route, stage=stage)
                    for step in range(2101):
                        if time.monotonic() - started > args.max_seconds - 30:
                            raise TimeoutError('Native operation deadline reached')
                        if policy is not None:
                            with torch.inference_mode():
                                raw = policy(obs['policy'])
                            physical = signed_vehicle_actions(raw)
                            episode['clipped_actions'] += int((raw.abs() > 1).any())
                        else:
                            if args.controller == 'ignore_objects':
                                from environments.lane_following_lab import LaneFollowingLabEnv
                                physical = LaneFollowingLabEnv.scripted_action(env)
                            else:
                                physical = env.scripted_action()
                            raw = torch.stack((physical[:, 0] / .5,
                                               physical[:, 1] - physical[:, 2]), dim=1)
                        if args.mode == 'collect':
                            observations.append(obs['policy'][0].detach().cpu().numpy().copy())
                            expert_actions.append(raw[0].detach().cpu().numpy().copy())
                        obs, reward, terminated, truncated, extras = env.step(physical)
                        tr = extras['lane_transition']
                        accumulator.update(tr)
                        episode['steps'] += 1
                        episode['return'] += float(reward[0])
                        episode['contacts'] += len(tr['contacts'])
                        episode['max_lateral_m'] = max(episode['max_lateral_m'], abs(tr['route_state']['lateral_error_m']))
                        records.write(json.dumps({'seed': seed, 'stage': stage,
                                                  'raw_policy_action': raw[0].tolist(),
                                                  'transition': tr}, allow_nan=False) + '\n')
                        if bool(terminated[0] or truncated[0]):
                            episode.update(success=bool(tr['success']), reason=tr['reason'],
                                           progress_m=tr['route_state']['progress_m'],
                                           final_transition=tr)
                            episode['independent_assessment'] = accumulator.finish(complete=True)
                            result['episodes'].append(episode)
                            records.flush()
                            atomic_json(output / 'result.json', result)
                            print(f'EPISODE seed={seed} stage={stage} success={tr["success"]} '
                                  f'reason={tr["reason"]} progress={episode["progress_m"]:.2f}', flush=True)
                            break
                    else:
                        raise RuntimeError('Episode exceeded its declared native tick limit')
            if args.mode == 'collect':
                np.savez_compressed(output / 'demonstrations.npz',
                                    observations=np.asarray(observations, dtype=np.float32),
                                    actions=np.asarray(expert_actions, dtype=np.float32))
                result['demonstration_steps'] = len(observations)
                result['demonstrations_qualified'] = all(ep['independent_assessment']['driving_success'] for ep in result['episodes'])
        result['completed'] = True
        if policy is not None:
            result['policy_state_after'] = policy_state_hash(policy)
            if result['policy_state_after'] != result['policy_state_before']:
                raise RuntimeError('Frozen policy parameters or normalization changed')
        result['scene_final'] = env.scene_contract()
    except BaseException as exc:
        result['error'] = f'{type(exc).__name__}: {exc}'
        result['traceback'] = traceback.format_exc()
        raise
    finally:
        try:
            if env is not None:
                env.close()
                result['contact_cleanup'] = env.contact_reporting.get('cleanup')
                result['viewport_cleanup'] = env.viewport_cleanup
        finally:
            result['wall_seconds'] = time.monotonic() - started
            atomic_json(output / 'result.json', result)
            app.close()


def train_native(args, output, env):
    from experiments.loop_hazard_training import train_native as train
    return train(args, output, env)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['probe', 'collect', 'train', 'evaluate'])
    parser.add_argument('--stage', choices=['clear', 'static', 'crossing', 'mixed'], default='mixed')
    parser.add_argument('--side', choices=['left', 'right'])
    parser.add_argument('--seeds', type=int, nargs='+', default=[31001, 31002, 31003, 31004])
    parser.add_argument('--policy', type=Path)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--demonstrations', type=Path)
    parser.add_argument('--demonstration-audit', type=Path)
    parser.add_argument('--controller', choices=['scripted', 'ignore_objects'], default='scripted')
    parser.add_argument('--max-seconds', type=int, default=600)
    parser.add_argument('--native', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--output', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if Path.cwd().resolve() != ROOT:
        parser.error('Run from the canonical checkout root')
    if not 60 <= args.max_seconds <= 14400 or not 1 <= len(args.seeds) <= 200:
        parser.error('Bounded time and seed count required')
    if any(seed < 0 or seed >= 2**32 for seed in args.seeds) or len(set(args.seeds)) != len(args.seeds):
        parser.error('Unique unsigned32 seeds required')
    for name in ('policy', 'config', 'demonstrations', 'demonstration_audit'):
        path = getattr(args, name)
        if path is not None:
            path = path.resolve()
            if not path.is_relative_to(ROOT) or not path.is_file():
                parser.error(f'{name} must be an existing project file')
            setattr(args, name, path)
    if args.mode == 'evaluate' and args.policy is None:
        parser.error('Evaluation requires a frozen policy')
    if args.mode == 'train' and args.config is None:
        parser.error('Training requires a versioned configuration')
    if args.controller != 'scripted' and args.mode != 'probe':
        parser.error('The unsafe contact diagnostic is restricted to probes')
    if args.native:
        output = args.output.resolve()
        if output.parent != ROOT / 'outputs/rsl_integration' or not output.is_dir():
            parser.error('Invalid native run directory')
        manifest = json.loads((output / 'manifest.json').read_text(encoding='utf-8'))
        if manifest['args'] != resolved_arguments(args) or manifest['source_sha256'] != source_hashes():
            raise ValueError('Native arguments or source identity changed before launch')
        for relative, digest in manifest['input_sha256'].items():
            if hashlib.sha256((ROOT / relative).read_bytes()).hexdigest() != digest:
                raise ValueError('Native input identity changed: ' + relative)
        with (output / 'started').open('x', encoding='utf-8') as stream:
            stream.write(datetime.now(timezone.utc).isoformat())
        native(args, output)
        return 0
    from experiments.train_lane_follow_rsl import dependency_check
    from experiments.verify_lane_following_lab import kit_process_snapshot, classify_runtime_log, KNOWN_STARTUP_ERRORS
    from experiments.verify_physics_vehicle import stop_child
    from experiments.probe_support import GpuMonitor, gpu_sample
    from hivemind.launcher import isaac_runtime, load_config
    dependency_check()
    before = kit_process_snapshot()
    if not before['available'] or before['count']:
        raise RuntimeError('Native jobs must run serially; an existing Kit process is active')
    gpu_before = gpu_sample()
    if gpu_before['used_mib'] / gpu_before['total_mib'] >= .9:
        raise RuntimeError('Whole-GPU memory preflight limit reached')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    output = ROOT / 'outputs/rsl_integration' / f'v01-{args.mode}-{stamp}-{uuid.uuid4().hex[:8]}'
    output.mkdir(parents=True, exist_ok=False)
    logs = ROOT / 'logs/rsl_integration' / output.name
    logs.mkdir(parents=True, exist_ok=False)
    hashes = source_hashes()
    for relative in hashes:
        destination = output / 'sources' / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / relative, destination)
    inputs = {getattr(args, name).relative_to(ROOT).as_posix(): hashlib.sha256(getattr(args, name).read_bytes()).hexdigest()
              for name in ('policy', 'config', 'demonstrations', 'demonstration_audit') if getattr(args, name) is not None}
    atomic_json(output / 'manifest.json', {'args': resolved_arguments(args), 'input_sha256': inputs,
                'source_sha256': hashes, 'git_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                'git_status': subprocess.check_output(['git', 'status', '--short'], cwd=ROOT, text=True),
                'created_utc': datetime.now(timezone.utc).isoformat()})
    runtime = isaac_runtime(ROOT, load_config(ROOT))
    command = [str(runtime), str(Path(__file__).resolve()), *sys.argv[1:], '--native', '--output', str(output)]
    environment = dict(os.environ)
    environment['PYTHONPATH'] = str(DEPS) + os.pathsep + environment.get('PYTHONPATH', '')
    print('RUN_DIRECTORY=' + str(output), flush=True)
    started = time.monotonic()
    with (logs / 'stdout.log').open('w', encoding='utf-8') as stdout, (logs / 'stderr.log').open('w', encoding='utf-8') as stderr, GpuMonitor() as monitor:
        process = subprocess.Popen(command, cwd=ROOT, env=environment, stdout=stdout, stderr=stderr,
                                   creationflags=subprocess.CREATE_NO_WINDOW)
        timed_out = False
        stop_reason = None
        try:
            while process.poll() is None:
                if time.monotonic() - started > args.max_seconds:
                    timed_out = True
                    stop_reason = 'wall_time_limit'
                elif monitor.errors:
                    stop_reason = 'gpu_monitor_unavailable'
                elif monitor.rows and monitor.rows[-1]['used_mib'] / monitor.rows[-1]['total_mib'] >= .9:
                    stop_reason = 'gpu_memory_limit'
                if stop_reason:
                    stop_child(process)
                    break
                time.sleep(2)
        finally:
            if process.poll() is None:
                stop_child(process)
    after = kit_process_snapshot()
    sources_unchanged = source_hashes() == hashes
    native_result = json.loads((output / 'result.json').read_text(encoding='utf-8')) if (output / 'result.json').exists() else {}
    runtime = classify_runtime_log((logs / 'stdout.log').read_text(encoding='utf-8', errors='replace') + '\n'
                                   + (logs / 'stderr.log').read_text(encoding='utf-8', errors='replace'), KNOWN_STARTUP_ERRORS)
    passed = bool(process.returncode == 0 and not stop_reason and sources_unchanged
                  and after['available'] and after['count'] == 0 and monitor.rows and not monitor.errors
                  and native_result.get('completed') and not native_result.get('error')
                  and native_result.get('contact_cleanup', {}).get('passed')
                  and not runtime['unexpected_runtime_errors'])
    atomic_json(output / 'gpu-samples.json', monitor.rows)
    atomic_json(output / 'exit.json', {'returncode': process.poll(), 'timed_out': timed_out,
                'wall_seconds': time.monotonic() - started, 'kit_after': after,
                'sources_unchanged': sources_unchanged, 'passed': passed, 'stop_reason': stop_reason,
                'runtime_log_assessment': runtime,
                'output_sha256': {p.relative_to(output).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                                  for p in output.rglob('*') if p.is_file() and p.name != 'exit.json'}})
    print(f'EXIT={process.poll()} OUTPUT={output}', flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
