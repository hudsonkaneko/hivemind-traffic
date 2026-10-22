"""Bounded frozen-JIT evaluation of the one-car, 22-observation RSL policy.

This evaluates trusted local TorchScript exports, not arbitrary downloaded code.
The supervisor launches one CPU-physics process per route. A valid recorded
failure is an evaluation result, not successful driving. No training is run.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
from numbers import Real
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback
import uuid

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from experiments.probe_support import GpuMonitor, gpu_sample
from experiments.verify_lane_following_lab import (
    EXPECTED_ACCEPTANCE, KNOWN_STARTUP_ERRORS, ROUTES, classify_runtime_log,
    kit_process_snapshot, source_files as lane_sources,
)
from experiments.verify_physics_vehicle import stop_child

DEFAULT_SEEDS = tuple(range(5101, 5106))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def atomic_json(path, value):
    """Publish complete JSON only; failed serialization leaves the old file intact."""
    data = json.dumps(value, indent=2, allow_nan=False) + '\n'
    path = Path(path)
    temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    try:
        with temporary.open('x', encoding='utf-8', newline='\n') as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def source_files():
    return sorted(set(lane_sources() + [
        'experiments/evaluate_lane_follow_rsl.py',
        'experiments/lane_follow_ppo_evaluation.py',
    ]))


def resolve_job(policy, routes=ROUTES, seeds=DEFAULT_SEEDS, max_episodes=15,
                max_process_seconds=600):
    path = Path(policy)
    if not path.is_absolute():
        path = ROOT / path
    if path.is_symlink():
        raise ValueError('Policy must be a regular local file')
    path = path.resolve()
    if not path.is_relative_to(ROOT) or not path.is_file() or path.suffix != '.pt':
        raise ValueError('Select a trusted project-local TorchScript .pt policy')
    if not 0 < path.stat().st_size <= 100 * 1024 * 1024:
        raise ValueError('Policy size is outside the bounded export contract')
    routes, seeds = list(routes), list(seeds)
    if not routes or any(route not in ROUTES for route in routes) or len(set(routes)) != len(routes):
        raise ValueError('Choose distinct known routes')
    if (not seeds or any(type(seed) is not int or not 0 <= seed < 2**32 for seed in seeds)
            or len(set(seeds)) != len(seeds)):
        raise ValueError('Choose distinct unsigned 32-bit seeds')
    if type(max_episodes) is not int or not 1 <= max_episodes <= 15 or len(routes)*len(seeds) > max_episodes:
        raise ValueError('The complete selected matrix must fit max-episodes in 1..15')
    if type(max_process_seconds) is not int or not 30 <= max_process_seconds <= 600:
        raise ValueError('Per-route process limit must be an integer in 30..600 seconds')
    return dict(schema_version=1, policy_path=path.relative_to(ROOT).as_posix(),
        policy_sha256=digest(path), routes=routes, seeds=seeds, max_episodes=max_episodes,
        max_process_seconds=max_process_seconds, acceptance=deepcopy(EXPECTED_ACCEPTANCE),
        gpu_memory_fraction_stop=.9, training=False, physics_device='cpu',
        observations=22, actions='normalized_steering_signed_longitudinal',
        reset_perturb=True, normalization='frozen inside TorchScript export')


def mapped_actions(values):
    """Return equivalent unclipped 3-axis request and the actual clipped control."""
    if (not isinstance(values, (list, tuple)) or len(values) != 2
            or any(not isinstance(v, Real) or isinstance(v, bool) or not math.isfinite(v) for v in values)):
        raise ValueError('Policy must return two finite signed actions')
    steer, longitudinal = map(float, values)
    raw = [steer*.5, max(longitudinal, 0.), max(-longitudinal, 0.)]
    clipped = [max(-.5, min(.5, raw[0])), min(1., raw[1]), min(1., raw[2])]
    return raw, clipped


def capture_transition(env, returned, signed_action, raw_action, applied_action, wall_seconds):
    """Detach the pre-reset terminal record from the next episode observation."""
    observation, reward, done, infos = returned
    extras = env.extras
    row = deepcopy(extras['lane_transition'])
    row.update(returned_observation=observation[0].tolist(), returned_reward=float(reward[0]),
        returned_terminated=bool(extras['final_terminated'][0]),
        returned_truncated=bool(extras['final_truncated'][0]), returned_done=bool(done[0]),
        final_observation=extras['final_observation']['policy'][0].cpu().tolist(),
        sb3_time_limit_truncated=bool(infos[0]['TimeLimit.truncated']),
        rsl_action_raw=list(signed_action), policy_action_raw=list(raw_action),
        env_action=list(applied_action), step_wall_seconds=wall_seconds)
    if done[0]:
        row['sb3_terminal_observation'] = infos[0]['terminal_observation'].tolist()
        row['auto_reset'] = deepcopy(extras['reset'])
    return row


def assess_case(records, reset, route, seed):
    from environments.lane_following_contract import make_reset_spec
    from experiments.lane_follow_ppo_evaluation import reduce_episode
    outcome = reduce_episode(records, reset, route, EXPECTED_ACCEPTANCE)
    initial = asdict(make_reset_spec(route, seed=seed, episode_index=0, perturb=True))
    following = asdict(make_reset_spec(route, seed=seed, episode_index=1, perturb=True))
    checks = dict(raw_evidence=outcome['evidence_valid'],
        seeded_reset=reset.get('reset_spec') == initial,
        seeded_auto_reset=bool(records) and records[-1].get('auto_reset', {}).get('reset_spec') == following)
    try:
        checks['signed_action_mapping'] = all(mapped_actions(row['rsl_action_raw']) ==
            (row['policy_action_raw'], row['env_action']) for row in records)
    except (ValueError, KeyError, TypeError):
        checks['signed_action_mapping'] = False
    return dict(valid=all(checks.values()), checks=checks, outcome=outcome)


def parameter_digest(policy):
    """Include normalization buffers, tensor names, dtypes and shapes."""
    result = hashlib.sha256()
    tensors = list(policy.named_parameters()) + list(policy.named_buffers())
    for name, value in sorted(tensors, key=lambda row: row[0]):
        tensor = value.detach().cpu().contiguous()
        result.update(json.dumps([name, str(tensor.dtype), list(tensor.shape)]).encode())
        result.update(tensor.numpy().tobytes())
    return result.hexdigest()


def verify_sources(manifest):
    hashes = manifest.get('source_hashes', {})
    if set(hashes) != set(source_files()):
        raise ValueError('Evaluator source inventory differs from the captured inventory')
    changed = [name for name, sha in hashes.items() if not (ROOT/name).is_file() or digest(ROOT/name) != sha]
    if changed:
        raise ValueError('Captured evaluator sources changed: ' + ', '.join(changed))


def verify_worker(output):
    output = Path(output).resolve()
    if not output.is_relative_to(ROOT/'outputs/lane_follow_rsl_eval') or not output.is_dir():
        raise ValueError('Worker requires a supervisor-created evaluation package')
    package = output.parent
    manifest = json.loads((package/'manifest.json').read_text(encoding='utf-8'))
    job = json.loads((package/'resolved-config.json').read_text(encoding='utf-8'))
    reconstructed = resolve_job(job['policy_path'], job['routes'], job['seeds'],
                                job['max_episodes'], job['max_process_seconds'])
    if manifest.get('status') != 'running' or manifest.get('config') != job or job != reconstructed:
        raise ValueError('Worker configuration differs from its running supervisor')
    if output.name not in job['routes'] or digest(package/'policy.pt') != job['policy_sha256']:
        raise ValueError('Worker route or captured policy differs')
    verify_sources(manifest)
    with (output/'worker-started.json').open('x', encoding='utf-8') as stream:
        json.dump(dict(pid=os.getpid(), started_utc=datetime.now(timezone.utc).isoformat()), stream)
    return job, manifest


def close_application(app, output, result):
    """Kit may terminate Python inside close; publish completed env evidence first.

    The supervisor supplies the actual process exit/isolation verdict. A saved
    close request is not a claim that shutdown returned successfully.
    """
    result['application_close_requested'] = app is not None
    result['application_close_returned'] = False
    atomic_json(Path(output)/'native-result.json', result)
    if app is not None:
        try:
            app.close()
            result['application_close_returned'] = True
        except Exception as error:
            result.update(passed=False, app_close_error=repr(error))
        atomic_json(Path(output)/'native-result.json', result)


def native_worker(output):
    """Lazy native entry point; the project supervisor alone should call this."""
    output = Path(output).resolve()
    job, manifest = verify_worker(output)  # Before any TorchScript deserialization or Kit startup.
    result = dict(passed=False, training_started=False, optimizer_steps=0, route_id=output.name, cases=[])
    app = env = raw = policy = None
    started = time.monotonic()
    try:
        from isaaclab.app import AppLauncher
        app = AppLauncher(dict(headless=True, device='cpu', visualizer=['none'])).app
        from traffic.physics_session import prepare_vehicle_runtime
        result['runtime_setup'] = prepare_vehicle_runtime()
        import isaaclab
        import numpy as np
        import torch
        from environments.lane_following_lab import LaneFollowingLabCfg, LaneFollowingLabEnv
        from environments.lane_follow_sb3 import make_lane_follow_sb3_wrapper
        from scripts.probe_vehicle_lab import lab_runtime_provenance
        torch.set_num_threads(4)
        result['runtime'] = lab_runtime_provenance(isaaclab, torch)
        policy = torch.jit.load(str(output.parent/'policy.pt'), map_location='cpu')
        policy.eval()
        for parameter in policy.parameters():
            parameter.requires_grad_(False)
        before = parameter_digest(policy)
        result['policy_state_before'] = before
        cfg = LaneFollowingLabCfg()
        cfg.route_id, cfg.seed, cfg.reset_perturb = output.name, job['seeds'][0], True
        cfg.routes_file = str(ROOT/'scenarios/physics-road/routes.json')
        cfg.scene_directory, cfg.sim.log_dir = str(output/'scene'), str(output/'lab-logs')
        cfg.render_during_step = False
        env = LaneFollowingLabEnv(cfg)
        raw = make_lane_follow_sb3_wrapper(env)
        initial = env.scene_contract()
        atomic_json(output/'scene-initial.json', initial)
        if not initial['passed'] or not initial['static_hashes_unchanged']:
            raise RuntimeError('Initial composed scene contract failed')
        print('LAB_LANE_TASK_READY', flush=True)
        for seed in job['seeds']:
            observation, extras = env.reset(seed=seed, options={'perturb': True})
            reset = deepcopy(extras['reset'])
            atomic_json(output/f'seed-{seed}-reset.json', reset)
            records = []
            # Append each completed boundary immediately, preserving partial failed cases.
            with (output/f'seed-{seed}-trajectory.jsonl').open('x', encoding='utf-8', newline='\n') as stream:
                for _ in range(2100):
                    if time.monotonic()-started > job['max_process_seconds']-10:
                        raise TimeoutError('Native evaluation deadline reached')
                    inputs = observation['policy'] if isinstance(observation, dict) else torch.as_tensor(observation)
                    if tuple(inputs.shape) != (1, 22) or not bool(torch.isfinite(inputs).all()):
                        raise RuntimeError('Malformed policy observation')
                    with torch.no_grad():
                        signed = policy(inputs.to(device='cpu', dtype=torch.float32))
                    if tuple(signed.shape) != (1, 2) or not bool(torch.isfinite(signed).all()):
                        raise RuntimeError('Malformed signed policy action')
                    signed_values = signed[0].cpu().tolist()
                    requested, supplied = mapped_actions(signed_values)
                    tick_started = time.perf_counter()
                    returned = raw.step(np.asarray([supplied], dtype=np.float32))
                    row = capture_transition(env, returned, signed_values, requested, supplied,
                                             time.perf_counter()-tick_started)
                    records.append(row)
                    stream.write(json.dumps(row, allow_nan=False)+'\n')
                    stream.flush()
                    observation = returned[0]
                    if returned[2][0]:
                        break
            assessment = assess_case(records, reset, env.route, seed)
            case = dict(seed=seed, route_id=output.name, steps=len(records), assessment=assessment)
            result['cases'].append(case)
            atomic_json(output/'cases.json', result['cases'])
            print('RSL_EVALUATION_CASE='+json.dumps(case), flush=True)
            if not assessment['valid']:
                raise RuntimeError('Independent raw episode evidence check failed')
        result['scene_contract'] = env.scene_contract()
        atomic_json(output/'scene-final.json', result['scene_contract'])
        result.update(policy_state_before=before, policy_state_after=parameter_digest(policy))
        result['frozen_inference'] = result['policy_state_before'] == result['policy_state_after']
        verify_sources(manifest)
        result['passed'] = bool(result['frozen_inference'] and result['scene_contract']['passed']
            and result['scene_contract']['static_hashes_unchanged'] and len(result['cases']) == len(job['seeds']))
    except Exception as error:
        result.update(passed=False, error=repr(error), traceback=traceback.format_exc())
        traceback.print_exc()
    finally:
        if policy is not None and 'policy_state_before' in result:
            try:
                result['policy_state_after'] = parameter_digest(policy)
                result['frozen_inference'] = result['policy_state_before'] == result['policy_state_after']
                result['passed'] = bool(result['passed'] and result['frozen_inference'])
            except Exception as error:
                result.update(passed=False, frozen_state_error=repr(error))
        raw = policy = None
        if env is not None:
            try:
                env.close()
                result.update(viewport_cleanup=deepcopy(env.viewport_cleanup),
                              contact_reporting=deepcopy(env.contact_reporting))
                result['passed'] = bool(result['passed'] and env.viewport_cleanup['passed']
                    and env.contact_reporting['cleanup']['passed'])
            except Exception as error:
                result.update(passed=False, close_error=repr(error))
        result['wall_seconds'] = time.monotonic()-started
        close_application(app, output, result)
    return 0 if result['passed'] else 1


def supervise(job):
    from hivemind.launcher import isaac_runtime, load_config
    runtime = isaac_runtime(ROOT, load_config(ROOT))
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:8]
    output, log_directory = ROOT/'outputs/lane_follow_rsl_eval'/stamp, ROOT/'logs/lane_follow_rsl_eval'/stamp
    output.mkdir(parents=True, exist_ok=False)
    log_directory.mkdir(parents=True, exist_ok=False)
    hashes = {name: digest(ROOT/name) for name in source_files()}
    for name in hashes:
        target = output/'sources'/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT/name, target)
    shutil.copyfile(ROOT/job['policy_path'], output/'policy.pt')
    manifest = dict(status='running', config=job, started_utc=stamp, source_hashes=hashes,
        git_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        runtime=str(runtime), log_directory=log_directory.relative_to(ROOT).as_posix())
    atomic_json(output/'manifest.json', manifest)
    atomic_json(output/'resolved-config.json', job)
    results, error = [], None
    try:
        for route in job['routes']:
            verify_sources(manifest)
            before, gpu = kit_process_snapshot(), gpu_sample()
            if not before['available'] or before['count'] != 0:
                raise RuntimeError('Kit isolation unavailable or another Kit process is active')
            if gpu['used_mib']/gpu['total_mib'] >= .9:
                raise RuntimeError('Preflight whole-GPU memory resource stop')
            folder = output/route
            folder.mkdir()
            command = [str(runtime), str(Path(__file__).resolve()), '--native', '--output', str(folder)]
            atomic_json(folder/'launch.json', dict(command=command, kit_before=before, gpu_before=gpu))
            child, stop_reason, started = None, None, time.monotonic()
            log = log_directory/(route+'.log')
            try:
                with log.open('w', encoding='utf-8') as stream, GpuMonitor() as monitor:
                    child = subprocess.Popen(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT,
                        creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0)
                    while child.poll() is None:
                        if time.monotonic()-started > job['max_process_seconds']:
                            stop_reason = 'Process deadline exceeded'
                        elif monitor.errors:
                            stop_reason = 'GPU monitor unavailable'
                        elif monitor.rows and monitor.rows[-1]['used_mib']/monitor.rows[-1]['total_mib'] >= .9:
                            stop_reason = 'Whole-GPU memory resource stop'
                        if stop_reason:
                            stop_child(child)
                            break
                        time.sleep(.2)
            finally:
                if child is not None and child.poll() is None:
                    stop_child(child)
            after = kit_process_snapshot()
            native = folder/'native-result.json'
            result = json.loads(native.read_text(encoding='utf-8')) if native.is_file() else {'passed': False, 'error': 'Missing native result'}
            result.update(exit_code=child.returncode, stop_reason=stop_reason, kit_after=after,
                gpu_monitor_errors=monitor.errors, gpu_sample_count=len(monitor.rows),
                **classify_runtime_log(log.read_text(encoding='utf-8', errors='replace'), KNOWN_STARTUP_ERRORS))
            result['passed'] = bool(result.get('passed') and child.returncode == 0 and not stop_reason
                and after['available'] and after['count'] == 0 and monitor.rows and not monitor.errors
                and not result['unexpected_runtime_errors'] and not result['usd_reference_warning_count'])
            atomic_json(folder/'gpu-samples.json', monitor.rows)
            atomic_json(folder/'summary.json', result)
            results.append(dict(result, route_id=route))
            atomic_json(output/'progress.json', results)
            verify_sources(manifest)
            if not result['passed']:
                raise RuntimeError('Route execution/evidence failed; further launches stopped')
    except Exception as exc:
        error = repr(exc)
    cases = [case for result in results for case in result.get('cases', [])]
    summary = dict(passed=error is None and len(results) == len(job['routes']), error=error,
        training_started=False, optimizer_steps=0, route_results=results, episodes=len(cases),
        driving_successes=sum(case['assessment']['outcome']['driving_success'] for case in cases),
        pass_scope='Execution and independent evidence validity; driving successes are reported separately')
    summary['kit_after'] = kit_process_snapshot()
    summary['passed'] = bool(summary['passed'] and summary['kit_after']['available']
                           and summary['kit_after']['count'] == 0)
    try:
        verify_sources(manifest)
        if digest(ROOT/job['policy_path']) != job['policy_sha256'] or digest(output/'policy.pt') != job['policy_sha256']:
            raise ValueError('Policy bytes changed during evaluation')
    except Exception as exc:
        summary.update(passed=False, final_integrity_error=repr(exc))
    atomic_json(output/'summary.json', summary)
    manifest.update(status='passed' if summary['passed'] else 'failed',
        finished_utc=datetime.now(timezone.utc).isoformat(),
        output_hashes={p.relative_to(output).as_posix(): digest(p) for p in output.rglob('*')
                       if p.is_file() and p != output/'manifest.json'},
        log_hashes={p.relative_to(ROOT).as_posix(): digest(p) for p in log_directory.glob('*.log')})
    atomic_json(output/'manifest.json', manifest)
    print('LANE_FOLLOW_RSL_EVALUATION='+str(output), flush=True)
    print(json.dumps(summary, indent=2), flush=True)
    return 0 if summary['passed'] else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy', type=Path)
    parser.add_argument('--routes', nargs='+', choices=ROUTES, default=list(ROUTES))
    parser.add_argument('--seeds', nargs='+', type=int, default=list(DEFAULT_SEEDS))
    parser.add_argument('--max-episodes', type=int, default=15)
    parser.add_argument('--max-process-seconds', type=int, default=600)
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--native', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--output', type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if Path.cwd().resolve() != ROOT:
        parser.error('Run from the canonical project root')
    if args.native:
        if args.output is None:
            parser.error('Native workers require an output package')
        return native_worker(args.output)
    if args.policy is None:
        parser.error('--policy is required')
    job = resolve_job(args.policy, args.routes, args.seeds, args.max_episodes, args.max_process_seconds)
    if args.check:
        print(json.dumps(job, indent=2))
        return 0
    return supervise(job)


if __name__ == '__main__':
    raise SystemExit(main())
