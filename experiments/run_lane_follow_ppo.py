"""Supervise the explicitly approved single PPO pilot and frozen evaluations."""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

from experiments.lane_follow_ppo_protocol import (
    APPROVAL_PATH, PILOT_ID, checkpoint_identity, digest, load_protocol, source_files, verify_sources,
)
from experiments.probe_support import ROOT, GpuMonitor, finish, gpu_sample, run_package, write_json
from experiments.verify_lane_following_lab import (
    EXPECTED_ACCEPTANCE, KNOWN_STARTUP_ERRORS, ROUTES, classify_runtime_log, kit_process_snapshot,
)
from experiments.verify_physics_vehicle import stop_child
from hivemind.launcher import isaac_runtime, load_config


def resolve_job(mode, *, route='left_r60', stage=None, controller='ppo', checkpoint=None):
    protocol = load_protocol()
    if mode not in ('smoke', 'train', 'evaluate') or route not in ROUTES:
        raise ValueError('Unknown pilot mode/route')
    if mode != 'evaluate' and (route != 'left_r60' or stage is not None or checkpoint is not None or controller != 'ppo'):
        raise ValueError('Smoke/training use the approved left route without evaluation overrides')
    identity = None
    if mode == 'evaluate':
        if stage not in ('selection', 'heldout') or controller not in ('ppo', 'scripted'):
            raise ValueError('Evaluation requires the declared stage/controller')
        if stage == 'selection' and (route != 'left_r60' or controller != 'ppo'):
            raise ValueError('Checkpoint selection uses PPO on left_r60 only')
        if controller == 'ppo':
            if checkpoint is None:
                raise ValueError('PPO evaluation requires an intact approved checkpoint')
            identity = checkpoint_identity(checkpoint)
        elif checkpoint is not None:
            raise ValueError('Scripted comparison does not load a policy checkpoint')
    limit = protocol['authorization'][{
        'smoke': 'smoke_process_limit_s', 'train': 'training_process_limit_s',
        'evaluate': 'evaluation_process_limit_s'}[mode]]
    return {'schema_version': 1, 'pilot_id': PILOT_ID, 'protocol': protocol,
        'mode': mode, 'route_id': route, 'stage': stage, 'controller': controller,
        'checkpoint_path': None if checkpoint is None else str(Path(checkpoint).resolve().relative_to(ROOT)),
        'checkpoint_identity': identity,
        'seeds': ([3101, 3102, 3103] if stage == 'selection' else [4101, 4102, 4103, 4104, 4105])
                 if mode == 'evaluate' else [2101],
        'max_process_seconds': limit, 'acceptance': dict(EXPECTED_ACCEPTANCE),
        'known_baseline_startup_errors': list(KNOWN_STARTUP_ERRORS),
        'gpu_memory_fraction_stop': .9, 'visible': False}


def verify_smoke(directory):
    directory = Path(directory).resolve()
    if not directory.is_relative_to(ROOT / 'outputs/lab_lane_ppo'):
        raise ValueError('Smoke evidence must belong to this pilot workspace')
    manifest = json.loads((directory / 'manifest.json').read_text(encoding='utf-8'))
    summary = json.loads((directory / 'summary.json').read_text(encoding='utf-8'))
    if (manifest['status'] != 'passed' or summary.get('passed') is not True
            or summary.get('mode') != 'smoke' or summary.get('optimizer_steps') != 0
            or summary.get('training_started') is not False):
        raise ValueError('A passing zero-optimizer native smoke is required')
    if set(manifest['source_hashes']) != set(source_files()):
        raise ValueError('Smoke source inventory differs from the training runner')
    verify_sources(manifest)
    actual = {p.relative_to(directory).as_posix() for p in directory.rglob('*')
              if p.is_file() and p.name != 'manifest.json'}
    inventory = {name.replace('\\', '/'): value for name, value in manifest['output_hashes'].items()}
    if actual != set(inventory):
        raise ValueError('Smoke package inventory changed')
    for name, expected in inventory.items():
        path = (directory / name).resolve()
        if not path.is_relative_to(directory) or digest(path) != expected:
            raise ValueError('Smoke artifact hash mismatch')
    return {'run': directory.relative_to(ROOT).as_posix(), 'manifest_sha256': digest(directory / 'manifest.json')}


def claim_training(output, smoke):
    """Atomic one-run claim survives failures; this command never resumes/reseeds."""
    claim = ROOT / 'outputs/lab_lane_ppo' / (PILOT_ID + '.claim.json')
    with claim.open('x', encoding='utf-8') as stream:
        json.dump({'pilot_id': PILOT_ID, 'output': output.relative_to(ROOT).as_posix(),
                   'smoke': smoke, 'approval_sha256': digest(ROOT / APPROVAL_PATH)}, stream, indent=2)
    return claim.relative_to(ROOT).as_posix()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('smoke', 'train', 'evaluate'), required=True)
    parser.add_argument('--route', choices=ROUTES, default='left_r60')
    parser.add_argument('--stage', choices=('selection', 'heldout'))
    parser.add_argument('--controller', choices=('ppo', 'scripted'), default='ppo')
    parser.add_argument('--checkpoint', type=Path)
    parser.add_argument('--smoke-evidence', type=Path)
    parser.add_argument('--approved-config-sha256')
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    config = resolve_job(args.mode, route=args.route, stage=args.stage,
                         controller=args.controller, checkpoint=args.checkpoint)
    if args.mode == 'train' and not args.check:
        if args.approved_config_sha256 != digest(ROOT / APPROVAL_PATH) or args.smoke_evidence is None:
            parser.error('Training needs the exact approval hash and passing smoke evidence')
        config['smoke_evidence'] = verify_smoke(args.smoke_evidence)
    elif args.smoke_evidence is not None or args.approved_config_sha256 is not None:
        parser.error('Training approval arguments apply only to an actual train launch')
    sources = source_files()
    if missing := [p for p in sources if not (ROOT / p).is_file()]:
        raise FileNotFoundError(missing)
    runtime = isaac_runtime(ROOT, load_config(ROOT))
    if args.check:
        print(json.dumps({'runtime': str(runtime), 'config': config,
                          'approval_sha256': digest(ROOT / APPROVAL_PATH), 'launches_runtime': False}, indent=2))
        return 0
    output, manifest = run_package('lab_lane_ppo', config, sources)
    logs = ROOT / 'logs/lab_lane_ppo' / output.name
    logs.mkdir(parents=True, exist_ok=False)
    runtime_log = logs / 'runtime.log'
    command = [str(runtime), str(ROOT / 'scripts/run_lane_follow_ppo.py'), '--output', str(output)]
    started, child, reason = time.perf_counter(), None, None
    result = {'passed': False, 'mode': args.mode, 'training_started': False, 'optimizer_steps': 0}
    try:
        manifest.update(command=command, runtime_log=runtime_log.relative_to(ROOT).as_posix(),
            isaac_version=(runtime.parent / 'VERSION').read_text(encoding='utf-8').strip(),
            kit_before=kit_process_snapshot(), gpu_before=gpu_sample())
        if not manifest['kit_before']['available'] or manifest['kit_before']['count'] != 0:
            raise RuntimeError('Another Kit process is active or inventory is unavailable')
        if manifest['gpu_before']['used_mib'] / manifest['gpu_before']['total_mib'] >= .9:
            raise RuntimeError('Preflight GPU memory stop')
        if shutil.disk_usage(ROOT).free < 5 * 1024**3:
            raise RuntimeError('At least5GiB free is required for bounded pilot evidence')
        if args.mode == 'train':
            manifest['training_claim'] = claim_training(output, config['smoke_evidence'])
        write_json(output / 'manifest.json', manifest)
        with runtime_log.open('wb') as log, GpuMonitor() as monitor:
            child = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                     creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0)
            stop_started = None
            while child.poll() is None:
                elapsed = time.perf_counter() - started
                if elapsed > config['max_process_seconds']:
                    reason = 'Process deadline reached'
                elif monitor.errors:
                    reason = 'GPU monitor failed'
                elif monitor.rows and monitor.rows[-1]['used_mib'] / monitor.rows[-1]['total_mib'] >= .9:
                    reason = 'Whole-GPU memory stop'
                if reason:
                    if stop_started is None:
                        (output / 'STOP').write_text(reason, encoding='utf-8')
                        stop_started = time.perf_counter()
                    if elapsed >= config['max_process_seconds'] or time.perf_counter()-stop_started > 10:
                        stop_child(child)
                        break
                time.sleep(.25)
        if (output / 'native-result.json').is_file():
            result = json.loads((output / 'native-result.json').read_text(encoding='utf-8'))
        else:
            result['error'] = 'No final native result; retained progress is authoritative for partial counts'
            if (output / 'training-progress.json').is_file():
                result['partial_training_progress'] = json.loads((output / 'training-progress.json').read_text(encoding='utf-8'))
                result['optimizer_steps'] = result['partial_training_progress'].get('optimizer_steps')
                result['training_started'] = result['partial_training_progress'].get('control_steps', 0) > 0
            elif args.mode == 'train' and child.returncode != 0:
                result.update(optimizer_steps=None, training_started=None)
        shutil.copyfile(runtime_log, output / 'runtime.log')
        result.update(exit_code=child.returncode, stop_reason=reason,
            wall_process_seconds=time.perf_counter()-started,
            gpu_peak_mib=max((r['used_mib'] for r in monitor.rows), default=None),
            gpu_monitor_errors=monitor.errors,
            **classify_runtime_log(runtime_log.read_text(encoding='utf-8', errors='replace'), KNOWN_STARTUP_ERRORS))
        write_json(output / 'gpu-samples.json', monitor.rows)
        result['passed'] = bool(result.get('passed') and child.returncode == 0 and not reason
            and monitor.rows and not monitor.errors and not result['unexpected_runtime_errors']
            and not result['usd_reference_warning_count'])
    except Exception as error:
        result.update(passed=False, supervisor_error=repr(error))
    finally:
        if child is not None and child.poll() is None:
            stop_child(child)
        result['kit_after'] = kit_process_snapshot()
        try:
            verify_sources(manifest)
            result['source_unchanged'] = True
        except ValueError as error:
            result.update(source_unchanged=False, source_error=str(error))
        result['passed'] = bool(result.get('passed') and result['source_unchanged']
            and result['kit_after']['available'] and result['kit_after']['count'] == 0)
        finish(output, manifest, result)
    print('LAB_LANE_PPO_RUN=' + str(output), flush=True)
    print(json.dumps(result, indent=2), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
