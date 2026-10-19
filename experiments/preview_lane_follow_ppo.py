"""Bounded illustrative replay of the strict report's already selected policy.

This is neither checkpoint selection nor an additional held-out trial. The
route, seed, captures and resource bounds are fixed; there is no training mode.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

from experiments.lane_follow_ppo_protocol import checkpoint_identity, digest, source_files, verify_sources
from experiments.probe_support import ROOT, GpuMonitor, finish, gpu_sample, run_package, write_json
from experiments.report_lane_follow_ppo import _read, assemble_held_out
from experiments.verify_lane_following_lab import KNOWN_STARTUP_ERRORS, classify_runtime_log, kit_process_snapshot
from experiments.verify_physics_vehicle import stop_child
from hivemind.launcher import isaac_runtime, load_config

STUDY = 'lab_lane_ppo_preview'
CAPTURE_SECONDS = [0, 2, 5, 10, 15, 20, 25, 30, 35]


def preview_sources():
    return source_files() + ['experiments/preview_lane_follow_ppo.py',
        'scripts/preview_lane_follow_ppo.py', 'experiments/report_lane_follow_ppo.py']


def resolve_preview(training, heldout_report):
    """Supervisor-only exact reduction in the project's report-writing Python."""
    training, report_path = Path(training).resolve(), Path(heldout_report).resolve()
    if not training.is_relative_to(ROOT / 'outputs/lab_lane_ppo') or not report_path.is_relative_to(ROOT / 'outputs/lab_lane_ppo'):
        raise ValueError('Preview inputs must be completed pilot evidence in this checkout')
    saved = _read(report_path)
    if saved.get('kind') != 'heldout':
        raise ValueError('Preview requires the completed strict held-out report')
    recomputed = assemble_held_out(training, ROOT / saved['selection']['path'],
                                  [ROOT / p['path'] for p in saved['packages']])
    if saved != recomputed:
        raise ValueError('Held-out report no longer reproduces exactly')
    return _preview_config(training, report_path, saved)


def _preview_config(training, report_path, saved):
    """Build fixed inputs from exact, hash-bound report data; no reductions."""
    selected = saved['report']['selected_checkpoint']
    selection_path = (ROOT / saved['selection']['path']).resolve()
    if not selection_path.is_relative_to(ROOT / 'outputs/lab_lane_ppo'):
        raise ValueError('Selection report escaped pilot outputs')
    selection = _read(selection_path)
    if (saved['report'].get('valid') is not True or selection.get('kind') != 'selection'
            or selection['report'].get('valid') is not True
            or selection['report']['selected_checkpoint'] != selected
            or digest(selection_path) != saved['selection']['sha256']
            or selection['training'] != saved['training']
            or saved['training']['path'] != training.relative_to(ROOT).as_posix()
            or len(saved['packages']) != 6 or len(selection['packages']) != 4):
        raise ValueError('Completed selection/held-out provenance differs')
    frozen_inputs = {report_path.relative_to(ROOT).as_posix(): digest(report_path),
                     selection_path.relative_to(ROOT).as_posix(): digest(selection_path)}
    references = [saved['training'], *selection['packages'], *saved['packages']]
    if len({row['path'] for row in references}) != 11:
        raise ValueError('Pilot evidence packages must remain distinct')
    for row in references:
        path = (ROOT / row['path'] / 'manifest.json').resolve()
        if not path.is_relative_to(ROOT / 'outputs/lab_lane_ppo') or digest(path) != row['manifest_sha256']:
            raise ValueError('Frozen pilot package manifest differs')
        frozen_inputs[path.relative_to(ROOT).as_posix()] = row['manifest_sha256']
    checkpoint = training / 'checkpoints' / f"update-{selected['update']:02d}"
    if checkpoint_identity(checkpoint) != selected:
        raise ValueError('Selected policy identity differs from completed training')
    references = []
    for package in saved['packages']:
        directory = ROOT / package['path']
        config = _read(directory / 'resolved-config.json')
        if config['controller'] == 'ppo' and config['route_id'] == 'left_r60':
            references.append(directory)
    if len(references) != 1:
        raise ValueError('Exactly one selected-policy left-route reference is required')
    reference = references[0]
    return {
        'schema_version': 1, 'mode': 'illustrative_selected_policy_replay',
        'excluded_from_selection_and_evaluation_counts': True,
        'training': training.relative_to(ROOT).as_posix(),
        'heldout_report': report_path.relative_to(ROOT).as_posix(),
        'heldout_report_sha256': digest(report_path),
        'frozen_report_and_manifest_hashes': frozen_inputs,
        'verification_contract': 'Supervisor exact numerical reassembly; native worker exact data/hash/identity readback without cross-Python numerical re-reduction.',
        'checkpoint_path': checkpoint.relative_to(ROOT).as_posix(), 'checkpoint_identity': selected,
        'reference_run': reference.relative_to(ROOT).as_posix(),
        'reference_trajectory_sha256': digest(reference / 'seed-4101-trajectory.json'),
        'reference_reset_sha256': digest(reference / 'seed-4101-reset.json'),
        'route_id': 'left_r60', 'seed': 4101, 'physics_hz': 120, 'control_hz': 60,
        'max_control_steps': 2100, 'max_process_seconds': 300,
        'gpu_memory_fraction_stop': .9, 'optimizer_steps': 0,
        'capture_sim_seconds': list(CAPTURE_SECONDS),
        'visible': {'width': 1440, 'height': 900, 'inspection_hold_wall_seconds': 20},
        'terminal_capture_rule': 'Do not label an auto-reset frame as a terminal pose; final overview is explicitly post-auto-reset.',
        'known_baseline_startup_errors': list(KNOWN_STARTUP_ERRORS),
    }


def verify_preview_job(job):
    """Native data-only verification before load, preserving all exact identities.

    The supervisor already reassembled both reports under project Python. Tiny
    Python-version reduction differences must not reinterpret or invalidate
    that frozen report in Isaac Python. No float tolerance is introduced here.
    The worker also verifies its running manifest/source inventory separately.
    """
    training = (ROOT / job['training']).resolve()
    report_path = (ROOT / job['heldout_report']).resolve()
    if (not training.is_relative_to(ROOT / 'outputs/lab_lane_ppo')
            or not report_path.is_relative_to(ROOT / 'outputs/lab_lane_ppo')
            or digest(report_path) != job['heldout_report_sha256']):
        raise ValueError('Frozen preview report path/hash differs')
    saved = _read(report_path)
    if saved.get('kind') != 'heldout':
        raise ValueError('Preview requires completed held-out data')
    expected = _preview_config(training, report_path, saved)
    if job != expected:
        raise ValueError('Frozen preview job, input hashes or selected checkpoint differ')
    return expected


def compare_trace(records, reset, reference_records, reference_reset):
    """Exact selected native/action fields; wall timing and episode IDs excluded.

    A difference is retained as a replay finding, not used to rerank policies or
    replace any held-out outcome. This is not a render-performance comparison.
    """
    keys = ('tick', 'physics_state', 'physics_samples', 'policy_action_raw', 'env_action',
            'requested_action', 'observation', 'route_state', 'reward', 'reward_components',
            'terminated', 'truncated', 'reason', 'success')
    differences = {key: [] for key in keys}
    for index, (actual, expected) in enumerate(zip(records, reference_records)):
        for key in keys:
            if actual[key] != expected[key]:
                differences[key].append(index)
    differing = {key: {'count': len(indices), 'first_control_step': indices[0] + 1}
                 for key, indices in differences.items() if indices}
    reset_equal = (reset['reset_spec'] == reference_reset['reset_spec']
                   and reset['physics_state'] == reference_reset['physics_state'])
    return {'reference_control_steps': len(reference_records), 'preview_control_steps': len(records),
        'reset_spec_and_native_state_exact': reset_equal,
        'exact_selected_fields': reset_equal and len(records) == len(reference_records) and not differing,
        'fields_compared': list(keys), 'differing_fields': differing,
        'excluded': ['wall timing', 'episode serial/IDs', 'capture metadata'],
        'contact_metadata_caveat': 'physics_samples retains raw contact metadata; process-specific contact identifiers may differ without proving changed motion.',
        'performance_comparison': False, 'changes_heldout_outcomes': False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--training', type=Path, required=True)
    parser.add_argument('--heldout-report', type=Path, required=True)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args(argv)
    config = resolve_preview(args.training, args.heldout_report)
    runtime = isaac_runtime(ROOT, load_config(ROOT))
    if args.check:
        print(json.dumps({'config': config, 'runtime': str(runtime), 'launches_runtime': False}, indent=2))
        return 0
    output, manifest = run_package(STUDY, config, preview_sources())
    logs = ROOT / 'logs' / STUDY / output.name
    logs.mkdir(parents=True, exist_ok=False)
    runtime_log = logs / 'runtime.log'
    command = [str(runtime), str(ROOT / 'scripts/preview_lane_follow_ppo.py'), '--output', str(output)]
    child, monitor, reason = None, None, None
    started = time.perf_counter()
    result = {'passed': False, 'mode': config['mode'], 'training_started': False, 'optimizer_steps': 0}
    try:
        manifest.update(command=command, runtime_log=runtime_log.relative_to(ROOT).as_posix(),
                        kit_before=kit_process_snapshot(), gpu_before=gpu_sample())
        if not manifest['kit_before']['available'] or manifest['kit_before']['count'] != 0:
            raise RuntimeError('Preview requires serial execution with no other Kit processes')
        if manifest['gpu_before']['used_mib'] / manifest['gpu_before']['total_mib'] >= .9:
            raise RuntimeError('Preflight whole-GPU memory guard')
        write_json(output / 'manifest.json', manifest)
        with runtime_log.open('wb') as log, GpuMonitor() as monitor:
            child = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0)
            stop_started = None
            while child.poll() is None:
                elapsed = time.perf_counter() - started
                if elapsed >= 300:
                    reason = 'Preview process deadline'
                elif monitor.errors:
                    reason = 'GPU monitor failed'
                elif monitor.rows and monitor.rows[-1]['used_mib'] / monitor.rows[-1]['total_mib'] >= .9:
                    reason = 'Whole-GPU memory guard'
                if reason:
                    if stop_started is None:
                        (output / 'STOP').write_text(reason, encoding='utf-8')
                        stop_started = time.perf_counter()
                    if elapsed >= 300 or time.perf_counter() - stop_started >= 10:
                        stop_child(child)
                        break
                time.sleep(.25)
        if (output / 'native-result.json').is_file():
            result = _read(output / 'native-result.json')
        else:
            result['error'] = 'Preview produced no final native result'
        shutil.copyfile(runtime_log, output / 'runtime.log')
        write_json(output / 'gpu-samples.json', monitor.rows)
        result.update(exit_code=child.returncode, stop_reason=reason,
            wall_process_seconds=time.perf_counter() - started,
            gpu_peak_mib=max((row['used_mib'] for row in monitor.rows), default=None),
            gpu_monitor_errors=monitor.errors,
            **classify_runtime_log(runtime_log.read_text(encoding='utf-8', errors='replace'), KNOWN_STARTUP_ERRORS))
        result['passed'] = bool(result.get('passed') and child.returncode == 0 and not reason
            and monitor.rows and not monitor.errors and not result['unexpected_runtime_errors']
            and result['usd_reference_warning_count'] == 0)
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
    print('LAB_LANE_PPO_PREVIEW=' + str(output), flush=True)
    print(json.dumps(result, indent=2), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
