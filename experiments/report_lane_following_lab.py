"""Audit immutable native packages and compare predeclared fresh-process repeats."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from experiments.probe_support import ROOT, write_json
from experiments.verify_lane_following_lab import ROUTES, classify_runtime_log, source_files, validate_config


def portable_keys(mapping):
    result = {key.replace('\\', '/'): value for key, value in mapping.items()}
    if len(result) != len(mapping):
        raise ValueError('Duplicate portable artifact names')
    return result


def artifact_path(directory, relative, inventory):
    relative = relative.replace('\\', '/')
    path = (directory / relative).resolve()
    if relative not in inventory or not path.is_relative_to(directory) or not path.is_file():
        raise ValueError('Missing or escaping recorded artifact: ' + relative)
    return path


def read_package(relative):
    directory = (ROOT / relative).resolve()
    if not directory.is_relative_to(ROOT / 'outputs/lab_lane_following'):
        raise ValueError('Expected a lane-following output package')
    manifest = json.loads((directory / 'manifest.json').read_text(encoding='utf-8'))
    inventory = portable_keys(manifest['output_hashes'])
    actual = {p.relative_to(directory).as_posix() for p in directory.rglob('*')
              if p.is_file() and p.name != 'manifest.json'}
    if set(inventory) != actual or not {'summary.json', 'probe-result.json', 'resolved-config.json',
                                       'runtime.log', 'scene-contract.json', 'resets.json'} <= set(inventory):
        raise ValueError('Artifact inventory is incomplete')
    if set(manifest['source_hashes']) != set(source_files()):
        raise ValueError('Source inventory differs from the declared protocol')
    for relative_file, digest in inventory.items():
        path = artifact_path(directory, relative_file, inventory)
        if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError('Artifact hash mismatch: ' + relative_file)
    for source, digest in manifest['source_hashes'].items():
        if inventory.get('source/' + source) != digest:
            raise ValueError('Captured source does not agree with source manifest: ' + source)
    manifest['output_hashes'] = inventory
    config = validate_config(json.loads((directory / 'resolved-config.json').read_text(encoding='utf-8')))
    if manifest['config'] != config:
        raise ValueError('Resolved config differs from manifest')
    summary = json.loads((directory / 'summary.json').read_text(encoding='utf-8'))
    return directory, manifest, config, summary


def audit_matrix(matrix):
    from experiments.lane_following_lab_validation import assess_episode, compare_episode_traces
    from traffic.lane_geometry import load_routes
    expected = {f'{route}-{kind}' for route in ROUTES for kind in ('nominal', 'repeat', 'perturbed')}
    expected |= {'diagnostics', 'contact-control', 'visible'}
    if set(matrix) != expected or len(set(matrix.values())) != len(matrix):
        raise ValueError('Matrix must contain all 12 distinct declared packages')
    rows, packages, sources, seen = [], {}, None, set()
    for label, relative in matrix.items():
        directory, manifest, config, summary = read_package(relative)
        if directory in seen:
            raise ValueError('Matrix reuses a resolved package')
        seen.add(directory)
        if sources is None:
            sources = manifest['source_hashes']
        if manifest['source_hashes'] != sources:
            raise ValueError('Cannot combine different runtime sources: ' + label)
        route = load_routes(directory / 'source' / config['routes_file'])[config['route_id']]
        expected_profile = label if label in ('diagnostics', 'contact-control', 'visible') else label.rsplit('-', 1)[1]
        if expected_profile == 'repeat':
            expected_profile = 'nominal'
        if config['profile'] != expected_profile or (label not in ('diagnostics', 'contact-control', 'visible')
                                                     and not label.startswith(config['route_id'] + '-')):
            raise ValueError('Package does not match its declared matrix cell: ' + label)
        special_routes = {'diagnostics': 'straight100', 'contact-control': 'straight100', 'visible': 'left_r60'}
        if label in special_routes and config['route_id'] != special_routes[label]:
            raise ValueError('Diagnostic/visible package uses the wrong declared route')
        declared_episodes = {
            'nominal': [('nominal', 101, 'success')], 'visible': [('nominal', 101, 'success')],
            'perturbed': [('corner-0', 1101, 'success'), ('corner-1', 1102, 'success')],
            'diagnostics': [('zero-drive-timeout', 101, 'time_limit'), ('hard-steer-departure', 101, 'lane_departure')],
            'contact-control': [('contact-positive-control', 101, 'rigid_contact')],
        }[expected_profile]
        if [(e.get('label'), e.get('seed'), e.get('expected_reason')) for e in summary['episodes']] != declared_episodes:
            raise ValueError('Package episodes differ from the declared matrix cell')
        # This report rechecks raw telemetry, independently of the stored pass flag.
        reassessed = []
        for episode in summary['episodes']:
            if (episode['trajectory_file'] != episode['label'] + '-trajectory.json'
                    or episode['reset_file'] != episode['label'] + '-reset.json'):
                raise ValueError('Unexpected episode artifact names')
            records = json.loads(artifact_path(directory, episode['trajectory_file'], manifest['output_hashes']).read_text(encoding='utf-8'))
            reset = json.loads(artifact_path(directory, episode['reset_file'], manifest['output_hashes']).read_text(encoding='utf-8'))
            spec = reset['reset_spec']
            corner = config['perturbation_corners'][int(episode['label'][-1])] if expected_profile == 'perturbed' else [0., 0.]
            if (spec['seed'] != episode['seed'] or spec['episode_index'] != 0
                    or [spec['lateral_offset_m'], spec['yaw_error_rad']] != corner):
                raise ValueError('Episode reset differs from the declared matrix cell')
            assessment = assess_episode(records, reset, route, config['acceptance'],
                                        expected_reason=episode['expected_reason'])
            reassessed.append(assessment)
        # Native subprocesses can emit Windows code-page bytes into the captured
        # log. Match supervisor decoding; the original bytes remain hash-checked.
        log_checks = classify_runtime_log((directory / 'runtime.log').read_text(encoding='utf-8', errors='replace'),
                                         config['known_baseline_startup_errors'])
        diagnostic_keys = {'same_seed_native_reset', 'different_seed_reset', 'signed_native_pose_reset',
                           'invalid_actions_rejected_without_physics', 'finite_clipping_and_brake_priority'}
        diagnostics_ok = expected_profile != 'diagnostics' or (
            set(summary.get('diagnostic_checks', {})) == diagnostic_keys
            and all(v is True for v in summary['diagnostic_checks'].values()))
        visible_ok = expected_profile != 'visible' or (summary.get('preview_complete') is True
            and len(summary.get('preview_files', [])) == 4
            and all(name in manifest['output_hashes'] for name in summary['preview_files']))
        scene = json.loads((directory / 'scene-contract.json').read_text(encoding='utf-8'))
        passed = (manifest['status'] == 'passed' and summary['passed'] is True
                  and summary['training_started'] is False
                  and type(summary['optimizer_steps']) is int and summary['optimizer_steps'] == 0
                  and summary.get('exit_code') == 0 and summary.get('stop_reason') is None
                  and summary.get('source_changed_during_run') == []
                  and summary.get('gpu_monitor_errors') == []
                  and summary.get('viewport_cleanup', {}).get('passed') is True
                  and summary.get('contact_reporting', {}).get('cleanup', {}).get('passed') is True
                  and summary.get('kit_after', {}).get('available') is True
                  and summary['kit_after']['count'] == 0
                  and manifest.get('kit_before', {}).get('available') is True
                  and manifest['kit_before']['count'] == 0
                  and not log_checks['unexpected_runtime_errors'] and not log_checks['usd_reference_warning_count']
                  and all(summary.get(key) == value for key, value in log_checks.items())
                  and scene == summary.get('scene_contract') and scene.get('passed') is True
                  and scene.get('static_hashes_unchanged') is True and diagnostics_ok and visible_ok
                  and bool(reassessed) and all(r['passed'] for r in reassessed))
        rows.append({'label': label, 'run': directory.name, 'passed': bool(passed),
                     'episodes': reassessed, 'wall_process_seconds': summary['wall_process_seconds'],
                     'gpu_peak_mib': summary['gpu_peak_mib'],
                     'known_startup_error_count': len(summary['known_baseline_startup_errors']),
                     'unexpected_runtime_error_count': len(summary['unexpected_runtime_errors']),
                     'usd_reference_warning_count': summary['usd_reference_warning_count'],
                     'steps_per_step_wall_second': sum(e['control_steps'] for e in summary['episodes']) /
                        sum(e['step_wall_seconds'] for e in summary['episodes']),
                     'steps_per_episode_loop_wall_second': sum(e['control_steps'] for e in summary['episodes']) /
                        sum(e['loop_wall_seconds'] for e in summary['episodes'])})
        packages[label] = (directory, config)
    repeats = {}
    for route in ROUTES:
        first, config = packages[f'{route}-nominal']
        second, _ = packages[f'{route}-repeat']
        a = json.loads((first / 'nominal-trajectory.json').read_text(encoding='utf-8'))
        b = json.loads((second / 'nominal-trajectory.json').read_text(encoding='utf-8'))
        repeats[route] = compare_episode_traces(a, b, config['acceptance'])
    return {'schema_version': 1, 'passed': all(r['passed'] for r in rows) and all(r['passed'] for r in repeats.values()),
            'training_started': False, 'optimizer_steps': 0, 'runs': rows, 'repeats': repeats,
            'source_hashes': sources, 'raw_artifacts_in_git': False,
            'scope': 'One CPU-physics native car at 3 m/s; scripted controller and privileged observations only.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--matrix', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT) or output.exists():
        parser.error('Use a fresh report path inside the project')
    report = audit_matrix(json.loads(args.matrix.read_text(encoding='utf-8')))
    write_json(output, report)
    print(json.dumps({'passed': report['passed'], 'runs': len(report['runs']), 'report': str(output)}))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
