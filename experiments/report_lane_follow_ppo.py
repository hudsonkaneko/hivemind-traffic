"""Verify completed packages and assemble immutable CPU-only PPO evidence.

Examples (all paths are inside this checkout):
  python -m experiments.report_lane_follow_ppo selection --training TRAIN \
      --packages EVAL8 EVAL16 EVAL24 EVAL32 --output outputs/lab_lane_ppo/reports/selection.json
  python -m experiments.report_lane_follow_ppo heldout --training TRAIN \
      --selection-report outputs/lab_lane_ppo/reports/selection.json \
      --packages PPO_STRAIGHT PPO_LEFT PPO_RIGHT SCRIPT_STRAIGHT SCRIPT_LEFT SCRIPT_RIGHT \
      --output outputs/lab_lane_ppo/reports/heldout.json

Execution validity never implies successful driving. No model is unpickled and
no native runtime, optimizer or process is launched. The evaluator is frozen
separately from this packaging command. Completed inputs are read-only.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path, PurePosixPath
import re

from experiments.lane_follow_ppo_evaluation import (
    HELD_OUT_SEEDS, ROUTE_IDS, SELECTION_SEEDS, UPDATES,
    assess_held_out, case_label, select_checkpoint,
)
from experiments.lane_follow_ppo_protocol import (
    PILOT_ID, checkpoint_identity, digest, load_protocol, source_files, verify_sources,
)
from experiments.probe_support import ROOT
from experiments.verify_lane_following_lab import EXPECTED_ACCEPTANCE
from traffic.lane_geometry import load_routes

ASSEMBLER = 'experiments/report_lane_follow_ppo.py'
ASSEMBLER_TEST = 'tests/test_report_lane_follow_ppo.py'
REDUCER = 'experiments/lane_follow_ppo_evaluation.py'


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _read(path):
    def reject_constant(value):
        raise ValueError('Nonfinite JSON constant: '+value)
    def unique_pairs(pairs):
        result = {}
        for key, value in pairs:
            _require(key not in result, 'Duplicate JSON key: '+key)
            result[key] = value
        return result
    return json.loads(Path(path).read_text(encoding='utf-8'), parse_constant=reject_constant,
                      object_pairs_hook=unique_pairs)


def _path(value, root):
    path = Path(value)
    return (path if path.is_absolute() else root/path).resolve()


def _member(directory, name):
    _require(isinstance(name, str), 'Artifact path must be a string')
    clean = name.replace('\\', '/')
    logical = PurePosixPath(clean)
    _require(bool(clean) and not logical.is_absolute() and ':' not in clean
             and all(part not in ('', '.', '..') for part in clean.split('/')), 'Unsafe artifact path: '+name)
    path = (directory/clean).resolve()
    _require(path.is_relative_to(directory) and path != directory, 'Artifact escaped package: '+name)
    return clean, path


def _inventory(directory, recorded):
    _require(isinstance(recorded, dict) and bool(recorded), 'Missing artifact hash inventory')
    normalized = {}
    for name, expected in recorded.items():
        clean, path = _member(directory, name)
        _require(clean not in normalized, 'Duplicate normalized artifact path')
        _require(isinstance(expected, str) and re.fullmatch('[0-9a-f]{64}', expected), 'Malformed artifact hash')
        _require(path.is_file() and digest(path) == expected, 'Artifact hash mismatch: '+clean)
        normalized[clean] = expected
    actual = set()
    for path in directory.rglob('*'):
        if path.is_file() and path != directory/'manifest.json':
            _require(path.resolve().is_relative_to(directory), 'File resolves outside package')
            actual.add(path.relative_to(directory).as_posix())
    _require(actual == set(normalized), 'Package has missing or unexpected files')
    return normalized


def _equivalent(a, b):
    """Only floating recomputation roundoff may differ between runtime Pythons."""
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(_equivalent(a[k], b[k]) for k in a)
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(_equivalent(x, y) for x, y in zip(a, b))
    if type(a) in (int, float) and type(b) in (int, float):
        return math.isfinite(a) and math.isfinite(b) and math.isclose(a, b, rel_tol=1e-12, abs_tol=1e-10)
    return type(a) is type(b) and a == b


def verify_package(path, *, root=ROOT, expected_sources=None):
    """Verify full files, frozen source inventory, execution and teardown."""
    root = Path(root).resolve()
    directory = _path(path, root)
    _require(directory.is_dir() and directory.is_relative_to(root/'outputs/lab_lane_ppo'),
             'Package must be a completed pilot directory in this checkout')
    manifest = _read(directory/'manifest.json')
    _require(isinstance(manifest, dict) and manifest.get('status') == 'passed'
             and isinstance(manifest.get('finished_utc'), str), 'Package did not complete successfully')
    inventory = _inventory(directory, manifest.get('output_hashes'))
    for required in ('summary.json', 'resolved-config.json', 'native-result.json', 'runtime.log', 'gpu-samples.json'):
        _require(required in inventory, 'Missing required completed artifact: '+required)
    sources = manifest.get('source_hashes')
    _require(isinstance(sources, dict) and set(sources) == set(source_files()), 'Runtime source inventory differs')
    _require(expected_sources is None or sources == expected_sources, 'Runtime source hashes differ from training')
    for name, expected in sources.items():
        clean, copied = _member(directory, 'source/'+name)
        _require(clean in inventory and inventory[clean] == expected and digest(copied) == expected,
                 'Captured source does not match manifest: '+name)
    verify_sources(manifest, root=root)
    config, summary, native = (_read(directory/name) for name in ('resolved-config.json', 'summary.json', 'native-result.json'))
    _require(all(isinstance(value, dict) for value in (config, summary, native)), 'Malformed completed package records')
    _require(manifest.get('config') == config, 'Resolved config differs from manifest')
    _require(config.get('pilot_id') == PILOT_ID and config.get('protocol') == load_protocol(root), 'Unapproved resolved protocol')
    _require(config.get('acceptance') == dict(EXPECTED_ACCEPTANCE), 'Acceptance thresholds differ from the frozen task')
    _require(summary.get('passed') is True and native.get('passed') is True, 'Native/supervised execution failed')
    _require(all(key in summary and _equivalent(value, summary[key]) for key, value in native.items()),
             'Native result differs from supervised summary')
    _require(summary.get('mode') == config.get('mode') and summary.get('source_unchanged') is True
        and type(summary.get('exit_code')) is int and summary['exit_code'] == 0 and summary.get('stop_reason') is None
        and summary.get('gpu_monitor_errors') == [] and summary.get('unexpected_runtime_errors') == []
        and summary.get('usd_reference_warning_count') == 0, 'Execution integrity checks failed')
    _require(manifest.get('kit_before', {}).get('available') is True and type(manifest['kit_before'].get('count')) is int and manifest['kit_before']['count'] == 0
        and summary.get('kit_after', {}).get('available') is True and type(summary['kit_after'].get('count')) is int and summary['kit_after']['count'] == 0,
        'Process isolation/cleanup failed')
    _require(summary.get('viewport_cleanup', {}).get('passed') is True
        and summary.get('contact_reporting', {}).get('cleanup', {}).get('passed') is True,
        'Runtime resource/settings cleanup failed')
    _require(summary.get('scene_contract', {}).get('passed') is True
        and summary['scene_contract'].get('static_hashes_unchanged') is True, 'Scene contract failed')
    gpu = _read(directory/'gpu-samples.json')
    _require(isinstance(gpu, list) and bool(gpu), 'GPU telemetry missing')
    _require(all(isinstance(row, dict) and all(type(row.get(key)) in (int, float)
        and math.isfinite(row[key]) for key in ('used_mib','total_mib'))
        and 0 <= row['used_mib'] < .9*row['total_mib'] and row['total_mib'] > 0 for row in gpu),
        'GPU resource telemetry violates the declared bound')
    return dict(directory=directory, manifest=manifest, config=config, summary=summary, inventory=inventory,
        reference=dict(path=directory.relative_to(root).as_posix(), manifest_sha256=digest(directory/'manifest.json')))


def _training(path, root):
    package = verify_package(path, root=root)
    config, summary = package['config'], package['summary']
    state = summary.get('training_state', {})
    _require(config['mode'] == 'train' and config['route_id'] == 'left_r60' and config['seeds'] == [2101]
        and config['stage'] is None and config['controller'] == 'ppo' and config['checkpoint_path'] is None
        and config['checkpoint_identity'] is None, 'Not the approved training job')
    _require(summary.get('training_started') is True and state.get('control_steps') == 65536
        and state.get('completed_updates') == 32 and type(state.get('optimizer_steps')) is int
        and 0 < state['optimizer_steps'] <= 5120 and summary.get('optimizer_steps') == state['optimizer_steps'],
        'Training did not complete the approved budget')
    checkpoints = summary.get('checkpoints')
    _require(isinstance(checkpoints, list) and len(checkpoints) == 4, 'Missing checkpoint inventory')
    identities, paths = {}, {}
    for record in checkpoints:
        _require(isinstance(record, dict) and set(record) == {'path', 'identity'}, 'Malformed checkpoint record')
        directory = _path(record['path'], root)
        identity = checkpoint_identity(directory, root=root)
        update = identity['update']
        _require(directory == package['directory']/'checkpoints'/f'update-{update:02d}'
            and record['identity'] == identity and update not in identities, 'Checkpoint is not uniquely owned by training')
        identities[update], paths[update] = identity, directory
    _require(set(identities) == set(UPDATES), 'Checkpoint update inventory differs')
    package.update(checkpoints=identities, checkpoint_paths=paths)
    return package


def _cases(package, training, stage, routes):
    config, summary = package['config'], package['summary']
    seeds = list(SELECTION_SEEDS if stage == 'selection' else HELD_OUT_SEEDS)
    _require(config['mode'] == 'evaluate' and config['stage'] == stage and config['seeds'] == seeds,
             'Evaluation stage or seed set differs')
    _require(summary.get('training_started') is False and type(summary.get('optimizer_steps')) is int and summary['optimizer_steps'] == 0
             and summary.get('evaluation_frozen') is True, 'Evaluation changed learned state or ran optimization')
    _require(summary.get('runtime') == training['summary'].get('runtime')
             and summary.get('versions') == training['summary'].get('versions'), 'Evaluation runtime differs from training')
    controller, route = config['controller'], config['route_id']
    _require(controller in ('ppo', 'scripted') and route in ROUTE_IDS, 'Unknown evaluation controller/route')
    _require(stage != 'selection' or (controller == 'ppo' and route == 'left_r60'), 'Invalid checkpoint-selection route/controller')
    identity = config['checkpoint_identity']
    if controller == 'ppo':
        _require(isinstance(identity, dict) and identity.get('update') in training['checkpoints'], 'Unknown evaluated checkpoint')
        update = identity['update']
        _require(identity == training['checkpoints'][update]
            and _path(config['checkpoint_path'], training['root']) == training['checkpoint_paths'][update],
            'Evaluated checkpoint differs from training')
    else:
        _require(identity is None and config['checkpoint_path'] is None, 'Scripted evaluation loaded a checkpoint')
        update = None
    saved = _read(package['directory']/'evaluation-cases.json')
    _require(saved == summary.get('cases') and isinstance(saved, list) and len(saved) == len(seeds)
             and all(isinstance(row, dict) for row in saved),
             'Case inventory differs from supervised result')
    _require([row.get('seed') for row in saved] == seeds, 'Case seeds missing, reordered or replaced')
    answer = []
    for row in saved:
        seed = row['seed']
        _require(row['label'] == case_label(stage, controller, route, seed, update)
            and row['policy'] == controller and row['route_id'] == route and row['checkpoint'] == identity,
            'Case identity differs from resolved job')
        _require(row['trajectory_file'] == f'seed-{seed}-trajectory.json' and row['reset_file'] == f'seed-{seed}-reset.json',
                 'Raw evidence filenames differ from the declared seed')
        raw = []
        for key in ('trajectory_file', 'reset_file'):
            name, path = _member(package['directory'], row[key])
            _require(name in package['inventory'], 'Raw case file is not hash-covered')
            raw.append(_read(path))
        answer.append(dict(label=row['label'], policy=controller, checkpoint=identity, route_id=route,
                           seed=seed, records=raw[0], reset_record=raw[1]))
    return answer, {(controller, route, update)}


def _context(training_path, root):
    root = Path(root).resolve()
    training = _training(training_path, root)
    training['root'] = root
    route_file = root/training['config']['protocol']['task']['route_file']
    routes = load_routes(route_file)
    _require(set(routes) == set(ROUTE_IDS), 'Frozen route inventory differs')
    return root, training, routes


def _packages(paths, training, root, count):
    _require(len(paths) == count and len({_path(path, root) for path in paths}) == count, 'Missing or duplicate evaluation packages')
    return [verify_package(path, root=root, expected_sources=training['manifest']['source_hashes']) for path in paths]


def _envelope(kind, training, packages, report, root):
    envelope = dict(schema_version=1, kind=kind, pilot_id=PILOT_ID,
        training=training['reference'], packages=sorted((p['reference'] for p in packages), key=lambda row: row['path']),
        assembler_source_sha256=digest(root/ASSEMBLER), reducer_source_sha256=digest(root/REDUCER),
        assembler_test_source_sha256=digest(root/ASSEMBLER_TEST),
        runtime_source_hashes=training['manifest']['source_hashes'], report=report,
        interpretation='valid means complete verified execution evidence; consult driving_successes and physical_failures for driving outcomes')
    # Reducer-native tuples (for example first guard tick/reason) become JSON
    # arrays on disk. Return that canonical shape so later exact recomputation
    # compares durable values rather than Python container implementation types.
    return json.loads(json.dumps(envelope, allow_nan=False))


def _check_native_assessments(packages, report):
    for package in packages:
        for case in package['summary']['cases']:
            _require(_equivalent(case['assessment'], report['cases'][case['label']]['outcome']),
                     'Native episode assessment differs from fresh raw reduction: '+case['label'])


def assemble_selection(training_path, package_paths, *, root=ROOT):
    root, training, routes = _context(training_path, root)
    packages = _packages(package_paths, training, root, 4)
    cases, inventory = [], set()
    for package in packages:
        raw, identity = _cases(package, training, 'selection', routes)
        _require(not inventory.intersection(identity), 'Duplicate selection checkpoint package')
        cases.extend(raw)
        inventory.update(identity)
    _require(inventory == {('ppo', 'left_r60', update) for update in UPDATES}, 'Selection package inventory differs')
    report = select_checkpoint(cases, [training['checkpoints'][u] for u in UPDATES], routes, dict(EXPECTED_ACCEPTANCE))
    _require(report['valid'], 'Selection raw evidence invalid: '+json.dumps(report['checks']))
    _check_native_assessments(packages, report)
    return _envelope('selection', training, packages, report, root)


def assemble_held_out(training_path, selection_path, package_paths, *, root=ROOT):
    root, training, routes = _context(training_path, root)
    selection_path = _path(selection_path, root)
    _require(selection_path.is_relative_to(root/'outputs/lab_lane_ppo'), 'Selection report must belong to this pilot workspace')
    saved = _read(selection_path)
    _require(isinstance(saved, dict) and saved.get('kind') == 'selection' and saved.get('training') == training['reference'],
             'Selection report belongs to another training package')
    references = saved.get('packages')
    _require(isinstance(references, list) and len(references) == 4 and all(isinstance(p, dict) and set(p) == {'path','manifest_sha256'} for p in references),
             'Selection package provenance missing')
    # Recompute all validation episodes and exact selection before reading held-out
    # results. Tampered rankings/identity/hash declarations cannot select a model.
    recomputed = assemble_selection(training_path, [p['path'] for p in references], root=root)
    _require(saved == recomputed, 'Saved selection report does not reproduce from its immutable inputs')
    selected = saved['report']['selected_checkpoint']
    packages = _packages(package_paths, training, root, 6)
    cases, inventory = [], set()
    for package in packages:
        raw, identity = _cases(package, training, 'heldout', routes)
        _require(not inventory.intersection(identity), 'Duplicate held-out policy/route package')
        cases.extend(raw)
        inventory.update(identity)
    _require(inventory == {(policy, route, selected['update'] if policy == 'ppo' else None)
        for policy in ('ppo','scripted') for route in ROUTE_IDS}, 'Held-out package inventory differs from the frozen choice')
    report = assess_held_out(cases, selected, routes, dict(EXPECTED_ACCEPTANCE))
    _require(report['valid'], 'Held-out raw evidence invalid: '+json.dumps(report['checks']))
    _check_native_assessments(packages, report)
    envelope = _envelope('heldout', training, packages, report, root)
    envelope['selection'] = dict(path=selection_path.relative_to(root).as_posix(), sha256=digest(selection_path))
    return envelope


def write_report(path, report, *, root=ROOT):
    """Exclusive creation, outside every completed input package."""
    root = Path(root).resolve()
    path = _path(path, root)
    _require(path.is_relative_to(root/'outputs/lab_lane_ppo') and path.suffix == '.json',
             'Report output must be a new JSON within outputs/lab_lane_ppo')
    for parent in path.parents:
        if parent == root:
            break
        _require(not (parent/'manifest.json').is_file(), 'Cannot write into an immutable run package')
    payload = json.dumps(report, indent=2, allow_nan=False)+'\n'
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', encoding='utf-8') as stream:
        stream.write(payload)
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('stage', choices=('selection','heldout'))
    parser.add_argument('--training', type=Path, required=True)
    parser.add_argument('--packages', type=Path, nargs='+', required=True)
    parser.add_argument('--selection-report', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.stage == 'selection':
            _require(args.selection_report is None, 'Selection cannot use a prior selection or held-out result')
            result = assemble_selection(args.training, args.packages)
        else:
            _require(args.selection_report is not None, 'Held-out reduction requires the frozen selection report')
            result = assemble_held_out(args.training, args.selection_report, args.packages)
        path = write_report(args.output, result)
    except (ValueError, KeyError, TypeError, OSError) as error:
        parser.exit(1, f'Evidence rejected: {error}\n')
    print(json.dumps(dict(report=str(path), sha256=digest(path), execution_evidence_valid=result['report']['valid'])))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
