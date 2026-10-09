"""Compare two completed, independently launched MainLane1 physical loop runs.

Usage: python -m experiments.highway_loop_repeatability --run-a PATH --run-b PATH
Inputs are read-only, immediate children of outputs/highway_loop. A fresh report
is written under outputs/highway_loop_repeatability. This measures same-machine,
same-version numerical repeatability, not cross-platform determinism or learned
policy skill. Matching project hashes alone is insufficient: the recorded Isaac
build, installed Factory source hash, GPU model/driver, platform, logical CPU
count and supervisor Python must also match. GPU utilisation/free-memory samples
are not runtime identity and are deliberately not required to be identical.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

from experiments.highway_loop_config import validate_config
from experiments.probe_support import ROOT, finish, run_package


INPUT_FILES = ('summary.json', 'manifest.json', 'trajectory.json', 'resolved-config.json', 'scene-contract.json')
SOURCES = ['experiments/highway_loop_repeatability.py', 'experiments/highway_loop_config.py']
RADIUS_M = 500.0  # The fixed MainLane1 fixture, not an arbitrary navigation map.


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _sha256(data):
    return hashlib.sha256(data).hexdigest()


def _hashes(value):
    return (isinstance(value, dict) and bool(value)
            and all(isinstance(name, str) and bool(name)
                    and isinstance(digest, str) and len(digest) == 64
                    and all(c in '0123456789abcdef' for c in digest)
                    for name, digest in value.items()))


def _same_json(left, right):
    try:
        return json.dumps(left, sort_keys=True, allow_nan=False) == json.dumps(right, sort_keys=True, allow_nan=False)
    except (TypeError, ValueError):
        return False


def resolve_run(path):
    """Reject broad roots, nested evidence, symlink escapes and other studies."""
    candidate = Path(path).resolve()
    parent = (ROOT / 'outputs' / 'highway_loop').resolve()
    if candidate.parent != parent or not candidate.is_dir():
        raise ValueError('Each run must be an existing immediate child of canonical outputs/highway_loop')
    return candidate


def load_run(path):
    """Load fixed evidence files and verify recorded output/source fingerprints."""
    directory = resolve_run(path)
    documents, fingerprints = {}, {}
    for name in INPUT_FILES:
        source = directory / name
        if not source.is_file() or source.resolve().parent != directory:
            raise ValueError(f'Missing or externally linked input evidence: {name}')
        data = source.read_bytes()
        fingerprints[name] = _sha256(data)
        documents[name] = json.loads(data)
    manifest = documents['manifest.json']
    errors = []
    if not isinstance(manifest, dict):
        errors.append('manifest must be an object')
    else:
        recorded_outputs = manifest.get('output_hashes', {})
        for name in ('summary.json', 'trajectory.json', 'resolved-config.json', 'scene-contract.json'):
            if not isinstance(recorded_outputs, dict) or recorded_outputs.get(name) != fingerprints[name]:
                errors.append(f'output hash mismatch or missing: {name}')
        recorded_sources = manifest.get('source_hashes')
        if not _hashes(recorded_sources):
            errors.append('source hashes are empty or malformed')
        else:
            source_root = (directory / 'source').resolve()
            if not source_root.is_relative_to(directory):
                errors.append('source snapshot resolves outside its run')
            else:
                for name, digest in recorded_sources.items():
                    source = (source_root / name).resolve()
                    if (not source.is_relative_to(source_root) or not source.is_file()
                            or _sha256(source.read_bytes()) != digest):
                        errors.append(f'source snapshot hash mismatch or missing: {name}')
    return dict(run_id=directory.name, relative_path=directory.relative_to(ROOT).as_posix(),
                summary=documents['summary.json'], manifest=manifest,
                trajectory=documents['trajectory.json'], config=documents['resolved-config.json'],
                scene_contract=documents['scene-contract.json'],
                file_hashes=fingerprints, load_errors=errors)


def _runtime_identity(run):
    """Extract matching installation/hardware evidence, not run-specific paths.

    The external NVIDIA Factory module is outside project source_hashes, so its
    fingerprint comes from the recorded vehicle contract. ``python`` is the
    supervisor's version as recorded by run_package, not a claim about Isaac's
    bundled interpreter. These recorded fields are not a unique hardware ID.
    """
    manifest = run.get('manifest')
    scene = run.get('scene_contract')
    if not isinstance(manifest, dict) or not isinstance(scene, dict):
        raise ValueError('manifest and scene contract are required for runtime identity')
    gpu, vehicle = manifest.get('gpu_before'), scene.get('vehicle')
    if not isinstance(gpu, dict) or not isinstance(vehicle, dict):
        raise ValueError('GPU and vehicle metadata are required for runtime identity')
    identity = dict(isaac_version=manifest.get('isaac_version'),
        supervisor_python=manifest.get('python'), platform=manifest.get('platform'),
        logical_cpus=manifest.get('logical_cpus'), gpu_name=gpu.get('name'),
        gpu_driver=gpu.get('driver'), factory_sha256=vehicle.get('factory_sha256'))
    for field, value in identity.items():
        if field == 'logical_cpus':
            if type(value) is not int or value <= 0:
                raise ValueError('logical_cpus must be a positive integer')
        elif not isinstance(value, str) or not value.strip():
            raise ValueError(f'runtime identity field is missing or empty: {field}')
    if not _hashes({'factory': identity['factory_sha256']}):
        raise ValueError('Factory source SHA-256 is malformed')
    return identity


def _trajectory_metrics(rows, config):
    """Validate 120 Hz records and independently unwrap the physical XY path."""
    expected = config['duration_s'] * config['physics_hz']
    if not isinstance(rows, list) or not rows or len(rows) != expected:
        raise ValueError('trajectory is empty, missing or does not cover the full configured episode')
    hz = config['physics_hz']
    if hz != 120 or config['lane_id'] != 'MainLane1':
        raise ValueError('repeatability fixture requires MainLane1 at 120 Hz')
    angle_previous = None
    measured_progress = 0.0
    drive_progress = None
    drive_recorded_progress = None
    drive_completed_laps = None
    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            raise ValueError(f'trajectory row {index} is not an object')
        tick = index + 1
        if (type(row.get('tick')) is not int or row['tick'] != tick
                or type(row.get('applied_tick')) is not int or row['applied_tick'] != index
                or not _finite(row.get('sim_time_s'))
                or not math.isclose(row['sim_time_s'], tick / hz, rel_tol=0, abs_tol=1e-6)):
            raise ValueError(f'noncontiguous or misaligned physics tick at row {index}')
        position = row.get('position_m')
        if (not isinstance(position, (list, tuple)) or len(position) != 3
                or not all(_finite(value) for value in position)
                or not _finite(row.get('speed_m_s')) or row['speed_m_s'] < 0
                or not _finite(row.get('progress_m'))):
            raise ValueError(f'nonfinite or malformed pose/speed/progress at tick {tick}')
        applied = row['applied_tick']
        phase = ('settle' if applied < config['settle_s'] * hz else
                 'drive' if applied < (config['settle_s'] + config['drive_s']) * hz else 'brake')
        if row.get('phase') != phase:
            raise ValueError(f'phase/tick mismatch at tick {tick}')
        x, y = position[:2]
        if abs(math.hypot(x, y) - RADIUS_M) > 1.85:
            raise ValueError(f'physical pose is not on the MainLane1 fixture at tick {tick}')
        angle = math.atan2(y, x)
        if angle_previous is not None:
            delta = (angle - angle_previous + math.pi) % math.tau - math.pi
            # Match the controller's generous measurement guard; don't turn a
            # teleport or missing circuit into evidence of a continuous lap.
            if abs(delta * RADIUS_M) > 24 / hz + 0.05:
                raise ValueError(f'implausible per-tick route jump at tick {tick}')
            measured_progress += delta * RADIUS_M
        angle_previous = angle
        if phase == 'drive':
            drive_progress = measured_progress
            drive_recorded_progress = row['progress_m']
            drive_completed_laps = row.get('completed_laps')
    length = math.tau * RADIUS_M
    if drive_progress is None or drive_progress < length:
        raise ValueError('fewer than one true complete lap in physical drive-phase positions')
    if (drive_recorded_progress < length or type(drive_completed_laps) is not int
            or drive_completed_laps < 1
            or abs(drive_recorded_progress - drive_progress) > 0.02):
        raise ValueError('recorded progress/laps disagree with the physical complete lap')
    return dict(samples=len(rows), measured_drive_progress_m=drive_progress,
                measured_complete_laps=int(drive_progress // length))


def compare_evidence(run_a, run_b):
    """Pure comparison of loaded evidence dictionaries; never reads or writes.

    File integrity belongs to load_run. Missing or malformed evidence produces
    passed=False rather than a vacuous pass. Every aligned physics state is
    compared, including settling, acceleration and the final braking/hold.
    """
    failures, metrics, runtime_identities = [], {}, {}
    if not isinstance(run_a, dict) or not isinstance(run_b, dict):
        return dict(passed=False, failures=['two evidence objects are required'])
    ids = [run.get('run_id') for run in (run_a, run_b)]
    if (not all(isinstance(value, str) and value.strip() for value in ids)
            or ids[0] == ids[1]):
        failures.append('run IDs must identify two independent invocations')
    configs = [run.get('config') for run in (run_a, run_b)]
    if not _same_json(*configs):
        failures.append('full resolved configurations differ')
    manifests = [run.get('manifest') for run in (run_a, run_b)]
    if not all(isinstance(value, dict) for value in manifests):
        failures.append('both manifests are required')
    elif (not all(_hashes(value.get('source_hashes')) for value in manifests)
          or manifests[0].get('source_hashes') != manifests[1].get('source_hashes')):
        failures.append('nonempty exact source hashes must match')
    for label, run in (('a', run_a), ('b', run_b)):
        try:
            runtime_identities[label] = _runtime_identity(run)
        except ValueError as error:
            failures.append(f'{label}: {error}')
    if len(runtime_identities) == 2 and runtime_identities['a'] != runtime_identities['b']:
        changed = [key for key in runtime_identities['a']
                   if runtime_identities['a'][key] != runtime_identities['b'][key]]
        failures.append('recorded runtime/machine identity differs: ' + ', '.join(changed))
    for label, run in (('a', run_a), ('b', run_b)):
        errors = run.get('load_errors', [])
        if errors:
            failures.append(f'{label}: input integrity errors: {errors}')
        config, manifest, summary = run.get('config'), run.get('manifest'), run.get('summary')
        try:
            if not isinstance(config, dict):
                raise ValueError('resolved configuration is missing')
            validate_config(config)
            if config['minimum_laps'] < 1:
                raise ValueError('seam-only smoke profiles do not qualify as complete-lap repeats')
            if not isinstance(manifest, dict) or manifest.get('status') != 'passed':
                raise ValueError('manifest is missing or is not completed/passed')
            if not manifest.get('finished_utc') or not _same_json(manifest.get('config'), config):
                raise ValueError('manifest lacks completion timestamp or matching full configuration')
            command = manifest.get('command')
            if (not isinstance(command, list) or len(command) < 4 or command[-2] != '--output'
                    or not isinstance(command[-1], str) or Path(command[-1]).name != run.get('run_id')):
                raise ValueError('manifest lacks a distinct supervised process launch for this run ID')
            if (not isinstance(summary, dict) or summary.get('passed') is not True
                    or type(summary.get('exit_code')) is not int or summary['exit_code'] != 0
                    or summary.get('source_changed_during_run') != []
                    or type(summary.get('completed_laps')) is not int or summary['completed_laps'] < 1):
                raise ValueError('summary must pass, exit zero, retain source hashes and complete a true lap')
            metrics[label] = _trajectory_metrics(run.get('trajectory'), config)
            if summary['completed_laps'] != metrics[label]['measured_complete_laps']:
                raise ValueError('summary lap count disagrees with physical trajectory')
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            failures.append(f'{label}: {error}')
    result = dict(passed=False, run_ids=ids, failures=failures, input_metrics=metrics,
        runtime_identities=runtime_identities,
        scope='Two fresh supervised processes, fixed MainLane1, same full configuration, project and Factory hashes, and recorded runtime/machine identity; not cross-platform determinism')
    if failures:
        return result
    rows_a, rows_b = run_a['trajectory'], run_b['trajectory']
    position_errors = [math.dist(a['position_m'], b['position_m']) for a, b in zip(rows_a, rows_b)]
    speed_errors = [abs(a['speed_m_s'] - b['speed_m_s']) for a, b in zip(rows_a, rows_b)]
    if not all(math.isfinite(value) for value in position_errors + speed_errors):
        failures.append('aligned pose/speed differences are not finite')
        return result
    position_max, speed_max = max(position_errors), max(speed_errors)
    config = configs[0]
    if position_max > config['repeatability_position_tolerance_m']:
        failures.append('maximum aligned 3D position difference exceeds declared tolerance')
    if speed_max > config['repeatability_speed_tolerance_m_s']:
        failures.append('maximum aligned speed difference exceeds declared tolerance')
    result.update(passed=not failures, samples=len(rows_a), physics_hz=120,
        max_position_difference_m=position_max, max_speed_difference_m_s=speed_max,
        max_position_difference_tick=position_errors.index(position_max) + 1,
        max_speed_difference_tick=speed_errors.index(speed_max) + 1,
        acceptance=dict(position_tolerance_m=config['repeatability_position_tolerance_m'],
                        speed_tolerance_m_s=config['repeatability_speed_tolerance_m_s'],
                        minimum_true_complete_laps_per_run=1))
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-a', type=Path, required=True)
    parser.add_argument('--run-b', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        directories = [resolve_run(args.run_a), resolve_run(args.run_b)]
    except ValueError as error:
        parser.error(str(error))
    config = dict(schema_version=1, study='highway_loop_repeatability',
                  inputs=[dict(run_id=path.name, relative_path=path.relative_to(ROOT).as_posix())
                          for path in directories])
    loaded, load_error = [], None
    try:
        for directory, input_record in zip(directories, config['inputs']):
            run = load_run(directory)
            loaded.append(run)
            input_record['file_hashes'] = run['file_hashes']
    except (OSError, ValueError) as error:
        load_error = repr(error)
    output, manifest = run_package('highway_loop_repeatability', config, SOURCES)
    manifest['input_evidence'] = config['inputs']
    result = (compare_evidence(*loaded) if load_error is None else
              dict(passed=False, failures=[load_error], run_ids=[path.name for path in directories]))
    result['input_evidence'] = config['inputs']
    finish(output, manifest, result)
    print(json.dumps(dict(output=str(output), **result), indent=2, allow_nan=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
