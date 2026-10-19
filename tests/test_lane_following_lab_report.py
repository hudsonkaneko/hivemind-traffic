"""Package-integrity tests with synthetic files, never native simulation or training.

Numerical episode assessment has its own tests. Matrix tests stub only that
assessment so missing, mismatched and tampered packages cannot hide behind it.
"""
from copy import deepcopy
import hashlib
import json

import pytest

from experiments import lane_following_lab_validation as assessment
from experiments import report_lane_following_lab as report
from experiments import verify_lane_following_lab as supervisor

REPO_ROOT = supervisor.ROOT
CONFIG = json.loads((REPO_ROOT / 'experiments/lab-lane-following-validation.json').read_text(encoding='utf-8'))
ROUTE_BYTES = (REPO_ROOT / CONFIG['routes_file']).read_bytes()
EPISODES = {
    'nominal': [('nominal', 101, 'success')],
    'visible': [('nominal', 101, 'success')],
    'perturbed': [('corner-0', 1101, 'success'), ('corner-1', 1102, 'success')],
    'diagnostics': [('zero-drive-timeout', 101, 'time_limit'), ('hard-steer-departure', 101, 'lane_departure')],
    'contact-control': [('contact-positive-control', 101, 'rigid_contact')],
}


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, allow_nan=False), encoding='utf-8')


def load(path):
    return json.loads(path.read_text(encoding='utf-8'))


def rehash(directory, manifest=None, *, windows_keys=False):
    manifest = load(directory / 'manifest.json') if manifest is None else manifest
    manifest['output_hashes'] = {
        (p.relative_to(directory).as_posix().replace('/', '\\') if windows_keys else p.relative_to(directory).as_posix()):
        hashlib.sha256(p.read_bytes()).hexdigest()
        for p in directory.rglob('*') if p.is_file() and p.name != 'manifest.json'
    }
    write_json(directory / 'manifest.json', manifest)


@pytest.fixture
def package_factory(tmp_path, monkeypatch):
    monkeypatch.setattr(report, 'ROOT', tmp_path)
    monkeypatch.setattr(supervisor, 'ROOT', tmp_path)
    canonical_routes = tmp_path / CONFIG['routes_file']
    canonical_routes.parent.mkdir(parents=True)
    canonical_routes.write_bytes(ROUTE_BYTES)

    def create(label='straight100-nominal', *, route='straight100', profile='nominal'):
        directory = tmp_path / 'outputs/lab_lane_following' / label
        directory.mkdir(parents=True)
        config = dict(deepcopy(CONFIG), route_id=route, profile=profile)
        source_hashes = {}
        for relative in supervisor.source_files():
            path = directory / 'source' / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            content = ROUTE_BYTES if relative == config['routes_file'] else ('synthetic captured source: ' + relative).encode()
            path.write_bytes(content)
            source_hashes[relative] = hashlib.sha256(content).hexdigest()
        episodes = []
        for index, (name, seed, reason) in enumerate(EPISODES[profile]):
            corner = config['perturbation_corners'][index] if profile == 'perturbed' else [0., 0.]
            reset = {'reset_spec': {'seed': seed, 'episode_index': 0,
                                   'lateral_offset_m': corner[0], 'yaw_error_rad': corner[1]}}
            write_json(directory / f'{name}-reset.json', reset)
            write_json(directory / f'{name}-trajectory.json', [{'synthetic': True}])
            episodes.append({'label': name, 'seed': seed, 'expected_reason': reason,
                             'trajectory_file': f'{name}-trajectory.json', 'reset_file': f'{name}-reset.json',
                             'control_steps': 120, 'step_wall_seconds': 2., 'loop_wall_seconds': 3.})
        scene = {'passed': True, 'static_hashes_unchanged': True}
        log = '\n'.join(config['known_baseline_startup_errors'] + ['LAB_LANE_TASK_READY'])
        summary = dict(passed=True, training_started=False, optimizer_steps=0, episodes=episodes,
                       wall_process_seconds=6., gpu_peak_mib=2000., exit_code=0, stop_reason=None,
                       source_changed_during_run=[], gpu_monitor_errors=[], viewport_cleanup={'passed': True},
                       kit_after={'available': True, 'count': 0}, scene_contract=scene,
                       contact_reporting={'cleanup': {'passed': True}},
                       **supervisor.classify_runtime_log(log, config['known_baseline_startup_errors']))
        if profile == 'diagnostics':
            summary['diagnostic_checks'] = dict.fromkeys([
                'same_seed_native_reset', 'different_seed_reset', 'signed_native_pose_reset',
                'invalid_actions_rejected_without_physics', 'finite_clipping_and_brake_priority'], True)
        if profile == 'visible':
            summary.update(preview_complete=True, preview_files=[f'preview-{i}.png' for i in range(4)])
            for name in summary['preview_files']:
                (directory / name).write_bytes(b'synthetic image inventory placeholder')
        for name, value in [('resolved-config.json', config), ('summary.json', summary),
                            ('probe-result.json', summary), ('scene-contract.json', scene), ('resets.json', [])]:
            write_json(directory / name, value)
        (directory / 'runtime.log').write_text(log, encoding='utf-8')
        manifest = {'status': 'passed', 'config': config, 'source_hashes': source_hashes,
                    'kit_before': {'available': True, 'count': 0}}
        rehash(directory, manifest)
        return directory

    return create


@pytest.fixture
def matrix(package_factory, monkeypatch):
    monkeypatch.setattr(assessment, 'assess_episode', lambda *args, **kwargs: {'passed': True})
    monkeypatch.setattr(assessment, 'compare_episode_traces', lambda *args: {'passed': True})
    result = {}
    for route in supervisor.ROUTES:
        for kind in ('nominal', 'repeat', 'perturbed'):
            label = f'{route}-{kind}'
            path = package_factory(label, route=route, profile='nominal' if kind == 'repeat' else kind)
            result[label] = path.relative_to(report.ROOT).as_posix()
    for label, route in [('diagnostics', 'straight100'), ('contact-control', 'straight100'), ('visible', 'left_r60')]:
        path = package_factory(label, route=route, profile=label)
        result[label] = path.relative_to(report.ROOT).as_posix()
    return result


def test_read_package_accepts_complete_portable_windows_inventory(package_factory):
    directory = package_factory()
    rehash(directory, windows_keys=True)
    actual, manifest, config, summary = report.read_package(directory)
    assert actual == directory and summary['training_started'] is False
    assert all('\\' not in key for key in manifest['output_hashes'])
    assert manifest['output_hashes']['source/' + config['routes_file']] == manifest['source_hashes'][config['routes_file']]


@pytest.mark.parametrize('filename', ['summary.json', 'resolved-config.json', 'nominal-trajectory.json',
                                    'nominal-reset.json', 'source/environments/lane_following_lab.py'])
def test_changed_captured_bytes_are_rejected(package_factory, filename):
    directory = package_factory()
    with (directory / filename).open('ab') as stream:
        stream.write(b'\n ')
    with pytest.raises(ValueError, match='hash mismatch'):
        report.read_package(directory)


@pytest.mark.parametrize('mode', ['empty', 'omit', 'extra_file', 'missing_file', 'missing_required'])
def test_partial_or_unrecorded_artifact_inventory_rejected(package_factory, mode):
    directory = package_factory()
    manifest = load(directory / 'manifest.json')
    if mode == 'empty':
        manifest['output_hashes'] = {}
    elif mode == 'omit':
        manifest['output_hashes'].pop('nominal-trajectory.json')
    elif mode == 'extra_file':
        (directory / 'unrecorded.txt').write_text('new bytes', encoding='utf-8')
    else:
        (directory / 'runtime.log').unlink()
        if mode == 'missing_required':
            manifest['output_hashes'].pop('runtime.log')
    write_json(directory / 'manifest.json', manifest)
    with pytest.raises(ValueError, match='inventory'):
        report.read_package(directory)


@pytest.mark.parametrize('mode', ['missing', 'extra', 'disagree'])
def test_source_inventory_must_be_complete_and_match_captured_bytes(package_factory, mode):
    directory = package_factory()
    manifest = load(directory / 'manifest.json')
    source = 'environments/lane_follow_sb3.py'
    if mode == 'missing':
        manifest['source_hashes'].pop(source)
    elif mode == 'extra':
        manifest['source_hashes']['unrecorded.py'] = '0' * 64
    else:
        manifest['source_hashes'][source] = '0' * 64
    write_json(directory / 'manifest.json', manifest)
    with pytest.raises(ValueError, match='[Ss]ource'):
        report.read_package(directory)


def test_duplicate_portable_names_and_traversal_rejected(package_factory):
    with pytest.raises(ValueError, match='Duplicate'):
        report.portable_keys({'source/a.py': 'x', 'source\\a.py': 'x'})
    directory = package_factory()
    outside = directory.parent / 'outside.json'
    outside.write_text('{}', encoding='utf-8')
    with pytest.raises(ValueError, match='escaping'):
        report.artifact_path(directory, '../outside.json', {'../outside.json': 'anything'})
    with pytest.raises(ValueError, match='recorded'):
        report.artifact_path(directory, 'nominal-trajectory.json', {})


def test_complete_twelve_cell_matrix_computes_rates_without_training(matrix):
    value = report.audit_matrix(matrix)
    assert value['passed'] is True and value['training_started'] is False and value['optimizer_steps'] == 0
    assert len(value['runs']) == 12 and set(value['repeats']) == set(supervisor.ROUTES)
    assert all(row['steps_per_step_wall_second'] == 60 for row in value['runs'])
    assert all(row['steps_per_episode_loop_wall_second'] == 40 for row in value['runs'])


@pytest.mark.parametrize('mode', ['missing', 'extra', 'duplicate', 'alias'])
def test_matrix_requires_distinct_complete_declared_packages(matrix, mode):
    first, second = 'straight100-nominal', 'straight100-repeat'
    if mode == 'missing':
        matrix.pop('visible')
    elif mode == 'extra':
        matrix['another-trial'] = 'outputs/lab_lane_following/another'
    elif mode == 'duplicate':
        matrix[second] = matrix[first]
    else:
        matrix[second] = matrix[first] + '/../straight100-nominal'
    with pytest.raises(ValueError, match='distinct|reuses'):
        report.audit_matrix(matrix)


@pytest.mark.parametrize('label,key,value', [
    ('straight100-nominal', 'route_id', 'left_r60'),
    ('straight100-nominal', 'profile', 'perturbed'),
    ('visible', 'route_id', 'right_r60'),
    ('contact-control', 'route_id', 'left_r60'),
])
def test_package_cannot_be_relabeled_into_another_matrix_cell(matrix, label, key, value):
    directory = report.ROOT / matrix[label]
    config, manifest = load(directory / 'resolved-config.json'), load(directory / 'manifest.json')
    config[key] = value
    write_json(directory / 'resolved-config.json', config)
    manifest['config'] = config
    rehash(directory, manifest)
    with pytest.raises(ValueError, match='declared'):
        report.audit_matrix(matrix)


@pytest.mark.parametrize('mode', ['omit_episode', 'wrong_seed', 'wrong_reason', 'wrong_corner', 'trace_escape'])
def test_successful_subset_or_changed_reset_cannot_satisfy_perturbed_cell(matrix, mode):
    directory = report.ROOT / matrix['straight100-perturbed']
    summary = load(directory / 'summary.json')
    if mode == 'omit_episode':
        summary['episodes'].pop()
    elif mode == 'wrong_seed':
        summary['episodes'][0]['seed'] = 2101
    elif mode == 'wrong_reason':
        summary['episodes'][0]['expected_reason'] = 'time_limit'
    elif mode == 'trace_escape':
        summary['episodes'][0]['trajectory_file'] = '../other.json'
    else:
        path = directory / 'corner-0-reset.json'
        reset = load(path)
        reset['reset_spec']['lateral_offset_m'] = 0.
        write_json(path, reset)
    write_json(directory / 'summary.json', summary)
    rehash(directory)
    with pytest.raises(ValueError, match='declared|artifact names'):
        report.audit_matrix(matrix)


def test_matrix_rejects_different_captured_runtime_sources(matrix):
    directory = report.ROOT / matrix['straight100-repeat']
    source = 'environments/lane_following_lab.py'
    path = directory / 'source' / source
    path.write_text('different source version', encoding='utf-8')
    manifest = load(directory / 'manifest.json')
    manifest['source_hashes'][source] = hashlib.sha256(path.read_bytes()).hexdigest()
    rehash(directory, manifest)
    with pytest.raises(ValueError, match='different runtime sources'):
        report.audit_matrix(matrix)


@pytest.mark.parametrize('field,value', [
    ('training_started', True), ('optimizer_steps', False), ('optimizer_steps', 1),
    ('exit_code', 1), ('stop_reason', 'deadline'), ('source_changed_during_run', ['source.py']),
    ('gpu_monitor_errors', ['sampling failed']), ('viewport_cleanup', {'passed': False}),
    ('contact_reporting', {'cleanup': {'passed': False}}),
    ('kit_after', {'available': True, 'count': 1}),
])
def test_explicit_runtime_failure_cannot_hide_behind_saved_pass_flag(matrix, field, value):
    directory = report.ROOT / matrix['straight100-nominal']
    summary = load(directory / 'summary.json')
    summary[field] = value
    write_json(directory / 'summary.json', summary)
    rehash(directory)
    assert report.audit_matrix(matrix)['passed'] is False


def test_unexpected_native_error_is_reclassified_from_log(matrix):
    directory = report.ROOT / matrix['straight100-nominal']
    with (directory / 'runtime.log').open('a', encoding='utf-8') as stream:
        stream.write('\n[Error] [physx] native failure\n')
    rehash(directory)
    assert report.audit_matrix(matrix)['passed'] is False


def test_native_cp1252_log_byte_preserves_strict_error_classification(matrix):
    directory = report.ROOT / matrix['straight100-nominal']
    with (directory / 'runtime.log').open('ab') as stream:
        stream.write(b'\n[Info] Native status \x97 ready\n')
    rehash(directory)
    result = report.audit_matrix(matrix)
    assert result['passed'] is True
    row = next(row for row in result['runs'] if row['label'] == 'straight100-nominal')
    assert row['known_startup_error_count'] == 2
    assert row['unexpected_runtime_error_count'] == 0
    # Invalid encoding must not hide an actual native error either.
    with (directory / 'runtime.log').open('ab') as stream:
        stream.write(b'[Error] [physx] Failed \x97 native state\n')
    summary = load(directory / 'summary.json')
    summary.update(supervisor.classify_runtime_log(
        (directory / 'runtime.log').read_text(encoding='utf-8', errors='replace'),
        CONFIG['known_baseline_startup_errors']))
    write_json(directory / 'summary.json', summary)
    rehash(directory)
    result = report.audit_matrix(matrix)
    assert result['passed'] is False
    row = next(row for row in result['runs'] if row['label'] == 'straight100-nominal')
    assert row['known_startup_error_count'] == 2
    assert row['unexpected_runtime_error_count'] == 1


def test_missing_diagnostic_check_and_false_repeat_fail_entire_matrix(matrix, monkeypatch):
    directory = report.ROOT / matrix['diagnostics']
    summary = load(directory / 'summary.json')
    summary['diagnostic_checks'].pop('signed_native_pose_reset')
    write_json(directory / 'summary.json', summary)
    rehash(directory)
    assert report.audit_matrix(matrix)['passed'] is False
    monkeypatch.setattr(assessment, 'compare_episode_traces', lambda *args: {'passed': False})
    result = report.audit_matrix(matrix)
    assert result['passed'] is False and all(not row['passed'] for row in result['repeats'].values())
