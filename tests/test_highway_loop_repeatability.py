"""Pure repeatability comparison, provenance checks and negative controls."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path

import pytest

from experiments.highway_loop_config import make_config
from experiments import highway_loop_repeatability as repeat


@pytest.fixture(scope='module')
def sample_rows():
    config = make_config('lap35', gui=False, paced=False)
    rows = []
    speed = 15.0
    for index in range(config['duration_s'] * 120):
        phase = ('settle' if index < 240 else 'drive' if index < 27240 else 'brake')
        distance = max(0, min(index + 1 - 240, 27000)) * speed / 120
        angle = -.04 + distance / 500
        rows.append(dict(tick=index + 1, applied_tick=index, sim_time_s=(index + 1) / 120,
            phase=phase, position_m=[500 * math.cos(angle), 500 * math.sin(angle), 1.0],
            speed_m_s=speed if phase == 'drive' else 0.0, progress_m=distance,
            completed_laps=int(distance // (math.tau * 500))))
    return rows


@pytest.fixture
def pair(sample_rows):
    config = make_config('lap35', gui=False, paced=False)
    source_hash = hashlib.sha256(b'fixture source').hexdigest()

    def evidence(run_id):
        return dict(run_id=run_id, config=deepcopy(config), load_errors=[],
            trajectory=sample_rows,
            scene_contract=dict(vehicle=dict(factory_sha256='5' * 64)),
            manifest=dict(status='passed', finished_utc='2026-10-09T00:00:00Z',
                isaac_version='6.0.1-rc.7+release.42383.32955d8d.gl', python='3.11.9',
                platform='Windows-10-10.0.26200-SP0', logical_cpus=32,
                gpu_before=dict(name='NVIDIA GeForce RTX 5070', driver='610.74',
                                total_mib=12227.0, used_mib=2914.0, utilization_percent=1.0),
                config=deepcopy(config), source_hashes={'fixture.py': source_hash},
                command=['isaac-python', 'physics_highway_loop.py', '--output', f'outputs/highway_loop/{run_id}']),
            summary=dict(passed=True, exit_code=0, source_changed_during_run=[], completed_laps=1))
    return evidence('run-a'), evidence('run-b')


def edit_row(run, index, **changes):
    run['trajectory'] = list(run['trajectory'])
    run['trajectory'][index] = dict(run['trajectory'][index], **changes)


def test_identical_independent_complete_laps_pass(pair):
    before = deepcopy(pair[0]['summary'])
    result = repeat.compare_evidence(*pair)
    assert result['passed']
    assert result['samples'] == 28200 and result['physics_hz'] == 120
    assert result['max_position_difference_m'] == result['max_speed_difference_m_s'] == 0
    assert result['input_metrics']['a']['measured_complete_laps'] == 1
    assert result['input_metrics']['a']['measured_drive_progress_m'] == pytest.approx(3375)
    assert pair[0]['summary'] == before


def test_declared_numerical_tolerance_allows_small_deviation(pair):
    row = pair[1]['trajectory'][1000]
    edit_row(pair[1], 1000, position_m=[*row['position_m'][:2], 1.01],
             speed_m_s=row['speed_m_s'] + .01)
    result = repeat.compare_evidence(*pair)
    assert result['passed']
    assert result['max_position_difference_m'] == pytest.approx(.01)
    assert result['max_speed_difference_m_s'] == pytest.approx(.01)


@pytest.mark.parametrize('field', ['position', 'speed'])
def test_tolerance_negative_control(pair, field):
    row = pair[1]['trajectory'][1000]
    changes = ({'position_m': [*row['position_m'][:2], 1.0201]} if field == 'position' else
               {'speed_m_s': row['speed_m_s'] + .0201})
    edit_row(pair[1], 1000, **changes)
    result = repeat.compare_evidence(*pair)
    assert not result['passed']
    assert any(field in failure and 'tolerance' in failure for failure in result['failures'])


def test_same_run_is_not_independent(pair):
    result = repeat.compare_evidence(pair[0], pair[0])
    assert not result['passed']
    assert any('independent' in failure for failure in result['failures'])


@pytest.mark.parametrize('change', [
    'different_config', 'missing_config', 'source_mismatch', 'empty_source_hashes',
    'bad_source_hash', 'missing_manifest', 'running_manifest', 'missing_finish',
    'manifest_config_mismatch', 'missing_launch', 'wrong_launch_id', 'summary_failed',
    'exit_failed', 'exit_boolean', 'source_changed', 'missing_summary', 'zero_summary_laps',
    'invented_summary_laps', 'load_integrity_error',
])
def test_provenance_negative_controls(pair, change):
    run = pair[1]
    if change == 'different_config': run['config']['capture'] = True
    elif change == 'missing_config': run.pop('config')
    elif change == 'source_mismatch': run['manifest']['source_hashes']['fixture.py'] = 'a' * 64
    elif change == 'empty_source_hashes': run['manifest']['source_hashes'] = {}
    elif change == 'bad_source_hash': run['manifest']['source_hashes']['fixture.py'] = 'not a hash'
    elif change == 'missing_manifest': run.pop('manifest')
    elif change == 'running_manifest': run['manifest']['status'] = 'running'
    elif change == 'missing_finish': run['manifest'].pop('finished_utc')
    elif change == 'manifest_config_mismatch': run['manifest']['config']['seed'] = 102
    elif change == 'missing_launch': run['manifest'].pop('command')
    elif change == 'wrong_launch_id': run['manifest']['command'][-1] = 'outputs/highway_loop/run-a'
    elif change == 'summary_failed': run['summary']['passed'] = False
    elif change == 'exit_failed': run['summary']['exit_code'] = 1
    elif change == 'exit_boolean': run['summary']['exit_code'] = False
    elif change == 'source_changed': run['summary']['source_changed_during_run'] = ['fixture.py']
    elif change == 'missing_summary': run.pop('summary')
    elif change == 'zero_summary_laps': run['summary']['completed_laps'] = 0
    elif change == 'invented_summary_laps': run['summary']['completed_laps'] = 4
    elif change == 'load_integrity_error': run['load_errors'] = ['hash mismatch']
    assert not repeat.compare_evidence(*pair)['passed']


@pytest.mark.parametrize('changes', [
    dict(tick=1), dict(tick=True), dict(applied_tick=0), dict(sim_time_s=1.0),
    dict(sim_time_s=math.nan), dict(position_m=[0, 0, 1]), dict(position_m=[500, 0]),
    dict(position_m=[math.inf, 0, 1]), dict(speed_m_s=math.nan), dict(speed_m_s=-1),
    dict(speed_m_s=True), dict(progress_m=math.nan), dict(phase='settle'),
])
def test_invalid_trajectory_records_fail(pair, changes):
    edit_row(pair[1], 1000, **changes)
    assert not repeat.compare_evidence(*pair)['passed']


@pytest.mark.parametrize('rows', [None, [], [{}]])
def test_missing_empty_truncated_trajectory_fails(pair, rows):
    pair[1]['trajectory'] = rows
    assert not repeat.compare_evidence(*pair)['passed']


def test_seam_only_profile_is_not_complete_lap_evidence(pair):
    for run in pair:
        run['config'] = make_config('seam35', gui=False, paced=False)
        run['manifest']['config'] = deepcopy(run['config'])
    result = repeat.compare_evidence(*pair)
    assert not result['passed']
    assert any('seam-only' in failure for failure in result['failures'])


def test_stationary_copied_positions_cannot_claim_complete_lap(pair):
    pair[1]['trajectory'] = [dict(row, position_m=[500 * math.cos(-.04), 500 * math.sin(-.04), 1])
                             for row in pair[1]['trajectory']]
    result = repeat.compare_evidence(*pair)
    assert not result['passed']
    assert any('physical drive-phase positions' in failure for failure in result['failures'])


def test_teleport_cannot_claim_lap(pair):
    row = pair[1]['trajectory'][1000]
    angle = -.04 + row['progress_m'] / 500 + .1
    edit_row(pair[1], 1000, position_m=[500 * math.cos(angle), 500 * math.sin(angle), 1])
    result = repeat.compare_evidence(*pair)
    assert not result['passed']
    assert any('route jump' in failure for failure in result['failures'])


def test_invented_recorded_progress_is_not_true_distance(pair):
    edit_row(pair[1], 27239, progress_m=4000)
    result = repeat.compare_evidence(*pair)
    assert not result['passed']
    assert any('recorded progress/laps disagree' in failure for failure in result['failures'])


def materialize(tmp_path, monkeypatch, evidence):
    monkeypatch.setattr(repeat, 'ROOT', tmp_path)
    directory = tmp_path / 'outputs' / 'highway_loop' / evidence['run_id']
    directory.mkdir(parents=True)
    source = directory / 'source' / 'fixture.py'
    source.parent.mkdir()
    source.write_bytes(b'fixture source')
    manifest = deepcopy(evidence['manifest'])
    manifest['output_hashes'] = {}
    documents = {'summary.json': evidence['summary'], 'trajectory.json': evidence['trajectory'],
                 'scene-contract.json': evidence['scene_contract'],
                 'resolved-config.json': evidence['config']}
    for name, document in documents.items():
        data = json.dumps(document, allow_nan=False).encode()
        (directory / name).write_bytes(data)
        manifest['output_hashes'][name] = hashlib.sha256(data).hexdigest()
    (directory / 'manifest.json').write_text(json.dumps(manifest), encoding='utf-8')
    return directory


def test_loading_validates_hashes_and_does_not_modify_sources(tmp_path, monkeypatch, pair):
    directory = materialize(tmp_path, monkeypatch, pair[0])
    before = {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
              for path in directory.iterdir() if path.is_file()}
    loaded = repeat.load_run(directory)
    assert loaded['load_errors'] == []
    assert set(loaded['file_hashes']) == set(repeat.INPUT_FILES)
    after = {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
             for path in directory.iterdir() if path.is_file()}
    assert before == after


@pytest.mark.parametrize('name', ['summary.json', 'trajectory.json', 'resolved-config.json', 'scene-contract.json', 'source/fixture.py'])
def test_input_or_source_tampering_is_recorded(tmp_path, monkeypatch, pair, name):
    directory = materialize(tmp_path, monkeypatch, pair[0])
    with (directory / name).open('ab') as stream:
        stream.write(b' ')
    loaded = repeat.load_run(directory)
    assert loaded['load_errors']
    assert not repeat.compare_evidence(loaded, pair[1])['passed']


def test_missing_file_is_not_accepted(tmp_path, monkeypatch, pair):
    directory = materialize(tmp_path, monkeypatch, pair[0])
    (directory / 'summary.json').unlink()
    with pytest.raises(ValueError, match='Missing'):
        repeat.load_run(directory)


def test_output_scope_is_exact_run_directory(tmp_path, monkeypatch):
    monkeypatch.setattr(repeat, 'ROOT', tmp_path)
    study = tmp_path / 'outputs' / 'highway_loop'
    run = study / 'run'
    (run / 'source').mkdir(parents=True)
    for invalid in (tmp_path, study, run / 'source', tmp_path / 'missing'):
        with pytest.raises(ValueError):
            repeat.resolve_run(invalid)
    assert repeat.resolve_run(run) == run.resolve()


def test_cli_records_a_fresh_report_with_input_fingerprints(tmp_path, monkeypatch, pair, capsys):
    directories = [materialize(tmp_path, monkeypatch, evidence) for evidence in pair]
    output = tmp_path / 'outputs' / 'highway_loop_repeatability' / 'new-report'
    seen = {}

    def package(study, config, sources):
        assert study == 'highway_loop_repeatability'
        assert sources == repeat.SOURCES
        output.mkdir(parents=True)
        seen['config'] = deepcopy(config)
        return output, {'status': 'running'}

    def finished(directory, manifest, result):
        assert directory == output
        seen['manifest'], seen['result'] = manifest, result

    monkeypatch.setattr(repeat, 'run_package', package)
    monkeypatch.setattr(repeat, 'finish', finished)
    assert repeat.main(['--run-a', str(directories[0]), '--run-b', str(directories[1])]) == 0
    assert seen['result']['passed']
    assert all(set(record['file_hashes']) == set(repeat.INPUT_FILES) for record in seen['config']['inputs'])
    assert seen['manifest']['input_evidence'] == seen['config']['inputs']
    assert json.loads(capsys.readouterr().out)['output'] == str(output)


def test_pure_compare_rejects_non_objects():
    assert not repeat.compare_evidence(None, {})['passed']


def test_finite_but_overflowing_pose_differences_fail_with_serializable_report(pair):
    for run, height in zip(pair, (1e308, -1e308)):
        row = run['trajectory'][1000]
        edit_row(run, 1000, position_m=[*row['position_m'][:2], height])
    result = repeat.compare_evidence(*pair)
    assert not result['passed']
    assert any('not finite' in reason for reason in result['failures'])
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize('field,value', [
    ('isaac_version', 'different build'), ('python', '3.12.0'), ('platform', 'different OS'),
    ('logical_cpus', 16), ('gpu_name', 'different GPU'), ('gpu_driver', 'different driver'),
    ('factory_sha256', 'a' * 64),
])
def test_runtime_and_machine_identity_must_match(pair, field, value):
    run = pair[1]
    if field == 'gpu_name': run['manifest']['gpu_before']['name'] = value
    elif field == 'gpu_driver': run['manifest']['gpu_before']['driver'] = value
    elif field == 'factory_sha256': run['scene_contract']['vehicle'][field] = value
    else: run['manifest'][field] = value
    result = repeat.compare_evidence(*pair)
    assert not result['passed']
    assert any('runtime/machine identity differs' in reason for reason in result['failures'])


@pytest.mark.parametrize('field', [
    'isaac_version', 'python', 'platform', 'logical_cpus', 'gpu_name', 'gpu_driver', 'factory_sha256',
])
def test_both_missing_identity_values_do_not_count_as_a_match(pair, field):
    for run in pair:
        if field == 'gpu_name': run['manifest']['gpu_before'].pop('name')
        elif field == 'gpu_driver': run['manifest']['gpu_before'].pop('driver')
        elif field == 'factory_sha256': run['scene_contract']['vehicle'].pop(field)
        else: run['manifest'].pop(field)
    assert not repeat.compare_evidence(*pair)['passed']


@pytest.mark.parametrize('change', ['empty_isaac', 'blank_driver', 'boolean_cpus', 'bad_factory_hash', 'no_scene'])
def test_runtime_identity_requires_valid_types_and_nonempty_values(pair, change):
    for run in pair:
        if change == 'empty_isaac': run['manifest']['isaac_version'] = ''
        elif change == 'blank_driver': run['manifest']['gpu_before']['driver'] = ' '
        elif change == 'boolean_cpus': run['manifest']['logical_cpus'] = True
        elif change == 'bad_factory_hash': run['scene_contract']['vehicle']['factory_sha256'] = 'invalid'
        elif change == 'no_scene': run.pop('scene_contract')
    assert not repeat.compare_evidence(*pair)['passed']


def test_dynamic_gpu_load_and_runtime_location_are_metadata_not_identity(pair):
    pair[1]['manifest']['gpu_before'].update(used_mib=3500.0, utilization_percent=15.0, wall=987.0)
    pair[1]['manifest']['command'][0] = 'another-location/same-runtime/python.bat'
    result = repeat.compare_evidence(*pair)
    assert result['passed']
    assert result['runtime_identities']['a'] == result['runtime_identities']['b']
