"""CPU checks for the frozen native protocol; no Kit or trainer is launched."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from experiments import verify_lane_following_lab as supervisor


@pytest.fixture
def config():
    value = json.loads((supervisor.ROOT / 'experiments/lab-lane-following-validation.json').read_text(encoding='utf-8'))
    return dict(value, profile='nominal', route_id='straight100')


def test_declared_routes_and_profiles_validate_without_mutating_protocol(config):
    original = deepcopy(config)
    for route in supervisor.ROUTES:
        for profile in supervisor.PROFILES:
            candidate = dict(config, route_id=route, profile=profile)
            assert supervisor.validate_config(candidate) == candidate
    assert config == original
    assert config['task']['stop_s_m'] == 60
    assert config['task']['episode_seconds'] == 35
    assert config['acceptance']['full_footprint_containment_required'] is True


@pytest.mark.parametrize('key,value', [
    ('schema_version', True), ('schema_version', 1.0), ('training', True), ('training', 0),
    ('physics_hz', 120.0), ('control_hz', 60.0), ('physics_hz', 60),
    ('route_ids', ['straight100']), ('route_id', 'unknown'), ('profile', 'train'),
    ('validation_seed', 101.0), ('validation_seed', 2101),
    ('perturbation_seeds', [1101.0, 1102]), ('perturbation_seeds', [1102, 1101]),
    ('routes_file', 'AGENTS.md'), ('routes_file', '../routes.json'),
    ('routes_file', 'scenarios/physics-road/../physics-road/routes.json'),
    ('max_process_seconds', True), ('max_process_seconds', 0), ('max_process_seconds', 601),
    ('max_process_seconds', float('nan')), ('max_process_seconds', float('inf')),
    ('gpu_memory_fraction_stop', 1.0),
    ('perturbation_corners', [[0.0, 0.0], [0.0, 0.0]]),
    ('known_baseline_startup_errors', []),
    ('known_baseline_startup_errors', ['[Error]']),
])
def test_changed_or_ambiguous_protocol_rejected(config, key, value):
    config[key] = value
    with pytest.raises(ValueError):
        supervisor.validate_config(config)


@pytest.mark.parametrize('section,key,value', [
    ('task', 'stop_s_m', 100.0), ('task', 'success_hold_seconds', 5.0),
    ('task', 'settle_seconds', 0.0), ('task', 'target_speed_m_s', 5.0),
    ('acceptance', 'full_episode_upright_z_min', 0.995),
    ('acceptance', 'full_footprint_containment_required', 1),
    ('acceptance', 'rigid_contacts_max', False),
    ('acceptance', 'maximum_unsupported_gap_s', 1.0),
    ('acceptance', 'driving_lateral_rms_max_m', 0.5),
    ('visible', 'window_width', 800), ('visible', 'viewport_height', 600.0),
    ('visible', 'capture_sim_seconds', [3, 12, 20]),
    ('visible', 'capture_sim_seconds', [0.0, 1.0, 2.0]),
    ('visible', 'inspection_hold_wall_seconds', 600.0),
])
def test_safety_task_and_evidence_thresholds_cannot_be_relaxed(config, section, key, value):
    config[section][key] = value
    with pytest.raises(ValueError):
        supervisor.validate_config(config)


def test_only_recorded_once_before_ready_startup_errors_are_classified_known():
    first, second = supervisor.KNOWN_STARTUP_ERRORS
    lines = [f'2026-10-19 10:00:00 {first}', f'[123ms] {second}', 'LAB_LANE_TASK_READY',
             first, '[Error] [physx] invalid actor']
    result = supervisor.classify_runtime_log('\n'.join(lines), supervisor.KNOWN_STARTUP_ERRORS)
    assert result['runtime_error_lines'] == lines[:2] + lines[3:]
    assert result['known_baseline_startup_errors'] == lines[:2]
    assert result['unexpected_runtime_errors'] == lines[3:]


def test_duplicate_or_altered_baseline_error_is_not_silenced():
    baseline = supervisor.KNOWN_STARTUP_ERRORS[0]
    lines = [baseline, baseline, baseline + ' additional failure', baseline.replace('OmniPlaybackAPI', 'OtherAPI')]
    result = supervisor.classify_runtime_log('\n'.join(lines), supervisor.KNOWN_STARTUP_ERRORS)
    assert result['known_baseline_startup_errors'] == [baseline]
    assert result['unexpected_runtime_errors'] == lines[1:]


def test_usd_reference_warnings_retained_even_without_error_severity():
    text = '[Warning] Unexpected reference count\nLAB_LANE_TASK_READY\n[Warning] Unexpected reference count'
    result = supervisor.classify_runtime_log(text, supervisor.KNOWN_STARTUP_ERRORS)
    assert result['runtime_error_lines'] == []
    assert result['usd_reference_warning_count'] == 2


def test_inventory_captures_entire_new_task_and_shared_physics_boundary():
    files = supervisor.source_files()
    assert len(files) == len(set(files))
    required = {
        'experiments/lab-lane-following-validation.json',
        'experiments/verify_lane_following_lab.py', 'experiments/lane_following_lab_validation.py',
        'scripts/probe_lane_following_lab.py', 'environments/lane_following_contract.py',
        'environments/lane_following_lab.py', 'environments/lane_following_scene.py',
        'environments/lane_follow_sb3.py', 'traffic/physx_vehicle.py',
        'traffic/physics_session.py', 'traffic/driver_control.py',
        'traffic/lane_geometry.py', 'traffic/path_following.py', 'traffic/vehicle_contacts.py',
        'usd/physical_scene.py', 'usd/scene_validation.py', 'scenarios/physics-road/routes.json',
    }
    assert required <= set(files)
    for relative in files:
        assert not Path(relative).is_absolute() and '..' not in Path(relative).parts
        resolved = (supervisor.ROOT / relative).resolve()
        assert resolved.is_relative_to(supervisor.ROOT) and resolved.is_file()
    assert all('CHANGELOG' not in path and not path.startswith('outputs/') for path in files)


def test_check_mode_cannot_create_package_or_launch_child(monkeypatch, capsys):
    def forbidden(*args, **kwargs):
        raise AssertionError('Read-only --check tried to launch or write a run')

    monkeypatch.setattr('sys.argv', ['verify_lane_following_lab', '--check'])
    monkeypatch.setattr(supervisor, 'isaac_runtime', lambda *args: Path('declared-runtime.bat'))
    monkeypatch.setattr(supervisor, 'load_config', lambda *args: {})
    monkeypatch.setattr(supervisor, 'run_package', forbidden)
    monkeypatch.setattr(supervisor.subprocess, 'Popen', forbidden)
    monkeypatch.setattr(supervisor, 'kit_process_snapshot', forbidden)
    assert supervisor.main() == 0
    result = json.loads(capsys.readouterr().out)
    assert result['launches_training'] is False
    assert result['config']['training'] is False
    assert set(result['sources']) == set(supervisor.source_files())


@pytest.mark.skipif(supervisor.os.name != 'nt', reason='Windows native inventory parser')
@pytest.mark.parametrize('stdout', ['null', '{}', '[true]', '[-1]', '[1.5]', 'not-json'])
def test_unavailable_or_malformed_kit_inventory_fails_closed(monkeypatch, stdout):
    monkeypatch.setattr(supervisor.subprocess, 'run', lambda *args, **kwargs: SimpleNamespace(stdout=stdout))
    result = supervisor.kit_process_snapshot()
    assert result['available'] is False and result['count'] is None


@pytest.mark.skipif(supervisor.os.name != 'nt', reason='Windows native inventory parser')
def test_kit_inventory_keeps_foreign_process_ids_without_killing_them(monkeypatch):
    monkeypatch.setattr(supervisor.subprocess, 'run', lambda *args, **kwargs: SimpleNamespace(stdout='[13, 9, 13]'))
    assert supervisor.kit_process_snapshot() == {'available': True, 'pids': [9, 13], 'count': 2}
