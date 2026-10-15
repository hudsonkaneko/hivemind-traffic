"""No local runtimes/assets required to exercise setup failure paths."""
import os
from pathlib import Path
from unittest.mock import patch

import pytest

from experiments.loop_showcase_config import make_config, validate_config
from hivemind.launcher import SetupError
from hivemind.showcase_setup import check_showcase
from scripts.demo_loop_showcase import main, next_run_config


def fixture_tree(tmp_path):
    runtime = tmp_path / 'external runtime'
    runtime.mkdir()
    (runtime / ('python.bat' if os.name == 'nt' else 'python.sh')).touch()
    (runtime / 'VERSION').write_text('test-runtime')
    road = tmp_path / 'highway_usd/_v02'
    road.mkdir(parents=True)
    for name in ('highway_v02.usda', 'navigation.json', 'manifest.json', 'parameters.json'):
        (road / name).touch()
    return runtime, {'isaac_root': str(runtime)}


def test_missing_prepared_vehicle_is_a_failure_not_a_success(tmp_path, monkeypatch):
    monkeypatch.delenv('HIVEMIND_ISAAC_ROOT', raising=False)
    runtime, local = fixture_tree(tmp_path)
    with pytest.raises(SetupError, match='--vehicle public'):
        check_showcase(tmp_path, make_config(), local)


def test_public_needs_factory_but_never_private_assets(tmp_path, monkeypatch):
    monkeypatch.delenv('HIVEMIND_ISAAC_ROOT', raising=False)
    runtime, local = fixture_tree(tmp_path)
    cfg = make_config('v09', vehicle='public')
    validate_config(cfg)
    assert cfg['vehicle_directory'] is None
    with pytest.raises(SetupError, match='Factory'):
        check_showcase(tmp_path, cfg, local)
    factory = runtime / 'extscache/omni.physx.vehicle-1.0/omni/physxvehicle/scripts/helpers/Factory.py'
    factory.parent.mkdir(parents=True)
    factory.touch()
    _, report = check_showcase(tmp_path, cfg, local)
    assert report['required_files_available']
    assert report['vehicle_profile'] == 'public'
    assert not (tmp_path / 'vehicles').exists()
    (tmp_path / cfg['source_directory'] / 'navigation.json').unlink()
    with pytest.raises(SetupError, match='navigation.json'):
        check_showcase(tmp_path, cfg, local)


def test_public_survives_sidebar_restart():
    cfg = make_config('v09', vehicle='public')
    updated = next_run_config(cfg, dict(cfg['controls'], traffic_count=12))
    assert updated['vehicle_profile'] == 'public'
    assert updated['vehicle_directory'] is None
    validate_config(updated)


def test_unknown_vehicle_and_silent_model_changes_rejected():
    with pytest.raises(ValueError):
        make_config(vehicle='automatic')
    cfg = make_config(vehicle='public')
    cfg['vehicle_directory'] = 'private-car'
    with pytest.raises(ValueError):
        validate_config(cfg)
    assert 'vehicle_profile' not in make_config()  # historical default unchanged


def test_check_returns_nonzero_and_never_launches_simulation(capsys):
    with patch('sys.argv', ['demo_loop_showcase.py', '--check']), \
            patch('scripts.demo_loop_showcase.check_showcase', side_effect=SetupError('missing vehicle')), \
            patch('scripts.demo_loop_showcase.run_once') as run:
        assert main() == 2
        run.assert_not_called()
    assert 'Setup needed: missing vehicle' in capsys.readouterr().err
