import math
import copy
import json
from pathlib import Path
import subprocess
import sys
import pytest

from environments.physics_vehicle_contract import (
    ACTION_NAMES, OBSERVATION_NAMES, bounded_action, observation_values, validate_config,
    require_finite_telemetry,
)
from traffic.driver_control import AppliedControl


def test_action_units_and_brake_priority():
    assert ACTION_NAMES == ('steering_rad', 'throttle', 'brake')
    assert bounded_action([0.2, 0.4, 0]) == (0.2, 0.4, 0)
    assert bounded_action([0.2, 0.4, 0.01]) == (0.2, 0.0, 0.01)


@pytest.mark.parametrize('values, expected', [
    ([3, 3, -3], (0.5, 1.0, 0.0)),
    ([-3, -3, 3], (-0.5, 0.0, 1.0)),
])
def test_action_clipping(values, expected):
    assert bounded_action(values) == expected


@pytest.mark.parametrize('values', [
    [math.nan, 0, 0], [0, math.inf, 0], [0, 0, -math.inf],
    [0, True, 0], [0, '1', 0], [0, 0], [0, 0, 0, 0],
])
def test_bad_actions_abort_before_physics(values):
    with pytest.raises(ValueError):
        bounded_action(values)


def test_observation_layout_is_explicit_simulator_state():
    state = dict(position_m=[1, 2, 3], yaw_rad=0.5, velocity_m_s=[4, 5, 6], upright_z=1)
    control = AppliedControl(0.1, 0.2, 0.0, 'accepted', 0, False)
    assert len(OBSERVATION_NAMES) == 11
    assert observation_values(state, control) == (1, 2, 3, 0.5, 4, 5, 6, 1, 0.1, 0.2, 0)
    state['yaw_rad'] = math.nan
    with pytest.raises(ValueError):
        observation_values(state, control)


def test_environment_package_does_not_load_traffic_backend():
    code = "import environments.physics_vehicle_contract, sys; assert 'environments.highway_parallel_env' not in sys.modules; assert 'traci' not in sys.modules; assert 'pettingzoo' not in sys.modules"
    subprocess.run([sys.executable, '-c', code], check=True,
                   cwd=Path(__file__).resolve().parents[1])


def test_legacy_environment_exports_are_preserved():
    from environments import HighwayParallelEnv, parallel_env
    from environments.highway_parallel_env import HighwayParallelEnv as DirectImport
    assert HighwayParallelEnv is DirectImport
    assert callable(parallel_env)


def test_frozen_lab_config_is_valid():
    path = Path(__file__).resolve().parents[1] / 'experiments/physics-lab.json'
    config = json.loads(path.read_text())
    assert validate_config(config) is config
    for key, value in [('repeats', 1), ('physics_hz', 60), ('control_hz', 120),
                       ('seed', True), ('command_ttl_ticks', 100),
                       ('target_speed_m_s', 6.0), ('max_process_seconds', 5)]:
        broken = copy.deepcopy(config)
        broken[key] = value
        with pytest.raises(ValueError):
            validate_config(broken)


def test_finite_trace_validates_controls_and_nested_arrays_not_only_observations():
    row = {'tick': 2, 'state': [1.0, [2.0, 3.0]], 'control': {'torques': [0, 70.0]},
           'reason': 'accepted', 'terminated': False, 'sequence': None}
    require_finite_telemetry([row])
    row['control']['torques'][1] = math.nan
    with pytest.raises(ValueError, match=r'control.torques\[1\]'):
        require_finite_telemetry([row])
    row['control']['torques'][1] = 1.0
    row['state'][1][0] = math.inf
    with pytest.raises(ValueError, match=r'state\[1\]\[0\]'):
        require_finite_telemetry([row])


def test_provenance_resolves_imported_runtime_not_machine_specific_path(tmp_path):
    from types import SimpleNamespace
    from scripts.probe_vehicle_lab import lab_runtime_provenance
    module = tmp_path / 'package' / '__init__.py'
    module.parent.mkdir()
    module.touch()
    result = lab_runtime_provenance(SimpleNamespace(__file__=str(module)),
                                    SimpleNamespace(__version__='test-version'))
    assert result['isaaclab_module_file'] == str(module.resolve())
    assert result['torch_version'] == 'test-version'
    assert result['python_version']
    # Pytest artifacts intentionally live inside this repository, so discovery
    # finds this checkout rather than assuming an IsaacLab installation path.
    assert result['isaaclab_checkout'] == str(Path(__file__).resolve().parents[1])
    assert len(result['isaaclab_git_commit']) == 40
