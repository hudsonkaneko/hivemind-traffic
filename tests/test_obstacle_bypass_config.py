from copy import deepcopy
from dataclasses import asdict
import json
from pathlib import Path

import pytest

from experiments.obstacle_bypass_config import FIXTURE, validate_config
from traffic.lidar_braking import LidarBrakeConfig
from traffic.obstacle_bypass import BypassConfig
from traffic.path_following import FollowerConfig


def test_checked_in_fixture_is_frozen():
    config = json.loads((Path(__file__).resolve().parents[1]/'experiments/physics-obstacle-bypass.json').read_text())
    assert validate_config(config) == FIXTURE


def resolved():
    return dict(deepcopy(FIXTURE), mode='pass', gui=False, camera='follow', points=False,
        capture=True, paced=False, braking=asdict(LidarBrakeConfig()),
        follower=asdict(FollowerConfig()), planner=asdict(BypassConfig()))


@pytest.mark.parametrize('key,value', [('barrier_x_m', 46), ('target_speed_m_s', 6),
    ('seed', True), ('minimum_clearance_m', 0), ('hold_s', 1), ('unexpected', 0)])
def test_reject_ignored_or_changed_fixture_values(key, value):
    config = dict(FIXTURE)
    config[key] = value
    with pytest.raises(ValueError):
        validate_config(config)


def test_resolved_settings_are_reproducible_and_bounded():
    assert validate_config(resolved(), resolved=True)
    for key, value in [('mode', 'racing'), ('camera', 'unknown'), ('gui', 1),
                       ('planner', dict(asdict(BypassConfig()), target_speed_m_s=6))]:
        config = resolved()
        config[key] = value
        with pytest.raises(ValueError):
            validate_config(config, resolved=True)
