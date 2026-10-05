from copy import deepcopy
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from traffic.lane_validation import assess_lane, footprint_lateral_bound, validate_config


CONFIG = json.loads((Path(__file__).resolve().parents[1]/'experiments/physics-lane-following.json').read_text())


def fixture():
    rows = []
    for tick in range(1, 6001):
        progress = min(100, max(0, (tick-240)/120*3))
        rows.append(dict(tick=tick, phase='settle' if tick <= 240 else 'drive', sim_time_s=tick/120,
                         position_m=[progress, 0, 1], progress_m=progress, speed_m_s=0,
                         lateral_error_m=0, footprint_lateral_bound_m=.9, upright_z=1,
                         wheel_on_ground=[True]*4, control={'is_fallback': False},
                         driver_diagnostics={'fallback': False}))
    return rows


def assess(rows):
    return assess_lane(rows, CONFIG, route_length_m=100, lane_width_m=3.6, contact_events=[])


def test_frozen_config():
    assert validate_config(CONFIG) is CONFIG


@pytest.mark.parametrize('key,value', [('physics_hz', 60), ('target_speed_m_s', 10), ('repeats', True),
    ('drive_s', 500), ('max_process_seconds', math.inf), ('route_ids', ['straight100'])])
def test_bad_config(key, value):
    config = deepcopy(CONFIG)
    config[key] = value
    with pytest.raises(ValueError):
        validate_config(config)


def test_complete_ideal_trace_passes():
    assert assess(fixture())['passed']


@pytest.mark.parametrize('key,value,gate', [('lateral_error_m', .51, 'lane_max'),
    ('footprint_lateral_bound_m', 1.81, 'no_road_departure'), ('upright_z', .9, 'upright')])
def test_single_bad_sample_fails(key, value, gate):
    rows = fixture()
    rows[300][key] = value
    assert not assess(rows)['gates'][gate]


def test_missing_tick_and_nan_fail():
    rows = fixture()
    assert not assess(rows[:-1])['passed']
    rows[300]['lateral_error_m'] = math.nan
    assert not assess(rows)['gates']['finite_telemetry']


def test_stop_at_start_is_not_route_completion():
    rows = fixture()
    for r in rows:
        r['progress_m'] = 0
    assert not assess(rows)['gates']['endpoint']


def test_wrong_spawn_cannot_pass_completion():
    rows = fixture()
    for row in rows:
        row['position_m'][0] = row['progress_m'] = 100
    result = assess(rows)
    assert not result['gates']['initial_station']
    assert not result['gates']['forward_progress']


@pytest.mark.parametrize('support', [[], [True]*3, [1]*4])
def test_bad_support_schema(support):
    rows = fixture()
    rows[0]['wheel_on_ground'] = support
    assert not assess(rows)['gates']['wheel_support_schema']


def test_departure_during_settle_fails():
    rows = fixture()
    rows[0]['footprint_lateral_bound_m'] = 10
    assert not assess(rows)['gates']['no_road_departure']


def test_hold_and_support_gaps():
    rows = fixture()
    rows[-1]['speed_m_s'] = .051
    for i in range(300, 313):
        rows[i]['wheel_on_ground'] = [False]*4
    result = assess(rows)
    assert not result['gates']['hold_speed']
    assert not result['gates']['support_gap']


def test_contact_even_if_centerline_good():
    result = assess_lane(fixture(), CONFIG, route_length_m=100, lane_width_m=3.6,
                         contact_events=[{'collider0': 'car', 'collider1': 'barrier'}])
    assert not result['gates']['no_rigid_contact']


def test_box_envelope_straight_curved_and_rotated():
    projection = SimpleNamespace(lateral_error_m=.1, point=SimpleNamespace(yaw_rad=0))
    state = {'quaternion_xyzw': [0, 0, 0, 1]}
    assert footprint_lateral_bound(state, projection, [4.8, 1.8, 1.4], 0) == pytest.approx(1.0)
    assert 1.0 < footprint_lateral_bound(state, projection, [4.8, 1.8, 1.4], 1/60) < 1.1
    state['quaternion_xyzw'] = [0, 0, math.sin(math.pi/4), math.cos(math.pi/4)]
    assert footprint_lateral_bound(state, projection, [4.8, 1.8, 1.4], 0) == pytest.approx(2.5)
