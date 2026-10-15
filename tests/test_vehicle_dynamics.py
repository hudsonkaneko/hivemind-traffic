"""CPU acceptance-check tests; synthetic traces do not validate vehicle physics."""
from copy import deepcopy
import json
import math
from pathlib import Path

import pytest

from traffic.dynamics_validation import assess_case, compare_repeats, rear_axle_position, validate_config, yaw_delta
from scripts.probe_vehicle_dynamics import phase_command


@pytest.fixture
def config():
    return json.loads((Path(__file__).parents[1] / 'experiments/physics-dynamics.json').read_text())


def synthetic_trace(case, config, *, radius_factor=1.0):
    """Produce exact planar test data, not a substitute physics model."""
    rows, x, y, yaw = [], 0.0, 0.0, 0.0
    dt = 1 / config['physics_hz']
    wheelbase = 3.2
    target = case['target_speed_m_s']
    for phase, seconds in case['phases']:
        for index in range(round(seconds / dt)):
            speed = target if phase in ('accelerate', 'turn_warmup', 'circle', 'coast') else 0.0
            if phase == 'brake':
                speed = target * max(0.0, 1 - (index + 1) * dt / 0.4)
            steering = case['steering_rad'] if phase in ('turn_warmup', 'circle') else 0.0
            if steering:
                radius = wheelbase / math.tan(steering) * radius_factor
                rear_x, rear_y = x - wheelbase / 2 * math.cos(yaw), y - wheelbase / 2 * math.sin(yaw)
                center_x, center_y = rear_x - radius * math.sin(yaw), rear_y + radius * math.cos(yaw)
                rear_speed = speed * abs(radius) / math.hypot(radius, wheelbase / 2)
                yaw += rear_speed / radius * dt
                x = center_x + radius * math.sin(yaw) + wheelbase / 2 * math.cos(yaw)
                y = center_y - radius * math.cos(yaw) + wheelbase / 2 * math.sin(yaw)
            else:
                x += speed * math.cos(yaw) * dt
                y += speed * math.sin(yaw) * dt
            rows.append(dict(tick=len(rows) + 1, sim_time_s=(len(rows) + 1) * dt,
                             phase=phase, position_m=[x, y, 1.0],
                             velocity_m_s=[speed * math.cos(yaw), speed * math.sin(yaw), 0.0],
                             speed_m_s=speed, yaw_rad=math.atan2(math.sin(yaw), math.cos(yaw)),
                             quaternion_xyzw=[0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)],
                             upright_z=1.0, wheel_on_ground=[True] * 4,
                             control=dict(steering_rad=steering, throttle=0.2 if phase in ('accelerate', 'turn_warmup', 'circle') else 0,
                                          brake=1.0 if phase in ('settle', 'brake', 'hold') else 0.0,
                                          is_fallback=False)))
    return rows


def test_frozen_config_is_bounded(config):
    assert validate_config(config) is config
    assert sum(seconds for c in config['cases'] for _, seconds in c['phases']) * config['repeats'] == 288
    assert [c['target_speed_m_s'] for c in config['cases'][:3]] == [1, 3, 6]


@pytest.mark.parametrize('index', range(6))
def test_accepts_complete_idealized_cases(config, index):
    case = config['cases'][index]
    result = assess_case(synthetic_trace(case, config), case, config, wheelbase_m=3.2)
    assert result['passed'], result
    assert result['metrics']['hold_duration_s'] == 5.0
    if case['kind'] == 'circle':
        assert result['metrics']['radius_relative_error'] < 0.00001


@pytest.mark.parametrize('failure,gate', [
    ('nan', 'finite_telemetry'), ('missing_field', 'finite_telemetry'),
    ('missing_sample', 'trace_complete'), ('wrong_tick', 'tick_phase_order'),
    ('wrong_phase', 'tick_phase_order'), ('bad_time', 'tick_phase_order'),
    ('speed', 'target_speed'), ('backwards', 'forward'),
    ('drift', 'straight_drift'), ('unstable', 'upright'), ('height', 'height'),
    ('airborne', 'grounded'), ('brake_not_stopped', 'braking'),
    ('brake_not_applied', 'brake_commands'), ('hold_speed', 'hold'),
    ('hold_drift', 'hold'), ('fallback', 'no_control_fallback'),
])
def test_rejects_invalid_straight_traces(config, failure, gate):
    case = config['cases'][1]
    rows = synthetic_trace(case, config)
    accelerate = [r for r in rows if r['phase'] == 'accelerate']
    brake = [r for r in rows if r['phase'] == 'brake']
    hold = [r for r in rows if r['phase'] == 'hold']
    if failure == 'nan': rows[0]['position_m'][0] = float('nan')
    elif failure == 'missing_field': rows[0].pop('quaternion_xyzw')
    elif failure == 'missing_sample': rows.pop()
    elif failure == 'wrong_tick': rows[0]['tick'] = 9
    elif failure == 'wrong_phase': rows[0]['phase'] = 'hold'
    elif failure == 'bad_time': rows[0]['sim_time_s'] += 1
    elif failure == 'speed': accelerate[-1]['speed_m_s'] = 0.0
    elif failure == 'backwards': accelerate[-1]['velocity_m_s'][0] = -3.0
    elif failure == 'drift': accelerate[-1]['position_m'][1] = 0.3
    elif failure == 'unstable': rows[0]['upright_z'] = 0.5
    elif failure == 'height': rows[0]['position_m'][2] = 4.0
    elif failure == 'airborne':
        for row in accelerate: row['wheel_on_ground'] = [False] * 4
    elif failure == 'brake_not_stopped': brake[-1]['speed_m_s'] = 0.5
    elif failure == 'brake_not_applied': brake[-1]['control']['brake'] = 0.0
    elif failure == 'hold_speed': hold[-1]['speed_m_s'] = 0.1
    elif failure == 'hold_drift': hold[-1]['position_m'][0] += 0.1
    elif failure == 'fallback': rows[0]['control']['is_fallback'] = True
    result = assess_case(rows, case, config, wheelbase_m=3.2)
    assert not result['passed']
    assert not result['gates'][gate]


def test_braking_limit_cannot_pass_merely_by_stopping_eventually(config):
    case = config['cases'][1]
    rows = synthetic_trace(case, config)
    first_brake = next(r for r in rows if r['phase'] == 'brake')
    first_brake['position_m'][0] += 10.0
    result = assess_case(rows, case, config, wheelbase_m=3.2)
    assert not result['gates']['braking']
    assert result['metrics']['brake_distance_m'] > 4.0


@pytest.mark.parametrize('index', [3, 4])
def test_rejects_circle_radius_outside_predeclared_tolerance(config, index):
    case = config['cases'][index]
    rows = synthetic_trace(case, config, radius_factor=1.2)
    result = assess_case(rows, case, config, wheelbase_m=3.2)
    assert not result['gates']['circle_radius']
    assert result['metrics']['radius_relative_error'] == pytest.approx(0.2, abs=1e-5)


def test_rejects_wrong_direction_and_steering(config):
    case = config['cases'][3]
    rows = synthetic_trace(config['cases'][4], config)
    result = assess_case(rows, case, config, wheelbase_m=3.2)
    assert not result['gates']['circle_turn_direction']
    assert not result['gates']['circle_steering']


def test_coast_requires_zero_actuation_not_idealized_drag(config):
    case = config['cases'][5]
    rows = synthetic_trace(case, config)
    coast = [r for r in rows if r['phase'] == 'coast']
    # Drift resistance is not calibrated, so speed-loss size is informational.
    coast[-1]['speed_m_s'] = 2.0
    result = assess_case(rows, case, config, wheelbase_m=3.2)
    assert result['passed']
    coast[-1]['control']['brake'] = 0.1
    assert not assess_case(rows, case, config, wheelbase_m=3.2)['gates']['coast_zero_actuation']


def test_repeat_comparison_exact_and_tolerated(config):
    rows = synthetic_trace(config['cases'][0], config)
    assert compare_repeats(rows, rows, config['acceptance'])['passed']
    second = deepcopy(rows)
    second[-1]['position_m'][0] += 0.0005
    assert compare_repeats(rows, second, config['acceptance'])['passed']
    second[-1]['position_m'][0] += 0.002
    assert not compare_repeats(rows, second, config['acceptance'])['passed']


@pytest.mark.parametrize('failure', ['short', 'phase', 'tick', 'time', 'nan', 'speed', 'yaw'])
def test_repeat_comparison_rejects_invalid_pairs(config, failure):
    first = synthetic_trace(config['cases'][0], config)
    second = deepcopy(first)
    if failure == 'short': second.pop()
    elif failure == 'phase': second[-1]['phase'] = 'other'
    elif failure == 'tick': second[-1]['tick'] += 1
    elif failure == 'time': second[-1]['sim_time_s'] += 1
    elif failure == 'nan': second[-1]['speed_m_s'] = float('nan')
    elif failure == 'speed': second[-1]['speed_m_s'] += 0.1
    elif failure == 'yaw': second[-1]['yaw_rad'] += 0.1
    assert not compare_repeats(first, second, config['acceptance'])['passed']


@pytest.mark.parametrize('field,value', [
    ('physics_hz', 60), ('control_hz', 30), ('repeats', 1),
    ('max_process_seconds', 301), ('max_total_simulated_seconds', 287),
    ('drive_torque_per_front_wheel_nm', float('nan')), ('command_ttl_ticks', 1),
    ('seed', True), ('schema_version', 2),
])
def test_config_rejects_invalid_globals(config, field, value):
    config[field] = value
    with pytest.raises(ValueError): validate_config(config)


@pytest.mark.parametrize('failure', ['name', 'duplicate', 'kind', 'speed', 'steering', 'phase', 'hold', 'limit'])
def test_config_rejects_invalid_cases(config, failure):
    case = config['cases'][0]
    if failure == 'name': case['name'] = '../bad'
    elif failure == 'duplicate': case['name'] = config['cases'][1]['name']
    elif failure == 'kind': case['kind'] = 'unknown'
    elif failure == 'speed': case['target_speed_m_s'] = 7
    elif failure == 'steering': case['steering_rad'] = 0.1
    elif failure == 'phase': case['phases'].pop(0)
    elif failure == 'hold': case['phases'][-1][1] = 4
    elif failure == 'limit': config['acceptance']['grounded_fraction_min'] = 2
    with pytest.raises(ValueError): validate_config(config)


def test_coordinate_and_angle_conventions():
    assert rear_axle_position(dict(position_m=[4, 2, 1], yaw_rad=0), 3.2) == [2.4, 2]
    assert rear_axle_position(dict(position_m=[4, 2, 1], yaw_rad=math.pi / 2), 3.2) == pytest.approx([4, 0.4])
    assert yaw_delta(math.pi - 0.01, -math.pi + 0.01) == pytest.approx(0.02)


def test_empty_trace_and_repeat_fail_cleanly(config):
    assert not assess_case([], config['cases'][0], config, wheelbase_m=3.2)['passed']
    assert not compare_repeats([], [], config['acceptance'])['passed']


@pytest.mark.parametrize('phase,throttle,brake,steering', [
    ('settle', 0, 1, 0), ('accelerate', 1, 0, 0), ('turn_warmup', 1, 0, .2),
    ('circle', 1, 0, .2), ('brake', 0, 1, 0), ('hold', 0, 1, 0), ('coast', 0, 0, 0),
])
def test_runner_command_semantics(config, phase, throttle, brake, steering):
    command = phase_command(episode_id='test', tick=2, sequence=1, phase=phase,
                            speed_m_s=0, case=config['cases'][3], config=config)
    assert command.throttle == throttle
    assert command.brake == brake
    assert command.steering_rad == steering
    assert command.issued_tick == 2
    assert command.expires_tick == 14
    assert command.vehicle_id == 'ego'


def test_runner_speed_servo_never_demands_reverse(config):
    command = phase_command(episode_id='test', tick=2, sequence=1, phase='accelerate',
                            speed_m_s=10, case=config['cases'][0], config=config)
    assert command.throttle == 0
