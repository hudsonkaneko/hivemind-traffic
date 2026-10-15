"""Acceptance-logic tests use synthetic traces, not physical validation evidence."""
from copy import deepcopy
import math

import pytest

from experiments.highway_loop_config import (
    PROFILES, assess, footprint_lane_error, make_config, validate_config,
)


LENGTH = math.tau * 500.0


def trace(profile='seam3'):
    """Deterministic on-circle fixture with two-second braking and full hold."""
    config = make_config(profile, gui=False, paced=False)
    hz = config['physics_hz']
    brake_tick = (config['settle_s'] + config['drive_s']) * hz
    last_drive_delivery = (brake_tick - 1) // 2 * 2
    expiry = last_drive_delivery + config['command_ttl_ticks']
    progress = travel = 0.0
    previous = (500 * math.cos(-.04), 500 * math.sin(-.04))
    rows = []
    for tick in range(config['duration_s'] * hz):
        if tick < config['settle_s'] * hz:
            phase, speed = 'settle', 0.0
        elif tick < brake_tick:
            phase, speed = 'drive', config['target_speed_m_s']
        else:
            phase = 'brake'
            actual_brake_start = expiry if config['dropout'] else brake_tick
            seconds_braking = max(0, tick - actual_brake_start + 1) / hz
            speed = config['target_speed_m_s'] * max(0, 1 - seconds_braking / 2)
        progress += speed / hz
        angle = -.04 + progress / 500
        xy = (500 * math.cos(angle), 500 * math.sin(angle))
        travel += math.dist(previous, xy)
        previous = xy
        fallback = config['dropout'] and phase == 'brake' and tick >= expiry
        explicit_stop = phase == 'brake' and not config['dropout']
        braking = fallback or explicit_stop or phase == 'settle'
        rows.append(dict(
            tick=tick+1, applied_tick=tick, sim_time_s=(tick+1)/hz,
            phase=phase, speed_m_s=speed, position_m=[*xy, 1.0],
            lane_error_m=0., footprint_error_m=.91, upright_z=1.0,
            wheel_on_ground=[True]*4, progress_m=progress,
            traveled_distance_m=travel,
            seam_crossings=math.floor((LENGTH-20+progress)/LENGTH),
            fallback=fallback, follower_fallback=False,
            follower_stop_latched=explicit_stop,
            command_expires_tick=(expiry if phase == 'brake' and config['dropout']
                                  else tick//2*2+config['command_ttl_ticks']),
            throttle=0. if braking else .2, brake=1. if braking else 0.,
        ))
    return config, rows


def result(rows, config, contact_count=0):
    return assess(rows, config, contact_count=contact_count, route_length_m=LENGTH)


def recalculate_travel(rows):
    for previous, current in zip(rows, rows[1:]):
        current['traveled_distance_m'] = previous['traveled_distance_m'] + math.dist(
            previous['position_m'][:2], current['position_m'][:2])


@pytest.mark.parametrize('profile', PROFILES)
def test_frozen_profiles_validate(profile):
    config = make_config(profile)
    validate_config(config)
    assert config['minimum_stationary_hold_s'] == 5.0
    assert config['max_stationary_drift_m'] == .05
    changed = deepcopy(config)
    changed['minimum_stationary_hold_s'] = 1.0
    with pytest.raises(ValueError):
        validate_config(changed)


@pytest.mark.parametrize('profile', ['seam3', 'seam6', 'seam35', 'dropout35', 'lap35'])
def test_nominal_synthetic_trace_passes(profile):
    config, rows = trace(profile)
    report = result(rows, config)
    assert report['passed'], report['gates']
    assert report['stationary_hold_s'] >= 5
    assert report['stationary_hold_drift_m'] <= .05
    assert report['braking_distance_m'] > 0
    assert report['completed_laps'] == (1 if profile == 'lap35' else 0)


def test_braking_uses_cumulative_physical_travel_not_progress():
    config, rows = trace()
    stopping = [r for r in rows if r['phase'] == 'brake']
    # Out-and-back lateral motion can have essentially zero net route progress.
    for index, row in enumerate(stopping[:240]):
        row['position_m'][1] += 10 * math.sin(math.pi * index / 239)
    recalculate_travel(rows)
    report = result(rows, config)
    assert report['gates']['traveled_distance_valid']
    assert not report['gates']['braking_distance']
    assert report['braking_distance_m'] > 10


def test_bad_distance_accumulator_is_rejected():
    config, rows = trace()
    for row in rows:
        row['traveled_distance_m'] = 0.
    report = result(rows, config)
    assert not report['gates']['traveled_distance_valid']
    assert not report['gates']['braking_distance']


def test_first_stop_ends_braking_measurement_and_subsequent_drift_fails_hold():
    config, rows = trace()
    before = result(rows, config)
    for row in rows:
        if row['sim_time_s'] > 27:
            row['position_m'][1] += .10
    recalculate_travel(rows)
    after = result(rows, config)
    assert after['braking_distance_m'] == before['braking_distance_m']
    assert after['stationary_hold_drift_m'] > .05
    assert not after['gates']['stationary_hold']


def test_one_second_final_stop_does_not_meet_five_second_hold():
    config, rows = trace()
    for row in rows:
        if row['phase'] == 'brake' and row['sim_time_s'] < 29:
            row['speed_m_s'] = .10
    report = result(rows, config)
    assert report['gates']['stopped']
    assert not report['gates']['stationary_hold']
    assert report['stationary_hold_s'] <= 1.01


def test_motion_after_first_stop_cannot_requalify_with_final_good_second():
    config, rows = trace()
    rows[27*120]['speed_m_s'] = .2
    report = result(rows, config)
    assert report['gates']['stopped']
    assert not report['gates']['stationary_hold']


def test_dropout_expires_on_exact_applied_tick_not_post_step_state_tick():
    config, rows = trace('dropout35')
    report = result(rows, config)
    expiry = report['command_expiry_tick']
    assert expiry == (config['settle_s']+config['drive_s'])*120 + 10
    assert report['gates']['stale_control_fallback']
    expiry_row = next(row for row in rows if row['applied_tick'] == expiry)
    expiry_row.update(fallback=False, throttle=.2, brake=0.)
    assert not result(rows, config)['gates']['stale_control_fallback']


def test_early_fallback_and_renewed_dropout_command_both_fail_exact_expiry():
    for mutation in ('early', 'renewal'):
        config, rows = trace('dropout35')
        stop = [row for row in rows if row['phase'] == 'brake']
        if mutation == 'early':
            stop[0].update(fallback=True, throttle=0., brake=1.)
        else:
            stop[0]['command_expires_tick'] += 2
        assert not result(rows, config)['gates']['stale_control_fallback']


def test_speed_servo_braking_does_not_mean_the_seam_stopped_the_car():
    config, rows = trace()
    row = next(row for row in rows if row['phase'] == 'drive' and row['seam_crossings'])
    row.update(throttle=0., brake=.05)
    assert result(rows, config)['gates']['no_seam_stop']
    row['follower_stop_latched'] = True
    assert not result(rows, config)['gates']['no_seam_stop']


def test_follower_fallback_cannot_hide_behind_valid_gate_command():
    config, rows = trace()
    row = next(row for row in rows if row['phase'] == 'drive')
    row.update(fallback=False, follower_fallback=True, throttle=0., brake=1.)
    assert not result(rows, config)['gates']['no_seam_stop']


def test_explicit_stop_requires_actual_brake_and_latched_request():
    for field, value in [('brake', 0.), ('follower_stop_latched', False), ('follower_fallback', True)]:
        config, rows = trace()
        next(row for row in rows if row['phase'] == 'brake')[field] = value
        assert not result(rows, config)['gates']['explicit_stop_control']


def test_missing_duplicate_or_mislabelled_ticks_fail_completeness():
    for mutation in ('missing', 'duplicate', 'applied', 'clock'):
        config, rows = trace()
        if mutation == 'missing':
            rows.pop()
        elif mutation == 'duplicate':
            rows[100]['tick'] = rows[99]['tick']
        elif mutation == 'applied':
            rows[100]['applied_tick'] = rows[100]['tick']
        else:
            rows[100]['sim_time_s'] += 1 / 120
        assert not result(rows, config)['gates']['complete']


def test_empty_trace_fails_without_claiming_a_stop_or_lap():
    config = make_config()
    report = result([], config)
    assert not report['passed']
    assert report['first_stopped_tick'] is None
    assert report['braking_distance_m'] is None


def test_footprint_not_just_chassis_center_is_checked():
    aligned = dict(position_m=[500., 0., 1.], yaw_rad=math.pi/2)
    rotated = dict(position_m=[500., 0., 1.], yaw_rad=0.)
    assert footprint_lane_error(aligned, 500) < 1.85
    assert footprint_lane_error(rotated, 500) > 1.85


@pytest.mark.parametrize('field,value,gate', [
    ('lane_error_m', .51, 'lane_max'),
    ('footprint_error_m', 1.86, 'footprint_in_lane'),
    ('upright_z', .97, 'upright'),
    ('position_m', [500., 0., .49], 'chassis_height'),
    ('wheel_on_ground', [True, True, True, False], 'wheel_support'),
])
def test_basic_physical_gates_are_preserved(field, value, gate):
    config, rows = trace()
    next(row for row in rows if row['phase'] == 'drive')[field] = value
    assert not result(rows, config)['gates'][gate]


def test_any_undesired_rigid_contact_fails():
    config, rows = trace()
    assert not result(rows, config, contact_count=1)['gates']['no_rigid_contacts']
