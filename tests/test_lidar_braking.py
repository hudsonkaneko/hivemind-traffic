"""No Isaac imports: geometric obstacle stops, scan faults and latch behavior."""
from dataclasses import replace
import math

import pytest

from traffic.lidar_braking import LidarBrakeConfig, LidarEmergencyBrake, LidarScan, VEHICLE_FRAME


def scan(points=((20., 0., 0.),), **changes):
    return replace(LidarScan('episode', 'ego', 0, 0, 0, points), **changes)


def decide(value, **kwargs):
    return LidarEmergencyBrake('episode', 'ego').evaluate(value, tick=kwargs.pop('tick', 0),
                                                       speed_m_s=kwargs.pop('speed_m_s', 3), **kwargs)


def test_fresh_three_m_s_stop_distance_is_predeclared():
    result = decide(scan())
    assert result.status == 'clear'
    assert result.stopping_distance_m == pytest.approx(4.6)
    assert result.nearest_gap_m == pytest.approx(17.6)
    assert result.target_speed_m_s == 3 and result.brake_override == 0
    assert result.observation_source == 'lidar_point_cloud'


def test_obstacle_uses_bumper_gap_not_sensor_range_and_brakes():
    result = decide(scan(((6., 0., .2),)))
    assert result.status == 'obstacle'
    assert result.nearest_gap_m == pytest.approx(3.6)
    assert result.target_speed_m_s == 0 and result.brake_override == 1
    assert result.stop_latched


def test_close_obstacle_is_not_hidden_in_an_extra_front_margin():
    assert decide(scan(((2.4001, 0., 0.),))).status == 'obstacle'


def test_geometry_filters_ground_self_rear_and_out_of_lane_points():
    points = ((-2., 0., 0.), (1., 0., .5), (3., 0., -1.),
              (3., 2., .1), (3., 0., 2.))
    result = decide(scan(points))
    assert result.status == 'clear'
    assert result.nearest_gap_m is None
    assert result.points_in_forward_corridor == 0


def test_nearest_point_and_boundary_points_are_included():
    points = ((8., 0., 0.), (5., 1.15, -.6), (7., -1.15, .6))
    result = decide(scan(points))
    assert result.status == 'obstacle'
    assert result.nearest_gap_m == pytest.approx(2.6)
    assert result.points_in_forward_corridor == 3


def test_oldest_acquisition_age_expands_stopping_distance():
    result = decide(scan(acquisition_start_tick=0, acquisition_end_tick=12,
                         delivery_tick=24), tick=24)
    assert result.oldest_sample_age_s == .2
    assert result.stopping_distance_m == pytest.approx(5.2)


@pytest.mark.parametrize('value,reason', [
    (None, 'missing_or_invalid_scan'),
    (scan(episode_id='old'), 'identity_mismatch'),
    (scan(vehicle_id='peer'), 'identity_mismatch'),
    (scan(frame='world'), 'wrong_coordinate_frame'),
    (scan(delivery_tick=True), 'invalid_scan_timestamp'),
    (scan(acquisition_start_tick=1), 'future_or_misordered_scan'),
    (scan(acquisition_end_tick=1, delivery_tick=1), 'future_or_misordered_scan'),
    (scan(()), 'empty_point_cloud'),
    (scan(((1, 2),)), 'invalid_point_cloud_shape'),
    (scan(((1, 2, 3, 4),)), 'invalid_point_cloud_shape'),
    (scan(((math.nan, 0, 0),)), 'nonfinite_or_invalid_point'),
    (scan(((3, math.inf, 0),)), 'nonfinite_or_invalid_point'),
    (scan(((True, 0, 0),)), 'nonfinite_or_invalid_point'),
    (scan('xyz'), 'invalid_point_cloud_shape'),
])
def test_invalid_scan_fails_safe(value, reason):
    result = decide(value)
    assert result.status == 'stale_invalid' and result.reason == reason
    assert result.target_speed_m_s == 0 and result.brake_override == 1


def test_stale_age_uses_acquisition_not_redelivery():
    result = decide(scan(delivery_tick=25), tick=25)
    assert result.reason == 'stale_scan'
    assert result.brake_override == 1


def test_oldest_sample_must_be_fresh_even_when_newest_is_recent():
    value = scan(acquisition_start_tick=0, acquisition_end_tick=24,
                 delivery_tick=25, coordinate_reference_tick=25)
    result = decide(value, tick=25)
    assert result.status == 'stale_invalid' and result.reason == 'stale_scan'
    assert result.target_speed_m_s == 0 and result.brake_override == 1


def test_oversized_acquisition_window_fails():
    result = decide(scan(acquisition_end_tick=25, delivery_tick=25), tick=25)
    assert result.reason == 'excessive_acquisition_span'


def test_held_scan_is_allowed_only_while_fresh_and_replay_is_rejected():
    monitor = LidarEmergencyBrake('episode', 'ego')
    value = scan(acquisition_start_tick=10, acquisition_end_tick=10, delivery_tick=10)
    assert monitor.evaluate(value, tick=10, speed_m_s=3).status == 'clear'
    assert monitor.evaluate(value, tick=20, speed_m_s=3).status == 'clear'
    assert monitor.evaluate(scan(), tick=21, speed_m_s=3).reason == 'replayed_scan'
    assert monitor.evaluate(value, tick=35, speed_m_s=3).reason == 'stale_scan'


def test_obstacle_stop_stays_latched_through_clear_fault_and_dropout():
    monitor = LidarEmergencyBrake('episode', 'ego')
    assert monitor.evaluate(scan(((5., 0., 0.),)), tick=0, speed_m_s=3).stop_latched
    clear = monitor.evaluate(scan(), tick=1, speed_m_s=2)
    assert clear.reason == 'latched_obstacle_stop' and clear.brake_override == 1
    fault = monitor.evaluate(None, tick=2, speed_m_s=1)
    assert fault.status == 'stale_invalid' and fault.stop_latched and fault.brake_override == 1
    stopped = monitor.evaluate(scan(), tick=3, speed_m_s=0)
    assert stopped.status == 'obstacle' and stopped.target_speed_m_s == 0
    monitor.reset(episode_id='new', vehicle_id='ego')
    assert monitor.evaluate(scan(episode_id='new'), tick=0, speed_m_s=0).status == 'clear'


def test_invalid_scan_can_recover_if_no_obstacle_was_latched():
    monitor = LidarEmergencyBrake('episode', 'ego')
    assert monitor.evaluate(None, tick=0, speed_m_s=0).brake_override == 1
    assert monitor.evaluate(scan(), tick=1, speed_m_s=0).brake_override == 0


@pytest.mark.parametrize('speed', [-1, True, math.nan, math.inf, 3.01])
def test_invalid_speed_fails_safe(speed):
    assert decide(scan(), speed_m_s=speed).reason == 'invalid_or_excessive_speed'


def test_bad_clock_and_target_do_not_allow_drive():
    monitor = LidarEmergencyBrake('episode', 'ego')
    monitor.evaluate(scan(), tick=0, speed_m_s=0)
    assert monitor.evaluate(scan(), tick=0, speed_m_s=0).reason == 'invalid_local_clock'
    assert decide(scan(), requested_target_speed_m_s=4).reason == 'invalid_target_speed'


def test_configuration_validation_and_point_budget():
    for changes in ({'physics_hz': True}, {'min_height_m': 2}, {'planning_deceleration_m_s2': 0},
                    {'stopping_margin_m': -1}, {'max_speed_m_s': 4}, {'max_scan_age_ticks': 0}):
        with pytest.raises(ValueError):
            LidarBrakeConfig(**changes)
    monitor = LidarEmergencyBrake('episode', 'ego', LidarBrakeConfig(max_points=1))
    assert monitor.evaluate(scan(((9., 0., 0.), (10., 0., 0.))), tick=0, speed_m_s=0).reason == 'excessive_point_count'


def test_numpy_point_arrays_do_not_require_object_labels():
    np = pytest.importorskip('numpy')
    result = decide(scan(np.array([[6., 0., .1]], dtype=np.float32)))
    assert result.status == 'obstacle' and result.frame == VEHICLE_FRAME


def test_current_odometry_frame_reference_does_not_renew_acquisition_age():
    value = scan(acquisition_end_tick=12, delivery_tick=12, coordinate_reference_tick=24)
    result = decide(value, tick=24)
    assert result.status == 'clear' and result.oldest_sample_age_s == .2
    assert decide(replace(value, coordinate_reference_tick=25), tick=24).reason == 'invalid_coordinate_reference_tick'
    assert decide(replace(value, coordinate_reference_tick=10), tick=24).reason == 'invalid_coordinate_reference_tick'
    assert decide(replace(value, coordinate_reference_tick=True), tick=24).reason == 'invalid_coordinate_reference_tick'
