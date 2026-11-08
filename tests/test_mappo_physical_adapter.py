"""CPU contract/geometry tests. Native tracking claims require physical evidence."""
from dataclasses import replace
import math

import pytest

from traffic.driver_control import DriverControlGate
from traffic.lane_geometry import LaneRoute, RouteSegment
from traffic.mappo_physical_adapter import (
    AdapterConfig, LaneTransition, LongitudinalCalibration,
    PhysicalPolicyAdapter, PolicyIntent,
)
from traffic.path_following import VehicleState


def route(**kwargs):
    return LaneRoute('lane0', 3.7, (RouteSegment(2000),), **kwargs)


def state(tick=0, x=0., y=0., speed=10., **changes):
    return replace(VehicleState('episode', 'ego', tick, x, y, 0., speed), **changes)


def intent(tick=0, sequence=0, acceleration=0., lane=0, **changes):
    return replace(PolicyIntent('episode', 'ego', sequence, tick, tick + 24,
                                acceleration, lane, True), **changes)


def adapter(**kwargs):
    return PhysicalPolicyAdapter(route(), 'episode', 'ego', **kwargs)


def test_calibration_inversion_retains_physical_units_and_exclusive_actuators():
    cal = LongitudinalCalibration(2.2, 7., .1, .01, source='measured-test-fixture')
    for speed in (0, 5, 20):
        for acceleration in (-4.5, -1., 0., 1., 2.):
            throttle, brake = cal.invert(speed, acceleration)
            assert throttle * brake == 0
            assert 0 <= throttle <= 1 and 0 <= brake <= 1
            expected = min(acceleration, cal.predict(speed, 1., 0.))
            assert cal.predict(speed, throttle, brake) == pytest.approx(expected)
    assert cal.invert(10, 100) == (1., 0.)
    with pytest.raises(ValueError):
        cal.invert(30, 1)
    with pytest.raises(ValueError):
        cal.predict(5, .1, .1)


@pytest.mark.parametrize('changes', [dict(brake_gain_m_s2=0), dict(coast_m_s2=-1),
                                   dict(throttle_gain_m_s2=math.nan), dict(source=''),
                                   dict(max_speed_m_s=0), dict(revision='unknown'),
                                   dict(max_linear_brake_fraction=0),
                                   dict(max_linear_brake_fraction=1.1),
                                   dict(max_linear_brake_fraction=True)])
def test_calibration_rejects_invalid_models(changes):
    with pytest.raises(ValueError):
        LongitudinalCalibration(**changes)


@pytest.mark.parametrize('changes', [dict(physics_hz=60), dict(lane_count=True),
    dict(state_max_age_ticks=3), dict(max_steering_rad=.6), dict(max_speed_m_s=30),
    dict(min_acceleration_m_s2=-6), dict(acceleration_feedback_gain=-.1),
    dict(lane_change_time_s=math.nan), dict(rear_axle_offset_m=10)])
def test_configuration_rejects_unbounded_or_unsupported_controls(changes):
    with pytest.raises(ValueError):
        AdapterConfig(**changes)


def test_curve_is_not_silently_treated_as_a_straight_road():
    curved = LaneRoute('curve', 3.7, (RouteSegment(100, .001),))
    with pytest.raises(ValueError, match='straight'):
        PhysicalPolicyAdapter(curved, 'episode', 'ego')


def test_policy_acceleration_is_held_without_integrating_a_new_request_each_control_tick():
    control = adapter(config=AdapterConfig(acceleration_feedback_gain=0))
    first = intent(acceleration=1)
    for tick in range(0, 24, 2):
        command = control.command(state(tick), tick, first if tick == 0 else None)
        assert command.throttle == .5 and command.brake == 0
        assert command.expires_tick <= first.expires_tick
        assert control.last_diagnostics['requested_acceleration_m_s2'] == 1
    expired = control.command(state(24), 24)
    assert expired.throttle == 0 and expired.brake == 1
    assert control.last_diagnostics['reason'] == 'missing_or_expired_intent'


@pytest.mark.parametrize('mutate,reason', [
    (lambda x: replace(x, vehicle_id='wrong'), 'intent_identity_mismatch'),
    (lambda x: replace(x, issued_tick=1), 'invalid_intent_time_or_sequence'),
    (lambda x: replace(x, acceleration_m_s2=math.nan), 'invalid_intent_action'),
    (lambda x: replace(x, acceleration_m_s2=3), 'invalid_intent_action'),
    (lambda x: replace(x, lane_delta=True), 'invalid_intent_action'),
    (lambda x: replace(x, lane_change_allowed=1), 'invalid_intent_action'),
])
def test_invalid_policy_delivery_fails_to_zero_drive_full_brake(mutate, reason):
    control = adapter()
    command = control.command(state(), 0, mutate(intent()))
    assert (command.throttle, command.brake) == (0, 1)
    assert control.last_diagnostics['reason'] == reason


def test_invalid_state_and_sequence_conflicts_are_fail_safe_and_reset_clears_history():
    control = adapter()
    control.command(state(), 0, intent(acceleration=1))
    bad = control.command(state(2), 2, intent(acceleration=-1))
    assert bad.brake == 1 and control.last_diagnostics['reason'] == 'replayed_or_conflicting_intent'
    stale = control.command(state(0), 4, intent(4, 1))
    assert stale.brake == 1 and control.last_diagnostics['reason'] == 'invalid_or_stale_state'
    control.reset(episode_id='episode', vehicle_id='ego', initial_lane=0)
    fresh = control.command(state(), 0, intent(acceleration=1))
    assert fresh.sequence == 0 and fresh.throttle > 0
    assert control.last_diagnostics['measured_acceleration_m_s2'] is None
    with pytest.raises(ValueError, match='strictly increase'):
        control.command(state(), 0)


def test_native_gate_holds_then_expires_even_if_adapter_stops_running():
    control, gate = adapter(), DriverControlGate('episode', 'ego')
    command = control.command(state(), 0, intent(acceleration=1))
    for tick in range(13):
        applied = gate.step(tick=tick, dt_s=1/120, command=command if tick == 0 else None)
        assert applied.is_fallback == (tick == 12)
    assert applied.throttle == 0 and applied.brake == 1


def test_near_speed_ceiling_limits_acceleration_and_at_ceiling_never_accelerates():
    cfg = AdapterConfig(max_speed_m_s=20, acceleration_feedback_gain=.5)
    control = adapter(config=cfg)
    command = control.command(state(speed=19.99), 0, intent(acceleration=2))
    assert control.last_diagnostics['limited_acceleration_m_s2'] == pytest.approx(.1)
    assert control.last_diagnostics['corrected_acceleration_m_s2'] <= .1 + 1e-10
    assert control.last_diagnostics['speed_ceiling_limited']
    command = control.command(state(2, speed=20), 2)
    assert control.last_diagnostics['corrected_acceleration_m_s2'] <= 0
    assert command.throttle == 0
    stopped = adapter().command(state(speed=0), 0, intent(acceleration=-4))
    assert (stopped.throttle, stopped.brake) == (0., 1.)


def test_exceeding_measured_speed_envelope_does_not_silently_extrapolate():
    control = adapter(calibration=LongitudinalCalibration(max_speed_m_s=12,
                                                        source='bounded-fit-fixture'))
    command = control.command(state(speed=12.01), 0, intent())
    assert command.throttle == 0 and command.brake == 1
    assert control.last_diagnostics['reason'] == 'outside_calibration_speed_envelope'


def test_lane_path_has_zero_slope_at_ends_and_a_continuous_monotonic_shift():
    path = LaneTransition(0, 3.7, 10, 60)
    assert path.offset_and_derivative(10) == (0, 0)
    assert path.offset_and_derivative(70) == (3.7, 0)
    offsets = [path.offset_and_derivative(x)[0] for x in range(10, 71)]
    assert offsets == sorted(offsets)
    assert path.offset_and_derivative(40)[0] == pytest.approx(1.85)
    for end in (10, 70):
        assert abs(path.offset_and_derivative(end - 1e-5)[1]
                   - path.offset_and_derivative(end + 1e-5)[1]) < 1e-8


def test_lane_transition_is_latched_across_repeated_and_opposite_policy_actions():
    control = adapter()
    control.command(state(), 0, intent(lane=1))
    committed = control.transition
    assert control.target_lane == 1
    for tick, delta in ((2, 1), (4, -1), (6, 0), (8, 1)):
        control.command(state(tick), tick, intent(tick, tick, lane=delta))
        assert control.transition == committed
        assert control.target_lane == 1
    # A physically supplied endpoint state completes one change. Zero holds it.
    control.command(state(10, x=41, y=3.7), 10, intent(10, 10))
    assert control.transition is None and control.current_lane == 1
    assert control.last_diagnostics['completed_lane_changes'] == 1
    control.command(state(12, x=42, y=3.7), 12, intent(12, 12))
    assert control.target_lane == 1
    # A subsequent authorized negative request explicitly returns right.
    control.command(state(14, x=42, y=3.7), 14, intent(14, 14, lane=-1))
    assert control.target_lane == 0 and control.transition is not None


@pytest.mark.parametrize('lane,allowed,reason', [(-1, True, 'lane_boundary_rejected'),
                                             (1, False, 'upstream_lane_gap_rejected')])
def test_blocked_lane_requests_hold_lane_without_disabling_longitudinal_control(lane, allowed, reason):
    control = adapter()
    command = control.command(state(), 0, intent(acceleration=1, lane=lane,
                                               lane_change_allowed=allowed))
    assert control.target_lane == 0 and control.transition is None
    assert command.throttle > 0
    assert control.last_diagnostics['lane_reason'] == reason


def test_world_transform_preserves_positive_left_steering_and_offset_contract():
    control = PhysicalPolicyAdapter(route(origin_xy_m=(30, 50), initial_yaw_rad=math.pi/2),
                                    'episode', 'ego')
    observed = state(x=30, y=50, yaw_rad=math.pi/2)
    command = control.command(observed, 0, intent(lane=1))
    assert command.steering_rad > 0
    x, y, heading, offset = control._path(40)
    assert (x, y) == pytest.approx((26.3, 90))
    assert heading == pytest.approx(math.pi/2) and offset == 3.7


def test_measured_acceleration_uses_simulation_time_and_records_fit_prediction():
    control = adapter(config=AdapterConfig(acceleration_feedback_gain=0))
    control.command(state(speed=10), 0, intent(acceleration=1))
    control.command(state(2, speed=10 + 2/120), 2)
    assert control.last_diagnostics['measured_acceleration_m_s2'] == pytest.approx(1)
    assert control.last_diagnostics['calibrated_acceleration_m_s2'] == pytest.approx(1)
    assert control.last_diagnostics['calibration_source'] == 'provisional_unmeasured'


def test_ordinary_braking_stays_within_fitted_domain_and_logs_saturation():
    cal = LongitudinalCalibration(brake_gain_m_s2=8., max_linear_brake_fraction=.5)
    assert cal.invert(10, -4.5) == (0., .5)
    assert cal.predict(10, 0., .5) == -4.
    with pytest.raises(ValueError, match='envelope'):
        cal.predict(10, 0., 1.)
    control = adapter(calibration=cal, config=AdapterConfig(acceleration_feedback_gain=0))
    command = control.command(state(), 0, intent(acceleration=-4.5))
    assert command.brake == .5
    assert control.last_diagnostics['requested_acceleration_m_s2'] == -4.5
    assert control.last_diagnostics['calibrated_acceleration_m_s2'] == -4.
    assert control.last_diagnostics['effort_saturated']
    assert control.last_diagnostics['calibration_model_applicable']


def test_full_brake_stop_and_fallback_do_not_claim_affine_response():
    cal = LongitudinalCalibration(max_linear_brake_fraction=.5)
    control = adapter(calibration=cal)
    stopped = control.command(state(speed=0), 0, intent())
    assert (stopped.throttle, stopped.brake) == (0, 1)
    assert control.last_diagnostics['calibrated_acceleration_m_s2'] is None
    assert not control.last_diagnostics['calibration_model_applicable']
    fallback = control.command(state(24, speed=10), 24)
    assert (fallback.throttle, fallback.brake) == (0, 1)
    assert control.last_diagnostics['calibrated_acceleration_m_s2'] is None
    assert not control.last_diagnostics['calibration_model_applicable']
