from dataclasses import asdict, replace
import math

import pytest

from traffic.driver_control import (
    AppliedControl,
    ControlLimits,
    DriverCommand,
    DriverControlGate,
    ackermann_steering,
    wheel_torques,
)


def command(**overrides):
    values = dict(
        episode_id="episode-1", vehicle_id="ego", sequence=0,
        issued_tick=0, expires_tick=12, steering_rad=0.3, throttle=0.4, brake=0.0,
    )
    return DriverCommand(**(values | overrides))


def test_accept_hold_and_exclusive_expiry_at_physics_rate():
    gate = DriverControlGate("episode-1", "ego")
    first = gate.step(tick=0, dt_s=1 / 120, command=command(expires_tick=2))
    assert first.reason == "accepted"
    assert first.steering_rad == pytest.approx(0.5 / 120)
    assert first.throttle == 0.4 and first.brake == 0 and not first.is_fallback
    held = gate.step(tick=1, dt_s=1 / 120)
    assert held.reason == "held" and held.command_sequence == 0
    expired = gate.step(tick=2, dt_s=1 / 120)
    assert expired.reason == "expired" and expired.is_fallback
    assert expired.throttle == 0 and expired.brake == 1 and expired.command_sequence is None
    assert expired.steering_rad == pytest.approx(0.5 / 120)
    missing = gate.step(tick=3, dt_s=1 / 120)
    assert missing.reason == "missing" and missing.steering_rad == pytest.approx(0)


def test_braking_overrides_propulsion_and_output_is_serializable():
    gate = DriverControlGate("episode-1", "ego")
    control = gate.step(tick=0, dt_s=0.1, command=command(throttle=1, brake=0.01))
    assert control.throttle == 0 and control.brake == 0.01
    assert asdict(control) == dict(
        steering_rad=0.05, throttle=0.0, brake=0.01,
        reason="accepted", command_sequence=0, is_fallback=False,
    )


@pytest.mark.parametrize("changes,reason", [
    ({"episode_id": "old-episode"}, "identity_mismatch"),
    ({"vehicle_id": "other-car"}, "identity_mismatch"),
    ({"sequence": -1}, "invalid_sequence"),
    ({"sequence": 1.5}, "invalid_sequence"),
    ({"sequence": True}, "invalid_sequence"),
    ({"issued_tick": -1}, "invalid_timestamp"),
    ({"issued_tick": True}, "invalid_timestamp"),
    ({"expires_tick": 1.5}, "invalid_timestamp"),
    ({"issued_tick": 1, "expires_tick": 1}, "invalid_timestamp"),
    ({"issued_tick": 3, "expires_tick": 5}, "future_command"),
    ({"expires_tick": 1}, "expired_command"),
    ({"steering_rad": 0.50001}, "invalid_controls"),
    ({"steering_rad": -0.50001}, "invalid_controls"),
    ({"throttle": -0.001}, "invalid_controls"),
    ({"throttle": 1.001}, "invalid_controls"),
    ({"brake": -0.001}, "invalid_controls"),
    ({"brake": 1.001}, "invalid_controls"),
    ({"throttle": "1"}, "invalid_controls"),
    ({"brake": True}, "invalid_controls"),
])
def test_invalid_delivery_clears_held_command_and_fails_safe(changes, reason):
    gate = DriverControlGate("episode-1", "ego")
    gate.step(tick=0, dt_s=0.1, command=command())
    result = gate.step(tick=1, dt_s=0.1, command=command(sequence=1, **changes) if "sequence" not in changes else command(**changes))
    assert result.reason == reason and result.is_fallback
    assert result.throttle == 0 and result.brake == 1 and result.steering_rad == 0
    assert gate.step(tick=2, dt_s=0.1).reason == "missing"


@pytest.mark.parametrize("field", ["steering_rad", "throttle", "brake"])
@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf"), 10 ** 1000])
def test_nonfinite_action_fails_safe(field, value):
    gate = DriverControlGate("episode-1", "ego")
    result = gate.step(tick=0, dt_s=0.1, command=command(**{field: value}))
    assert result.reason == "invalid_controls"
    assert result.throttle == 0 and result.brake == 1


def test_non_command_delivery_fails_safe():
    gate = DriverControlGate("episode-1", "ego")
    result = gate.step(tick=0, dt_s=0.1, command={"throttle": 1})
    assert result.reason == "invalid_type" and result.is_fallback


def test_exact_duplicate_holds_without_extending_expiry():
    gate = DriverControlGate("episode-1", "ego")
    original = command(expires_tick=2)
    gate.step(tick=0, dt_s=0.1, command=original)
    assert gate.step(tick=1, dt_s=0.1, command=replace(original)).reason == "duplicate_held"
    expired = gate.step(tick=2, dt_s=0.1, command=original)
    assert expired.reason == "expired_command" and expired.throttle == 0


def test_sequence_conflict_cannot_extend_lifetime():
    gate = DriverControlGate("episode-1", "ego")
    gate.step(tick=0, dt_s=0.1, command=command(expires_tick=3))
    result = gate.step(tick=1, dt_s=0.1, command=command(expires_tick=100))
    assert result.reason == "sequence_conflict" and result.is_fallback
    assert gate.step(tick=2, dt_s=0.1, command=command(expires_tick=3)).is_fallback
    recovered = gate.step(tick=3, dt_s=0.1, command=command(sequence=1, issued_tick=3))
    assert recovered.reason == "accepted" and recovered.throttle == 0.4


def test_older_sequence_is_replay_even_when_timestamps_are_current():
    gate = DriverControlGate("episode-1", "ego")
    gate.step(tick=0, dt_s=0.1, command=command(sequence=4))
    result = gate.step(tick=1, dt_s=0.1, command=command(sequence=3, issued_tick=1))
    assert result.reason == "replayed_sequence" and result.throttle == 0


def test_reset_accepts_new_identity_and_restarts_ticks_and_sequences():
    gate = DriverControlGate("episode-1", "ego")
    gate.step(tick=10, dt_s=0.1, command=command(sequence=100))
    gate.reset(episode_id="episode-2", vehicle_id="peer")
    bad = gate.step(tick=0, dt_s=0.1, command=command())
    assert bad.reason == "identity_mismatch"
    new = gate.step(tick=1, dt_s=0.1, command=command(episode_id="episode-2", vehicle_id="peer"))
    assert new.reason == "accepted" and new.command_sequence == 0
    assert new.steering_rad == pytest.approx(0.05)


def test_failed_reset_does_not_partially_change_identity_or_history():
    gate = DriverControlGate("episode-1", "ego")
    gate.step(tick=0, dt_s=0.1, command=command())
    with pytest.raises(ValueError):
        gate.reset(episode_id="episode-2", vehicle_id="")
    assert gate.episode_id == "episode-1" and gate.vehicle_id == "ego"
    assert gate.step(tick=1, dt_s=0.1).reason == "held"


@pytest.mark.parametrize("tick,dt", [
    (-1, 0.1), (True, 0.1), (0.5, 0.1), (0, 0), (0, -0.1),
    (0, float("nan")), (0, float("inf")), (0, True),
])
def test_invalid_local_clock_rejected_without_advancing_gate(tick, dt):
    gate = DriverControlGate("episode-1", "ego")
    with pytest.raises(ValueError):
        gate.step(tick=tick, dt_s=dt, command=command())
    assert gate.step(tick=0, dt_s=0.1, command=command()).reason == "accepted"


@pytest.mark.parametrize("repeated_tick", [0, 1])
def test_ticks_must_increase(repeated_tick):
    gate = DriverControlGate("episode-1", "ego")
    gate.step(tick=1, dt_s=0.1)
    with pytest.raises(ValueError):
        gate.step(tick=repeated_tick, dt_s=0.1)


def test_rate_limit_in_both_directions_and_fallback():
    limits = ControlLimits(max_steering_rad=0.5, steering_rate_rad_s=0.2, fallback_brake=0.75)
    gate = DriverControlGate("episode-1", "ego", limits)
    first = gate.step(tick=0, dt_s=0.5, command=command(steering_rad=0.5))
    assert first.steering_rad == pytest.approx(0.1)
    second = gate.step(tick=1, dt_s=0.5, command=command(sequence=1, steering_rad=-0.5))
    assert second.steering_rad == pytest.approx(0)
    third = gate.step(tick=2, dt_s=0.5)
    assert third.steering_rad == pytest.approx(-0.1)
    fallback = gate.step(tick=3, dt_s=0.25, command=command(sequence=2, throttle=-1))
    assert fallback.steering_rad == pytest.approx(-0.05) and fallback.brake == 0.75


@pytest.mark.parametrize("changes", [
    {"max_steering_rad": 0}, {"max_steering_rad": math.pi / 2},
    {"max_steering_rad": float("nan")}, {"steering_rate_rad_s": 0},
    {"steering_rate_rad_s": -1}, {"steering_rate_rad_s": float("inf")},
    {"fallback_brake": 0}, {"fallback_brake": 1.01}, {"fallback_brake": True},
])
def test_limit_validation(changes):
    with pytest.raises(ValueError):
        ControlLimits(**changes)


@pytest.mark.parametrize("identity", ["", " ", None, 3])
def test_initial_identity_validation(identity):
    with pytest.raises(ValueError):
        DriverControlGate(identity, "ego")
    with pytest.raises(ValueError):
        DriverControlGate("episode-1", identity)


def test_ackermann_geometry_matches_shared_turn_center_and_mirrors():
    left, right = ackermann_steering(0.3, wheelbase_m=2.7, track_width_m=1.6)
    assert left > 0.3 > right > 0
    radius = 2.7 / math.tan(0.3)
    assert 2.7 / math.tan(left) + 0.8 == pytest.approx(radius)
    assert 2.7 / math.tan(right) - 0.8 == pytest.approx(radius)
    mirrored = ackermann_steering(-0.3, wheelbase_m=2.7, track_width_m=1.6)
    assert mirrored == pytest.approx((-right, -left))
    assert ackermann_steering(0, wheelbase_m=2.7, track_width_m=1.6) == (0, 0)


@pytest.mark.parametrize("steer,wheelbase,track", [
    (float("nan"), 2.7, 1.6), (math.pi / 2, 2.7, 1.6),
    (0.3, 0, 1.6), (0.3, 2.7, 0), (0.3, float("inf"), 1.6),
    (1.0, 0.1, 2.0),
])
def test_ackermann_invalid_geometry_rejected(steer, wheelbase, track):
    with pytest.raises(ValueError):
        ackermann_steering(steer, wheelbase_m=wheelbase, track_width_m=track)


def test_wheel_torques_are_per_wheel_and_braking_has_priority():
    control = AppliedControl(0, 0.5, 0, "accepted", 0, False)
    assert wheel_torques(control, max_drive_torque_nm=700, max_brake_torque_nm=1500) == (
        (350, 350, 0, 0), (0, 0, 0, 0),
    )
    assert wheel_torques(control, max_drive_torque_nm=700, max_brake_torque_nm=1500, driven_wheels=(2, 3))[0] == (0, 0, 350, 350)
    braking = replace(control, brake=0.2)
    assert wheel_torques(braking, max_drive_torque_nm=700, max_brake_torque_nm=1500) == (
        (0, 0, 0, 0), (300, 300, 300, 300),
    )


@pytest.mark.parametrize("changes", [
    {"max_drive_torque_nm": -1}, {"max_brake_torque_nm": float("nan")},
    {"driven_wheels": (0, 0)}, {"driven_wheels": (4,)},
    {"driven_wheels": (True,)}, {"driven_wheels": [0, 1]},
])
def test_invalid_torque_mapping_rejected(changes):
    args = dict(max_drive_torque_nm=700, max_brake_torque_nm=1500) | changes
    with pytest.raises(ValueError):
        wheel_torques(AppliedControl(0, 0.5, 0, "accepted", 0, False), **args)
