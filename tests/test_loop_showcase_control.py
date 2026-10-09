"""CPU contracts and geometry only; runtime PhysX qualification is separate."""
from dataclasses import replace
import math

import pytest

from traffic.driver_control import DriverControlGate
from traffic.loop_showcase_control import (
    ObjectTrack, RadialPath, ShowcaseConfig, ShowcaseController,
    signed_station_gap, track_prediction,
)
from traffic.path_following import FRAME, SOURCE, VehicleState


def state(tick=0, station=0., lane=0, speed=15., **changes):
    point = RadialPath(lane, lane, 0, 80).pose(station)
    return replace(VehicleState('episode', 'ego', tick, *point, speed), **changes)


def track(tick=0, station=80., lane=0, speed=7., identity='slow_01', **changes):
    point = RadialPath(lane, lane, 0, 80).pose(station)
    return replace(ObjectTrack('episode', identity, tick, *point, speed), **changes)


@pytest.mark.parametrize('from_lane,to_lane', [(0, 1), (1, 0), (2, 3), (3, 2), (0, 0)])
def test_radial_path_is_smooth_and_stays_between_lane_centres(from_lane, to_lane):
    p = RadialPath(from_lane, to_lane, 100, 80)
    start = 500 + from_lane * 3.7
    end = 500 + to_lane * 3.7
    assert p.radius_and_derivative(99) == (start, 0)
    assert p.radius_and_derivative(100) == (start, 0)
    assert p.radius_and_derivative(180) == (end, 0)
    assert p.radius_and_derivative(190) == (end, 0)
    for i in range(81):
        radius, _ = p.radius_and_derivative(100 + i)
        assert min(start, end) <= radius <= max(start, end)
    assert p.radius_and_derivative(140)[0] == pytest.approx((start + end) / 2)
    for endpoint in (100, 180):
        before, after = p.pose(endpoint - 1e-5), p.pose(endpoint + 1e-5)
        assert math.dist(before[:2], after[:2]) < 3e-5
        assert abs(before[2] - after[2]) < 1e-6


def test_station_wrap_and_path_geometry_remain_continuous_across_seam():
    length = 500 * math.tau
    assert signed_station_gap(length - 2, 3) == pytest.approx(5)
    assert signed_station_gap(3, length - 2) == pytest.approx(-5)
    p = RadialPath(0, 1, length - 40, 80)
    assert math.dist(p.pose(length - .01)[:2], p.pose(length + .01)[:2]) < .021


def test_exact_background_prediction_uses_its_lane_radius_not_base_radius():
    obj = track(lane=3, station=500 * math.tau - 5)
    predicted = track_prediction(obj, 10)
    initial_angle = math.atan2(obj.y_m, obj.x_m)
    angle = initial_angle + 70 / 511.1
    assert (predicted.x_m, predicted.y_m) == pytest.approx((511.1 * math.cos(angle), 511.1 * math.sin(angle)))


@pytest.mark.parametrize('changes', [
    dict(radius_m=503), dict(lane_width_m=4), dict(lane_count=3), dict(physics_hz=60),
    dict(target_speed_m_s=20), dict(target_speed_m_s=math.nan), dict(lane_change_length_m=20),
    dict(planning_interval_ticks=24), dict(command_ttl_ticks=24), dict(prediction_step_s=.3),
    dict(ego_width_m=3.5), dict(rear_axle_offset_m=4), dict(max_steering_rad=.6),
])
def test_configuration_guards(changes):
    with pytest.raises(ValueError):
        ShowcaseConfig(**changes)


@pytest.mark.parametrize('args', [(-1, 0, 0, 80), (0, 2, 0, 80), (0, 1, math.nan, 80), (0, 1, 0, 0)])
def test_invalid_paths_are_rejected(args):
    with pytest.raises(ValueError):
        RadialPath(*args)


def test_empty_road_cruise_returns_bounded_command_with_correct_identity_and_ttl():
    control = ShowcaseController('episode', 'ego')
    cmd = control.command(state(speed=3), 0, [])
    assert cmd.throttle > 0 and cmd.brake == 0
    assert 0 < cmd.steering_rad < .02
    assert cmd.expires_tick == 12
    assert cmd.episode_id == 'episode' and cmd.vehicle_id == 'ego'
    assert not control.diagnostics['is_fallback']
    assert control.diagnostics['source'] == SOURCE
    gate = DriverControlGate('episode', 'ego')
    assert not gate.step(tick=0, dt_s=1 / 120, command=cmd).is_fallback


def test_slower_lead_causes_adjacent_lane_change_and_path_overlay_adapts():
    control = ShowcaseController('episode', 'ego')
    control.command(state(), 0, [track()])
    assert control.diagnostics['maneuver_active']
    assert control.diagnostics['target_lane'] == 1
    assert control.diagnostics['planner_reason'] == 'safe_adjacent_lane_selected'
    points = control.path_points(state())
    assert len(points) == 61
    assert math.hypot(*points[0][:2]) == pytest.approx(500)
    assert math.hypot(*points[-1][:2]) == pytest.approx(503.7)
    assert all(p[2] == .06 for p in points)


def test_nearby_target_lane_vehicle_blocks_change_without_forcing_weave():
    control = ShowcaseController('episode', 'ego')
    control.command(state(), 0, [track(station=30), track(lane=1, station=5, identity='target')])
    assert not control.diagnostics['maneuver_active']
    assert control.diagnostics['target_lane'] == 0
    assert control.diagnostics['target_speed_m_s'] < 15
    assert control.diagnostics['planner_reason'] in ('blocked_following', 'unsafe_closing_gap_brake')


def test_fast_target_lane_rear_vehicle_blocks_change():
    control = ShowcaseController('episode', 'ego')
    control.command(state(speed=10), 0, [track(station=80),
        track(lane=1, station=-30, speed=20, identity='fast_rear')])
    assert not control.diagnostics['maneuver_active']


def test_picks_safe_side_with_clearer_downstream_traffic():
    control = ShowcaseController('episode', 'ego', initial_lane=1)
    control.command(state(lane=1), 0, [track(lane=1), track(lane=0, station=5, identity='left_block')])
    assert control.diagnostics['target_lane'] == 2


def test_no_need_to_change_lane_for_distant_or_equal_speed_vehicle():
    for obj in (track(station=200), track(station=80, speed=15.6464)):
        control = ShowcaseController('episode', 'ego')
        control.command(state(), 0, [obj])
        assert not control.diagnostics['maneuver_active']


@pytest.mark.parametrize('changes', [
    dict(tick=1), dict(tick=-1), dict(episode_id='other'), dict(vehicle_id='ego'),
    dict(vehicle_id=''), dict(x_m=math.nan), dict(frame='wrong'), dict(source='lidar'),
    dict(speed_m_s=-1), dict(speed_m_s=30), dict(length_m=0), dict(width_m=5),
    dict(yaw_rad=0), dict(x_m=501),
])
def test_bad_object_frame_latches_brake_and_does_not_recover_silently(changes):
    control = ShowcaseController('episode', 'ego')
    obj = track(station=0, lane=1)
    cmd = control.command(state(), 0, [replace(obj, **changes)])
    assert cmd.throttle == 0 and cmd.brake == 1
    assert control.diagnostics['is_fallback']
    cmd = control.command(state(tick=2, station=.25), 2, [replace(obj, tick=2)])
    assert cmd.throttle == 0 and cmd.brake == 1


@pytest.mark.parametrize('tracks', [None, {}, 'missing'])
def test_missing_frame_is_not_an_empty_road(tracks):
    control = ShowcaseController('episode', 'ego')
    cmd = control.command(state(), 0, tracks)
    assert cmd.brake == 1 and control.diagnostics['is_fallback']


def test_missing_id_duplicate_id_or_unexpected_new_id_fails_safe():
    for after in ([], [track(2), track(2)], [track(2), track(2, identity='new')]):
        control = ShowcaseController('episode', 'ego')
        control.command(state(speed=0), 0, [track()])
        cmd = control.command(state(tick=2, speed=0), 2, after)
        assert cmd.brake == 1 and control.diagnostics['is_fallback']


def test_too_close_object_triggers_emergency_brake():
    control = ShowcaseController('episode', 'ego')
    cmd = control.command(state(), 0, [track(station=5)])
    assert cmd.brake == 1 and cmd.throttle == 0
    assert control.diagnostics['reason'] == 'object_clearance_emergency'


def test_requested_stop_is_latched_without_claiming_error_and_retains_steering():
    control = ShowcaseController('episode', 'ego')
    cmd = control.command(state(), 0, [], stop=True)
    assert cmd.throttle == 0 and cmd.brake == 1 and cmd.steering_rad > 0
    assert not control.diagnostics['is_fallback']
    cmd = control.command(state(2, station=.25), 2, [], stop=False)
    assert cmd.brake == 1 and control.diagnostics['reason'] == 'requested_stop'


def test_clock_and_state_guardrails():
    control = ShowcaseController('episode', 'ego')
    control.command(state(), 0, [])
    with pytest.raises(ValueError):
        control.command(state(), 0, [])
    control = ShowcaseController('episode', 'ego')
    control.command(state(), 0, [])
    cmd = control.command(state(2, station=10), 2, [])
    assert cmd.brake == 1 and control.diagnostics['reason'] == 'implausible_or_missing_ego_progress'


def test_controller_tracks_reference_across_seam_and_counts_completed_change_once():
    control = ShowcaseController('episode', 'ego')
    start = 500 * math.tau - 30
    speed = 15.
    for tick in range(0, 800, 2):
        station = start + speed * tick / 120
        if tick == 0:
            ego = state(tick, station, speed=speed)
        else:
            pose = control.path.pose(station)
            ego = VehicleState('episode', 'ego', tick, *pose, speed)
        obj = track(tick, station=start + 80 + 7 * tick / 120)
        cmd = control.command(ego, tick, [obj])
        assert not control.diagnostics['is_fallback'], control.diagnostics
        assert math.isfinite(cmd.steering_rad)
    assert control.diagnostics['lane_changes'] == 1
    assert control.diagnostics['target_lane'] == 1
    assert control.diagnostics['seam_crossings'] == 1


def test_pass_counter_needs_initially_ahead_and_then_fully_behind():
    control = ShowcaseController('episode', 'ego', initial_lane=1)
    for tick in range(0, 700, 2):
        station = 15 * tick / 120
        obj = track(tick, station=30 + 7 * tick / 120)
        control.command(state(tick, station=station, lane=1), tick, [obj])
    assert control.diagnostics['passes'] == 1
    assert control.diagnostics['passed_ids'] == ['slow_01']


def test_deterministic_controller_repeats_identical_commands():
    def run():
        c = ShowcaseController('episode', 'ego')
        result = []
        for tick in range(0, 120, 2):
            station = 15 * tick / 120
            pose = c.path.pose(station)
            ego = VehicleState('episode', 'ego', tick, *pose, 15)
            result.append(c.command(ego, tick, [track(tick, station=80 + 7 * tick / 120)]))
        return result
    assert run() == run()
