"""CPU contract/geometry checks; none of these certify vehicle dynamics."""
from dataclasses import replace
import json
import math
from pathlib import Path

import pytest

from traffic.driver_control import DriverControlGate
from traffic.path_following import FRAME, SOURCE, VehicleState
from traffic.loop_driving import CircularLoop, LoopFollower, LoopFollowerConfig, LoopProgress, LoopReference


ROOT = Path(__file__).resolve().parents[1]


def state(tick=0, station_m=0.0, speed=0.0, **changes):
    point = CircularLoop().evaluate(station_m)
    return replace(VehicleState('episode', 'ego', tick, *point.position_xy,
                                point.yaw_rad, speed), **changes)


def reference(tick=0, speed=3.0, **changes):
    return replace(LoopReference('episode', 'ego', 'MainLane1', tick, tick + 24, speed), **changes)


def follower(**kwargs):
    return LoopFollower(CircularLoop(), 'episode', 'ego', **kwargs)


def navigation():
    route = CircularLoop()
    return dict(schema_version=2, units='meters', up_axis='Z', east_axis='+X',
        north_axis='+Y', forward_axis='+X', traffic_direction='counterclockwise',
        point_reference='surface centerline; apply vehicle-specific height offset',
        lanes={'MainLane1': dict(id='MainLane1', closed=True, radius_m=500, width_m=3.7,
            points=[[*route.evaluate(route.length_m * i / 16).position_xy, 0] for i in range(16)],
            speed_hint_m_s=29.0576)})


def load_fixture(tmp_path, doc):
    path = tmp_path / 'navigation.json'
    path.write_text(json.dumps(doc), encoding='utf-8')
    return CircularLoop.from_navigation(path)


def test_published_v02_route_is_loaded_and_speed_hint_does_not_set_target():
    route = CircularLoop.from_navigation(ROOT / 'highway_usd/_v02/navigation.json')
    assert route.radius_m == 500 and route.width_m == 3.7
    assert route.length_m == pytest.approx(3141.592653589793)
    control = LoopFollower(route, 'episode', 'ego')
    control.command(state(), 0, reference(speed=3))
    assert control.last_diagnostics['target_speed_m_s'] == 3


@pytest.mark.parametrize('key,value', [
    ('schema_version', 1), ('units', 'cm'), ('up_axis', 'Y'), ('east_axis', '-X'),
    ('north_axis', '-Y'), ('forward_axis', '-X'), ('traffic_direction', 'clockwise'),
    ('point_reference', 'front bumper'), ('lanes', []),
])
def test_navigation_contract_mismatch_is_rejected(tmp_path, key, value):
    doc = navigation()
    doc[key] = value
    with pytest.raises(ValueError):
        load_fixture(tmp_path, doc)


@pytest.mark.parametrize('key,value', [
    ('id', 'wrong'), ('closed', False), ('closed', 1), ('radius_m', None),
    ('radius_m', math.nan), ('width_m', -1), ('points', []), ('points', [[0, 0, 0]] * 16),
])
def test_non_circle_lane_is_rejected(tmp_path, key, value):
    doc = navigation()
    doc['lanes']['MainLane1'][key] = value
    with pytest.raises(ValueError):
        load_fixture(tmp_path, doc)


@pytest.mark.parametrize('replacement', [[500, 0], [500, 0, math.nan], [500, 0, 1], [499, 0, 0], [True, 0, 0]])
def test_bad_navigation_sample_is_rejected(tmp_path, replacement):
    doc = navigation()
    doc['lanes']['MainLane1']['points'][0] = replacement
    with pytest.raises(ValueError):
        load_fixture(tmp_path, doc)


def test_clockwise_or_duplicate_samples_rejected(tmp_path):
    doc = navigation()
    doc['lanes']['MainLane1']['points'].reverse()
    with pytest.raises(ValueError):
        load_fixture(tmp_path, doc)
    doc = navigation()
    doc['lanes']['MainLane1']['points'].append(doc['lanes']['MainLane1']['points'][0])
    with pytest.raises(ValueError):
        load_fixture(tmp_path, doc)


@pytest.mark.parametrize('changes', [
    dict(radius_m=0), dict(radius_m=True), dict(radius_m=math.inf), dict(radius_m=1e308), dict(width_m=1000),
    dict(route_id=''), dict(center_xy_m=(0,)), dict(center_xy_m=(0, math.nan)),
])
def test_circle_constructor_guards(changes):
    with pytest.raises(ValueError):
        CircularLoop(**changes)


def test_periodic_geometry_projection_and_lateral_sign():
    route = CircularLoop(center_xy_m=(10, 20))
    for station in (-route.length_m, 0, route.length_m, 3 * route.length_m):
        assert route.evaluate(station).position_xy == pytest.approx((510, 20))
        assert route.evaluate(station).yaw_rad == pytest.approx(math.pi / 2)
    assert route.project(509, 20).lateral_error_m == pytest.approx(1)
    assert route.project(511, 20).lateral_error_m == pytest.approx(-1)
    point = route.evaluate(-0.5)
    projection = route.project(*point.position_xy)
    assert projection.s_m == pytest.approx(route.length_m - .5)
    assert projection.lateral_error_m == pytest.approx(0)
    with pytest.raises(ValueError):
        route.project(10, 20)
    with pytest.raises(ValueError):
        route.evaluate(math.nan)
    with pytest.raises(ValueError):
        route.project(True, 0)


def test_seam_crossing_is_not_completed_lap_and_reverse_subtracts():
    route = CircularLoop()
    progress = LoopProgress(route)
    progress.update(route.length_m - 0.1)
    info = progress.update(0.1)
    assert info['seam_crossings'] == 1 and info['completed_laps'] == 0
    assert info['progress_m'] == pytest.approx(.2)
    info = progress.update(route.length_m - .2)
    assert info['seam_crossings'] == 0 and info['completed_laps'] == 0
    assert info['progress_m'] == pytest.approx(-.1)
    assert info['reverse_from_high_water_m'] == pytest.approx(.3)


def test_completed_laps_require_full_net_distance_from_spawn():
    route = CircularLoop()
    progress = LoopProgress(route)
    initial = route.length_m - 20
    progress.update(initial)
    for index in range(1, 3 * 3600 + 1):
        progress.update((initial + route.length_m * index / 3600) % route.length_m)
    info = progress.snapshot()
    assert info['progress_m'] == pytest.approx(3 * route.length_m)
    assert info['completed_laps'] == 3
    assert info['seam_crossings'] == 3


def test_progress_rejects_ambiguous_or_invalid_station_without_mutation():
    route = CircularLoop()
    progress = LoopProgress(route)
    progress.update(0)
    for station in (route.length_m / 2, route.length_m, -1, True, math.nan):
        with pytest.raises(ValueError):
            progress.update(station)
    assert progress.distance_m == 0


@pytest.mark.parametrize('speed', [3.0, 6.0, 15.6464])
def test_periodic_control_continues_through_seam_at_each_requested_speed(speed):
    route = CircularLoop()
    control = follower()
    initial = route.length_m - .1
    steering = []
    for tick in range(0, 120, 2):
        command = control.command(state(tick, initial + speed * tick / 120, speed - .1), tick,
                                  reference(tick, speed=speed))
        assert command.throttle > 0 and command.brake == 0
        assert command.expires_tick == tick + 12
        assert not control.last_diagnostics['fallback']
        assert not control.last_diagnostics['stop_latched']
        assert control.last_diagnostics['target_speed_m_s'] == speed
        steering.append(command.steering_rad)
    assert max(steering) - min(steering) < 1e-10
    assert 0 < steering[0] < .02
    assert control.last_diagnostics['seam_crossings'] == 1
    assert control.last_diagnostics['completed_laps'] == 0
    assert control.last_diagnostics['progress_m'] == pytest.approx(speed * 118 / 120)


@pytest.mark.parametrize('changes', [
    dict(episode_id='old'), dict(vehicle_id='other'), dict(tick=-1), dict(tick=True),
    dict(tick=1), dict(x_m=math.nan), dict(y_m=math.inf), dict(yaw_rad=math.nan),
    dict(speed_m_s=-1), dict(speed_m_s=True), dict(speed_m_s=25),
    dict(frame='unknown'), dict(source='lidar'),
])
def test_invalid_state_brakes_and_does_not_accumulate_progress(changes):
    control = follower()
    command = control.command(state(**changes), 0, reference())
    assert command.throttle == 0 and command.brake == 1 and command.steering_rad == 0
    assert control.last_diagnostics['fallback']
    assert control.last_diagnostics['progress_m'] == 0


@pytest.mark.parametrize('changes', [
    dict(episode_id='old'), dict(vehicle_id='other'), dict(path_id='other'),
    dict(issued_tick=1), dict(issued_tick=True), dict(expires_tick=0), dict(expires_tick=25),
    dict(target_speed_m_s=-1), dict(target_speed_m_s=math.nan), dict(target_speed_m_s=16.1),
    dict(target_speed_m_s=True), dict(stop_requested=1), dict(frame='sumo'), dict(source='lidar'),
])
def test_invalid_reference_brakes(changes):
    control = follower()
    command = control.command(state(), 0, reference(**changes))
    assert command.throttle == 0 and command.brake == 1
    assert control.last_diagnostics['fallback']


@pytest.mark.parametrize('which', ['state', 'reference'])
def test_missing_input_brakes(which):
    control = follower()
    command = control.command(None if which == 'state' else state(), 0,
                              None if which == 'reference' else reference())
    assert command.brake == 1 and control.last_diagnostics['fallback']


def test_stale_state_and_reference_brake():
    control = follower()
    command = control.command(state(), 4, reference())
    assert command.brake == 1
    assert control.last_diagnostics['reason'] == 'stale_or_future_state'
    command = control.command(state(24), 24, reference())
    assert command.brake == 1
    assert control.last_diagnostics['reason'] == 'stale_or_future_reference'


def test_expiry_is_capped_by_reference_and_gate_dropout_brakes():
    control = follower()
    command = control.command(state(22), 22, reference())
    assert command.expires_tick == 24
    gate = DriverControlGate('episode', 'ego')
    assert gate.step(tick=22, dt_s=1 / 120, command=command).throttle > 0
    applied = gate.step(tick=24, dt_s=1 / 120)
    assert applied.is_fallback and applied.throttle == 0 and applied.brake == 1


def test_reference_and_state_conflicts_cannot_mutate_stop_or_progress():
    control = follower()
    control.command(state(), 0, reference())
    control.command(state(2), 2, reference(stop_requested=True))
    assert control.last_diagnostics['reason'] == 'replayed_or_conflicting_reference'
    assert not control.last_diagnostics['stop_latched']
    control.command(state(4), 4, reference(4))
    control.command(state(4, station_m=.01), 6, reference(6))
    assert control.last_diagnostics['reason'] == 'replayed_or_conflicting_state'


@pytest.mark.parametrize('bad_tick', [-1, True, math.nan, 0])
def test_broken_local_clock_raises(bad_tick):
    control = follower()
    control.command(state(), 0, reference())
    with pytest.raises(ValueError):
        control.command(state(), bad_tick, reference())


def test_stop_latches_until_reset_and_keeps_left_turn_steering():
    control = follower()
    command = control.command(state(speed=6), 0, reference(speed=6, stop_requested=True))
    assert command.throttle == 0 and command.brake == 1 and command.steering_rad > 0
    assert control.last_diagnostics['reason'] == 'requested_stop'
    command = control.command(state(2, speed=5.9), 2, reference(2, speed=6))
    assert command.brake == 1 and control.last_diagnostics['stop_latched']
    control.reset(episode_id='new', vehicle_id='ego')
    command = control.command(state(0, episode_id='new'), 0, reference(episode_id='new'))
    assert command.throttle > 0 and not control.last_diagnostics['stop_latched']
    assert command.sequence == 0 and control.last_diagnostics['progress_m'] == 0


def test_zero_speed_behavior_holds_but_can_resume_without_episode_reset():
    control = follower()
    command = control.command(state(), 0, reference(speed=0))
    assert command.brake == 1 and not control.last_diagnostics['stop_latched']
    assert control.command(state(2), 2, reference(2, speed=3)).throttle > 0


@pytest.mark.parametrize('speed,expected', [(2.0, 'tracking'), (3.0, 'speed_coasting'), (4.0, 'speed_braking')])
def test_speed_servo_is_bounded_and_never_simultaneously_drives_and_brakes(speed, expected):
    control = follower()
    command = control.command(state(speed=speed), 0, reference(speed=3))
    assert control.last_diagnostics['reason'] == expected
    assert 0 <= command.throttle <= 1 and 0 <= command.brake <= 1
    assert command.throttle * command.brake == 0
    assert abs(command.steering_rad) <= .5


def test_off_lane_wrong_heading_and_circle_center_fail_safe():
    cases = [
        (dict(x_m=502), 'outside_lane_recovery_envelope'),
        (dict(yaw_rad=-math.pi / 2), 'heading_outside_recovery_envelope'),
        (dict(x_m=0, y_m=0), 'invalid_route_projection'),
    ]
    for changes, reason in cases:
        control = follower()
        assert control.command(state(**changes), 0, reference()).brake == 1
        assert control.last_diagnostics['reason'] == reason


def test_reverse_motion_accumulates_signed_progress_and_then_brakes():
    control = follower()
    control.command(state(station_m=10), 0, reference())
    control.command(state(2, station_m=9.96), 2, reference(2))
    assert not control.last_diagnostics['fallback']
    control.command(state(4, station_m=9.92), 4, reference(4))
    assert not control.last_diagnostics['fallback']
    command = control.command(state(6, station_m=9.88), 6, reference(6))
    assert command.brake == 1 and control.last_diagnostics['reason'] == 'wrong_way_progress'
    assert control.last_diagnostics['progress_m'] == pytest.approx(-.12)


def test_teleport_and_long_sample_gap_fail_without_inventing_laps():
    control = follower()
    control.command(state(), 0, reference())
    command = control.command(state(2, station_m=10), 2, reference(2))
    assert command.brake == 1
    assert control.last_diagnostics['reason'] == 'implausible_route_progress'
    assert control.last_diagnostics['progress_m'] == 0
    command = control.command(state(30), 30, reference(30))
    assert command.brake == 1 and control.last_diagnostics['reason'] == 'progress_sample_gap'
    command = control.command(state(32), 32, reference(32))
    assert command.brake == 1 and control.last_diagnostics['reason'] == 'progress_sample_gap'
    control.reset(episode_id='episode', vehicle_id='ego')
    assert control.command(state(32), 32, reference(32)).throttle > 0


def test_follower_fallback_and_gate_fallback_are_different_diagnostics():
    control = follower()
    command = control.command(state(), 0, None)
    gate = DriverControlGate('episode', 'ego')
    applied = gate.step(tick=0, dt_s=1 / 120, command=command)
    assert control.last_diagnostics['is_fallback']
    assert not applied.is_fallback
    assert applied.brake == 1 and applied.throttle == 0


@pytest.mark.parametrize('changes', [
    dict(physics_hz=True), dict(physics_hz=0), dict(max_speed_m_s=17), dict(max_steering_rad=.6),
    dict(max_observed_speed_m_s=15), dict(progress_slack_m=0), dict(reverse_tolerance_m=math.inf),
    dict(max_sample_gap_ticks=25), dict(max_sample_gap_ticks=True),
])
def test_config_guards(changes):
    with pytest.raises(ValueError):
        LoopFollowerConfig(**changes)


def test_progress_guard_rejects_tiny_ambiguous_loop():
    with pytest.raises(ValueError):
        LoopFollower(CircularLoop(radius_m=1, width_m=.1), 'episode', 'ego')


def test_all_diagnostics_and_commands_are_json_finite():
    control = follower()
    control.command(state(), 0, reference())
    json.dumps(control.last_diagnostics, allow_nan=False)
    assert control.last_diagnostics['source'] == SOURCE
    assert control.last_diagnostics['state_source'] == SOURCE
    assert control.last_diagnostics['path_source'] == SOURCE
    assert FRAME == reference().frame
