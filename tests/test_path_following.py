"""CPU contracts/geometric tests, not a substitute for PhysX road trials."""
from dataclasses import asdict, replace
import math
from pathlib import Path

import pytest

from traffic.driver_control import DriverControlGate
from traffic.lane_geometry import LaneRoute, load_routes
from traffic.path_following import (
    FRAME, SOURCE, BehaviorIntent, FollowerConfig, PathFollower, PathPlanner,
    PathReference, ScriptedCruiseBehavior, VehicleState,
)


@pytest.fixture
def routes():
    return load_routes(Path(__file__).resolve().parents[1] / 'scenarios/physics-road/routes.json')


def state(tick=0, **kwargs):
    return replace(VehicleState('episode', 'ego', tick, 0, 0, 0, 0), **kwargs)


def reference(tick=0, **kwargs):
    return replace(PathReference('episode', 'ego', 'straight100', tick, tick + 24, 3.0, 100.0), **kwargs)


def test_behavior_planner_controller_gate_responsibilities(routes):
    behavior = ScriptedCruiseBehavior('episode', 'ego', 'straight100')
    intent = behavior.decide(0)
    assert isinstance(intent, BehaviorIntent)
    planner = PathPlanner(routes, 'episode', 'ego')
    path = planner.plan(intent, 0)
    assert isinstance(path, PathReference)
    assert path.expires_tick == 24
    assert path.stop_s_m == 100
    follower = PathFollower(routes, 'episode', 'ego')
    command = follower.command(state(), 0, path)
    gate = DriverControlGate('episode', 'ego')
    applied = gate.step(tick=0, dt_s=1 / 120, command=command)
    assert applied.throttle == 1
    assert applied.brake == 0
    assert applied.steering_rad == 0
    assert follower.last_diagnostics['rear_progress_m'] == -1.6
    assert follower.last_diagnostics['center_progress_m'] == 0
    assert follower.last_diagnostics['source'] == SOURCE
    assert follower.last_diagnostics['fallback'] is False


def test_plan_cannot_renew_expiring_intent(routes):
    planner = PathPlanner(routes, 'episode', 'ego')
    intent = ScriptedCruiseBehavior('episode', 'ego', 'straight100').decide(0)
    assert planner.plan(intent, 30).expires_tick == 36
    assert planner.plan(intent, 36) is None


@pytest.mark.parametrize('changes', [
    {'episode_id': 'old'}, {'vehicle_id': 'peer'}, {'path_id': 'missing'},
    {'target_speed_m_s': math.nan}, {'target_speed_m_s': 4},
    {'target_speed_m_s': -1}, {'issued_tick': 2}, {'expires_tick': 0},
    {'expires_tick': 500}, {'issued_tick': True},
])
def test_invalid_behavior_is_not_planned(routes, changes):
    planner = PathPlanner(routes, 'episode', 'ego')
    intent = ScriptedCruiseBehavior('episode', 'ego', 'straight100').decide(0)
    assert planner.plan(replace(intent, **changes), 0) is None


@pytest.mark.parametrize('changes', [
    {'episode_id': 'old'}, {'vehicle_id': 'peer'}, {'x_m': math.nan},
    {'y_m': math.inf}, {'yaw_rad': -math.inf}, {'speed_m_s': math.nan},
    {'speed_m_s': -1}, {'speed_m_s': True}, {'tick': 10}, {'tick': -1},
    {'frame': 'sumo_front_bumper'}, {'source': 'lidar'},
])
def test_invalid_state_commands_brake_not_old_drive(routes, changes):
    follower = PathFollower(routes, 'episode', 'ego')
    command = follower.command(state(**changes), 0, reference())
    assert command.throttle == 0 and command.brake == 1
    assert command.steering_rad == 0
    assert follower.last_diagnostics['fallback'] is True


@pytest.mark.parametrize('changes', [
    {'episode_id': 'old'}, {'vehicle_id': 'peer'}, {'path_id': 'missing'},
    {'target_speed_m_s': math.nan}, {'target_speed_m_s': 4},
    {'stop_s_m': math.inf}, {'stop_s_m': 101}, {'stop_s_m': 0},
    {'issued_tick': 1}, {'expires_tick': 0}, {'expires_tick': 500},
    {'frame': 'unknown'}, {'source': 'lidar'},
])
def test_invalid_reference_commands_brake(routes, changes):
    follower = PathFollower(routes, 'episode', 'ego')
    command = follower.command(state(), 0, reference(**changes))
    assert command.throttle == 0 and command.brake == 1
    assert follower.last_diagnostics['fallback'] is True


def test_missing_and_stale_inputs_fail_safe_then_can_recover(routes):
    follower = PathFollower(routes, 'episode', 'ego')
    assert follower.command(state(), 0, reference()).throttle > 0
    assert follower.command(state(2), 2, None).brake == 1
    assert follower.command(state(4), 4, reference()).throttle > 0
    assert follower.command(state(4), 8, reference()).brake == 1
    assert follower.last_diagnostics['reason'] == 'stale_or_future_state'
    assert follower.command(state(24), 24, reference()).brake == 1
    assert follower.last_diagnostics['reason'] == 'stale_or_future_reference'


def test_reference_expiry_limits_driver_command_lifetime(routes):
    follower = PathFollower(routes, 'episode', 'ego')
    command = follower.command(state(22), 22, reference())
    assert command.expires_tick == 24
    gate = DriverControlGate('episode', 'ego')
    assert gate.step(tick=22, dt_s=1 / 120, command=command).throttle > 0
    assert gate.step(tick=24, dt_s=1 / 120).is_fallback


def test_command_dropout_gate_stops_independently(routes):
    follower = PathFollower(routes, 'episode', 'ego')
    command = follower.command(state(), 0, reference())
    gate = DriverControlGate('episode', 'ego')
    assert gate.step(tick=0, dt_s=1 / 120, command=command).throttle > 0
    applied = gate.step(tick=12, dt_s=1 / 120)
    assert applied.throttle == 0 and applied.brake == 1 and applied.is_fallback


def test_replayed_or_conflicting_references_rejected(routes):
    follower = PathFollower(routes, 'episode', 'ego')
    follower.command(state(12), 12, reference(12))
    assert follower.command(state(14), 14, reference(0)).brake == 1
    assert follower.last_diagnostics['reason'] == 'replayed_or_conflicting_reference'
    assert follower.command(state(16), 16, reference(12, target_speed_m_s=2)).brake == 1
    assert follower.last_diagnostics['reason'] == 'replayed_or_conflicting_reference'


def test_clock_error_requires_runner_stop_and_reset_clears_sequence(routes):
    follower = PathFollower(routes, 'episode', 'ego')
    assert follower.command(state(), 0, reference()).sequence == 0
    with pytest.raises(ValueError):
        follower.command(state(), 0, reference())
    follower.reset(episode_id='episode', vehicle_id='ego')
    assert follower.command(state(), 0, reference()).sequence == 0


@pytest.mark.parametrize('offset, sign', [(0.5, -1), (-0.5, 1)])
def test_pure_pursuit_turns_back_toward_centerline(routes, offset, sign):
    follower = PathFollower(routes, 'episode', 'ego')
    command = follower.command(state(y_m=offset, speed_m_s=3), 0, reference())
    assert 0 < sign * command.steering_rad <= 0.5


def test_gate_applies_steering_slew_not_follower(routes):
    follower = PathFollower(routes, 'episode', 'ego')
    command = follower.command(state(y_m=0.8), 0, reference())
    assert command.steering_rad < -0.05
    gate = DriverControlGate('episode', 'ego')
    applied = gate.step(tick=0, dt_s=1 / 120, command=command)
    assert applied.steering_rad == pytest.approx(-0.5 / 120)


def test_left_and_right_curves_are_symmetric(routes):
    steering = []
    for name in ('left_r60', 'right_r60'):
        point = routes[name].evaluate(40)
        follower = PathFollower(routes, 'episode', 'ego')
        command = follower.command(state(x_m=point.position_xy[0], y_m=point.position_xy[1],
                                         yaw_rad=point.yaw_rad, speed_m_s=3),
                                   0, reference(path_id=name))
        steering.append(command.steering_rad)
    assert steering[0] > 0
    assert steering[1] == pytest.approx(-steering[0], abs=1e-12)


def test_wrong_heading_and_off_lane_fail_safe(routes):
    for sample in (state(yaw_rad=math.pi), state(y_m=2.0)):
        follower = PathFollower(routes, 'episode', 'ego')
        command = follower.command(sample, 0, reference())
        assert command.throttle == 0 and command.brake == 1
        assert follower.last_diagnostics['fallback'] is True


def test_end_stop_uses_chassis_station_and_latches_hold(routes):
    follower = PathFollower(routes, 'episode', 'ego')
    near = follower.command(state(x_m=99.0, speed_m_s=3.0), 0, reference())
    assert near.brake > 0 and near.throttle == 0
    assert follower.last_diagnostics['target_speed_m_s'] == pytest.approx(math.sqrt(2 * 1.5 * 0.8))
    stop = follower.command(state(2, x_m=99.81, speed_m_s=0.5), 2, reference())
    assert stop.brake == 1 and stop.throttle == 0
    assert follower.last_diagnostics['rear_progress_m'] == pytest.approx(98.21)
    assert follower.last_diagnostics['endpoint_latched'] is True
    # A tiny backwards drift must not restart propulsion after completing.
    hold = follower.command(state(4, x_m=99.7, speed_m_s=0), 4, reference())
    assert hold.brake == 1 and hold.throttle == 0


def test_zero_speed_intent_brakes_without_endpoint_latch(routes):
    follower = PathFollower(routes, 'episode', 'ego')
    command = follower.command(state(speed_m_s=2), 0, reference(target_speed_m_s=0))
    assert command.throttle == 0 and command.brake == 1
    assert follower.last_diagnostics['reason'] == 'behavior_stop'
    assert follower.last_diagnostics['endpoint_latched'] is False


def test_physics_state_adapter_does_not_reinterpret_sumo_pose():
    sample = dict(position_m=[1, 2, 3], yaw_rad=0.5, speed_m_s=2)
    adapted = VehicleState.from_physics(sample, episode_id='e', vehicle_id='v', tick=10)
    assert (adapted.x_m, adapted.y_m, adapted.yaw_rad) == (1, 2, 0.5)
    assert adapted.frame == FRAME and adapted.source == SOURCE


def test_configuration_is_plain_serializable_and_bounded():
    assert asdict(FollowerConfig())['planning_deceleration_m_s2'] == 1.5
    for changes in ({'max_speed_m_s': 6}, {'max_steering_rad': 1},
                    {'throttle_kp': math.nan}, {'state_max_age_ticks': 3},
                    {'command_ttl_ticks': 120}, {'wheelbase_m': -1}):
        with pytest.raises(ValueError):
            FollowerConfig(**changes)


def test_translated_rotated_map_does_not_change_driver_outputs():
    base = LaneRoute.from_dict(dict(route_id='r', width_m=3.6, origin_xy_m=[0, 0],
        initial_yaw_rad=0, segments=[dict(length_m=100, curvature_rad_m=0)]))
    other = LaneRoute.from_dict(dict(route_id='r', width_m=3.6, origin_xy_m=[20, -40],
        initial_yaw_rad=math.pi / 2, segments=[dict(length_m=100, curvature_rad_m=0)]))
    a = PathFollower({'r': base}, 'episode', 'ego').command(
        state(x_m=5, y_m=0.5, speed_m_s=2), 0, reference(path_id='r'))
    b = PathFollower({'r': other}, 'episode', 'ego').command(
        state(x_m=19.5, y_m=-35, yaw_rad=math.pi / 2, speed_m_s=2), 0, reference(path_id='r'))
    assert (a.steering_rad, a.throttle, a.brake) == pytest.approx((b.steering_rad, b.throttle, b.brake))


@pytest.mark.parametrize('route_id', ['straight100', 'left_r60', 'right_r60'])
def test_complete_route_in_cpu_bicycle_fixture(routes, route_id):
    """Only an algorithm sanity check; this toy model is NOT PhysX evidence."""
    behavior = ScriptedCruiseBehavior('episode', 'ego', route_id)
    planner = PathPlanner(routes, 'episode', 'ego')
    follower = PathFollower(routes, 'episode', 'ego')
    gate = DriverControlGate('episode', 'ego')
    dt, speed, yaw = 1 / 120, 0.0, 0.02
    # Chassis begins 0.2 m left of the centerline; initial pose set once.
    rear_x, rear_y = -1.6 * math.cos(yaw), 0.2 - 1.6 * math.sin(yaw)
    errors, last_five_speeds = [], []
    for tick in range(48 * 120):
        x, y = rear_x + 1.6 * math.cos(yaw), rear_y + 1.6 * math.sin(yaw)
        if tick % 12 == 0:
            path = planner.plan(behavior.decide(tick), tick)
        command = None
        if tick % 2 == 0:
            sample = state(tick, x_m=x, y_m=y, yaw_rad=yaw, speed_m_s=speed)
            command = follower.command(sample, tick, path)
            assert follower.last_diagnostics['fallback'] is False
        applied = gate.step(tick=tick, dt_s=dt, command=command)
        # Deliberately crude forward-only acceleration fixture with no tires.
        speed = max(0.0, speed + (2.0 * applied.throttle - 6.0 * applied.brake) * dt)
        rear_x += speed * math.cos(yaw) * dt
        rear_y += speed * math.sin(yaw) * dt
        yaw += speed / 3.2 * math.tan(applied.steering_rad) * dt
        errors.append(abs(routes[route_id].project(x, y).lateral_error_m))
        if tick >= 43 * 120:
            last_five_speeds.append(speed)
    end = routes[route_id].project(rear_x + 1.6 * math.cos(yaw), rear_y + 1.6 * math.sin(yaw))
    assert end.s_m == pytest.approx(100.0, abs=0.5)
    assert max(errors) < 0.5
    assert max(last_five_speeds) <= 0.05
    assert follower.last_diagnostics['endpoint_latched'] is True
