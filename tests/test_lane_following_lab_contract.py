"""Synthetic contract checks, never evidence of native dynamics or learning."""
from dataclasses import asdict, replace
import copy
import json
import math
from pathlib import Path
import random
import subprocess
import sys

import pytest

from environments.lane_following_contract import (
    ACTION_NAMES, LaneFollowingTask, LaneTaskConfig, OBSERVATION_NAMES,
    OBSERVATION_SOURCE, bounded_action, make_reset_spec,
)
from traffic.driver_control import AppliedControl
from traffic.lane_geometry import LaneRoute, RouteSegment, load_routes

ROOT = Path(__file__).resolve().parents[1]
ROUTES = load_routes(ROOT/'scenarios/physics-road/routes.json')
BRAKE = AppliedControl(0.0, 0.0, 1.0, 'reset', None, False)
COAST = AppliedControl(0.0, 0.0, 0.0, 'accepted', 1, False)


def state(route=ROUTES['straight100'], *, station=0.0, lateral=0.0, heading=0.0,
          speed=0.0, side_speed=0.0, height=0.8):
    point = route.evaluate(station)
    yaw = point.yaw_rad + heading
    vx = speed*math.cos(point.yaw_rad) - side_speed*math.sin(point.yaw_rad)
    vy = speed*math.sin(point.yaw_rad) + side_speed*math.cos(point.yaw_rad)
    return dict(position_m=[point.position_xy[0]-lateral*math.sin(point.yaw_rad),
                            point.position_xy[1]+lateral*math.cos(point.yaw_rad), height],
                quaternion_xyzw=[0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2)],
                velocity_m_s=[vx, vy, 0.0], yaw_rad=yaw, upright_z=1.0,
                speed_m_s=math.hypot(vx, vy), wheel_on_ground=[True]*4)


def initialized(route=ROUTES['straight100']):
    task = LaneFollowingTask(route)
    task.initialize(state(route), BRAKE)
    return task


def settle(task):
    for tick in range(2, 241, 2):
        transition = task.advance(state(task.route), BRAKE, tick=tick)
        assert transition.reward == 0.0
        assert not transition.terminated and not transition.truncated
    return transition


def drive_to_goal(task):
    """Uniform synthetic movement; makes no acceleration/controller claim."""
    settle(task)
    for index in range(1, 1197):
        transition = task.advance(state(task.route, station=index*0.05, speed=3.0),
                                  COAST, tick=240+index*2)
        assert not transition.terminated and not transition.truncated
    transition = task.advance(state(task.route, station=59.8), BRAKE, tick=2634)
    return transition


def test_frozen_contract_keeps_compatibility_action_units_and_old_module_separate():
    from environments.physics_vehicle_contract import OBSERVATION_NAMES as old_names
    assert ACTION_NAMES == ('steering_rad', 'throttle', 'brake')
    assert bounded_action([9, 1, 0.1]) == (0.5, 0.0, 0.1)
    assert len(old_names) == 11 and len(OBSERVATION_NAMES) == 22
    assert len(set(OBSERVATION_NAMES)) == 22
    assert OBSERVATION_SOURCE == 'privileged_simulator_state_and_known_map'
    assert LaneTaskConfig().control_ticks == 2
    json.dumps(asdict(LaneTaskConfig()), allow_nan=False)


@pytest.mark.parametrize('changes', [dict(physics_hz=60), dict(control_hz=True),
    dict(target_speed_m_s=3.1), dict(episode_length_s=36), dict(stop_s_m=100),
    dict(lateral_reset_bound_m=0.2), dict(min_height_m=math.nan)])
def test_configuration_cannot_quietly_expand_frozen_protocol(changes):
    with pytest.raises(ValueError):
        replace(LaneTaskConfig(), **changes)


def test_seeded_reset_is_repeatable_independent_of_global_rng_and_case_order():
    route = ROUTES['left_r60']
    expected = make_reset_spec(route, seed=1101, episode_index=4, perturb=True)
    random.seed(999)
    for _ in range(100):
        random.random()
    make_reset_spec(ROUTES['right_r60'], seed=1102, episode_index=2, perturb=True)
    assert make_reset_spec(route, seed=1101, episode_index=4, perturb=True) == expected
    assert make_reset_spec(route, seed=1102, episode_index=4, perturb=True) != expected
    assert make_reset_spec(route, seed=1101, episode_index=5, perturb=True) != expected
    assert abs(expected.lateral_offset_m) <= 0.15
    assert abs(expected.yaw_error_rad) <= 0.035
    assert expected.station_m == 0
    nominal = make_reset_spec(route, seed=101)
    assert nominal.x_m == nominal.y_m == nominal.yaw_rad == 0


@pytest.mark.parametrize('lateral,heading', [(-.15, -.035), (-.15, .035), (.15, -.035), (.15, .035)])
def test_corner_reset_offsets_are_route_relative(lateral, heading):
    route = LaneRoute('rotated', 3.6, (RouteSegment(100),), (20, -3), math.pi/2)
    reset = make_reset_spec(route, seed=1101, lateral_offset_m=lateral, yaw_error_rad=heading)
    assert reset.x_m == pytest.approx(20-lateral)
    assert reset.y_m == pytest.approx(-3)
    assert reset.yaw_rad == pytest.approx(math.pi/2+heading)
    task = LaneFollowingTask(route)
    obs = task.initialize(state(route, lateral=lateral, heading=heading), BRAKE)
    assert obs[2] == pytest.approx(lateral)


@pytest.mark.parametrize('kwargs', [dict(seed=True), dict(seed=-1), dict(seed=1, episode_index=1.5),
    dict(seed=1, perturb=1), dict(seed=1, lateral_offset_m=.151),
    dict(seed=1, yaw_error_rad=.0351), dict(seed=1, lateral_offset_m=math.nan), dict(seed=2**32)])
def test_bad_reset_requests_reject_before_native_writes(kwargs):
    with pytest.raises(ValueError):
        make_reset_spec(ROUTES['straight100'], **kwargs)


@pytest.mark.parametrize('route_id', ['straight100', 'left_r60', 'right_r60'])
def test_named_observations_use_map_relative_signs_and_known_stopping_profile(route_id):
    task = LaneFollowingTask(ROUTES[route_id])
    task.initialize(state(task.route, lateral=.1, heading=-.03), BRAKE)
    # Directly evaluate a read-only physical boundary; no fake movement of task history.
    physical = state(task.route, station=35, lateral=.1, heading=-.03, speed=2, side_speed=.2)
    relative = task._relative(physical, 300)
    obs = dict(zip(OBSERVATION_NAMES, task._observation(physical, (0.1, .2, 0), 300, relative)))
    assert obs['progress_m'] == pytest.approx(35)
    assert obs['remaining_m'] == pytest.approx(25)
    assert obs['lateral_error_m'] == pytest.approx(.1)
    assert obs['sin_heading_error'] == pytest.approx(math.sin(-.03))
    assert obs['forward_speed_m_s'] == pytest.approx(2)
    assert obs['lateral_speed_m_s'] == pytest.approx(.2)
    assert obs['planar_speed_m_s'] == pytest.approx(math.hypot(2, .2))
    assert obs['target_speed_m_s'] == 3
    near = task._relative(state(task.route, station=59.5), 300)
    assert near.target_speed_m_s == pytest.approx(math.sqrt(.9))
    stopped = task._relative(state(task.route, station=59.8), 300)
    assert stopped.target_speed_m_s == pytest.approx(0, abs=1e-6)
    assert task._relative(physical, 239).target_speed_m_s == 0


@pytest.mark.parametrize('field,value', [
    ('position_m', [0, math.nan, 1]), ('position_m', [0, 1]),
    ('velocity_m_s', [False, 0, 0]), ('speed_m_s', 1),
    ('quaternion_xyzw', [0, 0, 0, 0]), ('upright_z', 2),
    ('wheel_on_ground', [1]*4), ('wheel_on_ground', [True]*3),
])
def test_invalid_native_state_never_becomes_synthetic_finite_observation(field, value):
    sample = state()
    sample[field] = value
    with pytest.raises(ValueError):
        initialized().physics_guard(sample, tick=1)


@pytest.mark.parametrize('sample,kwargs,reason', [
    (state(), {'contact_count': 1}, 'rigid_contact'),
    (state(), {'command_fallback': True}, 'command_fallback'),
    ({**state(), 'upright_z': .99}, {}, 'excessive_tilt'),
    (state(height=2.01), {}, 'unsafe_height'),
    (state(lateral=.901), {}, 'lane_departure'),
    (state(speed=4.01), {}, 'overspeed'),
    (state(speed=-.201), {}, 'reverse_motion'),
    (state(station=-1.01), {}, 'before_route_start'),
    (state(station=60.501), {}, 'endpoint_overshoot'),
])
def test_physics_guard_boundary_failures_are_explicit(sample, kwargs, reason):
    assert initialized().physics_guard(sample, tick=500, **kwargs) == reason


def test_containment_uses_body_not_center_point_and_curvature():
    straight = initialized()
    assert straight.physics_guard(state(lateral=.9), tick=500) is None
    curved = initialized(ROUTES['left_r60'])
    assert curved.physics_guard(state(curved.route, station=30, lateral=.9), tick=500) == 'lane_departure'


def test_support_loss_uses_completed_physics_ticks_and_explicit_settling_exclusion():
    task = initialized()
    unsupported = {**state(), 'wheel_on_ground': [True, True, True, False]}
    assert task.physics_guard(unsupported, tick=240, unsupported_ticks=13) is None
    assert task.physics_guard(unsupported, tick=252, unsupported_ticks=12) is None
    reason = task.physics_guard(unsupported, tick=253, unsupported_ticks=13)
    assert reason == 'wheel_support_loss'
    transition = task.advance(state(), BRAKE, tick=2, failure_reason=reason)
    assert transition.reason == 'wheel_support_loss' and transition.terminated
    with pytest.raises(ValueError):
        task.physics_guard(unsupported, tick=253, unsupported_ticks=True)


def test_transient_physics_failure_survives_later_safe_control_sample():
    task = initialized()
    reason = task.physics_guard(state(lateral=1), tick=1)
    transition = task.advance(state(), BRAKE, tick=2, failure_reason=reason)
    assert transition.terminated and not transition.truncated
    assert transition.reason == 'lane_departure'
    assert transition.reward_components['terminal'] == -10
    assert all(v == 0 for k, v in transition.reward_components.items() if k != 'terminal')
    with pytest.raises(RuntimeError):
        task.advance(state(), BRAKE, tick=4)


def test_progress_reward_is_signed_displacement_once_and_running_costs_scale_with_time():
    task = initialized()
    settle(task)
    samples = [task.advance(state(station=index*.05, speed=3), COAST, tick=240+index*2)
               for index in range(1, 61)]
    assert sum(row.reward_components['progress'] for row in samples) == pytest.approx(1)
    assert sum(row.reward_components['time'] for row in samples) == pytest.approx(-.05)
    assert sum(row.reward_components['speed'] for row in samples) == 0
    before = samples[-1].route_state.progress_m
    backward = task.advance(state(station=before-.001, speed=-.06), COAST, tick=362)
    assert backward.reward_components['progress'] == pytest.approx(-.001/3)
    assert not backward.terminated


def test_steering_cost_uses_applied_rate_and_not_raw_policy_request():
    task = initialized()
    settle(task)
    applied = AppliedControl(.5/60, .3, 0, 'accepted', 1, False)
    transition = task.advance(state(), applied, tick=242)
    assert transition.reward_components['steering_rate'] == pytest.approx(-.01/60)
    with pytest.raises(ValueError):
        task.advance(state(), AppliedControl(0, .5, .5, 'bad', 2, False), tick=244)


def test_teleport_cannot_manufacture_success_or_large_progress_reward():
    task = initialized()
    settle(task)
    transition = task.advance(state(station=60), BRAKE, tick=242)
    assert transition.reason == 'state_discontinuity'
    assert transition.terminated and not transition.success
    assert transition.reward_components['progress'] == 0


def test_success_needs_full_observed_progress_and_complete_continuous_stopped_second():
    task = initialized()
    entered = drive_to_goal(task)
    assert entered.observation[-1] == 0
    for index in range(1, 60):
        transition = task.advance(state(station=59.8), BRAKE, tick=2634+index*2)
        assert not transition.terminated
    transition = task.advance(state(station=59.8), BRAKE, tick=2754)
    assert transition.terminated and transition.success and not transition.truncated
    assert transition.reason == 'success'
    assert transition.observation[-1] == 1
    assert transition.reward_components['terminal'] == 10
    saved = copy.deepcopy(transition.as_dict())
    assert transition.terminal_observation == transition.observation
    task.initialize(state(), BRAKE)
    assert transition.as_dict() == saved
    assert saved['route_state']['progress_m'] == pytest.approx(59.8)
    assert saved['terminal_observation'][0] == pytest.approx(59.8)
    json.dumps(saved, allow_nan=False)


def test_hold_interrupts_on_motion_and_stopped_at_start_never_succeeds():
    task = initialized()
    drive_to_goal(task)
    first = task.advance(state(station=59.8), BRAKE, tick=2636)
    assert first.observation[-1] > 0
    moving = task.advance(state(station=59.801, speed=.06), COAST, tick=2638)
    assert moving.observation[-1] == 0 and not moving.success
    parked = initialized()
    for tick in range(2, 4201, 2):
        transition = parked.advance(state(), BRAKE, tick=tick)
    assert transition.truncated and not transition.terminated and not transition.success
    assert transition.reason == 'time_limit'
    assert transition.reward_components['terminal'] == 0
    assert transition.terminal_observation[0] == 0


def test_failure_wins_on_exact_timeout_boundary():
    task = initialized()
    for tick in range(2, 4200, 2):
        task.advance(state(), BRAKE, tick=tick)
    transition = task.advance(state(), BRAKE, tick=4200, contact_count=1)
    assert transition.terminated and not transition.truncated
    assert transition.reason == 'rigid_contact'


def test_failure_wins_on_exact_success_boundary():
    task = initialized()
    drive_to_goal(task)
    for tick in range(2636, 2754, 2):
        task.advance(state(station=59.8), BRAKE, tick=tick)
    transition = task.advance(state(station=59.8), BRAKE, tick=2754, failure_reason='rigid_contact')
    assert transition.reason == 'rigid_contact'
    assert transition.terminated and not transition.success
    assert transition.reward_components['terminal'] == -10


def test_success_wins_over_time_limit_when_its_full_hold_ends_exactly_at_deadline():
    task = initialized()
    for tick in range(2, 1687, 2):
        task.advance(state(), BRAKE, tick=tick)
    for index in range(1, 1197):
        task.advance(state(station=index*.05, speed=3), COAST, tick=1686+index*2)
    task.advance(state(station=59.8), BRAKE, tick=4080)
    for tick in range(4082, 4201, 2):
        transition = task.advance(state(station=59.8), BRAKE, tick=tick)
    assert transition.reason == 'success'
    assert transition.terminated and transition.success and not transition.truncated


def test_reset_cannot_begin_at_goal_or_keep_old_terminal_state():
    task = initialized()
    with pytest.raises(ValueError, match='start envelope'):
        task.initialize(state(station=60), BRAKE)
    failed = task.advance(state(), BRAKE, tick=2, contact_count=1)
    reset_obs = task.initialize(state(lateral=.1, heading=.01), BRAKE)
    assert reset_obs[0] == 0 and reset_obs[-1] == 0
    assert failed.terminal_observation[0] == 0
    assert not task.advance(state(lateral=.1, heading=.01), BRAKE, tick=2).terminated


@pytest.mark.parametrize('tick', [0, 1, 3, -1, True, 2.0])
def test_duplicate_missing_out_of_order_and_wrong_type_ticks_reject(tick):
    with pytest.raises(ValueError):
        initialized().advance(state(), BRAKE, tick=tick)


def test_task_import_does_not_load_native_or_learning_backends():
    code = ('import environments.lane_following_contract,sys; '
            "assert not {'torch','isaaclab','isaacsim','traci','pettingzoo','stable_baselines3'} & sys.modules.keys()")
    subprocess.run([sys.executable, '-c', code], cwd=ROOT, check=True)


def test_unsupported_chassis_and_self_bending_route_reject():
    with pytest.raises(ValueError, match='chassis'):
        LaneFollowingTask(ROUTES['straight100'], dimensions_m=(4.8, 2, 1.4))
    route = LaneRoute('switchback', 3.6, (RouteSegment(50, 1/60), RouteSegment(50, -1/60)))
    with pytest.raises(ValueError, match='single-direction'):
        LaneFollowingTask(route)
