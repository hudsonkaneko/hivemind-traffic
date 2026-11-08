import io
import json
from unittest.mock import MagicMock

import numpy as np
import pytest

from environments.highway_stage1_env import (
    HighwayStage1Env, LOCAL_FEATURES, MESSAGE_FEATURES, DEFAULT_REWARD_WEIGHTS,
    SCHEMA, acceleration_constraints, feasible_acceleration_target,
)
from tests.test_highway_merge_context import graph, vehicle


def bare_env(graph, state):
    env = HighwayStage1Env.__new__(HighwayStage1Env)
    env.graph = graph
    env.agents = [state['id']]
    env.states = {state['id']: state}
    env.previous = {state['id']: (0., 0)}
    env.last_change = {state['id']: -5.}
    env.inbox = {state['id']: {}}
    env.time = env.start_time = 0.
    env.communication = 'none'
    env.packet_loss = 0.
    env.horizon = 800
    env.task, env.density, env.control_mode = 'cruise', 'light', 'policy'
    env.steps = 0
    env.merged, env.passed, env.arrived, env.collisions = set(), set(), set(), set()
    env.conn = MagicMock()
    env.journal = io.StringIO()
    env.reward_weights = dict(DEFAULT_REWARD_WEIGHTS)
    env.stats = dict(collisions=0, teleports=0, overrides=0, agent_steps=0,
        lane_changes=0, successful_merges=0, passes=0, delivered_messages=0,
        dropped_messages=0, delivered_bytes=0, overspeed_steps=0,
        peak_abs_jerk=0., reward_terms={}, speed_ratio_sum=0., speed_samples=0,
        override_causes={}, policy_filter_changes=0, native_realization_changes=0,
        emergency_braking_observed=0, acceleration_error_sum=0.,
        abs_acceleration_sum=0., abs_jerk_sum=0., minimum_ttc_s=None, minimum_headway_s=None)
    env.obs = env._observations()
    return env


def test_versioned_observation_layout_and_missing_agent_state(graph):
    env = bare_env(graph, vehicle('agent_0', 'sb1_1', 100.))
    obs = env.obs['agent_0']
    assert SCHEMA == 'highway_stage1_graph_messages_v2'
    assert len(LOCAL_FEATURES) == 45 and len(MESSAGE_FEATURES) == 8
    assert obs.shape == (93,) and obs[31] == 1 and np.all(obs[45:] == 0)
    state = env.state()
    assert state.shape == (376,) and state[93] == 1 and np.all(state[94:] == 0)


def test_constraint_boundaries_and_feasible_target_are_current_state():
    state = vehicle('agent_0', 'sb1_1', 100., speed=25., allowed_speed=25.)
    bounds, ceiling = acceleration_constraints(state, None, None)
    assert ceiling == 0 and bounds == dict(speed_ceiling=0., following=None, lane_end=None)
    assert feasible_acceleration_target(state, None, None) == 0
    _, close = acceleration_constraints(state, {'bumper_gap_m': 3., 'relative_speed_m_s': -15.}, None)
    assert close == -4.5
    _, ending = acceleration_constraints(state, None, 2.5)
    assert ending == -4.5


def test_reward_target_respects_predicted_acc_response_without_changing_guard():
    state = vehicle('agent_0', 'sb1_1', 100., speed=20., allowed_speed=25.)
    front = dict(bumper_gap_m=40., relative_speed_m_s=0.)
    before_bounds, before_ceiling = acceleration_constraints(state, front, None)
    target = feasible_acceleration_target(state, front, None)
    # Native ordinary gap-closing response prediction: .04*(40-2.5-1.5*20).
    assert target == pytest.approx(.3)
    assert before_ceiling == 2.
    assert acceleration_constraints(state, front, None) == (before_bounds, before_ceiling)


def test_no_leader_reward_taper_preserves_cruise_and_lane_end_bounds():
    state = vehicle('agent_0', 'sb1_1', 100., speed=10., allowed_speed=25.)
    assert feasible_acceleration_target(state, None, None) == 2.
    assert feasible_acceleration_target(state, None, 2.5) == -4.5


def test_native_difference_is_measured_without_invented_attribution(graph):
    env = bare_env(graph, vehicle('agent_0', 'sb1_1', 100., speed=25., allowed_speed=25.))
    def advance():
        env.time += .1
        env.states = {'agent_0': dict(env.states['agent_0'], speed=24.98, acceleration=-.2, distance=102.498)}
    env._advance = advance
    _, _, _, _, infos = env.step({'agent_0': np.array([1., 0.], np.float32)})
    info = infos['agent_0']
    assert info['requested_acceleration'] == 2. and info['filtered_acceleration'] == 0.
    assert info['realized_acceleration'] == -.2 and info['native_cause'] == 'unattributed'
    assert set(info['override_causes']) == {'speed_ceiling', 'native_realization_unattributed'}
    assert info['policy_filter_changed'] and info['native_realization_changed']
    assert env.stats['overrides'] == 1 and env.stats['agent_steps'] == 1
    assert json.loads(env.journal.getvalue())['agents']['agent_0']['acceleration_bounds']['speed_ceiling'] == 0.


def test_native_baseline_counterfactual_is_not_counted_as_intervention(graph):
    env = bare_env(graph, vehicle('agent_0', 'sb1_1', 100., speed=25., allowed_speed=25.))
    env.control_mode = 'native'
    env._advance = lambda: None
    _, _, _, _, infos = env.step({'agent_0': np.array([1., 0.], np.float32)})
    info = infos['agent_0']
    assert not info['command_actually_applied'] and info['counterfactual_filter_difference']
    assert not info['override'] and not info['policy_filter_changed']
    assert info['override_causes'] == [] and env.stats['overrides'] == 0
    env.conn.vehicle.setAcceleration.assert_not_called()


def test_removed_vehicle_has_no_fabricated_realized_acceleration(graph):
    env = bare_env(graph, vehicle('agent_0', 'sb1_1', 100.))
    def advance():
        env.states = {}
        env.arrived = {'agent_0'}
    env._advance = advance
    observation, _, terminated, _, infos = env.step({'agent_0': np.array([0., 0.], np.float32)})
    assert terminated['agent_0'] and infos['agent_0']['realized_acceleration'] is None
    assert np.all(observation['agent_0'] == 0) and not env.agents


def test_invalid_joint_action_sends_no_commands(graph):
    env = bare_env(graph, vehicle('agent_0', 'sb1_1', 100.))
    with pytest.raises(ValueError, match='hybrid'):
        env.step({'agent_0': np.array([np.nan, 0.], np.float32)})
    env.conn.vehicle.setAcceleration.assert_not_called()
    env.conn.vehicle.changeLaneRelative.assert_not_called()


def test_lane_gap_guard_still_blocks_close_rear(graph):
    env = bare_env(graph, vehicle('agent_0', 'sb2_0', 5., ('mlk_on', 'sb2', 'sb3', 'sb4')))
    env.states['background'] = vehicle('background', 'sb1_1', 195., speed=25.)
    context = env._context('agent_0')
    assert env._mask('agent_0', context)[2] == 0
    assert env._lane_rejection_causes('agent_0', context, 1) == ['lane_gap']
