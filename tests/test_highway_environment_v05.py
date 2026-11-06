import json

import numpy as np
import pytest

from environments.highway_environment_v05 import (
    HighwayEnvironmentV05Env, ContinuousMotionLaneGraph, LOCAL_FEATURES, MOTION_FEATURES, FEATURES,
    OBSERVATION_SIZE, STATE_SIZE, legacy_observation, teacher_actions,
)
from environments.highway_stage1_env import HighwayStage1Env
from environments.i110_merge_env import I110MergeEnv
from tests.test_highway_merge_context import graph, vehicle
from tests.test_highway_stage1_env import bare_env
from traffic.highway_stage1_teacher import teacher_actions as legacy_teacher_actions
from traffic.highway_v05_metrics import PostMergeTracker


def native_vehicle(name, lane='sb1_1', position=100., **kwargs):
    return dict(vehicle(name, lane, position, **kwargs), lateral_position=0., lateral_speed=0.,
                lane_width=3.7, angle=180., lane_identity={'kind': 'through', 'index_from_right': 0})


def v05_bare(graph, count=1):
    env = bare_env(graph, native_vehicle('agent_0'))
    env.__class__ = HighwayEnvironmentV05Env
    env.graph.__class__ = ContinuousMotionLaneGraph
    env.control_target_version = 'v3'
    env.possible_agents = [f'agent_{i}' for i in range(24)]
    env.agents = env.possible_agents[:count]
    env.states = {a: native_vehicle(a, lane=f'sb1_{1 + i % 4}', position=20. + 30. * (i // 4))
                  for i, a in enumerate(env.agents)}
    env.obs = {}
    env.previous = {a: (0., 0) for a in env.agents}
    env.last_change = {a: -5. for a in env.agents}
    env.inbox = {a: {} for a in env.agents}
    env.controlled_count, env.total_vehicles = count, 24
    env.merge_agents, env.merge_trackers = (), {}
    env.maneuvers, env.completed_maneuvers = {}, {}
    env.completed_this_step = set()
    env.lane_change_duration = 3.
    env.route_trackers = {a: PostMergeTracker(vehicle_id=a) for a in env.agents}
    env.motion_stats = dict(started=0, completed=0, native_refused=0, interrupted=0,
        active_samples=0, nonzero_lateral_samples=0, completion_durations_s=[])
    env.population_stats = dict(peer_observations=0, selected_peer_sum=0,
        maximum_selected_peers=0, overflow_peer_observations=0, zero_peer_observations=0,
        selected_peer_histogram={str(i): 0 for i in range(7)})
    env.obs = env._observations()
    return env


def test_v05_exact_layout_preserves_shared_v3_local_meanings(graph):
    env = v05_bare(graph)
    old = bare_env(graph, env.states['agent_0'])
    old.control_target_version = 'v3'
    old_obs = old._observations()['agent_0']
    observation = env.obs['agent_0']
    assert len(LOCAL_FEATURES) == 51 and len(MOTION_FEATURES) == 6 and len(FEATURES) == 99
    assert observation.shape == (OBSERVATION_SIZE,) == (99,)
    np.testing.assert_array_equal(observation[:45], old_obs[:45])
    np.testing.assert_array_equal(legacy_observation(observation), old_obs)
    assert env.observation_space('agent_0').contains(observation)


@pytest.mark.parametrize('count', [0, 1, 8, 16, 24])
def test_24_critic_slots_and_active_masks_stay_stable(graph, count):
    env = v05_bare(graph, count)
    state = env.state()
    assert state.shape == (STATE_SIZE,) == (2400,)
    rows = state.reshape(24, 100)
    np.testing.assert_array_equal(rows[:, -1], np.r_[np.ones(count), np.zeros(24 - count)])
    assert not rows[count:].any()
    if count:
        env.agents.pop(count // 2)
        rows = env.state().reshape(24, 100)
        assert not rows[count // 2].any()
        assert rows[:, -1].sum() == count - 1


def test_all_24_agents_observe_at_most_six_fresh_messages(graph):
    env = v05_bare(graph, 24)
    env.time, env.communication = 1., 'local'
    for receiver in env.agents:
        for sender in env.agents:
            if receiver != sender:
                s = env.states[sender]
                env.inbox[receiver][sender] = dict(time=.9, x=s['x'], y=s['y'], speed=s['speed'],
                    acceleration=0., request=0., lane_intent=0., requires_merge=0.)
    observations = env._observations()
    assert all(o[51:].reshape(6, 8)[:, -1].sum() == 6 for o in observations.values())
    assert all(env.observation_space(a).contains(o) for a, o in observations.items())
    assert env.population_stats['overflow_peer_observations'] == 24


def test_maneuver_masks_are_actor_visible_and_teacher_is_exact_projection(graph):
    env = v05_bare(graph, 24)
    env.maneuvers['agent_3'] = dict(direction=-1, remaining_fraction=.4)
    env.states['agent_3'].update(lateral_position=-.8, lateral_speed=-1.2)
    observations = env._observations()
    obs = observations['agent_3']
    np.testing.assert_array_equal(obs[30:33], [0., 1., 0.])
    np.testing.assert_allclose(obs[45:51], [-.8 / 3.7, -1.2 / 3., -1., 1., .4, 1.])
    actual = teacher_actions(observations)
    expected = legacy_teacher_actions({a: legacy_observation(o) for a, o in observations.items()}, version='v3')
    for a in env.agents:
        np.testing.assert_array_equal(actual[a], expected[a])
        np.testing.assert_array_equal(actual[a], env.teacher_actions(observations)[a])
    assert actual['agent_3'][1] == 0.


def test_absolute_target_is_sent_once_and_midpoint_does_not_complete(graph):
    env = v05_bare(graph)
    env._begin_maneuver('agent_0', 1)
    env.conn.vehicle.changeLane.assert_called_once_with('agent_0', 2, 3.1)
    assert env.maneuvers['agent_0']['target_lane'] == 'sb1_2'
    env.time = 1.6
    env.states['agent_0'].update(lane_id='sb1_2', lateral_position=-1.5, lateral_speed=1.2)
    env._update_maneuvers()
    assert 'agent_0' in env.maneuvers and not env.completed_this_step
    assert env.maneuvers['agent_0']['remaining_fraction'] == pytest.approx(1.5 / 3.7)
    env.time = 3.1
    env.states['agent_0'].update(lateral_position=0., lateral_speed=0.)
    env._update_maneuvers()
    assert not env.maneuvers and env.completed_this_step == {'agent_0'}
    assert env.motion_stats['completed'] == 1 and env.last_change['agent_0'] == 3.1
    assert env._motion_features('agent_0') == [0., 0., 0., 0., 0., 0.]


def test_native_refusal_never_fabricates_completion(graph):
    env = v05_bare(graph)
    env._begin_maneuver('agent_0', 1)
    env.time = 3.3
    env._update_maneuvers()
    assert not env.maneuvers and not env.completed_this_step
    assert env.motion_stats['native_refused'] == 1 and env.motion_stats['completed'] == 0


def test_conflicting_raw_lane_request_is_logged_without_silently_changing_it(graph):
    env = v05_bare(graph)
    env._begin_maneuver('agent_0', 1)
    env.obs = env._observations()
    def advance():
        env.time += .1
        env.states = {'agent_0': dict(env.states['agent_0'], lateral_position=.12, lateral_speed=1.2)}
        env._update_maneuvers()
    env._advance = advance
    _, _, _, _, infos = env.step({'agent_0': np.array([0., -1.], np.float32)})
    info = infos['agent_0']
    assert info['lane_request'] == -1 and info['lane_rejected']
    assert 'maneuver_in_progress' in info['override_causes']
    assert info['lane_applied_delta'] == 0 and info['maneuver']['in_progress']
    assert env.maneuvers['agent_0']['direction'] == 1
    assert env.conn.vehicle.changeLane.call_count == 1
    assert json.loads(env.journal.getvalue())['agents']['agent_0']['lane_request'] == -1


def test_invalid_joint_action_validates_all_24_before_commands(graph):
    env = v05_bare(graph, 24)
    actions = {a: np.array([0., 0.], np.float32) for a in env.agents}
    actions['agent_23'][0] = np.nan
    with pytest.raises(ValueError, match='hybrid'):
        env.step(actions)
    env.conn.vehicle.setAcceleration.assert_not_called()
    env.conn.vehicle.changeLane.assert_not_called()


def test_terminal_observation_has_99_fields_and_no_fabricated_motion(graph):
    env = v05_bare(graph)
    def advance():
        env.time += .1
        env.states = {}
        env.arrived = {'agent_0'}
        env._update_maneuvers()
    env._advance = advance
    obs, _, term, _, infos = env.step({'agent_0': np.array([0., 0.], np.float32)})
    assert obs['agent_0'].shape == (99,) and not obs['agent_0'].any() and term['agent_0']
    assert infos['agent_0']['maneuver']['native_lateral_position_m'] is None
    assert env.route_trackers['agent_0'].summary()['finish_reason'] == 'arrived'


def test_zero_av_native_step_advances_once_without_fabricated_decisions(graph):
    env = v05_bare(graph, 0)
    calls = []
    env._advance = lambda: calls.append(True)
    assert env.step({}) == ({}, {}, {}, {}, {})
    assert calls == [True] and env.stats['agent_steps'] == 0 and env.steps == 1


def test_legacy_sumo_launch_is_unchanged_and_v05_declares_native_motion(graph, tmp_path):
    assert I110MergeEnv.__new__(I110MergeEnv)._sumo_extra_args() == []
    env = v05_bare(graph)
    env.directory = tmp_path
    args = env._sumo_extra_args()
    assert args[:2] == ['--lanechange.duration', '3.0']
    assert '--lanechange-output.started' in args and '--lanechange-output.ended' in args


@pytest.mark.parametrize('bad', [np.zeros(93), np.full(99, np.nan), np.zeros((99, 1))])
def test_teacher_rejects_wrong_or_nonfinite_abi(bad):
    with pytest.raises(ValueError, match='99-field'):
        legacy_observation(bad)


def test_moving_adjacent_vehicle_is_visible_before_midpoint_and_stationary_is_not(graph):
    env = v05_bare(graph)
    env.states['agent_0'] = native_vehicle('agent_0', lane='sb2_1', position=100.)
    other = native_vehicle('human', lane='sb2_0', position=140.)
    env.states['human'] = other
    assert env._context('agent_0')['neighbors']['own']['front'] is None
    other.update(lateral_position=.12, lateral_speed=1.067)
    front = env._context('agent_0')['neighbors']['own']['front']
    assert front['id'] == 'human' and front['bumper_gap_m'] == pytest.approx(35.2)
    obs = env._observations()['agent_0']
    assert obs[8] == 1. and obs[6] * 120. == pytest.approx(35.2)
    assert env.teacher_actions({'agent_0': obs})['agent_0'][0] == teacher_actions({'agent_0': obs})['agent_0'][0]


def test_vehicle_leaving_lane_retains_source_corridor_after_midpoint(graph):
    env = v05_bare(graph)
    env.states['agent_0'] = native_vehicle('agent_0', lane='sb2_1', position=100.)
    other = native_vehicle('human', lane='sb2_2', position=140.)
    other.update(lateral_position=-1.4, lateral_speed=1.067)
    env.states['human'] = other
    assert env._context('agent_0')['neighbors']['own']['front']['id'] == 'human'
    other.update(lateral_position=0., lateral_speed=0.)
    assert env._context('agent_0')['neighbors']['own']['front'] is None


def test_midpoint_identity_change_preserves_same_front_gap(graph):
    env = v05_bare(graph)
    env.states['agent_0'] = native_vehicle('agent_0', lane='sb2_1', position=100.)
    other = native_vehicle('human', lane='sb2_0', position=140.)
    other.update(lateral_position=1.6, lateral_speed=1.067, lane_width=3.2)
    env.states['human'] = other
    before = env._context('agent_0')['neighbors']['own']['front']
    other.update(lane_id='sb2_1', lateral_position=-1.49)
    after = env._context('agent_0')['neighbors']['own']['front']
    assert before == after


def test_ego_moving_corridor_and_rear_conflicts_are_symmetric(graph):
    env = v05_bare(graph)
    ego = native_vehicle('agent_0', lane='sb2_1', position=100.)
    ego.update(lateral_position=.12, lateral_speed=1.067)
    env.states = {'agent_0': ego,
                  'front': native_vehicle('front', lane='sb2_2', position=140.),
                  'rear': native_vehicle('rear', lane='sb2_2', position=60.)}
    neighbors = env._context('agent_0')['neighbors']['own']
    assert neighbors['front']['id'] == 'front' and neighbors['rear']['id'] == 'rear'
    assert neighbors['front']['bumper_gap_m'] == pytest.approx(35.2)
    assert neighbors['rear']['bumper_gap_m'] == pytest.approx(35.2)


def test_projected_occupancy_uses_lane_lengths_and_route_seams_not_xy(graph):
    env = v05_bare(graph)
    graph.lanes['sb2_2']['length'] = 660.
    ego = native_vehicle('agent_0', lane='sb1_1', position=190.)
    other = native_vehicle('human', lane='sb2_2', position=22.)
    other.update(lateral_position=-.12, lateral_speed=-1.067, x=100000., y=100000.)
    env.states = {'agent_0': ego, 'human': other}
    front = env._context('agent_0')['neighbors']['own']['front']
    # 10 m to the seam + 22/660*600 = 20 m on the target lane, minus body length.
    assert front['bumper_gap_m'] == pytest.approx(25.2)
    other.update(lateral_position=.12, lateral_speed=1.067)
    assert env._context('agent_0')['neighbors']['own']['front'] is None
