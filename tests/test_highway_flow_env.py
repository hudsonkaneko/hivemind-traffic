from types import SimpleNamespace
import sys
from copy import deepcopy

import numpy as np
import pytest

from environments.highway_flow_env import HighwayFlowEnv, OBSERVATION_SIZE, STATE_SIZE
from environments.highway_environment_v05 import HighwayEnvironmentV05Env
from tests.test_highway_environment_v05 import v05_bare, native_vehicle
from tests.test_highway_smooth_control import fake_motion
from tests.test_highway_merge_context import graph
from traffic.highway_flow_context import legacy_observation


def flow_bare(graph, count=1):
    env = v05_bare(graph, count)
    env.__class__ = HighwayFlowEnv
    env.fixture = None
    env._reset_flow()
    env.obs = env._observations()
    return env


def test_flow_space_state_and_legacy_projection(graph):
    old = v05_bare(graph, 3)
    old_obs = deepcopy(old.obs)
    env = flow_bare(graph, 3)
    assert env.state().shape == (STATE_SIZE,)
    assert env.state().reshape(24, 118)[:, -1].sum() == 3
    for a, obs in env.obs.items():
        assert env.observation_space(a).contains(obs) and obs.shape == (OBSERVATION_SIZE,)
        np.testing.assert_array_equal(legacy_observation(obs), old_obs[a])


def test_flow_reward_retains_legacy_weights_and_native_path(graph, monkeypatch):
    monkeypatch.setitem(sys.modules, 'traffic.highway_flow_reference', SimpleNamespace(
        reference_details=lambda obs: dict(acceleration_m_s2=.5, version='test')))
    env = flow_bare(graph)
    fake_motion(env, [.2])
    before_weights = dict(env.reward_weights)
    _, rewards, _, _, infos = env.step({'agent_0': np.array([0., 0.], np.float32)})
    info = infos['agent_0']
    assert env.reward_weights == before_weights
    assert info['feasible_acceleration_target'] == .5
    assert info['reward_terms']['feasible_acceleration'] == pytest.approx(-.25 * 1.75 / 4.5)
    assert info['reward_terms']['jerk'] == pytest.approx(-.02)
    assert 'command_reference_residual_slew' not in info['reward_terms']
    assert rewards['agent_0'] == pytest.approx(sum(info['reward_terms'].values()))
    assert env.obs['agent_0'].shape == (117,)


def test_terminal_flow_observation_has_correct_shape(graph, monkeypatch):
    monkeypatch.setitem(sys.modules, 'traffic.highway_flow_reference', SimpleNamespace(
        reference_details=lambda obs: dict(acceleration_m_s2=0., version='test')))
    env = flow_bare(graph)
    def arrived():
        env.time += .1
        env.states = {}
        env.arrived = {'agent_0'}
    env._advance = arrived
    observations, _, terminated, _, _ = env.step({'agent_0': np.array([0., 0.], np.float32)})
    assert terminated['agent_0'] and observations['agent_0'].shape == (117,)
    assert not observations['agent_0'].any()


def test_flow_native_tail_measurement_is_separate_from_decisions(graph, monkeypatch):
    env = flow_bare(graph)
    env._measurement_phase = True
    env.fixture = None
    env.agents = []
    env.states = {'human': native_vehicle('human')}
    def advance(self):
        self.time += .1
        self.states['human'] = dict(self.states['human'], acceleration=1., speed=20.1)
    monkeypatch.setattr(HighwayEnvironmentV05Env, '_advance', advance)
    env._advance()
    report = env.flow_metrics.report()
    assert report['all']['decisions'] == 0
    assert report['all']['jerk_m_s3']['realized']['samples'] == 1


def test_fixture_actor_event_is_refused_by_existing_guard(graph):
    env = flow_bare(graph)
    env._measurement_phase = True
    env.time, env.start_time = 1., 0.
    env.fixture = dict(events=[dict(at_s=1., vehicle_id='agent_0', lane_delta=1)])
    env.maneuvers['agent_0'] = dict(direction=1)
    env._fixture_commands()
    assert env.fixture_event_log[0]['outcome'] == 'guard_refused'
    assert not env.fixture_event_log[0]['command_issued']
    env._fixture_commands()
    assert len(env.fixture_event_log) == 1
