"""Teacher-free reward/control checks using fake motion; no SUMO or training."""
from copy import deepcopy
import json
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from environments.highway_environment_v05 import HighwayEnvironmentV05Env
from environments.highway_flow_env import HighwayFlowEnv
from environments.highway_flow_rl_env import HighwayFlowRLEnv, REWARD_VERSION, SCHEMA
from tests.test_highway_environment_v05 import v05_bare
from tests.test_highway_flow_env import flow_bare
from tests.test_highway_merge_context import graph
from tests.test_highway_smooth_control import fake_motion
from traffic.highway_flow_context import legacy_observation


def rl_bare(graph, *, jerk_reward_weight=.01):
    env = flow_bare(graph)
    env.__class__ = HighwayFlowRLEnv
    env.jerk_reward_weight = jerk_reward_weight
    env.reward_weights.update(feasible_acceleration=0., jerk=jerk_reward_weight)
    return env


def forbidden(*args, **kwargs):
    raise AssertionError('Flow teacher must not run')


def test_step_never_calls_flow_teacher_and_has_no_imitation_labels(graph, monkeypatch):
    monkeypatch.setitem(sys.modules, 'traffic.highway_flow_reference', SimpleNamespace(
        reference_details=forbidden, teacher_actions=forbidden))
    monkeypatch.setattr(HighwayFlowEnv, 'step', forbidden)
    env = rl_bare(graph)
    fake_motion(env, [.2])
    observations, rewards, _, _, infos = env.step({'agent_0': np.array([0., 0.], np.float32)})
    info = infos['agent_0']
    assert observations['agent_0'].shape == (117,)
    assert env.state().shape == (2832,)
    assert env._flow_references == {}
    assert not info['teacher_reference_used']
    for key in ('flow_reference', 'smooth_reference', 'feasible_acceleration_target', 'feasible_target_source'):
        assert key not in info
    assert 'feasible_acceleration' not in info['reward_terms']
    assert rewards['agent_0'] == pytest.approx(sum(info['reward_terms'].values()))
    assert all(row['reference_m_s2'] is None for row in env.flow_metrics.traces['agent_0'] if not row.get('baseline'))
    for method in (env.teacher_actions, env.scripted_actions):
        with pytest.raises(RuntimeError, match='Teacher-free'):
            method()


def test_removal_updates_returned_rewards_journal_and_cumulative_statistics(graph):
    env = rl_bare(graph)
    # Exercise actual subtraction rather than relying only on a zero multiplier.
    env.reward_weights['feasible_acceleration'] = .25
    fake_motion(env, [-1., .5])
    total = 0.
    for _ in range(2):
        _, rewards, _, _, infos = env.step({'agent_0': np.array([0., 0.], np.float32)})
        info = infos['agent_0']
        total += rewards['agent_0']
        assert rewards['agent_0'] == pytest.approx(sum(info['reward_terms'].values()))
        assert 'feasible_acceleration' not in info['reward_terms']
    assert total == pytest.approx(sum(env.stats['reward_terms'].values()))
    assert env.stats['acceleration_error_sum'] == 0.
    rows = [json.loads(line)['agents']['agent_0'] for line in env.journal.getvalue().splitlines()]
    assert total == pytest.approx(sum(sum(row['reward_terms'].values()) for row in rows))
    assert all('feasible_acceleration_target' not in row for row in rows)


@pytest.mark.parametrize('jerk_weight', [.01, .03])
def test_rewards_commands_guards_and_lifecycle_match_native_v05_except_alignment_and_jerk(graph, jerk_weight):
    base, env = v05_bare(graph), rl_bare(graph, jerk_reward_weight=jerk_weight)
    for item in (base, env):
        # Request an infeasible acceleration above the legal speed ceiling and
        # a new lane change during a maneuver: both guards must match exactly.
        item.states['agent_0'].update(speed=19., allowed_speed=18.)
        item.maneuvers['agent_0'] = dict(direction=-1, remaining_fraction=.5)
        item.obs = item._observations()
        fake_motion(item, [-4.5, 2.])
    for _ in range(2):
        action = {'agent_0': np.array([1., -1.], np.float32)}
        left, right = base.step(action), env.step(action)
        a, b = left[4]['agent_0'], right[4]['agent_0']
        expected = dict(a['reward_terms'])
        expected.pop('feasible_acceleration')
        expected['jerk'] *= jerk_weight / .01
        assert b['reward_terms'] == pytest.approx(expected)
        assert right[1]['agent_0'] == pytest.approx(sum(expected.values()))
        assert b['reward_terms']['unsafe_request'] < 0.
        assert b['lane_rejected'] and b['override']
        for key in ('requested_acceleration', 'filtered_acceleration', 'realized_acceleration',
                    'acceleration_bounds', 'feasible_acceleration_ceiling', 'override_causes',
                    'lane_applied_delta', 'maneuver'):
            assert b[key] == a[key]
        assert left[2:4] == right[2:4]
        np.testing.assert_array_equal(legacy_observation(right[0]['agent_0']), left[0]['agent_0'])
    assert env.states == base.states
    assert env.previous == base.previous
    assert env.stats['overrides'] == base.stats['overrides']
    assert env.conn.vehicle.setAcceleration.call_args_list == base.conn.vehicle.setAcceleration.call_args_list
    assert env.conn.vehicle.changeLane.call_args_list == base.conn.vehicle.changeLane.call_args_list
    assert env._mask.__func__ is HighwayEnvironmentV05Env._mask
    assert env._lane_rejection_causes.__func__ is HighwayEnvironmentV05Env._lane_rejection_causes


def test_terminal_padding_and_failed_step_restore_flow_observations(graph):
    env = rl_bare(graph)
    original = deepcopy(env.obs)
    with pytest.raises(ValueError, match='One action'):
        env.step({})
    np.testing.assert_array_equal(env.obs['agent_0'], original['agent_0'])
    assert not env._flow_step_active
    def arrived():
        env.time += .1
        env.states, env.arrived = {}, {'agent_0'}
    env._advance = arrived
    observations, _, terminated, _, _ = env.step({'agent_0': np.array([0., 0.], np.float32)})
    assert terminated['agent_0'] and observations['agent_0'].shape == (117,)
    assert not observations['agent_0'].any()


@pytest.mark.parametrize('weight', [True, -1., float('nan'), float('inf'), '0.01'])
def test_invalid_jerk_weight_rejected_before_runtime(tmp_path, weight):
    with pytest.raises(ValueError, match='jerk_reward_weight'):
        HighwayFlowRLEnv(tmp_path / 'missing.xml', tmp_path, jerk_reward_weight=weight)


def test_constructor_pins_other_weights_and_records_separate_jerk_setting(tmp_path, monkeypatch):
    from environments.highway_stage1_env import DEFAULT_REWARD_WEIGHTS
    def initialize(self, network, run_directory, **kwargs):
        self.reward_weights = dict(DEFAULT_REWARD_WEIGHTS)
    monkeypatch.setattr(HighwayFlowEnv, '__init__', initialize)
    env = HighwayFlowRLEnv(tmp_path / 'missing.xml', tmp_path, jerk_reward_weight=.03)
    assert env.reward_weights['feasible_acceleration'] == 0.
    assert env.reward_weights['jerk'] == .03
    assert env.reward_weights['unsafe_request'] == DEFAULT_REWARD_WEIGHTS['unsafe_request']


def test_report_cannot_claim_reference_supervision_or_error(graph, monkeypatch):
    monkeypatch.setattr(HighwayFlowEnv, 'report', lambda self: dict(
        control_target_version='highway_flow_reference_v1', mean_feasible_acceleration_error=12.,
        reward_weights=self.reward_weights))
    env = rl_bare(graph, jerk_reward_weight=.03)
    canonical_weights = dict(env.reward_weights)
    report = env.report()
    assert report['schema'] == SCHEMA and report['reward_version'] == REWARD_VERSION
    assert report['control_target_version'] is None
    assert report['mean_feasible_acceleration_error'] is None
    assert not report['teacher_reference_used'] and not report['imitation_labels_available']
    assert report['legacy_target_computed_and_discarded']
    assert report['jerk_reward_weight'] == .03
    assert 'feasible_acceleration' not in report['reward_weights']
    assert report['reward_weights'] is not env.reward_weights
    assert env.reward_weights == canonical_weights
    # Reporting between live decisions must neither remove a required base
    # coefficient nor corrupt the next step's reconstructed cumulative terms.
    fake_motion(env, [.2, -.2])
    for _ in range(2):
        _, rewards, _, _, infos = env.step({'agent_0': np.array([0., 0.], np.float32)})
        env.report()
        assert env.reward_weights == canonical_weights
        assert rewards['agent_0'] == pytest.approx(sum(infos['agent_0']['reward_terms'].values()))
        assert 'feasible_acceleration' not in env.stats['reward_terms']
