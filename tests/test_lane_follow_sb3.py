"""Synthetic trainer-boundary checks: no simulator, PPO, or optimizer steps."""
import json
from pathlib import Path
import subprocess
import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

from environments.lane_follow_sb3 import make_lane_follow_sb3_wrapper, terminal_infos


def snapshots(terminated=(False,), truncated=(True,)):
    term, trunc = np.asarray(terminated, dtype=bool), np.asarray(truncated, dtype=bool)
    return dict(final_observation={'policy': np.tile([[4.0, 5.0]], (len(term), 1)).astype(np.float32)},
                _final_observation=term | trunc,
                final_terminated=term.copy(), final_truncated=trunc.copy())


def convert(extras, *, infos=None, terminated=None, truncated=None, shape=(2,)):
    return terminal_infos([{}] if infos is None else infos,
                          extras['final_terminated'] if terminated is None else terminated,
                          extras['final_truncated'] if truncated is None else truncated,
                          extras, observation_shape=shape)


def test_timeout_uses_final_state_not_reset_state_and_hand_computed_bootstrap():
    extras = snapshots()
    reset = np.array([-100.0, -100.0], dtype=np.float32)
    existing = [{'terminal_observation': reset, 'episode': {'r': 1.0, 'l': 12}}]
    info = convert(extras, infos=existing)[0]
    # A transparent frozen critic V(x)=2*x0+x1 gives V(final)=13, V(reset)=-300.
    value = lambda obs: 2 * obs[0] + obs[1]
    reward, gamma = 1.0, 0.9
    assert info['TimeLimit.truncated'] is True
    assert reward + gamma * value(info['terminal_observation']) == pytest.approx(12.7)
    assert reward + gamma * value(reset) == pytest.approx(-269.0)
    assert info['episode'] == {'r': 1.0, 'l': 12}
    np.testing.assert_array_equal(existing[0]['terminal_observation'], reset)


@pytest.mark.parametrize('terminated,truncated,bootstrap', [
    (True, False, False), (False, True, True), (True, True, False),
])
def test_terminal_bootstrap_precedence_and_pre_reset_observation(terminated, truncated, bootstrap):
    info = convert(snapshots((terminated,), (truncated,)))[0]
    assert info['TimeLimit.truncated'] is bootstrap
    np.testing.assert_array_equal(info['terminal_observation'], [4, 5])
    target = 1 + (0.9 * 13 if info['TimeLimit.truncated'] else 0)
    assert target == pytest.approx(12.7 if bootstrap else 1)


def test_nonterminal_clears_old_terminal_data_and_does_not_mutate_input():
    extras = snapshots((False,), (False,))
    original = [{'terminal_observation': np.array([99.0, 99.0]), 'TimeLimit.truncated': True}]
    info = convert(extras, infos=original)[0]
    assert 'terminal_observation' not in info
    assert info['TimeLimit.truncated'] is False
    assert original[0]['TimeLimit.truncated'] is True


def test_terminal_copy_survives_later_reset_buffer_writes():
    extras = snapshots()
    info = convert(extras)[0]
    extras['final_observation']['policy'][:] = 200
    np.testing.assert_array_equal(info['terminal_observation'], [4, 5])
    info['terminal_observation'][:] = -5
    np.testing.assert_array_equal(extras['final_observation']['policy'], [[200, 200]])


@pytest.mark.parametrize('key', ['final_observation','_final_observation','final_terminated','final_truncated'])
def test_missing_snapshot_fields_fail_closed(key):
    extras = snapshots()
    del extras[key]
    with pytest.raises(ValueError, match='missing'):
        terminal_infos([{}], np.array([False]), np.array([True]), extras, observation_shape=(2,))


@pytest.mark.parametrize('key,value', [
    ('_final_observation', np.array([False])),
    ('_final_observation', np.array([1])),
    ('_final_observation', np.array([[True]])),
    ('final_terminated', np.array([True])),
    ('final_truncated', np.array([False])),
    ('final_truncated', np.array([True, False])),
])
def test_stale_or_malformed_flags_fail_closed(key, value):
    extras = snapshots()
    extras[key] = value
    with pytest.raises(ValueError):
        convert(extras, terminated=np.array([False]), truncated=np.array([True]))


@pytest.mark.parametrize('observation', [
    [[np.nan, 0.0]], [[0.0, np.inf]], [[1e300, 0.0]],
    [[1.0]], [1.0, 2.0], [[1, 2]], [[True, False]], [['1', '2']],
])
def test_nonfinite_wrong_shape_or_nonfloating_final_observations_fail(observation):
    extras = snapshots()
    extras['final_observation']['policy'] = np.asarray(observation)
    with pytest.raises(ValueError):
        convert(extras)


@pytest.mark.parametrize('shape', [(), (0,), (True,), (2,2), [2]])
def test_only_flat_nonempty_observation_spaces_are_supported(shape):
    with pytest.raises(ValueError):
        convert(snapshots(), shape=shape)


def test_batch_conversion_keeps_each_episode_boundary_separate():
    extras = snapshots((False,True,False), (True,False,False))
    extras['final_observation']['policy'][:] = [[1,2],[3,4],[5,6]]
    infos = convert(extras, infos=[{}, {}, {}])
    assert [i['TimeLimit.truncated'] for i in infos] == [True,False,False]
    np.testing.assert_array_equal(infos[0]['terminal_observation'], [1,2])
    np.testing.assert_array_equal(infos[1]['terminal_observation'], [3,4])
    assert 'terminal_observation' not in infos[2]


def test_factory_override_replaces_base_wrappers_post_reset_sentinel(monkeypatch):
    # This fake models the inspected native wrapper's boundary without importing Kit.
    # It deliberately returns the wrong reset sentinel; the project override must act.
    class Base:
        def __init__(self, env, fast_variant):
            self.observation_space = SimpleNamespace(shape=(2,))
            self.fast_variant = fast_variant

        def _process_extras(self, obs, terminated, truncated, extras, reset_ids):
            return [{'terminal_observation': obs[0], 'episode': {'l': 3}}]

    module = ModuleType('isaaclab_rl.sb3')
    module.Sb3VecEnvWrapper = Base
    package = ModuleType('isaaclab_rl')
    monkeypatch.setitem(sys.modules, 'isaaclab_rl', package)
    monkeypatch.setitem(sys.modules, 'isaaclab_rl.sb3', module)
    env = SimpleNamespace(unwrapped=SimpleNamespace(num_envs=1))
    wrapper = make_lane_follow_sb3_wrapper(env)
    assert wrapper.fast_variant is True
    reset = np.array([[-100, -100]], dtype=np.float32)
    infos = wrapper._process_extras(reset, np.array([False]), np.array([True]), snapshots(), np.array([0]))
    np.testing.assert_array_equal(infos[0]['terminal_observation'], [4,5])
    np.testing.assert_array_equal(reset, [[-100,-100]])
    assert infos[0]['episode'] == {'l':3}


def test_factory_rejects_replication_before_importing_runtime():
    with pytest.raises(ValueError, match='one environment'):
        make_lane_follow_sb3_wrapper(SimpleNamespace(unwrapped=SimpleNamespace(num_envs=2)))


def test_import_is_runtime_and_trainer_free():
    code = "import sys; import environments.lane_follow_sb3; assert not any(x in sys.modules for x in ('isaaclab', 'isaaclab_rl', 'stable_baselines3', 'torch', 'traci'))"
    subprocess.run([sys.executable, '-c', code], cwd=Path(__file__).resolve().parents[1], check=True)


def test_unapproved_proposal_matches_task_spaces_and_exact_bounded_budget():
    from environments.lane_following_contract import OBSERVATION_NAMES
    root = Path(__file__).resolve().parents[1]
    proposal = json.loads((root / 'experiments/lane-follow-ppo-proposal.json').read_text())
    assert proposal['status'] == 'UNAPPROVED_PROPOSAL_NOT_LAUNCHED'
    assert proposal['launch_enabled'] is proposal['training_started'] is False
    assert proposal['optimizer_steps_executed'] == 0
    assert proposal['trainer_entrypoint'] is None
    assert proposal['observation']['names'] == list(OBSERVATION_NAMES)
    assert proposal['observation']['shape'] == [1, len(OBSERVATION_NAMES)]
    budget, trainer, checkpoints = proposal['budget'], proposal['trainer'], proposal['checkpoints']
    assert budget['total_control_transitions'] == trainer['n_steps'] * budget['completed_rollout_updates']
    assert trainer['n_steps'] % trainer['batch_size'] == 0
    assert budget['max_minibatch_optimizer_steps'] == (
        budget['completed_rollout_updates'] * trainer['n_epochs'] * trainer['n_steps'] // trainer['batch_size'])
    assert checkpoints['control_steps'] == [u * trainer['n_steps'] for u in checkpoints['after_completed_updates']]
    assert checkpoints['control_steps'][-1] == budget['total_control_transitions']
    assert checkpoints['save_after_optimizer_update'] is True
    validation = json.loads((root / 'experiments/lab-lane-following-validation.json').read_text())
    partitions = [{validation['validation_seed'], *validation['perturbation_seeds']},
                  {budget['training_seed']}, set(checkpoints['selection_seeds']),
                  set(proposal['evaluation']['held_out_seeds'])]
    assert sum(map(len, partitions)) == len(set.union(*partitions))
    assert checkpoints['held_out_selection'] is False
    assert proposal['normalization']['evaluation_statistics_update'] is False
