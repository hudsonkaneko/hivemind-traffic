import json
from pathlib import Path

import numpy as np
import pytest

from experiments.train_lane_follow_local import ROOT, read_config, save_policy, vehicle_actions, wrap_actions


def test_config_budget_is_exact_and_fresh():
    cfg = read_config(ROOT / 'experiments/lane-follow-local-v2.json')
    assert cfg['total_timesteps'] == 262144
    assert cfg['total_timesteps'] % cfg['ppo']['n_steps'] == 0
    assert cfg['run_name'] != 'lane-follow-ppo-2101-v1'


def test_longitudinal_actions_are_exclusive_and_steering_scaled():
    result = vehicle_actions([[-1, -1], [1, 1], [.4, 0], [2, -2]])
    np.testing.assert_allclose(result, [[-.5, 0, 1], [.5, 1, 0], [.2, 0, 0], [.5, 0, 1]])
    assert result.dtype == np.float32
    assert not np.any(result[:, 1] * result[:, 2])


@pytest.mark.parametrize('actions', [[1, 2], [[0, 1, 2]], [[np.nan, 0]], [[0, np.inf]]])
def test_invalid_actions_fail_before_physics(actions):
    with pytest.raises(ValueError):
        vehicle_actions(actions)


def test_atomic_pair_is_complete_and_cannot_overwrite(tmp_path):
    class Model:
        num_timesteps = 16384
        def save(self, path):
            Path(path).write_bytes(b'model')
    class Normalizer:
        def save(self, path):
            Path(path).write_bytes(b'normalizer')
    target = tmp_path / 'checkpoint'
    save_policy(Model(), Normalizer(), target, {'test': True})
    meta = json.loads((target / 'policy.json').read_text())
    assert set(meta['sha256']) == {'model.zip', 'vecnormalize.pkl'}
    assert meta['timesteps'] == 16384
    with pytest.raises(OSError):
        save_policy(Model(), Normalizer(), target, {'test': False})
    assert json.loads((target / 'policy.json').read_text())['config'] == {'test': True}


def test_wrapper_preserves_terminal_observation_and_reset():
    gym = pytest.importorskip('gymnasium')
    pytest.importorskip('stable_baselines3')
    from stable_baselines3.common.vec_env import DummyVecEnv

    class Car(gym.Env):
        observation_space = gym.spaces.Box(-100, 100, (22,), dtype=np.float32)
        action_space = gym.spaces.Box(np.array([-.5, 0, 0], np.float32),
                                      np.array([.5, 1, 1], np.float32))
        def reset(self, *, seed=None, options=None):
            return np.zeros(22, np.float32), {}
        def step(self, action):
            self.received = action.copy()
            return np.ones(22, np.float32), 2., False, True, {}

    car = Car()
    wrapped = wrap_actions(DummyVecEnv([lambda: car]))
    try:
        assert wrapped.action_space.shape == (2,)
        wrapped.reset()
        obs, rewards, done, infos = wrapped.step(np.array([[.4, -.7]], np.float32))
        np.testing.assert_allclose(car.received, [.2, 0, .7])
        np.testing.assert_array_equal(obs, np.zeros((1, 22)))
        np.testing.assert_array_equal(infos[0]['terminal_observation'], np.ones(22))
        assert done[0] and rewards[0] == 2 and infos[0]['TimeLimit.truncated']
    finally:
        wrapped.close()
