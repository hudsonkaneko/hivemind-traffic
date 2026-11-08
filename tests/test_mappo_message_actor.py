import torch
from policies.mappo import MappoActorCritic, export_actor


def test_masked_messages_and_permutation(tmp_path):
    torch.set_num_threads(1)
    model = MappoActorCritic(observation_size=69, state_size=280, message_encoder=True)
    obs = torch.zeros(2, 69)
    obs[:, :27] = torch.rand(2, 27)
    absent = obs.clone()
    absent[:, 27:] = torch.randn(2, 42)
    absent[:, 33::7] = 0
    torch.testing.assert_close(model.actor(obs), model.actor(absent))
    obs[:, 27:] = torch.rand(2, 42)
    obs[:, 33::7] = 1
    permuted = torch.cat((obs[:, :27], obs[:, 27:].reshape(2, 6, 7).flip(1).reshape(2, 42)), -1)
    torch.testing.assert_close(model.actor(obs), model.actor(permuted))
    path = export_actor(model, tmp_path / 'actor.pt')
    actor = torch.jit.load(str(path))
    torch.testing.assert_close(actor(obs), model.actor(obs))
    # Delivered content has a real computational path to the action.
    obs.requires_grad_()
    model.actor(obs).sum().backward()
    assert obs.grad[:, 27:].abs().sum() > 0


def test_wave_config_rejects_mismatched_channel_width():
    import copy
    import pytest
    from experiments.sumo_mappo_support import load_config, validate_config
    config = load_config('experiments/configs/sumo_mappo_v3.json')
    for channel in [None, dict(config['environment']['communication'], max_neighbors=5)]:
        changed = copy.deepcopy(config)
        changed['environment']['communication'] = channel
        with pytest.raises(ValueError, match='dimensions'):
            validate_config(changed)


def test_v3_scripted_reference_respects_cruise_margin():
    import numpy as np
    from experiments.run_sumo_mappo import baseline_actions
    from environments.sumo_mappo_env import action_to_acceleration
    obs = np.zeros(69, dtype=np.float32)
    obs[0] = 26.8224 / 26.8224
    for kind in ['scripted', 'cooperative_scripted']:
        action = baseline_actions(kind, {'agent_0':obs}, np.random.default_rng(1))['agent_0'][0]
        assert action_to_acceleration(float(action)) < 0
