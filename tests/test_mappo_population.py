"""Population-model contracts; synthetic tensors only, no simulator run."""
import copy

import pytest
import torch

from policies.mappo import MappoActorCritic, MappoTrainer, export_actor


@pytest.fixture
def model():
    torch.set_num_threads(1)
    torch.manual_seed(407)
    return MappoActorCritic(observation_size=93, state_size=752, num_agents=8,
        message_encoder=True, hybrid_actions=True, driving_local_size=45,
        population_critic=True, actor_hidden_sizes=(16,), critic_hidden_sizes=(16,))


def observation(count):
    result = torch.randn(count, 93).clamp(-1, 1)
    result[:, 30:33] = 1.
    result[:, 52::8] = 1.
    return result


def state(active=4):
    result = torch.zeros(8, 94)
    result[:active, :93] = observation(active)
    result[:active, -1] = 1.
    return result.flatten()


@pytest.mark.parametrize('active', [1, 2, 4, 8])
def test_shared_actor_and_critic_support_population_sizes(model, active):
    obs = observation(active)
    action, log_prob, entropy, latent = model.act(obs)
    values = model.values(state(active).repeat(active, 1), torch.arange(active))
    assert action.shape == (active, 2)
    assert values.shape == (active,)
    assert all(torch.isfinite(x).all() for x in (action, log_prob, entropy, latent, values))
    assert (action[:, 0].abs() <= 1).all()
    assert set(action[:, 1].tolist()) <= {-1., 0., 1.}


def test_critic_slot_permutation_preserves_focal_value(model):
    rows = state(4).reshape(8, 94)
    original = model.values(rows.flatten().repeat(8, 1), torch.arange(8))
    permutation = torch.tensor([4, 2, 7, 0, 3, 6, 1, 5])
    relabeled = model.values(rows[permutation].flatten().repeat(8, 1), torch.arange(8))
    # Float32 reduction order can differ by a few ulps after permutation.
    torch.testing.assert_close(relabeled, original[permutation], rtol=0, atol=1e-6)


def test_inactive_padding_cannot_change_values(model):
    rows = state(2).reshape(8, 94)
    expected = model.values(rows.flatten().repeat(8, 1), torch.arange(8))
    rows[2:, :-1] = float('nan')
    actual = model.values(rows.flatten().repeat(8, 1), torch.arange(8))
    torch.testing.assert_close(actual, expected, rtol=0, atol=0)
    assert actual[2:].eq(0).all()
    assert model.values(torch.zeros(752), torch.tensor(0)).item() == 0.


def test_critic_arbitrary_rollout_dimensions_and_presence_validation(model):
    states = state(4).repeat(2, 3, 1)
    indices = torch.tensor([[0, 1, 2], [1, 2, 3]])
    assert model.values(states, indices).shape == (2, 3)
    bad = states.clone()
    bad[..., 93] = .5
    with pytest.raises(ValueError, match='presence'):
        model.values(bad, indices)
    bad = states.clone()
    bad[..., 0] = float('nan')
    with pytest.raises(ValueError, match='finite'):
        model.values(bad, indices)
    with pytest.raises(ValueError, match='outside'):
        model.values(states, indices + 8)


def test_actor_preserves_message_permutation_and_missing_peer_invariance(model, tmp_path):
    obs = observation(8)
    permuted = torch.cat((obs[:, :45], obs[:, 45:].reshape(8, 6, 8).flip(1).flatten(1)), -1)
    torch.testing.assert_close(model.actor(obs), model.actor(permuted))
    obs[:, 52::8] = 0.
    altered = obs.clone()
    altered[:, 45:] = 1000. * torch.randn(8, 48)
    altered[:, 52::8] = 0.
    torch.testing.assert_close(model.actor(obs), model.actor(altered), rtol=0, atol=0)
    exported = torch.jit.load(str(export_actor(model, tmp_path / 'policy.pt')))
    for size in (1, 2, 4, 8):
        torch.testing.assert_close(exported(obs[:size]), model.actor(obs[:size]))
    assert not any('critic' in key for key in exported.state_dict())


def test_legacy_actor_weights_copy_exactly_and_default_critic_is_unchanged(model):
    legacy = MappoActorCritic(observation_size=93, state_size=376, num_agents=4,
        message_encoder=True, hybrid_actions=True, driving_local_size=45,
        actor_hidden_sizes=(16,), critic_hidden_sizes=(16,))
    assert isinstance(legacy.critic, torch.nn.Sequential)
    assert legacy.critic[0].in_features == 380
    assert set(legacy.actor.state_dict()) == set(model.actor.state_dict())
    model.actor.load_state_dict(copy.deepcopy(legacy.actor.state_dict()), strict=True)
    with torch.no_grad():
        model.log_std.copy_(legacy.log_std)
    obs = observation(8)
    torch.testing.assert_close(model.actor(obs), legacy.actor(obs), rtol=0, atol=0)
    # Fixed-four guards remain in force unless explicitly opted in.
    with pytest.raises(ValueError, match='contract'):
        MappoActorCritic(observation_size=93, state_size=752, num_agents=8,
            message_encoder=True, hybrid_actions=True, driving_local_size=45)


def test_population_critic_contract_rejects_wrong_state_width():
    with pytest.raises(ValueError, match='padded'):
        MappoActorCritic(observation_size=93, state_size=376, num_agents=8,
            message_encoder=True, hybrid_actions=True, driving_local_size=45,
            population_critic=True)


def test_ppo_update_uses_population_critic_and_masks_inactive_rows(model):
    trainer = MappoTrainer(model, epochs=1, minibatch_size=8)
    obs = observation(8)
    states = state(4).repeat(8, 1)
    indices = torch.arange(8)
    with torch.no_grad():
        actions, log_probs, _, latent = model.act(obs)
        values = model.values(states, indices)
    before = {key: value.clone() for key, value in model.critic.state_dict().items()}
    batch = dict(local_obs=obs, global_states=states, agent_indices=indices,
        actions=actions, pre_tanh_actions=latent, old_log_probs=log_probs,
        old_values=values, advantages=torch.arange(8).float(), returns=values + 1.,
        active=indices < 4)
    # Invalid inactive fields must be filtered before they reach the networks.
    batch['local_obs'][4:] = float('nan')
    result = trainer.update(batch)
    assert result['active_samples'] == 4
    assert result['optimizer_steps'] == 1
    assert all(torch.isfinite(parameter).all() for parameter in model.parameters())
    assert any(not torch.equal(before[key], value) for key, value in model.critic.state_dict().items())
    assert any(parameter.grad is not None and parameter.grad.abs().sum() > 0
               for parameter in model.critic.row_encoder.parameters())
