"""Synthetic algorithm verification; no driving rollouts or policy training."""

import math

import pytest
import torch

from policies.mappo import MappoActorCritic, MappoTrainer, compute_gae, export_actor


@pytest.fixture(autouse=True)
def deterministic_cpu():
    previous_threads = torch.get_num_threads()
    torch.set_num_threads(1)
    torch.manual_seed(912)
    yield
    torch.set_num_threads(previous_threads)


def small_model():
    return MappoActorCritic(actor_hidden_sizes=(16, 16), critic_hidden_sizes=(24, 24))


def synthetic_batch(model, count=16):
    local = torch.randn(count, 27)
    states = torch.randn(count, 112)
    identities = torch.arange(count) % 4
    with torch.no_grad():
        actions, log_prob, _, latent = model.act(local)
        values = model.values(states, identities)
    return dict(
        local_obs=local, global_states=states, agent_indices=identities,
        actions=actions, pre_tanh_actions=latent, old_log_probs=log_prob,
        old_values=values, advantages=torch.linspace(-1, 1, count),
        returns=values + torch.linspace(-2, 2, count), active=torch.ones(count, dtype=torch.bool),
    )


def test_actor_is_local_and_critic_is_centralized():
    model = small_model()
    local = torch.randn(4, 27)
    state = torch.randn(4, 112)
    ids = torch.arange(4)
    before = model.act(local, deterministic=True)[0]
    values_before = model.values(state, ids)
    values_changed = model.values(state + 0.7, ids)
    assert not torch.allclose(values_before, values_changed)
    with torch.no_grad():
        for parameter in model.critic.parameters():
            parameter.add_(0.2)
    assert torch.equal(before, model.act(local, deterministic=True)[0])
    assert model.critic[0].in_features == 116
    with pytest.raises(ValueError, match="local observations"):
        model.act(state)
    with pytest.raises(ValueError, match="stable slots"):
        model.values(state, ids + 4)


def test_squashed_likelihood_roundtrip_and_density():
    model = small_model()
    obs = torch.randn(64, 27)
    actions, old_log_prob, entropy, latent = model.act(obs)
    log_prob, reevaluated_entropy = model.evaluate_actions(obs, actions, latent)
    assert torch.equal(old_log_prob, log_prob)
    torch.testing.assert_close((log_prob - old_log_prob).exp(), torch.ones(64), rtol=0, atol=0)
    distribution = model._distribution(obs)
    direct = (distribution.log_prob(latent) - torch.log1p(-actions.square())).sum(-1)
    torch.testing.assert_close(log_prob, direct, rtol=1e-5, atol=1e-5)
    torch.testing.assert_close(entropy, reevaluated_entropy)
    assert (actions.abs() <= 1).all() and torch.isfinite(entropy).all()
    loss = -log_prob.mean() - 0.01 * entropy.mean()
    loss.backward()
    assert all(torch.isfinite(p.grad).all() for p in model.actor.parameters())
    assert torch.isfinite(model.log_std.grad).all()
    assert all(p.grad is None for p in model.critic.parameters())


def test_saturated_actions_preserve_stored_likelihood_and_finite_gradient():
    model = small_model()
    with torch.no_grad():
        model.actor.mean_net[-1].bias.fill_(30.0)
    obs = torch.randn(4, 27)
    actions, old, entropy, latent = model.act(obs)
    assert torch.equal(actions, torch.ones_like(actions))
    new, _ = model.evaluate_actions(obs, actions, latent)
    torch.testing.assert_close(old, new, rtol=0, atol=0)
    assert torch.isfinite(new).all() and torch.isfinite(entropy).all()
    (-new.mean() - 0.01 * entropy.mean()).backward()
    assert all(torch.isfinite(p.grad).all() for p in model.actor.parameters())
    with pytest.raises(ValueError, match="Saturated"):
        model.evaluate_actions(obs, actions)
    with pytest.raises(ValueError, match="disagree"):
        model.evaluate_actions(obs, torch.zeros_like(actions), latent)


def test_gae_termination_and_truncation_cut_reset_trace():
    # The large next-episode reward at t=1 must not leak through either ending.
    rewards = torch.tensor([[1.0, 1.0], [100.0, 100.0]])
    values = torch.zeros_like(rewards)
    next_values = torch.tensor([[float("nan"), 10.0], [0.0, 0.0]])
    terminated = torch.tensor([[True, False], [True, True]])
    truncated = torch.tensor([[False, True], [False, False]])
    active = torch.ones_like(terminated)
    advantages, returns = compute_gae(rewards, values, next_values, terminated, truncated, active, gamma=0.9, gae_lambda=0.8)
    torch.testing.assert_close(advantages, torch.tensor([[1.0, 10.0], [100.0, 100.0]]))
    torch.testing.assert_close(returns, advantages)


def test_gae_rollout_cut_bootstraps_and_inactive_slots_are_removed():
    rewards = torch.tensor([1.0, 2.0, 3.0]).reshape(3, 1, 1)
    values = torch.tensor([0.5, 0.4, 0.3]).reshape_as(rewards)
    next_values = torch.tensor([0.4, 0.3, 7.0]).reshape_as(rewards)
    ending = torch.zeros_like(rewards, dtype=torch.bool)
    active = ~ending
    advantages, returns = compute_gae(rewards, values, next_values, ending, ending, active, gamma=0.9, gae_lambda=0.8)
    torch.testing.assert_close(advantages.flatten(), torch.tensor([6.872, 8.35, 9.0]))
    torch.testing.assert_close(returns, advantages + values)
    active[1] = False
    rewards[1] = values[1] = next_values[1] = float("nan")
    advantages, returns = compute_gae(rewards, values, next_values, ending, ending, active, gamma=0.9, gae_lambda=0.8)
    torch.testing.assert_close(advantages.flatten(), torch.tensor([0.86, 0.0, 9.0]))
    torch.testing.assert_close(returns.flatten(), torch.tensor([1.36, 0.0, 9.3]))


def test_export_is_actor_only_and_roundtrips(tmp_path):
    model = small_model()
    model.train()
    path = export_actor(model, tmp_path / "actor.pt")
    deployed = torch.jit.load(str(path), map_location="cpu")
    observations = torch.randn(7, 27)
    expected = model.act(observations, deterministic=True)[0]
    torch.testing.assert_close(deployed(observations), expected)
    assert deployed(observations).shape == (7, 1)
    assert "critic" not in deployed.code
    assert "global" not in deployed.code
    assert model.training  # Export must not modify the trainable model.
    with pytest.raises((ValueError, torch.jit.Error), match="local observations"):
        deployed(torch.randn(7, 112))


def test_synthetic_update_masks_padding_and_updates_both_networks():
    model = small_model()
    batch = synthetic_batch(model)
    batch["active"][-1] = False
    for key, value in batch.items():
        if value.is_floating_point():
            value[-1] = float("nan")
    actor_before = [p.detach().clone() for p in model.actor.parameters()]
    critic_before = [p.detach().clone() for p in model.critic.parameters()]
    trainer = MappoTrainer(model, epochs=2, minibatch_size=8)
    result = trainer.update(batch)
    assert result["active_samples"] == 15
    assert result["optimizer_steps"] == 4
    assert result["completed_epochs"] == 2 and not result["early_stopped"]
    assert all(math.isfinite(value) for value in result.values())
    assert any(not torch.equal(a, b) for a, b in zip(actor_before, model.actor.parameters()))
    assert any(not torch.equal(a, b) for a, b in zip(critic_before, model.critic.parameters()))
    norm = torch.stack([p.grad.square().sum() for p in model.parameters() if p.grad is not None]).sum().sqrt()
    assert norm <= 0.500001


def test_synthetic_ratio_and_value_clipping_match_objective():
    model = small_model()
    batch = synthetic_batch(model, count=2)
    batch["old_log_probs"] -= math.log(1.4)
    # Critic's raw estimate equals the target; the clipped estimate stays 1.8 away.
    batch["returns"] = batch["old_values"].clone()
    batch["old_values"] -= 2.0
    trainer = MappoTrainer(model, epochs=1, minibatch_size=2, target_kl=1.0)
    result = trainer.update(batch)
    assert result["clip_fraction"] == 1.0
    assert result["actor_loss"] == pytest.approx(0.1, abs=1e-6)
    assert result["value_loss"] == pytest.approx(0.5 * 1.8**2, abs=1e-6)


def test_kl_guard_stops_before_an_optimizer_step():
    model = small_model()
    batch = synthetic_batch(model)
    batch["old_log_probs"] -= 3.0
    before = {key: value.clone() for key, value in model.state_dict().items()}
    result = MappoTrainer(model).update(batch)
    assert result["early_stopped"]
    assert result["optimizer_steps"] == 0 and result["last_kl"] > 0.02
    assert all(torch.equal(before[key], value) for key, value in model.state_dict().items())


def test_update_rejects_empty_or_invalid_batches():
    model = small_model()
    batch = synthetic_batch(model)
    batch["active"].zero_()
    with pytest.raises(ValueError, match="active transition"):
        MappoTrainer(model).update(batch)
    batch["active"].fill_(True)
    batch["local_obs"][0, 0] = float("nan")
    with pytest.raises(ValueError, match="not finite"):
        MappoTrainer(model).update(batch)
