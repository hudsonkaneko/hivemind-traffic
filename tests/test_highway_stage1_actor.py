import pytest
import torch

from policies.mappo import MappoActorCritic, MappoTrainer, export_actor


def model():
    return MappoActorCritic(observation_size=93, state_size=376, num_agents=4,
        message_encoder=True, hybrid_actions=True, driving_local_size=45)


def observations():
    obs = torch.zeros(4, 93)
    obs[:, 31] = 1.
    obs[:, 43] = 2 / 4.5
    return obs


def test_new_layout_requires_explicit_configuration():
    with pytest.raises(ValueError, match='explicit supported'):
        MappoActorCritic(observation_size=93, state_size=376, num_agents=4,
                        message_encoder=True, hybrid_actions=True)
    assert model().actor.mean_net[0].local_size == 45


def test_old_layout_remains_unchanged():
    old = MappoActorCritic(observation_size=81, state_size=328, num_agents=4,
                         message_encoder=True, hybrid_actions=True)
    assert old.actor.mean_net[0].local_size == 33
    assert old.actor.mean_net[1][0].weight.shape[1] == 50


def test_new_actor_stored_likelihood_and_mask_agree():
    net = model(); obs = observations()
    actions, logp, entropy, latent = net.act(obs)
    actual, _ = net.evaluate_actions(obs, actions, latent)
    torch.testing.assert_close(logp, actual)
    assert torch.equal(actions[:, 1], torch.zeros(4))
    assert torch.isfinite(entropy).all()


def test_new_actor_permutation_invariant_delivered_messages():
    net = model(); obs = observations()
    slots = torch.randn(4, 6, 8).clamp(-1, 1); slots[:, :, -1] = 1
    obs[:, 45:] = slots.flatten(1)
    shuffled = obs.clone(); shuffled[:, 45:] = slots[:, [3, 1, 0, 5, 4, 2]].flatten(1)
    torch.testing.assert_close(net.actor(obs), net.actor(shuffled), rtol=1e-5, atol=1e-6)


def test_new_actor_export_round_trip(tmp_path):
    net = model().eval(); obs = observations()
    exported = torch.jit.load(str(export_actor(net, tmp_path / 'policy.pt')))
    torch.testing.assert_close(exported(obs), net.actor(obs), rtol=0, atol=0)
    with pytest.raises(Exception): exported(torch.zeros(1, 81))


def test_old_checkpoint_rejected_by_new_model():
    old = MappoActorCritic(observation_size=81, state_size=328, num_agents=4,
                         message_encoder=True, hybrid_actions=True)
    with pytest.raises(RuntimeError, match='size mismatch'):
        model().load_state_dict(old.state_dict())


def test_expired_update_deadline_performs_zero_optimizer_steps():
    net = model(); obs = observations()
    actions, logp, _, latent = net.act(obs)
    state = torch.zeros(4, 376); indices = torch.arange(4)
    batch = dict(local_obs=obs, global_states=state, agent_indices=indices, actions=actions,
        pre_tanh_actions=latent, old_log_probs=logp, old_values=net.values(state, indices),
        advantages=torch.ones(4), returns=torch.ones(4), active=torch.ones(4, dtype=torch.bool))
    before = {k: v.clone() for k, v in net.state_dict().items()}
    result = MappoTrainer(net).update(batch, deadline=0.)
    assert result['budget_exhausted'] and result['optimizer_steps'] == 0
    assert all(torch.equal(before[k], v) for k, v in net.state_dict().items())
