"""Imitation-assisted MAPPO checks; tiny synthetic batches, no traffic training."""
import copy
import time

import pytest
import torch

from policies.mappo import MappoActorCritic, MappoTrainer, export_actor


def model():
    torch.set_num_threads(1)
    torch.manual_seed(19)
    return MappoActorCritic(observation_size=81, state_size=328, message_encoder=True,
                           hybrid_actions=True, actor_hidden_sizes=(16,), critic_hidden_sizes=(16,))


def demonstrations():
    obs = torch.zeros(24, 81)
    obs[:, 30:33] = 1
    obs[:12, 0] = -1
    obs[12:, 0] = 1
    actions = torch.zeros(24, 2)
    actions[:12] = torch.tensor([-.5, -1.])
    actions[12:] = torch.tensor([.5, 1.])
    return obs, actions


def rollout(policy, *, inactive=False):
    obs, teacher = demonstrations()
    with torch.no_grad():
        actions, likelihood, _, latents = policy.act(obs)
        state = torch.zeros(len(obs), 328)
        ids = torch.arange(len(obs)) % 4
        values = policy.values(state, ids)
    batch = dict(local_obs=obs, teacher_actions=teacher, actions=actions,
                 pre_tanh_actions=latents, global_states=state, agent_indices=ids,
                 old_log_probs=likelihood, old_values=values,
                 advantages=torch.linspace(-1, 1, len(obs)), returns=values + .1,
                 active=torch.ones(len(obs), dtype=torch.bool))
    if inactive:
        batch['active'][-1] = False
        batch['teacher_actions'][-1] = float('nan')
    return batch


def test_behavior_clone_learns_teacher_without_critic_or_ppo_changes(tmp_path):
    policy = model()
    trainer = MappoTrainer(policy)
    critic = copy.deepcopy(policy.critic.state_dict())
    obs, teacher = demonstrations()
    metrics = trainer.behavior_clone(obs, teacher, epochs=80, minibatch_size=24,
                                     learning_rate=.01, seed=7)
    assert metrics['optimizer_steps'] == 80
    assert metrics['samples_seen'] == 80 * 24
    assert metrics['final_loss'] < metrics['initial_loss'] * .05
    assert torch.equal(policy.actor(obs)[:, 1], teacher[:, 1])
    assert (policy.actor(obs)[:, 0] - teacher[:, 0]).abs().max() < .1
    assert float(policy.log_std.detach()) == -2
    assert not trainer.optimizer.state
    for name, value in policy.critic.state_dict().items():
        torch.testing.assert_close(value, critic[name], rtol=0, atol=0)
    exported = torch.jit.load(str(export_actor(policy, tmp_path / 'policy.pt')))
    torch.testing.assert_close(exported(obs), policy.actor(obs))


def test_behavior_clone_is_seeded_and_deadline_bounded():
    first, second = model(), model()
    obs, teacher = demonstrations()
    settings = dict(epochs=3, minibatch_size=7, seed=812)
    a = MappoTrainer(first).behavior_clone(obs, teacher, **settings)
    b = MappoTrainer(second).behavior_clone(obs, teacher, **settings)
    assert a == b
    for name, value in first.state_dict().items():
        torch.testing.assert_close(value, second.state_dict()[name], rtol=0, atol=0)
    with pytest.raises(TimeoutError, match='budget'):
        MappoTrainer(first).behavior_clone(obs, teacher, epochs=1, deadline=time.monotonic()-1)


def test_imitation_checks_contract_and_mask():
    trainer = MappoTrainer(model())
    obs, teacher = demonstrations()
    obs[0, 30] = 0
    with pytest.raises(ValueError, match='mask'):
        trainer.behavior_clone(obs, teacher, epochs=1)
    obs[0, 30] = 1
    teacher[0, 1] = .5
    with pytest.raises(ValueError, match='contract'):
        trainer.behavior_clone(obs, teacher, epochs=1)
    teacher[0, 1] = -1
    teacher[0, 0] = float('nan')
    with pytest.raises(ValueError, match='finite'):
        trainer.behavior_clone(obs, teacher, epochs=1)


def test_imitation_lane_change_weight_is_explicit():
    policy = model()
    obs, teacher = demonstrations()
    ordinary = MappoTrainer(policy, imitation_lane_change_weight=1)._imitation_loss(obs, teacher)
    weighted = MappoTrainer(policy, imitation_lane_change_weight=10)._imitation_loss(obs, teacher)
    torch.testing.assert_close(ordinary[1], weighted[1])
    torch.testing.assert_close(weighted[2], ordinary[2] * 10)


def test_ppo_imitation_uses_only_active_teacher_rows():
    policy = model()
    trainer = MappoTrainer(policy, imitation_coef=1., epochs=1, minibatch_size=32)
    batch = rollout(policy, inactive=True)
    result = trainer.update(batch)
    assert result['active_samples'] == 23
    assert result['optimizer_steps'] == 1
    assert result['imitation_loss'] > 0
    assert result['imitation_lane_loss'] > 0
    assert torch.isfinite(torch.tensor(list(result.values()), dtype=torch.float32)).all()


def test_ppo_requires_teacher_only_when_enabled():
    policy = model()
    batch = rollout(policy)
    del batch['teacher_actions']
    with pytest.raises(ValueError, match='teacher_actions'):
        MappoTrainer(policy, imitation_coef=1.).update(batch)
    result = MappoTrainer(policy, epochs=1).update(batch)
    assert result['optimizer_steps'] > 0
    assert 'imitation_loss' not in result


def test_invalid_imitation_settings_and_legacy_actor():
    with pytest.raises(ValueError, match='imitation'):
        MappoTrainer(model(), imitation_coef=float('nan'))
    with pytest.raises(ValueError, match='hybrid'):
        MappoTrainer(MappoActorCritic(), imitation_coef=1.)
    obs, teacher = demonstrations()
    with pytest.raises(ValueError, match='settings'):
        MappoTrainer(model()).behavior_clone(obs, teacher, epochs=0)
