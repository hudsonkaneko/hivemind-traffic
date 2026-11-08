"""Synthetic-only tests; no simulator, experiment training or saved policies."""
import copy
import json

import pytest
import torch

from policies.mappo import MappoActorCritic, MappoTrainer
from policies.mappo_diagnostics import critic_fit_metrics, current_gradient_norms, weighted_gradient_report


@pytest.fixture(autouse=True)
def one_torch_thread():
    previous = torch.get_num_threads()
    torch.set_num_threads(1)
    yield
    torch.set_num_threads(previous)


def example():
    torch.manual_seed(631)
    model = MappoActorCritic(observation_size=93, state_size=188, num_agents=2,
        actor_hidden_sizes=(8,), critic_hidden_sizes=(8,), message_encoder=True,
        hybrid_actions=True, driving_local_size=45, population_critic=True)
    obs = torch.randn(6, 2, 93) * .2
    obs[..., 30:33] = 1.
    # Absent message slots must not contain arbitrary nonbinary presence flags.
    obs[..., 45:] = 0.
    state = torch.cat((obs, torch.ones(6, 2, 1)), -1).reshape(6, 1, 188).repeat(1, 2, 1)
    index = torch.arange(2).repeat(6, 1)
    with torch.no_grad():
        actions, logp, _, latent = model.act(obs)
        values = model.values(state, index)
    target = values + torch.linspace(-1., 2., 12).reshape(6, 2)
    batch = dict(local_obs=obs, global_states=state, agent_indices=index,
        actions=actions, pre_tanh_actions=latent, old_log_probs=logp, old_values=values,
        returns=target, advantages=target - values, active=torch.ones(6, 2, dtype=torch.bool),
        teacher_actions=torch.stack((torch.full((6, 2), .6), torch.zeros(6, 2)), -1))
    return model, batch


def trainer(model, **extra):
    return MappoTrainer(model, epochs=2, minibatch_size=5, target_kl=10.,
        imitation_coef=.04, imitation_acceleration_weight=3., entropy_coef=.01,
        value_coef=.5, **extra)


def assert_same(left, right):
    if isinstance(left, torch.Tensor):
        assert torch.equal(left, right)
    elif isinstance(left, dict):
        assert left.keys() == right.keys()
        for key in left:
            assert_same(left[key], right[key])
    elif isinstance(left, (tuple, list)):
        assert len(left) == len(right)
        for a, b in zip(left, right):
            assert_same(a, b)
    else:
        assert left == right


def test_opt_in_diagnostics_leave_rng_gradients_and_optimizer_result_identical():
    model, batch = example()
    plain = trainer(model)
    measured = trainer(copy.deepcopy(model), gradient_diagnostics=True)
    rng = torch.get_rng_state()
    plain_stats = plain.update(batch)
    plain_rng = torch.get_rng_state()
    torch.set_rng_state(rng)
    measured_stats = measured.update(batch, phase='ppo_after_warmup')
    assert torch.equal(torch.get_rng_state(), plain_rng)
    assert_same(plain.model.state_dict(), measured.model.state_dict())
    assert_same(plain.optimizer.state_dict(), measured.optimizer.state_dict())
    for p, q in zip(plain.model.parameters(), measured.model.parameters()):
        assert_same(p.grad, q.grad)
    for key in plain_stats:
        assert plain_stats[key] == measured_stats[key]
    report = measured_stats['gradient_diagnostics']
    assert report['phase'] == 'ppo_after_warmup'
    assert report['optimizer_steps'] == len(report['actual_steps']) == 6
    assert report['representative_minibatch']['samples'] == 5
    assert report['critic_before']['samples'] == report['critic_after']['samples'] == 12
    for step in report['actual_steps']:
        assert step['after_clip']['joint'] <= .500001
        assert step['after_clip']['actor'] == pytest.approx(
            step['before_clip']['actor'] * step['global_clip_factor'], rel=2e-6)
    json.dumps(measured_stats, allow_nan=False)


def test_no_update_preflight_preserves_existing_gradients_rng_targets_and_weights(monkeypatch):
    model, batch = example()
    item = trainer(model, gradient_diagnostics=True)
    for p in model.parameters():
        p.grad = torch.full_like(p, .123)
    parameters = copy.deepcopy(model.state_dict())
    gradients = [p.grad.clone() for p in model.parameters()]
    targets = {k: v.clone() for k, v in batch.items()}
    rng = torch.get_rng_state()
    monkeypatch.setattr(item.optimizer, 'step', lambda: pytest.fail('No-step diagnostic called optimizer'))
    report = item.diagnose_batch(batch)
    assert report['optimizer_steps'] == 0
    assert not item.optimizer.state
    assert torch.equal(torch.get_rng_state(), rng)
    assert_same(parameters, model.state_dict())
    assert_same(targets, batch)
    for p, gradient in zip(model.parameters(), gradients):
        assert_same(p.grad, gradient)
    json.dumps(report, allow_nan=False)


def test_component_gradients_use_actual_weights_and_do_not_contaminate_backward():
    model, batch = example()
    item = trainer(model)
    data, _ = item._prepare_batch(batch)
    logp, entropy = model.evaluate_actions(data['local_obs'], data['actions'], data['pre_tanh_actions'])
    loss, weighted, _ = item._loss_terms(data, (logp - data['old_log_probs']).exp(), entropy)
    _, acceleration, lane = item._imitation_loss(data['local_obs'], data['teacher_actions'])
    assert torch.equal(weighted['acceleration'], acceleration * .04 * 3.)
    assert torch.equal(weighted['lane'], lane * .04)
    assert torch.equal(weighted['entropy'], -.01 * entropy.mean())
    report = weighted_gradient_report(model, weighted)
    assert all(p.grad is None for p in model.parameters())
    actor = list(model.actor.parameters()) + [model.log_std]
    actual = torch.autograd.grad(.04 * 3. * acceleration, actor, retain_graph=True, allow_unused=True)
    norm = torch.cat([torch.zeros_like(p).reshape(-1) if g is None else g.reshape(-1)
                      for p, g in zip(actor, actual)]).norm()
    assert report['components']['acceleration']['actor'] == pytest.approx(float(norm), rel=1e-6)
    assert report['components']['value']['actor'] == 0.
    assert report['components']['ppo']['critic'] == 0.
    assert report['actor_cosines']['ppo__value'] is None
    loss.backward()
    assert current_gradient_norms(model)['joint'] > 0.


def test_padding_excluded_before_diagnostic_normalization_and_value_checks():
    model, batch = example()
    item = trainer(model)
    # Both forms contain identical valid rows; malformed padding is ignored.
    batch['active'][-1] = False
    trimmed = {name: value[:-1].clone() for name, value in batch.items()}
    for name, value in batch.items():
        if name == 'active':
            continue
        value[-1] = -99 if name == 'agent_indices' else float('nan')
    report = item.diagnose_batch(batch)
    assert report == item.diagnose_batch(trimmed)
    assert report['active_samples'] == 10
    fit = critic_fit_metrics(model, batch['global_states'], batch['agent_indices'], batch['returns'], batch['active'])
    assert fit == report['critic']
    json.dumps(report, allow_nan=False)


def test_degenerate_targets_and_zero_step_update_are_json_safe():
    model, batch = example()
    batch['returns'].fill_(2.)
    item = trainer(model, gradient_diagnostics=True)
    report = item.update(batch, deadline=0.)['gradient_diagnostics']
    assert report['optimizer_steps'] == 0
    assert report['representative_minibatch'] is None
    assert report['gradient_clip_fraction'] is None
    assert report['critic_before'] == report['critic_after']
    assert report['critic_after']['explained_variance'] is None
    assert report['critic_after']['mse'] >= 0.
    empty = critic_fit_metrics(model, batch['global_states'], batch['agent_indices'], batch['returns'],
                               torch.zeros_like(batch['active']))
    assert empty == dict(samples=0, mse=None, explained_variance=None, target_variance=None)
    json.dumps(report, allow_nan=False)


def test_diagnostics_validation_and_zero_weight_cosines():
    model, batch = example()
    with pytest.raises(ValueError, match='boolean'):
        trainer(model, gradient_diagnostics=1)
    loss = model.actor(batch['local_obs'])[..., 0].mean()
    report = weighted_gradient_report(model, dict(zero=loss * 0., ppo=loss))
    assert report['actor_cosines']['zero__ppo'] is None
    assert report['components']['zero']['joint'] == 0.
    with pytest.raises(ValueError, match='finite scalars'):
        weighted_gradient_report(model, dict(bad=loss * float('nan')))
    with pytest.raises(ValueError, match='incompatible'):
        critic_fit_metrics(model, batch['global_states'][:-1], batch['agent_indices'], batch['returns'])
