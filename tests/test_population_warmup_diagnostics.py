"""Synthetic critic-warmup instrumentation checks; no simulator or checkpoints."""
import copy
import json

import pytest
import torch

from experiments.run_highway_population import actor_digest, critic_update
from policies.mappo import MappoActorCritic, MappoTrainer
from policies.mappo_diagnostics import critic_fit_metrics


@pytest.fixture(autouse=True)
def one_thread():
    old = torch.get_num_threads()
    torch.set_num_threads(1)
    yield
    torch.set_num_threads(old)


def example():
    torch.manual_seed(983)
    model = MappoActorCritic(observation_size=93, state_size=752, num_agents=8,
        actor_hidden_sizes=(8,), critic_hidden_sizes=(8,), message_encoder=True,
        hybrid_actions=True, driving_local_size=45, population_critic=True)
    active = torch.arange(8).repeat(3, 1) < 4
    observations = torch.randn(3, 8, 93) * .2
    rows = torch.cat((observations, active[..., None].float()), -1)
    states = rows.reshape(3, 1, 752).repeat(1, 8, 1)
    indices = torch.arange(8).repeat(3, 1)
    with torch.no_grad():
        targets = model.values(states, indices) + torch.linspace(-2., 5., 24).reshape(3, 8)
    # Nonfinite padded targets/state and invalid padded focal IDs are excluded.
    states[~active] = float('nan')
    targets[~active] = float('nan')
    indices[~active] = -99
    return model, dict(global_states=states, agent_indices=indices, returns=targets, active=active)


def trainer(model, **kwargs):
    return MappoTrainer(model, epochs=2, minibatch_size=5, **kwargs)


def same(left, right):
    if isinstance(left, torch.Tensor):
        assert torch.equal(left, right)
    elif isinstance(left, dict):
        assert left.keys() == right.keys()
        for key in left:
            same(left[key], right[key])
    elif isinstance(left, (list, tuple)):
        assert len(left) == len(right)
        for a, b in zip(left, right):
            same(a, b)
    else:
        assert left == right


def test_warmup_diagnostics_preserve_update_optimizer_rng_and_actor():
    model, batch = example()
    plain = trainer(model)
    measured = trainer(copy.deepcopy(model), gradient_diagnostics=True)
    initial_actor = actor_digest(model)
    initial_std = model.log_std.detach().clone()
    before = critic_fit_metrics(model, batch['global_states'], batch['agent_indices'],
                                batch['returns'], batch['active'])
    rng = torch.get_rng_state()
    plain_report = critic_update(model, plain, batch, deadline=None)
    assert torch.equal(rng, torch.get_rng_state())
    measured_report = critic_update(measured.model, measured, batch, deadline=None)
    assert torch.equal(rng, torch.get_rng_state())
    same(plain.model.state_dict(), measured.model.state_dict())
    same(plain.optimizer.state_dict(), measured.optimizer.state_dict())
    for p, q in zip(plain.model.parameters(), measured.model.parameters()):
        same(p.grad, q.grad)
    for key in plain_report:
        assert plain_report[key] == measured_report[key]
    assert actor_digest(model) == actor_digest(measured.model) == initial_actor
    assert torch.equal(measured.model.log_std, initial_std)
    assert all(p.grad is None for p in measured.model.actor.parameters())
    assert measured.model.log_std.grad is None
    details = measured_report['gradient_diagnostics']
    assert details['phase'] == 'critic_warmup'
    assert details['actual_step_count'] == measured_report['optimizer_steps'] == 6
    assert details['critic_fit_before'] == before
    assert details['critic_fit_before']['samples'] == details['critic_fit_after']['samples'] == 12
    after = critic_fit_metrics(measured.model, batch['global_states'], batch['agent_indices'],
                               batch['returns'], batch['active'])
    assert details['critic_fit_after'] == after
    for stage in ('before', 'after'):
        assert details['actual_gradient_means'][stage]['actor'] == 0.
        assert details['actual_gradient_means'][stage]['critic'] == pytest.approx(
            details['actual_gradient_means'][stage]['joint'])
    assert details['actual_gradient_means']['after']['critic'] <= .500001
    assert 0 < details['mean_critic_clip_factor'] < 1
    assert details['clipped_step_fraction'] == 1.
    json.dumps(measured_report, allow_nan=False)


def test_expired_deadline_has_no_updates_or_clipping_measurements(monkeypatch):
    model, batch = example()
    batch['returns'][batch['active']] = 2.
    item = trainer(model, gradient_diagnostics=True)
    original = copy.deepcopy(model.state_dict())
    monkeypatch.setattr(item.optimizer, 'step', lambda: pytest.fail('Expired warmup stepped optimizer'))
    report = critic_update(model, item, batch, deadline=0.)
    same(original, model.state_dict())
    assert not item.optimizer.state
    assert report['optimizer_steps'] == 0
    assert report['budget_exhausted'] is True
    details = report['gradient_diagnostics']
    assert details['actual_step_count'] == 0
    assert details['critic_fit_before'] == details['critic_fit_after']
    assert details['critic_fit_after']['explained_variance'] is None
    assert details['critic_fit_after']['samples'] == 12
    assert details['clipped_step_fraction'] is None
    assert details['mean_critic_clip_factor'] is None
    assert all(value is None for stage in details['actual_gradient_means'].values() for value in stage.values())
    json.dumps(report, allow_nan=False)


def test_warmup_half_mse_is_independent_of_ppo_value_coefficient():
    model, batch = example()
    low = MappoTrainer(model, epochs=1, minibatch_size=64, value_coef=.01, gradient_diagnostics=True)
    high = MappoTrainer(copy.deepcopy(model), epochs=1, minibatch_size=64, value_coef=.9, gradient_diagnostics=True)
    active = batch['active']
    with torch.no_grad():
        expected = .5 * (model.values(batch['global_states'][active], batch['agent_indices'][active])
                         - batch['returns'][active]).square().mean()
    low_report = critic_update(model, low, batch, deadline=None)
    high_report = critic_update(high.model, high, batch, deadline=None)
    assert low_report['value_loss'] == high_report['value_loss'] == float(expected)
    same(model.state_dict(), high.model.state_dict())
    same(low.optimizer.state_dict(), high.optimizer.state_dict())
    assert low_report['gradient_diagnostics']['actual_gradient_means'] == high_report['gradient_diagnostics']['actual_gradient_means']
