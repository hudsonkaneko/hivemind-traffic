"""Synthetic derivative checks only: no simulator, optimizer steps or saved actor."""
import copy
import math

import pytest
import torch

from policies.mappo import MappoActorCritic, MappoTrainer


CAP = 26.8224


@pytest.fixture(autouse=True)
def one_thread():
    previous = torch.get_num_threads()
    torch.set_num_threads(1)
    yield
    torch.set_num_threads(previous)


def model_with_acceleration(acceleration=1.):
    torch.manual_seed(404)
    model = MappoActorCritic(observation_size=93, state_size=188, num_agents=2,
        actor_hidden_sizes=(8,), critic_hidden_sizes=(8,), message_encoder=True,
        hybrid_actions=True, driving_local_size=45, population_critic=True)
    head = model.actor.mean_net[-1][-1]
    with torch.no_grad():
        head.weight.zero_()
        head.bias.zero_()
        head.bias[0] = math.atanh((acceleration + 1.25) / 3.25)
    return model


def observations(speeds, limit=12.):
    result = torch.zeros(len(speeds), 93)
    result[:, 0] = torch.tensor(speeds) / CAP
    result[:, 4] = limit / CAP
    result[:, 30:33] = 1.
    return result


def labels(accelerations):
    values = torch.tensor(accelerations)
    return torch.stack(((values + 1.25) / 3.25, torch.zeros_like(values)), -1)


def test_target_loss_uses_physical_units_without_below_band_dilution():
    trainer = MappoTrainer(model_with_acceleration(), near_limit_coef=.1)
    obs = observations([12., 12., 2., 4.])
    targets = labels([-.04, -.04, 2., 2.])
    loss, stats = trainer._near_limit_terms(obs, targets)
    assert float(loss.detach()) == pytest.approx(1.04 ** 2, abs=1e-6)
    assert stats['near_limit_samples'] == 2
    assert stats['near_limit_sample_fraction'] == .5
    assert stats['near_limit_above_target_0p05_fraction'] == 1.
    assert torch.equal(loss, trainer._near_limit_loss(obs[:2], targets[:2]))


def test_selection_includes_exact_lower_boundary_and_overspeed():
    trainer = MappoTrainer(model_with_acceleration(), near_limit_coef=.1)
    obs = observations([11., 10.99, 12.2])
    obs[0, 0] = obs[0, 4] - trainer.near_limit_band_m_s / CAP
    _, stats = trainer._near_limit_terms(obs, labels([0., 0., -.4]))
    assert stats['near_limit_samples'] == 2


def test_empty_selection_has_zero_loss_and_zero_gradients():
    model = model_with_acceleration()
    trainer = MappoTrainer(model, near_limit_coef=.1)
    loss, stats = trainer._near_limit_terms(observations([2., 4.]), labels([2., 2.]))
    assert float(loss.detach()) == 0.
    assert stats['near_limit_samples'] == 0
    gradients = torch.autograd.grad(loss, tuple(model.actor.parameters()), allow_unused=True)
    assert all(g is None or torch.count_nonzero(g) == 0 for g in gradients)


@pytest.mark.parametrize('prediction,target,expected_sign', [(1., -.04, 1), (.5, -.4, 1), (-1., .2, -1)])
def test_gradient_corrects_excess_acceleration_and_recovers_from_overbraking(prediction, target, expected_sign):
    model = model_with_acceleration(prediction)
    trainer = MappoTrainer(model, near_limit_coef=.1)
    teacher = labels([target]).requires_grad_(True)
    loss = trainer._near_limit_loss(observations([12.]), teacher)
    head = model.actor.mean_net[-1][-1]
    grad = torch.autograd.grad(loss, head.bias, retain_graph=True)[0]
    assert float(grad[0]) * expected_sign > 0
    assert torch.count_nonzero(grad[1:]) == 0
    others = torch.autograd.grad(loss, [model.log_std, *model.critic.parameters(), teacher], allow_unused=True)
    assert all(g is None for g in others)


def batch_for(model):
    obs = observations([12., 12.1, 11.2, 5.])
    state = torch.cat((obs, torch.ones(4, 1)), -1).reshape(2, 188).repeat_interleave(2, dim=0)
    indices = torch.tensor([0, 1, 0, 1])
    with torch.no_grad():
        actions, logp, _, latent = model.act(obs, deterministic=True)
        values = model.values(state, indices)
    return dict(local_obs=obs, global_states=state, agent_indices=indices,
        actions=actions, pre_tanh_actions=latent, old_log_probs=logp, old_values=values,
        returns=values + torch.arange(4), advantages=torch.arange(4).float(),
        active=torch.ones(4, dtype=torch.bool), teacher_actions=labels([-.04, -.1, .3, 2.]))


def test_actual_objective_contains_exact_separate_weighted_loss_without_updates(monkeypatch):
    model = model_with_acceleration()
    trainer = MappoTrainer(model, near_limit_coef=.1, gradient_diagnostics=True)
    batch = batch_for(model)
    state = copy.deepcopy(model.state_dict())
    rng = torch.get_rng_state()
    monkeypatch.setattr(trainer.optimizer, 'step', lambda: pytest.fail('No optimizer steps allowed'))
    data, _ = trainer._prepare_batch(batch)
    logp, entropy = model.evaluate_actions(data['local_obs'], data['actions'], data['pre_tanh_actions'])
    loss, parts, stats = trainer._loss_terms(data, (logp-data['old_log_probs']).exp(), entropy)
    assert torch.equal(parts['near_limit'], .1 * trainer._near_limit_loss(data['local_obs'], data['teacher_actions']))
    legacy = MappoTrainer(model, near_limit_coef=0)
    original, original_parts, _ = legacy._loss_terms(data, (logp-data['old_log_probs']).exp(), entropy)
    assert torch.equal(loss, original + parts['near_limit'])
    assert all(torch.equal(parts[k], value) for k, value in original_parts.items())
    report = trainer.diagnose_batch(batch)
    assert report['representative_minibatch']['components']['near_limit']['actor'] > 0
    assert report['representative_minibatch']['components']['near_limit']['critic'] == 0
    assert 'ppo__near_limit' in report['representative_minibatch']['actor_cosines']
    assert stats['near_limit_samples'] == 3
    assert report['optimizer_steps'] == 0
    assert not trainer.optimizer.state
    assert torch.equal(rng, torch.get_rng_state())
    assert all(torch.equal(value, model.state_dict()[key]) for key, value in state.items())
    assert all(p.grad is None for p in model.parameters())


def test_disabled_default_is_exactly_legacy_loss_and_requires_no_extra_targets():
    model = model_with_acceleration()
    batch = batch_for(model)
    del batch['teacher_actions']
    original, explicit = MappoTrainer(model), MappoTrainer(model, near_limit_coef=0)
    left, _ = original._prepare_batch(batch)
    right, _ = explicit._prepare_batch(batch)
    logp, entropy = model.evaluate_actions(left['local_obs'], left['actions'], left['pre_tanh_actions'])
    ratio = (logp-left['old_log_probs']).exp()
    a, pa, sa = original._loss_terms(left, ratio, entropy)
    b, pb, sb = explicit._loss_terms(right, ratio, entropy)
    assert torch.equal(a, b) and pa.keys() == pb.keys() and sa == sb
    assert 'near_limit' not in pa


def test_update_telemetry_distinguishes_step_losses_from_selected_row_exposures():
    trainer = MappoTrainer(model_with_acceleration(1.), near_limit_coef=.1)
    _, first = trainer._near_limit_terms(observations([12., 4.]), labels([0., 0.]))
    _, second = trainer._near_limit_terms(observations([12., 12., 12.]), labels([-1., -1., -1.]))
    rows = [dict(loss=7., **first), dict(loss=9., **second)]
    original = copy.deepcopy(rows)
    result = trainer._aggregate_update_stats(rows)
    assert result['loss'] == 8.
    assert result['near_limit_conditional_step_mean_mse_m2_s4'] == pytest.approx(2.5)
    assert result['near_limit_selected_exposure_weighted_mse_m2_s4'] == pytest.approx(3.25)
    assert result['near_limit_selected_row_exposures'] == 4
    assert result['near_limit_row_exposures'] == 5
    assert result['near_limit_selected_exposure_fraction'] == .8
    assert result['near_limit_selected_exposure_weighted_raw_acceleration_mean_m_s2'] == pytest.approx(1.)
    assert result['near_limit_selected_exposure_weighted_target_mean_m_s2'] == pytest.approx(-.75)
    assert result['near_limit_selected_exposure_weighted_above_target_0p05_fraction'] == 1.
    assert 'near_limit_samples' not in result and 'near_limit_mse_m2_s4' not in result
    assert rows == original
    repeated = trainer._aggregate_update_stats(rows * 2)
    assert repeated['near_limit_selected_row_exposures'] == 8
    assert repeated['near_limit_selected_exposure_weighted_mse_m2_s4'] == result['near_limit_selected_exposure_weighted_mse_m2_s4']


def test_update_telemetry_empty_selection_is_not_zero_error_evidence():
    trainer = MappoTrainer(model_with_acceleration(), near_limit_coef=.1)
    _, stats = trainer._near_limit_terms(observations([3., 5.]), labels([0., 0.]))
    result = trainer._aggregate_update_stats([stats])
    assert result['near_limit_conditional_step_mean_mse_m2_s4'] == 0.
    assert result['near_limit_selected_row_exposures'] == 0
    assert result['near_limit_row_exposures'] == 2
    assert result['near_limit_selected_exposure_fraction'] == 0.
    assert all(value is None for name, value in result.items() if name.startswith('near_limit_selected_exposure_weighted'))


def test_update_telemetry_disabled_preserves_legacy_aggregation():
    assert MappoTrainer._aggregate_update_stats([{'loss': 2., 'entropy': .5}, {'loss': 4., 'entropy': 1.5}]) == {'loss': 3., 'entropy': 1.}
    assert MappoTrainer._aggregate_update_stats([]) == {}


@pytest.mark.parametrize('setting', [{'near_limit_coef': -.1}, {'near_limit_coef': float('nan')},
    {'near_limit_band_m_s': 0}, {'near_limit_band_m_s': float('inf')}])
def test_invalid_settings_rejected(setting):
    with pytest.raises(ValueError, match='near-limit'):
        MappoTrainer(model_with_acceleration(), **setting)


def test_active_near_limit_loss_rejects_wrong_contract_and_missing_targets():
    with pytest.raises(ValueError, match='93-field'):
        MappoTrainer(MappoActorCritic(), near_limit_coef=.1)
    model = model_with_acceleration()
    trainer = MappoTrainer(model, near_limit_coef=.1)
    batch = batch_for(model)
    del batch['teacher_actions']
    with pytest.raises(ValueError, match='teacher_actions'):
        trainer._prepare_batch(batch)


@pytest.mark.parametrize('problem', ['nan', 'bad_limit', 'bad_speed', 'bad_target'])
def test_invalid_active_observations_or_labels_fail_loudly(problem):
    trainer = MappoTrainer(model_with_acceleration(), near_limit_coef=.1)
    obs, target = observations([12.]), labels([0.])
    if problem == 'nan': obs[0, 0] = float('nan')
    if problem == 'bad_limit': obs[0, 4] = 0
    if problem == 'bad_speed': obs[0, 0] = -1
    if problem == 'bad_target': target[0, 0] = 2
    with pytest.raises(ValueError):
        trainer._near_limit_loss(obs, target)
