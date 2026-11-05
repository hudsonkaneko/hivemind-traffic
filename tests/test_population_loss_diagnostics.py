from copy import deepcopy

import pytest
import torch

from experiments.diagnose_population_losses import (
    clipping_report, cosine, flatten_active, gradient_report, model_digest,
    objectives, rows_for, explained_variance,
)
from experiments.run_highway_population import DEFAULT, load_config, model_for
from policies.mappo import MappoTrainer


def batch_data(model):
    local = torch.zeros(12, 93); local[:, 31] = 1
    state = torch.zeros(12, 752)
    state[:, :93] = local; state[:, 93] = 1
    indices = torch.zeros(12, dtype=torch.long)
    with torch.no_grad():
        actions, logp, _, latent = model.act(local)
        values = model.values(state, indices)
    return dict(local_obs=local, global_states=state, agent_indices=indices,
        actions=actions, old_log_probs=logp, pre_tanh_actions=latent,
        old_values=values, returns=values + torch.linspace(-1, 2, 12),
        advantages=torch.linspace(-1, 1, 12), teacher_actions=torch.zeros(12, 2))


def test_objective_decomposition_matches_trainer_and_is_frozen():
    torch.set_num_threads(1); torch.manual_seed(23)
    cfg = load_config(DEFAULT); model = model_for(cfg); data = batch_data(model)
    before = model_digest(model)
    parts, ratio = objectives(model, data, cfg)
    trainer = MappoTrainer(model, **cfg['optimizer'])
    imitation = trainer._imitation_loss(data['local_obs'], data['teacher_actions'])
    assert torch.allclose(parts['acceleration'], imitation[1])
    assert torch.allclose(parts['lane'], imitation[2])
    assert ratio['max_abs_log_ratio'] < 1e-6
    report = gradient_report(model, data, cfg)
    assert report['per_weight']['50']['weighted_contribution_norms']['acceleration']['actor'] == pytest.approx(
        50 * report['per_weight']['1']['weighted_contribution_norms']['acceleration']['actor'])
    assert report['per_weight']['50']['weighted_contribution_norms']['lane'] == report['per_weight']['1']['weighted_contribution_norms']['lane']
    assert report['per_weight']['1']['weighted_contribution_norms']['value']['actor'] == 0
    assert model_digest(model) == before
    assert all(p.grad is None for p in model.parameters())


def test_global_critic_gradient_can_reduce_actor_derivative():
    value = clipping_report(torch.tensor([3., 4., 120.]), 2)
    assert value['actor_norm'] == 5
    assert value['actor_norm_after_global_clip'] < value['actor_norm_after_hypothetical_separate_clip']
    assert value['optimizer_updates'] == 0 and value['counterfactual_only']
    assert cosine(torch.zeros(2), torch.ones(2)) is None
    assert explained_variance(torch.ones(3), torch.zeros(3)) is None


def test_masked_advantage_normalization_excludes_padding():
    active = torch.tensor([[True, False], [True, False]])
    batch = dict(active=active, advantages=torch.tensor([[1., 1000.], [3., -1000.]]),
                 local_obs=torch.zeros(2, 2, 93))
    data = flatten_active(batch, samples=2)
    assert sorted(data['advantages'].tolist()) == [-1., 1.]


def test_rows_only_training_and_equal_task_population_cases():
    cfg = load_config(DEFAULT); rows = rows_for(cfg)
    assert len(rows) == 12
    assert all(row['case']['split'] == 'train' for row in rows)
    assert len(set(row['seed'] for row in rows)) == 12
    for count in (1, 4, 8):
        assert sum(row['controlled_count'] == count for row in rows) == 4
    assert not (set(r['seed'] for r in rows) & set(r['seed'] for r in cfg['validation_cases']))
