"""Frozen-policy, train-only objective diagnostics; never updates parameters.

Fresh population critic reproduces pilot initialization, not the original
four-agent trained critic. PPO derivatives use parent stochastic on-policy
rollouts. Later actors are compared only through fixed-observation fit and
critic scores, not through invalid on-policy PPO claims.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import random
import subprocess
import time
import uuid

import numpy as np
import torch
from torch.nn import functional as F

from experiments.run_highway_population import (
    AGENTS, ROOT, atomic_json, digest, env_for, episode, load_config,
    model_for, sources, warm_start,
)

PARENT = 'outputs/highway-stage1/v03/train-20261103T230506Z-04ba1761/selected-00036-d97376b9/best.pt'
LATER = {
    'population_v01': 'outputs/highway-population/v01/train-20261104T225405Z-302c4943/last.pt',
    'population_v02': 'outputs/highway-population/v01/train-20261105T045039Z-a6db8f1b/last.pt',
}


def model_digest(model):
    value = hashlib.sha256()
    for key, tensor in sorted(model.state_dict().items()):
        value.update(key.encode()); value.update(tensor.detach().cpu().numpy().tobytes())
    return value.hexdigest()


def rows_for(cfg):
    """Twelve declared cases, one for each task/count; no validation geometry."""
    rows = []
    for task_index, task in enumerate(('cruise', 'following', 'passing', 'merge')):
        bank = [c for c in cfg['training_cases'] if c['task'] == task]
        for count_index, count in enumerate((1, 4, 8)):
            case = deepcopy(bank[(task_index + count_index) % len(bank)])
            assert case['split'] == 'train'
            total = (24, 16, 8)[count_index]
            if task in ('following', 'passing') and total == count:
                total = 16
            channel = cfg['population']['channels'][(task_index + count_index) % 3]
            rows.append(dict(case=case, seed=6530000 + len(rows),
                controlled_count=count, total_vehicles=total, **channel))
    assert not (set(r['seed'] for r in rows) & set(r['seed'] for r in cfg['validation_cases']))
    return rows


def flatten_active(batch, *, samples=128, seed=0):
    """Normalize advantages on the full active rollout before diagnostic sampling."""
    active = batch['active'].reshape(-1).bool()
    data = {}
    for key, tensor in batch.items():
        if key == 'active':
            continue
        tail = tensor.shape[2:]
        data[key] = tensor.reshape((-1,) + tail)[active].detach()
    advantage = data['advantages']
    data['advantages'] = (advantage - advantage.mean()) / (advantage.std(unbiased=False) + 1e-8)
    count = min(samples, len(advantage))
    indices = torch.randperm(len(advantage), generator=torch.Generator().manual_seed(seed))[:count]
    return {key: value[indices] for key, value in data.items()}


def objectives(model, data, cfg):
    """Exactly the trainer's five pre-weighted terms on a frozen on-policy batch."""
    logp, entropy = model.evaluate_actions(data['local_obs'], data['actions'], data['pre_tanh_actions'])
    log_ratio = logp - data['old_log_probs']
    ratio = log_ratio.exp()
    clip = cfg['optimizer']['clip']
    ppo = -torch.minimum(ratio * data['advantages'],
        ratio.clamp(1 - clip, 1 + clip) * data['advantages']).mean()
    predicted = model.values(data['global_states'], data['agent_indices'])
    value_clip = cfg['optimizer']['value_clip']
    clipped = data['old_values'] + (predicted - data['old_values']).clamp(-value_clip, value_clip)
    value = .5 * torch.maximum((predicted - data['returns']).square(),
        (clipped - data['returns']).square()).mean()
    raw = model.actor.mean_net(data['local_obs'])
    acceleration = F.mse_loss(raw[:, 0].tanh(), data['teacher_actions'][:, 0])
    logits = model.actor.lane_logits(data['local_obs'], raw)
    labels = (data['teacher_actions'][:, 1] + 1).long()
    weights = torch.where(labels == 1, torch.ones_like(labels, dtype=torch.float32),
        torch.full_like(labels, cfg['optimizer']['imitation_lane_change_weight'], dtype=torch.float32))
    lane = (F.cross_entropy(logits, labels, reduction='none') * weights).mean()
    return dict(ppo=ppo, acceleration=acceleration, lane=lane,
                entropy=entropy.mean(), value=value), dict(
                    approximate_kl=float(((ratio - 1) - log_ratio).mean().detach()),
                    max_abs_log_ratio=float(log_ratio.abs().max().detach()))


def gradient_vector(loss, parameters):
    gradients = torch.autograd.grad(loss, parameters, allow_unused=True, retain_graph=True)
    return torch.cat([torch.zeros_like(p).reshape(-1) if g is None else g.detach().reshape(-1)
                      for p, g in zip(parameters, gradients)])


def cosine(a, b):
    denominator = a.norm() * b.norm()
    return None if float(denominator) < 1e-15 else float(torch.dot(a, b) / denominator)


def clipping_report(gradient, actor_width, threshold=.5):
    actor, critic = gradient[:actor_width], gradient[actor_width:]
    global_factor = min(1., threshold / (float(gradient.norm()) + 1e-6))
    actor_factor = min(1., threshold / (float(actor.norm()) + 1e-6))
    return dict(global_norm=float(gradient.norm()), actor_norm=float(actor.norm()),
        critic_norm=float(critic.norm()), global_clip_factor=global_factor,
        actor_norm_after_global_clip=float(actor.norm()) * global_factor,
        actor_only_clip_factor=actor_factor,
        actor_norm_after_hypothetical_separate_clip=float(actor.norm()) * actor_factor,
        counterfactual_only=True, optimizer_updates=0)


def gradient_report(model, data, cfg):
    actor_params = list(model.actor.parameters()) + [model.log_std]
    parameters = actor_params + list(model.critic.parameters())
    width = sum(p.numel() for p in actor_params)
    losses, ratio = objectives(model, data, cfg)
    vectors = {key: gradient_vector(loss, parameters) for key, loss in losses.items()}
    output = dict(samples=len(data['advantages']), loss_values={k: float(v.detach()) for k, v in losses.items()},
        likelihood=ratio, per_weight={})
    for weight in (1., 50.):
        coefficients = dict(ppo=1., acceleration=cfg['optimizer']['imitation_coef'] * weight,
            lane=cfg['optimizer']['imitation_coef'], entropy=-cfg['optimizer']['entropy_coef'],
            value=cfg['optimizer']['value_coef'])
        parts = {key: vectors[key] * coefficients[key] for key in vectors}
        total = sum(parts.values())
        output['per_weight'][str(int(weight))] = dict(
            weighted_contribution_norms={key: dict(actor=float(v[:width].norm()), critic=float(v[width:].norm()))
                for key, v in parts.items()}, clipping=clipping_report(total, width),
            actor_cosines=dict(ppo_acceleration=cosine(parts['ppo'][:width], parts['acceleration'][:width]),
                ppo_lane=cosine(parts['ppo'][:width], parts['lane'][:width]),
                acceleration_lane=cosine(parts['acceleration'][:width], parts['lane'][:width])))
    return output


def explained_variance(target, prediction):
    variance = target.var(unbiased=False)
    return None if float(variance) < 1e-12 else float(1 - (target - prediction).var(unbiased=False) / variance)


def fit_report(model, data):
    with torch.no_grad():
        actions = model.actor(data['local_obs'])
        teacher_error = (actions[:, 0] - data['teacher_actions'][:, 0]) * 3.25
        predicted_accel = -4.5 + (actions[:, 0] + 1) * 3.25
        feasible_error = predicted_accel - data['feasible_target']
        teacher_physical = -4.5 + (data['teacher_actions'][:, 0] + 1) * 3.25
        teacher_reward_difference = teacher_physical - data['feasible_target']
        values = model.values(data['global_states'], data['agent_indices'])
    return dict(samples=len(actions), teacher_acceleration_rmse_m_s2=float(teacher_error.square().mean().sqrt()),
        teacher_acceleration_mae_m_s2=float(teacher_error.abs().mean()),
        teacher_acceleration_bias_m_s2=float(teacher_error.mean()),
        lane_agreement=float((actions[:, 1] == data['teacher_actions'][:, 1]).float().mean()),
        feasible_target_mae_m_s2=float(feasible_error.abs().mean()),
        teacher_vs_feasible_target_mae_m_s2=float(teacher_reward_difference.abs().mean()),
        critic_explained_variance_against_parent_gae_targets=explained_variance(data['returns'], values),
        critic_mse_against_parent_gae_targets=float((data['returns'] - values).square().mean()),
        critic_target_description='Fixed parent-policy GAE targets bootstrapped with fresh population critic; diagnostic, not true value or on-policy targets for later actors')


def run_diagnostics(run, max_seconds=360, horizon=200):
    started = time.monotonic(); deadline = started + max_seconds
    cfg = load_config(ROOT / 'experiments/configs/highway_population_v1.json')
    torch.set_num_threads(1); torch.manual_seed(cfg['training']['seed'])
    np.random.seed(cfg['training']['seed']); random.seed(cfg['training']['seed'])
    torch.use_deterministic_algorithms(True)
    models = {'parent_actor_fresh_population_critic': model_for(cfg)}
    parent = models['parent_actor_fresh_population_critic']
    warm_start(parent, cfg)
    files = {'parent': PARENT, **LATER}
    before_files = {key: digest(ROOT / path) for key, path in files.items()}
    for name, path in LATER.items():
        saved = torch.load(ROOT / path, map_location='cpu', weights_only=False)
        models[name] = model_for(cfg); models[name].load_state_dict(saved['model']); models[name].eval()
    before_models = {name: model_digest(model) for name, model in models.items()}
    source_hashes = sources(); source_hashes['experiments/diagnose_population_losses.py'] = digest(Path(__file__))
    rows = rows_for(cfg)
    atomic_json(run / 'protocol.json', dict(rows=rows, horizon=horizon, max_seconds=max_seconds,
        optimizer_updates=0, source_hashes=source_hashes, checkpoint_hashes=before_files,
        git_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        critic='New masked population critic initialized with seed9207, not original four-agent critic',
        sampling='Parent stochastic on-policy rollout. Equal128-row samples/case after per-rollout active-only advantage normalization. Cases balanced by task and AV count, not a full factorial.',
        evaluation='Short training-only diagnostic windows; no acceptance or held-out evaluation'))
    # Loading alternative models changes torch RNG; reset to the declared seed.
    torch.manual_seed(cfg['training']['seed'])
    dataset = []; reports = []
    for index, row in enumerate(rows):
        if time.monotonic() >= deadline:
            raise TimeoutError('Diagnostic wall budget reached before collection completed')
        env = env_for(cfg, run / 'episodes' / str(index), row, horizon=horizon)
        original_step = env.step
        targets = []
        def capture_step(actions):
            outcome = original_step(actions)
            targets.append(torch.tensor([outcome[4].get(a, {}).get('feasible_acceleration_target', 0.) for a in AGENTS]))
            return outcome
        env.step = capture_step
        report, batch = episode(env, row, model=parent, collect=True, deadline=deadline)
        if batch is None:
            raise ValueError('No diagnostic samples')
        batch['feasible_target'] = torch.stack(targets)
        torch.save(batch, run / f'rollout-{index:02d}.pt')
        atomic_json(run / f'case-{index:02d}.json', report)
        data = flatten_active(batch, samples=128, seed=6531000 + index)
        data['case_index'] = torch.full((len(data['advantages']),), index, dtype=torch.long)
        dataset.append(data); reports.append(report)
    data = {key: torch.cat([d[key] for d in dataset]) for key in dataset[0]}
    torch.save(data, run / 'fixed-diagnostic-observations.pt')
    sealed = digest(run / 'fixed-diagnostic-observations.pt')
    summary = dict(passed=False, optimizer_updates=0, learning=False,
        dataset_sha256=sealed, sampled_rows=len(data['advantages']),
        actual_agent_transitions=sum(r['agent_transitions'] for r in reports),
        sampling='128 rows per case, twelve cases, equal case weights; PPO advantages normalized within each full rollout',
        aggregate_gradients=gradient_report(parent, data, cfg),
        per_case_gradients=[dict(case_index=i, task=rows[i]['case']['task'], population=rows[i]['controlled_count'],
            gradient=gradient_report(parent, item, cfg)) for i, item in enumerate(dataset)],
        fixed_observation_fits={name: fit_report(model, data) for name, model in models.items()},
        per_case_fits={name: [dict(case_index=i, task=rows[i]['case']['task'], population=rows[i]['controlled_count'],
            fit=fit_report(model, item)) for i, item in enumerate(dataset)] for name, model in models.items()},
        limitations=['No optimizer step or hypothetical Adam update was applied.',
            'Gradient comparison is at the frozen parent before an update, not proof of historical cause.',
            'Counterfactual separate clipping scales derivatives only; Adam moments can change resulting steps.',
            'Fresh critic represents population initialization, not the original trained four-car critic.',
            'Later actor fit uses parent-visited states; critic targets use parent fresh-critic GAE.',
            'No full-horizon acceptance, held-out data or physical/sensor validation.'])
    after_files = {key: digest(ROOT / path) for key, path in files.items()}
    after_models = {name: model_digest(model) for name, model in models.items()}
    if before_files != after_files or before_models != after_models:
        raise ValueError('Frozen checkpoint or model changed')
    if sealed != digest(run / 'fixed-diagnostic-observations.pt'):
        raise ValueError('Sealed dataset changed')
    changes = [name for name, sha in source_hashes.items() if digest(ROOT / name) != sha]
    if changes:
        raise ValueError('Sources changed during diagnostics: ' + str(changes))
    summary.update(passed=True, checkpoint_hashes_before=before_files, checkpoint_hashes_after=after_files,
        model_hashes_before=before_models, model_hashes_after=after_models,
        source_hashes=source_hashes, sources_changed=changes, wall_seconds=time.monotonic() - started)
    atomic_json(run / 'loss-summary.json', summary)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--max-seconds', type=int, default=360)
    args = parser.parse_args()
    if Path.cwd().resolve() != ROOT or not 1 <= args.max_seconds <= 360:
        raise ValueError('Canonical root and bounded no-learning budget required')
    run = ROOT / 'outputs/population-diagnostics/v01' / ('loss-' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:8])
    run.mkdir(parents=True)
    try:
        summary = run_diagnostics(run, args.max_seconds)
    except Exception as exc:
        atomic_json(run / 'failure.json', dict(passed=False, learning=False, optimizer_updates=0, error=repr(exc)))
        raise
    print(json.dumps(dict(run=str(run), passed=summary['passed'], optimizer_updates=0,
        samples=summary['sampled_rows'], seconds=summary['wall_seconds']), indent=2))


if __name__ == '__main__':
    main()
