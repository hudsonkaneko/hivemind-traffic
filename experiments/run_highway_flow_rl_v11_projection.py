"""V11 shield projection: fine-tune the actor onto the command filters' band without PPO.

Round 15 showed that PPO updates at this scale change far more than the term they
are given: 200,000 decisions took the policy from 5 of 8 gates to 2 of 8. This
runner instead makes the smallest change that removes requests the filters would
alter. It alternates two steps:

1. Collect complete training episodes with the current actor in the v11
   environment (ordinary sampled rollouts; decisions are charged to the shared
   ledger as training).
2. Fit the actor on every observation collected so far with
   `hinge_weight * shield_consistency + preserve_weight * (request - reference
   request)^2 + lane_weight * (lane outputs - reference lane outputs)^2`,
   where the reference is the frozen warm-start actor.

Inside the filters' band the first term is zero, so the fit only holds the actor
to the warm start there. No reward, advantage, critic or teacher label is used;
the critic and the action noise are not touched. After each fit the 8 development
validation cases are run and the actor is saved.
"""
from __future__ import annotations

import argparse
import json
import random
import sys
import time
from copy import deepcopy
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402
import torch  # noqa: E402

from experiments import run_highway_flow_rl_v11 as runner  # noqa: E402
from experiments.run_highway_flow_rl import training_row  # noqa: E402
from policies.mappo import export_actor  # noqa: E402
from traffic.highway_flow_contract import file_digest  # noqa: E402

base = runner.base
CHECKPOINT_SCHEMA = 'highway_flow_rl_checkpoint_v11_projection'


def actor_outputs(model, observations):
    """Mean acceleration request (m/s2) and the raw lane outputs of the actor."""
    raw = model.actor.mean_net(observations)
    return -4.5 + (raw[..., 0].tanh() + 1.) * 3.25, raw[..., 1:]


def projection_loss(model, reference, observations, settings):
    request, lanes = actor_outputs(model, observations)
    with torch.no_grad():
        reference_request, reference_lanes = actor_outputs(reference, observations)
    outside = runner.shield_consistency(request,
        observations[..., runner.ACCELERATION_INDEX] * runner.OBSERVATION_SCALE_M_S2,
        observations[..., runner.CEILING_INDEX] * runner.OBSERVATION_SCALE_M_S2,
        ceiling_margin=settings['ceiling_margin_m_s2'], rate_bound=settings['rate_bound_m_s2'])
    preserve = (request - reference_request).square()
    lane = (lanes - reference_lanes).square().mean(-1)
    loss = (settings['hinge_weight'] * outside + settings['preserve_weight'] * preserve
            + settings['lane_weight'] * lane).mean()
    return loss, dict(shield_m2_s4=float(outside.mean().detach()), outside_fraction=float((outside > 0).float().mean().detach()),
                      preserve_m2_s4=float(preserve.mean().detach()), lane_mse=float(lane.mean().detach()),
                      max_request_change_m_s2=float((request - reference_request).abs().max().detach()))


def fit(model, reference, observations, settings, generator):
    """Fit the actor only; returns the measured terms before and after."""
    optimizer = torch.optim.Adam(model.actor.parameters(), lr=settings['learning_rate'])
    with torch.no_grad():
        before = projection_loss(model, reference, observations, settings)[1]
    steps = 0
    for _ in range(settings['epochs']):
        order = torch.randperm(len(observations), generator=generator)
        for start in range(0, len(order), settings['batch_size']):
            loss, _ = projection_loss(model, reference, observations[order[start:start + settings['batch_size']]], settings)
            if not torch.isfinite(loss):
                raise ValueError('Nonfinite projection loss')
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.actor.parameters(), 1., error_if_nonfinite=True)
            optimizer.step()
            steps += 1
    with torch.no_grad():
        after = projection_loss(model, reference, observations, settings)[1]
    return dict(samples=len(observations), optimizer_steps=steps, before=before, after=after)


def load_actor(cfg):
    path = base.project_path(cfg['warm_start']['path'])
    if file_digest(path) != cfg['warm_start']['sha256']:
        raise ValueError('Pinned warm-start checksum changed')
    saved = torch.load(path, map_location='cpu', weights_only=False)
    model = base.FlowModel()
    model.load_state_dict(saved['model'], strict=True)
    return model


def run(args):
    started = time.monotonic()
    if Path.cwd().resolve() != ROOT.resolve():
        raise ValueError('Run from the Claude worktree root')
    runner.install()
    config_path = base.project_path(args.config)
    cfg = json.loads(config_path.read_text())
    base.validate_config(cfg, require_training=True)
    settings = cfg['v11_projection']
    fingerprint, sources = file_digest(config_path), base.hashes()
    output = base.project_path(args.output)
    if not output.is_relative_to(base.OUTPUT_ROOT.resolve()) or output.exists():
        raise ValueError('Use a fresh output directory under outputs/highway-flow-rl/v11')
    with base.LedgerSession(cfg, 'train', output, fingerprint, started=started) as session:
        output.mkdir(parents=True)
        torch.set_num_threads(1)
        torch.use_deterministic_algorithms(True)
        torch.manual_seed(cfg['seed'])
        np.random.seed(cfg['seed'])
        random.seed(cfg['seed'])
        generator = torch.Generator().manual_seed(cfg['seed'])
        base.save(output / 'resolved-config.json', cfg)
        base.save(output / 'manifest.json', dict(config_sha256=fingerprint, source_hashes=sources,
            procedure='shield projection (supervised actor fit; no PPO, no critic update)',
            warm_start=cfg['warm_start'], previous_charged_seconds=session.totals['total'],
            previous_round_decisions=session.totals['round_decisions']))
        if session.totals['round_decisions']:
            raise ValueError('This round already has charged training decisions')
        model, reference = load_actor(cfg), load_actor(cfg)
        reference.eval()
        for parameter in reference.parameters():
            parameter.requires_grad_(False)
        frozen = {name: value.clone() for name, value in model.state_dict().items() if not name.startswith('actor.')}
        limit = min(cfg['budget']['per_round_decisions'],
                    cfg['budget']['total_decisions'] - session.totals['total_decisions'])
        decisions, episodes, collected, iterations = 0, 0, [], []
        state = dict(ppo_episodes=0)
        for iteration in range(settings['iterations']):
            reports = []
            for _ in range(settings['episodes_per_iteration']):
                row = training_row(cfg, state, False)
                reservation = row['horizon_steps'] * row['controlled_count']
                if decisions + reservation > limit:
                    raise ValueError('Declared decision budget cannot cover the declared collection')
                directory = output / 'train' / f'episode-{episodes:04d}'
                report, batch = base.episode(cfg, row, directory, model, collect=True, limit=reservation,
                                             deadline=session.deadline)
                if report['partial_control_horizon'] or batch is None:
                    raise ValueError('Collection episode did not complete')
                decisions += report['agent_transitions']
                episodes += 1
                state['ppo_episodes'] += 1
                active = batch['active'].reshape(-1).bool()
                collected.append(batch['local_obs'].reshape(-1, 117)[active].clone())
                reports.append(dict(case_id=report['case_id'], seed=row['seed'], decisions=report['agent_transitions'],
                    collisions=report['collisions'], intervention_fraction=report['intervention_fraction'],
                    peak_abs_jerk=report['peak_abs_jerk'], override_causes=report.get('override_causes'),
                    stable_merge={k: report.get('stable_merge', {}).get(k) for k in ('targets', 'stable')}))
                session.checkpoint(decisions)
            observations = torch.cat(collected)
            model.train()
            stats = fit(model, reference, observations, settings, generator)
            model.eval()
            changed = [name for name, value in model.state_dict().items()
                       if not name.startswith('actor.') and not torch.equal(value, frozen[name])]
            if changed:
                raise ValueError('Projection changed parameters outside the actor: ' + ', '.join(changed))
            label = f'projection-validation-{iteration + 1:02d}'
            validation = base.evaluate(cfg, output, label, model, session.deadline)
            export_actor(model, output / 'candidates' / f'policy-{iteration + 1:02d}.pt')
            checkpoint = output / 'candidates' / f'checkpoint-{iteration + 1:02d}.pt'
            torch.save(dict(schema=CHECKPOINT_SCHEMA, model=model.state_dict(), config_sha256=fingerprint,
                            source_hashes=sources, iteration=iteration + 1, decisions=decisions), checkpoint)
            iterations.append(dict(iteration=iteration + 1, episodes=reports, fit=stats, validation_label=label,
                validation_rank=list(base.validation_rank(validation)),
                gates_passed=sum(g['passed'] for g in validation['summary']['case_gates']),
                merge_speed_match=validation['summary'].get('merge_speed_match'),
                checkpoint=checkpoint.relative_to(ROOT).as_posix(), checkpoint_sha256=file_digest(checkpoint)))
            base.save(output / 'iterations.json', iterations)
            session.checkpoint(decisions)
        if base.hashes() != sources:
            raise ValueError('Sources changed during the run')
        best = min(iterations, key=lambda item: tuple(item['validation_rank']))
        result = dict(completed=True, training=True, procedure='shield_projection', round_index=cfg['round_index'],
            decisions=decisions, episodes=episodes, iterations=iterations, best_iteration=best['iteration'],
            elapsed_seconds=session.elapsed(), total_charged_seconds=session.totals['total'] + session.elapsed(),
            training_acceptance='experimental_not_promoted')
        base.save(output / 'summary.json', result)
        return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    result = run(parser.parse_args(argv))
    print(json.dumps({k: result[k] for k in ('completed', 'decisions', 'episodes', 'best_iteration', 'elapsed_seconds')}))
    print(json.dumps([{k: item[k] for k in ('iteration', 'gates_passed', 'merge_speed_match')} for item in result['iterations']]))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
