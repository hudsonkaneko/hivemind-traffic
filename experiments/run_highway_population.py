"""Bounded population-adaptation MAPPO pilot; never launches implicitly."""
from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import random
import shutil
import subprocess
import sys
import time
import traceback
import uuid

import numpy as np
import torch

from experiments.sumo_mappo_support import config_digest
from experiments.run_highway_stage1 import primary_accounting, check_deadline, SOURCES as OLD_SOURCES
from experiments.run_i110_merge import PilotBudgetExhausted
from policies.mappo import MappoActorCritic, MappoTrainer, compute_gae, export_actor

ROOT = Path(__file__).resolve().parents[1]
DEFAULT = ROOT / 'experiments/configs/highway_population_v1.json'
SCHEMA = 'highway_population_v1'
AGENTS = tuple(f'agent_{i}' for i in range(8))
SOURCES = tuple(dict.fromkeys((*OLD_SOURCES, 'experiments/run_highway_population.py',
    'environments/highway_population_env.py')))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def actor_digest(model):
    sha = hashlib.sha256()
    for name, value in sorted(dict(model.actor.state_dict(), log_std=model.log_std).items()):
        sha.update(name.encode()); sha.update(value.detach().cpu().numpy().tobytes())
    return sha.hexdigest()


def atomic_write(path, writer, *, attempts=5):
    """Unique same-directory temporary; preserve the previous valid generation."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    try:
        writer(temporary)
        for attempt in range(attempts):
            try:
                os.replace(temporary, path)
                return
            except PermissionError:
                if attempt + 1 == attempts:
                    raise
                time.sleep(.05 * (attempt + 1))
    finally:
        if temporary.exists():
            temporary.unlink()


def atomic_json(path, value):
    atomic_write(path, lambda p: p.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8'))


def save_checkpoint(path, payload):
    atomic_write(path, lambda p: torch.save(payload, p))


def sources():
    return {name: digest(ROOT / name) for name in SOURCES}


def preparation_digest(cfg):
    return config_digest({k: v for k, v in cfg.items() if k not in ('training_authorized', 'readiness')})


def validate_config(cfg):
    if cfg['schema'] != SCHEMA or type(cfg['training_authorized']) is not bool:
        raise ValueError('Population schema/authorization mismatch')
    if cfg['model'] != dict(observation_size=93, state_size=752, num_agents=8,
            driving_local_size=45, driving_message_size=8, driving_mask_start=30, population_critic=True):
        raise ValueError('Population actor/critic contract mismatch')
    t = cfg['training']
    if (t['device'] != 'cpu' or t['num_workers'] != 1 or t['torch_threads'] != 1
            or not 1 <= t['max_agent_transitions'] <= 100000
            or not 1 <= t['max_wall_seconds'] <= 1800
            or not 0 <= t['critic_warmup_transitions'] <= 4096
            or not 0 < t['final_evaluation_reserve_seconds'] < t['max_wall_seconds']):
        raise ValueError('Pilot training bound mismatch')
    if cfg['environment']['horizon'] != 800 or cfg['population']['training_counts'] != [1, 2, 4, 6, 8]:
        raise ValueError('Population pilot requires full horizons and declared training counts')
    if cfg['population']['total_vehicles'] != [8, 16, 24]:
        raise ValueError('Unexpected demand cohort sizes')
    if cfg['population']['channels'] != [dict(communication='none', packet_loss=0.),
            dict(communication='local', packet_loss=0.), dict(communication='local', packet_loss=.3)]:
        raise ValueError('Unexpected communication domain')
    seeds = [r['seed'] for r in cfg['validation_cases']]
    if len(seeds) != 16 or len(set(seeds)) != 16:
        raise ValueError('Expected sixteen fixed validation cases')
    if any(t['seed_start'] <= seed <= t['seed_end'] for seed in seeds):
        raise ValueError('Training/validation seed leakage')
    if set(t['tasks']) != {'cruise', 'following', 'passing', 'merge'}:
        raise ValueError('All four training tasks required')
    for row in cfg['validation_cases']:
        if row['case']['split'] != 'validation' or row['controlled_count'] > row['total_vehicles']:
            raise ValueError('Invalid validation partition/population')
        if row['case']['task'] in ('following', 'passing') and row['controlled_count'] == row['total_vehicles']:
            raise ValueError('Slow-leader tasks require at least one human vehicle')
    if set(r['controlled_count'] for r in cfg['validation_cases']) != {1, 3, 5, 7, 8}:
        raise ValueError('Validation must include unseen population counts')
    if not cfg['training_cases'] or any(r['split'] != 'train' for r in cfg['training_cases']):
        raise ValueError('Training can only select the training partition')
    if cfg['preflight']['learning'] is not False or not 1 <= cfg['preflight']['max_wall_seconds'] <= 900:
        raise ValueError('Invalid no-learning preflight')


def load_config(path):
    cfg = json.loads(Path(path).read_text(encoding='utf-8'))
    validate_config(cfg)
    return cfg


def model_for(cfg):
    return MappoActorCritic(**cfg['model'], message_encoder=True, hybrid_actions=True)


def warm_start(model, cfg):
    spec = cfg['initial_checkpoint']
    path = (ROOT / spec['path']).resolve()
    if not path.is_relative_to(ROOT / 'outputs') or digest(path) != spec['sha256']:
        raise ValueError('Frozen warm-start checkpoint checksum/path mismatch')
    saved = torch.load(path, map_location='cpu', weights_only=False)
    if saved['schema'] != 'highway_stage1_graph_messages_v2':
        raise ValueError('Parent actor observation contract mismatch')
    actor = {k[len('actor.'):]: v for k, v in saved['model'].items() if k.startswith('actor.')}
    model.actor.load_state_dict(actor, strict=True)
    with torch.no_grad():
        model.log_std.copy_(saved['model']['log_std'])
    return dict(**spec, imported='actor and exploration log_std only; fresh masked critic/optimizers',
                parent_config_sha256=saved['config_sha256'])


def training_case(cfg, episode_index):
    """Seeded shuffled factorial schedule; never selects validation populations."""
    combinations = [(n, total, channel, task) for n in cfg['population']['training_counts']
        for total in cfg['population']['total_vehicles']
        for channel in cfg['population']['channels'] for task in cfg['training']['tasks']
        if not (task in ('following', 'passing') and n == total)]
    order = np.random.default_rng(cfg['training']['seed']).permutation(len(combinations))
    n, total, channel, task = combinations[int(order[episode_index % len(order)])]
    bank = [c for c in cfg['training_cases'] if c['task'] == task]
    cycle = episode_index // len(order)
    case = bank[(episode_index + cycle) % len(bank)]
    seed = cfg['training']['seed_start'] + episode_index
    if seed > cfg['training']['seed_end']:
        raise ValueError('Training seed allocation exhausted')
    return dict(case=case, controlled_count=n, total_vehicles=total, **channel, seed=seed)


def env_for(cfg, output, row, *, horizon=None):
    from environments.highway_population_env import HighwayPopulationEnv
    case = row['case']
    path = ROOT / case['network']
    if digest(path) != case['network_sha256']:
        raise ValueError('Frozen road checksum changed')
    env = HighwayPopulationEnv(path, output, controlled_count=row['controlled_count'],
        total_vehicles=row['total_vehicles'], max_agents=8,
        horizon=horizon or cfg['environment']['horizon'], communication=row['communication'],
        packet_loss=row['packet_loss'], through_lanes=case['through_lanes'], task=case['task'],
        density=case['density'], reward_weights=cfg['reward_weights'])
    env.case_id = case['id']
    return env


def episode(env, row, *, model=None, kind='learned', collect=False, limit=None, deadline=None):
    from traffic.highway_stage1_teacher import teacher_actions
    samples, transitions, reward_sum = [], 0, 0.
    rng = np.random.default_rng(row['seed'] + 70000000)
    try:
        check_deadline(deadline)
        obs, _ = env.reset(seed=row['seed'])
        env.control_mode = 'native' if kind == 'native' else 'policy'
        while env.agents:
            check_deadline(deadline)
            if limit is not None and transitions + len(env.agents) > limit:
                break
            active = torch.tensor([a in env.agents for a in AGENTS])
            indices = torch.arange(8)
            local = torch.tensor(np.stack([obs.get(a, np.zeros(93, np.float32)) for a in AGENTS]))
            state = torch.as_tensor(env.state()).repeat(8, 1)
            labels = teacher_actions({a: obs[a] for a in env.agents})
            if kind == 'learned':
                with torch.no_grad():
                    actions, logp, _, latent = model.act(local, deterministic=not collect)
                    values = model.values(state, indices)
                chosen = {a: actions[i].numpy() for i, a in enumerate(AGENTS) if a in env.agents}
            elif kind in ('teacher', 'native'):
                if kind == 'native':
                    for a in env.agents:
                        env.conn.vehicle.setSpeed(a, -1)
                        env.conn.vehicle.setLaneChangeMode(a, 1621)
                chosen = labels
            elif kind == 'random':
                chosen = {a: np.array([rng.uniform(-1, 1), rng.choice(np.flatnonzero(obs[a][30:33] > .5)) - 1], np.float32)
                          for a in env.agents}
            else:
                raise ValueError('Unknown controller')
            next_obs, rewards, term, trunc, _ = env.step(chosen)
            if not all(math.isfinite(float(v)) for v in rewards.values()):
                raise ValueError('Nonfinite reward')
            transitions += len(rewards)
            reward_sum += sum(rewards.values())
            if collect:
                final = np.concatenate([np.r_[next_obs.get(a, np.zeros(93)), float(not term.get(a, True))]
                                        for a in AGENTS]).astype(np.float32)
                with torch.no_grad():
                    next_values = model.values(torch.tensor(final).repeat(8, 1), indices)
                samples.append(dict(local_obs=local, global_states=state, agent_indices=indices,
                    actions=actions, pre_tanh_actions=latent, old_log_probs=logp, old_values=values,
                    next_values=next_values, rewards=torch.tensor([rewards.get(a, 0.) for a in AGENTS]),
                    terminated=torch.tensor([term.get(a, False) for a in AGENTS]),
                    truncated=torch.tensor([trunc.get(a, False) for a in AGENTS]), active=active,
                    teacher_actions=torch.tensor(np.stack([labels.get(a, np.zeros(2, np.float32)) for a in AGENTS]))))
            obs = {a: next_obs[a] for a in env.agents}
        report = primary_accounting(env, partial=bool(env.agents), deadline=deadline)
        report.update(seed=row['seed'], controller=kind, case_id=env.case_id, reward=reward_sum,
            agent_transitions=transitions, partial_control_horizon=bool(env.agents),
            episode_directory=str(env.directory.relative_to(ROOT)),
            population_setting={k: row[k] for k in ('controlled_count', 'total_vehicles', 'communication', 'packet_loss')})
        batch = None
        if samples:
            batch = {k: torch.stack([r[k] for r in samples]) for k in samples[0]}
            batch['advantages'], batch['returns'] = compute_gae(batch['rewards'], batch['old_values'],
                batch['next_values'], batch['terminated'], batch['truncated'], batch['active'])
        return report, batch
    except Exception as exc:
        if hasattr(env, 'ledger'):
            # Reset can fail before Stage 1 initializes its control statistics.
            # Retain the original failure even if the richer report is unavailable.
            try:
                try:
                    interrupted = env.report()
                except Exception as report_error:
                    interrupted = dict(accounting=env.ledger.snapshot(),
                        report_error=repr(report_error))
                atomic_json(env.directory / 'interrupted-accounting.json', dict(interrupted,
                    primary_horizon_complete=False, accounting_comparison_valid=False,
                    incomplete_episode=True, actual_agent_transitions=transitions,
                    original_error=repr(exc)))
            except Exception as evidence_error:
                exc.add_note('Interrupted evidence could not be saved: ' + repr(evidence_error))
        raise
    finally:
        env.close()


def summarize(reports):
    steps = sum(r['agent_steps'] for r in reports)
    failures = []
    for r in reports:
        causes = []
        required = ('collisions', 'teleports', 'overspeed_steps', 'intervention_fraction',
                    'mean_fraction_of_speed_limit', 'passes', 'successful_merges', 'agent_steps', 'overrides')
        if any(not isinstance(r.get(k), (int, float)) or not math.isfinite(r[k]) for k in required):
            raise ValueError('Missing/nonfinite evaluation metric')
        if any(r[k] for k in ('collisions', 'teleports', 'overspeed_steps')):
            causes.append('safety_event')
        if not r.get('accounting_comparison_valid', False) or r.get('partial_control_horizon'):
            causes.append('incomplete_accounting')
        if r['intervention_fraction'] > .05:
            causes.append('overrides')
        if r['task'] == 'cruise' and r['mean_fraction_of_speed_limit'] < .9:
            causes.append('cruise_speed')
        if r['task'] == 'passing' and r['passes'] < 1:
            causes.append('passing')
        targets = r.get('merge_target_count', 0)
        if isinstance(targets, (list, tuple)):
            targets = len(targets)
        if r['task'] == 'merge' and r['agent_steps']:
            if not isinstance(targets, (int, float)) or not math.isfinite(targets) or targets <= 0:
                raise ValueError('Learned merge report requires its positive target denominator')
            if r['successful_merges'] / targets < .9:
                causes.append('merging')
        if causes:
            failures.append(dict(case_id=r['case_id'], seed=r['seed'], population=r['population_setting'], causes=causes))
    return dict(cases=len(reports), pooled_override_fraction=sum(r['overrides'] for r in reports) / max(1, steps),
        worst_override_fraction=max((r['intervention_fraction'] for r in reports), default=0.),
        failures=failures, nominal_gates_passed=bool(reports) and not failures,
        held_out_test_run=False, acceptance_status='experimental; held-out qualification not run')


def evaluate(cfg, run, model, *, label, deadline, kind='learned', rows=None):
    reports = []
    for index, row in enumerate(rows if rows is not None else cfg['validation_cases']):
        report, _ = episode(env_for(cfg, run / label / str(index), row), row,
                            model=model, kind=kind, deadline=deadline)
        reports.append(report)
        atomic_json(run / label / f'{index:02d}.json', report)
    value = dict(summary=summarize(reports), reports=reports)
    atomic_json(run / f'{label}.json', value)
    return value


def critic_update(model, trainer, batch, deadline):
    active = batch['active'].reshape(-1).bool()
    states = batch['global_states'].reshape(-1, 752)[active]
    indices = batch['agent_indices'].reshape(-1)[active]
    returns = batch['returns'].reshape(-1)[active].detach()
    count, losses = 0, []
    frozen = actor_digest(model)
    exhausted = False
    for _ in range(trainer.epochs):
        for start in range(0, len(returns), trainer.minibatch_size):
            if deadline is not None and time.monotonic() >= deadline:
                exhausted = True
                break
            target = returns[start:start + trainer.minibatch_size]
            value = model.values(states[start:start + trainer.minibatch_size], indices[start:start + trainer.minibatch_size])
            loss = .5 * (value - target).square().mean()
            if not torch.isfinite(loss):
                raise ValueError('Nonfinite critic warmup')
            trainer.optimizer.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.critic.parameters(), trainer.max_grad_norm, error_if_nonfinite=True)
            trainer.optimizer.step()
            count += 1
            losses.append(float(loss.detach()))
        if exhausted:
            break
    if actor_digest(model) != frozen:
        raise ValueError('Critic warmup changed the actor or exploration')
    return dict(optimizer_steps=count, value_loss=sum(losses) / max(1, len(losses)), actor_frozen=True,
                actor_sha256=frozen, budget_exhausted=exhausted)


def readiness_check(cfg):
    ready = cfg.get('readiness') or {}
    path = (ROOT / ready.get('summary_path', '')).resolve()
    if not path.is_relative_to(ROOT / 'outputs') or not path.is_file() or digest(path) != ready.get('summary_sha256'):
        raise ValueError('Verified preflight required before training')
    report = json.loads(path.read_text())
    if not report.get('passed') or report.get('optimizer_steps') != 0 or report.get('source_hashes') != sources():
        raise ValueError('Preflight failed, learned, or used different sources')
    if report.get('preparation_sha256') != preparation_digest(cfg):
        raise ValueError('Preflight configuration drift')


def train(cfg, run, started, hashes, resume=None):
    model = model_for(cfg)
    parent = warm_start(model, cfg)
    trainer = MappoTrainer(model, **cfg['optimizer'])
    fingerprint = config_digest(cfg)
    state = dict(spent=0, actual=0, critic_actual=0, ppo_actual=0, episode_index=0,
        updates=0, optimizer_steps=0, critic_optimizer_steps=0, elapsed_seconds=0.,
        run_closed=False, pending=None, initial_complete=False, final_complete=False, coverage={})
    prior = 0.
    if resume is not None:
        if resume.name != 'last.pt':
            raise ValueError('Resume requires last.pt')
        old = torch.load(resume, map_location='cpu', weights_only=False)
        if old['schema'] != SCHEMA or old['config_sha256'] != fingerprint or old['source_hashes'] != hashes:
            raise ValueError('Resume configuration/source mismatch')
        model.load_state_dict(old['model']); trainer.optimizer.load_state_dict(old['optimizer'])
        state = old['state']; prior = state['elapsed_seconds']
        if not state['run_closed']:
            prior += max(0., time.time() - old['saved_at_unix'])
        torch.set_rng_state(old['torch_rng']); np.random.set_state(old['numpy_rng']); random.setstate(old['python_rng'])
        if state['pending'] and state['pending']['kind'] == 'episode':
            state['pending'] = None  # The prior reservation remains charged after failure.
        for name in ('initial-validation.json', 'native-baseline.json'):
            if (resume.parent / name).exists():
                shutil.copyfile(resume.parent / name, run / name)
    deadline = started + max(0., cfg['training']['max_wall_seconds'] - prior)
    learning_deadline = deadline - cfg['training']['final_evaluation_reserve_seconds']
    state['run_closed'] = False

    def checkpoint():
        state['elapsed_seconds'] = prior + time.monotonic() - started
        save_checkpoint(run / 'last.pt', dict(schema=SCHEMA, config_sha256=fingerprint, config=cfg,
            source_hashes=hashes, parent=parent, model=model.state_dict(), optimizer=trainer.optimizer.state_dict(),
            state=deepcopy(state), saved_at_unix=time.time(), torch_rng=torch.get_rng_state(),
            numpy_rng=np.random.get_state(), python_rng=random.getstate()))

    def pending_update():
        pending = state['pending']
        if not pending or pending['kind'] != 'update':
            return
        if pending['critic_only']:
            stats = critic_update(model, trainer, pending['batch'], learning_deadline)
            state['critic_optimizer_steps'] += stats['optimizer_steps']
        else:
            stats = trainer.update(pending['batch'], deadline=learning_deadline)
            state['optimizer_steps'] += stats['optimizer_steps']
        state['updates'] += 1
        atomic_json(run / f'update-{state["updates"]:05d}.json', dict(report=pending['report'], optimization=stats,
            critic_only=pending['critic_only'], spent=state['spent'], actual=state['actual']))
        state['pending'] = None
        checkpoint()

    checkpoint()
    try:
        if not state['initial_complete']:
            evaluate(cfg, run, model, label='initial-validation', deadline=learning_deadline)
            native = []
            for total in cfg['population']['total_vehicles']:
                row = deepcopy(cfg['validation_cases'][0])
                row.update(controlled_count=0, total_vehicles=total, communication='none', packet_loss=0.)
                native.append(row)
            evaluate(cfg, run, model, label='native-baseline', deadline=learning_deadline, kind='native', rows=native)
            state['initial_complete'] = True
            checkpoint()
        pending_update()
        while state['spent'] < cfg['training']['max_agent_transitions'] and time.monotonic() < learning_deadline:
            row = training_case(cfg, state['episode_index'])
            remaining = cfg['training']['max_agent_transitions'] - state['spent']
            critic_only = state['critic_actual'] < cfg['training']['critic_warmup_transitions']
            if critic_only:
                remaining = min(remaining, cfg['training']['critic_warmup_transitions'] - state['critic_actual'])
                # Avoid a remainder smaller than the current active cohort.
                if remaining < row['controlled_count']:
                    row = dict(row, controlled_count=1)
            reserve = min(remaining, cfg['environment']['horizon'] * row['controlled_count'])
            if reserve < row['controlled_count']:
                break
            state['spent'] += reserve
            state['episode_index'] += 1
            state['pending'] = dict(kind='episode', reserved=reserve, row=row, critic_only=critic_only)
            checkpoint()
            report, batch = episode(env_for(cfg, run / 'train' / str(state['episode_index']), row), row,
                model=model, collect=True, limit=reserve, deadline=learning_deadline)
            actual = report['agent_transitions']
            state['spent'] -= reserve - actual
            state['actual'] += actual
            state['critic_actual' if critic_only else 'ppo_actual'] += actual
            key = '|'.join(map(str, (row['case']['task'], row['controlled_count'], row['total_vehicles'],
                                     row['communication'], row['packet_loss'])))
            coverage = state['coverage'].setdefault(key, dict(episodes=0, actual_agent_transitions=0))
            coverage['episodes'] += 1; coverage['actual_agent_transitions'] += actual
            if batch is None:
                raise ValueError('Training episode yielded no active transitions')
            state['pending'] = dict(kind='update', batch=batch, report=report, critic_only=critic_only)
            checkpoint()
            pending_update()
        stop_reason = 'transition_budget' if state['spent'] >= cfg['training']['max_agent_transitions'] else 'learning_time_or_remainder'
    except PilotBudgetExhausted:
        if not state['initial_complete']:
            state['run_closed'] = True
            checkpoint()
            raise
        stop_reason = 'learning_time_reserve'
        # Interrupted rollouts stay conservatively charged and are not optimized.
        if state['pending'] and state['pending']['kind'] == 'episode':
            state['pending'] = None
        elif state['pending']:
            # Warmup can stop between critic steps; do not replay the same update.
            state['pending'] = None
        checkpoint()
    except Exception:
        # Close the last coherent generation, not a partially completed update.
        saved = torch.load(run / 'last.pt', map_location='cpu', weights_only=False)
        saved['state']['run_closed'] = True
        saved['state']['elapsed_seconds'] = prior + time.monotonic() - started
        save_checkpoint(run / 'last.pt', saved)
        raise
    try:
        final = evaluate(cfg, run, model, label='final-validation', deadline=deadline)
    except Exception:
        state['run_closed'] = True
        checkpoint()
        raise
    state['final_complete'] = True
    export_actor(model, run / 'policy.pt')
    state['run_closed'] = True
    checkpoint()
    initial = json.loads((run / 'initial-validation.json').read_text())
    comparisons = [dict(case_id=b['case_id'], seed=b['seed'], population=b['population_setting'],
        initial_overrides=a['intervention_fraction'], final_overrides=b['intervention_fraction'],
        override_delta=b['intervention_fraction'] - a['intervention_fraction'],
        speed_ratio_delta=b['mean_fraction_of_speed_limit'] - a['mean_fraction_of_speed_limit'],
        initial_passes=a['passes'], final_passes=b['passes'],
        initial_merges=a['successful_merges'], final_merges=b['successful_merges'])
        for a, b in zip(initial['reports'], final['reports'])]
    atomic_json(run / 'comparison.json', dict(initial=initial['summary'], final=final['summary'], cases=comparisons,
        protocol='Fixed final endpoint compared with identical frozen-start cases; no checkpoint cherry-picking'))
    atomic_json(run / 'training-coverage.json', state['coverage'])
    atomic_json(run / 'policy-contract.json', dict(schema=SCHEMA, observation_size=93, action_size=2,
        model='shared hybrid MAPPO actor; acceleration and masked lane intent', source_hashes=hashes,
        config_sha256=fingerprint, parent=parent, policy_sha256=digest(run / 'policy.pt'),
        checkpoint_sha256=digest(run / 'last.pt'), validation=final['summary'], accepted=False,
        tested_population_counts=[1, 3, 5, 7, 8], no_live_handoff=True, held_out_test_run=False))
    return dict(passed=True, completed=True, stop_reason=stop_reason,
        optimizer_steps=state['optimizer_steps'], critic_optimizer_steps=state['critic_optimizer_steps'],
        counters={k: state[k] for k in ('spent', 'actual', 'critic_actual', 'ppo_actual', 'updates', 'optimizer_steps', 'critic_optimizer_steps', 'elapsed_seconds')},
        initial_validation=initial['summary'], final_validation=final['summary'],
        training_coverage=state['coverage'],
        policy=str((run / 'policy.pt').relative_to(ROOT)), accepted=False, held_out_test_run=False)


def preflight(cfg, run, started):
    deadline = started + cfg['preflight']['max_wall_seconds']
    model = model_for(cfg); warm_start(model, cfg)
    reports = []
    for index, n in enumerate((1, 2, 4, 6, 8)):
        row = training_case(cfg, index)
        row.update(controlled_count=n, total_vehicles=24, seed=6501000 + index)
        for kind in ('teacher', 'random'):
            report, _ = episode(env_for(cfg, run / kind / str(n), row, horizon=100), row,
                                model=model, kind=kind, deadline=deadline)
            reports.append(report)
    row = training_case(cfg, 0); row.update(seed=6501100, controlled_count=8, total_vehicles=24)
    one, batch = episode(env_for(cfg, run / 'collector', row, horizon=30), row,
                         model=model, collect=True, limit=128, deadline=deadline)
    if batch is None or not all(torch.isfinite(v).all() for v in batch.values()):
        raise ValueError('Nonfinite collector')
    replay, _ = episode(env_for(cfg, run / 'repeat-a', row, horizon=100), row, kind='teacher', deadline=deadline)
    repeat, _ = episode(env_for(cfg, run / 'repeat-b', row, horizon=100), row, kind='teacher', deadline=deadline)
    comparable = lambda r: {k: v for k, v in r.items() if k != 'episode_directory'}
    if comparable(replay) != comparable(repeat):
        raise ValueError('Deterministic seed replay failed')
    from pettingzoo.test import parallel_api_test, parallel_seed_test
    def factory():
        check_deadline(deadline)
        return env_for(cfg, run / 'api', row, horizon=100)
    env = factory()
    try:
        parallel_api_test(env, num_cycles=30)
    finally:
        env.close()
    parallel_seed_test(factory, num_cycles=30)
    row = dict(row, controlled_count=0, communication='none')
    native, _ = episode(env_for(cfg, run / 'native-zero', row, horizon=100), row, kind='native', deadline=deadline)
    export_actor(model, run / 'frozen-parent-contract.pt')
    probe = torch.jit.load(str(run / 'frozen-parent-contract.pt'))
    if not torch.equal(probe(batch['local_obs'][0]), model.actor(batch['local_obs'][0])):
        raise ValueError('Actor export mismatch')
    atomic_json(run / 'reports.json', reports)
    return dict(passed=True, optimizer_steps=0, learning=False, reports=reports, zero_av_native=native,
        api_passed=True, seed_test_passed=True, teacher_repeat_exact=True, collector_finite=True,
        actor_export_passed=True, nominal_driving_acceptance=False,
        scope='100-step plumbing checks only; full 800-step initial/final validation runs in the approved training budget')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT)
    parser.add_argument('--mode', choices=('config', 'preflight', 'train', 'evaluate'), default='config')
    parser.add_argument('--approved-config-sha256')
    parser.add_argument('--resume', type=Path)
    parser.add_argument('--checkpoint', type=Path)
    args = parser.parse_args(argv)
    if Path.cwd().resolve() != ROOT:
        raise ValueError('Run from canonical project root')
    cfg = load_config(args.config)
    fingerprint = config_digest(cfg)
    if args.mode == 'config':
        print(json.dumps(dict(config=cfg, sha256=fingerprint), indent=2)); return 0
    if args.mode == 'train':
        if not cfg['training_authorized'] or fingerprint != args.approved_config_sha256:
            raise ValueError('Training requires explicit authorization and reviewed configuration checksum')
        readiness_check(cfg)
    if args.resume and (args.mode != 'train' or not args.resume.resolve().is_relative_to(ROOT / 'outputs')):
        raise ValueError('Resume is restricted to project-local training checkpoints')
    if args.checkpoint and not args.checkpoint.resolve().is_relative_to(ROOT / 'outputs'):
        raise ValueError('Evaluation checkpoint must remain in project outputs')
    torch.set_num_threads(1); torch.manual_seed(cfg['training']['seed'])
    np.random.seed(cfg['training']['seed']); random.seed(cfg['training']['seed'])
    torch.use_deterministic_algorithms(True)
    run = ROOT / 'outputs/highway-population/v01' / (args.mode + '-' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:8])
    run.mkdir(parents=True)
    atomic_json(run / 'config.json', cfg)
    hashes = sources(); started = time.monotonic()
    import traci
    from traffic.sumo_backend import find_sumo_binary
    try:
        sumo_binary = find_sumo_binary()
        sumo_version = subprocess.check_output([str(sumo_binary), '--version'], text=True).splitlines()[0]
    except Exception as exc:
        sumo_version = 'unavailable: ' + repr(exc)
    atomic_json(run / 'provenance.json', dict(config_sha256=fingerprint, source_hashes=hashes,
        git_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        git_status=subprocess.check_output(['git', 'status', '--short'], cwd=ROOT, text=True),
        python=platform.python_version(), platform=platform.platform(), torch=torch.__version__, traci=traci.__version__,
        sumo=sumo_version, network_hashes={r['network']: r['network_sha256'] for r in cfg['training_cases'] + [v['case'] for v in cfg['validation_cases']]},
        command=list(argv) if argv is not None else sys.argv[1:], resume=str(args.resume) if args.resume else None))
    (run / 'working-tree.diff').write_bytes(subprocess.check_output(['git', 'diff', 'HEAD', '--binary'], cwd=ROOT))
    for name, expected in hashes.items():
        destination = run / 'source' / name; destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / name, destination)
        if digest(destination) != expected:
            raise ValueError('Runtime changed while snapshotting')
    result = dict(passed=False, optimizer_steps=0, training=args.mode == 'train', held_out_test_run=False,
                  source_hashes=hashes, preparation_sha256=preparation_digest(cfg))
    try:
        if args.mode == 'preflight':
            result.update(preflight(cfg, run, started))
        elif args.mode == 'train':
            result.update(train(cfg, run, started, hashes, args.resume))
        else:
            if args.checkpoint is None:
                raise ValueError('Evaluation requires a frozen checkpoint')
            old = torch.load(args.checkpoint, map_location='cpu', weights_only=False)
            if old['schema'] != SCHEMA or preparation_digest(old['config']) != preparation_digest(cfg):
                raise ValueError('Evaluation checkpoint/configuration mismatch')
            model = model_for(cfg); model.load_state_dict(old['model']); model.eval()
            value = evaluate(cfg, run, model, label='validation', deadline=started + 600)
            result.update(passed=True, evaluation=value['summary'])
    except Exception as exc:
        result.update(error=repr(exc), traceback=traceback.format_exc())
    finally:
        result['sources_changed'] = [name for name, expected in hashes.items() if digest(ROOT / name) != expected]
        if result['sources_changed']:
            result['passed'] = False
        result['wall_seconds'] = time.monotonic() - started
        atomic_json(run / 'summary.json', result)
    print(json.dumps(dict(run=str(run), passed=result['passed'], error=result.get('error'),
                         counters=result.get('counters'), final_validation=result.get('final_validation')), indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
