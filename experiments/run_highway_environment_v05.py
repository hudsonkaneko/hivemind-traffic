"""Bounded v05 maneuver-environment MAPPO pilot; explicit reviewed launch only."""
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
from experiments import near_limit_support
from experiments import highway_v05_support as support
from experiments import highway_v05_curriculum as curriculum
from policies.mappo import MappoActorCritic, MappoTrainer, compute_gae, export_actor

ROOT = Path(__file__).resolve().parents[1]
DEFAULT = ROOT / 'experiments/configs/highway_environment_v05.json'
SCHEMA = support.SCHEMA
AGENTS = tuple(f'agent_{i}' for i in range(24))
SOURCES = tuple(dict.fromkeys((*OLD_SOURCES, 'experiments/run_highway_population.py', 'experiments/run_highway_environment_v05.py',
    'experiments/highway_v05_support.py', 'experiments/highway_v05_curriculum.py',
    'environments/highway_environment_v05.py', 'traffic/highway_v05_demand.py', 'traffic/highway_v05_metrics.py',
    'experiments/prepare_highway_environment_v05.py', 'scripts/build_highway_environment_v05_bank.py',
    'scripts/build_highway_curriculum_v2.py',
    'environments/highway_population_env.py', 'traffic/sumo_warnings.py',
    'policies/mappo_diagnostics.py', 'experiments/near_limit_support.py',
    'experiments/diagnose_near_limit.py', 'traffic/mappo_physical_policy.py')))


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
    support.validate_config(cfg)


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
        raise ValueError('Unexpected parent schema')
    migration = support.migrate_actor(model, saved)
    return dict(**spec, migration=migration, parent_config_sha256=saved['config_sha256'],
                semantic_transfer=True, observation_semantics=cfg['contract']['observation_semantics'])


def training_case(cfg, state):
    return curriculum.next_case(cfg, state)


def env_for(cfg, output, row, *, horizon=None):
    from environments.highway_environment_v05 import HighwayEnvironmentV05Env
    case = row['case']
    path = ROOT / case['network']
    if digest(path) != case['network_sha256']:
        raise ValueError('Frozen road checksum changed')
    env = HighwayEnvironmentV05Env(path, output, controlled_count=row['controlled_count'],
        total_vehicles=row['total_vehicles'], horizon=horizon or cfg['environment']['horizon'],
        communication=row['communication'], packet_loss=row['packet_loss'],
        through_lanes=case['through_lanes'], task=case['task'], density=case['density'],
        reward_weights=cfg['reward_weights'], control_target_version='v3',
        speed_limit_m_s=row['speed_limit_m_s'], difficulty=row.get('difficulty', 0),
        traffic_mode=row.get('traffic_mode', 'initial'),
        lane_change_duration=cfg['environment']['lane_change_duration'],
        stabilization_seconds=cfg['environment']['stabilization_seconds'])
    env.case_id = case['id']
    env.near_limit_metrics_enabled = True
    return env


def episode(env, row, *, model=None, kind='learned', collect=False, limit=None, deadline=None):
    from traffic.highway_stage1_teacher import teacher_actions
    samples, transitions, reward_sum = [], 0, 0.
    report = None
    near_limit_metrics = (near_limit_support.RawNearLimitMetrics()
        if kind != 'native' and getattr(env, 'near_limit_metrics_enabled', False) else None)
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
            indices = torch.arange(24)
            local = torch.tensor(np.stack([obs.get(a, np.zeros(99, np.float32)) for a in AGENTS]))
            state = torch.as_tensor(env.state()).repeat(24, 1)
            labels = env.teacher_actions({a: obs[a] for a in env.agents})
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
            if near_limit_metrics is not None:
                for agent in env.agents:
                    near_limit_metrics.add(obs[agent], chosen[agent], labels[agent])
            next_obs, rewards, term, trunc, _ = env.step(chosen)
            if not all(math.isfinite(float(v)) for v in rewards.values()):
                raise ValueError('Nonfinite reward')
            transitions += len(rewards)
            reward_sum += sum(rewards.values())
            if collect:
                final = np.concatenate([np.r_[next_obs.get(a, np.zeros(99)), float(not term.get(a, True))]
                                        for a in AGENTS]).astype(np.float32)
                with torch.no_grad():
                    next_values = model.values(torch.tensor(final).repeat(24, 1), indices)
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
            population_setting={k: row[k] for k in ('controlled_count', 'total_vehicles', 'communication', 'packet_loss')},
            difficulty=row.get('difficulty',0), traffic_mode=row.get('traffic_mode','initial'),
            control_target_version=getattr(env, 'control_target_version', 'v2'),
            native_warning_accounting_required=getattr(env, 'control_target_version', 'v2') == 'v3')
        if near_limit_metrics is not None:
            report['raw_near_limit'] = near_limit_metrics.report()
            report['requested_episode_speed_ceiling_m_s'] = row.get('speed_limit_m_s')
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
        if hasattr(env, 'directory'):
            from traffic.sumo_warnings import native_emergency_report
            native_events = native_emergency_report(env.directory / 'sumo-errors.log')
            if report is not None:
                report['native_emergencies'] = native_events
            interrupted_path = env.directory / 'interrupted-accounting.json'
            if interrupted_path.exists():
                interrupted = json.loads(interrupted_path.read_text(encoding='utf-8'))
                interrupted['native_emergencies'] = native_events
                atomic_json(interrupted_path, interrupted)


def summarize(reports):
    return support.acceptance(reports)


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
    states = batch['global_states'].reshape(-1, 2400)[active]
    indices = batch['agent_indices'].reshape(-1)[active]
    returns = batch['returns'].reshape(-1)[active].detach()
    count, losses = 0, []
    diagnostic = getattr(trainer, 'gradient_diagnostics', False)
    diagnostic_steps = []
    if diagnostic:
        from policies.mappo_diagnostics import critic_fit_metrics, current_gradient_norms
        fit_before = critic_fit_metrics(model, states, indices, returns)
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
            if diagnostic:
                before_clip = current_gradient_norms(model)
            torch.nn.utils.clip_grad_norm_(model.critic.parameters(), trainer.max_grad_norm, error_if_nonfinite=True)
            if diagnostic:
                after_clip = current_gradient_norms(model)
                diagnostic_steps.append(dict(before=before_clip, after=after_clip,
                    critic_clip_factor=min(1., trainer.max_grad_norm / (before_clip['critic'] + 1e-6))))
            trainer.optimizer.step()
            count += 1
            losses.append(float(loss.detach()))
        if exhausted:
            break
    if actor_digest(model) != frozen:
        raise ValueError('Critic warmup changed the actor or exploration')
    result = dict(optimizer_steps=count, value_loss=sum(losses) / max(1, len(losses)), actor_frozen=True,
                actor_sha256=frozen, budget_exhausted=exhausted)
    if diagnostic:
        result['gradient_diagnostics'] = dict(phase='critic_warmup',
            clipping='critic only; actor has no gradient',
            loss_definition='0.5 * mean squared value error; PPO value_coef is not applied during warmup',
            actual_step_count=count,
            critic_fit_before=fit_before,
            critic_fit_after=critic_fit_metrics(model, states, indices, returns),
            actual_gradient_means={stage: {group: sum(s[stage][group] for s in diagnostic_steps) / count
                if count else None for group in ('actor', 'critic', 'joint')} for stage in ('before', 'after')},
            mean_critic_clip_factor=sum(s['critic_clip_factor'] for s in diagnostic_steps) / count if count else None,
            clipped_step_fraction=sum(s['critic_clip_factor'] < 1 for s in diagnostic_steps) / count if count else None,
            target_description='Frozen on-policy rollout GAE returns; identical targets before/after this warmup update')
    return result


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
        run_closed=False, pending=None, initial_complete=False, final_complete=False, coverage={}, curriculum={}, critic_warmup_closed=False)
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
        if not pending['critic_only'] and stats['optimizer_steps'] > 0:
            curriculum.observe(state['curriculum'], dict(pending['report'], curriculum_training=True), pending['row'])
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
            row = training_case(cfg, state['curriculum'])
            remaining = cfg['training']['max_agent_transitions'] - state['spent']
            critic_only = not state['critic_warmup_closed'] and state['critic_actual'] < cfg['training']['critic_warmup_transitions']
            if critic_only:
                remaining = min(remaining, cfg['training']['critic_warmup_transitions'] - state['critic_actual'])
                # Never rewrite an issued population/hash to fill a tiny remainder.
                if remaining < row['controlled_count']:
                    state['critic_warmup_closed'] = True
                    critic_only = False
                    remaining = cfg['training']['max_agent_transitions'] - state['spent']
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
            if 'speed_limit_m_s' in row:
                key += '|speed_ceiling=' + str(row['speed_limit_m_s'])
            coverage = state['coverage'].setdefault(key, dict(episodes=0, actual_agent_transitions=0))
            coverage['episodes'] += 1; coverage['actual_agent_transitions'] += actual
            if batch is None:
                raise ValueError('Training episode yielded no active transitions')
            state['pending'] = dict(kind='update', batch=batch, report=report, critic_only=critic_only, row=row)
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
    for comparison, before, after in zip(comparisons, initial['reports'], final['reports']):
        comparison.update(initial_raw_near_limit=before['raw_near_limit'],
                          final_raw_near_limit=after['raw_near_limit'],
                          requested_episode_speed_ceiling_m_s=after['requested_episode_speed_ceiling_m_s'])
    atomic_json(run / 'comparison.json', dict(initial=initial['summary'], final=final['summary'], cases=comparisons,
        protocol='Fixed final endpoint compared with identical frozen-start cases; no checkpoint cherry-picking'))
    atomic_json(run / 'training-coverage.json', state['coverage'])
    atomic_json(run / 'curriculum.json', state['curriculum'])
    atomic_json(run / 'policy-contract.json', dict(schema=SCHEMA, observation_size=99, action_size=2,
        model='shared hybrid MAPPO actor; acceleration and masked lane intent', source_hashes=hashes,
        config_sha256=fingerprint, parent=parent, policy_sha256=digest(run / 'policy.pt'),
        checkpoint_sha256=digest(run / 'last.pt'), validation=final['summary'], accepted=False,
        tested_population_counts=sorted({r['controlled_count'] for r in cfg['validation_cases']}), no_live_handoff=True, held_out_test_run=False,
        control_target_version=cfg['environment'].get('control_target_version', 'v2'),
        observation_semantics=cfg.get('contract', {}).get('observation_semantics', 'legacy_v2'),
        warm_start_semantic_transfer=cfg['environment'].get('control_target_version', 'v2') == 'v3',
        objective_version=cfg.get('objective_version', 'legacy'),
        speed_curriculum=cfg.get('speed_curriculum')))
    return dict(passed=True, completed=True, stop_reason=stop_reason,
        optimizer_steps=state['optimizer_steps'], critic_optimizer_steps=state['critic_optimizer_steps'],
        counters={k: state[k] for k in ('spent', 'actual', 'critic_actual', 'ppo_actual', 'updates', 'optimizer_steps', 'critic_optimizer_steps', 'elapsed_seconds')},
        initial_validation=initial['summary'], final_validation=final['summary'],
        training_coverage=state['coverage'],
        policy=str((run / 'policy.pt').relative_to(ROOT)), accepted=False, held_out_test_run=False)


def preflight(cfg, run, started):
    deadline = started + cfg['preflight']['max_wall_seconds']
    model = model_for(cfg); warm_start(model, cfg)
    frozen = {name:value.detach().clone() for name,value in model.state_dict().items()}
    def row_for(task, count=4, total=16, speed=18., difficulty=0, mode='initial', seed=6802000):
        case = next(c for c in cfg['training_cases'] if c['task']==task and c['through_lanes']==3)
        return dict(case=case, controlled_count=count,total_vehicles=total,communication='local',packet_loss=0.,
                    speed_limit_m_s=speed,difficulty=difficulty,traffic_mode=mode,seed=seed)
    reports=[]
    for i,task in enumerate(('cruise','following','passing','merge')):
        row=row_for(task,seed=6802000+i)
        report,_=episode(env_for(cfg,run/'teacher'/task,row),row,kind='teacher',deadline=deadline)
        reports.append(report)
    i110 = row_for('merge',seed=6802500)
    i110['case'] = next(c for c in cfg['training_cases'] if c['id']=='i110-merge')
    report,_ = episode(env_for(cfg,run/'teacher'/'i110-merge',i110),i110,kind='teacher',deadline=deadline)
    reports.append(report)
    atomic_json(run/'teacher-baselines.json',reports)
    baseline=summarize(reports)
    # Readiness is distinct from policy acceptance. Refuse learning if nominal
    # scripted cases cannot complete or have native safety events.
    fatal=[x for x in baseline['failures'] if any(c in x['reasons'] for c in
           ('safety_event','native_emergency_or_missing_accounting','incomplete_episode','stable_merge','passing'))]
    if fatal:
        raise ValueError('Nominal teacher baseline failed: '+json.dumps(fatal))
    for i,count in enumerate((1,12,24)):
        row=row_for('merge',count=count,total=24,seed=6802100+i)
        report,_=episode(env_for(cfg,run/'population'/str(count),row,horizon=150),row,
                          kind='teacher',deadline=deadline)
        reports.append(report)
    row=row_for('cruise',count=8,total=16,speed=12.,mode='sustained',seed=6802200)
    # Long enough to exercise delayed departures, rather than just XML authoring.
    inflow,_=episode(env_for(cfg,run/'inflow',row,horizon=300),row,kind='teacher',deadline=deadline)
    reports.append(inflow)
    row=row_for('merge',count=4,seed=6802300)
    one,batch=episode(env_for(cfg,run/'collector',row,horizon=80),row,model=model,
                      collect=True,limit=256,deadline=deadline)
    if batch is None or not all(torch.isfinite(v).all() for v in batch.values()):
        raise ValueError('Nonfinite collector')
    trainer=MappoTrainer(model,**cfg['optimizer'])
    diagnostic=trainer.diagnose_batch(batch,phase='frozen_preflight_before_critic_warmup')
    atomic_json(run/'frozen-gradient-diagnostics.json',diagnostic)
    if trainer.optimizer.state:
        raise ValueError('Preflight initialized optimizer state')
    repeat_a,_=episode(env_for(cfg,run/'repeat-a',row,horizon=120),row,kind='teacher',deadline=deadline)
    repeat_b,_=episode(env_for(cfg,run/'repeat-b',row,horizon=120),row,kind='teacher',deadline=deadline)
    comparable=lambda r:{k:v for k,v in r.items() if k!='episode_directory'}
    if comparable(repeat_a)!=comparable(repeat_b):
        raise ValueError('Deterministic repeat mismatch')
    from pettingzoo.test import parallel_api_test,parallel_seed_test
    def factory():
        check_deadline(deadline)
        return env_for(cfg,run/'api',row,horizon=80)
    env=factory()
    try: parallel_api_test(env,num_cycles=25)
    finally: env.close()
    parallel_seed_test(factory,num_cycles=25)
    native_row=row_for('cruise',count=0,total=24,seed=6802400)
    native,_=episode(env_for(cfg,run/'native',native_row,horizon=100),native_row,kind='native',deadline=deadline)
    export_actor(model,run/'frozen-initialization.pt')
    exported=torch.jit.load(str(run/'frozen-initialization.pt'))
    if not torch.equal(exported(batch['local_obs'][0]),model.actor(batch['local_obs'][0])):
        raise ValueError('Export mismatch')
    if any(not torch.equal(frozen[n],v) for n,v in model.state_dict().items()):
        raise ValueError('Preflight changed weights')
    atomic_json(run/'reports.json',reports)
    return dict(passed=True,optimizer_steps=0,learning=False,teacher_baseline=baseline,
        reports=reports,zero_av_native=native,api_passed=True,seed_test_passed=True,
        teacher_repeat_exact=True,collector_finite=True,actor_export_passed=True,
        frozen_parameters_unchanged=True,nominal_driving_acceptance=False,
        scope='Nominal scripted tasks plus bounded population/inflow/API/export probes; no learned-policy acceptance')


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
    run = ROOT / cfg.get('output_directory', 'outputs/highway-population/v01') / (args.mode + '-' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:8])
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
    (run / 'working-tree.diff').write_bytes(subprocess.check_output(['git', 'diff', 'HEAD', '--binary', '--', *SOURCES, str(args.config)], cwd=ROOT))
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
