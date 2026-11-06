"""Separately gated 117-field longitudinal pilot; configuration never authorizes learning."""
from __future__ import annotations
import argparse
from contextlib import contextmanager
from copy import deepcopy
from datetime import datetime,timezone
import json
import math
from pathlib import Path
import random
import shutil
import subprocess
import sys
import time
import traceback
import uuid

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0,str(ROOT))
import numpy as np
import torch
from experiments import run_highway_environment_v05 as old
from experiments import highway_v05_support
from policies.mappo import compute_gae,export_actor
from policies.highway_flow import FlowModel,FlowTrainer,migrate_actor_99,critic_update
from traffic.highway_flow_contract import (ARMS,SCHEMA,INITIALIZERS,PARENT_CONFIG,PARENT_CONFIG_SHA,
    frozen_base,reference_parameters,comparison_settings,compare_endpoints,rank_selection,
    complete_episode,elapsed_on_resume,file_digest,new_arm_state,preparation_digest,project_path,
    recover_pending,require_training,reserve_episode,validate_config)
from traffic.highway_flow_schedule import build_schedule,coverage_report,add_coverage,digest,next_case

DEFAULT=ROOT/'configs/training/highway-flow-v01.json'
AGENTS=old.AGENTS
SOURCES=tuple(dict.fromkeys((*old.SOURCES,'experiments/run_highway_flow.py','policies/highway_flow.py',
    'traffic/highway_flow_contract.py','traffic/highway_flow_schedule.py','traffic/highway_flow_context.py',
    'traffic/highway_flow_reference.py','environments/highway_flow_env.py','traffic/highway_smooth_contract.py',
    'traffic/highway_smooth_schedule.py','traffic/highway_smooth_metrics.py','scripts/build_highway_flow_bank.py',
    'scripts/prepare_highway_flow.py','scripts/record_highway_environment_v05.py',
    'traffic/highway_smooth_reference.py','scripts/check_highway_flow_gradients.py')))


def sources():
    return {name:file_digest(ROOT/name) for name in SOURCES}


def build_config(network_manifest='outputs/highway-flow/v01/bank/bank.json'):
    from traffic.highway_flow_reference import REFERENCE_VERSION
    bank=None
    if network_manifest is not None:
        path=project_path(network_manifest,outputs=True)
        manifest=json.loads(path.read_text(encoding='utf-8'))
        bank=dict(manifest_path=path.relative_to(ROOT).as_posix(),manifest_sha256=file_digest(path),
            network_overrides=manifest.get('network_overrides',{}))
    base=frozen_base(bank['network_overrides'] if bank else None)
    cfg=dict(schema=SCHEMA,training_authorized=False,readiness=None,
        status='ready_for_bounded_testing_not_training',
        output_directory='outputs/highway-flow/v01',base_config=base,network_bank=bank,
        initialization=dict(seed=9507,candidates=deepcopy(INITIALIZERS),selected_id=None,
            migration='93â†’99â†’117 or99â†’117; new local input columns zero, messages preserved',
            critic='Fresh identical critics/optimizers; actor and log_std only imported',
            sampling='Separate identical initial RNG streams and immutable matched case order; fresh PPO only'),
        arms=[dict(id=ARMS[0],imitation_acceleration_weight=1.),dict(id=ARMS[1],imitation_acceleration_weight=5.)],
        optimizer=deepcopy(base['optimizer']),gae=dict(gamma=.99,gae_lambda=.95),
        reference_version=REFERENCE_VERSION,reference_parameters=reference_parameters(),
        reward_version='highway_flow_legacy_weights_new_reference_v1',comparison=comparison_settings(),
        budget=dict(per_arm_decisions=30000,total_decisions=60000,wall_seconds=5400,
            critic_warmup_per_arm=2048,preparation_wall_seconds=1800,final_evaluation_reserve_seconds=900),
        schedule=dict(seed=9507,seed_start=7100000,seed_end=7100999,tasks=['cruise','following'],
            planned_case_shares=dict(cruise=.5,following=.5),
            first_ppo_block_controlled_agents=1,first_ppo_block_cases=18,warmup_cursor='independent; PPO restarts the full coverage block',
            definition='First18 PPO cases use1AV each: at most14400 active decisions plus2048 warmup. Later blocks vary participation. Every longitudinal geometry once per block; difficulty cycles0/1/2; independent total population/speed/channel cycles. Case shares are not decision quotas.'),
        evaluation=dict(shared_initial_cases=24,final_cases_per_arm=24,initializer_cases_per_candidate=24,held_out=False),
        preparation_ledger='outputs/highway-flow/v01/preparation-ledger.json',
        provenance=dict(parent_config=PARENT_CONFIG,parent_config_sha256=PARENT_CONFIG_SHA,
            plan='documentation/highway-training-rethink-v01.md'))
    cfg['schedule_sha256']=digest(build_schedule(cfg))
    validate_config(cfg)
    return cfg


def load_config(path=DEFAULT):
    cfg=json.loads(project_path(path).read_text(encoding='utf-8'))
    validate_config(cfg)
    return cfg


def env_for(cfg,output,row,arm_id=ARMS[0]):
    from environments.highway_flow_env import HighwayFlowEnv
    if arm_id not in ARMS:
        raise ValueError('Unknown flow arm')
    case,base=row['case'],cfg['base_config']
    path=project_path(case['network'])
    if file_digest(path)!=case['network_sha256']:
        raise ValueError('Frozen road checksum changed')
    fixture=row.get('fixture')
    if isinstance(fixture,dict) and 'path' in fixture:
        fixture_path=project_path(fixture['path'])
        if file_digest(fixture_path)!=fixture['sha256']:
            raise ValueError('Frozen fixture changed')
        fixture=json.loads(fixture_path.read_text(encoding='utf-8'))
    env=HighwayFlowEnv(path,output,fixture=fixture,controlled_count=row['controlled_count'],
        total_vehicles=row['total_vehicles'],horizon=base['environment']['horizon'],
        communication=row['communication'],packet_loss=row['packet_loss'],through_lanes=case['through_lanes'],
        task=case['task'],density=case['density'],reward_weights=base['reward_weights'],control_target_version='v3',
        speed_limit_m_s=row['speed_limit_m_s'],difficulty=row.get('difficulty',0),
        traffic_mode=row.get('traffic_mode','initial'),lane_change_duration=3.,stabilization_seconds=8.)
    env.case_id=case['id']
    return env


def initialize_candidate(cfg,initializer_id):
    validate_config(cfg)
    if initializer_id not in INITIALIZERS:
        raise ValueError('Explicit frozen initializer ID required')
    spec=cfg['initialization']['candidates'][initializer_id]
    path=project_path(spec['path'])
    if file_digest(path)!=spec['sha256']:
        raise ValueError('Frozen initializer checksum mismatch')
    saved=torch.load(path,map_location='cpu',weights_only=False)
    outer=capture_rng()
    try:
        seed=cfg['initialization']['seed']
        torch.manual_seed(seed); np.random.seed(seed); random.seed(seed)
        parent_cfg=json.loads((ROOT/PARENT_CONFIG).read_text(encoding='utf-8'))
        parent=old.model_for(parent_cfg)
        if spec['observation_size']==93:
            highway_v05_support.migrate_actor(parent,saved)
        else:
            parent.actor.load_state_dict({k[6:]:v for k,v in saved['model'].items() if k.startswith('actor.')},strict=True)
            with torch.no_grad():
                parent.log_std.copy_(saved['model']['log_std'])
        model=FlowModel()
        migration=migrate_actor_99(model,parent)
        probe=torch.zeros(24,99); probe[:,30:33]=1.
        migrated=torch.cat((probe[:,:51],torch.ones(24,18)*.3,probe[:,51:]),dim=-1)
        with torch.no_grad():
            expected=parent.act(probe,deterministic=True)[0]
            actual=model.act(migrated,deterministic=True)[0]
        if not torch.isfinite(actual).all() or not torch.allclose(actual,expected,atol=1e-7,rtol=0.):
            raise ValueError('Actor migration changed prior predictions')
        if spec.get('actor_path'):
            artifact=project_path(spec['actor_path'])
            if file_digest(artifact)!=spec['actor_sha256']:
                raise ValueError('Frozen actor export checksum mismatch')
            frozen=torch.jit.load(str(artifact),map_location='cpu').eval()
            with torch.no_grad():
                if not torch.equal(expected,frozen(probe)):
                    raise ValueError('Checkpoint differs from frozen actor export')
        model.eval()
        return dict(model=model,actor=model.actor,rng=capture_rng(),provenance=dict(initializer_id=initializer_id,
            checkpoint=deepcopy(spec),migration=migration,actor_sha256=old.actor_digest(model)))
    finally:
        restore_rng(outer)


def initialize_candidates(cfg):
    return {name:initialize_candidate(cfg,name) for name in INITIALIZERS}


def initialize_model(cfg,initializer_id):
    return initialize_candidate(cfg,initializer_id)['model']


def initialize_models(cfg):
    return {name:value['model'] for name,value in initialize_candidates(cfg).items()}


def initialize_arms(cfg,initializer_id=None):
    selected=initializer_id or cfg['initialization']['selected_id']
    if selected is None:
        raise ValueError('Initializer selection is unresolved; explicit frozen candidate required')
    result={}
    for spec in cfg['arms']:
        candidate=initialize_candidate(cfg,selected)
        trainer=FlowTrainer(candidate['model'],**dict(cfg['optimizer'],
            imitation_acceleration_weight=spec['imitation_acceleration_weight']))
        result[spec['id']]=dict(model=candidate['model'],trainer=trainer,rng=candidate['rng'],provenance=candidate['provenance'])
    return result


def capture_rng():
    return dict(torch=torch.get_rng_state().clone(), numpy=deepcopy(np.random.get_state()), python=random.getstate())


def restore_rng(rng):
    torch.set_rng_state(rng['torch']); np.random.set_state(rng['numpy']); random.setstate(rng['python'])


@contextmanager
def arm_rng(arm):
    outer = capture_rng()
    restore_rng(arm['rng'])
    try:
        yield
    finally:
        arm['rng'] = capture_rng()
        restore_rng(outer)


def tree_equal(a, b):
    if isinstance(a, torch.Tensor):
        return isinstance(b, torch.Tensor) and torch.equal(a, b)
    if isinstance(a, np.ndarray):
        return isinstance(b, np.ndarray) and np.array_equal(a, b)
    if isinstance(a, dict):
        return isinstance(b, dict) and a.keys() == b.keys() and all(tree_equal(a[k], b[k]) for k in a)
    if isinstance(a, (list, tuple)):
        return type(a) is type(b) and len(a) == len(b) and all(tree_equal(x, y) for x, y in zip(a, b))
    return a == b




def frozen_loss_check(cfg, batches_by_arm, arms=None):
    """Evaluate real loss/gradient scales without stepping or changing state."""
    validate_config(cfg)
    if set(batches_by_arm) != set(ARMS):
        raise ValueError('One frozen on-policy training batch per arm is required')
    arms = initialize_arms(cfg) if arms is None else arms
    results = {}
    for name in ARMS:
        arm = arms[name]
        model, trainer = arm['model'], arm['trainer']
        before = deepcopy(dict(model=model.state_dict(), optimizer=trainer.optimizer.state_dict(),
            gradients=[parameter.grad for parameter in model.parameters()], rng=capture_rng(), arm_rng=arm['rng']))
        result = trainer.diagnose_batch(batches_by_arm[name], phase='frozen_no_update')
        after = dict(model=model.state_dict(), optimizer=trainer.optimizer.state_dict(),
            gradients=[parameter.grad for parameter in model.parameters()], rng=capture_rng(), arm_rng=arm['rng'])
        if not tree_equal(before, after) or result.get('optimizer_steps') != 0:
            raise ValueError('Frozen diagnostics changed model, optimizer, gradients or RNG')
        json.dumps(result, allow_nan=False)
        results[name] = result
    return dict(passed=True, training=False, optimizer_steps=0, state_unchanged=True, arms=results)


def _serialized_arms(arms):
    return {name: dict(model=arm['model'].state_dict(), optimizer=arm['trainer'].optimizer.state_dict(), rng=arm['rng'])
            for name, arm in arms.items()}


def _load_arms(arms, saved):
    for name, arm in arms.items():
        arm['model'].load_state_dict(saved[name]['model'])
        arm['trainer'].optimizer.load_state_dict(saved[name]['optimizer'])
        arm['rng'] = saved[name]['rng']


def _recover_interrupted(state, elapsed):
    for arm in state['arms'].values():
        pending = arm.get('pending')
        if pending and pending['kind'] in ('episode', 'updating'):
            arm['learning_seconds'] += max(0., elapsed - pending['stage_start_elapsed'])
            if pending['kind'] == 'episode':
                matches = list(project_path(pending['output'], outputs=True).rglob('interrupted-accounting.json'))
                if len(matches) == 1:
                    record = json.loads(matches[0].read_text(encoding='utf-8'))
                    actual = record.get('actual_agent_transitions')
                    if type(actual) is int and 0 <= actual <= pending['reserved']:
                        arm['actual'] += actual
                        arm['critic_actual' if pending['critic_only'] else 'ppo_actual'] += actual
                        pending['observed_failed_decisions'] = actual
            recover_pending(arm)


def _claim_run(cfg, run, resume, fingerprint):
    """One paired study, one durable resume chain; fresh retries cannot reset costs."""
    directory = project_path(cfg['output_directory'], outputs=True)
    directory.mkdir(parents=True, exist_ok=True)
    ledger_path, lock_path = directory/'training-launch.json', directory/'training-launch.lock'
    # Exclusive creation protects simultaneous launches. A stale lock requires
    # explicit review, never an automatic fresh launch after a process loss.
    with lock_path.open('x', encoding='utf-8') as lock:
        lock.write(str(run))
    try:
        previous = json.loads(ledger_path.read_text(encoding='utf-8')) if ledger_path.exists() else None
        if previous is not None:
            if (resume is None or previous.get('config_sha256') != fingerprint
                    or project_path(previous['latest_checkpoint'], outputs=True) != project_path(resume, outputs=True)):
                raise ValueError('Study already launched; resume its latest charged checkpoint instead of resetting the budget')
        elif resume is not None:
            raise ValueError('Resume lacks the study launch ledger')
        segments = [] if previous is None else previous['segments']
        segments.append(dict(output=run.relative_to(ROOT).as_posix(),
            resumed_from=project_path(resume).relative_to(ROOT).as_posix() if resume else None))
        old.atomic_json(ledger_path, dict(schema='highway_flow_launch_chain_v01',config_sha256=fingerprint,
            latest_checkpoint=(run/'last.pt').relative_to(ROOT).as_posix(),segments=segments))
    finally:
        lock_path.unlink()




def episode(env, row, *, model=None, kind='learned', collect=False, limit=None, deadline=None):
    from traffic.highway_flow_reference import reference_details, REFERENCE_VERSION
    samples, transitions, reward_sum = [], 0, 0.
    eligibility = dict(all_active_decisions=0, acceleration_eligible_decisions=0, lane_eligible_decisions=0)
    report = None
    rng = np.random.default_rng(row['seed'] + 70000000)
    try:
        old.check_deadline(deadline)
        obs, _ = env.reset(seed=row['seed'])
        env.control_mode = 'native' if kind == 'native' else 'policy'
        while env.agents:
            old.check_deadline(deadline)
            if limit is not None and transitions + len(env.agents) > limit:
                break
            active = torch.tensor([a in env.agents for a in AGENTS])
            indices = torch.arange(24)
            local = torch.tensor(np.stack([obs.get(a, np.zeros(117, np.float32)) for a in AGENTS]))
            state = torch.as_tensor(env.state()).repeat(24, 1)
            details = {a:reference_details(obs[a]) for a in env.agents}
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
            next_obs, rewards, term, trunc, _ = env.step(chosen)
            if not all(math.isfinite(float(v)) for v in rewards.values()):
                raise ValueError('Nonfinite reward')
            transitions += len(rewards)
            reward_sum += sum(rewards.values())
            eligibility['all_active_decisions'] += len(rewards)
            eligibility['acceleration_eligible_decisions'] += sum(bool(details[a]['acceleration_supervision_eligible']) for a in rewards)
            eligibility['lane_eligible_decisions'] += sum(bool(details[a]['lane_supervision_eligible']) for a in rewards)
            if collect:
                final = np.concatenate([np.r_[next_obs.get(a, np.zeros(117)), float(not term.get(a, True))]
                                        for a in AGENTS]).astype(np.float32)
                with torch.no_grad():
                    next_values = model.values(torch.tensor(final).repeat(24, 1), indices)
                samples.append(dict(local_obs=local, global_states=state, agent_indices=indices,
                    actions=actions, pre_tanh_actions=latent, old_log_probs=logp, old_values=values,
                    next_values=next_values, rewards=torch.tensor([rewards.get(a, 0.) for a in AGENTS]),
                    terminated=torch.tensor([term.get(a, False) for a in AGENTS]),
                    truncated=torch.tensor([trunc.get(a, False) for a in AGENTS]), active=active,
                    acceleration_supervision_eligible=torch.tensor([bool(details.get(a,{}).get('acceleration_supervision_eligible',False)) for a in AGENTS]),
                    lane_supervision_eligible=torch.tensor([bool(details.get(a,{}).get('lane_supervision_eligible',False)) for a in AGENTS]),
                    teacher_actions=torch.tensor(np.stack([labels.get(a, np.zeros(2, np.float32)) for a in AGENTS]))))
            obs = {a: next_obs[a] for a in env.agents}
        report = old.primary_accounting(env, partial=bool(env.agents), deadline=deadline)
        report.update(observation_size=117,state_size=2832,seed=row['seed'], controller=kind, case_id=env.case_id, reward=reward_sum,
            agent_transitions=transitions, partial_control_horizon=bool(env.agents),
            episode_directory=str(env.directory.relative_to(ROOT)),
            population_setting={k: row[k] for k in ('controlled_count', 'total_vehicles', 'communication', 'packet_loss')},
            difficulty=row.get('difficulty',0), traffic_mode=row.get('traffic_mode','initial'),
            control_target_version=REFERENCE_VERSION, reference_version=REFERENCE_VERSION,
            native_safety_adapter_version=getattr(env, 'control_target_version', 'v3'),
            native_warning_accounting_required=True, supervision_eligibility=eligibility)
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
                old.atomic_json(env.directory / 'interrupted-accounting.json', dict(interrupted,
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
                old.atomic_json(interrupted_path, interrupted)




def train(cfg, run, *, approved_config_sha256, resume=None):
    """Future authorized launch entry point; both API and CLI enforce readiness."""
    started = time.monotonic()
    hashes = sources()
    proof = require_training(cfg, approved_config_sha256, hashes)
    run = project_path(run, outputs=True)
    run.mkdir(parents=True, exist_ok=False)
    fingerprint, schedule = digest(cfg), build_schedule(cfg)
    prior = proof['charged_preparation_wall_seconds']
    state = dict(elapsed_seconds=prior, run_closed=False, phase='initial', evaluations={},
        arms={name: new_arm_state() for name in ARMS}, turn=0, per_arm_learning_limit_seconds=None)
    saved_arms = None
    if resume:
        path = project_path(resume, outputs=True)
        if path.name != 'last.pt':
            raise ValueError('Resume requires last.pt')
        saved = torch.load(path, map_location='cpu', weights_only=False)
        if saved['schema'] != SCHEMA or saved['config_sha256'] != fingerprint or saved['source_hashes'] != hashes:
            raise ValueError('Resume configuration/source mismatch')
        state = saved['state']
        prior = elapsed_on_resume(state, saved['saved_at_unix'], time.time())
        saved_arms = saved['arms']
        _recover_interrupted(state, prior)
    state['run_closed'] = False
    # Persist the charged launch before source copying or model initialization.
    # A setup failure can therefore resume without resetting preparation costs.
    bootstrap = dict(schema=SCHEMA, config_sha256=fingerprint, source_hashes=hashes,
        state=deepcopy(state), arms=saved_arms, saved_at_unix=time.time())
    bootstrap['state']['elapsed_seconds'] = prior + time.monotonic() - started
    old.save_checkpoint(run / 'last.pt', bootstrap)
    try:
        _claim_run(cfg, run, resume, fingerprint)
        old.atomic_json(run / 'config.json', cfg)
        old.atomic_json(run / 'schedule.json', schedule)
        for name in hashes:
            destination = run / 'source' / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / name, destination)
        old.atomic_json(run / 'provenance.json', dict(source_hashes=hashes, config_sha256=fingerprint,
            preparation=proof, git_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True, encoding='utf-8').strip(),
            scoped_diff=subprocess.check_output(['git', 'diff', '--', *hashes], cwd=ROOT, text=True, encoding='utf-8') if hashes else '',
            resume=str(resume) if resume else None))
        arms = initialize_arms(cfg)
        if saved_arms is not None:
            _load_arms(arms, saved_arms)
    except Exception:
        bootstrap['state'].update(elapsed_seconds=prior + time.monotonic()-started, run_closed=True)
        old.save_checkpoint(run / 'last.pt', bootstrap)
        old.atomic_json(run / 'failure.json', dict(completed=False, phase='setup',
            elapsed_seconds=bootstrap['state']['elapsed_seconds'], traceback=traceback.format_exc(), checkpoint='last.pt'))
        raise
    deadline = started + max(0., cfg['budget']['wall_seconds'] - prior)
    learning_deadline = deadline - cfg['budget']['final_evaluation_reserve_seconds']

    def elapsed():
        return prior + time.monotonic() - started

    def checkpoint():
        state['elapsed_seconds'] = elapsed()
        old.save_checkpoint(run / 'last.pt', dict(schema=SCHEMA, config_sha256=fingerprint,
            source_hashes=hashes, state=deepcopy(state), arms=_serialized_arms(arms), saved_at_unix=time.time()))

    def evaluate(label, arm_name, until):
        records = state['evaluations'].setdefault(label, [])
        rows = cfg['base_config']['validation_cases']
        reports = []
        for index, row in enumerate(rows):
            if index < len(records):
                record = records[index]
                path = project_path(record['path'], outputs=True)
                if file_digest(path) != record['sha256'] or record['row_sha256'] != digest(row):
                    raise ValueError('Saved evaluation row changed')
                report = json.loads(path.read_text(encoding='utf-8'))
            else:
                old.check_deadline(until)
                with arm_rng(arms[arm_name]):
                    report, _ = episode(env_for(cfg, run / label / str(index), row, arm_name), row,
                        model=arms[arm_name]['model'], deadline=until)
                path = run / label / f'{index:02d}.json'
                old.atomic_json(path, report)
                records.append(dict(path=path.relative_to(ROOT).as_posix(), sha256=file_digest(path), row_sha256=digest(row)))
                checkpoint()
            reports.append(report)
        result = dict(summary=old.summarize(reports), reports=reports)
        old.atomic_json(run / (label + '.json'), result)
        return result

    def update_pending(name):
        arm, counter = arms[name], state['arms'][name]
        pending = counter['pending']
        if not pending or pending['kind'] != 'update_ready':
            return
        until = min(learning_deadline, time.monotonic() + max(0., state['per_arm_learning_limit_seconds'] - counter['learning_seconds']))
        if time.monotonic() >= until:
            counter['failures'].append(dict(kind='unused_rollout', reason='No learning time remains'))
            counter['pending'] = None
            checkpoint()
            return
        pending.update(kind='updating', stage_start_elapsed=elapsed())
        checkpoint()
        with arm_rng(arm):
            stats = critic_update(arm['model'], arm['trainer'], pending['batch'], until) if pending['critic_only'] else arm['trainer'].update(pending['batch'], deadline=until)
        counter['learning_seconds'] += elapsed() - pending['stage_start_elapsed']
        counter['critic_optimizer_steps' if pending['critic_only'] else 'optimizer_steps'] += stats['optimizer_steps']
        counter['updates'] += 1
        old.atomic_json(run / name / f'update-{counter["updates"]:05d}.json', dict(
            row=pending['row'], report=pending['report'], optimization=stats, critic_only=pending['critic_only']))
        counter['pending'] = None
        checkpoint()

    checkpoint()
    try:
        if state['phase'] == 'initial':
            evaluate('initial-validation', ARMS[0], learning_deadline)
            state['per_arm_learning_limit_seconds'] = max(0., learning_deadline - time.monotonic()) / 2
            state['phase'] = 'learning'
            checkpoint()
        if state['phase'] == 'learning':
            for name in ARMS:
                update_pending(name)
            while time.monotonic() < learning_deadline:
                eligible = [name for name in ARMS if state['arms'][name]['spent'] < cfg['budget']['per_arm_decisions']
                    and state['arms'][name]['learning_seconds'] < state['per_arm_learning_limit_seconds']
                    and not state['arms'][name].get('exhausted')]
                if not eligible:
                    break
                name = next((ARMS[(state['turn'] + i) % 2] for i in range(2) if ARMS[(state['turn'] + i) % 2] in eligible))
                state['turn'] = (ARMS.index(name) + 1) % 2
                counter, arm = state['arms'][name], arms[name]
                critic = not counter['critic_closed']
                row = next_case(schedule, counter, critic_only=critic)
                if critic and cfg['budget']['critic_warmup_per_arm'] - counter['critic_spent'] < row['controlled_count']:
                    counter['critic_closed'], critic = True, False
                    row = next_case(schedule, counter, critic_only=False)
                reserve = reserve_episode(cfg, counter, row, critic_only=critic)
                if not reserve:
                    counter['exhausted'] = True
                    checkpoint()
                    continue
                output = run / name / 'train' / str(counter['episodes'])
                counter['pending'].update(output=output.relative_to(ROOT).as_posix(), stage_start_elapsed=elapsed())
                checkpoint()
                until = min(learning_deadline, time.monotonic() + max(0., state['per_arm_learning_limit_seconds'] - counter['learning_seconds']))
                try:
                    with arm_rng(arm):
                        report, batch = episode(env_for(cfg, output, row, name), row, model=arm['model'],
                            collect=True, limit=reserve, deadline=until)
                except old.PilotBudgetExhausted:
                    _recover_interrupted(state, elapsed())
                    counter['exhausted'] = True
                    checkpoint()
                    continue
                counter['learning_seconds'] += elapsed() - counter['pending']['stage_start_elapsed']
                complete_episode(counter, report['agent_transitions'])
                add_coverage(counter, row, report['agent_transitions'], critic_only=critic, report=report)
                if batch is None:
                    raise ValueError('Collected episode has no usable active batch')
                counter['pending'] = dict(kind='update_ready', batch=batch, report=report, row=row,
                    critic_only=critic, reserved=reserve)
                checkpoint()
                update_pending(name)
            state['phase'] = 'final'
            checkpoint()
        initial = evaluate('initial-validation', ARMS[0], deadline)
        finals = {name: evaluate('final-' + name, name, deadline) for name in ARMS}
        comparison = compare_endpoints(cfg, initial['reports'], finals[ARMS[0]]['reports'], finals[ARMS[1]]['reports'])
        policies = {}
        for name, arm in arms.items():
            path = run / name / 'policy.pt'
            path.parent.mkdir(parents=True, exist_ok=True)
            export_actor(arm['model'], path)
            policies[name] = dict(path=path.relative_to(ROOT).as_posix(), sha256=file_digest(path))
        state.update(phase='complete', run_closed=True)
        checkpoint()
        result = dict(completed=True, accepted=False, held_out_test_run=False,
            elapsed_seconds=state['elapsed_seconds'], preparation_wall_seconds=proof['charged_preparation_wall_seconds'],
            counters={name: {k: v for k, v in counter.items() if k not in ('pending', 'coverage')} for name, counter in state['arms'].items()},
            coverage={name: coverage_report(counter,cfg) for name, counter in state['arms'].items()},
            initial_validation=initial['summary'], final_validation={name: value['summary'] for name, value in finals.items()},
            comparison=comparison,
            policies=policies, attribution='Only acceleration-imitation multiplier1 versus5 differs; common qualified flow reference/rewards/ABI')
        result['optimizer_counter_scope'] = 'Durably committed optimizer steps only. An interrupted update may have attempted uncommitted steps; count is unknown, recorded in arm failures, and that generation is rolled back without replay.'
        result['stage_advance'] = {name: bool(result['comparison']['arms'][name]['foundation_stage_gates_passed']
            and result['coverage'][name]['full_case_coverage']) for name in ARMS}
        old.atomic_json(run / 'summary.json', result)
        return result
    except Exception:
        failure_traceback = traceback.format_exc()
        # Restore the durable pre-update generation. Unknown partial updates are
        # discarded, never silently replayed; their elapsed time remains charged.
        saved = torch.load(run / 'last.pt', map_location='cpu', weights_only=False)
        _load_arms(arms, saved['arms'])
        state = saved['state']
        _recover_interrupted(state, elapsed())
        state['run_closed'] = True
        checkpoint()
        old.atomic_json(run / 'failure.json', dict(completed=False, elapsed_seconds=state['elapsed_seconds'],
            phase=state['phase'], traceback=failure_traceback, checkpoint='last.pt',
            unknown_partial_updates_discarded=True))
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('config', 'gradients', 'train'))
    parser.add_argument('--config', type=Path, default=DEFAULT)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--batches', type=Path)
    parser.add_argument('--resume', type=Path)
    parser.add_argument('--approved-config-sha256')
    args = parser.parse_args(argv)
    if args.mode == 'config':
        cfg = build_config()
        old.atomic_json(project_path(args.output or args.config), cfg)
        print(json.dumps(dict(training_authorized=False, readiness=None, config_sha256=digest(cfg))))
        return 0
    cfg = load_config(args.config)
    torch.set_num_threads(1)
    if args.mode == 'gradients':
        if not args.batches or not args.output:
            parser.error('gradients requires --batches and --output')
        batches = torch.load(project_path(args.batches, outputs=True), map_location='cpu', weights_only=False)
        result = frozen_loss_check(cfg, batches)
        old.atomic_json(project_path(args.output, outputs=True), result)
    else:
        run = args.output or ROOT / cfg['output_directory'] / ('train-' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:8])
        result = train(cfg, run, approved_config_sha256=args.approved_config_sha256, resume=args.resume)
    print(json.dumps(result, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
