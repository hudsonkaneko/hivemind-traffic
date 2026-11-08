"""Matched, bounded smooth-objective experiment. Training is separately gated.

Configuration and frozen-gradient modes never collect native rollouts or update
weights. The preparation script owns native preflight. Both training arms retain
the existing v05 collector, bootstrap handling, 99-field actor and 24-slot critic.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from copy import deepcopy
from datetime import datetime, timezone
import json
from pathlib import Path
import random
import shutil
import subprocess
import sys
import time
import traceback
import uuid

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np
import torch

from experiments import run_highway_environment_v05 as old
from policies.mappo import MappoTrainer, export_actor
from traffic.highway_smooth_contract import (
    ARMS, SCHEMA, compare_endpoints, comparison_settings, complete_episode, elapsed_on_resume, file_digest,
    new_arm_state, preparation_digest, project_path, recover_pending,
    require_training, reserve_episode, validate_config,
)
from traffic.highway_smooth_schedule import build_schedule, coverage_report, add_coverage, digest, next_case

DEFAULT = ROOT / 'configs/training/highway-smooth-v01.json'
PARENT_CONFIG = 'policies/versions/highway-environment-v05/training-config.json'
PARENT_CONFIG_SHA = '17a28932f90c0ae64d875beab733c4d275de506487f7cad7b1a11909423121f9'
ACTOR_SHA = 'fd639c39f0c514256a458b2b85d35baba242125f44dcc3d9542d767eb2dd46ec'
CHECKPOINT_SHA = '9279b6103d13d2be3fd18fe05e9bae72462e581a16d881fc9e4ff87b0b2ac02d'
NETWORK_SHA = 'ae829282ad98cfaeeba806fba30e44536361ccabf4d0861708c5638fff512836'
SOURCES = tuple(dict.fromkeys((*old.SOURCES, 'experiments/run_highway_smooth_control.py',
    'traffic/highway_smooth_contract.py', 'traffic/highway_smooth_schedule.py',
    'traffic/highway_smooth_reference.py', 'traffic/highway_smooth_metrics.py',
    'environments/highway_smooth_control.py', 'scripts/prepare_highway_smooth_control.py',
    'scripts/record_highway_environment_v05.py', 'scripts/check_highway_smooth_gradients.py',
    'scripts/summarize_highway_smooth_preflight.py')))


def sources():
    return {name: file_digest(ROOT / name) for name in SOURCES}


def build_config():
    from traffic.highway_smooth_reference import REFERENCE_VERSION, IMPLEMENTATION_NOTES
    if file_digest(ROOT / PARENT_CONFIG) != PARENT_CONFIG_SHA:
        raise ValueError('Frozen parent configuration changed')
    base = json.loads((ROOT / PARENT_CONFIG).read_text(encoding='utf-8'))
    base.update(training_authorized=False, readiness=None)
    base['training'].update(seed_start=7000000, seed_end=7000999,
        max_agent_transitions=50000, critic_warmup_transitions=2048,
        final_evaluation_reserve_seconds=900,
        counting='Per-arm 50k includes critic warmup and failed reservations; paired outer budget is authoritative',
        schedule='Immutable matched case strata; actual decision exposure separately measured')
    for case in base['training_cases'] + [row['case'] for row in base['validation_cases']]:
        if 'i110' in case['id'].lower():
            case.update(network='outputs/i110-merge-blocking/v01/network-v02/network.net.xml', network_sha256=NETWORK_SHA)
    cfg = dict(schema=SCHEMA, training_authorized=False, readiness=None,
        status='implemented_requires_no_update_readiness_and_explicit_training_approval',
        output_directory='outputs/highway-smooth-control/v02', base_config=base,
        initialization=dict(seed=9507, actor=dict(path='policies/versions/highway-environment-v05/policy.pt', sha256=ACTOR_SHA),
            checkpoint=dict(path='outputs/highway-environment/v05/train-20261106T024828Z-90b25880/last.pt', sha256=CHECKPOINT_SHA),
            critic='Fresh identical critic and optimizer; only frozen actor and log_std imported',
            sampling='Independent saved RNG streams initialized identically; no replayed PPO samples'),
        arms=[dict(id=ARMS[0], objective_version='v05', imitation_acceleration_weight=1.),
              dict(id=ARMS[1], objective_version='smooth_v01', imitation_acceleration_weight=5.)],
        optimizer=deepcopy(base['optimizer']),
        gae=dict(gamma=.99, gae_lambda=.95), comparison=comparison_settings(),
        reference_version=REFERENCE_VERSION, reference_implementation_notes=list(IMPLEMENTATION_NOTES),
        reference_parameters=dict(legal_speed_buffer_m_s=.1, entry_preview_time_s=2.,
            nominal_reference_jerk_m_s3=2., desired_speed_error_gain_s_inverse=.4),
        reward_bundle=dict(feasible_acceleration=1., jerk=.03, command_reference_residual_slew=.01,
            huber='H(x)=0.5*x^2 for abs(x)<=1; abs(x)-0.5 otherwise; uncapped',
            jerk_argument='realized jerk / (1 m/s^3)',
            residual_slew_argument='delta(requested acceleration - current reference) / (0.1 s * 1 m/s^3)'),
        budget=dict(per_arm_decisions=50000, total_decisions=100000, wall_seconds=3600,
            critic_warmup_per_arm=2048, final_evaluation_reserve_seconds=900,
            accounting='Preparation including failures, initial 24, both arms and final 2x24 share one wall ceiling; warmup is included'),
        schedule=dict(seed=9507, seed_start=7000000, seed_end=7000999,
            planned_case_shares=dict(cruise=.2, following=.3, passing=.2, merge=.3),
            planned_difficulty_case_shares={'0': .4, '1': .3, '2': .3}, dense_following_case_share=.2,
            definition='Twenty-stratum CASE allocation, not planned active-decision allocation. Full 80 s episodes except remaining phase/budget limits. Actual decision shares and target shortfalls reported.',
            refinement='Identical case order and warmup leading cases per arm; behavior-dependent arrivals and population prevent exact decision quotas'),
        evaluation=dict(shared_initial_cases=24, final_cases_per_arm=24, held_out=False,
            protocol='All fixed rows, corrected I-110; final endpoints only, no selection or validation demonstrations'),
        preparation_ledger='outputs/highway-smooth-control/v02/preparation-ledger.json',
        provenance=dict(parent_config=PARENT_CONFIG, parent_config_sha256=PARENT_CONFIG_SHA,
            plan='documentation/highway-smooth-training-plan-v01.md'))
    cfg['schedule_sha256'] = digest(build_schedule(cfg))
    validate_config(cfg)
    return cfg


def load_config(path=DEFAULT):
    cfg = json.loads(project_path(path).read_text(encoding='utf-8'))
    validate_config(cfg)
    return cfg


def env_for(cfg, output, row, arm_id=ARMS[0]):
    from environments.highway_smooth_control import HighwaySmoothControlEnv
    arm = next((item for item in cfg['arms'] if item['id'] == arm_id), None)
    if arm is None:
        raise ValueError('Unknown objective arm')
    base, case = cfg['base_config'], row['case']
    path = project_path(case['network'])
    if file_digest(path) != case['network_sha256']:
        raise ValueError('Frozen road checksum changed')
    env = HighwaySmoothControlEnv(path, output, objective_version=arm['objective_version'],
        reference_parameters=cfg['reference_parameters'], controlled_count=row['controlled_count'],
        total_vehicles=row['total_vehicles'], horizon=base['environment']['horizon'],
        communication=row['communication'], packet_loss=row['packet_loss'],
        through_lanes=case['through_lanes'], task=case['task'], density=case['density'],
        reward_weights=base['reward_weights'], control_target_version='v3',
        speed_limit_m_s=row['speed_limit_m_s'], difficulty=row.get('difficulty', 0),
        traffic_mode=row.get('traffic_mode', 'initial'), lane_change_duration=3., stabilization_seconds=8.)
    env.case_id = case['id']
    env.near_limit_metrics_enabled = True
    return env


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


def initialize_arms(cfg):
    validate_config(cfg)
    for key in ('actor', 'checkpoint'):
        spec = cfg['initialization'][key]
        if file_digest(project_path(spec['path'])) != spec['sha256']:
            raise ValueError('Frozen ' + key + ' checksum mismatch')
    saved = torch.load(project_path(cfg['initialization']['checkpoint']['path']), map_location='cpu', weights_only=False)
    frozen = torch.jit.load(str(project_path(cfg['initialization']['actor']['path'])), map_location='cpu').eval()
    outer, result = capture_rng(), {}
    try:
        for spec in cfg['arms']:
            seed = cfg['initialization']['seed']
            torch.manual_seed(seed); np.random.seed(seed); random.seed(seed)
            model = old.model_for(cfg['base_config'])
            values = model.state_dict()
            for name in values:
                if name.startswith('actor.') or name == 'log_std':
                    values[name] = saved['model'][name].clone()
            model.load_state_dict(values)
            options = dict(cfg['optimizer'], imitation_acceleration_weight=spec['imitation_acceleration_weight'])
            trainer = MappoTrainer(model, **options)
            observations = torch.zeros(24, 99)
            observations[:, 30:33] = 1.
            with torch.no_grad():
                actions = model.act(observations, deterministic=True)[0]
                expected = frozen(observations)
            if actions.shape != (24, 2) or not torch.isfinite(actions).all() or not torch.equal(actions, expected):
                raise ValueError('Full checkpoint actor differs from frozen 99-to-2 artifact')
            result[spec['id']] = dict(model=model, trainer=trainer, rng=capture_rng())
    finally:
        restore_rng(outer)
    return result


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
                    report, _ = old.episode(env_for(cfg, run / label / str(index), row, arm_name), row,
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
            stats = old.critic_update(arm['model'], arm['trainer'], pending['batch'], until) if pending['critic_only'] else arm['trainer'].update(pending['batch'], deadline=until)
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
                row = next_case(schedule, counter)
                critic = not counter['critic_closed']
                if critic and cfg['budget']['critic_warmup_per_arm'] - counter['critic_spent'] < row['controlled_count']:
                    counter['critic_closed'], critic = True, False
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
                        report, batch = old.episode(env_for(cfg, output, row, name), row, model=arm['model'],
                            collect=True, limit=reserve, deadline=until)
                except old.PilotBudgetExhausted:
                    _recover_interrupted(state, elapsed())
                    counter['exhausted'] = True
                    checkpoint()
                    continue
                counter['learning_seconds'] += elapsed() - counter['pending']['stage_start_elapsed']
                complete_episode(counter, report['agent_transitions'])
                add_coverage(counter, row, report['agent_transitions'], critic_only=critic)
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
        comparison = compare_endpoints(cfg, finals[ARMS[0]]['reports'], finals[ARMS[1]]['reports'])
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
            coverage={name: coverage_report(counter) for name, counter in state['arms'].items()},
            initial_validation=initial['summary'], final_validation={name: value['summary'] for name, value in finals.items()},
            comparison=comparison,
            policies=policies, attribution='Combined reference/objective bundle; individual terms cannot be attributed')
        result['optimizer_counter_scope'] = 'Durably committed optimizer steps only. An interrupted update may have attempted uncommitted steps; count is unknown, recorded in arm failures, and that generation is rolled back without replay.'
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
