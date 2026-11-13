"""Separately gated flow117 v05 preparation, evaluation and resumable MAPPO."""
from __future__ import annotations

import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import random
import shutil
import platform
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
import numpy as np
import torch

from experiments import run_highway_flow as flow
from experiments.run_highway_flow_rl import summary as legacy_summary, training_row
from experiments.run_highway_stage1 import PilotBudgetExhausted, check_deadline, primary_accounting
from policies.highway_flow import FlowModel, critic_update
from policies.mappo import MappoTrainer, compute_gae, export_actor
from traffic.highway_flow_rl_v05_contract import validate_config, validate_preflight, file_digest

AGENTS = tuple(f'agent_{i}' for i in range(24))
# Shared ledger and lock retain every previous preparation/failure charge.
STUDY = ROOT / 'outputs/highway-flow-rl/v02'
OUTPUT_ROOT = ROOT / 'outputs/highway-flow-rl/v05'
CHECKPOINT_SCHEMA = 'highway_flow_rl_checkpoint_v05'
LEDGER_SCHEMA = 'highway_flow_rl_cost_ledger_v02'
NEW_SOURCES = ('experiments/run_highway_flow_rl_v05.py', 'experiments/run_highway_flow_rl.py',
    'environments/highway_flow_rl_env.py', 'environments/highway_flow_rl_v05_env.py',
    'traffic/highway_flow_rl_v02_rewards.py', 'environments/highway_flow_rl_v02_env.py',
    'traffic/highway_flow_rl_v05_contract.py', 'traffic/highway_flow_rl_v05_rewards.py',
    'traffic/highway_flow_rl_v03_rewards.py',
    'scripts/prepare_highway_flow.py', 'scripts/screen_flow_rl_v02_network.py',
    'environments/highway_flow_rl_v04_env.py', 'traffic/highway_flow_rl_v04_rewards.py',
    'scripts/run_highway_flow_rl_v05_job.py')


def hashes():
    return {name: file_digest(ROOT / name) for name in dict.fromkeys((*flow.SOURCES, *NEW_SOURCES))}


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    flow.old.atomic_json(path, value)


def finite(value, name, *, minimum=0.):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < minimum:
        raise ValueError('Invalid ' + name)
    return float(value)


def project_path(value):
    path = (ROOT / value).resolve()
    if not path.is_relative_to(ROOT.resolve()):
        raise ValueError('All v05 artifacts must remain in this checkout')
    return path


def ledger_totals(ledger, round_index):
    if ledger.get('schema') != LEDGER_SCHEMA or ledger.get('inherited_seconds') != 0.:
        raise ValueError('v05 requires the existing v02 study ledger')
    result = dict(total=0., preparation=0., training=0., final_evaluation=0.,
        round_seconds=0., round_decisions=0, total_decisions=0, diagnostic_seconds=0., diagnostic_episodes=0)
    for segment in ledger.get('segments', []):
        if segment.get('status') not in ('complete', 'failed'):
            raise ValueError('Unclosed segment requires explicit elapsed-time reconciliation')
        seconds = finite(segment.get('charged_seconds'), 'charged segment time')
        decisions = segment.get('decisions', 0)
        if type(decisions) is not int or decisions < 0:
            raise ValueError('Invalid charged decisions')
        category = segment.get('category', 'preparation' if segment.get('command') == 'network_screen' else None)
        if category not in ('preparation', 'training', 'final_evaluation'):
            raise ValueError('Unknown ledger category')
        result['total'] += seconds
        result[category] += seconds
        if category == 'training':
            result['total_decisions'] += decisions
            if segment.get('round_index') == round_index:
                result['round_seconds'] += seconds
                result['round_decisions'] += decisions
        elif category == 'preparation' and segment.get('command') != 'network_screen':
            episodes = segment.get('episodes', 0)
            if type(episodes) is not int or episodes < 0:
                raise ValueError('Invalid diagnostic episode count')
            result['diagnostic_seconds'] += seconds
            result['diagnostic_episodes'] += episodes
    return result


def available_seconds(cfg, totals, category):
    budget = cfg['budget']
    remaining = budget['total_wall_seconds'] - totals['total']
    if category == 'training':
        remaining = min(remaining - max(0., budget['final_reserve_seconds'] - totals['final_evaluation']),
            budget['per_round_wall_seconds'] - totals['round_seconds'])
    elif category == 'preparation':
        remaining = min(remaining, budget['preparation_wall_seconds'] - totals['preparation'],
            budget['diagnostic_wall_seconds'] - totals['diagnostic_seconds'])
    else:
        remaining = min(remaining, budget['final_reserve_seconds'] - totals['final_evaluation'])
    return max(0., remaining)


class LedgerSession:
    """One serialized segment; failures and source/config revisions retain costs."""
    def __init__(self, cfg, command, output, fingerprint, *, started, episodes=0, final=False, max_wall_seconds=None):
        self.cfg, self.command, self.output, self.fingerprint = cfg, command, output, fingerprint
        self.started, self.episodes, self.max_wall_seconds = started, episodes, max_wall_seconds
        self.category = 'training' if command == 'train' else ('final_evaluation' if final else 'preparation')
        self.path, self.lock = STUDY / 'cost-ledger.json', STUDY / 'active.lock'

    def __enter__(self):
        if not self.path.is_file():
            raise ValueError('V05 requires the existing study ledger; never reset historical charges')
        STUDY.mkdir(parents=True, exist_ok=True)
        with self.lock.open('x', encoding='utf-8') as stream:
            stream.write(str(self.output))
        try:
            self.ledger = json.loads(self.path.read_text()) if self.path.exists() else dict(
                schema=LEDGER_SCHEMA, inherited_seconds=0., segments=[])
            self.totals = ledger_totals(self.ledger, self.cfg['round_index'])
            allowed = available_seconds(self.cfg, self.totals, self.category)
            if self.max_wall_seconds is not None:
                allowed = min(allowed, finite(self.max_wall_seconds, 'requested wall bound', minimum=.001))
            episode_cap = self.cfg['budget'].get('max_diagnostic_episodes', 8)
            if self.category == 'preparation' and self.totals['diagnostic_episodes'] + self.episodes > episode_cap:
                raise ValueError(f'Preparation exceeds the {episode_cap} authorized diagnostic episodes')
            if allowed <= self.elapsed():
                raise ValueError('No charged wall-time budget remains')
            self.deadline = self.started + allowed
            self.segment = dict(command=self.command, category=self.category, round_index=self.cfg['round_index'],
                output=self.output.relative_to(ROOT).as_posix(), status='running',
                charged_seconds=allowed, reserved_seconds=allowed, decisions=0, episodes=self.episodes,
                config_sha256=self.fingerprint, started_at_unix=time.time() - self.elapsed())
            self.ledger['segments'].append(self.segment)
            save(self.path, self.ledger)
            return self
        except BaseException:
            self.lock.unlink(missing_ok=True)
            raise

    def elapsed(self):
        return max(0., time.monotonic() - self.started)

    def checkpoint(self, decisions=0):
        self.segment.update(charged_seconds=self.elapsed(), decisions=decisions)
        save(self.path, self.ledger)

    def __exit__(self, kind, error, tb):
        try:
            self.segment.update(status='failed' if error is not None else 'complete', charged_seconds=self.elapsed())
            save(self.path, self.ledger)
        finally:
            self.lock.unlink(missing_ok=True)


def build_model(cfg, *, checkpoint=None, resume=False, fingerprint=None, sources=None, evaluation=False):
    """Load the full flow117 checkpoint directly; no old initializer migration."""
    path = project_path(checkpoint or cfg['warm_start']['path'])
    saved = torch.load(path, map_location='cpu', weights_only=False)
    if saved.get('schema') not in ('highway_flow_rl_checkpoint_v01', 'highway_flow_rl_checkpoint_v03', 'highway_flow_rl_checkpoint_v04', CHECKPOINT_SCHEMA):
        raise ValueError('A full flow117 RL checkpoint is required')
    if any(key not in saved for key in ('model', 'optimizer', 'rng', 'state')):
        raise ValueError('Incomplete full checkpoint')
    if resume:
        if (saved['schema'] != CHECKPOINT_SCHEMA or saved.get('config_sha256') != fingerprint
                or saved.get('source_hashes') != sources):
            raise ValueError('Exact resume requires matching v05 configuration and sources')
    elif path == project_path(cfg['warm_start']['path']):
        if file_digest(path) != cfg['warm_start']['sha256']:
            raise ValueError('Pinned warm-start checksum changed')
    elif evaluation and path.as_posix() in {(ROOT/r['path']).as_posix() for r in cfg['evaluation_initializers']}:
        pin = next(r for r in cfg['evaluation_initializers'] if (ROOT/r['path']) == path)
        if file_digest(path) != pin['sha256'] or saved['schema'] != 'highway_flow_rl_checkpoint_v04':
            raise ValueError('Frozen v04 evaluation initializer changed')
    elif saved['schema'] != CHECKPOINT_SCHEMA or saved.get('source_hashes') != sources:
        raise ValueError('Evaluation checkpoint must match the resolved v05 sources')
    elif saved.get('config_sha256') != fingerprint:
        previous_path = path.parent / 'resolved-config.json'
        if not evaluation or not previous_path.is_file() or file_digest(previous_path) != saved.get('config_sha256'):
            raise ValueError('Evaluation checkpoint configuration is not verified')
        previous, current = json.loads(previous_path.read_text()), deepcopy(cfg)
        for value in (previous, current):
            for key in ('training_authorized', 'held_out_authorized', 'development_gates_passed'):
                value.pop(key, None)
        if previous != current:
            raise ValueError('Evaluation changed the trained objective or declared cases')
    model = FlowModel()
    model.load_state_dict(saved['model'], strict=True)
    trainer = MappoTrainer(model, **cfg['optimizer'])
    if resume:
        trainer.optimizer.load_state_dict(saved['optimizer'])
        flow.restore_rng(saved['rng'])
    return model, trainer, saved


def env_for(cfg, row, directory):
    from environments.highway_flow_rl_v05_env import HighwayFlowRLV05Env
    case = row['case']
    network = project_path(case['network'])
    if file_digest(network) != case['network_sha256']:
        raise ValueError('Frozen network changed')
    fixture = row.get('fixture')
    if row.get('fixture_path'):
        fixture = dict(path=row['fixture_path'], sha256=row['fixture_sha256'])
    if isinstance(fixture, dict) and 'path' in fixture:
        path = project_path(fixture['path'])
        if file_digest(path) != fixture['sha256']:
            raise ValueError('Frozen fixture changed')
        fixture = json.loads(path.read_text())
    env = HighwayFlowRLV05Env(network, directory, fixture=fixture,
        controlled_count=row['controlled_count'], total_vehicles=row['total_vehicles'],
        horizon=row['horizon_steps'], communication=row['communication'], packet_loss=row['packet_loss'],
        through_lanes=case['through_lanes'], task=case['task'], density=case['density'],
        control_target_version='v3', speed_limit_m_s=row['speed_limit_m_s'], difficulty=row['difficulty'],
        traffic_mode=row['traffic_mode'], lane_change_duration=3., stabilization_seconds=8.,
        jerk_reward_weight=cfg['reward_weights']['jerk'], v05_weights=cfg['v05_reward_weights'],
        v05_variant=cfg['variant'],
        controlled_car_follow_model=cfg.get('controlled_car_follow_model', 'ACC'),
        road_end_leader_exit_rule=cfg.get('road_end_leader_exit_rule', False))
    env.case_id = case['id']
    if fixture and fixture.get('native_conflict_schedule'):
        from scripts.screen_flow_rl_v02_network import install_conflict_schedule
        install_conflict_schedule(env, fixture)
    return env


def episode(cfg, row, directory, model, *, collect=False, limit=None, deadline=None, record=False):
    env = env_for(cfg, row, directory)
    samples, frames, decisions, steps, report = [], [], 0, 0, None
    modes = {}
    try:
        check_deadline(deadline)
        obs, _ = env.reset(seed=row['seed'])
        controlled = list(env._controlled_agents())
        if record:
            from scripts.prepare_highway_flow import capture_frame
            frames.append(capture_frame(env))
            advance = env._advance
            def recorded_advance():
                result = advance()
                frames.append(capture_frame(env))
                return result
            env._advance = recorded_advance
        while env.agents:
            check_deadline(deadline)
            if limit is not None and decisions + len(env.agents) > limit:
                break
            if row.get('limit_steps') is not None and steps >= row['limit_steps']:
                break
            for agent in env.agents:
                value = np.asarray(obs[agent])
                if value.shape != (117,) or not np.isfinite(value).all():
                    raise ValueError('Invalid local observation')
                modes[agent] = dict(speed=env.conn.vehicle.getSpeedMode(agent), lane_change=env.conn.vehicle.getLaneChangeMode(agent))
                if modes[agent] != dict(speed=31, lane_change=512):
                    raise ValueError('Native safety mode changed')
            active, indices = torch.tensor([agent in env.agents for agent in AGENTS]), torch.arange(24)
            local = torch.tensor(np.stack([obs.get(agent, np.zeros(117, np.float32)) for agent in AGENTS]))
            global_state = np.asarray(env.state())
            if global_state.shape != (2832,) or not np.isfinite(global_state).all():
                raise ValueError('Invalid centralized critic state')
            state = torch.as_tensor(global_state).repeat(24, 1)
            with torch.no_grad():
                actions, logp, _, latent = model.act(local, deterministic=not collect)
                values = model.values(state, indices)
            if not torch.isfinite(actions).all() or (actions[:, 0].abs() > 1).any() or not ((actions[:, 1] == -1) | (actions[:, 1] == 0) | (actions[:, 1] == 1)).all():
                raise ValueError('Invalid normalized acceleration/lane action')
            chosen = {agent: actions[i].numpy() for i, agent in enumerate(AGENTS) if agent in env.agents}
            nxt, rewards, term, trunc, infos = env.step(chosen)
            if not all(np.isfinite(value) for value in rewards.values()) or any(info.get('teacher_reference_used') is not False for info in infos.values()):
                raise ValueError('Nonfinite or teacher-derived reward')
            decisions += len(rewards)
            steps += 1
            if collect:
                successor = np.concatenate([np.r_[nxt.get(agent, np.zeros(117)), float(not term.get(agent, True))] for agent in AGENTS]).astype(np.float32)
                with torch.no_grad():
                    next_values = model.values(torch.tensor(successor).repeat(24, 1), indices)
                samples.append(dict(local_obs=local, global_states=state, agent_indices=indices,
                    actions=actions, pre_tanh_actions=latent, old_log_probs=logp, old_values=values,
                    next_values=next_values, rewards=torch.tensor([rewards.get(agent, 0.) for agent in AGENTS]),
                    terminated=torch.tensor([term.get(agent, False) for agent in AGENTS]),
                    truncated=torch.tensor([trunc.get(agent, False) for agent in AGENTS]), active=active))
            obs = {agent: nxt[agent] for agent in env.agents}
        partial = bool(env.agents)
        if samples and partial:
            samples[-1]['truncated'] |= samples[-1]['active'] & ~samples[-1]['terminated']
        report = primary_accounting(env, partial=partial, deadline=deadline)
        report.update(case_id=env.case_id, seed=row['seed'], task=row['case']['task'], agent_transitions=decisions,
            partial_control_horizon=partial, population_setting=deepcopy(row), native_control_modes=modes,
            episode_directory=directory.relative_to(ROOT).as_posix(), teacher_reference_used=False)
        if record:
            from scripts.prepare_highway_flow import route_metrics
            report['full_route_metrics'] = route_metrics(frames, controlled)
            save(directory / 'recording.json', dict(schema='highway_flow_rl_recording_v05',
                frames=frames, controlled_ids=controlled, case_id=env.case_id, seed=row['seed'], dt_s=.1))
        batch = None
        if samples:
            batch = {key: torch.stack([sample[key] for sample in samples]) for key in samples[0]}
            batch['advantages'], batch['returns'] = compute_gae(batch['rewards'], batch['old_values'],
                batch['next_values'], batch['terminated'], batch['truncated'], batch['active'],
                gamma=cfg['gae']['gamma'], gae_lambda=cfg['gae']['lambda'])
        return report, batch
    except BaseException:
        save(directory / 'interrupted-accounting.json', dict(actual_agent_transitions=decisions,
            incomplete_episode=True, traceback=traceback.format_exc()))
        if record and frames:
            save(directory / 'recording-partial.json', dict(frames=frames, incomplete_episode=True))
        raise
    finally:
        env.close()
        if report is not None:
            from traffic.sumo_warnings import native_emergency_report
            report['native_emergencies'] = native_emergency_report(env.directory / 'sumo-errors.log')


def ruled_gates(report):
    """Apply opt-in evaluation rules, then the unchanged development gates.

    road_end_leader_exit_rule: each car's peak jerk excludes only the single
    decision right after its own leader left the network at its route end
    (environment evidence in report['road_end']). Missing evidence fails the
    gate. The original uncensored peak stays in the report.
    """
    if report.get('road_end_leader_exit_rule') is not True:
        return development_gates(report)
    ruled = deepcopy(report)
    for agent, car in ruled.get('smooth_control', {}).get('per_car', {}).items():
        realized = car.get('all', {}).get('jerk_m_s3', {}).get('realized')
        if realized is not None:
            realized['peak_abs'] = report.get('road_end', {}).get('per_car', {}).get(agent, {}).get('peak_abs_jerk_m_s3')
    result = development_gates(ruled)
    result['evaluation_rules'] = dict(road_end_leader_exit=True,
        exempt=[dict(agent=a, **e) for a, c in report.get('road_end', {}).get('per_car', {}).items() for e in c['exempt']])
    return result


def development_gates(report):
    """Uncensored per-car gates; missing evidence fails instead of becoming zero."""
    failures, cars = [], {}
    if any(report.get(key) != 0 for key in ('collisions', 'teleports', 'overspeed_steps', 'emergency_braking_observed')):
        failures.append('safety_events_or_missing_counts')
    native = report.get('native_emergencies', {})
    if native.get('available') is not True or native.get('unique_events') != 0:
        failures.append('native_emergency_or_missing_accounting')
    if (report.get('primary_horizon_complete') is not True or report.get('partial_control_horizon') is not False
            or report.get('accounting_comparison_valid') is not True):
        failures.append('incomplete_primary_horizon')
    for agent, car in report.get('smooth_control', {}).get('per_car', {}).items():
        if not car.get('assigned_controlled'):
            continue
        all_rows = car.get('all', {})
        count = all_rows.get('decisions', 0)
        peak = all_rows.get('jerk_m_s3', {}).get('realized', {}).get('peak_abs')
        ratio = all_rows.get('legal_speed_utilization', {}).get('mean')
        intervention = all_rows.get('intervention_decisions')
        checks = dict(measured=count > 0, completed_route=car.get('route_completed') is True,
            peak_jerk=peak is not None and math.isfinite(peak) and peak <= 5.,
            interventions=count > 0 and intervention is not None and intervention / count <= .05)
        route = report.get('per_car_full_route', {}).get(agent, {})
        headway, ttc = route.get('minimum_headway_s'), route.get('minimum_ttc_s')
        checks.update(headway_ttc_coverage=type(route.get('leader_samples')) is int
            and route.get('missing_headway_samples') == 0 and route.get('missing_ttc_samples') == 0,
            headway=headway is None or (math.isfinite(headway) and headway >= 1.5),
            ttc=ttc is None or (math.isfinite(ttc) and ttc >= 3.))
        if report.get('task') == 'cruise':
            checks['cruise_speed'] = ratio is not None and math.isfinite(ratio) and ratio >= .9
            diagnosis = report.get('avoidable_braking', {}).get(agent, {})
            clear = diagnosis.get('clear_cruise_decisions', 0)
            checks['clear_cruise_coverage'] = clear > 0
            checks['clear_cruise_speed'] = clear > 0 and diagnosis.get('clear_cruise_speed_fraction_sum', 0.) / clear >= .9
            checks['avoidable_braking'] = diagnosis.get('events') == 0
        cars[agent] = dict(checks=checks, passed=all(checks.values()), decisions=count,
            intervention_fraction=intervention / count if count and intervention is not None else None,
            peak_abs_jerk_m_s3=peak, mean_legal_speed_fraction=ratio,
            minimum_headway_s=headway, minimum_ttc_s=ttc, leader_samples=route.get('leader_samples'),
            missing_headway_samples=route.get('missing_headway_samples'), missing_ttc_samples=route.get('missing_ttc_samples'))
    if not cars or not all(car['passed'] for car in cars.values()):
        failures.append('per_car_route_comfort_or_control')
    if report.get('v04_reward_summary', {}).get('detour_motion_events') != 0:
        failures.append('optional_ending_lane_detour_or_missing_audit')
    if report.get('task') == 'passing' and report.get('passes', 0) < 1:
        failures.append('passing')
    if report.get('task') == 'merge':
        merge = report.get('stable_merge', {})
        if merge.get('targets', 0) <= 0 or merge.get('stable', 0) / merge['targets'] < .9:
            failures.append('stable_merge')
        quality = report.get('quality_merges', {})
        if merge.get('targets',0) <= 0 or sum(r['qualified'] for r in quality.values())/merge['targets'] < .9:
            failures.append('rolling_merge_quality')
    return dict(passed=not failures, failures=failures, per_car=cars)


def summarize(reports):
    result = legacy_summary(reports)
    gates = [dict(case_id=report['case_id'], seed=report['seed'], **ruled_gates(report)) for report in reports]
    result.update(development_gates_passed=bool(gates) and all(gate['passed'] for gate in gates), case_gates=gates,
        acceptance_scope='Development only; uncensored full-route per-car gates, no promotion')
    return result


def evaluate(cfg, output, label, model, deadline, *, rows=None, record=False, progress=None, completed=None):
    reports = list(completed or [])
    rows = cfg['validation_rows'] if rows is None else rows
    for index in range(len(reports), len(rows)):
        report, _ = episode(cfg, rows[index], output / label / f'case-{index:02d}', model, deadline=deadline, record=record)
        save(output / label / f'{index:02d}.json', report)
        reports.append(report)
        if progress:
            progress(reports)
    result = dict(summary=summarize(reports), reports=reports)
    save(output / f'{label}.json', result)
    return result


def validation_rank(result):
    reports = result['reports']
    safety = sum(r.get('collisions',0)+r.get('teleports',0)+r.get('emergency_braking_observed',0)
                 + r.get('native_emergencies',{}).get('unique_events',0) for r in reports)
    failures = sum(not g['passed'] for g in result['summary']['case_gates'])
    overrides = max((c['intervention_fraction'] or 0. for g in result['summary']['case_gates'] for c in g['per_car'].values()), default=1.)
    quality = sum(sum(q['qualified'] for q in r.get('quality_merges',{}).values()) for r in reports)
    completed = sum(c['checks']['completed_route'] for g in result['summary']['case_gates'] for c in g['per_car'].values())
    stops = sum(sum(r.get('pre_merge_stop_seconds',{}).values()) for r in reports)
    return safety, failures, -completed, -quality, stops, overrides


def new_state():
    return dict(phase='initial_evaluation', episodes=0, ppo_episodes=0, critic_episodes=0,
        decisions=0, critic_decisions=0, updates=0, optimizer_steps=0, critic_optimizer_steps=0,
        coverage={}, pending=None, recoveries=[], evaluations={})


def recover_pending(state):
    pending = state.get('pending')
    if not pending:
        return
    if pending['kind'] == 'episode':
        state['recoveries'].append(dict(kind='discarded_interrupted_rollout', reserved_decisions=pending['reserved'],
            output=pending['output'], reason='SUMO state cannot be resumed; full reservation remains charged'))
        state['episodes'] += 1
        state['critic_episodes' if pending['critic'] else 'ppo_episodes'] += 1
        state['pending'] = None
    elif pending['kind'] == 'updating':
        pending['kind'] = 'update_ready'
        state['recoveries'].append(dict(kind='rolled_back_uncommitted_update',
            reason='Durable pre-update model, optimizer, RNG and on-policy batch restored; elapsed time remains charged'))
    elif pending['kind'] != 'update_ready':
        raise ValueError('Unknown pending checkpoint generation')


def update_pending(state, model, trainer, checkpoint, output, deadline):
    pending = state.get('pending')
    if not pending or pending['kind'] != 'update_ready':
        return
    check_deadline(deadline)
    pending['kind'] = 'updating'
    checkpoint()
    before = deepcopy(dict(model=model.state_dict(), optimizer=trainer.optimizer.state_dict(),
        rng=flow.capture_rng(), state=state))
    try:
        stats = critic_update(model, trainer, pending['batch'], deadline) if pending['critic'] else trainer.update(pending['batch'], deadline=deadline)
        state['critic_optimizer_steps' if pending['critic'] else 'optimizer_steps'] += stats['optimizer_steps']
        state['updates'] += 1
        state['episodes'] += 1
        state['critic_episodes' if pending['critic'] else 'ppo_episodes'] += 1
        state['approximate_kl'] = stats.get('last_kl', stats.get('approx_kl', 0.))
        case_id = pending['row']['case']['id']
        coverage = state['coverage'].setdefault(case_id, dict(ppo_decisions=0, critic_decisions=0, episodes=0))
        coverage['critic_decisions' if pending['critic'] else 'ppo_decisions'] += pending['report']['agent_transitions']
        coverage['episodes'] += 1
        save(output / 'updates' / f'{state["updates"]:05d}.json', dict(report=pending['report'], optimization=stats,
            critic_only=pending['critic'], row=pending['row']))
        state['pending'] = None
        checkpoint()
    except BaseException:
        model.load_state_dict(before['model'])
        trainer.optimizer.load_state_dict(before['optimizer'])
        flow.restore_rng(before['rng'])
        state.clear()
        state.update(before['state'])
        state['pending']['kind'] = 'update_ready'
        state['recoveries'].append(dict(kind='failed_update_rolled_back', unknown_attempted_steps=True,
            reason='Full rollout retained for exact resume; all elapsed time charged'))
        checkpoint()
        raise


def run_preflight(cfg, output, model, trainer, fingerprint, sources, deadline):
    row = cfg.get('recovery_preflight_row', cfg['training_rows'][0])
    report, batch = episode(cfg, row, output / 'smoke', model, collect=True, limit=64, deadline=deadline)
    save(output / 'smoke-report.json', report)
    flow.old.save_checkpoint(output / 'preflight-batch.pt', batch)
    return finish_preflight(cfg, output, model, trainer, report, batch, fingerprint, sources, deadline)


def finish_preflight(cfg, output, model, trainer, report, batch, fingerprint, sources, deadline):
    """CPU-only proof reduction, reusable from saved smoke evidence without rerunning SUMO."""
    if batch is None:
        raise ValueError('Preflight yielded no active on-policy batch')
    before = deepcopy(dict(model=model.state_dict(), optimizer=trainer.optimizer.state_dict(), rng=flow.capture_rng()))
    diagnostics = trainer.diagnose_batch(batch, phase='v05_zero_update_preflight')
    json.dumps(diagnostics, allow_nan=False)
    if not flow.tree_equal(before, dict(model=model.state_dict(), optimizer=trainer.optimizer.state_dict(), rng=flow.capture_rng())):
        raise ValueError('Zero-update diagnostics changed weights, optimizer or RNG')
    if any(report.get(key) != 0 for key in ('collisions', 'teleports', 'overspeed_steps', 'emergency_braking_observed')):
        raise ValueError('Preflight observed native safety events')
    native = report.get('native_emergencies', {})
    if native.get('available') is not True or native.get('unique_events') != 0 or not report['native_control_modes']:
        raise ValueError('Preflight lacks clean native warning/control-mode evidence')
    if not report.get('v04_reward_summary') or not report.get('v04_mask_audit'):
        raise ValueError('Preflight lacks measured v05 reward/mask audit')
    if report.get('reward_version') != cfg['reward_version'] or report['v04_mask_audit'].get('version') != cfg['mask_version']:
        raise ValueError('Measured reward or mask version differs from the resolved configuration')
    actor_path = output / 'actor-preflight.pt'
    export_actor(model, actor_path)
    actor = torch.jit.load(str(actor_path), map_location='cpu')
    probe = batch['local_obs'].reshape(-1, 117)[batch['active'].reshape(-1)]
    with torch.no_grad():
        if not torch.equal(model.act(probe, deterministic=True)[0], actor(probe)):
            raise ValueError('Export changed deterministic actor actions')
    check_deadline(deadline)
    if hashes() != sources:
        raise ValueError('Sources changed during preflight')
    proof = dict(schema='highway_flow_rl_preflight_v05', passed=True, training=False, optimizer_steps=0,
        config_sha256=fingerprint, source_hashes=sources,
        checks=dict(api=True, shapes=True, native_guards=True, finite_gradient=True, export=True, reward_version=True, mask_version=True),
        report=report, gradient_diagnostics=diagnostics, scope='Zero-update runtime evidence; no teacher or performance qualification')
    validate_preflight(proof, fingerprint, sources)
    save(output / 'preflight.json', proof)
    return dict(completed=True, training=False, preflight=str(output / 'preflight.json'))


def full_episode_reservation(row, remaining):
    required = row['horizon_steps'] * row['controlled_count']
    if row.get('rollout_decisions') != required:
        raise ValueError('Full-episode allocation must cover every active agent at every tick')
    return required if remaining >= required else None


def train(cfg, output, model, trainer, saved, session, fingerprint, sources, *, resume=False, preflight=None):
    validate_config(cfg, require_training=True)
    validate_preflight(preflight or {}, fingerprint, sources)
    state = deepcopy(saved['state']) if resume else new_state()
    if resume:
        state['phase'] = state.pop('resume_phase', state['phase'])
        recover_pending(state)
    segment_start_decisions = state['decisions']
    if session.totals['round_decisions'] != segment_start_decisions:
        raise ValueError('Checkpoint decisions do not match the append-only round ledger')
    remaining = min(cfg['budget']['per_round_decisions'] - session.totals['round_decisions'],
        cfg['budget']['total_decisions'] - session.totals['total_decisions'])
    decision_limit = segment_start_decisions + max(0, remaining)
    if state['phase'] == 'complete':
        raise ValueError('Round is already complete')
    reserve = min(900., max(0., session.deadline - time.monotonic()) / 3.)
    learning_deadline = session.deadline - reserve

    def checkpoint():
        flow.old.save_checkpoint(output / 'last.pt', dict(schema=CHECKPOINT_SCHEMA,
            model=model.state_dict(), optimizer=trainer.optimizer.state_dict(), rng=flow.capture_rng(),
            config_sha256=fingerprint, source_hashes=sources, state=deepcopy(state),
            segment_charged_seconds=session.elapsed(), saved_at_unix=time.time()))
        session.segment['checkpoint'] = (output / 'last.pt').relative_to(ROOT).as_posix()
        session.checkpoint(state['decisions'] - segment_start_decisions)

    def evaluate_phase(label, deadline):
        def progress(reports):
            state['evaluations'][label] = reports
            checkpoint()
        return evaluate(cfg, output, label, model, deadline,
            progress=progress, completed=state['evaluations'].get(label))

    def periodic_evaluation():
        if state['decisions'] < state.get('next_validation_decisions', cfg['evaluation_interval_decisions']):
            return
        label = f'periodic-validation-{state["updates"]:05d}'
        result = evaluate_phase(label, learning_deadline)
        score = validation_rank(result)
        export_actor(model, output / 'candidates' / f'policy-{state["updates"]:05d}.pt')
        improved = 'best_validation_rank' not in state or score < tuple(state['best_validation_rank'])
        if improved:
            state['best_validation_rank'] = list(score)
            state['best_validation_label'] = label
            export_actor(model, output / 'best-policy.pt')
        state['next_validation_decisions'] = state['decisions'] + cfg['evaluation_interval_decisions']
        checkpoint()
        shutil.copyfile(output / 'last.pt', output / 'candidates' / f'checkpoint-{state["updates"]:05d}.pt')
        if improved:
            shutil.copyfile(output / 'last.pt', output / 'best.pt')

    checkpoint()
    try:
        if state['phase'] == 'initial_evaluation':
            baseline = evaluate_phase('initial-validation', learning_deadline)
            state['best_validation_rank'] = list(validation_rank(baseline))
            state['best_validation_label'] = 'initial-validation'
            export_actor(model, output / 'best-policy.pt')
            export_actor(model, output / 'candidates' / 'initial-policy.pt')
            state['phase'] = 'learning'
            checkpoint()
            shutil.copyfile(output / 'last.pt', output / 'best.pt')
        if state['phase'] == 'learning':
            try:
                update_pending(state, model, trainer, checkpoint, output, learning_deadline)
                while state['decisions'] < decision_limit:
                    check_deadline(learning_deadline)
                    critic = state['critic_decisions'] < cfg['critic_warmup_decisions']
                    row = training_row(cfg, state, critic)
                    reservation = full_episode_reservation(row, decision_limit-state['decisions'])
                    if reservation is None:
                        state['learning_stop'] = 'remaining_budget_below_complete_episode_reservation'
                        break
                    state['decisions'] += reservation
                    directory = output / 'train' / f'episode-{state["episodes"]:04d}'
                    state['pending'] = dict(kind='episode', reserved=reservation, critic=critic, row=row,
                        output=directory.relative_to(ROOT).as_posix())
                    checkpoint()
                    report, batch = episode(cfg, row, directory, model, collect=True, limit=reservation, deadline=learning_deadline)
                    if report['partial_control_horizon']:
                        raise ValueError('Training reset before its complete declared episode')
                    actual = report['agent_transitions']
                    if not 0 < actual <= reservation or batch is None:
                        raise ValueError('Rollout has no valid active batch or exceeded its reservation')
                    state['decisions'] -= reservation - actual
                    if critic:
                        state['critic_decisions'] += actual
                    state['pending'] = dict(kind='update_ready', batch=batch, report=report, row=row, critic=critic)
                    checkpoint()
                    update_pending(state, model, trainer, checkpoint, output, learning_deadline)
                    periodic_evaluation()
            except PilotBudgetExhausted:
                recover_pending(state)
                if state.get('pending'):
                    state['recoveries'].append(dict(kind='unused_rollout_at_budget_end',
                        decisions=state['pending']['report']['agent_transitions'], reason='No learning time remains; measured decisions stay charged'))
                    state['pending'] = None
                state['learning_stop'] = 'charged_wall_limit'
            state['phase'] = 'final_evaluation'
            checkpoint()
        final = evaluate_phase('final-validation', session.deadline)
        export_actor(model, output / 'policy.pt')
        score = validation_rank(final)
        if 'best_validation_rank' not in state or score < tuple(state['best_validation_rank']):
            state['best_validation_rank'] = list(score)
            state['best_validation_label'] = 'final-validation'
            export_actor(model, output / 'best-policy.pt')
        if hashes() != sources:
            raise ValueError('Sources changed during training')
        state['phase'] = 'complete'
        checkpoint()
        if state.get('best_validation_label') == 'final-validation':
            shutil.copyfile(output / 'last.pt', output / 'best.pt')
        initial = summarize(state['evaluations']['initial-validation'])
        return dict(completed=True, failed=False, training=True, round_index=cfg['round_index'],
            initial=initial, final=final['summary'], **final['summary'], state=state,
            initial_pooled_intervention_fraction=initial['pooled_intervention_fraction'],
            approximate_kl=state.get('approximate_kl', 0.), checkpoint=str(output / 'last.pt'),
            policy=str(output / 'policy.pt'), training_acceptance='experimental_not_promoted')
    except BaseException:
        state['resume_phase'] = state['phase']
        state['phase'] = 'failed'
        checkpoint()
        raise


def selected_rows(cfg, args):
    rows = deepcopy(cfg['held_out_rows'] if args.final else cfg['validation_rows'])
    if args.case_index is not None:
        if not 0 <= args.case_index < len(rows):
            raise ValueError('Case index is outside the declared evaluation split')
        rows = [rows[args.case_index]]
    if args.limit_steps is not None:
        if args.limit_steps <= 0:
            raise ValueError('Diagnostic step limit must be positive')
        for row in rows:
            row['limit_steps'] = args.limit_steps
    return rows


def run(args):
    started = time.monotonic()
    if Path.cwd().resolve() != ROOT.resolve():
        raise ValueError('Use the canonical project checkout')
    config_path = project_path(args.config)
    cfg = json.loads(config_path.read_text())
    validate_config(cfg, require_training=args.command == 'train')
    if args.final and (args.command != 'evaluate' or cfg.get('held_out_authorized') is not True
                       or cfg.get('development_gates_passed') is not True):
        raise ValueError('Held-out evaluation requires explicit authorization and passed development gates')
    fingerprint, sources = file_digest(config_path), hashes()
    proof = None
    if args.command == 'train':
        if args.preflight is None:
            raise ValueError('Training requires measured preflight')
        proof = json.loads(project_path(args.preflight).read_text())
        validate_preflight(proof, fingerprint, sources)
    output = project_path(args.output)
    if not output.is_relative_to(OUTPUT_ROOT.resolve()) or output.exists():
        raise ValueError('Use a fresh output directory under outputs/highway-flow-rl/v05')
    rows = selected_rows(cfg, args) if args.command == 'evaluate' else None
    episodes = len(rows) if rows is not None else (1 if args.command == 'preflight' else 0)
    with LedgerSession(cfg, args.command, output, fingerprint, started=started, episodes=episodes,
                       final=args.final, max_wall_seconds=args.max_wall_seconds) as session:
        output.mkdir(parents=True)
        try:
            torch.set_num_threads(1)
            torch.use_deterministic_algorithms(True)
            torch.manual_seed(cfg['seed'])
            np.random.seed(cfg['seed'])
            random.seed(cfg['seed'])
            prior = [segment for segment in session.ledger['segments'][:-1]
                if segment.get('category') == 'training' and segment['round_index'] == cfg['round_index']]
            resumable = [segment for segment in prior if segment.get('checkpoint')]
            if args.command == 'train' and resumable:
                latest = project_path(resumable[-1]['checkpoint'])
                if not args.resume or args.checkpoint is None or project_path(args.checkpoint) != latest:
                    raise ValueError('An existing round must resume its latest charged checkpoint')
            elif args.resume:
                raise ValueError('Resume lacks a prior charged training segment')
            save(output / 'resolved-config.json', cfg)
            save(output / 'manifest.json', dict(config_sha256=fingerprint, source_hashes=sources,
                publication_branch=cfg.get('publication_branch'),
                platform=platform.platform(), processor=platform.processor(),
                git_dirty=bool(subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True)),
                git_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                previous_charged_seconds=session.totals['total'], python=sys.version, torch=torch.__version__,
                numpy=np.__version__, warm_start=str(args.checkpoint or cfg['warm_start']['path']),
                optimizer_handling='exact checkpoint continuation' if args.resume else 'fresh optimizer for v05 objective; actor/critic warm-start only'))
            for name, checksum in sources.items():
                destination = output / 'source' / name
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_bytes((ROOT / name).read_bytes())
                if file_digest(destination) != checksum:
                    raise ValueError('Source changed during snapshot')
            model, trainer, saved = build_model(cfg, checkpoint=args.checkpoint, resume=args.resume,
                fingerprint=fingerprint, sources=sources, evaluation=args.command == 'evaluate')
            if args.command == 'preflight':
                result = run_preflight(cfg, output, model, trainer, fingerprint, sources, session.deadline)
            elif args.command == 'evaluate':
                result = dict(completed=True, training=False, **evaluate(cfg, output, 'evaluation', model,
                    session.deadline, rows=rows, record=args.record))
                result['summary'].update(held_out=bool(args.final),
                    evaluation_split='held_out' if args.final else 'development',
                    acceptance_scope=('Held-out measurement only; no automatic promotion' if args.final
                                      else 'Development only; uncensored full-route per-car gates, no promotion'))
            else:
                result = train(cfg, output, model, trainer, saved, session, fingerprint, sources, resume=args.resume, preflight=proof)
            result.update(elapsed_seconds=session.elapsed(), total_charged_seconds=session.totals['total'] + session.elapsed())
            save(output / 'summary.json', result)
            return result
        except BaseException:
            save(output / 'failure.json', dict(completed=False, failed=True, training=args.command == 'train',
                traceback=traceback.format_exc(), elapsed_seconds=session.elapsed(),
                total_charged_seconds=session.totals['total'] + session.elapsed(), checkpoint=str(output / 'last.pt')))
            raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('preflight', 'evaluate', 'train'))
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--preflight', type=Path)
    parser.add_argument('--checkpoint', type=Path)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--case-index', type=int)
    parser.add_argument('--limit-steps', type=int)
    parser.add_argument('--record', action='store_true')
    parser.add_argument('--final', action='store_true')
    parser.add_argument('--max-wall-seconds', type=float)
    args = parser.parse_args(argv)
    if args.resume and (args.command != 'train' or args.checkpoint is None):
        parser.error('Exact resume requires train --checkpoint')
    result = run(args)
    print(json.dumps({key: result[key] for key in ('completed', 'training', 'elapsed_seconds', 'total_charged_seconds')}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
