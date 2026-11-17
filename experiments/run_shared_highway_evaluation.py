"""Serial frozen-actor comparison. No trainer, optimizer, critic or rollout collection."""
from __future__ import annotations

import argparse
from copy import deepcopy
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def read(path):
    return json.loads(path.read_text())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def content_digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def canonical_root():
    return Path(subprocess.check_output(['git', 'rev-parse', '--path-format=absolute',
        '--git-common-dir'], cwd=ROOT, text=True).strip()).parent.resolve()


def inside(base, value):
    path = (base / value).resolve()
    if not path.is_relative_to(base.resolve()):
        raise ValueError('Artifact escapes declared project directory')
    return path


def verify_sources(spec):
    for relative, expected in spec['source_hashes'].items():
        if digest(inside(ROOT, relative)) != expected:
            raise ValueError('Source changed: ' + relative)


def validate_spec(spec):
    if (spec.get('evaluation_authorized') is not True or spec.get('training_authorized') is not False
            or spec.get('held_out_authorized') is not False):
        raise ValueError('Only explicitly approved development evaluation is supported')
    if len(spec['candidates']) != 3 or len(spec['scenarios']) != 10:
        raise ValueError('Expected three frozen actors and ten scenarios')
    if sum(bool(r.get('cutin_stress')) for r in spec['scenarios']) != 2:
        raise ValueError('Exactly two verified cut-in cases required')
    if len({c['id'] for c in spec['candidates']}) != 3:
        raise ValueError('Candidate identifiers must be unique')
    if any(Path(c['id']).name != c['id'] or c['id'] in ('.', '..') for c in spec['candidates']):
        raise ValueError('Unsafe candidate directory name')
    budget = spec['budget']
    if (budget.get('approved') is not True or budget['max_episode_runs'] != 30
            or budget['max_wall_seconds'] != 900 or budget['training_decisions'] != 0):
        raise ValueError('The approved cap is 30 episodes / 900 seconds / zero training')
    if spec['environment']['raw_request_scope'] != 'all':
        raise ValueError('Full-route scoring cannot exclude startup decisions')


def load_actor(candidate, canonical):
    import torch
    base = canonical if candidate['path_base'] == 'canonical_checkout' else ROOT
    path = inside(base, candidate['path'])
    if digest(path) != candidate['sha256']:
        raise ValueError('Actor checksum changed: ' + candidate['id'])
    if candidate['artifact_type'] == 'full_checkpoint_actor_only':
        from policies.highway_flow import FlowActor
        saved = torch.load(path, map_location='cpu', weights_only=False)
        if saved.get('schema') != 'highway_flow_rl_checkpoint_v04':
            raise ValueError('Baseline must be the pinned full v04 checkpoint')
        actor = FlowActor()
        actor.load_state_dict({k.removeprefix('actor.'): v for k, v in saved['model'].items()
            if k.startswith('actor.')}, strict=True)
    elif candidate['artifact_type'] == 'torchscript_actor':
        actor = torch.jit.load(str(path), map_location='cpu')
    else:
        raise ValueError('Unsupported frozen actor format')
    actor.eval()
    for parameter in actor.parameters():
        parameter.requires_grad_(False)
    return actor


def checked_actions(actor, observations):
    import torch
    if observations.shape != (24, 117) or not torch.isfinite(observations).all():
        raise ValueError('Actor needs finite local observations only')
    with torch.inference_mode():
        actions = actor(observations)
    if (actions.shape != (24, 2) or not torch.isfinite(actions).all()
            or (actions[:, 0].abs() > 1).any()
            or not ((actions[:, 1] == -1) | (actions[:, 1] == 0) | (actions[:, 1] == 1)).all()):
        raise ValueError('Invalid normalized acceleration / lane action')
    mask = observations[:, 30:33] > .5
    mask[~mask.any(dim=1), 1] = True
    if not mask.gather(1, (actions[:, 1].long() + 1).unsqueeze(1)).all():
        raise ValueError('Actor selected a masked lane')
    return actions


def accumulate(stats, agent, index, metrics):
    for skip in (0, 1, 5):
        if index < skip:
            continue
        group = stats.setdefault(agent, {}).setdefault(str(skip), {})
        for key, value in metrics.items():
            if value is None:
                continue
            if not math.isfinite(value):
                raise ValueError('Nonfinite jerk sample')
            row = group.setdefault(key, dict(samples=0, peak_abs=0., sum_squared=0.))
            row['samples'] += 1
            row['peak_abs'] = max(row['peak_abs'], abs(value))
            row['sum_squared'] += value * value


def native_evidence(env):
    from traffic.sumo_warnings import native_emergency_report
    # SUMO creates a unique episode child; the case directory is only its parent.
    return native_emergency_report(Path(env.directory) / 'sumo-errors.log')


def evaluate_case(cfg, row, directory, actor, deadline):
    import numpy as np
    import torch
    from experiments import run_highway_merge_recovery_v08 as runtime
    env = runtime.env_for(cfg, row, directory)
    base = runtime.base
    report = None
    modes, counts, previous, jerk = {}, {}, {}, {}
    transitions = 0
    try:
        base.check_deadline(deadline)
        observations, _ = env.reset(seed=row['seed'])
        with gzip.open(directory / 'actor-actions.jsonl.gz', 'wt', compresslevel=1) as trace:
            while env.agents:
                base.check_deadline(deadline)
                for agent in env.agents:
                    modes[agent] = dict(speed=env.conn.vehicle.getSpeedMode(agent),
                        lane_change=env.conn.vehicle.getLaneChangeMode(agent))
                    if modes[agent] != dict(speed=31, lane_change=512):
                        raise ValueError('Native safety mode changed')
                local = torch.tensor(np.stack([observations.get(a, np.zeros(117, np.float32))
                    for a in base.AGENTS]), dtype=torch.float32)
                actions = checked_actions(actor, local)
                chosen = {a: actions[i].numpy() for i, a in enumerate(base.AGENTS) if a in env.agents}
                before_time = env.time
                nxt, rewards, _, _, infos = env.step(chosen)
                for agent, info in infos.items():
                    if not np.isfinite(rewards[agent]) or info.get('teacher_reference_used') is not False:
                        raise ValueError('Invalid reward or teacher-controlled episode')
                    audit = info['v05_audit']['conditioning']
                    ordinal = counts.get(agent, 0)
                    raw_request = float(audit['requested'])
                    slew = None if agent not in previous else (raw_request - previous[agent]) / .1
                    accumulate(jerk, agent, ordinal, dict(command_slew=slew,
                        request_vs_actuator_jerk=float(audit['raw_requested_jerk']),
                        realized_jerk=info['realized_jerk_m_s3']))
                    trace.write(json.dumps(dict(time_s=before_time, agent=agent,
                        decision_index=ordinal, observation=observations[agent].tolist(),
                        normalized_action=chosen[agent].tolist(), command_slew_m_s3=slew,
                        info=info), allow_nan=False) + '\n')
                    previous[agent], counts[agent] = raw_request, ordinal + 1
                transitions += len(infos)
                observations = {a: nxt[a] for a in env.agents}
            report = base.primary_accounting(env, partial=False, deadline=deadline)
            report.update(case_id=row['case']['id'], seed=row['seed'], task=row['case']['task'],
                agent_transitions=transitions, partial_control_horizon=False, population_setting=row,
                native_control_modes=modes, episode_directory=directory.relative_to(ROOT).as_posix(),
                teacher_reference_used=False, actor_only_evaluation=True, optimizer_steps=0)
            for scopes in jerk.values():
                for values in scopes.values():
                    for metric in values.values():
                        metric['rms'] = math.sqrt(metric.pop('sum_squared') / metric['samples'])
            report['evaluation_jerk_windows'] = jerk
            if not report['primary_horizon_complete'] or not report['accounting_comparison_valid']:
                raise ValueError('Incomplete full-horizon accounting')
            if row.get('cutin_stress'):
                proof = report.get('cutin_fixture', {})
                if not proof.get('moving_at_handoff') or not proof.get('completed_into_target'):
                    raise ValueError('Missing actual cut-in exposure')
    except BaseException:
        save(directory / 'interrupted-accounting.json', dict(actual_agent_transitions=transitions,
            incomplete_episode=True, traceback=traceback.format_exc()))
        raise
    finally:
        env.close()
        if report is not None:
            report['native_emergencies'] = native_evidence(env)
            report['native_episode_directory'] = Path(env.directory).relative_to(ROOT).as_posix()
            save(directory / 'report.json', report)
    return report


def scalar_summary(value):
    return {k: v for k, v in value.items() if not isinstance(v, (dict, list))}


def worker(spec_path, output, started):
    import torch
    from experiments import run_highway_merge_recovery_v08 as runtime
    base = runtime.base
    canonical = canonical_root()
    base.STUDY = canonical / 'outputs/highway-flow-rl/v02'
    base.ruled_gates = runtime.v07.previous.v06_gates
    spec = read(spec_path)
    validate_spec(spec)
    verify_sources(spec)
    cfg_path = ROOT / spec['environment_config']['path']
    if digest(cfg_path) != spec['environment_config']['sha256']:
        raise ValueError('Common environment configuration changed')
    cfg = read(cfg_path)
    if spec['scenarios'] != cfg['validation_rows']:
        raise ValueError('The reviewed ten-case suite changed')
    cfg.update(training_authorized=False, held_out_authorized=False, round_index=spec['round_index'])
    cfg['budget'].update(spec['budget']['resolved_ledger_limits'])
    for row in spec['scenarios']:
        if digest(inside(ROOT, row['case']['network'])) != row['case']['network_sha256']:
            raise ValueError('Scenario network changed')
    save(output / 'resolved-spec.json', spec)
    save(output / 'resolved-environment.json', cfg)
    for relative in spec['source_hashes']:
        target = output / 'sources' / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / relative, target)
    manifest = dict(source_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        git_dirty=bool(subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True).strip()),
        source_hashes=spec['source_hashes'], config_sha256=digest(spec_path), python=sys.version,
        torch=torch.__version__, candidates=spec['candidates'], training=False, optimizer_steps=0)
    save(output / 'manifest.json', manifest)
    torch.set_num_threads(1)
    torch.manual_seed(0)
    all_results, table = {}, []
    with base.LedgerSession(cfg, 'shared_actor_evaluation', output, digest(spec_path),
            started=started, episodes=30, max_wall_seconds=900) as session:
        previous_ledger = {**session.ledger, 'segments': session.ledger['segments'][:-1]}
        if content_digest(previous_ledger) != spec['budget']['ledger_before_content_sha256']:
            raise ValueError('Shared ledger changed since the evaluation allowance was resolved')
        session.segment.update(authorization=spec['authorization'], evaluation_only=True,
            intended_episodes=30, completed_episodes=0, attempted_episodes=0)
        session.checkpoint()
        actors = {c['id']: load_actor(c, canonical) for c in spec['candidates']}
        probes = torch.zeros(24, 117)
        probes[:, 30:33] = 1.
        proof = {}
        for name, actor in actors.items():
            first = checked_actions(actor, probes)
            if not torch.equal(first, checked_actions(actor, probes)):
                raise ValueError('Actor is not deterministic')
            one_lane = probes.clone(); one_lane[:, 30:33] = 0.; one_lane[:, 31] = 1.
            checked_actions(actor, one_lane)
            proof[name] = dict(interface_passed=True, deterministic=True, optimizer_steps=0)
        save(output / 'actor-preflight.json', proof)
        for name, actor in actors.items():
            reports = []
            for index, row in enumerate(spec['scenarios']):
                base.check_deadline(session.deadline)
                session.segment['attempted_episodes'] += 1
                session.checkpoint()
                directory = output / name / f'case-{index:02d}'
                report = evaluate_case(cfg, row, directory, actor, session.deadline)
                reports.append(report)
                session.segment['completed_episodes'] += 1
                session.checkpoint()
                gate = base.ruled_gates(report)
                matches = [q.get('speed_match_fraction', 0.) for q in report.get('quality_merges', {}).values()]
                table.append(dict(candidate=name, case_index=index, case_id=row['case']['id'],
                    seed=row['seed'], stress=bool(row.get('cutin_stress')), passed=gate['passed'],
                    merge_match=sum(matches)/len(matches) if matches else None,
                    overrides=report['intervention_fraction'],
                    peak_realized_jerk=max(c['peak_abs_jerk_m_s3'] for c in gate['per_car'].values()),
                    collisions=report['collisions'], teleports=report['teleports'],
                    cutin_verified=bool(report.get('cutin_fixture', {}).get('completed_into_target')),
                    failures=';'.join(gate['failures'])))
                save(output / 'completed-cases.json', table)
            verify_sources(spec)
            groups = {}
            for label, subset in [('nominal', [r for r in reports if not r.get('cutin_stress')]),
                                  ('cutin', [r for r in reports if r.get('cutin_stress')])]:
                summary = base.summarize(subset)
                groups[label] = dict(**scalar_summary(summary),
                    strict_cases_passed=sum(g['passed'] for g in summary['case_gates']),
                    merge_speed_match=runtime.v07.previous.merge_speed_match(subset))
            save(output / name / 'evaluation.json', dict(summary=groups, reports=reports))
            all_results[name] = groups
        for candidate in spec['candidates']:
            base_path = canonical if candidate['path_base'] == 'canonical_checkout' else ROOT
            if digest(inside(base_path, candidate['path'])) != candidate['sha256']:
                raise ValueError('Frozen actor file changed during evaluation')
        with (output / 'cases.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(table[0])); writer.writeheader(); writer.writerows(table)
        save(output / 'summary.json', dict(completed=True, training=False, optimizer_steps=0,
            completed_episodes=len(table), elapsed_seconds=session.elapsed(), results=all_results,
            promoted=False, held_out=False, acceptance_scope='Matched development comparison only'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spec', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--worker', action='store_true')
    parser.add_argument('--started', type=float)
    args = parser.parse_args()
    spec_path, output = inside(ROOT, args.spec), inside(ROOT, args.output)
    if args.worker:
        try:
            worker(spec_path, output, args.started)
        except BaseException:
            save(output / 'failure.json', dict(training=False, failed=True,
                elapsed_seconds=time.monotonic()-args.started, traceback=traceback.format_exc()))
            raise
        return 0
    spec = read(spec_path)
    validate_spec(spec)
    output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    logs = ROOT / 'logs'; logs.mkdir(exist_ok=True)
    status = dict(status='running', training=False, started_unix=time.time(), cap_seconds=900,
        output=output.relative_to(ROOT).as_posix())
    save(output / 'job-status.json', status)
    command = [sys.executable, '-u', str(Path(__file__).resolve()), '--spec', str(spec_path),
        '--output', str(output), '--worker', '--started', str(started)]
    with (logs / (output.name + '.log')).open('x') as log:
        process = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        status['worker_pid'] = process.pid; save(output / 'job-status.json', status)
        try:
            code = process.wait(timeout=max(.01, 900-(time.monotonic()-started)))
        except subprocess.TimeoutExpired:
            # Terminate only this evaluator and its own SUMO children at the user's cap.
            subprocess.run(['taskkill', '/PID', str(process.pid), '/T', '/F'], stdout=log, stderr=log)
            process.wait()
            code = 124
            study = canonical_root() / 'outputs/highway-flow-rl/v02'
            lock = study / 'active.lock'
            if lock.exists() and lock.read_text() == str(output):
                ledger = read(study / 'cost-ledger.json')
                segment = ledger['segments'][-1]
                if segment['status'] == 'running' and segment['config_sha256'] == digest(spec_path):
                    segment.update(status='failed', charged_seconds=time.monotonic()-started,
                        failure='900-second parent watchdog', decisions=0)
                    save(study / 'cost-ledger.json', ledger)
                    lock.unlink()
            save(output / 'failure.json', dict(training=False, failed=True, timeout=True,
                elapsed_seconds=time.monotonic()-started, reason='900-second evaluation cap'))
    status.update(status='complete' if code == 0 else 'failed', exit_code=code,
        elapsed_seconds=time.monotonic()-started, finished_unix=time.time())
    save(output / 'job-status.json', status)
    print(json.dumps(status))
    return code


if __name__ == '__main__':
    raise SystemExit(main())
