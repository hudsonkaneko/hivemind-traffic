"""Frozen-actor SUMO command calibration; no learning or guard changes.

Runs only named training-partition fixtures and diagnostic seeds. A duration
probe reissues the same guarded command each 0.1 s; it never leaves a long
command unattended. Native model speed is a stored counterfactual, not a
separate uncontrolled rollout or proof of a particular active constraint.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import re
import subprocess
import time
import uuid

import numpy as np
import torch

from experiments.run_highway_population import ROOT, DEFAULT, digest, env_for, load_config
from traffic.highway_stage1_teacher import acc_response_ceiling, teacher_actions

ACTOR = ROOT / 'policies/versions/highway-stage1-v03-phase0/policy.pt'
ACTOR_SHA256 = '8526f6134b7eae90d3370bd12cc0487926a6ec4356c6c6ea2dd231e8aa8b1bb8'
SOURCES = ('experiments/diagnose_population_acceleration.py',
           'environments/highway_population_env.py', 'environments/highway_stage1_env.py',
           'environments/i110_merge_env.py', 'traffic/highway_stage1_teacher.py')


def finite_difference(before_speed, after_speed, before_time, after_time):
    dt = after_time - before_time
    if dt <= 0 or not np.isfinite([before_speed, after_speed, dt]).all():
        raise ValueError('Finite synchronized states and increasing timestamps required')
    return (after_speed - before_speed) / dt


def emergency_events(text):
    """Retain native warning lines and deduplicate exact vehicle/time events."""
    lines = [line for line in text.splitlines() if any(phrase in line.lower()
             for phrase in ('emergency braking', 'emergency stop'))]
    events = {}
    for line in lines:
        vehicle = re.search(r"[Vv]ehicle ['\"]([^'\"]+)['\"]", line)
        timestamp = re.search(r'(?:time[=: ]+)([0-9.]+)', line)
        key = (vehicle.group(1) if vehicle else None, float(timestamp.group(1).rstrip('.')) if timestamp else None)
        if key == (None, None):
            key = ('unparsed', line)
        deceleration = re.search(r'decel=([-0-9.]+)', line)
        events[key] = dict(vehicle_id=key[0], time_s=key[1], line=line,
            kind='lane_end_emergency_stop' if 'emergency stop' in line.lower() else 'emergency_braking',
            native_reported_decel=float(deceleration.group(1)) if deceleration else None)
    return dict(warning_lines=len(lines), unique_vehicle_time_events=len(events), events=list(events.values()))


def summarize_rows(rows):
    live = [r for r in rows if r['realized_acceleration_m_s2'] is not None]
    mismatches = [r for r in live if r['native_realization_changed']]
    mean = lambda key, items=live: float(np.mean([r[key] for r in items])) if items else None
    return dict(agent_steps=len(rows), paired_speed_samples=len(live),
        override_rate=mean('override', rows), native_mismatch_rate=mean('native_realization_changed', rows),
        policy_filter_rate=mean('policy_filter_changed', rows),
        requested_mean_m_s2=mean('requested_acceleration_m_s2'), filtered_mean_m_s2=mean('filtered_acceleration_m_s2'),
        realized_mean_m_s2=mean('realized_acceleration_m_s2'), speed_ratio=mean('speed_ratio'),
        request_realized_mae_m_s2=float(np.mean([abs(r['requested_acceleration_m_s2']-r['realized_acceleration_m_s2']) for r in live])) if live else None,
        derivative_max_error_m_s2=max((abs(r['speed_derivative_m_s2']-r['realized_acceleration_m_s2']) for r in live), default=None),
        native_mismatch_below_filtered=sum(r['realized_acceleration_m_s2'] < r['filtered_acceleration_m_s2'] for r in mismatches),
        native_mismatch_above_filtered=sum(r['realized_acceleration_m_s2'] > r['filtered_acceleration_m_s2'] for r in mismatches),
        mismatch_equal_native_counterfactual=sum(abs(r['speed_after_m_s']-r['native_speed_without_traci_after_m_s']) < 1e-6 for r in mismatches),
        speed_mode_values=sorted(set(r['speed_mode'] for r in rows)), lane_change_mode_values=sorted(set(r['lane_change_mode'] for r in rows)),
        road_seam_samples=sum(r['road_seam'] for r in live), ending_lane_samples=sum(r['lane_end_distance_m'] is not None for r in rows),
        near_limit_samples=sum(r['speed_before_m_s'] >= r['speed_limit_m_s']-.5 for r in rows),
        no_observed_front_samples=sum(r['front_gap_m'] is None for r in rows),
        observed_front_samples=sum(r['front_gap_m'] is not None for r in rows),
        sampled_emergency_steps=sum(r['realized_acceleration_m_s2'] < -4.55 for r in live))


def duration_comparison(left, right):
    fields = ('time_before_s', 'time_after_s', 'speed_before_m_s', 'speed_after_m_s',
              'requested_acceleration_m_s2', 'filtered_acceleration_m_s2', 'realized_acceleration_m_s2')
    if len(left) != len(right):
        return dict(equal_sample_count=False, left=len(left), right=len(right))
    if any((a['step'], a['agent']) != (b['step'], b['agent']) for a, b in zip(left, right)):
        raise ValueError('Duration comparisons require aligned seed, step and agent')
    delta = {field: max((abs(a[field]-b[field]) for a, b in zip(left, right)
                        if a[field] is not None and b[field] is not None), default=0.) for field in fields}
    return dict(equal_sample_count=True, samples=len(left), max_absolute_difference=delta)


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def run_episode(cfg, actor, output, spec, deadline):
    row = dict(case=next(c for c in cfg['training_cases'] if c['id'] == spec['case_id']),
               controlled_count=1, total_vehicles=spec['total_vehicles'], communication='none', packet_loss=0., seed=spec['seed'])
    if row['case']['split'] != 'train' or not 6520000 <= spec['seed'] < 6520100:
        raise ValueError('Diagnostic is restricted to training roads and diagnostic seeds')
    env = env_for(cfg, output, row, horizon=spec['horizon'])
    records, report, native_log = [], None, ''
    try:
        obs, _ = env.reset(seed=spec['seed'])
        version = env.conn.getVersion()
        native_set_acceleration = env.conn.vehicle.setAcceleration
        def guarded_duration(agent, acceleration, duration):
            # The existing environment computes and applies every guard first.
            # Only command duration changes; the command is refreshed every tick.
            native_set_acceleration(agent, acceleration, spec['command_duration_s'])
        env.conn.vehicle.setAcceleration = guarded_duration
        while env.agents:
            if time.monotonic() >= deadline:
                raise TimeoutError('Bounded calibration wall budget reached')
            agents = list(env.agents)
            if spec['controller'] == 'frozen_parent':
                with torch.inference_mode():
                    chosen = actor(torch.tensor(np.stack([obs[a] for a in agents]))).numpy()
                actions = dict(zip(agents, chosen))
            elif spec['controller'] == 'teacher':
                actions = teacher_actions({a: obs[a] for a in agents})
            else:
                # Fixed positive request reaches both acceleration and speed-cap regimes.
                actions = {a: np.array([1., 0.], np.float32) for a in agents}
            before = {a: dict(env.states[a]) for a in agents}
            before_time = env.time
            contexts = {a: env._context(a) for a in agents}
            ends = {a: env.merge_distance(before[a]) for a in agents}
            modes = {a: (env.conn.vehicle.getSpeedMode(a), env.conn.vehicle.getLaneChangeMode(a)) for a in agents}
            native_before = {a: env.conn.vehicle.getSpeedWithoutTraCI(a) for a in agents}
            obs, _, _, _, infos = env.step(actions)
            for agent in agents:
                old, new, info = before[agent], env.states.get(agent), infos[agent]
                front = contexts[agent]['neighbors']['own']['front']
                gap = front['bumper_gap_m'] if front else None
                relative = front['relative_speed_m_s'] if front else None
                limit = min(26.8224, old['allowed_speed'])
                records.append(dict(run=spec['id'], case_id=spec['case_id'], seed=spec['seed'], controller=spec['controller'],
                    step=env.steps, agent=agent, command_duration_s=spec['command_duration_s'],
                    time_before_s=before_time, time_after_s=env.time, dt_s=env.time-before_time,
                    speed_before_m_s=old['speed'], speed_after_m_s=new['speed'] if new else None,
                    speed_limit_m_s=limit, speed_ratio=new['speed']/limit if new else 0.,
                    requested_acceleration_m_s2=info['requested_acceleration'], filtered_acceleration_m_s2=info['filtered_acceleration'],
                    realized_acceleration_m_s2=info['realized_acceleration'],
                    speed_derivative_m_s2=finite_difference(old['speed'], new['speed'], before_time, env.time) if new else None,
                    native_speed_without_traci_before_m_s=native_before[agent],
                    native_speed_without_traci_after_m_s=env.conn.vehicle.getSpeedWithoutTraCI(agent) if new else None,
                    analytical_ceiling_m_s2=info['feasible_acceleration_ceiling'], current_target_m_s2=info['feasible_acceleration_target'],
                    acc_model_prediction_m_s2=acc_response_ceiling(old['speed'], limit, (gap, old['speed']+relative) if front else None),
                    front_gap_m=gap, front_relative_speed_m_s=relative, front_id=front.get('id') if front else None,
                    lane_before=old['lane_id'], lane_after=new['lane_id'] if new else None,
                    road_seam=bool(new and env.graph.lanes[old['lane_id']]['edge'] != env.graph.lanes[new['lane_id']]['edge']),
                    lane_end_distance_m=ends[agent], speed_mode=modes[agent][0], lane_change_mode=modes[agent][1],
                    override=info['override'], policy_filter_changed=info['policy_filter_changed'],
                    native_realization_changed=info['native_realization_changed'], lane_request=info['lane_request'],
                    override_causes='|'.join(info['override_causes'])))
        report = env.report()
        episode_directory = env.directory.relative_to(ROOT).as_posix()
    finally:
        env.close()
        if hasattr(env, 'directory') and (env.directory / 'sumo-errors.log').exists():
            native_log = (env.directory / 'sumo-errors.log').read_text(encoding='utf-8', errors='replace')
        if records:
            with (output / (spec['id']+'.csv')).open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(records[0]))
                writer.writeheader(); writer.writerows(records)
    summary = dict(spec=spec, runtime_version=version, episode_directory=episode_directory,
        measurements=summarize_rows(records), native_emergency_warnings=emergency_events(native_log), environment_report=report)
    write_json(output/(spec['id']+'.json'), summary)
    return records, summary


def draw_svg(path, rows):
    # Dependency-free vector plot of actual synchronized measurements.
    width, height = 1100, 700
    panels = [('frozen_parent', rows[:0]), ('teacher', rows[:0])]
    panels = [(kind, [r for r in rows if r['controller'] == kind and r['case_id'] == 'train-2-lane-cruise']) for kind, _ in panels]
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
           '<rect width="1100" height="700" fill="#fcfcfa"/>',
           '<style>text{font:16px sans-serif;fill:#222}</style>',
           '<text x="50" y="30">Frozen parent versus teacher: actual SUMO acceleration, train-only 1 AV / 24 cars</text>']
    for index, (name, records) in enumerate(panels):
        top = 75 + index*300
        svg += [f'<text x="80" y="{top-10}">{name}; seed 6520000; existing safety modes retained</text>',
                f'<path d="M80 {top}V{top+225}H1050" stroke="#555" fill="none"/>']
        for value in (-4.5, -2, 0, 2):
            y = top + (2.2-value)/7*225
            svg += [f'<path d="M80 {y:.2f}H1050" stroke="#ddd"/>', f'<text x="25" y="{y+5:.2f}">{value}</text>']
        for key, color in [('requested_acceleration_m_s2','#d07916'), ('realized_acceleration_m_s2','#176eb3')]:
            points = ' '.join(f"{80+(r['time_after_s']-records[0]['time_before_s'])/60*970:.2f},{top+(2.2-r[key])/7*225:.2f}" for r in records if r[key] is not None)
            svg += [f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="1.5"/>']
        svg += [f'<text x="80" y="{top+250}">0 s</text>', f'<text x="1000" y="{top+250}">60 s</text>']
    svg += ['<text x="85" y="680" fill="#d07916">Orange: requested acceleration</text>',
            '<text x="390" y="680" fill="#176eb3">Blue: measured acceleration (m/s²)</text>', '</svg>']
    Path(path).write_text('\n'.join(svg), encoding='utf-8')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--max-wall-seconds', type=int, default=360)
    args = parser.parse_args(argv)
    if not 1 <= args.max_wall_seconds <= 360:
        raise ValueError('Wall limit must be 1..360 seconds')
    if digest(ACTOR) != ACTOR_SHA256:
        raise ValueError('Frozen actor hash mismatch')
    torch.set_num_threads(1)
    cfg = load_config(DEFAULT)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    output = ROOT/'outputs/highway-population-diagnostics/v01'/f'acceleration-{stamp}-{uuid.uuid4().hex[:8]}'
    output.mkdir(parents=True, exist_ok=False)
    start = time.monotonic()
    before_hashes = {p: digest(ROOT/p) for p in SOURCES}
    actor = torch.jit.load(str(ACTOR), map_location='cpu').eval()
    specs = []
    for i, (case, total) in enumerate((('train-2-lane-cruise',24), ('train-2-lane-following',24),
                                      ('train-3-lane-passing',16), ('train-3-lane-merge',16))):
        for kind in ('frozen_parent','teacher'):
            specs.append(dict(id=f'{case}-{kind}', case_id=case, controller=kind, seed=6520000+i,
                              total_vehicles=total, horizon=600, command_duration_s=.1))
    for i, (case, total) in enumerate((('train-3-lane-cruise',8), ('train-2-lane-following',24), ('train-3-lane-merge',16))):
        for duration in (.1,.5):
            specs.append(dict(id=f'{case}-constant-duration-{duration}', case_id=case, controller='constant_2_m_s2',
                seed=6520010+i, total_vehicles=total, horizon=400, command_duration_s=duration))
    result = dict(schema='population_acceleration_diagnostics_v01', learning=False,
        actor=dict(path=ACTOR.relative_to(ROOT).as_posix(),sha256=ACTOR_SHA256),
        source_commit=subprocess.check_output(['git','rev-parse','HEAD'], cwd=ROOT, text=True).strip(),
        git_status_at_start=subprocess.check_output(['git','status','--porcelain'], cwd=ROOT, text=True).splitlines(),
        config_sha256=digest(DEFAULT), source_sha256_before=before_hashes,
        runtime=dict(python=platform.python_version(),torch=torch.__version__,numpy=np.__version__),
        scope='train-only diagnosis, one controlled car, no acceptance or generalization claim',
        api_limitations=['getSpeedWithoutTraCI is SUMO stored native counterfactual speed, not an independent rollout or causal limiter label',
            'getFollowSpeed deliberately not called: pinned ACC followSpeed ignores CalcReason and its _v may update native mode state',
            'Duration probes change only timeline duration; identical seed/reset and refreshed guarded commands every 0.1 s',
            'Frozen parent and teacher can choose different lanes; only paired constant-command duration probes isolate duration'],
        primary_sources=['https://sumo.dlr.de/docs/TraCI/Vehicle_Value_Retrieval.html',
            'https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html',
            'https://raw.githubusercontent.com/eclipse-sumo/sumo/v1_27_1/src/libsumo/Vehicle.cpp',
            'https://raw.githubusercontent.com/eclipse-sumo/sumo/v1_27_1/src/microsim/cfmodels/MSCFModel_ACC.cpp'],
        episodes=[], status='running')
    write_json(output/'resolved-config.json', cfg)
    (output/'executed-diagnostic-source.py').write_bytes(Path(__file__).read_bytes())
    all_rows, per_run = [], {}
    try:
        for spec in specs:
            records, summary = run_episode(cfg, actor, output, spec, start+args.max_wall_seconds)
            result['episodes'].append(summary)
            per_run[spec['id']] = records
            all_rows.extend(records)
        result['duration_comparisons'] = {case: duration_comparison(per_run[f'{case}-constant-duration-0.1'],per_run[f'{case}-constant-duration-0.5'])
            for case in ('train-3-lane-cruise','train-2-lane-following','train-3-lane-merge')}
        draw_svg(output/'acceleration-curves.svg', all_rows)
        result['status'] = 'complete'
    except Exception as error:
        result.update(status='failed', error=repr(error))
        raise
    finally:
        result['wall_seconds'] = time.monotonic()-start
        result['actor_sha256_after'] = digest(ACTOR)
        result['source_sha256_after'] = {p: digest(ROOT/p) for p in SOURCES}
        result['frozen_inputs_unchanged'] = result['actor_sha256_after']==ACTOR_SHA256 and result['source_sha256_after']==before_hashes
        result['artifacts'] = {p.name: digest(p) for p in sorted(output.iterdir()) if p.is_file()}
        write_json(output/'acceleration-summary.json', result)
        print(json.dumps(dict(output=str(output), status=result['status'],wall_seconds=result['wall_seconds'])))


if __name__ == '__main__':
    main()
