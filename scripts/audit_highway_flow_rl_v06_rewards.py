"""V06 Phase 2: offline reward audit on saved frozen-v04 trajectories (no simulation).

Recomputes every v06 reward term per decision from saved v05 evaluation traces
(startup mode, whose base terms and speed target are the ones v06 keeps):
- base v04 terms as recorded (same weights and target as v06);
- gap preparation re-scored with the 1 s blended potential;
- unit-weight merge_tracking, jerk_excess and avoidable_braking.
Merge tracking is also scored on the best trajectory a car could drive against
the same recorded target (acceleration/jerk-limited dynamic program), giving
the reward a smooth, well-matched merge would earn. Weights are then chosen by
the rules stated in the plan. Open-loop caveat as in the Phase 0a audit.
"""
import argparse
import gzip
import hashlib
import importlib.util
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from traffic import highway_flow_rl_v06_rewards as v06  # noqa: E402

_spec = importlib.util.spec_from_file_location('feasibility', ROOT / 'scripts' / 'audit_highway_flow_rl_v06_merge_target.py')
feasibility = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(feasibility)

GAMMA, GAP_WEIGHT = .99, 2.
STEP_S = .1


def best_tracking_cost(targets, speed0, accel0, *, accel_min=-4.5, accel_max=2., jerk_max=4.,
                       speed_step=.05, accel_step=.1, speed_max=27.):
    """Minimum sum of min(1, |v_{k+1}-target_k|/2) over acceleration/jerk-limited trajectories.

    Same grid and dynamics as the Phase 0a feasibility DP; jerk 4 m/s3 is the
    v06 whole-episode command rate limit. Ineligible targets cost nothing.
    """
    speeds = np.arange(0., speed_max + 1e-9, speed_step)
    accels = np.arange(accel_min, accel_max + 1e-9, accel_step)
    nv, na = len(speeds), len(accels)
    reach = int(round(jerk_max * STEP_S / accel_step + 1e-9))
    inf = np.inf
    cost = np.full((nv, na), inf)
    cost[int(round(min(max(speed0, 0.), speed_max) / speed_step)),
         int(round((min(max(accel0, accel_min), accel_max) - accel_min) / accel_step))] = 0.
    offsets = np.rint(accels * STEP_S / speed_step).astype(int)
    for target in targets:
        spread = np.full((nv, na), inf)
        for shift in range(-reach, reach + 1):
            shifted = np.full((nv, na), inf)
            if shift >= 0:
                shifted[:, shift:] = cost[:, :na - shift]
            else:
                shifted[:, :na + shift] = cost[:, -shift:]
            spread = np.minimum(spread, shifted)
        moved = np.full((nv, na), inf)
        for j, offset in enumerate(offsets):
            column = spread[:, j]
            if offset > 0:
                moved[offset:, j] = column[:nv - offset]
                moved[-1, j] = min(moved[-1, j], column[nv - offset:].min())
            elif offset < 0:
                moved[:nv + offset, j] = column[-offset:]
                moved[0, j] = min(moved[0, j], column[:-offset].min())
            else:
                moved[:, j] = column
        if target is not None:
            step = np.minimum(1., np.abs(speeds - target) / v06.SETTINGS['tracking_tolerance_m_s'])[:, None]
            moved = moved + step
        cost = moved
    return float(cost.min())


def percentiles(values):
    values = np.abs(np.asarray(values, float))
    if not len(values):
        return None
    return dict(n=int(len(values)), p50=float(np.percentile(values, 50)), p95=float(np.percentile(values, 95)),
                p99=float(np.percentile(values, 99)), max=float(values.max()))


def car_rows(trace):
    rows = [json.loads(line) for line in gzip.open(trace, 'rt', encoding='utf-8')]
    cars = {}
    for row in rows:
        for agent, record in row['agents'].items():
            cars.setdefault(agent, []).append(record)
    return cars


def audit_car(records):
    """Per-decision v06 terms (unit weights for the new costs) for one car."""
    merge_agent = any(abs(r['reward_terms'].get('gap_preparation', 0.)) > 0. for r in records)
    run, steps = None, []
    for record in records:
        audit, v05 = record['v04_reward_audit'], record['v05_audit']
        pre = audit['pre_action']
        plan, successor = pre['gap_plan'], audit['successor_gap_plan']
        if run is None:
            run = int(bool(plan.get('required')))
        before = v06.blend_potential(plan, run)['potential']
        run = run + 1 if successor.get('required') else 0
        terminal = bool(record.get('route_completed'))
        after = 0. if terminal else v06.blend_potential(successor, run)['potential']
        blended = GAP_WEIGHT * (GAMMA * after - before) if merge_agent else 0.
        front = (pre['context'].get('corridors', {}).get('own', {}) or {}).get('front')
        conditioning = v05['conditioning']
        merge_active = bool(v05.get('merge_active'))
        target = v05['legacy_acceptance_target']
        base = {k: v for k, v in record['reward_terms'].items() if k != 'gap_preparation'}
        steps.append(dict(
            time_s=pre['time_s'], merge_active=merge_active, merge_agent=merge_agent,
            recorded_gap=record['reward_terms'].get('gap_preparation', 0.), blended_gap=blended,
            base=sum(base.values()), speed_error=record['reward_terms'].get('speed_error', 0.),
            progress=record['reward_terms'].get('progress', 0.),
            tracking=v06.merge_tracking(target, audit['measured_speed_m_s'], merge_active, 1.),
            jerk=v06.jerk_excess(record['realized_jerk_m_s3'], conditioning['raw_requested_jerk'], 1.),
            raw_jerk=abs(conditioning['raw_requested_jerk']),
            braking=v06.avoidable_braking(audit.get('avoidable_braking') or {}, conditioning['requested'],
                                          pre['ego_state']['speed'], front, 1.),
            braking_request=conditioning['requested'] < v06.SETTINGS['braking_request_m_s2'],
            merge_onset=bool(plan.get('required')) and run <= 2))
    return steps


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='append', required=True)
    parser.add_argument('--feasibility', required=True, help='Phase 0a merge-target-feasibility.json')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    feasible = {(c['run'], c['case_id'], c['agent']): c for c in
                json.loads(Path(args.feasibility).read_text(encoding='utf-8'))['cars']}
    cars, all_steps, traces = [], [], []
    for run in args.run:
        for trace in sorted(Path(run).glob('evaluation/case-*/*/actions.jsonl.gz')):
            case = json.loads((trace.parents[1].parent / f"{trace.parents[1].name.split('-')[1]}.json").read_text())
            traces.append(dict(path=str(trace).replace('\\', '/'), sha256=hashlib.sha256(trace.read_bytes()).hexdigest()))
            windows = feasibility.merge_windows(trace)
            for agent, records in car_rows(trace).items():
                steps = audit_car(records)
                all_steps += steps
                row = dict(run=Path(run).name, case_id=case['case_id'], agent=agent, decisions=len(steps),
                           merge_agent=steps[0]['merge_agent'] if steps else False,
                           totals={k: float(sum(s[k] for s in steps)) for k in
                                   ('base', 'progress', 'speed_error', 'recorded_gap', 'blended_gap', 'tracking', 'jerk', 'braking')},
                           pending_decisions=sum(s['merge_active'] for s in steps),
                           peak_raw_requested_jerk=max((s['raw_jerk'] for s in steps), default=0.),
                           braking_charged=sum(s['braking'] < 0 for s in steps),
                           braking_requests=sum(s['braking_request'] for s in steps))
                window = windows.get(agent)
                if window:
                    targets = [s['target'] for s in window['window']]
                    first = window['window'][0]
                    speed0 = first['speed'] if first['speed'] is not None else first['measured_speed']
                    observed = sum(min(1., abs(s['measured_speed'] - s['target']) / 2.) for s in window['window']
                                   if s['target'] is not None and s['measured_speed'] is not None)
                    best = best_tracking_cost(targets, speed0, first['accel'] or 0.)
                    meta = feasible.get((row['run'], case['case_id'], agent), {})
                    row['merge_window'] = dict(samples=len(targets), observed_tracking_units=observed,
                        best_tracking_units=best, improvement_units=observed - best,
                        observed_match=meta.get('observed_speed_match'),
                        ceiling_match=(meta.get('ceilings') or {}).get('gate', {}).get('best_fraction'))
                cars.append(row)
    merge_steps = [s for s in all_steps if s['merge_agent']]
    shaping = dict(recorded=percentiles([s['recorded_gap'] for s in merge_steps]),
                   blended=percentiles([s['blended_gap'] for s in merge_steps]),
                   recorded_at_onset=percentiles([s['recorded_gap'] for s in merge_steps if s['merge_onset']]),
                   blended_at_onset=percentiles([s['blended_gap'] for s in merge_steps if s['merge_onset']]))
    unit_jerk_8 = -v06.jerk_excess(8., 0., 1.)
    windows = [c['merge_window'] for c in cars if c.get('merge_window')]
    report = dict(schema='highway_flow_rl_v06_reward_audit', gamma=GAMMA, gap_weight=GAP_WEIGHT,
                  caveat=__doc__.strip().split('\n\n')[0], traces=traces, shaping_per_step=shaping,
                  unit_jerk_cost_at_8=unit_jerk_8,
                  unit_terms_all_nonpositive=dict(tracking=all(s['tracking'] <= 0 for s in all_steps),
                      jerk=all(s['jerk'] <= 0 for s in all_steps), braking=all(s['braking'] <= 0 for s in all_steps)),
                  merge_windows=dict(count=len(windows),
                      improvement_units=[w['improvement_units'] for w in windows]),
                  per_step_progress=percentiles([s['progress'] for s in all_steps]),
                  weights=dict(v06.WEIGHTS), weights_status=v06.WEIGHTS_STATUS,
                  weighted_new_terms_per_car=[dict(case_id=c['case_id'], agent=c['agent'],
                      merge_tracking=v06.WEIGHTS['merge_tracking'] * c['totals']['tracking'],
                      jerk_excess=v06.WEIGHTS['jerk_excess'] * c['totals']['jerk'],
                      avoidable_braking=v06.WEIGHTS['avoidable_braking'] * c['totals']['braking'],
                      base=c['totals']['base'], progress=c['totals']['progress'])
                      for c in cars if c['merge_agent'] or c['totals']['jerk'] or c['totals']['braking']],
                  cars=cars)
    Path(args.output).write_text(json.dumps(report, indent=1), encoding='utf-8')
    print(json.dumps({k: v for k, v in report.items() if k not in ('cars', 'traces')}, indent=1))
    for c in cars:
        w = c.get('merge_window')
        if c['merge_agent'] or c['braking_charged'] or c['peak_raw_requested_jerk'] > 5:
            print(c['run'], c['case_id'], c['agent'], 'pending', c['pending_decisions'],
                  {k: round(v, 2) for k, v in c['totals'].items()}, 'rawjerk %.1f' % c['peak_raw_requested_jerk'],
                  'brake %d/%d' % (c['braking_charged'], c['braking_requests']),
                  w and {k: (round(v, 2) if isinstance(v, float) else v) for k, v in w.items()})


if __name__ == '__main__':
    main()
