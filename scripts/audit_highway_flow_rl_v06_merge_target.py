"""V06 Phase 0a: can a smooth car reach 90% merge speed matching against the gate's target?

Offline only: reads saved v05 evaluation traces, runs no simulation and charges
no episodes. For every merging car it rebuilds the merge-quality window, then
computes the best speed-match fraction any car could reach against the SAME
recorded target sequence while respecting acceleration and jerk limits.

Open-loop caveat: the recorded target depends on the car's own past motion and
neighbors. A different trajectory would see a somewhat different target, so the
ceiling is an estimate for this target sequence, not a proof.
"""
import argparse
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np

STEP_S = .1
TOLERANCE_M_S = 2.          # gate: |speed - target| <= 2 m/s
WINDOW_S = 8.               # gate: samples from completion-8 s onward
LIMITS = dict(
    gate=dict(accel_min=-4.5, accel_max=2., jerk_max=5.),      # vehicle limits, gate jerk
    comfort=dict(accel_min=-2., accel_max=1.5, jerk_max=4.),   # v05 preparation bounds, comfort jerk
)


def best_match_fraction(targets, speed0, accel0, *, accel_min, accel_max, jerk_max,
                        speed_step=.05, accel_step=.1, speed_max=27.):
    """Max count of steps k with |v_{k+1} - target_k| <= tolerance (target None never matches).

    Dynamic program on a (speed, acceleration) grid; acceleration may change by at
    most jerk_max*STEP_S per step. Returns (fraction, count).
    """
    speeds = np.arange(0., speed_max + 1e-9, speed_step)
    accels = np.arange(accel_min, accel_max + 1e-9, accel_step)
    nv, na = len(speeds), len(accels)
    reach = int(round(jerk_max * STEP_S / accel_step + 1e-9))
    best = np.full((nv, na), -1, dtype=np.int32)
    best[int(round(min(max(speed0, 0.), speed_max) / speed_step)),
         int(round((min(max(accel0, accel_min), accel_max) - accel_min) / accel_step))] = 0
    for target in targets:
        # Next acceleration index within the jerk limit of the current one.
        spread = np.full((nv, na), -1, dtype=np.int32)
        for shift in range(-reach, reach + 1):
            shifted = np.full((nv, na), -1, dtype=np.int32)
            if shift >= 0:
                shifted[:, shift:] = best[:, :na - shift]
            else:
                shifted[:, :na + shift] = best[:, -shift:]
            spread = np.maximum(spread, shifted)
        # Apply the new acceleration for one step: speed index moves by a*dt/dv.
        moved = np.full((nv, na), -1, dtype=np.int32)
        offsets = np.rint(accels * STEP_S / speed_step).astype(int)
        for j, offset in enumerate(offsets):
            column = spread[:, j]
            if offset >= 0:
                moved[offset:, j] = column[:nv - offset] if offset else column
                if offset:
                    moved[-1, j] = max(moved[-1, j], column[nv - offset:].max())
            else:
                moved[:nv + offset, j] = column[-offset:]
                moved[0, j] = max(moved[0, j], column[:-offset].max())
        if target is not None:
            hit = (np.abs(speeds - target) <= TOLERANCE_M_S + 1e-9)[:, None]
            moved = np.where((moved >= 0) & hit, moved + 1, moved)
        best = moved
    count = int(best.max())
    return (count / len(targets) if targets else 0.), count


def merge_windows(path):
    rows = [json.loads(line) for line in gzip.open(path, 'rt', encoding='utf-8')]
    cars = {}
    for row in rows:
        for agent, record in row['agents'].items():
            audit = record.get('v04_reward_audit') or {}
            sample, pre = audit.get('quality_sample'), audit.get('pre_action') or {}
            target = (record.get('v05_audit') or {}).get('legacy_acceptance_target') or {}
            if sample is None:
                continue
            car = cars.setdefault(agent, dict(samples=[], merge_active=False, completed_at=None, stable_at=None))
            car['merge_active'] |= bool((record.get('v05_audit') or {}).get('merge_active'))
            car['samples'].append(dict(time_s=sample['time_s'], speed_match=bool(sample['speed_match']),
                target=target.get('target_m_s') if target.get('eligible') else None,
                speed=(pre.get('ego_state') or {}).get('speed'), accel=record.get('realized_acceleration'),
                measured_speed=audit.get('measured_speed_m_s')))
            if record.get('merge_completed') and car['completed_at'] is None:
                car['completed_at'] = sample['time_s']
            if record.get('stable_merge_status') not in (None, 'pending') and car['stable_at'] is None \
                    and car['completed_at'] is not None:
                car['stable_at'] = sample['time_s']
    result = {}
    for agent, car in cars.items():
        if not car['merge_active'] or car['completed_at'] is None:
            continue
        end = car['stable_at'] if car['stable_at'] is not None else car['samples'][-1]['time_s']
        window = [s for s in car['samples'] if car['completed_at'] - WINDOW_S - 1e-9 <= s['time_s'] <= end + 1e-9]
        result[agent] = dict(window=window, completed_at=car['completed_at'], stable_at=car['stable_at'])
    return result


def audit_car(window):
    targets = [s['target'] for s in window]
    first = window[0]
    jumps = [abs(b - a) for a, b in zip(targets, targets[1:]) if a is not None and b is not None]
    row = dict(samples=len(window), observed_speed_match=sum(s['speed_match'] for s in window) / len(window),
               ineligible_target_samples=sum(t is None for t in targets),
               target_step_change_m_s=dict(max=max(jumps, default=0.),
                   over_0_2=sum(j > .2 + 1e-9 for j in jumps), over_1_0=sum(j > 1. + 1e-9 for j in jumps)),
               ceilings={})
    speed0 = first['speed'] if first['speed'] is not None else first['measured_speed']
    for name, limits in LIMITS.items():
        fraction, count = best_match_fraction(targets, speed0, first['accel'] or 0., **limits)
        row['ceilings'][name] = dict(best_fraction=fraction, best_count=count, limits=limits)
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='append', required=True, help='v05 evaluation run directory')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    report = dict(schema='highway_flow_rl_v06_merge_target_feasibility', tolerance_m_s=TOLERANCE_M_S,
                  window_s=WINDOW_S, required_fraction=.9, open_loop_caveat=__doc__.split('\n\n')[-1].strip(),
                  cars=[])
    for run in args.run:
        for trace in sorted(Path(run).glob('evaluation/case-*/*/actions.jsonl.gz')):
            case = json.loads((trace.parents[1].parent / f"{trace.parents[1].name.split('-')[1]}.json").read_text())
            for agent, car in merge_windows(trace).items():
                row = audit_car(car['window'])
                row.update(run=Path(run).name, case_id=case.get('case_id') or case.get('case', {}).get('case_id'),
                           agent=agent, completed_at_s=car['completed_at'], stable_at_s=car['stable_at'],
                           trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest())
                report['cars'].append(row)
    Path(args.output).write_text(json.dumps(report, indent=1), encoding='utf-8')
    for row in report['cars']:
        print(row['run'], row['case_id'], row['agent'], row['samples'],
              'observed %.3f' % row['observed_speed_match'],
              'gate-limit ceiling %.3f' % row['ceilings']['gate']['best_fraction'],
              'comfort ceiling %.3f' % row['ceilings']['comfort']['best_fraction'],
              'ineligible', row['ineligible_target_samples'], 'jumps', row['target_step_change_m_s'])


if __name__ == '__main__':
    main()
