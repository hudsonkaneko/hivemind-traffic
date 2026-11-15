"""Diagnose v06 round 5 from its saved validation episodes and update records.

Read-only: no simulation, no training, no ledger charge. Answers:
1. Where do raw-request jerk check failures come from? Each decision with
   |request - measured| > 0.5 m/s2 (5 m/s3) belongs to a lag run, attributed
   to what started it: the car's first decision (inserted at 0 m/s2), the actor
   moving its own command by > 0.5 m/s2, measured acceleration leaving the last
   applied command, or gradual drift.
2. What caused realized jerk above 5 m/s3?
3. Which guards intervened, per case?
4. How would the gates count if the first decision were excluded from the
   raw-request check (diagnostic only, not a rule change)?
5. PPO update statistics (KL early stopping, optimizer steps, value loss).
"""
import argparse
import glob
import gzip
import hashlib
import json
import os
import statistics as st
import sys
from collections import Counter, defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from experiments.run_highway_flow_rl_v05 import development_gates  # noqa: E402

LABELS = ('initial-validation', 'periodic-validation-00042', 'periodic-validation-00078', 'final-validation')
STEP = .1
LIMIT = .5  # m/s2 per decision = 5 m/s3


def episode_steps(run, label, case):
    path = glob.glob(os.path.join(run, label, case, 'episode-*', 'actions.jsonl.gz'))[0]
    with gzip.open(path, 'rt') as f:
        for line in f:
            yield json.loads(line)


def analyze_case(run, label, case):
    prev, cars, realized = {}, {}, Counter()
    for step in episode_steps(run, label, case):
        for agent, d in step['agents'].items():
            audit = d.get('v05_audit')
            if not audit:
                continue
            c = audit['conditioning']
            req, meas = c['requested'], c['measured']
            s = cars.setdefault(agent, dict(decisions=0, over=0, origin=None, peak_raw=0., peak_jump=0.,
                                            peak_by_origin=defaultdict(float), over_by_origin=Counter()))
            s['decisions'] += 1
            p = prev.get(agent)
            raw = abs(req - meas)
            s['peak_raw'] = max(s['peak_raw'], raw / STEP)
            if p:
                s['peak_jump'] = max(s['peak_jump'], abs(req - p['req']) / STEP)
            if raw > LIMIT + 1e-9:
                if s['origin'] is None:
                    s['origin'] = ('first_decision' if p is None else
                                   'request_jump' if abs(req - p['req']) > LIMIT else
                                   'measured_departure' if abs(meas - p['applied']) > LIMIT else 'drift')
                s['over'] += 1
                s['over_by_origin'][s['origin']] += 1
                s['peak_by_origin'][s['origin']] = max(s['peak_by_origin'][s['origin']], raw / STEP)
            else:
                s['origin'] = None
            prev[agent] = dict(req=req, applied=c['expected_applied'])
            jerk = d.get('realized_jerk_m_s3')
            if jerk is not None and abs(jerk) > 5 + 1e-9:
                causes = [x for x in d.get('override_causes') or [] if x != 'comfort_rate_limit']
                if c['safety_exception']:
                    key = 'safety guard below rate-limited command: ' + ','.join(causes or ['ceiling'])
                elif d.get('native_realization_changed'):
                    key = 'native SUMO realization'
                else:
                    key = 'other: ' + ','.join(causes or ['none'])
                realized[key] += 1
    out = {}
    for agent, s in cars.items():
        s.pop('origin')
        s['peak_by_origin'] = {k: round(v, 2) for k, v in s['peak_by_origin'].items()}
        s['over_by_origin'] = dict(s['over_by_origin'])
        s['peak_raw'], s['peak_jump'] = round(s['peak_raw'], 2), round(s['peak_jump'], 2)
        s['peak_raw_excluding_first_decision'] = max(
            [v for k, v in s['peak_by_origin'].items() if k != 'first_decision'] or [0.])
        out[agent] = s
    return out, dict(realized)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True, help='outputs/highway-flow-rl/v06/round-05')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    run = str(args.run)
    result = dict(schema='highway_flow_rl_v06_round05_diagnosis_v01', run=args.run.as_posix(),
                  limit_m_s3=LIMIT / STEP, validations={}, inputs={})
    for label in LABELS:
        cases, why_fail, realized, overrides = [], Counter(), Counter(), Counter()
        counts = Counter()
        for i in range(8):
            report_path = os.path.join(run, label, f'{i:02d}.json')
            result['inputs'][f'{label}/{i:02d}.json'] = hashlib.sha256(Path(report_path).read_bytes()).hexdigest()
            report = json.loads(Path(report_path).read_text())
            gates = development_gates(report)
            cars, real = analyze_case(run, label, f'case-{i:02d}')
            realized.update(real)
            overrides.update(report['override_causes'])
            assigned = [a for a, c in report['smooth_control']['per_car'].items() if c.get('assigned_controlled')]
            raw_ok = all(cars[a]['peak_raw'] <= 5 + 1e-9 for a in assigned)
            raw_ok_ex = all(cars[a]['peak_raw_excluding_first_decision'] <= 5 + 1e-9 for a in assigned)
            for a in assigned:
                if cars[a]['peak_raw'] > 5 + 1e-9:
                    counts['raw_failing_cars'] += 1
                    failing = sorted(k for k, v in cars[a]['peak_by_origin'].items() if v > 5 + 1e-9)
                    why_fail['+'.join(failing)] += 1
                counts['cars'] += 1
            counts['original'] += gates['passed']
            counts['v06'] += gates['passed'] and raw_ok
            counts['v06_excluding_first_decision'] += gates['passed'] and raw_ok_ex
            quality = (report.get('quality_merges') or {})
            cases.append(dict(case=report['case_id'], task=report['task'], controlled=report['controlled_count'],
                original_gate_failures=gates['failures'], raw_check_passed=raw_ok,
                raw_check_passed_excluding_first_decision=raw_ok_ex,
                intervention_fraction=round(report['intervention_fraction'], 4),
                override_causes=report['override_causes'],
                mean_fraction_of_speed_limit=round(report['mean_fraction_of_speed_limit'], 4),
                merge_speed_match={a: (quality.get(a) or {}).get('speed_match_fraction') for a in report['merge_target_ids']},
                merge_stop_seconds={a: (quality.get(a) or {}).get('stop_seconds') for a in report['merge_target_ids']},
                stable_merge={a: {k: report['stable_merge']['per_agent'][a].get(k) for k in
                              ('status', 'failure_reasons', 'lateral_merge_complete_time_s')}
                              for a in report['merge_target_ids'] if a in report['stable_merge'].get('per_agent', {})},
                cars=cars))
        result['validations'][label] = dict(gate_counts=dict(counts), raw_failure_origins=dict(why_fail),
            realized_jerk_over_5_causes=dict(realized), override_causes=dict(overrides), cases=cases)
    updates = [json.loads(Path(f).read_text()) for f in sorted(glob.glob(os.path.join(run, 'updates', '*.json')))]
    opt = [u['optimization'] for u in updates if not u['critic_only']]
    quarters = []
    for q in range(4):
        part = opt[q * len(opt) // 4:(q + 1) * len(opt) // 4]
        quarters.append({k: round(st.mean(x[k] for x in part), 4) for k in
                         ('approx_kl', 'last_kl', 'clip_fraction', 'value_loss', 'entropy', 'grad_norm_before_clip',
                          'optimizer_steps', 'completed_epochs')})
    result['ppo'] = dict(updates=len(opt), critic_only_updates=len(updates) - len(opt),
        early_stopped=sum(x['early_stopped'] for x in opt),
        optimizer_steps=dict(total=sum(x['optimizer_steps'] for x in opt),
                             median=st.median(x['optimizer_steps'] for x in opt),
                             max=max(x['optimizer_steps'] for x in opt)),
        completed_epochs=dict(median=st.median(x['completed_epochs'] for x in opt),
                              distribution=dict(sorted(Counter(x['completed_epochs'] for x in opt).items()))),
        samples_per_update_median=st.median(x['active_samples'] for x in opt), quarters=quarters)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=1) + '\n', encoding='utf-8')
    for label, v in result['validations'].items():
        print(label, v['gate_counts'], v['raw_failure_origins'], v['override_causes'])
    print(json.dumps({k: v for k, v in result['ppo'].items() if k != 'quarters'}))


if __name__ == '__main__':
    main()
