"""Summarize one saved validation (or evaluation) of the highway-flow merge study per case and per merging car.

Reads only saved files: the validation JSON written by the shared runner and each
case's action journal. For every merging car it reports the rolling merge-quality
conditions one by one (speed match, stops, jerk, overrides, neighbor braking), so a
failed `rolling_merge_quality` gate can be traced to its cause. No simulation.
"""
import argparse
import glob
import gzip
import json
from pathlib import Path


def quality_conditions(samples):
    """The gate's own conditions (traffic.highway_flow_rl_v04_rewards.quality_merge_eligible), itemized."""
    n = len(samples)
    return dict(samples=n,
        speed_match_fraction=sum(s['speed_match'] for s in samples) / n if n else None,
        stopped_seconds=sum(s['stopped'] for s in samples) * .1,
        peak_abs_jerk_m_s3=max((abs(s['jerk']) for s in samples if s['jerk'] is not None), default=None),
        override_fraction=sum(s['override'] for s in samples) / n if n else None,
        neighbor_braking_samples=sum(s['neighbor_acceleration'] is not None and s['neighbor_acceleration'] < -3. for s in samples),
        minimum_neighbor_acceleration_m_s2=min((s['neighbor_acceleration'] for s in samples
                                                if s['neighbor_acceleration'] is not None), default=None))


def car_summary(report, journal, car):
    stable = (report.get('stable_merge') or {}).get('per_agent', {}).get(car) or {}
    quality = (report.get('quality_merges') or {}).get(car) or {}
    done, entry = stable.get('lateral_merge_complete_time_s'), stable.get('entry_time_s')
    rows = [(r['time'], r['agents'][car]) for r in journal if car in r['agents']]
    result = dict(stable_merge=stable.get('stable_merge_succeeded'), qualified=quality.get('qualified'),
                  speed_match_fraction=quality.get('speed_match_fraction'), entry_time_s=entry,
                  lateral_merge_complete_time_s=done, first_decision_time_s=rows[0][0] if rows else None,
                  v11=(report.get('v11_metrics') or {}).get('per_car', {}).get(car))
    request = next(((t, a) for t, a in rows if a['lane_applied_delta']), None)
    if request:
        t, a = request
        pre = a['v04_reward_audit']['pre_action']
        target = a['v05_audit']['legacy_acceptance_target']
        rear = (pre['context'].get('entry') or {}).get('rear') or {}
        result['merge_request'] = dict(time_s=t, speed_m_s=pre['ego_state']['speed'],
            target_m_s=target.get('target_m_s') if target.get('eligible') else None,
            lane=pre['ego_state']['lane_id'], rear_gap_m=rear.get('bumper_gap_m'),
            rear_closing_m_s=rear.get('relative_speed_m_s'))
    if done is not None:
        window = [a['v04_reward_audit']['quality_sample'] for t, a in rows
                  if done - 8 - 1e-6 <= a['v04_reward_audit']['quality_sample']['time_s'] <= done + 8 + 1e-6]
        result['window'] = quality_conditions(window)
        missed = [(t, a) for t, a in rows if done - 8 - 1e-6 <= a['v04_reward_audit']['quality_sample']['time_s'] <= done + 8 + 1e-6
                  and not a['v04_reward_audit']['quality_sample']['speed_match']]
        below = above = ineligible = 0
        for t, a in missed:
            target, speed = a['v05_audit']['legacy_acceptance_target'], a['v04_reward_audit']['measured_speed_m_s']
            if not target.get('eligible') or speed is None:
                ineligible += 1
            elif speed < target['target_m_s']:
                below += 1
            else:
                above += 1
        result['window'].update(missed_below_target=below, missed_above_target=above, missed_target_not_eligible=ineligible,
            missed_times_s=[round(t, 1) for t, _ in missed][:1] + [round(t, 1) for t, _ in missed][-1:])
        post = [a for t, a in rows if done - 1e-6 <= t <= done + 8 + 1e-6]
        result['post_merge_override_decisions'] = sum(bool(a['override']) for a in post)
    return result


def summarize(directory, label):
    directory = Path(directory)
    data = json.loads((directory / f'{label}.json').read_text(encoding='utf-8'))
    cases = []
    for index, (report, gate) in enumerate(zip(data['reports'], data['summary']['case_gates'])):
        journals = glob.glob(str(directory / label / f'case-{index:02d}' / '*' / 'actions.jsonl.gz'))
        journal = [json.loads(line) for line in gzip.open(journals[0], 'rt')] if len(journals) == 1 else []
        cases.append(dict(index=index, case_id=report['case_id'], seed=report['seed'], gate_passed=gate['passed'],
            gate_failures=gate['failures'], collisions=report.get('collisions'),
            intervention_fraction=report.get('intervention_fraction'), peak_abs_jerk=report.get('peak_abs_jerk'),
            cars={car: car_summary(report, journal, car) for car in report.get('merge_target_ids') or []}))
    matches = [(car.get('speed_match_fraction') or 0.) if car.get('stable_merge') else 0.
               for case in cases for car in case['cars'].values()]
    summary = data['summary']
    return dict(schema='highway_flow_rl_v11_validation_summary_v01', source=(directory / f'{label}.json').as_posix(),
        gates_passed=sum(c['gate_passed'] for c in cases), cases_total=len(cases),
        pooled_merge_speed_match=sum(matches) / len(matches) if matches else None,
        stable_merges=summary.get('stable_merges'), merge_targets=summary.get('merge_targets'),
        collisions=summary.get('collisions'), pooled_intervention_fraction=summary.get('pooled_intervention_fraction'),
        max_peak_jerk_m_s3=summary.get('max_peak_jerk_m_s3'), cases=cases)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path, help='run output directory')
    parser.add_argument('--label', default='evaluation', help='evaluation, initial-validation, final-validation, ...')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    result = summarize(args.directory, args.label)
    if args.output:
        args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(f"gates {result['gates_passed']}/{result['cases_total']}  pooled merge match {result['pooled_merge_speed_match']:.3f}  "
          f"stable merges {result['stable_merges']}/{result['merge_targets']}  collisions {result['collisions']}  "
          f"interventions {result['pooled_intervention_fraction']:.4f}  peak jerk {result['max_peak_jerk_m_s3']:.2f}")
    for case in result['cases']:
        print(f"[{case['index']}] {case['case_id']}: {'PASS' if case['gate_passed'] else 'fail ' + ','.join(case['gate_failures'])}")
        for name, car in case['cars'].items():
            request, window, rule = car.get('merge_request') or {}, car.get('window') or {}, car.get('v11') or {}
            fmt = lambda v, spec='.2f': '-' if v is None else format(v, spec)  # noqa: E731
            print(f"     {name}: match {fmt(car['speed_match_fraction'], '.3f')} stable {car['stable_merge']} qualified {car['qualified']} | "
                  f"request at {fmt(request.get('time_s'), '.1f')}s v={fmt(request.get('speed_m_s'))} target={fmt(request.get('target_m_s'))} "
                  f"({rule.get('started_by')}, blocked {rule.get('blocked_decisions')}) merged {fmt(car['lateral_merge_complete_time_s'], '.1f')}s")
            if window:
                print(f"        window n={window['samples']} missed: below {window['missed_below_target']} above {window['missed_above_target']} "
                      f"not-eligible {window['missed_target_not_eligible']} {window['missed_times_s']} | stops {window['stopped_seconds']:.1f}s "
                      f"jerk {fmt(window['peak_abs_jerk_m_s3'])} overrides {fmt(window['override_fraction'], '.3f')} "
                      f"neighbor braking samples {window['neighbor_braking_samples']} (min {fmt(window['minimum_neighbor_acceleration_m_s2'])}) "
                      f"post-merge overrides {car.get('post_merge_override_decisions')}")


if __name__ == '__main__':
    main()
