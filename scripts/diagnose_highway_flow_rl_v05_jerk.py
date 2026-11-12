"""Attribute peak realized jerk in saved v04 traces; no simulation or training.

Reads the per-decision action journals written by the v04 final validation and
classifies every sample whose |realized jerk| exceeds the acceptance limit.
Each sample receives exactly one primary cause, checked in this order:

1. ``startup``          first decision after the car is controlled
2. ``safety_override``  shield/native guard changed the command
3. ``policy_filter``    the policy-side filter changed the request
4. ``native_realization`` SUMO realized a different acceleration than applied
5. ``override_release`` the previous step was overridden and control returned
6. ``lane_change``      a lateral maneuver is in progress or just completed
7. ``policy_request``   the actor's own requested acceleration swung
8. ``other``            none of the above (should be rare; inspect manually)

The classification is descriptive evidence about saved trajectories. It does
not prove causality or predict how a changed policy would behave.
"""
import argparse
import gzip
import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RUN = Path('outputs/highway-flow-rl/v04/long-round-03')
JERK_LIMIT = 5.0
STEP_S = 0.1


def classify(sample, previous, first):
    """Return the primary cause for one high-jerk sample."""
    if first or previous is None:
        return 'startup'
    if sample.get('override'):
        return 'safety_override'
    if sample.get('policy_filter_changed'):
        return 'policy_filter'
    if sample.get('native_realization_changed'):
        return 'native_realization'
    if previous.get('override') or previous.get('native_realization_changed'):
        # SUMO hands control back: realized acceleration jumps from the
        # native/override value to the policy request in one step.
        return 'override_release'
    maneuver = sample.get('maneuver') or {}
    if maneuver.get('in_progress') or maneuver.get('completed_this_step') or sample.get('lane_change_observed'):
        return 'lane_change'
    if abs(sample['requested_acceleration'] - previous['requested_acceleration']) * 1.0 >= JERK_LIMIT * STEP_S:
        return 'policy_request'
    return 'other'


def active_bound(sample):
    bounds = {k: v for k, v in (sample.get('acceleration_bounds') or {}).items() if v is not None}
    return min(bounds, key=bounds.get) if bounds else None


def analyse_case(path):
    rows = [json.loads(line) for line in gzip.open(path, 'rt')]
    previous, seen = {}, set()
    cars = defaultdict(lambda: dict(samples=0, peaks=[]))
    for row in rows:
        for agent, s in row['agents'].items():
            jerk = s.get('realized_jerk_m_s3')
            if jerk is None:
                continue
            car = cars[agent]
            car['samples'] += 1
            first = agent not in seen
            seen.add(agent)
            if abs(jerk) > JERK_LIMIT:
                prev = previous.get(agent)
                car['peaks'].append(dict(
                    time_s=row['time'], jerk_m_s3=jerk,
                    cause=classify(s, prev, first),
                    requested=s['requested_acceleration'],
                    previous_requested=prev['requested_acceleration'] if prev else None,
                    realized=s['realized_acceleration'],
                    previous_realized=prev['realized_acceleration'] if prev else None,
                    override_causes=s.get('override_causes') or [],
                    native_cause=s.get('native_cause'),
                    active_bound=active_bound(s),
                    merge_completed=s.get('merge_completed'),
                    route_completed=s.get('route_completed'),
                    stable_merge_status=s.get('stable_merge_status')))
            previous[agent] = s
    return rows, dict(cars)


def summarise(cases):
    causes, cause_peak = Counter(), {}
    for case in cases:
        for car in case['cars'].values():
            for p in car['peaks']:
                causes[p['cause']] += 1
                cause_peak[p['cause']] = max(cause_peak.get(p['cause'], 0.), abs(p['jerk_m_s3']))
    failing = [dict(case_id=c['case_id'], agent=a, peaks=len(car['peaks']),
                    max_abs_jerk_m_s3=max(abs(p['jerk_m_s3']) for p in car['peaks']),
                    causes=dict(Counter(p['cause'] for p in car['peaks'])))
               for c in cases for a, car in c['cars'].items() if car['peaks']]
    failing.sort(key=lambda f: -f['max_abs_jerk_m_s3'])
    startup_only = [f for f in failing if set(f['causes']) == {'startup'}]
    policy_owned = [f for f in failing if set(f['causes']) & {'policy_request', 'lane_change'}]
    native_owned = [f for f in failing if set(f['causes']) & {'safety_override', 'native_realization', 'override_release'}]
    return dict(jerk_limit_m_s3=JERK_LIMIT, peak_samples=sum(causes.values()),
                failing_car_count=len(failing), failing_only_at_startup=len(startup_only),
                failing_with_policy_owned_peak=len(policy_owned),
                failing_with_native_or_override_peak=len(native_owned),
                peaks_by_cause=dict(causes.most_common()),
                max_abs_jerk_by_cause=cause_peak, failing_cars=failing)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--run', type=Path, default=DEFAULT_RUN)
    parser.add_argument('--output', type=Path,
                        default=Path('outputs/highway-flow-rl/v05/jerk-diagnosis-01/jerk-diagnosis.json'))
    parser.add_argument('--summary-file', default='final-validation.json',
                        help='Run-relative JSON with summary.case_gates (summary.json for evaluate runs)')
    parser.add_argument('--cases-dir', default='final-validation',
                        help='Run-relative directory holding case-NN/episode-*/actions.jsonl.gz')
    args = parser.parse_args(argv)
    run = args.run if args.run.is_absolute() else ROOT / args.run
    validation = json.loads((run / args.summary_file).read_text())
    gates = {g['case_id']: g for g in validation['summary']['case_gates']}
    cases = []
    for path in sorted((run / args.cases_dir).glob('case-*/*/actions.jsonl.gz')):
        rows, cars = analyse_case(path)
        index = int(path.parent.parent.name.split('-')[1])
        case_id = validation['summary']['case_gates'][index]['case_id']
        cases.append(dict(case_id=case_id, seed=gates[case_id]['seed'],
                          trace=str(path.relative_to(run)).replace('\\', '/'),
                          trace_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                          decisions=len(rows), cars=cars))
    report = dict(schema='highway_flow_rl_v05_jerk_diagnosis', training=False, simulation_run=False,
                  source_run=str(args.run).replace('\\', '/'), summary=summarise(cases), cases=cases,
                  limitation='Descriptive attribution on saved trajectories; not a causal or counterfactual claim.')
    out = args.output if args.output.is_absolute() else ROOT / args.output
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2))
    print(json.dumps(report['summary'], indent=2))
    return report


if __name__ == '__main__':
    main()
