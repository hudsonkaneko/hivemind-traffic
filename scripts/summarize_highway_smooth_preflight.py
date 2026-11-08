"""Offline gate summary and measured plots; never launches SUMO or trains.

Select the candidate study explicitly. Baselines resolve first from that study,
then from --baseline-studies in declared order, without inspecting outcomes.
An incomplete comparison is never a readiness pass. The native baseline is
context only, and this subset report cannot authorize learning.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

REQUIRED_FIXTURES = ('cruise', 'following', 'passing', 'merge', 'dense-following', 'i110-merge')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def project_path(path):
    path = Path(path).resolve()
    if not path.is_relative_to(ROOT):
        raise ValueError('Artifacts must remain inside the project checkout')
    return path


def relative(path):
    return project_path(path).relative_to(ROOT).as_posix()


def gate(passed, **evidence):
    return dict(status='incomplete' if passed is None else 'passed' if passed else 'failed', **evidence)


def mean_boundary_time(report):
    """Include all requested trips, with unfinished trips charged to horizon."""
    totals = report.get('accounting', {}).get('totals', {})
    requested, duration = totals.get('requested'), totals.get('boundary_time_s')
    if type(requested) is not int or requested <= 0 or not isinstance(duration, (int, float)) or not math.isfinite(duration):
        return None
    if requested != report.get('requested_trips') or totals.get('completed') != report.get('completed_trips'):
        return None
    return duration / requested


def measured_trace(recording, vehicle_id):
    result, previous = [], None
    for frame in recording['frames']:
        vehicle = next((v for v in frame['vehicles'] if v['id'] == vehicle_id), None)
        if vehicle is None:
            previous = None
            continue
        time = float(frame['time'])
        jerk = ((vehicle['acceleration'] - previous['acceleration']) / .1
                if previous and abs(time - previous['time_s'] - .1) < 1e-7 else None)
        row = dict(time_s=time, speed_m_s=vehicle['speed'], acceleration=vehicle['acceleration'],
                   jerk_m_s3=jerk, lane_id=vehicle['lane_id'],
                   lane_remaining_m=vehicle['lane_remaining_m'],
                   maneuver_in_progress=vehicle['maneuver_in_progress'])
        result.append(row)
        previous = row
    return result


def recording_audit(recording, report):
    from traffic.highway_smooth_metrics import nominal_merge_stop
    controlled = recording.get('controlled_ids', [])
    cars, all_jerk, startup = {}, [], []
    stops = []
    all_ids = sorted({v['id'] for frame in recording['frames'] for v in frame['vehicles']})
    for vehicle_id in all_ids:
        trace = measured_trace(recording, vehicle_id)
        values = [r['jerk_m_s3'] for r in trace if r['jerk_m_s3'] is not None]
        stopped, current = [], []
        for row in trace:
            if row['speed_m_s'] < .1:
                if current and row['time_s'] - current[-1]['time_s'] > .1000001:
                    stopped.append(current)
                    current = []
                current.append(row)
            elif current:
                stopped.append(current)
                current = []
        if current:
            stopped.append(current)
        for block in stopped:
            duration = len(block) * .1
            stops.append(dict(vehicle_id=vehicle_id, controlled=vehicle_id in controlled,
                start_s=block[0]['time_s'], last_sample_s=block[-1]['time_s'],
                sampled_duration_s=duration, observed_span_s=block[-1]['time_s'] - block[0]['time_s'],
                at_least_two_seconds=block[-1]['time_s'] - block[0]['time_s'] >= 2. - 1e-8,
                lane_id=block[0]['lane_id']))
        if vehicle_id in controlled:
            all_jerk.extend(values)
            if values:
                startup.append(values[0])
            cars[vehicle_id] = dict(measured_intervals=len(values),
                peak_abs_jerk_m_s3=max(map(abs, values)) if values else None,
                startup_jerk_m_s3=values[0] if values else None)
            if vehicle_id in report.get('merge_target_ids', []):
                stable_time = next((e['time'] for e in recording.get('stable_events', [])
                    if e['agent'] == vehicle_id and e['status'] == 'succeeded'), None)
                cars[vehicle_id]['nominal_merge_stop'] = nominal_merge_stop(trace, stable_time)
    observed_peak = max(map(abs, all_jerk)) if all_jerk else None
    reported = report.get('smooth_control', {}).get('controlled', {}).get('jerk_m_s3', {}).get('realized', {})
    matches = (observed_peak is not None and reported.get('peak_abs') is not None
               and math.isclose(observed_peak, reported['peak_abs'], rel_tol=1e-9, abs_tol=1e-7)
               and len(all_jerk) == reported.get('samples'))
    return dict(controlled_peak_abs_jerk_m_s3=observed_peak,
        controlled_measured_intervals=len(all_jerk),
        controlled_startup_peak_abs_jerk_m_s3=max(map(abs, startup)) if startup else None,
        reported_controlled_jerk_agrees=matches, per_controlled_car=cars,
        stopped_intervals=stops, stopped_intervals_at_least_two_seconds=sum(s['at_least_two_seconds'] for s in stops),
        stop_duration_definition='Sampled exposure is N*0.1; observed span is last-minus-first consecutive measured low-speed timestamps. The nominal stop gate uses observed span, with no inferred first/arrival interval.')


def load_run(path):
    path = project_path(path)
    if not (path / 'evaluation.json').is_file():
        return None
    report, manifest, check = (read(path / name) for name in ('evaluation.json', 'manifest.json', 'recording-check.json'))
    recording = read(path / 'recording.json')
    failures = []
    for name, artifact in [('report', report), ('manifest', manifest), ('check', check), ('recording', recording)]:
        if artifact.get('training') is not False or artifact.get('optimizer_steps') != 0:
            failures.append(name + '_is_not_zero_update')
    if check.get('error') or check.get('changed_sources') or check.get('policy_unchanged') is not True:
        failures.append('recording_execution_or_integrity')
    if sha(path / 'recording.json') != check.get('recording_sha256'):
        failures.append('recording_checksum')
    for name, digest in manifest.get('source_sha256', {}).items():
        source = path / 'source' / name
        if not source.is_file() or sha(source) != digest:
            failures.append('source_snapshot:' + name)
    if not manifest.get('source_sha256'):
        failures.append('missing_source_snapshot')
    if manifest.get('selected_case', {}).get('case', {}).get('split') != 'train':
        failures.append('not_training_split')
    audit = recording_audit(recording, report)
    if not audit['reported_controlled_jerk_agrees']:
        failures.append('recorded_and_reported_jerk_disagree')
    return dict(path=relative(path), report=report, manifest=manifest, check=check,
        audit=audit, integrity_failures=failures,
        hashes={name: sha(path / name) for name in ('evaluation.json', 'manifest.json', 'recording-check.json', 'recording.json')})


def matching(candidate, baseline):
    if baseline is None:
        return False
    return (candidate['manifest']['selected_case'] == baseline['manifest']['selected_case']
            and candidate['check']['demand_sha256'] == baseline['check']['demand_sha256']
            and candidate['manifest']['network_sha256'] == baseline['manifest']['network_sha256']
            and baseline['report'].get('primary_horizon_complete') is True
            and not baseline['integrity_failures'])


def observed_merge_coverage(report, vehicle_id):
    """Allow nonexistent pre-reset history, never missing existing observations."""
    car = report.get('smooth_control', {}).get('per_car', {}).get(vehicle_id, {})
    window = car.get('merge_slowdown', {})
    start, end = window.get('available_start_s'), window.get('available_end_s')
    wanted_start, wanted_end = window.get('requested_start_s'), window.get('requested_end_s')
    first = car.get('first_observed_time_s')
    numbers = (start, end, wanted_start, wanted_end, first)
    if any(not isinstance(n, (int, float)) or not math.isfinite(n) for n in numbers):
        return dict(usable=False, boundary_censored=False)
    boundary_censored = wanted_start < first - 1e-7 and math.isclose(start, first, abs_tol=1e-7)
    usable = (report.get('primary_horizon_complete') is True
              and report.get('accounting_comparison_valid') is True
              and window.get('nonconsecutive_gaps') == 0 and window.get('samples', 0) >= 2
              and end >= wanted_end - 1e-7 and (start <= wanted_start + 1e-7 or boundary_censored))
    return dict(usable=usable, boundary_censored=boundary_censored,
        full_requested_window_complete=window.get('coverage_complete') is True,
        explanation='Pre-reset history does not exist; use every available measured approach sample through completion+8s, retain left censoring explicitly')


def continuous_merge_gate(candidate):
    """Predeclared 2026-11-06 before probe03, in addition to original gates.

    Twenty measured 0.1-second intervals between consecutive low-speed samples
    constitute the gate. No unobserved onset/arrival interval is invented.
    Danger fixtures require safe yielding and do not use this nominal-only gate.
    """
    report, audit = candidate['report'], candidate['audit']
    targets = report.get('merge_target_ids', [])
    if not targets or 'per_controlled_car' not in audit:
        return gate(None, reason='Missing nominal merge target/stop evidence')
    metrics = {a: audit['per_controlled_car'].get(a, {}).get('nominal_merge_stop', {}) for a in targets}
    failures = [dict(vehicle_id=a, **m) for a, m in metrics.items()
                if m.get('longest_consecutive_stop_s', 0.) >= 2. - 1e-7]
    complete = all(m.get('coverage_complete') is True for m in metrics.values())
    return gate(False if failures else True if complete else None,
        target_ids=targets, failed_intervals=failures, per_target=metrics,
        threshold_m_s=.1, minimum_consecutive_intervals=20, sample_interval_s=.1,
        minimum_observed_span_s=2., scope='Nominal controlled merge targets through first stable completion inclusive; danger stops excluded from this criterion',
        introduced='2026-11-06 before entry-probe-03; original relative slowdown and trip/time bounds remain separately reported')


def drawdown_comparison(current, prior, current_coverage, prior_coverage):
    a, b = current.get('observed_loss_m_s'), prior.get('observed_loss_m_s')
    numeric = all(isinstance(v, (int, float)) and math.isfinite(v) and v >= 0 for v in (a, b))
    bound = a <= b + .1 if numeric else None
    covered = current_coverage['usable'] and prior_coverage['usable']
    # A complete candidate window below an observed baseline lower bound proves
    # the same inequality. Additional baseline samples cannot reduce its maximum
    # drawdown. This does not promote the baseline's missing post-window to full
    # coverage, and it cannot prove a failure when the lower bound is smaller.
    fields = ('requested_start_s', 'requested_end_s', 'available_start_s', 'available_end_s')
    finite_bounds = all(isinstance(prior.get(k), (int, float)) and math.isfinite(prior[k]) for k in fields)
    one_sided = bool(current.get('coverage_complete') is True and current_coverage['usable']
        and bound is True and finite_bounds and prior.get('samples', 0) >= 2
        and prior['requested_start_s'] <= prior['available_start_s'] + 1e-7
        and prior['available_start_s'] < prior['available_end_s']
        and prior['available_end_s'] < prior['requested_end_s'] - 1e-7)
    verdict = bound if covered else True if one_sided else None
    return dict(observed_bound_pass=bound, complete_available_coverage=covered,
        comparison_pass=verdict, proof_type='complete_available_windows' if covered else
        'one_sided_observed_baseline_lower_bound' if one_sided else 'incomplete',
        baseline_full_window_claim=prior.get('coverage_complete') is True,
        candidate_coverage=current_coverage, frozen_actor_coverage=prior_coverage)


def evaluate_candidate(candidate, teacher=None, actor=None):
    """Subset gates only: a failed/missing invariant or gradient proof stays external."""
    from experiments.highway_v05_support import acceptance
    report = candidate['report']
    try:
        original = acceptance([report])
        original_gate = gate(original['all_gates_passed'], unchanged_v05_result=original)
    except (ValueError, KeyError, TypeError) as error:
        original_gate = gate(None, error=str(error))
    peak = candidate['audit']['controlled_peak_abs_jerk_m_s3']
    gates = dict(integrity=gate(not candidate['integrity_failures'], failures=candidate['integrity_failures']),
        unchanged_v05_safety_and_task=original_gate,
        nominal_controlled_peak_jerk=gate(peak <= 5. + 1e-7 if peak is not None else None,
            measured_m_s3=peak, maximum_m_s3=5., startup_included=True,
            measured_intervals=candidate['audit']['controlled_measured_intervals']))
    teacher_matches = matching(candidate, teacher)
    actor_matches = matching(candidate, actor)
    completed = report.get('completed_trips')
    previous_completed = teacher['report'].get('completed_trips') if teacher_matches else None
    gates['completed_trips_vs_teacher'] = gate(completed >= previous_completed if type(completed) is int and type(previous_completed) is int else None,
        candidate=completed, baseline=previous_completed, matched=teacher_matches)
    boundary = mean_boundary_time(report)
    previous_boundary = mean_boundary_time(teacher['report']) if teacher_matches else None
    gates['boundary_time_vs_teacher'] = gate(boundary <= 1.05 * previous_boundary if boundary is not None and previous_boundary is not None else None,
        candidate_mean_s=boundary, teacher_mean_s=previous_boundary,
        maximum_ratio=1.05, matched=teacher_matches, unfinished_trips_included=True)
    if report.get('task') == 'merge':
        gates['nominal_continuous_merge'] = continuous_merge_gate(candidate)
        comparisons = []
        targets = report.get('merge_target_ids', [])
        for vehicle_id in targets:
            current = report.get('smooth_control', {}).get('per_car', {}).get(vehicle_id, {}).get('merge_slowdown', {})
            prior = actor['report'].get('smooth_control', {}).get('per_car', {}).get(vehicle_id, {}).get('merge_slowdown', {}) if actor_matches else {}
            current_coverage = observed_merge_coverage(report, vehicle_id)
            prior_coverage = observed_merge_coverage(actor['report'], vehicle_id) if actor_matches else dict(usable=False)
            comparisons.append(dict(vehicle_id=vehicle_id, candidate=current, frozen_actor=prior,
                **drawdown_comparison(current, prior, current_coverage, prior_coverage)))
        complete = bool(comparisons) and all(c['comparison_pass'] is not None for c in comparisons)
        gates['merge_drawdown_vs_frozen_actor'] = gate(
            all(c['comparison_pass'] is True for c in comparisons) if complete else None,
            maximum_additional_loss_m_s=.1, matched=actor_matches, comparisons=comparisons,
            explanation='Complete available windows, or a complete candidate below the observed baseline lower bound; retain boundary/post-window censoring and never fill missing samples with zeros')
    status = ('failed' if any(v['status'] == 'failed' for v in gates.values()) else
              'incomplete' if any(v['status'] == 'incomplete' for v in gates.values()) else 'passed_measured_subset')
    return dict(status=status, gates=gates, candidate_path=candidate['path'],
        teacher_path=teacher['path'] if teacher else None, frozen_actor_path=actor['path'] if actor else None,
        metrics=dict(interventions=report.get('overrides'), decisions=report.get('agent_steps'),
            intervention_fraction=report.get('intervention_fraction'), completed_trips=completed,
            requested_trips=report.get('requested_trips'), mean_boundary_time_s=boundary,
            passes=report.get('passes'), stable_merges=report.get('stable_merge', {}).get('stable'),
            controlled_jerk=report.get('smooth_control', {}).get('controlled', {}).get('jerk_m_s3'),
            background_jerk=report.get('smooth_control', {}).get('background', {}).get('jerk_m_s3')),
        recording_audit=candidate['audit'], source_sha256=candidate['manifest'].get('source_sha256'),
        input_sha256=candidate['hashes'])


def summarize(candidate_study, baseline_studies=(), fixtures=REQUIRED_FIXTURES):
    candidate_study = project_path(candidate_study)
    locations = [candidate_study, *(project_path(p) for p in baseline_studies)]
    results, contexts = {}, {}
    for fixture in fixtures:
        candidate = load_run(candidate_study / fixture / 'smooth')
        if candidate is None:
            results[fixture] = dict(status='incomplete', missing='smooth candidate recording')
            continue
        def baseline(kind):
            for location in locations:
                result = load_run(location / fixture / kind)
                if result is not None:
                    return result
            return None
        teacher, actor, native = (baseline(kind) for kind in ('teacher', 'frozen-v05', 'native'))
        results[fixture] = evaluate_candidate(candidate, teacher, actor)
        if native:
            contexts[fixture] = dict(path=native['path'], input_sha256=native['hashes'],
                integrity_failures=native['integrity_failures'], recording_audit=native['audit'],
                role='Native baseline context only; never accepted as a learned-controller result')
    statuses = [r['status'] for r in results.values()]
    status = 'failed' if 'failed' in statuses else 'incomplete' if 'incomplete' in statuses else 'passed_measured_subset'
    return dict(schema='highway_smooth_offline_gate_summary_v01', training=False, optimizer_steps=0,
        training_ready=False, status=status, candidate_study=relative(candidate_study),
        baseline_studies=[relative(p) for p in locations[1:]], required_fixtures=list(fixtures),
        baseline_selection='Candidate study first, then explicit baseline-study order; selection does not inspect outcomes',
        limitations=['This is a measured gate subset, not a training-readiness certificate.',
            'Gradient, dangerous-boundary reference, all-fixture coverage and frozen-actor invariance proofs remain separate.',
            'Native baseline metrics are context, not evidence of learned-policy improvement.'],
        fixtures=results, native_baseline_context=contexts,
        helper_source_sha256=sha(__file__),
        recomputed_metrics_source_sha256=sha(ROOT/'traffic/highway_smooth_metrics.py'),
        original_acceptance_source_sha256=sha(ROOT/'experiments/highway_v05_support.py'))


def plot_run(run_path, output, vehicle_id=None):
    # Isaac's standalone Python supplies matplotlib without starting SimulationApp.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    run_path = project_path(run_path)
    recording = read(run_path / 'recording.json')
    vehicle_id = vehicle_id or recording['focal_vehicle']
    trace = measured_trace(recording, vehicle_id)
    if not trace:
        raise ValueError('Vehicle has no recorded samples')
    commands = {}
    journals = sorted(run_path.glob('simulation/*/actions.jsonl.gz'))
    if len(journals) != 1:
        raise ValueError('Require exactly one retained action journal')
    with gzip.open(journals[0], 'rt', encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            if vehicle_id in row['agents']:
                commands[round(row['time'], 7)] = row['agents'][vehicle_id]
    times = [r['time_s'] for r in trace]
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True, constrained_layout=True)
    axes[0].plot(times, [r['speed_m_s'] for r in trace], label='Measured speed', color='#26768b')
    axes[1].plot(times, [r['acceleration'] for r in trace], label='Measured acceleration', color='#26768b')
    for field, label, style in [('requested_acceleration', 'Requested', '--'),
                                 ('feasible_acceleration_target', 'Reference', ':')]:
        axes[1].plot(times, [commands.get(round(t, 7), {}).get(field, math.nan) for t in times], style, label=label, alpha=.8)
    axes[2].plot(times, [r['jerk_m_s3'] if r['jerk_m_s3'] is not None else math.nan for r in trace], color='#a04e45', label='Measured jerk (uncensored)')
    axes[2].axhline(5., color='gray', linestyle='--', linewidth=.8)
    axes[2].axhline(-5., color='gray', linestyle='--', linewidth=.8, label='Nominal ±5 m/s³ gate')
    events = [(e['time'], 'Junction entry') for e in recording.get('junction_entry_events', []) if e['agent'] == vehicle_id and e['phase'] == 'started']
    events += [(e['time'], 'Settled merge') for e in recording.get('merge_events', []) if e['agent'] == vehicle_id]
    events += [(e['time'], 'Stable window end') for e in recording.get('stable_events', []) if e['agent'] == vehicle_id and e['status'] == 'succeeded']
    for stamp, label in events:
        for axis in axes:
            axis.axvline(stamp, color='gray', linestyle=':', linewidth=.8)
        axes[0].annotate(label, (stamp, .99), xycoords=('data', 'axes fraction'),
                         rotation=90, va='top', ha='right', fontsize=8)
    for axis, ylabel in zip(axes, ('Speed (m/s)', 'Acceleration (m/s²)', 'Jerk (m/s³)')):
        axis.set_ylabel(ylabel)
        axis.grid(alpha=.2)
        axis.legend(loc='best', fontsize=8)
    axes[-1].set_xlabel('Native simulation time (s)')
    fig.suptitle(f"{recording['controller']} · {vehicle_id} · seed {recording['seed']} · no training")
    output = project_path(output)
    if output.exists():
        raise ValueError('Preserve earlier plots: choose a new output path')
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160)
    plt.close(fig)
    output.with_suffix(output.suffix + '.json').write_text(json.dumps(dict(
        recording=relative(run_path/'recording.json'), recording_sha256=sha(run_path/'recording.json'),
        journal=relative(journals[0]), journal_sha256=sha(journals[0]), vehicle_id=vehicle_id,
        image_sha256=sha(output), source_sha256=sha(__file__), generated_from='Actual native samples; no synthetic images or simulator rerun'), indent=2)+'\n', encoding='utf-8')


def action_projection(rows, *, filtered=False):
    keys = ('filtered_acceleration', 'lane_applied_delta') if filtered else ('requested_acceleration', 'lane_request')
    return [dict(time=row['time'], agents={a: [info[key] for key in keys]
                 for a, info in sorted(row['agents'].items())}) for row in rows]


def compare_frozen_runs(first, second):
    first, second = project_path(first), project_path(second)
    runs = [load_run(path) for path in (first, second)]
    if any(run is None for run in runs):
        raise ValueError('Both complete frozen recordings are required')
    recordings = [read(path/'recording.json') for path in (first, second)]
    journals, actions = [], []
    for path in (first, second):
        matches = sorted(path.glob('simulation/*/actions.jsonl.gz'))
        if len(matches) != 1:
            raise ValueError('Exactly one action journal per frozen run is required')
        journals.append(matches[0])
        with gzip.open(matches[0], 'rt', encoding='utf-8') as stream:
            actions.append([json.loads(line) for line in stream])
    checks = dict(expected_controllers=[r['manifest']['controller'] for r in runs] == ['frozen-v05', 'frozen-smooth'],
        clean_saved_integrity=not any(r['integrity_failures'] for r in runs),
        matched_case_and_demand=matching(runs[0], runs[1]),
        matching_frozen_policy=bool(recordings[0].get('policy_sha256')) and recordings[0].get('policy_sha256') == recordings[1].get('policy_sha256'),
        all_native_frames_equal=recordings[0]['frames'] == recordings[1]['frames'],
        all_raw_actions_equal=action_projection(actions[0]) == action_projection(actions[1]),
        all_filtered_commands_equal=action_projection(actions[0], filtered=True) == action_projection(actions[1], filtered=True))
    return dict(schema='highway_smooth_frozen_invariance_v01', training=False, optimizer_steps=0,
        passed=all(checks.values()), checks=checks, frames=[len(r['frames']) for r in recordings],
        action_ticks=[len(a) for a in actions], policy_sha256=recordings[0].get('policy_sha256'),
        runs=[dict(path=r['path'], input_sha256=r['hashes']) for r in runs],
        action_journals=[dict(path=relative(p), sha256=sha(p)) for p in journals],
        observations='No live observation-array trace was recorded; exact observation-array invariance is covered separately by unit tests. This proof compares all native frames and raw/filtered actions.',
        helper_source_sha256=sha(__file__))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate-study', type=Path)
    parser.add_argument('--baseline-studies', nargs='*', type=Path, default=[])
    parser.add_argument('--fixtures', nargs='+', default=list(REQUIRED_FIXTURES))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plot-run', type=Path)
    parser.add_argument('--compare-frozen-runs', nargs=2, type=Path)
    parser.add_argument('--vehicle')
    args = parser.parse_args()
    if Path.cwd().resolve() != ROOT:
        raise ValueError('Run from the canonical project root')
    if args.plot_run:
        plot_run(args.plot_run, args.output, args.vehicle)
        return 0
    if args.compare_frozen_runs:
        output = project_path(args.output)
        if output.exists():
            raise ValueError('Preserve earlier evidence: choose a new output path')
        result = compare_frozen_runs(*args.compare_frozen_runs)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
        print(json.dumps(dict(passed=result['passed'], output=relative(output))))
        return 0
    if not args.candidate_study:
        parser.error('--candidate-study is required for gate aggregation')
    output = project_path(args.output)
    if output.exists():
        raise ValueError('Preserve earlier evidence: choose a new output path')
    result = summarize(args.candidate_study, args.baseline_studies, args.fixtures)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps(dict(status=result['status'], training_ready=False, output=relative(output))))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
