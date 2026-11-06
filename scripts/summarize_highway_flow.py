"""Fail-closed offline flow diagnostics; never launches simulation or learning.

Reads complete saved recordings, including their uncropped native tail. A
passing summary is diagnostic evidence, not a training authorization/readiness
writer. Nominal/stress classification must precede the recording.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DT = .1
MAX_INTERVENTION_FRACTION = .05
MAX_NOMINAL_STOP_SPAN_S = 2.
MAX_NOMINAL_LOW_SPEED_EXPOSURE_S = 2.
MAX_NOMINAL_PEAK_JERK_M_S3 = 5.


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _lane_number(row):
    identity = row.get('lane_identity', {})
    if identity.get('kind') == 'auxiliary':
        return 0
    return identity.get('index_from_right') if identity.get('kind') == 'through' else None


def _event_completion(fixture, report, traces):
    expected = fixture.get('events', []) if fixture else []
    count = fixture.get('expected_external_event_count', len(expected)) if fixture else 0
    actual = report.get('fixture_events')
    details = []
    if not isinstance(actual, list) or count != len(expected) or len(actual) != len(expected):
        return dict(complete=False, expected=count, observed=len(actual) if isinstance(actual, list) else None,
            details=[], reason='Missing or mismatched declared external-event accounting')
    for index, event in enumerate(expected):
        matches = [r for r in actual if r.get('index') == index]
        recorded = matches[0] if len(matches) == 1 else {}
        issued = recorded.get('command_issued') is True and recorded.get('movement_observed') is True
        requested = recorded.get('requested_at')
        trace = traces.get(event.get('vehicle_id'), [])
        before = [r for r in trace if _finite(requested) and r['time'] <= requested + 1e-7]
        source = before[-1] if before else {}
        lane = _lane_number(source)
        target = lane + event['lane_delta'] if isinstance(lane, int) else None
        settled = next((r['time'] for r in trace if _finite(requested) and r['time'] > requested
            and target is not None and _lane_number(r) == target
            and _finite(r.get('lateral_position')) and abs(r['lateral_position']) <= .02
            and r.get('maneuver_in_progress') is False), None)
        details.append(dict(index=index, vehicle_id=event.get('vehicle_id'), issued_and_motion_observed=issued,
            source_lane_number=lane, target_lane_number=target, settled_at_s=settled,
            complete=bool(issued and settled is not None)))
    return dict(complete=all(r['complete'] for r in details), expected=count, observed=len(actual), details=details,
        definition='Predeclared request plus observed motion and centered target-corridor occupancy; a request alone is not completion')


def summarize_recording(report, recording, recording_check=None):
    """Summarize one unmodified recording/report pair without filling missing data."""
    errors, traces = [], {}
    frames = recording.get('frames')
    if not isinstance(frames, list) or len(frames) < 2:
        errors.append('missing_native_frames')
        frames = []
    previous = None
    for sample in frames:
        time = sample.get('time')
        if not _finite(time) or (previous is not None and abs(time - previous - DT) > 1e-7):
            errors.append('nonconsecutive_native_frame_times')
        previous = time if _finite(time) else None
        ids = set()
        for row in sample.get('vehicles', []):
            identity = row.get('id')
            if not isinstance(identity, str) or identity in ids:
                errors.append('invalid_or_duplicate_vehicle_identity')
                continue
            ids.add(identity)
            if (not all(_finite(row.get(k)) for k in ('speed', 'acceleration', 'distance_m', 'legal_speed_m_s'))
                    or row.get('legal_speed_m_s', 0.) <= 0.
                    or type(row.get('route_reachable_without_lateral')) is not bool):
                errors.append('missing_or_invalid_full_route_measurement')
            traces.setdefault(identity, []).append(dict(row, time=time))
    selected = recording.get('selected_case', {})
    fixture = selected.get('fixture') or {}
    nominal = fixture.get('nominal_unobstructed', selected.get('nominal_unobstructed'))
    if type(nominal) is not bool:
        errors.append('nominal_classification_missing')
    comfort = fixture.get('nominal_comfort', selected.get('nominal_comfort', nominal if fixture else None))
    if type(comfort) is not bool:
        errors.append('nominal_comfort_classification_missing')
    nominal_any = nominal is True or comfort is True
    controlled = recording.get('controlled_ids')
    count = selected.get('controlled_count')
    if (not isinstance(controlled, list) or not controlled or len(controlled) != len(set(controlled))
            or not isinstance(count, int) or len(controlled) != count):
        errors.append('controlled_cohort_missing_or_mismatched')
        controlled = controlled if isinstance(controlled, list) else []
    requested_controlled = {r['vehicle_id'] for r in fixture.get('vehicles', []) if r.get('controlled') is True}
    if requested_controlled and set(controlled) != requested_controlled:
        errors.append('controlled_fixture_identity_mismatch')
    horizon = recording.get('horizon_seconds')
    full_horizon = bool(frames and _finite(horizon) and abs(horizon - 80.) <= 1e-8 and _finite(frames[0].get('time'))
        and _finite(frames[-1].get('time')) and frames[-1]['time'] - frames[0]['time'] >= horizon - 1e-7
        and report.get('primary_horizon_complete') is True
        and report.get('accounting_comparison_valid') is True
        and report.get('partial_control_horizon') is False)
    if not full_horizon:
        errors.append('incomplete_primary_horizon')
    if (recording_check is None or recording_check.get('error') is not None
            or recording_check.get('source_changed') != []):
        errors.append('recording_integrity_check_missing_or_failed')
    if report.get('training') is not False or report.get('optimizer_steps') != 0:
        errors.append('recording_not_explicitly_no_update')
    accounting = report.get('accounting', {})
    trips = accounting.get('trips')
    if not isinstance(trips, list) or accounting.get('comparison_valid') is not True:
        errors.append('trip_ledger_missing_or_invalid')
        trips = []
    trip_map = {r.get('vehicle_id'): r for r in trips}
    if len(trip_map) != len(trips):
        errors.append('duplicate_trip_ledger_identity')
    per_car = report.get('full_route_metrics', {}).get('per_car')
    if not isinstance(per_car, dict) or set(per_car) != set(traces):
        errors.append('full_route_metrics_missing_or_identity_mismatch')
        per_car = per_car if isinstance(per_car, dict) else {}
    coverage = {}
    for identity, trace in traces.items():
        metric, trip = per_car.get(identity, {}), trip_map.get(identity, {})
        finite_time = all(_finite(r.get('time')) for r in trace)
        adjacent = finite_time and all(abs(b['time'] - a['time'] - DT) < 1e-7 for a, b in zip(trace, trace[1:]))
        departed, arrived = trip.get('departed_at'), trip.get('arrived_at')
        successful = trip.get('state') == 'completed' and _finite(arrived) and trip.get('had_teleport') is False
        boundary = arrived - DT if successful else frames[-1]['time'] if frames else None
        first = max(frames[0]['time'], departed) if frames and _finite(departed) and _finite(frames[0].get('time')) else None
        complete = bool(adjacent and _finite(first) and _finite(boundary)
            and trace[0]['time'] <= first + DT + 1e-7 and trace[-1]['time'] >= boundary - 1e-7
            and metric.get('native_samples') == len(trace)
            and metric.get('measured_intervals') == len(trace) - 1
            and metric.get('missing_intervals') == 0)
        coverage[identity] = dict(observed_interval_complete=complete, successful_route_completion=successful,
            state=trip.get('state'), first_observed_s=trace[0]['time'], last_observed_s=trace[-1]['time'],
            arrival_time_s=arrived, full_route_complete=complete and successful)
    if any(not r['observed_interval_complete'] for r in coverage.values()):
        errors.append('incomplete_observed_vehicle_interval')
    if set(controlled) - set(coverage):
        errors.append('controlled_vehicle_never_observed')
    for trip in trips:
        if _finite(trip.get('departed_at')) and trip.get('vehicle_id') not in traces:
            errors.append('departed_vehicle_missing_from_capture')
    safety = {}
    for key in ('collisions', 'teleports', 'overspeed_steps', 'emergency_braking_observed'):
        safety[key] = _finite(report.get(key)) and report[key] == 0
    emergencies = report.get('native_emergencies', {})
    safety['native_warning_accounting'] = emergencies.get('available') is True and emergencies.get('warning_lines') == 0
    steps, overrides, fraction = report.get('agent_steps'), report.get('overrides'), report.get('intervention_fraction')
    denominator_valid = (_finite(steps) and steps > 0 and _finite(overrides) and 0 <= overrides <= steps
        and _finite(fraction) and abs(fraction - overrides / steps) <= 1e-12)
    safety['raw_intervention_fraction_at_most_5_percent'] = bool(denominator_valid and fraction <= MAX_INTERVENTION_FRACTION)
    if not denominator_valid:
        errors.append('raw_override_denominator_missing_or_inconsistent')
    events = _event_completion(fixture, report, traces)
    if not events['complete']:
        errors.append('counterfactual_event_not_completed')
    controlled_checks = {}
    required = ('longest_consecutive_stop_s', 'below_half_local_limit_exposure_s', 'peak_abs_measured_jerk_m_s3')
    for identity in controlled:
        metric = per_car.get(identity, {})
        valid = all(_finite(metric.get(k)) and metric[k] >= 0 for k in required)
        excursions = metric.get('ending_lane_excursions')
        if not valid or not isinstance(excursions, list):
            errors.append('controlled_full_route_metric_missing')
        check = dict(measurements_complete=valid,
            no_ending_lane_excursion=isinstance(excursions, list) and len(excursions) == 0,
            full_route_complete=coverage.get(identity, {}).get('full_route_complete', False),
            nominal_stop_applies=nominal_any, unobstructed_low_speed_applies=nominal is True,
            nominal_comfort_applies=comfort is True)
        if nominal_any:
            check['no_two_second_stop'] = valid and metric['longest_consecutive_stop_s'] < MAX_NOMINAL_STOP_SPAN_S - 1e-7
        if nominal is True:
            check['low_speed_exposure_at_most_two_seconds'] = valid and metric['below_half_local_limit_exposure_s'] <= MAX_NOMINAL_LOW_SPEED_EXPOSURE_S + 1e-7
        if comfort is True:
            check['peak_jerk_at_most_five'] = valid and metric['peak_abs_measured_jerk_m_s3'] <= MAX_NOMINAL_PEAK_JERK_M_S3 + 1e-6
        controlled_checks[identity] = check
    task = selected.get('case', {}).get('task')
    task_gate = True
    if task == 'merge':
        stable = report.get('stable_merge', {})
        target = report.get('merge_target_count')
        task_gate = isinstance(target, int) and target > 0 and stable.get('targets') == target and stable.get('stable') == target
    elif task == 'passing':
        task_gate = _finite(report.get('passes')) and report['passes'] >= 1
    elif task == 'cruise':
        task_gate = _finite(report.get('mean_fraction_of_speed_limit')) and report['mean_fraction_of_speed_limit'] >= .9
    elif task != 'following':
        errors.append('unknown_task_contract')
        task_gate = False
    nominal_pass = all(all(value for key, value in row.items() if not key.endswith('_applies'))
        for row in controlled_checks.values()) if nominal_any else None
    route_pass = bool(controlled) and all(r['no_ending_lane_excursion'] for r in controlled_checks.values())
    passed = not errors and all(safety.values()) and task_gate and route_pass and (not nominal_any or nominal_pass is True)
    totals = accounting.get('totals', {})
    requested = totals.get('requested')
    boundary = totals.get('boundary_time_s')
    mean_boundary = boundary / requested if _finite(boundary) and _finite(requested) and requested > 0 else None
    return dict(schema='highway_flow_recording_summary_v1', passed=bool(passed), training=False, optimizer_steps=0,
        controller=recording.get('controller'), case_id=selected.get('id', selected.get('case', {}).get('id')),
        seed=selected.get('seed'), task=task, nominal_unobstructed=nominal, nominal_comfort=comfort,
        integrity_failures=sorted(set(errors)), full_primary_horizon=full_horizon,
        safety_gates=safety, task_gate=bool(task_gate), route_utility_gate=route_pass,
        nominal_controlled_gates_pass=nominal_pass, controlled_gates=controlled_checks,
        event_completion=events, vehicle_coverage=coverage,
        all_car_metrics=deepcopy(per_car), raw_interventions=dict(overrides=overrides, agent_steps=steps, fraction=fraction),
        completed_trips=report.get('completed_trips'), requested_trips=requested, mean_boundary_time_s=mean_boundary,
        scope='Full uncropped observed intervals; missing/full-route coverage cannot pass nominal gates; stress peaks remain in all_car_metrics')


def summarize_study(directory):
    directory = Path(directory)
    study = json.loads((directory / 'study.json').read_text(encoding='utf-8'))
    results = json.loads((directory / 'results.json').read_text(encoding='utf-8'))
    expected = {(r.get('id', r['case']['id']), r['seed'], c) for r in study['rows'] for c in study['controllers']}
    records, identities, errors = [], set(), []
    for result in results:
        path = (ROOT / result['path']).resolve()
        if not path.is_relative_to(directory.resolve()):
            errors.append('recording_path_outside_study')
            continue
        report = json.loads((path / 'evaluation.json').read_text(encoding='utf-8'))
        recording = json.loads((path / 'recording.json').read_text(encoding='utf-8'))
        check = json.loads((path / 'recording-check.json').read_text(encoding='utf-8'))
        if recording.get('source_sha256') != study.get('source_hashes'):
            errors.append('recording_source_manifest_mismatch')
        matching_rows = [row for row in study['rows'] if row == recording.get('selected_case')]
        if len(matching_rows) != 1:
            errors.append('recorded_case_differs_from_predeclared_inputs')
        for name in ('evaluation', 'recording'):
            digest = hashlib.sha256((path / (name + '.json')).read_bytes()).hexdigest()
            if digest != check.get(name + '_sha256'):
                errors.append('recording_checksum_mismatch')
        summary = summarize_recording(report, recording, check)
        identity = (summary['case_id'], summary['seed'], summary['controller'])
        if identity in identities:
            errors.append('duplicate_case_controller')
        identities.add(identity)
        records.append(summary)
    if identities != expected:
        errors.append('missing_or_unexpected_case_controller')
    reference = [row for row in records if row['controller'] == 'reference']
    comparisons = []
    for candidate in reference:
        matches = [row for row in records if row['case_id'] == candidate['case_id']
            and row['seed'] == candidate['seed'] and row['controller'] == 'v05_final']
        baseline = matches[0] if len(matches) == 1 else None
        comparable = baseline is not None and not baseline['integrity_failures'] and not candidate['integrity_failures']
        completed_ok = bool(comparable and _finite(candidate['completed_trips']) and _finite(baseline['completed_trips'])
            and candidate['completed_trips'] >= baseline['completed_trips'])
        times = comparable and _finite(candidate['mean_boundary_time_s']) and _finite(baseline['mean_boundary_time_s'])
        boundary_ok = bool(times and candidate['mean_boundary_time_s'] <= 1.02 * baseline['mean_boundary_time_s'] + 1e-7)
        comparisons.append(dict(case_id=candidate['case_id'], seed=candidate['seed'], baseline='v05_final',
            comparison_complete=bool(comparable), no_completed_trip_deficit=completed_ok,
            mean_boundary_regression_at_most_two_percent=boundary_ok, passed=completed_ok and boundary_ok))
    controller_table = []
    for controller in sorted({row['controller'] for row in records}):
        group = [row for row in records if row['controller'] == controller]
        fractions = [row['raw_interventions']['fraction'] for row in group if _finite(row['raw_interventions']['fraction'])]
        peaks = [metric.get('peak_abs_measured_jerk_m_s3') for row in group
                 for identity, metric in row['all_car_metrics'].items()
                 if identity in row['controlled_gates'] and _finite(metric.get('peak_abs_measured_jerk_m_s3'))]
        boundaries = [row['mean_boundary_time_s'] for row in group if _finite(row['mean_boundary_time_s'])]
        controller_table.append(dict(controller=controller, runs=len(group), passing_runs=sum(row['passed'] for row in group),
            integrity_failed_runs=sum(bool(row['integrity_failures']) for row in group),
            unobstructed_runs=sum(row['nominal_unobstructed'] is True for row in group),
            nominal_comfort_runs=sum(row['nominal_comfort'] is True for row in group),
            worst_raw_intervention_fraction=max(fractions) if fractions else None,
            uncensored_worst_controlled_peak_jerk_m_s3=max(peaks) if peaks else None,
            equal_case_mean_boundary_time_s=sum(boundaries) / len(boundaries) if boundaries else None))
    return dict(schema='highway_flow_study_summary_v1', training=False, optimizer_steps=0,
        source_study=str(directory), integrity_failures=sorted(set(errors)), expected_recordings=len(expected),
        actual_recordings=len(records), controller_table=controller_table,
        recordings=records, paired_reference_progress=comparisons,
        criteria=dict(raw_intervention_fraction_max=.05, nominal_stop_span_strictly_below_s=2.,
            unobstructed_low_speed_exposure_max_s=2., nominal_peak_jerk_max_m_s3=5.,
            paired_mean_boundary_regression_max=.02, controlled_only_for_nominal_gates=True,
            all_cars_retained=True, stress_classification='Predeclared, never inferred from outcome'),
        reference_candidate_qualified=bool(not errors and reference and all(r['passed'] for r in reference)
            and all(c['passed'] for c in comparisons)),
        training_authorization=False, readiness_written=False,
        qualification_scope='Recorded train-only bank only; this summary does not replace source/config readiness or authorize learning')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--study', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT / 'outputs') or output.exists():
        raise ValueError('Use a fresh output file inside the workspace outputs directory')
    summary = summarize_study(args.study)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    return 0 if summary['reference_candidate_qualified'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
