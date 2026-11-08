"""Verify the fixed 54 cached + 27 new flow recordings without rerunning them.

This is explicitly compatible cached evidence, not one-version native capture.
Only the bank builder may differ; runtime controls and physical case inputs
must agree. Original studies, failures and charged preparation costs stay intact.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from scripts.summarize_highway_flow import summarize_recording

BUILDER = 'scripts/build_highway_flow_bank.py'
CONTROLLERS = ('native', 'reference', 'v05_final')
CACHED_ROWS = 18
TOTAL_ROWS = 27
REPAIR_IDENTITY = ('legacy-i110-merge', 6800019, 'v05_final')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def normalized_case(row):
    """Ignore only non-executed, declared bank metadata; keep all conditions."""
    value = deepcopy(row)
    value['case'].pop('network', None)  # Content SHA remains mandatory below.
    fixture = value.get('fixture')
    if fixture:
        fixture.pop('fixture_version', None)
        # v04 records an empty yield-approach validation list where v03 omitted
        # it. A nonempty/new changed validation result is deliberately retained.
        for vehicle in fixture.get('static_spawn_checks', {}).get('vehicles', []):
            if vehicle.get('yield_approaches') == []:
                vehicle.pop('yield_approaches')
    return value


def _compute_yield_approaches(row):
    """Recompute added validation metadata from immutable map/vehicle inputs."""
    from traffic.highway_merge_context import Stage1LaneGraph
    from traffic.highway_v05_demand import PROFILES
    network = ROOT / row['case']['network']
    if sha(network) != row['case']['network_sha256']:
        raise ValueError('Cannot recompute metadata from a changed network')
    graph = Stage1LaneGraph(network, row['case']['through_lanes'])
    result = {}
    for vehicle in row['fixture']['vehicles']:
        route = ('mlk_on','sb2','sb3','sb4','sb5') if vehicle['route']=='ramp' else ('sb1','sb2','sb3','sb4','sb5')
        distances = graph.distances(vehicle['lane_id'], vehicle['position'], route)
        tau = 1.5 if vehicle['controlled'] else PROFILES[vehicle['driver_profile']]['tau']
        speed = vehicle['speed']
        required = tau * speed + speed**2 / 9. + 2.5
        approaches = []
        for source, entry in graph.yield_entries.items():
            if source in distances:
                available = distances[source] + graph.lanes[source]['length'] - entry['stop_offset_m']
                if not all(math.isfinite(x) for x in (available, required)) or required > available + 1e-8:
                    raise ValueError('Recomputed yield approach is not safely insertable')
                approaches.append(dict(source_lane=source, available_stop_room_m=available, required_stop_room_m=required))
        result[vehicle['vehicle_id']] = approaches
    return result


def verify_case(expected, actual, *, allow_added_yield_metadata=False):
    if not expected.get('case', {}).get('network_sha256'):
        raise ValueError('Expected case must pin network content')
    new, old = normalized_case(expected), normalized_case(actual)
    added = []
    expected_checks = (expected.get('fixture') or {}).get('static_spawn_checks', {}).get('vehicles', [])
    actual_checks = (actual.get('fixture') or {}).get('static_spawn_checks', {}).get('vehicles', [])
    if len(expected_checks) == len(actual_checks):
        for before, after in zip(actual_checks, expected_checks):
            if 'yield_approaches' not in before and 'yield_approaches' in after:
                added.append(dict(vehicle_id=after.get('vehicle_id'), before='absent', after=deepcopy(after['yield_approaches'])))
    proof = None
    if added:
        if not allow_added_yield_metadata:
            raise ValueError('Added yield validation metadata requires explicit recomputation exception')
        computed = _compute_yield_approaches(expected)
        for difference in added:
            if computed.get(difference['vehicle_id']) != difference['after']:
                raise ValueError('Added yield validation metadata differs from immutable map/vehicle recomputation')
        for normalized in (new, old):
            for vehicle in (normalized.get('fixture') or {}).get('static_spawn_checks', {}).get('vehicles', []):
                if vehicle.get('vehicle_id') in {r['vehicle_id'] for r in added}:
                    vehicle.pop('yield_approaches', None)
        proof = dict(formula='tau*speed + speed^2/9 +2.5 <= graph-distance-to-yield-end minus actual map stop offset',
            network_sha256=expected['case']['network_sha256'], differences=added,
            all_values_exactly_recomputed=True, all_required_room_inequalities_pass=True)
    if new != old:
        raise ValueError('Physical case or undeclared metadata differs; cached recording is incompatible')
    return dict(network_path_changed=expected['case']['network'] != actual['case']['network'],
        fixture_version_changed=(expected.get('fixture') or {}).get('fixture_version') !=
                                (actual.get('fixture') or {}).get('fixture_version'),
        empty_yield_approach_metadata_equivalence='Only missing versus [] is normalized',
        added_yield_validation_proof=proof,
        physical_inputs_identical=True, network_sha256=expected['case']['network_sha256'])


def verify_sources(cached, fresh, *, allow_builder_change=False):
    if not isinstance(cached, dict) or not cached or set(cached) != set(fresh):
        raise ValueError('Source inventory differs')
    changed = [name for name in cached if cached[name] != fresh[name]]
    if any(name != BUILDER for name in changed) or (changed and not allow_builder_change):
        raise ValueError('Runtime source differs or builder exception was not explicitly declared')
    return dict(identical_runtime_sources={k:v for k,v in cached.items() if k != BUILDER},
        declared_only_allowed_difference=BUILDER, explicit_exception=bool(allow_builder_change),
        builder_sha256=dict(cached=cached.get(BUILDER), fresh=fresh.get(BUILDER)), changed_sources=changed)


def normalized_config(config):
    result = deepcopy(config)
    bank = result.get('network_bank', {})
    bank.pop('manifest_path', None)
    bank.pop('manifest_sha256', None)
    return result


def _identity(row):
    return row.get('id', row['case']['id']), row['seed']


def allocation(expected_rows):
    if len(expected_rows) != TOTAL_ROWS or len({_identity(r) for r in expected_rows}) != TOTAL_ROWS:
        raise ValueError('The fixed bank requires exactly27 unique rows')
    return [(row, controller, 'cached' if i < CACHED_ROWS else 'fresh')
            for i, row in enumerate(expected_rows) for controller in CONTROLLERS]


def verify_deadline_repair_eligibility(result, report, recording):
    selected = recording.get('selected_case', {})
    identity = (result.get('row_id'), selected.get('seed'), result.get('controller'))
    error = result.get('error')
    terminal = error.strip().splitlines()[-1] if isinstance(error, str) and error.strip() else ''
    if (identity != REPAIR_IDENTITY or selected.get('id') != REPAIR_IDENTITY[0]
            or recording.get('controller') != REPAIR_IDENTITY[2]
            or result.get('source_changed') != []
            or report.get('primary_horizon_complete') is not False
            or report.get('accounting_comparison_valid') is not False
            or not terminal.startswith('experiments.run_i110_merge.PilotBudgetExhausted:')):
        raise ValueError('Only the fixed incomplete I110 frozen-actor aggregate-deadline failure may be repaired')
    return dict(row_id=identity[0],seed=identity[1],controller=identity[2],
        failure_class='PilotBudgetExhausted',replacement_is_outcome_independent=True,
        original_incomplete=True)


def _file_record(path):
    path = Path(path).resolve()
    if not path.is_relative_to(ROOT):
        raise ValueError('Evidence must stay inside canonical workspace')
    return dict(path=path.relative_to(ROOT).as_posix(), sha256=sha(path), bytes=path.stat().st_size)


def _verify_demand(directory, row):
    paths = list(directory.glob('simulation/**/routes.rou.xml'))
    if len(paths) != 1:
        raise ValueError('Recording must retain exactly one actual demand XML')
    fixture = row.get('fixture')
    if fixture:
        nodes = ET.parse(paths[0]).getroot().findall('vehicle')
        vehicles = {node.get('id'):node.attrib for node in nodes}
        expected = fixture['vehicles']
        if len(vehicles) != len(nodes) or set(vehicles) != {r['vehicle_id'] for r in expected}:
            raise ValueError('Actual native vehicle identities differ from explicit fixture')
        fields = {'depart':'depart', 'edge':'departEdge', 'lane':'departLane',
                  'position':'departPos', 'speed':'departSpeed'}
        for vehicle in expected:
            actual = vehicles[vehicle['vehicle_id']]
            if actual.get('route') != vehicle['route'] or actual.get('type') != ('controlled' if vehicle['controlled'] else vehicle['driver_profile']):
                raise ValueError('Actual route/driver type differs from explicit fixture')
            if any(float(actual[key]) != float(vehicle[source]) for source,key in fields.items()):
                raise ValueError('Actual departure/lane/position/speed differs from explicit fixture')
    return _file_record(paths[0])


def _load_record(study_dir, study, result, expected, *, reused):
    directory = (ROOT / result['path']).resolve()
    if not directory.is_relative_to(study_dir.resolve()):
        raise ValueError('Recording path escapes its source study')
    recording, report, check = (read(directory / name) for name in ('recording.json', 'evaluation.json', 'recording-check.json'))
    if check != result or result.get('error') is not None or result.get('source_changed') != []:
        raise ValueError('Selected recording has an execution/integrity error')
    for name in ('recording', 'evaluation'):
        if sha(directory / (name + '.json')) != result.get(name + '_sha256'):
            raise ValueError('Saved recording/evaluation checksum changed')
    if recording.get('source_sha256') != study['source_hashes']:
        raise ValueError('Recording source map differs from captured study')
    if report.get('training') is not False or report.get('optimizer_steps') != 0:
        raise ValueError('Evidence is not explicitly frozen/no-update')
    if recording.get('controller', '').startswith('v05_'):
        candidate = read(study_dir/'config.json')['initialization']['candidates'][recording['controller']]
        if recording.get('policy_sha256') != candidate['actor_sha256'] or sha(ROOT/candidate['actor_path']) != candidate['actor_sha256']:
            raise ValueError('Frozen actor identity differs from the captured initialization contract')
    original = recording.get('selected_case')
    if original not in study['rows']:
        raise ValueError('Recorded case was not in original predeclared study')
    proof = verify_case(expected, original, allow_added_yield_metadata=True)
    if result['row_id'] != _identity(original)[0] or recording['controller'] != result['controller']:
        raise ValueError('Recording identity mismatch')
    expected_sha = expected['case']['network_sha256']
    for path in (ROOT / expected['case']['network'], ROOT / original['case']['network'], directory / 'network.net.xml'):
        if sha(path) != expected_sha:
            raise ValueError('Network content differs despite recorded identity')
    if report.get('fixture') != original.get('fixture'):
        raise ValueError('Runtime fixture differs from predeclared fixture')
    if not report.get('primary_horizon_complete') or not report.get('accounting_comparison_valid'):
        raise ValueError('Selected native horizon is incomplete')
    demand = _verify_demand(directory, original)
    files = [_file_record(p) for p in sorted(directory.rglob('*')) if p.is_file()]
    summary = summarize_recording(report, recording, check)
    return dict(expected_row_id=_identity(expected)[0], seed=expected['seed'], controller=recording['controller'],
        evidence_source='compatible_cached' if reused else 'new_capture', original_recording_directory=result['path'],
        compatibility=proof, actual_demand=demand, files=files, native_runtime_version=report.get('sumo_runtime_version'),
        source_wall_seconds=result['wall_seconds'], summary=summary)


def combine(expected_rows_path, cached_directory, fresh_directory, ledger_path, *, allow_builder_change=False,
            repair_directory=None):
    expected = read(expected_rows_path)
    plan = allocation(expected)
    cached_directory, fresh_directory = Path(cached_directory).resolve(), Path(fresh_directory).resolve()
    studies = {'cached':read(cached_directory / 'study.json'), 'fresh':read(fresh_directory / 'study.json')}
    configs = {'cached':read(cached_directory / 'config.json'), 'fresh':read(fresh_directory / 'config.json')}
    if normalized_config(configs['cached']) != normalized_config(configs['fresh']):
        raise ValueError('Control/training configuration differs beyond bank manifest metadata')
    bank_files = []
    for config in configs.values():
        bank = config['network_bank']
        path = ROOT / bank['manifest_path']
        if sha(path) != bank['manifest_sha256']:
            raise ValueError('Bank manifest content differs from captured configuration')
        bank_files.append(_file_record(path))
    for study in studies.values():
        if study.get('controllers') != list(CONTROLLERS) or study.get('training') is not False or study.get('optimizer_steps') != 0:
            raise ValueError('Fixed controller set/no-update study contract differs')
    for key in ('python', 'torch', 'traci', 'sumo_version'):
        if not studies['cached'].get(key) or studies['cached'][key] != studies['fresh'].get(key):
            raise ValueError('Runtime version differs or is missing: ' + key)
    source_proof = verify_sources(studies['cached']['source_hashes'], studies['fresh']['source_hashes'],
                                  allow_builder_change=allow_builder_change)
    for name, digest in source_proof['identical_runtime_sources'].items():
        if sha(ROOT/name) != digest:
            raise ValueError('Current runtime changed after capture; metadata recomputation must use the frozen sources')
    for label, directory in (('cached', cached_directory), ('fresh', fresh_directory)):
        for name, digest in studies[label]['source_hashes'].items():
            snapshot = (directory / 'source' / name).resolve()
            if not snapshot.is_relative_to(directory/'source') or sha(snapshot) != digest:
                raise ValueError('Retained runtime source snapshot changed')
    if len(studies['cached']['rows']) != TOTAL_ROWS or studies['fresh']['rows'] != expected[CACHED_ROWS:]:
        raise ValueError('Study allocation is not the fixed first18/last9 split')
    for want, actual in zip(expected[:CACHED_ROWS], studies['cached']['rows'][:CACHED_ROWS]):
        verify_case(want, actual, allow_added_yield_metadata=True)
    results = {'cached':read(cached_directory / 'results.json'), 'fresh':read(fresh_directory / 'results.json')}
    endings = {'cached':read(cached_directory / 'summary.json'), 'fresh':read(fresh_directory / 'summary.json')}
    if len(results['cached']) != 55 or len(results['fresh']) != 27:
        raise ValueError('Require exact54 completed plusone retained failure, and27 new recordings')
    failed = results['cached'][-1]
    if (not failed.get('error') or failed.get('row_id') != _identity(expected[CACHED_ROWS])[0]
            or failed.get('controller') != CONTROLLERS[0] or not endings['cached'].get('error')
            or (endings['fresh'].get('error') is not None and repair_directory is None)):
        raise ValueError('Retained failure/completion structure differs from fixed allocation')
    for label in studies:
        if endings[label].get('runs') != results[label]:
            raise ValueError('Study completion summary and raw result list differ')
    repair = None
    repaired_failure = None
    if repair_directory is not None:
        if not endings['fresh'].get('error'):
            raise ValueError('A completed study cannot be replaced by a repair for better outcomes')
        repair_directory = Path(repair_directory).resolve()
        repaired_failure = deepcopy(results['fresh'][-1])
        incomplete_dir = (ROOT/repaired_failure['path']).resolve()
        if not incomplete_dir.is_relative_to(fresh_directory):
            raise ValueError('Incomplete recording escapes source study')
        old_record, old_report, old_check = (read(incomplete_dir/name) for name in
            ('recording.json','evaluation.json','recording-check.json'))
        if old_check != repaired_failure or old_record.get('source_sha256') != studies['fresh']['source_hashes']:
            raise ValueError('Incomplete recording integrity/provenance differs')
        for name in ('recording','evaluation'):
            if sha(incomplete_dir/(name+'.json')) != repaired_failure.get(name+'_sha256'):
                raise ValueError('Incomplete recording evidence changed')
        repair_proof = verify_deadline_repair_eligibility(repaired_failure,old_report,old_record)
        repair_study, repair_config, repair_results, repair_end = (read(repair_directory/name) for name in
            ('study.json','config.json','results.json','summary.json'))
        if (repair_study.get('rows') != [expected[-1]] or repair_study.get('controllers') != ['v05_final']
                or old_record['selected_case'] != expected[-1]
                or repair_study.get('source_hashes') != studies['fresh']['source_hashes']
                or repair_config != configs['fresh'] or len(repair_results) != 1
                or repair_end.get('error') is not None or repair_end.get('runs') != repair_results
                or repair_study.get('training') is not False or repair_study.get('optimizer_steps') != 0):
            raise ValueError('Repair must exactly repeat the fixed row, source, configuration and actor')
        for key in ('python','torch','traci','sumo_version'):
            if repair_study.get(key) != studies['fresh'].get(key):
                raise ValueError('Repair runtime version differs')
        for name,digest in repair_study['source_hashes'].items():
            if sha(repair_directory/'source'/name) != digest:
                raise ValueError('Repair source snapshot changed')
        repair = _load_record(repair_directory,repair_study,repair_results[0],expected[-1],reused=False)
        old_demand = _verify_demand(incomplete_dir,old_record['selected_case'])
        if old_demand['sha256'] != repair['actual_demand']['sha256']:
            raise ValueError('Repair changed actual physical demand')
        if old_record.get('policy_sha256') != read(ROOT/repair['original_recording_directory']/'recording.json').get('policy_sha256'):
            raise ValueError('Repair changed frozen actor')
        repair.update(evidence_source='fixed_deadline_repair',deadline_repair_proof=repair_proof,
            replaced_incomplete_directory=repaired_failure['path'],original_demand=old_demand)
        endings['repair'] = repair_end
    records = []
    for offset, (row, controller, source) in enumerate(plan):
        index = offset if source == 'cached' else offset - CACHED_ROWS * len(CONTROLLERS)
        result = results[source][index]
        if result['row_id'] != _identity(row)[0] or result['controller'] != controller:
            raise ValueError('Source recording order differs; no cherry-selection/fallback is allowed')
        if repair is not None and offset == len(plan)-1:
            records.append(repair)
            continue
        records.append(_load_record(cached_directory if source == 'cached' else fresh_directory,
            studies[source], result, row, reused=source == 'cached'))
    versions = [json.dumps(r['native_runtime_version'], sort_keys=True) for r in records]
    if any(r['native_runtime_version'] is None for r in records) or len(set(versions)) != 1:
        raise ValueError('Actual native runtime versions differ or are missing')
    for row in expected:
        demands = {r['actual_demand']['sha256'] for r in records if (r['expected_row_id'],r['seed']) == _identity(row)}
        if len(demands) != 1:
            raise ValueError('Matched controllers did not receive identical actual demand XML')
    ledger = read(ledger_path)
    costs = []
    for attempt in ledger.get('attempts', []):
        charged = attempt.get('charged_wall_seconds')
        if attempt.get('status') not in ('complete', 'completed', 'failed') or not isinstance(charged, (int,float)) or not math.isfinite(charged) or charged < 0:
            raise ValueError('Preparation ledger has unfinished/unaccounted reservation')
        costs.append({key:attempt.get(key) for key in ('output','status','charged_wall_seconds','reserved_wall_seconds')})
    source_directories = (cached_directory,fresh_directory) + ((repair_directory,) if repair is not None else ())
    for directory in source_directories:
        identity = directory.relative_to(ROOT).as_posix()
        if len([c for c in costs if c['output'] == identity]) != 1:
            raise ValueError('Source study is missing from retained preparation costs')
        cost = next(c for c in costs if c['output'] == identity)
        end = endings['cached' if directory == cached_directory else 'fresh' if directory == fresh_directory else 'repair']
        if abs(cost['charged_wall_seconds'] - end['wall_seconds']) > 1e-8:
            raise ValueError('Source study and ledger charged costs differ')
    failures = []
    for attempt in costs:
        directory = (ROOT / attempt['output']).resolve()
        if not directory.is_relative_to(ROOT / 'outputs/highway-flow/v01'):
            raise ValueError('Preparation evidence path outside approved study directory')
        if attempt['status'] == 'failed':
            failure_paths = sorted(directory.rglob('failure.json'))
            failed_record_files = [_file_record(p) for folder in {p.parent for p in failure_paths}
                                   for p in sorted(folder.rglob('*')) if p.is_file()]
            failures.append(dict(cost=attempt, failure_files=[_file_record(p) for p in failure_paths],
                failed_recording_files=failed_record_files,
                summary_file=_file_record(directory/'summary.json') if (directory/'summary.json').exists() else None))
    table, paired = [], []
    for controller in CONTROLLERS:
        group = [r['summary'] for r in records if r['controller'] == controller]
        peaks = [m['peak_abs_measured_jerk_m_s3'] for s in group for identity,m in s['all_car_metrics'].items()
                 if identity in s['controlled_gates'] and m.get('peak_abs_measured_jerk_m_s3') is not None]
        table.append(dict(controller=controller, recordings=len(group), passed=sum(s['passed'] for s in group),
            integrity_failed=sum(bool(s['integrity_failures']) for s in group),
            worst_intervention_fraction=max(s['raw_interventions']['fraction'] for s in group),
            uncensored_worst_controlled_peak_jerk_m_s3=max(peaks) if peaks else None))
    for row in expected:
        group = {r['controller']:r['summary'] for r in records if (r['expected_row_id'],r['seed']) == _identity(row)}
        candidate, baseline = group['reference'], group['v05_final']
        complete = not candidate['integrity_failures'] and not baseline['integrity_failures']
        trips = complete and candidate['completed_trips'] is not None and baseline['completed_trips'] is not None and candidate['completed_trips'] >= baseline['completed_trips']
        times = complete and candidate['mean_boundary_time_s'] is not None and baseline['mean_boundary_time_s'] is not None and candidate['mean_boundary_time_s'] <= 1.02 * baseline['mean_boundary_time_s'] + 1e-7
        paired.append(dict(row_id=_identity(row)[0], seed=row['seed'], comparison_complete=complete,
            no_completed_trip_deficit=bool(trips), mean_boundary_regression_at_most_two_percent=bool(times), passed=bool(trips and times)))
    reference = [r['summary'] for r in records if r['controller'] == 'reference']
    proof_files = [*bank_files, _file_record(expected_rows_path), _file_record(ledger_path), _file_record(__file__),
        _file_record(ROOT/'scripts/summarize_highway_flow.py')]
    for directory in source_directories:
        proof_files.extend(_file_record(directory/name) for name in ('study.json','config.json','results.json','summary.json'))
    charged_total = sum(c['charged_wall_seconds'] for c in costs)
    within_budget = charged_total <= 1800.
    return dict(schema='highway_flow_compatible_cached_evidence_v1', training=False, optimizer_steps=0,
        evidence_class='Compatible cached evidence from two bank-builder versions; NOT exact single-version native capture',
        selection_rule='Fixed first18 rows ×3 controllers frommatrix02; last9 ×3 frommatrix03; only declared final I110/frozen aggregate-deadline failure may use exact-input repair; no outcome-dependent selection',
        expected_recordings=81, reused_recordings=54, new_recordings=27,
        new_from_matrix03=26 if repair is not None else 27, deadline_repair_recordings=int(repair is not None),
        allocation_complete=len(records)==81,
        source_compatibility=source_proof, controller_table=table, paired_reference_fleet_gates=paired,
        reference_candidate_qualified=within_budget and all(s['passed'] for s in reference) and all(p['passed'] for p in paired),
        preparation_costs=dict(all_attempts=costs, total_charged_wall_seconds=charged_total,
            approved_wall_seconds=1800., within_approved_budget=within_budget,
            historical_failures=failures, all_prior_costs_retained=True),
        superseded_failed_dense_result=deepcopy(failed), superseded_deadline_result=repaired_failure,
        proof_files=proof_files, recordings=records,
        training_authorization=False, readiness_written=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--expected-rows', type=Path, required=True)
    parser.add_argument('--cached-study', type=Path, required=True)
    parser.add_argument('--fresh-study', type=Path, required=True)
    parser.add_argument('--repair-study', type=Path)
    parser.add_argument('--ledger', type=Path, required=True)
    parser.add_argument('--allow-builder-source-change', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'outputs/highway-flow/v01') or output.exists():
        raise ValueError('Use a fresh combined evidence file within highway-flow/v01')
    result = combine(args.expected_rows,args.cached_study,args.fresh_study,args.ledger,
                     allow_builder_change=args.allow_builder_source_change,repair_directory=args.repair_study)
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('allocation_complete','reused_recordings','new_recordings','reference_candidate_qualified','controller_table')}))
    return 0 if result['reference_candidate_qualified'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
