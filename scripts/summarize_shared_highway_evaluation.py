"""Re-score retained comparison reports with native logs; no simulator or actor call."""
import argparse
from copy import deepcopy
import csv
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from experiments import run_highway_merge_recovery_v08 as runtime
from experiments.run_shared_highway_evaluation import digest, read, save, scalar_summary
from traffic.sumo_warnings import native_emergency_report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    args = parser.parse_args()
    run = (ROOT / args.run).resolve()
    if not run.is_relative_to(ROOT / 'outputs'):
        raise ValueError('Use a retained project run')
    output = run / 'native-log-rescore-v01'
    output.mkdir(exist_ok=False)
    base = runtime.base
    base.ruled_gates = runtime.v07.previous.v06_gates
    spec, original = read(run / 'resolved-spec.json'), read(run / 'summary.json')
    groups, table, provenance = {}, [], []
    for candidate in spec['candidates']:
        name = candidate['id']
        reports = []
        for i, row in enumerate(spec['scenarios']):
            case = run / name / f'case-{i:02d}'
            report_path = case / 'report.json'
            report = deepcopy(read(report_path))
            logs = list(case.glob('episode-*/sumo-errors.log'))
            if len(logs) != 1:
                raise ValueError(f'Expected one retained native episode log: {case}')
            report['native_emergencies'] = native_emergency_report(logs[0])
            if not report['native_emergencies']['available']:
                raise ValueError('Native evidence unavailable')
            provenance.append(dict(report=report_path.relative_to(ROOT).as_posix(),
                report_sha256=digest(report_path), native_log=logs[0].relative_to(ROOT).as_posix(),
                native_log_sha256=digest(logs[0])))
            gate = base.ruled_gates(report)
            reports.append(report)
            matches = [q.get('speed_match_fraction', 0.) for q in report.get('quality_merges', {}).values()]
            windows = report['evaluation_jerk_windows']
            def peak(scope, metric):
                return max((w.get(scope, {}).get(metric, {}).get('peak_abs', 0.)
                    for w in windows.values()), default=None)
            admission = report.get('v07_admission', {})
            proof = report.get('cutin_fixture', {})
            table.append(dict(candidate=name, case_index=i, case_id=row['case']['id'],
                seed=row['seed'], stress=bool(row.get('cutin_stress')), passed=gate['passed'],
                merge_match=sum(matches)/len(matches) if matches else None,
                overrides=report['intervention_fraction'],
                peak_realized_jerk=max(c['peak_abs_jerk_m_s3'] for c in gate['per_car'].values()),
                raw_request_jerk_all=peak('0', 'request_vs_actuator_jerk'),
                raw_request_jerk_after5=peak('5', 'request_vs_actuator_jerk'),
                command_slew_peak=peak('0', 'command_slew'),
                native_emergencies=report['native_emergencies']['unique_events'],
                collisions=report['collisions'], teleports=report['teleports'],
                cutin_verified=bool(proof.get('moving_at_handoff') and proof.get('completed_into_target')),
                av_lane_requests_blocked=admission.get('av_blocked'),
                failures=';'.join(gate['failures'])))
        summaries = {}
        for label, subset in [('nominal', [r for r in reports if not r.get('cutin_stress')]),
                              ('cutin', [r for r in reports if r.get('cutin_stress')])]:
            s = base.summarize(subset)
            summaries[label] = dict(**scalar_summary(s),
                strict_cases_passed=sum(g['passed'] for g in s['case_gates']),
                merge_speed_match=runtime.v07.previous.merge_speed_match(subset))
        groups[name] = summaries
        save(output / name / 'evaluation.json', dict(summary=summaries, reports=reports))
    result = dict(schema='shared_highway_comparison_evidence_v01', completed_episodes=len(table),
        training=False, optimizer_steps=0, promoted=False, held_out=False,
        run=run.relative_to(ROOT).as_posix(), elapsed_seconds=original['elapsed_seconds'],
        source_commit=read(run / 'manifest.json')['source_commit'], config_sha256=digest(run / 'resolved-spec.json'),
        source_manifest_sha256=digest(run / 'manifest.json'), candidates=spec['candidates'],
        results=groups, source_evidence=provenance,
        postprocessing_correction=dict(reason='Initial scorer looked in case parent instead of native episode child.',
            changed_fields='Native warning availability/events and dependent acceptance gates only.',
            original_summary_sha256=digest(run / 'summary.json'), original_files_unchanged=True,
            extra_simulator_episodes=0, reducer_sha256=digest(Path(__file__))),
        limits=['Single fixed seed per scenario, development cases only; no held-out claims.',
            'Same v08 lane admission for all candidates; this is a transfer evaluation for v06.',
            'All-decision scoring is primary; startup exclusions are diagnostics only.',
            'All candidates retain failed merge-quality and cut-in comfort checks; no promotion.'])
    save(output / 'result.json', result)
    with (output / 'cases.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(table[0])); writer.writeheader(); writer.writerows(table)
    evidence = ROOT / 'documentation/evidence/shared-highway-evaluation-2026-11-17'
    evidence.mkdir(parents=True, exist_ok=True)
    save(evidence / 'result.json', result)
    (evidence / 'cases.csv').write_bytes((output / 'cases.csv').read_bytes())
    print(json.dumps(groups, indent=2))


if __name__ == '__main__':
    main()
