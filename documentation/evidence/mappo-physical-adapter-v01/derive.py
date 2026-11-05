"""Rebuild compact public evidence from immutable local physical-adapter runs."""
import collections
import csv
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[3]
DEST = Path(__file__).resolve().parent
RUNS = ['20261105T204440Z-e1a2ab4f', '20261105T204908Z-32331979']


def main():
    evidence = dict(no_training=True, optimizer_steps=0, runs=[])
    csv_rows = []
    for name in RUNS:
        run = ROOT/'outputs/mappo-physical-adapter/v01'/name
        summary = json.loads((run/'summary.json').read_text())
        manifest = json.loads((run/'manifest.json').read_text())
        record = {k: summary.get(k) for k in ('passed', 'checkpoint_sha256', 'checkpoint_unchanged',
            'calibration', 'python', 'total_wall_s', 'wall_process_s', 'gpu_peak_mib',
            'runtime_error_lines', 'usd_reference_warning_count', 'source_changed_during_run', 'error')}
        record.update(run_id=name, local_artifact=str(run.relative_to(ROOT)).replace('\\', '/'),
                      source_commit=manifest['git_commit'], source_sha256=manifest['source_hashes'],
                      isaac_version=manifest['isaac_version'], gpu=manifest['gpu_before'],
                      config_sha256=hashlib.sha256((run/'resolved-config.json').read_bytes()).hexdigest(), cases=[])
        for case in summary['cases']:
            compact = {k: case.get(k) for k in ('name', 'passed', 'error', 'gates', 'metrics',
                       'completed_simulation_s', 'contact_count', 'static_files_unchanged')}
            decisions_path = run/case['name']/'policy-decisions.json'
            decisions = json.loads(decisions_path.read_text())
            compact['trajectory_sha256'] = hashlib.sha256((run/case['name']/'trajectory.json').read_bytes()).hexdigest()
            compact['policy_decisions_sha256'] = hashlib.sha256(decisions_path.read_bytes()).hexdigest()
            if decisions:
                corrections = sorted(abs(d['requested_acceleration_m_s2']-d['filtered_acceleration_m_s2']) for d in decisions)
                compact['correction_diagnostics'] = dict(
                    causes=dict(collections.Counter(c for d in decisions for c in d['causes'])),
                    median_m_s2=corrections[len(corrections)//2], maximum_m_s2=max(corrections),
                    above_0p05_m_s2=sum(v > .05 for v in corrections))
            record['cases'].append(compact)
            csv_rows.append(dict(run=name, run_passed=summary['passed'], case=case['name'], case_passed=case['passed'],
                                 simulation_s=case['completed_simulation_s'],
                                 failed_gates=';'.join(k for k,v in (case.get('gates') or {}).items() if not v)))
        reopen = run/'reopen-validation.json'
        if reopen.exists():
            check = json.loads(reopen.read_text())
            record['fresh_usd_reopen'] = dict(passed=check['passed'], usd_version=check['usd_version'],
                scenes=[dict(scene=s['scene'], passed=s['passed'], errors=s['errors'],
                             prim_count=len(s['prims']), layers=s['layers']) for s in check['scenes']])
        evidence['runs'].append(record)
    (DEST/'summary.json').write_text(json.dumps(evidence, indent=2)+'\n', encoding='utf-8')
    with (DEST/'results.csv').open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=csv_rows[0])
        writer.writeheader()
        writer.writerows(csv_rows)
    final = ROOT/'outputs/mappo-physical-adapter/v01'/RUNS[-1]
    for source, target in [('identification/capture-025s.png', 'calibration-native.png'),
                           ('scripted_18mps/capture-025s.png', 'scripted-lane-change-native.png'),
                           ('frozen_pass/capture-015s.png', 'frozen-passing-native.png'),
                           ('frozen_pass/capture-025s.png', 'frozen-after-pass-native.png')]:
        shutil.copyfile(final/source, DEST/target)
    shutil.copyfile(final/'resolved-config.json', DEST/'resolved-config.json')
    print(json.dumps(dict(output=str(DEST), runs=len(RUNS), cases=len(csv_rows))))


if __name__ == '__main__':
    main()
