"""Regenerate compact stress results from immutable manifests and traces."""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def report(verification, output):
    verification = Path(verification).resolve()
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(__file__, output/'report-source.py')
    suite = json.loads((verification/'verification.json').read_text())
    rows = []
    inputs = {str(verification/'verification.json'):digest(verification/'verification.json')}
    for process in suite['runs']:
        run = verification/process['name']
        manifest = json.loads((run/'manifest.json').read_text())
        for file, expected in {**manifest['inputs'], **manifest['outputs']}.items():
            if digest(run/file) != expected:
                raise ValueError('Manifest hash mismatch: '+str(run/file))
        for file, expected in manifest['source_sha256'].items():
            if digest(run/'source'/file) != expected:
                raise ValueError('Source snapshot hash mismatch: '+file)
        inputs[str(run/'manifest.json')] = digest(run/'manifest.json')
        metrics = json.loads((run/'metrics.json').read_text())
        event = json.loads((run/'event-summary.json').read_text())
        config = json.loads((run/'resolved-config.json').read_text())
        counts = defaultdict(int)
        max_error = 0.
        max_braking = 0.
        following = []
        canonical = hashlib.sha256()
        with (run/'trace.jsonl').open(encoding='utf-8') as trace:
            for line in trace:
                canonical.update(line.encode())
                frame = json.loads(line)
                for vid, state in frame['states'].items():
                    if not vid.startswith('agent_'):
                        continue
                    max_braking = max(max_braking, -state['acceleration'])
                    if state.get('realization_mismatch'):
                        counts[vid] += 1
                    if 'predicted_speed' in state:
                        max_error = max(max_error, abs(state['speed']-state['predicted_speed']))
                follower = frame['states'].get('agent_1', {})
                leader = follower.get('leader')
                if leader and leader[0] == 'event_actor':
                    following.append(frame['time'])
        if canonical.hexdigest() != metrics['trace_sha256']:
            raise ValueError('Canonical trace mismatch')
        changes = [dict(change.attrib) for change in ET.parse(run/'lanechanges.xml').getroot()
                   if change.tag == 'change' and change.get('id','').startswith('ramp_')]
        row = dict(run=process['name'], event_id=event['event_id'], seed=process['seed'],
            controller=process['controller'], passed=metrics['passed'],
            network_gate=metrics['network_gate_passed'], event=event,
            comfort_gate=metrics['comfort_gate_passed'], peak_jerk_m_s3=metrics['max_controlled_abs_jerk_m_s3'],
            jerk_exceedance_steps=metrics['jerk_exceedance_steps'], rms_jerk_m_s3=metrics['jerk_rms_m_s3'],
            peak_controlled_braking_m_s2=max_braking,
            realization_mismatch_steps=metrics['realization_mismatch_steps'],
            mismatch_steps_by_vehicle=dict(counts) if process['controller']=='smooth_acc' else None,
            max_abs_requested_realized_speed_error_m_s=max_error if process['controller']=='smooth_acc' else None,
            min_bumper_gap_m=metrics['min_controlled_bumper_gap_m'],min_closing_ttc_s=metrics['min_controlled_ttc_s'],
            requested=metrics['primary']['totals']['requested'],
            primary=metrics['primary']['totals'],drain=metrics['drain']['totals'],
            primary_groups=metrics['primary']['groups'],drain_groups=metrics['drain']['groups'],
            end_s=metrics['drain']['time'],collisions=metrics['collisions'],
            emergency_stop_ids=metrics['emergency_stop_ids'],
            follower_observed_first_s=min(following) if following else None,
            follower_observed_last_s=max(following) if following else None,
            ramp_realized_lane_changes=changes,
            interface_observations=metrics['interface_observations'],
            manifest=str(run/'manifest.json'),trace_sha256=metrics['trace_sha256'],
            event_spec_sha256=manifest['inputs']['event-spec.json'],source_sha256=manifest['source_sha256'])
        rows.append(row)
    pairs = []
    for row in rows:
        if row['controller'] != 'native_acc':
            continue
        smooth = next(r for r in rows if r['event_id']==row['event_id'] and r['seed']==row['seed'] and r['controller']=='smooth_acc')
        pairs.append(dict(event_id=row['event_id'],seed=row['seed'],
            delta_peak_jerk_smooth_minus_native=smooth['peak_jerk_m_s3']-row['peak_jerk_m_s3'],
            delta_boundary_time_H_s=smooth['primary']['boundary_time_s']-row['primary']['boundary_time_s'],
            delta_ramp_boundary_time_H_s=smooth['primary_groups']['origin:mlk_ramp']['boundary_time_s']-row['primary_groups']['origin:mlk_ramp']['boundary_time_s'],
            limit='Descriptive paired fixture contrast; neither traffic benefit nor generalization.'))
    summary=dict(study_id='i110-acc-stress-20261028',verification=str(verification),
        suite={k:v for k,v in suite.items() if k!='runs'},runs=rows,paired_differences=pairs,
        limits='Synthetic scripted engineering; SUMO internal intervention causes unavailable. No empirical safety/comfort, learned actor, ramp-conflict acceptance, traffic benefit or USD/physics event agreement.')
    (output/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    table=['# I-110 stress tranche: generated results','',
        'All rows, including the repeat and comfort failures, are retained. H = 60 s; drain is separate. Jerk is the controlled-fleet peak. Native internal overrides are unavailable, not zero.','',
        '| Run | Event gate | Completed at H / requested | Completed after drain | Peak jerk (m/s³) | Smooth request mismatches |',
        '|---|---|---|---|---|---|']
    for r in rows:
        table.append(f"| {r['run']} | {r['event']['event_gate_passed']} | {r['primary']['completed']} / {r['requested']} | {r['drain']['completed']} | {r['peak_jerk_m_s3']:.3f} | {r['realization_mismatch_steps']} |")
    table += ['', 'These are synthetic fixtures. Comfort failures do not invalidate retained collision/trip accounting evidence or establish road safety.', '',
              '## Paired descriptive differences (smooth minus native)', '',
              '| Event | Seed | Peak jerk difference | Boundary time at H difference (s) | Ramp burden at H difference (s) |',
              '|---|---|---|---|---|']
    for p in pairs:
        table.append(f"| {p['event_id']} | {p['seed']} | {p['delta_peak_jerk_smooth_minus_native']:.3f} | {p['delta_boundary_time_H_s']:.3f} | {p['delta_ramp_boundary_time_H_s']:.3f} |")
    (output/'results.md').write_text('\n'.join(table)+'\n',encoding='utf-8')
    (output/'manifest.json').write_text(json.dumps(dict(status='completed',inputs=inputs,
        outputs={p.name:digest(p) for p in output.iterdir() if p.is_file()}),indent=2)+'\n')
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--verification',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    result=report(**vars(parser.parse_args()))
    print(json.dumps(result['suite'],indent=2))
