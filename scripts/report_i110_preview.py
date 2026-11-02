"""Report every frozen preview treatment, failure and complete-cohort burden."""
import argparse
import json
from pathlib import Path
import shutil
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.report_i110_anticipation import read,write,sha,verify_run


def peak_record(run):
    previous={};peak=None
    for line in (run/'trace.jsonl').open(encoding='utf-8'):
        frame=json.loads(line)
        for vid,s in frame['states'].items():
            if not vid.startswith('agent_'):continue
            if vid in previous:
                old_t,old=previous[vid]
                jerk=(s['acceleration']-old['acceleration'])/(frame['time']-old_t)
                if peak is None or abs(jerk)>abs(peak['jerk_m_s3'])+1e-9:
                    peak=dict(time_s=frame['time'],vehicle=vid,jerk_m_s3=jerk,lane_id=s['lane_id'],
                        speed_m_s=s['speed'],leader=s['leader'],realized_acceleration=s['acceleration'],
                        previous_realized_acceleration=old['acceleration'],
                        requested=s.get('requested_acceleration'),applied=s.get('applied_acceleration'),
                        realization_mismatch=s.get('realization_mismatch'),preview=s.get('preview'),
                        terminal_release=s.get('terminal_release'))
            previous[vid]=(frame['time'],s)
    return peak


def report(suite,output):
    suite,output=Path(suite).resolve(),Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    plan=read(suite/'plan.json');verification=read(suite/'verification.json')
    assert verification['status']=='completed' and verification['infrastructure_passed']
    shutil.copyfile(__file__,output/'report-source.py')
    pairs=[];inputs={str(suite/'verification.json'):sha(suite/'verification.json')}
    for case in plan['runs']:
        baseline=ROOT/plan['source_suite']/case;verify_run(baseline)
        b=read(baseline/'metrics.json')
        for radius in [plan['primary_range_m'],plan['sensitivity_range_m']]:
            path=suite/f'{case}-r{radius}';verify_run(path)
            p=read(path/'metrics.json')
            inputs[str(path/'manifest.json')]=sha(path/'manifest.json')
            inputs[str(baseline/'manifest.json')]=sha(baseline/'manifest.json')
            delta={}
            for phase in ('primary','drain'):
                delta[phase]={}
                for group in ['totals',*b[phase]['groups']]:
                    left=b[phase]['totals'] if group=='totals' else b[phase]['groups'][group]
                    right=p[phase]['totals'] if group=='totals' else p[phase]['groups'][group]
                    delta[phase][group]={key:right[key]-value for key,value in left.items()}
            cost=(p['primary']['totals']['boundary_time_s']/b['primary']['totals']['boundary_time_s']-1)*100
            peak=peak_record(path)
            assert abs(abs(peak['jerk_m_s3'])-p['max_controlled_abs_jerk_m_s3'])<1e-7
            pairs.append(dict(case=case,range_m=radius,
                baseline_jerk=b['max_controlled_abs_jerk_m_s3'],preview_jerk=p['max_controlled_abs_jerk_m_s3'],
                baseline_rms=b['jerk_rms_m_s3'],preview_rms=p['jerk_rms_m_s3'],
                baseline_exceedance_steps=b['jerk_exceedance_steps'],preview_exceedance_steps=p['jerk_exceedance_steps'],
                baseline_mismatch_steps=b['realization_mismatch_steps'],preview_mismatch_steps=p['realization_mismatch_steps'],
                primary_boundary_time_change_percent=cost,burden_delta=delta,peak=peak,
                network_gate=p['network_gate_passed'],comfort_gate=p['comfort_gate_passed'],cost_gate=cost<=5+1e-9,
                event_gate=p['event_gate_passed'],interaction_observed=p['interaction_observed'],
                preview_counts=p['preview_counts'],min_gap=p['min_controlled_bumper_gap_m'],min_ttc=p['min_controlled_ttc_s'],
                unchanged_input_hashes={name:sha(path/name) for name in ('network.net.xml','routes.rou.xml','scenario.sumocfg')},
                baseline_ledger=dict(primary=b['primary'],drain=b['drain']),
                preview_ledger=dict(primary=p['primary'],drain=p['drain'])))
    primary=[p for p in pairs if p['range_m']==plan['primary_range_m']]
    summary=dict(study_id=plan['study_id'],status='completed development comparison',
        accepted=all(p['network_gate'] and p['comfort_gate'] and p['cost_gate'] and p['event_gate'] is not False for p in primary),
        primary_comfort_failures=[p['case'] for p in primary if not p['comfort_gate']],
        primary_cost_failures=[p['case'] for p in primary if not p['cost_gate']],
        verification=verification,pairs=pairs,limits=plan['limits'])
    write(output/'summary.json',summary)
    lines=['# Preview control versus frozen smooth ACC','',
        '| Case | Range m | Baseline peak jerk | Preview peak jerk | Primary boundary time change | Comfort | <=5% cost |',
        '|---|---:|---:|---:|---:|---|---|']
    for p in pairs:
        lines.append(f"| {p['case']} | {p['range_m']} | {p['baseline_jerk']:.3f} | {p['preview_jerk']:.3f} | {p['primary_boundary_time_change_percent']:+.3f}% | {'pass' if p['comfort_gate'] else 'FAIL'} | {'pass' if p['cost_gate'] else 'FAIL'} |")
    lines+=['', 'All rows retained. Complete primary and separate-drain totals, role/origin differences and every original/preview trip are in summary.json. This inspected development comparison is not held-out validation.']
    (output/'results.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    write(output/'manifest.json',dict(status='completed',command=[sys.executable,*sys.argv],
        source_sha256=sha(__file__),inputs=inputs,
        outputs={p.name:sha(p) for p in output.iterdir() if p.is_file()},simulator_launched=False))
    print('\n'.join(lines));print('ACCEPTED='+str(summary['accepted']))
    return summary


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--suite',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    report(**vars(p.parse_args()))
