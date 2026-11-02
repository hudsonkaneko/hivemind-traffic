"""Compare the complete v2 matrix with both the unchanged baseline and failed v1."""
import argparse
import json
from pathlib import Path
import shutil
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.report_i110_preview import report
from scripts.report_i110_anticipation import read,write,sha,verify_run


def stop_exposure(run):
    """Sampled controlled vehicle-seconds below 0.1 m/s, not SUMO waiting time."""
    previous={}; primary=0.; drain=0.; horizon=read(run/'resolved-config.json')['horizon_s']
    for line in (run/'trace.jsonl').open(encoding='utf-8'):
        f=json.loads(line)
        for vid,s in f['states'].items():
            if not vid.startswith('agent_'):continue
            if vid in previous and s['speed']<.1:
                dt=f['time']-previous[vid]
                if f['time']<=horizon+1e-8:primary+=dt
                else:drain+=dt
            previous[vid]=f['time']
    return dict(primary_stopped_vehicle_s=primary,additional_drain_stopped_vehicle_s=drain)


def revision(suite, output):
    suite,output=Path(suite).resolve(),Path(output).resolve()
    result=report(suite,output)
    plan=read(suite/'plan.json')
    version=plan.get('display_version','v2')
    old=ROOT/'outputs/i110-preview-20261102/development-v1'
    rows=[];inputs={}
    for pair in result['pairs']:
        if pair['range_m']!=plan['primary_range_m']:continue
        case=pair['case'];v1=old/(case+'-r240');v2=suite/(case+'-r240')
        baseline=ROOT/plan['source_suite']/case
        verify_run(v1)
        for p in (baseline,v1,v2):inputs[str(p/'manifest.json')]=sha(p/'manifest.json')
        previous=read(v1/'metrics.json'); current=read(v2/'metrics.json')
        b=read(baseline/'metrics.json')
        row=dict(case=case,baseline_jerk=pair['baseline_jerk'],v1_jerk=previous['max_controlled_abs_jerk_m_s3'],v2_jerk=pair['preview_jerk'],v1_cost_change_percent=(previous['primary']['totals']['boundary_time_s']/b['primary']['totals']['boundary_time_s']-1)*100,v2_cost_change_percent=pair['primary_boundary_time_change_percent'],v1_stopped=stop_exposure(v1),v2_stopped=stop_exposure(v2),baseline_stopped=stop_exposure(baseline),v1_mismatch_steps=previous['realization_mismatch_steps'],v2_mismatch_steps=current['realization_mismatch_steps'],peak=pair['peak'],burden_delta=pair['burden_delta'],comfort_pass=pair['comfort_gate'],cost_pass=pair['cost_gate'],network_pass=pair['network_gate'],event_pass=pair['event_gate'],preview_counts=pair['preview_counts'])
        rows.append(row)
    summary=dict(study_id=plan['study_id'],candidate_version=version,accepted=result['accepted'],rows=rows,verification=result['verification'],primary_comfort_failures=result['primary_comfort_failures'],primary_cost_failures=result['primary_cost_failures'],limits=plan['limits'],stopped_metric='Sum of observed step intervals with controlled speed <0.1 m/s, primary and added drain separate; not native waiting-time metric.')
    write(output/'v1-v2-comparison.json',summary)
    lines=['# Preview v2 versus unchanged baseline and v1','',
        '| Case | Baseline peak jerk | V1 peak jerk | V2 peak jerk | V1 time cost | V2 time cost | V2 comfort |',
        '|---|---:|---:|---:|---:|---:|---|']
    for r in rows:
        lines.append(f"| {r['case']} | {r['baseline_jerk']:.3f} | {r['v1_jerk']:.3f} | {r['v2_jerk']:.3f} | {r['v1_cost_change_percent']:+.3f}% | {r['v2_cost_change_percent']:+.3f}% | {'pass' if r['comfort_pass'] else 'FAIL'} |")
    lines+=['','Peak jerk is in m/s³. Time cost is paired total fixed-horizon boundary time, including external waiting. All cases retained. See results.md for both observation ranges and summary.json for complete original/treatment primary and drain trip ledgers.','', '## Recovery and stopped exposure','', '| Case | Baseline stopped vehicle-s | V1 stopped vehicle-s | V2 stopped vehicle-s |','|---|---:|---:|---:|']
    for r in rows:
        lines.append(f"| {r['case']} | {r['baseline_stopped']['primary_stopped_vehicle_s']:.1f} | {r['v1_stopped']['primary_stopped_vehicle_s']:.1f} | {r['v2_stopped']['primary_stopped_vehicle_s']:.1f} |")
    lines+=['','This is sampled controlled speed <0.1 m/s in the primary window, not SUMO waiting time. Lower stopped exposure alone is not comfort or safety acceptance. Exact additional-drain exposure is retained in v1-v2-comparison.json.','', '## Full primary-window burdens','', '| Case | Total Δ vehicle-s | Controlled Δ | Human Δ | Ramp-origin Δ | External wait Δ | Unfinished Δ |','|---|---:|---:|---:|---:|---:|---:|']
    for r in rows:
        d=r['burden_delta']['primary']
        lines.append(f"| {r['case']} | {d['totals']['boundary_time_s']:+.1f} | {d['role:controlled']['boundary_time_s']:+.1f} | {d['role:human_proxy']['boundary_time_s']:+.1f} | {d['origin:mlk_ramp']['boundary_time_s']:+.1f} | {d['totals']['external_wait_s']:+.1f} | {d['totals']['unfinished']:+d} |")
    lines+=['','Role and origin overlap: do not add the ramp-origin column to the role totals. Full original and treatment ledgers, including every trip and all drain outcomes, remain in summary.json.']
    lines[0]=f'# Preview {version} versus unchanged baseline and v1'
    lines=[line.replace('V2',version.upper()) for line in lines]
    (output/'revision.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    shutil.copyfile(__file__,output/'revision-source.py')
    write(output/'revision-manifest.json',dict(generator=__file__,generator_sha256=sha(__file__),inputs=inputs,outputs={name:sha(output/name) for name in ['v1-v2-comparison.json','revision.md','revision-source.py']},limits=plan['limits']))
    print('\n'.join(lines[:11]))
    return summary


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--suite',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    revision(**vars(p.parse_args()))
