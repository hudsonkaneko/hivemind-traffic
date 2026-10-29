"""Hash-verify runs and derive ramp exposure and cut-in mechanism evidence."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))
def write(path,value):Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')


def trace_diagnostics(path):
    previous={};peak=None;ego=[];physical=[];canonical=hashlib.sha256()
    with Path(path).open(encoding='utf-8') as f:
        for line in f:
            canonical.update(line.encode());frame=json.loads(line)
            t=frame['time'];states=frame['states']
            for vid,s in states.items():
                if vid.startswith('agent_') and vid in previous:
                    old=previous[vid];jerk=(s['acceleration']-old['acceleration'])/.1
                    if peak is None or abs(jerk)>abs(peak['jerk_m_s3']):
                        peak=dict(time_s=t,vehicle=vid,jerk_m_s3=jerk,before=old,after=s)
            if 'agent_1' in states and t<=6:
                ego.append(dict(time_s=t,**states['agent_1']))
            physical.append(dict(time=t,states={v:{k:s[k] for k in
                ('speed','acceleration','x','y','lane_id','road_id')} for v,s in states.items()}))
            previous=states
    return dict(peak=peak,agent1_first6s=ego,canonical_trace_sha256=canonical.hexdigest()),physical


def physical_difference(first,second):
    if len(first)!=len(second):return dict(same_frame_count=False)
    maxima={k:0. for k in ('speed','acceleration','x','y')};lane_differences=0;identity_differences=0
    for a,b in zip(first,second):
        if a['time']!=b['time']:raise ValueError('Unaligned trace clocks')
        identity_differences+=set(a['states'])!=set(b['states'])
        for vid in a['states'].keys() & b['states'].keys():
            x,y=a['states'][vid],b['states'][vid]
            for key in maxima:maxima[key]=max(maxima[key],abs(x[key]-y[key]))
            lane_differences+=x['lane_id']!=y['lane_id']
    return dict(same_frame_count=True,identity_difference_steps=identity_differences,
        lane_difference_samples=lane_differences,max_abs_differences=maxima,
        physical_match_within_1e_minus_9=not identity_differences and not lane_differences and max(maxima.values())<=1e-9)


def report(verification,output):
    verification=Path(verification).resolve();output=Path(output).resolve()
    output.mkdir(parents=True,exist_ok=False);shutil.copyfile(__file__,output/'report-source.py')
    suite=read(verification/'verification.json');rows=[];inputs={str(verification/'verification.json'):sha(verification/'verification.json')}
    cut_traces={};cut_diagnostics={}
    for spec in suite['runs']:
        run=verification/spec['name'];manifest=read(run/'manifest.json');m=read(run/'metrics.json')
        for p,h in {**manifest['inputs'],**manifest['outputs']}.items():
            if sha(run/p)!=h:raise ValueError('Run byte mismatch: '+str(run/p))
        for p,h in manifest['source_sha256'].items():
            if sha(run/'source'/p)!=h:raise ValueError('Source mismatch: '+p)
        inputs[str(run/'manifest.json')]=sha(run/'manifest.json')
        diag,physical=trace_diagnostics(run/'trace.jsonl')
        if diag['canonical_trace_sha256']!=m['trace_sha256']:raise ValueError('Canonical trace mismatch')
        if abs(abs(diag['peak']['jerk_m_s3'])-m['max_controlled_abs_jerk_m_s3'])>1e-8:
            raise ValueError('Derived peak differs from runner')
        interaction=read(run/'interaction-summary.json') if (run/'interaction-summary.json').exists() else None
        changes=[dict(e.attrib) for e in ET.parse(run/'lanechanges.xml').getroot() if e.tag=='change' and e.get('id','').startswith('ramp_')]
        row={k:spec[k] for k in ('name','kind','seed','controller','depart','lateral')}
        row.update(network_gate=m['network_gate_passed'],comfort_gate=m['comfort_gate_passed'],
            event_gate=m['event_gate_passed'],collisions=m['collisions'],
            peak_jerk=m['max_controlled_abs_jerk_m_s3'],mismatches=m['realization_mismatch_steps'],
            min_gap=m['min_controlled_bumper_gap_m'],min_ttc=m['min_controlled_ttc_s'],
            primary=m['primary']['totals'],drain=m['drain']['totals'],
            primary_groups=m['primary']['groups'],drain_groups=m['drain']['groups'],
            agent0_primary=next(t for t in m['primary']['trips'] if t['vehicle_id']=='agent_0'),
            end_s=m['drain']['time'],interaction=interaction,ramp_lane_changes=changes,
            peak_frame=diag['peak'],trace_sha256=m['trace_sha256'],source_sha256=manifest['source_sha256'])
        rows.append(row)
        if spec['kind']=='cutin':cut_traces[spec['name']]=physical;cut_diagnostics[spec['name']]=diag
    comparisons={}
    for lateral in (0,1):
        a=f'cutin-smooth_acc-{lateral}';b=f'cutin-follow_cap_acc-{lateral}'
        comparisons[str(lateral)]=physical_difference(cut_traces[a],cut_traces[b])
    old=ROOT/'outputs/i110-acc-stress-20261028/verification-v1/cutin-v1-731101-smooth_acc'
    old_manifest=read(old/'manifest.json')
    if sha(old/'trace.jsonl')!=old_manifest['outputs']['trace.jsonl']:raise ValueError('Old trace changed')
    old_diag,_=trace_diagnostics(old/'trace.jsonl');inputs[str(old/'manifest.json')]=sha(old/'manifest.json')
    paired=[]
    for row in rows:
        if row['kind']!='ramp' or row['controller']!='native_acc':continue
        other=next(r for r in rows if r['kind']=='ramp' and r['seed']==row['seed'] and r['depart']==row['depart'] and r['controller']=='smooth_acc')
        paired.append(dict(seed=row['seed'],depart=row['depart'],
            boundary_delta_s=other['primary']['boundary_time_s']-row['primary']['boundary_time_s'],
            ramp_boundary_delta_s=other['primary_groups']['origin:mlk_ramp']['boundary_time_s']-row['primary_groups']['origin:mlk_ramp']['boundary_time_s'],
            peak_jerk_delta=other['peak_jerk']-row['peak_jerk']))
    result=dict(study_id='i110-ramp-cutin-20261029',suite={k:v for k,v in suite.items() if k!='runs'},
        runs=rows,ramp_paired_differences=paired,cutin_physical_comparisons=comparisons,
        cutin_diagnostics=cut_diagnostics,previous_failure=old_diag,
        limits=suite['limits'],causal_scope='Only matched controller and lateral-command interventions. No claim to uniquely identify SUMO internal branches or human gap decisions.')
    write(output/'summary.json',result)
    table=['# Ramp interaction and cut-in diagnosis: generated results','',
        'H = 90 s. Completion after a separate drain is not completion at H. Exposure means a direct ramp/agent_0 leader relation within 120 m on sb2/sb3.','',
        '| Run | Exposure | Completed at H / requested | Completed after drain | Peak jerk (m/s³) | Mismatches |',
        '|---|---|---|---|---|---|']
    for r in rows:
        exposure=r['interaction']['interaction_observed'] if r['interaction'] else None
        table.append(f"| {r['name']} | {exposure} | {r['primary']['completed']} / {r['primary']['requested']} | {r['drain']['completed']} | {r['peak_jerk']:.4f} | {r['mismatches']} |")
    table += ['','Native intervention counts are unavailable. Blocked-desire timestep counts are not independent rejected opportunities. See summary.json for all subgroup burdens, peak frames and realized native lane changes.']
    (output/'results.md').write_text('\n'.join(table)+'\n',encoding='utf-8')
    os.environ.setdefault('MPLCONFIGDIR',str(output/'matplotlib-cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,1,figsize=(9,6),sharex=True,layout='constrained')
    for mode,color in [('smooth_acc','#0072B2'),('follow_cap_acc','#D55E00')]:
        trace=cut_diagnostics[f'cutin-{mode}-1']['agent1_first6s']
        times=[r['time_s'] for r in trace]
        axes[0].plot(times,[r.get('applied_acceleration',float('nan')) for r in trace],label=mode+' requested',color=color)
        axes[1].plot(times,[r['acceleration'] for r in trace],label=mode+' realized',color=color,ls='--' if mode=='follow_cap_acc' else '-')
    control=cut_diagnostics['cutin-smooth_acc-0']['agent1_first6s']
    axes[1].plot([r['time_s'] for r in control],[r['acceleration'] for r in control],label='same actor, no cut-in',color='#009E73',ls=':')
    for ax in axes:
        ax.set_ylabel('Acceleration (m/s²)');ax.grid(alpha=.25);ax.legend(fontsize=8)
        ax.set_xlim(1.5,3.5)
    axes[1].set_xlabel('Simulation time (s); values aligned to resulting step')
    fig.suptitle('Synthetic cut-in: command changes versus realized braking\nSingle synthetic seed; no real-world comfort or safety claim',fontsize=11)
    fig.savefig(output/'cutin-control-response.png',dpi=170);plt.close(fig)
    write(output/'manifest.json',dict(status='completed',inputs=inputs,
        outputs={p.name:sha(p) for p in output.iterdir() if p.is_file()}))
    print(json.dumps(dict(runs=len(rows),physical_comparisons=comparisons,unexposed=suite['unexposed_shifted_cases']),indent=2))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--verification',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);report(**vars(parser.parse_args()))
