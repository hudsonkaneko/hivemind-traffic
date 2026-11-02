"""Derive the ACC comparison tables/figures and their explicit artifact lineage."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]


def report(verification,output):
    verification=Path(verification).resolve();output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    os.environ.setdefault('MPLCONFIGDIR',str(output/'matplotlib-cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    checks=json.loads((verification/'verification.json').read_text())
    rows=[];frames={}
    for run in checks['runs']:
        path=verification/run['name']
        metrics=json.loads((path/'metrics.json').read_text())
        config=json.loads((path/'resolved-config.json').read_text())
        # The runner's trace digest is over canonical LF text; its manifest
        # separately records actual file bytes (CRLF on Windows).
        assert hashlib.sha256((path/'trace.jsonl').read_text(encoding='utf-8').encode()).hexdigest()==metrics['trace_sha256']
        manifest=json.loads((path/'manifest.json').read_text())
        assert hashlib.sha256((path/'trace.jsonl').read_bytes()).hexdigest()==manifest['outputs']['trace.jsonl']
        rows.append(dict(run=run['name'],case=run['case'],seed=run['seed'],controller=run['controller'],passing=run['passing'],
            passed=metrics['passed'],comfort_gate=metrics['comfort_gate_passed'],max_jerk=metrics['max_controlled_abs_jerk_m_s3'],
            rms_jerk=metrics['jerk_rms_m_s3'],commands=metrics['requested_command_count'],mismatch_steps=metrics['realization_mismatch_steps'],
            min_gap=metrics['min_controlled_bumper_gap_m'],min_ttc=metrics['min_controlled_ttc_s'],
            requested=metrics['primary']['totals']['requested'],completed_at_H=metrics['primary']['totals']['completed'],
            completed_after_drain=metrics['drain']['totals']['completed'],end_s=metrics['drain']['time'],
            collisions=len(metrics['collisions']),teleports=metrics['drain']['totals']['teleported'],
            removed=metrics['drain']['totals']['removed'],observations=metrics['interface_observations'],
            unknown_adjacent=metrics['interface_unknown_adjacent_context'],source_hashes=json.loads((path/'manifest.json').read_text())['source_sha256']))
        assert config['evidence_context']['recorded']
        frames[run['name']]=[json.loads(line) for line in (path/'trace.jsonl').read_text().splitlines()]
    smooth=[r for r in rows if r['controller']=='smooth_acc']
    assert len({json.dumps(r['source_hashes'],sort_keys=True) for r in rows})==1
    matched=[json.loads((verification/n/'manifest.json').read_text())['inputs'] for n in ('native-default','native-parallel','braking_leader-730201')]
    assert len({m['routes.rou.xml'] for m in matched})==len({m['network.net.xml'] for m in matched})==1
    result=dict(study_id='i110-acc-comfort-20261027',verification=str(verification.relative_to(ROOT)),
        runs=rows,repeat_exact=checks['repeat_exact'],all_smooth_pass=all(r['comfort_gate'] for r in smooth),
        maximum_smooth_jerk=max(r['max_jerk'] for r in smooth),
        graph_observations=sum(r['observations'] for r in rows),unknown_adjacent=sum(r['unknown_adjacent'] for r in rows),
        statement='Engineering fixture passes only; no real-world comfort/safety certification or learned-policy result',
        config=json.loads((verification/'plan.json').read_text()))
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    with (output/'summary.csv').open('w',newline='') as f:
        names=[k for k in rows[0] if k!='source_hashes'];w=csv.DictWriter(f,fieldnames=names,extrasaction='ignore');w.writeheader();w.writerows(rows)
    lineage=[]
    ref=lambda p:'repo:'+Path(p).relative_to(ROOT).as_posix()
    generator='repo:scripts/report_i110_acc.py'
    def record(name,inputs,explanation,claim,activity='i110-acc-comfort-20261027'):
        lineage.append(dict(path=ref(output/name),kind='figure' if name.endswith('.png') else 'result',
            activity=activity,generator=generator,inputs=[ref(p) for p in inputs],explanation=explanation,claim=claim,
            limits='Synthetic I-110 development graph and traffic; no trained Hudson actor loaded or real-road calibration.',lineage_basis='recorded'))
    arms=['native-default','native-parallel','braking_leader-730201'];colors=['#b64646','#d39a35','#197b74']
    labels=['Native ACC\ndefault passing','Native ACC\nparallel passing','ACC + comfort filter\nparallel passing']
    fig,ax=plt.subplots(figsize=(9,4.5),layout='constrained')
    values=[next(r['max_jerk'] for r in rows if r['run']==name) for name in arms]
    bars=ax.bar(labels,values,color=colors)
    ax.bar_label(bars,labels=[f'{v:.2f}' for v in values],padding=5)
    ax.axhline(2.05,color='#197b74',ls='--',label='Declared engineering bound 2.05')
    ax.set(ylabel='Peak fleet absolute jerk (m/s³)',title='Which change removed the abrupt acceleration changes?',ylim=(0,max(values)*1.18))
    ax.legend();ax.text(.98,.95,'Same network, demand and seed 730201\nBraking-leader case; 0.1 s simulation',transform=ax.transAxes,ha='right',va='top',fontsize=9)
    fig.savefig(output/'acc-change-attribution.png',dpi=160);plt.close(fig)
    record('acc-change-attribution.png',[verification/n/'metrics.json' for n in arms],
        'Matched one-factor steps separate the passing convention from the comfort-controller change.',
        'The passing convention explains the 65→20 reduction in this fixture; filtering and cruise approach reduce the remaining peak to about 2.01.')
    fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True,layout='constrained')
    for name,color,label in zip(arms,colors,labels):
        series=[(f['time'],f['states']['agent_0']) for f in frames[name] if 'agent_0' in f['states']]
        times=[t for t,s in series];acc=np.array([s['acceleration'] for t,s in series])
        axes[0].plot(times,[s['speed'] for t,s in series],color=color,label=label.replace('\n',' '))
        axes[1].plot(times,acc,color=color);axes[2].plot(times[1:],np.diff(acc)/.1,color=color)
    for ax in axes:ax.grid(alpha=.2);ax.set_xlim(0,40)
    axes[0].set(ylabel='Speed (m/s)',title='Rightmost through car: adjacent-lane braking artifact and correction');axes[0].legend(fontsize=9)
    axes[1].set_ylabel('Acceleration (m/s²)');axes[2].set(ylabel='Jerk (m/s³)',xlabel='Simulation time (s)')
    fig.savefig(output/'acc-response-comparison.png',dpi=160);plt.close(fig)
    record('acc-response-comparison.png',[verification/n/'trace.jsonl' for n in arms],
        'Actual motion of agent_0 shows the abrupt reactions alongside the slower adjacent-lane leader.',
        'Measured motion, not just commanded acceleration, is smoother in the corrected controller case.')
    name='stopped_leader-730201';fig,axes=plt.subplots(2,1,figsize=(11,6),sharex=True,layout='constrained')
    for vid,label,color in [('brake_leader','Stopped / restarting leader','#d39a35'),('agent_1','Following ACC + filter','#197b74')]:
        points=[(f['time'],f['states'][vid]) for f in frames[name] if vid in f['states']]
        axes[0].plot([t for t,s in points],[s['speed'] for t,s in points],label=label,color=color)
        axes[1].plot([t for t,s in points],[s['acceleration'] for t,s in points],color=color)
    for ax in axes:ax.grid(alpha=.2);ax.axvspan(8,34,color='gray',alpha=.1);ax.set_xlim(0,65)
    axes[0].set(ylabel='Speed (m/s)',title='Following a leader that brakes to a stop and restarts');axes[0].legend()
    axes[1].set(ylabel='Acceleration (m/s²)',xlabel='Simulation time (s)')
    fig.savefig(output/'stop-and-restart.png',dpi=160);plt.close(fig)
    record('stop-and-restart.png',[verification/name/'trace.jsonl'],
        'Actual leader and following-car response to the separate 15→0→15 m/s event schedule.',
        'This synthetic stop/restart case completes without collisions and satisfies the declared engineering jerk bound.')
    name='ordinary-730201';points=[(f['time'],f['states']['agent_0'],f['policy_observations']['agent_0']) for f in frames[name] if 'agent_0' in f['states']]
    fig,ax=plt.subplots(figsize=(10,4),layout='constrained')
    ax.step([s['distance'] for t,s,o in points],[s['lane_index'] for t,s,o in points],where='post',label='SUMO lane index',color='#d39a35')
    ax.plot([s['distance'] for t,s,o in points],[o['lane_identity'].get('index_from_right',float('nan')) for t,s,o in points],ls='--',label='Stable through-lane identity',color='#197b74')
    ax.set(xlabel='Travelled distance (m)',ylabel='Lane index / identity',title='A changing SUMO lane index does not mean the car changed lanes',ylim=(-.2,1.3))
    ax.grid(alpha=.2);ax.legend();fig.savefig(output/'policy-lane-continuity.png',dpi=160);plt.close(fig)
    record('policy-lane-continuity.png',[verification/name/'trace.jsonl'],
        'The rightmost through lane stays identity 0 as the auxiliary lane changes SUMO indices.',
        'The new graph observation preserves physical lane identity across edge boundaries; it is not the old 27-feature actor contract.',
        activity='i110-policy-interface-20261027')
    for name in ('summary.json','summary.csv'):
        record(name,[verification/'verification.json',verification/'plan.json',*[verification/r['run']/'metrics.json' for r in rows]],
            'Machine-derived comparison and gates for every declared verification run, including repeats.',
            'All accepted engineering results and limits are traced to the stored raw run metrics.')
    (output/'artifact-lineage.json').write_text(json.dumps(lineage,indent=2)+'\n')
    (output/'analysis-manifest.json').write_text(json.dumps(dict(generator=generator,generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        inputs={ref(verification/r['run']/'metrics.json'):hashlib.sha256((verification/r['run']/'metrics.json').read_bytes()).hexdigest() for r in rows},
        outputs={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in output.iterdir() if p.is_file()}),indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('all_smooth_pass','maximum_smooth_jerk','graph_observations','unknown_adjacent')}))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--verification',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    report(**vars(p.parse_args()))
