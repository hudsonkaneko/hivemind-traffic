"""Plot actual pilot evidence after strict report/package revalidation.

python scripts/plot_lane_follow_ppo.py --training TRAIN --selection-report SELECT \
    --heldout-report HELDOUT --output outputs/lab_lane_ppo/reports/outcomes.png

Matplotlib is optional and imported only for rendering. An existing compatible
package directory can be supplied with --matplotlib-path; nothing is installed.
--check-only verifies/reduces inputs without importing any plotting dependency.
The PNG and adjacent JSON are new exclusive outputs. No physics image is made.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from experiments.lane_follow_ppo_evaluation import FAILURE_REASONS, HELD_OUT_SEEDS, ROUTE_IDS, UPDATES, case_label
from experiments.lane_follow_ppo_protocol import digest
from experiments.report_lane_follow_ppo import _read, assemble_held_out


def require(condition, message):
    if not condition:
        raise ValueError(message)


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def outcome(row):
    require(isinstance(row, dict) and row.get('evidence_valid') is True, 'Invalid episode evidence cannot be plotted')
    reason = row.get('terminal_reason')
    require(isinstance(reason, str) and reason in FAILURE_REASONS | {'success','time_limit'}, 'Unknown terminal reason')
    require(type(row.get('driving_success')) is bool and type(row.get('physical_failure')) is bool, 'Missing driving outcome')
    require(row['physical_failure'] == (reason in FAILURE_REASONS)
            and (not row['driving_success'] or reason == 'success'), 'Inconsistent driving outcome')
    progress, rms = row['metrics']['progress_m'], row['metrics']['driving_lateral_rms_m']
    require(finite(progress) and (rms is None or finite(rms) and rms >= 0), 'Nonfinite plot metric')
    return dict(reason=reason, success=row['driving_success'], failure=row['physical_failure'], progress_m=progress, rms_m=rms)


def chart_data(episodes, training_summary, selection, heldout):
    """Pure-data layout input; no drawing, files, runtime, or reward proxy."""
    require(isinstance(episodes, list) and bool(episodes), 'No completed training episodes')
    state = training_summary['training_state']
    require(state['control_steps'] == 65536 and state['completed_updates'] == 32, 'Incomplete approved training budget')
    require(selection.get('valid') is True and heldout.get('valid') is True
            and heldout.get('held_out_used_for_selection') is False, 'Invalid or contaminated comparison')
    require(selection['selected_checkpoint'] == heldout['selected_checkpoint'], 'Selected checkpoint differs')
    training = []
    previous = 0
    for row in episodes:
        steps = row['global_control_step']
        require(type(steps) is int and previous < steps <= 65536, 'Training episode step order differs')
        training.append(dict(step=steps, **outcome(row)))
        previous = steps
    rankings = selection['rankings']
    require(len(rankings) == 4 and {r['checkpoint']['update'] for r in rankings} == set(UPDATES), 'Four checkpoint results required')
    validation = []
    for row in sorted(rankings,key=lambda r:r['checkpoint']['update']):
        metrics = row['aggregate']
        require(metrics['episodes'] == 3 and finite(metrics['mean_progress_m'])
                and finite(metrics['progress_m']['std_population']), 'Incomplete selection samples')
        validation.append(dict(update=row['checkpoint']['update'], progress_m=metrics['mean_progress_m'],
            progress_std_m=metrics['progress_m']['std_population'], successes=metrics['driving_successes'],
            failures=metrics['physical_failures'], count=metrics['episodes']))
    update = selection['selected_checkpoint']['update']
    pairs=[]
    expected={case_label('heldout',policy,route,seed,update if policy=='ppo' else None)
              for policy in ('ppo','scripted') for route in ROUTE_IDS for seed in HELD_OUT_SEEDS}
    require(set(heldout['cases']) == expected, 'Held-out plot needs every matched episode')
    for route in ROUTE_IDS:
        for seed in HELD_OUT_SEEDS:
            pair=dict(route_id=route,seed=seed)
            for policy in ('ppo','scripted'):
                label=case_label('heldout',policy,route,seed,update if policy=='ppo' else None)
                require(heldout['cases'][label]['valid'] is True, 'Invalid paired case')
                pair[policy]=outcome(heldout['cases'][label]['outcome'])
            pairs.append(pair)
    partial=training_summary.get('partial_episode_control_steps')
    require(type(partial) is int and partial >= 0 and previous+partial == 65536,
            'Completed and partial training episode counts do not cover the budget')
    return dict(title='PPO pilot: measured outcomes', training=training, validation=validation,
        pairs=pairs, selected_update=update, partial_training_steps=partial,
        control_steps=65536, training_seed=2101,
        interpretation='Progress and tracking include failed episodes. Low tracking error alone is not successful driving. No improvement is inferred from reward.')


def load_chart_data(training_path, selection_path, heldout_path):
    paths=[Path(p).resolve() for p in (training_path,selection_path,heldout_path)]
    require(all(p.is_relative_to(ROOT/'outputs/lab_lane_ppo') for p in paths), 'Inputs must belong to this pilot checkout')
    training,selection_path,heldout_path=paths
    saved=_read(heldout_path)
    require(saved.get('kind') == 'heldout' and isinstance(saved.get('packages'),list), 'Not a strict held-out report')
    # This also revalidates all training/selection packages, all4checkpoint
    # identities, and all30held-out raw episodes before trusting report metrics.
    regenerated=assemble_held_out(training,selection_path,[ROOT/row['path'] for row in saved['packages']])
    require(saved == regenerated, 'Held-out report does not reproduce from immutable evidence')
    selection=_read(selection_path)
    episodes=_read(training/'training-episodes.json')
    summary=_read(training/'summary.json')
    data=chart_data(episodes,summary,selection['report'],saved['report'])
    data['provenance']=dict(training=training.relative_to(ROOT).as_posix(),
        training_manifest_sha256=digest(training/'manifest.json'),
        training_episodes_sha256=digest(training/'training-episodes.json'),
        selection_report=selection_path.relative_to(ROOT).as_posix(),selection_sha256=digest(selection_path),
        heldout_report=heldout_path.relative_to(ROOT).as_posix(),heldout_sha256=digest(heldout_path),
        plot_source_sha256=digest(Path(__file__)))
    return data


def render_chart(data, output, *, matplotlib_path=None):
    output=Path(output).resolve()
    require(output.is_relative_to(ROOT/'outputs/lab_lane_ppo') and output.suffix.lower()=='.png',
            'Plot must be a new PNG inside outputs/lab_lane_ppo')
    for parent in output.parents:
        if parent == ROOT:break
        require(not (parent/'manifest.json').exists(), 'Cannot change a completed evidence package')
    sidecar=output.with_suffix('.json')
    require(not output.exists() and not sidecar.exists(), 'Plot or sidecar already exists; choose a new name')
    os.environ['MPLCONFIGDIR']=str(ROOT/'outputs/lab_lane_ppo/review/matplotlib-cache')
    if matplotlib_path is not None:
        sys.path.append(str(Path(matplotlib_path).resolve()))
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.lines import Line2D
    except ImportError as error:
        raise ValueError('Matplotlib is optional and unavailable in this interpreter; use an existing compatible package directory') from error
    plt.rcParams.update({'font.size':10,'axes.titlesize':12,'axes.labelsize':10,
        'axes.spines.top':False,'axes.spines.right':False,'figure.facecolor':'#f7f8fa',
        'axes.facecolor':'white','grid.color':'#e1e5eb','grid.linewidth':.7})
    figure,axes=plt.subplots(2,2,figsize=(14,9.4))
    blue,green,orange,gray='#3066be','#18756b','#ce6b29','#718096'
    training=data['training']
    ax=axes[0,0]
    ax.plot([r['step'] for r in training],[r['progress_m'] for r in training],color='#c8ced8',lw=.8,zorder=1)
    groups=[('Successful stop',green,lambda r:r['success']),
            ('Physical guard termination',orange,lambda r:r['failure']),
            ('Timeout',blue,lambda r:r['reason']=='time_limit'),
            ('Stop below quality gate',gray,lambda r:r['reason']=='success' and not r['success'])]
    for label,color,predicate in groups:
        points=[r for r in training if predicate(r)]
        if points:ax.scatter([r['step'] for r in points],[r['progress_m'] for r in points],s=16,color=color,label=f'{label}: {len(points)}',zorder=2)
    ax.set(title=f'Training: {len(training)} completed episodes',xlabel='Global control transitions',ylabel='Terminal route progress (m)',xlim=(0,65536))
    ax.legend(fontsize=8,loc='best',frameon=False)
    ax.text(.02,-.24,f'Incomplete final episode: {data["partial_training_steps"]} transitions, excluded from points.',transform=ax.transAxes,fontsize=8,color=gray)
    ax=axes[0,1]
    validation=data['validation']
    bars=ax.bar(range(4),[r['progress_m'] for r in validation],yerr=[r['progress_std_m'] for r in validation],
                color=['#3066be' if r['update']==data['selected_update'] else '#bcc9dd' for r in validation],capsize=4,width=.62)
    for bar,row in zip(bars,validation):
        guards=f'{row["failures"]} guard termination'+('' if row['failures']==1 else 's')
        ax.annotate(f'{row["successes"]}/{row["count"]} success\n{guards}',
            (bar.get_x()+bar.get_width()/2,max(0,bar.get_height()+row['progress_std_m'])),
            xytext=(0,7),textcoords='offset points',ha='center',fontsize=8)
    upper=max(1.,max(r['progress_m']+r['progress_std_m'] for r in validation))
    ax.set_ylim(min(-.1,min(r['progress_m']-r['progress_std_m'] for r in validation)),upper*1.35+.3)
    ax.set_xticks(range(4),[str(r['update'])+(' (selected)' if r['update']==data['selected_update'] else '') for r in validation])
    ax.set(title='Selection: left route, 3 fixed seeds per checkpoint',xlabel='Completed PPO updates',ylabel='Mean terminal progress ± population SD (m)')
    names=['Straight (transfer)','Left (trained route)','Right (transfer)']
    pairs=data['pairs']
    successes={p:sum(row[p]['success'] for row in pairs) for p in ('ppo','scripted')}
    failures={p:sum(row[p]['failure'] for row in pairs) for p in ('ppo','scripted')}
    timeouts={p:sum(row[p]['reason']=='time_limit' for row in pairs) for p in ('ppo','scripted')}
    for ax,metric,title,ylabel in [(axes[1,0],'progress_m','Held-out: matched route progress','Terminal route progress (m)'),
                                  (axes[1,1],'rms_m','Held-out: tracking on each observed episode','Post-settle lateral RMS (m)')]:
        missing={p:0 for p in ('ppo','scripted')}
        for route_index,route in enumerate(ROUTE_IDS):
            route_pairs=[row for row in pairs if row['route_id']==route]
            for seed_index,row in enumerate(route_pairs):
                jitter=(seed_index-2)*.035
                x1,x2=route_index-.18+jitter,route_index+.18+jitter
                a,b=row['ppo'][metric],row['scripted'][metric]
                if a is not None and b is not None:ax.plot([x1,x2],[a,b],color='#ccd2dc',lw=.8,zorder=1)
                for policy,x,color in [('ppo',x1,blue),('scripted',x2,green)]:
                    value=row[policy][metric]
                    if value is None:missing[policy]+=1
                    else:ax.scatter(x,value,color=color,s=27,zorder=2,
                        marker='x' if row[policy]['failure'] else 'o',
                        label=policy.upper() if route_index==0 and seed_index==0 else None)
        ax.set_xticks(range(3),names)
        ax.set(title=title,ylabel=ylabel,xlim=(-.55,2.55))
        ax.legend(handles=[Line2D([0],[0],marker='o',linestyle='none',color=blue,label='PPO'),
                           Line2D([0],[0],marker='o',linestyle='none',color=green,label='SCRIPTED')],
                  frameon=False,fontsize=8)
        note='5 matched seeds per route; × = physical guard termination.' if metric=='progress_m' else f'No post-settle RMS: PPO {missing["ppo"]}, scripted {missing["scripted"]}; no values imputed.'
        ax.text(.02,-.24,note,transform=ax.transAxes,fontsize=8,color=gray)
    for ax in axes.flat:ax.grid(axis='y',alpha=.7);ax.set_axisbelow(True)
    figure.suptitle(data['title'],x=.07,ha='left',fontsize=20,fontweight='bold',y=.98)
    figure.text(.07,.93,f'Seed 2101 · 65,536 training transitions · selected update {data["selected_update"]} · 15 held-out episodes per controller',fontsize=10,color=gray)
    figure.text(.07,.065,f'Held-out successful stops — PPO {successes["ppo"]}/15; scripted {successes["scripted"]}/15.  '
                f'Timeouts — PPO {timeouts["ppo"]}/15; scripted {timeouts["scripted"]}/15.',fontsize=10)
    figure.text(.07,.04,f'Physical guard terminations — PPO {failures["ppo"]}/15; scripted {failures["scripted"]}/15.',fontsize=9,color=gray)
    figure.text(.07,.015,'Low tracking error alone is not successful driving. All failed episodes remain included. Reward is not used as an improvement claim.',fontsize=9,color=gray)
    figure.subplots_adjust(left=.07,right=.98,top=.87,bottom=.17,wspace=.25,hspace=.57)
    output.parent.mkdir(parents=True,exist_ok=True)
    try:
        with output.open('xb') as stream:figure.savefig(stream,format='png',dpi=170)
        payload=dict(data,rendering=dict(matplotlib_version=matplotlib.__version__,backend='Agg',png_sha256=digest(output)))
        with sidecar.open('x',encoding='utf-8') as stream:json.dump(payload,stream,indent=2,allow_nan=False)
    finally:
        plt.close(figure)
    return output,sidecar


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    for name in ('training','selection-report','heldout-report'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--matplotlib-path',type=Path)
    parser.add_argument('--check-only',action='store_true')
    args=parser.parse_args(argv)
    try:
        data=load_chart_data(args.training,args.selection_report,args.heldout_report)
        if args.check_only:
            print(json.dumps(dict(verified=True,completed_training_episodes=len(data['training']),heldout_pairs=len(data['pairs']))))
            return 0
        require(args.output is not None,'--output is required unless --check-only')
        output,sidecar=render_chart(data,args.output,matplotlib_path=args.matplotlib_path)
        print(json.dumps(dict(png=str(output),sidecar=str(sidecar),png_sha256=digest(output))))
    except (ValueError,KeyError,TypeError,OSError) as error:
        parser.exit(1,f'Plot inputs rejected: {error}\n')
    return 0


if __name__=='__main__':
    raise SystemExit(main())
