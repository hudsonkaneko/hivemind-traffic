"""Rebuild compact v05 pilot tables and plots from immutable local run artifacts."""
from pathlib import Path
import csv
import gzip
import hashlib
import json
import subprocess
from collections import Counter

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
RUN = ROOT / 'outputs/highway-environment/v05/train-20261106T024828Z-90b25880'
TASKS = ('cruise', 'following', 'passing', 'merge')


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def source(path):
    return dict(path=path.relative_to(ROOT).as_posix(), bytes=path.stat().st_size,
                sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def save(name, value):
    (HERE / name).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def pooled(reports):
    n = sum(r['agent_steps'] for r in reports)
    o = sum(r['overrides'] for r in reports)
    return dict(decisions=n, interventions=o, intervention_fraction=o / n,
        policy_filter_changes=sum(r['policy_filter_changes'] for r in reports),
        native_realization_changes=sum(r['native_realization_changes'] for r in reports),
        passes=sum(r['passes'] for r in reports), stable=sum(r['stable_merge']['stable'] for r in reports),
        merge_targets=sum(r['merge_target_count'] for r in reports),
        lateral_merges=sum(r['successful_merges'] for r in reports),
        **{key:sum(r[key] for r in reports) for key in ('collisions','teleports','overspeed_steps','emergency_braking_observed')},
        native_emergencies=sum(r['native_emergencies']['unique_events'] for r in reports),
        all_native_logs_available=all(r['native_emergencies']['available'] for r in reports),
        all_primary_horizons_complete=all(r['primary_horizon_complete'] and r['accounting_comparison_valid'] and not r['partial_control_horizon'] for r in reports))


def raw_near(reports):
    rows = [r['raw_near_limit'] for r in reports]
    near = sum(r['near_limit_decisions'] for r in rows)
    at = sum(r['at_limit_decisions'] for r in rows)
    positive = sum(r['positive_at_limit_count'] for r in rows)
    excess = sum(r['raw_speed_ceiling_excess_count'] for r in rows)
    return dict(near_limit_decisions=near, at_limit_decisions=at, positive_at_limit_count=positive,
        positive_at_limit_fraction=positive / at, raw_ceiling_excess_count=excess,
        raw_ceiling_excess_fraction=excess / near,
        raw_teacher_rmse_m_s2=(sum(r['raw_teacher_rmse_m_s2'] ** 2 * r['near_limit_decisions'] for r in rows) / near) ** .5,
        scope='Conditional sampled-state diagnostics; initial/final policies visit different states')


def main():
    initial, final = read(RUN/'initial-validation.json'), read(RUN/'final-validation.json')
    cfg, outcome, provenance = read(RUN/'config.json'), read(RUN/'summary.json'), read(RUN/'provenance.json')
    contract, curriculum = read(RUN/'policy-contract.json'), read(RUN/'curriculum.json')
    assert len(initial['reports']) == len(final['reports']) == 24
    rows = []
    for before, after, setting in zip(initial['reports'], final['reports'], cfg['validation_cases']):
        assert before['seed'] == after['seed'] == setting['seed']
        assert before['case_id'] == after['case_id'] == setting['case']['id']
        row = dict(case_id=before['case_id'], seed=before['seed'], task=before['task'],
            **before['population_setting'], speed_limit_m_s=setting['speed_limit_m_s'],
            difficulty=setting['difficulty'], traffic_mode=setting['traffic_mode'])
        for label, value in [('initial', before), ('final', after)]:
            for key in ('agent_steps','overrides','intervention_fraction','policy_filter_changes',
                        'native_realization_changes','mean_fraction_of_speed_limit','passes','successful_merges','merge_target_count'):
                row[label+'_'+key] = value[key]
            row[label+'_stable_merges'] = value['stable_merge']['stable']
            row[label+'_pending_merges'] = value['stable_merge']['pending']
        row['intervention_percentage_point_change'] = 100 * (after['intervention_fraction'] - before['intervention_fraction'])
        rows.append(row)
    with (HERE/'paired-validation.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    results = {}
    for label, evidence in [('initial', initial), ('final', final)]:
        result = pooled(evidence['reports'])
        assert abs(result['intervention_fraction'] - evidence['summary']['pooled_intervention_fraction']) < 1e-12
        result.update(cases=24, failed_cases=len(evidence['summary']['failures']),
            passed_cases=24-len(evidence['summary']['failures']), failures=evidence['summary']['failures'],
            worst_intervention_fraction=evidence['summary']['worst_intervention_fraction'],
            raw_near_limit=raw_near(evidence['reports']),
            by_task={task:pooled([r for r in evidence['reports'] if r['task']==task]) for task in TASKS})
        results[label] = result
    save('comparison-summary.json', dict(schema='highway_environment_v05_pilot_results_v1',
        run=RUN.relative_to(ROOT).as_posix(), accepted=False, held_out_test_run=False,
        comparison='Same24 predeclared validation rows; initial migrated parent versus final endpoint',
        **results, paired_cases=rows))
    recovery = []
    for name in ('recovery-departure-precision','recovery-native-spawn'):
        path=RUN.parent/name/'migration.json'; recovery.append(dict(source=source(path),data=read(path)))
    save('run-provenance.json', dict(run_summary_source=source(RUN/'summary.json'),
        policy=source(RUN/'policy.pt'), checkpoint=source(RUN/'last.pt'), config=source(RUN/'config.json'),
        config_semantic_sha256=provenance['config_sha256'], git_head=provenance['git_head'],
        worktree_dirty=bool(provenance['git_status'].strip()),
        note='Unrelated worktree paths omitted here; exact runtime files/hashes and scoped diff remain in raw run',
        runtime={k:provenance[k] for k in ('python','platform','torch','traci','sumo')},
        source_hashes=provenance['source_hashes'], sources_changed=outcome['sources_changed'],
        command=provenance['command'], resume=provenance['resume'], counters=outcome['counters'],
        stop_reason=outcome['stop_reason'], current_process_wall_seconds=outcome['wall_seconds'],
        recoveries=recovery, configuration={k:cfg[k] for k in ('training','optimizer','environment','population','contract','readiness')},
        parent=contract['parent']))
    updates=[read(p) for p in sorted(RUN.glob('update-*.json'))]
    coverage={task:dict(episodes=sum(r['report']['task']==task for r in updates),
        decisions=sum(r['report']['agent_transitions'] for r in updates if r['report']['task']==task)) for task in TASKS}
    report_keys=('case_id','seed','task','difficulty','traffic_mode','agent_transitions','partial_control_horizon',
        'primary_horizon_complete','accounting_comparison_valid','intervention_fraction','mean_fraction_of_speed_limit')
    save('curriculum-coverage.json',dict(source=source(RUN/'curriculum.json'),
        final_grade=curriculum['grade'], grade_history=curriculum['grade_history'],
        rollouts_issued=curriculum['rollouts_issued'], ordinary_draws=curriculum['ordinary_index'],
        retry_draws=curriculum['retry_rollouts'], complete_reports_observed=len(curriculum['observed_rollout_ids']),
        complete_report_agent_decisions=curriculum['actual_agent_decisions'], by_task=coverage,
        recent_passes={task:dict(reports=sum(r['task']==task for r in curriculum['recent']),
            passed=sum(r['task']==task and r['passed'] for r in curriculum['recent'])) for task in TASKS},
        updates=[dict(index=i+1,critic_only=r['critic_only'],
            optimizer_steps=r['optimization']['optimizer_steps'],
            report={k:r['report'].get(k) for k in report_keys},population=r['report']['population_setting']) for i,r in enumerate(updates)]))
    warning_audit = {}
    for label, evidence in [('initial', initial), ('final', final)]:
        logs=[]
        for report in evidence['reports']:
            path=ROOT/Path(report['episode_directory'])/'sumo-errors.log'
            lines=[line for line in path.read_text(errors='replace').splitlines() if line.strip()]
            warnings=[line for line in lines if 'warning' in line.lower()]
            logs.append(dict(case_id=report['case_id'],seed=report['seed'],source=source(path),
                warning_lines=warnings,other_nonempty_lines=[line for line in lines if line not in warnings]))
        warning_audit[label]=dict(logs=logs,total_warning_lines=sum(len(r['warning_lines']) for r in logs),
            other_nonempty_lines=sum(len(r['other_nonempty_lines']) for r in logs))
    save('native-warning-audit.json',warning_audit)
    entry = {}
    for label, evidence in [('initial', initial), ('final', final)]:
        report=next(r for r in evidence['reports'] if r['seed']==6900019)
        path=ROOT/Path(report['episode_directory'])/'actions.jsonl.gz'
        with gzip.open(path,'rt') as stream:
            journal=[json.loads(line) for line in stream]
        samples=[(r['time'],r['agents']['agent_0']) for r in journal if 'agent_0' in r['agents']]
        tail=[info for time,info in samples if time>5.]
        entry[label]=dict(source=source(path),merge_target=report['stable_merge']['per_agent']['agent_0'],
            total_decisions=len(samples),stay_requests=sum(r['lane_request']==0 for _,r in samples),
            positive_raw_requests=sum(r['requested_acceleration']>0 for _,r in samples),
            after_5_seconds=dict(samples=len(tail),zero_realized_acceleration=sum(r['realized_acceleration']==0 for r in tail),
                zero_progress_reward_samples=sum(r['reward_terms']['progress']==0 for r in tail),
                positive_raw_requests=sum(r['requested_acceleration']>0 for r in tail),
                positive_teacher_targets=sum(r['feasible_acceleration_target']>0 for r in tail),
                zero_teacher_targets=sum(r['feasible_acceleration_target']==0 for r in tail),
                mean_raw_request_m_s2=sum(r['requested_acceleration'] for r in tail)/len(tail),
                mean_teacher_target_m_s2=sum(r['feasible_acceleration_target'] for r in tail)/len(tail)),
            limits='Lane masks are not logged; native hold cause is unattributed; no validation-data fitting authorized')
    save('i110-entry-audit.json',entry)
    old_files=subprocess.check_output(['git','ls-tree','-r','--name-only',provenance['git_head'],'--','policies/versions'],cwd=ROOT,text=True).splitlines()
    preserved=[]
    for name in old_files:
        if name.endswith('.pt'):
            before=subprocess.check_output(['git','show',provenance['git_head']+':'+name],cwd=ROOT)
            preserved.append(dict(source=source(ROOT/name),unchanged_from_run_git_head=before==(ROOT/name).read_bytes()))
    save('previous-policy-preservation.json',dict(baseline_git_head=provenance['git_head'],policies=preserved))
    save('source-index.json',dict(run=RUN.relative_to(ROOT).as_posix(),sources=[source(RUN/name) for name in
        ('summary.json','initial-validation.json','final-validation.json','comparison.json','curriculum.json',
         'training-coverage.json','provenance.json','policy-contract.json','config.json')],
        artifact_policy='Compact numerical extracts and actual-data plots only; no raw trajectories or weights',
        plot_builder=source(Path(__file__))))
    plots(rows,results,coverage)


def plots(rows, results, coverage):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    initial_color, final_color = '#7c8a96', '#c04a2d'
    fig, ax = plt.subplots(figsize=(11.5,10.5), constrained_layout=True)
    y=np.arange(len(rows)); a=np.array([r['initial_intervention_fraction']*100 for r in rows]);b=np.array([r['final_intervention_fraction']*100 for r in rows])
    ax.hlines(y,np.minimum(a,b),np.maximum(a,b),color='#c7cdd2',lw=1.4)
    ax.scatter(a,y,color=initial_color,s=33,label='Initial migrated parent',zorder=3)
    ax.scatter(b,y,color=final_color,s=33,label='Final v05 endpoint',zorder=3)
    labels=[f"{r['seed']}  {r['task'].title()}  {r['controlled_count']}/{r['total_vehicles']} AV" for r in rows]
    ax.set_yticks(y,labels);ax.invert_yaxis();ax.set_xlim(0,80)
    ax.axvline(5,color='#2d6151',ls='--',lw=1.4,label='5% per-case gate')
    ax.set_xlabel('Intervened active-agent decisions (%)')
    ax.set_title('Matched validation: pooled interventions worsened\n13.3749% initial → 19.8287% final; all 24 fixed cases retained',loc='left',fontsize=15,pad=14)
    ax.grid(axis='x',alpha=.2);ax.legend(loc='upper right',frameon=False)
    fig.savefig(HERE/'paired-interventions.png',dpi=180);plt.close(fig)
    fig, axes=plt.subplots(2,2,figsize=(11.5,8.3),constrained_layout=True)
    ax=axes[0,0];x=np.arange(4);width=.35
    ax.bar(x-width/2,[100*results['initial']['by_task'][t]['intervention_fraction'] for t in TASKS],width,color=initial_color,label='Initial')
    ax.bar(x+width/2,[100*results['final']['by_task'][t]['intervention_fraction'] for t in TASKS],width,color=final_color,label='Final')
    ax.set_xticks(x,[t.title() for t in TASKS]);ax.set_ylim(0,30);ax.set_ylabel('Interventions (%)');ax.set_title('Every task regressed on pooled interventions');ax.axhline(5,color='#2d6151',ls='--',lw=1);ax.legend(frameon=False)
    ax=axes[0,1];x=np.arange(2)
    ax.bar(x-.18,[results['initial']['stable'],results['initial']['passed_cases']],.36,color=initial_color)
    ax.bar(x+.18,[results['final']['stable'],results['final']['passed_cases']],.36,color=final_color)
    ax.set_xticks(x,['Stable merges\n(out of 9)','Cases passing every gate\n(out of 24)']);ax.set_ylim(0,10);ax.set_ylabel('Count');ax.set_title('Task gains did not satisfy overall acceptance')
    for i,(before,after) in enumerate([(2,8),(1,5)]):
        ax.text(i-.18,before+.15,str(before),ha='center');ax.text(i+.18,after+.15,str(after),ha='center')
    ax=axes[1,0];ax.bar(TASKS,[coverage[t]['decisions'] for t in TASKS],color='#34697b')
    ax.set_ylabel('Collected active-agent decisions');ax.ticklabel_format(axis='y',style='plain');ax.set_title('All collected scenarios remained at difficulty 0')
    for i,t in enumerate(TASKS):ax.text(i,coverage[t]['decisions']+1800,f"{coverage[t]['decisions']:,}",ha='center',fontsize=9)
    ax.set_ylim(0,125000)
    ax=axes[1,1];x=np.arange(2)
    ax.bar(x-.18,[100*results['initial']['raw_near_limit']['positive_at_limit_fraction'],100*results['initial']['raw_near_limit']['raw_ceiling_excess_fraction']],.36,color=initial_color)
    ax.bar(x+.18,[100*results['final']['raw_near_limit']['positive_at_limit_fraction'],100*results['final']['raw_near_limit']['raw_ceiling_excess_fraction']],.36,color=final_color)
    ax.set_xticks(x,['Positive requests\nat the limit','Raw ceiling excess\nnear the limit']);ax.set_ylabel('Conditional sampled decisions (%)');ax.set_title('Raw requests improved while native differences grew');ax.set_ylim(0,42)
    fig.suptitle('v05 endpoint: mixed outcomes, not accepted driving',fontsize=16)
    fig.savefig(HERE/'outcomes-and-coverage.png',dpi=180);plt.close(fig)


if __name__=='__main__':
    main()
