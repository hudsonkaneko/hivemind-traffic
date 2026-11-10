"""Read-only policy audit of frozen traces/checkpoints. No simulator or optimizer."""
from pathlib import Path
from collections import Counter, defaultdict
import csv
import gzip
import hashlib
import json
import math
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
import torch
from experiments import run_highway_flow_rl_v03 as runner
from policies.highway_flow import FlowModel


def distribution(values):
    a=np.asarray([v for v in values if v is not None and math.isfinite(v)],dtype=float)
    return dict(n=len(a),mean=float(a.mean()),median=float(np.median(a)),
        p05=float(np.quantile(a,.05)),p95=float(np.quantile(a,.95)),minimum=float(a.min()),maximum=float(a.max())) if len(a) else dict(n=0)


def contiguous(rows, predicate):
    result=[];start=None;last=None;count=0
    for row in rows:
        valid=predicate(row)
        if valid and start is None:start=row['time_s'];count=0
        if valid:last=row['time_s'];count+=1
        if not valid and start is not None:
            result.append(dict(start_s=start,end_s=last,duration_s=round(count*.1,6),samples=count));start=None
    if start is not None:result.append(dict(start_s=start,end_s=last,duration_s=round(count*.1,6),samples=count))
    return result


def summarize(rows):
    reward=Counter()
    for r in rows:reward.update(r['reward_terms'])
    return dict(decisions=len(rows),overrides=sum(r['override'] for r in rows),
        causes=dict(Counter(c for r in rows if r['override'] for c in r['causes'])),
        stage_changes={k:sum(r[k] for r in rows) for k in ('policy_filter_changed','native_realization_changed','lane_rejected')},
        target_eligible=sum(r['target_eligible'] for r in rows),
        exclusions=dict(Counter(c for r in rows for c in r['target_reasons'])),
        left_allowed=sum(r['left_allowed'] for r in rows),left_requested=sum(r['lane_request']==1 for r in rows),
        eligible_left_but_stayed=sum(r['left_allowed'] and r['lane_request']==0 for r in rows),
        reward_terms=dict(reward),stopped_windows=contiguous(rows,lambda r:r['speed_m_s']<.1),
        lane_end_override_windows=contiguous(rows,lambda r:'lane_end' in r['causes'] and r['override']),
        left_mask_blocks=dict(Counter(c for r in rows if not r['left_allowed'] for c in r['left_block_reasons'])),
        messages=distribution([r['message_count'] for r in rows]),
        peak_jerks=sorted(rows,key=lambda r:abs(r['jerk_m_s3'] or 0),reverse=True)[:5])


def read_case(path, actor, model):
    per_car=defaultdict(list);observations=[];info_rows=[]
    with gzip.open(path,'rt') as stream:
        for line in stream:
            step=json.loads(line)
            for agent,info in step['agents'].items():
                pre=info['v03_reward_audit']['pre_action'];s=pre['ego_state'];o=pre['observation117'];ctx=pre['context']
                target=pre['target'];blocks=[]
                if s['lane_id'].startswith(':'):blocks.append('internal_segment')
                if o[29]>1e-6:blocks.append('cooldown')
                if o[48]>.5:blocks.append('committed_maneuver')
                if not o[24]>.5:blocks.append('no_adjacent_left_lane')
                for label,start in [('front',12),('rear',15)]:
                    if o[start+2]>.5:
                        neighbor=max(0.,s['speed']+o[start+1]*26.8224)
                        follower,leader=(s['speed'],neighbor) if label=='front' else (neighbor,s['speed'])
                        required=2.5+max(1.5*follower,.1*follower+follower*follower/9.-leader*leader/18.)
                        if o[start]*120.<required-1e-4:blocks.append('current_'+label+'_gap')
                if not pre['allowed_actions'][2]>.5 and not blocks:blocks.append('other_map_or_runway_constraint')
                row=dict(agent=agent,time_s=pre['time_s'],result_time_s=step['time'],lane=s['lane_id'],
                    speed_m_s=s['speed'],legal_speed_m_s=min(26.8224,s['allowed_speed']),
                    requested=info['requested_acceleration'],filtered=info['filtered_acceleration'],realized=info['realized_acceleration'],
                    jerk_m_s3=info['realized_jerk_m_s3'],override=info['override'],causes=info['override_causes'],
                    policy_filter_changed=info['policy_filter_changed'],native_realization_changed=info['native_realization_changed'],
                    lane_rejected=info['lane_rejected'],lane_request=info['lane_request'],left_allowed=pre['allowed_actions'][2]>.5,
                    left_block_reasons=blocks,target_eligible=target['eligible'],target_m_s=target['target_m_s'],target_reasons=target['reasons'],
                    remaining_lane_m=ctx['corridors']['own']['route']['distance_m'],route_reachable=ctx['corridors']['own']['route']['reachable'],
                    lane_end_bound=info['acceleration_bounds']['lane_end'],
                    message_count=sum(o[i] for i in range(76,117,8)),
                    reward_terms=info['reward_terms'],native_refused=bool(info['maneuver']['native_refusal']),
                    maneuver_active=bool(info['maneuver']['in_progress']),
                    target_front=ctx['entry'].get('front'),target_rear=ctx['entry'].get('rear'))
                observations.append(o);info_rows.append(row);per_car[agent].append(row)
    x=torch.tensor(observations,dtype=torch.float32)
    with torch.no_grad():
        exported=actor(x);full=model.act(x,deterministic=True)[0]
        silent=x.clone();silent[:,69:]=0.;silent_actions=actor(silent)
        raw=model.actor.mean_net(x);probs=model.actor.lane_logits(x,raw).softmax(-1)
    requested=np.array([r['requested'] for r in info_rows]);decoded=(-1.25+3.25*exported[:,0]).numpy()
    error=float(np.max(np.abs(requested-decoded)))
    if error>2e-6 or not torch.equal(exported,full):raise ValueError('Recorded/exported actor mismatch')
    if any(int(a)!=r['lane_request'] for a,r in zip(exported[:,1],info_rows)):raise ValueError('Lane replay mismatch')
    for row,a,p,silent_a in zip(info_rows,exported.numpy(),probs.numpy(),silent_actions.numpy()):
        row.update(left_probability=float(p[2]),zero_message_acceleration_delta=float(3.25*(silent_a[0]-a[0])),
            zero_message_lane_changed=bool(silent_a[1]!=a[1]))
    evidence=dict(decisions=len(info_rows),overrides=sum(r['override'] for r in info_rows),
        actor_replay_max_abs_acceleration_error=error,export_matches_full_model=True,
        observation_nonfinite=int((~torch.isfinite(x)).sum()),max_abs_observation=float(x.abs().max()),
        constant_observation_columns=[i for i in range(117) if float(x[:,i].max()-x[:,i].min())<1e-8],
        per_car={a:summarize(rows) for a,rows in per_car.items()},
        message_ablation=dict(scope='Same saved states with messages zeroed; sensitivity only, not a closed-loop communication-benefit test',
            acceleration_delta=distribution([r['zero_message_acceleration_delta'] for r in info_rows]),
            changed_lane_decisions=sum(r['zero_message_lane_changed'] for r in info_rows)))
    return evidence,per_car


def training_audit(root):
    cases={};updates=[];gradients=[];epoch_counts=Counter();paths=[]
    for folder in ('long-round-02','long-round-02-resumed'):
        for path in sorted((root/folder/'updates').glob('*.json')):
            paths.append(path);u=json.loads(path.read_text());r=u['report'];opt=u['optimization'];d=opt['gradient_diagnostics']
            row=cases.setdefault(r['case_id'],dict(episodes=0,decisions=0,merge_agent_decisions=0,target_eligible=0,
                merge_agent_target_eligible=0,partial_rollouts=0,merge_targets=0,stable_merges=0,reward_terms=Counter()))
            row['episodes']+=1;row['decisions']+=r['agent_transitions'];row['target_eligible']+=r['v03_reward_summary']['target_eligible']
            row['partial_rollouts']+=int(r['partial_control_horizon']);row['merge_targets']+=r['stable_merge']['targets'];row['stable_merges']+=r['stable_merge']['stable']
            row['reward_terms'].update(r['reward_terms'])
            for agent in r['merge_target_ids']:
                a=r['v03_reward_summary']['per_agent'].get(agent,{})
                row['merge_agent_decisions']+=a.get('decisions',0);row['merge_agent_target_eligible']+=a.get('target_eligible',0)
            rep=d.get('representative_minibatch') or {};parts=rep.get('components',{})
            updates.append(dict(case_id=r['case_id'],samples=r['agent_transitions'],completed_epochs=opt['completed_epochs'],
                early_stopped=opt['early_stopped'],last_kl=opt['last_kl'],actor_loss=opt.get('actor_loss'),value_loss=opt.get('value_loss'),
                critic_explained_variance_before=d['critic_before']['explained_variance'],critic_explained_variance_after=d['critic_after']['explained_variance'],
                ppo_actor_gradient=parts.get('ppo',{}).get('actor'),value_critic_gradient=parts.get('value',{}).get('critic')))
            gradients.extend(d['actual_steps']);epoch_counts[opt['completed_epochs']]+=1
    actual=sum(r['decisions'] for r in cases.values());merge=sum(r['merge_agent_decisions'] for r in cases.values())
    return dict(actual_samples=actual,merge_agent_samples=merge,merge_agent_fraction=merge/actual,
        by_case=cases,updates=updates,completed_epochs_distribution=dict(epoch_counts),
        kl_early_stopped_updates=sum(r['early_stopped'] for r in updates),
        global_clip_factors=distribution([r['global_clip_factor'] for r in gradients]),
        optimizer_steps=len(gradients),steps_critic_norm_exceeds_actor=sum(r['before_clip']['critic']>r['before_clip']['actor'] for r in gradients),
        actor_gradient_before_clip=distribution([r['before_clip']['actor'] for r in gradients]),
        critic_gradient_before_clip=distribution([r['before_clip']['critic'] for r in gradients]),
        critic_explained_variance_after=distribution([r['critic_explained_variance_after'] for r in updates])),paths


def run():
    started=time.monotonic();torch.set_num_threads(1)
    root=ROOT/'outputs/highway-flow-rl/v03';out=root/'policy-audit-01'
    if out.exists():raise ValueError('Use a fresh audit directory')
    cfg_path=ROOT/'configs/training/highway-flow-rl-v03-long.json';cfg=json.loads(cfg_path.read_text())
    with runner.LedgerSession(cfg,'offline_policy_audit',out,runner.file_digest(cfg_path),started=started,episodes=0) as session:
        out.mkdir(parents=True)
        final=root/'long-round-02-resumed';manifest=json.loads((final/'manifest.json').read_text())
        if manifest['source_hashes']!=runner.hashes():raise ValueError('Runtime sources differ from trained snapshot')
        saved=torch.load(final/'last.pt',map_location='cpu',weights_only=False)
        model=FlowModel();model.load_state_dict(saved['model']);model.eval()
        actor=torch.jit.load(str(final/'policy.pt'),map_location='cpu');actor.eval()
        cases=[];inputs=[cfg_path,final/'last.pt',final/'policy.pt',final/'manifest.json'];focal=None
        for index in range(6):
            runner.check_deadline(session.deadline)
            report_path=final/'final-validation'/f'{index:02d}.json';report=json.loads(report_path.read_text())
            path=next((final/'final-validation'/f'case-{index:02d}').rglob('actions.jsonl.gz'));inputs += [path,report_path]
            result,rows=read_case(path,actor,model)
            result.update(case_id=report['case_id'],seed=report['seed'],task=report['task'])
            cases.append(result)
            with (out/f'case-{index:02d}.csv').open('w',newline='',encoding='utf-8') as stream:
                flattened=[dict(r,**{k:json.dumps(r[k],sort_keys=True) for k in ('causes','left_block_reasons','target_reasons','reward_terms','target_front','target_rear')}) for rr in rows.values() for r in rr]
                writer=csv.DictWriter(stream,fieldnames=list(flattened[0]));writer.writeheader();writer.writerows(flattened)
            if index==0:focal=rows['agent_0']
        training,update_paths=training_audit(root);inputs+=update_paths
        f=summarize(focal);first_allowed=next(r['time_s'] for r in focal if r['left_allowed'])
        premerge=[r for r in focal if not r['route_reachable']]
        stages={name:summarize([r for r in focal if lo<=r['time_s']<hi]) for name,lo,hi in
            [('ramp',0,3.5),('gap_formation',3.5,22.5),('forced_braking',22.5,26.5),('stopped_merge',26.5,40),('after_merge',40,60)]}
        result=dict(schema='highway_policy_full_audit_v01',training=False,new_simulator_runs=0,optimizer_steps=0,
            policy='outputs/highway-flow-rl/v03/long-round-02-resumed/policy.pt',cases=cases,
            focal=dict(summary=f,stages=stages,first_left_allowed_s=first_allowed,
                before_first_allowed=summarize([r for r in focal if r['time_s']<first_allowed]),
                unresolved_merge_decisions=len(premerge),unresolved_merge_target_eligible=sum(r['target_eligible'] for r in premerge)),
            training_audit=training,algorithm=dict(config=cfg,gaussian_log_std=float(model.log_std),
                gaussian_latent_std=float(model.log_std.clamp(-5,2).exp()),gamma=.99,gae_lambda=.95,
                reward_discount_at_seconds={str(t):.99**(t/.1) for t in (1,5,10,20,30)},
                direct_gae_trace_weight_at_seconds={str(t):(.99*.95)**(t/.1) for t in (1,5,10,20,30)}),
            source_hashes=runner.hashes(),audit_script_sha256=runner.file_digest(Path(__file__)),
            inputs={p.relative_to(ROOT).as_posix():runner.file_digest(p) for p in inputs},
            prior_study_seconds=session.totals['total'],analysis_seconds=session.elapsed())
        runner.save(out/'audit.json',result)
        runner.save(out/'focal-timeline.json',focal)
        print(json.dumps(dict(cases=len(cases),focal={k:f[k] for k in ('decisions','overrides','target_eligible','left_allowed','left_requested','stopped_windows','left_mask_blocks')},
            training={k:training[k] for k in ('actual_samples','merge_agent_samples','merge_agent_fraction','global_clip_factors','critic_explained_variance_after')},
            gaussian_std=result['algorithm']['gaussian_latent_std'],analysis_seconds=session.elapsed()),indent=2))


if __name__=='__main__':run()
