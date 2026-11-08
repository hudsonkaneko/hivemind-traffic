"""Bounded teacher-free MAPPO development rounds; terminal review, no live tuning."""
from __future__ import annotations
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import random
import sys
import time
import traceback

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import numpy as np
import torch
from experiments import run_highway_flow as flow
from environments.highway_flow_rl_env import HighwayFlowRLEnv
from policies.mappo import MappoTrainer, compute_gae, export_actor
from policies.highway_flow import critic_update
from traffic.highway_flow_rl_contract import validate_config, validate_preflight, file_digest
from experiments.run_highway_stage1 import PilotBudgetExhausted

AGENTS=flow.AGENTS
STUDY=ROOT/'outputs/highway-flow-rl/v01'
NEW_SOURCES=('experiments/run_highway_flow_rl.py','environments/highway_flow_rl_env.py',
             'traffic/highway_flow_rl_contract.py')


def hashes():
    return {p:file_digest(ROOT/p) for p in (*flow.SOURCES,*NEW_SOURCES)}


def save(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    flow.old.atomic_json(path,value)


def build_model(cfg, checkpoint=None, exact_resume=False):
    model=flow.initialize_candidate(flow.load_config(),cfg['initializer_id'])['model']
    trainer=MappoTrainer(model,**cfg['optimizer'])
    saved=None
    if checkpoint:
        saved=torch.load(checkpoint,map_location='cpu',weights_only=False)
        if saved['schema']!='highway_flow_rl_checkpoint_v01':
            raise ValueError('Wrong checkpoint schema')
        model.load_state_dict(saved['model'],strict=True)
        if exact_resume:
            trainer.optimizer.load_state_dict(saved['optimizer'])
            flow.restore_rng(saved['rng'])
    return model,trainer,saved


def env_for(cfg,row,directory):
    case=row['case']; path=ROOT/case['network']
    if file_digest(path)!=case['network_sha256']:
        raise ValueError('Network changed')
    env=HighwayFlowRLEnv(path,directory,controlled_count=row['controlled_count'],
        total_vehicles=row['total_vehicles'],horizon=800,communication=row['communication'],
        packet_loss=row['packet_loss'],through_lanes=case['through_lanes'],task=case['task'],
        density=case['density'],control_target_version='v3',speed_limit_m_s=row['speed_limit_m_s'],
        difficulty=row['difficulty'],traffic_mode='initial',lane_change_duration=3.,
        stabilization_seconds=8.,jerk_reward_weight=cfg['reward_weights']['jerk'])
    env.case_id=case['id']
    return env


def episode(cfg,row,directory,model,*,collect=False,limit=None,deadline=None):
    env=env_for(cfg,row,directory); samples=[]; decisions=0; report=None
    try:
        obs,_=env.reset(seed=row['seed'])
        while env.agents:
            flow.old.check_deadline(deadline)
            if limit is not None and decisions+len(env.agents)>limit: break
            active=torch.tensor([a in env.agents for a in AGENTS]); indices=torch.arange(24)
            local=torch.tensor(np.stack([obs.get(a,np.zeros(117,np.float32)) for a in AGENTS]))
            state=torch.as_tensor(env.state()).repeat(24,1)
            with torch.no_grad():
                actions,logp,_,latent=model.act(local,deterministic=not collect)
                values=model.values(state,indices)
            chosen={a:actions[i].numpy() for i,a in enumerate(AGENTS) if a in env.agents}
            nxt,rewards,term,trunc,infos=env.step(chosen)
            if not all(np.isfinite(v) for v in rewards.values()): raise ValueError('Nonfinite reward')
            if any(info.get('teacher_reference_used') is not False for info in infos.values()):
                raise ValueError('Teacher reward entered RL-only episode')
            decisions+=len(rewards)
            if collect:
                final=np.concatenate([np.r_[nxt.get(a,np.zeros(117)),float(not term.get(a,True))]
                                      for a in AGENTS]).astype(np.float32)
                with torch.no_grad():
                    next_values=model.values(torch.tensor(final).repeat(24,1),indices)
                samples.append(dict(local_obs=local,global_states=state,agent_indices=indices,
                    actions=actions,pre_tanh_actions=latent,old_log_probs=logp,old_values=values,
                    next_values=next_values,rewards=torch.tensor([rewards.get(a,0.) for a in AGENTS]),
                    terminated=torch.tensor([term.get(a,False) for a in AGENTS]),
                    truncated=torch.tensor([trunc.get(a,False) for a in AGENTS]),active=active))
            obs={a:nxt[a] for a in env.agents}
        if samples and env.agents:
            samples[-1]['truncated'] |= samples[-1]['active'] & ~samples[-1]['terminated']
        report=flow.old.primary_accounting(env,partial=bool(env.agents),deadline=deadline)
        report.update(case_id=env.case_id,seed=row['seed'],agent_transitions=decisions,
            partial_control_horizon=bool(env.agents),teacher_reference_used=False,
            actual_merge_events=report.get('stable_merge',{}),population_setting=row,
            episode_directory=str(env.directory.relative_to(ROOT)))
        report['native_control_modes']={a:dict(speed=env.conn.vehicle.getSpeedMode(a),
            lane_change=env.conn.vehicle.getLaneChangeMode(a)) for a in env.agents if a in env.states}
        batch=None
        if samples:
            batch={k:torch.stack([s[k] for s in samples]) for k in samples[0]}
            batch['advantages'],batch['returns']=compute_gae(batch['rewards'],batch['old_values'],
                batch['next_values'],batch['terminated'],batch['truncated'],batch['active'])
        return report,batch
    except BaseException:
        if hasattr(env,'directory'):
            save(env.directory/'interrupted-accounting.json',dict(actual_agent_transitions=decisions,
                incomplete_episode=True,traceback=traceback.format_exc()))
        raise
    finally:
        env.close()
        if report is not None:
            from traffic.sumo_warnings import native_emergency_report
            report['native_emergencies']=native_emergency_report(env.directory/'sumo-errors.log')


def summary(reports):
    total=sum(r['agent_steps'] for r in reports)
    return dict(cases=len(reports),decisions=total,
        pooled_intervention_fraction=sum(r['overrides'] for r in reports)/max(1,total),
        max_peak_jerk_m_s3=max((r['peak_abs_jerk'] for r in reports),default=0.),
        collisions=sum(r['collisions'] for r in reports),teleports=sum(r['teleports'] for r in reports),
        stable_merges=sum(r.get('stable_merge',{}).get('stable',0) for r in reports),
        merge_targets=sum(r.get('stable_merge',{}).get('targets',0) for r in reports),
        merge_entries=sum(r.get('stable_merge',{}).get('entered',0) for r in reports),
        lateral_merges=sum(r.get('stable_merge',{}).get('lateral_completed',0) for r in reports),
        full_horizon_cases=sum(r.get('primary_horizon_complete',False) for r in reports),
        quality_accepted=False,held_out=False)


def training_row(cfg,state,critic):
    # Critic warmup must not consume the PPO curriculum's first merge case.
    cursor=state.get('critic_episodes',0) if critic else state.get('ppo_episodes',0)
    row=deepcopy(cfg['training_rows'][cursor%len(cfg['training_rows'])])
    row['seed']+=1000*(cursor//len(cfg['training_rows']))+10000*(cfg['round_index']-1)+(50000 if critic else 0)
    return row


def evaluate(cfg,run,label,model,deadline):
    reports=[]
    for i,row in enumerate(cfg['validation_rows']):
        report,_=episode(cfg,row,run/label/f'case-{i}',model,deadline=deadline)
        save(run/label/f'{i:02d}.json',report); reports.append(report)
    result=dict(summary=summary(reports),reports=reports)
    save(run/f'{label}.json',result)
    return result


def run(args):
    if Path.cwd().resolve()!=ROOT: raise ValueError('Use the canonical workspace')
    cfg=json.loads(args.config.read_text());validate_config(cfg)
    run=args.output.resolve()
    if not run.is_relative_to(ROOT/'outputs') or run.exists(): raise ValueError('Fresh output under outputs required')
    run.mkdir(parents=True);STUDY.mkdir(parents=True,exist_ok=True)
    fingerprint=file_digest(args.config); source=hashes(); started=time.monotonic()
    if args.command=='train': validate_preflight(json.loads(args.preflight.read_text()),fingerprint,source)
    if args.checkpoint and not args.resume:
        spec=cfg.get('warm_start')
        if not spec or args.checkpoint.resolve()!= (ROOT/spec['path']).resolve() or file_digest(args.checkpoint)!=spec['sha256']:
            raise ValueError('Revision warm-start requires a pinned checkpoint in its config')
    torch.set_num_threads(1);torch.use_deterministic_algorithms(True)
    torch.manual_seed(cfg['seed']);np.random.seed(cfg['seed']);random.seed(cfg['seed'])
    lock=STUDY/'active.lock'
    ledger_path=STUDY/'cost-ledger.json'
    ledger=json.loads(ledger_path.read_text()) if ledger_path.exists() else dict(
        inherited_seconds=cfg['budget']['inherited_spent_seconds'],segments=[])
    if any(s['status']=='running' for s in ledger['segments']):
        raise ValueError('Unclosed prior process needs cost reconciliation')
    previous=ledger['inherited_seconds']+sum(s['charged_seconds'] for s in ledger['segments'])
    segment=dict(command=args.command,round_index=cfg['round_index'],output=str(run.relative_to(ROOT)),
        status='running',charged_seconds=0.,decisions=0,config_sha256=fingerprint)
    ledger['segments'].append(segment)
    save(run/'resolved-config.json',cfg)
    save(run/'manifest.json',dict(config_sha256=fingerprint,source_hashes=source,
        git_commit=flow.subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        inherited_spent_seconds=previous,python=sys.version,torch=torch.__version__,numpy=np.__version__,
        teacher_imitation=False,warm_start=str(args.checkpoint) if args.checkpoint else None))
    for name in source:
        target=run/'source'/name;target.parent.mkdir(parents=True,exist_ok=True)
        target.write_bytes((ROOT/name).read_bytes())
    state=dict(episodes=0,ppo_episodes=0,critic_episodes=0,decisions=0,critic_decisions=0,updates=0,ppo_optimizer_steps=0,
        critic_optimizer_steps=0,pending=None,coverage={},phase='setup')
    model=trainer=None
    def elapsed(): return time.monotonic()-started
    def checkpoint():
        if model is not None:
            flow.old.save_checkpoint(run/'last.pt',dict(schema='highway_flow_rl_checkpoint_v01',
                model=model.state_dict(),optimizer=trainer.optimizer.state_dict(),rng=flow.capture_rng(),
                config_sha256=fingerprint,source_hashes=source,state=deepcopy(state),
                charged_seconds=elapsed(),saved_at_unix=time.time()))
        segment.update(charged_seconds=elapsed(),decisions=state['decisions'])
        save(ledger_path,ledger)
    with lock.open('x') as f: f.write(str(run))
    try:
        save(ledger_path,ledger)
        model,trainer,saved=build_model(cfg,args.checkpoint,args.resume)
        if args.resume:
            if saved['config_sha256']!=fingerprint or saved['source_hashes']!=source:
                raise ValueError('Exact resume requires matching config and source')
            state.update(saved['state']);state['decisions']=0
            if state.get('pending_update') is not None:
                state['episodes']+=1
                state['critic_episodes' if state['pending_update']['critic'] else 'ppo_episodes']+=1
                state['pending_update']=None
        deadline=started+max(0.,cfg['budget']['total_wall_seconds']-previous)
        if args.command=='preflight':
            report,batch=episode(cfg,cfg['training_rows'][0],run/'smoke',model,collect=True,limit=64,
                                 deadline=min(deadline,time.monotonic()+90))
            before=deepcopy(model.state_dict()); optimizer_before=deepcopy(trainer.optimizer.state_dict())
            diagnostics=trainer.diagnose_batch(batch,phase='zero_update_preflight')
            assert all(torch.equal(v,before[k]) for k,v in model.state_dict().items())
            assert flow.tree_equal(optimizer_before,trainer.optimizer.state_dict())
            if report['collisions'] or report['teleports']: raise ValueError('Native safety smoke failed')
            if not report['native_control_modes'] or any(v!={'speed':31,'lane_change':512}
                for v in report['native_control_modes'].values()): raise ValueError('Native guard modes changed')
            proof=dict(schema='highway_flow_rl_preflight_v01',passed=True,training=False,optimizer_steps=0,
                config_sha256=fingerprint,source_hashes=source,
                checks=dict(api=True,shapes=True,native_guards=True,finite_gradient=True),
                report=report,gradient_diagnostics=diagnostics,scope='Experimental RL runtime checks; not teacher/performance qualification')
            save(run/'preflight.json',proof)
            segment.update(status='complete',charged_seconds=elapsed());save(ledger_path,ledger)
            print(json.dumps(dict(completed=True,preflight=str(run/'preflight.json'),wall_seconds=elapsed())))
            return 0
        prior_round_seconds=sum(s['charged_seconds'] for s in ledger['segments'][:-1]
                                if s['command']=='train' and s['round_index']==cfg['round_index'])
        learning_deadline=min(deadline-cfg['budget']['final_reserve_seconds'],
            time.monotonic()+max(0.,cfg['budget']['per_round_wall_seconds']-prior_round_seconds))
        previous_round=sum(s.get('decisions',0) for s in ledger['segments'][:-1]
                           if s['command']=='train' and s['round_index']==cfg['round_index'])
        total_prior=sum(s.get('decisions',0) for s in ledger['segments'][:-1] if s['command']=='train')
        remaining=min(cfg['budget']['per_round_decisions']-previous_round,
                      cfg['budget']['total_decisions']-total_prior)
        if remaining<=0 or learning_deadline<=time.monotonic(): raise ValueError('No training budget remains')
        checkpoint()
        state['phase']='initial_evaluation'
        initial=evaluate(cfg,run,'initial-validation',model,learning_deadline)
        state['phase']='learning';checkpoint()
        while state['decisions']<remaining and time.monotonic()<learning_deadline-60:
            index=state['episodes']
            critic=state['critic_decisions']<cfg['critic_warmup_decisions'] and (args.checkpoint is None or args.resume)
            row=training_row(cfg,state,critic)
            reserve=min(remaining-state['decisions'],800*row['controlled_count'])
            if critic: reserve=min(reserve,cfg['critic_warmup_decisions']-state['critic_decisions'])
            if reserve<row['controlled_count']: break
            state['pending']=dict(index=index,reserved=reserve);checkpoint()
            try:
                report,batch=episode(cfg,row,run/'train'/f'episode-{index:03d}',model,collect=True,
                                     limit=reserve,deadline=learning_deadline)
            except PilotBudgetExhausted:
                state['decisions']+=reserve;state['pending']=None
                state['learning_stop']='wall_deadline_partial_episode_charged'
                checkpoint();break
            actual=report['agent_transitions'];state['decisions']+=actual
            if critic: state['critic_decisions']+=actual
            state['pending']=None;state['pending_update']=dict(index=index,critic=critic);checkpoint()
            before_update=deepcopy(dict(model=model.state_dict(),optimizer=trainer.optimizer.state_dict(),rng=flow.capture_rng()))
            try:
                opt=critic_update(model,trainer,batch,learning_deadline) if critic else trainer.update(batch,deadline=learning_deadline)
            except BaseException:
                model.load_state_dict(before_update['model']);trainer.optimizer.load_state_dict(before_update['optimizer'])
                flow.restore_rng(before_update['rng']);raise
            state['critic_optimizer_steps' if critic else 'ppo_optimizer_steps']+=opt['optimizer_steps']
            state['updates']+=1;state['episodes']+=1
            state['critic_episodes' if critic else 'ppo_episodes']+=1
            state['pending_update']=None
            state['last_kl']=opt.get('last_kl',opt.get('approx_kl',0.))
            key=row['case']['id'];entry=state['coverage'].setdefault(key,dict(ppo_decisions=0,critic_decisions=0,episodes=0,merge_entries=0))
            entry['critic_decisions' if critic else 'ppo_decisions']+=actual
            entry['episodes']+=1;entry['merge_entries']+=report.get('stable_merge',{}).get('entered',0)
            save(run/'updates'/f'{index:03d}.json',dict(report=report,optimization=opt,critic_only=critic,row=row))
            checkpoint()
        state['phase']='final_evaluation';checkpoint()
        final=evaluate(cfg,run,'final-validation',model,deadline)
        export_actor(model,run/'policy.pt')
        state['phase']='complete';checkpoint()
        result=dict(completed=True,failed=False,training=True,round_index=cfg['round_index'],
            initial=initial['summary'],final=final['summary'],**final['summary'],
            initial_pooled_intervention_fraction=initial['summary']['pooled_intervention_fraction'],
            approximate_kl=state.get('last_kl',0.),
            state=state,elapsed_seconds=elapsed(),total_charged_seconds=previous+elapsed(),
            checkpoint=str(run/'last.pt'),policy=str(run/'policy.pt'),training_acceptance='experimental_not_promoted')
        save(run/'summary.json',result);segment.update(status='complete',charged_seconds=elapsed());save(ledger_path,ledger)
        print(json.dumps({k:result[k] for k in ('completed','round_index','final','elapsed_seconds','checkpoint')}))
        return 0
    except BaseException:
        if state.get('pending'):
            state['decisions']+=state['pending']['reserved'];state['pending']=None
        state['phase']='failed';checkpoint()
        failure=dict(completed=False,failed=True,traceback=traceback.format_exc(),state=state,
            elapsed_seconds=elapsed(),total_charged_seconds=previous+elapsed())
        save(run/'failure.json',failure);segment.update(status='failed',charged_seconds=elapsed(),decisions=state['decisions'])
        save(ledger_path,ledger);print(json.dumps(failure));return 1
    finally:
        if lock.exists(): lock.unlink()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('command',choices=['preflight','train'])
    p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--preflight',type=Path);p.add_argument('--checkpoint',type=Path);p.add_argument('--resume',action='store_true')
    a=p.parse_args()
    if a.command=='train' and not a.preflight:p.error('Measured preflight required')
    if a.resume and not a.checkpoint:p.error('Resume checkpoint required')
    return run(a)


if __name__=='__main__':raise SystemExit(main())
