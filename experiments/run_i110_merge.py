"""Separate I-110 merge MAPPO pilot; default prints config, no implicit training."""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import random
import shutil
import platform
import subprocess
import time
import traceback
import uuid
import numpy as np
import torch
from environments.i110_merge_env import I110MergeEnv, AGENTS, SCHEMA, LOCAL_FEATURES, MESSAGE_FEATURES
from policies.mappo import MappoActorCritic, MappoTrainer, compute_gae, export_actor
from experiments.sumo_mappo_support import atomic_json, config_digest

ROOT=Path(__file__).resolve().parents[1]
DEFAULT=ROOT/'experiments/configs/i110_merge_v1.json'
RANKING_VERSION=2


class PilotBudgetExhausted(TimeoutError):
    """Normal completion of the explicitly bounded training budget."""


def restore_selected_checkpoint(resume, output, fingerprint):
    source=Path(resume).parent/'best.pt'
    if not source.exists():return
    selected=torch.load(source,map_location='cpu',weights_only=False)
    if selected.get('schema')!=SCHEMA or selected.get('config_sha256')!=fingerprint or selected.get('selection_status')!='validation_selected':
        raise ValueError('Prior selected checkpoint contract mismatch')
    actor=source.parent/'policy.pt';contract_source=source.parent/'policy-contract.json'
    if not actor.exists() or not contract_source.exists():
        raise ValueError('Prior selected actor export or contract is missing')
    sidecar=json.loads(contract_source.read_text())
    if sidecar.get('policy_sha256')!=hashlib.sha256(actor.read_bytes()).hexdigest():
        raise ValueError('Prior selected actor checksum mismatch')
    score=selected['best_rank']
    if selected.get('ranking_version')!=RANKING_VERSION:
        reports=source.parent/f"validation-{selected['update']:05d}.json"
        if not reports.exists():raise ValueError('Prior validation reports required to migrate checkpoint ranking')
        score=rank(json.loads(reports.read_text()))
    for name in ('best.pt','policy.pt','policy-contract.json'):
        item=source.parent/name
        shutil.copyfile(item,Path(output)/name)
    selected.update(best_rank=score,ranking_version=RANKING_VERSION)
    save_checkpoint(Path(output)/'best.pt',selected)
    contract=Path(output)/'policy-contract.json'
    if contract.exists():
        sidecar=json.loads(contract.read_text())
        sidecar.update(validation_rank=score,ranking_version=RANKING_VERSION)
        atomic_json(contract,sidecar)
    return tuple(score)


def load_config(path):
    cfg=json.loads(Path(path).read_text())
    if cfg['observation_schema']!=SCHEMA or cfg['initial_checkpoint'] is not None:
        raise ValueError('New graph ABI requires fresh initialization; legacy v04 weights are incompatible')
    if cfg['environment']['horizon']!=800 or cfg['model']!={'observation_size':81,'state_size':328,'num_agents':4}:
        raise ValueError('Unsupported I-110 pilot contract')
    if not 4<=cfg['training']['max_agent_transitions']<=500000 or not 1<=cfg['training']['max_wall_seconds']<=7200:
        raise ValueError('Pilot budget exceeds reviewed bounds')
    sets=[set(cfg['evaluation'][k]) for k in ('validation_seeds','test_seeds','preflight_seeds')]
    if any(a&b for i,a in enumerate(sets) for b in sets[i+1:]):raise ValueError('Seed partitions overlap')
    if any(cfg['training']['seed_start']<=s<=cfg['training']['seed_end'] for subset in sets for s in subset):
        raise ValueError('Training and evaluation seeds overlap')
    return cfg


def model_for(cfg):
    return MappoActorCritic(**cfg['model'],message_encoder=True,hybrid_actions=True)


def env_for(cfg, output, seed=None, split='train'):
    bank=[c for c in cfg['curriculum'] if c['split']==split]
    if not bank:raise ValueError('Empty curriculum partition')
    case=bank[(int(seed or 0))%len(bank)]
    network=ROOT/case['network']
    if hashlib.sha256(network.read_bytes()).hexdigest()!=case['network_sha256']:raise ValueError('Curriculum network hash mismatch')
    env=I110MergeEnv(network,output,**cfg['environment'],through_lanes=case['through_lanes'],task=case['task'],density=case['density'])
    env.case_id=case['id']
    return env


def tensor_state(env):return torch.as_tensor(env.state()).repeat(4,1)


def episode(env, seed, *, model=None, kind='scripted', collect=False, deadline=None, limit=None):
    obs,_=env.reset(seed=seed);rng=np.random.default_rng(seed+70000000)
    env.control_mode='native' if kind=='native' else 'policy'
    rows=[];reward_sum=0.;transitions=0
    try:
        while env.agents:
            if deadline is not None and time.monotonic()>=deadline:raise PilotBudgetExhausted('Pilot wall budget exhausted')
            if limit is not None and transitions+len(env.agents)>limit:break
            active=torch.tensor([a in env.agents for a in AGENTS]);indices=torch.arange(4)
            local=torch.tensor(np.stack([obs.get(a,np.zeros(81,np.float32)) for a in AGENTS]))
            state=tensor_state(env)
            if kind=='learned':
                with torch.no_grad():
                    actions,logp,_,pre=model.act(local,deterministic=not collect)
                    values=model.values(state,indices)
                chosen={a:actions[i].numpy() for i,a in enumerate(AGENTS) if a in env.agents}
            elif kind=='scripted':chosen=env.scripted_actions()
            elif kind=='native':
                for a in env.agents:
                    env.conn.vehicle.setSpeed(a,-1);env.conn.vehicle.setLaneChangeMode(a,1621)
                chosen=env.scripted_actions()
            else:
                chosen={a:np.array([rng.uniform(-1,1),rng.choice(np.flatnonzero(obs[a][30:33]>.5))-1],np.float32) for a in env.agents}
            next_obs,rewards,term,trunc,info=env.step(chosen)
            reward_sum+=sum(rewards.values());transitions+=len(rewards)
            if collect:
                with torch.no_grad():next_values=model.values(tensor_state(env),indices)
                rows.append(dict(local_obs=local,global_states=state,agent_indices=indices,actions=actions,
                    pre_tanh_actions=pre,old_log_probs=logp,old_values=values,next_values=next_values,
                    rewards=torch.tensor([rewards.get(a,0.) for a in AGENTS],dtype=torch.float32),
                    terminated=torch.tensor([term.get(a,False) for a in AGENTS]),
                    truncated=torch.tensor([trunc.get(a,False) for a in AGENTS]),active=active))
            obs={a:next_obs[a] for a in env.agents}
        # Genuine final observation must bootstrap time limits; state() masks
        # departed agents, so compute this explicitly for truncation-only slots.
        if rows and any(trunc.values()):
            final=np.concatenate([np.r_[next_obs.get(a,np.zeros(81)),float(not term.get(a,True))] for a in AGENTS]).astype(np.float32)
            with torch.no_grad():rows[-1]['next_values']=model.values(torch.tensor(final).repeat(4,1),torch.arange(4))
        report=env.finish_accounting()
        report.update(seed=seed,controller=kind,case_id=env.case_id,reward=reward_sum,agent_transitions=transitions,
                      controlled_completed=report['accounting']['groups']['role:controlled']['completed'])
        if collect and rows:
            batch={k:torch.stack([r[k] for r in rows]) for k in rows[0]}
            batch['advantages'],batch['returns']=compute_gae(batch['rewards'],batch['old_values'],batch['next_values'],
                batch['terminated'],batch['truncated'],batch['active'])
            return report,batch
        return report,None
    finally:env.close()


def rank(reports):
    # Passing the basic driving task must precede intervention minimization.
    # Otherwise a stationary policy can beat a useful moving policy merely by
    # requesting fewer actions while both leave long-corridor trips unfinished.
    missing=0
    cruise=[]
    for r in reports:
        if r['task']=='cruise':
            cruise.append(r['mean_fraction_of_speed_limit'])
            missing+=r['mean_fraction_of_speed_limit']<.9
        elif r['task']=='passing':missing+=r['passes']<1
        elif r['task']=='merge':missing+=r['successful_merges']<2
    return (-sum(r['collisions']+r['teleports']+r['overspeed_steps'] for r in reports),
        -missing, sum(r['successful_merges'] for r in reports),sum(r['passes'] for r in reports),
        min(cruise,default=0.),
        -sum(r['accounting']['totals']['unfinished'] for r in reports),
        -sum(r['overrides'] for r in reports)/max(1,sum(r['agent_steps'] for r in reports)),
        sum(r['reward'] for r in reports))


def save_checkpoint(path, payload):
    tmp=path.with_suffix('.tmp');torch.save(payload,tmp);tmp.replace(path)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=DEFAULT)
    parser.add_argument('--mode',choices=('config','preflight','train','evaluate'),default='config')
    parser.add_argument('--approved-config-sha256')
    parser.add_argument('--resume',type=Path)
    parser.add_argument('--checkpoint',type=Path)
    parser.add_argument('--split',choices=('validation','test'),default='validation')
    args=parser.parse_args(argv)
    if Path.cwd().resolve()!=ROOT:raise ValueError('Run from project root')
    cfg=load_config(args.config);fingerprint=config_digest(cfg)
    if args.mode=='config':print(json.dumps(dict(config=cfg,sha256=fingerprint),indent=2));return 0
    for case in cfg['curriculum']:
        if hashlib.sha256((ROOT/case['network']).read_bytes()).hexdigest()!=case['network_sha256']:raise ValueError('Frozen network hash mismatch')
    if args.mode=='train' and (not cfg['training_authorized'] or args.approved_config_sha256!=fingerprint):
        raise ValueError('Training awaits final config review and matching authorization hash')
    torch.set_num_threads(1);torch.manual_seed(cfg['training']['seed']);random.seed(cfg['training']['seed'])
    np.random.seed(cfg['training']['seed']);torch.use_deterministic_algorithms(True)
    run=ROOT/'outputs/i110-merge/v01'/(args.mode+'-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:8])
    run.mkdir(parents=True);atomic_json(run/'config.json',cfg)
    sources=['environments/i110_merge_env.py','experiments/run_i110_merge.py','policies/mappo.py',
             'traffic/i110_policy_interface.py','traffic/trip_accounting.py',
             'scripts/build_i110_sumo.py','scripts/build_highway_curriculum.py','scenarios/i110/sumo-road-profiles.json']
    hashes={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sources}
    import traci
    atomic_json(run/'provenance.json',dict(source_hashes=hashes,network_hashes={c['id']:c['network_sha256'] for c in cfg['curriculum']},
        config_sha256=fingerprint,git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        python=platform.python_version(),torch=torch.__version__,traci=traci.__version__,
        training=args.mode=='train',source_import='documentation/evidence/i110-merge-training-v01/source-import.json'))
    atomic_json(run/'contract.json',dict(schema=SCHEMA,local_features=LOCAL_FEATURES,message_features=MESSAGE_FEATURES,
        actor_compatible_with_v04=False,information_source='exact current SUMO state and delivered delayed messages, not cameras/LiDAR'))
    result=dict(passed=False,training=args.mode=='train',optimizer_steps=0)
    started=time.monotonic();model=None;trainer=None
    try:
        if args.mode=='preflight':
            from pettingzoo.test import parallel_api_test, parallel_seed_test
            factory=lambda:env_for(cfg,run/'api')
            api=factory()
            try:parallel_api_test(api,num_cycles=100)
            finally:api.close()
            parallel_seed_test(factory,num_cycles=100)
            reports=[]
            for kind in ('scripted','random','native'):
                seeds=cfg['evaluation']['preflight_seeds'] if kind=='scripted' else cfg['evaluation']['preflight_seeds'][:2]
                for seed in seeds:
                    report,_=episode(env_for(cfg,run/kind,seed),seed,kind=kind,deadline=started+600)
                    atomic_json(run/f'{kind}-{seed}.json',report);reports.append(report)
            seed=cfg['evaluation']['preflight_seeds'][0]
            repeat,_=episode(env_for(cfg,run/'repeat',seed),seed,deadline=started+600)
            assert reports[0]==repeat,'Seeded scripted rollout changed'
            model=model_for(cfg)
            probe=env_for(cfg,run/'model-contract')
            try:
                obs,_=probe.reset(seed=cfg['evaluation']['preflight_seeds'][0])
                with torch.no_grad():a,_,_,pre=model.act(torch.tensor(np.stack(list(obs.values()))))
                assert a.shape==(4,2) and torch.isfinite(pre).all()
            finally:probe.close()
            report,batch=episode(env_for(cfg,run/'collector',seed),seed,model=model,kind='learned',collect=True,limit=128,deadline=started+600)
            assert all(torch.isfinite(v).all() for v in batch.values()),'Nonfinite training rollout'
            result.update(passed=all(r['collisions']==0 and r['teleports']==0 for r in reports),
                          reports=[{k:r[k] for k in ('seed','controller','case_id','task','through_lanes','successful_merges','passes','collisions','teleports','intervention_fraction','mean_fraction_of_speed_limit')} for r in reports],
                          api_passed=True,seed_test_passed=True,scripted_repeat_exact=True,model_forward_passed=True,collector_finite=True)
        elif args.mode=='evaluate':
            if args.checkpoint is None or not args.checkpoint.resolve().is_relative_to(ROOT/'outputs'):
                raise ValueError('Provide a project-local frozen checkpoint')
            saved=torch.load(args.checkpoint,map_location='cpu',weights_only=False)
            if saved['schema']!=SCHEMA or saved['config_sha256']!=fingerprint:raise ValueError('Checkpoint contract mismatch')
            if args.split=='test' and saved.get('selection_status')!='validation_selected':raise ValueError('Test requires validation-selected checkpoint')
            model=model_for(cfg);model.load_state_dict(saved['model']);model.eval()
            reports=[]
            for seed in cfg['evaluation'][args.split+'_seeds']:
                report,_=episode(env_for(cfg,run/args.split,seed,args.split),seed,model=model,kind='learned',deadline=started+1800)
                atomic_json(run/f'{args.split}-{seed}.json',report);reports.append(report)
            result.update(passed=True,reports=reports,quality_accepted=False,split=args.split)
        else:
            model=model_for(cfg);trainer=MappoTrainer(model,**cfg['optimizer'])
            counter=update=0;best=None;prior=0.;next_seed=cfg['training']['seed_start']
            if args.resume:
                if not args.resume.resolve().is_relative_to(ROOT/'outputs'):raise ValueError('Resume only local project checkpoint')
                saved=torch.load(args.resume,map_location='cpu',weights_only=False)
                if saved['config_sha256']!=fingerprint or saved['schema']!=SCHEMA:raise ValueError('Resume contract mismatch')
                model.load_state_dict(saved['model']);trainer.optimizer.load_state_dict(saved['optimizer'])
                counter,update,best,prior,next_seed=(saved[k] for k in ('transitions','update','best_rank','elapsed_seconds','next_seed'))
                torch.set_rng_state(saved['torch_rng'])
                best=restore_selected_checkpoint(args.resume,run,fingerprint)
                if saved['best_rank'] is not None and best is None:
                    raise ValueError('Resume is missing the prior selected actor')
            deadline=started+max(0,cfg['training']['max_wall_seconds']-prior)
            def checkpoint(name):
                save_checkpoint(run/name,dict(schema=SCHEMA,model=model.state_dict(),optimizer=trainer.optimizer.state_dict(),
                    selection_status='validation_selected' if name=='best.pt' else 'resume_only',
                    config_sha256=fingerprint,config=cfg,transitions=counter,update=update,best_rank=best,ranking_version=RANKING_VERSION,
                    elapsed_seconds=prior+time.monotonic()-started,next_seed=next_seed,torch_rng=torch.get_rng_state()))
            checkpoint('last.pt')
            while counter<cfg['training']['max_agent_transitions'] and time.monotonic()<deadline:
                if next_seed>cfg['training']['seed_end']:break
                report,batch=episode(env_for(cfg,run/'train',next_seed),next_seed,model=model,kind='learned',collect=True,
                    deadline=deadline,limit=cfg['training']['max_agent_transitions']-counter)
                if batch is None:break
                stats=trainer.update(batch);counter+=int(batch['active'].sum());update+=1;next_seed+=1
                result['optimizer_steps']+=stats['optimizer_steps']
                atomic_json(run/f'train-{update:05d}.json',dict(episode=report,optimization=stats))
                checkpoint('last.pt')
                if update%cfg['evaluation']['every_episodes']==0:
                    reports=[]
                    for seed in cfg['evaluation']['validation_seeds']:
                        r,_=episode(env_for(cfg,run/'validation',seed,'validation'),seed,model=model,kind='learned',deadline=deadline);reports.append(r)
                    atomic_json(run/f'validation-{update:05d}.json',reports)
                    score=rank(reports)
                    if best is None or score>tuple(best):
                        best=score;checkpoint('best.pt');export_actor(model,run/'policy.pt')
                        atomic_json(run/'policy-contract.json',dict(schema=SCHEMA,config_sha256=fingerprint,
                            validation_rank=best,ranking_version=RANKING_VERSION,observation_size=81,action_size=2,legacy_v04_compatible=False,
                            policy_sha256=hashlib.sha256((run/'policy.pt').read_bytes()).hexdigest()))
                checkpoint('last.pt')
            result.update(passed=True,transitions=counter,updates=update,best_rank=best,held_out_test_run=False)
    except Exception as error:
        if args.mode=='train' and (run/'last.pt').exists():
            saved=torch.load(run/'last.pt',map_location='cpu',weights_only=False)
            saved['elapsed_seconds']=prior+time.monotonic()-started
            save_checkpoint(run/'last.pt',saved)
            if isinstance(error,PilotBudgetExhausted):
                result.update(passed=True,stop_reason='wall_time_budget',transitions=saved['transitions'],
                    updates=saved['update'],best_rank=saved['best_rank'],held_out_test_run=False,
                    incomplete_episode_or_validation_discarded=True)
        if not result['passed']:
            result.update(error=repr(error),traceback=traceback.format_exc());traceback.print_exc()
    finally:
        result['sources_changed']=[p for p,h in hashes.items() if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h]
        if result['sources_changed']:result['passed']=False
        result['wall_seconds']=time.monotonic()-started;atomic_json(run/'summary.json',result)
    print(json.dumps(dict(run=str(run),**result),indent=2));return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
