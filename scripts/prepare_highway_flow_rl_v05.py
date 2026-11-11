"""Saved-trace audit and matched frozen SUMO comparisons; no optimizer updates."""
import argparse
from copy import deepcopy
import gzip
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from experiments import run_highway_flow_rl_v05 as runner
from traffic.highway_flow_rl_v05_rewards import stable_target, condition_acceleration, extra_rewards
from traffic.highway_flow_rl_v03_rewards import speed_error_reward


def audit(output):
    source=ROOT/'outputs/highway-flow-rl/v04/long-round-03/final-validation.json'
    reports=json.loads(source.read_text())['reports']
    cases=[]
    for report in reports:
        cars, memory, totals={}, {}, dict(speed_error=0.,jerk_excess=0.,merge_tracking=0.)
        paths=sorted((ROOT/report['episode_directory']).rglob('actions.jsonl.gz'))
        for path in paths:
            with gzip.open(path,'rt') as f:
                for line in f:
                    row=json.loads(line)
                    for agent,info in row['agents'].items():
                        a=info['v04_reward_audit'];pre=a['pre_action']
                        target,plan,reference=stable_target(pre['ego_state'],pre['context'],pre['map_preview'],memory.get(agent))
                        memory[agent]=dict(target=target,plan=plan)
                        conditioned=condition_acceleration(info['requested_acceleration'],pre['ego_state']['acceleration'],info['feasible_acceleration_ceiling'])
                        new=extra_rewards(info['realized_jerk_m_s3'],conditioned['raw_requested_jerk'],target,a['measured_speed_m_s'],merge_active=info['stable_merge_status']=='pending')
                        new['speed_error']=speed_error_reward(target,a['measured_speed_m_s'],.45)
                        for k,v in new.items(): totals[k]+=v
                        item=cars.setdefault(agent,dict(first_time=row['time'],samples=0,first_peak=0.,later_peak=0.,hypothetical_conditioning_changes=0))
                        key='first_peak' if item['samples']==0 else 'later_peak'
                        item[key]=max(item[key],abs(info['realized_jerk_m_s3'] or 0.))
                        item['samples']+=1
                        item['hypothetical_conditioning_changes']+=conditioned['changed']
        cases.append(dict(case_id=report['case_id'],per_car=cars,new_reward_on_fixed_trace=totals,
            inputs=[dict(path=p.relative_to(ROOT).as_posix(),sha256=runner.file_digest(p)) for p in paths]))
    result=dict(training=False,simulation_run=False,optimizer_steps=0,
        reference=source.relative_to(ROOT).as_posix(),reference_sha256=runner.file_digest(source),cases=cases,
        limitation='Recomputed rewards and one-step conditioning on unchanged saved states; no counterfactual trajectory or improvement claim')
    runner.save(output/'offline-audit.json',result)
    return result


def comparison(output,config):
    cfg=json.loads(config.read_text())
    jobs=[]
    final='outputs/highway-flow-rl/v04/long-round-03/last.pt'
    for variant,checkpoint,label in [('legacy',final,'legacy-final'),('continuity',final,'continuity-final'),
                                     ('full',final,'full-final'),('full',cfg['warm_start']['path'],'full-best')]:
        current=deepcopy(cfg);current['variant']=variant
        path=output/(label+'-config.json');runner.save(path,current)
        args=SimpleNamespace(command='evaluate',config=path,output=output/label,checkpoint=Path(checkpoint),
            preflight=None,resume=False,case_index=None,limit_steps=None,record=True,final=False,max_wall_seconds=600.)
        result=runner.run(args)
        jobs.append(dict(label=label,variant=variant,checkpoint=checkpoint,checkpoint_sha256=runner.file_digest(ROOT/checkpoint),
            output=(output/label).relative_to(ROOT).as_posix(),elapsed_seconds=result['elapsed_seconds'],summary=result['summary']))
        runner.save(output/'comparison.json',dict(training=False,optimizer_steps=0,jobs=jobs,complete=False))
        print(json.dumps(dict(completed=label,seconds=result['elapsed_seconds'])),flush=True)
    # Targets/rewards must not silently become a scripted acceleration controller.
    equal=[]
    for i in range(6):
        a=output/'continuity-final'/'evaluation'/f'case-{i:02d}'/'recording.json'
        b=output/'full-final'/'evaluation'/f'case-{i:02d}'/'recording.json'
        same=json.loads(a.read_text())['frames']==json.loads(b.read_text())['frames']
        equal.append(same)
    result=dict(training=False,optimizer_steps=0,jobs=jobs,complete=True,
        reward_only_variant_motion_identical=equal,
        scope='Same six development seeds/horizons; no held-out test and no acceptance/promotion')
    runner.save(output/'comparison.json',result)
    if not all(equal):raise ValueError('Reward-only change unexpectedly changed frozen actor motion')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=('audit','compare'))
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/training/highway-flow-rl-v05-pilot.json')
    args=parser.parse_args();out=args.output.resolve()
    if Path.cwd().resolve()!=ROOT or not out.is_relative_to(ROOT/'outputs/highway-flow-rl/v05') or out.exists():
        raise ValueError('Canonical workspace and fresh v05 output required')
    out.mkdir(parents=True);started=time.monotonic()
    try:
        audit(out) if args.command=='audit' else comparison(out,args.config.resolve())
        runner.save(out/'completion.json',dict(complete=True,training=False,optimizer_steps=0,wall_seconds=time.monotonic()-started))
    except BaseException:
        import traceback
        runner.save(out/'failure.json',dict(traceback=traceback.format_exc(),training=False,wall_seconds=time.monotonic()-started))
        raise


if __name__=='__main__':main()
