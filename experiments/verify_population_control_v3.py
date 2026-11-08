"""Bounded, train-only v2/v3 control comparison. No optimizer or training."""
from __future__ import annotations
import argparse
from copy import deepcopy
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import time
import uuid
import numpy as np
import torch

from experiments.run_highway_population import ROOT, DEFAULT, load_config, env_for, digest
from experiments.diagnose_population_acceleration import ACTOR, ACTOR_SHA256, emergency_events
from traffic.highway_stage1_teacher import teacher_actions

SOURCES=('traffic/highway_stage1_teacher.py','traffic/highway_merge_context.py',
    'environments/highway_stage1_env.py','environments/highway_population_env.py',
    'experiments/run_highway_population.py','experiments/verify_population_control_v3.py')


def write(path, value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')


def episode(cfg,actor,output,spec,deadline):
    cfg=deepcopy(cfg)
    # Explicitly set the target on the environment too, keeping this probe
    # independent of the population runner's training/configuration wiring.
    cfg['environment']['control_target_version']=spec['version']
    case=next(c for c in cfg['training_cases'] if c['id']==spec['case_id'])
    assert case['split']=='train' and 6520000 <= spec['seed'] < 6520200
    row=dict(case=case,seed=spec['seed'],controlled_count=spec.get('controlled_count',1),
        total_vehicles=spec['total_vehicles'],communication='none',packet_loss=0.)
    env=env_for(cfg,output/spec['id'],row,horizon=spec.get('horizon',600))
    env.control_target_version=spec['version']
    records=[]
    try:
        obs,_=env.reset(seed=spec['seed'])
        version=env.conn.getVersion()
        while env.agents:
            if time.monotonic() >= deadline:
                raise TimeoutError('No-learning control verification wall budget reached')
            agents=list(env.agents)
            labels=teacher_actions({a:obs[a] for a in agents},version=spec['version'])
            if spec['controller']=='teacher':
                chosen=labels
            else:
                with torch.inference_mode():
                    action=actor(torch.tensor(np.stack([obs[a] for a in agents]))).numpy()
                chosen=dict(zip(agents,action))
            old={a:dict(env.states[a]) for a in agents}
            observations={a:obs[a].copy() for a in agents}
            t=env.time
            obs,_,_,_,infos=env.step(chosen)
            for a in agents:
                new=env.states.get(a); info=infos[a]; o=observations[a]
                records.append(dict(step=env.steps,agent=a,time_before=t,time_after=env.time,
                    lane_before=old[a]['lane_id'],lane_after=new['lane_id'] if new else None,
                    speed_before=old[a]['speed'],speed_after=new['speed'] if new else None,
                    requested=info['requested_acceleration'],filtered=info['filtered_acceleration'],realized=info['realized_acceleration'],
                    teacher_target=-4.5+(float(labels[a][0])+1)*3.25,reward_target=info['feasible_acceleration_target'],
                    override=info['override'],native_mismatch=info['native_realization_changed'],policy_filter=info['policy_filter_changed'],
                    lane_request=info['lane_request'],lane_rejected=info['lane_rejected'],
                    distance_to_merge_end=float(o[27])*240.,context_known=float(o[33]),context_delta=float(o[35]),
                    context_boundary_distance=float(o[36])*240.,context_front_gap=float(o[37])*120. if o[39]>.5 else None,
                    context_rear_gap=float(o[40])*120. if o[42]>.5 else None,
                    speed_mode=env.conn.vehicle.getSpeedMode(a) if new else None,
                    lane_mode=env.conn.vehicle.getLaneChangeMode(a) if new else None))
        report=env.report(); directory=env.directory; runtime=version
    finally:
        env.close()
        if records:
            with (output/(spec['id']+'.csv')).open('w',newline='',encoding='utf-8') as stream:
                writer=csv.DictWriter(stream,fieldnames=list(records[0]));writer.writeheader();writer.writerows(records)
    log=directory/'sumo-errors.log'
    result=dict(spec=spec,report=report,runtime=runtime,native_warnings=emergency_events(log.read_text(encoding='utf-8')),
        episode_directory=directory.relative_to(ROOT).as_posix(),raw_csv_sha256=digest(output/(spec['id']+'.csv')),
        trace_sha256=hashlib.sha256(json.dumps(records,sort_keys=True,allow_nan=False).encode()).hexdigest(),
        max_teacher_reward_target_error=max(abs(r['teacher_target']-r['reward_target']) for r in records),
        teacher_reward_target_mae=float(np.mean([abs(r['teacher_target']-r['reward_target']) for r in records])),
        max_abs_acceleration=max(abs(r['realized']) for r in records if r['realized'] is not None),
        minimum_speed=min(r['speed_after'] for r in records if r['speed_after'] is not None),
        override_count=sum(r['override'] for r in records),decisions=len(records))
    write(output/(spec['id']+'.json'),result)
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--max-wall-seconds',type=int,default=300)
    parser.add_argument('--config',type=Path,default=DEFAULT)
    parser.add_argument('--merge-only',action='store_true')
    args=parser.parse_args(argv)
    if not 1<=args.max_wall_seconds<=900:
        raise ValueError('One no-learning attempt must be bounded to1..900 seconds')
    if digest(ACTOR)!=ACTOR_SHA256:
        raise ValueError('Frozen actor hash mismatch')
    torch.set_num_threads(1)
    actor=torch.jit.load(str(ACTOR),map_location='cpu').eval()
    cfg=load_config(args.config)
    specs=[]
    cases=(('train-2-lane-cruise',24),('train-2-lane-following',24),('train-3-lane-passing',16),('train-3-lane-merge',16))
    for i,(case,total) in enumerate(cases):
        if args.merge_only and not case.endswith('-merge'):
            continue
        for version in ('v2','v3'):
            for kind in ('teacher','frozen_parent'):
                specs.append(dict(id=f'{case}-{version}-{kind}',case_id=case,seed=6520000+i,
                    total_vehicles=total,version=version,controller=kind))
    specs.append(dict(id='train-3-lane-merge-v3-teacher-repeat',case_id='train-3-lane-merge',seed=6520003,
        total_vehicles=16,version='v3',controller='teacher'))
    for lanes,seed in ((3,6520104),(5,6520105)):
        for version in ('v2','v3'):
            specs.append(dict(id=f'train-{lanes}-lane-merge-{version}-teacher-multi',case_id=f'train-{lanes}-lane-merge',seed=seed,
                total_vehicles=24,controlled_count=4,version=version,controller='teacher'))
    output=ROOT/'outputs/population-control-v03'/f'verify-{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}-{uuid.uuid4().hex[:8]}'
    output.mkdir(parents=True,exist_ok=False)
    before={p:digest(ROOT/p) for p in SOURCES}
    for source in SOURCES:
        destination=output/'source'/source;destination.parent.mkdir(parents=True,exist_ok=True);destination.write_bytes((ROOT/source).read_bytes())
    write(output/'resolved-config.json',cfg)
    result=dict(schema='population_control_v3_no_learning',learning=False,status='running',scope='training roads only',
        source_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        git_status=subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).splitlines(),
        source_sha256_before=before,actor_sha256=ACTOR_SHA256,runtime=dict(python=platform.python_version(),torch=torch.__version__),
        specs=specs,episodes=[])
    start=time.monotonic()
    try:
        for spec in specs:
            result['episodes'].append(episode(cfg,actor,output,spec,start+args.max_wall_seconds))
        first=next(e for e in result['episodes'] if e['spec']['id']=='train-3-lane-merge-v3-teacher')
        repeated=next(e for e in result['episodes'] if e['spec']['id'].endswith('-repeat'))
        result['repeat_trace_equal']=first['trace_sha256']==repeated['trace_sha256']
        result['status']='complete'
    except Exception as error:
        result.update(status='failed',error=repr(error));raise
    finally:
        result['wall_seconds']=time.monotonic()-start
        result['source_sha256_after']={p:digest(ROOT/p) for p in SOURCES}
        result['actor_sha256_after']=digest(ACTOR)
        result['frozen_actor_unchanged']=result['actor_sha256_after']==ACTOR_SHA256
        write(output/'control-summary.json',result)
        print(json.dumps(dict(output=str(output),status=result['status'],wall_seconds=result['wall_seconds'])))


if __name__=='__main__':
    main()
