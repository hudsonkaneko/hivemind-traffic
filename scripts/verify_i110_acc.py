"""Run the declared bounded comfort/attribution checks in fresh child processes."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import os
from dataclasses import asdict

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from traffic.smooth_acc import ComfortConfig


def verify(network,output,config):
    plan=json.loads(Path(config).read_text())
    # This frozen v1 harness uses the baseline CLI defaults. Fail closed if a
    # future plan changes them instead of silently running a different study.
    if (plan['comfort'] != asdict(ComfortConfig()) or plan['step_s'] != .1
        or plan['horizon_s'] != 60 or plan['drain_budget_s'] != 90
        or plan['verification_seeds'] != [730201,730202]
        or plan['cases'] != ['ordinary','braking_leader','stopped_leader']
        or plan['repeat'] != dict(case='braking_leader',seed=730201,controller='smooth_acc',passing='parallel')):
        raise ValueError('This frozen v1 harness requires its declared defaults; implement a new protocol for changed settings.')
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    (output/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    runs=[('native-default','braking_leader',730201,'native_acc','default'),
          ('native-parallel','braking_leader',730201,'native_acc','parallel')]
    runs += [(f'{case}-{seed}',case,seed,'smooth_acc','parallel') for case in plan['cases'] for seed in plan['verification_seeds']]
    runs.append(('braking-repeat','braking_leader',730201,'smooth_acc','parallel'))
    records=[]
    for name,case,seed,controller,passing in runs:
        command=[sys.executable,'-m','traffic.i110_baseline','--network',str(Path(network).resolve()),
            '--output',str(output/name),'--case',case,'--seed',str(seed),'--controller',controller,
            '--passing',passing,'--observe-policy-interface','--study-id',plan['study_id'],'--reason',plan['trigger']]
        started=time.monotonic()
        with (output/(name+'-process.log')).open('x') as log:
            process=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
            try:code=process.wait(timeout=120)
            except subprocess.TimeoutExpired:
                if os.name=='nt':subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],capture_output=True,timeout=15)
                else:process.kill()
                code=124
        row=dict(name=name,case=case,seed=seed,controller=controller,passing=passing,
            command=command,exit_code=code,wall_s=time.monotonic()-started,wall_limit_s=120)
        records.append(row)
        (output/'processes.json').write_text(json.dumps(records,indent=2)+'\n')
        print(json.dumps({k:row[k] for k in ('name','exit_code','wall_s')}),flush=True)
        if code:raise RuntimeError('Verification case failed; preserved: '+name)
    hashes={name:json.loads((output/name/'metrics.json').read_text())['trace_sha256'] for name,*_ in runs}
    repeated=hashes['braking_leader-730201']==hashes['braking-repeat']
    assert repeated,'Same-seed replay changed'
    (output/'verification.json').write_text(json.dumps(dict(passed=True,repeat_exact=repeated,
        plan_sha256=hashlib.sha256(Path(config).read_bytes()).hexdigest(),runs=records,trace_hashes=hashes),indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--network',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--config',type=Path,default=ROOT/'experiments/configs/i110_acc_comfort_v1.json')
    verify(**vars(p.parse_args()))
