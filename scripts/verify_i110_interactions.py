"""Execute predeclared ramp exposure and cut-in mechanism checks, preserving failures."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from traffic.i110_baseline import sha, write_json
from traffic.i110_events import load_event


def verify(output,phase='verification'):
    plan_path=ROOT/'experiments/configs/i110_ramp_cutin_v1.json'
    plan=json.loads(plan_path.read_text(encoding='utf-8'))
    # All values used by the fixed v1 matrix are checked instead of ignored.
    expected=dict(schema_version=1,development_seed=732001,ramp_verification_seeds=[732101,732102],
        ramp_agent0_departures_s=[0,38,44,50],ramp_controllers=['native_acc','smooth_acc'],
        ramp_repeat=dict(seed=732101,agent0_depart_s=44,controller='smooth_acc'),
        cutin_verification_seed=732101,cutin_controllers=['smooth_acc','follow_cap_acc'],
        cutin_lane_request=[False,True],
        cutin_repeat=dict(seed=732101,controller='follow_cap_acc',request_lane_change=True),
        step_s=.1,horizon_s=90,drain_budget_s=90,wall_limit_s=120,passing='parallel',
        safety=dict(speed_mode=31,controlled_lane_change_mode=512,native_ramp_lane_changes=True))
    if any(plan.get(k)!=v for k,v in expected.items()):
        raise ValueError('Changed v1 matrix requires explicit supervisor update')
    source_event=ROOT/plan['cutin_event'];event=load_event(source_event)
    network=ROOT/plan['network']
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    shutil.copyfile(plan_path,output/'plan.json');shutil.copyfile(__file__,output/'supervisor.py')
    inputs={str(p):sha(p) for p in (plan_path,network,source_event)}
    write_json(output/'input-hashes.json',inputs)
    for lateral in (False,True):
        write_json(output/f'cutin-{int(lateral)}.json',dict(event,request_lane_change=lateral))
    if phase=='development':
        ramp_runs=[dict(kind='ramp',seed=732001,controller='smooth_acc',depart=d,lateral=None,
                        name=f'ramp-{d}') for d in [0,38,44,50]]
        cut_runs=[dict(kind='cutin',seed=732001,controller=c,depart=0,lateral=l,
                      name=f'cutin-{c}-{int(l)}') for c in plan['cutin_controllers'] for l in (False,True)]
        ramp_runs.insert(1,dict(kind='control',seed=732001,controller='smooth_acc',depart=0,lateral=None,name='telemetry-off'))
    else:
        ramp_runs=[dict(kind='ramp',seed=s,controller=c,depart=d,lateral=None,name=f'ramp-{d}-{s}-{c}')
                   for s in plan['ramp_verification_seeds'] for d in plan['ramp_agent0_departures_s']
                   for c in plan['ramp_controllers']]
        ramp_runs.append(dict(kind='ramp',seed=732101,controller='smooth_acc',depart=44,lateral=None,name='ramp-repeat'))
        cut_runs=[dict(kind='cutin',seed=732101,controller=c,depart=0,lateral=l,
                      name=f'cutin-{c}-{int(l)}') for c in plan['cutin_controllers'] for l in (False,True)]
        cut_runs.append(dict(kind='cutin',seed=732101,controller='follow_cap_acc',depart=0,lateral=True,name='cutin-repeat'))
    records=[]
    for spec in ramp_runs+cut_runs:
        command=[sys.executable,'-m','traffic.i110_baseline','--network',str(network),
            '--output',str(output/spec['name']),'--seed',str(spec['seed']),'--controller',spec['controller'],
            '--agent0-depart',str(spec['depart']),'--horizon','90','--drain','90','--passing','parallel',
            '--observe-policy-interface','--study-id',plan['study_id'],'--reason',plan['trigger']]
        if spec['kind']!='control':command.append('--observe-interactions')
        if spec['kind']=='cutin':command.extend(['--event-spec',str(output/f"cutin-{int(spec['lateral'])}.json")])
        start=time.monotonic()
        with (output/(spec['name']+'-process.log')).open('x') as log:
            proc=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
            try:code=proc.wait(timeout=plan['wall_limit_s'])
            except subprocess.TimeoutExpired:
                if os.name=='nt':subprocess.run(['taskkill','/PID',str(proc.pid),'/T','/F'],capture_output=True,timeout=15)
                else:proc.kill()
                proc.wait(timeout=15);code=124
        row=dict(**spec,command=command,exit_code=code,wall_s=time.monotonic()-start)
        records.append(row);write_json(output/'processes.json',records)
        print(json.dumps({k:row[k] for k in ('name','exit_code','wall_s')}),flush=True)
        run=output/spec['name']
        if code not in (0,1) or not (run/'metrics.json').exists():
            write_json(output/'verification.json',dict(status='infrastructure_failed',runs=records))
            raise RuntimeError('Infrastructure failure retained: '+spec['name'])
    rows={r['name']:json.loads((output/r['name']/'metrics.json').read_text(encoding='utf-8')) for r in records}
    unexposed=[r['name'] for r in records if r['kind']=='ramp' and r['depart']>0 and not rows[r['name']]['interaction_observed']]
    checks={}
    if phase=='development':
        checks['telemetry_does_not_change_trace']=rows['ramp-0']['trace_sha256']==rows['telemetry-off']['trace_sha256']
    else:
        for first,repeat in [('ramp-44-732101-smooth_acc','ramp-repeat'),('cutin-follow_cap_acc-1','cutin-repeat')]:
            checks[repeat+'_trace_exact']=rows[first]['trace_sha256']==rows[repeat]['trace_sha256']
            checks[repeat+'_diagnostics_exact']=sha(output/first/'interactions.jsonl')==sha(output/repeat/'interactions.jsonl')
        checks['cutin_repeat_event_exact']=sha(output/'cutin-follow_cap_acc-1/event-journal.json')==sha(output/'cutin-repeat/event-journal.json')
        for seed in plan['ramp_verification_seeds']:
            for depart in plan['ramp_agent0_departures_s']:
                for file in ('network.net.xml','routes.rou.xml'):
                    checks[f'matched-{seed}-{depart}-{file}']=sha(output/f'ramp-{depart}-{seed}-native_acc'/file)==sha(output/f'ramp-{depart}-{seed}-smooth_acc'/file)
    for file in ('network.net.xml','routes.rou.xml'):
        checks['cutin-all-matched-'+file]=len({sha(output/r['name']/file) for r in cut_runs})==1
    left=json.loads((output/'cutin-0.json').read_text());right=json.loads((output/'cutin-1.json').read_text())
    left.pop('request_lane_change');right.pop('request_lane_change')
    checks['cutin-spec-only-lateral-factor']=left==right
    checks['inputs_unchanged']=all(sha(p)==h for p,h in inputs.items())
    failures=[n for n,m in rows.items() if not m['passed']]
    result=dict(status='completed',phase=phase,checks=checks,unexposed_shifted_cases=unexposed,
        fixture_failures=failures,comfort_failures=[n for n,m in rows.items() if m['comfort_gate_passed'] is False],
        passed=all(checks.values()) and not failures and not unexposed,runs=records,
        limits=plan['limits'])
    write_json(output/'verification.json',result)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--phase',choices=['development','verification'],default='verification')
    result=verify(**vars(parser.parse_args()))
    print(json.dumps({k:v for k,v in result.items() if k!='runs'},indent=2))
    raise SystemExit(0 if result['passed'] else 1)
