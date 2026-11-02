"""Run the frozen preview development matrix without hiding failed treatments."""
import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.report_i110_anticipation import read,write,sha,verify_run
from traffic.i110_preview import PreviewConfig
from traffic.i110_preview_v2 import PreviewV2Config
from traffic.i110_preview_v2_1 import PreviewV21Config


def verify(output, config=None):
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    plan_path=Path(config) if config else ROOT/'experiments/configs/i110_preview_v1.json'
    plan=read(plan_path)
    kind=plan.get('controller_mode','preview_acc')
    defaults={'preview_v2_acc':PreviewV2Config,'preview_v2_1_acc':PreviewV21Config,'preview_acc':PreviewConfig}[kind]()
    if asdict(defaults)!=plan['preview']:
        raise ValueError('Executed defaults differ from frozen preview contract')
    write(output/'plan.json',plan)
    shutil.copyfile(__file__,output/'supervisor.py')
    baselines={name:ROOT/plan['source_suite']/name for name in plan['runs']}
    for path in baselines.values():verify_run(path)
    records=[]

    def execute(name,case,controller,radius):
        config=read(baselines[case]/'resolved-config.json')
        command=[sys.executable,'-m','traffic.i110_baseline','--network',str(ROOT/plan['network']),
            '--output',str(output/name),'--seed',str(plan['seed']),'--controller',controller,
            '--preview-range',str(radius),'--agent0-depart',str(config['agent0_depart_s']),
            '--horizon',str(plan['horizon_s']),'--drain',str(plan['drain_s']),
            '--passing','parallel','--observe-policy-interface','--observe-interactions',
            '--study-id',plan['study_id'],'--reason',plan['trigger']]
        if case.startswith('cutin'):
            command+=['--event-spec',str(baselines[case]/'event-spec.json')]
        start=time.monotonic()
        with (output/(name+'-process.log')).open('x',encoding='utf-8') as log:
            process=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
            try:code=process.wait(timeout=plan['wall_limit_s'])
            except subprocess.TimeoutExpired:
                if os.name=='nt':subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],capture_output=True,timeout=15)
                else:process.kill()
                process.wait(timeout=15);code=124
        record=dict(name=name,case=case,controller=controller,range_m=radius,
                    command=command,exit_code=code,wall_s=time.monotonic()-start)
        records.append(record);write(output/'processes.json',records)
        print(json.dumps({k:record[k] for k in ('name','exit_code','wall_s')}),flush=True)
        if code not in (0,1) or not (output/name/'metrics.json').exists():
            raise RuntimeError('Infrastructure failure retained: '+name)
        for file in ('network.net.xml','routes.rou.xml','scenario.sumocfg'):
            if sha(output/name/file)!=sha(baselines[case]/file):
                raise ValueError('Treatment changed frozen '+file)
        if case.startswith('cutin') and sha(output/name/'event-spec.json')!=sha(baselines[case]/'event-spec.json'):
            raise ValueError('Treatment changed event specification')
        return read(output/name/'metrics.json')

    try:
        baseline=execute('baseline-regression','ramp-44','smooth_acc',240)
        baseline_exact=baseline['trace_sha256']==read(baselines['ramp-44']/'metrics.json')['trace_sha256']
        if not baseline_exact:raise ValueError('Edited runner changed the complete baseline trace')
        # Repeat the first bounded primary case before scaling the new controller.
        smoke_repeats={}
        for radius in [plan['primary_range_m'],plan['sensitivity_range_m']]:
            for case in plan['runs']:
                name=f'{case}-r{radius}'
                measured=execute(name,case,kind,radius)
                if plan.get('smoke_repeat')==name:
                    repeated=execute(name+'-smoke-repeat',case,kind,radius)
                    exact=dict(trace_exact=measured['trace_sha256']==repeated['trace_sha256'],
                        interaction_exact=sha(output/name/'interactions.jsonl')==sha(output/(name+'-smoke-repeat')/'interactions.jsonl'))
                    smoke_repeats[name]=exact
                    if not all(exact.values()):raise ValueError('Smoke repeat differs; refusing to scale')
        repeats={}
        for name in plan['repeats']:
            original=next(r for r in records if r['name']==name)
            repeated=execute(name+'-repeat',original['case'],kind,original['range_m'])
            repeats[name]=dict(trace_exact=repeated['trace_sha256']==read(output/name/'metrics.json')['trace_sha256'],
                interaction_exact=sha(output/(name+'-repeat')/'interactions.jsonl')==sha(output/name/'interactions.jsonl'))
        result=dict(status='completed',baseline_trace_exact=baseline_exact,repeats=repeats,smoke_repeats=smoke_repeats,
            input_matching=True,executed_runs=len(records),records=records,
            infrastructure_passed=all(all(v.values()) for v in repeats.values()),
            limits=plan['limits'])
        write(output/'verification.json',result)
    except BaseException as exc:
        write(output/'verification.json',dict(status='infrastructure_failed',error=repr(exc),records=records))
        raise
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--config',type=Path)
    verify(**vars(parser.parse_args()))
