"""Retained-evidence checks for the post-GC stationary sensor refresh."""
import argparse
import json
import os
import subprocess

from experiments.probe_support import ROOT, finish, run_package, write_json
from traffic.live_runtime import find_isaac
from scripts.audit_cooperative_lidar import audit


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--long',action='store_true')
    args=parser.parse_args()
    durations=[25,25]+([120] if args.long else [])
    source=['experiments/verify_cooperative_refresh.py','experiments/probe_support.py',
            'scripts/cooperative_lidar_drive.py','traffic/runtime_metrics.py']
    output,manifest=run_package('refresh_verification',{'durations':durations,'seed':42,'comm':'ideal'},source)
    cases=[]
    for index,seconds in enumerate(durations):
        before=set((ROOT/'outputs/cooperative_lidar').iterdir())
        command=[str(find_isaac()),str(ROOT/'scripts/cooperative_lidar_drive.py'),'--seconds',str(seconds),'--seed','42','--comm','ideal']
        timed_out=False
        with (output/f'run-{index}.log').open('w') as stream:
            process=subprocess.Popen(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT)
            try:code=process.wait(timeout=210)
            except subprocess.TimeoutExpired:
                timed_out=True
                if os.name=='nt':subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],capture_output=True)
                else:process.kill()
                code=process.wait()
        created=[p for p in set((ROOT/'outputs/cooperative_lidar').iterdir())-before if p.is_dir()]
        case=dict(seconds=seconds,exit_code=code,timed_out=timed_out,command=command,passed=False)
        if len(created)==1:
            run=created[0];case['path']=str(run.relative_to(ROOT))
            case['audit']=audit(run)
            if (run/'telemetry.json').is_file():
                rows=json.loads((run/'telemetry.json').read_text())
                if rows:
                    case['first_fresh']={vid:o['fresh'] for vid,o in rows[0]['observations'].items()}
                    case['first_receipt_age_s']={vid:o['receipt_age'] for vid,o in rows[0]['observations'].items()}
            if (run/'summary.json').is_file():case['summary']=json.loads((run/'summary.json').read_text())
            case['passed']=bool(case['audit'].get('passed') and case.get('first_fresh') and all(case['first_fresh'].values()) and code==0)
        cases.append(case);write_json(output/'cases.json',cases)
        print(json.dumps({k:v for k,v in case.items() if k not in ('summary','command')}),flush=True)
    finish(output,manifest,{'passed':all(c['passed'] for c in cases),'cases':len(cases),
        'scope':'Startup freshness and bounded behavior; native sensor exact repeatability not guaranteed'})
    print('REFRESH_SUITE='+str(output),flush=True)
    return int(not all(c['passed'] for c in cases))


if __name__=='__main__':raise SystemExit(main())
