"""Serial GPU validation matrix. Retains failures; never overwrites previous runs."""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import subprocess
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from traffic.live_runtime import ROOT,find_isaac
from scripts.analyze_live_lidar import evaluate


def main():
    p=argparse.ArgumentParser();p.add_argument('--gui-last',action='store_true')
    p.add_argument('--diagnostics',action='store_true',help='Repeat default stop and compare two ideal-sensor following runs')
    args=p.parse_args()
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    runtime=find_isaac()
    (ROOT/"logs").mkdir(exist_ok=True)
    cases=[('stop',4,20,42,-1),('stop',8,40,42,-1),('stop',12,65,42,-1),
           ('follow',6,30,42,-1),('follow',6,30,42,-1),('stop',8,40,43,-1),
           ('stop',8,40,42,40)]
    if args.diagnostics: cases=[('stop',8,40,42,-1),('follow',6,30,42,-1),('follow',6,30,42,-1)]
    report=[]
    for i,(mode,speed,gap,seed,fault) in enumerate(cases):
        log=ROOT/f'logs/live-suite-{stamp}-{i}.log'
        cmd=[str(runtime),'scripts/live_lidar_drive.py','--mode',mode,'--speed',str(speed),
             '--gap',str(gap),'--seed',str(seed),'--fault-step',str(fault)]
        if args.gui_last and i==len(cases)-1:cmd.append('--gui')
        if args.diagnostics and mode=='follow':cmd.append('--ideal-sensor')
        print(f'START {i+1}/{len(cases)} {mode} speed={speed} gap={gap} seed={seed} fault={fault}',flush=True)
        with log.open('x') as f: result=subprocess.run(cmd,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,timeout=600)
        lines=[s.split('LIVE_RESULT=',1)[1] for s in log.read_text(errors='replace').splitlines() if 'LIVE_RESULT=' in s]
        if not lines: raise RuntimeError(f'No run result: {log}')
        path=Path(json.loads(lines[-1])['output'])
        verified=evaluate(path);verified['exit_code']=result.returncode
        report.append(verified)
        print('DONE '+json.dumps(verified),flush=True)
    dest=ROOT/f'outputs/live_lidar/suite-{stamp}.json'
    with dest.open('x') as f:json.dump({'runs':report,'diagnostics':args.diagnostics},f,indent=2)
    print('SUITE_REPORT='+str(dest),flush=True)
    return 0 if all(r['summary']['passed'] and r['exit_code']==0 for r in report) else 1


if __name__=='__main__':raise SystemExit(main())
