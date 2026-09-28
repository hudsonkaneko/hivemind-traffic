"""Serial GPU cases for static obstacle avoidance; source/evidence retained per run."""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import subprocess
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from traffic.live_runtime import ROOT
from scripts.audit_lidar_avoidance import audit


def main():
    p=argparse.ArgumentParser();p.add_argument('--gui-last',action='store_true');args=p.parse_args()
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    cases=[(6,40,42,False),(6,40,42,False),(8,55,43,False),(6,40,42,True)]
    records=[]
    for i,(speed,gap,seed,blocked) in enumerate(cases):
        cmd=[sys.executable,'scripts/demo_live_lidar.py','--mode','avoid','--speed',str(speed),'--gap',str(gap),'--seed',str(seed)]
        if not (args.gui_last and i==len(cases)-1):cmd.append('--headless')
        if blocked:cmd.append('--blocked-lane')
        log=ROOT/f'logs/avoid-suite-{stamp}-{i}.log'
        print(f'START {i+1}/{len(cases)} speed={speed} gap={gap} blocked={blocked}',flush=True)
        with log.open('x') as f:completed=subprocess.run(cmd,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,timeout=600)
        lines=[line.split('LIVE_RESULT=',1)[1] for line in log.read_text(errors='replace').splitlines() if 'LIVE_RESULT=' in line]
        if not lines:
            result=dict(verified=False,summary=dict(passed=False),error='Process ended without LIVE_RESULT',
                        log=str(log.relative_to(ROOT)),exit_code=completed.returncode)
        else:
            try:
                result=audit(Path(json.loads(lines[-1])['output']))
            except Exception as error:
                result=dict(verified=False,summary=dict(passed=False),error=str(error),log=str(log.relative_to(ROOT)))
            result['exit_code']=completed.returncode
        records.append(result);print('DONE '+json.dumps(result),flush=True)
        checkpoint=ROOT/f'outputs/live_lidar/avoid-suite-{stamp}-partial-{i}.json'
        with checkpoint.open('x') as f:json.dump(dict(runs=records),f,indent=2)
    differences={};repeated=False
    if all(r.get('verified') and r['summary']['passed'] for r in records[:2]):
        traces=[json.loads((ROOT/'outputs/live_lidar'/r['run']/'telemetry.json').read_text()) for r in records[:2]]
        differences={k:max(abs(a['ego_after'][k]-b['ego_after'][k]) for a,b in zip(*traces)) for k in ('x','y','speed')}
        repeated=len(traces[0])==len(traces[1]) and all(v<=.01 for v in differences.values()) and all(a['phase']==b['phase'] and a['lane_request']==b['lane_request'] for a,b in zip(*traces))
    report=dict(passed=all(r['summary']['passed'] and r['exit_code']==0 for r in records) and repeated,
                runs=records,repeat_max_differences=differences,repeat_passed=repeated,repeat_tolerance=.01)
    dest=ROOT/f'outputs/live_lidar/avoid-suite-{stamp}.json'
    with dest.open('x') as f:json.dump(report,f,indent=2)
    print('AVOID_SUITE='+str(dest),flush=True)
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
