"""Serial fixed real-time validation cases; never hide a crash or failed gate."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from traffic.live_runtime import ROOT
from scripts.audit_realtime_lidar import audit


def main():
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    cases=[('headless-a',60,['--headless']),('headless-b',60,['--headless']),
           ('gui',60,[]),('blocked',25,['--headless','--blocked-lane']),
           ('dropout',25,['--headless','--dropout-step','20'])]
    records=[]
    for index,(name,seconds,extra) in enumerate(cases):
        cmd=[sys.executable,'scripts/demo_live_lidar.py','--mode','avoid','--speed','6','--gap','40',
             '--seconds',str(seconds),'--realtime','--world-motion','NONCOMPENSATED','--sensor-quality','demo',*extra]
        log=ROOT/f'logs/realtime-suite-{stamp}-{name}.log'
        print('START '+name,flush=True)
        with log.open('x') as f:
            process=subprocess.Popen(cmd,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
            try: code=process.wait(timeout=180)
            except subprocess.TimeoutExpired:
                # Only this exact child tree created by the suite is terminated.
                if os.name=='nt': subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],capture_output=True)
                else: process.kill()
                process.wait();code=-1
        record=dict(case=name,exit_code=code,log=str(log.relative_to(ROOT)),passed=False)
        lines=[s.split('LIVE_RESULT=',1)[1] for s in log.read_text(errors='replace').splitlines() if 'LIVE_RESULT=' in s]
        try:
            if not lines: raise RuntimeError('No completed result: crash or timeout; inspect retained log')
            result=json.loads(lines[-1]);record['run']=Path(result['output']).name
            record.update(audit(result['output']))
            record['passed']=record['passed'] and code==0
        except Exception as e: record.update(passed=False,audit_error=repr(e))
        records.append(record)
        with (ROOT/f'outputs/live_lidar/realtime-suite-{stamp}-{index}.json').open('x') as f:
            json.dump(dict(passed=all(r['passed'] for r in records),runs=records),f,indent=2)
        print('DONE '+json.dumps(record),flush=True)
    return 0 if all(r['passed'] for r in records) else 1


if __name__=='__main__': raise SystemExit(main())
