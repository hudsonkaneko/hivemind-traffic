"""Re-audit a completed suite and compare repeated sensor-control trajectories."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scripts.analyze_live_lidar import evaluate


def report(suite):
    records=json.loads(suite.read_text())['runs']
    audited=[evaluate(suite.parent/r['run']) for r in records]
    following=[r for r in audited if r['config']['mode']=='follow']
    if len(following)!=2:raise ValueError('Expected two following runs')
    traces=[json.loads((suite.parent/r['run']/'telemetry.json').read_text()) for r in following]
    if len(traces[0])!=len(traces[1]):raise ValueError('Repeat length mismatch')
    differences={key:max(abs(a[key]-b[key]) for a,b in zip(*traces))
                 for key in ['ego_x','speed','lidar_distance','post_gap']}
    tolerance=.001
    repeat_pass=all(d<=tolerance for d in differences.values())
    faults=[r for r in audited if r['config']['fault_step']>=0]
    fault_pass=len(faults)==1 and faults[0]['summary']['sensor_fault_actions']==1
    return dict(passed=all(r['summary']['passed'] for r in audited) and repeat_pass and fault_pass,
                runs=audited,repeat_tolerance=tolerance,repeat_max_absolute_differences=differences,
                repeat_passed=repeat_pass,fault_injection_passed=fault_pass)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('suite',type=Path);p.add_argument('--output',type=Path)
    args=p.parse_args();result=report(args.suite);text=json.dumps(result,indent=2)
    if args.output:
        with args.output.open('x') as f:f.write(text+'\n')
    print(text)
    raise SystemExit(0 if result['passed'] else 1)
