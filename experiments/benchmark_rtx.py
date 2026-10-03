"""Serial bounded GPU probes with external, global GPU telemetry and headroom gates."""
import argparse
import json
import os
import subprocess
import sys
import time

from experiments.probe_support import ROOT, GpuMonitor, finish, gpu_sample, run_package, write_json
from traffic.live_runtime import find_isaac


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--single', nargs=2, type=int, metavar=('CARS','SENSORS'))
    args=parser.parse_args()
    cases=[tuple(args.single)] if args.single else [(2,0),(2,1),(2,2),(20,2),(50,2),(20,4),(2,2)]
    source=['scripts/probe_rtx_scaling.py','experiments/benchmark_rtx.py','experiments/probe_support.py','traffic/runtime_metrics.py','traffic/live_runtime.py']
    suite,manifest=run_package('rtx_scaling_suites',{'cases':cases,'frames':240,'headroom_stop_fraction':.9},source)
    results=[]
    for cars,sensors in cases:
        baseline=gpu_sample()
        if baseline['used_mib']/baseline['total_mib']>=.9:
            results.append({'passed':False,'reason':'Preflight global VRAM >=90%; remaining runs not launched'});break
        output,child_manifest=run_package('rtx_scaling',{'cars':cars,'sensors':sensors,'frames':240},source)
        command=[str(find_isaac()),str(ROOT/'scripts/probe_rtx_scaling.py'),'--cars',str(cars),'--sensors',str(sensors),'--output',str(output)]
        child_manifest.update(command=command,gpu_before=baseline)
        reason=None;process=None
        with (output/'runtime.log').open('w') as log, GpuMonitor() as monitor:
            process=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
            started=time.perf_counter()
            while process.poll() is None:
                if time.perf_counter()-started>180:
                    reason='180 second process deadline exceeded'
                elif monitor.rows and monitor.rows[-1]['used_mib']/monitor.rows[-1]['total_mib']>=.9:
                    reason='Global GPU memory >=90%; stopped to preserve headroom'
                if reason:
                    if os.name=='nt':subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],capture_output=True)
                    else:process.kill()
                    process.wait();break
                time.sleep(.2)
        path=output/'probe-result.json'
        result=json.loads(path.read_text()) if path.exists() else {'passed':False,'error':'No result produced'}
        result.update(exit_code=process.returncode,stop_reason=reason,gpu_peak_mib=max([r['used_mib'] for r in monitor.rows],default=None),
            gpu_peak_utilization=max([r['utilization_percent'] for r in monitor.rows],default=None),gpu_scope='whole GPU incl desktop, sampled every ~1s',gpu_monitor_errors=monitor.errors)
        result['passed']=bool(result.get('passed') and not reason and process.returncode==0 and monitor.rows and not monitor.errors)
        write_json(output/'gpu-samples.json',monitor.rows)
        finish(output,child_manifest,result)
        results.append({'path':str(output.relative_to(ROOT)),**result});write_json(suite/'cases.json',results)
        print(json.dumps(results[-1]),flush=True)
        if not result['passed']:break
    # Persist preflight failures too, even when no child was launched.
    write_json(suite/'cases.json',results)
    finish(suite,manifest,{'passed':len(results)==len(cases) and all(r['passed'] for r in results),'completed_cases':len(results),'planned_cases':len(cases)})
    print('RTX_SUITE='+str(suite),flush=True)
    return int(not all(r['passed'] for r in results))


if __name__=='__main__':raise SystemExit(main())
