"""Portable bounded supervisor for the low-speed PhysX vehicle foundation."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

from experiments.probe_support import ROOT, GpuMonitor, finish, gpu_sample, run_package, write_json
from hivemind.launcher import isaac_runtime, load_config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Check configuration without starting Isaac')
    args = parser.parse_args()
    runtime = isaac_runtime(ROOT, load_config(ROOT))
    source = ['experiments/physics-vehicle-smoke.json', 'experiments/verify_physics_vehicle.py',
              'experiments/probe_support.py', 'scripts/probe_physics_vehicle.py',
              'traffic/driver_control.py', 'traffic/physx_vehicle.py', 'traffic/wheel_geometry.py', 'traffic/physics_session.py',
              'hivemind/launcher.py']
    config = json.loads((ROOT/source[0]).read_text())
    if config['repeats'] != 2 or config['physics_hz'] != 120 or config['control_hz'] != 60:
        raise ValueError('This bounded fixture requires two repeats, 120 Hz physics and 60 Hz control')
    if sum(seconds for _,seconds in config['phases']) > 60 or config['max_process_seconds'] > 180:
        raise ValueError('Bounded fixture permits at most 60 simulated seconds and 180 wall seconds')
    if args.check:
        print(json.dumps(dict(runtime=str(runtime), sumo_required=False, config=config), indent=2))
        return 0
    output, manifest = run_package('physics_vehicle', config, source)
    command = [str(runtime), str(ROOT/'scripts/probe_physics_vehicle.py'), '--output', str(output)]
    result = dict(passed=False)
    child = None
    try:
        manifest.update(command=command, isaac_version=(runtime.parent/'VERSION').read_text().strip())
        baseline = gpu_sample()
        manifest['gpu_before'] = baseline
        if baseline['used_mib']/baseline['total_mib'] >= .9:
            raise RuntimeError('Preflight global GPU memory >=90%; close other GPU applications')
        reason = None
        with (output/'runtime.log').open('w', encoding='utf-8') as log, GpuMonitor() as monitor:
            child = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                     creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0)
            start = time.perf_counter()
            while child.poll() is None:
                if time.perf_counter()-start > config['max_process_seconds']:
                    reason = 'Process deadline exceeded'
                elif monitor.rows and monitor.rows[-1]['used_mib']/monitor.rows[-1]['total_mib'] >= .9:
                    reason = 'Global GPU memory >=90%'
                if reason:
                    stop_child(child)
                    break
                time.sleep(.2)
        probe = output/'probe-result.json'
        result = json.loads(probe.read_text()) if probe.exists() else {'passed':False, 'error':'No probe result'}
        result.update(exit_code=child.returncode, stop_reason=reason,
                      gpu_peak_mib=max((r['used_mib'] for r in monitor.rows), default=None),
                      gpu_scope='Whole GPU including desktop; roughly 1 Hz sampling',
                      gpu_monitor_errors=monitor.errors)
        result['passed'] = bool(result['passed'] and child.returncode == 0 and not reason and monitor.rows and not monitor.errors)
        write_json(output/'gpu-samples.json', monitor.rows)
    except Exception as error:
        result.update(passed=False, error=repr(error))
    finally:
        if child is not None and child.poll() is None:
            stop_child(child)
        changed = [rel for rel,digest in manifest['source_hashes'].items()
                   if not (ROOT/rel).is_file() or hashlib.sha256((ROOT/rel).read_bytes()).hexdigest() != digest]
        result['source_changed_during_run'] = changed
        result['passed'] = bool(result.get('passed') and not changed)
        finish(output, manifest, result)
    print('PHYSICS_VEHICLE_RUN='+str(output), flush=True)
    print(json.dumps(result, indent=2), flush=True)
    return 0 if result['passed'] else 1


def stop_child(child):
    if os.name == 'nt':
        subprocess.run(['taskkill', '/PID', str(child.pid), '/T', '/F'], capture_output=True)
    else:
        child.kill()
    child.wait(timeout=15)


if __name__ == '__main__':
    raise SystemExit(main())
