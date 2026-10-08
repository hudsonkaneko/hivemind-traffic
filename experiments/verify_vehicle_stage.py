"""Bounded, source-captured vehicle reset/dynamics/Lab verification; no training."""
import argparse
import hashlib
import json
import os
import subprocess
import time

from experiments.probe_support import ROOT, GpuMonitor, finish, gpu_sample, run_package, write_json
from experiments.verify_physics_vehicle import stop_child
from hivemind.launcher import isaac_runtime, load_config


STAGES = {
    'resets': ('physics-resets.json', 'probe_vehicle_resets.py'),
    'reset-control': ('physics-reset-control.json', 'probe_vehicle_resets.py'),
    'dynamics': ('physics-dynamics.json', 'probe_vehicle_dynamics.py'),
    'lab': ('physics-lab.json', 'probe_vehicle_lab.py'),
    'lane-following': ('physics-lane-following.json', 'probe_lane_following.py'),
}


def source_files(stage):
    config, script = STAGES[stage]
    files = [f'experiments/{config}', f'scripts/{script}',
             'experiments/verify_vehicle_stage.py', 'experiments/probe_support.py',
             'experiments/verify_physics_vehicle.py', 'hivemind/launcher.py',
             'traffic/physx_vehicle.py', 'traffic/wheel_geometry.py', 'traffic/driver_control.py', 'traffic/physics_session.py']
    if stage == 'dynamics':
        files.append('traffic/dynamics_validation.py')
    if stage == 'lab':
        files.extend(['environments/__init__.py', 'environments/physics_vehicle_lab.py',
                      'environments/physics_vehicle_contract.py'])
    if stage == 'lane-following':
        files.extend(['traffic/lane_geometry.py', 'traffic/path_following.py', 'traffic/lane_validation.py',
                      'traffic/speed_profiles.py',
                      'traffic/vehicle_contacts.py', 'traffic/dynamics_validation.py',
                      'scenarios/physics-road/routes.json'])
    return files


def validate_bounds(config):
    if config.get('physics_hz') != 120 or config.get('control_hz') != 60:
        raise ValueError('This fixture requires 120 Hz physics and 60 Hz control')
    if not 1 <= config.get('repeats', 0) <= 10:
        raise ValueError('Fixture requires 1–10 repetitions')
    deadline = config.get('max_process_seconds', 0)
    if isinstance(deadline, bool) or not isinstance(deadline, (float, int)) or not 0 < deadline <= 600:
        raise ValueError('Fixture process deadline must be in (0,600] seconds')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', choices=STAGES, required=True)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    runtime = isaac_runtime(ROOT, load_config(ROOT))
    sources = source_files(args.stage)
    missing = [p for p in sources if not (ROOT/p).is_file()]
    if missing:
        raise FileNotFoundError(missing)
    config = json.loads((ROOT/sources[0]).read_text(encoding='utf-8'))
    validate_bounds(config)
    if args.stage == 'dynamics':
        from traffic.dynamics_validation import validate_config
    elif args.stage == 'lab':
        from environments.physics_vehicle_contract import validate_config
    elif args.stage == 'lane-following':
        from traffic.lane_validation import validate_config
    else:
        from scripts.probe_vehicle_resets import validate_config
    validate_config(config)
    if args.check:
        print(json.dumps(dict(stage=args.stage, runtime=str(runtime), sumo_required=False,
                              config=config, sources=sources), indent=2))
        return 0
    output, manifest = run_package('vehicle_'+args.stage.replace('-', '_'), config, sources)
    command = [str(runtime), str(ROOT/sources[1]), '--output', str(output)]
    result = {'passed': False}
    child = None
    try:
        manifest.update(command=command, isaac_version=(runtime.parent/'VERSION').read_text().strip())
        manifest['gpu_before'] = gpu_sample()
        if manifest['gpu_before']['used_mib']/manifest['gpu_before']['total_mib'] >= .9:
            raise RuntimeError('Preflight global GPU memory >=90%; close other GPU applications')
        write_json(output/'manifest.json', manifest)
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
        result = json.loads(probe.read_text()) if probe.exists() else {'passed': False, 'error': 'No probe result'}
        log_text = (output/'runtime.log').read_text(encoding='utf-8', errors='replace')
        warning_count = log_text.count('Unexpected reference count')
        result.update(exit_code=child.returncode, stop_reason=reason,
                      usd_reference_warning_count=warning_count,
                      runtime_error_lines=[line for line in log_text.splitlines() if '[Error]' in line],
                      wall_process_seconds=time.perf_counter()-start,
                      gpu_peak_mib=max((r['used_mib'] for r in monitor.rows), default=None),
                      gpu_scope='Whole GPU including desktop; roughly 1 Hz sampling',
                      gpu_monitor_errors=monitor.errors)
        if args.stage != 'reset-control' and warning_count:
            result['lifecycle_warning_gate_failed'] = True
        result['passed'] = bool(result.get('passed') and child.returncode == 0 and not reason
                                and monitor.rows and not monitor.errors
                                and not result.get('lifecycle_warning_gate_failed'))
        write_json(output/'gpu-samples.json', monitor.rows)
    except Exception as error:
        result.update(passed=False, error=repr(error))
    finally:
        if child is not None and child.poll() is None:
            stop_child(child)
        changed = [rel for rel, digest in manifest['source_hashes'].items()
                   if not (ROOT/rel).is_file() or hashlib.sha256((ROOT/rel).read_bytes()).hexdigest() != digest]
        result['source_changed_during_run'] = changed
        result['passed'] = bool(result.get('passed') and not changed)
        finish(output, manifest, result)
    print('VEHICLE_STAGE_RUN='+str(output), flush=True)
    print(json.dumps(result, indent=2), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
