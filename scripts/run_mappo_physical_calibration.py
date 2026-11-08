"""Supervise bounded frozen-MAPPO/PhysX calibration; never starts a trainer."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from experiments.probe_support import GpuMonitor, finish, gpu_sample, run_package, write_json
from experiments.verify_physics_vehicle import stop_child
from hivemind.launcher import isaac_runtime, load_config

SOURCES = [
    'scripts/run_mappo_physical_calibration.py', 'scripts/calibrate_mappo_physical.py',
    'experiments/configs/mappo_physical_adapter_v01.json',
    'traffic/mappo_physical_adapter.py', 'traffic/mappo_physical_policy.py',
    'traffic/driver_control.py', 'traffic/path_following.py', 'traffic/lane_geometry.py',
    'traffic/physx_vehicle.py', 'traffic/wheel_geometry.py', 'traffic/physics_session.py',
    'traffic/rendered_physics_session.py', 'traffic/vehicle_contacts.py',
    'usd/physical_scene.py', 'usd/scene_validation.py',
    'experiments/probe_support.py', 'experiments/verify_physics_vehicle.py',
    'hivemind/launcher.py', 'visualization/preview_timing.py',
    'visualization/physics_road_view.py',
]


def competing_native_jobs():
    """Read-only workstation check before owning the serial native runtime."""
    if os.name != 'nt':
        raise RuntimeError('This native runtime supervisor currently supports Windows only')
    query = (
        "@(Get-CimInstance Win32_Process | Where-Object { "
        "$_.Name -eq 'kit.exe' -or ($_.Name -match '^python(w)?\\.exe$' -and "
        "$_.CommandLine -match 'isaacsim|IsaacLab|calibrate_mappo_physical|ARACHNID.*train') "
        "} | Select-Object ProcessId,Name,CommandLine) | ConvertTo-Json -Compress"
    )
    completed = subprocess.run(['powershell', '-NoProfile', '-Command', query],
                               capture_output=True, text=True, check=True, timeout=15,
                               creationflags=subprocess.CREATE_NO_WINDOW)
    rows = json.loads(completed.stdout.strip() or '[]')
    return rows if isinstance(rows, list) else [rows]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path,
                        default=ROOT/'experiments/configs/mappo_physical_adapter_v01.json')
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    if not Path.cwd().resolve().is_relative_to(ROOT):
        raise RuntimeError('Run from the canonical checkout or its children')
    config_path = args.config.resolve()
    if not config_path.is_relative_to(ROOT):
        raise ValueError('Configuration must be inside the checkout')
    cfg = json.loads(config_path.read_text(encoding='utf-8'))
    from scripts.calibrate_mappo_physical import validate_config
    validate_config(cfg)
    runtime = isaac_runtime(ROOT, load_config(ROOT))
    if args.check:
        print(json.dumps(dict(runtime=str(runtime), config=cfg, no_training=True), indent=2))
        return 0
    jobs = competing_native_jobs()
    if jobs:
        raise RuntimeError('Native simulator already occupied: '+json.dumps(jobs))
    sources = list(dict.fromkeys([*SOURCES, str(config_path.relative_to(ROOT)).replace('\\', '/')]))
    output, manifest = run_package('mappo-physical-adapter/v01', cfg, sources)
    log_dir = ROOT/'logs/mappo-physical-adapter'
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir/(output.name+'.log')
    command = [str(runtime), str(ROOT/'scripts/calibrate_mappo_physical.py'), '--output', str(output)]
    result = dict(passed=False, no_training=True)
    child = None
    started = time.perf_counter()
    try:
        manifest.update(command=command, runtime_log=str(log_path.relative_to(ROOT)),
                        isaac_version=(runtime.parent/'VERSION').read_text().strip(),
                        gpu_before=gpu_sample(), competing_jobs_before=jobs)
        write_json(output/'manifest.json', manifest)
        max_gpu_fraction = cfg.get('max_gpu_fraction', .9)
        if manifest['gpu_before']['used_mib']/manifest['gpu_before']['total_mib'] >= max_gpu_fraction:
            raise RuntimeError('GPU memory already exceeds the experiment limit')
        print('MAPPO_PHYSICAL_RUN='+str(output), flush=True)
        reason = None
        with log_path.open('w', encoding='utf-8') as log, GpuMonitor() as monitor:
            child = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                     creationflags=subprocess.CREATE_NO_WINDOW)
            while child.poll() is None:
                if time.perf_counter()-started > cfg['max_process_seconds']:
                    reason = 'Process deadline exceeded'
                elif monitor.rows and monitor.rows[-1]['used_mib']/monitor.rows[-1]['total_mib'] >= max_gpu_fraction:
                    reason = 'GPU memory limit exceeded'
                if reason:
                    stop_child(child)
                    break
                time.sleep(.25)
        probe = output/'probe-result.json'
        result = json.loads(probe.read_text()) if probe.exists() else dict(passed=False, error='No native result')
        log_text = log_path.read_text(encoding='utf-8', errors='replace')
        result.update(exit_code=child.returncode, stop_reason=reason,
                      runtime_log=str(log_path.relative_to(ROOT)),
                      runtime_error_lines=[line for line in log_text.splitlines() if '[Error]' in line],
                      usd_reference_warning_count=log_text.count('Unexpected reference count'),
                      wall_process_s=time.perf_counter()-started,
                      gpu_peak_mib=max((r['used_mib'] for r in monitor.rows), default=None),
                      gpu_scope='Whole GPU including desktop, approximately 1 Hz',
                      gpu_monitor_errors=monitor.errors)
        result['passed'] = bool(result.get('passed') and child.returncode == 0 and not reason
                                and not result['runtime_error_lines']
                                and not result['usd_reference_warning_count']
                                and monitor.rows and not monitor.errors)
        write_json(output/'gpu-samples.json', monitor.rows)
    except Exception as error:
        result.update(passed=False, error=repr(error))
    finally:
        if child and child.poll() is None:
            stop_child(child)
        changed = [p for p, h in manifest['source_hashes'].items()
                   if not (ROOT/p).is_file() or hashlib.sha256((ROOT/p).read_bytes()).hexdigest() != h]
        result['source_changed_during_run'] = changed
        result['passed'] = bool(result.get('passed') and not changed)
        finish(output, manifest, result)
    print(json.dumps(dict(output=str(output), passed=result['passed'], gates=result.get('gates'),
                          error=result.get('error'), runtime_log=str(log_path)), indent=2), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
