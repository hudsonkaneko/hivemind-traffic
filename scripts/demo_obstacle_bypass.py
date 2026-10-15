"""Portable, bounded LiDAR-triggered physical obstacle bypass. No training."""
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
from experiments.obstacle_bypass_config import validate_config, controller_settings
from hivemind.launcher import isaac_runtime, load_config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--profile', choices=['low-speed', '35mph'], default='35mph',
                        help='Physical driving fixture; low-speed preserves the original 3 m/s demo')
    parser.add_argument('--mode', choices=['pass', 'blocked', 'dropout'], default='pass')
    parser.add_argument('--camera', choices=['follow', 'overview'], default='follow')
    parser.add_argument('--points', action='store_true')
    parser.add_argument('--capture', action='store_true')
    parser.add_argument('--unpaced', action='store_true')
    parser.add_argument('--real-time', action='store_true',
                        help='Lighter graphics and best-effort 1x pacing; measured lag can still fail the timing gate')
    args = parser.parse_args()
    if args.real_time and args.unpaced:
        parser.error('--real-time requires pacing; omit --unpaced')
    config_path = ('experiments/physics-obstacle-bypass-35mph.json' if args.profile == '35mph'
                   else 'experiments/physics-obstacle-bypass.json')
    config = json.loads((ROOT/config_path).read_text())
    validate_config(config)
    config.update(mode=args.mode, gui=not args.headless, camera=args.camera,
                  points=args.points, capture=args.capture, paced=not args.unpaced,
                  real_time=args.real_time,
                  **controller_settings(args.profile))
    if args.real_time:
        config['render_hz'] = 20
    validate_config(config, resolved=True)
    runtime = isaac_runtime(ROOT, load_config(ROOT))
    sources = ['scripts/demo_obstacle_bypass.py', 'scripts/physics_obstacle_bypass.py',
        config_path, 'experiments/probe_support.py', 'traffic/speed_profiles.py',
        'experiments/verify_physics_vehicle.py', 'hivemind/launcher.py',
        'experiments/obstacle_bypass_config.py',
        'traffic/obstacle_bypass.py', 'traffic/bypass_validation.py',
        'traffic/driver_control.py', 'traffic/path_following.py', 'traffic/lane_geometry.py',
        'traffic/physx_vehicle.py', 'traffic/wheel_geometry.py', 'traffic/physics_session.py',
        'traffic/rendered_physics_session.py', 'traffic/physical_lidar.py', 'traffic/lidar_braking.py',
        'traffic/runtime_profile.py', 'traffic/evidence_chunks.py',
        'traffic/vehicle_contacts.py', 'visualization/physics_road_view.py',
        'visualization/preview_timing.py', 'usd/physical_scene.py', 'usd/scene_validation.py']
    if args.check:
        print(json.dumps(dict(runtime=str(runtime), config=config, sources=sources, no_training=True), indent=2))
        return 0
    output, manifest = run_package('vehicle_obstacle_bypass', config, sources)
    command = [str(runtime), str(ROOT/'scripts/physics_obstacle_bypass.py'), '--output', str(output)]
    child = None
    result = dict(passed=False)
    try:
        manifest.update(command=command, isaac_version=(runtime.parent/'VERSION').read_text().strip(), gpu_before=gpu_sample())
        write_json(output/'manifest.json', manifest)
        if manifest['gpu_before']['used_mib']/manifest['gpu_before']['total_mib'] >= config['max_gpu_fraction']:
            raise RuntimeError('GPU memory guard: close other GPU applications')
        reason, started = None, time.perf_counter()
        print('OBSTACLE_BYPASS_RUN='+str(output), flush=True)
        with (output/'runtime.log').open('w', encoding='utf-8') as log, GpuMonitor() as monitor:
            child = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)
            while child.poll() is None:
                if time.perf_counter()-started > config['max_process_seconds']:
                    reason = 'Process deadline exceeded'
                elif monitor.rows and monitor.rows[-1]['used_mib']/monitor.rows[-1]['total_mib'] >= config['max_gpu_fraction']:
                    reason = 'GPU memory guard'
                if reason:
                    stop_child(child)
                    break
                time.sleep(.2)
        probe = output/'probe-result.json'
        result = json.loads(probe.read_text()) if probe.exists() else dict(passed=False,error='No probe result')
        log = (output/'runtime.log').read_text(encoding='utf-8', errors='replace')
        result.update(exit_code=child.returncode, stop_reason=reason,
            usd_reference_warning_count=log.count('Unexpected reference count'),
            runtime_error_lines=[line for line in log.splitlines() if '[Error]' in line],
            wall_process_s=time.perf_counter()-started,
            gpu_peak_mib=max((r['used_mib'] for r in monitor.rows), default=None),
            gpu_scope='Whole GPU including desktop, sampled approximately 1 Hz',
            gpu_monitor_errors=monitor.errors)
        result['passed'] = bool(result['passed'] and child.returncode==0 and not reason
            and not result['usd_reference_warning_count'] and not result['runtime_error_lines']
            and monitor.rows and not monitor.errors)
        write_json(output/'gpu-samples.json', monitor.rows)
    except Exception as error:
        result.update(passed=False, error=repr(error))
    finally:
        if child and child.poll() is None:
            stop_child(child)
        changed = [p for p,h in manifest['source_hashes'].items()
                   if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h]
        result['source_changed_during_run'] = changed
        result['passed'] = bool(result.get('passed') and not changed)
        finish(output, manifest, result)
    print(json.dumps(result, indent=2), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
