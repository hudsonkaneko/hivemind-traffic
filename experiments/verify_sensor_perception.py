"""Bounded native camera/LiDAR validation; never launches policy training."""
import argparse
import hashlib
import json
import os
import subprocess
from experiments.probe_support import ROOT, finish, run_package
from experiments.verify_physics_vehicle import stop_child
from hivemind.launcher import isaac_runtime, load_config

CONFIG = dict(version='sensor-perception-v01', seed=17, physics_hz=120,
              camera_hz=10, lidar_hz=20, camera_resolution=[1280,720],
              duration_s=15, target_speed_m_s=2, max_process_seconds=600,
              barrier_x_m=14,
              source_vehicle='vehicles/sim_ready/americano-i7-ev/v08',
              training=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    runtime = isaac_runtime(ROOT, load_config(ROOT))
    if args.check:
        print(json.dumps(dict(runtime=str(runtime), **CONFIG), indent=2))
        return 0
    if os.name == 'nt':
        active = subprocess.check_output(['powershell', '-NoProfile', '-Command',
            "Get-Process | Where-Object ProcessName -eq 'kit' | Select-Object -ExpandProperty Id"],
            text=True).strip()
        if active:
            raise RuntimeError('Another Kit process is active; defer native validation until it exits')
    sources = ['experiments/verify_sensor_perception.py',
               'scripts/probe_sensor_perception.py', 'traffic/sensor_pipeline.py',
               'traffic/native_sensor_capture.py', 'traffic/physical_lidar.py',
               'traffic/rendered_physics_session.py', 'usd/vehicle_sensor_rig.py',
               'usd/showcase_vehicle.py', 'traffic/lidar_tracking.py',
               'traffic/driver_control.py', 'traffic/physx_vehicle.py',
               'traffic/physics_session.py', 'traffic/vehicle_contacts.py',
               'traffic/lane_geometry.py', 'traffic/bypass_validation.py',
               'usd/physical_scene.py', 'visualization/physics_road_view.py',
               'experiments/probe_support.py', 'hivemind/launcher.py']
    output, manifest = run_package('sensor-perception/v01', CONFIG, sources)
    log_path = ROOT/'logs'/'sensor-perception'/f'{output.name}.log'
    log_path.parent.mkdir(parents=True, exist_ok=True)
    command = [str(runtime), str(ROOT/'scripts/probe_sensor_perception.py'),
               '--output', str(output)]
    manifest.update(command=command, runtime_log=str(log_path.relative_to(ROOT)))
    child = None
    result = dict(passed=False)
    try:
        with log_path.open('w', encoding='utf-8') as log:
            child = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                     creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0)
            child.wait(timeout=CONFIG['max_process_seconds'])
        report = output/'probe-result.json'
        result = json.loads(report.read_text()) if report.exists() else dict(passed=False, error='No native result')
        result['exit_code'] = child.returncode
        result['passed'] = bool(result.get('passed') and child.returncode == 0)
    except Exception as error:
        result.update(passed=False, error=repr(error))
    finally:
        if child is not None and child.poll() is None:
            stop_child(child)
        changed = [rel for rel, digest in manifest['source_hashes'].items()
                   if hashlib.sha256((ROOT/rel).read_bytes()).hexdigest() != digest]
        result.update(source_changed_during_run=changed, runtime_log=str(log_path.relative_to(ROOT)))
        result['passed'] = bool(result.get('passed') and not changed)
        finish(output, manifest, result)
    print('SENSOR_PERCEPTION_RUN='+str(output), flush=True)
    print(json.dumps(result, indent=2), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
