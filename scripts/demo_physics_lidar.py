"""Portable supervised visual physics-car demo; no SUMO or training required."""
import argparse
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from experiments.probe_support import GpuMonitor, finish, gpu_sample, run_package, write_json
from experiments.verify_physics_vehicle import stop_child
from hivemind.launcher import isaac_runtime, load_config
from traffic.lidar_braking import LidarBrakeConfig
from traffic.visual_lidar_validation import validate_config


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--check',action='store_true')
    parser.add_argument('--mode',choices=['smoke','obstacle','dropout'],default='obstacle')
    parser.add_argument('--camera',choices=['overview','follow'],default='follow')
    parser.add_argument('--points',action='store_true')
    parser.add_argument('--unpaced',action='store_true')
    parser.add_argument('--capture',action='store_true')
    parser.add_argument('--seconds',type=int)
    args=parser.parse_args()
    config=json.loads((ROOT/'experiments/physics-visual-lidar.json').read_text())
    config.update(gui=not args.headless,mode=args.mode,camera=args.camera,points=args.points,
                  paced=not args.unpaced,capture=args.capture,dropout_at_s=12)
    if args.seconds is not None:config['duration_s']=args.seconds
    elif args.mode=='smoke':config['duration_s']=8
    config['braking']=asdict(LidarBrakeConfig(min_height_m=-.6,max_height_m=.6))
    config['follower']=json.loads((ROOT/config['driver_config']).read_text())['follower']
    validate_config(config)
    runtime=isaac_runtime(ROOT,load_config(ROOT))
    sources=['scripts/demo_physics_lidar.py','scripts/physics_lidar_drive.py','experiments/physics-visual-lidar.json',
        config['route_file'],config['driver_config'],'traffic/lane_geometry.py','traffic/path_following.py',
        'traffic/driver_control.py','traffic/physx_vehicle.py','traffic/physics_session.py','traffic/rendered_physics_session.py',
        'traffic/lidar_braking.py','traffic/physical_lidar.py','traffic/vehicle_contacts.py','traffic/lane_validation.py',
        'traffic/speed_profiles.py',
        'traffic/visual_lidar_validation.py','traffic/wheel_geometry.py','visualization/physics_road_view.py',
        'experiments/probe_support.py','experiments/verify_physics_vehicle.py','hivemind/launcher.py']
    if args.check:
        print(json.dumps(dict(runtime=str(runtime),config=config,sources=sources,sumo_required=False,no_training=True),indent=2))
        return 0
    output,manifest=run_package('vehicle_visual_lidar',config,sources)
    command=[str(runtime),str(ROOT/'scripts/physics_lidar_drive.py'),'--output',str(output)]
    child=None;result={'passed':False}
    try:
        manifest.update(command=command,isaac_version=(runtime.parent/'VERSION').read_text().strip(),gpu_before=gpu_sample())
        write_json(output/'manifest.json',manifest)
        if manifest['gpu_before']['used_mib']/manifest['gpu_before']['total_mib']>=config['max_gpu_fraction']:
            raise RuntimeError('GPU memory guard: close other GPU applications')
        reason=None;started=time.perf_counter()
        print('PHYSICS_LIDAR_RUN='+str(output),flush=True)
        with (output/'runtime.log').open('w',encoding='utf-8') as log,GpuMonitor() as monitor:
            child=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)
            while child.poll() is None:
                if time.perf_counter()-started>config['max_process_seconds']:reason='Process deadline exceeded'
                elif monitor.rows and monitor.rows[-1]['used_mib']/monitor.rows[-1]['total_mib']>=config['max_gpu_fraction']:reason='GPU memory guard'
                if reason:stop_child(child);break
                time.sleep(.2)
        probe=output/'probe-result.json'
        result=json.loads(probe.read_text()) if probe.exists() else dict(passed=False,error='No probe result')
        log=(output/'runtime.log').read_text(encoding='utf-8',errors='replace')
        result['functional_passed']=bool(result.get('passed'))
        warnings=log.count('Unexpected reference count')
        result.update(exit_code=child.returncode,stop_reason=reason,usd_reference_warning_count=warnings,
            runtime_error_lines=[line for line in log.splitlines() if '[Error]' in line],
            wall_process_s=time.perf_counter()-started,gpu_peak_mib=max((r['used_mib'] for r in monitor.rows),default=None),
            gpu_scope='Whole GPU, including desktop; roughly1Hz sampling',gpu_monitor_errors=monitor.errors)
        result['passed']=bool(result.get('passed') and child.returncode==0 and not reason and not warnings
                              and monitor.rows and not monitor.errors and not result['runtime_error_lines'])
        write_json(output/'gpu-samples.json',monitor.rows)
    except Exception as error:
        result.update(passed=False,error=repr(error))
    finally:
        if child and child.poll() is None:stop_child(child)
        changed=[p for p,h in manifest['source_hashes'].items() if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h]
        result['source_changed_during_run']=changed
        result['passed']=bool(result.get('passed') and not changed)
        finish(output,manifest,result)
    print(json.dumps(result,indent=2),flush=True)
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
