"""Portable, supervised one-car V02 loop fixture. Existing demos are unchanged."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from experiments.highway_loop_config import PROFILES, make_config
from experiments.probe_support import GpuMonitor, finish, gpu_sample, run_package, write_json
from experiments.verify_physics_vehicle import stop_child
from hivemind.launcher import isaac_runtime, load_config


SOURCES=['scripts/demo_highway_loop.py','scripts/physics_highway_loop.py',
    'experiments/highway_loop_config.py','experiments/probe_support.py',
    'experiments/verify_physics_vehicle.py','hivemind/launcher.py',
    'traffic/loop_driving.py','traffic/driver_control.py','traffic/path_following.py',
    'traffic/lane_geometry.py','traffic/speed_profiles.py','traffic/physx_vehicle.py',
    'traffic/wheel_geometry.py','traffic/physics_session.py','traffic/rendered_physics_session.py',
    'traffic/vehicle_contacts.py','traffic/evidence_chunks.py',
    'visualization/highway_loop_view.py','visualization/preview_timing.py',
    'usd/physical_scene.py','usd/highway_loop_scene.py','usd/scene_validation.py',
    'highway_usd/_v02/highway_v02.usda','highway_usd/_v02/navigation.json',
    'highway_usd/_v02/manifest.json','highway_usd/_v02/parameters.json']


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',choices=PROFILES,default='seam3')
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--unpaced',action='store_true')
    parser.add_argument('--camera',choices=['follow','overview'],default='follow')
    parser.add_argument('--capture',action='store_true')
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    config=make_config(args.profile,gui=not args.headless,paced=not args.unpaced,
                       camera=args.camera,capture=args.capture)
    runtime=isaac_runtime(ROOT,load_config(ROOT))
    if args.check:
        print(json.dumps(dict(runtime=str(runtime),config=config,sources=SOURCES),indent=2))
        return 0
    output,manifest=run_package('highway_loop',config,SOURCES)
    command=[str(runtime),str(ROOT/'scripts/physics_highway_loop.py'),'--output',str(output)]
    child=None
    result=dict(passed=False)
    try:
        manifest.update(command=command,isaac_version=(runtime.parent/'VERSION').read_text().strip(),gpu_before=gpu_sample())
        write_json(output/'manifest.json',manifest)
        if manifest['gpu_before']['used_mib']/manifest['gpu_before']['total_mib']>=config['max_gpu_fraction']:
            raise RuntimeError('GPU memory guard: close other GPU applications')
        reason=None; started=time.perf_counter()
        print('HIGHWAY_LOOP_RUN='+str(output),flush=True)
        with (output/'runtime.log').open('w',encoding='utf-8') as log, GpuMonitor() as monitor:
            child=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)
            while child.poll() is None:
                if time.perf_counter()-started>config['max_process_seconds']:
                    reason='Process deadline exceeded'
                elif monitor.rows and monitor.rows[-1]['used_mib']/monitor.rows[-1]['total_mib']>=config['max_gpu_fraction']:
                    reason='GPU memory guard'
                if reason:
                    stop_child(child);break
                time.sleep(.2)
        probe=output/'probe-result.json'
        result=json.loads(probe.read_text()) if probe.exists() else dict(passed=False,error='No probe result')
        log=(output/'runtime.log').read_text(encoding='utf-8',errors='replace')
        result.update(exit_code=child.returncode,stop_reason=reason,
            usd_reference_warning_count=log.count('Unexpected reference count'),
            runtime_error_lines=[line for line in log.splitlines() if '[Error]' in line],
            wall_process_s=time.perf_counter()-started,
            gpu_peak_mib=max((r['used_mib'] for r in monitor.rows),default=None),
            gpu_scope='Whole GPU including desktop; approximately 1 Hz',gpu_monitor_errors=monitor.errors)
        result['passed']=bool(result['passed'] and child.returncode==0 and not reason
            and not result['usd_reference_warning_count'] and not result['runtime_error_lines']
            and monitor.rows and not monitor.errors)
        write_json(output/'gpu-samples.json',monitor.rows)
    except Exception as error:
        result.update(passed=False,error=repr(error))
    finally:
        if child and child.poll() is None: stop_child(child)
        changed=[p for p,h in manifest['source_hashes'].items()
                 if not (ROOT/p).is_file() or hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h]
        result['source_changed_during_run']=changed
        result['passed']=bool(result.get('passed') and not changed)
        finish(output,manifest,result)
    compact={key:result.get(key) for key in ('passed','gates','distance_during_drive_m',
        'completed_laps','lane_rms_m','max_lane_error_m','braking_distance_m',
        'preview_timing','error','exit_code','runtime_error_lines','source_changed_during_run')}
    print(json.dumps(dict(output=str(output),**compact),indent=2),flush=True)
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
