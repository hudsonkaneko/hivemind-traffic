"""Portable, supervised loop-traffic showcase; defaults to a paced live preview."""
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
from experiments.loop_showcase_config import MODES,VERSIONS,make_config
from experiments.probe_support import GpuMonitor,finish,gpu_sample,run_package,write_json
from experiments.verify_physics_vehicle import stop_child
from hivemind.launcher import isaac_runtime,load_config

SOURCES=['scripts/demo_loop_showcase.py','scripts/physics_loop_showcase.py',
 'experiments/loop_showcase_config.py','experiments/probe_support.py','experiments/verify_physics_vehicle.py',
 'hivemind/launcher.py','traffic/loop_showcase_control.py','traffic/loop_driving.py',
 'traffic/driver_control.py','traffic/path_following.py','traffic/lane_geometry.py','traffic/speed_profiles.py',
 'traffic/bypass_validation.py','traffic/lidar_braking.py','traffic/physx_vehicle.py','traffic/wheel_geometry.py',
 'traffic/physics_session.py','traffic/rendered_physics_session.py','traffic/vehicle_contacts.py','traffic/evidence_chunks.py',
 'visualization/loop_showcase_view.py','visualization/highway_loop_view.py','visualization/preview_timing.py',
 'usd/physical_scene.py','usd/highway_loop_scene.py','usd/showcase_vehicle.py','usd/showcase_fleet.py','usd/scene_validation.py',
 'highway_usd/_v02/highway_v02.usda','highway_usd/_v02/navigation.json','highway_usd/_v02/manifest.json','highway_usd/_v02/parameters.json']


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--version',choices=VERSIONS,default='v04')
    parser.add_argument('--mode',choices=MODES,default='showcase')
    parser.add_argument('--headless',action='store_true');parser.add_argument('--unpaced',action='store_true')
    parser.add_argument('--camera',choices=['follow','traffic','overview'],default='follow')
    parser.add_argument('--capture',action='store_true');parser.add_argument('--drive-seconds',type=int)
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    cfg=make_config(args.version,mode=args.mode,gui=not args.headless,paced=not args.unpaced,
        camera=args.camera,capture=args.capture,drive_s=args.drive_seconds)
    runtime=isaac_runtime(ROOT,load_config(ROOT))
    if args.check:
        print(json.dumps(dict(runtime=str(runtime),config=cfg,vehicle_available=(ROOT/cfg['vehicle_directory']/'world.usda').is_file()),indent=2));return 0
    if not (ROOT/cfg['vehicle_directory']/'world.usda').is_file():
        parser.error('This showcase requires the separately supplied local prepared vehicle; it is not distributed by Git')
    sources=SOURCES+[p.relative_to(ROOT).as_posix() for p in (ROOT/cfg['vehicle_directory']).rglob('*') if p.is_file()]
    output,manifest=run_package('loop_showcase',cfg,sources)
    command=[str(runtime),str(ROOT/'scripts/physics_loop_showcase.py'),'--output',str(output)]
    child=None;result=dict(passed=False);start=time.perf_counter()
    try:
        manifest.update(command=command,isaac_version=(runtime.parent/'VERSION').read_text().strip(),gpu_before=gpu_sample())
        write_json(output/'manifest.json',manifest)
        if manifest['gpu_before']['used_mib']/manifest['gpu_before']['total_mib']>=cfg['max_gpu_fraction']:
            raise RuntimeError('GPU memory guard: close other GPU applications')
        reason=None
        print('LOOP_SHOWCASE_RUN='+str(output),flush=True)
        with (output/'runtime.log').open('w',encoding='utf-8') as log,GpuMonitor() as monitor:
            child=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)
            while child.poll() is None:
                if time.perf_counter()-start>cfg['max_process_seconds']:reason='Process deadline'
                elif monitor.rows and monitor.rows[-1]['used_mib']/monitor.rows[-1]['total_mib']>=cfg['max_gpu_fraction']:reason='GPU memory guard'
                if reason:stop_child(child);break
                time.sleep(.2)
        path=output/'probe-result.json'
        result=json.loads(path.read_text()) if path.is_file() else dict(passed=False,error='No probe result')
        log=(output/'runtime.log').read_text(encoding='utf-8',errors='replace')
        result.update(exit_code=child.returncode,stop_reason=reason,
            runtime_error_lines=[line for line in log.splitlines() if '[Error]' in line],
            usd_reference_warning_count=log.count('Unexpected reference count'),
            gpu_peak_mib=max((r['used_mib'] for r in monitor.rows),default=None),gpu_monitor_errors=monitor.errors,
            gpu_scope='Whole desktop GPU at approximately 1 Hz',wall_process_s=time.perf_counter()-start)
        result['passed']=bool(result['passed'] and child.returncode==0 and not reason and not result['runtime_error_lines']
            and not result['usd_reference_warning_count'] and monitor.rows and not monitor.errors)
        write_json(output/'gpu-samples.json',monitor.rows)
    except Exception as error:result.update(passed=False,error=repr(error))
    finally:
        if child and child.poll() is None:stop_child(child)
        changed=[p for p,h in manifest['source_hashes'].items() if not (ROOT/p).is_file() or hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h]
        result['source_changed_during_run']=changed;result['passed']=bool(result.get('passed') and not changed)
        finish(output,manifest,result)
    print(json.dumps(dict(output=str(output),**{key:result.get(key) for key in ('passed','gates','completed_passes',
        'completed_lane_changes','min_clearance_m','preview_timing','error','runtime_error_lines','source_changed_during_run')}),indent=2),flush=True)
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
