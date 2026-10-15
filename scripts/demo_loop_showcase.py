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
from hivemind.launcher import SetupError
from hivemind.showcase_setup import check_showcase

SOURCES=['scripts/demo_loop_showcase.py','scripts/physics_loop_showcase.py',
 'experiments/loop_showcase_config.py','experiments/probe_support.py','experiments/verify_physics_vehicle.py',
 'hivemind/launcher.py','hivemind/showcase_setup.py','traffic/loop_showcase_control.py','traffic/loop_driving.py',
 'traffic/driver_control.py','traffic/path_following.py','traffic/lane_geometry.py','traffic/speed_profiles.py',
 'traffic/bypass_validation.py','traffic/showcase_clearance.py','traffic/lidar_braking.py','traffic/physx_vehicle.py','traffic/wheel_geometry.py',
 'traffic/physics_session.py','traffic/rendered_physics_session.py','traffic/vehicle_contacts.py','traffic/evidence_chunks.py','traffic/showcase_runtime.py',
 'traffic/runtime_profile.py','traffic/showcase_performance.py','visualization/showcase_responsiveness.py',
 'visualization/loop_showcase_view.py','visualization/highway_loop_view.py','visualization/preview_timing.py',
 'usd/physical_scene.py','usd/highway_loop_scene.py','usd/showcase_vehicle.py','usd/showcase_fleet.py','usd/scene_validation.py',
 'highway_usd/_v02/highway_v02.usda','highway_usd/_v02/navigation.json','highway_usd/_v02/manifest.json','highway_usd/_v02/parameters.json']


def read_probe_result(output):
    """Keep pre-close diagnostics after a native crash, but never accept them."""
    completed=output/'probe-result.json'
    if completed.is_file():
        return json.loads(completed.read_text())
    partial=output/'pre-close-result.json'
    if partial.is_file():
        result=json.loads(partial.read_text())
        result.update(passed=False,cleanup_verified=False,
            incomplete_result_source='pre-close-result.json',
            completion_error='Native process did not publish its post-cleanup probe result')
        return result
    return dict(passed=False,error='No probe result')


def runtime_failure_lines(log):
    return [line for line in log.splitlines() if '[Error]' in line or '[Fatal]' in line]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--version',choices=VERSIONS,default='v04')
    parser.add_argument('--mode',choices=MODES,default='showcase')
    parser.add_argument('--headless',action='store_true');parser.add_argument('--unpaced',action='store_true')
    parser.add_argument('--camera',choices=['follow','traffic','overview'],default='follow')
    parser.add_argument('--capture',action='store_true');parser.add_argument('--drive-seconds',type=int)
    parser.add_argument('--evidence',choices=['presentation','full'],help='v06/v07 only; presentation keeps safety traces but samples optional audits')
    parser.add_argument('--render-hz',type=int,choices=[10,20,30,60],help='v06/v07 display cadence; 60 Hz requires v07; never physics cadence')
    parser.add_argument('--ui-profile',choices=['showcase','editor'],help='v07 only; regular editor (default) or optional minimal showcase app')
    parser.add_argument('--cpu-threads',type=int,choices=[4,8,16,32],help='v07 worker-count experiment; does not alter physics cadence')
    parser.add_argument('--profile-render',action='store_true',help='v07 bounded renderer diagnosis; not an FPS qualification run')
    parser.add_argument('--check',action='store_true')
    parser.add_argument('--vehicle',choices=['prepared','public'],default='prepared',
                        help='prepared: local Americano; public: procedural PhysX block car, no private asset')
    parser.add_argument('--traffic-count',type=int,help='v08/v09 background cars, 4–80')
    parser.add_argument('--ego-mph',type=float,help='v08/v09 main-car target, 5–35 mph')
    parser.add_argument('--traffic-mph',type=float,help='v08/v09 inner-lane traffic speed, 5–25 mph')
    parser.add_argument('--following-gap',type=float,help='v08/v09 preferred following gap, 1.8–3 s')
    parser.add_argument('--panel-hidden',action='store_true',help='v08/v09 matched UI-overhead test')
    parser.add_argument('--control-smoke',action='store_true',help='Native-model test: v08 restart, v09 live changes (use 90 s drive); not mouse latency or FPS qualification')
    args=parser.parse_args()
    controls=None
    overrides={'traffic_count':args.traffic_count,'ego_mph':args.ego_mph,
               'traffic_mph':args.traffic_mph,'following_s':args.following_gap}
    if args.version in ('v08','v09'):
        from experiments.showcase_controls import defaults
        controls={**defaults(),**{k:v for k,v in overrides.items() if v is not None}}
        if args.render_hz is not None:controls['render_hz']=args.render_hz
    elif any(v is not None for v in overrides.values()):parser.error('Traffic/driving overrides require v08 or v09')
    cfg=make_config(args.version,mode=args.mode,gui=not args.headless,paced=not args.unpaced,
        camera=args.camera,capture=args.capture,drive_s=args.drive_seconds,evidence=args.evidence,render_hz=args.render_hz,
        ui_profile=args.ui_profile,cpu_threads=args.cpu_threads,profile_render=args.profile_render,
        controls=controls,panel_hidden=args.panel_hidden,control_smoke=args.control_smoke,vehicle=args.vehicle)
    try:
        runtime,setup=check_showcase(ROOT,cfg,load_config(ROOT))
    except (SetupError,OSError,ValueError) as error:
        print('Setup needed: '+str(error),file=sys.stderr);return 2
    if args.check:
        print(json.dumps(dict(**setup,config=cfg),indent=2));return 0
    for episode in range(11):
        result=run_once(cfg,runtime)
        if not result.get('restart_verified'):return 0 if result['passed'] else 1
        if episode==10:
            print('Restart limit reached. Relaunch the command for a fresh session.',flush=True)
            return 0
        cfg=next_run_config(cfg,result['restart_settings'])
        print('Safe stop and clean shutdown verified. Starting requested settings in a fresh process.',flush=True)


def next_run_config(cfg,values):
    if cfg['version'] not in ('v08','v09'):raise ValueError('Only sidebar versions support requested restarts')
    return make_config(cfg['version'],gui=cfg['gui'],paced=cfg['paced'],camera=cfg['camera'],capture=cfg['capture'],
        drive_s=cfg['drive_s'],evidence=cfg['evidence_profile'],cpu_threads=cfg['cpu_threads'],
        controls=values,panel_hidden=cfg['panel_hidden'],control_smoke=False,
        vehicle=cfg.get('vehicle_profile','prepared'))


def can_restart(result):
    return bool(result.get('restart_ready') and result.get('run_status')=='stopped_for_requested_restart'
        and result.get('exit_code')==0 and result.get('cleanup_verified') is True
        and result.get('gates') and all(result['gates'].values())
        and not result.get('stop_reason') and not result.get('runtime_error_lines')
        and not result.get('usd_reference_warning_count') and not result.get('gpu_monitor_errors')
        and result.get('gpu_peak_mib') is not None and not result.get('source_changed_during_run')
        and not any(v for k,v in result.items() if k=='error' or k.endswith('_error')))


def run_once(cfg,runtime):
    sources=list(SOURCES)
    if cfg['vehicle_directory']:
        sources += [p.relative_to(ROOT).as_posix() for p in (ROOT/cfg['vehicle_directory']).rglob('*') if p.is_file()]
    if cfg['version'] in ('v07','v08','v09'):sources.extend(('visualization/highwaysim.showcase.kit','visualization/showcase_render_profile.py'))
    if cfg['version'] in ('v08','v09'):sources.extend(('experiments/showcase_controls.py','visualization/showcase_sidebar.py'))
    if cfg['version']=='v09':sources.extend(('traffic/live_showcase.py','usd/live_showcase_fleet.py'))
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
        result=read_probe_result(output)
        log=(output/'runtime.log').read_text(encoding='utf-8',errors='replace')
        result.update(exit_code=child.returncode,stop_reason=reason,
            runtime_error_lines=runtime_failure_lines(log),
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
        result['restart_verified']=can_restart(result) if cfg['version'] in ('v08','v09') else False
        finish(output,manifest,result)
    print(json.dumps(dict(output=str(output),**{key:result.get(key) for key in ('passed','gates','completed_passes',
        'completed_lane_changes','min_clearance_m','preview_timing','error','runtime_error_lines','source_changed_during_run')}),indent=2),flush=True)
    return result


if __name__=='__main__':raise SystemExit(main())
