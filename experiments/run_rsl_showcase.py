"""Supervise one bounded full-highway RSL deployment; no training capability."""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import uuid

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path,data):
    payload=json.dumps(data,indent=2,allow_nan=False)+'\n'
    temp=Path(path).with_suffix('.json.tmp')
    with temp.open('x',encoding='utf-8') as stream:stream.write(payload)
    os.replace(temp,path)


def resolve_job(policy=None,*,controller='rsl',seed=61001,stage='crossing',side='left',
                gui=False,capture=False,max_seconds=600,continuous=False):
    if (controller not in ('rsl','scripted') or stage not in ('clear','static','crossing')
            or side not in (('left','right') if stage=='crossing' else (None,'left','right'))
            or type(seed) is not int or not 0<=seed<2**32
            or type(gui) is not bool or type(capture) is not bool or capture and not gui
            or type(continuous) is not bool or continuous and stage=='static'
            or type(max_seconds) is not int or not 60<=max_seconds<=900):
        raise ValueError('Invalid bounded full-sim job')
    if controller=='rsl':
        if policy is None:raise ValueError('RSL deployment requires a trusted exported42-feature policy')
        path=Path(policy).absolute()
        if (not path.is_relative_to(ROOT/'outputs') or path.resolve()!=path or path.is_symlink()
                or not path.is_file() or path.suffix!='.pt' or not 0<path.stat().st_size<=100*1024*1024):
            raise ValueError('Policy must be a bounded regular .pt export under outputs')
        policy_path,policy_sha=path.relative_to(ROOT).as_posix(),digest(path)
    else:
        if policy is not None:raise ValueError('Scripted baseline does not load a policy')
        policy_path=policy_sha=None
    return dict(schema_version=1,controller=controller,policy_path=policy_path,policy_sha256=policy_sha,
        seed=seed,stage=stage,side=side if stage=='crossing' else None,gui=gui,capture=capture,max_seconds=max_seconds,
        physics_hz=120,control_hz=60,episode_seconds=105 if continuous else 35,
        task_progress_m=180 if continuous else 60,window_progress_m=60,window_seconds=35,
        window_count=3 if continuous else 1,cruise_m_s=3.,
        background_count=12,background_pool=80,background_lanes=[1,2,3],
        highway_directory='highway_usd/_v02',route_file='scenarios/physics-road/loop-hazard-routes-v1.json',
        observation_size=42,object_slots=2,observation_source='privileged_measured_native_state_known_map',
        physics_car_count=1,background_owner='scripted_nonreactive_kinematic',
        training=False,continuous_windows=continuous,safety_shield=False,gpu_memory_stop_fraction=.90,
        capture_seconds=[3.,12.,20.,40.,60.] if continuous else [3.,12.,20.])


def validate_job(job):
    expected=resolve_job(None if job.get('policy_path') is None else ROOT/job['policy_path'],
        controller=job['controller'],seed=job['seed'],stage=job['stage'],side=job['side'],
        gui=job['gui'],capture=job['capture'],max_seconds=job['max_seconds'],continuous=job['continuous_windows'])
    if job!=expected:raise ValueError('Full-sim configuration or pinned policy changed')
    return job


def source_hashes():
    from experiments.run_loop_hazard_rsl import source_hashes as hazard_sources
    from experiments.preview_lane_follow_ppo import preview_sources
    names=set(hazard_sources())|set(preview_sources())|{
        'experiments/run_rsl_showcase.py','scripts/physics_rsl_showcase.py',
        'environments/full_loop_hazard_lab.py','environments/full_loop_hazard_contract.py',
        'environments/continuous_loop_hazard_lab.py','environments/continuous_loop_hazard_contract.py',
        'usd/highway_loop_scene.py','usd/showcase_fleet.py','usd/live_showcase_fleet.py',
        'traffic/live_showcase.py','traffic/loop_driving.py','traffic/loop_showcase_control.py',
        'traffic/showcase_clearance.py','experiments/showcase_controls.py',
        'traffic/obstacle_bypass.py','traffic/bypass_validation.py',
    }
    return {name:digest(ROOT/name) for name in sorted(names)}


def verify_worker_inputs(output):
    """Before AppLauncher, bind both current and captured source to the job."""
    output=Path(output)
    manifest=json.loads((output/'manifest.json').read_text(encoding='utf-8'))
    job=validate_job(json.loads((output/'resolved-config.json').read_text(encoding='utf-8')))
    current=source_hashes()
    captured={name:digest(output/'source'/name) for name in current}
    if (manifest.get('status')!='running' or manifest.get('source_hashes')!=current
            or captured!=current or manifest.get('config')!=job):
        raise ValueError('Fullsim manifest, source snapshot, or configuration identity changed')
    return job


def supervise(job):
    from experiments.probe_support import GpuMonitor
    from experiments.verify_lane_following_lab import kit_process_snapshot,classify_runtime_log,KNOWN_STARTUP_ERRORS
    from experiments.verify_physics_vehicle import stop_child
    from hivemind.launcher import isaac_runtime,load_config
    validate_job(job)
    before=kit_process_snapshot()
    if not before.get('available') or before['count']:
        raise RuntimeError('Full-sim jobs require an idle native process inventory')
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    output=ROOT/'outputs/rsl_full_sim'/f'v01-{stamp}-{uuid.uuid4().hex[:8]}'
    output.mkdir(parents=True,exist_ok=False)
    logs=ROOT/'logs/rsl_full_sim'/output.name;logs.mkdir(parents=True,exist_ok=False)
    hashes=source_hashes()
    for name in hashes:
        destination=output/'source'/name;destination.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(ROOT/name,destination)
    write_json(output/'resolved-config.json',job)
    runtime=isaac_runtime(ROOT,load_config(ROOT))
    command=[str(runtime),str(ROOT/'scripts/physics_rsl_showcase.py'),'--output',str(output)]
    manifest=dict(schema_version=1,status='running',created_utc=datetime.now(timezone.utc).isoformat(),
        source_hashes=hashes,config=job,command=command,kit_before=before,
        git_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        git_status=subprocess.check_output(['git','status','--short'],cwd=ROOT,text=True),
        runtime=str(runtime),training=False)
    write_json(output/'manifest.json',manifest)
    started=time.monotonic();timed_out=False;resource_stop=None;process=None;launch_error=None
    print('RUN_DIRECTORY='+str(output),flush=True)
    with GpuMonitor() as gpu, (logs/'stdout.log').open('w',encoding='utf-8') as stdout, (logs/'stderr.log').open('w',encoding='utf-8') as stderr:
        try:
            # One native stream preserves startup/error ordering relative to the
            # flushed READY marker. Keep stderr.log empty for package parity.
            process=subprocess.Popen(command,cwd=ROOT,stdout=stdout,stderr=subprocess.STDOUT,
                                     creationflags=subprocess.CREATE_NO_WINDOW)
            while process.poll() is None:
                if time.monotonic()-started>job['max_seconds']:
                    timed_out=True;stop_child(process);break
                if gpu.rows and gpu.rows[-1]['used_mib']/gpu.rows[-1]['total_mib']>=job['gpu_memory_stop_fraction']:
                    resource_stop='whole_gpu_memory';stop_child(process);break
                time.sleep(.5)
        except Exception as exc:
            launch_error=f'{type(exc).__name__}: {exc}'
        finally:
            if process is not None and process.poll() is None:stop_child(process)
    for name in ('stdout.log','stderr.log'):shutil.copyfile(logs/name,output/name)
    runtime_log='\n'.join((output/name).read_text(encoding='utf-8',errors='replace') for name in ('stdout.log','stderr.log'))
    classified=classify_runtime_log(runtime_log,KNOWN_STARTUP_ERRORS)
    after=kit_process_snapshot()
    result_path=output/'result.json'
    native=json.loads(result_path.read_text(encoding='utf-8')) if result_path.is_file() else {}
    clean=(native.get('contact_cleanup') or {}).get('passed') is True and (native.get('viewport_cleanup') or {}).get('passed') is True
    source_clean=source_hashes()==hashes
    policy_clean=job['policy_path'] is None or ((ROOT/job['policy_path']).is_file()
                    and digest(ROOT/job['policy_path'])==job['policy_sha256'])
    execution=(process is not None and process.returncode==0 and not timed_out and resource_stop is None
        and native.get('completed') is True and clean and source_clean and policy_clean
        and after.get('available') and after['count']==0 and not classified['unexpected_runtime_errors']
        and classified['usd_reference_warning_count']==0 and bool(gpu.rows) and not gpu.errors)
    summary=dict(execution_passed=bool(execution),driving_passed=bool(execution and native.get('driving_passed')),
        native=native,returncode=None if process is None else process.returncode,timed_out=timed_out,
        resource_stop=resource_stop,launch_error=launch_error,sources_unchanged=source_clean,policy_unchanged=policy_clean,
        kit_after=after,wall_seconds=time.monotonic()-started,gpu_rows=gpu.rows,gpu_errors=gpu.errors,**classified)
    write_json(output/'summary.json',summary)
    manifest.update(status='completed' if execution else 'failed',execution_passed=bool(execution),
        driving_passed=summary['driving_passed'],output_hashes={p.relative_to(output).as_posix():digest(p)
            for p in sorted(output.rglob('*')) if p.is_file() and p!=output/'manifest.json'})
    write_json(output/'manifest.json',manifest)
    print(json.dumps({'output':str(output),'execution_passed':bool(execution),'driving_passed':summary['driving_passed']}),flush=True)
    return 0 if execution else 1


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy',type=Path)
    parser.add_argument('--controller',choices=['rsl','scripted'],default='rsl')
    parser.add_argument('--seed',type=int,default=61001)
    parser.add_argument('--stage',choices=['clear','static','crossing'],default='crossing')
    parser.add_argument('--side',choices=['left','right'],default='left')
    parser.add_argument('--gui',action='store_true');parser.add_argument('--capture',action='store_true')
    parser.add_argument('--continuous',action='store_true',help='Three60m windows; one initial physical reset, one original hazard schedule')
    parser.add_argument('--max-seconds',type=int,default=600);parser.add_argument('--check',action='store_true')
    args=parser.parse_args(argv)
    if Path.cwd().resolve()!=ROOT:parser.error('Run from canonical project root')
    try:job=resolve_job(args.policy,controller=args.controller,seed=args.seed,stage=args.stage,side=args.side,
                       gui=args.gui,capture=args.capture,max_seconds=args.max_seconds,continuous=args.continuous)
    except ValueError as exc:parser.error(str(exc))
    if args.check:print(json.dumps(job,indent=2));return 0
    return supervise(job)


if __name__=='__main__':raise SystemExit(main())
