"""Bounded, inference-only CIRCLES physical-car smoke; no training entry point."""
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
import traceback
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.fetch_circles_controllers import REVISION, MODELS
from experiments.run_loop_hazard_rsl import atomic_json, source_hashes


def hashes():
    result=source_hashes()
    for name in ('experiments/probe_circles_control.py','traffic/circles_control.py',
                 'scripts/fetch_circles_controllers.py'):
        result[name]=hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
    return result


def native(args, output):
    from isaaclab.app import AppLauncher
    app=AppLauncher({'headless':True,'device':'cpu','visualizer':['none']}).app
    env=None
    result=dict(completed=False, training=False, optimizer_steps=0, episodes=[],
                scope='isolated existing physical car and static native obstacle; no background traffic',
                observation_source='privileged current native state', steering='existing PathFollower',
                output_interpretation='CRA documented acceleration; others experimental acceleration interpretation',
                policy_hz=20, control_hz=60, physics_hz=120,
                target_speed_cap_m_s=3., target_time_s=.6, throttle_gain=.7, brake_gain=.35,
                emergency_shield=False)
    started=time.monotonic()
    try:
        from traffic.physics_session import prepare_vehicle_runtime
        prepare_vehicle_runtime()
        import torch
        import numpy as np
        sys.path.insert(0,str(ROOT/'runtimes/circles_onnx312'))
        import onnxruntime as ort
        from environments.loop_hazard_lab import LoopHazardLabCfg, LoopHazardLabEnv
        from environments.lane_following_lab import LaneFollowingLabEnv
        from traffic.circles_control import measured_leader_input, acceleration_to_pedals
        torch.set_num_threads(4)
        result['versions']=dict(python=sys.version, torch=torch.__version__, numpy=np.__version__, onnxruntime=ort.__version__)
        cfg=LoopHazardLabCfg()
        cfg.scene_directory=str(output/'scene')
        cfg.sim.log_dir=str(ROOT/'logs/circles_control'/output.name/'isaac')
        cfg.hazard_stage='static'
        cfg.seed=31001
        cfg.reset_perturb=False
        env=LoopHazardLabEnv(cfg)
        atomic_json(output/'scene-contract.json',env.scene_contract())
        print('LAB_LANE_TASK_READY',flush=True)
        options=ort.SessionOptions()
        options.intra_op_num_threads=1
        options.inter_op_num_threads=1
        with (output/'steps.jsonl').open('x',encoding='utf-8') as stream:
            for name in args.models:
                path=output/'models'/(name+'.onnx')
                session=ort.InferenceSession(str(path),sess_options=options,providers=['CPUExecutionProvider'])
                env.reset(seed=31001)
                episode=dict(model=name, model_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    reset=deepcopy(env.last_reset), max_speed_m_s=0., max_lateral_m=0., contacts=0,
                    throttle_steps=0, brake_steps=0, max_throttle=0., max_brake=0.,
                    minimum_gap_m=None, acceleration_min=None, acceleration_max=None,
                    stopped_seconds=0., reason='bounded_probe_end', terminal_task_success=False)
                acceleration=0.
                for step in range(2100):
                    if time.monotonic()-started>args.max_seconds-30:
                        raise TimeoutError('Bounded native deadline')
                    state=env.last_state
                    measured=measured_leader_input(state,env.hazard_frames[0],now_s=env.episode_tick/120.)
                    if step%3==0:
                        raw=session.run(None,{session.get_inputs()[0].name:np.array([measured],dtype=np.float32)})[0]
                        if raw.size != 1 or not np.isfinite(raw).all():
                            raise ValueError('Expected one finite longitudinal output')
                        acceleration=float(raw.reshape(-1)[0])
                    # Only take steering. Neither the hazard safe-speed target nor
                    # the scripted throttle/brake enters the submitted action.
                    action=LaneFollowingLabEnv.scripted_action(env)
                    throttle,brake,target=acceleration_to_pedals(measured[0],acceleration)
                    settling=env.episode_tick<240
                    if settling: throttle,brake=0.,1.
                    action[0,1]=throttle
                    action[0,2]=brake
                    _,_,terminated,truncated,extras=env.step(action)
                    tr=extras['lane_transition']
                    state=tr['physics_state']
                    episode['max_speed_m_s']=max(episode['max_speed_m_s'],state['speed_m_s'])
                    episode['max_lateral_m']=max(episode['max_lateral_m'],abs(tr['route_state']['lateral_error_m']))
                    episode['contacts']+=len(tr['contacts'])
                    episode['minimum_gap_m']=min(measured[2],episode['minimum_gap_m'] or measured[2])
                    if not settling:
                        episode['throttle_steps']+=int(throttle>1e-5)
                        episode['brake_steps']+=int(brake>1e-5)
                        episode['max_throttle']=max(episode['max_throttle'],throttle)
                        episode['max_brake']=max(episode['max_brake'],brake)
                        episode['acceleration_min']=min(acceleration,episode['acceleration_min'] if episode['acceleration_min'] is not None else acceleration)
                        episode['acceleration_max']=max(acceleration,episode['acceleration_max'] if episode['acceleration_max'] is not None else acceleration)
                    if tr['route_state']['progress_m']>5 and state['speed_m_s']<.1:
                        episode['stopped_seconds']+=1/60
                    else: episode['stopped_seconds']=0.
                    stream.write(json.dumps(dict(model=name, step=step, policy_input=measured,
                        acceleration_m_s2=acceleration, target_speed_m_s=target, settling=settling,
                        transition=tr),allow_nan=False)+'\n')
                    if bool(terminated[0] or truncated[0]):
                        episode['reason']=tr['reason']
                        episode['terminal_task_success']=bool(tr['success'])
                        break
                episode.update(steps=step+1, progress_m=tr['route_state']['progress_m'],
                    final_speed_m_s=state['speed_m_s'], first_guard_failure=tr['first_guard_failure'],
                    final_transition=tr)
                episode['actuation_smoke_passed']=bool(episode['progress_m']>5 and episode['max_speed_m_s']>.5
                    and episode['throttle_steps']>0 and episode['contacts']==0
                    and episode['max_lateral_m']<.5 and not episode['first_guard_failure'])
                # Low terminal speed can be actuator drag/stalling, not successful
                # policy braking. Task success remains the separate strict gate.
                episode['near_stationary_at_end']=bool(episode['actuation_smoke_passed'] and episode['stopped_seconds']>=1.)
                result['episodes'].append(episode)
                stream.flush()
                atomic_json(output/'result.json',result)
                print('CIRCLES_EPISODE '+json.dumps({k:v for k,v in episode.items()
                    if k not in ('reset','final_transition')},allow_nan=False),flush=True)
        result['scene_final']=env.scene_contract()
        result['completed']=True
    except BaseException as exc:
        result.update(error=f'{type(exc).__name__}: {exc}',traceback=traceback.format_exc())
        raise
    finally:
        try:
            if env is not None:
                env.close()
                result['contact_cleanup']=env.contact_reporting.get('cleanup')
                result['viewport_cleanup']=env.viewport_cleanup
        finally:
            result['wall_seconds']=time.monotonic()-started
            atomic_json(output/'result.json',result)
            app.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--models',choices=list(MODELS),nargs='+',default=list(MODELS))
    parser.add_argument('--max-seconds',type=int,default=420)
    parser.add_argument('--native',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--output',type=Path,help=argparse.SUPPRESS)
    args=parser.parse_args()
    if Path.cwd().resolve()!=ROOT or not 60<=args.max_seconds<=600:
        parser.error('Run from checkout root with 60..600 second bound')
    if len(args.models)!=len(set(args.models)):
        parser.error('Choose unique models')
    if args.native:
        output=args.output.resolve()
        if output.parent!=ROOT/'outputs/circles_control' or not output.is_dir():
            parser.error('Invalid native run directory')
        manifest=json.loads((output/'manifest.json').read_text())
        if manifest['source_sha256']!=hashes() or manifest['models']!=args.models:
            raise ValueError('Sources or arguments changed')
        for name,digest in manifest['model_sha256'].items():
            if hashlib.sha256((output/'models'/name).read_bytes()).hexdigest()!=digest:
                raise ValueError('Model identity changed')
        (output/'started').open('x').close()
        native(args,output)
        return 0
    from experiments.verify_lane_following_lab import kit_process_snapshot, classify_runtime_log, KNOWN_STARTUP_ERRORS
    from experiments.verify_physics_vehicle import stop_child
    from experiments.probe_support import GpuMonitor, gpu_sample
    from hivemind.launcher import isaac_runtime, load_config
    snapshot=kit_process_snapshot()
    if not snapshot['available'] or snapshot['count']:
        raise RuntimeError('Native jobs must run serially')
    gpu=gpu_sample()
    if gpu['used_mib']/gpu['total_mib']>=.9: raise RuntimeError('GPU memory preflight failed')
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    output=ROOT/'outputs/circles_control'/f'v01-{stamp}-{uuid.uuid4().hex[:8]}'
    output.mkdir(parents=True,exist_ok=False)
    logs=ROOT/'logs/circles_control'/output.name
    logs.mkdir(parents=True,exist_ok=False)
    sources=hashes()
    for relative in sources:
        target=output/'sources'/relative
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(ROOT/relative,target)
    models=ROOT/'outputs/circles_control/models'/REVISION
    shutil.copytree(models,output/'models')
    model_hashes={name+'.onnx':hashlib.sha256((models/(name+'.onnx')).read_bytes()).hexdigest() for name in args.models}
    atomic_json(output/'manifest.json',dict(models=args.models,model_sha256=model_hashes,
        source_sha256=sources,upstream_revision=REVISION,max_seconds=args.max_seconds,seed=31001,
        actuation_pass_rule='progress>5m, peak speed>.5m/s, throttle>0, no contacts/guard, max lateral<.5m',
        stop_rule='after >5m progress, final speed<.1m/s for >=1s, positive bumper gap',
        git_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True,cwd=ROOT).strip(),
        git_status=subprocess.check_output(['git','status','--short'],text=True,cwd=ROOT)))
    runtime=isaac_runtime(ROOT,load_config(ROOT))
    command=[str(runtime),str(Path(__file__).resolve()),*sys.argv[1:],'--native','--output',str(output)]
    print('RUN_DIRECTORY='+str(output),flush=True)
    started=time.monotonic()
    stop_reason=None
    with (logs/'runtime.log').open('w') as stdout,GpuMonitor() as monitor:
        # Preserve startup/error chronology. Concatenating separate streams puts
        # early stderr errors after the readiness marker and misclassifies them.
        environment=dict(os.environ, PYTHONUNBUFFERED='1')
        process=subprocess.Popen(command,cwd=ROOT,env=environment,stdout=stdout,stderr=subprocess.STDOUT,
                                 creationflags=subprocess.CREATE_NO_WINDOW)
        try:
            while process.poll() is None:
                if time.monotonic()-started>args.max_seconds: stop_reason='wall_time_limit'
                elif monitor.errors: stop_reason='gpu_monitor_unavailable'
                elif monitor.rows and monitor.rows[-1]['used_mib']/monitor.rows[-1]['total_mib']>=.9: stop_reason='gpu_memory_limit'
                if stop_reason:
                    stop_child(process)
                    break
                time.sleep(1)
        finally:
            if process.poll() is None: stop_child(process)
    result=json.loads((output/'result.json').read_text()) if (output/'result.json').is_file() else {}
    after=kit_process_snapshot()
    assessment=classify_runtime_log((logs/'runtime.log').read_text(errors='replace'),KNOWN_STARTUP_ERRORS)
    valid=bool(process.returncode==0 and not stop_reason and hashes()==sources and after['available'] and after['count']==0
        and result.get('completed') and len(result.get('episodes',[]))==len(args.models)
        and result.get('contact_cleanup',{}).get('passed') and result.get('viewport_cleanup',{}).get('passed')
        and not assessment['unexpected_runtime_errors'] and not assessment['usd_reference_warning_count'])
    atomic_json(output/'gpu-samples.json',monitor.rows)
    atomic_json(output/'exit.json',dict(evidence_valid=valid,returncode=process.returncode,stop_reason=stop_reason,
        sources_unchanged=hashes()==sources,kit_after=after,runtime_log_assessment=assessment,
        wall_seconds=time.monotonic()-started))
    print(json.dumps(dict(evidence_valid=valid,output=str(output))),flush=True)
    return 0 if valid else 1


if __name__=='__main__': raise SystemExit(main())
