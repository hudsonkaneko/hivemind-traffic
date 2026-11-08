"""Native worker for run_rsl_showcase. No optimization or automatic training."""
from __future__ import annotations
import argparse
from copy import deepcopy
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time
import traceback

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))


def continuous_boundary_contract(reset_count,completed_windows,boundaries,retained):
    """A valid failure may end after any number of completed windows."""
    return (type(reset_count) is int and reset_count==1 and type(completed_windows) is int
        and 0<=completed_windows<=3 and retained is True and isinstance(boundaries,list)
        and len(boundaries)==min(completed_windows,2)
        and all(b.get('equal') is True and b.get('physical_reset_count')==1 for b in boundaries))


def run(output):
    from experiments.run_rsl_showcase import verify_worker_inputs,write_json
    job=verify_worker_inputs(output)
    app=env=viewport=None;captures=[];metadata=[]
    result=dict(completed=False,driving_passed=False,training=False,optimizer_steps=0,
        scope=('three60m windows with one reset and one hazard schedule' if job['continuous_windows']
               else 'one bounded60m episode on complete highway; one dynamic ego and twelve kinematic backgrounds'),
        continuous_windows=job['continuous_windows'],emergency_shield=False)
    started=time.monotonic()
    try:
        from isaaclab.app import AppLauncher
        app=AppLauncher({'headless':not job['gui'],'device':'cpu',
                         'visualizer':['kit'] if job['gui'] else ['none']}).app
        from traffic.physics_session import prepare_vehicle_runtime
        prepare_vehicle_runtime()
        import torch
        from environments.full_loop_hazard_lab import FullLoopHazardLabCfg,FullLoopHazardLabEnv
        if job['continuous_windows']:
            from environments.continuous_loop_hazard_lab import ContinuousFullLoopHazardLabEnv
            Environment=ContinuousFullLoopHazardLabEnv
        else:
            Environment=FullLoopHazardLabEnv
        from traffic.rsl_loop_policy import RslLoopPolicy
        torch.set_num_threads(4);torch.manual_seed(job['seed'])
        cfg=FullLoopHazardLabCfg()
        cfg.seed=job['seed'];cfg.reset_perturb=True
        cfg.scene_directory=str(output/'scene')
        cfg.sim.log_dir=str(ROOT/'logs/rsl_full_sim'/output.name/'isaac')
        cfg.hazard_stage=job['stage'];cfg.hazard_side=job['side']
        env=Environment(cfg)
        result['scene_initial']=env.scene_contract()
        obs,extras=env.reset(seed=job['seed'])
        write_json(output/'reset.json',deepcopy(env.last_reset))
        policy=(RslLoopPolicy(ROOT/job['policy_path'],job['policy_sha256'],episode_id=f'full-{output.name}')
                if job['controller']=='rsl' else None)
        result['controller']=job['controller']
        if policy:
            from experiments.run_loop_hazard_rsl import policy_state_hash
            result['policy']=deepcopy(policy.metadata)
            result['policy_state_before']=policy_state_hash(policy._actor)
        if job['gui']:
            from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_file
            from omni.kit.renderer.capture import acquire_renderer_capture_interface
            from scripts.preview_lane_follow_ppo import wait_for_capture,verify_capture_png
            viewport=get_active_viewport()
            if viewport is None:raise RuntimeError('Visible fullsim requires a native viewport')
            viewport.resolution=(960,600)
        def guard():
            if time.monotonic()-started>job['max_seconds']-20:
                raise TimeoutError('Bounded fullsim deadline')
        def capture(name,camera,**labels):
            guard();tick=env.episode_tick
            handle=capture_viewport_to_file(viewport,str(output/name));captures.append(handle)
            wait_for_capture(handle,render=env.sim.render,guard=guard,
                flush=acquire_renderer_capture_interface().wait_async_capture)
            image=verify_capture_png(output/name,expected_dimensions=(960,600))
            if env.episode_tick!=tick:raise RuntimeError('Capture advanced physics')
            metadata.append(dict(file=name,**labels,**camera,**image,native_completion_confirmed=True,
                camera_state_at_request_not_exact_frame_timestamp=True))
        print('LAB_LANE_TASK_READY full_highway',flush=True)
        completed=False;records=0;contacts=0;minimum=None;clipped=0;masked=0
        captured=set()
        with (output/'trajectory.jsonl').open('x',encoding='utf-8') as stream:
            for _ in range(job['episode_seconds']*60+1):
                guard()
                before=obs['policy'][0].detach().cpu().tolist()
                if policy:
                    physical=policy.predict(before)
                    action=torch.tensor([physical],dtype=torch.float32,device='cpu')
                    diagnostics=deepcopy(policy.diagnostics)
                    clipped+=diagnostics['clipping_count']
                else:
                    action=env.scripted_action()
                    diagnostics={'source':'scripted_full_loop_reference','physical_action':action[0].tolist()}
                obs,reward,terminated,truncated,extras=env.step(action)
                tr=deepcopy(extras['lane_transition'])
                done=bool(terminated[0] or truncated[0])
                row=dict(transition=tr,input_observation=before,policy=diagnostics,
                    returned_reward=float(reward[0]),returned_terminated=bool(terminated[0]),
                    returned_truncated=bool(truncated[0]),returned_observation=obs['policy'][0].detach().cpu().tolist(),
                    final_observation=extras['final_observation']['policy'][0].detach().cpu().tolist())
                if done:
                    if job['continuous_windows']:
                        row['autoreset_suppressed']=deepcopy(extras.get('autoreset_suppressed'))
                    else:
                        row['auto_reset']=deepcopy(extras.get('reset'))
                stream.write(json.dumps(row,allow_nan=False)+'\n');records+=1
                contacts+=len(tr['contacts'])
                masked+=bool(tr['deployment_observation']['masked_hazard_ids'])
                for sample in tr['physics_samples']:
                    clearance=sample['all_object_clearance']['clearance_m']
                    minimum=clearance if minimum is None else min(minimum,clearance)
                if job['gui'] and not done and tr['tick']%6==0:
                    camera=env.render_snapshot('follow',render=False)
                    viewport.camera_path=camera['camera_path'];env.sim.render()
                    for second in job['capture_seconds']:
                        if job['capture'] and tr['tick']>=round(second*120) and second not in captured:
                            captured.add(second);name=f'full-follow-{int(second):02d}s.png'
                            capture(name,camera,requested_sim_s=second)
                if done:
                    stream.flush();completed=True
                    result.update(terminal_reason=tr['reason'],success=tr['success'],
                        progress_m=(tr['continuation']['total_progress_m'] if job['continuous_windows']
                                    else tr['route_state']['progress_m']),
                        local_window_progress_m=tr['route_state']['progress_m'],final_transition=tr)
                    break
        if not completed:raise RuntimeError('Fullsim episode did not terminate within declared ticks')
        result.update(completed=True,control_steps=records,contacts=contacts,minimum_clearance_m=minimum,
            clipping_components=clipped,hazard_masked_control_steps=masked,
            driving_passed=bool(result['success'] and contacts==0 and minimum is not None and minimum>=0),
            scene_final=env.scene_contract())
        if job['continuous_windows']:
            result.update(physical_reset_count=env.reset_count,
                completed_windows=env.task.completed_windows,
                native_window_boundaries=deepcopy(env.native_window_boundaries),
                original_hazard_schedule=env.schedule.as_dict(),
                terminal_state_retained=bool(extras.get('autoreset_suppressed',{}).get('terminal_state_retained')))
            if not continuous_boundary_contract(env.reset_count,env.task.completed_windows,
                    env.native_window_boundaries,result['terminal_state_retained']):
                raise RuntimeError('Continuous reset/boundary evidence contract failed')
            result['driving_passed']=result['driving_passed'] and env.task.completed_windows==3
        if policy:
            result['policy_state_after']=policy_state_hash(policy._actor)
            result['policy_state_unchanged']=result['policy_state_after']==result['policy_state_before']
            if not result['policy_state_unchanged']:raise RuntimeError('Inference mutated policy weights or buffers')
        if job['capture']:
            # Ordinary mode auto-resets; continuous mode retains its terminal body.
            camera=env.render_snapshot('follow' if job['continuous_windows'] else 'overview',render=False)
            viewport.camera_path=camera['camera_path']
            for _ in range(8):env.sim.render()
            name='full-scene-terminal.png' if job['continuous_windows'] else 'full-scene-post-autoreset.png'
            capture(name,camera,phase='terminal_native_state' if job['continuous_windows'] else 'post_autoreset_scene')
            result['capture_files']=[m['file'] for m in metadata]
    except BaseException as exc:
        result.update(completed=False,error=f'{type(exc).__name__}: {exc}',traceback=traceback.format_exc())
        traceback.print_exc()
    finally:
        try:
            captures.clear();viewport=None
            if env is not None:
                env.close()
                result['contact_cleanup']=deepcopy(env.contact_reporting.get('cleanup'))
                result['viewport_cleanup']=deepcopy(env.viewport_cleanup)
        except BaseException as exc:
            result.update(completed=False,cleanup_error=f'{type(exc).__name__}: {exc}')
        finally:
            result['wall_seconds']=time.monotonic()-started
            write_json(output/'captures.json',metadata)
            write_json(output/'result.json',result)
            if app is not None:app.close()
    return 0 if result['completed'] else 1


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(argv)
    if Path.cwd().resolve()!=ROOT:parser.error('Run from canonical project root')
    output=args.output.resolve()
    if output.parent!=ROOT/'outputs/rsl_full_sim' or not output.is_dir() or (output/'result.json').exists():
        parser.error('Fresh supervisor-created fullsim output required')
    with (output/'started').open('x',encoding='utf-8') as stream:stream.write(str(time.time()))
    return run(output)


if __name__=='__main__':raise SystemExit(main())
