"""Continuous lidar experiment; SUMO alone owns vehicle motion.

WORLD packets are produced by RTX with Motion BVH enabled; the selected output
compensation mode is recorded explicitly, not inferred from the world frame.
An interpolated USD pose is a display/sensor sample of two SUMO states, never
an independent controller. Warm-up is excluded from real-time factor.
"""
import json
import math
import time
import numpy as np
from traffic.lidar_avoidance import body_bounds, rectangle_gap, summarize


def interpolate(a,b,f):
    result={k:a[k]+(b[k]-a[k])*f for k in ('x','y','speed')}
    result['angle']=a['angle']+((b['angle']-a['angle']+180)%360-180)*f
    return result


def packet_points(packet,ego,now,max_age=.25,*,require_scan_complete=False):
    """Validate timing and filter WORLD points. No obstacle truth is accepted."""
    xyz=np.asarray(packet['xyz']); flags=np.asarray(packet['flags'])
    if xyz.ndim!=2 or xyz.shape[1]!=3 or flags.shape!=(len(xyz),):
        raise ValueError('Malformed world point packet')
    offsets=np.asarray(packet['offset'])
    if offsets.shape!=(len(xyz),) or not len(xyz): raise ValueError('Malformed ray timing')
    ray_times=packet['timestamp']*1e-9+offsets*1e-9
    start=float(ray_times.min());end=float(ray_times.max());age=now-start
    # GMO frameStart/frameEnd describe the latest render interval, not the
    # accumulated rotary scan. Per-ray offsets define the acquisition window.
    timing=(.09<=end-start<=.11 and -.002<=now-end and -.002<=age<=max_age)
    if require_scan_complete:
        # Full-scan metadata, not the span of successful hits, establishes
        # completion. A narrow road can leave entire angular sectors empty.
        age=now-packet['timestamp']*1e-9
        timing=(packet.get('scan_complete')==1 and np.isfinite(offsets).all()
                and offsets.min()>=0 and offsets.max()<=110_000_000
                and -.002<=now-end and -.002<=age<=max_age)
    valid=np.isfinite(xyz).all(axis=1)&((flags&64)!=0)
    healthy=bool(timing and len(xyz)>=1000 and valid.sum()>=20)
    yaw=math.radians(90-ego['angle'])
    dx=xyz[:,0]-ego['x'];dy=xyz[:,1]-ego['y']
    local_x=dx*math.cos(yaw)+dy*math.sin(yaw)
    local_y=-dx*math.sin(yaw)+dy*math.cos(yaw)
    # Conservative ego exclusion over the recent packet's travel interval.
    margin=max(0.,age)*max(ego['speed'],0.)+.2
    self_hit=(local_x>=-5-margin)&(local_x<=.2)&(np.abs(local_y)<=1.35)
    keep=valid&~self_hit&(xyz[:,2]>.3)&(xyz[:,2]<1.5)&(dx*dx+dy*dy<80**2)
    return xyz[keep,:2],healthy,float(age)


def summarize_realtime(rows,blocked,dropout,wall,alignment):
    summary=summarize(rows,blocked)
    if not rows: return summary
    if dropout>=0:
        post_fault=[r for r in rows if not r['fresh']]
        summary['passed']=bool(rows[-1]['realized_speed']<.2 and post_fault
            and all(r['reason']=='sensor-failsafe' and r['lane_request'] is None for r in post_fault)
            and all(r['obstacle_gap']>.3 and not r['collisions'] for r in rows)
            and summary['max_lateral_step_m']<=.15 and summary['speed_interventions']==0
            and all(body_bounds(r['ego_after'])[2]>=-6.4 and body_bounds(r['ego_after'])[3]<=0 for r in rows))
    behavioral=summary['passed'];seconds=len(rows)*.1;rtf=seconds/wall
    misses=sum(r['deadline_lateness']>.1 for r in rows)
    summary.update(behavior_passed=behavioral,passed=bool(behavioral and .95<=rtf<=1.05 and alignment<=.3 and misses==0),
        loop_wall_seconds=wall,traffic_seconds=seconds,loop_real_time_factor=rtf,
        deadline_misses_over_100ms=misses,max_alignment_p95_m=alignment,
        max_sensor_age_seconds=max(r['sensor_age'] for r in rows),
        max_used_sensor_age_seconds=max((r['sensor_age'] for r in rows if r.get('fresh',r['healthy'])),default=0.),
        max_receipt_age_seconds=max(r.get('receipt_age',0) for r in rows),
        sensor_failsafe_actions=sum(r.get('reason')=='sensor-failsafe' for r in rows),
        profile_totals_seconds={k:sum(r['profile_seconds'][k] for r in rows) for k in rows[0]['profile_seconds']},
        limitation='Static obstacles; known lane map; SUMO kinematics; WORLD RTX output mode recorded in manifest; not hard-real-time certified')
    return summary


def run_realtime(app,timeline,traffic,latest,errors,planner,set_pose,display,args,out,rows_obstacles,rows):
    import gc
    enabled=gc.isenabled();events=[];active={}
    def observe(phase,info):
        generation=info['generation'];now=time.perf_counter()
        if phase=='start': active[generation]=now
        elif generation in active:
            events.append(dict(start=active.pop(generation),end=now,generation=generation))
    gc.callbacks.append(observe)
    try:
        if args.gc_mode=='deferred':
            gc.collect();gc.disable()
        return _run_realtime(app,timeline,traffic,latest,errors,planner,set_pose,display,args,out,rows_obstacles,rows)
    finally:
        gc.callbacks.remove(observe)
        if enabled: gc.enable()
        else: gc.disable()
        (out/'gc-profile.json').write_text(json.dumps(events,indent=2))


def _run_realtime(app,timeline,traffic,latest,errors,planner,set_pose,display,args,out,rows_obstacles,rows):
    state=traffic.snapshot();set_pose(state['vehicles'])
    # Fixed renderer ticks make the sensor/traffic epoch mapping deterministic.
    for _ in range(36): app.update()
    if errors: raise RuntimeError(errors[-1])
    if not latest: raise RuntimeError('No RTX packet after warm-up')
    sensor_origin=timeline.get_current_time();traffic_origin=state['time']
    try:
        import psutil
        process=psutil.Process();rss_start=process.memory_info().rss
    except ImportError:
        process=None;rss_start=None
    contract=dict(version=2,sensor_origin=sensor_origin,traffic_origin=traffic_origin,
        alignment_policy='Only fresh new packets admitted to the controller are scored; rejected/stale packets remain saved',
        units='meters',world_axes='X road forward; Y left; Z up',pose_reference='front bumper; sensor z=1m',
        scan_coordinates='WORLD CARTESIAN '+args.world_motion,max_oldest_ray_age_seconds=.25,
        control_hz=10,render_hz=30,authority='SUMO',sensor='Example_Rotary '+args.sensor_quality+' density; native noise unless ideal-sensor')
    started=time.perf_counter();previous_stamp=-1;accepted=None
    contract['loop_wall_origin']=started
    (out/'timing-contract.json').write_text(json.dumps(contract,indent=2))
    profiles=[];max_alignment_error=0.;misses=0
    for step in range(round(args.seconds*10)):
        if not app.is_running(): raise RuntimeError('Viewer closed')
        begin=time.perf_counter();ego=state['vehicles']['ego'];now=timeline.get_current_time()
        if errors: raise RuntimeError(errors[-1])
        if args.dropout_step<0 or step<args.dropout_step: accepted=latest.copy()
        if accepted is None: raise RuntimeError('Missing initial packet')
        points,healthy,age=packet_points(accepted,ego,now)
        is_new=accepted['timestamp']>previous_stamp
        receipt_age=begin-accepted['received_wall']
        fresh=healthy and -.001<=receipt_age<=.25 and step!=args.fault_step
        decision=planner.step(points if is_new else np.empty((0,2)),healthy,ego,fresh=fresh)
        previous_stamp=max(previous_stamp,accepted['timestamp'])
        control_end=time.perf_counter()
        before=state;state=traffic.step(decision.speed,decision.lane_request)
        after=state['vehicles']['ego']
        traci_end=time.perf_counter()
        a,b=body_bounds(ego),body_bounds(after)
        swept=(min(a[0],b[0])-.02,max(a[1],b[1])+.02,min(a[2],b[2])-.02,max(a[3],b[3])+.02)
        # Ground truth is used only here, after control, to audit static point alignment.
        evaluate_alignment=bool(len(points) and fresh and is_new)
        if evaluate_alignment:
            distances=[]
            for xmin,xmax,ymin,ymax in rows_obstacles:
                dx=np.maximum(np.maximum(xmin-points[:,0],points[:,0]-xmax),0)
                dy=np.maximum(np.maximum(ymin-points[:,1],points[:,1]-ymax),0)
                distances.append(np.hypot(dx,dy))
            alignment=float(np.quantile(np.min(distances,axis=0),.95))
            max_alignment_error=max(max_alignment_error,alignment)
        else: alignment=None
        row=dict(step=step,traffic_time=before['time'],sensor_time=now,
            sensor_frame_start=accepted['frame_start']*1e-9,sensor_frame_end=accepted['frame_end']*1e-9,
            sensor_age=age,receipt_age=receipt_age,packet_timestamp=accepted['timestamp'],is_new=is_new,
            healthy=healthy,fresh=fresh,ego=ego,ego_after=after,phase=decision.phase,
            reason=decision.reason,lane_request=decision.lane_request,requested_speed=decision.speed,
            realized_speed=after['speed'],collisions=state['collisions'],lidar_distance=decision.clearance,
            obstacle_end_x=rows_obstacles[0][1],obstacle_gap=min(rectangle_gap(swept,o) for o in rows_obstacles),
            alignment_p95_m=alignment,alignment_evaluated=evaluate_alignment,wall_elapsed=begin-started)
        np.savez(out/f'continuous_{step:04d}.npz',**{k:accepted[k] for k in ('xyz','flags','offset','timestamp','frame_start','frame_end','start_position','end_position')})
        save_end=time.perf_counter();render_cost=0.;sleep_cost=0.
        for frame in range(1,4):
            set_pose({vid:interpolate(before['vehicles'][vid],v,frame/3) for vid,v in state['vehicles'].items()})
            display(after,f'CONTINUOUS LIDAR | traffic {(step+1)*.1:.1f}s\n{decision.phase} | speed {after["speed"]:.2f} m/s\nOldest ray age {age*1000:.0f} ms')
            render_start=time.perf_counter();app.update();render_cost+=time.perf_counter()-render_start
            if not args.unpaced:
                wait=started+(step*3+frame)/30-time.perf_counter()
                if wait>0:
                    sleep_start=time.perf_counter();time.sleep(wait);sleep_cost+=time.perf_counter()-sleep_start
        drift=abs(timeline.get_current_time()-sensor_origin-(state['time']-traffic_origin))
        if drift>1e-4: raise RuntimeError(f'Sensor/traffic clock drift {drift}')
        elapsed=time.perf_counter()-started
        lateness=elapsed-(step+1)*.1
        if lateness>.1: misses+=1
        row['deadline_lateness']=lateness
        row['profile_seconds']=dict(control=control_end-begin,traci=traci_end-control_end,
            evidence=save_end-traci_end,render=render_cost,pacing=sleep_cost)
        profiles.append(row['profile_seconds']);rows.append(row)
        with (out/'telemetry.jsonl').open('a') as f: f.write(json.dumps(row)+'\n')
        if args.gui and args.capture_preview and step in (50,130,180):
            from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_file
            capture_viewport_to_file(get_active_viewport(),str(out/f'realtime_preview_{step:04d}.png'))
        if step%50==0: print(f'REALTIME step={step} phase={decision.phase} age={age:.3f} rtf={(step+1)*.1/elapsed:.3f} alignment={alignment}',flush=True)
    wall=time.perf_counter()-started
    summary=summarize_realtime(rows,args.blocked_lane,args.dropout_step,wall,max_alignment_error)
    summary['rss_start_bytes']=rss_start
    summary['rss_end_bytes']=process.memory_info().rss if process else None
    return summary
