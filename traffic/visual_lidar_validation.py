"""Predeclared gates for the bounded, one-car RTX stopping demonstration.

Obstacle geometry is evaluation-only, never a controller input. Trace rows are
post-step: the decision/command tick is row['tick'] - 1.
"""
from dataclasses import asdict
import math
from numbers import Real


def _finite(v):
    return isinstance(v, Real) and not isinstance(v, bool) and math.isfinite(v)


def _integer(v):
    return isinstance(v, int) and not isinstance(v, bool)


def validate_config(config):
    """Reject ambiguous or relaxed fixture settings before starting Isaac."""
    def number(mapping, name, low, high, integer=False):
        v = mapping.get(name)
        if not _finite(v) or (integer and not _integer(v)) or not low <= v <= high:
            raise ValueError(f'{name} must be finite and in [{low},{high}]')
        return v
    if not isinstance(config, dict):
        raise ValueError('Configuration must be an object')
    for k, rate in [('physics_hz',120),('control_hz',60),('planner_hz',10),('render_hz',30),('sensor_hz',20)]:
        number(config,k,rate,rate,True)
    number(config,'schema_version',1,1,True)
    number(config,'seed',0,2**32-1,True)
    if config.get('mode') not in ('smoke','obstacle','dropout'):
        raise ValueError('Unknown mode')
    duration = number(config,'duration_s',8 if config['mode']=='smoke' else 35,60)
    if not math.isclose(duration*120,round(duration*120),rel_tol=0,abs_tol=1e-9):
        raise ValueError('Duration must contain whole physics ticks')
    for k,v in [('settle_s',2),('target_speed_m_s',3),('barrier_station_m',70),('dropout_at_s',12)]:
        number(config,k,v,v)
    number(config,'max_process_seconds',540,540,True)
    number(config,'max_gpu_fraction',.01,.90)
    if config.get('barrier_dimensions_m') != [2,1.6,1.5]:
        raise ValueError('Use the fixed 2x1.6x1.5 metre barrier')
    for k,v in [('route_id','left_r60'),('route_file','scenarios/physics-road/routes.json'),
                ('driver_config','experiments/physics-lane-following.json')]:
        if config.get(k) != v:
            raise ValueError('Use the frozen route and driver configuration')
    for k in ('gui','points','paced','capture'):
        if not isinstance(config.get(k),bool):
            raise ValueError(f'{k} must be boolean')
    if config.get('camera') not in ('overview','follow'):
        raise ValueError('Unknown camera')
    from traffic.lidar_braking import LidarBrakeConfig
    from traffic.path_following import FollowerConfig
    for k,factory in [('braking',LidarBrakeConfig),('follower',FollowerConfig)]:
        if not isinstance(config.get(k),dict):
            raise ValueError(f'Resolved {k} settings required')
        try:
            resolved = factory(**config[k])
        except (TypeError,ValueError) as error:
            raise ValueError(f'Invalid {k} settings') from error
        if asdict(resolved) != config[k]:
            raise ValueError(f'Every {k} setting must be resolved')
    fixed = dict(physics_hz=120,max_speed_m_s=3.,front_bumper_offset_m=2.4,half_width_m=.9,
        lateral_margin_m=.25,min_height_m=-.6,max_height_m=.6,planning_deceleration_m_s2=1.5,
        reaction_margin_s=.2,stopping_margin_m=1.,max_scan_age_ticks=24,max_scan_span_ticks=24)
    if any(config['braking'].get(k)!=v for k,v in fixed.items()):
        raise ValueError('Use the declared braking geometry and oldest-sample age limit')
    f=config['follower']
    if f['wheelbase_m']!=3.2 or f['rear_axle_offset_m']!=1.6 or f['max_speed_m_s']!=3:
        raise ValueError('Follower geometry/speed must match the physical vehicle')
    a=config.get('acceptance')
    if not isinstance(a,dict):
        raise ValueError('Acceptance limits required')
    for k,maximum in [('lane_rms_max_m',.2),('lane_max_m',.5),('stop_speed_max_m_s',.05),
                      ('sensor_pose_error_max_m',.05),('sensor_max_age_s',.2),('physics_clock_error_max_s',1e-6)]:
        number(a,k,1e-12,maximum)
    number(a,'minimum_barrier_gap_m',.5,6)
    number(a,'minimum_progress_m',20,20)
    number(a,'hold_s',5,5)
    if a.get('render_must_not_advance_physics') is not True:
        raise ValueError('Rendering must not advance physics')
    return config


def barrier_plane_gap(state, point, dimensions_m=(2,1.6,1.5)):
    """Evaluation-only signed separation using the full rotated chassis box."""
    from traffic.physical_lidar import rotation_matrix
    rotation = rotation_matrix(state['quaternion_xyzw'])
    normal = [math.cos(point.yaw_rad), math.sin(point.yaw_rad), 0]
    extent = sum(abs(sum(normal[j]*rotation[j,i] for j in range(3)))*d/2
                 for i,d in enumerate((4.8,1.8,1.4)))
    delta = sum((point.position_xy[j]-state['position_m'][j])*normal[j] for j in range(2))
    return float(delta-dimensions_m[0]/2-extent)


def _finite_tree(value):
    if isinstance(value,dict):
        return all(_finite_tree(v) for v in value.values())
    if isinstance(value,(list,tuple)):
        return all(_finite_tree(v) for v in value)
    return not isinstance(value,Real) or isinstance(value,bool) or math.isfinite(value)


def assess_visual_lidar(rows, config, *, contacts, errors, render_clock_checks,
                        sensor_pose_errors, sensor_frames):
    """Require per-frame pose coverage and exact original-scan dropout expiry."""
    a,hz=config['acceptance'],config['physics_hz']
    stride=hz//config['control_hz']
    total=round(config['duration_s']*hz)
    gates=dict(complete=len(rows)==total,finite_telemetry=bool(rows) and _finite_tree(rows),
        sensor_errors=not errors,no_contacts=not contacts,
        render_does_not_step=bool(render_clock_checks) and all(v is True for v in render_clock_checks))
    try:
        gates['trace_schema']=bool(rows) and all(
            _integer(r['tick']) and r['tick']==i+1 and _finite(r['sim_time_s'])
            and abs(r['sim_time_s']-(i+1)/hz)<=1e-9
            and isinstance(r['control_tick'],bool) and r['control_tick']==(i%stride==0)
            and all(_finite(r[k]) for k in ('speed_m_s','progress_m','lateral_error_m','footprint_lateral_bound_m','barrier_gap_m'))
            and r['speed_m_s']>=0 and r['lidar']['status'] in ('clear','obstacle','stale_invalid')
            and isinstance(r['lidar']['stop_latched'],bool)
            and isinstance(r['control']['is_fallback'],bool) and isinstance(r['driver']['fallback'],bool)
            and all(_finite(r['control'][k]) and 0<=r['control'][k]<=1 for k in ('throttle','brake'))
            for i,r in enumerate(rows))
    except (KeyError,TypeError,ValueError):
        gates['trace_schema']=False
    if not gates['trace_schema'] or not gates['finite_telemetry']:
        return dict(passed=False,gates={k:bool(v) for k,v in gates.items()},metrics={'samples':len(rows)})
    active=[r for r in rows if r['tick']>round(config['settle_s']*hz)]
    decisions=[r for r in active if r['control_tick']]
    hold=rows[-round(a['hold_s']*hz):]
    rms=math.sqrt(sum(r['lateral_error_m']**2 for r in active)/len(active)) if active else None
    lane_max=max((abs(r['lateral_error_m']) for r in active),default=None)
    gates.update(lane_tracking=rms is not None and rms<=a['lane_rms_max_m'] and lane_max<=a['lane_max_m'],
        no_departure=max(r['footprint_lateral_bound_m'] for r in rows)<=1.8,
        safe_gap=min(r['barrier_gap_m'] for r in rows)>=a['minimum_barrier_gap_m'],
        initial_station=abs(rows[0]['progress_m'])<=.5,
        moving=rows[-1]['progress_m']-rows[0]['progress_m']>=(10 if config['mode']=='smoke' else a['minimum_progress_m']),
        no_driver_fallback=bool(active) and all(not r['control']['is_fallback'] and not r['driver']['fallback'] for r in active))
    frame_schema=bool(sensor_frames) and all(isinstance(f,dict) and _integer(f.get('delivery_tick'))
        and 0<=f['delivery_tick']<=total for f in sensor_frames)
    frames=[f for f in sensor_frames if f['delivery_tick']>round(config['settle_s']*hz)] if frame_schema else []
    frame_schema=frame_schema and all(_integer(f.get('acquisition_start_tick')) and _integer(f.get('acquisition_end_tick'))
        and 0<=f['acquisition_start_tick']<=f['acquisition_end_tick']<=f['delivery_tick'] for f in frames)
    covered=frame_schema and len(frames)>=10 and all(_finite(f.get('sensor_pose_error_m')) for f in frames)
    pose_values=[float(f['sensor_pose_error_m']) for f in frames if _finite(f.get('sensor_pose_error_m'))]
    gates['sensor_pose_coverage']=covered
    gates['sensor_pose']=(covered and bool(sensor_pose_errors)
        and all(_finite(v) and 0<=v<=a['sensor_pose_error_max_m'] for v in sensor_pose_errors)
        and all(0<=v<=a['sensor_pose_error_max_m'] for v in pose_values))
    accepted=[r for r in decisions if r['lidar']['status']!='stale_invalid']
    clear=[r for r in decisions if r['lidar']['status']=='clear']
    gates['fresh_driving_scans']=bool(clear) and all(_finite(r['lidar'].get('oldest_sample_age_s'))
        and 0<=r['lidar']['oldest_sample_age_s']<=a['sensor_max_age_s'] for r in accepted)
    if config['mode']!='smoke':
        gates['hold']=len(hold)==round(a['hold_s']*hz) and max(r['speed_m_s'] for r in hold)<=a['stop_speed_max_m_s']
    if config['mode']=='obstacle':
        latched=[r for r in decisions if r['lidar']['stop_latched']]
        gates['lidar_obstacle_stop']=(bool(latched) and latched[0]['progress_m']>=60
            and all(r['lidar']['stop_latched'] and r['control']['throttle']==0 and r['control']['brake']==1
                    for r in rows if r['tick']>=latched[0]['tick']))
        gates['obstacle_stop_location']=rows[-1]['progress_m']>=60 and a['minimum_barrier_gap_m']<=rows[-1]['barrier_gap_m']<=6
        gates['no_unexpected_sensor_fault']=all(r['lidar']['status']!='stale_invalid' for r in decisions)
    expected_fault_tick=first_fault_tick=None
    if config['mode']=='dropout':
        injection=round(config['dropout_at_s']*hz)
        before=[r for r in decisions if injection-round(.5*hz)<=r['tick']-1<injection]
        gates['moving_before_dropout']=len(before)==round(.5*config['control_hz']) and all(
            r['speed_m_s']>2 and r['lidar']['status']=='clear' and not r['lidar']['stop_latched'] for r in before)
        gates['no_pre_fault_sensor_failure']=all(r['lidar']['status']!='stale_invalid' for r in decisions if r['tick']-1<injection)
        eligible=[f for f in sensor_frames if frame_schema and f['delivery_tick']<=injection]
        frozen=max(eligible,key=lambda f:f['delivery_tick']) if eligible else None
        if frozen is not None and _integer(frozen.get('acquisition_start_tick')):
            last_fresh=frozen['acquisition_start_tick']+config['braking']['max_scan_age_ticks']
            expected_fault_tick=max(injection,(last_fresh//stride+1)*stride)
        faults=[r for r in decisions if r['tick']-1>=injection and r['lidar']['status']=='stale_invalid']
        first_fault_tick=faults[0]['tick']-1 if faults else None
        gates['dropout_deadline']=(expected_fault_tick is not None and first_fault_tick==expected_fault_tick
            and expected_fault_tick<=injection+config['braking']['max_scan_age_ticks']+stride)
        after=[r for r in rows if expected_fault_tick is not None and r['tick']-1>=expected_fault_tick]
        gates['dropout_braking']=bool(after) and all(r['lidar']['status']=='stale_invalid'
            and r['lidar'].get('reason')=='stale_scan' and not r['lidar']['stop_latched']
            and r['control']['throttle']==0 and r['control']['brake']==1 for r in after)
    gates={k:bool(v) for k,v in gates.items()}
    return dict(passed=all(gates.values()),gates=gates,metrics=dict(samples=len(rows),
        lane_rms_m=float(rms) if rms is not None else None,lane_max_m=float(lane_max) if lane_max is not None else None,
        final_progress_m=float(rows[-1]['progress_m']),minimum_barrier_gap_m=float(min(r['barrier_gap_m'] for r in rows)),
        final_barrier_gap_m=float(rows[-1]['barrier_gap_m']),final_speed_m_s=float(rows[-1]['speed_m_s']),
        hold_speed_max_m_s=float(max(r['speed_m_s'] for r in hold)),sensor_pose_error_max_m=max(pose_values,default=None),
        sensor_frames_after_settle=len(frames),sensor_frames_pose_matched=len(pose_values),
        maximum_accepted_scan_age_s=max((float(r['lidar']['oldest_sample_age_s']) for r in accepted
            if _finite(r['lidar'].get('oldest_sample_age_s'))),default=None),
        expected_dropout_fault_tick=expected_fault_tick,first_dropout_fault_tick=first_fault_tick,
        contact_events=len(contacts),render_clock_checks=len(render_clock_checks)))
