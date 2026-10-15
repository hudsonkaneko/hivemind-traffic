"""Predeclared gates for the separate mentor-loop fixture; no training."""
import math


PROFILES = {
    'seam3': (3.0, 20, 0),
    'seam6': (6.0, 20, 0),
    'seam35': (15.6464, 20, 0),
    'dropout35': (15.6464, 20, 0),
    'lap35': (15.6464, 225, 1),
    'three-laps35': (15.6464, 640, 3),
}


def make_config(profile='seam3', *, gui=True, paced=True, camera='follow', capture=False):
    if profile not in PROFILES or camera not in ('follow', 'overview'):
        raise ValueError('Unknown loop profile or camera')
    if any(type(v) is not bool for v in (gui, paced, capture)):
        raise ValueError('GUI, pacing and capture flags must be boolean')
    speed, drive_s, laps = PROFILES[profile]
    duration = drive_s + 2 + 8
    return dict(schema_version=1, profile=profile, seed=101,
        lane_id='MainLane1', source_directory='highway_usd/_v02',
        start_angle_rad=-0.04, physics_hz=120, control_hz=60, render_hz=20,
        target_speed_m_s=speed, settle_s=2, drive_s=drive_s, brake_s=8,
        duration_s=duration, minimum_laps=laps, dropout=profile=='dropout35',
        gui=gui, paced=paced, camera=camera, capture=capture,
        max_process_seconds=max(300, duration*4+180), max_gpu_fraction=0.9,
        drive_torque_per_front_wheel_nm=700, brake_torque_per_wheel_nm=1500,
        command_ttl_ticks=12, max_lane_rms_m=0.20, max_lane_error_m=0.50,
        max_cruise_speed_rms_error_m_s=0.25, max_cruise_speed_error_m_s=0.50,
        minimum_upright_z=0.98, stopped_speed_m_s=0.05,
        minimum_stationary_hold_s=5.0, max_stationary_drift_m=0.05,
        max_braking_distance_m=speed**2/(2*3.0)+speed*0.1+2.0,
        repeatability_position_tolerance_m=0.02,
        repeatability_speed_tolerance_m_s=0.02,
        observations='privileged PhysX chassis state + known MainLane1 geometry',
        no_sumo=True, no_lidar=True, no_training=True)


def footprint_lane_error(state, radius_m, center_xy=(0., 0.)):
    """Largest radial error of the four 4.8 x 1.8 m chassis corners."""
    x, y = state['position_m'][:2]
    yaw = state['yaw_rad']
    c, s = math.cos(yaw), math.sin(yaw)
    return max(abs(math.hypot(x+c*dx-s*dy-center_xy[0],
                             y+s*dx+c*dy-center_xy[1])-radius_m)
               for dx in (-2.4,2.4) for dy in (-0.9,0.9))


def validate_config(config):
    expected=make_config(config['profile'],gui=config['gui'],paced=config['paced'],
                         camera=config['camera'],capture=config['capture'])
    if config != expected:
        raise ValueError('Use the supervisor; resolved fixture differs from its declared profile')


def assess(rows, config, *, contact_count, route_length_m):
    """Assess post-step observations and the commands applied before that step.

    ``tick`` labels the resulting state and ``applied_tick`` labels its command;
    their difference is exactly one. Braking distance is accumulated XY travel,
    not net route progress. Holding starts at the first stopped observation and
    must remain stopped for five simulated seconds, including lateral motion.
    """
    active = [r for r in rows if r['phase'] != 'settle']
    drive = [r for r in active if r['phase'] == 'drive']
    stop = [r for r in active if r['phase'] == 'brake']
    cruise = [r for r in drive if r['sim_time_s'] >= config['settle_s']+10]
    rms = lambda xs: math.sqrt(sum(x*x for x in xs)/len(xs)) if xs else None
    lane_rms = rms([r['lane_error_m'] for r in active])
    speed_rms = rms([r['speed_m_s']-config['target_speed_m_s'] for r in cruise])
    first_stopped = next((i for i, row in enumerate(stop)
                          if row['speed_m_s'] <= config['stopped_speed_m_s']), None)
    hold = stop[first_stopped:] if first_stopped is not None else []
    stop_distance = (hold[0]['traveled_distance_m'] - drive[-1]['traveled_distance_m']
                     if hold and drive else None)
    hold_seconds = hold[-1]['sim_time_s'] - hold[0]['sim_time_s'] if hold else 0.0
    hold_drift = (max(math.dist(row['position_m'][:2], hold[0]['position_m'][:2])
                      for row in hold) if hold else None)
    # Verify the recorded distance accumulator against adjacent physical poses.
    # This prevents an angular-progress accumulator from passing as XY travel.
    travel_valid = bool(rows) and all(
        math.isfinite(row['traveled_distance_m']) and row['traveled_distance_m'] >= 0
        for row in rows) and all(math.isclose(
            b['traveled_distance_m'] - a['traveled_distance_m'],
            math.dist(a['position_m'][:2], b['position_m'][:2]),
            rel_tol=1e-8, abs_tol=1e-6) for a, b in zip(rows, rows[1:]))
    distance = drive[-1]['progress_m'] if drive else 0.
    gates = dict(
        complete=len(rows)==config['duration_s']*config['physics_hz'] and all(
            row['tick']==index+1 and row['applied_tick']==index
            and math.isclose(row['sim_time_s'], (index+1)/config['physics_hz'],
                             rel_tol=0.0, abs_tol=1e-6)
            for index, row in enumerate(rows)),
        no_rigid_contacts=contact_count==0,
        wheel_support=bool(active) and all(all(r['wheel_on_ground']) for r in active),
        upright=bool(active) and min(r['upright_z'] for r in active)>=config['minimum_upright_z'],
        chassis_height=bool(active) and all(0.5<=r['position_m'][2]<=1.5 for r in active),
        lane_rms=lane_rms is not None and lane_rms<=config['max_lane_rms_m'],
        lane_max=bool(active) and max(abs(r['lane_error_m']) for r in active)<=config['max_lane_error_m'],
        footprint_in_lane=bool(active) and all(r['footprint_error_m']<=1.85 for r in active),
        cruise_speed_rms=speed_rms is not None and speed_rms<=config['max_cruise_speed_rms_error_m_s'],
        cruise_speed_max=bool(cruise) and max(abs(r['speed_m_s']-config['target_speed_m_s']) for r in cruise)<=config['max_cruise_speed_error_m_s'],
        seam_crossed=bool(drive) and max(r['seam_crossings'] for r in drive)>=1,
        no_seam_stop=bool(drive) and all(not r['fallback'] and not r['follower_fallback']
            and not r['follower_stop_latched'] for r in drive),
        requested_laps=distance>=route_length_m*config['minimum_laps'],
        no_reverse_progress=bool(active) and all(b['progress_m']>=a['progress_m']-0.01 for a,b in zip(active,active[1:])),
        stopped=bool(stop) and all(r['speed_m_s']<=config['stopped_speed_m_s'] for r in stop[-config['physics_hz']:]),
        stationary_hold=bool(hold)
            and hold_seconds + 1e-9 >= config['minimum_stationary_hold_s']
            and all(r['speed_m_s']<=config['stopped_speed_m_s'] for r in hold)
            and hold_drift <= config['max_stationary_drift_m'],
        traveled_distance_valid=travel_valid,
        braking_distance=travel_valid and stop_distance is not None
            and 0<=stop_distance<=config['max_braking_distance_m'],
    )
    if config['dropout']:
        deadline = drive[-1]['command_expires_tick'] if drive else None
        valid_deadline = isinstance(deadline, int) and not isinstance(deadline, bool)
        expired = [r for r in stop if valid_deadline and r['applied_tick'] >= deadline]
        gates['stale_control_fallback'] = bool(expired) and valid_deadline and all(
            r['command_expires_tick'] == deadline
            and r['fallback'] == (r['applied_tick'] >= deadline)
            and (r['applied_tick'] < deadline or (r['throttle'] == 0 and r['brake'] == 1))
            for r in stop) and expired[0]['applied_tick'] == deadline
    else:
        gates['explicit_stop_control']=bool(stop) and all(not r['fallback']
            and not r['follower_fallback'] and r['follower_stop_latched']
            and r['throttle']==0 and r['brake']==1 for r in stop)
    return dict(passed=all(gates.values()), gates=gates, distance_during_drive_m=distance,
        completed_laps=int(max(0,distance)//route_length_m), lane_rms_m=lane_rms,
        max_lane_error_m=max((abs(r['lane_error_m']) for r in active),default=None),
        cruise_speed_rms_error_m_s=speed_rms, braking_distance_m=stop_distance,
        first_stopped_tick=hold[0]['tick'] if hold else None,
        stationary_hold_s=hold_seconds, stationary_hold_drift_m=hold_drift,
        command_expiry_tick=deadline if config['dropout'] else None,
        maximum_speed_m_s=max((r['speed_m_s'] for r in rows),default=None),
        final_speed_m_s=rows[-1]['speed_m_s'] if rows else None,
        seam_crossings=max((r['seam_crossings'] for r in rows),default=0))
