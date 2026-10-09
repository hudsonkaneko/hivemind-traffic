"""Bounded presentation fixtures, distinct from traffic-policy experiments."""
import math

VERSIONS = ('v01', 'v02', 'v03')
MODES = ('showcase', 'blocked', 'dropout', 'contact-check')


def make_config(version='v03', *, mode='showcase', gui=True, paced=True,
                camera='follow', capture=False, drive_s=None):
    if version not in VERSIONS or mode not in MODES or camera not in ('follow','traffic','overview'):
        raise ValueError('Unknown showcase version, mode or camera')
    if any(type(v) is not bool for v in (gui,paced,capture)):
        raise ValueError('Flags must be boolean')
    duration = drive_s if drive_s is not None else (30 if version=='v01' else 90 if version=='v02' else 240)
    if type(duration) is not int or not 15 <= duration <= 1800:
        raise ValueError('Drive duration must be an integer in [15,1800] s')
    return dict(schema_version=1,version=version,mode=mode,seed=101,
        vehicle_directory='vehicles/sim_ready/americano-i7-ev/v07',
        source_directory='highway_usd/_v02', initial_lane=1,
        physics_hz=120,control_hz=60,planner_hz=10,render_hz=20,
        settle_s=2,drive_s=duration,brake_s=10,duration_s=duration+12,
        target_speed_m_s=6.0 if version=='v01' else 15.6464,
        drive_torque_nm=700.,brake_torque_nm=1500.,command_ttl_ticks=12,
        gui=gui,paced=paced,camera=camera,capture=capture,
        max_process_seconds=max(300,(duration+12)*4+180),max_gpu_fraction=.90,
        min_clearance_m=.6,max_background_pose_error_m=.02,
        min_upright_z=.98,max_speed_m_s=17.,stop_speed_m_s=.05,
        min_hold_s=5.,max_hold_drift_m=.05,
        minimum_passes=3 if version!='v01' and mode=='showcase' and duration>=90 else 0,
        minimum_lane_changes=2 if version!='v01' and mode=='showcase' and duration>=90 else 0,
        observations='privileged timestamped simulator object tracks and chassis state; known circular map',
        no_sumo=True,no_lidar=True,no_training=True)


def fleet_specs(config):
    if config['mode']=='blocked':
        entries=[(lane,65.,6.7056) for lane in range(4)]
    elif config['mode']=='contact-check':
        entries=[(1,16.,0.)]
    else:
        entries=[(1,80.,6.7056),(0,165.,6.7056),(1,255.,6.7056),
                 (2,345.,6.7056),(1,435.,6.7056),(0,525.,6.7056),
                 (1,615.,6.7056),(2,705.,6.7056),(3,795.,7.15264),
                 (2,1000.,7.59968),(0,1250.,8.04672),(1,1500.,8.49376)]
    return [dict(vehicle_id=f'background_{i:03d}',lane=lane,station_m=s,speed_m_s=v)
            for i,(lane,s,v) in enumerate(entries)]


def validate_config(cfg):
    expected=make_config(cfg['version'],mode=cfg['mode'],gui=cfg['gui'],paced=cfg['paced'],
        camera=cfg['camera'],capture=cfg['capture'],drive_s=cfg['drive_s'])
    if cfg!=expected:raise ValueError('Resolved showcase settings differ from declared profile')


def assess(rows,config,contact_count,passes,lane_changes):
    # Validate each sample before reductions. min/max can silently ignore a NaN
    # in a non-first element, and all([]) is not evidence for four tire contacts.
    numeric_fields=('sim_time_s','speed_m_s','traveled_distance_m','clearance_m',
                    'upright_z','background_pose_error_m','throttle','brake')
    tick_fields=('tick','applied_tick','command_expires_tick')
    bool_fields=('overlap','footprint_in_road','fallback')
    finite=lambda value: (type(value) in (int,float) and math.isfinite(value))
    def valid_sample(row):
        if not isinstance(row,dict):return False
        if row.get('phase') not in ('settle','drive','brake'):return False
        if not all(finite(row.get(name)) for name in numeric_fields):return False
        if not all(type(row.get(name)) is int and row[name]>=0 for name in tick_fields):return False
        if not all(type(row.get(name)) is bool for name in bool_fields):return False
        position=row.get('position_m');wheels=row.get('wheel_on_ground')
        return (isinstance(position,(list,tuple)) and len(position)==3 and all(finite(v) for v in position)
            and isinstance(wheels,(list,tuple)) and len(wheels)==4 and all(type(v) is bool for v in wheels))
    valid_trace=isinstance(rows,(list,tuple)) and bool(rows) and all(valid_sample(row) for row in rows)
    valid_counts=(type(contact_count) is int and contact_count>=0 and type(lane_changes) is int and lane_changes>=0
        and isinstance(passes,(list,tuple,set,frozenset)) and all(isinstance(v,str) and v for v in passes))
    if not valid_trace or not valid_counts:
        return dict(passed=False,gates=dict(trace_schema=valid_trace,summary_schema=valid_counts),
            contact_report_count=contact_count,min_clearance_m=None,passed_vehicle_ids=[],completed_passes=0,
            completed_lane_changes=lane_changes,braking_distance_m=None,braking_limit_m=None,
            stationary_hold_s=0.,stationary_drift_m=None,maximum_speed_m_s=None,distance_m=None)
    passes=set(passes)
    active=[r for r in rows if r['phase']!='settle']
    drive=[r for r in rows if r['phase']=='drive']
    stop=[r for r in rows if r['phase']=='brake']
    stopped=next((i for i,r in enumerate(stop) if r['speed_m_s']<=config['stop_speed_m_s']),None)
    hold=stop[stopped:] if stopped is not None else []
    brake_distance=hold[0]['traveled_distance_m']-drive[-1]['traveled_distance_m'] if hold and drive else None
    brake_limit=drive[-1]['speed_m_s']**2/6+drive[-1]['speed_m_s']*.1+2 if drive else 0
    hold_s=hold[-1]['sim_time_s']-hold[0]['sim_time_s'] if hold else 0
    drift=max((math.dist(r['position_m'][:2],hold[0]['position_m'][:2]) for r in hold),default=None)
    positive=config['mode']=='contact-check'
    clock=all(math.isclose(r['sim_time_s'],r['tick']/config['physics_hz'],rel_tol=0.,abs_tol=1e-6)
              for r in rows)
    def expected_phase(tick):
        if tick<config['settle_s']*config['physics_hz']:return 'settle'
        if tick<(config['settle_s']+config['drive_s'])*config['physics_hz']:return 'drive'
        return 'brake'
    phase_contract=all(r['phase']==expected_phase(r['applied_tick']) for r in rows)
    actual_travel=(all(r['traveled_distance_m']>=0 for r in rows) and all(math.isclose(
        b['traveled_distance_m']-a['traveled_distance_m'],math.dist(a['position_m'][:2],b['position_m'][:2]),
        rel_tol=1e-8,abs_tol=1e-6) for a,b in zip(rows,rows[1:])))
    gates=dict(trace_schema=True,summary_schema=True,simulation_clock=clock,phase_contract=phase_contract,
        traveled_distance_matches_pose=actual_travel,
        control_ranges=all(0<=r['throttle']<=1 and 0<=r['brake']<=1 for r in rows),
        complete=len(rows)==config['duration_s']*config['physics_hz'],
        contiguous_ticks=all(r['tick']==i+1 and r['applied_tick']==i for i,r in enumerate(rows)),
        no_rigid_contacts=contact_count==0,
        clearance=bool(active) and min(r['clearance_m'] for r in active)>=config['min_clearance_m'],
        no_overlap=bool(active) and not any(r['overlap'] for r in active),
        road_containment=bool(active) and all(r['footprint_in_road'] for r in active),
        wheel_support=bool(active) and all(all(r['wheel_on_ground']) for r in active),
        upright=bool(active) and min(r['upright_z'] for r in active)>=config['min_upright_z'],
        speed_bounded=bool(active) and all(0<=r['speed_m_s']<=config['max_speed_m_s'] for r in active),
        background_pose=bool(rows) and max(r['background_pose_error_m'] for r in rows)<=config['max_background_pose_error_m'],
        requested_passes=len(passes)>=config['minimum_passes'],
        requested_lane_changes=lane_changes>=config['minimum_lane_changes'],
        braking=brake_distance is not None and 0<=brake_distance<=brake_limit,
        stationary_hold=bool(hold) and hold_s>=config['min_hold_s'] and drift<=config['max_hold_drift_m']
            and all(r['speed_m_s']<=config['stop_speed_m_s'] for r in hold))
    if config['mode']=='blocked':
        tail=[r for r in drive if r['sim_time_s']>config['settle_s']+config['drive_s']-10]
        gates['blocked_following']=bool(tail) and lane_changes==0 and all(r['speed_m_s']<=7.4 for r in tail)
    if config['mode']=='dropout':
        expired=[r for r in stop if r['applied_tick']>=drive[-1]['command_expires_tick']] if drive else []
        gates['command_loss_braking']=bool(expired) and all(r['fallback'] and r['throttle']==0 and r['brake']==1 for r in expired)
    if positive:
        # A contact probe intentionally stops on its first contact, but its
        # clock, measurements and native background checks must still be valid.
        gates={name:gates[name] for name in ('trace_schema','summary_schema','simulation_clock',
            'phase_contract','traveled_distance_matches_pose','control_ranges','contiguous_ticks','background_pose')}
        gates['contact_detected']=contact_count>0
    return dict(passed=all(gates.values()),gates=gates,contact_report_count=contact_count,
        min_clearance_m=min((r['clearance_m'] for r in active),default=None),
        passed_vehicle_ids=sorted(passes),completed_passes=len(passes),completed_lane_changes=lane_changes,
        braking_distance_m=brake_distance,braking_limit_m=brake_limit,stationary_hold_s=hold_s,
        stationary_drift_m=drift,maximum_speed_m_s=max((r['speed_m_s'] for r in rows),default=None),
        distance_m=rows[-1]['traveled_distance_m'] if rows else 0.)
