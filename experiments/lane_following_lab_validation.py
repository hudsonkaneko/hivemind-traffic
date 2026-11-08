"""Independent evidence checks for the frozen one-car Lab lane task.

This evaluator never calls the task's observation, reward, guard or termination
functions. It derives those claims from the recorded 120 Hz native states.
Shared analytic map projection is already covered by its own geometry tests.
"""
from __future__ import annotations

import math
from numbers import Real

from traffic.lane_geometry import LaneRoute
from usd.physical_scene import PATHS

PHYSICS_HZ = 120
CONTROL_TICKS = 2
SETTLE_TICKS = 240
DEADLINE_TICKS = 4200
STOP_STATION_M = 60.0
OBSERVATIONS = 22
SUPPORTED_REASONS = {'success', 'time_limit', 'lane_departure', 'rigid_contact'}


def _numeric(value):
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


def _integer(value):
    return type(value) is int and value >= 0


def _finite(value):
    if isinstance(value, dict):
        return all(_finite(v) for v in value.values())
    if isinstance(value, (list, tuple)):
        return all(_finite(v) for v in value)
    return not isinstance(value, Real) or isinstance(value, bool) or math.isfinite(value)


def _vector(value, length):
    return isinstance(value, (list, tuple)) and len(value) == length and all(_numeric(v) for v in value)


def _angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def _close(a, b, absolute=1e-5, relative=1e-6):
    if not _numeric(a) or not _numeric(b):
        return False
    return abs(a-b) <= absolute + relative*max(abs(a), abs(b))


def _same_vector(a, b, length=OBSERVATIONS):
    return _vector(a, length) and _vector(b, length) and all(_close(x, y) for x, y in zip(a, b))


def _state_schema(state):
    if not isinstance(state, dict) or not all(_vector(state.get(k), n) for k, n in
            (('position_m', 3), ('quaternion_xyzw', 4), ('velocity_m_s', 3))):
        return False
    if not all(_numeric(state.get(k)) for k in ('yaw_rad', 'upright_z', 'speed_m_s')):
        return False
    wheels = state.get('wheel_on_ground')
    if not isinstance(wheels, (list, tuple)) or len(wheels) != 4 or any(type(v) is not bool for v in wheels):
        return False
    qx, qy, qz, qw = state['quaternion_xyzw']
    norm = math.sqrt(qx*qx+qy*qy+qz*qz+qw*qw)
    yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
    upright = 1-2*(qx*qx+qy*qy)
    return (abs(norm-1) <= 1e-3 and abs(_angle(yaw-state['yaw_rad'])) <= 1e-5
            and abs(upright-state['upright_z']) <= 1e-5
            and abs(math.hypot(*state['velocity_m_s'][:2])-state['speed_m_s']) <= 1e-5)


def _raw_states_match(a, b):
    return all(_same_vector(a[k], b[k], n) for k, n in
               (('position_m', 3), ('quaternion_xyzw', 4), ('velocity_m_s', 3))) and all(
        _close(a[k], b[k]) for k in ('speed_m_s', 'upright_z', 'yaw_rad')) and a['wheel_on_ground'] == b['wheel_on_ground']


def _geometry(state, route):
    projection = route.project(*state['position_m'][:2])
    tangent = projection.point.yaw_rad
    c, s = math.cos(tangent), math.sin(tangent)
    vx, vy = state['velocity_m_s'][:2]
    # Independent matrix rotation of all eight centered 4.8 x 1.8 x 1.4 corners.
    q = state['quaternion_xyzw']
    norm = math.sqrt(sum(v*v for v in q))
    x, y, z, w = [v/norm for v in q]
    r0 = (1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w))
    r1 = (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w))
    corners = [(sum(a*b for a, b in zip(r0, corner)), sum(a*b for a, b in zip(r1, corner)))
               for corner in ((lx, ly, lz) for lx in (-2.4, 2.4) for ly in (-.9, .9) for lz in (-.7, .7))]
    radius = max(math.hypot(*corner) for corner in corners)
    curvature = max(abs(seg.curvature_rad_m) for seg in route.segments)
    denominator = 1-curvature*(abs(projection.lateral_error_m)+radius)
    footprint = math.inf if denominator <= 0 else max(
        abs(projection.lateral_error_m-s*cx+c*cy) for cx, cy in corners
    ) + curvature*radius*radius/(2*denominator)
    return dict(progress_m=projection.s_m, remaining_m=STOP_STATION_M-projection.s_m,
                lateral_error_m=projection.lateral_error_m,
                heading_error_rad=_angle(state['yaw_rad']-tangent),
                forward_speed_m_s=c*vx+s*vy, lateral_speed_m_s=-s*vx+c*vy,
                planar_speed_m_s=math.hypot(vx, vy), footprint_lateral_bound_m=footprint)


def _target(geometry, tick):
    return 0.0 if tick < SETTLE_TICKS else min(3.0, math.sqrt(3*max(0.0, geometry['remaining_m']-.2)))


def _observation(state, geometry, control, tick, hold_ticks, route):
    return [geometry['progress_m'], geometry['remaining_m'], geometry['lateral_error_m'],
            math.sin(geometry['heading_error_rad']), math.cos(geometry['heading_error_rad']),
            geometry['forward_speed_m_s'], geometry['lateral_speed_m_s'], geometry['planar_speed_m_s'],
            state['velocity_m_s'][2], state['position_m'][2], state['upright_z'],
            *(route.evaluate(geometry['progress_m']+offset).curvature_rad_m for offset in (0, 3, 6)),
            3.0, _target(geometry, tick), control['steering_rad'], control['throttle'], control['brake'],
            min(1.0, tick/DEADLINE_TICKS), float(tick < SETTLE_TICKS), min(1.0, hold_ticks/120)]


def _control_schema(control):
    if not isinstance(control, dict) or not all(_numeric(control.get(k)) for k in ('steering_rad', 'throttle', 'brake')):
        return False
    return (abs(control['steering_rad']) <= .500001 and 0 <= control['throttle'] <= 1
            and 0 <= control['brake'] <= 1 and not (control['brake'] > 0 and control['throttle'] > 0)
            and type(control.get('is_fallback')) is bool)


def _member(path, root):
    return path == root or path.startswith(root+'/')


def _contact_schema(event):
    if not isinstance(event, dict) or not _integer(event.get('contact_count')):
        return False
    if event.get('event_type') not in ('found', 'persist') or not _integer(event.get('tick')):
        return False
    if not all(isinstance(event.get(k), str) and event[k].startswith('/')
               for k in ('actor0', 'actor1', 'collider0', 'collider1')):
        return False
    if not any(_member(event[k], PATHS.chassis) for k in ('actor0', 'actor1', 'collider0', 'collider1')):
        return False
    points = event.get('contacts')
    return (isinstance(points, list) and len(points) == event['contact_count'] and all(
        isinstance(p, dict) and all(_vector(p.get(k), 3) for k in ('position', 'normal', 'impulse'))
        and _numeric(p.get('separation_m')) for p in points))


def _reset_checks(record, route):
    schema = (isinstance(record, dict) and isinstance(record.get('reset_spec'), dict)
              and _state_schema(record.get('physics_state')) and _vector(record.get('observation'), OBSERVATIONS)
              and _vector(record.get('wheel_rotation_speed_rad_s'), 4))
    if not schema:
        return {'schema': False}, None
    spec, state = record['reset_spec'], record['physics_state']
    spec_schema = (spec.get('route_id') == route.route_id and _integer(spec.get('seed'))
                   and spec['seed'] <= 2**32-1 and _integer(spec.get('episode_index'))
                   and all(_numeric(spec.get(k)) for k in ('station_m', 'lateral_offset_m', 'yaw_error_rad', 'x_m', 'y_m', 'yaw_rad')))
    if not spec_schema:
        return {'schema': False}, None
    p = route.evaluate(0)
    expected_xy = (p.position_xy[0]-math.sin(p.yaw_rad)*spec['lateral_offset_m'],
                   p.position_xy[1]+math.cos(p.yaw_rad)*spec['lateral_offset_m'])
    geometry = _geometry(state, route)
    expected_observation = _observation(state, geometry, dict(steering_rad=0, throttle=0, brake=1), 0, 0, route)
    checks = dict(schema=True, at_tick_zero=record.get('episode_tick') == 0 and type(record.get('episode_tick')) is int,
                  bounded_spec=spec['station_m'] == 0 and abs(spec['lateral_offset_m']) <= .15
                  and abs(spec['yaw_error_rad']) <= .035,
                  declared_pose=math.dist((spec['x_m'], spec['y_m']), expected_xy) <= 1e-9
                  and abs(_angle(spec['yaw_rad']-p.yaw_rad-spec['yaw_error_rad'])) <= 1e-9,
                  native_position=math.dist(state['position_m'][:2], expected_xy) <= .001,
                  native_yaw=abs(_angle(state['yaw_rad']-spec['yaw_rad'])) <= .001,
                  native_velocity=math.sqrt(sum(v*v for v in state['velocity_m_s'])) <= .001,
                  wheels_at_rest=max(abs(v) for v in record['wheel_rotation_speed_rad_s']) <= .001,
                  pose_safe=0 <= state['position_m'][2] <= 2 and state['upright_z'] >= math.cos(math.radians(5))
                  and geometry['footprint_lateral_bound_m'] <= route.width_m/2,
                  observation=_same_vector(record['observation'], expected_observation))
    if 'returned_observation' in record:
        checks['returned_observation'] = _same_vector(record['returned_observation'], expected_observation)
    return checks, geometry


def _guard(state, geometry, sample, unsupported_ticks, fallback):
    if sample['contacts']:
        return 'rigid_contact'
    if fallback:
        return 'command_fallback'
    if unsupported_ticks > 12:
        return 'wheel_support_loss'
    if state['upright_z'] < math.cos(math.radians(5)):
        return 'excessive_tilt'
    if not 0 <= state['position_m'][2] <= 2:
        return 'unsafe_height'
    if geometry['footprint_lateral_bound_m'] > 1.8:
        return 'lane_departure'
    if abs(geometry['heading_error_rad']) > 1:
        return 'heading_outside_envelope'
    if geometry['planar_speed_m_s'] > 4:
        return 'overspeed'
    if geometry['forward_speed_m_s'] < -.2:
        return 'reverse_motion'
    if geometry['progress_m'] < -1:
        return 'before_route_start'
    if geometry['progress_m'] > 60.5:
        return 'endpoint_overshoot'
    return None


def assess_episode(records, reset_record, route, acceptance, *, expected_reason='success'):
    """Check a complete episode, returning finite JSON evidence even on bad data.

    Expected fault episodes must exhibit that physical mechanism and terminate
    promptly. Their intentionally violated safety metric is not mistaken for a
    successful-driving requirement; unrelated physical failures still fail.
    """
    checks, metrics = {}, {}
    def report():
        return dict(passed=bool(checks) and all(checks.values()), checks=checks, metrics=metrics,
                    expected_reason=expected_reason)
    checks['supported_expectation'] = expected_reason in SUPPORTED_REASONS
    checks['records_present'] = isinstance(records, list) and bool(records)
    checks['finite_telemetry'] = _finite(records) and _finite(reset_record)
    checks['route_contract'] = isinstance(route, LaneRoute) and route.width_m == 3.6 and route.length_m >= 60
    if not all(checks.values()):
        return report()
    required = ('minimum_progress_m', 'endpoint_error_max_m', 'stopped_speed_max_m_s', 'hold_drift_max_m',
                'driving_lateral_rms_max_m', 'driving_lateral_max_m', 'post_settle_wheel_support_fraction_min',
                'full_episode_upright_z_min', 'maximum_unsupported_gap_s', 'rigid_contacts_max')
    checks['acceptance_schema'] = isinstance(acceptance, dict) and all(_numeric(acceptance.get(k)) and acceptance[k] >= 0 for k in required)
    checks['acceptance_requires_containment'] = isinstance(acceptance, dict) and acceptance.get('full_footprint_containment_required') is True
    reset_checks, reset_geometry = _reset_checks(reset_record, route)
    checks.update({f'reset_{k}': v for k, v in reset_checks.items()})
    if not all(checks.values()):
        return report()
    schema = all(isinstance(row, dict) and _integer(row.get('tick'))
                 and row.get('episode_tick') == row['tick'] and _state_schema(row.get('physics_state'))
                 and _control_schema(row.get('control')) and isinstance(row.get('physics_samples'), list)
                 and isinstance(row.get('contacts'), list) and isinstance(row.get('reward_components'), dict)
                 and isinstance(row.get('route_state'), dict)
                 and all(_vector(row.get(k), OBSERVATIONS) for k in ('observation', 'returned_observation', 'final_observation'))
                 and all(_numeric(row.get(k)) for k in ('elapsed_s', 'reward', 'returned_reward'))
                 and all(type(row.get(k)) is bool for k in ('terminated', 'truncated', 'success', 'returned_terminated',
                    'returned_truncated', 'returned_done', 'sb3_time_limit_truncated'))
                 and all(isinstance(sample, dict) and _integer(sample.get('tick')) and _state_schema(sample.get('physics_state'))
                         and isinstance(sample.get('contacts'), list) and _integer(sample.get('contact_count'))
                         and sample['contact_count'] == len(sample['contacts'])
                         and all(_contact_schema(event) for event in sample['contacts'])
                         for sample in row['physics_samples']) for row in records)
    checks['record_schema'] = schema
    if not schema:
        return report()
    samples = [sample for row in records for sample in row['physics_samples']]
    terminal_tick = records[-1]['tick']
    checks['control_tick_order'] = [row['tick'] for row in records] == list(range(2, terminal_tick+1, 2))
    checks['physics_tick_order'] = [sample['tick'] for sample in samples] == list(range(1, terminal_tick+1))
    checks['episode_clock'] = terminal_tick <= DEADLINE_TICKS and all(_close(row['elapsed_s'], row['tick']/120, 1e-10, 0) for row in records)
    checks['two_substeps_per_record'] = all([s['tick'] for s in row['physics_samples']] == [row['tick']-1, row['tick']] for row in records)
    checks['native_control_state_agreement'] = all(len(row['physics_samples']) == 2
        and _raw_states_match(row['physics_state'], row['physics_samples'][-1]['physics_state']) for row in records)
    checks['contact_aggregation'] = all(row['contacts'] == [e for sample in row['physics_samples'] for e in sample['contacts']] for row in records)
    checks['episode_identity'] = (_integer(reset_record.get('episode_serial')) and all(
        row.get('episode_serial') == reset_record['episode_serial'] for row in records))
    if not all(checks.values()):
        return report()
    geometries = [_geometry(sample['physics_state'], route) for sample in samples]
    checks['finite_geometry'] = all(_finite(g) for g in geometries)
    if not checks['finite_geometry']:
        return report()
    unsupported = longest = supported = active_count = 0
    first_failure = None
    raw_guards, contacts = [], []
    for index, (sample, geometry) in enumerate(zip(samples, geometries)):
        state, tick = sample['physics_state'], sample['tick']
        if tick > SETTLE_TICKS:
            active_count += 1
            grounded = all(state['wheel_on_ground'])
            supported += grounded
            unsupported = 0 if grounded else unsupported+1
        else:
            unsupported = 0
        longest = max(longest, unsupported)
        reason = _guard(state, geometry, sample, unsupported, records[index//2]['control']['is_fallback'])
        raw_guards.append(reason)
        if first_failure is None and reason:
            first_failure = (tick, reason)
        contacts.extend(sample['contacts'])
    driving = [g for sample, g in zip(samples, geometries) if sample['tick'] > SETTLE_TICKS]
    lateral = [g['lateral_error_m'] for g in driving]
    metrics.update(physics_samples=len(samples), control_samples=len(records), terminal_tick=terminal_tick,
        simulated_seconds=terminal_tick/120, progress_m=geometries[-1]['progress_m']-reset_geometry['progress_m'],
        final_station_m=geometries[-1]['progress_m'], endpoint_error_m=abs(geometries[-1]['remaining_m']),
        driving_lateral_rms_m=math.sqrt(sum(x*x for x in lateral)/len(lateral)) if lateral else None,
        driving_lateral_max_m=max(map(abs, lateral)) if lateral else None,
        footprint_lateral_bound_max_m=max(g['footprint_lateral_bound_m'] for g in geometries),
        minimum_upright_z=min(s['physics_state']['upright_z'] for s in samples),
        post_settle_wheel_support_fraction=supported/active_count if active_count else None,
        longest_unsupported_gap_s=longest/120, rigid_contact_events=len(contacts),
        contact_points=sum(event['contact_count'] for event in contacts), first_physical_failure=first_failure)
    checks['guard_evidence'] = all(sample.get('guard_reason') == reason for sample, reason in zip(samples, raw_guards))
    checks['prompt_failure_terminal'] = first_failure is None or terminal_tick == 2*math.ceil(first_failure[0]/2)
    checks['first_failure_record'] = all(row.get('first_guard_failure') is None for row in records) if first_failure is None else (
        isinstance(records[-1].get('first_guard_failure'), dict)
        and records[-1]['first_guard_failure'].get('tick') == first_failure[0]
        and records[-1]['first_guard_failure'].get('reason') == first_failure[1]
        and _state_schema(records[-1]['first_guard_failure'].get('physics_state'))
        and _raw_states_match(records[-1]['first_guard_failure']['physics_state'], samples[first_failure[0]-1]['physics_state']))
    # Recompute control-rate hold, observations and rewards from native states.
    previous_geometry, previous_state = reset_geometry, reset_record['physics_state']
    previous_control = dict(steering_rad=0.0, throttle=0.0, brake=1.0)
    hold = 0
    observations_ok = rewards_ok = decisions_ok = routes_ok = controls_ok = True
    for index, row in enumerate(records):
        tick, geometry, state, control = row['tick'], geometries[row['tick']-1], row['physics_state'], row['control']
        failure = first_failure[1] if first_failure and first_failure[0] <= tick else None
        discontinuity = math.dist(previous_state['position_m'][:2], state['position_m'][:2]) > 4/60+.005
        if discontinuity and failure is None:
            failure = 'state_discontinuity'
        drive = tick-2 >= SETTLE_TICKS
        def goal(g):
            return abs(g['remaining_m']) <= .5 and g['progress_m']-reset_geometry['progress_m'] >= 59.5 and g['planar_speed_m_s'] <= .05
        hold = hold+2 if drive and goal(geometry) and goal(previous_geometry) and failure is None else 0
        success = failure is None and hold >= 120
        terminated = failure is not None or success
        truncated = not terminated and tick >= DEADLINE_TICKS
        reason = failure or ('success' if success else 'time_limit' if truncated else 'driving' if drive else 'settling')
        decisions_ok &= (row['terminated'] == terminated and row['truncated'] == truncated and row['success'] == success
                         and row.get('reason') == reason and (not terminated and not truncated or index == len(records)-1))
        target = _target(geometry, tick)
        routes_ok &= (set(row['route_state']) == {*geometry, 'target_speed_m_s'}
                      and all(_close(row['route_state'].get(k), v) for k, v in geometry.items())
                      and _close(row['route_state'].get('target_speed_m_s'), target))
        expected_obs = _observation(state, geometry, control, tick, hold, route)
        observations_ok &= _same_vector(row['observation'], expected_obs) and _same_vector(row['final_observation'], expected_obs)
        components = dict(progress=0.0, lateral=0.0, heading=0.0, speed=0.0, steering_rate=0.0, time=0.0, terminal=0.0)
        if drive:
            components.update(progress=0 if discontinuity else (geometry['progress_m']-previous_geometry['progress_m'])/3,
                lateral=-(geometry['lateral_error_m']/.5)**2/60,
                heading=-.25*(geometry['heading_error_rad']/.25)**2/60,
                speed=-.25*((geometry['forward_speed_m_s']-target)/3)**2/60,
                steering_rate=-.01*((control['steering_rad']-previous_control['steering_rad'])*120)**2/60,
                time=-.05/60)
        components['terminal'] = 10.0 if success else -10.0 if terminated else 0.0
        rewards_ok &= (set(row['reward_components']) == set(components)
                       and all(_close(row['reward_components'].get(k), v, 1e-9, 1e-7) for k, v in components.items())
                       and _close(row['reward'], sum(components.values()), 1e-9, 1e-7)
                       and _close(row['returned_reward'], row['reward']))
        controls_ok &= abs(control['steering_rad']-previous_control['steering_rad']) <= .5/60+1e-6
        if not drive:
            controls_ok &= control['steering_rad'] == 0 and control['throttle'] == 0 and control['brake'] == 1
        previous_geometry, previous_state, previous_control = geometry, state, control
    checks.update(observations_recomputed=bool(observations_ok), route_state_recomputed=bool(routes_ok),
                  reward_components_recomputed=bool(rewards_ok), terminal_decisions_recomputed=bool(decisions_ok),
                  controls_bounded_and_settling_held=bool(controls_ok))
    final = records[-1]
    checks['terminal_reason'] = final.get('reason') == expected_reason
    checks['terminal_flags'] = final['truncated'] and not final['terminated'] if expected_reason == 'time_limit' else final['terminated'] and not final['truncated']
    checks['returned_flags'] = all(row['returned_terminated'] == row['terminated'] and row['returned_truncated'] == row['truncated'] for row in records)
    checks['returned_done'] = all(row['returned_done'] == (row['terminated'] or row['truncated']) for row in records)
    checks['sb3_time_limit'] = all(row['sb3_time_limit_truncated'] == (row['truncated'] and not row['terminated']) for row in records)
    checks['nonterminal_returned_observations'] = all(_same_vector(row['returned_observation'], row['observation'])
                                                    and row.get('terminal_observation') is None and row.get('auto_reset') is None for row in records[:-1])
    checks['terminal_snapshot'] = _same_vector(final.get('terminal_observation'), final['observation'])
    auto_checks, _ = _reset_checks(final.get('auto_reset'), route)
    checks.update({f'auto_reset_{k}': v for k, v in auto_checks.items()})
    auto = final.get('auto_reset')
    checks['auto_reset_identity'] = (isinstance(auto, dict) and auto.get('episode_serial') == reset_record['episode_serial']+1
                                     and _integer(reset_record.get('reset_count')) and auto.get('reset_count') == reset_record['reset_count']+1)
    checks['terminal_return_is_reset'] = isinstance(auto, dict) and _same_vector(final['returned_observation'], auto.get('observation'))
    checks['sb3_terminal_observation'] = all(
        _same_vector(row.get('sb3_terminal_observation'), row['observation']) if index == len(records)-1
        else 'sb3_terminal_observation' not in row
        for index, row in enumerate(records))
    # One full second contains120intervals and121measured boundary states.
    hold_samples = samples[-121:]
    hold_geometry = geometries[-121:]
    hold_drift = max(math.dist(hold_samples[0]['physics_state']['position_m'][:2], sample['physics_state']['position_m'][:2])
                     for sample in hold_samples)
    metrics.update(hold_samples=len(hold_samples), hold_duration_s=(hold_samples[-1]['tick']-hold_samples[0]['tick'])/120,
        hold_drift_m=hold_drift,
        hold_endpoint_error_max_m=max(abs(g['remaining_m']) for g in hold_geometry),
        hold_speed_max_m_s=max(g['planar_speed_m_s'] for g in hold_geometry),
        total_reward=sum(row['reward'] for row in records))
    # Healthy pose/support/motion and no unrelated faults are required for all cases.
    checks['upright'] = metrics['minimum_upright_z'] >= acceptance['full_episode_upright_z_min']
    checks['native_heights'] = all(0 <= sample['physics_state']['position_m'][2] <= 2 for sample in samples)
    checks['support_fraction'] = (supported/active_count >= acceptance['post_settle_wheel_support_fraction_min']
                                  if active_count else expected_reason in ('lane_departure', 'rigid_contact'))
    checks['unsupported_gap'] = longest/120 <= acceptance['maximum_unsupported_gap_s']
    checks['no_rigid_contacts'] = expected_reason == 'rigid_contact' or len(contacts) <= acceptance['rigid_contacts_max']
    checks['footprint_containment'] = expected_reason == 'lane_departure' or metrics['footprint_lateral_bound_max_m'] <= route.width_m/2
    checks['other_motion_envelopes'] = all(g['planar_speed_m_s'] <= 4 and g['forward_speed_m_s'] >= -.2
        and abs(g['heading_error_rad']) <= 1 and -1 <= g['progress_m'] <= 60.5 for g in geometries)
    if expected_reason == 'success':
        checks.update(no_physical_failure=first_failure is None,
            forward_progress=metrics['progress_m'] >= acceptance['minimum_progress_m'],
            lateral_rms=bool(lateral) and metrics['driving_lateral_rms_m'] <= acceptance['driving_lateral_rms_max_m'],
            lateral_max=bool(lateral) and metrics['driving_lateral_max_m'] <= acceptance['driving_lateral_max_m'],
            stopped_hold=len(hold_samples) == 121 and hold_samples[-1]['tick']-hold_samples[0]['tick'] == 120
            and metrics['hold_speed_max_m_s'] <= acceptance['stopped_speed_max_m_s']
            and metrics['hold_endpoint_error_max_m'] <= acceptance['endpoint_error_max_m']
            and hold_drift <= acceptance['hold_drift_max_m'])
    elif expected_reason == 'time_limit':
        checks.update(no_physical_failure=first_failure is None, exact_timeout=terminal_tick == DEADLINE_TICKS)
    else:
        checks['expected_physical_mechanism'] = first_failure is not None and first_failure[1] == expected_reason
        if expected_reason == 'rigid_contact':
            checks['decoded_nonempty_contact'] = any(event['contact_count'] > 0 for event in contacts)
            checks['chassis_barrier_pair'] = any(event['contact_count'] > 0 and (
                any(_member(event[k], PATHS.chassis) for k in ('actor0', 'collider0'))
                and any(_member(event[k], PATHS.barrier) for k in ('actor1', 'collider1'))
                or any(_member(event[k], PATHS.chassis) for k in ('actor1', 'collider1'))
                and any(_member(event[k], PATHS.barrier) for k in ('actor0', 'collider0'))) for event in contacts)
    return report()


def compare_episode_traces(records_a, records_b, acceptance):
    """Compare every raw physical tick, rejecting missing data and early resets."""
    checks = dict(records_present=isinstance(records_a, list) and isinstance(records_b, list)
                  and bool(records_a) and bool(records_b), finite_telemetry=_finite(records_a) and _finite(records_b))
    metrics = {}
    def report():
        return dict(passed=all(checks.values()), checks=checks, metrics=metrics)
    if not all(checks.values()):
        return report()
    try:
        a = [sample for row in records_a for sample in row['physics_samples']]
        b = [sample for row in records_b for sample in row['physics_samples']]
        checks['matched_control_ticks'] = [r['tick'] for r in records_a] == [r['tick'] for r in records_b]
        checks['complete_physics_ticks'] = bool(a) and len(a) == len(b) and [r['tick'] for r in a] == [r['tick'] for r in b] == list(range(1, a[-1]['tick']+1))
        checks['complete_control_ticks'] = bool(a) and [r['tick'] for r in records_a] == list(range(2, a[-1]['tick']+1, 2))
        checks['control_physics_alignment'] = all([s['tick'] for s in row['physics_samples']] == [row['tick']-1, row['tick']]
            for row in records_a+records_b)
        checks['native_state_schema'] = all(_state_schema(sample['physics_state']) for sample in a+b)
        checks['matching_decisions'] = all((x['terminated'], x['truncated'], x['reason'], x['success']) ==
            (y['terminated'], y['truncated'], y['reason'], y['success']) for x, y in zip(records_a, records_b))
        checks['complete_terminal_episodes'] = all(
            all(type(row.get(k)) is bool for row in rows for k in ('terminated', 'truncated'))
            and all(not row['terminated'] and not row['truncated'] for row in rows[:-1])
            and rows[-1]['terminated'] != rows[-1]['truncated'] for rows in (records_a, records_b))
        if not all(checks.values()):
            return report()
        metrics.update(maximum_position_difference_m=max(math.dist(x['physics_state']['position_m'], y['physics_state']['position_m']) for x, y in zip(a, b)),
            maximum_speed_difference_m_s=max(abs(x['physics_state']['speed_m_s']-y['physics_state']['speed_m_s']) for x, y in zip(a, b)),
            maximum_yaw_difference_rad=max(abs(_angle(x['physics_state']['yaw_rad']-y['physics_state']['yaw_rad'])) for x, y in zip(a, b)),
            compared_physics_samples=len(a))
        checks['position_repeatability'] = metrics['maximum_position_difference_m'] <= acceptance['repeat_position_tolerance_m']
        checks['speed_repeatability'] = metrics['maximum_speed_difference_m_s'] <= acceptance['repeat_speed_tolerance_m_s']
        checks['yaw_repeatability'] = metrics['maximum_yaw_difference_rad'] <= acceptance['repeat_yaw_tolerance_rad']
        checks['matching_support_and_contacts'] = all(x['physics_state']['wheel_on_ground'] == y['physics_state']['wheel_on_ground']
            and x['contact_count'] == y['contact_count'] for x, y in zip(a, b))
    except (KeyError, TypeError, ValueError, IndexError):
        checks['schema'] = False
    return report()
