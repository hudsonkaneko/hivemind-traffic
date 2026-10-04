"""CPU-only gates for gentle non-self-intersecting lane-following fixtures."""
import math


def validate_config(config):
    def number(mapping, name, low, high, integer=False):
        value = mapping.get(name)
        if (isinstance(value, bool) or not isinstance(value, int if integer else (int, float))
                or not math.isfinite(value) or not low <= value <= high):
            raise ValueError(f'{name} must be finite and in [{low}, {high}]')
        return value
    number(config, 'schema_version', 1, 1, True)
    number(config, 'seed', 0, 2**32-1, True)
    for name, value in [('physics_hz', 120), ('control_hz', 60), ('planner_hz', 10), ('behavior_hz', 10)]:
        number(config, name, value, value, True)
    number(config, 'repeats', 2, 2, True)
    number(config, 'target_speed_m_s', 1, 3)
    number(config, 'settle_s', 2, 2)
    number(config, 'drive_s', 48, 48)
    number(config, 'drive_torque_per_front_wheel_nm', 1, 700)
    number(config, 'brake_torque_per_wheel_nm', 1, 1500)
    number(config, 'command_ttl_ticks', 12, 12, True)
    number(config, 'contact_positive_control_s', 8, 8)
    number(config, 'max_process_seconds', 60, 420)
    if config.get('routes_file') != 'scenarios/physics-road/routes.json':
        raise ValueError('This study uses the frozen project route fixture')
    if config.get('route_ids') != ['straight100', 'left_r60', 'right_r60']:
        raise ValueError('Run all three declared routes in the fixed order')
    if not isinstance(config.get('follower'), dict):
        raise ValueError('Follower settings must be an object')
    from traffic.path_following import FollowerConfig
    from dataclasses import asdict
    follower = FollowerConfig(**config['follower'])
    if asdict(follower) != config['follower']:
        raise ValueError('Resolve every follower setting before source capture')
    if (follower.wheelbase_m != 3.2 or follower.rear_axle_offset_m != 1.6
            or follower.max_speed_m_s > 3 or follower.max_steering_rad > .5
            or follower.command_ttl_ticks != config['command_ttl_ticks']):
        raise ValueError('Follower does not match this bounded physical vehicle')
    a = config.get('acceptance', {})
    for name, maximum in [('lane_rms_max_m', .2), ('lane_error_max_m', .5),
                          ('endpoint_tolerance_m', .5), ('stop_speed_max_m_s', .05),
                          ('hold_drift_max_m', .05), ('unsupported_time_max_s', .1),
                          ('repeat_position_tolerance_m', .001), ('repeat_speed_tolerance_m_s', .001),
                          ('repeat_yaw_tolerance_rad', .001)]:
        number(a, name, 1e-9, maximum)
    number(a, 'hold_s', 5, 5)
    number(a, 'upright_min_z', math.cos(math.radians(5)), 1)
    number(a, 'grounded_fraction_min', .99, 1)
    return config


def footprint_lateral_bound(state, projection, dimensions_m, max_curvature_rad_m):
    """Conservative projected box envelope for our simple straight/arc corridors.

    Rotates all eight chassis corners, including roll/pitch. The normal linear
    offset includes a curvature remainder bound K*r²/[2(1-K*d)] within a
    non-self-overlapping tubular neighborhood. Return infinity if that bound is
    unavailable. This is not a general urban/intersection containment algorithm.
    Longitudinal route endpoints have tangent road padding, not physical cliffs.
    """
    qx, qy, qz, qw = state['quaternion_xyzw']
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if not math.isfinite(norm) or norm < 1e-9:
        return math.inf
    qx, qy, qz, qw = [v/norm for v in (qx, qy, qz, qw)]
    offsets = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                x, y, z = [s*d/2 for s, d in zip((sx, sy, sz), dimensions_m)]
                # Quaternion-vector rotation v + 2*q.xyz cross(q.xyz cross v + qw*v).
                tx, ty, tz = 2*(qy*z-qz*y), 2*(qz*x-qx*z), 2*(qx*y-qy*x)
                offsets.append((x+qw*tx+qy*tz-qz*ty, y+qw*ty+qz*tx-qx*tz))
    k = abs(max_curvature_rad_m)
    radius = max(math.hypot(x, y) for x, y in offsets)
    denominator = 1-k*(abs(projection.lateral_error_m)+radius)
    if denominator <= 0:
        return math.inf
    remainder = k*radius*radius/(2*denominator)
    nx, ny = -math.sin(projection.point.yaw_rad), math.cos(projection.point.yaw_rad)
    return max(abs(projection.lateral_error_m+nx*x+ny*y) for x, y in offsets)+remainder


def assess_lane(rows, config, *, route_length_m, lane_width_m, contact_events):
    hz, a = config['physics_hz'], config['acceptance']
    total = round((config['settle_s']+config['drive_s'])*hz)
    gates = {'trace_complete': len(rows) == total}
    def finite(value):
        if isinstance(value, dict):
            return all(finite(v) for v in value.values())
        if isinstance(value, (list, tuple)):
            return all(finite(v) for v in value)
        return not isinstance(value, (int, float)) or math.isfinite(value)
    gates['finite_telemetry'] = bool(rows) and all(finite(r) for r in rows)
    gates['wheel_support_schema'] = bool(rows) and all(
        isinstance(r.get('wheel_on_ground'), list) and len(r['wheel_on_ground']) == 4
        and all(isinstance(value, bool) for value in r['wheel_on_ground']) for r in rows)
    gates['tick_order'] = gates['trace_complete'] and all(r['tick'] == i+1
        and r['phase'] == ('settle' if i < round(config['settle_s']*hz) else 'drive')
        and abs(r['sim_time_s']-(i+1)/hz) <= 1e-9 for i, r in enumerate(rows))
    if not all(gates.values()):
        return {'passed': False, 'gates': gates, 'samples': len(rows)}
    active = [r for r in rows if r['phase'] == 'drive']
    hold = active[-round(a['hold_s']*hz):]
    errors = [r['lateral_error_m'] for r in active]
    rms = math.sqrt(sum(e*e for e in errors)/len(errors))
    maximum = max(abs(e) for e in errors)
    unsupported, longest = 0, 0
    for r in active:
        unsupported = 0 if all(r['wheel_on_ground']) else unsupported+1
        longest = max(longest, unsupported)
    grounded = sum(all(r['wheel_on_ground']) for r in active)/len(active)
    hold_drift = max(math.dist(hold[0]['position_m'][:2], r['position_m'][:2]) for r in hold)
    terminal_errors = [abs(r['progress_m']-route_length_m) for r in hold]
    envelope = max(r['footprint_lateral_bound_m'] for r in rows)
    gates.update(initial_station=abs(rows[0]['progress_m']) <= .05,
                 forward_progress=rows[-1]['progress_m']-rows[0]['progress_m'] >= route_length_m-a['endpoint_tolerance_m'],
                 lane_rms=rms <= a['lane_rms_max_m'], lane_max=maximum <= a['lane_error_max_m'],
                 endpoint=max(terminal_errors) <= a['endpoint_tolerance_m'],
                 hold_speed=max(r['speed_m_s'] for r in hold) <= a['stop_speed_max_m_s'],
                 hold_drift=hold_drift <= a['hold_drift_max_m'],
                 no_road_departure=envelope <= lane_width_m/2,
                 no_rigid_contact=not contact_events,
                 upright=min(r['upright_z'] for r in rows) >= a['upright_min_z'],
                 grounded=grounded >= a['grounded_fraction_min'],
                 support_gap=longest/hz <= a['unsupported_time_max_s'],
                 no_command_fallback=not any(r['control']['is_fallback'] for r in active),
                 no_planner_fallback=all(r['driver_diagnostics'].get('fallback') is False for r in active))
    return dict(passed=all(gates.values()), gates=gates, samples=len(rows), metrics=dict(
        lane_center_rms_m=rms, lane_center_max_m=maximum, final_progress_m=rows[-1]['progress_m'],
        endpoint_error_max_during_hold_m=max(terminal_errors), hold_s=len(hold)/hz,
        hold_speed_max_m_s=max(r['speed_m_s'] for r in hold), hold_drift_m=hold_drift,
        footprint_lateral_bound_max_m=envelope, all_wheels_grounded_fraction=grounded,
        longest_unsupported_s=longest/hz, rigid_contact_events=len(contact_events),
        minimum_upright_z=min(r['upright_z'] for r in rows),
        target_speed_m_s=config['target_speed_m_s'], maximum_speed_m_s=max(r['speed_m_s'] for r in active)))
