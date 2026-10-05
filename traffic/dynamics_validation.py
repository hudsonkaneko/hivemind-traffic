"""CPU-only, predeclared acceptance checks for a bounded vehicle dynamics suite.

No gate here establishes real-vehicle calibration. The turn comparison is the
rear-axle trajectory's arc length divided by unwrapped heading change, compared
with a low-slip bicycle radius. Coasting is an observation/control-integrity
test, not a claim that this sample models aerodynamic or rolling drag correctly.
"""
from __future__ import annotations

import math
from statistics import fmean


def _positive(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and value > 0


def validate_config(config):
    """Reject unbounded or malformed plans before starting the GPU runtime."""
    if config.get('schema_version') != 1:
        raise ValueError('Unsupported dynamics configuration schema')
    if not isinstance(config.get('seed'), int) or isinstance(config['seed'], bool) or config['seed'] < 0:
        raise ValueError('seed must be a nonnegative integer')
    for name in ('physics_hz', 'control_hz', 'repeats', 'command_ttl_ticks'):
        if not isinstance(config.get(name), int) or isinstance(config[name], bool) or config[name] <= 0:
            raise ValueError(f'{name} must be a positive integer')
    if config['physics_hz'] != 120 or config['control_hz'] != 60 or config['repeats'] != 2:
        raise ValueError('This validation suite requires 120 Hz physics, 60 Hz control and two repeats')
    if not 2 <= config['command_ttl_ticks'] <= 120:
        raise ValueError('Command lifetime must be 2 to 120 physics ticks')
    for name in ('drive_torque_per_front_wheel_nm', 'brake_torque_per_wheel_nm',
                 'speed_proportional_gain', 'max_process_seconds', 'max_total_simulated_seconds'):
        if not _positive(config.get(name)):
            raise ValueError(f'{name} must be finite and positive')
    if config['max_process_seconds'] > 300 or config['max_total_simulated_seconds'] > 300:
        raise ValueError('Bounded suite allows at most 300 wall or total simulated seconds')
    cases = config.get('cases')
    if not isinstance(cases, list) or not 1 <= len(cases) <= 6:
        raise ValueError('Expected one to six cases')
    names, duration = set(), 0.0
    expected_phases = {
        'straight': ['settle', 'accelerate', 'brake', 'hold'],
        'circle': ['settle', 'accelerate', 'turn_warmup', 'circle', 'brake', 'hold'],
        'coast': ['settle', 'accelerate', 'coast', 'brake', 'hold'],
    }
    for case in cases:
        name = case.get('name', '')
        if not isinstance(name, str) or not name or not all(c.isalnum() or c == '_' for c in name) or name in names:
            raise ValueError('Case names must be unique nonempty filename-safe identifiers')
        names.add(name)
        if case.get('kind') not in expected_phases:
            raise ValueError('Unsupported case kind')
        if not _positive(case.get('target_speed_m_s')) or case['target_speed_m_s'] > 6:
            raise ValueError('Target speed must be in (0, 6] m/s')
        steer = case.get('steering_rad')
        if isinstance(steer, bool) or not isinstance(steer, (int, float)) or not math.isfinite(steer) or abs(steer) > 0.2:
            raise ValueError('Steering must be finite and within +/-0.2 rad')
        if (case['kind'] == 'circle') != (steer != 0):
            raise ValueError('Only circle cases use nonzero steering')
        if not _positive(case.get('braking_distance_max_m')):
            raise ValueError('Braking distance limit must be finite and positive')
        phases = case.get('phases')
        if not isinstance(phases, list) or any(not isinstance(p, list) or len(p) != 2 for p in phases):
            raise ValueError('Phases must be [name, seconds] pairs')
        if [p[0] for p in phases] != expected_phases[case['kind']]:
            raise ValueError('Missing or misordered case phases')
        for phase, seconds in phases:
            if not _positive(seconds) or seconds > 30:
                raise ValueError('Each phase must be in (0, 30] seconds')
            if abs(seconds * config['physics_hz'] - round(seconds * config['physics_hz'])) > 1e-8:
                raise ValueError('Phase duration must be a whole number of physics ticks')
            if phase == 'hold' and seconds < 5:
                raise ValueError('Hold phase must cover at least five seconds')
            duration += seconds
    if duration * config['repeats'] > config['max_total_simulated_seconds']:
        raise ValueError('Total simulation budget exceeded')
    limits = config.get('acceptance', {})
    required = ('speed_tolerance_m_s', 'straight_lateral_drift_max_m', 'root_height_min_m',
                'root_height_max_m', 'grounded_fraction_min', 'stopped_speed_max_m_s',
                'hold_duration_min_s', 'hold_drift_max_m', 'upright_min_z',
                'circle_radius_relative_tolerance', 'circle_yaw_progress_min_rad',
                'circle_steering_tolerance_rad', 'repeat_position_tolerance_m',
                'repeat_speed_tolerance_m_s', 'repeat_yaw_tolerance_rad')
    if any(not _positive(limits.get(name)) for name in required):
        raise ValueError('Every acceptance limit must be present, finite and positive')
    if limits['root_height_min_m'] >= limits['root_height_max_m']:
        raise ValueError('Height range must be ordered')
    if any(limits[n] > 1 for n in ('grounded_fraction_min', 'upright_min_z', 'circle_radius_relative_tolerance')):
        raise ValueError('Fractions must be in (0, 1]')
    if limits['hold_duration_min_s'] < 5:
        raise ValueError('The acceptance hold window must be at least five seconds')
    return config


def yaw_delta(a, b):
    """Signed shortest increment b-a; evaluated once per physics tick."""
    return math.atan2(math.sin(b - a), math.cos(b - a))


def rear_axle_position(row, wheelbase_m):
    """Planar rear axle for the symmetric Factory chassis (root halfway axles)."""
    yaw = row['yaw_rad']
    return [row['position_m'][0] - wheelbase_m / 2 * math.cos(yaw),
            row['position_m'][1] - wheelbase_m / 2 * math.sin(yaw)]


def _path_length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def _finite_row(row):
    try:
        numbers = list(row['position_m']) + list(row['velocity_m_s']) + list(row['quaternion_xyzw'])
        numbers += [row[key] for key in ('sim_time_s', 'speed_m_s', 'yaw_rad', 'upright_z')]
        numbers += [row['control'][key] for key in ('steering_rad', 'throttle', 'brake')]
        return (len(row['position_m']) == 3 and len(row['velocity_m_s']) == 3
                and len(row['quaternion_xyzw']) == 4 and len(row['wheel_on_ground']) == 4
                and all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in numbers))
    except (KeyError, TypeError):
        return False


def assess_case(rows, case, config, *, wheelbase_m):
    """Assess the complete raw trace without dropping failures or changing gates."""
    hz, limits = config['physics_hz'], config['acceptance']
    expected = [name for name, seconds in case['phases'] for _ in range(round(seconds * hz))]
    gates = {'trace_complete': len(rows) == len(expected), 'finite_telemetry': bool(rows) and all(_finite_row(r) for r in rows)}
    gates['tick_phase_order'] = gates['trace_complete'] and gates['finite_telemetry'] and all(
        r.get('tick') == i + 1 and r.get('phase') == expected[i]
        and abs(r.get('sim_time_s', -1) - (i + 1) / hz) <= 1e-9 for i, r in enumerate(rows))
    if not all(gates.values()):
        return dict(passed=False, gates=gates, case=case['name'], samples=len(rows), expected_samples=len(expected))
    if not _positive(wheelbase_m):
        raise ValueError('wheelbase_m must be finite and positive')
    phase = lambda name: [r for r in rows if r['phase'] == name]
    accelerate, brake, hold = [phase(name) for name in ('accelerate', 'brake', 'hold')]
    active = [r for r in rows if r['phase'] != 'settle']
    target = case['target_speed_m_s']
    steady_acceleration = accelerate[-hz:]
    last_accel_speed = accelerate[-1]['speed_m_s']
    brake_start = rows[rows.index(brake[0]) - 1]
    stop = next((r for r in brake if r['speed_m_s'] <= limits['stopped_speed_max_m_s']), None)
    stopping = [brake_start] + [r for r in brake if stop is None or r['tick'] <= stop['tick']]
    distance = _path_length([r['position_m'][:2] for r in stopping])
    hold_start = brake[-1]
    hold_drift = max(math.dist(r['position_m'][:2], hold_start['position_m'][:2]) for r in hold)
    lateral_drift = max(abs(r['position_m'][1] - accelerate[0]['position_m'][1]) for r in accelerate)
    grounded = sum(all(r['wheel_on_ground']) for r in active) / len(active)
    gates.update(
        target_speed=all(abs(r['speed_m_s'] - target) <= limits['speed_tolerance_m_s'] for r in steady_acceleration),
        forward=accelerate[-1]['position_m'][0] > accelerate[0]['position_m'][0] and accelerate[-1]['velocity_m_s'][0] > 0,
        straight_drift=lateral_drift <= limits['straight_lateral_drift_max_m'],
        braking=stop is not None and distance <= case['braking_distance_max_m'] and brake[-1]['speed_m_s'] <= limits['stopped_speed_max_m_s'],
        brake_commands=all(r['control']['throttle'] == 0 and r['control']['brake'] == 1 for r in brake),
        hold_duration=len(hold) / hz >= limits['hold_duration_min_s'],
        hold=hold_drift <= limits['hold_drift_max_m'] and max(r['speed_m_s'] for r in hold) <= limits['stopped_speed_max_m_s'],
        upright=min(r['upright_z'] for r in rows) >= limits['upright_min_z'],
        height=all(limits['root_height_min_m'] <= r['position_m'][2] <= limits['root_height_max_m'] for r in rows),
        grounded=grounded >= limits['grounded_fraction_min'],
        no_control_fallback=not any(r['control'].get('is_fallback', True) for r in rows),
    )
    metrics = dict(steady_acceleration_mean_speed_m_s=fmean(r['speed_m_s'] for r in steady_acceleration),
                   acceleration_final_speed_m_s=last_accel_speed,
                   straight_lateral_drift_m=lateral_drift, brake_entry_speed_m_s=brake_start['speed_m_s'],
                   brake_distance_m=distance, brake_final_speed_m_s=brake[-1]['speed_m_s'],
                   brake_stop_time_s=None if stop is None else stop['sim_time_s'] - brake_start['sim_time_s'],
                   hold_duration_s=len(hold) / hz, hold_drift_m=hold_drift,
                   all_wheels_grounded_fraction=grounded, minimum_upright_z=min(r['upright_z'] for r in rows))
    if case['kind'] != 'coast':
        gates['brake_entry_speed'] = abs(brake_start['speed_m_s'] - target) <= limits['speed_tolerance_m_s']
    if case['kind'] == 'circle':
        circle = phase('circle')
        signed_yaw = sum(yaw_delta(a['yaw_rad'], b['yaw_rad']) for a, b in zip(circle, circle[1:]))
        arc = _path_length([rear_axle_position(r, wheelbase_m) for r in circle])
        radius = arc / abs(signed_yaw) if abs(signed_yaw) > 1e-9 else None
        expected_radius = abs(wheelbase_m / math.tan(case['steering_rad']))
        relative = abs(radius - expected_radius) / expected_radius if radius is not None else None
        gates.update(
            circle_turn_direction=signed_yaw * math.copysign(1, case['steering_rad']) >= limits['circle_yaw_progress_min_rad'],
            circle_radius=relative is not None and relative <= limits['circle_radius_relative_tolerance'],
            circle_speed=all(abs(r['speed_m_s'] - target) <= limits['speed_tolerance_m_s'] for r in circle),
            circle_steering=all(abs(r['control']['steering_rad'] - case['steering_rad']) <= limits['circle_steering_tolerance_rad'] for r in circle),
        )
        metrics.update(circle_signed_yaw_rad=signed_yaw, rear_axle_arc_length_m=arc,
                       measured_rear_axle_radius_m=radius, bicycle_radius_m=expected_radius,
                       radius_relative_error=relative, circle_mean_speed_m_s=fmean(r['speed_m_s'] for r in circle))
    if case['kind'] == 'coast':
        coast = phase('coast')
        gates['coast_zero_actuation'] = all(r['control']['throttle'] == 0 and r['control']['brake'] == 0
                                          and r['control']['steering_rad'] == 0 for r in coast)
        metrics.update(coast_entry_speed_m_s=accelerate[-1]['speed_m_s'], coast_final_speed_m_s=coast[-1]['speed_m_s'],
                       coast_speed_change_m_s=coast[-1]['speed_m_s'] - accelerate[-1]['speed_m_s'],
                       coast_distance_m=_path_length([accelerate[-1]['position_m'][:2]] + [r['position_m'][:2] for r in coast]),
                       coast_interpretation='Finite free rolling under the installed sample model; no real drag calibration claim')
    return dict(passed=all(gates.values()), case=case['name'], samples=len(rows), gates=gates, metrics=metrics)


def compare_repeats(first, second, acceptance):
    """Compare every tick, rejecting missing/reordered samples rather than zip truncation."""
    matched = bool(first) and len(first) == len(second) and all(
        a.get('tick') == b.get('tick') and a.get('phase') == b.get('phase')
        and a.get('sim_time_s') == b.get('sim_time_s') for a, b in zip(first, second))
    finite = matched and all(_finite_row(r) for r in first + second)
    if not finite:
        return dict(passed=False, matched_samples=matched, finite_telemetry=finite)
    position = max(math.dist(a['position_m'], b['position_m']) for a, b in zip(first, second))
    speed = max(abs(a['speed_m_s'] - b['speed_m_s']) for a, b in zip(first, second))
    yaw = max(abs(yaw_delta(a['yaw_rad'], b['yaw_rad'])) for a, b in zip(first, second))
    passed = (position <= acceptance['repeat_position_tolerance_m']
              and speed <= acceptance['repeat_speed_tolerance_m_s']
              and yaw <= acceptance['repeat_yaw_tolerance_rad'])
    return dict(passed=passed, matched_samples=True, finite_telemetry=True,
                maximum_position_difference_m=position, maximum_speed_difference_m_s=speed,
                maximum_yaw_difference_rad=yaw)
