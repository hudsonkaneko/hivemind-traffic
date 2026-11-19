"""V14 merge target: the policy is the only controller, so the target judges yield gaps itself.

V14 turns SUMO's own checks off for controlled cars (speed mode 96 and lane change mode 0,
SUMO 1.27.1 TraCI documentation): SUMO moves the car exactly as commanded after the
environment's own limits (speed limit, following gap, lane end, comfort rate). SUMO no
longer brakes or holds a car at a yield line, so the merge target has to say when to wait.

Measured with round 19's starting policy and the v12 target, SUMO's checks off
(outputs/test-runs/policy-only-probe1-stopped, focused northbound LA merge): the gap plan
chose "behind_rear" (fall in behind main_13) with a 20.1 m/s target. The car crossed the
yield line at 13.3 m/s with main_13 0.3 m behind it in the shared stop-line projection and
9.4 m/s faster; main_13 braked at 9 m/s^2 and collided at t = 3.5 s.

The cause: v04's gap plan checks each candidate's gaps only at the end of its horizon
(3, 6 or 10 s), as if the car could wait beside the slot as on an acceleration lane. At a
yield entry the car enters the mainline the moment it crosses the line. Two changes, both
only at yield-junction entries (entry_active); every other merge keeps v12's target exactly:

1. Crossing check. Each gap-plan candidate also needs its gaps from the moment it crosses
   the line through v04's 3.1 s maneuver window, with v03's required_gap. A candidate that
   stops before the line never crosses and needs no crossing gap.
2. Reachable target. Once a car's target comes from a yield entry, it changes no faster
   than v05's stable-target limits (+1.5 / -2.0 m/s^2), until it meets the plan's target
   after the line. A rise starts from the larger of the previous target and the car's
   speed. Map and occupied-lane stop bounds still apply at once.

Everything else (observation widths, rewards, weights, gate thresholds, the environment's
own limits) is v12's. Every input is the car's own map and neighbor context.
"""
from copy import deepcopy
import math

from traffic.highway_flow_rl_v03_rewards import CAP, finite, required_gap, safe_speed_target
from traffic.highway_flow_rl_v04_rewards import TARGET_SETTINGS as V04_SETTINGS, gap_plan as v04_gap_plan
from traffic.highway_flow_rl_v05_rewards import SETTINGS as V05_SETTINGS

REWARD_VERSION = 'highway_flow_rl_merge_v14_policy_only_yield'
# SUMO TraCI speed mode 96 = "all checks off"; lane change mode 0 = no SUMO lane changes and no
# SUMO safety checks on the policy's requests (doc/userdoc/TraCI/Change_Vehicle_State.html).
CONTROL = dict(speed_mode=96, lane_change_mode=0)
SETTINGS = dict(
    crossing_window_s=V04_SETTINGS['maneuver_horizon_s'],             # 3.1
    crossing_step_s=V05_SETTINGS['step_s'],                           # 0.1
    target_acceleration_m_s2=V05_SETTINGS['target_acceleration_m_s2'],  # 1.5
    target_deceleration_m_s2=V05_SETTINGS['target_deceleration_m_s2'],  # 2.0
    initial_preview_s=V05_SETTINGS['initial_preview_s'])              # 3.0


def validate_settings(settings):
    if dict(settings) != SETTINGS:
        raise ValueError('Unreviewed v14 settings')
    return dict(settings)


def yield_entry(context):
    entry = (context or {}).get('entry') or {}
    return bool(entry.get('entry_active'))


def profile(speed, target, horizon, t):
    """v04's candidate motion: constant acceleration to the target over the horizon, then hold."""
    accel = (target - speed) / horizon
    if t <= horizon:
        return speed * t + .5 * accel * t * t, speed + accel * t
    return speed * horizon + .5 * accel * horizon * horizon + target * (t - horizon), target


def crossing_time(speed, target, horizon, distance):
    """First time the candidate profile covers the distance to the line, or None if it stops first."""
    if distance <= 0.:
        return 0.
    accel = (target - speed) / horizon
    if abs(accel) < 1e-12:
        first = distance / speed if speed > 0. else None
    else:
        disc = speed * speed + 2. * accel * distance
        first = (-speed + math.sqrt(disc)) / accel if disc >= 0. else None
    if first is not None and 0. <= first <= horizon + 1e-9:
        return first
    covered, _ = profile(speed, target, horizon, horizon)
    if target <= 0. or covered >= distance:
        return None
    return horizon + (distance - covered) / target


def crossing_deficit(speed, target, horizon, distance, ahead, behind, settings=SETTINGS):
    """Worst normalized gap shortfall from crossing the line through the maneuver window.

    Positions are front-bumper offsets in the shared stop-line projection (v04 gap_plan's
    frame): the ego crosses the line when it has travelled `distance`.
    """
    start = crossing_time(speed, target, horizon, distance)
    if start is None:
        return 0., None
    length = V04_SETTINGS['vehicle_length_m']
    steps = int(round(settings['crossing_window_s'] / settings['crossing_step_s']))
    worst = 0.
    for k in range(steps + 1):
        t = start + k * settings['crossing_step_s']
        travel, ego = profile(speed, target, horizon, t)
        for other, direction in ((ahead, 'front'), (behind, 'rear')):
            if other is None:
                continue
            signed = other['x'] + other['speed'] * t - travel
            gap = signed - length if direction == 'front' else -signed - length
            need = (required_gap(ego, other['speed']) if direction == 'front'
                    else required_gap(other['speed'], ego))
            worst = max(worst, max(0., need - gap) / max(10., need))
    return worst, start


def gap_plan(state, context, *, previous_plan=None, switch_margin=.10, settings=SETTINGS):
    """v04's gap plan, plus the crossing check at yield entries; identical elsewhere."""
    if not yield_entry(context):
        return v04_gap_plan(state, context, previous_plan=previous_plan, switch_margin=switch_margin)
    entry, own = context['entry'], context['corridors']['own']
    if not entry.get('known') or not entry.get('target_lane'):
        return dict(known=False, required=True, potential=0., reason='unknown_merge_target')
    speed = state['speed']
    limit = min(CAP, state['allowed_speed'], entry['target_speed_limit'])
    distance = own['route']['distance_m']
    line = entry.get('distance_to_start_m')
    if not all(finite(v) and v >= 0 for v in (speed, limit, distance, line)) or limit <= 0:
        return dict(known=False, required=True, potential=0., reason='invalid_map_or_speed')
    length = V04_SETTINGS['vehicle_length_m']

    def vehicle(neighbor, direction):
        if neighbor is None:
            return None
        gap, relative = neighbor['bumper_gap_m'], neighbor['relative_speed_m_s']
        if not finite(gap) or not finite(relative) or gap < 0 or speed + relative < -1e-6:
            raise ValueError('Invalid merge neighbor')
        return dict(id=neighbor.get('id'), x=gap + length if direction == 'front' else -gap - length,
                    speed=max(0., speed + relative))
    try:
        front, rear = vehicle(entry.get('front'), 'front'), vehicle(entry.get('rear'), 'rear')
    except (ValueError, KeyError, TypeError):
        return dict(known=False, required=True, potential=0., reason='invalid_merge_neighbor')
    slots = [('between', front, rear, False)]
    if rear:
        slots.append(('behind_rear', rear, None, True))
    if front:
        slots.append(('ahead_front', None, front, True))
    candidates = []
    for name, ahead, behind, outer_unknown in slots:
        for horizon in V04_SETTINGS['preparation_horizons_s']:
            for i in range(17):
                target = limit * i / 16.
                accel = (target - speed) / horizon
                if not -2. - 1e-8 <= accel <= 1.5 + 1e-8:
                    continue
                travel = .5 * (speed + target) * horizon
                deficit = 0.
                for other, direction in ((ahead, 'front'), (behind, 'rear')):
                    if other is None:
                        continue
                    signed = other['x'] + other['speed'] * horizon - travel
                    gap = signed - length if direction == 'front' else -signed - length
                    relative = other['speed'] - target
                    end_gap = gap + (relative if direction == 'front' else -relative) * 3.1
                    need = required_gap(target, other['speed']) if direction == 'front' else required_gap(other['speed'], target)
                    deficit += max(0., need - min(gap, end_gap)) / max(10., need)
                runway = travel + target * 3.1 + 2.5
                if not own['route']['reachable']:
                    deficit += max(0., runway - distance) / max(10., distance)
                crossing, crossing_s = crossing_deficit(speed, target, horizon, line, ahead, behind, settings)
                deficit += crossing
                cost = deficit + .15 * (limit - target) / limit + .05 * abs(accel) / 2. + .05 * outer_unknown
                candidates.append((cost, horizon, name, target, ahead, behind, outer_unknown, deficit,
                                   crossing, crossing_s))
    if not candidates:
        return dict(known=False, required=True, potential=0., reason='no_reachable_speed_candidate')
    selected = min(candidates, key=lambda c: (c[0], c[1], c[2], -c[3]))
    if previous_plan and previous_plan.get('known') and previous_plan.get('required'):
        previous_key = (previous_plan.get('slot'), previous_plan.get('front_id'), previous_plan.get('rear_id'))
        retained = [c for c in candidates if c[7] <= 1e-8 and
                    (c[2], c[4]['id'] if c[4] else None, c[5]['id'] if c[5] else None) == previous_key]
        if retained:
            alternative = min(retained, key=lambda c: (c[0], c[1], c[2], -c[3]))
            if alternative[0] <= selected[0] + switch_margin:
                selected = alternative
    cost, horizon, name, target, ahead, behind, unknown, deficit, crossing, crossing_s = selected
    error = abs(speed - target) / max(5., limit)
    for other, direction in ((ahead, 'front'), (behind, 'rear')):
        if other is None:
            continue
        gap = other['x'] - length if direction == 'front' else -other['x'] - length
        end_gap = gap + (other['speed'] - speed) * (3.1 if direction == 'front' else -3.1)
        need = required_gap(speed, other['speed']) if direction == 'front' else required_gap(other['speed'], speed)
        error += max(0., need - min(gap, end_gap)) / max(10., need)
    return dict(known=True, required=True, potential=1. / (1. + error), target_m_s=target,
        slot=name, front_id=ahead['id'] if ahead else None, rear_id=behind['id'] if behind else None,
        outer_neighbor_unknown=unknown, prediction_horizon_s=horizon, predicted_deficit=deficit,
        cost=cost, lateral_permission=False, crossing_deficit=crossing, crossing_time_s=crossing_s,
        distance_to_line_m=line,
        projection='current velocities, bounded preparation acceleration, gaps from crossing the yield line')


def merge_speed_target(state, context, preview):
    """v04's merge_speed_target with the v14 plan; identical to v04 away from yield entries.

    Returns (target, plan, ceiling): ceiling is the occupied-lane/map bound that applies at once.
    """
    plan = gap_plan(state, context)
    original = safe_speed_target(state, context, preview)
    if not plan.get('required') or not plan.get('known'):
        return original, plan, None
    longitudinal = deepcopy(context)
    longitudinal['entry'].update(front=None, rear=None)
    target = safe_speed_target(state, longitudinal, preview)
    ceiling = target['target_m_s'] if target['eligible'] else None
    if target['eligible']:
        target['target_m_s'] = min(target['target_m_s'], plan['target_m_s'])
        target['projection'] = 'gap preparation bounded by occupied-lane/map stopping constraints'
        target['original_target_reasons'] = original['reasons']
        target['clear_cruise'] = False
    return target, plan, ceiling


def reachable(desired, ceiling, speed, previous, now, settings=SETTINGS):
    """v05's stable-target rate limits: the target moves toward `desired` at +1.5 / -2.0 m/s^2.

    previous: dict(time=..., value=...) of the last target emitted for this car, or None at the
    start of a ramp (then the target starts no more than 1.5 m/s^2 x 3 s above the car's speed).
    A rise starts from the larger of the previous target and the car's speed. v05 started it
    from the previous target alone; in the v14 smoke check that left the target up to 13 m/s
    below a car already at mainline speed past the line (southbound focused LA merge), which
    would reward braking on the mainline. A car held at the line still gets the 1.5 m/s^2
    ramp, and a slow car cannot hold the target down. The occupied-lane/map ceiling applies
    at once.
    """
    if not all(finite(v) for v in (desired, speed, now)):
        raise ValueError('Reachable target needs finite values')
    if previous is None:
        bounded = min(desired, speed + settings['target_acceleration_m_s2'] * settings['initial_preview_s'])
    else:
        old, dt = previous['value'], now - previous['time']
        if not finite(old) or not finite(dt) or dt < -1e-9:
            raise ValueError('Reachable target needs an earlier finite target')
        bounded = max(old - settings['target_deceleration_m_s2'] * dt,
                      min(max(old, speed) + settings['target_acceleration_m_s2'] * dt, desired))
    return min(ceiling, bounded) if ceiling is not None else bounded
