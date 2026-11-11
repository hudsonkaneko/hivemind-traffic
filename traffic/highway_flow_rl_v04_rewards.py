"""Merge preparation from current observations; no action override or future truth."""
from copy import deepcopy
import math

from traffic.highway_flow_rl_v03_rewards import (
    CAP, finite, required_gap, safe_speed_target, TARGET_SETTINGS as V03_SETTINGS)

REWARD_VERSION = 'highway_flow_rl_gap_preparation_v04'
MASK_VERSION = 'native_v05_plus_projected_maneuver_gaps_v04'
DEFAULT_WEIGHTS = dict(speed_error=.30, jerk_huber=.09, route_detour=2.,
                       gap_preparation=2., quality_merge=5.)
TARGET_SETTINGS = dict(V03_SETTINGS, maneuver_horizon_s=3.1,
    preparation_horizons_s=[3., 6., 10.], preparation_accel_m_s2=1.5,
    preparation_decel_m_s2=2., vehicle_length_m=4.8,
    gamma=.99, quality_window_s=8., quality_speed_tolerance_m_s=2.,
    quality_min_speed_match_fraction=.9, quality_max_stop_seconds=1.,
    quality_max_jerk_m_s3=5., quality_neighbor_braking_m_s2=-3.)


def validate_weights(weights=None):
    result = dict(DEFAULT_WEIGHTS if weights is None else weights)
    if set(result) != set(DEFAULT_WEIGHTS) or any(not finite(v) or v < 0 for v in result.values()):
        raise ValueError('Invalid v04 reward weights')
    return result


def maneuver_gap_evidence(speed, neighbors, duration=3.1):
    """Current and end gaps under constant speeds; native guards remain final.

    Linear gap motion has its minimum at an endpoint. This is an additional
    mask, not a certificate against later acceleration or unexpected cut-ins.
    """
    result = dict(permitted=True, horizon_s=duration, checks={})
    if not finite(speed) or speed < 0:
        return dict(result, permitted=False, reason='invalid_speed')
    for direction in ('front', 'rear'):
        other = neighbors.get(direction)
        if other is None:
            continue
        gap, relative = other.get('bumper_gap_m'), other.get('relative_speed_m_s')
        if not finite(gap) or not finite(relative) or gap < 0 or speed + relative < -1e-6:
            return dict(result, permitted=False, reason='invalid_neighbor')
        other_speed = max(0., speed + relative)
        follower, leader = (speed, other_speed) if direction == 'front' else (other_speed, speed)
        end_gap = gap + (relative if direction == 'front' else -relative) * duration
        need = required_gap(follower, leader)
        permitted = min(gap, end_gap) >= need
        result['checks'][direction] = dict(current_gap_m=gap, projected_gap_m=end_gap,
            required_gap_m=need, permitted=permitted)
        result['permitted'] &= permitted
    return result


def gap_plan(state, context, *, previous_plan=None, switch_margin=.10):
    """Score the current slot and moving behind/ahead of an observed neighbor.

    Outer neighbors of alternative slots are unknown: these slots only shape
    longitudinal preparation. They NEVER authorize lateral motion. The actor
    gets no privileged state; every input is already in its local/map contract.
    Score is continuous even when the immediate front/rear interval is empty.
    """
    entry, own = context['entry'], context['corridors']['own']
    required = not own['route']['reachable'] or entry.get('entry_active', False)
    if not required:
        return dict(known=True, required=False, potential=1., target_m_s=None)
    if not entry.get('known') or not entry.get('target_lane'):
        return dict(known=False, required=True, potential=0., reason='unknown_merge_target')
    speed = state['speed']
    limit = min(CAP, state['allowed_speed'], entry['target_speed_limit'])
    distance = own['route']['distance_m']
    if not all(finite(v) and v >= 0 for v in (speed, limit, distance)) or limit <= 0:
        return dict(known=False, required=True, potential=0., reason='invalid_map_or_speed')
    length = TARGET_SETTINGS['vehicle_length_m']
    def vehicle(neighbor, direction):
        if neighbor is None:
            return None
        gap, relative = neighbor['bumper_gap_m'], neighbor['relative_speed_m_s']
        if not finite(gap) or not finite(relative) or gap < 0 or speed + relative < -1e-6:
            raise ValueError('Invalid merge neighbor')
        # Front-bumper positions relative to ego front; all demand cars are 4.8m.
        return dict(id=neighbor.get('id'), x=gap + length if direction == 'front' else -gap-length,
                    speed=max(0., speed+relative))
    try:
        front, rear = vehicle(entry.get('front'), 'front'), vehicle(entry.get('rear'), 'rear')
    except (ValueError, KeyError, TypeError):
        return dict(known=False, required=True, potential=0., reason='invalid_merge_neighbor')
    slots = [('between', front, rear, False)]
    if rear: slots.append(('behind_rear', rear, None, True))
    if front: slots.append(('ahead_front', None, front, True))
    candidates = []
    for name, ahead, behind, outer_unknown in slots:
        for horizon in TARGET_SETTINGS['preparation_horizons_s']:
            for i in range(17):
                target = limit*i/16.
                accel = (target-speed)/horizon
                if not -2.-1e-8 <= accel <= 1.5+1e-8:
                    continue
                travel = .5*(speed+target)*horizon
                deficit = 0.
                for other, direction in ((ahead,'front'),(behind,'rear')):
                    if other is None: continue
                    signed = other['x']+other['speed']*horizon-travel
                    gap = signed-length if direction=='front' else -signed-length
                    relative = other['speed']-target
                    end_gap = gap+(relative if direction=='front' else -relative)*3.1
                    need = required_gap(target,other['speed']) if direction=='front' else required_gap(other['speed'],target)
                    deficit += max(0., need-min(gap,end_gap))/max(10.,need)
                runway = travel+target*3.1+2.5
                if not own['route']['reachable']:
                    deficit += max(0.,runway-distance)/max(10.,distance)
                cost = deficit + .15*(limit-target)/limit + .05*abs(accel)/2. + .05*outer_unknown
                candidates.append((cost, horizon, name, target, ahead, behind, outer_unknown, deficit))
    if not candidates:
        return dict(known=False, required=True, potential=0., reason='no_reachable_speed_candidate')
    selected = min(candidates,key=lambda c:(c[0],c[1],c[2],-c[3]))
    # Opt-in v05 reward-target hysteresis. Legacy callers retain exactly the
    # original candidate choice; this never grants permission to change lanes.
    if previous_plan and previous_plan.get('known') and previous_plan.get('required'):
        previous_key = (previous_plan.get('slot'), previous_plan.get('front_id'), previous_plan.get('rear_id'))
        retained = [c for c in candidates if c[7] <= 1e-8 and
                    (c[2], c[4]['id'] if c[4] else None, c[5]['id'] if c[5] else None) == previous_key]
        if retained:
            alternative = min(retained,key=lambda c:(c[0],c[1],c[2],-c[3]))
            if alternative[0] <= selected[0] + switch_margin:
                selected = alternative
    cost,horizon,name,target,ahead,behind,unknown,deficit = selected
    # Readiness uses present gaps for the selected slot, not the optimized future
    # score alone. A future solution must still be approached to earn shaping.
    error = abs(speed-target)/max(5.,limit)
    for other,direction in ((ahead,'front'),(behind,'rear')):
        if other is None: continue
        gap = other['x']-length if direction=='front' else -other['x']-length
        end_gap = gap+(other['speed']-speed)*(3.1 if direction=='front' else -3.1)
        need = required_gap(speed,other['speed']) if direction=='front' else required_gap(other['speed'],speed)
        error += max(0.,need-min(gap,end_gap))/max(10.,need)
    return dict(known=True, required=True, potential=1./(1.+error), target_m_s=target,
        slot=name, front_id=ahead['id'] if ahead else None, rear_id=behind['id'] if behind else None,
        outer_neighbor_unknown=unknown, prediction_horizon_s=horizon, predicted_deficit=deficit,
        cost=cost, lateral_permission=False, projection='current velocities, bounded preparation acceleration')


def merge_speed_target(state, context, preview):
    plan = gap_plan(state, context)
    original = safe_speed_target(state, context, preview)
    if not plan.get('required') or not plan.get('known'):
        return original, plan
    # Remove only the immediate merge-gap intersection. Preserve occupied-lane
    # leaders, lower limits, ending-lane runway and visibility stopping bounds.
    longitudinal = deepcopy(context)
    longitudinal['entry'].update(front=None,rear=None)
    target = safe_speed_target(state, longitudinal, preview)
    if target['eligible']:
        target['target_m_s'] = min(target['target_m_s'],plan['target_m_s'])
        target['projection'] = 'gap preparation bounded by occupied-lane/map stopping constraints'
        target['original_target_reasons'] = original['reasons']
        target['clear_cruise'] = False
    return target, plan


def shaping_reward(before, after, *, terminated=False, weight=2., gamma=.99):
    # No clipping, phase reset, or target-switch reset: retain the potential
    # difference even when identity/visibility changes. True terminals use zero;
    # time-limit truncations keep the actual successor potential for GAE.
    successor = 0. if terminated else after['potential']
    return weight*(gamma*successor-before['potential'])


def quality_merge_eligible(samples):
    if not samples: return False
    return (sum(s['stopped'] for s in samples)*.1 <= 1.+1e-8
        and all(s['jerk'] is not None and abs(s['jerk']) <= 5. for s in samples)
        and sum(s['override'] for s in samples)/len(samples) <= .05
        and all(s['neighbor_acceleration'] is None or s['neighbor_acceleration'] >= -3. for s in samples)
        and sum(s['speed_match'] for s in samples)/len(samples) >= .9)
