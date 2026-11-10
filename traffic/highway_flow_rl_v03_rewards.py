"""Pure pre-action speed targets and uncapped comfort costs for the v03 pilot.

Targets supervise rewards only. They do not change native commands or masks.
Merge projection uses current speeds over the existing three-second maneuver;
it is a declared prediction, not future simulator state or a safety certificate.
"""
from __future__ import annotations

import math

from traffic.highway_flow_rl_v02_rewards import CAP, MASK_VERSION, optional_detour_evidence

REWARD_VERSION = 'highway_flow_rl_safe_speed_huber_v03'
DEFAULT_WEIGHTS = dict(speed_error=.30, jerk_huber=.09, route_detour=2.)
TARGET_SETTINGS = dict(speed_deadband_m_s=.5, jerk_scale_m_s3=2.,
    headway_s=1.5, reaction_s=.3, follower_braking_m_s2=4.5,
    leader_braking_m_s2=9., preview_braking_m_s2=2., min_gap_m=2.5,
    merge_horizon_s=3.)


def finite(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def validate_weights(weights=None):
    result = dict(DEFAULT_WEIGHTS if weights is None else weights)
    if set(result) != set(DEFAULT_WEIGHTS) or any(not finite(v) or v < 0 for v in result.values()):
        raise ValueError('v03 requires finite nonnegative speed_error/jerk_huber/route_detour weights')
    return result


def required_gap(follower, leader):
    return max(2.5 + 1.5 * follower,
               2.5 + .3 * follower + follower * follower / 9. - leader * leader / 18.)


def front_speed_bound(gap, leader_speed, horizon=0.):
    """Largest candidate speed whose projected bumper gap covers both bounds."""
    if gap + leader_speed * horizon < required_gap(0., leader_speed):
        return 0.
    lo, hi = 0., CAP
    for _ in range(48):
        speed = (lo + hi) / 2.
        if gap + (leader_speed - speed) * horizon >= required_gap(speed, leader_speed):
            lo = speed
        else:
            hi = speed
    return lo


def rear_speed_bound(gap, rear_speed, horizon=3.):
    def safe(speed):
        return gap + (speed - rear_speed) * horizon >= required_gap(rear_speed, speed)
    if not safe(CAP):
        return math.inf
    lo, hi = 0., CAP
    if safe(lo):
        return lo
    for _ in range(48):
        speed = (lo + hi) / 2.
        if safe(speed): hi = speed
        else: lo = speed
    return hi


def map_evidence(graph, state):
    distances = graph.distances(state['lane_id'], state['lane_position'], state['route'])
    lower = [dict(lane_id=lane, distance_m=max(0., distance),
                  limit_m_s=float(graph.speed_limits[lane]))
             for lane, distance in distances.items()
             if graph.speed_limits[lane] < state['allowed_speed'] - 1e-6]
    return dict(known=bool(distances), lower_limits=lower,
                source='current route/map distances; no future traffic')


def safe_speed_target(state, context, preview):
    """Freeze a feasible speed interval before inspecting the selected action.

    Physically occupied lanes constrain ordinary following. Required lateral
    merges additionally constrain the target with both front and rear gaps.
    An optional adjacent lane has no effect until physically occupied.
    """
    audit = dict(eligible=False, target_m_s=None, reasons=[], bounds=[],
                 clear_cruise=False, obligations=[], settings=dict(TARGET_SETTINGS))
    def abstain(reason):
        audit['reasons'].append(reason)
        return audit
    try:
        speed, allowed = state['speed'], state['allowed_speed']
        if not finite(speed) or speed < 0 or not finite(allowed) or allowed <= 0:
            return abstain('invalid_ego_speed_or_limit')
        corridors, occupied = context['corridors'], context['occupied_lanes']
        own = corridors['own']
        if not occupied or own['lane_id'] not in occupied or not preview.get('known'):
            return abstain('unknown_map_or_occupancy')
        relevant = [r for r in corridors.values() if r.get('lane_id') in occupied]
        if {r['lane_id'] for r in relevant} != set(occupied):
            return abstain('unrepresented_occupied_lane')
        upper, lower = min(CAP, allowed), 0.
        front_present = False
        def add_bound(name, value):
            nonlocal upper
            upper = min(upper, max(0., value))
            audit['bounds'].append(dict(reason=name, ceiling_m_s=max(0., value)))
        def neighbor_values(neighbor):
            gap, relative = neighbor['bumper_gap_m'], neighbor['relative_speed_m_s']
            if not finite(gap) or gap < 0 or not finite(relative) or speed + relative < -1e-6:
                raise ValueError('invalid neighbor')
            return float(gap), max(0., speed + relative)
        for row in relevant:
            route, legal = row['route'], row['legal_speed_m_s']
            if not row['known'] or not route['known'] or not finite(legal) or legal <= 0:
                return abstain('unknown_corridor')
            if not finite(route['distance_m']) or route['distance_m'] < 0:
                return abstain('invalid_route_runway')
            add_bound('occupied_lane_limit', legal)
            if row['front'] is not None:
                front_present = True
                add_bound('occupied_leader', front_speed_bound(*neighbor_values(row['front'])))
            if not route['reachable']:
                audit['obligations'].append('ending_lane')
                # Preserve one complete maneuver plus a stop before the lane end.
                runway = max(0., route['distance_m'] - speed * 3.1 - 2.5)
                add_bound('ending_lane_runway', math.sqrt(4. * runway))
        for row in preview.get('lower_limits', []):
            distance, limit = row['distance_m'], row['limit_m_s']
            if not finite(distance) or distance < 0 or not finite(limit) or limit <= 0:
                return abstain('invalid_upcoming_limit')
            add_bound('upcoming_limit', math.sqrt(limit * limit + 4. * max(0., distance - 2.5)))
        # A possible cut-in is not assumed safe merely because it is not yet in
        # the occupied-lane list. Abstention is explicitly counted in coverage.
        for row in corridors.values():
            if row.get('lane_id') not in occupied and row.get('front') is not None:
                if abs(row.get('front_lateral_speed_over3', 0.)) > .05 / 3.:
                    return abstain('unresolved_neighbor_lateral_motion')
        entry = context['entry']
        merge_required = not own['route']['reachable'] or entry.get('entry_active')
        if merge_required:
            audit['obligations'].append('merge_or_yield')
            if not entry.get('known') or not entry.get('target_lane'):
                return abstain('required_merge_target_unknown')
            limit = entry['target_speed_limit']
            if not finite(limit) or limit <= 0:
                return abstain('invalid_merge_limit')
            add_bound('merge_target_limit', limit)
            distance = entry['distance_to_start_m']
            if not finite(distance): return abstain('invalid_merge_distance')
            if entry.get('entry_active'):
                visibility = entry['minor_link_visibility_m']
                if not finite(visibility) or visibility <= 0:
                    return abstain('yield_visibility_unknown')
                if distance > visibility:
                    add_bound('unseen_yield_stop', math.sqrt(4. * max(0., distance - 2.5)))
            if entry['front'] is not None:
                add_bound('merge_front_gap', front_speed_bound(*neighbor_values(entry['front']), horizon=3.))
            if entry['rear'] is not None:
                lower = rear_speed_bound(*neighbor_values(entry['rear']), horizon=3.)
            if lower > upper + 1e-6:
                return abstain('no_current_admissible_merge_gap')
        audit.update(eligible=True, target_m_s=upper, lower_bound_m_s=lower,
            legal_speed_m_s=min(CAP, allowed, own['legal_speed_m_s']),
            clear_cruise=not audit['obligations'] and not front_present and len(occupied) == 1,
            projection='current-speed three-second merge interval plus conservative stopping bounds')
        return audit
    except (KeyError, TypeError, ValueError, OverflowError):
        return abstain('missing_or_invalid_evidence')


def speed_error_reward(target, next_speed, weight=.30):
    if next_speed is None: return 0.  # Disappeared/arrived state is not invented.
    if not finite(next_speed) or next_speed < -1e-6: raise ValueError('Invalid measured speed')
    if not target['eligible']: return 0.
    reference = target['target_m_s']
    error = min(1., max(0., abs(next_speed - reference) - .5) / max(reference, 5.))
    return -weight * error


def jerk_reward(jerk, weight=.09):
    if jerk is None: return 0.
    if not finite(jerk): raise ValueError('Invalid measured jerk')
    x = abs(jerk) / 2.
    return -weight * (.5 * x * x if x <= 1. else x - .5)


class BrakingDiagnostic:
    """Count persistent commands, never alter actions or censor safety metrics."""
    def __init__(self):
        self.rows = {}

    def observe(self, agent, time_s, target, speed, requested, safe_ceiling):
        row = self.rows.setdefault(agent, dict(decisions=0, target_eligible=0,
            clear_cruise_decisions=0, streak=0, events=0, reaccelerations=0,
            clear_cruise_speed_fraction_sum=0.,
            awaiting_reacceleration=False, last_time=None))
        if row['last_time'] is None or abs(time_s - row['last_time'] - .1) > 1e-5:
            row['streak'] = 0
            row['awaiting_reacceleration'] = False
        row['last_time'] = time_s
        row['decisions'] += 1
        row['target_eligible'] += int(target['eligible'])
        row['clear_cruise_decisions'] += int(target.get('clear_cruise', False))
        if target.get('clear_cruise'):
            row['clear_cruise_speed_fraction_sum'] += speed / target['legal_speed_m_s']
        eligible = (target['eligible'] and not target['obligations']
            and target['target_m_s'] >= speed + .5 and finite(safe_ceiling) and safe_ceiling >= .2)
        braking = eligible and requested < -.2
        if not eligible:
            row['awaiting_reacceleration'] = False
        row['streak'] = row['streak'] + 1 if braking else 0
        if row['streak'] == 3:
            row['events'] += 1
            row['awaiting_reacceleration'] = True
        if row['awaiting_reacceleration'] and requested > .2:
            row['reaccelerations'] += 1
            row['awaiting_reacceleration'] = False
        return dict(eligible=eligible, braking=braking, persistent_event=row['streak'] == 3)
