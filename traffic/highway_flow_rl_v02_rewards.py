"""Conservative current-evidence rewards; no simulator calls or teacher labels."""
from __future__ import annotations

import math

CAP = 26.8224
REWARD_VERSION = 'highway_flow_rl_current_evidence_v02'
MASK_VERSION = 'native_v05_complete_maneuver_runway_unchanged_v1'
DEFAULT_REWARD_WEIGHTS = {'cruise_speed': .1, 'route_detour': .5}


def validate_weights(weights=None):
    result = dict(DEFAULT_REWARD_WEIGHTS if weights is None else weights)
    if set(result) != set(DEFAULT_REWARD_WEIGHTS) or any(
            isinstance(v, bool) or not isinstance(v, (int, float))
            or not math.isfinite(v) or not 0. <= v <= 1. for v in result.values()):
        raise ValueError('v02_reward_weights require cruise_speed/route_detour in [0, 1]')
    return {k: float(v) for k, v in result.items()}


def map_preview(graph, state, *, lane_change_duration=3.):
    """Current route map only; conservative braking+one-maneuver lookahead.

    This is reward eligibility metadata, not a new actor input. An imminent
    lower limit or declared yielding link removes the cruise bonus. No future
    vehicle motion, native intent, or trajectory is queried.
    """
    speed = max(0., float(state['speed']))
    horizon = speed * speed / 4. + speed * (lane_change_duration + .1) + 2.5
    distances = graph.distances(state['lane_id'], state['lane_position'], state['route'])
    nearby = {lane: distance for lane, distance in distances.items() if distance <= horizon}
    limits = [graph.speed_limits[lane] for lane in nearby]
    yields = [lane for lane, distance in nearby.items()
              if lane in graph.yield_entries
              and distance + graph.lanes[lane]['length'] <= horizon]
    return dict(known=bool(nearby), horizon_m=horizon,
                minimum_legal_speed_m_s=min(limits) if limits else None,
                yielding_link_ahead=bool(yields), yielding_source_lanes=sorted(yields),
                source='current route map; no future motion')


def cruise_speed_component(state, context, preview, *, maneuver_in_progress=False, weight=.1):
    """Positive [0, weight] bonus: weight * (v/legal)^2 for evidenced free cruise.

    Zero is added during braking, constrained following, entry, lateral motion, a route
    obligation, approaching a lower limit/end, or missing evidence. Overspeed
    never earns this bonus. Existing progress/jerk/safety rewards remain intact.
    An observed leader is eligible only with >=3 s headway, >=1 m/s opening
    speed and >=0.2 m/s2 current response estimate. Squaring makes this more
    than another linear distance/progress term over a route.
    """
    own = context['corridors']['own']
    route = own['route']
    speed = float(state['speed'])
    legal = min(CAP, float(state['allowed_speed']), float(own['legal_speed_m_s']))
    reasons = []
    if state.get('lane_identity', {}).get('kind') != 'through':
        reasons.append('not_through_lane')
    if not own.get('known') or not route.get('known') or not route.get('reachable'):
        reasons.append('route_obligation_or_unknown')
    leader = own.get('front')
    leader_headway = (leader['bumper_gap_m'] / speed if speed > .1 else None) if leader else None
    opening_leader = bool(leader and leader_headway is not None and leader_headway >= 3.
        and leader['relative_speed_m_s'] >= 1.
        and own['native_response_estimate_m_s2'] >= .2)
    if leader is not None and not opening_leader:
        reasons.append('leader_not_safely_opening')
    if maneuver_in_progress or abs(state.get('lateral_position', 0.)) > .05 or abs(state.get('lateral_speed', 0.)) > .05:
        reasons.append('lateral_maneuver')
    if context.get('entry', {}).get('entry_active') or context.get('entry', {}).get('known'):
        reasons.append('entry_context')
    if state['acceleration'] < -.05 or context['occupied_worst_acceleration_m_s2'] < -1e-6:
        reasons.append('braking_or_response_bound')
    if any(abs(row.get('front_lateral_speed_over3', 0.)) > .05 / 3.
           or abs(row.get('front_lateral_offset_overwidth', 0.)) > .02
           for row in context['corridors'].values()):
        reasons.append('neighbor_lateral_motion')
    if not preview.get('known') or preview.get('minimum_legal_speed_m_s') is None:
        reasons.append('map_preview_unknown')
    elif preview['minimum_legal_speed_m_s'] < legal - 1e-6:
        reasons.append('lower_limit_ahead')
    if preview.get('yielding_link_ahead'):
        reasons.append('yield_ahead')
    if route.get('distance_m', 0.) <= preview.get('horizon_m', math.inf):
        reasons.append('route_end_approach')
    if legal <= 0. or not 0. <= speed <= legal + 1e-4:
        reasons.append('invalid_or_overspeed')
    raw = min(1., max(0., speed / legal)) if legal > 0. and not reasons else 0.
    return weight * raw ** 2, dict(eligible=not reasons, exclusion_reasons=reasons,
        speed_m_s=speed, legal_speed_m_s=legal, normalized=raw ** 2,
        opening_leader_exception=opening_leader, leader_headway_s=leader_headway,
        formula='weight * (pre_action_speed / min(global_cap, executed_allowed_speed, current_lane_limit))^2',
        weight=weight, map_preview=preview)


def optional_detour_evidence(state, context, delta, lane_kinds, *, lane_change_duration=3.):
    """Classify only optional through -> ending auxiliary moves without benefit.

    Route-required changes and all other lane types abstain. A slower leader
    plus an improved adjacent response/speed and complete out-and-back runway
    is a conservative passing-benefit exception, not a safety certificate.
    This classification never changes an action mask or native command.
    """
    result = dict(optional_detour=False, reason='stay_or_unknown', passing_benefit=False)
    if delta not in (-1, 1):
        return result
    own = context['corridors']['own']
    target = context['corridors']['right' if delta == -1 else 'left']
    origin_route, target_route = own['route'], target['route']
    result.update(source_lane=own.get('lane_id'), target_lane=target.get('lane_id'),
                  target_route_reachable=target_route.get('reachable'),
                  target_runway_m=target_route.get('distance_m'))
    if not all((own.get('known'), target.get('known'), origin_route.get('known'), target_route.get('known'))):
        return result
    if lane_kinds.get(own['lane_id']) != 'through' or lane_kinds.get(target['lane_id']) != 'auxiliary':
        return dict(result, reason='not_through_to_auxiliary')
    if not origin_route['reachable'] or target_route['reachable']:
        return dict(result, reason='required_or_route_preserving_change')
    # A positive declared entry is an obligation/ambiguity, not an optional detour.
    if context.get('entry', {}).get('entry_active'):
        return dict(result, reason='entry_obligation')
    speed = max(0., state['speed'])
    duration = 2. * (lane_change_duration + .1)
    required_runway = speed * duration + duration * duration + 2.5
    leader = own.get('front')
    passing = False
    if leader is not None:
        leader_speed = max(0., speed + leader['relative_speed_m_s'])
        target_front = target.get('front')
        target_speed = min(target['legal_speed_m_s'],
                           max(0., speed + target_front['relative_speed_m_s']) if target_front else CAP)
        # Include the present lead gap and vehicle/clearance length in addition
        # to both complete lane changes; do not infer successful future passing.
        required_runway += max(0., leader['bumper_gap_m']) + state.get('length', 4.8) + 10.
        passing = (leader_speed <= own['legal_speed_m_s'] - 1.5
                   and target_speed >= leader_speed + 1.5
                   and target['native_response_estimate_m_s2'] >= own['native_response_estimate_m_s2'] + .2
                   and target_route['distance_m'] >= required_runway)
    return dict(result, optional_detour=not passing, passing_benefit=bool(passing),
                return_runway_required_m=required_runway,
                reason='plausible_passing_benefit' if passing else 'optional_ending_lane_without_benefit')
