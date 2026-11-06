"""Current-observation route/gap reference; candidate labels, not an accepted policy.

The 117-field flow ABI exposes route runway, actual map visibility and separate
ordinary ACC estimates for each corridor. Predictions assume observed neighbors
keep their current speeds. They are hypotheses, never future SUMO trajectories.
No simulator getter, private history, teacher intent input or native guard is
introduced here. Actual three-second maneuver commitment is actor-visible.
"""
from __future__ import annotations

from collections.abc import Mapping
import math

import numpy as np

from traffic import highway_stage1_teacher as legacy


REFERENCE_VERSION = 'highway_flow_reference_v1'
REFERENCE_STATUS = 'implemented_requires_native_qualification'
OBSERVATION_SIZE = 117
DT = .1
PLANNING_HORIZON_S = 6.
LANE_CHANGE_DURATION_S = 3.
REFERENCE_JERK_M_S3 = 2.
SPEED_GAIN = .4
COMFORT_DECELERATION_M_S2 = 2.
PASS_ADVANTAGE_M_S = 1.5
GAP_BUFFER_M = 3.
# This named layout is the frozen highway_flow_actor_v1 contract. Importing the
# environment is deliberately unnecessary; reference generation is pure CPU.
FIELD_INDEX = {
    'own_route_reachable': 51, 'own_route_end_distance_over1000': 52,
    'right_route_reachable': 53, 'right_route_end_distance_over1000': 54,
    'left_route_reachable': 55, 'left_route_end_distance_over1000': 56,
    'entry_visibility_over200': 57,
    'own_ACC_over4p5': 58, 'right_ACC_over4p5': 59,
    'left_ACC_over4p5': 60, 'occupied_worst_ACC_over4p5': 61,
    'right_front_lateral_offset_overwidth': 62,
    'right_front_lateral_speed_over3': 63,
    'left_front_lateral_offset_overwidth': 64,
    'left_front_lateral_speed_over3': 65,
    'own_legal_speed_overcap': 66, 'right_legal_speed_overcap': 67,
    'left_legal_speed_overcap': 68,
}


def _json_value(value):
    """Diagnostics are journal data: no NumPy scalars or nonfinite numbers."""
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, dict):
        return {key: _json_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_value(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError('Nonfinite flow reference diagnostic')
    return value


def _observation(value):
    obs = np.asarray(value, dtype=np.float64)
    if obs.shape != (OBSERVATION_SIZE,) or not np.isfinite(obs).all():
        raise ValueError('Flow reference requires one finite 117-field observation')
    if np.any(obs < -1.000001) or np.any(obs > 1.000001):
        raise ValueError('Flow reference observation is outside the normalized contract')
    return obs


def _gap(follower, leader):
    """Unchanged analytical 1.5-second/stopping-distance requirement, vectorized."""
    return legacy.MIN_GAP + np.maximum(legacy.HEADWAY * follower,
        DT * follower + follower**2 / 9. - leader**2 / 18.)


def _route(obs, name):
    reachable = bool(obs[FIELD_INDEX[name + '_route_reachable']] > .5)
    distance = max(0., float(obs[FIELD_INDEX[name + '_route_end_distance_over1000']]) * 1000.)
    return dict(reachable_without_lateral=reachable, runway_m=distance,
        exists=reachable or distance > 0.,
        legal_limit_m_s=max(0., float(obs[FIELD_INDEX[name + '_legal_speed_overcap']]) * legacy.SPEED_CAP),
        ordinary_response_m_s2=float(obs[FIELD_INDEX[name + '_ACC_over4p5']]) * 4.5)


def _forecast(speed, measured, limit, targets):
    """Jerk-limited candidate paths on the known 0.1-second control grid."""
    count = int(round(PLANNING_HORIZON_S / DT))
    v = np.full((len(targets), count + 1), speed)
    a = np.full_like(v, measured)
    x = np.zeros_like(v)
    for tick in range(1, count + 1):
        desired = np.minimum(targets, SPEED_GAIN * (limit - v[:, tick - 1]))
        accel = np.clip(desired, a[:, tick - 1] - REFERENCE_JERK_M_S3 * DT,
                        a[:, tick - 1] + REFERENCE_JERK_M_S3 * DT)
        a[:, tick] = np.maximum(-v[:, tick - 1] / DT,
            np.clip(accel, legacy.ACCELERATION_MIN, legacy.ACCELERATION_MAX))
        v[:, tick] = np.maximum(0., v[:, tick - 1] + DT * a[:, tick])
        x[:, tick] = x[:, tick - 1] + DT * v[:, tick]
    return v, a, x


def _gap_plan(*, speed, measured, limit, free_acceleration, front, rear,
              own_front, own_runway, destination_runway, destination_response,
              earliest_entry_s=0.):
    """Joint acceleration and entry-window search using only current tracks.

    Planning spans six seconds, including a complete three-second lateral
    overlap. A passing car outside the observed bounding pair is not invented.
    Every candidate is checked on the simulator's declared 0.1-second grid.
    """
    target_values = np.unique(np.r_[np.linspace(-COMFORT_DECELERATION_M_S2, 2., 17),
        np.clip(free_acceleration, -4.5, 2.), np.clip(destination_response, -4.5, 2.), 0.])
    target_values = np.minimum(target_values, free_acceleration)
    target_values = np.unique(target_values)
    v, a, x = _forecast(speed, measured, limit, target_values)
    times = np.arange(v.shape[1]) * DT
    front_margin = np.full_like(v, np.inf)
    rear_margin = np.full_like(v, np.inf)
    if front is not None:
        front_margin = front[0] + front[1] * times - x - _gap(v, front[1]) - GAP_BUFFER_M
    if rear is not None:
        rear_margin = rear[0] + x - rear[1] * times - _gap(rear[1], v) - GAP_BUFFER_M
    own_margin = np.full_like(v, np.inf)
    if own_front is not None:
        own_margin = own_front[0] + own_front[1] * times - x - _gap(v, own_front[1])
    overlap_ticks = int(round(LANE_CHANGE_DURATION_S / DT))
    choices = []
    first_start = max(0, int(math.ceil((earliest_entry_s - 1e-9) / DT)))
    for start in range(first_start, v.shape[1] - overlap_ticks):
        end = start + overlap_ticks
        good = ((front_margin[:, start:end + 1].min(axis=1) >= -1e-8)
            & (rear_margin[:, start:end + 1].min(axis=1) >= -1e-8)
            & (own_margin[:, :end + 1].min(axis=1) >= -1e-8)
            & (v[:, start:end + 1].max(axis=1) <= limit + legacy.SPEED_BUFFER + 1e-8)
            & (x[:, end] + legacy.MIN_GAP <= own_runway + 1e-8)
            & (x[:, end] + legacy.MIN_GAP <= destination_runway + 1e-8)
            & (a[:, max(1, start)] <= destination_response + 1e-8))
        for candidate in np.flatnonzero(good):
            # Progress first; stable deterministic ties prefer the free-flow
            # request and earlier entry. This is not persistent gap-ID memory.
            score = (float(x[candidate, -1]), -abs(float(target_values[candidate]) - free_acceleration), -start)
            choices.append((score, int(candidate), start, end))
    if not choices:
        return dict(feasible=False, acceleration_m_s2=None, entry_after_s=None,
            completion_after_s=None, reason='No observed bounded slot remains feasible through complete maneuver',
            horizon_s=PLANNING_HORIZON_S, prediction='constant observed neighbor speeds')
    _, selected, start, end = max(choices)
    def finite(value):
        return float(value) if math.isfinite(value) else None
    return dict(feasible=True, acceleration_m_s2=float(target_values[selected]),
        entry_after_s=start * DT, completion_after_s=end * DT,
        predicted_distance_m=float(x[selected, -1]),
        predicted_entry_speed_m_s=float(v[selected, start]),
        front_margin_m=finite(front_margin[selected, start:end + 1].min()),
        rear_margin_m=finite(rear_margin[selected, start:end + 1].min()),
        reason='Current observed bounding pair admits complete maneuver',
        horizon_s=PLANNING_HORIZON_S, prediction='constant observed neighbor speeds')


def _stopping_profile(distance, speed):
    distance = max(0., distance)
    b, jerk = COMFORT_DECELERATION_M_S2, REFERENCE_JERK_M_S3
    if distance >= b**3 / (6. * jerk**2):
        target = math.sqrt(2. * b * (distance - b**3 / (24. * jerk**2)))
        return target, -b * speed / target
    if distance > 0.:
        target = (4.5 * jerk)**(1. / 3.) * distance**(2. / 3.)
        return target, -(2. / 3.) * target * speed / distance
    return 0., -4.5 if speed > 0. else 0.


def _recovery_floor(speed):
    step = REFERENCE_JERK_M_S3 * DT
    n = max(1, math.ceil((math.sqrt(1. + 8. * speed / (DT * step)) - 1.) / 2.))
    return -(speed / (n * DT) + step * (n - 1) / 2.)


def reference_details(observation) -> dict:
    """Pure, auditable candidate target from exactly the actor's observation."""
    obs = _observation(observation)
    speed, measured = max(0., obs[0] * legacy.SPEED_CAP), float(obs[1]) * 4.5
    routes = {name: _route(obs, name) for name in ('own', 'right', 'left')}
    limit = min(max(0., obs[4] * legacy.SPEED_CAP), routes['own']['legal_limit_m_s'])
    buffered_limit = max(0., limit - legacy.SPEED_BUFFER)
    own_front = legacy._neighbor(obs, 6, speed)
    own_speed = legacy._following_target(speed, buffered_limit, own_front)
    ordinary = min(SPEED_GAIN * (limit - speed),
        float(obs[FIELD_INDEX['occupied_worst_ACC_over4p5']]) * 4.5,
        routes['own']['ordinary_response_m_s2'])
    soft = dict(speed_tracking=SPEED_GAIN * (own_speed - speed), occupied_response=ordinary)
    free_acceleration = min(legacy.ACCELERATION_MAX, *soft.values())
    requires_merge, known_merge = obs[28] > .5, obs[33] > .5
    merge_delta = int(np.rint(obs[35])) if known_merge else 0
    # The old lateral-projection fallback can also have delta zero while the
    # target is not yet adjacent. Only a declared positive map visibility
    # identifies an actual minor/yield entry in the flow ABI. Zero means no
    # declared yield entry, not an invisible junction requiring a stop.
    entry_visibility = max(0., float(obs[FIELD_INDEX['entry_visibility_over200']]) * 200.)
    minor_entry = bool(requires_merge and known_merge and merge_delta == 0 and entry_visibility > 0.)
    in_progress = bool(obs[48] > .5)
    masks = obs[30:33] > .5
    considered, rejected = [], []
    directions = []
    if requires_merge and known_merge and not in_progress:
        directions = [merge_delta]
    elif not requires_merge and not in_progress and own_front is not None:
        if own_front[0] < 90. and own_front[1] < buffered_limit - 2.:
            directions = [1, -1]
    for delta in directions:
        name = 'left' if delta == 1 else 'right' if delta == -1 else 'own'
        route = routes[name]
        if not route['exists']:
            rejected.append(dict(delta=delta, cause='unknown_or_absent_corridor'))
            continue
        if not requires_merge and not route['reachable_without_lateral']:
            rejected.append(dict(delta=delta, cause='passing_into_ending_lane_has_no_route_benefit'))
            continue
        front = legacy._neighbor(obs, 37 if requires_merge else 12 if delta == 1 else 18, speed)
        rear = legacy._neighbor(obs, 40 if requires_merge else 15 if delta == 1 else 21, speed)
        destination_limit = min(buffered_limit, max(0., route['legal_limit_m_s'] - legacy.SPEED_BUFFER))
        advantage = legacy._following_target(speed, destination_limit, front) - own_speed
        if not requires_merge and advantage <= PASS_ADVANTAGE_M_S:
            rejected.append(dict(delta=delta, cause='insufficient_sustained_speed_advantage'))
            continue
        plan = _gap_plan(speed=speed, measured=measured, limit=destination_limit,
            free_acceleration=free_acceleration, front=front, rear=rear, own_front=own_front,
            own_runway=routes['own']['runway_m'], destination_runway=route['runway_m'],
            destination_response=min(ordinary, route['ordinary_response_m_s2']),
            earliest_entry_s=max(0., float(obs[29]) * 5.) if delta else 0.)
        considered.append(dict(delta=delta, route=name, advantage_m_s=advantage, **plan))
    feasible = [p for p in considered if p['feasible']]
    selected = max(feasible, key=lambda p: (p['predicted_distance_m'], p['advantage_m_s'], p['delta'])) if feasible else None
    lane_delta = 0
    if selected:
        soft['joint_gap_plan'] = selected['acceleration_m_s2']
        # Consider destination response before initiating any lateral motion.
        soft['destination_response'] = routes[selected['route']]['ordinary_response_m_s2']
        if selected['entry_after_s'] <= 1e-8 and masks[selected['delta'] + 1]:
            lane_delta = selected['delta']
    elif requires_merge and considered and not minor_entry:
        # A mandatory merge may need longitudinal preparation before a complete
        # gap becomes feasible. Do not wait for overlap to discover the slower
        # destination leader. This is a soft estimate, never lane permission.
        pending = considered[0]
        soft['pending_destination_response'] = routes[pending['route']]['ordinary_response_m_s2']
        projected_front = legacy._neighbor(obs, 37, speed)
        projected_speed = legacy._following_target(speed, buffered_limit, projected_front)
        soft['pending_destination_following'] = SPEED_GAIN * (projected_speed - speed)
    slot_wait = dict(active=False, reason=None)
    if requires_merge and known_merge and not selected and not in_progress:
        front, rear = legacy._neighbor(obs, 37, speed), legacy._neighbor(obs, 40, speed)
        if rear is not None and rear[1] > 0.:
            deficit = legacy.required_gap(rear[1], speed) + GAP_BUFFER_M - rear[0]
            surplus = math.inf if front is None else front[0] - legacy.required_gap(speed, front[1]) - GAP_BUFFER_M
            if deficit > max(0., surplus):
                target = max(0., min(speed, rear[1] - 1.))
                soft['wait_for_observed_rear_to_pass'] = SPEED_GAIN * (target - speed)
                slot_wait.update(active=True, reason='No certified current slot; yield while rear gap dominates',
                    speed_target_m_s=target, certified_gap=False)
    ending = dict(active=False, profile_speed_m_s=None, feed_forward_m_s2=None,
        reason='Route continues or a feasible complete exit is planned')
    if requires_merge and not minor_entry:
        stop_speed, feed_forward = _stopping_profile(routes['own']['runway_m'] - legacy.MIN_GAP, speed)
        # Feed-forward only applies when this profile actually binds the speed
        # target. Its derivative is not a free-standing penalty on highway flow.
        active = selected is None and not in_progress and stop_speed <= own_speed + 1e-9
        ending.update(active=active, profile_speed_m_s=stop_speed, feed_forward_m_s2=feed_forward,
            reason='Ending profile is binding and no complete gap plan exists' if active else 'Ending profile is not the active speed target')
        if active:
            soft['ending_profile'] = feed_forward + SPEED_GAIN * (stop_speed - speed)
    entry = dict(active=minor_entry, visibility_m=entry_visibility,
        distance_m=None, gaps_safe=None, mandatory_stop_estimate_m_s2=None)
    mandatory = {}
    if minor_entry:
        distance = max(0., float(obs[36]) * 240.)
        front, rear = legacy._neighbor(obs, 37, speed), legacy._neighbor(obs, 40, speed)
        safe = ((front is None or front[0] >= legacy.required_gap(speed, front[1]))
            and (rear is None or rear[0] >= legacy.required_gap(rear[1], speed)))
        visibility = entry['visibility_m']
        stop_active = distance > visibility or not safe
        entry.update(distance_m=distance, gaps_safe=safe)
        if stop_active:
            mandatory['minor_entry_stop'] = (legacy.discrete_stop_speed(max(0., distance - legacy.MINOR_STOP_OFFSET_M)) - speed) / DT
            entry['mandatory_stop_estimate_m_s2'] = mandatory['minor_entry_stop']
            terminal = legacy.discrete_stop_speed(max(0., visibility - legacy.MINOR_STOP_OFFSET_M)) if safe else 0.
            remaining = distance - (visibility if safe else legacy.MINOR_STOP_OFFSET_M)
            if remaining > 0. and speed > terminal:
                soft['entry_distance'] = (terminal**2 - speed**2) / (2. * remaining)
    physical_floor = max(legacy.ACCELERATION_MIN, -speed / DT)
    desired = float(np.clip(max(_recovery_floor(speed), min(soft.values())), -4.5, 2.))
    ramped = float(np.clip(desired, measured - REFERENCE_JERK_M_S3 * DT,
                          measured + REFERENCE_JERK_M_S3 * DT))
    hard = float(np.clip(obs[43] * 4.5, -4.5, 2.))
    ceiling = max(physical_floor, min(hard, *mandatory.values())) if mandatory else max(physical_floor, hard)
    acceleration = float(np.clip(min(ramped, ceiling), physical_floor, 2.))
    uncertainty = []
    if requires_merge and not known_merge:
        uncertainty.append('unknown_merge_context')
    if considered and not feasible:
        uncertainty.append('no_certified_gap_plan')
    if acceleration < ramped - 1e-8:
        uncertainty.append('mandatory_braking_overrides_comfort_target')
    return _json_value(dict(version=REFERENCE_VERSION, status=REFERENCE_STATUS,
        acceleration_m_s2=acceleration, lane_change_delta=lane_delta,
        acceleration_supervision_eligible=not uncertainty,
        lane_supervision_eligible=not uncertainty,
        supervision_ineligibility_reasons=uncertainty,
        speed_m_s=speed, measured_acceleration_m_s2=measured, legal_limit_m_s=limit,
        desired_acceleration_m_s2=desired, ramped_acceleration_m_s2=ramped,
        hard_ceiling_m_s2=hard, achievable_acceleration_floor_m_s2=physical_floor,
        reference_ceiling_m_s2=ceiling, ordinary_response_estimate_m_s2=ordinary,
        soft_acceleration_components_m_s2=soft, mandatory_components_m_s2=mandatory,
        reference_slew_m_s3=(acceleration - measured) / DT,
        braking_override_soft_slew=acceleration < measured - REFERENCE_JERK_M_S3 * DT - 1e-8,
        routes=routes, considered_gap_plans=considered, rejected_lanes=rejected,
        selected_gap_plan=selected, gap_wait=slot_wait, ending_lane=ending, entry=entry,
        maneuver_in_progress=in_progress,
        intent_persistence='Actual maneuver lock only; deterministic current-observation gap selection has no hidden retained gap-ID state',
        native_authority='SUMO; estimates and predictions do not guarantee feasibility or comfort',
        qualification='Eligibility marks local label completeness only; native qualification remains required before supervision'))


def teacher_action(observation) -> np.ndarray:
    details = reference_details(observation)
    normalized = (details['acceleration_m_s2'] - legacy.ACCELERATION_MIN) / 3.25 - 1.
    return np.array([np.clip(normalized, -1., 1.), details['lane_change_delta']], np.float32)


def teacher_actions(observations: Mapping[str, np.ndarray]) -> dict[str, np.ndarray]:
    return {identity: teacher_action(obs) for identity, obs in observations.items()}
