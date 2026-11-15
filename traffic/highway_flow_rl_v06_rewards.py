"""V06 merge training objective (Phase 1 of documentation/highway-merge-training-plan-v06.md).

Pure functions only; the environment wiring is environments/highway_flow_rl_v06_env.py.
Nothing here changes the safety shield, native guards, action contract or the
117-field observation size. Weights are PROVISIONAL until the Phase 2 offline
reward audit; the environment refuses any other weights.
"""
import math

import numpy as np

REWARD_VERSION = 'highway_flow_rl_merge_v06'
OBSERVATION_VERSION = 'highway_flow_actor117_v1_faded_merge_v06'
WEIGHTS_STATUS = 'provisional: set by the Phase 2 offline reward audit before any training'

# v04 base terms (unchanged values from v04 DEFAULT_WEIGHTS) plus v06 additions.
WEIGHTS = dict(speed_error=.30, jerk_huber=.09, route_detour=2., gap_preparation=2., quality_merge=5.,
               merge_tracking=.50, jerk_excess=1.50, avoidable_braking=.05)
BASE_KEYS = ('speed_error', 'jerk_huber', 'route_detour', 'gap_preparation', 'quality_merge')

SETTINGS = dict(
    step_s=.1,
    fade_steps=10,                 # 1 s fade for actor merge inputs and the merge-guidance potential
    jerk_acceptance_m_s3=5.,       # same threshold as the per-car gate
    tracking_tolerance_m_s=2.,     # same tolerance as the gate's speed_match
    braking_request_m_s2=-.2,      # same as BrakingDiagnostic
    braking_scale_m_s2=2.,
    free_headway_s=1.5,            # no leader inside this time headway
    late_lane_fraction=.2,         # last 20% of the acceleration lane
)

# Flow117 actor indices of the merge inputs (legacy block, unchanged positions):
# 27 merge distance, 28 requires_merge, 33 merge_context_known, 34 target lane
# index, 35 target lane delta, 36 distance to entry start, 37-39 target front
# gap / relative speed / present, 40-42 target rear gap / relative speed / present.
# Not faded: the mask (30-32), the acceleration ceiling (43) and every safety input.
FADE_INDICES = (27, 28, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42)


def fade_observation(previous, raw, *, steps=SETTINGS['fade_steps']):
    """Move each merge input toward its raw value by at most 1/steps per decision.

    A 0->1 flip therefore takes `steps` decisions. Continuous fields that move
    less than 1/steps per decision pass through unchanged. Returns a new array;
    `raw` is never modified (the safety guard and critic keep raw values).
    """
    raw = np.asarray(raw, np.float32)
    out = raw.copy()
    if previous is None:
        return out
    previous = np.asarray(previous, np.float32)
    index = list(FADE_INDICES)
    rate = 1. / steps
    out[index] = previous[index] + np.clip(raw[index] - previous[index], -rate, rate)
    return out


def blend_potential(plan, run, *, steps=SETTINGS['fade_steps']):
    """Merge-guidance potential that fades from 1 to the plan's value over `steps` decisions.

    `run` counts consecutive decisions (states) in which a merge has been
    required, 1 for the first. The potential is a function of (state, run), so
    shaping stays potential-based on that augmented state.
    """
    result = dict(plan)
    if not plan.get('required'):
        return result
    weight = min(1., max(0, run - 1) / steps)
    result.update(raw_potential=plan['potential'], blend_weight=weight,
                  potential=(1. - weight) * 1. + weight * plan['potential'])
    return result


def jerk_excess(realized_jerk, requested_jerk, weight):
    peak = max(abs(requested_jerk), abs(realized_jerk) if realized_jerk is not None else 0.)
    excess = max(0., peak - SETTINGS['jerk_acceptance_m_s3']) / SETTINGS['jerk_acceptance_m_s3']
    return -weight * excess * excess


def merge_tracking(target, next_speed, merge_active, weight):
    """Cost for missing the gate's own target (|speed - target| > 2 m/s saturates)."""
    if not merge_active or next_speed is None or not target.get('eligible'):
        return 0.
    error = abs(next_speed - target['target_m_s']) / SETTINGS['tracking_tolerance_m_s']
    return -weight * min(1., error)


def free_road(ego_speed, front):
    """No leader inside the free headway (time gap at current speed)."""
    if front is None:
        return True
    gap = front.get('bumper_gap_m')
    if gap is None or not math.isfinite(gap):
        return False
    return gap >= SETTINGS['free_headway_s'] * max(ego_speed, 0.)


def avoidable_braking(diagnosis, requested, ego_speed, front, weight):
    """Cost on the actor's own braking request when nothing requires braking.

    `diagnosis` is BrakingDiagnostic.observe's result: eligible already means an
    eligible target with no obligations, target at least 0.5 m/s above speed
    (no lower speed target) and a safety ceiling of at least 0.2 m/s2 (not binding).
    """
    if not diagnosis.get('eligible') or requested >= SETTINGS['braking_request_m_s2']:
        return 0.
    if not free_road(ego_speed, front):
        return 0.
    depth = (SETTINGS['braking_request_m_s2'] - requested) / SETTINGS['braking_scale_m_s2']
    return -weight * min(1., depth)


def late_in_acceleration_lane(context, lane_length_m, merge_active):
    """Metric only: merge pending and within the last 20% of a lane that ends."""
    route = (context or {}).get('corridors', {}).get('own', {}).get('route', {})
    if not merge_active or route.get('reachable', True) or not lane_length_m:
        return False
    distance = route.get('distance_m')
    return distance is not None and distance <= lane_length_m and \
        distance <= SETTINGS['late_lane_fraction'] * lane_length_m


def raw_request_jerk_check(report, limit=SETTINGS['jerk_acceptance_m_s3']):
    """Extra per-car evaluation check: the actor's raw requested jerk stays under 5 m/s3.

    Stops a command rate limit from hiding a jerky policy. Missing evidence fails.
    """
    per_car = (report.get('v05_metrics') or {}).get('per_car')
    assigned = [a for a, c in (report.get('smooth_control') or {}).get('per_car', {}).items()
                if c.get('assigned_controlled')]
    cars = {}
    for agent in assigned:
        peak = (per_car or {}).get(agent, {}).get('peak_raw_requested_jerk_m_s3')
        cars[agent] = dict(peak_raw_requested_jerk_m_s3=peak,
                           passed=peak is not None and peak <= limit + 1e-9)
    return dict(limit_m_s3=limit, passed=bool(cars) and all(c['passed'] for c in cars.values()), per_car=cars)


def v06_gates(report):
    """Original development gates (no exemption) plus the raw-request jerk check."""
    from experiments.run_highway_flow_rl_v05 import development_gates
    result = development_gates(report)
    raw = raw_request_jerk_check(report)
    result['raw_request_jerk'] = raw
    if not raw['passed']:
        result['failures'] = list(result['failures']) + ['raw_request_jerk']
        result['passed'] = False
    return result
