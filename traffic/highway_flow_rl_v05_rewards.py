"""Continuous acceleration and stable, current-observation merge supervision."""
from copy import deepcopy
import math

from traffic.highway_flow_rl_v04_rewards import gap_plan, merge_speed_target
from traffic.highway_flow_rl_v03_rewards import safe_speed_target

REWARD_VERSION = 'highway_flow_rl_continuity_v05'
SETTINGS = dict(normal_jerk_limit_m_s3=4., step_s=.1, slot_switch_margin=.10,
                target_acceleration_m_s2=1.5, target_deceleration_m_s2=2.,
                initial_preview_s=3., jerk_acceptance_m_s3=5., tracking_tolerance_m_s=2.)
WEIGHTS = dict(speed_error=.45, jerk_huber=.09, route_detour=2., gap_preparation=2.,
               quality_merge=5., jerk_excess=.10, merge_tracking=.12)


def condition_acceleration(requested, measured, ceiling, *, enabled=True, settings=SETTINGS):
    if any(not math.isfinite(v) for v in (requested, measured, ceiling)):
        raise ValueError('Acceleration conditioning requires finite measured state')
    delta = settings['normal_jerk_limit_m_s3'] * settings['step_s']
    lower, upper = measured-delta, measured+delta
    # Explicit safety constraints take priority over normal comfort. A maximum
    # braking request also bypasses the downward rate limit; release is bounded.
    conditioned = max(lower, min(upper, requested)) if enabled else requested
    if enabled and requested < lower and (ceiling < lower or requested <= -4.5 + 1e-6):
        conditioned = requested
    conditioned = max(-4.5, min(2., conditioned))
    applied = min(conditioned, ceiling)
    return dict(requested=requested, measured=measured, conditioned=conditioned,
                expected_applied=applied, ceiling=ceiling, normal_lower=lower, normal_upper=upper,
                changed=abs(conditioned-requested)>1e-4,
                safety_exception=enabled and applied < lower-1e-6,
                raw_requested_jerk=(requested-measured)/settings['step_s'])


def stable_target(state, context, preview, previous=None, *, settings=SETTINGS):
    """Reward supervision only. No target is converted into an actor command."""
    reference, _ = merge_speed_target(state, context, preview)
    previous = previous or {}
    plan = gap_plan(state, context, previous_plan=previous.get('plan'),
                    switch_margin=settings['slot_switch_margin'])
    if not plan.get('required') or not plan.get('known'):
        return reference, plan, reference
    longitudinal = deepcopy(context)
    longitudinal['entry'].update(front=None, rear=None)
    safe = safe_speed_target(state, longitudinal, preview)
    target = deepcopy(safe)
    if safe['eligible']:
        desired = min(safe['target_m_s'], plan['target_m_s'])
        old = previous.get('target', {}).get('target_m_s')
        if old is None or not previous.get('plan', {}).get('required'):
            old = min(desired, state['speed'] + settings['target_acceleration_m_s2']*settings['initial_preview_s'])
        bounded = max(old-settings['target_deceleration_m_s2']*settings['step_s'],
                      min(old+settings['target_acceleration_m_s2']*settings['step_s'], desired))
        target.update(target_m_s=min(safe['target_m_s'], bounded), clear_cruise=False,
                      unsmoothed_target_m_s=desired,
                      projection='persistent gap reward target; occupied-lane/map bounds immediate')
    return target, plan, reference


def extra_rewards(realized_jerk, requested_jerk, target, next_speed, *, merge_active, weights=WEIGHTS):
    # Smooth executed actions must not conceal harsh actor requests during PPO.
    peak = max(abs(requested_jerk), abs(realized_jerk) if realized_jerk is not None else 0.)
    excess = max(0., peak-SETTINGS['jerk_acceptance_m_s3']) / SETTINGS['jerk_acceptance_m_s3']
    tracking = 0.
    if merge_active and next_speed is not None and target['eligible']:
        tracking = -weights['merge_tracking'] * min(1., abs(next_speed-target['target_m_s'])/SETTINGS['tracking_tolerance_m_s'])
    # A cost rather than a positive per-step reward prevents farming by lingering.
    return dict(jerk_excess=-weights['jerk_excess']*excess*excess, merge_tracking=tracking)
