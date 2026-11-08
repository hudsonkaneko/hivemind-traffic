"""Explicit comfort filter for a frozen actor; never replaces the final guard."""
import math


def shape_acceleration(ego, requested, front, allowed_speed, settings):
    """Bound physical acceleration, anticipate following, then limit routine jerk.

    The caller MUST apply guarded_step afterwards: smoothing must never delay a
    required safety intervention. Steering and pretrained action decoding are
    unchanged. This is controller conditioning, not learned policy improvement.
    """
    dt = .1
    bounded = max(-4.5, min(2., float(requested)))
    desired_gap = settings['standstill_gap_m'] + settings['headway_s'] * ego['speed']
    following_target = None
    if front:
        following_target = (settings['gap_gain_s2'] * (front['bumper_gap_m'] - desired_gap)
                            + settings['relative_speed_gain_s'] * front['relative_speed_m_s'])
    speed_target = settings['speed_gain_s'] * (min(26.8224, allowed_speed) - ego['speed'])
    target = max(-4.5, min(bounded, speed_target,
                          following_target if following_target is not None else math.inf))
    previous = ego['acceleration']
    delta = settings['jerk_limit_m_s3'] * dt
    conditioned = max(-4.5, min(2., max(previous-delta, min(previous+delta, target))))
    return conditioned, dict(raw_request= float(requested), range_bounded=bounded,
        following_target=following_target, speed_target=speed_target,
        comfort_target=target, conditioned_request=conditioned,
        previous_applied_acceleration=previous, desired_gap_m=desired_gap,
        front=front, range_clipped=abs(bounded-requested)>1e-8,
        comfort_changed=abs(target-bounded)>1e-8,
        jerk_limited=abs(conditioned-target)>1e-8)


def control_metrics(actions):
    """Separate overlapping command changes without changing the raw-rate metric."""
    count = len(actions)
    changed = lambda name: sum(bool(a['longitudinal'][name]) for a in actions)
    gap_rows = [a['longitudinal']['front'] for a in actions if a['longitudinal']['front']]
    headways = [a['longitudinal']['front']['bumper_gap_m']/a['ego_before']['speed']
        for a in actions if a['longitudinal']['front'] and a['ego_before']['speed']>1.]
    ttcs = [f['bumper_gap_m']/(-f['relative_speed_m_s'])
            for f in gap_rows if f['relative_speed_m_s']<-.01]
    after_filter = sum(a['safety']['conditioned_request_changed'] for a in actions)
    return dict(decisions=count,range_clipped_decisions=changed('range_clipped'),
        comfort_changed_decisions=changed('comfort_changed'),
        jerk_limited_decisions=changed('jerk_limited'),
        final_guard_changed_decisions=after_filter,
        final_guard_changed_fraction=after_filter/max(1,count),
        minimum_recorded_leader_gap_m=min((f['bumper_gap_m'] for f in gap_rows),default=None),
        minimum_recorded_leader_time_headway_s=min(headways,default=None),
        minimum_recorded_leader_ttc_s=min(ttcs,default=None),
        scope='Current graph-selected front vehicle; not all lateral conflicts. Counts overlap.')
