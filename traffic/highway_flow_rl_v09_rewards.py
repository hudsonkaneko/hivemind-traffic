"""V09 merge reward: three targeted changes to the v06 objective (Claude proposal, November 17, 2026).

Pure functions only; the environment wiring is environments/highway_flow_rl_v09_env.py.
Evidence: documentation/highway-merge-training-plan-v09.md (round-9 reward analysis).

1. Merge-speed tracking keeps its slope past the 2 m/s tolerance. In v06 the cost
   saturates at 2 m/s, so a merging car 2-8 m/s off its target gets no signal to
   close the gap (37% of round 9's I-110 merge decisions, 53-63% on the ramps).
   Inside 2 m/s the cost is identical to v06; it now keeps rising to a cap at 8 m/s.
2. Merge quality pays in proportion to the speed-match fraction of the gate's
   own quality window, instead of only at 90% (never reached in any validation).
   It is paid once, at the stable-merge decision, only when the car's own
   conditions hold (stops, realized jerk, override fraction), and it is scaled by
   the fraction of window samples without neighbor braking below -3 m/s2.
3. The jerk-excess cost ignores the raw request during each car's fixed
   5-decision insertion lag (the window the raw-request check already skips);
   realized jerk is still charged there. In round 9, 99.6% of this cost fell in
   those decisions, where SUMO's 0 m/s2 insertion makes any acceleration request
   look like a jerk.
Everything else (v04/v06 terms, weights, safety shield, observation, actions) is
unchanged.
"""
from traffic import highway_flow_rl_v06_rewards as v06

REWARD_VERSION = 'highway_flow_rl_merge_v09'
SETTINGS = dict(
    tracking_tolerance_m_s=2.,       # unchanged v06 tolerance (also the gate's speed-match tolerance)
    tracking_cap=4.,                 # cost stops rising at 4 tolerances = 8 m/s (v06: 1 tolerance = 2 m/s)
    graded_quality_weight=5.,        # same scale as the binary quality_merge bonus
    quality_max_stop_seconds=1.,     # the gate's own-car conditions (traffic.highway_flow_rl_v04_rewards)
    quality_max_jerk_m_s3=5.,
    quality_max_override_fraction=.05,
    quality_neighbor_braking_m_s2=-3.,
    insertion_lag_decisions=v06.INSERTION_LAG_DECISIONS,
)


def merge_tracking(target, next_speed, merge_active, weight, settings=SETTINGS):
    """Cost for missing the gate's target; v06 inside 2 m/s, still rising to a cap at 8 m/s."""
    if not merge_active or next_speed is None or not target.get('eligible'):
        return 0.
    error = abs(next_speed - target['target_m_s']) / settings['tracking_tolerance_m_s']
    return -weight * min(settings['tracking_cap'], error)


def jerk_excess(realized_jerk, requested_jerk, weight, decisions_before, settings=SETTINGS):
    """v06 jerk excess, except the raw request is not charged during the insertion lag.

    `decisions_before` counts this car's earlier decisions (0 for its first).
    """
    if decisions_before < settings['insertion_lag_decisions']:
        requested_jerk = 0.
    return v06.jerk_excess(realized_jerk, requested_jerk, weight)


def graded_quality(history, approach_stop_seconds, settings=SETTINGS):
    """Graded merge-quality payment from the gate's quality window (0 when the car's own conditions fail).

    `history` holds the gate's samples (stopped, jerk, override,
    neighbor_acceleration, speed_match) for the window ending at the stable merge.
    Returns (payment, details).
    """
    if not history:
        return 0., dict(eligible=False, reason='empty_window')
    n = len(history)
    stop_s = .1 * sum(bool(s['stopped']) for s in history)
    own_ok = (stop_s <= settings['quality_max_stop_seconds'] + 1e-8
              and approach_stop_seconds <= settings['quality_max_stop_seconds'] + 1e-8
              and all(s['jerk'] is not None and abs(s['jerk']) <= settings['quality_max_jerk_m_s3'] for s in history)
              and sum(bool(s['override']) for s in history) / n <= settings['quality_max_override_fraction'])
    match = sum(bool(s['speed_match']) for s in history) / n
    calm = sum(s['neighbor_acceleration'] is None or s['neighbor_acceleration'] >= settings['quality_neighbor_braking_m_s2']
               for s in history) / n
    payment = settings['graded_quality_weight'] * match * calm if own_ok else 0.
    return payment, dict(eligible=own_ok, samples=n, speed_match_fraction=match, neighbor_calm_fraction=calm,
                         stop_seconds=stop_s, approach_stop_seconds=approach_stop_seconds)
