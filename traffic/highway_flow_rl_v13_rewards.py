"""V13 merge target: at a yield-junction entry, the target follows SUMO's own yield decision.

User direction, November 18-19, 2026: train on the existing 3D model's roads as they are
("just use that road for the training"), then "fix" the target so it follows SUMO's
yield decision.

Problem this fixes (round 19, measured):
- on the model-exact LA corridors, the ramp joins the right lane at a yield junction
  (connection state 'm'); there is no acceleration lane;
- the gate's merge-speed target (traffic/highway_flow_rl_v04_rewards.merge_speed_target)
  ignores whether SUMO will admit the car. In the focused NB validation case it said
  16-20 m/s while SUMO's link was closed. SUMO braked the car at -4.5 m/s2 at 14 m
  from the line and held it stopped for 13 s, and the target stayed at 13.8 m/s.
  Every held decision cost the full band penalty, and the gate counted no speed match.

What SUMO reports (TraCI vehicle.getNextLinks, SUMO 1.27.1; tuple order read from
traci/_lane.py: lane, hasPrio, isOpen, hasFoe, via, state, direction, length). In a
deterministic replay of that case, the link to sb2_0 read closed from t=0.6 s, 1.2 s
before SUMO's brake. It stayed closed while the car was held, and read open at
t=17.1 s, 0.3 s before the car moved. In one window (t=3.2-3.7 s) it read open while
SUMO was still braking the car to a stop. The environment counts such open reads per
car (open_while_braking) and does not hide them.

Rule: while the car's pending merge is a yield entry (merge_context entry_active) and
SUMO reports the link to the entry's target lane closed, the target is capped by a
stop at the yield line:
    sqrt(2 * stop_decel * max(0, distance_to_line - stop_margin))
with 2 m/s2 and 2.5 m, the same form and constants as v03's 'unseen_yield_stop'
bound (traffic/highway_flow_rl_v03_rewards.safe_speed_target). The cap only lowers
the target. When SUMO reports the link open, when the link is not listed, or when the
merge is not a yield entry, the target is exactly v12's. Acceleration-lane merges
are therefore unchanged.

The same capped target feeds every consumer: the four actor target inputs, the
target_band and merge_tracking rewards, and the gate's speed match. Dynamics, the
safety shield, native guards, masks, gates' thresholds and every reward weight are
unchanged.
"""
from copy import deepcopy
import math

REWARD_VERSION = 'highway_flow_rl_merge_v13_yield_target'
SETTINGS = dict(stop_decel_m_s2=2.0, stop_margin_m=2.5)
BOUND_REASON = 'closed_yield_stop'


def validate_settings(settings):
    settings = dict(settings)
    if settings != SETTINGS:
        raise ValueError('Unreviewed v13 yield-target settings')
    return settings


def yield_link(next_links, target_lane):
    """SUMO's link to the entry's target lane from a getNextLinks result.

    Returns found=False when the link is not listed (the caller leaves the target as v12).
    """
    for link in next_links or ():
        if len(link) < 6:
            raise ValueError('Unexpected getNextLinks tuple')
        lane, priority, is_open, foe, via, state = link[:6]
        if lane == target_lane:
            return dict(found=True, open=bool(is_open), foe=bool(foe), priority=bool(priority),
                        via=via, state=state)
    return dict(found=False, open=None, foe=None, priority=None, via=None, state=None)


def stop_bound(distance_m, settings=SETTINGS):
    """Largest speed that stops before the yield line at the declared deceleration."""
    if distance_m is None or not math.isfinite(distance_m):
        raise ValueError('Invalid distance to the yield line')
    return math.sqrt(2. * settings['stop_decel_m_s2'] * max(0., distance_m - settings['stop_margin_m']))


def apply_yield(target, entry, link, settings=SETTINGS):
    """v12 target, capped by a stop at the line while SUMO reports the yield link closed.

    Returns (target, applied). The input target is never modified.
    """
    if (not target or not target.get('eligible') or not entry or not entry.get('entry_active')
            or not link.get('found') or link.get('open')):
        return target, False
    value = target.get('target_m_s')
    if value is None or not math.isfinite(value):
        return target, False
    bound = stop_bound(entry['distance_to_start_m'], settings)
    result = deepcopy(target)
    result['target_m_s'] = min(value, bound)
    result.setdefault('bounds', []).append(dict(reason=BOUND_REASON, ceiling_m_s=bound))
    result['yield_link'] = dict(link, distance_to_line_m=entry['distance_to_start_m'])
    result['v12_target_m_s'] = value
    return result, True
