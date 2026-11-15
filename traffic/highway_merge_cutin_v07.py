"""Additional lane-change admission checks and decision-balanced curriculum.

Constant-speed prediction is checked again each tick; it is not a guarantee
against subsequent acceleration. SUMO collision avoidance remains enabled.
"""
from copy import deepcopy
import math

VERSION = 'projected_headway_ttc_reservations_v07'
SETTINGS = dict(headway_s=1.5, ttc_s=3., min_gap_m=2.5, horizon_s=3.1,
                stress_headway_s=.5, stress_max_events=1)
MIX = dict(normal_merge=.30, missed_gap=.20, cutin_recovery=.10,
           cruise_following=.25, passing=.15)


def gap_check(speed, neighbors, *, horizon=3.1, headway=1.5):
    result = dict(permitted=True, checks={})
    if not math.isfinite(speed) or speed < 0:
        return dict(result, permitted=False, reason='invalid_speed')
    for side in ('front', 'rear'):
        other = neighbors.get(side)
        if other is None:
            continue
        gap, relative = other['bumper_gap_m'], other['relative_speed_m_s']
        if not all(math.isfinite(x) for x in (gap, relative)) or speed + relative < 0:
            return dict(result, permitted=False, reason='invalid_neighbor')
        follower = speed if side == 'front' else speed + relative
        closing = -relative if side == 'front' else relative
        end = gap - closing * horizon
        minimum = min(gap, end)
        required = max(SETTINGS['min_gap_m'], headway * follower,
                       SETTINGS['ttc_s'] * max(0., closing))
        allowed = minimum >= required - 1e-8
        result['checks'][side] = dict(id=other.get('id'), gap_m=gap,
            projected_gap_m=end, required_gap_m=required, permitted=allowed)
        result['permitted'] &= allowed
    return result


def training_row(cfg, state, critic):
    """Choose the most underrepresented category using actual agent transitions.

Complete episodes remain intact. The deterministic per-row episode cursor and
coverage live in the existing checkpoint, including critic-only experience.
"""
    field = 'critic_decisions' if critic else 'ppo_decisions'
    coverage = state.get('coverage', {})
    rows = cfg['training_rows']
    spent = {group: sum(coverage.get(r['case']['id'], {}).get(field, 0)
                       for r in rows if r['curriculum_group'] == group) for group in MIX}
    group = min(MIX, key=lambda g: (spent[g] / MIX[g], -MIX[g]))
    choices = [r for r in rows if r['curriculum_group'] == group]
    row = deepcopy(min(choices, key=lambda r: (
        coverage.get(r['case']['id'], {}).get(field, 0), r['case']['id'])))
    cursor = coverage.get(row['case']['id'], {}).get('episodes', 0)
    row['seed'] += 1000 * cursor + 10000 * (cfg['round_index'] - 1) + (50000 if critic else 0)
    return row
