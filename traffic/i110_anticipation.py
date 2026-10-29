"""Current-state screening only: no actuator, event schedule or future frame API."""
import math


def headway_signal(speed, observation, *, headway_s, standstill_gap_m=2.5,
                   closing_deceleration_m_s2=2.5, adjacent=False):
    """Return a hypothetical insertion margin, not a safe or jerk-feasible gap.

    Adjacent front vehicles need not intend to merge. Zero clipped bumper gaps
    may describe side-by-side cars. Unknown adjacency is never labeled clear.
    """
    values = (speed, headway_s, standstill_gap_m, closing_deceleration_m_s2)
    if not all(math.isfinite(v) for v in values) or speed < 0 or min(values[1:]) <= 0:
        raise ValueError('Invalid margin parameters')
    if observation['schema'] != 'i110_graph_v1':
        raise ValueError('Expected the explicitly named graph observation')
    known = observation['adjacent_context_known'] if adjacent else True
    lanes = ('own', 'left', 'right') if adjacent else ('own',)
    candidates = []
    for lane in lanes:
        # One aggregate mask cannot identify which adjacent side is known.
        if lane != 'own' and not known:
            continue
        other = observation['neighbors'][lane]['front']
        if other is None:
            continue
        gap, relative = other['bumper_gap_m'], other['relative_speed_m_s']
        if not math.isfinite(gap) or gap < 0 or not math.isfinite(relative):
            raise ValueError('Invalid neighbor state')
        closing = max(0., -relative)
        required = standstill_gap_m + headway_s * speed + closing**2 / (2 * closing_deceleration_m_s2)
        margin = gap - required
        candidates.append(dict(lane=lane, vehicle=other['id'], bumper_gap_m=gap,
                               relative_speed_m_s=relative, required_gap_m=required,
                               margin_m=margin, warning=margin < 0))
    warning = any(c['warning'] for c in candidates)
    return dict(warning=warning, context_known=bool(known),
                clear=not warning if known else None, candidates=candidates)


def prior_warning(rows, vehicle, time_s, target, window_s):
    """Retrospective scoring helper; never called by headway_signal.

    Use strictly earlier samples, restricted to the actual later leader ID.
    Earliest warning and latest warning disclose intermittent evidence.
    """
    hits = [r['time_s'] for r in rows
            if r['vehicle'] == vehicle and 1e-8 < time_s-r['time_s'] <= window_s+1e-8
            and any(c['vehicle'] == target and c['warning'] for c in r['signal']['candidates'])]
    return dict(target=target, prior_warning_samples=len(hits),
                earliest_lead_s=round(time_s-min(hits), 9) if hits else None,
                latest_lead_s=round(time_s-max(hits), 9) if hits else None)
