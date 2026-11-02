"""Current-state conditional path screening; no intent or controller decisions.

Projection onto a common edge is NOT a physical gap between different paths.
The response envelope constructs one jerk-bounded stop under two stated lead
motion assumptions. It cannot establish a counterfactual SUMO outcome.
"""
from __future__ import annotations

from dataclasses import dataclass
import heapq
import math

from traffic.i110_policy_interface import LaneGraph


class ConvergenceGraph:
    def __init__(self, path, path_limit_m=300.0):
        self.graph = LaneGraph(path)
        self.limit = path_limit_m
        # Static distances from each lane start, without vehicle route knowledge.
        self._paths = {}
        for start in self.graph.lanes:
            distances = {start: 0.0}
            queue = [(0.0, start)]
            while queue:
                cost, lane = heapq.heappop(queue)
                if cost != distances[lane]:
                    continue
                end = cost + self.graph.lanes[lane]['length']
                for target in sorted(self.graph.successors[lane]):
                    if end < distances.get(target, math.inf):
                        distances[target] = end
                        heapq.heappush(queue, (end, target))
            self._paths[start] = distances

    def paths(self, state):
        lid, position = state.get('lane_id'), state.get('lane_position')
        if (lid not in self.graph.lanes or not isinstance(position, (int, float))
                or not math.isfinite(position) or position < 0
                or position > self.graph.lanes[lid]['length'] + 1e-6):
            return None
        return {lane: distance-position for lane, distance in self._paths[lid].items()
                if distance-position <= self.limit and not self.graph.lanes[lane]['internal']}

    def pair(self, ego, other):
        a, b = self.paths(ego), self.paths(other)
        if a is None or b is None:
            return dict(status='unknown_lane_context')
        options = []
        for la, da in a.items():
            lane_a = self.graph.lanes[la]
            for lb, db in b.items():
                lane_b = self.graph.lanes[lb]
                if lane_a['edge'] != lane_b['edge']:
                    continue
                separation = abs(lane_a['index']-lane_b['index'])
                if separation <= 1:
                    options.append((max(0., da)+max(0., db), lane_a['edge'],
                                    separation, la, lb, da, db))
        if not options:
            return dict(status='no_same_or_adjacent_path_within_limit')
        _, edge, separation, la, lb, da, db = min(options)
        delta = da-db
        return dict(status='conditional_projection', anchor_edge=edge,
            ego_anchor_lane=la, other_anchor_lane=lb,
            relation='same_lane_path' if separation == 0 else 'adjacent_lane_path',
            ego_distance_to_anchor_m=da, other_distance_to_anchor_m=db,
            front_position_delta_m=delta, projected_gap_m=delta-other['length'],
            forward_candidate=delta > 0,
            intent_known=False, actual_gap_known=ego['lane_id'] == other['lane_id'])


@dataclass(frozen=True)
class Segment:
    start: float
    duration: float
    x: float
    v: float
    a: float
    j: float

    def at(self, t):
        q = t-self.start
        return (self.x+self.v*q+self.a*q*q/2+self.j*q*q*q/6,
                self.v+self.a*q+self.j*q*q/2, self.a+self.j*q)


def stopping_profile(speed, acceleration, *, jerk=2.0, deceleration=2.5, reaction=0.1):
    """Jerk down / bounded braking / jerk up to rest with zero acceleration.

Return None when the initial state cannot support this monotone stop, rather
than clamp acceleration or introduce a jerk discontinuity at zero speed.
"""
    if (not all(math.isfinite(x) for x in (speed, acceleration, jerk, deceleration, reaction))
            or speed < 0 or jerk <= 0 or deceleration <= 0 or reaction < 0
            or acceleration < -deceleration-1e-8):
        return None
    # Roundoff tolerance is only for a boundary value, not a shield override.
    acceleration = max(acceleration, -deceleration)
    v1 = speed+acceleration*reaction
    if v1 < 0 or (acceleration < 0 and v1 < acceleration**2/(2*jerk)-1e-9):
        return None
    peak = min(deceleration, math.sqrt(jerk*v1+acceleration**2/2))
    down = (acceleration+peak)/jerk
    if down < -1e-8:
        return None
    hold = ((v1+(acceleration**2-peak**2)/(2*jerk)-peak**2/(2*jerk))/peak
            if peak > 1e-12 else 0.)
    t, x, v, a = 0., 0., speed, acceleration
    segments = []
    for duration, j in [(reaction, 0.), (max(0., down), -jerk),
                        (max(0., hold), 0.), (peak/jerk, jerk)]:
        if duration <= 1e-12:
            continue
        segment = Segment(t, duration, x, v, a, j)
        segments.append(segment)
        t += duration
        x, v, a = segment.at(t)
    if not segments:
        segments = [Segment(0., 0., 0., speed, acceleration, 0.)]
    if abs(v) > 1e-7 or abs(a) > 1e-7:
        raise ArithmeticError('Stopping profile did not end at rest with zero acceleration')
    return segments


def roots(a, b, c):
    if abs(a) < 1e-12:
        return [] if abs(b) < 1e-12 else [-c/b]
    discriminant = b*b-4*a*c
    if discriminant < -1e-10:
        return []
    d = math.sqrt(max(0., discriminant))
    return [(-b-d)/(2*a), (-b+d)/(2*a)]


def other_motion(speed, t, model):
    if model == 'constant_speed':
        return speed*t, speed, 0.
    if model != 'brake_9':
        raise ValueError('Unknown lead motion assumption')
    if t < speed/9:
        return speed*t-4.5*t*t, speed-9*t, -9.
    return speed*speed/18, 0., 0.


def response_envelope(profile, other_speed, *, model, headway, standstill=2.5):
    """Exact extrema of x_ego-x_other + headway*v_ego, including reaction."""
    if profile is None or not math.isfinite(other_speed) or other_speed < 0:
        return dict(known=False, reason='initial_state_outside_bounded_stopping_profile')
    if headway < 0 or standstill < 0:
        raise ValueError('Negative gap parameter')
    maximum, at = -math.inf, 0.
    for segment in profile:
        end = segment.start+segment.duration
        cuts = [segment.start, end]
        if model == 'brake_9' and segment.start < other_speed/9 < end:
            cuts.insert(1, other_speed/9)
        for lo, hi in zip(cuts, cuts[1:]):
            _, ve, ae = segment.at(lo)
            _, vo, ao = other_motion(other_speed, lo, model)
            critical = [lo, hi]
            critical += [lo+q for q in roots(segment.j/2, ae-ao+headway*segment.j,
                                             ve-vo+headway*ae) if 0 < q < hi-lo]
            for t in critical:
                xe, ve, _ = segment.at(t)
                xo, _, _ = other_motion(other_speed, t, model)
                closure = xe-xo+headway*ve
                if closure > maximum:
                    maximum, at = closure, t
    return dict(known=True, required_projected_gap_m=standstill+max(0., maximum),
        tightest_time_s=at, stopping_time_s=profile[-1].start+profile[-1].duration,
        stopping_distance_m=profile[-1].at(profile[-1].start+profile[-1].duration)[0])


def screen(vid, states, graph, plan):
    """Accept only a contemporaneous state dictionary and static map/config."""
    ego = states[vid]
    profile = stopping_profile(ego['speed'], ego['acceleration'], jerk=plan['jerk_m_s3'],
        deceleration=plan['comfortable_deceleration_m_s2'], reaction=plan['reaction_s'])
    candidates = []
    for oid, other in sorted(states.items()):
        if oid == vid:
            continue
        distance = math.hypot(other['x']-ego['x'], other['y']-ego['y'])
        if distance > max(plan['ranges_m']):
            continue
        pair = graph.pair(ego, other)
        if pair['status'] == 'no_same_or_adjacent_path_within_limit':
            continue
        envelopes = {}
        if pair.get('forward_candidate'):
            for model in plan['other_models']:
                for h in plan['headways_s']:
                    envelope = response_envelope(profile, other['speed'], model=model,
                                                  headway=h, standstill=plan['standstill_gap_m'])
                    if envelope['known']:
                        margin = pair['projected_gap_m']-envelope['required_projected_gap_m']
                        envelope.update(margin_m=margin, deficit=margin < 0)
                    envelopes[f'{model}-h{h:g}'] = envelope
        candidates.append(dict(id=oid, distance_m=distance, projection=pair, envelopes=envelopes))
    return dict(ego_context_known=graph.paths(ego) is not None,
        stopping_profile_known=profile is not None, candidates=candidates)
