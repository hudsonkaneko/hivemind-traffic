"""Current-state merge projection on actual directed lane links.

The projection is longitudinal route distance to a common through-lane anchor,
not an assumption that two unrelated roads share their lane-position origin.
This is an exact SUMO/map development observation, not a sensor estimator.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

from environments.i110_merge_env import MergeLaneGraph


CAP = 26.8224
MERGE_FEATURES = (
    'merge_context_known', 'merge_target_lane_index_over_7',
    'merge_target_lane_delta', 'distance_to_target_lane_start_over_240',
    'target_front_gap_over_120', 'target_front_relative_speed_over_cap',
    'target_front_present', 'target_rear_gap_over_120',
    'target_rear_relative_speed_over_cap', 'target_rear_present',
    'feasible_acceleration_ceiling_over_4p5', 'target_allowed_speed_over_cap',
)


class Stage1LaneGraph(MergeLaneGraph):
    def __init__(self, path, through_lanes=4):
        super().__init__(path, through_lanes)
        self.speed_limits = {
            lane.attrib['id']: min(CAP, float(lane.get('speed', str(CAP))))
            for lane in ET.parse(path).findall('./edge/lane')
        }


def _signed_progress(graph, anchor, other, route, limit):
    """Other front-bumper coordinate relative to the anchor's start, if connected."""
    combined = tuple(other['route']) + tuple(route)
    forward = graph.distances(anchor, 0., combined, limit=limit)
    lid = other['lane_id']
    if lid in forward:
        return forward[lid] + other['lane_position']
    backward = graph.distances(anchor, 0., combined, forward=False, limit=limit)
    if lid in backward:
        return -(backward[lid] + graph.lanes[lid]['length'] - other['lane_position'])
    return None


def merge_target_context(graph, ego, states):
    """Identify the nearest reachable auxiliary-to-through lane opportunity.

    Before the auxiliary segment begins, target-lane neighbors are projected
    into its coordinate using current positions and real incoming links. The
    lane command remains zero until that target is actually adjacent. No future
    traffic position is queried or predicted. Returns unknown if the route is
    already reachable without a lane change, or no target can be established.
    """
    unknown = dict(known=False, target_lane=None, target_index=0, lane_delta=0,
                   distance_to_start_m=0., front=None, rear=None,
                   target_speed_limit=0., projection='connected_lane_route_distance')
    reachable = graph.distances(ego['lane_id'], ego['lane_position'], ego['route'])
    if any(graph.lanes[lid]['edge'] == ego['route'][-1] for lid in reachable):
        return unknown
    candidates = []
    for lid, distance in reachable.items():
        lane = graph.lanes[lid]
        if lane['internal']:
            continue
        for delta in (-1, 1):
            target, known = graph.adjacent(lid, delta)
            if not known or not target or graph.identity(target).get('kind') != 'through':
                continue
            continuation = graph.distances(target, 0., ego['route'])
            if not any(graph.lanes[x]['edge'] == ego['route'][-1] for x in continuation):
                continue
            candidates.append((max(0., distance), distance, lid, target, delta))
    if not candidates:
        return unknown
    _, distance, source, anchor, delta = min(candidates)
    # Current parallel lanes use fractional edge progress as in the original
    # observation contract. A future segment uses directed distance to its start.
    ego_progress = (-distance if source != ego['lane_id'] else
                    ego['lane_position'] / graph.lanes[source]['length'] * graph.lanes[anchor]['length'])
    front, rear = [], []
    for vid, other in sorted(states.items()):
        if vid == ego['id']:
            continue
        progress = _signed_progress(graph, anchor, other, ego['route'], max(400., abs(ego_progress) + 130.))
        if progress is None:
            continue
        difference = progress - ego_progress
        gap = max(0., difference - other['length']) if difference >= 0 else max(0., -difference - ego['length'])
        if gap > 120.:
            continue
        item = (gap, vid, other['speed'] - ego['speed'])
        (front if difference >= 0 else rear).append(item)
    def nearest(items):
        if not items:
            return None
        gap, vid, relative = min(items)
        return dict(id=vid, bumper_gap_m=gap, relative_speed_m_s=relative)
    return dict(known=True, target_lane=anchor,
                target_index=graph.lanes[anchor]['index'],
                lane_delta=delta if source == ego['lane_id'] else 0,
                distance_to_start_m=-ego_progress,
                front=nearest(front), rear=nearest(rear),
                target_speed_limit=graph.speed_limits.get(anchor, CAP),
                projection='connected_lane_route_distance')


def merge_features(context, acceleration_ceiling):
    clip = lambda v: max(-1., min(1., float(v)))
    vector = [float(context['known']), clip(context['target_index'] / 7.),
              float(context['lane_delta']), clip(context['distance_to_start_m'] / 240.)]
    for direction in ('front', 'rear'):
        item = context[direction]
        vector.extend([min(1., item['bumper_gap_m'] / 120.),
                       clip(item['relative_speed_m_s'] / CAP), 1.] if item else [0., 0., 0.])
    vector += [clip(acceleration_ceiling / 4.5), clip(context['target_speed_limit'] / CAP)]
    if len(vector) != 12 or not all(math.isfinite(v) for v in vector):
        raise ValueError('Invalid current-state merge context')
    return vector
