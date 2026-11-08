"""Versioned current-state/map flow contract; no future trajectory or teacher action.

Keep the 51 v05 local fields, insert 18 measured/map fields, then retain the six
eight-field delivered messages. Legacy projection is explicit, never inferred.
"""
from __future__ import annotations

import math
import numpy as np

from environments.highway_environment_v05 import ContinuousMotionLaneGraph
from traffic.highway_merge_context import Stage1LaneGraph, merge_target_context, _signed_progress, CAP
from traffic.highway_stage1_teacher import acc_response_ceiling

SCHEMA = 'highway_flow_actor117_v1'
LOCAL_SIZE = 69
OBSERVATION_SIZE = 117
STATE_SIZE = 24 * (OBSERVATION_SIZE + 1)
EXTRA_FEATURES = (
    'own_route_reachable', 'own_route_end_distance_over1000',
    'right_route_reachable', 'right_route_end_distance_over1000',
    'left_route_reachable', 'left_route_end_distance_over1000',
    'entry_visibility_over200', 'own_ACC_over4p5', 'right_ACC_over4p5',
    'left_ACC_over4p5', 'occupied_worst_ACC_over4p5',
    'right_front_lateral_offset_overwidth', 'right_front_lateral_speed_over3',
    'left_front_lateral_offset_overwidth', 'left_front_lateral_speed_over3',
    'own_legal_speed_overcap', 'right_legal_speed_overcap', 'left_legal_speed_overcap',
)
FIELD_INDEX = {name: 51 + i for i, name in enumerate(EXTRA_FEATURES)}
LEGACY_TO_FLOW = tuple(range(51)) + tuple(range(69, 117))


def legacy_observation(observation):
    vector = np.asarray(observation, np.float32)
    if vector.shape != (117,) or not np.isfinite(vector).all():
        raise ValueError('Expected finite flow117 observation')
    return vector[list(LEGACY_TO_FLOW)].copy()


def inject_observation(legacy, extras):
    legacy, extras = np.asarray(legacy, np.float32), np.asarray(extras, np.float32)
    if legacy.shape != (99,) or extras.shape != (18,):
        raise ValueError('Expected legacy99 and 18 flow fields')
    result = np.concatenate((legacy[:51], extras, legacy[51:]))
    if not np.isfinite(result).all() or np.any(np.abs(result) > 1.000001):
        raise ValueError('Flow fields violate normalized finite contract')
    return result


def route_continuation(graph, ego, lane_id):
    """Current route/map only; distance to mandatory lateral change, not seam."""
    if lane_id is None:
        return dict(known=False, reachable=False, distance_m=0.)
    position = ego['lane_position'] / graph.lanes[ego['lane_id']]['length'] * graph.lanes[lane_id]['length']
    distances = graph.distances(lane_id, position, ego['route'])
    reachable = any(graph.lanes[lane]['edge'] == ego['route'][-1] for lane in distances)
    ends = [distance + graph.lanes[lane]['length'] for lane, distance in distances.items()
            if not (graph.successors[lane] & set(distances))]
    return dict(known=True, reachable=reachable,
                distance_m=max(0., min(ends)) if ends else 1000.)


def merge_context(graph, ego, states):
    """Actual declared minor entry; fallback is the original lateral projection.

    The historical v3 function rejects nondefault visibility. This separate
    implementation accepts explicit finite positive map visibility and records
    it. It does not grant right of way or disable the genuine conflict.
    """
    reachable = graph.distances(ego['lane_id'], ego['lane_position'], ego['route'])
    entries = [(d + graph.lanes[lane]['length'], lane, graph.yield_entries[lane])
               for lane, d in reachable.items() if lane in graph.yield_entries]
    if not entries:
        result = merge_target_context(graph, ego, states)
        if graph.lanes[ego['lane_id']]['internal'] and result['lane_delta'] == 0:
            result = dict(result, known=False, front=None, rear=None)
        return dict(result, minor_link_visibility_m=0., entry_active=False)
    distance, source, entry = min(entries, key=lambda value: value[0])
    visibility = entry['visibility_m']
    if not math.isfinite(visibility) or not .1 < visibility <= 200.:
        raise ValueError('Flow contract supports declared visibility in (0.1,200] m')
    if entry['stop_offset_m'] > .1 + 1e-8:
        raise ValueError('Flow contract requires explicitly modeled default stop offset')
    anchor = entry['target']
    if anchor not in reachable:
        raise ValueError('Minor link target absent from connected route')
    ego_progress = -reachable[anchor]
    front, rear = [], []
    for vid, other in sorted(states.items()):
        if vid == ego['id']:
            continue
        progress = _signed_progress(graph, anchor, other, ego['route'], max(400., abs(ego_progress) + 130.))
        if progress is None:
            continue
        delta = progress - ego_progress
        gap = max(0., delta - other['length']) if delta >= 0. else max(0., -delta - ego['length'])
        if gap <= 120.:
            (front if delta >= 0. else rear).append((gap, vid, other['speed'] - ego['speed']))
    def nearest(values):
        if not values:
            return None
        gap, vid, speed = min(values)
        return dict(id=vid, bumper_gap_m=gap, relative_speed_m_s=speed)
    return dict(known=True, target_lane=anchor, target_index=graph.lanes[anchor]['index'],
                lane_delta=0, distance_to_start_m=max(0., distance), front=nearest(front), rear=nearest(rear),
                target_speed_limit=graph.speed_limits[anchor], projection='shared_entry_stopline',
                source_lane=source, minor_link_visibility_m=visibility, entry_active=True)


def flow_context(graph, ego, states):
    """Keep each physical corridor's limiting leader before taking a minimum.

    The nearest car inside a corridor remains its car-following leader. Unlike
    v05's nearest across occupied corridors, a farther slow destination leader
    is retained in its own response channel. All fields use current state/map.
    """
    expanded, aliases = graph.occupancy_states(states, ego['id'])
    corridors = {}
    for name, delta in (('own', 0), ('right', -1), ('left', 1)):
        lane, known = (ego['lane_id'], True) if not delta else graph.adjacent(ego['lane_id'], delta)
        legal = graph.speed_limits[lane] if lane is not None else 0.
        front = None
        if lane is not None:
            position = ego['lane_position'] / graph.lanes[ego['lane_id']]['length'] * graph.lanes[lane]['length']
            # Deliberately bypass ContinuousMotionLaneGraph.neighbor's reduction
            # across ego corridors, but retain others' measured double occupancy.
            front = Stage1LaneGraph.neighbor(graph, ego, expanded, lane, position, forward=True)
            if front is not None:
                front = dict(front, id=aliases.get(front['id'], front['id']))
        neighbor = None if front is None else (front['bumper_gap_m'], max(0., ego['speed'] + front['relative_speed_m_s']))
        response = min(2., .4 * (legal - ego['speed']), acc_response_ceiling(ego['speed'], legal, neighbor)) if lane is not None else 2.
        tracked = states[front['id']] if front is not None else {}
        corridors[name] = dict(lane_id=lane, known=known, legal_speed_m_s=legal,
            route=route_continuation(graph, ego, lane), front=front,
            native_response_estimate_m_s2=float(np.clip(response, -4.5, 2.)),
            front_lateral_offset_overwidth=float(np.clip(tracked.get('lateral_position', 0.) / tracked.get('lane_width', 3.2), -1., 1.)),
            front_lateral_speed_over3=float(np.clip(tracked.get('lateral_speed', 0.) / 3., -1., 1.)))
    occupied = graph.occupied_lanes(ego)
    relevant = [row for row in corridors.values() if row['lane_id'] in occupied]
    limiting = min(relevant, key=lambda row: row['native_response_estimate_m_s2'])
    entry = merge_context(graph, ego, expanded)
    extras = []
    for name in ('own', 'right', 'left'):
        route = corridors[name]['route']
        extras += [float(route['reachable']), min(1., route['distance_m'] / 1000.) if route['known'] else 0.]
    extras += [entry['minor_link_visibility_m'] / 200.]
    extras += [corridors[name]['native_response_estimate_m_s2'] / 4.5 for name in ('own', 'right', 'left')]
    extras += [limiting['native_response_estimate_m_s2'] / 4.5]
    for name in ('right', 'left'):
        extras += [corridors[name]['front_lateral_offset_overwidth'], corridors[name]['front_lateral_speed_over3']]
    extras += [min(CAP, corridors[name]['legal_speed_m_s']) / CAP for name in ('own', 'right', 'left')]
    return dict(extras=extras, corridors=corridors, occupied_lanes=occupied,
                limiting_occupied_lane=limiting['lane_id'],
                occupied_worst_acceleration_m_s2=limiting['native_response_estimate_m_s2'], entry=entry,
                source='current measured SUMO state and route/map; not future motion or teacher recommendation')
