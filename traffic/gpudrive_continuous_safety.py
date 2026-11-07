"""Bounded continuous-control transfer guards, not a SUMO native safety clone.

Preserve the frozen actor's steering. Reject unsafe proposals instead of silently
replacing steering with a tactical lane request or a scripted driver.
"""
import math
import numpy as np

from environments.highway_stage1_env import acceleration_constraints
from traffic.gpudrive_sumo_bridge import bicycle_step, overlaps, front_from_center

CAP = 26.8224
DT = .1


def restore_destination_route(conn, ego, destination_edge):
    """Repair moveToXY's truncated route without changing pose or destination.

    keepRoute=2 lets SUMO associate an externally steered car with another road.
    SUMO may then retain only that road as its route; findRoute restores a legal
    edge sequence for this vehicle type. Internal links retain a valid existing
    route, but a missing route there is not guessed.
    """
    route = tuple(ego['route']); edge = ego['edge_id']
    if route and route[-1] == destination_edge and (edge in route or edge.startswith(':')):
        return ego, None
    if not edge or edge.startswith(':'):
        raise ValueError('Cannot restore a destination route from an unmatched or internal edge')
    replacement = tuple(conn.simulation.findRoute(edge, destination_edge,
        vType=conn.vehicle.getTypeID(ego['id'])).edges)
    if not replacement or replacement[0] != edge or replacement[-1] != destination_edge:
        raise ValueError('No connected route from the current road to the fixed destination')
    conn.vehicle.setRoute(ego['id'], replacement)
    if tuple(conn.vehicle.getRoute(ego['id'])) != replacement:
        raise ValueError('SUMO destination route readback mismatch')
    return dict(ego, route=replacement), dict(source_edge=edge,
        previous_route=list(route), restored_route=list(replacement), destination_edge=destination_edge)


def recover_lane_state(ego, road, graph):
    """Project an unmatched external pose onto the route's map, without moving it.

    moveToXY may report no lane for a turning car even when its entire footprint
    remains on the road. Use a front-bumper coordinate, matching TraCI semantics.
    Fail closed if no route lane ribbon contains that point.
    """
    if ego['lane_id'] in graph.lanes and ego['lane_position'] >= 0:
        return ego
    point = np.asarray(front_from_center(ego)[:2])
    candidates = []
    allowed = graph.allowed(ego['route'])
    for identity in sorted(allowed):
        lane = road.lanes[identity]
        points = lane['points']; delta = np.diff(points, axis=0)
        length = np.linalg.norm(delta, axis=1)
        valid = length > 1e-8
        a = points[:-1][valid]; delta = delta[valid]; length = length[valid]
        if not len(length):
            continue
        fraction = np.clip(np.sum((point-a)*delta, axis=1)/length**2, 0., 1.)
        distances = np.linalg.norm(point-a-fraction[:,None]*delta, axis=1)
        index = int(np.argmin(distances))
        if distances[index] <= lane['width']/2 + .05:
            position = (sum(length[:index])+fraction[index]*length[index])/sum(length)*lane['length']
            candidates.append((float(distances[index]), identity, float(position)))
    if not candidates:
        raise ValueError('No route lane contains the external front-bumper pose')
    _, identity, position = min(candidates)
    return dict(ego, lane_id=identity, lane_position=position,
                allowed_speed=min(CAP, road.lanes[identity]['speed']),
                lane_association='geometric_front_bumper_recovery')


def guarded_step(ego, acceleration, steering, road, others=(), *,
                 allowed_speed=CAP, front=None, lane_end_distance=None,
                 blocked_entry_distance=None):
    state = dict(ego, allowed_speed=min(CAP, allowed_speed))
    bounds, ceiling = acceleration_constraints(state, front, lane_end_distance)
    if blocked_entry_distance is not None:
        safe = math.sqrt(9. * max(0., blocked_entry_distance - 2.5))
        bounds['junction_yield'] = (safe - ego['speed']) / DT
        ceiling = min(ceiling, max(-4.5, bounds['junction_yield']))
    applied = max(-4.5, min(2., float(acceleration), ceiling))
    reasons = [name for name, bound in bounds.items()
               if bound is not None and acceleration > bound + 1e-8]
    if not -4.5 <= acceleration <= 2.:
        reasons.append('acceleration_range')
    # Hard projection includes legal limits below 60 mph and prevents the
    # upstream absolute-speed convention from bouncing backwards through zero.
    final_speed = max(0., min(state['allowed_speed'], ego['speed'] + applied * DT))
    projected = (final_speed - ego['speed']) / DT
    if abs(projected - applied) > 1e-8:
        reasons.append('hard_speed_projection')
    applied = projected
    proposed = bicycle_step(ego, applied, steering)
    rejected = []
    for tick in range(1, 6):
        elapsed = DT * tick / 5
        sample = bicycle_step(ego, applied, steering, elapsed)
        if road.offroad(sample):
            rejected.append('predicted_offroad')
        for other in others:
            predicted = dict(other,
                x=other['x'] + other['speed'] * math.cos(other['yaw']) * elapsed,
                y=other['y'] + other['speed'] * math.sin(other['yaw']) * elapsed)
            if overlaps(sample, predicted):
                rejected.append('predicted_contact:' + other['id'])
    return proposed, dict(accepted=not rejected, reasons=sorted(set(reasons)),
        rejection_reasons=sorted(set(rejected)), acceleration_bounds=bounds,
        requested_acceleration=float(acceleration), applied_acceleration=applied,
        requested_steering_rad=float(steering), applied_steering_rad=float(steering),
        acceleration_changed=abs(applied - acceleration) > 1e-8)


def current_constraints(conn, graph, ego, cars):
    """Read route-connected neighbors and live minor-junction link state."""
    front = graph.neighbor(ego, {c['id']: c for c in cars}, ego['lane_id'],
                           ego['lane_position'], forward=True)
    distances = graph.distances(ego['lane_id'], ego['lane_position'], ego['route'])
    end = None
    if not any(graph.lanes[l]['edge'] == ego['route'][-1] for l in distances):
        ends = [d + graph.lanes[l]['length'] for l, d in distances.items()
                if not (graph.successors[l] & set(distances))]
        end = max(0., min(ends)) if ends else 0.
    blocked = []
    for source, entry in graph.yield_entries.items():
        if source not in distances:
            continue
        distance = distances[source] + graph.lanes[source]['length'] - entry['stop_offset_m']
        if distance < 0:
            continue
        links = [link for link in conn.lane.getLinks(source, extended=True)
                 if link[0] == entry['target'] and link[4] == entry['via']]
        if not links or any(not link[2] or link[3] for link in links):
            blocked.append(distance)
    return dict(allowed_speed=ego['allowed_speed'], front=front,
                lane_end_distance=end, blocked_entry_distance=min(blocked) if blocked else None)
