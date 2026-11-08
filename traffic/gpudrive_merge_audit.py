"""Audit recorded front-bumper lane mappings; topology does not prove legality."""
import math
import xml.etree.ElementTree as ET


def _project_front(vehicle, lane):
    points = [tuple(map(float, p.split(',')[:2])) for p in lane['shape'].split()]
    walked = 0.; candidates = []
    for a, b in zip(points, points[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]; length = math.hypot(dx, dy)
        if not length:
            continue
        t = max(0., min(1., ((vehicle['x']-a[0])*dx + (vehicle['y']-a[1])*dy)/length**2))
        q = (a[0]+t*dx, a[1]+t*dy)
        candidates.append((math.hypot(vehicle['x']-q[0], vehicle['y']-q[1]), walked+t*length, q))
        walked += length
    if not candidates:
        return None
    distance, position, point = min(candidates)
    position *= float(lane['length'])/walked
    return dict(projected_front_bumper_xy=[round(v, 4) for v in point],
                projected_position_m=round(position, 4),
                projected_distance_to_ramp_end_m=round(float(lane['length'])-position, 4),
                distance_to_ramp_centerline_m=round(distance, 4))


def audit_merge(recording, network_path):
    """Return topology evidence for agent_0; recording x/y must be front-bumper coordinates."""
    root = ET.parse(network_path).getroot()
    lanes = {}; indices = {}; internal = set()
    for edge in root.findall('edge'):
        for lane in edge.findall('lane'):
            identity = lane.get('id')
            lanes[identity] = dict(lane.attrib, edge=edge.get('id'))
            indices[edge.get('id'), lane.get('index')] = identity
            if edge.get('function') == 'internal' or edge.get('id', '').startswith(':'):
                internal.add(identity)
    links = {identity: set() for identity in lanes}
    for link in root.findall('connection'):
        source = indices.get((link.get('from'), link.get('fromLane')))
        target = link.get('via') or indices.get((link.get('to'), link.get('toLane')))
        if source and target:
            links[source].add(target)

    def connected(source, target):
        pending = list(links[source]); visited = set()
        while pending:
            candidate = pending.pop()
            if candidate == target:
                return True
            if candidate in internal and candidate not in visited:
                visited.add(candidate); pending.extend(links[candidate])
        return False

    transitions = []; first_exit = None; previous = None; missing = 0
    for frame in recording['frames']:
        vehicle = next((v for v in frame['vehicles'] if v['id'] == 'agent_0'), None)
        if vehicle is None:
            previous = None; missing += 1; continue
        current = vehicle.get('lane_id', '')
        if previous is not None and current != previous[0]:
            source, before = previous
            if source not in lanes or current not in lanes:
                status = 'unknown_lane_mapping'
            elif lanes[source]['edge'] == lanes[current]['edge']:
                status = 'same_edge_lane_change'
            else:
                status = 'graph_connected' if connected(source, current) else 'unconnected_cross_road'
            event = dict(time_s=frame['time'], previous_sample_time_s=before,
                         from_lane=source, to_lane=current, classification=status)
            transitions.append(event)
            if source == 'mlk_on_0' and first_exit is None:
                first_exit = dict(event, source_ramp_projection=_project_front(vehicle, lanes[source]))
        previous = current, frame['time']
    return dict(mapped_lane_transitions=transitions, first_ramp_exit=first_exit,
                unconnected_cross_road_transition_count=sum(
                    t['classification'] == 'unconnected_cross_road' for t in transitions),
                unknown_mapping_transition_count=sum(
                    t['classification'] == 'unknown_lane_mapping' for t in transitions),
                missing_ego_frames=missing, lane_marking_legality_verified=False,
                legitimate_merge_verified=False,
                limitations=['Lane mappings are sampled associations, not exact physical crossing times.',
                             'No verified painted-line, gore or barrier masks are available.',
                             'Same-edge lane changes and graph links do not establish physical or legal permission.',
                             'Three-second mainline occupancy alone does not establish a legitimate merge.'])
