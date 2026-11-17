"""Merge recovery objective and explicit, seeded cut-in initial conditions."""
import math
import xml.etree.ElementTree as ET
import numpy as np

REWARD_VERSION = 'merge_recovery_request_slew_v08'
SETTINGS = dict(merge_acceleration_weight=.35, merge_speed_weight=.40,
    positive_limit_request_weight=.50, tracking_scale_m_s=8., response_seconds=2.,
    ordinary_acceleration_m_s2=2., ordinary_deceleration_m_s2=2., initial_std=.12)


def recovery_terms(speed, next_speed, target, requested, ceiling, speed_ceiling, merge_active):
    """Pre-action local target; no future traffic, imitation actor or action change.

    Unsaturated tracking over ordinary merge errors and request alignment add a
    dense learning signal even when a short ramp cannot reach its final limit.
    Safety ceilings always take priority over the nominal acceleration target.
    """
    terms = dict(merge_acceleration_alignment=0., merge_speed_tracking_v08=0., positive_limit_request=0.)
    if speed_ceiling is not None:
        excess = max(0., requested - speed_ceiling)
        terms['positive_limit_request'] = -.5 * min(1., excess / .2)
    desired = None
    if merge_active and target.get('eligible') and next_speed is not None:
        desired = min(ceiling, max(-2., min(2., (target['target_m_s'] - speed) / 2.)))
        terms['merge_acceleration_alignment'] = -.35 * min(4., (requested - desired) ** 2)
        terms['merge_speed_tracking_v08'] = -.4 * min(2., abs(next_speed - target['target_m_s']) / 8.)
    return terms, desired


def write_cutin_demand(path, seed, graph, speed_limit):
    """An eight-car physical scene; native motion starts the cut-in before handoff.

    The adjacent cutter is present at reset, never teleported into the road.
    Same-lane spawns retain static validation. The deliberate cross-lane gap is
    close but has a faster cutter and full native collision avoidance.
    """
    from traffic.highway_v05_demand import (write_demand, _through_paths, _main_position,
        validate_initial_spawns, LENGTH)
    manifest = write_demand(path, seed, graph, 'cruise', 1, 8,
        speed_limit_m_s=speed_limit, controlled_car_follow_model='Krauss')
    paths = _through_paths(graph)
    if len(paths) < 2:
        raise ValueError('Cut-in exercise needs two through lanes')
    rng = np.random.default_rng(seed)
    limit = min(speed_limit, graph.speed_limits[paths[0][0]], graph.speed_limits[paths[1][0]])
    speed = float(rng.uniform(.55, .65) * limit)
    headway = float(rng.uniform(.95, 1.15))
    ego = next(r for r in manifest if r['controlled'])
    cutter = next(r for r in manifest if not r['controlled'])
    others = [r for r in manifest if r is not ego and r is not cutter]
    def place(row, lane, distance, velocity):
        edge, index, pos, _, lid = _main_position(graph, paths[lane], distance)
        row.update(route='main', edge=edge, lane=index, position=float(pos), speed=velocity, lane_id=lid)
    place(ego, 0, 60., speed)
    place(cutter, 1, 60. + LENGTH + speed * headway, speed + 3.)
    for i, row in enumerate(others):
        place(row, i % len(paths), 220. + 85. * (i // len(paths)), .75 * limit)
    validate_initial_spawns(graph, manifest, ceiling=speed_limit)
    tree = ET.parse(path)
    root = tree.getroot()
    nodes = {v.attrib['id']: v for v in root.findall('vehicle')}
    for node in nodes.values():
        root.remove(node)
    for row in sorted(manifest, key=lambda r: (-r['edge'], -r['position'], r['vehicle_id'])):
        node = nodes[row['vehicle_id']]
        node.attrib.update(route='main', departEdge=str(row['edge']), departLane=str(row['lane']),
                           departPos=str(row['position']), departSpeed=str(row['speed']))
        root.append(node)
    tree.write(path, encoding='utf-8', xml_declaration=True)
    spec = dict(ego=ego['vehicle_id'], cutter=cutter['vehicle_id'], target_lane=ego['lane_id'],
        ego_speed=speed, cutter_speed=speed + 3., initial_headway_s=headway,
        initial_gap_m=speed * headway, seed=seed, rule='Native moving cut-in required before policy handoff')
    manifest.scenario.update(cutin_fixture=spec, physical_matching='Versioned paired cut-in layout',
                            initial_speed_range_m_s=[speed, max(r['speed'] for r in manifest)])
    return manifest, spec
