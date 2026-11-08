"""Seeded v05 demand, separate from the historical population fixture.

Physical draws precede controller assignment. All initial vehicles are explicit
departures, with conservative connected-lane clearance checked before writing.
Later native departures are scheduled demand, not a claim of realized throughput.
"""
from __future__ import annotations

import math
from numbers import Integral, Real
import xml.etree.ElementTree as ET

import numpy as np


SCHEMA = 'highway_v05_varied_demand_v1'
CAP = 26.8224
MAX_AGENTS = 24
MAIN_ROUTE = ('sb1', 'sb2', 'sb3', 'sb4', 'sb5')
RAMP_ROUTE = ('mlk_on', 'sb2', 'sb3', 'sb4', 'sb5')
LENGTH = 4.8
MIN_GAP = 2.5
PROFILES = {
    'native_cautious': dict(tau=2.0, accel=1.6, decel=4.5, speedFactor=.94),
    'native_balanced': dict(tau=1.75, accel=2.0, decel=4.5, speedFactor=.97),
    'native_attentive': dict(tau=1.5, accel=2.2, decel=4.5, speedFactor=1.0),
}
DIFFICULTIES = {
    0: dict(name='easy', spacing_slack_m=(35., 55.), speed_spread=.015, arrival_interval_s=4.),
    1: dict(name='moderate', spacing_slack_m=(15., 30.), speed_spread=.06, arrival_interval_s=2.8),
    2: dict(name='dense_closing', spacing_slack_m=(4., 12.), speed_spread=.12, arrival_interval_s=2.),
}


class DemandManifest(list):
    """Ordinary JSON-serializable rows with separate resolved scenario metadata."""

    def __init__(self, rows, scenario):
        super().__init__(rows)
        self.scenario = scenario


def _integer(name, value, low, high=None):
    if isinstance(value, bool) or not isinstance(value, Integral) or value < low or (high is not None and value > high):
        raise ValueError(f'{name} must be an integer in [{low}, {high if high is not None else "infinity"}]')
    return int(value)


def _finite(name, value, minimum, maximum=math.inf):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError(f'{name} must be finite and in [{minimum}, {maximum}]')
    return float(value)


def _departure_seconds(value):
    """Canonical SUMO millisecond time, with its positive half-up rounding.

    SUMOTime.h TIME2STEPS/string2time rounds numeric XML times to integer ms.
    Keeping more precision in the manifest makes a native departure appear
    earlier than its request at a step boundary. Do not round the cumulative
    random arrival schedule itself: only serialize the existing native value.
    """
    return math.floor(value * 1000. + .5) / 1000.


def _lane_limit(graph, lane_id, ceiling):
    try:
        source = _finite('source lane speed', graph.speed_limits[lane_id], .01)
    except KeyError as error:
        raise ValueError(f'Missing speed limit for {lane_id}') from error
    return min(source, CAP, ceiling)


def _through_paths(graph):
    """Resolve lane shifts from graph identities, including imported I-110."""
    count = _integer('through_lanes', graph.through_lanes, 1, 7)
    paths = []
    for index in range(count):
        path = []
        for edge in MAIN_ROUTE:
            candidates = [lid for lid in graph.edges.get(edge, ())
                          if graph.identity(lid) == dict(kind='through', index_from_right=index)]
            if len(candidates) != 1:
                raise ValueError('v05 demand requires connected sb1..sb5 through lanes')
            lid = candidates[0]
            _finite('source lane length', graph.lanes[lid]['length'], 21.)
            if path and lid not in graph.distances(path[-1], 0., MAIN_ROUTE):
                raise ValueError(f'Disconnected through route at {lid}')
            path.append(lid)
        paths.append(path)
    return paths


def _main_position(graph, path, distance):
    """Map route position to an interior lane location, retaining any seam shift."""
    offset = 0.
    for edge_index, lid in enumerate(path):
        length = graph.lanes[lid]['length']
        position = max(10., distance - offset)
        if position <= length - 10.:
            return edge_index, graph.lanes[lid]['index'], position, offset + position, lid
        offset += length
    raise ValueError('Initial population exceeds available route length; no vehicles were dropped')


def _safe_gap(rear_speed, front_speed):
    # Use the most cautious native tau for every assignment, retaining the
    # existing 1.5 s AV guard. Additional closing-speed braking room is explicit.
    safety = 2. * rear_speed + max(0., rear_speed ** 2 - front_speed ** 2) / 9.
    # SUMO1.27.1 ACC insertionFollowSpeed iterates to a stationary speed. Its
    # gap-closing response must not be negative at the requested departure
    # speed: .8*(front-rear)+.04*(gap-1.5*rear-d0) >= 0. The free-speed branch
    # applies above120m net gap; the independent braking floor still applies.
    # Resolve this before assignment, so partial/full AV cohorts remain matched.
    dynamic_margin = min(2., max(0., 75. / max(rear_speed, 1e-6) - 5.))
    acc_insertion = min(120.001, 1.5 * rear_speed + dynamic_margin + 20. * max(0., rear_speed - front_speed))
    return MIN_GAP + max(safety, acc_insertion)


def validate_initial_spawns(graph, rows, *, ceiling=CAP):
    """Reject invalid positions/limits and connected-lane overlap or short gaps.

    This validates authored demand. The environment must additionally require
    every initial ID to be present after native SUMO insertion and stabilization.
    """
    initial = [row for row in rows if row['initial_cohort']]
    if len({row['slot'] for row in rows}) != len(rows):
        raise ValueError('Duplicate physical population slot')
    for row in rows:
        lid = row['lane_id']
        if lid not in graph.lanes:
            raise ValueError('Unknown spawn lane')
        length = graph.lanes[lid]['length']
        if not LENGTH + MIN_GAP <= row['position'] <= length - 10.:
            raise ValueError('Invalid spawn position')
        if not 0. <= row['speed'] <= .94 * _lane_limit(graph, lid, ceiling) + 1e-8:
            raise ValueError('Invalid spawn speed')
        if not math.isfinite(row['depart']) or row['depart'] < 0.:
            raise ValueError('Invalid departure time')
    for rear in initial:
        route = MAIN_ROUTE if rear['route'] == 'main' else RAMP_ROUTE
        distances = graph.distances(rear['lane_id'], rear['position'], route)
        for front in initial:
            if rear is front or front['lane_id'] not in distances:
                continue
            distance = distances[front['lane_id']] + front['position']
            if distance < 0.:
                continue
            gap = distance - LENGTH
            if gap + 1e-8 < _safe_gap(rear['speed'], front['speed']):
                raise ValueError(f'Unsafe initial spawn gap: {rear["slot"]} -> {front["slot"]}')


def write_demand(path, seed, graph, task, controlled_count, total_vehicles, *,
                 speed_limit_m_s=None, difficulty=0, traffic_mode='initial',
                 stabilization_seconds=8., horizon_seconds=80., assignment_seed=None):
    """Write deterministic initial/sustained traffic and return all vehicle rows.

    ``total_vehicles`` means the complete initial cohort (8, 16 or 24), including
    the native leader for following/passing. Sustained arrivals are native-only,
    explicitly outside that denominator; 100% AV therefore requires initial mode.
    No controller changes membership during an episode. Changing only
    ``controlled_count`` or ``assignment_seed`` preserves physical demand.
    Here ``stabilization_seconds`` means native reset warmup before measurement,
    not the environment's separate post-merge stabilization scoring window.
    """
    seed = _integer('seed', seed, 0)
    assignment_seed = seed if assignment_seed is None else _integer('assignment_seed', assignment_seed, 0)
    controlled_count = _integer('controlled_count', controlled_count, 0, MAX_AGENTS)
    total_vehicles = _integer('total_vehicles', total_vehicles, 8, MAX_AGENTS)
    if total_vehicles not in (8, 16, 24) or controlled_count > total_vehicles:
        raise ValueError('total_vehicles must be 8, 16 or 24 and include every controlled vehicle')
    difficulty = _integer('difficulty', difficulty, 0, 2)
    if task not in ('cruise', 'following', 'passing', 'merge'):
        raise ValueError('Unknown v05 task')
    if traffic_mode not in ('initial', 'sustained'):
        raise ValueError('traffic_mode must be initial or sustained')
    leader_count = int(task in ('following', 'passing'))
    if controlled_count > total_vehicles - leader_count:
        raise ValueError('Following/passing requires a native slow leader within the total population')
    if controlled_count == total_vehicles and traffic_mode == 'sustained':
        raise ValueError('100% AV requires initial traffic mode; sustained demand introduces native vehicles')
    ceiling = CAP if speed_limit_m_s is None else _finite('speed_limit_m_s', speed_limit_m_s, 6., CAP)
    stabilization_seconds = _finite('stabilization_seconds', stabilization_seconds, 0., 120.)
    horizon_seconds = _finite('horizon_seconds', horizon_seconds, .1, 3600.)
    if traffic_mode == 'sustained' and horizon_seconds <= 3.:
        raise ValueError('Sustained horizon must exceed the three-second post-stabilization buffer')
    settings = DIFFICULTIES[difficulty]
    rng = np.random.default_rng(np.random.SeedSequence([seed, 5001]))
    assignment_rng = np.random.default_rng(np.random.SeedSequence([assignment_seed, 5002]))
    flow_rng = np.random.default_rng(np.random.SeedSequence([seed, 5003]))
    paths = _through_paths(graph)
    lane_limits = [min(_lane_limit(graph, lid, ceiling) for lid in lane_path) for lane_path in paths]
    base_ratio = float(rng.uniform(.72, .90))
    profile_names = tuple(PROFILES)

    def physical(slot, route, edge, lane, position, speed, lid, *, depart=0., initial=True):
        return dict(slot=slot, route=route, edge=edge, lane=lane, lane_id=lid,
                    position=float(position), speed=float(speed), depart=_departure_seconds(float(depart)),
                    initial_cohort=initial, driver_profile=str(rng.choice(profile_names)))

    # Ramp capacity is resolved from this existing network, never by lengthening
    # a road silently or by dropping requested initial vehicles.
    ramps = []
    ramp_length = None
    if task == 'merge':
        lid = 'mlk_on_0'
        if lid not in graph.lanes or 'sb2' not in {graph.lanes[k]['edge'] for k in graph.distances(lid, 0., RAMP_ROUTE)}:
            raise ValueError('Merge demand requires a connected mlk_on ramp')
        ramp_length = _finite('ramp length', graph.lanes[lid]['length'], 21.)
        ramp_speed = float(_lane_limit(graph, lid, ceiling) * rng.uniform(.53, .75))
        front_position = ramp_length - max(25., ramp_speed ** 2 / 9. + 15.) - float(rng.uniform(0., 15.))
        spacing = LENGTH + _safe_gap(ramp_speed, ramp_speed) + float(rng.uniform(*settings['spacing_slack_m']))
        capacity = min(4, 1 + math.floor((front_position - 10.) / spacing))
        if capacity < 1:
            raise ValueError('Ramp too short for a feasible initial merge vehicle')
        ramp_count = int(rng.integers(1, capacity + 1))
        for i in range(ramp_count):
            ramps.append(physical(f'ramp_{i}', 'ramp', 0, 0, front_position - spacing * i, ramp_speed, lid))
    main_count = total_vehicles - len(ramps) - leader_count
    physical_rows = []
    lane_order = list(rng.permutation(len(paths)))
    lane_members = {i: [] for i in range(len(paths))}
    for i in range(main_count):
        lane_index = int(lane_order[i % len(paths)])
        ratio = float(np.clip(base_ratio + rng.uniform(-settings['speed_spread'], settings['speed_spread']), .50, .92))
        lane_members[lane_index].append((f'main_{i}', lane_limits[lane_index] * ratio))
    if leader_count:
        legacy_speed = 10. if task == 'passing' else 16.
        leader_speed = min(legacy_speed, (.6 if task == 'passing' else .8) * ceiling, .90 * lane_limits[0])
        lane_members[0].append(('slow_leader', leader_speed))
    for lane_index, members in lane_members.items():
        distance = float(rng.uniform(25., 80.))
        previous = None
        for slot, speed in members:
            if previous is not None:
                distance += LENGTH + _safe_gap(previous, speed) + float(rng.uniform(*settings['spacing_slack_m']))
            edge, lane, position, distance, lid = _main_position(graph, paths[lane_index], distance)
            physical_rows.append(physical(slot, 'main', edge, lane, position, speed, lid))
            previous = speed
    physical_rows.extend(ramps)

    if traffic_mode == 'sustained':
        depart = stabilization_seconds + 3.
        index = 0
        while depart < stabilization_seconds + horizon_seconds:
            ramp = task == 'merge' and index % (2 * len(paths) + 1) == 2 * len(paths)
            lane_index = index % len(paths)
            lid = 'mlk_on_0' if ramp else paths[lane_index][0]
            speed = _lane_limit(graph, lid, ceiling) * float(flow_rng.uniform(.55 if ramp else .70, .78 if ramp else .90))
            row = physical(f'inflow_{index}', 'ramp' if ramp else 'main', 0,
                           0 if ramp else graph.lanes[lid]['index'], 10., speed, lid,
                           depart=depart, initial=False)
            physical_rows.append(row)
            depart += settings['arrival_interval_s'] * float(flow_rng.uniform(.9, 1.1))
            index += 1
    validate_initial_spawns(graph, physical_rows, ceiling=ceiling)
    eligible = [row['slot'] for row in physical_rows if row['initial_cohort'] and row['slot'] != 'slow_leader']
    order = list(assignment_rng.permutation(eligible))
    if ramps:
        first_ramp = next(slot for slot in order if slot.startswith('ramp_'))
        order.remove(first_ramp)
        order.insert(0, first_ramp)
    assignment = {slot: f'agent_{i}' for i, slot in enumerate(order[:controlled_count])}
    root = ET.Element('routes')
    common = dict(length=str(LENGTH), minGap=str(MIN_GAP), emergencyDecel='9',
                  maxSpeed=str(CAP), speedDev='0', sigma='0')
    ET.SubElement(root, 'vType', id='controlled', carFollowModel='ACC', tau='1.5',
                  accel='2', decel='4.5', speedFactor='1', **common)
    for name, profile in PROFILES.items():
        ET.SubElement(root, 'vType', id=name, carFollowModel='IDM',
                      **{key: str(value) for key, value in profile.items()}, **common)
    ET.SubElement(root, 'route', id='main', edges=' '.join(MAIN_ROUTE))
    if task == 'merge':
        ET.SubElement(root, 'route', id='ramp', edges=' '.join(RAMP_ROUTE))
    rows = []
    # Native SUMO requires time-sorted departures; front-to-back ordering at
    # equal times is physical and independent of assignment or agent IDs.
    for row in sorted(physical_rows, key=lambda item: (item['depart'], item['route'], -item['edge'], -item['position'], item['slot'])):
        controlled = row['slot'] in assignment
        vid = assignment.get(row['slot'], row['slot'])
        vehicle = ET.SubElement(root, 'vehicle', id=vid,
            type='controlled' if controlled else row['driver_profile'], route=row['route'],
            depart=str(row['depart']), departEdge=str(row['edge']), departLane=str(row['lane']),
            departPos=str(row['position']), departSpeed=str(row['speed']))
        for key, value in [('research.origin', row['route']), ('research.population_slot', row['slot']),
                           ('research.initial_cohort', str(row['initial_cohort']).lower())]:
            ET.SubElement(vehicle, 'param', key=key, value=value)
        rows.append(dict(row, vehicle_id=vid, controlled=controlled))
    scenario = dict(schema=SCHEMA, physical_seed=seed, assignment_seed=assignment_seed,
        difficulty=difficulty, difficulty_name=settings['name'], traffic_mode=traffic_mode,
        speed_limit_m_s=speed_limit_m_s, resolved_speed_ceiling_m_s=ceiling,
        stabilization_seconds=stabilization_seconds, horizon_seconds=horizon_seconds,
        departure_warmup_seconds=stabilization_seconds,
        departure_time_encoding='SUMO integer milliseconds; nearest millisecond with positive half-up rounding',
        through_lanes=len(paths), ramp_length_m=ramp_length,
        base_mainline_speed_fraction=base_ratio, driver_profiles=PROFILES,
        assignment_rule='Independent seeded nested subset; one ramp agent first for merge; fixed throughout episode',
        physical_matching='Positions, speeds, native driver profiles and departure times unchanged across AV counts',
        spawn_validation='Static connected-lane clearance plus required native reset verification',
        spawn_gap_rule='Maximum of cautious native headway/braking floor and stationary ACC insertion clearance, before AV assignment',
        sustained_arrival_interval_s=settings['arrival_interval_s'] if traffic_mode == 'sustained' else None)
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)
    return DemandManifest(rows, scenario)


def demand_report(manifest):
    """Compact resolved demand; observed on-road fractions belong to the env."""
    initial = [row for row in manifest if row['initial_cohort']]
    later = [row for row in manifest if not row['initial_cohort']]
    controlled = sum(row['controlled'] for row in initial)
    speeds = [row['speed'] for row in initial]
    return dict(getattr(manifest, 'scenario', {}), initial_vehicle_count=len(initial),
        controlled_count=controlled, initial_native_count=len(initial) - controlled,
        requested_av_fraction=controlled / max(1, len(initial)),
        scheduled_vehicle_count=len(manifest), scheduled_later_native_count=len(later),
        scheduled_all_departure_av_fraction=controlled / max(1, len(manifest)),
        scheduled_fraction_dilution=(controlled / max(1, len(initial)) - controlled / max(1, len(manifest))),
        initial_ramp_count=sum(row['route'] == 'ramp' for row in initial),
        initial_speed_range_m_s=[min(speeds), max(speeds)] if speeds else [],
        first_later_departure_s=min((row['depart'] for row in later), default=None),
        last_later_departure_s=max((row['depart'] for row in later), default=None),
        realized_on_road_fraction='Must be measured from native departed/present/arrived IDs; scheduled fraction is not realized traffic')
