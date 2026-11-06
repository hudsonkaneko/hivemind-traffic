"""Build immutable synthetic flow counterfactuals; never run traffic or training.

An added-lane ramp is geometrically distinct from a shared auxiliary yield.
Open visibility is an explicit synthetic unobstructed-road assumption, not a
measurement of I-110. The unchanged imported I-110 remains a legacy fixture.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np

from scripts.build_highway_curriculum_v2 import source_geometry as old_source_geometry, normalize_network, _shape, _segments_cross
from scripts.build_i110_sumo import write_xml
from traffic.sumo_backend import find_sumo_binary
from environments.highway_environment_v05 import ContinuousMotionLaneGraph
from traffic.highway_v05_demand import PROFILES, _safe_gap

SCHEMA = 'highway_flow_counterfactual_bank_v4'
FIXTURE_VERSION = 'highway_flow_explicit_spawns_v3'
ADDED_LANE_GEOMETRY_VERSION = 'highway_flow_parallel_added_lane_v2'
SHARED_GEOMETRY_VERSION = 'highway_flow_shared_auxiliary_v1'
SEEDS = (6810101, 6810102, 6810103)
FIXTURES = ('empty-ramp', 'target-leader', 'rear', 'cutin', 'farther-slow-leader', 'ending-lane-temptation', 'dense-blocked')
VARIANTS = ('added-lane', 'shared-open', 'shared-default')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_geometry(variant):
    if variant not in VARIANTS:
        raise ValueError('Unknown flow geometry variant')
    nodes, edges, connections, placement = old_source_geometry(lanes=3, length=1200., bend=0., speed_mph=18. / .44704, ramp_upstream_distance=160.)
    for edge in edges:
        if edge.get('id') == 'mlk_on':
            edge.set('speed', '18.0')
        if variant == 'added-lane' and edge.get('id') == 'sb1':
            edge.set('numLanes', '3')
    for connection in list(connections):
        if connection.get('from') == 'mlk_on':
            if variant != 'added-lane':
                connection.set('visibility', '100.0' if variant == 'shared-open' else '4.5')
        if variant == 'added-lane' and connection.get('from') == 'sb1':
            connections.remove(connection)
    if variant == 'added-lane':
        for lane in range(3):
            ET.SubElement(connections, 'connection', {'from': 'sb1', 'to': 'sb2', 'fromLane': str(lane), 'toLane': str(lane + 1)})
        # A synthetic parallel acceleration lane meets the new lane on the same
        # tangent. Do not force a sharp diagonal endpoint into a short connector
        # and then override SUMO's physically derived corner speed to hide it.
        # PlainXML permits shape endpoints at the junction boundary rather than
        # the node reference point. Center spreading fixes the single lane's
        # center at the outgoing auxiliary center (four right-spread lanes).
        width = 3.2
        ramp_x = round(-(4. - .5) * width, 6)
        start_y, end_y = -80., -240.
        ramp_node = next(n for n in nodes if n.get('id') == 'ramp_start')
        ramp_node.set('x', str(ramp_x))
        ramp_node.set('y', str(start_y))
        ramp_edge = next(e for e in edges if e.get('id') == 'mlk_on')
        ramp_edge.set('spreadType', 'center')
        ramp_edge.set('shape', f'{ramp_x},{start_y} {ramp_x},{end_y}')
        placement.update(right_offset_m=abs(ramp_x), ramp_start_xy=[ramp_x, start_y],
            ramp_geometry_endpoint_xy=[ramp_x, end_y],
            ramp_tangent=[0., -1.], ramp_geometry='Straight parallel synthetic approach, tangent to added lane')
    return nodes, edges, connections, dict(placement, variant=variant,
        geometry_version=ADDED_LANE_GEOMETRY_VERSION if variant == 'added-lane' else SHARED_GEOMETRY_VERSION,
        visibility_assumption='100 m unobstructed planar synthetic approaches, no authored occluders; not surveyed I-110' if variant == 'shared-open' else
            'Preserved default 4.5 m sensitivity control' if variant == 'shared-default' else 'Separate added lane; no shared successor at entry',
        site_accuracy_claim=False, mainline_and_ramp_limit_m_s=18.)


def tangent_entry_evidence(lanes, ramp):
    """Measured compiled geometry; fail closed rather than raising turn speed."""
    approach = _shape(lanes['mlk_on_0'])
    connector = _shape(lanes[ramp.get('via')])
    outgoing = _shape(lanes['sb2_0'])
    vectors = [((b[0] - a[0]), (b[1] - a[1])) for a, b in
        [(approach[-2], approach[-1]), *zip(connector, connector[1:]), (outgoing[0], outgoing[1])]
        if math.hypot(b[0] - a[0], b[1] - a[1]) > 1e-8]
    if len(vectors) < 3:
        raise ValueError('Tangent entry requires nondegenerate compiled approach/connector/departure')
    unit = [(x / math.hypot(x, y), y / math.hypot(x, y)) for x, y in vectors]
    maximum_angle = max(math.degrees(math.acos(max(-1., min(1., ax * bx + ay * by))))
        for (ax, ay), (bx, by) in zip(unit, unit[1:]))
    separation = max(math.dist(approach[-1], connector[0]), math.dist(connector[-1], outgoing[0]))
    speeds = {name: float(lanes[name].get('speed')) for name in ('mlk_on_0', ramp.get('via'), 'sb2_0')}
    return dict(maximum_heading_change_degrees=maximum_angle, maximum_endpoint_gap_m=separation,
        compiled_lane_limits_m_s=speeds, connector_length_m=float(lanes[ramp.get('via')].get('length')),
        speed_override_applied=False, geometry_source='Compiled lane polylines and native compiler limits')


def validate_geometry(path, variant):
    root = ET.parse(path).getroot()
    lanes = {lane.get('id'): lane for lane in root.findall('./edge/lane')}
    connections = root.findall('connection')
    ramp = [c for c in connections if c.get('from') == 'mlk_on']
    if len(ramp) != 1 or ramp[0].get('to') != 'sb2' or ramp[0].get('toLane') != '0':
        raise ValueError('Missing unique ramp-to-auxiliary connection')
    ramp = ramp[0]
    junction = next(j for j in root.findall('junction') if ramp.get('via') in j.get('intLanes', '').split())
    internal = junction.get('intLanes').split()
    requests = {int(r.get('index')): r for r in junction.findall('request')}
    ri = internal.index(ramp.get('via'))
    request = requests[ri]
    bit = lambda row, kind, i: row.get(kind)[::-1][i] == '1'
    through = [c for c in connections if c.get('from') == 'sb1' and c.get('to') == 'sb2' and int(c.get('toLane')) > 0]
    if len(through) != 3:
        raise ValueError('Three independent through connections required')
    for c in through:
        ci = internal.index(c.get('via'))
        if any(bit(row, kind, i) for row, i in ((request, ci), (requests[ci], ri)) for kind in ('foes', 'response')):
            raise ValueError('False through-lane foe in flow network')
        if any(_segments_cross(a, b, c0, d) for a, b in zip(_shape(lanes[ramp.get('via')]), _shape(lanes[ramp.get('via')])[1:])
               for c0, d in zip(_shape(lanes[c.get('via')]), _shape(lanes[c.get('via')])[1:])):
            raise ValueError('Ramp crosses a through connector')
    auxiliary = [c for c in connections if c.get('from') == 'sb1' and c.get('to') == 'sb2' and c.get('toLane') == '0']
    if variant == 'added-lane':
        passed = not auxiliary and ramp.get('state') == 'M' and '1' not in request.get('foes') and '1' not in request.get('response')
        visibility = None
        tangent = tangent_entry_evidence(lanes, ramp)
        if (tangent['maximum_heading_change_degrees'] > .1 or tangent['maximum_endpoint_gap_m'] > .02
                or min(tangent['compiled_lane_limits_m_s'].values()) < 18. - 1e-8):
            raise ValueError('Added-lane nominal entry must be geometrically tangent and natively support18m/s')
    else:
        if len(auxiliary) != 1:
            raise ValueError('Shared variant must retain true auxiliary stream')
        ai = internal.index(auxiliary[0].get('via'))
        visibility = float(ramp.get('visibility', ramp.get('visibilityDistance', '4.5')))
        tangent = None
        passed = (ramp.get('state') == 'm' and bit(request, 'foes', ai) and bit(request, 'response', ai)
            and bit(requests[ai], 'foes', ri) and not bit(requests[ai], 'response', ri)
            and visibility == (100. if variant == 'shared-open' else 4.5))
    if any(c.get('from') == 'sb3' and c.get('fromLane') == '0' and c.get('to') == 'sb4' for c in connections):
        raise ValueError('Ending auxiliary lane must not silently jump to through lane')
    if not passed:
        raise ValueError('Compiled entry does not match declared flow semantics')
    return dict(passed=True, variant=variant, ramp_link_state=ramp.get('state'), visibility_m=visibility,
        geometry_version=ADDED_LANE_GEOMETRY_VERSION if variant == 'added-lane' else SHARED_GEOMETRY_VERSION,
        tangent_entry=tangent, compiled_connector_speed_m_s=float(lanes[ramp.get('via')].get('speed')),
        real_auxiliary_foe_retained=variant != 'added-lane', no_false_through_conflicts=True,
        scope='Static lane geometry/priority/compiled speed only; no vehicle test or physical sightline measurement')


def build_network(output, variant):
    output = Path(output).resolve()
    if not output.is_relative_to(ROOT / 'outputs') or output.exists():
        raise ValueError('Use a fresh project output directory')
    output.mkdir(parents=True)
    nodes, edges, connections, placement = source_geometry(variant)
    for name, value in (('nodes.nod.xml', nodes), ('edges.edg.xml', edges), ('connections.con.xml', connections)):
        write_xml(output / name, value)
    binary = find_sumo_binary().with_name('netconvert.exe' if os.name == 'nt' else 'netconvert')
    args = ['--node-files', 'nodes.nod.xml', '--edge-files', 'edges.edg.xml', '--connection-files', 'connections.con.xml',
        '--output-file', 'network.raw.net.xml', '--offset.disable-normalization', 'true', '--no-turnarounds', 'true', '--check-lane-foes.all', 'true']
    result = subprocess.run([str(binary), *args], cwd=output, capture_output=True, text=True, timeout=60)
    (output / 'netconvert.log').write_text(result.stdout + result.stderr, encoding='utf-8')
    result.check_returncode()
    normalize_network(output / 'network.raw.net.xml', output / 'network.net.xml')
    checks = validate_geometry(output / 'network.net.xml', variant)
    manifest = dict(schema=SCHEMA, variant=variant, placement=placement, static_checks=checks,
        source_sha256={p: sha(ROOT / p) for p in ('scripts/build_highway_flow_bank.py', 'scripts/build_highway_curriculum_v2.py', 'scripts/build_i110_sumo.py')},
        command=[str(binary), *args], netconvert_version=subprocess.check_output([str(binary), '--version'], text=True).splitlines()[0],
        network=(output / 'network.net.xml').relative_to(ROOT).as_posix(), network_sha256=sha(output / 'network.net.xml'),
        source_files={n: sha(output / n) for n in ('nodes.nod.xml', 'edges.edg.xml', 'connections.con.xml')},
        training=False, traffic_run=False, historical_sources_modified=False)
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return manifest


def fixture_definition(name, seed, graph):
    if name not in FIXTURES or seed not in SEEDS:
        raise ValueError('Counterfactual name/seed not in frozen train bank')
    rng = np.random.default_rng(seed)
    jitter = float(rng.uniform(-1., 1.))
    rows, events, controls = [], [], []
    def add(vid, lane, pos, speed, *, origin='main'):
        edge = graph.lanes[lane]['edge']
        route = ('mlk_on', 'sb2', 'sb3', 'sb4', 'sb5') if origin == 'ramp' else ('sb1', 'sb2', 'sb3', 'sb4', 'sb5')
        rows.append(dict(vehicle_id=vid, slot=vid, controlled=vid == 'agent_0', route=origin, edge=route.index(edge), lane=graph.lanes[lane]['index'],
            lane_id=lane, position=float(pos), speed=float(speed), depart=0., initial_cohort=True, driver_profile='native_attentive'))
    if name in ('empty-ramp', 'target-leader'):
        add('agent_0', 'mlk_on_0', graph.lanes['mlk_on_0']['length'] - 100. + jitter, 16., origin='ramp')
        if name == 'target-leader':
            add('fixture_leader', 'sb2_1', 45. + jitter, 12.)
            controls.append(dict(vehicle_id='fixture_leader', speed_m_s=12.))
    elif name == 'rear':
        add('agent_0', 'sb2_0', 35. + jitter, 14., origin='ramp')
        # Native insertion rejected18m/s so close to the next lane, even though
        # both declared limits are18. Start16, then the same safe native speed
        # command requests18 after reset. Preserve the approaching-rear role.
        add('fixture_rear', 'sb1_1', graph.lanes['sb1_1']['length'] - 25. + jitter, 16.)
        controls.append(dict(vehicle_id='fixture_rear', speed_m_s=18.))
    elif name == 'cutin':
        add('agent_0', 'sb2_1', 60. + jitter, 18.)
        add('fixture_cutin', 'sb2_2', 110. + jitter, 12.)
        controls.append(dict(vehicle_id='fixture_cutin', speed_m_s=12.))
        events.append(dict(at_s=1., vehicle_id='fixture_cutin', lane_delta=-1,
                           purpose='Known schedule is evaluator-only; measured lateral motion enters actor only after it occurs'))
    elif name == 'farther-slow-leader':
        add('agent_0', 'sb2_1', 60. + jitter, 16.)
        add('fixture_fast', 'sb2_1', 105. + jitter, 15.5)
        add('fixture_far_slow', 'sb2_2', 150. + jitter, 10.)
        controls += [dict(vehicle_id='fixture_fast', speed_m_s=15.5), dict(vehicle_id='fixture_far_slow', speed_m_s=10.)]
        events.append(dict(at_s=1., vehicle_id='agent_0', lane_delta=1,
                           purpose='External common safe lane request exercises simultaneous occupied-corridor response; not a policy-chosen action'))
    elif name == 'ending-lane-temptation':
        add('agent_0', 'sb2_1', 70. + jitter, 16.)
        add('fixture_leader', 'sb2_1', 162.8 + jitter, 13.)
        add('fixture_left_rear', 'sb2_2', 65. + jitter, 18.)
        add('fixture_left_front', 'sb2_2', 150. + jitter, 18.)
        controls.append(dict(vehicle_id='fixture_leader', speed_m_s=13.))
    else:
        add('agent_0', 'sb3_0', 70. + jitter, 12., origin='ramp')
        for i, position in enumerate((20., 65., 110., 155.)):
            vid = f'fixture_blocker_{i}'
            add(vid, 'sb3_1', position + jitter, 8.)
            controls.append(dict(vehicle_id=vid, speed_m_s=8.))
        # Preserve actual priority-path traffic in a shared-entry conflict fixture.
        add('fixture_priority', 'sb1_0', graph.lanes['sb1_0']['length'] - 30., 14.)
        # Keep this real yield-conflict vehicle, but its40m approach must admit
        # native insertion while preparing to stop at an unprioritized link.
        add('fixture_ramp_conflict', 'mlk_on_0', graph.lanes['mlk_on_0']['length'] - 40., 8., origin='ramp')
    # Fixed-size v05-compatible initial cohort. Remote filler cars cannot affect
    # the local approach until after its initial diagnostic event; all are kept
    # in demand/accounting, never silently discarded.
    while len(rows) < 8:
        i = len(rows)
        lane = f'sb5_{i % 3}'
        pos = 30. + 65. * (i // 3)
        add(f'fixture_remote_{i}', lane, min(pos, graph.lanes[lane]['length'] - 15.), 18.)
    checks = validate_fixture_spawns(graph, rows)
    return dict(schema='highway_flow_fixture_v1', fixture_version=FIXTURE_VERSION,
        static_spawn_checks=checks, name=name, seed=seed, vehicles=rows, events=events, speed_controls=controls,
        description='Explicit synthetic train-only causal fixture; eight-car initial cohort with remote filler, no stochastic inflow',
        assumption='Events are external recorded counterfactual inputs, not future actor observations; native safety may refuse lane requests',
        expected_external_event_count=len(events), safety_exception=name == 'dense-blocked',
        nominal_unobstructed=name in ('empty-ramp', 'ending-lane-temptation'))


def validate_fixture_spawns(graph, rows):
    """Conservative authored checks, not a replacement for native insertion.

    Retain each case's explicit speeds: fail rather than silently clipping them.
    A native vehicle inserted within braking lookahead of a connector needs a
    small margin below its current/next limit. Future speed requests are separate
    native-safe controls and do not erase the actual initial physical state.
    """
    if len({row['vehicle_id'] for row in rows}) != len(rows):
        raise ValueError('Duplicate fixture vehicle identity')
    summaries = []
    for rear in rows:
        lid = rear['lane_id']
        if lid not in graph.lanes:
            raise ValueError('Unknown fixture spawn lane')
        length, speed = graph.lanes[lid]['length'], rear['speed']
        if not all(math.isfinite(x) for x in (rear['position'], speed)) or not 7.3 <= rear['position'] <= length - 10.:
            raise ValueError('Fixture spawn position is not safely inside lane')
        factor = 1. if rear['controlled'] else PROFILES[rear['driver_profile']]['speedFactor']
        limit = min(18., graph.speed_limits[lid]) * factor
        if not 0. <= speed <= limit + 1e-8:
            raise ValueError('Fixture departure exceeds current lane/profile limit')
        route = ('mlk_on', 'sb2', 'sb3', 'sb4', 'sb5') if rear['route'] == 'ramp' else ('sb1', 'sb2', 'sb3', 'sb4', 'sb5')
        distances = graph.distances(rear['lane_id'], rear['position'], route)
        yield_approaches = []
        reaction = 1.5 if rear['controlled'] else PROFILES[rear['driver_profile']]['tau']
        full_stop_room = reaction * speed + speed**2 / 9. + 2.5
        for source, entry in graph.yield_entries.items():
            if source not in distances:
                continue
            available = distances[source] + graph.lanes[source]['length'] - entry['stop_offset_m']
            if full_stop_room > available + 1e-8:
                raise ValueError('Fixture yield-approach spawn lacks conservative full-stop room')
            yield_approaches.append(dict(source_lane=source, available_stop_room_m=available,
                required_stop_room_m=full_stop_room))
        ahead = [(distance, lane, min(18., graph.speed_limits[lane]) * factor)
                 for lane, distance in distances.items() if lane != lid and distance >= 0.]
        for distance, lane, ahead_limit in ahead:
            if speed > ahead_limit and .1 * speed + (speed**2 - ahead_limit**2) / 9. > distance + 1e-8:
                raise ValueError('Fixture departure cannot brake for upcoming map limit')
        upcoming = min(ahead) if ahead else None
        lookahead = 1.5 * speed + speed**2 / 9. + 2.5
        if not rear['controlled'] and upcoming and upcoming[0] <= lookahead:
            if speed > min(limit, upcoming[2]) - .1 + 1e-8:
                raise ValueError('Native near-connector insertion requires speed margin')
        for front in rows:
            if front is rear or front['lane_id'] not in distances:
                continue
            distance = distances[front['lane_id']] + front['position']
            if distance >= 0 and distance - 4.8 < _safe_gap(speed, front['speed']) - 1e-8:
                raise ValueError('Counterfactual has insufficient connected-lane initial clearance')
        summaries.append(dict(vehicle_id=rear['vehicle_id'], speed_m_s=speed, current_limit_m_s=limit,
            next_lane=upcoming[1] if upcoming else None, next_lane_distance_m=upcoming[0] if upcoming else None,
            next_lane_limit_m_s=upcoming[2] if upcoming else None, yield_approaches=yield_approaches))
    return dict(passed=True, vehicles=summaries, scope='Static authored position/profile/map/braking/clearance checks; native insertion still required')


def legacy_rows(config):
    specifications = [('cruise', 'train-3-lane-cruise', 6800020, 16, 0), ('following', 'train-3-lane-following', 6800021, 16, 0),
        ('passing', 'train-3-lane-passing', 6800022, 16, 0), ('merge', 'train-3-lane-merge', 6800023, 16, 0),
        ('dense-following', 'train-3-lane-following', 6800024, 24, 2), ('i110-merge', 'i110-merge', 6800019, 16, 2)]
    result = []
    for label, case_id, seed, total, difficulty in specifications:
        case = dict(next(c for c in config['training_cases'] if c['id'] == case_id))
        if label == 'i110-merge':
            path = ROOT / 'outputs/i110-merge-blocking/v01/network-v02/network.net.xml'
            case.update(network=path.relative_to(ROOT).as_posix(), network_sha256=sha(path))
        result.append(dict(id='legacy-' + label, case=case, seed=seed, controlled_count=8, total_vehicles=total, difficulty=difficulty,
            speed_limit_m_s=18., communication='local', packet_loss=0., traffic_mode='sustained', fixture=None, group='legacy'))
    return result


def build(output, parent_config):
    output = Path(output).resolve()
    if not output.is_relative_to(ROOT / 'outputs') or output.exists():
        raise ValueError('Bank must use a fresh project output directory')
    output.mkdir(parents=True)
    networks = {variant: build_network(output / variant, variant) for variant in VARIANTS}
    cases = []
    for name in FIXTURES:
        variant = 'added-lane' if name == 'empty-ramp' else 'shared-open'
        network = networks[variant]
        graph = ContinuousMotionLaneGraph(ROOT / network['network'], 3)
        for seed in SEEDS:
            fixture = fixture_definition(name, seed, graph)
            case = dict(id='flow-' + name, split='train', network=network['network'], network_sha256=network['network_sha256'],
                        through_lanes=3, speed_limit_mph=18. / .44704, task='merge' if name in ('empty-ramp', 'target-leader', 'rear', 'dense-blocked') else 'cruise', density='light')
            cases.append(dict(id=f'{name}-{seed}', case=case, seed=seed, controlled_count=1, total_vehicles=8, difficulty=0,
                speed_limit_m_s=18., communication='none', packet_loss=0., traffic_mode='initial', fixture=fixture, group='counterfactual'))
    config = json.loads(Path(parent_config).read_text())
    old = legacy_rows(config)
    manifest = dict(schema=SCHEMA, training=False, horizon_steps=800, dt_s=.1, seeds=SEEDS, counterfactuals=cases, legacy=old,
        cases=cases + old, network_alternatives=networks, preparation_wall_seconds=1800,
        planned_counterfactual_runs=63, legacy_runs_additional=18,
        coverage_policy='63 counterfactual +18 legacy controller runs share1800s; incomplete if cap prevents completion, never shorten horizons',
        primary_controllers=['native', 'frozen-v05', 'flow-reference'],
        provenance=dict(parent_config=str(Path(parent_config).resolve().relative_to(ROOT)), parent_config_sha256=sha(parent_config), builder_sha256=sha(__file__)),
        visibility_scope='Synthetic shared-open100m and shared-default4.5m alternatives retained; imported I-110 untouched',
        held_out_used=False)
    (output / 'bank.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--parent-config', type=Path, default=ROOT / 'outputs/highway-environment/v05/train-20261106T024828Z-90b25880/config.json')
    args = parser.parse_args()
    bank = build(args.output, args.parent_config)
    print(json.dumps(dict(path=str(args.output / 'bank.json'), cases=len(bank['cases']), training=False)))
