"""Build right-side highway entries without altering historical v01 fixtures.

V01's ramp approached the southbound road from its left and crossed through
lanes. V02 places the ramp outside the right lane extent using the local road
tangent. Structural checks reject any through-connector crossing or foe link.
No traffic simulation or policy evaluation is performed by this builder.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from scripts.build_i110_sumo import write_xml
from traffic.sumo_backend import find_sumo_binary

ROOT = Path(__file__).resolve().parents[1]
GEOMETRY_VERSION = 'highway_right_entry_v2'
LANE_WIDTH = 3.2


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_geometry(*, lanes, length, bend, speed_mph):
    if isinstance(lanes, bool) or lanes not in range(2, 7):
        raise ValueError('Supported through-lane counts: 2..6')
    if not all(math.isfinite(value) for value in (length, bend, speed_mph)) or length < 1000 or not 0 < speed_mph <= 60:
        raise ValueError('Invalid highway dimensions or speed ceiling')
    nodes, edges, connections = ET.Element('nodes'), ET.Element('edges'), ET.Element('connections')
    def position(distance):
        return bend * math.sin(distance / length * math.pi), -distance
    for index in range(6):
        x, y = position(index * length / 5)
        ET.SubElement(nodes, 'node', id=f'n{index}', x=str(x), y=str(y), type='priority')
    for index in range(1, 6):
        ET.SubElement(edges, 'edge', {
            'id': f'sb{index}', 'from': f'n{index-1}', 'to': f'n{index}',
            'numLanes': str(lanes + (index < 4)), 'width': str(LANE_WIDTH),
            'speed': str(speed_mph * .44704), 'priority': '3',
        })
    for index in range(1, 5):
        for lane in range(lanes + (index < 4)):
            if index == 3 and lane == 0:
                continue
            ET.SubElement(connections, 'connection', {
                'from': f'sb{index}', 'to': f'sb{index+1}', 'fromLane': str(lane),
                'toLane': str(lane - 1 if index == 3 else lane),
            })
    distance = length / 5
    x, y = position(distance)
    dx, dy = bend * math.pi / length * math.cos(math.pi / 5), -1.
    magnitude = math.hypot(dx, dy)
    tangent = (dx / magnitude, dy / magnitude)
    right = (tangent[1], -tangent[0])
    offset = (lanes + 1) * LANE_WIDTH + 45.
    ramp_x = x - 160. * tangent[0] + offset * right[0]
    ramp_y = y - 160. * tangent[1] + offset * right[1]
    ET.SubElement(nodes, 'node', id='ramp_start', x=str(ramp_x), y=str(ramp_y), type='priority')
    ET.SubElement(edges, 'edge', {
        'id': 'mlk_on', 'from': 'ramp_start', 'to': 'n1', 'numLanes': '1',
        'width': str(LANE_WIDTH), 'speed': str(30 * .44704), 'priority': '2',
    })
    ET.SubElement(connections, 'connection', {'from': 'mlk_on', 'to': 'sb2', 'fromLane': '0', 'toLane': '0'})
    placement = dict(road_tangent=list(tangent), right_normal=list(right),
                     right_offset_m=offset, upstream_distance_m=160.,
                     mainline_lane_extent_m=(lanes + 1) * LANE_WIDTH,
                     ramp_start_xy=[ramp_x, ramp_y], junction_xy=[x, y])
    return nodes, edges, connections, placement


def normalize_network(raw_path, destination):
    """Discard netconvert metadata comments; preserve every network element."""
    tree = ET.parse(raw_path)  # The standard parser omits XML comments.
    ET.indent(tree, space='    ')
    tree.write(destination, encoding='utf-8', xml_declaration=True)


def _shape(lane):
    return [tuple(map(float, value.split(',')[:2])) for value in lane.attrib['shape'].split()]


def _segments_cross(a, b, c, d):
    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    # Proper crossings; sharing an auxiliary merge endpoint is allowed. Distinct
    # through-lane connectors must also be absent from the ramp's SUMO foe set.
    return orient(a, b, c) * orient(a, b, d) < -1e-8 and orient(c, d, a) * orient(c, d, b) < -1e-8


def validate_geometry(network_path, through_lanes):
    root = ET.parse(network_path).getroot()
    lane_elements = {lane.attrib['id']: lane for lane in root.findall('./edge/lane')}
    ramp = [c for c in root.findall('connection') if c.get('from') == 'mlk_on' and c.get('to') == 'sb2']
    if len(ramp) != 1 or ramp[0].get('toLane') != '0':
        raise ValueError('Exactly one ramp connection to the right auxiliary lane is required')
    ramp = ramp[0]
    through = [c for c in root.findall('connection') if c.get('from') == 'sb1' and c.get('to') == 'sb2' and int(c.get('fromLane')) >= 1]
    if len(through) != through_lanes:
        raise ValueError('Unexpected through-lane connector count')
    junctions = [j for j in root.findall('junction') if ramp.get('via') in j.get('intLanes', '').split()]
    if len(junctions) != 1:
        raise ValueError('Ramp internal connector has no unique junction')
    junction = junctions[0]
    internal = junction.attrib['intLanes'].split()
    request = next(q for q in junction.findall('request') if int(q.attrib['index']) == internal.index(ramp.attrib['via']))
    auxiliary = [c for c in root.findall('connection') if c.get('from') == 'sb1' and
                 c.get('to') == 'sb2' and c.get('fromLane') == '0' and c.get('toLane') == '0']
    if len(auxiliary) != 1:
        raise ValueError('Actual auxiliary-lane merge conflict must remain defined')
    auxiliary_index = internal.index(auxiliary[0].attrib['via'])
    auxiliary_foe = request.attrib['foes'][::-1][auxiliary_index] == '1'
    auxiliary_yield = request.attrib['response'][::-1][auxiliary_index] == '1'
    ramp_shape = _shape(lane_elements[ramp.attrib['via']])
    crossings, foes = [], []
    for connection in through:
        via = connection.attrib['via']
        shape = _shape(lane_elements[via])
        if any(_segments_cross(a, b, c, d) for a, b in zip(ramp_shape, ramp_shape[1:]) for c, d in zip(shape, shape[1:])):
            crossings.append(via)
        index = internal.index(via)
        if request.attrib['foes'][::-1][index] == '1' or request.attrib['response'][::-1][index] == '1':
            foes.append(via)
    # Compare the approaching ramp's first point against the incoming auxiliary
    # lane direction, avoiding a fixed world-X assumption on curved roads.
    main_shape = _shape(lane_elements['sb1_0'])
    endpoint = main_shape[-1]
    tx, ty = endpoint[0] - main_shape[-2][0], endpoint[1] - main_shape[-2][1]
    scale = math.hypot(tx, ty)
    right = (ty / scale, -tx / scale)
    ramp_start = _shape(lane_elements['mlk_on_0'])[0]
    signed_right = (ramp_start[0] - endpoint[0]) * right[0] + (ramp_start[1] - endpoint[1]) * right[1]
    passed = not crossings and not foes and signed_right > 0 and auxiliary_foe and auxiliary_yield
    return dict(passed=passed, geometry_version=GEOMETRY_VERSION,
                junction=junction.attrib['id'], ramp_internal_lane=ramp.attrib['via'],
                through_lane_count=through_lanes, connector_crossings=crossings,
                through_lane_foes=foes, approach_right_of_auxiliary_m=signed_right,
                auxiliary_merge_foe_retained=auxiliary_foe,
                auxiliary_merge_yield_retained=auxiliary_yield,
                scope='compiled geometry only; no traffic episode run')


def build(output, *, lanes, length, bend, speed_mph):
    output = Path(output).resolve()
    if not output.is_relative_to(ROOT / 'outputs') or output.exists():
        raise ValueError('Use a fresh project output directory')
    nodes, edges, connections, placement = source_geometry(lanes=lanes, length=length, bend=bend, speed_mph=speed_mph)
    output.mkdir(parents=True)
    for name, data in [('nodes.nod.xml', nodes), ('edges.edg.xml', edges), ('connections.con.xml', connections)]:
        write_xml(output / name, data)
    binary = find_sumo_binary().with_name('netconvert.exe' if os.name == 'nt' else 'netconvert')
    arguments = ['--node-files', 'nodes.nod.xml', '--edge-files', 'edges.edg.xml',
                 '--connection-files', 'connections.con.xml', '--output-file', 'network.raw.net.xml',
                 '--no-turnarounds', 'true', '--offset.disable-normalization', 'true',
                 '--check-lane-foes.all', 'true']
    result = subprocess.run([str(binary), *arguments], cwd=output, capture_output=True, text=True, timeout=60)
    (output / 'netconvert.log').write_text(result.stdout + result.stderr, encoding='utf-8')
    result.check_returncode()
    normalize_network(output / 'network.raw.net.xml', output / 'network.net.xml')
    checks = validate_geometry(output / 'network.net.xml', lanes)
    if not checks['passed']:
        (output / 'failed-geometry.json').write_text(json.dumps(checks, indent=2) + '\n')
        raise ValueError('Compiled right-entry geometry failed crossing/foe checks')
    version = subprocess.run([str(binary), '--version'], capture_output=True, text=True, timeout=15, check=True).stdout
    entry = dict(network=(output / 'network.net.xml').relative_to(ROOT).as_posix(),
                 network_sha256=sha256(output / 'network.net.xml'),
                 through_lanes=lanes, length_m=length, bend_m=bend, speed_limit_mph=speed_mph)
    manifest = dict(geometry_version=GEOMETRY_VERSION, **entry, placement=placement,
                    compiled_checks=checks, normalized_network='network.net.xml',
                    normalization='XML metadata comments removed; network elements retained',
                    input_sha256={name: sha256(output / name) for name in ('nodes.nod.xml', 'edges.edg.xml', 'connections.con.xml')},
                    source_sha256={name: sha256(ROOT / name) for name in ('scripts/build_highway_curriculum_v2.py', 'scripts/build_i110_sumo.py')},
                    netconvert_version=version.strip(), command=['netconvert', *arguments])
    manifest_path = output / 'geometry-manifest.json'
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    (output / 'raw-build-provenance.json').write_text(json.dumps(dict(
        raw_network_sha256=sha256(output / 'network.raw.net.xml'),
        normalized_network_sha256=entry['network_sha256'],
        note='Raw network retains generated timestamp/command comments; excluded from portable semantic bank hash'), indent=2) + '\n')
    entry.update(geometry_version=GEOMETRY_VERSION,
                 geometry_manifest=manifest_path.relative_to(ROOT).as_posix(),
                 geometry_manifest_sha256=sha256(manifest_path))
    return entry


def specifications():
    result = [('train', n, 1800 + 100 * n, 20 * (n % 2), 60 if n % 2 else 50) for n in (2, 3, 4, 5)]
    result += [('validation', n, 1900 + 100 * n, -30, 55) for n in (2, 4, 5)]
    result += [('test', n, 2200 + 100 * n, 45, 45 if n == 6 else 60) for n in (3, 5, 6)]
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'outputs/highway-curriculum/v02')
    args = parser.parse_args(argv)
    output = args.output.resolve()
    if not output.is_relative_to(ROOT / 'outputs') or output.exists():
        raise ValueError('Bank output must be a fresh project outputs directory')
    output.mkdir(parents=True)
    layouts = []
    for split, lanes, length, bend, speed in specifications():
        ident = f'{split}-{lanes}-lane'
        entry = build(output / ident, lanes=lanes, length=length, bend=bend, speed_mph=speed)
        entry.update(id=ident, split=split)
        layouts.append(entry)
    bank = dict(geometry_version=GEOMETRY_VERSION, layouts=layouts,
                historical_v01_preserved=True, imported_i110_unchanged=True,
                held_out_traffic_run=False)
    path = output / 'bank.json'
    path.write_text(json.dumps(bank, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps(dict(bank=path.relative_to(ROOT).as_posix(), sha256=sha256(path), layouts=len(layouts))))


if __name__ == '__main__':
    main()
