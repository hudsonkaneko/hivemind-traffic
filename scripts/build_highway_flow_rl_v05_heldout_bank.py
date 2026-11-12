"""Build a frozen held-out GEOMETRY bank of merge designs (v05, Nov 11 2026).

Every training/validation road so far is one family: uniform 5-segment
highway, 2-5 through lanes, 3.2 m lanes, 50-60 mph, 30 mph right-side ramp
whose auxiliary lane ends at n3, gentle bend (<= 30 m). The v04 held-out rows
reuse those roads with new demand seeds. This bank changes the geometry
itself, while staying inside what the environment can represent: fixed edge
names (sb1..sb5, mlk_on), a constant through-lane count, a right-side
auxiliary lane fed only by the ramp (the ramp-started design).

HELD-OUT DISCIPLINE: build once, record hashes, and evaluate a frozen actor
only. Never tune rewards, gates or training data on these results.

Designs that need environment changes (left entries, no acceleration lane,
mainline lane drops, weaving with an exit, consecutive ramps, loop ramps) are
listed in UNSUPPORTED and are not approximated here.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from scripts.build_highway_curriculum_v2 import normalize_network, sha256
from scripts.build_highway_flow_rl_v05_ramp_networks import validate as validate_ramp_started
from scripts.build_i110_sumo import write_xml
from traffic.sumo_backend import find_sumo_binary

ROOT = Path(__file__).resolve().parents[1]
GEOMETRY_VERSION = 'highway_heldout_merge_designs_v05'
TRAINED_FAMILY = dict(through_lanes=[2, 3, 4, 5], lane_width_m=3.2, speed_limit_mph=[50, 55, 60],
                      ramp_speed_mph=30, ramp_upstream_m=[120, 180, 240], aux_lane_ends='n3',
                      max_abs_bend_m=30, segments='uniform')

# id, description, and builder options. Defaults: uniform 2000 m road, 3.2 m
# lanes, 30 mph ramp, 160 m upstream ramp, aux lane ends at n3, no bend.
DESIGNS = [
    ('heldout-short-accel-3lane', 'Short 180 m acceleration lane (aux lane ends at n2)',
     dict(lanes=3, segments=[400, 180, 520, 450, 450], speed_mph=55, aux_end=2, ramp_upstream=160.)),
    ('heldout-long-accel-2lane', 'Long acceleration lane continuing to n4 (~1250 m)',
     dict(lanes=2, segments=[400, 420, 420, 420, 400], speed_mph=50, aux_end=4, ramp_upstream=150.)),
    ('heldout-6lane', 'Six through lanes (trained maximum is five)',
     dict(lanes=6, segments=[420, 420, 420, 420, 420], speed_mph=60, aux_end=3, ramp_upstream=200.)),
    ('heldout-tight-curve-3lane', 'Merge on a strong curve (150 m lateral bend vs <= 30 m trained)',
     dict(lanes=3, segments=[420, 420, 420, 420, 420], speed_mph=55, bend=150., aux_end=3, ramp_upstream=160.)),
    ('heldout-45mph-4lane', 'Slower 45 mph road with a 25 mph ramp',
     dict(lanes=4, segments=[400, 400, 400, 400, 400], speed_mph=45, ramp_speed_mph=25, aux_end=3, ramp_upstream=140.)),
    ('heldout-wide-lanes-2lane', 'US-standard 3.65 m lanes (trained 3.2 m)',
     dict(lanes=2, segments=[400, 400, 400, 400, 400], speed_mph=55, lane_width=3.65, aux_end=3, ramp_upstream=160.)),
    ('heldout-short-ramp-4lane', 'Short 60 m-upstream ramp with a brief parallel approach',
     dict(lanes=4, segments=[420, 420, 420, 420, 420], speed_mph=60, aux_end=3, ramp_upstream=60.)),
    ('heldout-fast-ramp-3lane', 'Long 45 mph ramp (300 m upstream) onto a 55 mph road',
     dict(lanes=3, segments=[500, 420, 420, 420, 420], speed_mph=55, ramp_speed_mph=45, aux_end=3, ramp_upstream=300.)),
]

UNSUPPORTED = {
    'left_side_entry': 'Lane identity counts auxiliary lanes from the right edge only',
    'no_acceleration_lane': 'Ramp joining a through lane directly needs a different merge-completion rule',
    'mainline_lane_drop': 'Demand requires the same through-lane count on sb1..sb5',
    'weaving_with_exit': 'Fixed main/ramp routes have no exit edge',
    'consecutive_ramps': 'Environment supports a single mlk_on ramp',
    'loop_ramp': 'Not representable by the single straight-to-parallel ramp shape',
}


def geometry(*, lanes, segments, speed_mph, aux_end, ramp_upstream, bend=0., lane_width=3.2,
             ramp_speed_mph=30, parallel_length=60., gore_setback=5.):
    if lanes not in range(2, 7) or len(segments) != 5 or min(segments) < 150 or aux_end not in (2, 3, 4):
        raise ValueError('Unsupported held-out design parameters')
    total = float(sum(segments))
    cumulative = [0.]
    for length in segments:
        cumulative.append(cumulative[-1] + length)

    def position(distance):
        return bend * math.sin(distance / total * math.pi), -distance

    nodes, edges, connections = ET.Element('nodes'), ET.Element('edges'), ET.Element('connections')
    for index, distance in enumerate(cumulative):
        x, y = position(distance)
        ET.SubElement(nodes, 'node', id=f'n{index}', x=f'{x:.3f}', y=f'{y:.3f}', type='priority')
    def lane_count(edge):  # edge index 1..5; aux lane on sb2..sb{aux_end}
        return lanes + (2 <= edge <= aux_end)
    for edge in range(1, 6):
        ET.SubElement(edges, 'edge', {'id': f'sb{edge}', 'from': f'n{edge-1}', 'to': f'n{edge}',
                                      'numLanes': str(lane_count(edge)), 'width': str(lane_width),
                                      'speed': str(speed_mph * .44704), 'priority': '3'})
    for edge in range(1, 5):
        here, there = lane_count(edge), lane_count(edge + 1)
        for lane in range(here):
            if here > there and lane == 0:
                continue  # auxiliary lane ends here
            target = lane + (there - here)
            ET.SubElement(connections, 'connection', {'from': f'sb{edge}', 'to': f'sb{edge+1}',
                                                     'fromLane': str(lane), 'toLane': str(target)})
    # Ramp placement follows the historical construction at n1.
    d1 = cumulative[1]
    jx, jy = position(d1)
    dx, dy = bend * math.pi / total * math.cos(math.pi * d1 / total), -1.
    magnitude = math.hypot(dx, dy)
    tx, ty = dx / magnitude, dy / magnitude
    rx, ry = ty, -tx
    offset = (lanes + 1) * lane_width + 45.
    sx, sy = jx - ramp_upstream * tx + offset * rx, jy - ramp_upstream * ty + offset * ry
    ET.SubElement(nodes, 'node', id='ramp_start', x=f'{sx:.3f}', y=f'{sy:.3f}', type='priority')
    side = (lanes + .5) * lane_width
    parallel = min(parallel_length, .6 * ramp_upstream)
    bend_point = (jx - parallel * tx + side * rx, jy - parallel * ty + side * ry)
    gore = (jx - gore_setback * tx + side * rx, jy - gore_setback * ty + side * ry)
    ET.SubElement(edges, 'edge', {'id': 'mlk_on', 'from': 'ramp_start', 'to': 'n1', 'numLanes': '1',
                                  'width': str(lane_width), 'speed': str(ramp_speed_mph * .44704), 'priority': '2',
                                  'shape': ' '.join(f'{x:.2f},{y:.2f}' for x, y in ((sx, sy), bend_point, gore))})
    ET.SubElement(connections, 'connection', {'from': 'mlk_on', 'to': 'sb2', 'fromLane': '0', 'toLane': '0'})
    return nodes, edges, connections, dict(road_tangent_at_n1=[tx, ty], parallel_approach_m=parallel - gore_setback,
                                           aux_lane_end_node=f'n{aux_end}', segment_lengths_m=list(segments))


def validate(network_path, options):
    checks = validate_ramp_started(network_path, options['lanes'])
    root = ET.parse(network_path).getroot()
    lanes = {l.get('id'): l for l in root.findall('./edge/lane')}
    ramp = next(c for c in root.findall('connection') if c.get('from') == 'mlk_on')
    connector_speed = float(lanes[ramp.get('via')].get('speed'))
    ramp_speed = options.get('ramp_speed_mph', 30) * .44704
    aux_edges = [f'sb{e}' for e in range(1, 6)
                 if len([l for l in lanes if l.startswith(f'sb{e}_')]) > options['lanes']]
    widths = {round(float(l.get('width', 3.2)), 2) for lid, l in lanes.items() if not lid.startswith(':')}
    checks.update(ramp_connector_speed_m_s=connector_speed, ramp_speed_limit_m_s=ramp_speed,
                  auxiliary_edges=aux_edges, lane_widths_m=sorted(widths))
    checks['passed'] = bool(checks['passed'] and connector_speed >= ramp_speed - 1e-6
                            and aux_edges == [f'sb{e}' for e in range(2, options['aux_end'] + 1)]
                            and widths == {round(options.get('lane_width', 3.2), 2)})
    return checks


def build(output, ident, description, options):
    output = Path(output)
    if output.exists():
        raise ValueError('Use a fresh output directory')
    nodes, edges, connections, placement = geometry(**options)
    output.mkdir(parents=True)
    for name, data in [('nodes.nod.xml', nodes), ('edges.edg.xml', edges), ('connections.con.xml', connections)]:
        write_xml(output / name, data)
    binary = find_sumo_binary().with_name('netconvert.exe' if os.name == 'nt' else 'netconvert')
    arguments = ['--node-files', 'nodes.nod.xml', '--edge-files', 'edges.edg.xml',
                 '--connection-files', 'connections.con.xml', '--output-file', 'network.raw.net.xml',
                 '--no-turnarounds', 'true', '--offset.disable-normalization', 'true', '--check-lane-foes.all', 'true']
    result = subprocess.run([str(binary), *arguments], cwd=output, capture_output=True, text=True, timeout=60)
    (output / 'netconvert.log').write_text(result.stdout + result.stderr, encoding='utf-8')
    result.check_returncode()
    normalize_network(output / 'network.raw.net.xml', output / 'network.net.xml')
    checks = validate(output / 'network.net.xml', options)
    ramp = ET.parse(output / 'network.net.xml').find("./edge[@id='mlk_on']/lane")
    entry = dict(id=ident, description=description, options=options, placement=placement,
                 compiled_checks=checks, compiled_ramp_length_m=float(ramp.get('length')),
                 network_sha256=sha256(output / 'network.net.xml'))
    (output / 'geometry-manifest.json').write_text(json.dumps(entry, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    if not checks['passed']:
        raise ValueError(f'{ident} failed structural checks: {checks}')
    return entry


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.exists():
        raise ValueError('Held-out bank output must be fresh; never rebuild over a frozen bank')
    args.output.mkdir(parents=True)
    layouts = [build(args.output / ident, ident, description, options) for ident, description, options in DESIGNS]
    binary = find_sumo_binary().with_name('netconvert.exe' if os.name == 'nt' else 'netconvert')
    bank = dict(geometry_version=GEOMETRY_VERSION, split='held_out_geometry', frozen=True,
                discipline='Frozen-actor evaluation only; never tune rewards, gates or training data on these results',
                trained_family=TRAINED_FAMILY, unsupported_designs=UNSUPPORTED,
                netconvert_version=subprocess.run([str(binary), '--version'], capture_output=True, text=True,
                                                  timeout=15, check=True).stdout.splitlines()[0],
                builder_sha256=sha256(Path(__file__)), layouts=layouts, traffic_run=False, training=False)
    (args.output / 'bank.json').write_text(json.dumps(bank, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps([dict(id=l['id'], passed=l['compiled_checks']['passed'], ramp_m=round(l['compiled_ramp_length_m'], 1),
                           connector_m_s=l['compiled_checks']['ramp_connector_speed_m_s'],
                           aux=l['compiled_checks']['auxiliary_edges']) for l in layouts], indent=1))


if __name__ == '__main__':
    main()
