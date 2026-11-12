"""Build v05 ramp networks whose acceleration lane begins at the ramp.

The v05 network bank (scripts/build_highway_environment_v05_bank.py) adds the
right auxiliary lane from the very start of the road (sb1_0). The ramp then
joins an existing lane, so SUMO compiles the ramp link as minor ('m') and ramp
cars must stay able to stop at the gore. On a real on-ramp the ramp itself
becomes the acceleration lane. Here sb1 carries only the through lanes, the
auxiliary lane sb2_0 is fed by the ramp alone, and sb1 lane i continues into
sb2 lane i+1. Everything else (dimensions, bend, ramp placement, speeds, lane
drop at n3) reuses the historical geometry unchanged.

The historical bank, its builder and its checks are not modified. This builder
compiles networks only; it runs no traffic, training or evaluation.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from scripts.build_highway_curriculum_v2 import normalize_network, sha256, source_geometry
from scripts.build_highway_environment_v05_bank import specifications
from scripts.build_i110_sumo import write_xml
from traffic.sumo_backend import find_sumo_binary

ROOT = Path(__file__).resolve().parents[1]
GEOMETRY_VERSION = 'highway_ramp_started_acceleration_lane_v05'
LANE_WIDTH = 3.2
PARALLEL_LENGTH_M = 60.
GORE_SETBACK_M = 5.


def ramp_started_geometry(**options):
    """Historical source geometry with the auxiliary lane starting at the ramp."""
    nodes, edges, connections, placement = source_geometry(**options)
    lanes = options['lanes']
    sb1 = edges.find("edge[@id='sb1']")
    sb1.set('numLanes', str(lanes))
    for connection in list(connections):
        if connection.get('from') == 'sb1':
            connections.remove(connection)
    for lane in range(lanes):
        ET.SubElement(connections, 'connection', {'from': 'sb1', 'to': 'sb2', 'fromLane': str(lane),
                                                 'toLane': str(lane + 1)})
    # Keep the ramp connection last, matching the historical file order.
    ramp = connections.find("connection[@from='mlk_on']")
    connections.remove(ramp)
    connections.append(ramp)
    # Real on-ramps run parallel to the road before the gore. Bend the ramp's
    # final stretch onto the auxiliary lane's line so the junction connector
    # is straight instead of a ~24 degree corner with a turn-speed limit.
    tx, ty = placement['road_tangent']
    rx, ry = placement['right_normal']
    jx, jy = placement['junction_xy']
    sx, sy = placement['ramp_start_xy']
    side = (lanes + .5) * LANE_WIDTH  # aux-lane centre, right of the road line
    parallel = min(PARALLEL_LENGTH_M, .6 * placement['upstream_distance_m'])
    bend_point = (jx - parallel * tx + side * rx, jy - parallel * ty + side * ry)
    gore = (jx - GORE_SETBACK_M * tx + side * rx, jy - GORE_SETBACK_M * ty + side * ry)
    edge = edges.find("edge[@id='mlk_on']")
    edge.set('shape', ' '.join(f'{x:.2f},{y:.2f}' for x, y in ((sx, sy), bend_point, gore)))
    placement = dict(placement, parallel_approach_m=parallel - GORE_SETBACK_M,
                     ramp_shape=[[sx, sy], list(bend_point), list(gore)])
    return nodes, edges, connections, placement


def validate(network_path, through_lanes):
    root = ET.parse(network_path).getroot()
    ramp = [c for c in root.findall('connection') if c.get('from') == 'mlk_on' and c.get('to') == 'sb2']
    if len(ramp) != 1 or ramp[0].get('toLane') != '0':
        raise ValueError('Exactly one ramp connection to sb2_0 is required')
    ramp = ramp[0]
    sb1 = root.find("edge[@id='sb1']")
    feeders = [c for c in root.findall('connection') if c.get('to') == 'sb2' and c.get('toLane') == '0'
               and not c.get('from').startswith(':')]
    junction = next(j for j in root.findall('junction') if ramp.get('via') in j.get('intLanes', '').split())
    internal = junction.attrib['intLanes'].split()
    request = next(q for q in junction.findall('request') if int(q.attrib['index']) == internal.index(ramp.get('via')))
    through = [c for c in root.findall('connection') if c.get('from') == 'sb1' and c.get('to') == 'sb2']
    checks = dict(
        sb1_lane_count=len(sb1.findall('lane')),
        through_connections=sorted((int(c.get('fromLane')), int(c.get('toLane'))) for c in through),
        sb2_0_feeders=[c.get('from') for c in feeders],
        ramp_link_state=ramp.get('state'),
        ramp_request_foes=request.attrib['foes'], ramp_request_response=request.attrib['response'])
    checks['passed'] = (checks['sb1_lane_count'] == through_lanes
                        and checks['through_connections'] == [(i, i + 1) for i in range(through_lanes)]
                        and checks['sb2_0_feeders'] == ['mlk_on']
                        and checks['ramp_link_state'] == 'M'
                        and set(request.attrib['foes']) == {'0'} and set(request.attrib['response']) == {'0'})
    return checks


def build(output, **options):
    output = Path(output).resolve()
    if output.exists():
        raise ValueError('Use a fresh output directory')
    nodes, edges, connections, placement = ramp_started_geometry(**options)
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
    checks = validate(output / 'network.net.xml', options['lanes'])
    manifest = dict(geometry_version=GEOMETRY_VERSION, options=options, placement=placement,
                    compiled_checks=checks, network_sha256=sha256(output / 'network.net.xml'),
                    input_sha256={n: sha256(output / n) for n in ('nodes.nod.xml', 'edges.edg.xml', 'connections.con.xml')},
                    source_sha256={n: sha256(ROOT / n) for n in (
                        'scripts/build_highway_flow_rl_v05_ramp_networks.py', 'scripts/build_highway_curriculum_v2.py',
                        'scripts/build_highway_environment_v05_bank.py')},
                    netconvert_version=subprocess.run([str(binary), '--version'], capture_output=True, text=True,
                                                      timeout=15, check=True).stdout.splitlines()[0],
                    command=['netconvert', *arguments],
                    lane_id_change='sb1 has no auxiliary lane: historical sb1_k (k>=1) is now sb1_{k-1}')
    (output / 'geometry-manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    if not checks['passed']:
        raise ValueError(f'Ramp-started geometry failed checks: {checks}')
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.exists():
        raise ValueError('Bank output must be a fresh directory')
    layouts = []
    for options in specifications():
        split = options.pop('split')
        ident = f'{split}-{options["lanes"]}-lane-ramp{options["ramp_upstream_distance"]:g}'
        manifest = build(args.output / ident, **options)
        layouts.append(dict(id=ident, split=split, network_sha256=manifest['network_sha256'],
                            checks=manifest['compiled_checks']))
    bank = dict(geometry_version=GEOMETRY_VERSION, layouts=layouts, historical_bank_unchanged=True,
                traffic_run=False, training=False)
    (args.output / 'bank.json').write_text(json.dumps(bank, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps([dict(id=l['id'], passed=l['checks']['passed'], ramp=l['checks']['ramp_link_state'])
                      for l in layouts], indent=1))


if __name__ == '__main__':
    main()
