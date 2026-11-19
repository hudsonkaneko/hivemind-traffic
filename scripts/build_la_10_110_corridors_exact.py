"""Build SUMO merge corridors that follow the LA I-10 / I-110 model exactly (layout v2).

User decision (November 18, 2026): "dont need to verify, just use that road for the
training" ... "the exusting 3d model we have". So the corridors take everything from
the model's lane graph (highway_usd/la_10_110/lanegraph.json) and invent nothing:

- the three CA-110 through lanes, their centerlines and 3.66 m width, in the model's
  own ENU frame (SUMO coordinates are model coordinates);
- the on-ramp lane's own centerline, unchanged, ending where the model ends it: at the
  start of the merge way's right lane, where the model's ramp lane and right lane join
  (both list it as their successor). The model has no acceleration lane, so neither do
  these networks;
- each lane's speed limit from the model (`speed`, m/s): 29.1 m/s (65 mph) mainline and
  20.1 m/s (45 mph) ramp. The highway environments cap every vehicle at 26.8224 m/s
  (60 mph, vType maxSpeed), so mainline traffic still drives at most 60 mph.

What SUMO needs that the model does not state, and how it is set here:
- the merge is a SUMO junction at the model's merge point: the ramp connection is a
  minor link that yields to the right lane (edge priority), as on the existing I-110
  corridor;
- its declared visibility is 100 m, the value the existing I-110 corridor declares;
  the highway environments require a declared value and the model has no sight-distance
  data;
- the downstream road is split into four equal edges (sb2..sb5) because the
  environments address the corridor by those edge names. The split points are
  ordinary priority nodes with straight-through connections; they change no geometry.

The v1 builder (scripts/build_la_10_110_corridors.py) and its networks are unchanged,
so round 18 stays reproducible. No traffic is run and no policy is evaluated here.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts import build_la_10_110_corridors as v1  # noqa: E402

GEOMETRY_VERSION = 'la_10_110_corridor_v2_model_exact'
DECLARED_VISIBILITY_M = 100.   # as the existing I-110 corridor; the model has no sight-distance data
DOWNSTREAM_EDGES = 4
MAX_RAMP_DEVIATION_M = .25
SITES = v1.SITES


def model_speeds(lanes, site):
    """The model's own speed limits: one value for the corridor's through lanes, one for its ramp."""
    n = site['through_lanes']
    through = {lanes[f'{way}_{i}']['speed'] for way in (*site['upstream_ways'], *site['downstream_ways'])
               for i in range(n)}
    ramp = {lanes[lane]['speed'] for lane in site['ramp_lanes']}
    if len(through) != 1 or len(ramp) != 1:
        raise ValueError(f'The model gives several speed limits on this corridor: {through} / {ramp}')
    main, ramp_speed = float(next(iter(through))), float(next(iter(ramp)))
    if not (0. < ramp_speed and 0. < main):
        raise ValueError('Invalid model speed limit')
    return main, ramp_speed


def model_ramp(lanes, site, merge_point):
    """The model's ramp lane centerline; it must end on the right lane at the merge point."""
    ramp = v1.join([v1.lane_of(lanes, *lane.rsplit('_', 1))['pts'] for lane in site['ramp_lanes']])
    gap = float(np.linalg.norm(ramp[-1] - merge_point))
    if gap > .01:
        raise ValueError(f'The model ramp ends {gap:.3f} m from the right lane at the merge point')
    return ramp


def plain_xml(lanes, ident):
    site = SITES[ident]
    n = site['through_lanes']
    reference, gore, total = site_geometry(lanes, site)
    start = gore - site['upstream_m']
    downstream = total - gore
    stops = [start, gore, *(gore + downstream * k / DOWNSTREAM_EDGES for k in range(1, DOWNSTREAM_EDGES)), total]
    main_speed, ramp_speed = model_speeds(lanes, site)
    nodes, edges, connections = ET.Element('nodes'), ET.Element('edges'), ET.Element('connections')
    centre = (n - 1) / 2 * v1.LANE_WIDTH
    for index, station in enumerate(stops):
        point = v1.lane_shape(reference, np.array([station]), centre)[0]
        ET.SubElement(nodes, 'node', id=f'n{index}', x=f'{point[0]:.3f}', y=f'{point[1]:.3f}', z=f'{point[2]:.3f}',
                      type='priority' if 0 < index < len(stops) - 1 else 'dead_end')
    shapes = {}
    for index in range(1, len(stops)):
        stations = v1.stations_between(stops[index - 1], stops[index])
        edge = ET.SubElement(edges, 'edge', {'id': f'sb{index}', 'from': f'n{index-1}', 'to': f'n{index}',
            'numLanes': str(n), 'speed': f'{main_speed}', 'priority': '3', 'width': f'{v1.LANE_WIDTH}',
            'spreadType': 'center', 'shape': v1.shape_text(v1.lane_shape(reference, stations, centre))})
        for sumo_index in range(n):                   # SUMO index 0 is the right lane
            shape = v1.lane_shape(reference, stations, sumo_index * v1.LANE_WIDTH)
            shapes[f'sb{index}_{sumo_index}'] = shape
            ET.SubElement(edge, 'lane', index=str(sumo_index), shape=v1.shape_text(shape), width=f'{v1.LANE_WIDTH}')
    merge_point = v1.lane_shape(reference, np.array([gore]), 0.)[0]
    ramp = model_ramp(lanes, site, merge_point)
    ET.SubElement(nodes, 'node', id='ramp_start', x=f'{ramp[0][0]:.3f}', y=f'{ramp[0][1]:.3f}',
                  z=f'{ramp[0][2]:.3f}', type='dead_end')
    ramp_edge = ET.SubElement(edges, 'edge', {'id': 'mlk_on', 'from': 'ramp_start', 'to': 'n1', 'numLanes': '1',
        'speed': f'{ramp_speed}', 'priority': '2', 'width': f'{v1.LANE_WIDTH}', 'spreadType': 'center',
        'shape': v1.shape_text(ramp)})
    ET.SubElement(ramp_edge, 'lane', index='0', shape=v1.shape_text(ramp), width=f'{v1.LANE_WIDTH}')
    shapes['mlk_on_0'] = ramp
    for index in range(1, len(stops) - 1):
        for lane in range(n):
            ET.SubElement(connections, 'connection', {'from': f'sb{index}', 'to': f'sb{index + 1}',
                                                      'fromLane': str(lane), 'toLane': str(lane)})
    ET.SubElement(connections, 'connection', {'from': 'mlk_on', 'to': 'sb2', 'fromLane': '0', 'toLane': '0',
                                              'visibility': f'{DECLARED_VISIBILITY_M}'})
    ramp_length = float(v1.arclength(ramp)[-1])
    layout = dict(stations_m=[float(s - start) for s in stops], upstream_m=site['upstream_m'],
                  auxiliary_length_m=0., corridor_length_m=float(total - start), downstream_m=float(downstream),
                  ramp_length_m=ramp_length, mainline_speed_m_s=main_speed, ramp_speed_m_s=ramp_speed,
                  declared_visibility_m=DECLARED_VISIBILITY_M)
    return nodes, edges, connections, layout, shapes, reference, ramp


def site_geometry(lanes, site):
    """v1's reference line and merge station, without v1's auxiliary-length requirement."""
    n = site['through_lanes']
    right = n - 1
    ways = (*site['upstream_ways'], *site['downstream_ways'])
    for way in ways:
        if v1.lane_of(lanes, way, 0)['count'] != n:
            raise ValueError(f'Way {way} does not have {n} lanes')
    for a, b in zip(ways, ways[1:]):
        if f'{b}_{right}' not in v1.lane_of(lanes, a, right)['succ']:
            raise ValueError(f'Way {b} does not continue way {a}')
    reference = v1.join([v1.lane_of(lanes, way, right)['pts'] for way in ways])
    upstream = v1.join([v1.lane_of(lanes, way, right)['pts'] for way in site['upstream_ways']])
    gore, total = v1.arclength(upstream)[-1], v1.arclength(reference)[-1]
    if gore < site['upstream_m'] or total - gore < 4 * 21.:
        raise ValueError('Site is too short for the declared upstream length and four downstream edges')
    ramp_last = v1.lane_of(lanes, *site['ramp_lanes'][-1].rsplit('_', 1))
    if f"{site['merge_way']}_{right}" not in ramp_last['succ']:
        raise ValueError('Ramp does not join the right through lane of the merge way')
    return reference, gore, total


def validate(network_path, lanes, ident, ramp_model):
    """Compiled checks: layout, the ramp's yield link, and fidelity to the model's lanes."""
    site = SITES[ident]
    n = site['through_lanes']
    root = ET.parse(network_path).getroot()
    edges = {e.get('id'): e for e in root.findall('edge') if e.get('function') != 'internal'}
    count = DOWNSTREAM_EDGES + 1
    expected = {**{f'sb{i}': n for i in range(1, count + 1)}, 'mlk_on': 1}
    counts = {k: len(e.findall('lane')) for k, e in edges.items()}
    plain = [c for c in root.findall('connection') if not c.get('from').startswith(':')]
    links = sorted((c.get('from'), int(c.get('fromLane')), c.get('to'), int(c.get('toLane'))) for c in plain)
    wanted = sorted([(f'sb{i}', lane, f'sb{i + 1}', lane) for i in range(1, count) for lane in range(n)]
                    + [('mlk_on', 0, 'sb2', 0)])
    ramp = next(c for c in plain if c.get('from') == 'mlk_on')
    right = next(c for c in plain if c.get('from') == 'sb1' and c.get('fromLane') == '0')
    main_speed, ramp_speed = model_speeds(lanes, site)
    speeds = {lane.get('id'): float(lane.get('speed')) for e in edges.values() for lane in e.findall('lane')}
    speed_ok = all(abs(v - (ramp_speed if k == 'mlk_on_0' else main_speed)) < 1e-6 for k, v in speeds.items())
    right_index = n - 1
    model_lines = {i: v1.join([v1.lane_of(lanes, way, right_index - i)['pts']
                               for way in (*site['upstream_ways'], *site['downstream_ways'])]).tolist()
                   for i in range(n)}
    lane_elements = {lane.get('id'): lane for lane in root.findall('./edge/lane')}
    deviation = 0.
    for i in range(1, count + 1):
        for lane in edges[f'sb{i}'].findall('lane'):
            for point in v1._shape(lane)[1:-1]:
                deviation = max(deviation, v1.point_to_polyline(point, model_lines[int(lane.get('index'))]))
    ramp_deviation = max((v1.point_to_polyline(p, ramp_model.tolist())
                          for p in v1._shape(lane_elements['mlk_on_0'])[1:-1]), default=0.)
    checks = dict(lane_counts=counts == expected, connections=links == wanted,
                  ramp_yields=ramp.get('state') == 'm', right_lane_priority=right.get('state') == 'M',
                  ramp_visibility_m=float(ramp.get('visibility', 'nan')), model_speeds=speed_ok,
                  max_through_lane_deviation_m=deviation, max_ramp_deviation_m=ramp_deviation)
    checks['passed'] = bool(checks['lane_counts'] and checks['connections'] and checks['ramp_yields']
                            and checks['right_lane_priority'] and checks['model_speeds']
                            and abs(checks['ramp_visibility_m'] - DECLARED_VISIBILITY_M) < 1e-6
                            and deviation <= v1.MAX_LANE_DEVIATION_M and ramp_deviation <= MAX_RAMP_DEVIATION_M)
    return checks


def build(output, ident, lanegraph=None):
    output = Path(output).resolve()
    if not output.is_relative_to(ROOT / 'outputs') or output.exists():
        raise ValueError('Use a fresh project output directory')
    lanes, graph = v1.load_lanes(lanegraph)
    nodes, edges, connections, layout, _, _, ramp = plain_xml(lanes, ident)
    output.mkdir(parents=True)
    for name, data in [('nodes.nod.xml', nodes), ('edges.edg.xml', edges), ('connections.con.xml', connections)]:
        v1.write_xml(output / name, data)
    from traffic.sumo_backend import find_sumo_binary
    binary = find_sumo_binary().with_name('netconvert.exe' if os.name == 'nt' else 'netconvert')
    arguments = ['--node-files', 'nodes.nod.xml', '--edge-files', 'edges.edg.xml',
                 '--connection-files', 'connections.con.xml', '--output-file', 'network.raw.net.xml',
                 '--no-turnarounds', 'true', '--offset.disable-normalization', 'true',
                 '--check-lane-foes.all', 'true']
    result = subprocess.run([str(binary), *arguments], cwd=output, capture_output=True, text=True, timeout=60)
    (output / 'netconvert.log').write_text(result.stdout + result.stderr, encoding='utf-8')
    result.check_returncode()
    tree = ET.parse(output / 'network.raw.net.xml')       # drops netconvert's timestamp comments
    ET.indent(tree, space='    ')
    tree.write(output / 'network.net.xml', encoding='utf-8', xml_declaration=True)
    checks = validate(output / 'network.net.xml', lanes, ident, ramp)
    if not checks['passed']:
        (output / 'failed-geometry.json').write_text(json.dumps(checks, indent=2) + '\n')
        raise ValueError(f'Compiled corridor {ident} failed its checks: {checks}')
    site = SITES[ident]
    version = subprocess.run([str(binary), '--version'], capture_output=True, text=True, timeout=15, check=True).stdout
    manifest = dict(geometry_version=GEOMETRY_VERSION, id=ident, direction=site['direction'],
        through_lanes=site['through_lanes'], network=(output / 'network.net.xml').relative_to(ROOT).as_posix(),
        network_sha256=v1.sha256(output / 'network.net.xml'), layout=layout, compiled_checks=checks,
        model=v1.MODEL, lanegraph=v1.LANEGRAPH, lanegraph_sha256=v1.sha256(ROOT / (lanegraph or v1.LANEGRAPH)),
        model_origin=graph['origin'], model_frame=graph['frame'],
        lane_graph_ways=dict(upstream=list(site['upstream_ways']), downstream=list(site['downstream_ways']),
                             ramp_lanes=list(site['ramp_lanes']), merge_way=site['merge_way']),
        from_model=['through-lane centerlines, count and 3.66 m width', 'deck elevation along those lanes',
                    'the ramp lane centerline, unchanged, and where it joins the right lane',
                    'lane speed limits (lanegraph speed, m/s)'],
        not_in_model=[f'merge is a SUMO yield junction at the model merge point, declared visibility '
                      f'{DECLARED_VISIBILITY_M:g} m (as the existing I-110 corridor)',
                      'downstream road split into four equal edges for the environment edge names',
                      'vehicles are capped at 26.8224 m/s (60 mph) by the environments, below the 65 mph mainline'],
        note=site['note'], input_sha256={k: v1.sha256(output / k) for k in ('nodes.nod.xml', 'edges.edg.xml',
                                                                              'connections.con.xml')},
        source_sha256={p: v1.sha256(ROOT / p) for p in ('scripts/build_la_10_110_corridors_exact.py',
                                                         'scripts/build_la_10_110_corridors.py')},
        netconvert_version=version.strip(), command=['netconvert', *arguments],
        attribution='Road geometry (c) OpenStreetMap contributors, ODbL, via the LA I-10/I-110 OpenUSD model')
    (output / 'corridor-manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n',
                                                    encoding='utf-8')
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'outputs/highway-flow-rl/v12/la-corridors-exact-01')
    args = parser.parse_args(argv)
    output = args.output.resolve()
    if not output.is_relative_to(ROOT / 'outputs') or output.exists():
        raise ValueError('Corridor output must be a fresh project outputs directory')
    output.mkdir(parents=True)
    entries = [build(output / ident, ident) for ident in SITES]
    bank = dict(geometry_version=GEOMETRY_VERSION, corridors=[{k: e[k] for k in
        ('id', 'direction', 'through_lanes', 'network', 'network_sha256', 'layout')} for e in entries],
        model=v1.MODEL, traffic_run=False, learning_performed=False)
    (output / 'bank.json').write_text(json.dumps(bank, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps([dict(id=e['id'], sha256=e['network_sha256'], **{k: round(e['layout'][k], 1) for k in
                     ('corridor_length_m', 'downstream_m', 'ramp_length_m', 'mainline_speed_m_s', 'ramp_speed_m_s')},
                     checks={k: (round(v, 3) if isinstance(v, float) else v)
                             for k, v in e['compiled_checks'].items()}) for e in entries], indent=1))


if __name__ == '__main__':
    main()
