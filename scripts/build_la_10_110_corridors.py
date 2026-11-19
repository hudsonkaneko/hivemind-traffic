"""Build SUMO merge corridors from the LA I-10 / I-110 OpenUSD model's lane graph.

The model (highway_usd/la_10_110, generated from OpenStreetMap, ODbL) is the
primary road model from v12 on. Training needs SUMO networks in the corridor
layout the highway environments already use (traffic/highway_v05_demand.py):

    sb1 (N through lanes) -> n1 -> sb2, sb3 (N + 1; lane 0 is an auxiliary lane)
    -> n3 (the auxiliary lane ends) -> sb4, sb5 (N lanes)
    mlk_on (one ramp lane) -> n1 -> sb2 lane 0

This builder cuts that layout out of the model at its two real single-lane
CA-110 on-ramps. Real: the through-lane centerlines (the model's own ENU frame,
metres, +X east, +Y north, so SUMO coordinates are model coordinates), their
3.66 m spacing, the deck elevation and the ramp alignment. Modeled: the
auxiliary (acceleration) lane. The lane graph joins each ramp straight into the
right lane with no acceleration lane, while the merge task needs one, so a lane
of `auxiliary_length_m` is added beside the right lane from the ramp junction
and the last 80 m of the ramp are blended onto its start. Speeds are the
existing I-110 corridor's (mainline 60 mph, ramp 30 mph); every row still sets
its own speed limit. Only plain XML is authored; netconvert owns the network.
No traffic is run and no policy is evaluated here.
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

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
MODEL = 'highway_usd/la_10_110'
LANEGRAPH = f'{MODEL}/lanegraph.json'
GEOMETRY_VERSION = 'la_10_110_corridor_v1'
LANE_WIDTH = 3.66              # the model's lane width (Caltrans standard)
MAIN_SPEED, RAMP_SPEED = 26.8224, 13.4112
STEP_M = 2.
RAMP_BLEND_M = 80.
MAX_LANE_DEVIATION_M = .25     # compiled through lanes against the model's lane centerlines

# Two real single-lane on-ramps onto three-lane CA-110 (lane graph ways; OSM way ids).
SITES = {
    'la-110-nb-pasadena-onramp': dict(
        direction='CA-110 northbound', through_lanes=3, merge_way=11340357201,
        upstream_ways=(148249774, 113403576, 148249776, 113403568, 229633967, 11340357200),
        downstream_ways=(11340357201,),
        ramp_lanes=('1020818512_0', '148356751_0', '113403565_0', '113403564_0'),
        upstream_m=450., auxiliary_length_m=250.,
        note='ends before the two-lane connector that widens CA-110 to five lanes'),
    'la-110-sb-adams-onramp': dict(
        direction='CA-110 southbound', through_lanes=3, merge_way=113403573,
        upstream_ways=(148252682, 148252661, 148252691, 113403574, 229633966, 113403570),
        downstream_ways=(113403573, 113403566),
        ramp_lanes=('526743920_0', '123561870_0'),
        upstream_m=450., auxiliary_length_m=250.,
        note='ends at the model edge, before the synthetic SE simulation loop'),
}


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_lanes(path=None):
    graph = json.loads((ROOT / (path or LANEGRAPH)).read_text(encoding='utf-8'))
    return {lane['id']: lane for lane in graph['lanes']}, graph


def join(polylines):
    """Concatenate polylines, dropping a repeated joint point."""
    points = []
    for line in polylines:
        for point in line:
            if points and math.dist(points[-1][:2], point[:2]) < 1e-3:
                continue
            points.append([float(v) for v in point[:3]])
    return np.asarray(points)


def arclength(points):
    return np.concatenate([[0.], np.cumsum(np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1))])


def resample(points, stations):
    s = arclength(points)
    return np.column_stack([np.interp(stations, s, points[:, j]) for j in range(3)])


def left_normals(points):
    tangent = np.gradient(points[:, :2], axis=0)
    tangent /= np.linalg.norm(tangent, axis=1, keepdims=True)
    return np.column_stack([-tangent[:, 1], tangent[:, 0]])


def lane_of(lanes, way, index):
    lane = lanes.get(f'{way}_{index}')
    if lane is None:
        raise ValueError(f'Lane {way}_{index} is not in the lane graph')
    return lane


def site_geometry(lanes, site):
    """Reference line (the right through lane), its stations, and the gore station."""
    n = site['through_lanes']
    right = n - 1                                   # lane graph index 0 is the leftmost lane
    ways = (*site['upstream_ways'], *site['downstream_ways'])
    for way in ways:
        if lane_of(lanes, way, 0)['count'] != n:
            raise ValueError(f'Way {way} does not have {n} lanes')
    for a, b in zip(ways, ways[1:]):
        if f'{b}_{right}' not in lane_of(lanes, a, right)['succ']:
            raise ValueError(f'Way {b} does not continue way {a}')
    reference = join([lane_of(lanes, way, right)['pts'] for way in ways])
    upstream = join([lane_of(lanes, way, right)['pts'] for way in site['upstream_ways']])
    gore = arclength(upstream)[-1]
    total = arclength(reference)[-1]
    if gore < site['upstream_m'] or total - gore < site['auxiliary_length_m'] + 80.:
        raise ValueError('Site is too short for the declared upstream and auxiliary lengths')
    ramp_last = lane_of(lanes, *site['ramp_lanes'][-1].rsplit('_', 1))
    if f"{site['merge_way']}_{right}" not in ramp_last['succ']:
        raise ValueError('Ramp does not join the right through lane of the merge way')
    return reference, gore, total


def lane_shape(reference, stations, offset):
    """Points `offset` metres left of the reference line (negative: right)."""
    xyz = resample(reference, stations)
    dense = resample(reference, np.arange(0., arclength(reference)[-1] + 1e-9, STEP_M))
    normal_dense = left_normals(dense)
    s_dense = arclength(dense)
    normal = np.column_stack([np.interp(stations, s_dense, normal_dense[:, j]) for j in range(2)])
    normal /= np.linalg.norm(normal, axis=1, keepdims=True)
    xyz[:, :2] += normal * offset
    return xyz


def stations_between(start, end):
    count = max(2, int(math.ceil((end - start) / STEP_M)) + 1)
    return np.linspace(start, end, count)


def shape_text(points):
    return ' '.join(','.join(f'{v:.3f}' for v in point) for point in points)


def ramp_shape(lanes, site, aux_start):
    ramp = join([lane_of(lanes, *lane.rsplit('_', 1))['pts'] for lane in site['ramp_lanes']])
    s = arclength(ramp)
    end = ramp[-1].copy()
    shift = aux_start - end
    weight = np.clip(1. - (s[-1] - s) / RAMP_BLEND_M, 0., 1.)
    weight = weight * weight * (3. - 2. * weight)   # smoothstep; zero before the blend zone
    blended = ramp + weight[:, None] * shift[None, :]
    keep = np.concatenate([[0.], np.cumsum(np.linalg.norm(np.diff(blended[:, :2], axis=0), axis=1))])
    blended = resample(blended, np.linspace(0., keep[-1], max(2, int(keep[-1] / STEP_M) + 1)))
    return blended, dict(ramp_length_m=float(keep[-1]), blend_m=RAMP_BLEND_M,
                         end_shift_m=float(np.linalg.norm(shift[:2])), end_shift_z_m=float(shift[2]))


def plain_xml(lanes, ident):
    site = SITES[ident]
    n = site['through_lanes']
    reference, gore, total = site_geometry(lanes, site)
    start, aux = gore - site['upstream_m'], site['auxiliary_length_m']
    stops = [start, gore, gore + aux / 2, gore + aux, gore + aux + (total - gore - aux) / 2, total]
    nodes, edges, connections = ET.Element('nodes'), ET.Element('edges'), ET.Element('connections')
    shapes = {}
    for index, station in enumerate(stops):
        width = n + (1 <= index <= 3)
        offset = (n - 1) / 2 * LANE_WIDTH - (LANE_WIDTH / 2 if 1 <= index <= 3 else 0.)
        point = lane_shape(reference, np.array([station]), offset)[0]
        ET.SubElement(nodes, 'node', id=f'n{index}', x=f'{point[0]:.3f}', y=f'{point[1]:.3f}', z=f'{point[2]:.3f}',
                      type='priority' if 0 < index < 5 else 'dead_end')
    for index in range(1, 6):
        auxiliary = index in (2, 3)
        count = n + auxiliary
        stations = stations_between(stops[index - 1], stops[index])
        edge = ET.SubElement(edges, 'edge', {'id': f'sb{index}', 'from': f'n{index-1}', 'to': f'n{index}',
            'numLanes': str(count), 'speed': f'{MAIN_SPEED}', 'priority': '3', 'width': f'{LANE_WIDTH}',
            'spreadType': 'center'})
        centre_offset = (n - 1) / 2 * LANE_WIDTH - (LANE_WIDTH / 2 if auxiliary else 0.)
        edge.set('shape', shape_text(lane_shape(reference, stations, centre_offset)))
        for sumo_index in range(count):
            through = sumo_index - auxiliary          # -1 is the auxiliary lane, right of the right lane
            shape = lane_shape(reference, stations, through * LANE_WIDTH)
            shapes[f'sb{index}_{sumo_index}'] = shape
            ET.SubElement(edge, 'lane', index=str(sumo_index), shape=shape_text(shape), width=f'{LANE_WIDTH}')
    aux_start = lane_shape(reference, np.array([gore]), -LANE_WIDTH)[0]
    ramp, ramp_info = ramp_shape(lanes, site, aux_start)
    ET.SubElement(nodes, 'node', id='ramp_start', x=f'{ramp[0][0]:.3f}', y=f'{ramp[0][1]:.3f}',
                  z=f'{ramp[0][2]:.3f}', type='dead_end')
    ramp_edge = ET.SubElement(edges, 'edge', {'id': 'mlk_on', 'from': 'ramp_start', 'to': 'n1', 'numLanes': '1',
        'speed': f'{RAMP_SPEED}', 'priority': '2', 'width': f'{LANE_WIDTH}', 'spreadType': 'center',
        'shape': shape_text(ramp)})
    ET.SubElement(ramp_edge, 'lane', index='0', shape=shape_text(ramp), width=f'{LANE_WIDTH}')
    shapes['mlk_on_0'] = ramp

    def connect(a, b, x, y):
        ET.SubElement(connections, 'connection', {'from': a, 'to': b, 'fromLane': str(x), 'toLane': str(y)})
    for lane in range(n):
        connect('sb1', 'sb2', lane, lane + 1)
    connect('mlk_on', 'sb2', 0, 0)
    for lane in range(n + 1):
        connect('sb2', 'sb3', lane, lane)
    for lane in range(1, n + 1):
        connect('sb3', 'sb4', lane, lane - 1)
    for lane in range(n):
        connect('sb4', 'sb5', lane, lane)
    layout = dict(stations_m=[float(s - start) for s in stops], upstream_m=site['upstream_m'],
                  auxiliary_length_m=aux, corridor_length_m=float(total - start), **ramp_info)
    return nodes, edges, connections, layout, shapes, reference, (start, total)


def _shape(lane):
    return [tuple(map(float, value.split(',')[:2])) for value in lane.attrib['shape'].split()]


def _segments_cross(a, b, c, d):
    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    return orient(a, b, c) * orient(a, b, d) < -1e-8 and orient(c, d, a) * orient(c, d, b) < -1e-8


def point_to_polyline(point, line):
    best = math.inf
    for a, b in zip(line, line[1:]):
        ab = np.subtract(b[:2], a[:2])
        t = 0. if not ab.any() else float(np.clip(np.dot(np.subtract(point[:2], a[:2]), ab) / ab.dot(ab), 0., 1.))
        best = min(best, float(np.linalg.norm(np.asarray(a[:2]) + t * ab - np.asarray(point[:2]))))
    return best


def validate(network_path, lanes, ident):
    """Compiled-network checks: layout, connections, ramp conflicts, and fidelity to the model's lanes."""
    site = SITES[ident]
    n = site['through_lanes']
    root = ET.parse(network_path).getroot()
    edges = {e.get('id'): e for e in root.findall('edge') if e.get('function') != 'internal'}
    expected = {'sb1': n, 'sb2': n + 1, 'sb3': n + 1, 'sb4': n, 'sb5': n, 'mlk_on': 1}
    counts = {k: len(e.findall('lane')) for k, e in edges.items()}
    links = sorted((c.get('from'), int(c.get('fromLane')), c.get('to'), int(c.get('toLane')))
                   for c in root.findall('connection') if not c.get('from').startswith(':'))
    wanted = sorted([('sb1', i, 'sb2', i + 1) for i in range(n)] + [('mlk_on', 0, 'sb2', 0)]
                    + [('sb2', i, 'sb3', i) for i in range(n + 1)] + [('sb3', i, 'sb4', i - 1) for i in range(1, n + 1)]
                    + [('sb4', i, 'sb5', i) for i in range(n)])
    lane_elements = {lane.get('id'): lane for lane in root.findall('./edge/lane')}
    ramp = next(c for c in root.findall('connection') if c.get('from') == 'mlk_on')
    through = [c for c in root.findall('connection') if c.get('from') == 'sb1' and c.get('to') == 'sb2']
    junction = next(j for j in root.findall('junction') if ramp.get('via') in j.get('intLanes', '').split())
    internal = junction.get('intLanes').split()
    request = next(q for q in junction.findall('request') if int(q.get('index')) == internal.index(ramp.get('via')))
    ramp_internal = _shape(lane_elements[ramp.get('via')])
    crossings, foes = [], []
    for connection in through:
        via = connection.get('via')
        shape = _shape(lane_elements[via])
        if any(_segments_cross(a, b, c, d) for a, b in zip(ramp_internal, ramp_internal[1:]) for c, d in zip(shape, shape[1:])):
            crossings.append(via)
        index = internal.index(via)
        if request.get('foes')[::-1][index] == '1' or request.get('response')[::-1][index] == '1':
            foes.append(via)
    main = _shape(lane_elements['sb1_0'])
    end, before = main[-1], main[-2]
    tx, ty = end[0] - before[0], end[1] - before[1]
    scale = math.hypot(tx, ty)
    ramp_start = _shape(lane_elements['mlk_on_0'])[0]
    approach_right_m = (ramp_start[0] - end[0]) * ty / scale - (ramp_start[1] - end[1]) * tx / scale
    # Fidelity: each compiled through lane against the model lane it was cut from.
    right = n - 1
    deviation = 0.
    model_lines = {}
    for through_index in range(n):
        graph_index = right - through_index
        model_lines[through_index] = join([lane_of(lanes, way, graph_index)['pts']
                                           for way in (*site['upstream_ways'], *site['downstream_ways'])])
    for edge_id in ('sb1', 'sb2', 'sb3', 'sb4', 'sb5'):
        auxiliary = edge_id in ('sb2', 'sb3')
        for lane in edges[edge_id].findall('lane'):
            through_index = int(lane.get('index')) - auxiliary
            if through_index < 0:
                continue
            for point in _shape(lane)[1:-1]:
                deviation = max(deviation, point_to_polyline(point, model_lines[through_index].tolist()))
    checks = dict(lane_counts=counts == expected, connections=links == wanted, ramp_crossings=crossings,
                  ramp_foes=foes, ramp_approach_right_of_main_m=approach_right_m,
                  max_through_lane_deviation_m=deviation)
    checks['passed'] = bool(checks['lane_counts'] and checks['connections'] and not crossings and not foes
                            and approach_right_m > 0 and deviation <= MAX_LANE_DEVIATION_M)
    return checks


def write_xml(path, root):
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def build(output, ident, lanegraph=None):
    output = Path(output).resolve()
    if not output.is_relative_to(ROOT / 'outputs') or output.exists():
        raise ValueError('Use a fresh project output directory')
    lanes, graph = load_lanes(lanegraph)
    nodes, edges, connections, layout, _, _, _ = plain_xml(lanes, ident)
    output.mkdir(parents=True)
    for name, data in [('nodes.nod.xml', nodes), ('edges.edg.xml', edges), ('connections.con.xml', connections)]:
        write_xml(output / name, data)
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
    checks = validate(output / 'network.net.xml', lanes, ident)
    if not checks['passed']:
        (output / 'failed-geometry.json').write_text(json.dumps(checks, indent=2) + '\n')
        raise ValueError(f'Compiled corridor {ident} failed its checks')
    site = SITES[ident]
    version = subprocess.run([str(binary), '--version'], capture_output=True, text=True, timeout=15, check=True).stdout
    manifest = dict(geometry_version=GEOMETRY_VERSION, id=ident, direction=site['direction'],
        through_lanes=site['through_lanes'], network=(output / 'network.net.xml').relative_to(ROOT).as_posix(),
        network_sha256=sha256(output / 'network.net.xml'), layout=layout, compiled_checks=checks,
        model=MODEL, lanegraph=LANEGRAPH, lanegraph_sha256=sha256(ROOT / (lanegraph or LANEGRAPH)),
        model_origin=graph['origin'], model_frame=graph['frame'],
        lane_graph_ways=dict(upstream=list(site['upstream_ways']), downstream=list(site['downstream_ways']),
                             ramp_lanes=list(site['ramp_lanes']), merge_way=site['merge_way']),
        modeled=['auxiliary (acceleration) lane of auxiliary_length_m beside the right lane',
                 f'last {RAMP_BLEND_M:g} m of the ramp blended onto the auxiliary lane start',
                 'speeds: mainline 60 mph, ramp 30 mph (as the earlier I-110 corridor)'],
        real=['through-lane centerlines and spacing', 'deck elevation', 'ramp alignment before the blend'],
        note=site['note'], input_sha256={k: sha256(output / k) for k in ('nodes.nod.xml', 'edges.edg.xml', 'connections.con.xml')},
        source_sha256={'scripts/build_la_10_110_corridors.py': sha256(Path(__file__))},
        netconvert_version=version.strip(), command=['netconvert', *arguments],
        attribution='Road geometry (c) OpenStreetMap contributors, ODbL, via the LA I-10/I-110 OpenUSD model')
    (output / 'corridor-manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'outputs/highway-flow-rl/v12/la-corridors-01')
    args = parser.parse_args(argv)
    output = args.output.resolve()
    if not output.is_relative_to(ROOT / 'outputs') or output.exists():
        raise ValueError('Corridor output must be a fresh project outputs directory')
    output.mkdir(parents=True)
    entries = [build(output / ident, ident) for ident in SITES]
    bank = dict(geometry_version=GEOMETRY_VERSION, corridors=[{k: e[k] for k in
        ('id', 'direction', 'through_lanes', 'network', 'network_sha256', 'layout')} for e in entries],
        model=MODEL, traffic_run=False, learning_performed=False)
    (output / 'bank.json').write_text(json.dumps(bank, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps([dict(id=e['id'], sha256=e['network_sha256'][:12], **{k: round(e['layout'][k], 1) for k in
                     ('corridor_length_m', 'ramp_length_m', 'end_shift_m')},
                     deviation_m=round(e['compiled_checks']['max_through_lane_deviation_m'], 3)) for e in entries]))


if __name__ == '__main__':
    main()
