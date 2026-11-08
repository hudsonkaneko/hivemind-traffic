"""Build a portable, provisional SUMO lane graph from the I-110 road profiles.

This is an engineering network, not an empirically admitted traffic model.
Only plain XML is authored; netconvert owns the compiled network.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np

from traffic.sumo_backend import find_sumo_binary

ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / 'scenarios/i110/sumo-road-profiles.json'
STATIONS = (0., 55., 280., 455., 650., 750., 815.)
COUNTS = (4, 5, 5, 5, 4, 4)
CAP = 26.8224


def write_xml(path, root):
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def shape_text(points):
    return ' '.join(','.join(f'{v:.6f}' for v in point) for point in points)


def sample(profile, stations, offset):
    s = profile['station_m']
    center = np.asarray(profile['center_xyz'])
    normal = np.asarray(profile['normal_xy'])
    xyz = np.column_stack([np.interp(stations, s, center[:, j]) for j in range(3)])
    n = np.column_stack([np.interp(stations, s, normal[:, j]) for j in range(2)])
    xyz[:, :2] += n * np.asarray(offset)[:, None]
    return xyz


def main_shape(profile, start, end, lane, count):
    stations = np.linspace(start, end, max(2, int(np.ceil((end-start)/3))+1))
    width = np.interp(stations, profile['station_m'], profile['widths_m'])
    # SUMO lane zero is rightmost; positive profile normal points east/left.
    offset = width/2 - 1.6 - 3.7 * (count - lane - .5)
    return sample(profile, stations, offset)


def build(output, profile_path=PROFILE, netconvert=None):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    data = json.loads(Path(profile_path).read_text())
    main = data['roads']['Southbound']
    nodes, edges, connections = ET.Element('nodes'), ET.Element('edges'), ET.Element('connections')
    for i, station in enumerate(STATIONS):
        point = sample(main, [station], [0])[0]
        ET.SubElement(nodes, 'node', id=f'n{i}', x=str(point[0]), y=str(point[1]),
                      z=str(point[2]), type='priority', radius='0')
    lane_shapes = {}
    for i, (a, b, count) in enumerate(zip(STATIONS, STATIONS[1:], COUNTS)):
        edge = ET.SubElement(edges, 'edge', {'id':f'sb{i}', 'from':f'n{i}', 'to':f'n{i+1}',
            'numLanes':str(count), 'speed':str(CAP), 'priority':'3', 'spreadType':'center'})
        for lane in range(count):
            xyz = main_shape(main, a, b, lane, count)
            ET.SubElement(edge, 'lane', index=str(lane), width='3.7', shape=shape_text(xyz))
            lane_shapes[f'sb{i}_{lane}'] = xyz.tolist()
    for i, (before, after) in enumerate(zip(COUNTS, COUNTS[1:])):
        for lane in range(before):
            if after < before and lane == 0:
                # The auxiliary lane ends. Drivers must merge while lanes are
                # adjacent; a 0->0 connection would teleport them sideways.
                continue
            target = lane + 1 if after > before else max(0, lane-1) if after < before else lane
            ET.SubElement(connections, 'connection', **{'from':f'sb{i}', 'to':f'sb{i+1}',
                'fromLane':str(lane), 'toLane':str(target)})
    # Endpoints are adapted to the lane graph over 30 m. These are explicitly
    # provisional ports, not additional imagery measurements or arterial roads.
    for name, edge_id, node_id, join, is_entry in (
        ('MlkEntrance', 'mlk_on', 'mlk_boundary', 2, True),
        ('VernonExit', 'vernon_off', 'vernon_boundary', 3, False)):
        profile = data['roads'][name]
        center = np.asarray(profile['center_xyz'])
        stations = np.asarray(profile['station_m'])
        xyz = sample(profile, stations, np.full(len(stations), .75))
        if is_entry:
            # Trim image trace at the selected graph merge port, y around -280.
            xyz = xyz[xyz[:, 1] >= sample(main, [STATIONS[join]], [0])[0, 1]]
            target = np.asarray(lane_shapes[f'sb{join}_0'][0])
            distance = np.r_[0, np.cumsum(np.linalg.norm(np.diff(xyz[:, :2], axis=0), axis=1))]
            weight = np.clip((distance-distance[-1]+30)/30, 0, 1)
            xyz += weight[:, None]*(target-xyz[-1])
            boundary, origin, destination = xyz[0], node_id, f'n{join}'
            conn = {'from':edge_id, 'to':f'sb{join}', 'fromLane':'0', 'toLane':'0'}
        else:
            target = np.asarray(lane_shapes[f'sb{join-1}_0'][-1])
            distance = np.r_[0, np.cumsum(np.linalg.norm(np.diff(xyz[:, :2], axis=0), axis=1))]
            weight = np.clip(1-distance/30, 0, 1)
            xyz += weight[:, None]*(target-xyz[0])
            boundary, origin, destination = xyz[-1], f'n{join}', node_id
            conn = {'from':f'sb{join-1}', 'to':edge_id, 'fromLane':'0', 'toLane':'0'}
        ET.SubElement(nodes, 'node', id=node_id, x=str(boundary[0]), y=str(boundary[1]),
                      z=str(boundary[2]), type='priority', radius='0')
        edge = ET.SubElement(edges, 'edge', {'id':edge_id, 'from':origin, 'to':destination,
            'numLanes':'1', 'speed':'13.4112', 'priority':'2'})
        ET.SubElement(edge, 'lane', index='0', width='3.7', shape=shape_text(xyz))
        ET.SubElement(connections, 'connection', **conn)
    # Automatic junction polygons can extrapolate independently authored lane
    # shapes backwards at a shallow freeway diverge. Bound each virtual seam
    # by a 2 m longitudinal band and the actual participating lane widths.
    # This is a traffic graph connector, not a measured paved intersection.
    for node in nodes:
        if node.get('id') not in {f'n{i}' for i in range(1,len(STATIONS)-1)}:
            continue
        endpoints=[]
        for edge in edges:
            if edge.get('from')==node.get('id') or edge.get('to')==node.get('id'):
                index=0 if edge.get('from')==node.get('id') else -1
                for lane in edge:
                    endpoints.append(float(lane.get('shape').split()[index].split(',')[0]))
        low,high=min(endpoints)-1.85,max(endpoints)+1.85
        y=float(node.get('y'))
        node.set('shape',f'{low},{y+1} {high},{y+1} {high},{y-1} {low},{y-1}')
    for name, root in (('nodes.nod.xml', nodes), ('edges.edg.xml', edges), ('connections.con.xml', connections)):
        write_xml(output/name, root)
    binary = Path(netconvert) if netconvert else find_sumo_binary().with_name('netconvert.exe' if __import__('os').name == 'nt' else 'netconvert')
    command = [str(binary), '--node-files', str(output/'nodes.nod.xml'), '--edge-files', str(output/'edges.edg.xml'),
        '--connection-files', str(output/'connections.con.xml'), '--output-file', str(output/'network.net.xml'),
        '--offset.disable-normalization', 'true', '--no-turnarounds', 'true', '--junctions.corner-detail', '0']
    result = subprocess.run(command, capture_output=True, text=True, timeout=60)
    (output/'netconvert.log').write_text(result.stdout+result.stderr)
    result.check_returncode()
    compiled=ET.parse(output/'network.net.xml')
    for lane in compiled.findall('./edge/lane'):
        points=np.array([[float(v) for v in xyz.split(',')] for xyz in lane.get('shape').split()])
        if np.any(np.diff(points[:,1])>.02):
            raise ValueError('Compiled southbound lane reverses northward: '+lane.get('id'))
    manifest = dict(schema_version=1, status='provisional_engineering_network',
        source_profile_sha256=hashlib.sha256(Path(profile_path).read_bytes()).hexdigest(),
        source_geometry_sha256=data['source_geometry_sha256'], coordinates=data['coordinates'],
        sumo_to_usd='Identity local XYZ in metres; no normalization or rotation',
        stations_m=STATIONS, lane_counts=COUNTS, mainline_speed_cap_m_s=CAP,
        command=command, netconvert_version=subprocess.check_output([str(binary),'--version'],text=True).splitlines()[0],
        limitations=['Mainline has four through lanes and a provisional auxiliary lane.',
          'Auxiliary graph drop at station 650 m lies within interpreted 590–750 m taper; exact stripe endpoint unverified.',
          'SUMO constant lane widths/junction connectors approximate the continuous pavement taper.',
          'Virtual junction seams use 2 m longitudinal bands to prevent backwards automatic connectors.',
          'Ramp ports blended over 30 m; no ramp-meter or arterial signal inferred.',
          'Upstream/downstream are open crop boundaries; no external network or measured demand.',
          'No PeMS calibration, empirical site admission, or physical dynamics claim.'])
    manifest['files'] = {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in output.iterdir() if p.is_file()}
    (output/'network-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profile', type=Path, default=PROFILE)
    parser.add_argument('--netconvert', type=Path)
    args = parser.parse_args()
    build(args.output, args.profile, args.netconvert)
