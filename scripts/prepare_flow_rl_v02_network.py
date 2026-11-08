"""Prepare one visibility-only synthetic I-110 sensitivity candidate; no simulation."""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
ORIGINAL = ROOT / 'outputs/i110-merge-blocking/v01/network-v02/network.net.xml'
ORIGINAL_SHA256 = 'ae829282ad98cfaeeba806fba30e44536361ccabf4d0861708c5638fff512836'
POLICY = ROOT / 'policies/versions/highway-flow-rl-v01/policy.pt'
CONFIG = ROOT / 'policies/versions/highway-flow-rl-v01/training-config.json'
POLICY_SHA256 = '888a380b7c6741aea623e9b99ad155f7cb9a2ef8fdc2e590823b7e2629031819'
VISIBILITY_M = 100.
SEED = 6900019
SCHEMA = 'flow_rl_v02_visibility_screen_v1'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def ramp_connection(root):
    rows = [c for c in root.findall('connection') if c.get('from') == 'mlk_on']
    if len(rows) != 1 or any(rows[0].get(k) != v for k, v in
            dict(to='sb2', fromLane='0', toLane='0', via=':n2_5_0', state='m').items()):
        raise ValueError('Expected the unchanged unique minor ramp-to-auxiliary link')
    return rows[0]


def semantic_tree(element):
    return (element.tag, tuple(sorted(element.attrib.items())),
            (element.text or '').strip(), tuple(semantic_tree(child) for child in element))


def verify_visibility_only(original, candidate):
    before, after = ET.parse(original).getroot(), ET.parse(candidate).getroot()
    a, b = ramp_connection(before), ramp_connection(after)
    original_visibility = float(a.get('visibility', a.get('visibilityDistance', '4.5')))
    if float(b.get('visibility', b.get('visibilityDistance', '4.5'))) != VISIBILITY_M:
        raise ValueError('Candidate visibility must be the declared 100 m')
    for element in (a, b):
        element.attrib.pop('visibility', None)
        element.attrib.pop('visibilityDistance', None)
    if semantic_tree(before) != semantic_tree(after):
        raise ValueError('Candidate changes more than the ramp visibility attribute')
    junction = next(j for j in before.findall('junction') if j.get('id') == 'n2')
    request = next(r for r in junction.findall('request') if r.get('index') == '5')
    if junction.get('type') != 'priority' or request.get('foes') != '000001' or request.get('response') != '000001':
        raise ValueError('Required native priority/foe protection is missing')
    if not any(c.get('from') == 'sb1' and c.get('fromLane') == '0'
               and c.get('to') == 'sb2' and c.get('toLane') == '0' and c.get('state') == 'M'
               for c in before.findall('connection')):
        raise ValueError('Shared priority traffic connection must remain present')
    lanes = {lane.get('id'): lane for lane in before.findall('./edge/lane')}
    approaches = {}
    for identity in ('mlk_on_0', 'sb1_0'):
        lane = lanes[identity]
        points = [tuple(map(float, point.split(','))) for point in lane.get('shape').split()]
        length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        if min(length, float(lane.get('length'))) < VISIBILITY_M:
            raise ValueError('Authored approaches do not support the declared sightline extent')
        approaches[identity] = dict(authored_lane_length_m=float(lane.get('length')),
            measured_polyline_length_m=length, junction_endpoint_xyz=points[-1])
    return dict(visibility_only=True, original_visibility_m=original_visibility,
        candidate_visibility_m=VISIBILITY_M, priority_retained=True, foe_links_retained=True,
        lane_limits_and_geometry_unchanged=True, topology_unchanged=True,
        approaches=approaches, site_sight_distance_verified=False,
        assumption='Synthetic unobstructed sightline over the final 100 m of both authored approaches; no surveyed I-110 visibility claim')


def occupied_fixture(graph, rows):
    from scripts.build_highway_flow_bank import validate_fixture_spawns
    vehicles = deepcopy(list(rows))
    blocker = next(v for v in vehicles if v['vehicle_id'] == 'main_4')
    if blocker['controlled'] or blocker['route'] != 'main':
        raise ValueError('Predeclared conflict car must be a native main-route car')
    original = deepcopy(blocker)
    blocker.update(lane_id='sb1_0', lane=0, edge=0,
                   position=graph.lanes['sb1_0']['length'] - 26., speed=8.)
    checks = validate_fixture_spawns(graph, vehicles)
    return dict(schema='highway_flow_fixture_v1', name='v02-occupied-priority-entry', seed=SEED,
        vehicles=vehicles, events=[], speed_controls=[],
        description='Same 16-car/8-AV cohort; relocate native main_4 to the actual shared priority approach, 26 m before entry at 8 m/s',
        counterfactual_change=dict(vehicle_id='main_4', before=original, after=blocker),
        native_conflict_schedule=dict(vehicle_id='main_4', through_time_s=8., speed_m_s=8.,
            speed_mode=31, lane_change_mode=512, release_speed=-1., release_lane_change_mode=1621,
            rationale='Prevent an early autonomous lane change from erasing the declared conflict; native speed/priority/collision protections remain active'),
        spawn_checks=checks)


def prepare(output):
    from environments.highway_environment_v05 import ContinuousMotionLaneGraph
    from traffic.highway_v05_demand import write_demand
    output = Path(output).resolve()
    if (Path.cwd().resolve() != ROOT or not output.is_relative_to(ROOT/'outputs/highway-flow-rl/v02')
            or not output.name.startswith('network-screen-') or output.exists()):
        raise ValueError('Use checkout root and a fresh v02/network-screen-* output')
    if digest(ORIGINAL) != ORIGINAL_SHA256 or digest(POLICY) != POLICY_SHA256:
        raise ValueError('Frozen network or actor identity changed')
    cfg = json.loads(CONFIG.read_text())
    row = deepcopy(cfg['validation_rows'][3])
    if row['seed'] != SEED or row['case']['network_sha256'] != ORIGINAL_SHA256:
        raise ValueError('Frozen development case changed')
    output.mkdir(parents=True)
    (output/'original').mkdir(); (output/'candidate').mkdir()
    shutil.copy2(ORIGINAL, output/'original/network.net.xml')
    tree = ET.parse(ORIGINAL)
    ramp_connection(tree.getroot()).set('visibility', str(VISIBILITY_M))
    tree.write(output/'candidate/network.net.xml', encoding='utf-8', xml_declaration=True)
    proof = verify_visibility_only(ORIGINAL, output/'candidate/network.net.xml')
    graph = ContinuousMotionLaneGraph(ORIGINAL, 4)
    rows = write_demand(output/'original-demand.rou.xml', SEED, graph, 'merge', 8, 16,
        speed_limit_m_s=18., difficulty=2, traffic_mode='initial', stabilization_seconds=0., horizon_seconds=80.)
    fixture = occupied_fixture(graph, rows)
    save(output/'occupied-fixture.json', fixture)
    save(output/'case.json', row)
    shutil.copy2(CONFIG, output/'frozen-config.json')
    manifest = dict(schema=SCHEMA, training=False, optimizer_steps=0, seed=SEED,
        policy=POLICY.relative_to(ROOT).as_posix(), policy_sha256=digest(POLICY),
        config=CONFIG.relative_to(ROOT).as_posix(), config_sha256=digest(CONFIG),
        original_network=ORIGINAL.relative_to(ROOT).as_posix(), original_network_sha256=digest(ORIGINAL),
        candidate_network=(output/'candidate/network.net.xml').relative_to(ROOT).as_posix(),
        candidate_network_sha256=digest(output/'candidate/network.net.xml'), geometry=proof,
        occupied_fixture_sha256=digest(output/'occupied-fixture.json'),
        protocol=dict(max_episodes=4, total_wall_seconds=180., horizon_seconds=80.,
            order=['clear-original', 'clear-candidate', 'occupied-original', 'occupied-candidate'],
            observation_caveat='Visibility is also actor input57; this is an end-to-end frozen-policy comparison, not pure-physics isolation',
            acceptance='Reduce clear-approach native hard braking; preserve native protection and actual conflicting-traffic coverage; retain all full-route failures'),
        source_sha256={Path(__file__).relative_to(ROOT).as_posix(): digest(__file__)})
    save(output/'manifest.json', manifest)
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    print(json.dumps(prepare(parser.parse_args().output), indent=2))
