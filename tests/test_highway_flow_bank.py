import json
import xml.etree.ElementTree as ET

import pytest

from environments.highway_environment_v05 import ContinuousMotionLaneGraph
from environments.highway_flow_env import HighwayFlowEnv
from scripts.build_highway_flow_bank import (source_geometry, fixture_definition, FIXTURES, SEEDS,
    build, build_network, validate_geometry, validate_fixture_spawns, ADDED_LANE_GEOMETRY_VERSION, old_source_geometry)
from traffic.highway_v05_demand import _safe_gap
from tests.test_highway_environment_v05 import v05_bare


@pytest.fixture
def fixture_graph(tmp_path):
    root = ET.Element('net')
    for edge, count in [('sb1', 4), ('sb2', 4), ('sb3', 4), ('sb4', 3), ('sb5', 3), ('mlk_on', 1)]:
        e = ET.SubElement(root, 'edge', id=edge)
        for index in range(count):
            ET.SubElement(e, 'lane', id=f'{edge}_{index}', index=str(index), length='240' if edge != 'mlk_on' else '170', speed='18')
    for source, target in [('sb1', 'sb2'), ('sb2', 'sb3'), ('sb3', 'sb4'), ('sb4', 'sb5')]:
        for index in range(4 if source != 'sb4' else 3):
            if source == 'sb3' and index == 0:
                continue
            ET.SubElement(root, 'connection', {'from': source, 'to': target, 'fromLane': str(index),
                'toLane': str(index - 1 if source == 'sb3' else index)})
    ET.SubElement(root, 'connection', {'from': 'mlk_on', 'to': 'sb2', 'fromLane': '0', 'toLane': '0'})
    path = tmp_path / 'network.net.xml'
    ET.ElementTree(root).write(path)
    return ContinuousMotionLaneGraph(path, 3)


def test_added_lane_is_not_shared_yield_with_visibility_disabled():
    nodes, edges, connections, placement = source_geometry('added-lane')
    assert next(e for e in edges if e.get('id') == 'sb1').get('numLanes') == '3'
    through = [(c.get('fromLane'), c.get('toLane')) for c in connections if c.get('from') == 'sb1']
    assert through == [('0', '1'), ('1', '2'), ('2', '3')]
    assert placement['site_accuracy_claim'] is False
    ramp = next(c for c in connections if c.get('from') == 'mlk_on')
    assert ramp.get('toLane') == '0' and ramp.get('pass') is None
    assert ramp.get('speed') is None and ramp.get('shape') is None
    approach = next(e for e in edges if e.get('id') == 'mlk_on')
    assert approach.get('spreadType') == 'center'
    points = [tuple(map(float, p.split(','))) for p in approach.get('shape').split()]
    assert points == [(-11.2, -80.), (-11.2, -240.)]
    start = next(n for n in nodes if n.get('id') == 'ramp_start')
    assert float(start.get('x')) == -11.2
    assert placement['geometry_version'] == ADDED_LANE_GEOMETRY_VERSION


@pytest.mark.parametrize('variant,visibility', [('shared-open', '100.0'), ('shared-default', '4.5')])
def test_shared_variants_preserve_real_conflicting_successor(variant, visibility):
    nodes, edges, connections, placement = source_geometry(variant)
    ramp = next(c for c in connections if c.get('from') == 'mlk_on')
    assert ramp.get('visibility') == visibility
    assert any(c.get('from') == 'sb1' and c.get('fromLane') == '0' and c.get('toLane') == '0' for c in connections)
    assert not any(c.get('pass') == 'true' for c in connections)
    assert not placement['site_accuracy_claim']
    original_nodes, original_edges, _, _ = old_source_geometry(lanes=3, length=1200., bend=0.,
        speed_mph=18. / .44704, ramp_upstream_distance=160.)
    next(e for e in original_edges if e.get('id') == 'mlk_on').set('speed', '18.0')
    assert ET.tostring(nodes) == ET.tostring(original_nodes)
    assert ET.tostring(edges) == ET.tostring(original_edges)


def compiled_added_lane(tmp_path):
    net = ET.Element('net')
    for edge, index, shape in [('mlk_on', 0, '-11.2,-80 -11.2,-239'),
            (':entry', 3, '-11.2,-239 -11.2,-240'), ('sb2', 0, '-11.2,-240 -11.2,-480')]:
        element = ET.SubElement(net, 'edge', id=edge)
        ET.SubElement(element, 'lane', id=f'{edge}_{index}', index=str(index), speed='18', length='160', shape=shape)
    junction = ET.SubElement(net, 'junction', id='entry', intLanes=':entry_0 :entry_1 :entry_2 :entry_3')
    for index in range(4):
        ET.SubElement(junction, 'request', index=str(index), foes='0000', response='0000')
    ET.SubElement(net, 'connection', {'from':'mlk_on','to':'sb2','fromLane':'0','toLane':'0','via':':entry_3','state':'M'})
    internal = next(e for e in net if e.get('id') == ':entry')
    for index in range(3):
        x = -8. + index * 3.2
        ET.SubElement(internal, 'lane', id=f':entry_{index}', index=str(index), speed='18', length='1', shape=f'{x},-239 {x},-240')
        ET.SubElement(net, 'connection', {'from':'sb1','to':'sb2','fromLane':str(index),'toLane':str(index+1),'via':f':entry_{index}','state':'M'})
    path = tmp_path/'compiled.xml'
    ET.ElementTree(net).write(path)
    return path


def test_compiled_added_lane_requires_native_speed_and_tangent_geometry(tmp_path):
    path = compiled_added_lane(tmp_path)
    proof = validate_geometry(path, 'added-lane')
    assert proof['tangent_entry']['maximum_heading_change_degrees'] == 0.
    assert proof['tangent_entry']['maximum_endpoint_gap_m'] == 0.
    assert proof['compiled_connector_speed_m_s'] == 18.
    assert not proof['tangent_entry']['speed_override_applied']
    tree = ET.parse(path)
    tree.find("./edge/lane[@id=':entry_3']").set('speed','15.29')
    tree.write(path)
    with pytest.raises(ValueError, match='natively support18'):
        validate_geometry(path, 'added-lane')


def test_compiled_added_lane_rejects_kink_even_if_speed_artificially_high(tmp_path):
    path = compiled_added_lane(tmp_path)
    tree = ET.parse(path)
    tree.find("./edge/lane[@id='mlk_on_0']").set('shape', '-57.8,-80 -11.2,-239')
    tree.write(path)
    with pytest.raises(ValueError, match='geometrically tangent'):
        validate_geometry(path, 'added-lane')


@pytest.mark.parametrize('name', FIXTURES)
def test_all_fixed_seeds_have_explicit_feasible_initial_cohorts(fixture_graph, name):
    for seed in SEEDS:
        fixture = fixture_definition(name, seed, fixture_graph)
        assert fixture == fixture_definition(name, seed, fixture_graph)
        assert len(fixture['vehicles']) == 8
        assert [r['vehicle_id'] for r in fixture['vehicles'] if r['controlled']] == ['agent_0']
        assert all(r['initial_cohort'] and r['depart'] == 0. for r in fixture['vehicles'])
        ego = fixture['vehicles'][0]
        route = ('mlk_on', 'sb2', 'sb3', 'sb4', 'sb5') if ego['route'] == 'ramp' else ('sb1', 'sb2', 'sb3', 'sb4', 'sb5')
        distances = fixture_graph.distances(ego['lane_id'], ego['position'], route)
        for front in fixture['vehicles'][1:]:
            if front['lane_id'] not in distances:
                continue
            distance = distances[front['lane_id']] + front['position']
            if distance >= 0.:
                assert distance - 4.8 >= _safe_gap(ego['speed'], front['speed'])


def test_named_events_and_conflicts_are_real_declared_inputs(fixture_graph):
    cut = fixture_definition('cutin', SEEDS[0], fixture_graph)
    far = fixture_definition('farther-slow-leader', SEEDS[0], fixture_graph)
    dense = fixture_definition('dense-blocked', SEEDS[0], fixture_graph)
    assert cut['events'][0]['vehicle_id'] == 'fixture_cutin'
    assert far['events'][0]['vehicle_id'] == 'agent_0'
    assert 'external' in far['events'][0]['purpose'].lower()
    assert {r['vehicle_id'] for r in dense['vehicles']} >= {'fixture_priority', 'fixture_ramp_conflict'}
    assert dense['safety_exception']


def test_rear_spawn_has_native_margin_then_keeps_closing_speed_request(fixture_graph):
    fixture = fixture_definition('rear', SEEDS[0], fixture_graph)
    rear = next(row for row in fixture['vehicles'] if row['vehicle_id'] == 'fixture_rear')
    assert rear['speed'] == 16.
    assert fixture['speed_controls'] == [dict(vehicle_id='fixture_rear', speed_m_s=18.)]
    assert fixture['static_spawn_checks']['passed']
    rear['speed'] = 18.
    with pytest.raises(ValueError, match='insertion requires speed margin'):
        validate_fixture_spawns(fixture_graph, fixture['vehicles'])


def test_spawn_rejects_unreachable_upcoming_map_limit(fixture_graph):
    fixture = fixture_definition('rear', SEEDS[0], fixture_graph)
    fixture_graph.speed_limits['sb2_1'] = 4.
    with pytest.raises(ValueError, match='cannot brake for upcoming map limit'):
        validate_fixture_spawns(fixture_graph, fixture['vehicles'])


def test_spawn_rejects_insufficient_native_following_gap(fixture_graph):
    fixture = fixture_definition('ending-lane-temptation', SEEDS[0], fixture_graph)
    leader = next(row for row in fixture['vehicles'] if row['vehicle_id'] == 'fixture_leader')
    leader['position'] = fixture['vehicles'][0]['position'] + 20.
    with pytest.raises(ValueError, match='initial clearance'):
        validate_fixture_spawns(fixture_graph, fixture['vehicles'])


def test_dense_conflict_vehicle_retained_with_full_stop_room(fixture_graph):
    fixture_graph.yield_entries['mlk_on_0'] = dict(target='sb2_0', via=None,
        visibility_m=100., stop_offset_m=.1)
    fixture = fixture_definition('dense-blocked', SEEDS[0], fixture_graph)
    ramp = next(row for row in fixture['vehicles'] if row['vehicle_id'] == 'fixture_ramp_conflict')
    assert ramp['speed'] == 8.
    assert fixture_graph.lanes[ramp['lane_id']]['length'] - ramp['position'] == 40.
    proof = next(row for row in fixture['static_spawn_checks']['vehicles'] if row['vehicle_id'] == ramp['vehicle_id'])
    assert proof['yield_approaches'][0]['available_stop_room_m'] == pytest.approx(39.9)
    assert proof['yield_approaches'][0]['required_stop_room_m'] == pytest.approx(1.5 * 8 + 64 / 9 + 2.5)
    ramp['speed'] = 16.
    with pytest.raises(ValueError, match='yield-approach spawn lacks'):
        validate_fixture_spawns(fixture_graph, fixture['vehicles'])


def test_fixture_xml_is_frozen_and_complete(fixture_graph, tmp_path):
    env = v05_bare(fixture_graph)
    env.__class__ = HighwayFlowEnv
    env.fixture = fixture_definition('ending-lane-temptation', SEEDS[0], fixture_graph)
    env.total_vehicles, env.speed_limit_m_s, env.horizon = 8, 18., 800
    path = tmp_path / 'routes.xml'
    env._write_demand(path, SEEDS[0])
    xml = ET.parse(path)
    assert len(xml.findall('vehicle')) == 8
    assert len(env.population_manifest) == 8
    assert xml.find("vType[@id='controlled']").get('carFollowModel') == 'ACC'
    with pytest.raises(ValueError, match='seed'):
        env._write_demand(tmp_path / 'wrong.xml', 1)


def test_existing_output_refused_before_any_native_build(tmp_path):
    # Outside canonical outputs must fail too; no subprocess is launched.
    with pytest.raises(ValueError, match='fresh'):
        build_network(tmp_path, 'added-lane')
    with pytest.raises(ValueError, match='fresh'):
        build(tmp_path, tmp_path / 'config.json')
