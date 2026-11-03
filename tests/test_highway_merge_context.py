import xml.etree.ElementTree as ET

import pytest

from traffic.highway_merge_context import Stage1LaneGraph, merge_target_context, merge_features


@pytest.fixture
def graph(tmp_path):
    net = ET.Element('net')
    for edge, count, length in [('sb1', 5, 200), ('sb2', 5, 600), ('sb3', 5, 200), ('sb4', 4, 200), ('mlk_on', 1, 150)]:
        element = ET.SubElement(net, 'edge', id=edge)
        for index in range(count):
            ET.SubElement(element, 'lane', id=f'{edge}_{index}', index=str(index), length=str(length), speed='25')
    for source, target in [('sb1', 'sb2'), ('sb2', 'sb3')]:
        for index in range(5):
            ET.SubElement(net, 'connection', {'from': source, 'to': target, 'fromLane': str(index), 'toLane': str(index)})
    for index in range(1, 5):
        ET.SubElement(net, 'connection', {'from': 'sb3', 'to': 'sb4', 'fromLane': str(index), 'toLane': str(index - 1)})
    ET.SubElement(net, 'connection', {'from': 'mlk_on', 'to': 'sb2', 'fromLane': '0', 'toLane': '0'})
    path = tmp_path / 'network.xml'
    ET.ElementTree(net).write(path)
    return Stage1LaneGraph(path)


MAIN = ('sb1', 'sb2', 'sb3', 'sb4')
RAMP = ('mlk_on', 'sb2', 'sb3', 'sb4')


def vehicle(name, lane, position, route=MAIN, speed=20., allowed_speed=25.):
    return dict(id=name, lane_id=lane, lane_position=position, route=route,
                length=4.8, speed=speed, acceleration=0., distance=position,
                allowed_speed=allowed_speed, x=0., y=-position)


def test_ramp_observes_mainline_front_and_rear_before_join(graph):
    ego = vehicle('ego', 'mlk_on_0', 100., RAMP)
    states = {'ego': ego,
              'human_front': vehicle('human_front', 'sb1_1', 180., speed=22.),
              'human_rear': vehicle('human_rear', 'sb1_1', 100., speed=21.),
              'unrelated_lane': vehicle('unrelated_lane', 'sb1_2', 151.)}
    context = merge_target_context(graph, ego, states)
    assert context['known'] and context['target_lane'] == 'sb2_1'
    assert context['lane_delta'] == 0  # A future target is not lane-change permission.
    assert context['distance_to_start_m'] == 50.
    assert context['front']['id'] == 'human_front'
    assert context['front']['bumper_gap_m'] == pytest.approx(25.2)
    assert context['front']['relative_speed_m_s'] == 2.
    assert context['rear']['id'] == 'human_rear'
    assert context['rear']['bumper_gap_m'] == pytest.approx(45.2)


def test_auxiliary_lane_sees_rear_on_different_incoming_route(graph):
    ego = vehicle('ego', 'sb2_0', 5., RAMP)
    rear = vehicle('background', 'sb1_1', 195.)
    context = merge_target_context(graph, ego, {'ego': ego, 'background': rear})
    assert context['lane_delta'] == 1
    assert context['distance_to_start_m'] == -5.
    assert context['rear']['bumper_gap_m'] == pytest.approx(5.2)


def test_long_segment_near_end_still_observes_next_segment(graph):
    ego = vehicle('ego', 'sb2_0', 590., RAMP)
    front = vehicle('front', 'sb3_1', 10.)
    context = merge_target_context(graph, ego, {'ego': ego, 'front': front})
    assert context['front']['bumper_gap_m'] == pytest.approx(15.2)


def test_connected_through_vehicle_does_not_claim_merge_context(graph):
    ego = vehicle('ego', 'sb2_1', 5.)
    context = merge_target_context(graph, ego, {'ego': ego})
    assert not context['known']
    features = merge_features(context, 2.)
    assert len(features) == 12 and features[10] == pytest.approx(2. / 4.5)
    assert not any(features[:10])


def test_no_neighbor_is_not_a_fabricated_zero_gap_vehicle(graph):
    ego = vehicle('ego', 'mlk_on_0', 100., RAMP)
    context = merge_target_context(graph, ego, {'ego': ego})
    features = merge_features(context, -4.5)
    assert features[0] == 1 and features[6] == features[9] == 0
    assert features[10] == -1.


def test_same_coordinates_on_unconnected_lane_do_not_create_neighbor(graph):
    ego = vehicle('ego', 'mlk_on_0', 100., RAMP)
    other = vehicle('other_ramp_car', 'mlk_on_0', 130., RAMP)
    context = merge_target_context(graph, ego, {'ego': ego, 'other_ramp_car': other})
    assert context['front'] is None and context['rear'] is None
