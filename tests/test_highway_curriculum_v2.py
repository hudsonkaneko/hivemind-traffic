import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from scripts.build_highway_curriculum_v2 import source_geometry, specifications, validate_geometry, normalize_network


@pytest.mark.parametrize('lanes,bend', [(2, 0), (3, 20), (4, -30), (5, 20), (6, 45)])
def test_ramp_approaches_outside_right_extent_using_local_tangent(lanes, bend):
    _, _, _, placement = source_geometry(lanes=lanes, length=2200, bend=bend, speed_mph=60)
    delta = [a - b for a, b in zip(placement['ramp_start_xy'], placement['junction_xy'])]
    right = sum(a * b for a, b in zip(delta, placement['right_normal']))
    longitudinal = sum(a * b for a, b in zip(delta, placement['road_tangent']))
    assert right == pytest.approx(placement['mainline_lane_extent_m'] + 45.)
    assert longitudinal == pytest.approx(-160.)


def test_historical_generator_unchanged_and_holdout_dimensions_preserved():
    old = Path(__file__).resolve().parents[1] / 'scripts/build_highway_curriculum.py'
    assert hashlib.sha256(old.read_text().replace('\r\n', '\n').encode()).hexdigest() == '04c0aad97408f7469248af67f87c240baa53a0dbb8552bf0815de1dc5a849ac8'
    specs = specifications()
    assert len(specs) == 10
    assert next(s for s in specs if s[0] == 'train' and s[1] == 3) == ('train', 3, 2100, 20, 60)
    assert next(s for s in specs if s[0] == 'test' and s[1] == 6) == ('test', 6, 2800, 45, 45)


def network(tmp_path, *, ramp_shape='-2,1 -1,-1 0,-4', foes='0001', response='0001', ramp_start='-20,10 -2,1'):
    root = ET.Element('net')
    for edge_id, lane_shapes in [('sb1', ['0,10 0,0', '3.2,10 3.2,0', '6.4,10 6.4,0']),
                                 ('mlk_on', [ramp_start]),
                                 (':j_0', ['0,0 0,-4', '3.2,0 3.2,-4', '6.4,0 6.4,-4']),
                                 (':j_3', [ramp_shape])]:
        edge = ET.SubElement(root, 'edge', id=edge_id)
        for i, shape in enumerate(lane_shapes):
            ET.SubElement(edge, 'lane', id=f'{edge_id}_{i}', shape=shape)
    internal = ':j_0_0 :j_0_1 :j_0_2 :j_3_0'
    junction = ET.SubElement(root, 'junction', id='j', intLanes=internal)
    for index in range(4):
        ET.SubElement(junction, 'request', index=str(index), foes=foes if index == 3 else '0000', response=response if index == 3 else '0000')
    for index in range(3):
        ET.SubElement(root, 'connection', {'from': 'sb1', 'to': 'sb2', 'fromLane': str(index), 'toLane': str(index), 'via': f':j_0_{index}'})
    ET.SubElement(root, 'connection', {'from': 'mlk_on', 'to': 'sb2', 'fromLane': '0', 'toLane': '0', 'via': ':j_3_0'})
    path = tmp_path / 'network.xml'
    ET.ElementTree(root).write(path)
    return path


def test_compiled_geometry_requires_real_auxiliary_priority_and_no_false_foes(tmp_path):
    report = validate_geometry(network(tmp_path), through_lanes=2)
    assert report['passed'] and report['auxiliary_merge_foe_retained'] and report['auxiliary_merge_yield_retained']
    assert report['connector_crossings'] == report['through_lane_foes'] == []


@pytest.mark.parametrize('changes', [
    {'foes': '0111'}, {'response': '0111'}, {'foes': '0000'}, {'response': '0000'},
    {'ramp_start': '20,10 2,1'}, {'ramp_shape': '-2,1 7,-2 0,-4'},
])
def test_false_conflicts_crossings_wrong_side_or_removed_real_yield_fail(tmp_path, changes):
    assert not validate_geometry(network(tmp_path, **changes), through_lanes=2)['passed']


def test_normalization_removes_timestamp_and_machine_metadata_only(tmp_path):
    a, b = tmp_path / 'a.xml', tmp_path / 'b.xml'
    a.write_text('<?xml version="1.0"?><net><!-- generated at A in C:/machine/A --><edge id="one"/></net>')
    b.write_text('<?xml version="1.0"?><net><!-- generated at B in D:/machine/B --><edge id="one"/></net>')
    out_a, out_b = tmp_path / 'normalized-a.xml', tmp_path / 'normalized-b.xml'
    normalize_network(a, out_a)
    normalize_network(b, out_b)
    assert out_a.read_bytes() == out_b.read_bytes()
    assert ET.parse(out_a).find('edge').get('id') == 'one'


@pytest.mark.parametrize('kwargs', [dict(lanes=1), dict(length=500), dict(speed_mph=61), dict(bend=float('nan'))])
def test_invalid_geometry_contract_rejected(kwargs):
    values = dict(lanes=3, length=2100, bend=20, speed_mph=60)
    values.update(kwargs)
    with pytest.raises(ValueError):
        source_geometry(**values)
