import json
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest

from scripts import build_i110_sumo as builder


def compiled_fixture(path, *, ramp_foes='000001', ramp_response='000001',
                     auxiliary_foes='100000', other_foes='000000',
                     ramp_shape='-1,0 0,-2', ramp_state='m'):
    root = ET.Element('net')
    for edge_id, shapes in [(':n2_0', [f'{i*3.7},0 {i*3.7},-2' for i in range(5)]),
                            (':n2_5', [ramp_shape])]:
        edge = ET.SubElement(root, 'edge', id=edge_id)
        for i, shape in enumerate(shapes):
            ET.SubElement(edge, 'lane', id=f'{edge_id}_{i}', shape=shape)
    junction = ET.SubElement(root, 'junction', id='n2', type='priority',
                             intLanes=':n2_0_0 :n2_0_1 :n2_0_2 :n2_0_3 :n2_0_4 :n2_5_0')
    for index in range(6):
        ET.SubElement(junction, 'request', index=str(index),
                      foes=ramp_foes if index == 5 else auxiliary_foes if index == 0 else other_foes,
                      response=ramp_response if index == 5 else '000000')
    for index in range(5):
        ET.SubElement(root, 'connection', {'from': 'sb1', 'to': 'sb2', 'fromLane': str(index),
                      'toLane': str(index), 'via': f':n2_0_{index}', 'state': 'M'})
    ET.SubElement(root, 'connection', {'from': 'mlk_on', 'to': 'sb2', 'fromLane': '0',
                  'toLane': '0', 'via': ':n2_5_0', 'state': ramp_state})
    ET.ElementTree(root).write(path)
    return path


def test_true_auxiliary_conflict_and_yield_are_retained(tmp_path):
    result = builder.validate_compiled_merge(compiled_fixture(tmp_path/'network.xml'))
    assert result['passed']
    assert result['auxiliary_merge_foe_retained'] and result['auxiliary_merge_yield_retained']
    assert result['through_lane_foes'] == result['connector_crossings'] == []


@pytest.mark.parametrize('changes', [
    {'ramp_foes': '011111'}, {'ramp_response': '011111'}, {'other_foes': '100000'},
    {'ramp_foes': '000000'}, {'ramp_response': '000000'}, {'auxiliary_foes': '000000'},
    {'ramp_shape': '-1,0 15,-1 0,-2'},
])
def test_false_foes_removed_safety_or_crossing_geometry_fail(tmp_path, changes):
    assert not builder.validate_compiled_merge(compiled_fixture(tmp_path/'network.xml', **changes))['passed']


@pytest.mark.parametrize('changes', [{'ramp_foes': '1'}, {'ramp_state': 'M'}])
def test_incomplete_matrix_or_forced_priority_is_rejected(tmp_path, changes):
    with pytest.raises(ValueError):
        builder.validate_compiled_merge(compiled_fixture(tmp_path/'network.xml', **changes))


def test_existing_output_is_never_overwritten(tmp_path):
    evidence = tmp_path/'network.net.xml'
    evidence.write_text('frozen historical evidence')
    with pytest.raises(FileExistsError):
        builder.build(tmp_path)
    assert evidence.read_text() == 'frozen historical evidence'


def test_build_fails_closed_and_retains_failed_conflict_report(tmp_path, monkeypatch):
    output = tmp_path/'invalid-build'
    def fake_netconvert(command, **kwargs):
        compiled_fixture(Path(command[command.index('--output-file')+1]), ramp_foes='011111')
        return SimpleNamespace(stdout='', stderr='', check_returncode=lambda: None)
    monkeypatch.setattr(builder.subprocess, 'run', fake_netconvert)
    with pytest.raises(ValueError, match='lane-conflict validation'):
        builder.build(output, netconvert='unused-netconvert')
    failure = json.loads((output/'failed-geometry.json').read_text())
    assert not failure['passed'] and len(failure['through_lane_foes']) == 4
    assert not (output/'network-manifest.json').exists()
