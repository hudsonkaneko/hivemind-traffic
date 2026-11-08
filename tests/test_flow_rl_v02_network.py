"""Pure preparation/protocol checks; these tests do not start SUMO."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import xml.etree.ElementTree as ET

import pytest

from scripts.prepare_flow_rl_v02_network import ramp_connection, verify_visibility_only, VISIBILITY_M
from scripts.screen_flow_rl_v02_network import install_conflict_schedule, assess, screen


@pytest.fixture
def network_pair(tmp_path):
    root = ET.Element('net')
    for edge, lid in [('mlk_on','mlk_on_0'),('sb1','sb1_0')]:
        e=ET.SubElement(root,'edge',id=edge)
        ET.SubElement(e,'lane',id=lid,length='200',speed='13.41',shape='0,0,0 0,200,0')
    ET.SubElement(root,'connection',{'from':'mlk_on','to':'sb2','fromLane':'0','toLane':'0','via':':n2_5_0','state':'m'})
    ET.SubElement(root,'connection',{'from':'sb1','to':'sb2','fromLane':'0','toLane':'0','via':':n2_0_0','state':'M'})
    junction=ET.SubElement(root,'junction',id='n2',type='priority')
    ET.SubElement(junction,'request',index='5',foes='000001',response='000001')
    original=tmp_path/'original.xml'; candidate=tmp_path/'candidate.xml'
    ET.ElementTree(root).write(original)
    ramp_connection(root).set('visibility',str(VISIBILITY_M))
    ET.ElementTree(root).write(candidate)
    return original,candidate


def test_only_visibility_may_change(network_pair):
    proof=verify_visibility_only(*network_pair)
    assert proof['visibility_only'] and proof['topology_unchanged']
    assert proof['original_visibility_m']==4.5 and proof['candidate_visibility_m']==100.
    assert not proof['site_sight_distance_verified']


@pytest.mark.parametrize('change',['priority','speed','foe','topology'])
def test_safety_geometry_and_speed_changes_rejected(network_pair,change):
    original,candidate=network_pair; tree=ET.parse(candidate)
    if change=='priority': ramp_connection(tree.getroot()).set('state','M')
    elif change=='speed': tree.find('./edge/lane').set('speed','26.82')
    elif change=='foe': tree.find('./junction/request').set('foes','000000')
    else: ET.SubElement(tree.getroot(),'connection',{'from':'sb3','to':'sb4','fromLane':'0','toLane':'0'})
    tree.write(candidate)
    with pytest.raises(ValueError): verify_visibility_only(original,candidate)


def test_conflict_schedule_retains_native_safety_and_releases_after_declared_time():
    api=Mock(); original=Mock()
    env=SimpleNamespace(_fixture_commands=original,_measurement_phase=True,states={'main_4':{}},
        time=4.,start_time=.1,conn=SimpleNamespace(vehicle=api),fixture_event_log=[])
    fixture=dict(native_conflict_schedule=dict(vehicle_id='main_4',through_time_s=8.,speed_m_s=8.,
        speed_mode=31,lane_change_mode=512,release_speed=-1.,release_lane_change_mode=1621))
    install_conflict_schedule(env,fixture)
    env._fixture_commands()
    api.setSpeedMode.assert_called_with('main_4',31)
    api.setLaneChangeMode.assert_called_with('main_4',512)
    api.setSpeed.assert_called_with('main_4',8.)
    env.time=8.2;env._fixture_commands()
    api.setLaneChangeMode.assert_called_with('main_4',1621)
    api.setSpeed.assert_called_with('main_4',-1.)
    assert len(env.network_screen_events)==1
    assert env.fixture_event_log==[]
    env._fixture_commands();assert len(env.network_screen_events)==1


def test_occupied_label_without_measured_traffic_does_not_pass_coverage():
    assessment=assess([],[],dict(collisions=0,teleports=0,overspeed_steps=0),True)
    assert not assessment['actual_conflict_coverage']
    assert assessment['observed_conflict_samples']==0


def test_actual_approach_conflict_and_entry_gap_uses_bumper_distance():
    before=dict(time=1.,focal=dict(lane_id='mlk_on_0',lane_remaining_m=10.),
                blocker=dict(lane_id='sb1_0',lane_position=190.,speed=8.))
    after=dict(time=1.1,focal=dict(lane_id='sb2_0',leader=('main_4',12.),speed=8.,native_min_gap_m=2.5))
    frames=[dict(time=1.1,vehicles=[dict(id='agent_0',speed=8.,distance_m=10.),dict(id='main_4',speed=8.)])]
    diagnostics=[dict(before=before,after=after,info={},conflict_approach_length_m=200.)]
    assessment=assess(frames,diagnostics,dict(collisions=0,teleports=0,overspeed_steps=0),True)
    assert assessment['actual_conflict_coverage']
    assert assessment['entry_gaps'][0]['bumper_gap_m']==14.5
    assert assessment['entry_gaps'][0]['margin_m']==0.
    assert assessment['entry_gap_gate']


def test_allocated_screen_cannot_be_rerun(tmp_path,monkeypatch):
    import scripts.screen_flow_rl_v02_network as module
    project=tmp_path; prepared=project/'outputs/highway-flow-rl/v02/network-screen-test'
    prepared.mkdir(parents=True);(prepared/'screen-ledger.json').write_text('{}')
    monkeypatch.setattr(module,'ROOT',project);monkeypatch.chdir(project)
    with pytest.raises(ValueError,match='cannot be retried'): screen(prepared)
