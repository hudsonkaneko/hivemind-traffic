import numpy as np
import pytest

from traffic.highway_stage1_teacher import teacher_action, longitudinal_target_v3, discrete_stop_speed
from tests.test_highway_stage1_teacher import observation, physical, neighbor
from traffic.highway_merge_context import Stage1LaneGraph, merge_target_context, merge_target_context_v3
import xml.etree.ElementTree as ET
from tests.test_highway_merge_context import graph, vehicle
from tests.test_highway_stage1_env import bare_env

@pytest.fixture
def entry_graph(tmp_path):
    net=ET.Element('net')
    for name,count,length in [('sb1',4,380.31),('sb2',4,441.3),('sb3',4,415.77),('sb4',3,415.84),('sb5',3,420.),('mlk_on',1,128.05),(':n1_0',1,18.32)]:
        edge=ET.SubElement(net,'edge',id=name,**({'function':'internal'} if name.startswith(':') else {}))
        for lane in range(count):
            ET.SubElement(edge,'lane',id=f'{name}_{lane}',index=str(lane),length=str(length),speed='26.82')
    for source,target,count,shift in [('sb1','sb2',4,0),('sb2','sb3',4,0),('sb3','sb4',3,1),('sb4','sb5',3,0)]:
        for lane in range(count):
            ET.SubElement(net,'connection',{'from':source,'to':target,'fromLane':str(lane+shift),'toLane':str(lane),'state':'M'})
    ET.SubElement(net,'connection',{'from':'mlk_on','to':'sb2','fromLane':'0','toLane':'0','state':'m','via':':n1_0_0'})
    ET.SubElement(net,'connection',{'from':':n1_0','to':'sb2','fromLane':'0','toLane':'0','state':'M'})
    path=tmp_path/'minor-entry.net.xml';ET.ElementTree(net).write(path)
    return Stage1LaneGraph(path,3)


def test_v3_shared_target_tapers_unobserved_native_leader_near_limit():
    obs = observation(26.5, 26.82)
    target = longitudinal_target_v3(obs)
    assert target <= .4*(26.82-.1-26.5)+1e-6
    assert physical(teacher_action(obs, version='v3')) == pytest.approx(target, abs=2e-7)
    assert physical(teacher_action(obs)) > target


def test_v3_minor_entry_approach_is_distance_based_and_no_lateral_command():
    obs = observation(12.,13.41)
    obs[28] = obs[33] = 1.
    obs[35] = 0.
    obs[36] = 8./240.
    action = teacher_action(obs, version='v3')
    assert physical(action) < 0.
    assert action[1] == 0
    obs[36] = 4./240.
    assert physical(teacher_action(obs, version='v3')) > physical(action)


def test_v3_entry_does_not_release_when_target_rear_gap_is_unsafe():
    obs = observation(5.,13.41)
    obs[28] = obs[33] = 1.
    obs[36] = .5/240.
    neighbor(obs,40,3.,20.)
    assert physical(teacher_action(obs,version='v3')) < 0.


def test_stop_speed_is_finite_monotonic_and_uses_no_simulator_state():
    values = [discrete_stop_speed(d) for d in [0.,.1,1.,4.5,10.,100.]]
    assert values[0] == 0.
    assert all(np.isfinite(values))
    assert values == sorted(values)


def test_minor_entry_includes_native_point1_stop_offset_before_euler_epsilon():
    obs=observation(11.158137504197665,13.41)
    obs[28]=obs[33]=1.;obs[36]=13.740071654319763/240.
    target=longitudinal_target_v3(obs)
    expected=(discrete_stop_speed(float(obs[36])*240.-.1)-float(obs[0])*26.8224)/.1
    assert target==pytest.approx(expected,abs=1e-7)
    assert target==pytest.approx(-3.02509,abs=1e-5)


def test_v3_context_exposes_first_conflict_and_keeps_v2_projection(entry_graph):
    graph = entry_graph
    ego=dict(id='ego',lane_id='mlk_on_0',lane_position=100.,route=('mlk_on','sb2','sb3','sb4','sb5'),length=4.8,speed=10.)
    foe=dict(id='foe',lane_id='sb1_0',lane_position=365.,route=('sb1','sb2','sb3','sb4','sb5'),length=4.8,speed=20.)
    states={'ego':ego,'foe':foe}
    old=merge_target_context(graph,ego,states)
    new=merge_target_context_v3(graph,ego,states)
    assert old['target_lane']=='sb2_1'
    assert new['target_lane']=='sb2_0'
    assert new['distance_to_start_m']==pytest.approx(28.05)
    assert new['lane_delta']==0
    assert new['front']['id']=='foe'
    assert old['front'] is None


def test_v3_context_does_not_invent_lane_change_on_internal_ramp(entry_graph):
    graph=entry_graph
    ego=dict(id='ego',lane_id=':n1_0_0',lane_position=1.,route=('mlk_on','sb2','sb3','sb4','sb5'),length=4.8,speed=7.)
    context=merge_target_context_v3(graph,ego,{'ego':ego})
    assert not context['known'] and context['lane_delta']==0


def test_v3_rejects_unsupported_minor_visibility_instead_of_silent_assumption(entry_graph):
    graph=entry_graph
    graph.yield_entries['mlk_on_0']['visibility_m']=10.
    ego=dict(id='ego',lane_id='mlk_on_0',lane_position=100.,route=('mlk_on','sb2','sb3','sb4','sb5'),length=4.8,speed=10.)
    with pytest.raises(ValueError,match='visibility'):
        merge_target_context_v3(graph,ego,{'ego':ego})
    graph.yield_entries['mlk_on_0']['visibility_m']=4.5
    graph.yield_entries['mlk_on_0']['stop_offset_m']=1.
    with pytest.raises(ValueError,match='stop offset'):
        merge_target_context_v3(graph,ego,{'ego':ego})


def test_v3_reads_edge_level_stop_offset_as_unsupported_map_configuration(entry_graph):
    tree=ET.parse(entry_graph.path)
    edge=next(e for e in tree.findall('edge') if e.get('id')=='mlk_on')
    ET.SubElement(edge,'stopOffset',value='1.0')
    tree.write(entry_graph.path)
    rebuilt=Stage1LaneGraph(entry_graph.path,3)
    assert rebuilt.yield_entries['mlk_on_0']['stop_offset_m']==1.
    ego=dict(id='ego',lane_id='mlk_on_0',lane_position=100.,route=('mlk_on','sb2','sb3','sb4','sb5'),length=4.8,speed=10.)
    with pytest.raises(ValueError,match='stop offset'):
        merge_target_context_v3(rebuilt,ego,{'ego':ego})


def test_v3_environment_reward_uses_exact_same_observation_target_as_teacher(graph):
    env=bare_env(graph,vehicle('agent_0','sb1_1',100.,speed=24.5,allowed_speed=25.))
    env.control_target_version='v3'
    env.obs=env._observations()
    obs=env.obs['agent_0'].copy()
    action=teacher_action(obs,version='v3')
    env._advance=lambda: None
    _,_,_,_,infos=env.step({'agent_0':action})
    assert infos['agent_0']['feasible_acceleration_target']==longitudinal_target_v3(obs)
    assert infos['agent_0']['requested_acceleration']==pytest.approx(infos['agent_0']['feasible_acceleration_target'],abs=2e-7)
    assert infos['agent_0']['feasible_target_source']=='shared v3 observation-only teacher/reward target'
