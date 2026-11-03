import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import torch
from environments.i110_merge_env import I110MergeEnv, MergeLaneGraph, LOCAL_FEATURES, MESSAGE_FEATURES, write_demand
from experiments.run_i110_merge import load_config, model_for, rank

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture
def graph(tmp_path):
    net=ET.Element('net')
    for edge,count,length in [('sb1',5,200),('sb2',5,200),('sb3',5,200),('sb4',4,200),('mlk_on',1,150)]:
        e=ET.SubElement(net,'edge',id=edge)
        for i in range(count):ET.SubElement(e,'lane',id=f'{edge}_{i}',index=str(i),length=str(length))
    for source,target in [('sb1','sb2'),('sb2','sb3')]:
        for i in range(5):ET.SubElement(net,'connection',{'from':source,'to':target,'fromLane':str(i),'toLane':str(i)})
    for i in range(1,5):ET.SubElement(net,'connection',{'from':'sb3','to':'sb4','fromLane':str(i),'toLane':str(i-1)})
    ET.SubElement(net,'connection',{'from':'mlk_on','to':'sb2','fromLane':'0','toLane':'0'})
    path=tmp_path/'net.xml';ET.ElementTree(net).write(path)
    return MergeLaneGraph(path)


def vehicle(lane,pos,route,speed=10):
    return dict(id='ego',lane_id=lane,lane_position=pos,route=route,length=4.8,speed=speed,
                acceleration=0.,distance=pos,allowed_speed=26.8224,x=0.,y=-pos)


def test_merge_distance_uses_lane_end_not_restarted_position(graph):
    env=I110MergeEnv.__new__(I110MergeEnv);env.graph=graph
    assert env.merge_distance(vehicle('mlk_on_0',100,('mlk_on','sb2','sb3','sb4')))==450
    assert env.merge_distance(vehicle('sb3_0',190,('mlk_on','sb2','sb3','sb4')))==10
    assert env.merge_distance(vehicle('sb3_1',190,('mlk_on','sb2','sb3','sb4'))) is None


def test_rear_neighbor_on_other_incoming_route_is_visible(graph):
    ego=vehicle('sb2_1',5,('mlk_on','sb2','sb3','sb4'))
    other=vehicle('sb1_1',195,('sb1','sb2','sb3','sb4'))
    found=graph.neighbor(ego,{'ego':ego,'rear':other},'sb2_1',5,forward=False)
    assert found['id']=='rear' and found['bumper_gap_m']==pytest.approx(5.2)


def test_demand_departures_are_exact_millisecond_grid(graph,tmp_path):
    path=tmp_path/'demand.xml';write_demand(path,1,graph)
    values=[v.get('depart') for v in ET.parse(path).findall('vehicle')]
    assert len(values)==44 and '13.700' in values
    original=path.read_bytes();write_demand(path,1,graph);assert path.read_bytes()==original


def test_graph_schema_is_not_legacy_even_with_same_width(tmp_path):
    cfg=load_config(ROOT/'experiments/configs/i110_merge_v1.json')
    assert not cfg['training_authorized'] and not cfg['contract']['legacy_actor_compatible']
    cfg['initial_checkpoint']='policy.pt';path=tmp_path/'bad.json';path.write_text(json.dumps(cfg))
    with pytest.raises(ValueError,match='legacy'):load_config(path)


def test_seed_partition_collision_rejected(tmp_path):
    cfg=load_config(ROOT/'experiments/configs/i110_merge_v1.json')
    cfg['evaluation']['test_seeds']=cfg['evaluation']['validation_seeds']
    path=tmp_path/'bad.json';path.write_text(json.dumps(cfg))
    with pytest.raises(ValueError,match='overlap'):load_config(path)


def test_actor_respects_stay_only_mask_without_learning():
    torch.set_num_threads(1)
    cfg=load_config(ROOT/'experiments/configs/i110_merge_v1.json');model=model_for(cfg)
    assert len(LOCAL_FEATURES)==33 and len(MESSAGE_FEATURES)==8
    obs=torch.zeros(4,81);obs[:,31]=1
    with torch.no_grad():
        act,logp,_,pre=model.act(obs)
        recalculated,_=model.evaluate_actions(obs,act,pre)
    assert torch.all(act[:,1]==0) and torch.isfinite(logp).all()
    assert torch.allclose(logp,recalculated)


def test_missing_agent_critic_slot_is_zero():
    env=I110MergeEnv.__new__(I110MergeEnv);env.agents=['agent_0'];env.obs={'agent_0':np.ones(81)}
    state=env.state();assert state.shape==(328,) and np.all(state[82:]==0) and state[81]==1


def test_selection_prioritizes_safety_before_merge_reward():
    good=dict(task='cruise',mean_fraction_of_speed_limit=1.,passes=0,collisions=0,teleports=0,overspeed_steps=0,successful_merges=1,
              accounting={'totals':{'unfinished':0}},overrides=0,agent_steps=10,reward=0.)
    bad=copy.deepcopy(good);bad.update(collisions=1,successful_merges=2,reward=1000.)
    assert rank([good])>rank([bad])


def test_curriculum_covers_tasks_and_reserves_six_lane_transfer():
    cfg=load_config(ROOT/'experiments/configs/i110_merge_v1.json')
    train=[c for c in cfg['curriculum'] if c['split']=='train']
    validation=[c for c in cfg['curriculum'] if c['split']=='validation']
    test=[c for c in cfg['curriculum'] if c['split']=='test']
    assert len(train)==20
    assert {c['task'] for c in train}=={'cruise','following','passing','merge'}
    assert {c['through_lanes'] for c in train}=={2,3,4,5}
    assert not any(c['through_lanes']==6 for c in train+validation)
    assert any(c['through_lanes']==6 for c in test)
    train_hashes={c['network_sha256'] for c in train}
    assert not train_hashes & {c['network_sha256'] for c in validation+test}


def test_highway_only_demand_has_no_ramp_actors(graph,tmp_path):
    path=tmp_path/'cruise.xml';write_demand(path,2,graph,task='cruise',density='light')
    vehicles=ET.parse(path).findall('vehicle')
    assert len(vehicles)==12
    assert all(v.get('route')=='main' for v in vehicles)
