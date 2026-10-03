from dataclasses import asdict, replace
import json
import math

import pytest
from traffic.state_contract import VehicleState, Snapshot, StateStream, from_dict, prim_path, decode_prim_path
from traffic.study_config import StudyConfig
from traffic.runtime_metrics import refresh_sensor_packets
from traffic.capacity_model import workload


@pytest.mark.parametrize('heading,center,yaw', [(0,(10,17.5,0),90),(90,(7.5,20,0),0),(180,(10,22.5,0),-90),(270,(12.5,20,0),-180)])
def test_center_offsets_rotate(heading,center,yaw):
    p,y=VehicleState('car',10,20,heading,0).center_pose()
    assert p == pytest.approx(center)
    assert y == yaw


@pytest.mark.parametrize('name', ['car_1','car/1','car-1','é 🚗'])
def test_identity_reversible(name):
    assert decode_prim_path(prim_path('episode',name)) == ('episode',name)
    assert prim_path('one',name) != prim_path('two',name)


def test_contract_roundtrip_and_lifecycle():
    v=VehicleState('a',0,0,90,2)
    s=Snapshot('e',0,0,.1,(v,))
    assert from_dict(json.loads(json.dumps(asdict(s)))) == s
    stream=StateStream()
    assert stream.accept(s)['spawned'] == ['a']
    with pytest.raises(ValueError):stream.accept(s)
    assert stream.accept(replace(s,step=1,time_s=.1,vehicles=()))['removed'] == ['a']
    with pytest.raises(ValueError):stream.accept(replace(s,step=2,time_s=.2))
    with pytest.raises(ValueError):stream.accept(replace(s,episode_id='next'))
    stream.reset()
    assert stream.accept(replace(s,episode_id='next'))['spawned'] == ['a']


@pytest.mark.parametrize('kwargs',[{'time_s':math.nan},{'schema_version':2},{'dt_s':0},{'step':True},{'pose_reference':'center'},{'frame':'feet'}])
def test_contract_rejects_ambiguity(kwargs):
    with pytest.raises(ValueError): Snapshot(**(dict(episode_id='e',step=0,time_s=0,dt_s=.1,vehicles=())|kwargs))


@pytest.mark.parametrize('kwargs',[{'vehicles':0},{'vehicles':True},{'av_fraction':1.1},{'control_hz':3},{'seconds':math.inf},{'control_mode':'coordinated'},{'seed':-1}])
def test_config_rejects_unsupported_claims(kwargs):
    with pytest.raises(ValueError):StudyConfig(**kwargs)


def test_roles_nested_deterministic_and_rounding():
    small=StudyConfig(vehicles=20,av_fraction=.25)
    large=replace(small,av_fraction=.5)
    ids=lambda c:{i for i,v in c.roles().items() if v=='av'}
    assert len(ids(small))==5 and ids(small)<ids(large)
    assert small.roles()==replace(small).roles()
    assert StudyConfig(vehicles=3,av_fraction=.5).av_count==2
    assert StudyConfig(av_fraction=0).av_count==0


def test_refresh_requires_new_recent_packets():
    packets={'a':{'timestamp':1,'received_wall':0}}
    def update():packets['a'].update(timestamp=packets['a']['timestamp']+1,received_wall=10)
    report=refresh_sensor_packets(update,packets,clock=lambda:10.01)
    assert report['frames']==6 and report['receipt_ages_s']['a']==pytest.approx(.01)
    with pytest.raises(RuntimeError):refresh_sensor_packets(lambda:None,packets,clock=lambda:10.01)
    with pytest.raises(RuntimeError):refresh_sensor_packets(update,packets,clock=lambda:11)


def test_workload_math_and_neighbor_bound():
    two=workload(vehicles=20,lidar_vehicles=2,av_vehicles=10)
    all_sensed=workload(vehicles=20,lidar_vehicles=20,av_vehicles=10)
    assert two['nominal_lidar_sample_opportunities_per_second']==460800
    assert all_sensed['nominal_lidar_sample_opportunities_per_second']==10*two['nominal_lidar_sample_opportunities_per_second']
    full=workload(vehicles=100,lidar_vehicles=2,av_vehicles=100)
    bounded=workload(vehicles=100,lidar_vehicles=2,av_vehicles=100,max_neighbors=8)
    assert full['logical_receiver_deliveries_per_second']==99000
    assert bounded['logical_receiver_deliveries_per_second']==8000
    assert full['full_return_payload_GB_per_hour']==pytest.approx(33.1776)


def test_invalid_workload_counts():
    with pytest.raises(ValueError):workload(vehicles=2,lidar_vehicles=3,av_vehicles=1)
    with pytest.raises(ValueError):workload(vehicles=2,lidar_vehicles=1,av_vehicles=1,control_hz=math.nan)
