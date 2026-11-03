import math
import xml.etree.ElementTree as ET
import pytest

from traffic.i110_policy_interface import LaneGraph, FEATURES, require_actor_schema
from traffic.smooth_acc import ComfortConfig, limit_acceleration


def test_comfort_ramps_using_actual_previous_acceleration():
    a=0.;v=15.;c=ComfortConfig()
    for _ in range(30):
        b=limit_acceleration(2.,a,v,.1,c)
        assert abs(b-a)<=.2+1e-9 and b<=1.5
        v+=b*.1;a=b
    for _ in range(30):
        b=limit_acceleration(-4.5,a,v,.1,c)
        assert abs(b-a)<=.2+1e-9 and b>=-2.5
        v+=b*.1;a=b
    # A safety-induced actual brake is the next step's reference; do not jump
    # immediately to the stale pre-intervention command.
    assert limit_acceleration(2.,-4.5,10.,.1,c)==pytest.approx(-4.3)


@pytest.mark.parametrize('args',[(math.nan,0,1,.1),(1,0,-1,.1),(1,0,1,0)])
def test_invalid_comfort_input_rejected(args):
    with pytest.raises(ValueError):limit_acceleration(*args)


def test_nonnegative_speed_and_invalid_settings():
    assert limit_acceleration(-4.5,-2.,.01,.1)>=-.1
    with pytest.raises(ValueError):ComfortConfig(max_jerk=0)


@pytest.fixture
def graph(tmp_path):
    # Simplified test topology: 4 lanes -> internal links -> 5 lanes; an exit
    # branches from the right through lane and a ramp joins the auxiliary lane.
    root=ET.Element('net')
    for edge,count,length in [('sb0',4,100),('sb1',5,100),('vernon_off',1,100),('mlk_on',1,100)]:
        e=ET.SubElement(root,'edge',id=edge)
        for i in range(count):ET.SubElement(e,'lane',id=f'{edge}_{i}',index=str(i),length=str(length))
    for i in range(4):
        e=ET.SubElement(root,'edge',id=f':join{i}',function='internal')
        ET.SubElement(e,'lane',id=f':join{i}_0',index='0',length='2')
        ET.SubElement(root,'connection',{'from':'sb0','fromLane':str(i),'to':'sb1','toLane':str(i+1),'via':f':join{i}_0'})
        ET.SubElement(root,'connection',{'from':f':join{i}','fromLane':'0','to':'sb1','toLane':str(i+1)})
    ET.SubElement(root,'connection',{'from':'sb0','fromLane':'0','to':'vernon_off','toLane':'0'})
    ET.SubElement(root,'connection',{'from':'mlk_on','fromLane':'0','to':'sb1','toLane':'0'})
    p=tmp_path/'network.net.xml';ET.ElementTree(root).write(p)
    return LaneGraph(p)


def state(lane,pos,route=('sb0','sb1'),speed=15.,x=0.,y=0.):
    return dict(lane_id=lane,lane_position=pos,route=route,length=4.8,speed=speed,
        acceleration=0.,distance=pos,allowed_speed=25.,x=x,y=y)


def test_neighbors_cross_edge_reset_and_exclude_exit_branch(graph):
    states={'ego':state('sb0_0',95),'leader':state('sb1_1',8),
        'exit':state('vernon_off_0',1,('sb0','vernon_off')),'rear':state('sb0_0',80)}
    packet=graph.observe('ego',states,1.)
    assert packet['neighbors']['own']['front']['id']=='leader'
    assert packet['neighbors']['own']['front']['bumper_gap_m']==pytest.approx(5+2+8-4.8)
    assert packet['neighbors']['own']['rear']['bumper_gap_m']==pytest.approx(15-4.8)
    assert packet['remaining_route_m']==pytest.approx(107)
    assert len(packet['features'])==len(FEATURES)==30
    assert not packet['legacy_checkpoint_compatible']


def test_lane_identity_survives_addition_and_internal_link(graph):
    assert graph.identity('sb0_0')==graph.identity(':join0_0')==graph.identity('sb1_1')
    assert graph.identity('sb1_0')['kind']=='auxiliary'
    assert graph.adjacent(':join0_0',1)==(':join1_0',True)
    packet=graph.observe('ego',{'ego':state(':join0_0',1)},1.)
    assert packet['adjacent_context_known']


def test_rear_search_crosses_internal_link(graph):
    packet=graph.observe('ego',{'ego':state('sb1_1',5),'rear':state('sb0_0',95)},1.)
    assert packet['neighbors']['own']['rear']['bumper_gap_m']==pytest.approx(5+2+5-4.8)


def test_missing_route_completion_is_masked_not_fabricated(graph):
    packet=graph.observe('ego',{'ego':state('sb1_0',20,('sb1','unreachable'))},1.)
    assert packet['remaining_route_m'] is None
    assert packet['features'][-1]==0.
    with pytest.raises(KeyError):graph.observe('absent',{},1.)


def test_legacy_actor_explicitly_rejected():
    with pytest.raises(ValueError):require_actor_schema({'observation_size':27})
    require_actor_schema({'observation_schema':'i110_graph_v1','observation_size':30})
