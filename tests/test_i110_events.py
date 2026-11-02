from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import xml.etree.ElementTree as ET

import pytest

from traffic.i110_events import EventAdapter, add_actor, load_event, physical_lane, speed_target
from traffic.i110_baseline import prepare_demand, CONTROLLED
from traffic.trip_accounting import read_requests

ROOT = Path(__file__).resolve().parents[1]


def spec(name='cutin'):
    return load_event(ROOT/f'scenarios/i110/events/{name}-v1.json')


@pytest.mark.parametrize('key,value', [('speed_mode',0),('lane_change_mode',0),('motion_owner','USD'),
    ('onset_s',float('nan')),('initial_physical_lane',4),('enabled','yes'),('release_s',1),('kind','construction')])
def test_invalid_event_rejected(tmp_path,key,value):
    s=spec();s[key]=value;p=tmp_path/'event.json';p.write_text(json.dumps(s))
    with pytest.raises(ValueError):load_event(p)


def test_disabled_event_leaves_demand_bytes_unchanged_and_enabled_counts_actor(tmp_path):
    prepare_demand(tmp_path,731001)
    path=tmp_path/'routes.rou.xml';before=path.read_bytes()
    s=spec();s['enabled']=False;add_actor(path,s)
    assert path.read_bytes()==before
    add_actor(path,spec('emergency-brake'))
    requests=read_requests(path,CONTROLLED)
    assert len(requests)==45 and sum(r.origin=='mlk_ramp' for r in requests)==8
    assert ET.parse(path).find("./vType[@id='acc']").get('decel')=='4.5'
    assert ET.parse(path).find("./vType[@id='event_type']").get('decel')=='9'


def test_braking_schedule_reaches_stop_recovers_and_releases():
    s=spec('emergency-brake')
    assert speed_target(s,8)==15
    assert speed_target(s,9)==6
    assert speed_target(s,10)==0
    assert speed_target(s,14)>0
    assert speed_target(s,25)==15
    assert speed_target(s,26)==-1


def state(lane=3,speed=15,acceleration=0):
    return dict(road_id='sb1',lane_index=lane,lane_position=30,speed=speed,acceleration=acceleration)


def test_safe_lane_request_is_not_counted_as_realized_and_no_stale_query():
    adapter=EventAdapter(spec());conn=SimpleNamespace(vehicle=Mock())
    adapter.observe(2,{'event_actor':state()})
    adapter.apply(conn,2,{'event_actor':state()},.1)
    conn.vehicle.changeLane.assert_called_once_with('event_actor',2,3)
    adapter.observe(2.1,{'event_actor':state()})
    assert not adapter.summary()['event_gate_passed']
    assert adapter.records[0]['realized']['lane_index']==3
    adapter.apply(conn,2.1,{},.1)
    assert conn.vehicle.setSpeed.call_count==1
    assert adapter.summary()['missing_actor_command_steps']==1


def test_realized_lane_transition_requires_actual_follower_and_release():
    adapter=EventAdapter(spec());conn=SimpleNamespace(vehicle=Mock())
    adapter.observe(2,{'event_actor':state()})
    adapter.apply(conn,2,{'event_actor':state()},.1)
    states={'event_actor':state(lane=2),'agent_1':{'leader':('event_actor',30)}}
    adapter.observe(2.1,states)
    assert not adapter.summary()['event_gate_passed']
    adapter.apply(conn,8,states,.1)
    assert adapter.summary()['event_gate_passed']
    assert physical_lane('sb1',2)==1 and physical_lane('sb4',1)==1
    assert physical_lane(':n1',0) is None and physical_lane('sb2',0) is None


def test_stopping_without_strong_braking_while_followed_fails_event_gate():
    adapter=EventAdapter(spec('emergency-brake'));conn=SimpleNamespace(vehicle=Mock())
    states={'event_actor':state(lane=2,speed=0,acceleration=-2),'agent_1':{'leader':('event_actor',30)}}
    adapter.observe(10,states);adapter.apply(conn,26,states,.1)
    assert not adapter.summary()['event_gate_passed']
    states['event_actor']=state(lane=2,speed=1,acceleration=-9)
    adapter.observe(11,states)
    assert adapter.summary()['event_gate_passed']
