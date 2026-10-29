import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import xml.etree.ElementTree as ET

import pytest
from traci import constants as tc
from traffic.i110_baseline import prepare_demand,CONTROLLED
from traffic.i110_events import EventAdapter,load_event
from traffic.i110_interactions import InteractionRecorder,lane_flags
from traffic.smooth_acc import command
from traffic.trip_accounting import read_requests

ROOT=Path(__file__).resolve().parents[1]


def test_delaying_controlled_request_preserves_every_other_request(tmp_path):
    a=tmp_path/'a';b=tmp_path/'b';a.mkdir();b.mkdir()
    prepare_demand(a,732001);prepare_demand(b,732001,agent0_depart=44)
    def vehicles(p):
        return {v.get('id'):(v.attrib,[child.attrib for child in v])
                for v in ET.parse(p/'routes.rou.xml').findall('vehicle')}
    av,bv=vehicles(a),vehicles(b)
    assert av.pop('agent_0')!=bv.pop('agent_0') and av==bv
    requests=read_requests(b/'routes.rou.xml',CONTROLLED)
    assert len(requests)==44 and sum(r.origin=='mlk_ramp' for r in requests)==8
    assert next(r for r in requests if r.vehicle_id=='agent_0').requested_at==44


@pytest.mark.parametrize('value',[-1,51,float('nan'),.05])
def test_departure_shift_validation(tmp_path,value):
    with pytest.raises(ValueError):prepare_demand(tmp_path,1,agent0_depart=value)
    assert not (tmp_path/'routes.rou.xml').exists()


def test_follow_cap_exposes_comfort_conflict_without_changing_modes():
    vehicle=Mock();vehicle.getSpeed.return_value=17.51;vehicle.getAcceleration.return_value=1.5
    vehicle.getLeader.return_value=('leader',30.539);vehicle.getFollowSpeed.return_value=17.326296
    vehicle.getDecel.return_value=4.5;vehicle.getAllowedSpeed.return_value=25
    conn=SimpleNamespace(vehicle=vehicle)
    smooth=command(conn,'ego',.1)
    capped=command(conn,'ego',.1,follow_cap=True)
    assert smooth['filtered_acceleration']==pytest.approx(1.3)
    assert smooth['applied_acceleration']==pytest.approx(1.3)
    assert capped['applied_acceleration']==pytest.approx(-1.83704)
    vehicle.setSpeedMode.assert_not_called();vehicle.setLaneChangeMode.assert_not_called()


def test_lateral_off_control_keeps_speed_requests_without_lane_command():
    spec=load_event(ROOT/'scenarios/i110/events/cutin-v1.json');spec['request_lane_change']=False
    adapter=EventAdapter(spec);conn=SimpleNamespace(vehicle=Mock())
    state=dict(road_id='sb0',lane_index=2,lane_position=45,speed=15,acceleration=0)
    for t in (1.,2.,3.,8.):
        adapter.observe(t,{'event_actor':state});adapter.apply(conn,t,{'event_actor':state},.1)
    conn.vehicle.changeLane.assert_not_called()
    assert conn.vehicle.setSpeed.call_count==4 and adapter.summary()['event_gate_passed']


def test_unknown_flags_are_not_unblocked_gaps():
    assert lane_flags(tc.LCA_UNKNOWN)['known'] is False
    assert lane_flags(tc.LCA_UNKNOWN)['blocked'] is None
    assert lane_flags(tc.LCA_LEFT|tc.LCA_BLOCKED_BY_LEFT_FOLLOWER)==dict(known=True,wants_left=True,blocked=True)


def test_interaction_requires_direct_leader_and_records_blocked_steps(tmp_path):
    recorder=InteractionRecorder(tmp_path/'journal.jsonl');conn=SimpleNamespace(vehicle=Mock())
    conn.vehicle.getLaneChangeState.return_value=(tc.LCA_LEFT|tc.LCA_BLOCKED_BY_LEFT_FOLLOWER,0)
    def state(lane,leader):return dict(road_id='sb2',lane_index=lane,lane_position=20,
        speed=12,acceleration=0,leader=leader,waiting_time=0)
    recorder.observe(conn,1.,{'ramp_000':state(0,None),'agent_0':state(1,None)})
    assert not recorder.summary()['interaction_observed']
    recorder.observe(conn,1.1,{'ramp_000':state(1,('agent_0',20)),'agent_0':state(1,None)})
    recorder.close();s=recorder.summary()
    assert s['interaction_observed'] and s['interaction_steps']==1
    assert s['ramps']['ramp_000']['blocked_desire_steps']==1
    assert s['ramps']['ramp_000']['first_through_lane_s']==1.1
    assert len((tmp_path/'journal.jsonl').read_text().splitlines())==2
