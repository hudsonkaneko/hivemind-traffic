from types import SimpleNamespace
import math
import pytest
from traffic.i110_preview_v2 import PreviewV2Config, PreviewV2Controller, following_request
from traffic.smooth_acc import limit_acceleration
from traffic.i110_preview import release_guard


def test_no_leader_does_not_brake_for_close_slow_adjacent_car():
    conn=SimpleNamespace(vehicle=SimpleNamespace(getLeader=lambda vid,r:None))
    states={'ego':dict(speed=20.,x=0.,y=0.),'other':dict(speed=0.,x=3.,y=3.)}
    d=PreviewV2Controller().observe(conn,'ego',states,0.)
    assert d['requested_ceiling_m_s2'] is None
    assert d['excluded_nonleader_count']==1 and d['context_known']


def test_actual_path_leader_brakes_before_native_gap_mode_and_releases():
    leader=['lead',180.]
    conn=SimpleNamespace(vehicle=SimpleNamespace(getLeader=lambda vid,r:tuple(leader)))
    states={'ego':dict(speed=25.,x=0.,y=0.),'lead':dict(speed=10.,x=185.,y=0.)}
    c=PreviewV2Controller()
    early=c.observe(conn,'ego',states,0.)
    assert early['requested_ceiling_m_s2']<0
    states['ego']['speed']=10.; states['lead']['speed']=15.; leader[1]=50.
    assert c.observe(conn,'ego',states,.1)['requested_ceiling_m_s2']>0
    # No stale braking dwell after the target departs.
    conn.vehicle.getLeader=lambda vid,r:None
    assert c.observe(conn,'ego',states,.2)['requested_ceiling_m_s2'] is None
    with pytest.raises(ValueError):c.observe(conn,'ego',states,.2)


def test_zero_speed_restart_and_smooth_recovery_from_braking():
    speed,acc=0.,0.
    for _ in range(100):
        request=following_request(speed,10.,60.)
        filtered=limit_acceleration(request,acc,speed,.1)
        applied,guard=release_guard(filtered,acc,speed,.1,2.)
        assert abs(applied-acc)<=.2+1e-8 and not guard['existing_state_conflict']
        speed+=applied*.1; acc=applied
    assert speed>9.


def test_continuous_at_native_mode_boundary_and_no_forced_stop_at_matched_speed():
    assert abs(following_request(15.,12.,100.001)-following_request(15.,12.,99.999))<.001
    assert following_request(10.,10.,35.)>0


def test_unknown_leader_is_explicit_not_stale_latch():
    conn=SimpleNamespace(vehicle=SimpleNamespace(getLeader=lambda vid,r:('missing',30.)))
    d=PreviewV2Controller().observe(conn,'ego',{'ego':dict(speed=10.,x=0.,y=0.)},0.)
    assert not d['context_known'] and d['requested_ceiling_m_s2'] is None


@pytest.mark.parametrize('args',[(math.nan,1.,1.),(-1.,1.,1.),(1.,-1.,1.)])
def test_invalid_state_rejected(args):
    with pytest.raises(ValueError):following_request(*args)


def test_invalid_contract_rejected():
    with pytest.raises(ValueError):PreviewV2Config(range_m=0.)
