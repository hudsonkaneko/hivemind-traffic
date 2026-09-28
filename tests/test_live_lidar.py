import numpy as np
import pytest
from traffic.lidar_control import Clearance,forward_clearance,target_speed


def test_labels_not_required_and_ground_excluded():
    az=np.zeros(2000);el=np.full(2000,-30.);r=np.full(2000,2.)
    el[:10]=0.;r[:10]=12.
    c=forward_clearance(az,el,r,np.full(2000,64))
    assert c.healthy and c.points==10 and c.distance==12.


def test_empty_and_corrupt_sensor_brakes():
    for c in [Clearance(80,False,0),Clearance(float('nan'),True,10)]:
        assert target_speed(c,8)[0]<8
    assert target_speed(Clearance(80,True,0),8,fresh=False)[1]=='sensor-failsafe'
    assert not forward_clearance([],[],[],np.array([],dtype=int)).healthy


def test_control_bounds_and_stop():
    assert target_speed(Clearance(4,True,10),0)[0]==0
    assert target_speed(Clearance(80,True,0),1)[0]==pytest.approx(1.2)
    assert target_speed(Clearance(5,True,10),8)[0]==pytest.approx(7.7)


def test_stale_clear_road_cannot_keep_cruising():
    clear=Clearance(80,True,20)
    assert target_speed(clear,8,fresh=True)[0]==8
    assert target_speed(clear,8,fresh=False)[0]==pytest.approx(7.7)


def test_invalid_arrays_rejected():
    with pytest.raises(ValueError): forward_clearance([1],[1,2],[1],[64])


@pytest.mark.parametrize('dt', [0,-1,float('nan'),float('inf')])
def test_invalid_control_interval_rejected(dt):
    with pytest.raises(ValueError):target_speed(Clearance(80,True,20),8,dt=dt)


def test_self_and_side_returns_excluded():
    az=np.full(2000,180.);el=np.zeros(2000);r=np.full(2000,2.)
    az[:1000]=90.
    c=forward_clearance(az,el,r,np.full(2000,64))
    assert c.healthy and c.points==0 and c.distance==80.


def test_invalid_flag_returns_excluded():
    assert not forward_clearance(np.zeros(2000),np.zeros(2000),np.ones(2000),np.zeros(2000,dtype=int)).healthy


@pytest.mark.parametrize('gap', [0,4,5,10,40,80])
@pytest.mark.parametrize('speed', [0,4,8,12])
def test_acceleration_limits(gap,speed):
    requested,_=target_speed(Clearance(gap,True,20),speed)
    assert max(0,speed-.3)-1e-10 <= requested <= speed+.2+1e-10
