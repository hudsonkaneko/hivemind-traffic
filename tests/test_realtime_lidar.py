import numpy as np
import pytest
from traffic.realtime_lidar import interpolate, packet_points
from traffic.realtime_lidar import summarize_realtime
from traffic.lidar_avoidance import AvoidancePlanner


def packet():
    return dict(xyz=np.tile([40.,-4.8,1.],(1200,1)),flags=np.full(1200,64),
        offset=np.linspace(0,99_999_999,1200),timestamp=1_000_000_000,
        frame_start=1_083_333_333,frame_end=1_100_000_000)


EGO=dict(x=20.,y=-4.8,angle=90.,speed=6.)


def test_scan_time_is_not_latest_render_interval():
    points,healthy,age=packet_points(packet(),EGO,1.2)
    assert healthy and age==pytest.approx(.2)
    assert points.shape==(1200,2)


@pytest.mark.parametrize('now',[.9,1.5])
def test_future_or_stale_scan_fails_closed(now):
    _,healthy,_=packet_points(packet(),EGO,now)
    assert not healthy


def test_partial_scan_cannot_clear_adjacent_lane():
    p=packet();p['offset']=np.zeros(1200)
    points,healthy,_=packet_points(p,EGO,1.2)
    decision=AvoidancePlanner().step(points,healthy,EGO)
    assert decision.lane_request is None
    assert decision.speed<EGO['speed']


def test_filter_ground_and_self():
    p=packet();p['xyz'][:400]=[25.,-4.8,0.]
    p['xyz'][400:800]=[18.,-4.8,1.]
    pts,healthy,_=packet_points(p,EGO,1.2)
    assert healthy and len(pts)==400


def test_world_points_are_not_shifted_with_current_car_pose():
    a,_,_=packet_points(packet(),EGO,1.2)
    b,_,_=packet_points(packet(),dict(EGO,x=30.),1.2)
    np.testing.assert_array_equal(a,b)


def test_bad_arrays_rejected():
    p=packet();p['offset']=np.zeros(10)
    with pytest.raises(ValueError): packet_points(p,EGO,1.2)


def test_interpolation_short_heading_path():
    a=dict(x=0.,y=0.,angle=359.,speed=2.)
    b=dict(x=2.,y=4.,angle=1.,speed=4.)
    mid=interpolate(a,b,.5)
    assert mid==dict(x=1.,y=2.,angle=360.,speed=3.)


def test_sensor_dropout_brakes_without_new_lane_request():
    planner=AvoidancePlanner();ego=EGO.copy()
    for _ in range(25):
        decision=planner.step(np.empty((0,2)),False,ego,fresh=False)
        assert decision.lane_request is None
        ego['speed']=decision.speed
    assert ego['speed']==0


def test_performance_gate_does_not_hide_alignment_failure():
    ego=dict(x=100.,y=-4.8,angle=90.,speed=6.)
    row=dict(ego=ego,ego_after=ego,phase='complete',obstacle_gap=1.,
        realized_speed=6.,requested_speed=6.,healthy=True,collisions=[],
        obstacle_end_x=64.,sensor_age=.1,profile_seconds={'control':.01},deadline_lateness=0)
    assert summarize_realtime([row],False,-1,.1,.1)['passed']
    assert not summarize_realtime([row],False,-1,.1,.4)['passed']
    assert not summarize_realtime([row],False,-1,.2,.1)['passed']
    row['deadline_lateness']=.11
    assert not summarize_realtime([row],False,-1,.1,.1)['passed']


def test_gc_state_restored_on_runtime_error(monkeypatch,tmp_path):
    import gc
    from types import SimpleNamespace
    import traffic.realtime_lidar as rt
    original=gc.isenabled();callbacks=list(gc.callbacks)
    def fail(*args):
        assert not gc.isenabled()
        raise RuntimeError('injected')
    monkeypatch.setattr(rt,'_run_realtime',fail)
    try:
        with pytest.raises(RuntimeError,match='injected'):
            rt.run_realtime(None,None,None,None,None,None,None,None,
                SimpleNamespace(gc_mode='deferred'),tmp_path,None,None)
        assert gc.isenabled()==original and gc.callbacks==callbacks
        assert (tmp_path/'gc-profile.json').exists()
    finally:
        if original: gc.enable()
        else: gc.disable()
