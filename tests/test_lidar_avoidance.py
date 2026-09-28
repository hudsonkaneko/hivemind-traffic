import numpy as np
import pytest
from traffic.lidar_avoidance import AvoidancePlanner, road_points, body_bounds, rectangle_gap, summarize


def ego(x=20.,y=-4.8,angle=90.,speed=6.):
    return dict(x=x,y=y,angle=angle,speed=speed)


def test_full_maneuver_uses_lidar_map():
    p=AvoidancePlanner()
    obstacle=np.array([[60.,-4.8],[64.,-4.8]])
    d=p.step(obstacle,True,ego());assert d.phase=='outbound' and d.lane_request==1
    # SUMO display heading settles after reaching the lane centre.
    d=p.step(obstacle,True,ego(50,-1.6,angle=84));assert d.phase=='passing'
    d=p.step(obstacle,True,ego(74,-1.6));assert d.lane_request is None
    d=p.step(obstacle,True,ego(77,-1.6));assert d.phase=='returning' and d.lane_request==0
    d=p.step(obstacle,True,ego(110,angle=96));assert d.phase=='complete'


@pytest.mark.parametrize('blocker_x',[0.,28.,50.])
def test_adjacent_front_side_and_rear_block_request(blocker_x):
    p=AvoidancePlanner()
    d=p.step(np.array([[60.,-4.8],[blocker_x,-1.6]]),True,ego())
    assert d.lane_request is None and not d.adjacent_clear and d.phase=='approach'
    # Occlusion does not erase a previously observed static obstacle.
    d=p.step(np.empty((0,2)),True,ego())
    assert d.lane_request is None


def test_bad_sensor_brakes_without_lane_change():
    p=AvoidancePlanner()
    for healthy,fresh in [(False,True),(True,False)]:
        d=p.step(np.array([[60.,-4.8]]),healthy,ego(),fresh=fresh)
        assert d.speed==pytest.approx(5.7) and d.lane_request is None
        assert p.phase=='approach'


def test_sensor_to_road_rotation_and_self_filter():
    az=np.zeros(2000);el=np.zeros(2000);r=np.full(2000,10.)
    # Body points behind the sensor must not become map obstacles.
    az[:1000]=180.;r[:1000]=2.
    points,healthy=road_points(az,el,r,np.full(2000,64),ego(angle=0))
    assert healthy and len(points)==1000
    assert np.allclose(points,[20.,5.2])


def test_evaluation_gap_is_conservative():
    assert body_bounds(ego())==pytest.approx((15,20,-5.8,-3.8))
    assert rectangle_gap(body_bounds(ego()),(60,64,-5.8,-3.8))==40
    assert rectangle_gap(body_bounds(ego()),(19,22,-5.8,-3.8))==0


def test_off_road_body_fails_even_if_maneuver_complete():
    row=dict(ego=ego(100,-.5),ego_after=ego(100,-.5),obstacle_gap=20.,
             realized_speed=6.,requested_speed=6.,phase='complete',healthy=True,
             collisions=[],obstacle_end_x=64.)
    assert not summarize([row])['passed']


def test_stale_scan_during_maneuver_does_not_issue_new_lane_request():
    p=AvoidancePlanner();p.step(np.array([[60.,-4.8]]),True,ego())
    d=p.step(np.empty((0,2)),True,ego(y=-3.2),fresh=False)
    assert d.phase=='outbound' and d.lane_request is None and d.speed<6
