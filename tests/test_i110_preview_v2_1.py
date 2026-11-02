import pytest
from traffic.i110_preview_v2_1 import conditional_request
from traffic.smooth_acc import limit_acceleration
from traffic.i110_preview import release_guard


def test_adjacent_overlap_is_not_a_forward_following_target():
    for gap in (-4.8,0.,2.5):
        assert conditional_request(20.,0.,gap,'adjacent_lane_path') is None


def test_adjacent_matched_speed_does_not_request_gap_opening_or_stop():
    assert conditional_request(15.,15.,5.,'adjacent_lane_path')==0.
    assert conditional_request(15.,15.,5.,'same_lane_path')<0.


def test_closing_on_adjacent_vehicle_releases_when_speeds_match():
    assert conditional_request(25.,10.,180.,'adjacent_lane_path')<0.
    assert conditional_request(10.,10.,40.,'adjacent_lane_path')>=0.
    assert conditional_request(10.,15.,40.,'adjacent_lane_path')>0.


def test_moving_adjacent_target_does_not_cause_v1_full_stop():
    speed,acc,gap=20.,0.,40.
    for _ in range(200):
        raw=conditional_request(speed,15.,gap,'adjacent_lane_path')
        assert raw is not None
        a=limit_acceleration(raw,acc,speed,.1)
        a,guard=release_guard(a,acc,speed,.1,2.)
        assert abs(a-acc)<=.2+1e-8 and not guard['existing_state_conflict']
        speed+=a*.1;gap+=(15.-speed)*.1;acc=a
        assert speed>10.
    assert speed==pytest.approx(15.,abs=.1)
