from dataclasses import replace
import json
import math
import random

import pytest

from traffic.lane_geometry import LaneRoute, RouteSegment
from traffic.road_hazards import (
    MeasuredObject,HazardSpec,HazardSchedule,make_hazard_schedule,feasible_envelope)

ROAD=LaneRoute('straight',3.6,(RouteSegment(100.),))


def test_private_seed_order_and_preserved_draws():
    a=make_hazard_schedule('crossing',2101,3,route=ROAD)
    random.seed(7);[random.random() for _ in range(100)]
    make_hazard_schedule('static',9,99)
    assert a==make_hazard_schedule('crossing',2101,3,route=ROAD)
    assert a!=make_hazard_schedule('crossing',2101,4,route=ROAD)
    assert feasible_envelope(a)['feasible']
    assert json.loads(json.dumps(a.as_dict(),allow_nan=False))['hazards'][0]['object_id']=='hazard_000'


@pytest.mark.parametrize('seed',range(20))
def test_generated_crossings_are_offroad_continuous_and_certified(seed):
    schedule=make_hazard_schedule('crossing',seed)
    h=schedule.hazards[0]
    assert abs(schedule.sample(ROAD,0)[0].y_m)>=3.3
    assert feasible_envelope(schedule)['feasible']
    previous=schedule.sample(ROAD,0)[0]
    for i in range(1,2401):
        current=schedule.sample(ROAD,i/120)[0]
        assert math.hypot(current.x_m-previous.x_m,current.y_m-previous.y_m)<=h.speed_m_s/120+1e-10
        previous=current
    assert abs(previous.y_m)>=3.3-1e-10
    assert (previous.vx_m_s,previous.vy_m_s)==(0.,0.)


def test_mirrored_crossings_and_rotated_road_velocity():
    left=HazardSpec('hazard_000','crossing',25,3.3,-3.3,5.,1.)
    right=replace(left,start_lateral_m=-3.3,end_lateral_m=3.3)
    for t in (0.,5.,6.,8.3,12.):
        a,b=left.sample(ROAD,t),right.sample(ROAD,t)
        assert a.x_m==b.x_m and a.y_m==-b.y_m and a.vy_m_s==-b.vy_m_s
    rotated=LaneRoute('rotated',3.6,(RouteSegment(100.),),(500.,0.),math.pi/2)
    o=left.sample(rotated,6.)
    assert o.x_m==pytest.approx(497.7) and o.y_m==pytest.approx(25.)
    assert o.vx_m_s==pytest.approx(1.) and o.vy_m_s==pytest.approx(0.)


def test_clear_empty_static_stationary_and_late_impossible_diagnostic():
    assert make_hazard_schedule('clear',1).sample(ROAD,100)==()
    static=make_hazard_schedule('static',1)
    assert replace(static.sample(ROAD,0)[0],timestamp_s=8)==static.sample(ROAD,8)[0]
    late=HazardSchedule('crossing',1,0,(HazardSpec('hazard_000','crossing',25,3.3,-3.3,20.,1.),))
    cert=feasible_envelope(late)
    assert not cert['feasible'] and cert['objects'][0]['reserve_m']<0


def test_forced_side_preserves_random_draws_and_mirrors_only_lateral_motion():
    a=make_hazard_schedule('crossing',101,side='left')
    b=make_hazard_schedule('crossing',101,side='right')
    for t in (0.,7.,10.,15.):
        x,y=a.sample(ROAD,t)[0],b.sample(ROAD,t)[0]
        assert x.x_m==y.x_m and x.y_m==-y.y_m and x.vy_m_s==-y.vy_m_s
    with pytest.raises(ValueError):make_hazard_schedule('static',1,side='left')
    with pytest.raises(ValueError):make_hazard_schedule('crossing',1,side='up')


@pytest.mark.parametrize('changes',[{'width_m':0},{'timestamp_s':float('nan')},{'vx_m_s':True},{'object_id':''}])
def test_invalid_measured_data(changes):
    fields=dict(object_id='o',x_m=1.,y_m=0.,yaw_rad=0.,vx_m_s=0.,vy_m_s=0.,length_m=1.,width_m=1.,timestamp_s=0.)
    with pytest.raises(ValueError):MeasuredObject(**(fields|changes))


@pytest.mark.parametrize('stage,seed,index',[('unknown',1,0),('clear',True,0),('clear',-1,0),('crossing',1,-1),('static',2**32,0)])
def test_invalid_schedule_identity(stage,seed,index):
    with pytest.raises(ValueError):make_hazard_schedule(stage,seed,index)
