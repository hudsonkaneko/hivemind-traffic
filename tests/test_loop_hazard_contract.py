from dataclasses import replace
import math

import pytest

from environments.loop_hazard_contract import HazardLaneTask,hazard_observation,safe_target_speed,HAZARD_OBSERVATION_NAMES
from traffic.driver_control import AppliedControl
from traffic.lane_geometry import LaneRoute,RouteSegment
from traffic.path_following import VehicleState
from traffic.road_hazards import MeasuredObject,make_hazard_schedule

ROAD=LaneRoute('straight',3.6,(RouteSegment(100.),))
COAST=AppliedControl(0.,0.,0.,'accepted',1,False)


def state(station=0.,speed=0.):
    return dict(position_m=[station,0.,.8],velocity_m_s=[speed,0.,0.],
                quaternion_xyzw=[0.,0.,0.,1.],yaw_rad=0.,upright_z=1.,
                speed_m_s=abs(speed),wheel_on_ground=[True]*4)


def base(speed=0.,side=0.,heading=0.):
    x=[0.]*22;x[3]=math.sin(heading);x[4]=math.cos(heading);x[5]=speed;x[6]=side
    return x


def obj(x=10.,y=0.,vx=0.,vy=0.,time=0.,identity='hazard_000'):
    return MeasuredObject(identity,x,y,0.,vx,vy,1.,1.,time)


def test_hand_computed_rotated_measured_velocity_and_mask():
    ego=VehicleState('e','ego',0,0.,0.,math.pi/2,math.sqrt(5))
    # World ego velocity (2,1), route heading0, ego heading pi/2 -> local(1,-2).
    o=replace(obj(3.,4.,5.,6.),yaw_rad=math.pi/2)
    frame=hazard_observation(ego,base(2.,1.,math.pi/2),[o],now_s=0.)
    assert len(frame.vector)==42==len(HAZARD_OBSERVATION_NAMES)
    assert frame.vector[22:32]==pytest.approx((1.,4.,-3.,5.,-3.,.5,.5,0.,1.,0.))
    assert frame.vector[32:]==(0.,)*10


def test_nearest_identity_order_no_schedule_leak_and_duplicate_rejection():
    ego=VehicleState('e','ego',0,0.,0.,0.,0.)
    objects=[obj(4.,identity='b'),obj(-4.,identity='a'),obj(9.,identity='c')]
    frame=hazard_observation(ego,base(),objects,now_s=0.)
    assert frame.selected_ids==('a','b')
    assert frame.vector==hazard_observation(ego,base(),list(reversed(objects)),now_s=0.).vector
    with pytest.raises(ValueError):hazard_observation(ego,base(),[objects[0],objects[0]],now_s=0.)


@pytest.mark.parametrize('timestamp',[-.1,1.])
def test_stale_future_is_unhealthy_and_brakes(timestamp):
    # All raw timestamps nonnegative; first case is stale at now1.
    now=1. if timestamp<0 else 0.
    o=obj(time=0. if timestamp<0 else timestamp)
    ego=VehicleState('e','ego',0,0.,0.,0.,0.)
    frame=hazard_observation(ego,base(),[o],now_s=now)
    assert not frame.healthy and frame.vector[22]==0 and frame.stale_ids==('hazard_000',)
    assert safe_target_speed(ego,base(),[o],now_s=now)[0]==0


def test_current_velocity_prediction_brakes_before_entry_but_not_receding():
    ego=VehicleState('e','ego',0,0.,0.,0.,3.)
    approaching=safe_target_speed(ego,base(3.),[obj(10.,3.3,vy=-1.)],now_s=0.)
    receding=safe_target_speed(ego,base(3.),[obj(10.,3.3,vy=1.)],now_s=0.)
    assert approaching[0]<3 and approaching[1]['reason']=='object_braking_target'
    assert receding[0]==3
    # Static gap is 4.6m to the bumper: v^2/(2*1.5)+.2v+1 => exactly3m/s.
    assert safe_target_speed(ego,base(3.),[obj(7.5)],now_s=0.)[0]==pytest.approx(3.)


def init(stage):
    schedule=make_hazard_schedule(stage,101,route=ROAD)
    task=HazardLaneTask(ROAD,stage=stage)
    task.set_objects(schedule.sample(ROAD,0.),0.)
    assert len(task.initialize(state(),COAST))==42
    return task,schedule


def step(task,schedule,tick,station=0.,speed=0.,**kwargs):
    task.set_objects(schedule.sample(ROAD,tick/120),tick/120)
    return task.advance(state(station,speed),COAST,tick=tick,**kwargs)


def settle(task,schedule):
    for tick in range(2,241,2):
        transition=step(task,schedule,tick)
        assert transition.reward==0 and not transition.terminated


@pytest.mark.parametrize('stage',['clear','static','crossing'])
def test_stationary_is_not_success_even_with_hazard_wait(stage):
    task,schedule=init(stage)
    for tick in range(2,4201,2):tr=step(task,schedule,tick)
    assert tr.truncated and not tr.success and not tr.terminated and tr.reason=='time_limit'
    assert tr.reward<0 and tr.terminal_observation==tr.observation


@pytest.mark.parametrize('stage',['clear','crossing'])
def test_positive_progress_completes_without_endpoint_stop(stage):
    task,schedule=init(stage);settle(task,schedule)
    for i in range(1,1201):tr=step(task,schedule,240+2*i,i*.05,3.)
    assert tr.success and tr.terminated and not tr.truncated
    assert tr.route_state.planar_speed_m_s==3 and tr.route_state.target_speed_m_s==3
    assert tr.reward_components['progress']==pytest.approx(.05/3)
    assert tr.reward_components['terminal']==10
    with pytest.raises(RuntimeError):step(task,schedule,2642,60.05,3.)


def test_static_success_requires_progress_and_continuous_stop_hold():
    task,schedule=init('static');settle(task,schedule)
    goal=task.static_stop_station()
    assert goal==pytest.approx(schedule.hazards[0].station_m-4.9)
    count=math.ceil(goal/.05)
    for i in range(1,count+1):
        tr=step(task,schedule,240+2*i,min(i*.05,goal),3.)
    assert not tr.success
    tick=240+2*count
    # First zero-speed sample does not yet establish a full stopped interval.
    tr=step(task,schedule,tick+2,goal,0.)
    assert not tr.success
    for i in range(1,61):tr=step(task,schedule,tick+2+2*i,goal,0.)
    assert tr.success and tr.reward_components['terminal']==10
    assert task.last_hazard_report['minimum_clearance_m']==pytest.approx(2.)


def test_failure_beats_success_and_static_no_missing_frame():
    task,schedule=init('clear');settle(task,schedule)
    for i in range(1,1200):step(task,schedule,240+2*i,i*.05,3.)
    tr=step(task,schedule,2640,60.,3.,contact_count=1)
    assert not tr.success and tr.reason=='rigid_contact' and tr.reward_components['terminal']==-10
    task,_=init('static')
    with pytest.raises(ValueError):task.set_objects([],0.)
    with pytest.raises(RuntimeError):task.set_stage('clear')


def test_skipped_ticks_stale_frame_and_unphysical_jump_rejected():
    task,schedule=init('clear')
    with pytest.raises(ValueError):step(task,schedule,4)
    with pytest.raises(ValueError):task.advance(state(),COAST,tick=2)
    # Previous failed call did not consume the physical transition.
    tr=step(task,schedule,2,2.)
    assert tr.terminated and tr.reason=='state_discontinuity' and tr.reward_components['progress']==0
