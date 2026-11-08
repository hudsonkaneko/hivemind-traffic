"""Rolling route mathematics and reset callbacks; no native launches."""
from copy import deepcopy
import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

from environments.continuous_loop_hazard_contract import ContinuousFullLoopHazardTask,MAX_TICKS,WINDOW_TICKS
from traffic.driver_control import AppliedControl
from traffic.lane_geometry import LaneRoute,RouteSegment
from traffic.road_hazards import make_hazard_schedule

ROOT=Path(__file__).resolve().parents[1]
ROAD=LaneRoute('loop_r500',3.6,(RouteSegment(100.,.002),),(500.,0.),math.pi/2)
LONG=LaneRoute('loop_r500',3.6,(RouteSegment(300.,.002),),(500.,0.),math.pi/2)
COAST=AppliedControl(0.,0.,0.,'accepted',1,False)


def state(progress=0.,speed=0.,lateral=0.):
    point=LONG.evaluate(progress);c,s=math.cos(point.yaw_rad),math.sin(point.yaw_rad)
    return dict(position_m=[point.position_xy[0]-s*lateral,point.position_xy[1]+c*lateral,.8],
        velocity_m_s=[c*speed,s*speed,0.],quaternion_xyzw=[0.,0.,math.sin(point.yaw_rad/2),math.cos(point.yaw_rad/2)],
        yaw_rad=point.yaw_rad,upright_z=1.,speed_m_s=abs(speed),wheel_on_ground=[True]*4)


def create(stage='clear'):
    task=ContinuousFullLoopHazardTask(ROAD,stage=stage)
    schedule=make_hazard_schedule(stage,61001,route=ROAD)
    task.set_objects(schedule.sample(ROAD,0.),now_s=0.)
    task.set_background_objects((),now_s=0.)
    task.initialize(state(),COAST)
    return task,schedule


def step(task,schedule,tick,progress,speed=3.,**kwargs):
    task.set_objects(schedule.sample(ROAD,tick/120),now_s=tick/120)
    task.set_background_objects((),now_s=tick/120)
    return task.advance(state(progress,speed),COAST,tick=tick,**kwargs)


def settle(task,schedule):
    for tick in range(2,241,2):
        tr=step(task,schedule,tick,0.,0.)
        assert not tr.terminated and tr.reward==0.


def first_window(task,schedule):
    settle(task,schedule)
    for index in range(1,1203):
        tick=240+2*index;progress=index*.05
        tr=step(task,schedule,tick,progress)
        if task.window_index==1:return tick,progress,tr
    raise AssertionError('Synthetic first window never rolled')


def test_three_windows_keep_measured_state_and_count_progress_once():
    task,schedule=create();settle(task,schedule)
    progress_sum=0.;seen=[]
    for index in range(1,3603):
        tick=240+2*index;progress=index*.05
        original=state(progress);before=deepcopy(original)
        task.set_objects((),now_s=tick/120);task.set_background_objects((),now_s=tick/120)
        tr=task.advance(original,COAST,tick=tick)
        assert original==before
        progress_sum+=tr.reward_components['progress']*3
        if task.last_continuation['window_roll']:
            event=task.last_continuation['window_roll'];seen.append(event)
            assert not tr.terminated and not tr.success and tr.terminal_observation is None
            assert tr.reward_components['terminal']==0.
            assert tr.observation[19:]==(0.,0.,0.)+(0.,)*20
            assert event['physics_state_before']==event['physics_state_after']
            assert event['total_progress_before']==pytest.approx(event['total_progress_after'],abs=1e-6)
        if tick==4200:assert not tr.truncated # Absolute35s is not the second window's deadline.
        if tr.terminated:break
    assert len(seen)==2 and task.completed_windows==3
    assert tr.success and tr.reason=='success' and tr.reward_components['terminal']==10.
    assert task.last_continuation['total_progress_m']==pytest.approx(progress,abs=1e-6)
    assert progress_sum==pytest.approx(progress,abs=1e-6)
    assert tick<MAX_TICKS and tr.elapsed_s>35.
    assert tr.observation[0]==pytest.approx(progress-120,abs=1e-6)
    with pytest.raises(RuntimeError):task.initialize(state(),COAST)
    with pytest.raises(RuntimeError):step(task,schedule,tick+2,progress+.05)


def test_first_window_timeout_remains_exact_and_unsuccessful():
    task,schedule=create()
    for tick in range(2,WINDOW_TICKS+1,2):tr=step(task,schedule,tick,0.,0.)
    assert tr.truncated and not tr.success and tr.reason=='time_limit'
    assert task.completed_windows==0


def test_second_window_has_bounded_deadline_not_unlimited_timeout_suppression():
    task,schedule=create();start,progress,_=first_window(task,schedule)
    for tick in range(start+2,start+WINDOW_TICKS+1,2):
        tr=step(task,schedule,tick,progress,0.)
        if tick<start+WINDOW_TICKS:assert not tr.truncated
    assert tr.truncated and tr.reason=='time_limit' and not tr.success
    assert task.completed_windows==1 and tr.observation[19]==1.


def test_physical_failure_at_roll_boundary_dominates_success():
    task,schedule=create();settle(task,schedule)
    for index in range(1,1200):step(task,schedule,240+index*2,index*.05)
    tr=step(task,schedule,2640,60.,contact_count=1)
    assert tr.reason=='rigid_contact' and tr.terminated and not tr.success
    assert task.window_index==0 and not task.window_events


def test_latched_failure_after_absolute35s_cannot_be_suppressed():
    task,schedule=create();tick,progress,_=first_window(task,schedule)
    for now in range(tick+2,4200,2):step(task,schedule,now,progress,0.)
    tr=step(task,schedule,4200,progress,0.,failure_reason='object_overlap')
    assert tr.reason=='object_overlap' and tr.terminated and not tr.truncated
    assert not tr.success


def test_crossing_completion_only_first_window_object_identity_retained():
    task,schedule=create('crossing')
    _,_,tr=first_window(task,schedule)
    assert task.initial_stage=='crossing' and task.stage=='clear'
    assert len(task._objects)==1 and task._objects[0].object_id=='hazard_000'
    assert task.last_continuation['original_hazard_stage']=='crossing'
    assert not task.last_continuation['window_roll']['hazard_reseed']
    assert len(tr.observation)==42 and tr.observation[22]==1.


@pytest.mark.parametrize('stage',['static','bad'])
def test_unsupported_continuation_stage_rejected(stage):
    with pytest.raises(ValueError):ContinuousFullLoopHazardTask(ROAD,stage=stage)


def test_unsupported_curvature_rejected():
    road=LaneRoute('straight',3.6,(RouteSegment(100.),))
    with pytest.raises(ValueError):ContinuousFullLoopHazardTask(road)


@pytest.fixture
def adapter(monkeypatch):
    class Parent:
        def _get_dones(self):
            self.extras={'lane_transition':{}}
            return ('done','timeout')
        def _reset_idx(self,env_ids):self.pose_writes+=1
        def reset(self,**kwargs):self._reset_idx([0]);return 'reset'
    monkeypatch.setitem(sys.modules,'environments.full_loop_hazard_lab',
        SimpleNamespace(FullLoopHazardLabEnv=Parent,FullLoopHazardLabCfg=object))
    spec=importlib.util.spec_from_file_location('_continuous_adapter_test',ROOT/'environments/continuous_loop_hazard_lab.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    env=module.ContinuousFullLoopHazardLabEnv.__new__(module.ContinuousFullLoopHazardLabEnv)
    env.route=ROAD;env.original_hazard_route=ROAD;env.schedule=make_hazard_schedule('crossing',61001,route=ROAD)
    env._initialized_episode=False;env.reset_count=0;env._transition=None;env.extras={};env.pose_writes=0
    env.native_window_boundaries=[];env.follower=SimpleNamespace(routes={ROAD.route_id:ROAD})
    return env


def test_rolling_route_does_not_translate_or_restart_hazard_schedule(adapter):
    before=adapter._poses(12.)
    endpoint=ROAD.evaluate(60.)
    adapter.route=LaneRoute(ROAD.route_id,ROAD.width_m,ROAD.segments,endpoint.position_xy,endpoint.yaw_rad)
    assert adapter._poses(12.)==before
    assert adapter.schedule.episode_index==0


def test_initial_reset_only_final_autoreset_suppressed(adapter):
    assert adapter.reset(seed=1)=='reset' and adapter.pose_writes==1
    adapter._initialized_episode=True;adapter.reset_count=1
    with pytest.raises(RuntimeError):adapter.reset(seed=2)
    with pytest.raises(RuntimeError):adapter._reset_idx([0])
    adapter._transition=SimpleNamespace(terminated=True,truncated=False)
    adapter.episode_tick=7500
    adapter._reset_idx([0])
    assert adapter.pose_writes==1 and adapter.extras['autoreset_suppressed']['terminal_state_retained']
    adapter._reset_idx([]);assert adapter.pose_writes==1


def test_native_boundary_rechecks_actual_state_and_preserves_follower(adapter):
    task,schedule=create();tick,progress,_=first_window(task,schedule)
    adapter.task=task;adapter.episode_tick=tick;adapter.reset_count=1
    adapter.vehicle=SimpleNamespace(state=lambda **kwargs:state(progress,3.))
    follower=adapter.follower
    assert adapter._get_dones()==('done','timeout')
    assert adapter.native_window_boundaries[0]['equal']
    assert adapter.route==task.route and adapter.follower is follower
    assert follower.routes[ROAD.route_id]==task.route
    adapter.vehicle.state=lambda **kwargs:state(progress+.001,3.)
    with pytest.raises(RuntimeError,match='native state'):adapter._get_dones()


@pytest.mark.parametrize('completed',[0,1,2,3])
def test_completed_or_failed_continuous_run_keeps_exact_roll_count(completed):
    from scripts.physics_rsl_showcase import continuous_boundary_contract
    boundaries=[{'equal':True,'physical_reset_count':1} for _ in range(min(completed,2))]
    assert continuous_boundary_contract(1,completed,boundaries,True)
    assert not continuous_boundary_contract(2,completed,boundaries,True)
    assert not continuous_boundary_contract(1,completed,boundaries,False)
    assert not continuous_boundary_contract(1,completed,[*boundaries,{'equal':True,'physical_reset_count':1}],True)
