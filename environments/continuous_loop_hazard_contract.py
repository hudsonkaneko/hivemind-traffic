"""Three rolling60m references on one physical run; no pose or reset authority.

The original120/60Hz guards and35s window budget remain unchanged. Each window
has its own elapsed observation; settling occurs only once at physical start.
A crossing is required in window0 only. Its same measured body remains an
observation in the later clear continuation windows; there is no new schedule.
"""
from copy import deepcopy
from dataclasses import replace

from environments.full_loop_hazard_contract import FullLoopHazardTask
from environments.lane_following_contract import _state
from traffic.lane_geometry import LaneRoute, RouteSegment

WINDOW_COUNT=3
WINDOW_METRES=60.
WINDOW_TICKS=4200
MAX_TICKS=WINDOW_COUNT*WINDOW_TICKS


class ContinuousFullLoopHazardTask(FullLoopHazardTask):
    def __init__(self,route,*,stage='clear'):
        if (stage not in ('clear','crossing') or len(route.segments)!=1
                or route.segments[0]!=RouteSegment(100.,.002) or route.width_m!=3.6):
            raise ValueError('Continuous v1 requires a clear/crossing100m R500 route')
        self.initial_route=route
        self.initial_stage=stage
        self.window_index=0
        self.window_start_tick=0
        self.completed_windows=0
        self.window_events=[]
        self.last_continuation={}
        self._ever_initialized=False
        super().__init__(route,stage=stage)

    def initialize(self,state,applied,*,tick=0):
        if self._ever_initialized:
            raise RuntimeError('Continuous task permits one initial physical reset only')
        observation=super().initialize(state,applied,tick=tick)
        self._ever_initialized=True
        return observation

    def _observation(self,state,applied,tick,relative):
        values=list(super()._observation(state,applied,tick,relative))
        values[19]=min(1.,(tick-self.window_start_tick)/WINDOW_TICKS)
        # Base index20 remains global initial settling; never restart it.
        return tuple(values)

    def _roll(self,state,applied,tick,transition):
        before=deepcopy(_state(state))
        previous_route=self.route
        endpoint=previous_route.evaluate(WINDOW_METRES)
        self.window_index+=1
        self.window_start_tick=tick
        self.route=LaneRoute(previous_route.route_id,previous_route.width_m,
            previous_route.segments,endpoint.position_xy,endpoint.yaw_rad)
        # Only the first window has a crossing completion requirement. The
        # measured object list is retained; it is never hidden or reseeded.
        self.set_stage('clear') # Parent just marked this intermediate success.
        self._start_s_m=0.
        self._hold_ticks=0
        self._terminal=False
        current=self._relative(before,tick)
        self._previous_relative=current
        self._previous_state=before
        self._report(before,tick)
        observation=self._observation(before,self._previous_control,tick,current)
        old_total=(self.window_index-1)*WINDOW_METRES+transition.route_state.progress_m
        new_total=self.window_index*WINDOW_METRES+current.progress_m
        if abs(old_total-new_total)>1e-6:
            raise RuntimeError('Rolling route reference changed measured total progress')
        event=dict(tick=tick,from_window=self.window_index-1,to_window=self.window_index,
            origin_before=list(previous_route.origin_xy_m),origin_after=list(self.route.origin_xy_m),
            yaw_before=previous_route.initial_yaw_rad,yaw_after=self.route.initial_yaw_rad,
            local_progress_before=transition.route_state.progress_m,local_progress_after=current.progress_m,
            total_progress_before=old_total,total_progress_after=new_total,
            physics_state_before=before,physics_state_after=deepcopy(_state(state)),
            physical_state_unchanged=before==_state(state),physical_reset=False,hazard_reseed=False)
        self.window_events.append(event)
        parts=dict(transition.reward_components)
        parts['terminal']=0. # Only the third window ends this continuous episode.
        return replace(transition,observation=observation,route_state=current,
            reward_components=parts,reward=sum(parts.values()),terminated=False,truncated=False,
            success=False,reason='driving',terminal_observation=None),event

    def advance(self,state,applied,*,tick,**kwargs):
        before_index=self.window_index
        before_start=self.window_start_tick
        transition=super().advance(state,applied,tick=tick,**kwargs)
        event=None
        if transition.success:
            self.completed_windows+=1
            if self.completed_windows<WINDOW_COUNT:
                transition,event=self._roll(state,applied,tick,transition)
        elif transition.truncated and transition.reason=='time_limit':
            # Parent's frozen35s clock is absolute. In this explicitly new
            # contract it applies to each window; physical failures still win.
            if tick<before_start+WINDOW_TICKS and tick<MAX_TICKS:
                self._terminal=False
                transition=replace(transition,truncated=False,reason='driving',terminal_observation=None)
        self.last_continuation=dict(schema_version=1,window_index_before=before_index,
            window_index_after=self.window_index,window_start_tick=self.window_start_tick,
            window_deadline_tick=self.window_start_tick+WINDOW_TICKS,
            global_deadline_tick=MAX_TICKS,completed_windows=self.completed_windows,
            total_progress_m=self.window_index*WINDOW_METRES+transition.route_state.progress_m,
            original_hazard_stage=self.initial_stage,current_task_stage=self.stage,
            window_roll=deepcopy(event),physical_reset_count_expected=1,
            hazard_schedule_count_expected=1,scope='three60m windows; single original hazard encounter')
        self.last_continuation['feature_distribution_shift']='Moving window starts retain speed/control; training resets started at rest. No repeated settling.'
        return transition
