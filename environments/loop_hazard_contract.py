"""Versioned privileged-state hazard task. Pure CPU; legacy lane task unchanged."""
from __future__ import annotations

from dataclasses import dataclass, replace
import math

from environments.lane_following_contract import (
    LaneFollowingTask, LaneTransition, OBSERVATION_NAMES, _state, _control, _integer)
from traffic.path_following import VehicleState, FRAME, SOURCE
from traffic.road_hazards import MeasuredObject, number
from traffic.bypass_validation import OrientedBox, box_clearance

OBJECT_FIELDS=('valid','forward_m','lateral_m','relative_forward_m_s',
               'relative_lateral_m_s','half_length_m','half_width_m',
               'sin_relative_heading','cos_relative_heading','age_s')
HAZARD_OBSERVATION_NAMES=OBSERVATION_NAMES+tuple(
    f'object_{slot}_{name}' for slot in range(2) for name in OBJECT_FIELDS)
OBSERVATION_SOURCE='privileged_measured_native_ego_and_objects_known_map'


@dataclass(frozen=True)
class HazardObservation:
    vector: tuple
    selected_ids: tuple
    stale_ids: tuple
    healthy: bool


def hazard_observation(ego, base22, objects, *, now_s, max_age_s=.2):
    """Chassis-frame slots, nearest squared distance then stable evidence ID.

    base22 contains route-tangent forward/lateral velocity and sine/cosine of
    ego-minus-route heading. These reconstruct measured ego velocity; scalar
    planar speed is never substituted for a signed velocity vector.
    A stale/future object is masked AND makes the returned frame unhealthy.
    Callers must fail/stop on unhealthy, never interpret masked stale as clear.
    """
    now=number(now_s,'now_s'); max_age=number(max_age_s,'max_age_s')
    if now<0 or max_age<0 or not isinstance(ego,VehicleState):
        raise ValueError('Invalid observation time or ego')
    if ego.frame!=FRAME or ego.source!=SOURCE or type(ego.tick) is not int or ego.tick<0:
        raise ValueError('Invalid ego provenance/tick')
    for value in (ego.x_m,ego.y_m,ego.yaw_rad,ego.speed_m_s):number(value,'ego')
    base=tuple(number(v,'base observation') for v in base22)
    if len(base)!=22 or abs(base[3]**2+base[4]**2-1)>1e-4:
        raise ValueError('Expected 22 fields and unit heading sine/cosine')
    objects=tuple(objects)
    if len(objects)>128 or any(not isinstance(o,MeasuredObject) for o in objects):
        raise ValueError('Expected at most 128 measured objects')
    if len({o.object_id for o in objects})!=len(objects):
        raise ValueError('Duplicate object identity')
    c,s=math.cos(ego.yaw_rad),math.sin(ego.yaw_rad)
    ego_vx=base[5]*base[4]+base[6]*base[3]
    ego_vy=-base[5]*base[3]+base[6]*base[4]
    ordered=sorted(objects,key=lambda o:((o.x_m-ego.x_m)**2+(o.y_m-ego.y_m)**2,o.object_id))
    stale=tuple(o.object_id for o in ordered if not 0<=now-o.timestamp_s<=max_age+1e-12)
    vector=list(base)
    for o in ordered[:2]:
        age=now-o.timestamp_s
        if o.object_id in stale:
            vector.extend((0.,0.,0.,0.,0.,0.,0.,0.,0.,age))
            continue
        dx,dy=o.x_m-ego.x_m,o.y_m-ego.y_m
        angle=o.yaw_rad-ego.yaw_rad
        vector.extend((1.,c*dx+s*dy,-s*dx+c*dy,c*o.vx_m_s+s*o.vy_m_s-ego_vx,
                       -s*o.vx_m_s+c*o.vy_m_s-ego_vy,o.length_m/2,o.width_m/2,
                       math.sin(angle),math.cos(angle),age))
    vector.extend((0.,)*(10*(2-min(2,len(ordered)))))
    if not all(math.isfinite(v) for v in vector):
        raise ValueError('Nonfinite transformed object state')
    return HazardObservation(tuple(vector),tuple(o.object_id for o in ordered[:2]),stale,not stale)


def safe_target_speed(ego, base22, objects, *, now_s, cruise_speed_m_s=3.,
                      max_age_s=.2, prediction_s=4., planning_deceleration_m_s2=1.5,
                      reaction_s=.2, margin_m=1., ego_half_length_m=2.4,
                      ego_half_width_m=.9, lateral_margin_m=.25):
    """Current constant-velocity swept corridor; no future schedule access.

    Objects crossing into the corridor within prediction_s constrain speed even
    before entering it. This conservative finite-horizon brake target is not a
    general collision-free motion planner or a calibrated stopping guarantee.
    """
    for key,value in list(locals().items()):
        if key not in ('ego','base22','objects'):number(value,key)
    if (cruise_speed_m_s<0 or prediction_s<=0 or planning_deceleration_m_s2<=0
            or min(reaction_s,margin_m,lateral_margin_m)<0 or min(ego_half_length_m,ego_half_width_m)<=0):
        raise ValueError('Invalid safe-speed parameters')
    obs=hazard_observation(ego,base22,objects,now_s=now_s,max_age_s=max_age_s)
    if not obs.healthy:return 0.,dict(reason='stale_object_frame',object_ids=obs.stale_ids)
    target=float(cruise_speed_m_s); limiting=[]
    for k,identity in enumerate(obs.selected_ids):
        v=obs.vector[22+k*10:32+k*10]
        _,x,y,vx,vy,hl,hw,sn,cs,age=v
        ex=abs(cs)*hl+abs(sn)*hw; ey=abs(sn)*hl+abs(cs)*hw
        corridor=ego_half_width_m+lateral_margin_m+ey
        if abs(vy)<1e-12:
            if abs(y)>corridor:continue
            enter,leave=0.,prediction_s
        else:
            a,b=sorted(((-corridor-y)/vy,(corridor-y)/vy))
            enter,leave=max(0.,a),min(prediction_s,b)
            if enter>leave:continue
        # Ignore only an object whose entire predicted longitudinal span stays
        # behind our rear. Otherwise use the smallest current/predicted gap.
        positions=(x,x+vx*enter)
        if max(positions)+ex < -ego_half_length_m:continue
        gap=min(positions)-ex-ego_half_length_m-margin_m
        available=max(0.,gap)
        dec=planning_deceleration_m_s2
        speed=max(0.,math.sqrt((dec*reaction_s)**2+2*dec*available)-dec*reaction_s)
        if speed<target:target=speed;limiting=[identity]
        elif speed==target and speed<cruise_speed_m_s:limiting.append(identity)
    return target,dict(reason='object_braking_target' if limiting else 'clear',object_ids=tuple(limiting))


class HazardLaneTask(LaneFollowingTask):
    """Compatible LaneFollowingTask extension; existing native physics guards.

    set_objects must receive a current measured frame before each observation.
    The native owner still latches guard failures at both 120 Hz substeps. Static
    success requires reaching a measured stop window, not stopping at reset.
    Crossing/clear stages require the unchanged 60 m drive-and-stop objective.
    """
    def __init__(self,route,config=None,dimensions_m=(4.8,1.8,1.4),*,stage='clear'):
        super().__init__(route,config,dimensions_m)
        self.stage='clear';self.set_stage(stage)
        self._objects=();self._objects_time=0.;self._static_goal=None
        self.last_hazard_observation=None;self.last_safe_target=None
        self.last_hazard_report={};self._crossing_encountered=False

    def set_stage(self,stage):
        if stage not in ('clear','static','crossing'):raise ValueError('Unknown stage')
        if getattr(self,'_initialized',False) and not self._terminal and stage!=self.stage:
            raise RuntimeError('Stage changes require an episode boundary')
        self.stage=stage

    def set_objects(self,objects,now_s):
        t=number(now_s,'now_s')
        values=tuple(objects)
        if (t<0 or len(values)>2 or any(not isinstance(o,MeasuredObject) for o in values)
                or self.stage!='clear' and not values):
            raise ValueError('Invalid object frame')
        self._objects,self._objects_time=values,t

    def static_stop_station(self,state=None):
        return self._static_goal if self.stage=='static' else None

    def current_safe_speed(self,state,tick):
        return self._relative(_state(state),tick).target_speed_m_s

    def _report(self,state,tick):
        ego=OrientedBox(state['position_m'][0],state['position_m'][1],state['yaw_rad'],4.8,1.8)
        clear=True;details=[]
        for obj in self._objects:
            projection=self.route.project(obj.x_m,obj.y_m)
            angle=obj.yaw_rad-projection.point.yaw_rad
            extent=abs(math.sin(angle))*obj.length_m/2+abs(math.cos(angle))*obj.width_m/2
            in_corridor=abs(projection.lateral_error_m)<=.9+.25+extent
            clear=clear and abs(projection.lateral_error_m)>self.route.width_m/2+extent+.25
            if self.stage=='crossing' and in_corridor:self._crossing_encountered=True
            geometry=box_clearance(ego,OrientedBox(obj.x_m,obj.y_m,obj.yaw_rad,obj.length_m,obj.width_m))
            details.append(dict(object_id=obj.object_id,station_m=projection.s_m,
                                lateral_m=projection.lateral_error_m,in_corridor=in_corridor,
                                outside_road=abs(projection.lateral_error_m)>self.route.width_m/2+extent+.25,
                                clearance_m=geometry['clearance_m'],overlap=geometry['overlap']))
        report=dict(stage=self.stage,tick=tick,objects=details,
                    minimum_clearance_m=min((d['clearance_m'] for d in details),default=None),
                    object_corridor_clear=clear,crossing_encountered=self._crossing_encountered,
                    static_stop_station_m=self.static_stop_station(),safe_target=self.last_safe_target,
                    observation_source=OBSERVATION_SOURCE)
        self.last_hazard_report=report
        return report

    def _frame(self,state,tick,relative=None):
        relative=super()._relative(state,tick) if relative is None else relative
        # Applied controls do not enter the geometric safe-target calculation.
        base=(relative.progress_m,relative.remaining_m,relative.lateral_error_m,
              math.sin(relative.heading_error_rad),math.cos(relative.heading_error_rad),
              relative.forward_speed_m_s,relative.lateral_speed_m_s,relative.planar_speed_m_s,
              0.,state['position_m'][2],state['upright_z'],0.,0.,0.,3.,relative.target_speed_m_s,
              0.,0.,0.,0.,0.,0.)
        ego=VehicleState.from_physics(state,episode_id='hazard-task',vehicle_id='ego',tick=tick)
        return ego,base

    def _relative(self,state,tick):
        relative=super()._relative(state,tick)
        ego,base=self._frame(state,tick,relative)
        safe,detail=safe_target_speed(ego,base,self._objects,now_s=tick/self.config.physics_hz)
        self.last_safe_target=detail
        target=self.config.target_speed_m_s
        remaining=relative.remaining_m
        if self.stage=='static' and self._static_goal is not None:
            remaining=self._static_goal-relative.progress_m
            target=min(self.config.target_speed_m_s,math.sqrt(2*self.config.planning_deceleration_m_s2*max(0.,remaining-self.config.stop_buffer_m)))
        if tick<round(self.config.settle_s*self.config.physics_hz):target=0.
        return replace(relative,remaining_m=remaining,target_speed_m_s=min(target,safe))

    def _observation(self,state,applied,tick,relative):
        base=super()._observation(state,applied,tick,relative)
        ego,_=self._frame(state,tick,relative)
        frame=hazard_observation(ego,base,self._objects,now_s=tick/self.config.physics_hz)
        if not frame.healthy or abs(self._objects_time-tick/self.config.physics_hz)>1/self.config.physics_hz+1e-9:
            raise ValueError('Missing/stale current object frame')
        self.last_hazard_observation=frame
        return frame.vector

    def initialize(self,state,applied,*,tick=0):
        self._static_goal=None
        self._crossing_encountered=False
        if self.stage=='static':
            candidates=[]
            for obj in self._objects:
                projection=self.route.project(obj.x_m,obj.y_m)
                angle=obj.yaw_rad-projection.point.yaw_rad
                lateral_extent=abs(math.sin(angle))*obj.length_m/2+abs(math.cos(angle))*obj.width_m/2
                if abs(projection.lateral_error_m)<=.9+.25+lateral_extent:
                    extent=abs(math.cos(angle))*obj.length_m/2+abs(math.sin(angle))*obj.width_m/2
                    candidates.append(projection.s_m-extent-2.4-2.)
            if not candidates or not 5.<=min(candidates)<=self.config.stop_s_m-1.:
                raise ValueError('Static stage requires a forward measured stop window')
            self._static_goal=min(candidates)
        elif self.stage=='crossing' and not self._objects:
            raise ValueError('Crossing stage requires a measured offroad or crossing object')
        result=super().initialize(state,applied,tick=tick)
        self._report(_state(state),tick)
        return result

    def advance(self,state,applied,*,tick,contact_count=0,command_fallback=False,failure_reason=None):
        if not self._initialized or self._terminal:raise RuntimeError('Reset required')
        _integer(tick,'tick')
        cfg=self.config
        if tick!=self._tick+cfg.control_ticks:raise ValueError('Exactly one control interval required')
        if failure_reason is not None and (not isinstance(failure_reason,str) or not failure_reason):
            raise ValueError('Invalid guard reason')
        state,applied=_state(state),_control(applied)
        relative=self._relative(state,tick)
        hazard=self._report(state,tick)
        reason=failure_reason or self.physics_guard(state,tick=tick,contact_count=contact_count,command_fallback=command_fallback)
        dt=cfg.control_ticks/cfg.physics_hz
        delta=relative.progress_m-self._previous_relative.progress_m
        discontinuity=math.dist(state['position_m'][:2],self._previous_state['position_m'][:2])>cfg.speed_max_m_s*dt+.005
        if discontinuity and reason is None:reason='state_discontinuity'
        goal=self._static_goal if self.stage=='static' else cfg.stop_s_m
        drive=self._tick>=round(cfg.settle_s*cfg.physics_hz)
        def at_goal(r):
            return abs(goal-r.progress_m)<=cfg.endpoint_tolerance_m and r.progress_m-self._start_s_m>=goal-self._start_s_m-cfg.endpoint_tolerance_m and r.planar_speed_m_s<=cfg.stop_speed_m_s
        self._hold_ticks=self._hold_ticks+cfg.control_ticks if self.stage=='static' and drive and reason is None and at_goal(relative) and at_goal(self._previous_relative) else 0
        reached=(self._hold_ticks>=round(cfg.success_hold_s*cfg.physics_hz) if self.stage=='static'
                 else drive and relative.progress_m>=cfg.stop_s_m and relative.progress_m-self._start_s_m>=cfg.stop_s_m-.05
                 and (self.stage=='clear' or hazard['crossing_encountered'] and hazard['object_corridor_clear']))
        success=reason is None and reached
        terminated=reason is not None or success
        truncated=not terminated and tick>=round(cfg.episode_length_s*cfg.physics_hz)
        reason='success' if success else reason or ('time_limit' if truncated else 'driving' if drive else 'settling')
        parts=dict(progress=0.,lateral=0.,heading=0.,speed=0.,steering_rate=0.,time=0.,terminal=0.)
        if drive:
            parts.update(progress=0. if discontinuity else delta/3.,lateral=-dt*(relative.lateral_error_m/.5)**2,
                         heading=-.25*dt*(relative.heading_error_rad/.25)**2,
                         speed=-.25*dt*((relative.forward_speed_m_s-relative.target_speed_m_s)/3.)**2,
                         steering_rate=-.01*dt*((applied[0]-self._previous_control[0])/(dt*.5))**2,time=-.05*dt)
        parts['terminal']=10. if success else -10. if terminated else 0.
        obs=self._observation(state,applied,tick,relative)
        self._previous_state,self._previous_control,self._previous_relative=state,applied,relative
        self._tick,self._terminal=tick,terminated or truncated
        return LaneTransition(tick,tick/cfg.physics_hz,obs,relative,parts,sum(parts.values()),terminated,truncated,reason,success,obs if self._terminal else None)
