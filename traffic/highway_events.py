"""Deterministic optional disruption schedules; independent of PeMS and renderer.

This first adapter exposes localized restrictions to a traffic controller. It does
not close an entire SUMO edge as a shortcut for a short spatial work zone.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class HighwayEvent:
    event_id: str
    kind: str
    onset_s: float
    end_s: float
    start_station_m: float
    end_station_m: float
    lane_ids: tuple[str, ...]
    speed_m_s: float = 0.
    sweep_m: float = 0.
    sweep_period_s: float = 8.
    provenance: str = 'synthetic'

    def __post_init__(self):
        values=(self.onset_s,self.end_s,self.start_station_m,self.end_station_m,
                self.speed_m_s,self.sweep_m,self.sweep_period_s)
        if not all(math.isfinite(v) for v in values):raise ValueError('Finite event parameters required')
        if self.kind not in ('construction','disabled_vehicle','police_traffic_break'):
            raise ValueError('Unsupported event type')
        if not self.event_id or not self.lane_ids or len(set(self.lane_ids))!=len(self.lane_ids):
            raise ValueError('Stable event and unique affected lane IDs required')
        if self.onset_s<0 or self.end_s<=self.onset_s or self.start_station_m<0 or self.end_station_m<self.start_station_m:
            raise ValueError('Invalid time/station interval')
        if min(self.speed_m_s,self.sweep_m)<0 or self.sweep_period_s<=0:raise ValueError('Invalid motion')
        if self.kind!='police_traffic_break' and (self.speed_m_s or self.sweep_m):raise ValueError('Stationary event cannot move')
        if self.provenance not in ('synthetic','observed','reconstructed'):raise ValueError('Provenance required')

    def state(self, time_s):
        if not math.isfinite(time_s) or time_s<0:raise ValueError('Nonnegative finite simulation time required')
        active=self.onset_s<=time_s<self.end_s
        elapsed=max(0.,min(time_s,self.end_s)-self.onset_s)
        station=min(self.end_station_m,self.start_station_m+self.speed_m_s*elapsed)
        lateral=self.sweep_m*math.sin(2*math.pi*elapsed/self.sweep_period_s) if active else 0.
        lateral_speed=self.sweep_m*2*math.pi/self.sweep_period_s*math.cos(2*math.pi*elapsed/self.sweep_period_s) if active else 0.
        return dict(event_id=self.event_id,kind=self.kind,active=active,station_m=station,
                    lateral_m=lateral,lateral_speed_m_s=lateral_speed,affected_lanes=list(self.lane_ids) if active else [],
                    footprint_end_m=self.end_station_m if self.kind=='construction' else station+5.,
                    obstacle_speed_m_s=self.speed_m_s if active and station<self.end_station_m else 0.,
                    provenance=self.provenance)


def active_restrictions(events,time_s,enabled=True):
    """Pure reset/seek: overlapping events remain distinct and independently clear."""
    if len({e.event_id for e in events})!=len(events):raise ValueError('Duplicate event identity')
    states=[e.state(time_s) for e in events]
    return [s for s in states if s['active']] if enabled else []


def restrictive_speed_cap(states,lane_id,station_m,speed_m_s,*,braking_m_s2=2.,margin_m=8.):
    """Synthetic preview driver bound, not a validated safety or human-behavior model.

Respects each event's longitudinal footprint. A controller adapter can consume the
same restriction records; no simulator-wide hidden schedule is an actor input.
"""
    if not all(math.isfinite(x) for x in (station_m,speed_m_s,braking_m_s2,margin_m)) or speed_m_s<0 or braking_m_s2<=0 or margin_m<0:
        raise ValueError('Invalid preview driver state')
    cap=speed_m_s
    for state in states:
        if not state['active'] or lane_id not in state['affected_lanes']:continue
        if station_m>state['footprint_end_m']:continue
        gap=max(0.,state['station_m']-station_m-margin_m)
        cap=min(cap,math.sqrt(state['obstacle_speed_m_s']**2+2*braking_m_s2*gap))
    return cap


def development_events():
    """Explicit synthetic presets; not reconstructions of recorded LA incidents."""
    return {
        'Off':(),
        'Construction':(HighwayEvent('work_zone_demo','construction',10,65,470,530,('sb_right',)),),
        'Disabled vehicle':(HighwayEvent('disabled_demo','disabled_vehicle',10,65,470,470,('sb_right',)),),
        'Police traffic break':(HighwayEvent('traffic_break_demo','police_traffic_break',10,65,180,530,
                                            ('sb_right','sb_middle','sb_left'),speed_m_s=5.,sweep_m=4.),)
    }


def preview_trajectory(events, duration_s=80., dt=.1, desired_speed=8.):
    """One scripted kinematic inspection car. No learning or traffic calibration."""
    if duration_s<=0 or dt<=0 or desired_speed<=0:raise ValueError('Positive preview parameters required')
    station=60.;speed=desired_speed;trace=[]
    for step in range(int(round(duration_s/dt))+1):
        t=step*dt;states=active_restrictions(events,t)
        trace.append(dict(time=t,station_m=station,speed=speed))
        cap=restrictive_speed_cap(states,'sb_right',station,desired_speed)
        next_speed=max(0.,min(cap,speed+dt))
        # This is a kinematic inspection constraint, not an actuator response claim.
        station+=next_speed*dt;speed=next_speed
    return trace
