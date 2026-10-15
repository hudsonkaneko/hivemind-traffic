"""Opt-in v09 live traffic state. No USD, native physics, or wall-clock inputs."""
from dataclasses import replace
import math

from experiments.showcase_controls import validate_settings
from traffic.loop_showcase_control import ShowcaseController, signed_station_gap
from usd.showcase_fleet import FleetSpec, fleet_pose, WHEEL_RADIUS_M


def approach(value, target, maximum_delta):
    if abs(target-value)<=maximum_delta:return target
    return value+math.copysign(maximum_delta,target-value)


def assess_live_smoke(rows, events, stages, restarted):
    """Prove native-model requests actually reached the intended populations."""
    checkpoints=((34.,12),(64.,40),(89.,12))
    counts=[]
    for time_s,_ in checkpoints:
        row=next((r for r in rows if r['sim_time_s']>=time_s),None)
        counts.append(None if row is None else row.get('active_background_count'))
    passed=(stages==3 and not restarted and counts==[n for _,n in checkpoints]
            and {'enter','exit'}<={e['type'] for e in events})
    return dict(passed=passed,checkpoint_seconds=[t for t,_ in checkpoints],
                actual_counts=counts,expected_counts=[n for _,n in checkpoints])


class LiveTraffic:
    """Fixed capacity, changing membership; constant angular spacing survives ramps.

    Inactive colliders park outside the road, never just become invisible on it.
    A re-entry gets a new logical identity while its reusable USD slot stays fixed.
    """
    capacity = 80
    transition_interval_ticks = 30  # at most four vehicles/s at 120 Hz
    exclusion_m = 160.

    def __init__(self, specs, settings):
        self.settings = validate_settings(settings)
        self.specs = {s['vehicle_id']: FleetSpec(**s) for s in specs}
        if len(self.specs) != self.capacity:raise ValueError('Live pool requires exactly 80 slots')
        keys = list(self.specs)
        count = settings['traffic_count']
        self.active = {keys[(i*self.capacity)//count] for i in range(count)}
        self.generations = {key: int(key in self.active) for key in keys}
        self.distance_m = 0.
        self.speed_m_s = settings['traffic_mph']*.44704
        self.ego_target_m_s = settings['ego_mph']*.44704
        self.following_s = settings['following_s']
        self.time_s = 0.
        self.stopping = False
        self.events = []
        self.pending_lifecycle = []
        self._next_transition = 0
        self.ramping = False

    def identity(self, key):return f'{key}_g{self.generations[key]:03d}'

    def configure(self, settings):self.settings = validate_settings(settings)

    def pose(self, key):
        spec = self.specs[key]
        if key not in self.active:
            i = list(self.specs).index(key)
            return dict(position_m=(10000.+i*10.,10000.,20.), yaw_rad=0.,
                        quaternion_xyzw=(0.,0.,0.,1.), velocity_m_s=(0.,0.,0.),wheel_spin_deg=0.)
        scale = (500+spec.lane*3.7)/500
        current = replace(spec,station_m=spec.station_m+self.distance_m,speed_m_s=self.speed_m_s*scale)
        pose = fleet_pose(current,0.)
        pose['wheel_spin_deg']=math.degrees(self.distance_m*scale/WHEEL_RADIUS_M)%360
        return pose

    def reconcile(self, tick, ego_xy, *, maneuver=False):
        """One deterministic, remote membership change at a safe tick boundary."""
        if self.stopping or maneuver or tick<self._next_transition:return []
        target=self.settings['traffic_count']
        if target==len(self.active):return []
        ego_station=math.atan2(ego_xy[1],ego_xy[0])*500
        adding=target>len(self.active)
        candidates=[]
        for key,spec in self.specs.items():
            if (key in self.active)==adding:continue
            station=spec.station_m+self.distance_m
            gap=abs(signed_station_gap(ego_station,station))
            if gap>=self.exclusion_m:candidates.append((gap,key))
        if not candidates:return []
        if len(self.events)>=2048:raise RuntimeError('Live fleet lifecycle log limit reached')
        _,key=max(candidates)
        if adding:
            self.generations[key]+=1
            self.active.add(key)
        else:self.active.remove(key)
        event=dict(tick=tick,type='enter' if adding else 'exit',slot=key,
                   vehicle_id=self.identity(key),active_count=len(self.active),
                   ego_exclusion_m=self.exclusion_m)
        self.events.append(event);self.pending_lifecycle.append(event)
        self._next_transition=tick+self.transition_interval_ticks
        return [event]

    def prepare_control(self, dt_s, *, maneuver=False):
        # Freeze tuning through committed maneuvers. New lane changes are held
        # while background speed changes; following/braking remain active.
        self.ramping=(not maneuver and not self.stopping and
                      abs(self.speed_m_s-self.settings['traffic_mph']*.44704)>1e-8)
        if not maneuver and not self.stopping:
            self.ego_target_m_s=approach(self.ego_target_m_s,self.settings['ego_mph']*.44704,dt_s)
            self.following_s=approach(self.following_s,self.settings['following_s'],.25*dt_s)

    def advance(self, time_s, *, maneuver=False):
        dt=time_s-self.time_s
        if not math.isfinite(dt) or not 0 < dt <= 1/120+1e-8:
            raise ValueError('Live fleet requires consecutive 120 Hz steps')
        target=0. if self.stopping else self.settings['traffic_mph']*.44704
        old=self.speed_m_s
        if self.stopping or not maneuver:
            self.speed_m_s=approach(old,target,(3. if self.stopping else .5)*dt)
        self.distance_m+=(old+self.speed_m_s)*.5*dt
        self.time_s=time_s


class LiveShowcaseController(ShowcaseController):
    """Explicit membership admission; unexpected dropped tracks still fail closed."""
    changing_traffic_speed=False
    traffic_transition_pending=False

    def accept_lifecycle(self, tracks, events):
        if not events:return
        expected=set(self._expected_ids or (t.vehicle_id for t in tracks))
        for event in events:
            identity=event['vehicle_id']
            if event['type']=='enter':expected.add(identity)
            elif event['type']=='exit':expected.discard(identity)
            else:raise ValueError('Unknown lifecycle event')
        if expected!={t.vehicle_id for t in tracks}:raise RuntimeError('Unannounced fleet membership change')
        if self._previous_tracks is not None:
            self._previous_tracks={t.vehicle_id:self._previous_tracks.get(t.vehicle_id,t) for t in tracks}
        self._expected_ids=expected

    def _candidate_safe(self,*args,**kwargs):
        return False if self.changing_traffic_speed or self.traffic_transition_pending else super()._candidate_safe(*args,**kwargs)
