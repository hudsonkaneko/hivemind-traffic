"""Opt-in v09 live traffic state. No USD, native physics, or wall-clock inputs."""
from dataclasses import replace
import math

from experiments.showcase_controls import validate_settings
from traffic.loop_showcase_control import ShowcaseController, signed_station_gap
from traffic.bypass_validation import OrientedBox
from traffic.showcase_clearance import nearest_clearance
from usd.showcase_fleet import (FleetSpec, fleet_pose, WHEEL_RADIUS_M,
                                BASE_RADIUS_M, LANE_WIDTH_M, LENGTH_M, WIDTH_M)


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
    # Base-circle station metres, not bumper distance. At the configured maximum
    # 25 mph and 3 s gap, 25*.44704*3 + 4.8 m body + .8 m clearance = 39.128 m.
    # Outer-lane arc spacing scales with speed, so 40 m covers every lane and
    # future ramp value. This is entry spacing, not reactive background driving.
    entry_min_station_gap_m = 40.
    entry_min_clearance_m = .8
    circumference_m = math.tau * BASE_RADIUS_M

    def __init__(self, specs, settings):
        self.settings = validate_settings(settings)
        self.specs = {s['vehicle_id']: FleetSpec(**s) for s in specs}
        if len(self.specs) != self.capacity:raise ValueError('Live pool requires exactly 80 slots')
        keys = list(self.specs)
        count = settings['traffic_count']
        self.active = {keys[(i*self.capacity)//count] for i in range(count)}
        self.generations = {key: int(key in self.active) for key in keys}
        # Runtime-only phase opinions; immutable asset/layout specs stay intact.
        # An offset changes only while its slot is dormant and entering.
        self.phase_offsets_m = {key: 0. for key in keys}
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

    def station(self, key):
        """Effective slot phase on the base circle, including retired placements."""
        return (self.specs[key].station_m + self.phase_offsets_m[key]
                + self.distance_m) % self.circumference_m

    @staticmethod
    def _box(lane, station):
        radius = BASE_RADIUS_M + lane * LANE_WIDTH_M
        angle = station / BASE_RADIUS_M
        return OrientedBox(radius * math.cos(angle), radius * math.sin(angle),
                           angle + math.pi / 2, LENGTH_M, WIDTH_M)

    def _active_geometry(self, exclude=None):
        return [(spec.lane, self.station(key), self._box(spec.lane, self.station(key)))
                for key, spec in self.specs.items() if key in self.active and key != exclude]

    def _placement_geometry(self, lane, station, active_geometry):
        """Exact clearance of analytic current target boxes, not native readback."""
        front = rear = math.inf
        for other_lane, other_station, _ in active_geometry:
            if other_lane == lane:
                front = min(front, (other_station - station) % self.circumference_m)
                rear = min(rear, (station - other_station) % self.circumference_m)
        clearance = (nearest_clearance(self._box(lane, station),
                     [box for _, _, box in active_geometry])['clearance_m']
                     if active_geometry else None)
        return dict(nearest_front_station_gap_m=None if math.isinf(front) else front,
                    nearest_rear_station_gap_m=None if math.isinf(rear) else rear,
                    minimum_background_clearance_m=clearance)

    def _entry_geometry(self, key, station, active_geometry):
        # Cheap same-lane rejection precedes exact polygon work. Every candidate,
        # including an unchanged phase, is checked against current placements.
        lane = self.specs[key].lane
        for other_lane, other_station, _ in active_geometry:
            if (other_lane == lane and
                    abs(signed_station_gap(station, other_station)) < self.entry_min_station_gap_m):
                return None
        evidence = self._placement_geometry(lane, station, active_geometry)
        clearance = evidence['minimum_background_clearance_m']
        if clearance is not None and clearance < self.entry_min_clearance_m:
            return None
        return evidence

    def _entry_placement(self, ego_station):
        active_geometry = self._active_geometry()
        dormant = [key for key in self.specs if key not in self.active]
        priority = lambda key: (abs(signed_station_gap(ego_station, self.station(key))), key)
        # Preserve the existing furthest-safe-slot priority whenever its current
        # phase is admissible; relocated old phases get no exemption from checks.
        for key in sorted(dormant, key=priority, reverse=True):
            station = self.station(key)
            if abs(signed_station_gap(ego_station, station)) < self.exclusion_m:
                continue
            evidence = self._entry_geometry(key, station, active_geometry)
            if evidence is not None:
                return key, station, 'existing_phase', evidence
        # At most one representative dormant slot per lane and one midpoint per
        # occupied same-lane gap. Empty lanes use the remote antipode. Together
        # this adds at most capacity candidates, with deterministic tie breaks.
        candidates = []
        for lane in sorted({self.specs[key].lane for key in dormant}):
            key = max((key for key in dormant if self.specs[key].lane == lane), key=priority)
            stations = sorted(station for other_lane, station, _ in active_geometry if other_lane == lane)
            if stations:
                gaps = [(a, b) for a, b in zip(stations, stations[1:] + [stations[0] + self.circumference_m])
                        if b - a >= 2 * self.entry_min_station_gap_m]
                placements = [((a + b) / 2 % self.circumference_m, (b - a) / 2,
                               'remote_gap_midpoint') for a, b in gaps]
            else:
                placements = [((ego_station + self.circumference_m / 2) % self.circumference_m,
                               self.circumference_m / 2, 'empty_lane_opposite')]
            for station, half_gap, method in placements:
                ego_gap = abs(signed_station_gap(ego_station, station))
                if ego_gap >= self.exclusion_m:
                    candidates.append((half_gap, ego_gap, key, station, method))
        for _, _, key, station, method in sorted(candidates, reverse=True):
            evidence = self._entry_geometry(key, station, active_geometry)
            if evidence is not None:
                return key, station, method, evidence
        return None

    def pose(self, key):
        spec = self.specs[key]
        if key not in self.active:
            i = list(self.specs).index(key)
            return dict(position_m=(10000.+i*10.,10000.,20.), yaw_rad=0.,
                        quaternion_xyzw=(0.,0.,0.,1.), velocity_m_s=(0.,0.,0.),wheel_spin_deg=0.)
        scale = (500+spec.lane*3.7)/500
        current = replace(spec,station_m=spec.station_m+self.phase_offsets_m[key]+self.distance_m,
                          speed_m_s=self.speed_m_s*scale)
        pose = fleet_pose(current,0.)
        pose['wheel_spin_deg']=math.degrees(self.distance_m*scale/WHEEL_RADIUS_M)%360
        return pose

    def reconcile(self, tick, ego_xy, *, maneuver=False):
        """One deterministic, remote membership change at a safe tick boundary.

        Failed admission searches also wait 30 ticks before retrying. The search
        inspects at most 80 retained phases plus 80 same-lane gap candidates; it
        never scans at physics frequency or changes a live vehicle's phase.
        """
        if self.stopping or maneuver or tick < self._next_transition:
            return []
        target = self.settings['traffic_count']
        if target == len(self.active):
            return []
        ego_station = math.atan2(ego_xy[1], ego_xy[0]) * BASE_RADIUS_M
        if not math.isfinite(ego_station):
            raise ValueError('Finite ego position required for live membership')
        self._next_transition = tick + self.transition_interval_ticks
        adding = target > len(self.active)
        if adding:
            placement = self._entry_placement(ego_station)
            if placement is None:
                return []
            key, station, method, evidence = placement
        else:
            candidates = [(abs(signed_station_gap(ego_station, self.station(key))), key)
                          for key in self.specs if key in self.active
                          and abs(signed_station_gap(ego_station, self.station(key))) >= self.exclusion_m]
            if not candidates:
                return []
            _, key = max(candidates)
            station, method = self.station(key), 'remote_exit'
            evidence = self._placement_geometry(self.specs[key].lane, station, self._active_geometry(exclude=key))
        if len(self.events) >= 2048:
            raise RuntimeError('Live fleet lifecycle log limit reached')
        old_phase = self.phase_offsets_m[key]
        if adding:
            if method != 'existing_phase':
                self.phase_offsets_m[key] = signed_station_gap(self.specs[key].station_m + self.distance_m, station)
            self.generations[key] += 1
            self.active.add(key)
        else:
            self.active.remove(key)
        event = dict(tick=tick, type='enter' if adding else 'exit', slot=key,
                     vehicle_id=self.identity(key), active_count=len(self.active),
                     ego_exclusion_m=self.exclusion_m,
                     actual_arc_gap_m=abs(signed_station_gap(ego_station, station)),
                     actual_station_m=station, placement_method=method,
                     old_phase_offset_m=old_phase, new_phase_offset_m=self.phase_offsets_m[key],
                     entry_required_station_gap_m=self.entry_min_station_gap_m if adding else None,
                     entry_required_clearance_m=self.entry_min_clearance_m if adding else None,
                     placement_geometry_source='analytic current target footprints; native audit remains separate',
                     **evidence)
        self.events.append(event)
        self.pending_lifecycle.append(event)
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
