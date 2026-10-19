"""Opt-in v09 live traffic state. No USD, native physics, or wall-clock inputs."""
from dataclasses import replace
import json
import math

from experiments.showcase_controls import validate_settings
from traffic.loop_showcase_control import ShowcaseController, RadialPath, signed_station_gap
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
    """Live membership and bounded, measured straight-lane gap creation.

    The extra speed cap never changes the full-cruise merge forecast. Its rear
    certificate covers a stationary ego for the entire attempt, including
    recovery; it does not assume a guaranteed physical acceleration response.
    Losing that certificate terminates this bounded simulator experiment.
    """
    changing_traffic_speed=False
    traffic_transition_pending=False
    gap_profile = dict(dwell_s=2., maximum_attempt_s=21., maximum_slow_s=11.,
        maximum_added_lag_m=50., release_lag_m=34., minimum_speed_m_s=2.5,
        maximum_speed_deficit_m_s=4.5, target_deceleration_m_s2=1.,
        minimum_traffic_speed_m_s=5., maximum_lateral_error_m=.2,
        maximum_heading_error_rad=.04, maximum_attempts=3,
        maximum_preflight_queries=12)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._gap_attempt = None
        self._gap_attempts = 0
        self._gap_examined = set()
        self._gap_dwell = None
        self._gap_lifecycle = 0
        self._gap_last_event = None
        self._gap_state = 'idle'
        self._gap_reason = 'not_blocked'
        self._gap_last = {}

    def _gap_event(self, state, reason, tick):
        self._gap_state, self._gap_reason = state, reason
        self._gap_last_event = dict(tick=tick, state=state, reason=reason,
                                    attempt=self._gap_attempts)
        if self._gap_attempt is not None:
            self._gap_last_event.update(start_tick=self._gap_attempt['start_tick'],
                deadline_tick=self._gap_attempt['deadline_tick'], lead_vehicle_id=self._gap_attempt['lead_id'])

    def _gap_failure(self, reason, tick):
        self._gap_event('failed', reason, tick)
        self.diagnostics['gap_creation'] = self._gap_diagnostics()
        # The runner persists repr(error) and traceback even when command() never
        # returns its last diagnostics. Keep the failure certificate in that path.
        try:
            certificate = json.dumps(self.diagnostics['gap_creation'], allow_nan=False, separators=(',', ':'))
        except (TypeError, ValueError):
            # Diagnostic formatting must never mask the original safety failure.
            certificate = json.dumps(dict(state='failed', reason=reason,
                diagnostic_error='nonfinite_or_unserializable_certificate'), separators=(',', ':'))
        raise RuntimeError('Live gap-creation certificate failed: ' + reason + '; certificate=' + certificate)

    def _gap_diagnostics(self):
        attempt = self._gap_attempt
        return dict(self._gap_last, state=self._gap_state, reason=self._gap_reason,
            attempts=self._gap_attempts, profile=dict(self.gap_profile),
            start_tick=None if attempt is None else attempt['start_tick'],
            deadline_tick=None if attempt is None else attempt['deadline_tick'],
            cap_target_m_s=(attempt['cap'] if attempt and self._gap_state == 'slowing' else None),
            last_event=self._gap_last_event)

    def _gap_geometry(self, state, tracks, remaining_s):
        """Conservative straight-lane corridor and stationary-ego rear window.

        All quantities below are base-circle station metres. Using physical body
        lengths without outer-lane scaling is conservative. The speed bound also
        covers the background circle's allowed .1 m radial readback tolerance.
        Adjacent lanes are still checked by the unchanged merge forecast.
        """
        cfg, limits = self.config, self.gap_profile
        radius = cfg.radius_m + self.current_lane * cfg.lane_width_m
        heading = signed_station_gap(0., state.yaw_rad - math.atan2(state.y_m, state.x_m)
                                     - math.pi / 2, radius_m=1.)
        if (abs(math.hypot(state.x_m, state.y_m) - radius) > limits['maximum_lateral_error_m']
                or abs(heading) > limits['maximum_heading_error_rad']):
            return dict(eligible=False, reason='straight_lane_tracking', minimum_rear_reserve_m=None,
                rear_vehicle_id=None, rear_center_gap_m=None, required_rear_center_gap_m=None,
                base_tracking_violation=(abs(math.hypot(state.x_m, state.y_m) - radius) > cfg.max_tracking_error_m
                                         or abs(heading) > cfg.max_heading_error_rad))
        curve_pad = max(.02, (cfg.ego_length_m / 2 + abs(cfg.ego_collision_offset_x_m)) ** 2
                        / (2 * (cfg.radius_m - cfg.lane_width_m / 2)))
        half = (cfg.ego_width_m / 2
                + (cfg.ego_length_m / 2 + abs(cfg.ego_collision_offset_x_m))
                  * math.sin(limits['maximum_heading_error_rad'])
                + limits['maximum_lateral_error_m'] + cfg.prediction_tracking_margin_m + curve_pad)
        minimum = math.inf
        maximum_rear_base_speed = 0.
        nearest = None
        nearest_gap = required_at_nearest = None
        for obj in tracks:
            obj_radius = math.hypot(obj.x_m, obj.y_m)
            # The .02 m term also covers the radial projection of a straight
            # finite body tangent to the curved circle (including 8 m tracks).
            other_half = obj.width_m / 2 + obj.length_m / 2 * math.sin(.05) + .02 + .1
            # Canonical circle plus the complete permitted radial tolerance,
            # rather than assuming the currently measured radial error persists.
            obj_lane_radius = cfg.radius_m + round((obj_radius - cfg.radius_m) / cfg.lane_width_m) * cfg.lane_width_m
            if abs(obj_lane_radius - radius) > half + other_half + cfg.prediction_clearance_m:
                continue
            gap = signed_station_gap(self._unwrapped_s, self._station(obj.x_m, obj.y_m))
            if gap >= 0:
                continue
            maximum_rear_base_speed = max(maximum_rear_base_speed, obj.speed_m_s * cfg.radius_m / obj_radius)
            padding = ((cfg.ego_length_m + obj.length_m) / 2 + abs(cfg.ego_collision_offset_x_m)
                + cfg.prediction_clearance_m + cfg.prediction_tracking_margin_m
                + (cfg.max_observed_speed_m_s + obj.speed_m_s) * cfg.prediction_step_s / 2
                + cfg.progress_slack_m)
            maximum_base_speed = cfg.max_observed_speed_m_s * cfg.radius_m / (cfg.radius_m - .1)
            required = maximum_base_speed * remaining_s + padding
            reserve = -gap - required
            if reserve < minimum:
                minimum, nearest, nearest_gap, required_at_nearest = reserve, obj.vehicle_id, -gap, required
        return dict(eligible=minimum >= 0., reason='rear_window' if minimum < 0. else 'certified',
            base_tracking_violation=False, maximum_rear_base_speed_m_s=maximum_rear_base_speed,
            minimum_rear_reserve_m=None if math.isinf(minimum) else minimum,
            rear_vehicle_id=nearest, rear_center_gap_m=nearest_gap,
            required_rear_center_gap_m=required_at_nearest)

    def _gap_opening(self, state, tracks):
        """At most twelve snapshot queries; never a native trajectory prediction."""
        cfg = self.config
        probe = ShowcaseController(self.episode_id, self.vehicle_id, cfg,
                                   initial_lane=self.current_lane, optimized=self.optimized)
        queries = 0
        for lag in (24., 30., 34.):
            angle = -lag / cfg.radius_m
            c, s = math.cos(angle), math.sin(angle)
            for increment in (2., 4., 6., 8.):
                speed = min(cfg.target_speed_m_s, state.speed_m_s + increment)
                counter = replace(state, x_m=c * state.x_m - s * state.y_m,
                    y_m=s * state.x_m + c * state.y_m, yaw_rad=state.yaw_rad + angle,
                    speed_m_s=speed)
                probe._unwrapped_s = self._unwrapped_s - lag
                probe._maneuver = False
                probe.path = RadialPath(self.current_lane, self.current_lane,
                                        probe._unwrapped_s, cfg.lane_change_length_m)
                queries += 1
                reason = probe._plan(counter, state.tick, tracks)
                if reason == 'safe_adjacent_lane_selected' and probe._target_speed >= speed:
                    return dict(possible=True, queries=queries, extra_lag_m=lag,
                                recovery_speed_m_s=speed, lane=probe.path.to_lane)
        return dict(possible=False, queries=queries)

    def _gap_guard(self, state, tick, tracks):
        attempt = self._gap_attempt
        original = attempt['config']
        if replace(self.config, target_speed_m_s=original.target_speed_m_s,
                   desired_time_gap_s=original.desired_time_gap_s) != original:
            self._gap_failure('configuration_contract_changed', tick)
        elapsed_s = (tick - attempt['start_tick']) / self.config.physics_hz
        lead = next((obj for obj in tracks if obj.vehicle_id == attempt['lead_id']), None)
        if lead is not None and attempt['reference_mode'] == 'measured_lead_progress':
            station = self._station(lead.x_m, lead.y_m)
            attempt['reference_progress'] += signed_station_gap(attempt['reference_station'], station)
            attempt['reference_station'] = station
        else:
            # A removed lead cannot supply measured progress. Keep a conservative
            # upper lag budget until recovery, without fabricating a replacement ID.
            attempt['reference_mode'] = 'removed_lead_maximum_speed_bound'
            attempt['reference_progress'] += (self.config.max_observed_speed_m_s * self.config.radius_m
                / (self.config.radius_m - .1) * (tick - attempt['reference_tick']) / self.config.physics_hz)
        attempt['reference_tick'] = tick
        lag = attempt['reference_progress'] - (self._unwrapped_s - attempt['start_station'])
        evidence = self._gap_geometry(state, tracks, max(0., self.gap_profile['maximum_attempt_s'] - elapsed_s))
        self._gap_last.update(added_lag_m=lag if math.isfinite(lag) else None, elapsed_s=elapsed_s,
            measured_speed_m_s=state.speed_m_s, lead_vehicle_id=attempt['lead_id'],
            lag_reference=attempt['reference_mode'], **evidence)
        # The base command performs its tracking check after _plan. Preserve its
        # established emergency result when the wider base envelope is violated.
        if not evidence['eligible'] and not evidence.get('base_tracking_violation'):
            self._gap_failure(evidence['reason'], tick)
        if not math.isfinite(lag) or lag > self.gap_profile['maximum_added_lag_m']:
            self._gap_failure('added_lag_budget', tick)
        return elapsed_s, lag

    def _gap_finish(self, reason, tick):
        self._gap_event('complete', reason, tick)
        self._gap_attempt = None
        self._gap_dwell = None

    def _plan(self, state, tick, tracks, geometry=None):
        # Full requested cruise, front-following rule and ordinary candidate
        # acceptance always run first. Only their longitudinal target may be capped.
        reason = super()._plan(state, tick, tracks, geometry)
        limits = self.gap_profile
        if self._gap_attempt is not None:
            attempt = self._gap_attempt
            elapsed, lag = self._gap_guard(state, tick, tracks)
            if self._gap_last.get('base_tracking_violation'):
                # Do not clear the attempt before the base command's later
                # tracking check: its emergency must still terminate this run.
                return reason
            if self._maneuver:
                self._gap_finish('ordinary_maneuver_accepted', tick)
                return reason
            changed = (self.config != attempt['config'] or self._gap_lifecycle != attempt['lifecycle']
                       or self.changing_traffic_speed or self.traffic_transition_pending)
            if self._gap_state == 'slowing' and (changed or reason != 'blocked_following'
                    or lag >= limits['release_lag_m'] or elapsed >= limits['maximum_slow_s']):
                self._gap_event('recovering', 'controls_or_membership_changed' if changed
                    else 'front_safety' if reason == 'unsafe_closing_gap_brake' else 'cap_released', tick)
            if self._gap_state == 'slowing':
                dt = (tick - attempt['cap_tick']) / self.config.physics_hz
                attempt['cap'] = approach(attempt['cap'], attempt['floor'], limits['target_deceleration_m_s2'] * dt)
                attempt['cap_tick'] = tick
                self._target_speed = min(self._target_speed, attempt['cap'])
                return 'gap_creation_slowing'
            if elapsed >= limits['maximum_attempt_s']:
                _, lead = self._lane_ahead(self.current_lane, state, tracks, geometry)
                reference = (lead.speed_m_s * math.hypot(state.x_m, state.y_m)
                    / math.hypot(lead.x_m, lead.y_m) if lead is not None else attempt['initial_speed'])
                reference = max(reference, self._gap_last['maximum_rear_base_speed_m_s']
                                * math.hypot(state.x_m, state.y_m) / self.config.radius_m)
                if state.speed_m_s < reference - .15 or self._target_speed < state.speed_m_s - .15:
                    self._gap_failure('recovery_deadline', tick)
                self._gap_finish('ordinary_following_recovered', tick)
                return reason
            return reason if reason == 'unsafe_closing_gap_brake' else 'gap_creation_recovering'
        if reason != 'blocked_following' or self.changing_traffic_speed or self.traffic_transition_pending:
            self._gap_dwell = None
            return reason
        _, lead = self._lane_ahead(self.current_lane, state, tracks, geometry)
        signature = (self.current_lane, lead.vehicle_id, self.config, tuple(sorted(self._expected_ids or ())))
        if signature in self._gap_examined:
            return reason
        # Approach transients do not consume the once-per-formation assessment.
        # Every planning sample in the dwell must satisfy these prerequisites.
        ready = self._gap_geometry(state, tracks, 0.)
        if (lead.speed_m_s < limits['minimum_traffic_speed_m_s'] or abs(state.speed_m_s - lead.speed_m_s) > .35
                or ready['reason'] == 'straight_lane_tracking'):
            self._gap_dwell = None
            self._gap_state, self._gap_reason = 'waiting', 'not_settled_following_speed'
            return reason
        if self._gap_dwell is None or self._gap_dwell[0] != signature:
            self._gap_dwell = (signature, tick)
            self._gap_state, self._gap_reason = 'waiting', 'blocked_dwell'
            return reason
        if tick - self._gap_dwell[1] < limits['dwell_s'] * self.config.physics_hz:
            return reason
        # Bounded memory and search even if a user repeatedly reconfigures a long run.
        if len(self._gap_examined) >= 16 or self._gap_attempts >= limits['maximum_attempts']:
            self._gap_state, self._gap_reason = 'rejected', 'attempt_budget'
            return reason
        self._gap_examined.add(signature)
        evidence = self._gap_geometry(state, tracks, limits['maximum_attempt_s'])
        self._gap_last = dict(evidence, lead_vehicle_id=lead.vehicle_id, measured_speed_m_s=state.speed_m_s)
        if not evidence['eligible']:
            self._gap_event('rejected', evidence['reason'], tick)
            return reason
        opening = self._gap_opening(state, tracks)
        self._gap_last['counterfactual_opening'] = opening
        if not opening['possible']:
            self._gap_event('rejected', 'no_bounded_recovery_opening', tick)
            return reason
        self._gap_attempts += 1
        self._gap_attempt = dict(start_tick=tick, deadline_tick=tick + int(limits['maximum_attempt_s'] * self.config.physics_hz),
            config=self.config, lifecycle=self._gap_lifecycle, lead_id=lead.vehicle_id,
            reference_station=self._station(lead.x_m, lead.y_m), reference_progress=0., reference_tick=tick,
            reference_mode='measured_lead_progress',
            start_station=self._unwrapped_s, initial_speed=state.speed_m_s, cap=state.speed_m_s, cap_tick=tick,
            floor=max(limits['minimum_speed_m_s'], state.speed_m_s - limits['maximum_speed_deficit_m_s']))
        self._gap_event('slowing', 'bounded_recovery_opening', tick)
        self._target_speed = min(self._target_speed, state.speed_m_s)
        return 'gap_creation_slowing'

    def command(self, state, tick, tracks, stop=False):
        # Configuration/lifecycle cancellation is noticed at every control tick,
        # without waiting for the next tactical planning interval.
        if self._gap_attempt is not None and (isinstance(tick, int) and tick >= self._gap_attempt['deadline_tick']
                or (self._gap_state == 'slowing' and (stop or self.config != self._gap_attempt['config']
                or self._gap_lifecycle != self._gap_attempt['lifecycle']
                or self.changing_traffic_speed or self.traffic_transition_pending))):
            self._last_planning_tick = None
        command = super().command(state, tick, tracks, stop)
        if self._stop_latched:
            if self._gap_attempt is not None:
                if stop is True and self._stop_latched == 'requested_stop':
                    self._gap_finish('coordinated_requested_stop', tick)
                else:
                    # No next physics step may outlive the rear certificate with
                    # an unexpectedly stopped ego and still-moving backgrounds.
                    self._gap_failure('base_stop:' + self._stop_latched, tick)
        elif self._gap_attempt is not None:
            self._gap_guard(state, tick, tracks)
        self.diagnostics['gap_creation'] = self._gap_diagnostics()
        return command

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
        self._gap_lifecycle += 1

    def _candidate_safe(self,*args,**kwargs):
        return False if self.changing_traffic_speed or self.traffic_transition_pending else super()._candidate_safe(*args,**kwargs)
