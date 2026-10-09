"""Scripted moving-object overtaking on the four-lane V02 circular highway.

This is a bounded visual showcase controller, NOT LiDAR perception, RL, or a
general autonomous-driving stack. Both ego odometry and object tracks are
explicitly privileged simulator state. Background objects must follow the
declared circular lanes at constant speed. Isaac/PhysX alone moves the ego.

The caller runs ``command`` at 60 Hz, DriverControlGate at 120 Hz. Tactical
planning runs internally at 10 Hz. Lane changes are frozen quintic radial
curves; rear-axle pure pursuit and a speed servo request steering and wheel
effort. Forecasts are sampled kinematic envelopes, not dynamics guarantees;
the runner must independently measure contact, clearance and road containment.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from traffic.bypass_validation import OrientedBox, box_clearance
from traffic.driver_control import DriverCommand
from traffic.lane_geometry import wrap_yaw
from traffic.loop_driving import CircularLoop, LoopProgress
from traffic.path_following import FRAME, SOURCE, VehicleState, _finite, _identity, _tick


@dataclass(frozen=True)
class ObjectTrack:
    """Complete current-frame object state; metres/radians/simulated ticks.

    Position is the collision-box centre. Speed is nonnegative forward planar
    speed. Objects are not sensor detections and this type never claims that.
    """

    episode_id: str
    vehicle_id: str
    tick: int
    x_m: float
    y_m: float
    yaw_rad: float
    speed_m_s: float
    length_m: float = 4.8
    width_m: float = 1.8
    frame: str = FRAME
    source: str = SOURCE


@dataclass(frozen=True)
class ShowcaseConfig:
    radius_m: float = 500.0
    lane_width_m: float = 3.7
    lane_count: int = 4
    physics_hz: int = 120
    planning_interval_ticks: int = 12
    command_ttl_ticks: int = 12
    target_speed_m_s: float = 15.6464
    max_observed_speed_m_s: float = 20.0
    lane_change_length_m: float = 80.0
    lane_change_lead_m: float = 2.0
    obstacle_trigger_m: float = 110.0
    desired_time_gap_s: float = 1.8
    minimum_follow_gap_m: float = 13.0
    prediction_step_s: float = 0.2
    prediction_clearance_m: float = 0.8
    prediction_tracking_margin_m: float = 0.30
    ego_length_m: float = 5.0
    ego_width_m: float = 2.22
    ego_collision_offset_x_m: float = 0.0
    wheelbase_m: float = 3.0
    rear_axle_offset_m: float = 1.5
    max_steering_rad: float = 0.5
    lookahead_base_m: float = 3.0
    lookahead_time_s: float = 0.6
    throttle_kp: float = 0.7
    brake_kp: float = 0.35
    max_tracking_error_m: float = 0.8
    max_heading_error_rad: float = 0.5
    progress_slack_m: float = 0.05

    def __post_init__(self):
        counts = (self.lane_count, self.physics_hz, self.planning_interval_ticks,
                  self.command_ttl_ticks)
        if not all(_tick(v) and v > 0 for v in counts):
            raise ValueError('Rates, lane count and lifetimes must be positive integers')
        if self.lane_count != 4 or self.physics_hz != 120:
            raise ValueError('Showcase is qualified only for four lanes and 120 Hz physics')
        if self.planning_interval_ticks > 12 or self.command_ttl_ticks > 12:
            raise ValueError('Planning and command lifetimes cannot exceed 12 ticks')
        positive = [v for k, v in vars(self).items()
                    if k not in ('ego_collision_offset_x_m',)]
        if not all(_finite(v) and v > 0 for v in positive):
            raise ValueError('Configuration values must be positive finite numbers')
        if not _finite(self.ego_collision_offset_x_m):
            raise ValueError('Collision offset must be finite')
        if (self.radius_m != 500.0 or self.lane_width_m != 3.7
                or self.target_speed_m_s > 15.6464 or self.max_observed_speed_m_s < self.target_speed_m_s
                or self.max_observed_speed_m_s > 24 or self.max_steering_rad > .5
                or self.lane_change_length_m < 60 or self.prediction_step_s > .2
                or self.ego_width_m + 2 * self.prediction_tracking_margin_m >= self.lane_width_m
                or self.rear_axle_offset_m > self.wheelbase_m):
            raise ValueError('Configuration exceeds the declared showcase envelope')


@dataclass(frozen=True)
class RadialPath:
    """Station is unwrapped angle * 500 m, NOT arc length in every lane.

    Lane zero is the inner main lane (R=500 m). Increasing lane numbers move
    outwards/right. First and second radial derivatives vanish at both ends.
    """

    from_lane: int
    to_lane: int
    start_s_m: float
    length_m: float
    radius_m: float = 500.0
    lane_width_m: float = 3.7

    def __post_init__(self):
        if (not all(_tick(v) and v < 4 for v in (self.from_lane, self.to_lane))
                or abs(self.to_lane - self.from_lane) > 1
                or not all(_finite(v) for v in (self.start_s_m, self.length_m, self.radius_m, self.lane_width_m))
                or min(self.length_m, self.radius_m, self.lane_width_m) <= 0):
            raise ValueError('Path must join adjacent valid lanes with finite dimensions')

    @property
    def end_s_m(self):
        return self.start_s_m + self.length_m

    def radius_and_derivative(self, station_m):
        if not _finite(station_m):
            raise ValueError('Station must be finite')
        q = max(0.0, min(1.0, (station_m - self.start_s_m) / self.length_m))
        if q == 0.0:
            return self.radius_m + self.from_lane * self.lane_width_m, 0.0
        if q == 1.0:
            return self.radius_m + self.to_lane * self.lane_width_m, 0.0
        blend = q ** 3 * (10 + q * (-15 + 6 * q))
        derivative = 30 * q * q * (1 - q) ** 2 / self.length_m
        shift = (self.to_lane - self.from_lane) * self.lane_width_m
        return self.radius_m + self.from_lane * self.lane_width_m + shift * blend, shift * derivative

    def pose(self, station_m):
        radius, dr_ds = self.radius_and_derivative(station_m)
        angle = station_m / self.radius_m
        c, s = math.cos(angle), math.sin(angle)
        dx, dy = dr_ds * c - radius / self.radius_m * s, dr_ds * s + radius / self.radius_m * c
        return radius * c, radius * s, math.atan2(dy, dx)


def signed_station_gap(a_m, b_m, radius_m=500.0):
    """Shortest signed station difference b-a, correctly crossing the seam."""
    length = math.tau * radius_m
    return (b_m - a_m + length / 2) % length - length / 2


def track_prediction(track, elapsed_s):
    """Exact constant-angular-speed circular prediction for declared backgrounds."""
    if not _finite(elapsed_s) or elapsed_s < 0:
        raise ValueError('Prediction time must be nonnegative finite seconds')
    radius = math.hypot(track.x_m, track.y_m)
    if radius <= 0:
        raise ValueError('Object cannot be at the undefined circle centre')
    angle = math.atan2(track.y_m, track.x_m) + track.speed_m_s * elapsed_s / radius
    return OrientedBox(radius * math.cos(angle), radius * math.sin(angle),
                       wrap_yaw(angle + math.pi / 2), track.length_m, track.width_m)


class ShowcaseController:
    """Scripted lane decisions -> a smooth path -> expiring wheel commands.

    Invalid/missing/stale object data latches full braking until reset/new
    controller. A blocked adjacent lane is normal and instead requests following
    speed. A committed lane change is never jerkily replaced by its opposite.
    """

    def __init__(self, episode_id, vehicle_id, config=None, initial_lane=0):
        if not _identity(episode_id) or not _identity(vehicle_id):
            raise ValueError('Episode and vehicle identities are required')
        if not _tick(initial_lane) or initial_lane >= 4:
            raise ValueError('Initial lane must be 0..3')
        self.config = config or ShowcaseConfig()
        if not isinstance(self.config, ShowcaseConfig):
            raise ValueError('Expected ShowcaseConfig')
        self.episode_id, self.vehicle_id = episode_id, vehicle_id
        self.base = CircularLoop()
        self.progress = LoopProgress(self.base)
        self.current_lane = initial_lane
        self.path = RadialPath(initial_lane, initial_lane, 0.0, self.config.lane_change_length_m)
        self._last_tick = self._last_planning_tick = self._last_state = None
        self._sequence = 0
        self._unwrapped_s = None
        self._expected_ids = None
        self._previous_tracks = None
        self._stop_latched = None
        self._target_speed = self.config.target_speed_m_s
        self._maneuver = False
        self._lane_changes = 0
        self._pass_candidates = set()
        self._passed_ids = set()
        self.diagnostics = dict(reason='reset', is_fallback=True, source=SOURCE)

    @property
    def last_diagnostics(self):
        return self.diagnostics

    def _validate(self, state, tick, tracks):
        cfg = self.config
        if (not isinstance(state, VehicleState) or state.episode_id != self.episode_id
                or state.vehicle_id != self.vehicle_id or state.frame != FRAME or state.source != SOURCE
                or not _tick(state.tick) or state.tick != tick
                or not all(_finite(v) for v in (state.x_m, state.y_m, state.yaw_rad, state.speed_m_s))
                or not 0 <= state.speed_m_s <= cfg.max_observed_speed_m_s):
            return 'invalid_ego_state'
        if not cfg.radius_m - 25 <= math.hypot(state.x_m, state.y_m) <= cfg.radius_m + 40:
            return 'invalid_ego_position'
        if not isinstance(tracks, (tuple, list)):
            return 'missing_object_frame'
        ids = []
        for obj in tracks:
            if (not isinstance(obj, ObjectTrack) or obj.episode_id != self.episode_id
                    or not _identity(obj.vehicle_id) or obj.vehicle_id == self.vehicle_id
                    or not _tick(obj.tick) or obj.tick != tick or obj.frame != FRAME or obj.source != SOURCE
                    or not all(_finite(v) for v in (obj.x_m, obj.y_m, obj.yaw_rad, obj.speed_m_s,
                                                    obj.length_m, obj.width_m))
                    or not 0 <= obj.speed_m_s <= cfg.max_observed_speed_m_s
                    or not 0 < obj.length_m <= 8 or not 0 < obj.width_m <= 2.5):
                return 'invalid_or_stale_object_track'
            radius = math.hypot(obj.x_m, obj.y_m)
            lane = round((radius - cfg.radius_m) / cfg.lane_width_m)
            tangent = math.atan2(obj.y_m, obj.x_m) + math.pi / 2
            if (not 0 <= lane < cfg.lane_count or abs(radius - (cfg.radius_m + lane * cfg.lane_width_m)) > .10
                    or abs(wrap_yaw(obj.yaw_rad - tangent)) > .05):
                return 'object_outside_constant_circle_contract'
            ids.append(obj.vehicle_id)
        if len(ids) != len(set(ids)):
            return 'duplicate_object_identity'
        if self._expected_ids is not None and set(ids) != self._expected_ids:
            return 'incomplete_or_changed_object_frame'
        if self._previous_tracks is not None:
            for obj in tracks:
                previous = self._previous_tracks[obj.vehicle_id]
                elapsed_s = (tick - previous.tick) / cfg.physics_hz
                predicted = track_prediction(previous, elapsed_s)
                # Native float32 poses around R=500 m can quantise a 120 Hz
                # finite-difference speed by a few mm/s. This tolerance covers
                # that readback noise, not a changing background policy.
                if (abs(obj.speed_m_s - previous.speed_m_s) > .05
                        or abs(obj.length_m - previous.length_m) > 1e-6
                        or abs(obj.width_m - previous.width_m) > 1e-6
                        or math.hypot(obj.x_m - predicted.x_m, obj.y_m - predicted.y_m) > .02):
                    return 'object_motion_contract_changed'
        return None

    def _command(self, tick, steering, throttle, brake):
        command = DriverCommand(self.episode_id, self.vehicle_id, self._sequence,
            tick, tick + self.config.command_ttl_ticks, steering, throttle, brake)
        self._sequence += 1
        return command

    def _ego_box(self, pose, *, inflated=False):
        cfg = self.config
        x, y, yaw = pose
        extra = 2 * cfg.prediction_tracking_margin_m if inflated else 0.0
        return OrientedBox(x + cfg.ego_collision_offset_x_m * math.cos(yaw),
            y + cfg.ego_collision_offset_x_m * math.sin(yaw), yaw,
            cfg.ego_length_m + extra, cfg.ego_width_m + extra)

    def _station(self, x, y):
        # atan2(-epsilon, +radius) may round to exactly tau after modulo;
        # normalise again after scaling so the seam stays in [0,length).
        return self.base.project(x, y).s_m % self.base.length_m

    def _lane_ahead(self, lane, state, tracks):
        cfg = self.config
        result = (math.inf, None)
        radius = cfg.radius_m + lane * cfg.lane_width_m
        for obj in tracks:
            if abs(math.hypot(obj.x_m, obj.y_m) - radius) > cfg.lane_width_m / 2:
                continue
            gap = signed_station_gap(self._unwrapped_s, self._station(obj.x_m, obj.y_m)) * radius / cfg.radius_m
            bumper = gap - (cfg.ego_length_m + obj.length_m) / 2
            if gap > 0 and bumper < result[0]:
                result = bumper, obj
        return result

    def _candidate_safe(self, path, state, tracks):
        """Conservative station/radius envelope over plausible acceleration.

        Ego future station lies between holding its current speed and reaching
        cruise at 3 m/s2. This includes intermediate acceleration, unlike just
        testing two endpoint trajectories. Inflated angular/radial intervals
        cover body length, tracking error and half a prediction sample's travel.
        This assumes the declared constant-speed circular background model.
        """
        cfg = self.config
        speed = max(state.speed_m_s, 1.0)
        horizon = min(12.0, max(6.0, (path.end_s_m - self._unwrapped_s) / speed + 1.0))
        count = math.ceil(horizon / cfg.prediction_step_s)
        for index in range(count + 1):
            t = horizon * index / count
            current_radius = math.hypot(state.x_m, state.y_m)
            low_speed = min(state.speed_m_s, cfg.target_speed_m_s)
            to_cruise = max(0.0, cfg.target_speed_m_s - low_speed) / 3.0
            accelerating = min(t, to_cruise)
            dist_low = low_speed * t
            dist_high = low_speed * accelerating + 1.5 * accelerating ** 2 + cfg.target_speed_m_s * max(0., t - accelerating)
            if state.speed_m_s > cfg.target_speed_m_s:
                dist_high = state.speed_m_s * t
            s_low = self._unwrapped_s + dist_low * cfg.radius_m / current_radius
            s_high = self._unwrapped_s + dist_high * cfg.radius_m / current_radius
            radial = [path.radius_and_derivative(s)[0] for s in (s_low, s_high)]
            # Small circular/path-angle projection terms are covered explicitly.
            max_lateral_slope = 1.875 * cfg.lane_width_m / path.length_m
            ego_radial_half = (cfg.ego_width_m / 2 + cfg.ego_length_m / 2 * max_lateral_slope
                               + cfg.prediction_tracking_margin_m + .02)
            for obj in tracks:
                obj_radius = math.hypot(obj.x_m, obj.y_m)
                gap = signed_station_gap(self._unwrapped_s, self._station(obj.x_m, obj.y_m))
                obj_s = self._unwrapped_s + gap + obj.speed_m_s * t * cfg.radius_m / obj_radius
                sample_pad = (cfg.max_observed_speed_m_s + obj.speed_m_s) * cfg.prediction_step_s / 2
                longitudinal = ((cfg.ego_length_m + obj.length_m) / 2 + cfg.prediction_clearance_m
                                + cfg.prediction_tracking_margin_m + sample_pad)
                lateral = ego_radial_half + obj.width_m / 2 + cfg.prediction_clearance_m
                if s_low - longitudinal <= obj_s <= s_high + longitudinal and min(radial) - lateral <= obj_radius <= max(radial) + lateral:
                    return False
        return True

    def _plan(self, state, tick, tracks):
        cfg = self.config
        if self._maneuver and self._unwrapped_s >= self.path.end_s_m:
            self.current_lane = self.path.to_lane
            self._maneuver = False
            self._lane_changes += 1
            self.path = RadialPath(self.current_lane, self.current_lane, self._unwrapped_s,
                                   cfg.lane_change_length_m)
        self._target_speed = cfg.target_speed_m_s
        reason = 'cruise'
        lead_gap, lead = self._lane_ahead(self.current_lane, state, tracks)
        if self._maneuver:
            reason = 'committed_lane_change'
        elif lead is not None and lead_gap < cfg.obstacle_trigger_m and lead.speed_m_s < cfg.target_speed_m_s - .5:
            candidates = []
            for lane in (self.current_lane - 1, self.current_lane + 1):
                if not 0 <= lane < cfg.lane_count:
                    continue
                path = RadialPath(self.current_lane, lane,
                    self._unwrapped_s + cfg.lane_change_lead_m, cfg.lane_change_length_m)
                ahead, _ = self._lane_ahead(lane, state, tracks)
                if ahead > lead_gap + 12 and self._candidate_safe(path, state, tracks):
                    candidates.append((min(ahead, 500.), -lane, path))
            if candidates:
                self.path = max(candidates, key=lambda item: item[:2])[2]
                self._maneuver = True
                reason = 'safe_adjacent_lane_selected'
            else:
                reason = 'blocked_following'
        # Follow any object overlapping the ego's present lane-width corridor.
        # During a lane change this also protects the old lane until separated.
        radius = math.hypot(state.x_m, state.y_m)
        for obj in tracks:
            if abs(math.hypot(obj.x_m, obj.y_m) - radius) > (cfg.ego_width_m + obj.width_m) / 2 + .25:
                continue
            gap = signed_station_gap(self._unwrapped_s, self._station(obj.x_m, obj.y_m))
            if gap <= 0:
                continue
            bumper = gap - (cfg.ego_length_m + obj.length_m) / 2 - abs(cfg.ego_collision_offset_x_m)
            desired_gap = max(cfg.minimum_follow_gap_m, state.speed_m_s * cfg.desired_time_gap_s)
            self._target_speed = min(self._target_speed, max(0., obj.speed_m_s + .4 * (bumper - desired_gap)))
            closing = max(0., state.speed_m_s - obj.speed_m_s)
            if bumper <= 4 + closing * .25 + closing ** 2 / (2 * 2.5):
                self._target_speed = 0.0
                reason = 'unsafe_closing_gap_brake'
        self._last_planning_tick = tick
        return reason

    def command(self, state, tick, tracks, stop=False):
        if not _tick(tick) or (self._last_tick is not None and tick <= self._last_tick):
            raise ValueError('Control ticks must strictly increase; stop physics on broken clock')
        self._last_tick = tick
        invalid = self._validate(state, tick, tracks)
        if not isinstance(stop, bool):
            invalid = 'invalid_stop_request'
        if invalid:
            self._stop_latched = invalid
            self.diagnostics = dict(reason=invalid, is_fallback=True, source=SOURCE,
                target_speed_m_s=0., lane_changes=self._lane_changes, passes=len(self._passed_ids))
            return self._command(tick, 0., 0., 1.)
        if self._expected_ids is None:
            self._expected_ids = {obj.vehicle_id for obj in tracks}
        self._previous_tracks = {obj.vehicle_id: obj for obj in tracks}
        station = self._station(state.x_m, state.y_m)
        delta = self.progress.delta(station)
        if self._last_state is not None:
            elapsed = tick - self._last_state.tick
            if elapsed > 24 or abs(delta) > self.config.max_observed_speed_m_s * elapsed / self.config.physics_hz + self.config.progress_slack_m:
                self._stop_latched = 'implausible_or_missing_ego_progress'
        if self._unwrapped_s is None:
            self._unwrapped_s = station
        else:
            self._unwrapped_s += delta
        progress = self.progress.update(station)
        self._last_state = state
        if progress['reverse_from_high_water_m'] > .1:
            self._stop_latched = 'wrong_way_progress'
        if stop:
            self._stop_latched = self._stop_latched or 'requested_stop'
        cfg = self.config
        actual = self._ego_box((state.x_m, state.y_m, state.yaw_rad))
        if any(not cfg.radius_m - cfg.lane_width_m / 2 < math.hypot(x, y) < cfg.radius_m + (cfg.lane_count - .5) * cfg.lane_width_m for x, y in actual.corners()):
            self._stop_latched = 'outside_road_footprint'
        minimum_clearance = math.inf
        for obj in tracks:
            other = OrientedBox(obj.x_m, obj.y_m, obj.yaw_rad, obj.length_m, obj.width_m)
            gap = signed_station_gap(self._unwrapped_s, self._station(obj.x_m, obj.y_m))
            if abs(gap) < 30:
                minimum_clearance = min(minimum_clearance, box_clearance(actual, other)['clearance_m'])
            if 10 < gap < 500:
                self._pass_candidates.add(obj.vehicle_id)
            if obj.vehicle_id in self._pass_candidates and gap < -((cfg.ego_length_m + obj.length_m) / 2 + 5):
                self._passed_ids.add(obj.vehicle_id)
        if minimum_clearance < .45:
            self._stop_latched = 'object_clearance_emergency'
        reason = self.diagnostics.get('planner_reason', 'cruise')
        if not self._stop_latched and (self._last_planning_tick is None or tick - self._last_planning_tick >= cfg.planning_interval_ticks):
            reason = self._plan(state, tick, tracks)
        rear_x = state.x_m - cfg.rear_axle_offset_m * math.cos(state.yaw_rad)
        rear_y = state.y_m - cfg.rear_axle_offset_m * math.sin(state.yaw_rad)
        rear_s = self._unwrapped_s + signed_station_gap(station, self._station(rear_x, rear_y))
        lookahead = cfg.lookahead_base_m + cfg.lookahead_time_s * state.speed_m_s
        tx, ty, _ = self.path.pose(rear_s + lookahead)
        dx, dy = tx - rear_x, ty - rear_y
        local_x = math.cos(state.yaw_rad) * dx + math.sin(state.yaw_rad) * dy
        local_y = -math.sin(state.yaw_rad) * dx + math.cos(state.yaw_rad) * dy
        chord2 = dx * dx + dy * dy
        px, py, pyaw = self.path.pose(self._unwrapped_s)
        tracking_error = math.hypot(state.x_m, state.y_m) - math.hypot(px, py)
        heading_error = wrap_yaw(pyaw - state.yaw_rad)
        if abs(tracking_error) > cfg.max_tracking_error_m or abs(heading_error) > cfg.max_heading_error_rad:
            self._stop_latched = 'path_tracking_envelope'
        if chord2 <= 1e-9 or local_x <= 0:
            self._stop_latched = 'pursuit_target_not_ahead'
        steering = max(-cfg.max_steering_rad, min(cfg.max_steering_rad,
            math.atan(cfg.wheelbase_m * 2 * local_y / max(chord2, 1e-9))))
        target = 0. if self._stop_latched else self._target_speed
        error = target - state.speed_m_s
        if target <= 0:
            throttle, brake = 0., 1.
        elif error < -.03:
            throttle, brake = 0., min(1., -error * cfg.brake_kp)
        elif error > .03:
            throttle, brake = min(1., error * cfg.throttle_kp), 0.
        else:
            throttle, brake = 0., 0.
        self.diagnostics = dict(reason=self._stop_latched or reason, planner_reason=reason,
            is_fallback=bool(self._stop_latched and self._stop_latched != 'requested_stop'),
            stop_latched=bool(self._stop_latched), source=SOURCE, state_source=SOURCE,
            object_source=SOURCE, target_speed_m_s=target, requested_steering_rad=steering,
            lane=self.current_lane, target_lane=self.path.to_lane, maneuver_active=self._maneuver,
            lane_changes=self._lane_changes, passes=len(self._passed_ids), passed_ids=sorted(self._passed_ids),
            center_progress_m=station, unwrapped_station_m=self._unwrapped_s,
            path_start_s_m=self.path.start_s_m, path_end_s_m=self.path.end_s_m,
            center_lateral_error_m=tracking_error, heading_error_rad=heading_error,
            target_xy_m=(tx, ty), minimum_nearby_clearance_m=None if math.isinf(minimum_clearance) else minimum_clearance,
            **progress)
        return self._command(tick, steering, throttle, brake)

    def path_points(self, state, count=61, horizon_m=120.0):
        """World XYZ overlay of the actual committed reference, road Z + 6 cm."""
        if not _tick(count) or count < 2 or not _finite(horizon_m) or horizon_m <= 0:
            raise ValueError('Overlay requires count>=2 and a positive finite horizon')
        station = self._station(state.x_m, state.y_m)
        start = station if self._unwrapped_s is None else self._unwrapped_s + signed_station_gap(self._unwrapped_s, station)
        return tuple((*self.path.pose(start + horizon_m * i / (count - 1))[:2], .06)
                     for i in range(count))
