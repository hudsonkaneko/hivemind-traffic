"""Continuous-circle scripted control; no pose writes and no learned policy.

Isaac/PhysX remains the movement authority. This controller consumes privileged
chassis-centre state and a known-map circle, then emits road-wheel steering and
normalised propulsion/braking requests for DriverControlGate. Physics is 120 Hz
by default; call the follower at 60 Hz and the gate on every physics tick.

Unlike the finite PathFollower, this module has no endpoint. Arc-length targets
wrap around the circle. A seam crossing is not a completed lap: completed laps
are measured from signed distance travelled since the first valid observation.
An explicit stop request latches until an episode reset and keeps path steering.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from numbers import Real
from pathlib import Path

from traffic.driver_control import DriverCommand
from traffic.lane_geometry import LaneProjection, RoutePoint, wrap_yaw
from traffic.path_following import FRAME, SOURCE, FollowerConfig, VehicleState


def _finite(value):
    try:
        return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)
    except (OverflowError, TypeError, ValueError):
        return False


def _tick(value):
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def _identity(value):
    return isinstance(value, str) and bool(value.strip())


@dataclass(frozen=True)
class CircularLoop:
    """Analytic CCW circle in the existing metre/XY/Z-up world convention.

    Station zero lies on the eastmost point; positive lateral error is inward
    (left of the direction of travel). Position is the road surface centreline;
    the physical vehicle adapter must supply its own spawn height.
    """

    route_id: str = 'MainLane1'
    radius_m: float = 500.0
    width_m: float = 3.7
    center_xy_m: tuple[float, float] = (0.0, 0.0)

    def __post_init__(self):
        if not _identity(self.route_id):
            raise ValueError('route_id must be a nonempty string')
        if not all(_finite(v) and v > 0 for v in (self.radius_m, self.width_m)):
            raise ValueError('Circle radius and width must be positive finite metres')
        if self.width_m / 2 >= self.radius_m:
            raise ValueError('Lane half-width must be smaller than radius')
        if (not isinstance(self.center_xy_m, (tuple, list)) or len(self.center_xy_m) != 2
                or not all(_finite(v) for v in self.center_xy_m)):
            raise ValueError('center_xy_m must contain two finite metre coordinates')
        if (not _finite(math.tau * self.radius_m)
                or not all(_finite(v + sign * self.radius_m)
                           for v in self.center_xy_m for sign in (-1, 1))):
            raise ValueError('Circle extent and circumference must remain finite')
        object.__setattr__(self, 'center_xy_m', tuple(float(v) for v in self.center_xy_m))

    @property
    def length_m(self):
        return math.tau * self.radius_m

    def evaluate(self, s_m):
        """Evaluate any signed station, wrapping periodically; no tangent tail."""
        if not _finite(s_m):
            raise ValueError('s_m must be finite metres')
        station = float(s_m) % self.length_m
        angle = station / self.radius_m
        return RoutePoint(station,
            (self.center_xy_m[0] + self.radius_m * math.cos(angle),
             self.center_xy_m[1] + self.radius_m * math.sin(angle)),
            wrap_yaw(angle + math.pi / 2), 1 / self.radius_m)

    def project(self, x_m, y_m):
        """Nearest centreline point; reject the ambiguous centre of the circle."""
        if not all(_finite(v) for v in (x_m, y_m)):
            raise ValueError('Projection coordinates must be finite metres')
        dx, dy = x_m - self.center_xy_m[0], y_m - self.center_xy_m[1]
        radial = math.hypot(dx, dy)
        if not math.isfinite(radial) or radial <= 1e-9:
            raise ValueError('Circle-centre projection is undefined')
        station = (math.atan2(dy, dx) % math.tau) * self.radius_m
        lateral = self.radius_m - radial
        return LaneProjection(station, lateral, abs(lateral), self.evaluate(station))

    @classmethod
    def from_navigation(cls, path, lane_id='MainLane1'):
        """Validate a V02 sampled circle, ignoring its unvalidated speed hint.

        This deliberately accepts only the published origin-centred circular
        lane representation, not connector splines or an inferred road fit.
        Samples must cover exactly one uniformly sampled counterclockwise lap.
        """
        document = json.loads(Path(path).read_text(encoding='utf-8'))
        expected = dict(schema_version=2, units='meters', up_axis='Z', east_axis='+X',
                        north_axis='+Y', forward_axis='+X', traffic_direction='counterclockwise')
        if not isinstance(document, dict) or any(document.get(k) != v for k, v in expected.items()):
            raise ValueError('Navigation schema, units, axes or direction are unsupported')
        if document.get('point_reference') != 'surface centerline; apply vehicle-specific height offset':
            raise ValueError('Navigation position reference must be the road surface centreline')
        lanes = document.get('lanes')
        lane = lanes.get(lane_id) if isinstance(lanes, dict) and _identity(lane_id) else None
        if not isinstance(lane, dict) or lane.get('id') != lane_id or lane.get('closed') is not True:
            raise ValueError('Requested navigation lane must be an identified closed circle')
        route = cls(lane_id, lane.get('radius_m'), lane.get('width_m'))
        samples = lane.get('points')
        if not isinstance(samples, list) or len(samples) < 16:
            raise ValueError('Circular navigation requires at least 16 complete samples')
        for index, point in enumerate(samples):
            if (not isinstance(point, list) or len(point) != 3
                    or not all(_finite(v) for v in point)):
                raise ValueError('Every navigation point must contain finite XYZ metres')
            predicted = route.evaluate(route.length_m * index / len(samples)).position_xy
            if math.dist(point[:2], predicted) > 1e-4 or abs(point[2]) > 1e-4:
                raise ValueError('Navigation points disagree with the declared origin-centred CCW circle')
        return route


@dataclass(frozen=True)
class LoopReference:
    """Fresh tactical cruise request; expiry is exclusive in physics ticks.

    ``target_speed_m_s`` is not an applied velocity. ``stop_requested`` latches
    a full-brake request in the follower until reset; it never teleports a car.
    """

    episode_id: str
    vehicle_id: str
    path_id: str
    issued_tick: int
    expires_tick: int
    target_speed_m_s: float
    stop_requested: bool = False
    frame: str = FRAME
    source: str = SOURCE


@dataclass(frozen=True)
class LoopFollowerConfig(FollowerConfig):
    """Independent loop envelope; selecting 35 mph does not validate dynamics.

    Kinematic jump detection uses a 24 m/s measurement envelope plus 5 cm of
    projection/quantisation slack. More than 10 cm of reverse progress from a
    previous high-water mark fails safe. These are input guards, not physics.
    """

    max_speed_m_s: float = 16.0
    speed_profile: str = '35mph'
    physics_hz: int = 120
    max_observed_speed_m_s: float = 24.0
    progress_slack_m: float = 0.05
    reverse_tolerance_m: float = 0.1
    max_sample_gap_ticks: int = 24

    def __post_init__(self):
        super().__post_init__()
        if not _tick(self.physics_hz) or not _finite(self.physics_hz) or self.physics_hz <= 0:
            raise ValueError('physics_hz must be a positive integer')
        if not all(_finite(v) and v > 0 for v in (
                self.max_observed_speed_m_s, self.progress_slack_m, self.reverse_tolerance_m)):
            raise ValueError('Progress guards must be positive finite values')
        if self.max_observed_speed_m_s < self.max_speed_m_s:
            raise ValueError('Observed-speed envelope cannot be below the command envelope')
        if not _tick(self.max_sample_gap_ticks) or not 1 <= self.max_sample_gap_ticks <= 24:
            raise ValueError('Sample gap must be 1..24 physics ticks')


class LoopProgress:
    """Signed, seam-safe distance accounting, separate from steering and physics.

    ``completed_laps`` requires a full net circuit from the first observation.
    ``seam_crossings`` counts signed crossings of the map's station zero. Reverse
    motion subtracts progress. More than half a circumference in one sample is
    inherently ambiguous and must be rejected by the caller's time/speed guard.
    """

    def __init__(self, route):
        if not isinstance(route, CircularLoop):
            raise ValueError('route must be CircularLoop')
        self.route = route
        self._last_s = None
        self._start_s = None
        self.distance_m = 0.0
        self.high_water_m = 0.0

    def delta(self, station_m):
        if not _finite(station_m) or not 0 <= station_m < self.route.length_m:
            raise ValueError('Station must be finite and in [0, circumference)')
        if self._last_s is None:
            return 0.0
        length = self.route.length_m
        difference = (station_m - self._last_s + length / 2) % length - length / 2
        if abs(abs(difference) - length / 2) < 1e-9:
            raise ValueError('Half-circuit progress is ambiguous')
        return difference

    def update(self, station_m):
        change = self.delta(station_m)
        if self._start_s is None:
            self._start_s = float(station_m)
        self._last_s = float(station_m)
        self.distance_m += change
        self.high_water_m = max(self.high_water_m, self.distance_m)
        return self.snapshot()

    def snapshot(self):
        length = self.route.length_m
        distance = self.distance_m
        # Tiny floating-point roundoff at an exact analytic lap is not a new
        # completion tolerance: this is less than a millionth of a millimetre.
        laps = int(math.floor(max(0.0, distance + 1e-9) / length))
        crossings = (0 if self._start_s is None else
                     math.floor((self._start_s + distance + 1e-9) / length))
        return dict(progress_m=distance, completed_laps=laps, seam_crossings=crossings,
                    reverse_from_high_water_m=self.high_water_m - distance)


class LoopFollower:
    """Periodic rear-axle pure pursuit plus bounded speed servo and fail-safe.

    ``command(state, tick, reference)`` returns DriverCommand. Feed that to
    DriverControlGate on delivery ticks and feed None on intervening ticks.
    Invalid input returns fresh zero-propulsion/full-brake fallback. Log the
    follower fallback separately: its brake command is valid at the control
    gate, so the gate need not mark it as its own fallback. A sample gap beyond
    the configured maximum requires reset (progress is no longer knowable).
    Clock failure raises ValueError and requires the runner to stop physics.
    """

    def __init__(self, route, episode_id, vehicle_id, config=None):
        if not isinstance(route, CircularLoop):
            raise ValueError('route must be CircularLoop')
        self.route = route
        self.config = LoopFollowerConfig() if config is None else config
        if not isinstance(self.config, LoopFollowerConfig):
            raise ValueError('config must be LoopFollowerConfig')
        cfg = self.config
        if cfg.max_observed_speed_m_s * cfg.max_sample_gap_ticks / cfg.physics_hz + cfg.progress_slack_m >= route.length_m / 2:
            raise ValueError('Route is too short for unambiguous progress at the configured sampling envelope')
        self.reset(episode_id=episode_id, vehicle_id=vehicle_id)

    def reset(self, *, episode_id, vehicle_id):
        if not _identity(episode_id) or not _identity(vehicle_id):
            raise ValueError('episode_id and vehicle_id must be nonempty strings')
        self.episode_id, self.vehicle_id = episode_id, vehicle_id
        self._last_tick = self._last_reference = self._last_state = None
        self._sequence = 0
        self._stop_latched = False
        self.progress = LoopProgress(self.route)
        self.last_diagnostics = dict(reason='reset', fallback=True, is_fallback=True,
                                     source=SOURCE, **self.progress.snapshot())

    def _invalid(self, state, tick, reference):
        cfg = self.config
        if not isinstance(state, VehicleState):
            return 'missing_or_invalid_state'
        if state.episode_id != self.episode_id or state.vehicle_id != self.vehicle_id:
            return 'state_identity_mismatch'
        if state.frame != FRAME or state.source != SOURCE:
            return 'state_frame_or_source_mismatch'
        if not _tick(state.tick) or not 0 <= tick - state.tick <= cfg.state_max_age_ticks:
            return 'stale_or_future_state'
        if not all(_finite(v) for v in (state.x_m, state.y_m, state.yaw_rad, state.speed_m_s)) or state.speed_m_s < 0:
            return 'nonfinite_or_invalid_state'
        if state.speed_m_s > cfg.max_observed_speed_m_s:
            return 'speed_outside_observed_envelope'
        if self._last_state is not None and (
                state.tick < self._last_state.tick
                or (state.tick == self._last_state.tick and state != self._last_state)):
            return 'replayed_or_conflicting_state'
        if not isinstance(reference, LoopReference):
            return 'missing_or_invalid_reference'
        if reference.episode_id != self.episode_id or reference.vehicle_id != self.vehicle_id:
            return 'reference_identity_mismatch'
        if reference.frame != FRAME or reference.source != SOURCE:
            return 'reference_frame_or_source_mismatch'
        if reference.path_id != self.route.route_id:
            return 'unknown_path'
        if (not _tick(reference.issued_tick) or not _tick(reference.expires_tick)
                or not reference.issued_tick <= tick < reference.expires_tick
                or not 0 < reference.expires_tick - reference.issued_tick <= 24):
            return 'stale_or_future_reference'
        if (not _finite(reference.target_speed_m_s)
                or not 0 <= reference.target_speed_m_s <= cfg.max_speed_m_s
                or not isinstance(reference.stop_requested, bool)):
            return 'invalid_reference_limits'
        if self._last_reference is not None and (
                reference.issued_tick < self._last_reference.issued_tick
                or (reference.issued_tick == self._last_reference.issued_tick and reference != self._last_reference)):
            return 'replayed_or_conflicting_reference'
        return None

    def _make_command(self, tick, steering, throttle, brake, expiry):
        result = DriverCommand(self.episode_id, self.vehicle_id, self._sequence,
                               tick, expiry, steering, throttle, brake)
        self._sequence += 1
        return result

    def _fallback(self, tick, reason):
        self.last_diagnostics = dict(reason=reason, fallback=True, is_fallback=True,
            source=SOURCE, target_speed_m_s=0.0, stop_latched=self._stop_latched,
            **self.progress.snapshot())
        return self._make_command(tick, 0.0, 0.0, 1.0, tick + self.config.command_ttl_ticks)

    def command(self, state, tick, reference):
        if not _tick(tick) or (self._last_tick is not None and tick <= self._last_tick):
            raise ValueError('Control tick must strictly increase; stop physics if its clock is broken')
        self._last_tick = tick
        reason = self._invalid(state, tick, reference)
        if reason:
            return self._fallback(tick, reason)
        cfg, route = self.config, self.route
        try:
            center = route.project(state.x_m, state.y_m)
        except ValueError:
            return self._fallback(tick, 'invalid_route_projection')
        heading_error = wrap_yaw(center.point.yaw_rad - state.yaw_rad)
        if abs(center.lateral_error_m) > route.width_m / 2:
            return self._fallback(tick, 'outside_lane_recovery_envelope')
        if abs(heading_error) > cfg.max_heading_error_rad:
            return self._fallback(tick, 'heading_outside_recovery_envelope')
        try:
            delta = self.progress.delta(center.s_m)
        except ValueError:
            return self._fallback(tick, 'ambiguous_route_progress')
        if self._last_state is not None:
            elapsed = state.tick - self._last_state.tick
            if elapsed > cfg.max_sample_gap_ticks:
                return self._fallback(tick, 'progress_sample_gap')
            maximum = cfg.max_observed_speed_m_s * elapsed / cfg.physics_hz + cfg.progress_slack_m
            if abs(delta) > maximum:
                return self._fallback(tick, 'implausible_route_progress')
        progress = self.progress.update(center.s_m)
        self._last_state, self._last_reference = state, reference
        self._stop_latched = self._stop_latched or reference.stop_requested
        if progress['reverse_from_high_water_m'] > cfg.reverse_tolerance_m:
            return self._fallback(tick, 'wrong_way_progress')
        rear_x = state.x_m - cfg.rear_axle_offset_m * math.cos(state.yaw_rad)
        rear_y = state.y_m - cfg.rear_axle_offset_m * math.sin(state.yaw_rad)
        rear = route.project(rear_x, rear_y)
        lookahead = cfg.lookahead_base_m + cfg.lookahead_time_s * state.speed_m_s
        target = route.evaluate(rear.s_m + lookahead)
        dx, dy = target.position_xy[0] - rear_x, target.position_xy[1] - rear_y
        local_x = math.cos(state.yaw_rad) * dx + math.sin(state.yaw_rad) * dy
        local_y = -math.sin(state.yaw_rad) * dx + math.cos(state.yaw_rad) * dy
        chord_squared = dx * dx + dy * dy
        if local_x <= 0 or chord_squared <= 1e-9:
            return self._fallback(tick, 'pursuit_target_not_ahead')
        steering = max(-cfg.max_steering_rad, min(cfg.max_steering_rad,
            math.atan(cfg.wheelbase_m * 2 * local_y / chord_squared)))
        target_speed = 0.0 if self._stop_latched else reference.target_speed_m_s
        error = target_speed - state.speed_m_s
        if target_speed <= 0:
            throttle, brake, reason = 0.0, 1.0, 'requested_stop' if self._stop_latched else 'behavior_stop'
        elif error < -cfg.speed_deadband_m_s:
            throttle, brake, reason = 0.0, min(1.0, -error * cfg.brake_kp), 'speed_braking'
        elif error > cfg.speed_deadband_m_s:
            throttle, brake, reason = min(1.0, error * cfg.throttle_kp), 0.0, 'tracking'
        else:
            throttle, brake, reason = 0.0, 0.0, 'speed_coasting'
        self.last_diagnostics = dict(reason=reason, fallback=False, is_fallback=False,
            source=SOURCE, state_source=state.source, path_source=reference.source,
            path_id=route.route_id, center_progress_m=center.s_m, rear_progress_m=rear.s_m,
            center_lateral_error_m=center.lateral_error_m, heading_error_rad=heading_error,
            lookahead_m=lookahead, target_s_m=target.s_m, target_xy_m=target.position_xy,
            target_speed_m_s=target_speed, requested_steering_rad=steering,
            stop_latched=self._stop_latched, state_age_ticks=tick - state.tick,
            reference_age_ticks=tick - reference.issued_tick, **progress)
        return self._make_command(tick, steering, throttle, brake,
            min(tick + cfg.command_ttl_ticks, reference.expires_tick))
