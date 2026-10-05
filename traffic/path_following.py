"""Scripted behavior -> explicit path reference -> physical wheel commands.

This CPU-only module neither simulates nor teleports cars. State and map inputs
are privileged simulator data by default, NOT LiDAR perception. A separate
planner may label a sensed detour with SENSOR_PATH_SOURCE; ego state stays
explicitly privileged. Isaac/PhysX owns movement;
the caller feeds returned DriverCommand objects through DriverControlGate once
per physics tick. Suggested rates: behavior/planning 10 Hz, control 60 Hz,
physics 120 Hz. Every timestamp is an integer physics tick in one episode.

World convention: meters, X/Y ground plane, Z up, positive yaw/steering left.
Pure pursuit uses the rear axle; progress/endpoint stopping use chassis center.
Reference: Coulter, CMU-RI-TR-92-01 (1992), geometric pure pursuit.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real
from typing import Mapping

from traffic.driver_control import DriverCommand
from traffic.lane_geometry import LaneRoute

FRAME = 'xy_m_z_up_yaw_ccw_rad'
SOURCE = 'privileged_simulator_state_and_known_map'
SENSOR_PATH_SOURCE = 'lidar_derived_path_with_privileged_odometry'


def _finite(value):
    try:
        return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)
    except (OverflowError, TypeError, ValueError):
        return False


def _tick(value):
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def _identity(value):
    return isinstance(value, str) and bool(value.strip())


def _identities(episode_id, vehicle_id):
    if not _identity(episode_id) or not _identity(vehicle_id):
        raise ValueError('episode_id and vehicle_id must be nonempty strings')


def _lifetime(issued, expires, now, maximum):
    return (_tick(issued) and _tick(expires) and issued <= now < expires
            and 0 < expires - issued <= maximum)


def _routes(routes):
    registry = dict(routes)
    if not registry:
        raise ValueError('At least one immutable LaneRoute is required')
    for key, route in registry.items():
        if (not _identity(key) or not isinstance(route, LaneRoute) or route.route_id != key
                or not _finite(route.length_m) or route.length_m <= 0
                or not _finite(route.width_m) or route.width_m <= 0):
            raise ValueError('Route registry IDs/lengths/widths are invalid')
    return registry


@dataclass(frozen=True)
class VehicleState:
    """Chassis-center pose and planar speed from simulator state; not a sensor."""

    episode_id: str
    vehicle_id: str
    tick: int
    x_m: float
    y_m: float
    yaw_rad: float
    speed_m_s: float
    frame: str = FRAME
    source: str = SOURCE

    @classmethod
    def from_physics(cls, state, *, episode_id, vehicle_id, tick):
        return cls(episode_id, vehicle_id, tick, state['position_m'][0],
                   state['position_m'][1], state['yaw_rad'], state['speed_m_s'])


@dataclass(frozen=True)
class BehaviorIntent:
    """A tactical choice: follow one lane at a requested speed, then stop."""

    episode_id: str
    vehicle_id: str
    path_id: str
    issued_tick: int
    expires_tick: int
    target_speed_m_s: float


@dataclass(frozen=True)
class PathReference:
    """Reference to an immutable registered route, not a pose command or trajectory.

    ``stop_s_m`` is the chassis-center station at which to stop. Validity is
    exclusive at expires_tick and cannot outlive the issuing behavior intent.
    """

    episode_id: str
    vehicle_id: str
    path_id: str
    issued_tick: int
    expires_tick: int
    target_speed_m_s: float
    stop_s_m: float
    frame: str = FRAME
    source: str = SOURCE


class ScriptedCruiseBehavior:
    """Known-clear route only: no traffic interactions or obstacle decisions."""

    def __init__(self, episode_id, vehicle_id, path_id, target_speed_m_s=3.0,
                 intent_ttl_ticks=36):
        _identities(episode_id, vehicle_id)
        if not _identity(path_id) or not _finite(target_speed_m_s) or not 0 <= target_speed_m_s <= 3:
            raise ValueError('Known path ID and speed in [0,3] m/s required')
        if not _tick(intent_ttl_ticks) or not 1 <= intent_ttl_ticks <= 36:
            raise ValueError('Behavior intent TTL must be 1..36 physics ticks')
        self.episode_id, self.vehicle_id, self.path_id = episode_id, vehicle_id, path_id
        self.target_speed_m_s, self.intent_ttl_ticks = target_speed_m_s, intent_ttl_ticks

    def decide(self, tick):
        if not _tick(tick):
            raise ValueError('Behavior tick must be a nonnegative integer')
        return BehaviorIntent(self.episode_id, self.vehicle_id, self.path_id,
                              tick, tick + self.intent_ttl_ticks, self.target_speed_m_s)


class PathPlanner:
    """Select an immutable lane route and endpoint; no obstacle avoidance yet."""

    def __init__(self, routes, episode_id, vehicle_id, reference_ttl_ticks=24):
        _identities(episode_id, vehicle_id)
        if not _tick(reference_ttl_ticks) or not 1 <= reference_ttl_ticks <= 24:
            raise ValueError('Path reference TTL must be 1..24 physics ticks')
        self.routes = _routes(routes)
        self.episode_id, self.vehicle_id = episode_id, vehicle_id
        self.reference_ttl_ticks = reference_ttl_ticks
        self.last_reason = 'uninitialized'
        self._last_intent = None

    def plan(self, intent, tick):
        if not _tick(tick):
            raise ValueError('Planner tick must be a nonnegative integer')
        reason = None
        if not isinstance(intent, BehaviorIntent):
            reason = 'missing_or_invalid_intent'
        elif intent.episode_id != self.episode_id or intent.vehicle_id != self.vehicle_id:
            reason = 'intent_identity_mismatch'
        elif not _identity(intent.path_id) or intent.path_id not in self.routes:
            reason = 'unknown_path'
        elif not _lifetime(intent.issued_tick, intent.expires_tick, tick, 36):
            reason = 'invalid_intent_time'
        elif not _finite(intent.target_speed_m_s) or not 0 <= intent.target_speed_m_s <= 3:
            reason = 'invalid_intent_speed'
        elif self._last_intent is not None and (
            intent.issued_tick < self._last_intent.issued_tick
            or (intent.issued_tick == self._last_intent.issued_tick and intent != self._last_intent)
        ):
            reason = 'replayed_or_conflicting_intent'
        if reason:
            self.last_reason = reason
            return None
        self._last_intent = intent
        self.last_reason = 'known_route_selected'
        return PathReference(self.episode_id, self.vehicle_id, intent.path_id, tick,
            min(tick + self.reference_ttl_ticks, intent.expires_tick),
            float(intent.target_speed_m_s), self.routes[intent.path_id].length_m)


@dataclass(frozen=True)
class FollowerConfig:
    wheelbase_m: float = 3.2
    rear_axle_offset_m: float = 1.6
    max_steering_rad: float = 0.5
    max_speed_m_s: float = 3.0
    lookahead_base_m: float = 3.0
    lookahead_time_s: float = 0.6
    throttle_kp: float = 0.7
    brake_kp: float = 0.35
    planning_deceleration_m_s2: float = 1.5
    stop_buffer_m: float = 0.2
    speed_deadband_m_s: float = 0.03
    max_heading_error_rad: float = 1.0
    state_max_age_ticks: int = 2
    command_ttl_ticks: int = 12

    def __post_init__(self):
        values = (self.wheelbase_m, self.rear_axle_offset_m, self.max_steering_rad,
                  self.max_speed_m_s, self.lookahead_base_m, self.lookahead_time_s,
                  self.throttle_kp, self.brake_kp, self.planning_deceleration_m_s2,
                  self.stop_buffer_m, self.speed_deadband_m_s, self.max_heading_error_rad)
        if not all(_finite(v) and v > 0 for v in values):
            raise ValueError('Controller geometry/gains/limits must be finite and positive')
        if not self.rear_axle_offset_m <= self.wheelbase_m:
            raise ValueError('Rear-axle offset must not exceed wheelbase')
        if self.max_steering_rad > 0.5 or self.max_speed_m_s > 3 or self.max_heading_error_rad >= math.pi / 2:
            raise ValueError('This low-speed controller supports at most 3 m/s and 0.5 rad steering')
        if not _tick(self.state_max_age_ticks) or self.state_max_age_ticks > 2:
            raise ValueError('State maximum age must be 0..2 physics ticks')
        if not _tick(self.command_ttl_ticks) or not 1 <= self.command_ttl_ticks <= 12:
            raise ValueError('Command TTL must be 1..12 physics ticks')


class PathFollower:
    """Rear-axle pure pursuit plus bounded speed servo, with explicit fallback.

    ``command`` runs at control rate. Feed its result into DriverControlGate;
    feed None to the gate on intervening physics ticks. Invalid input produces
    a fresh zero-drive/full-brake command and diagnostic fallback reason. The
    gate still applies steering slew and independently expires commands. This
    is not a guaranteed stopping-distance or collision-avoidance controller.
    """

    def __init__(self, routes: Mapping, episode_id, vehicle_id, config=None):
        self.routes = _routes(routes)
        self.config = FollowerConfig() if config is None else config
        if not isinstance(self.config, FollowerConfig):
            raise ValueError('config must be FollowerConfig')
        self.reset(episode_id=episode_id, vehicle_id=vehicle_id)

    def reset(self, *, episode_id, vehicle_id):
        _identities(episode_id, vehicle_id)
        self.episode_id, self.vehicle_id = episode_id, vehicle_id
        self._last_tick = None
        self._last_reference = None
        self._sequence = 0
        self._stopped_path = None
        self.last_diagnostics = {'reason': 'reset', 'is_fallback': True, 'fallback': True, 'source': SOURCE}

    def _invalid(self, state, tick, reference):
        if not isinstance(state, VehicleState):
            return 'missing_or_invalid_state'
        if state.episode_id != self.episode_id or state.vehicle_id != self.vehicle_id:
            return 'state_identity_mismatch'
        if state.frame != FRAME or state.source != SOURCE:
            return 'state_frame_or_source_mismatch'
        if not _tick(state.tick) or not 0 <= tick - state.tick <= self.config.state_max_age_ticks:
            return 'stale_or_future_state'
        if not all(_finite(v) for v in (state.x_m, state.y_m, state.yaw_rad, state.speed_m_s)) or state.speed_m_s < 0:
            return 'nonfinite_or_invalid_state'
        if not isinstance(reference, PathReference):
            return 'missing_or_invalid_reference'
        if reference.episode_id != self.episode_id or reference.vehicle_id != self.vehicle_id:
            return 'reference_identity_mismatch'
        if reference.frame != FRAME or reference.source not in (SOURCE, SENSOR_PATH_SOURCE):
            return 'reference_frame_or_source_mismatch'
        if not _identity(reference.path_id) or reference.path_id not in self.routes:
            return 'unknown_path'
        if not _lifetime(reference.issued_tick, reference.expires_tick, tick, 24):
            return 'stale_or_future_reference'
        if (not _finite(reference.target_speed_m_s)
            or not 0 <= reference.target_speed_m_s <= self.config.max_speed_m_s
            or not _finite(reference.stop_s_m)
            or not 0 < reference.stop_s_m <= self.routes[reference.path_id].length_m):
            return 'invalid_reference_limits'
        if self._last_reference is not None and (
            reference.issued_tick < self._last_reference.issued_tick
            or (reference.issued_tick == self._last_reference.issued_tick and reference != self._last_reference)
        ):
            return 'replayed_or_conflicting_reference'
        return None

    def _make_command(self, tick, steering, throttle, brake, expires_tick):
        command = DriverCommand(self.episode_id, self.vehicle_id, self._sequence,
                                tick, expires_tick, steering, throttle, brake)
        self._sequence += 1
        return command

    def _fallback(self, tick, reason):
        self.last_diagnostics = {'reason': reason, 'is_fallback': True, 'fallback': True, 'source': SOURCE,
                                 'target_speed_m_s': 0.0}
        return self._make_command(tick, 0.0, 0.0, 1.0, tick + self.config.command_ttl_ticks)

    def command(self, state, tick, reference):
        if not _tick(tick) or (self._last_tick is not None and tick <= self._last_tick):
            raise ValueError('Control tick must strictly increase; stop the runner if its clock is broken')
        self._last_tick = tick
        reason = self._invalid(state, tick, reference)
        if reason:
            return self._fallback(tick, reason)
        self._last_reference = reference
        cfg, route = self.config, self.routes[reference.path_id]
        center = route.project(state.x_m, state.y_m)
        rear_x = state.x_m - cfg.rear_axle_offset_m * math.cos(state.yaw_rad)
        rear_y = state.y_m - cfg.rear_axle_offset_m * math.sin(state.yaw_rad)
        rear = route.project(rear_x, rear_y)
        heading_error = math.atan2(math.sin(center.point.yaw_rad - state.yaw_rad),
                                   math.cos(center.point.yaw_rad - state.yaw_rad))
        if abs(center.lateral_error_m) > route.width_m / 2:
            return self._fallback(tick, 'outside_lane_recovery_envelope')
        if abs(heading_error) > cfg.max_heading_error_rad:
            return self._fallback(tick, 'heading_outside_recovery_envelope')
        remaining = reference.stop_s_m - center.s_m
        if remaining <= cfg.stop_buffer_m:
            self._stopped_path = reference.path_id
        at_endpoint = self._stopped_path == reference.path_id
        lookahead = cfg.lookahead_base_m + cfg.lookahead_time_s * state.speed_m_s
        target_s = min(reference.stop_s_m, rear.s_m + lookahead)
        target = route.evaluate(target_s)
        dx, dy = target.position_xy[0] - rear_x, target.position_xy[1] - rear_y
        local_x = math.cos(state.yaw_rad) * dx + math.sin(state.yaw_rad) * dy
        local_y = -math.sin(state.yaw_rad) * dx + math.cos(state.yaw_rad) * dy
        chord_squared = dx * dx + dy * dy
        if not at_endpoint and (local_x <= 0 or chord_squared <= 1e-9):
            return self._fallback(tick, 'pursuit_target_not_ahead')
        curvature = 0.0 if at_endpoint else 2 * local_y / chord_squared
        steering = max(-cfg.max_steering_rad, min(cfg.max_steering_rad,
                                               math.atan(cfg.wheelbase_m * curvature)))
        target_speed = min(reference.target_speed_m_s,
                           math.sqrt(2 * cfg.planning_deceleration_m_s2 * max(0.0, remaining - cfg.stop_buffer_m)))
        error = target_speed - state.speed_m_s
        if at_endpoint or target_speed <= 0:
            throttle, brake, reason = 0.0, 1.0, 'endpoint_hold' if at_endpoint else 'behavior_stop'
        elif error < -cfg.speed_deadband_m_s:
            throttle, brake, reason = 0.0, min(1.0, -error * cfg.brake_kp), 'speed_braking'
        elif error > cfg.speed_deadband_m_s:
            throttle, brake, reason = min(1.0, error * cfg.throttle_kp), 0.0, 'tracking'
        else:
            throttle, brake, reason = 0.0, 0.0, 'speed_coasting'
        self.last_diagnostics = dict(
            reason=reason, is_fallback=False, fallback=False, source=SOURCE, path_id=reference.path_id,
            state_source=state.source, path_source=reference.source,
            center_progress_m=center.s_m, rear_progress_m=rear.s_m,
            center_lateral_error_m=center.lateral_error_m, heading_error_rad=heading_error,
            remaining_m=remaining, stop_s_m=reference.stop_s_m, endpoint_latched=at_endpoint,
            lookahead_m=lookahead, target_s_m=target_s, target_xy_m=target.position_xy,
            target_speed_m_s=target_speed, requested_steering_rad=steering,
            state_age_ticks=tick - state.tick, reference_age_ticks=tick - reference.issued_tick)
        return self._make_command(tick, steering, throttle, brake,
                                  min(tick + cfg.command_ttl_ticks, reference.expires_tick))
