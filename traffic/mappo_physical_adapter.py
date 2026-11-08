"""Frozen high-level policy to native wheel-control bridge, without learning.

This bounded first adapter supports parallel, straight LaneRoute lanes. Lane 0
is the supplied centreline; increasing lane indices move left, matching SUMO.
States are explicitly privileged simulator odometry. An upstream traffic guard
must authorize lane changes; this module neither perceives nor predicts traffic.
PhysX remains the sole pose authority. Feed returned DriverCommands through
DriverControlGate on every physics tick, including ticks without a new command.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from traffic.driver_control import DriverCommand
from traffic.lane_geometry import LaneRoute, wrap_yaw
from traffic.path_following import FRAME, SOURCE, VehicleState, _finite, _identity, _tick


@dataclass(frozen=True)
class LongitudinalCalibration:
    """Local forward-response fit, not a real-car calibration or safety proof.

    Acceleration = throttle_gain*T - brake_gain*B - coast - speed_gain*v.
    Torque/mass/friction settings must match the fitting run. The default is
    explicitly provisional. Calibration data/provenance belong in the run
    manifest. The validity envelope is checked before each inversion. Ordinary
    braking is bounded by ``max_linear_brake_fraction``; full emergency/stopped
    braking remains independent and has no affine acceleration prediction.
    """

    throttle_gain_m_s2: float = 2.0
    brake_gain_m_s2: float = 6.0
    coast_m_s2: float = 0.0
    coast_speed_gain_s_inv: float = 0.0
    max_speed_m_s: float = 26.8224
    source: str = 'provisional_unmeasured'
    revision: str = 'longitudinal_affine_v1'
    max_linear_brake_fraction: float = 1.0

    def __post_init__(self):
        if not all(_finite(v) and v > 0 for v in (
                self.throttle_gain_m_s2, self.brake_gain_m_s2, self.max_speed_m_s)):
            raise ValueError('Calibration gains and speed bound must be positive finite numbers')
        if not all(_finite(v) and v >= 0 for v in (self.coast_m_s2, self.coast_speed_gain_s_inv)):
            raise ValueError('Coasting resistance must be finite and nonnegative')
        if not _identity(self.source) or self.revision != 'longitudinal_affine_v1':
            raise ValueError('Calibration source and supported revision are required')
        if (not _finite(self.max_linear_brake_fraction)
                or not 0 < self.max_linear_brake_fraction <= 1):
            raise ValueError('Linear brake support must be a finite fraction in (0,1]')

    def predict(self, speed_m_s, throttle, brake):
        if (not _finite(speed_m_s) or not 0 <= speed_m_s <= self.max_speed_m_s
                or not all(_finite(v) and 0 <= v <= 1 for v in (throttle, brake))
                or brake > self.max_linear_brake_fraction
                or throttle * brake > 0):
            raise ValueError('Response prediction outside calibration/control envelope')
        return (self.throttle_gain_m_s2 * throttle - self.brake_gain_m_s2 * brake
                - self.coast_m_s2 - self.coast_speed_gain_s_inv * speed_m_s)

    def invert(self, speed_m_s, acceleration_m_s2):
        if not _finite(acceleration_m_s2):
            raise ValueError('Requested acceleration must be finite')
        resistance = -self.predict(speed_m_s, 0.0, 0.0)
        effort = acceleration_m_s2 + resistance
        if effort >= 0:
            return min(1.0, effort / self.throttle_gain_m_s2), 0.0
        return 0.0, min(self.max_linear_brake_fraction, -effort / self.brake_gain_m_s2)


@dataclass(frozen=True)
class AdapterConfig:
    physics_hz: int = 120
    policy_hz: int = 10
    command_ttl_ticks: int = 12
    intent_max_ttl_ticks: int = 24
    state_max_age_ticks: int = 2
    lane_width_m: float = 3.7
    lane_count: int = 3
    wheelbase_m: float = 3.2
    rear_axle_offset_m: float = 1.6
    max_steering_rad: float = 0.5
    max_speed_m_s: float = 26.8224
    min_acceleration_m_s2: float = -4.5
    max_acceleration_m_s2: float = 2.0
    lane_change_time_s: float = 4.0
    lane_change_min_length_m: float = 40.0
    lane_settle_tolerance_m: float = 0.25
    lane_settle_heading_rad: float = 0.10
    lookahead_base_m: float = 3.0
    lookahead_time_s: float = 0.6
    max_tracking_error_m: float = 0.9
    max_heading_error_rad: float = 0.6
    acceleration_feedback_gain: float = 0.15
    acceleration_filter_time_s: float = 0.15
    stop_hold_speed_m_s: float = 0.05

    def __post_init__(self):
        integers = (self.physics_hz, self.policy_hz, self.command_ttl_ticks,
                    self.intent_max_ttl_ticks, self.lane_count)
        if not all(_tick(v) and v > 0 for v in integers):
            raise ValueError('Rates, lifetimes and lane count must be positive integers')
        if (self.physics_hz != 120 or self.physics_hz % self.policy_hz
                or self.command_ttl_ticks > 12 or self.intent_max_ttl_ticks > 24
                or not _tick(self.state_max_age_ticks) or self.state_max_age_ticks > 2):
            raise ValueError('Adapter requires 120 Hz physics and bounded command/state lifetimes')
        positive = (self.lane_width_m, self.wheelbase_m, self.rear_axle_offset_m,
                    self.max_steering_rad, self.max_speed_m_s, self.max_acceleration_m_s2,
                    self.lane_change_time_s, self.lane_change_min_length_m,
                    self.lane_settle_tolerance_m, self.lane_settle_heading_rad,
                    self.lookahead_base_m, self.lookahead_time_s,
                    self.max_tracking_error_m, self.max_heading_error_rad,
                    self.acceleration_filter_time_s, self.stop_hold_speed_m_s)
        if not all(_finite(v) and v > 0 for v in positive):
            raise ValueError('Geometry, controller gains and bounds must be finite and positive')
        if (not _finite(self.min_acceleration_m_s2) or self.min_acceleration_m_s2 >= 0
                or not _finite(self.acceleration_feedback_gain)
                or not 0 <= self.acceleration_feedback_gain <= 1
                or self.max_steering_rad > 0.5 or self.max_speed_m_s > 26.8224
                or self.max_acceleration_m_s2 > 2 or self.min_acceleration_m_s2 < -4.5
                or self.rear_axle_offset_m > self.wheelbase_m
                or self.max_heading_error_rad >= math.pi / 2):
            raise ValueError('Adapter exceeds declared physical/policy limits')


@dataclass(frozen=True)
class PolicyIntent:
    """Untrusted timestamped policy output in physical units.

    A zero lane delta holds the selected destination, including during a lane
    change. A committed transition finishes before accepting another lane delta.
    ``lane_change_allowed`` is explicit upstream gap authorization for this
    observation; its default is false. Hold semantics never extend expiry.
    """

    episode_id: str
    vehicle_id: str
    sequence: int
    issued_tick: int
    expires_tick: int
    acceleration_m_s2: float
    lane_delta: int
    lane_change_allowed: bool = False


@dataclass(frozen=True)
class LaneTransition:
    from_offset_m: float
    to_offset_m: float
    start_s_m: float
    length_m: float

    def __post_init__(self):
        if (not all(_finite(v) for v in (self.from_offset_m, self.to_offset_m,
                                        self.start_s_m, self.length_m))
                or self.length_m <= 0):
            raise ValueError('Lane transition requires finite geometry and positive length')

    def offset_and_derivative(self, station_m):
        if not _finite(station_m):
            raise ValueError('Station must be finite')
        q = max(0.0, min(1.0, (station_m - self.start_s_m) / self.length_m))
        blend = q ** 3 * (10 + q * (-15 + 6 * q))
        slope = 30 * q * q * (1 - q) ** 2 / self.length_m
        shift = self.to_offset_m - self.from_offset_m
        return self.from_offset_m + shift * blend, shift * slope


class PhysicalPolicyAdapter:
    """Pure pursuit of a frozen quintic lane path plus calibrated effort control.

    ``command`` should run at 60 Hz. Tick is episode-local physics time; the
    native DriverControlGate owns final steering slew and command expiry.
    No position, orientation or velocity is ever written by this adapter.
    The runner must independently stop before the finite road ends. Fallback
    clears intent and acceleration history, retaining the committed lane path;
    a newer valid delivery can recover, while reset clears every controller state.
    """

    def __init__(self, route, episode_id, vehicle_id, *, config=None,
                 calibration=None, initial_lane=0):
        if not isinstance(route, LaneRoute) or any(s.curvature_rad_m != 0 for s in route.segments):
            raise ValueError('This adapter is qualified only for straight parallel LaneRoute lanes')
        self.route = route
        self.config = AdapterConfig() if config is None else config
        self.calibration = LongitudinalCalibration() if calibration is None else calibration
        if not isinstance(self.config, AdapterConfig) or not isinstance(self.calibration, LongitudinalCalibration):
            raise ValueError('Expected AdapterConfig and LongitudinalCalibration')
        self.reset(episode_id=episode_id, vehicle_id=vehicle_id, initial_lane=initial_lane)

    def reset(self, *, episode_id, vehicle_id, initial_lane=0):
        if not _identity(episode_id) or not _identity(vehicle_id):
            raise ValueError('Episode and vehicle identities are required')
        if not _tick(initial_lane) or initial_lane >= self.config.lane_count:
            raise ValueError('Initial lane outside configured road')
        self.episode_id, self.vehicle_id = episode_id, vehicle_id
        self.current_lane = self.target_lane = initial_lane
        self.transition = None
        self._last_tick = self._last_state = self._held = None
        self._last_intent_sequence = -1
        self._command_sequence = 0
        self._filtered_acceleration = 0.0
        self._completed_lane_changes = 0
        self.last_diagnostics = dict(reason='reset', is_fallback=True)

    def _make(self, tick, steering, throttle, brake, expires):
        command = DriverCommand(self.episode_id, self.vehicle_id, self._command_sequence,
                                tick, expires, steering, throttle, brake)
        self._command_sequence += 1
        return command

    def _fallback(self, tick, reason):
        self._held = self._last_state = None
        self._filtered_acceleration = 0.0
        self.last_diagnostics = dict(reason=reason, is_fallback=True, source=SOURCE,
                                     current_lane=self.current_lane, target_lane=self.target_lane,
                                     calibrated_acceleration_m_s2=None,
                                     calibration_model_applicable=False)
        return self._make(tick, 0.0, 0.0, 1.0, tick + self.config.command_ttl_ticks)

    def _intent_error(self, intent, tick):
        cfg = self.config
        if not isinstance(intent, PolicyIntent):
            return 'invalid_intent_type'
        if intent.episode_id != self.episode_id or intent.vehicle_id != self.vehicle_id:
            return 'intent_identity_mismatch'
        if (not _tick(intent.sequence) or not _tick(intent.issued_tick)
                or not _tick(intent.expires_tick)
                or not intent.issued_tick <= tick < intent.expires_tick
                or not 0 < intent.expires_tick - intent.issued_tick <= cfg.intent_max_ttl_ticks):
            return 'invalid_intent_time_or_sequence'
        if (not _finite(intent.acceleration_m_s2)
                or not cfg.min_acceleration_m_s2 <= intent.acceleration_m_s2 <= cfg.max_acceleration_m_s2
                or type(intent.lane_delta) is not int or intent.lane_delta not in (-1, 0, 1)
                or type(intent.lane_change_allowed) is not bool):
            return 'invalid_intent_action'
        if (intent.sequence < self._last_intent_sequence
                or (intent.sequence == self._last_intent_sequence and intent != self._held)):
            return 'replayed_or_conflicting_intent'
        return None

    def _path(self, station):
        if self.transition is None:
            offset, slope = self.target_lane * self.config.lane_width_m, 0.0
        else:
            offset, slope = self.transition.offset_and_derivative(station)
        base = self.route.evaluate(station)
        x, y = base.position_xy
        return (x - math.sin(base.yaw_rad) * offset,
                y + math.cos(base.yaw_rad) * offset,
                wrap_yaw(base.yaw_rad + math.atan(slope)), offset)

    def command(self, state, tick, intent=None):
        if not _tick(tick) or (self._last_tick is not None and tick <= self._last_tick):
            raise ValueError('Control ticks must strictly increase; stop on a broken clock')
        self._last_tick = tick
        cfg = self.config
        if (not isinstance(state, VehicleState) or state.episode_id != self.episode_id
                or state.vehicle_id != self.vehicle_id or state.frame != FRAME or state.source != SOURCE
                or not _tick(state.tick) or not 0 <= tick - state.tick <= cfg.state_max_age_ticks
                or not all(_finite(v) for v in (state.x_m, state.y_m, state.yaw_rad, state.speed_m_s))
                or state.speed_m_s < 0):
            return self._fallback(tick, 'invalid_or_stale_state')
        if state.speed_m_s > self.calibration.max_speed_m_s:
            return self._fallback(tick, 'outside_calibration_speed_envelope')
        new_intent = intent is not None and intent != self._held
        if intent is not None:
            reason = self._intent_error(intent, tick)
            if reason:
                return self._fallback(tick, reason)
            self._held = intent
            self._last_intent_sequence = intent.sequence
        if self._held is None or tick >= self._held.expires_tick:
            return self._fallback(tick, 'missing_or_expired_intent')
        intent = self._held
        center = self.route.project(state.x_m, state.y_m)
        base_heading_error = wrap_yaw(center.point.yaw_rad - state.yaw_rad)
        if not -cfg.lane_width_m / 2 <= center.lateral_error_m <= (cfg.lane_count - .5) * cfg.lane_width_m:
            return self._fallback(tick, 'outside_road_envelope')
        if abs(base_heading_error) > cfg.max_heading_error_rad:
            return self._fallback(tick, 'heading_outside_envelope')
        lane_reason = 'holding_lane'
        completed_now = False
        if self.transition is not None:
            lane_reason = 'lane_change_committed'
            if (center.s_m >= self.transition.start_s_m + self.transition.length_m
                    and abs(center.lateral_error_m - self.transition.to_offset_m) <= cfg.lane_settle_tolerance_m
                    and abs(base_heading_error) <= cfg.lane_settle_heading_rad):
                self.current_lane = self.target_lane
                self.transition = None
                self._completed_lane_changes += 1
                completed_now = True
                lane_reason = 'lane_change_completed'
        if new_intent and intent.lane_delta and self.transition is None and not completed_now:
            destination = self.current_lane + intent.lane_delta
            if not 0 <= destination < cfg.lane_count:
                lane_reason = 'lane_boundary_rejected'
            elif not intent.lane_change_allowed:
                lane_reason = 'upstream_lane_gap_rejected'
            else:
                length = max(cfg.lane_change_min_length_m, cfg.lane_change_time_s * state.speed_m_s)
                self.target_lane = destination
                self.transition = LaneTransition(self.current_lane * cfg.lane_width_m,
                                                 destination * cfg.lane_width_m, center.s_m, length)
                lane_reason = 'lane_change_started'
        _, path_y, path_heading, path_offset = self._path(center.s_m)
        tracking_error = center.lateral_error_m - path_offset
        if abs(tracking_error) > cfg.max_tracking_error_m:
            return self._fallback(tick, 'path_tracking_outside_envelope')
        rear_x = state.x_m - cfg.rear_axle_offset_m * math.cos(state.yaw_rad)
        rear_y = state.y_m - cfg.rear_axle_offset_m * math.sin(state.yaw_rad)
        rear_s = self.route.project(rear_x, rear_y).s_m
        lookahead = cfg.lookahead_base_m + cfg.lookahead_time_s * state.speed_m_s
        target_x, target_y, _, _ = self._path(rear_s + lookahead)
        dx, dy = target_x - rear_x, target_y - rear_y
        local_x = math.cos(state.yaw_rad) * dx + math.sin(state.yaw_rad) * dy
        local_y = -math.sin(state.yaw_rad) * dx + math.cos(state.yaw_rad) * dy
        chord_squared = dx * dx + dy * dy
        if local_x <= 0 or chord_squared <= 1e-9:
            return self._fallback(tick, 'pursuit_target_not_ahead')
        steering = max(-cfg.max_steering_rad, min(cfg.max_steering_rad,
                       math.atan(cfg.wheelbase_m * 2 * local_y / chord_squared)))
        measured_acceleration = None
        if self._last_state is not None and state.tick > self._last_state.tick:
            dt = (state.tick - self._last_state.tick) / cfg.physics_hz
            measured_acceleration = (state.speed_m_s - self._last_state.speed_m_s) / dt
            alpha = dt / (cfg.acceleration_filter_time_s + dt)
            self._filtered_acceleration += alpha * (measured_acceleration - self._filtered_acceleration)
        self._last_state = state
        requested = float(intent.acceleration_m_s2)
        # Limit requested speed change over one policy period. This is not a
        # substitute for an upstream following/obstacle safety guard.
        limited = min(requested, max(cfg.min_acceleration_m_s2,
                      (cfg.max_speed_m_s - state.speed_m_s) * cfg.policy_hz))
        limited = max(limited, -state.speed_m_s * cfg.policy_hz)
        corrected = limited + cfg.acceleration_feedback_gain * (limited - self._filtered_acceleration)
        corrected = max(cfg.min_acceleration_m_s2, min(cfg.max_acceleration_m_s2, corrected))
        if limited < requested:
            corrected = min(corrected, limited)
        if state.speed_m_s >= cfg.max_speed_m_s:
            corrected = min(corrected, limited, 0.0)
        throttle, brake = self.calibration.invert(state.speed_m_s, corrected)
        reason = 'tracking'
        if state.speed_m_s <= cfg.stop_hold_speed_m_s and requested <= 0:
            throttle, brake, reason = 0.0, 1.0, 'stopped_hold'
        model_applicable = reason != 'stopped_hold' and brake <= self.calibration.max_linear_brake_fraction
        predicted = self.calibration.predict(state.speed_m_s, throttle, brake) if model_applicable else None
        self.last_diagnostics = dict(
            reason=reason, is_fallback=False, source=SOURCE, lane_reason=lane_reason,
            current_lane=self.current_lane, lane_index=self.current_lane, target_lane=self.target_lane,
            lane_change_active=self.transition is not None,
            completed_lane_changes=self._completed_lane_changes,
            requested_acceleration_m_s2=requested, limited_acceleration_m_s2=limited,
            corrected_acceleration_m_s2=corrected, calibrated_acceleration_m_s2=predicted,
            measured_acceleration_m_s2=measured_acceleration,
            filtered_measured_acceleration_m_s2=self._filtered_acceleration,
            speed_ceiling_limited=limited < requested,
            calibration_source=self.calibration.source, calibration_revision=self.calibration.revision,
            calibration_model_applicable=model_applicable,
            max_linear_brake_fraction=self.calibration.max_linear_brake_fraction,
            effort_saturated=throttle >= 1 or brake >= self.calibration.max_linear_brake_fraction,
            station_m=center.s_m, lateral_position_m=center.lateral_error_m,
            path_offset_m=path_offset, reference_y_m=path_y, tracking_error_m=tracking_error,
            heading_error_rad=wrap_yaw(path_heading - state.yaw_rad),
            requested_steering_rad=steering, throttle=throttle, brake=brake,
            intent_age_ticks=tick - intent.issued_tick, state_age_ticks=tick - state.tick,
            upstream_lane_change_allowed=intent.lane_change_allowed)
        return self._make(tick, steering, throttle, brake,
                          min(tick + cfg.command_ttl_ticks, intent.expires_tick))
