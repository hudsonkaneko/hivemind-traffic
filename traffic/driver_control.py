"""Simulator-independent, tick-expiring low-level driver commands.

This module does not move a vehicle. A physics adapter consumes ``AppliedControl``
and applies steering and wheel torques; Isaac/PhysX remains the movement authority.
The coordinate convention is X forward, Y left, Z up, with positive steering left.

Call ``step`` once per physics tick (for example at 120 Hz). A slower controller
(for example 60 Hz) supplies a command only when it has a new one. ``None`` means
no new delivery, not "cancel": the last accepted command is held until its
exclusive expiry tick. Missing/expired or invalid commands fail safe immediately.
Expiry and sequence numbers belong to an episode/vehicle identity, not wall time.

No promise about real stopping distances follows from these software checks.
Brake torque, tire contact, and vehicle stability require separate physics tests.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real


def _finite_number(value: object) -> bool:
    if not isinstance(value, Real) or isinstance(value, bool):
        return False
    try:
        return math.isfinite(value)
    except (OverflowError, TypeError, ValueError):
        return False


def _nonnegative_integer(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def _identity(value: object, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a nonempty string")
    return value


@dataclass(frozen=True)
class ControlLimits:
    """Limits at the road-wheel/bicycle steering interface, in radians and seconds."""

    max_steering_rad: float = 0.5
    steering_rate_rad_s: float = 0.5
    fallback_brake: float = 1.0

    def __post_init__(self) -> None:
        if not _finite_number(self.max_steering_rad) or not 0 < self.max_steering_rad < math.pi / 2:
            raise ValueError("max_steering_rad must be finite and between 0 and pi/2")
        if not _finite_number(self.steering_rate_rad_s) or self.steering_rate_rad_s <= 0:
            raise ValueError("steering_rate_rad_s must be finite and positive")
        if not _finite_number(self.fallback_brake) or not 0 < self.fallback_brake <= 1:
            raise ValueError("fallback_brake must be finite and in (0, 1]")


@dataclass(frozen=True)
class DriverCommand:
    """Untrusted delivery: validated by ``DriverControlGate``, not at construction.

    ``sequence`` strictly increases for each new command in one episode. The
    valid interval is ``issued_tick <= current_tick < expires_tick``. Steering
    is a bicycle-equivalent road-wheel angle in radians; throttle/brake are
    normalized [0, 1] requests, not a speed or deceleration guarantee. Forward
    propulsion only is supported in this foundation milestone.
    """

    episode_id: str
    vehicle_id: str
    sequence: int
    issued_tick: int
    expires_tick: int
    steering_rad: float
    throttle: float
    brake: float


@dataclass(frozen=True)
class AppliedControl:
    """Validated, rate-limited output suitable for ``dataclasses.asdict`` logging."""

    steering_rad: float
    throttle: float
    brake: float
    reason: str
    command_sequence: int | None
    is_fallback: bool


class DriverControlGate:
    """Validate identity, ordering, lifetime and action limits before actuation.

    Malformed, wrong-identity, future, stale, or replayed deliveries clear the
    held command. The fallback is zero propulsion, configured braking, and
    steering returning toward zero at the normal rate limit. An exact duplicate
    of the *currently held* command is harmless and does not extend its expiry;
    conflicting contents at the same sequence fail safe. Recovery requires a
    valid command newer than the last accepted sequence, or an explicit reset.

    Invalid local tick/dt arguments raise ``ValueError`` rather than advancing
    internal state: a runner with a broken clock must stop its physics loop.
    Ticks must strictly increase; dt is elapsed simulated time, never wall time.
    """

    def __init__(
        self,
        episode_id: str,
        vehicle_id: str,
        limits: ControlLimits | None = None,
    ) -> None:
        self.limits = limits if limits is not None else ControlLimits()
        if not isinstance(self.limits, ControlLimits):
            raise ValueError("limits must be ControlLimits")
        self.reset(episode_id=episode_id, vehicle_id=vehicle_id)

    def reset(self, *, episode_id: str, vehicle_id: str) -> None:
        """Clear command/sequence/tick history at a physical episode reset only."""
        episode = _identity(episode_id, "episode_id")
        vehicle = _identity(vehicle_id, "vehicle_id")
        self.episode_id = episode
        self.vehicle_id = vehicle
        self._held: DriverCommand | None = None
        self._last_sequence = -1
        self._last_tick: int | None = None
        self._steering_rad = 0.0

    def _reject_reason(self, command: object, tick: int) -> str | None:
        if not isinstance(command, DriverCommand):
            return "invalid_type"
        if command.episode_id != self.episode_id or command.vehicle_id != self.vehicle_id:
            return "identity_mismatch"
        if not _nonnegative_integer(command.sequence):
            return "invalid_sequence"
        if (
            not _nonnegative_integer(command.issued_tick)
            or not _nonnegative_integer(command.expires_tick)
            or command.expires_tick <= command.issued_tick
        ):
            return "invalid_timestamp"
        if not all(_finite_number(value) for value in (command.steering_rad, command.throttle, command.brake)):
            return "invalid_controls"
        if (
            abs(command.steering_rad) > self.limits.max_steering_rad
            or not 0 <= command.throttle <= 1
            or not 0 <= command.brake <= 1
        ):
            return "invalid_controls"
        if command.issued_tick > tick:
            return "future_command"
        if command.expires_tick <= tick:
            return "expired_command"
        if command.sequence < self._last_sequence:
            return "replayed_sequence"
        if command.sequence == self._last_sequence and command != self._held:
            return "sequence_conflict"
        return None

    def step(self, *, tick: int, dt_s: float, command: DriverCommand | None = None) -> AppliedControl:
        """Advance the gate once, optionally accepting a new controller delivery."""
        if not _nonnegative_integer(tick) or (self._last_tick is not None and tick <= self._last_tick):
            raise ValueError("tick must be a strictly increasing nonnegative integer")
        if not _finite_number(dt_s) or dt_s <= 0:
            raise ValueError("dt_s must be finite and positive")
        self._last_tick = tick
        reason = "held"
        if command is not None:
            rejection = self._reject_reason(command, tick)
            if rejection is not None:
                self._held = None
                reason = rejection
            elif command == self._held:
                reason = "duplicate_held"
            else:
                self._held = command
                self._last_sequence = command.sequence
                reason = "accepted"
        elif self._held is None:
            reason = "missing"

        if self._held is not None and tick >= self._held.expires_tick:
            self._held = None
            reason = "expired"

        fallback = self._held is None
        target_steering = 0.0 if fallback else float(self._held.steering_rad)
        max_change = float(self.limits.steering_rate_rad_s) * float(dt_s)
        steering_change = max(-max_change, min(max_change, target_steering - self._steering_rad))
        self._steering_rad += steering_change
        brake = float(self.limits.fallback_brake) if fallback else float(self._held.brake)
        # A nonzero brake request always suppresses propulsion.
        throttle = 0.0 if fallback or brake > 0 else float(self._held.throttle)
        return AppliedControl(
            steering_rad=self._steering_rad,
            throttle=throttle,
            brake=brake,
            reason=reason,
            command_sequence=None if fallback else self._held.sequence,
            is_fallback=fallback,
        )


def ackermann_steering(steering_rad: float, wheelbase_m: float, track_width_m: float) -> tuple[float, float]:
    """Convert bicycle steering into (front-left, front-right) angles in radians.

    Positive is left for both wheels. This is geometry only, not a lateral tire
    or stability model. Reject geometry whose turn center is within the axle.
    """
    if not _finite_number(steering_rad) or abs(steering_rad) >= math.pi / 2:
        raise ValueError("steering_rad must be finite and strictly between -pi/2 and pi/2")
    if not _finite_number(wheelbase_m) or wheelbase_m <= 0:
        raise ValueError("wheelbase_m must be finite and positive")
    if not _finite_number(track_width_m) or track_width_m <= 0:
        raise ValueError("track_width_m must be finite and positive")
    if steering_rad == 0:
        return (0.0, 0.0)
    radius = wheelbase_m / math.tan(steering_rad)
    if abs(radius) <= track_width_m / 2:
        raise ValueError("turn radius must exceed half the track width")
    return (
        math.atan(wheelbase_m / (radius - track_width_m / 2)),
        math.atan(wheelbase_m / (radius + track_width_m / 2)),
    )


def wheel_torques(
    control: AppliedControl,
    max_drive_torque_nm: float,
    max_brake_torque_nm: float,
    *,
    driven_wheels: tuple[int, ...] = (0, 1),
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """Return (drive, brake) torque magnitudes for FL, FR, RL, RR wheels.

    Maximum torques are **per wheel**, not axle/vehicle totals. All four wheels
    brake; only selected driven wheels propel forward. There is no reverse,
    speed servo, ABS, or tire-friction compensation here. The physics adapter
    is responsible for its own wheel-axis sign convention.
    """
    if not isinstance(control, AppliedControl):
        raise ValueError("control must be AppliedControl")
    if not all(_finite_number(value) and 0 <= value <= 1 for value in (control.throttle, control.brake)):
        raise ValueError("control throttle/brake must be finite and in [0, 1]")
    if not all(_finite_number(value) and value >= 0 for value in (max_drive_torque_nm, max_brake_torque_nm)):
        raise ValueError("torque limits must be finite and nonnegative")
    if not isinstance(driven_wheels, tuple) or any(
        not _nonnegative_integer(index) or index > 3 for index in driven_wheels
    ) or len(set(driven_wheels)) != len(driven_wheels):
        raise ValueError("driven_wheels must be a tuple of unique wheel indices in [0, 3]")
    throttle = 0.0 if control.brake > 0 else control.throttle
    drive = tuple(float(throttle * max_drive_torque_nm) if index in driven_wheels else 0.0 for index in range(4))
    brake = (float(control.brake * max_brake_torque_nm),) * 4
    return drive, brake
