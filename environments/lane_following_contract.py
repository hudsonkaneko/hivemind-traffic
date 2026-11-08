"""CPU contract for one low-speed, simulator-state Isaac Lab lane task.

The task stops at station 60 m on the existing 100 m maps. It does not claim
sensor autonomy, traffic safety, a trained policy, or completion of the full map.
All motion belongs to the native physics adapter; this module never writes poses.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import math
from numbers import Real
import random

from environments.physics_vehicle_contract import ACTION_HIGH, ACTION_LOW, ACTION_NAMES, bounded_action
from traffic.lane_geometry import LaneRoute, wrap_yaw
from traffic.lane_validation import footprint_lateral_bound

OBSERVATION_SOURCE = 'privileged_simulator_state_and_known_map'
OBSERVATION_NAMES = (
    'progress_m', 'remaining_m', 'lateral_error_m', 'sin_heading_error', 'cos_heading_error',
    'forward_speed_m_s', 'lateral_speed_m_s', 'planar_speed_m_s', 'vertical_speed_m_s',
    'height_m', 'upright_z', 'curvature_rad_m', 'curvature_3m_ahead_rad_m',
    'curvature_6m_ahead_rad_m', 'target_cruise_speed_m_s', 'target_speed_m_s',
    'applied_steering_rad', 'applied_throttle', 'applied_brake',
    'elapsed_fraction', 'settling', 'hold_fraction',
)


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    return float(value)


def _integer(value, name, minimum=0):
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(f'{name} must be an integer >= {minimum}')
    return value


@dataclass(frozen=True)
class LaneTaskConfig:
    physics_hz: int = 120
    control_hz: int = 60
    settle_s: float = 2.0
    episode_length_s: float = 35.0
    stop_s_m: float = 60.0
    target_speed_m_s: float = 3.0
    endpoint_tolerance_m: float = 0.5
    stop_speed_m_s: float = 0.05
    success_hold_s: float = 1.0
    lateral_reset_bound_m: float = 0.15
    yaw_reset_bound_rad: float = 0.035
    upright_min_z: float = math.cos(math.radians(5))
    speed_max_m_s: float = 4.0
    reverse_speed_max_m_s: float = 0.2
    heading_error_max_rad: float = 1.0
    min_height_m: float = 0.0
    max_height_m: float = 2.0
    planning_deceleration_m_s2: float = 1.5
    stop_buffer_m: float = 0.2

    def __post_init__(self):
        for name in ('physics_hz', 'control_hz'):
            _integer(getattr(self, name), name, 1)
        if (self.physics_hz, self.control_hz) != (120, 60):
            raise ValueError('Lane task requires 120 Hz physics and 60 Hz controls')
        for name, value in asdict(self).items():
            _number(value, name)
        # This is a bounded first task, not a general configuration surface.
        expected = LaneTaskConfig.__dataclass_fields__
        if any(getattr(self, name) != field.default for name, field in expected.items()):
            raise ValueError('Lane task v1 uses its frozen limits; declare a new protocol before changing them')

    @property
    def control_ticks(self):
        return self.physics_hz // self.control_hz


@dataclass(frozen=True)
class ResetSpec:
    route_id: str
    seed: int
    episode_index: int
    station_m: float
    lateral_offset_m: float
    yaw_error_rad: float
    x_m: float
    y_m: float
    yaw_rad: float


def make_reset_spec(route, *, seed, episode_index=0, perturb=False,
                    lateral_offset_m=None, yaw_error_rad=None):
    """Deterministic bounded reset; the native adapter retains authored height.

    A private hash-derived RNG makes samples independent of global RNG state and
    case execution order. Explicit corner values override only their own axis.
    Root velocity and wheel state must be zeroed by the adapter, only at reset.
    """
    if not isinstance(route, LaneRoute):
        raise ValueError('route must be a LaneRoute')
    _integer(seed, 'seed')
    _integer(episode_index, 'episode_index')
    if seed > 2**32-1:
        raise ValueError('seed must fit an unsigned 32-bit value')
    if not isinstance(perturb, bool):
        raise ValueError('perturb must be bool')
    cfg = LaneTaskConfig()
    key = f'lane-task-v1|{seed}|{episode_index}|{route.route_id}'.encode('utf-8')
    rng = random.Random(int.from_bytes(hashlib.sha256(key).digest(), 'big'))
    lateral = rng.uniform(-cfg.lateral_reset_bound_m, cfg.lateral_reset_bound_m) if perturb else 0.0
    heading = rng.uniform(-cfg.yaw_reset_bound_rad, cfg.yaw_reset_bound_rad) if perturb else 0.0
    if lateral_offset_m is not None:
        lateral = _number(lateral_offset_m, 'lateral_offset_m')
    if yaw_error_rad is not None:
        heading = _number(yaw_error_rad, 'yaw_error_rad')
    if abs(lateral) > cfg.lateral_reset_bound_m or abs(heading) > cfg.yaw_reset_bound_rad:
        raise ValueError('Reset perturbation exceeds the frozen task envelope')
    point = route.evaluate(0.0)
    x = point.position_xy[0] - math.sin(point.yaw_rad) * lateral
    y = point.position_xy[1] + math.cos(point.yaw_rad) * lateral
    return ResetSpec(route.route_id, seed, episode_index, 0.0, lateral, heading,
                     x, y, wrap_yaw(point.yaw_rad + heading))


def _state(state):
    """Validate without replacing missing or nonfinite physical measurements."""
    if not isinstance(state, dict):
        raise ValueError('Physics state must be a dictionary')
    values = {}
    for name, size in (('position_m', 3), ('velocity_m_s', 3), ('quaternion_xyzw', 4)):
        raw = state.get(name)
        if not isinstance(raw, (tuple, list)) or len(raw) != size:
            raise ValueError(f'{name} must contain {size} finite numbers')
        values[name] = tuple(_number(v, name) for v in raw)
    for name in ('yaw_rad', 'upright_z', 'speed_m_s'):
        values[name] = _number(state.get(name), name)
    qnorm = math.sqrt(sum(v*v for v in values['quaternion_xyzw']))
    if abs(qnorm - 1.0) > 1e-3:
        raise ValueError('Physics quaternion must be normalized')
    speed = math.hypot(*values['velocity_m_s'][:2])
    if values['speed_m_s'] < 0 or abs(values['speed_m_s'] - speed) > 1e-5:
        raise ValueError('Physics speed must agree with planar velocity')
    if not -1.000001 <= values['upright_z'] <= 1.000001:
        raise ValueError('upright_z must be a physical unit-axis projection')
    wheels = state.get('wheel_on_ground')
    if not isinstance(wheels, (tuple, list)) or len(wheels) != 4 or any(type(v) is not bool for v in wheels):
        raise ValueError('wheel_on_ground must contain four booleans')
    values['wheel_on_ground'] = tuple(wheels)
    return values


def _control(applied):
    values = tuple(_number(getattr(applied, name, None), name) for name in ACTION_NAMES)
    if values != bounded_action(values):
        raise ValueError('Applied control is outside bounds or violates brake priority')
    return values


@dataclass(frozen=True)
class RouteState:
    progress_m: float
    remaining_m: float
    lateral_error_m: float
    heading_error_rad: float
    forward_speed_m_s: float
    lateral_speed_m_s: float
    planar_speed_m_s: float
    footprint_lateral_bound_m: float
    target_speed_m_s: float


@dataclass(frozen=True)
class LaneTransition:
    tick: int
    elapsed_s: float
    observation: tuple
    route_state: RouteState
    reward_components: dict
    reward: float
    terminated: bool
    truncated: bool
    reason: str
    success: bool
    terminal_observation: tuple | None

    def as_dict(self):
        """Detached, JSON-compatible evidence, including pre-reset observations."""
        return asdict(self)


class LaneFollowingTask:
    """Stateful reward and terminal accounting at exactly one control interval.

    ``physics_guard`` is also called by the adapter at every completed physics
    tick. Its first nonempty reason must be latched into the next ``advance``.
    Malformed telemetry raises so the runner can stop; it is never zero-filled.
    """
    def __init__(self, route, config=None, dimensions_m=(4.8, 1.8, 1.4)):
        self.config = LaneTaskConfig() if config is None else config
        if not isinstance(self.config, LaneTaskConfig) or not isinstance(route, LaneRoute):
            raise ValueError('Expected LaneTaskConfig and LaneRoute')
        if route.length_m < self.config.stop_s_m or route.width_m != 3.6:
            raise ValueError('Task requires a 3.6 m lane extending through the 60 m stop target')
        if not isinstance(dimensions_m, (tuple, list)) or len(dimensions_m) != 3:
            raise ValueError('Expected three chassis collision dimensions')
        self.dimensions_m = tuple(_number(v, 'dimensions_m') for v in dimensions_m)
        if self.dimensions_m != (4.8, 1.8, 1.4):
            raise ValueError('Lane task v1 requires the validated 4.8 x 1.8 x 1.4 m chassis')
        self.route = route
        self.max_curvature = max(abs(segment.curvature_rad_m) for segment in route.segments)
        if self.max_curvature > 1/60 + 1e-12:
            raise ValueError('Lane task v1 supports straight and radius >= 60 m fixtures')
        signed_turns = [segment.length_m*segment.curvature_rad_m for segment in route.segments]
        if (sum(abs(turn) for turn in signed_turns) > 1.0 + 1e-12
                or any(turn > 0 for turn in signed_turns) and any(turn < 0 for turn in signed_turns)):
            raise ValueError('Task containment bound requires a simple straight or single-direction gentle turn')
        self._initialized = False
        self._terminal = False
        self._hold_ticks = 0

    def _relative(self, state, tick):
        cfg, route = self.config, self.route
        projection = route.project(*state['position_m'][:2])
        yaw = projection.point.yaw_rad
        vx, vy = state['velocity_m_s'][:2]
        remaining = cfg.stop_s_m - projection.s_m
        target = min(cfg.target_speed_m_s, math.sqrt(2*cfg.planning_deceleration_m_s2
                     * max(0.0, remaining - cfg.stop_buffer_m)))
        if tick < round(cfg.settle_s * cfg.physics_hz):
            target = 0.0
        return RouteState(projection.s_m, remaining, projection.lateral_error_m,
                          wrap_yaw(state['yaw_rad'] - yaw), vx*math.cos(yaw)+vy*math.sin(yaw),
                          -vx*math.sin(yaw)+vy*math.cos(yaw), state['speed_m_s'],
                          footprint_lateral_bound(state, projection, self.dimensions_m, self.max_curvature), target)

    def _observation(self, state, applied, tick, relative):
        cfg = self.config
        values = (
            relative.progress_m, relative.remaining_m, relative.lateral_error_m,
            math.sin(relative.heading_error_rad), math.cos(relative.heading_error_rad),
            relative.forward_speed_m_s, relative.lateral_speed_m_s, relative.planar_speed_m_s,
            state['velocity_m_s'][2], state['position_m'][2], state['upright_z'],
            *(self.route.evaluate(relative.progress_m + offset).curvature_rad_m for offset in (0, 3, 6)),
            cfg.target_speed_m_s, relative.target_speed_m_s, *applied,
            min(1.0, tick / (cfg.episode_length_s*cfg.physics_hz)),
            float(tick < round(cfg.settle_s*cfg.physics_hz)),
            min(1.0, self._hold_ticks / (cfg.success_hold_s*cfg.physics_hz)),
        )
        if len(values) != len(OBSERVATION_NAMES) or not all(math.isfinite(v) for v in values):
            raise ValueError('Observation contract must contain 22 finite values')
        return tuple(values)

    def physics_guard(self, state, *, tick, contact_count=0, command_fallback=False, unsupported_ticks=0):
        """Check one measured tick, with adapter-owned consecutive support count.

        Count only completed post-settling physics ticks with fewer than four
        supported wheels. A supported tick/reset clears that counter. Aggregate
        support fraction remains a separate validation metric, not a reward.
        """
        _integer(tick, 'tick')
        _integer(contact_count, 'contact_count')
        _integer(unsupported_ticks, 'unsupported_ticks')
        if type(command_fallback) is not bool:
            raise ValueError('command_fallback must be bool')
        state = _state(state)
        relative, cfg = self._relative(state, tick), self.config
        if contact_count:
            return 'rigid_contact'
        if command_fallback:
            return 'command_fallback'
        if tick > round(cfg.settle_s*cfg.physics_hz) and unsupported_ticks > round(.1*cfg.physics_hz):
            return 'wheel_support_loss'
        if state['upright_z'] < cfg.upright_min_z:
            return 'excessive_tilt'
        if not cfg.min_height_m <= state['position_m'][2] <= cfg.max_height_m:
            return 'unsafe_height'
        if relative.footprint_lateral_bound_m > self.route.width_m/2:
            return 'lane_departure'
        if abs(relative.heading_error_rad) > cfg.heading_error_max_rad:
            return 'heading_outside_envelope'
        if relative.planar_speed_m_s > cfg.speed_max_m_s:
            return 'overspeed'
        if relative.forward_speed_m_s < -cfg.reverse_speed_max_m_s:
            return 'reverse_motion'
        if relative.progress_m < -1.0:
            return 'before_route_start'
        if relative.progress_m > cfg.stop_s_m + cfg.endpoint_tolerance_m:
            return 'endpoint_overshoot'
        return None

    def initialize(self, state, applied, *, tick=0):
        if tick != 0 or isinstance(tick, bool):
            raise ValueError('Initialize only at physical reset tick zero')
        state, applied = _state(state), _control(applied)
        relative = self._relative(state, 0)
        cfg = self.config
        if (abs(relative.progress_m) > 0.05 or abs(relative.lateral_error_m) > cfg.lateral_reset_bound_m + 1e-6
                or abs(relative.heading_error_rad) > cfg.yaw_reset_bound_rad + 1e-6
                or relative.planar_speed_m_s > cfg.stop_speed_m_s):
            raise ValueError('Measured reset state lies outside the declared start envelope')
        reason = self.physics_guard(state, tick=0)
        if reason:
            raise ValueError(f'Unsafe measured reset state: {reason}')
        self._previous_state, self._previous_control, self._previous_relative = state, applied, relative
        self._tick = 0
        self._start_s_m = relative.progress_m
        self._hold_ticks = 0
        self._initialized, self._terminal = True, False
        return self._observation(state, applied, 0, relative)

    def advance(self, state, applied, *, tick, contact_count=0, command_fallback=False, failure_reason=None):
        if not self._initialized or self._terminal:
            raise RuntimeError('A physical reset and initialize are required before another transition')
        _integer(tick, 'tick')
        if tick != self._tick + self.config.control_ticks:
            raise ValueError('Each transition must advance exactly one 60 Hz control interval')
        if failure_reason is not None and (not isinstance(failure_reason, str) or not failure_reason):
            raise ValueError('failure_reason must be None or a nonempty latched guard reason')
        state, applied = _state(state), _control(applied)
        relative, cfg = self._relative(state, tick), self.config
        reason = self.physics_guard(state, tick=tick, contact_count=contact_count,
                                    command_fallback=command_fallback)
        reason = failure_reason or reason
        dt = cfg.control_ticks / cfg.physics_hz
        delta_s = relative.progress_m - self._previous_relative.progress_m
        displacement = math.dist(state['position_m'][:2], self._previous_state['position_m'][:2])
        discontinuity = displacement > cfg.speed_max_m_s*dt + 0.005
        if discontinuity and reason is None:
            reason = 'state_discontinuity'
        drive = self._tick >= round(cfg.settle_s*cfg.physics_hz)
        at_goal = (abs(relative.remaining_m) <= cfg.endpoint_tolerance_m
                   and relative.progress_m-self._start_s_m >= cfg.stop_s_m-cfg.endpoint_tolerance_m
                   and relative.planar_speed_m_s <= cfg.stop_speed_m_s)
        previous = self._previous_relative
        previously_at_goal = (abs(previous.remaining_m) <= cfg.endpoint_tolerance_m
                              and previous.progress_m-self._start_s_m >= cfg.stop_s_m-cfg.endpoint_tolerance_m
                              and previous.planar_speed_m_s <= cfg.stop_speed_m_s)
        self._hold_ticks = (self._hold_ticks + cfg.control_ticks
                            if drive and at_goal and previously_at_goal and reason is None else 0)
        success = reason is None and self._hold_ticks >= round(cfg.success_hold_s*cfg.physics_hz)
        terminated = reason is not None or success
        truncated = not terminated and tick >= round(cfg.episode_length_s*cfg.physics_hz)
        if success:
            reason = 'success'
        elif truncated:
            reason = 'time_limit'
        elif reason is None:
            reason = 'driving' if drive else 'settling'
        # Progress is a displacement once, not velocity multiplied by dt twice.
        # No positive survival reward. Running costs use simulated seconds.
        components = dict(progress=0.0, lateral=0.0, heading=0.0, speed=0.0,
                          steering_rate=0.0, time=0.0, terminal=0.0)
        if drive:
            components.update(
                progress=0.0 if discontinuity else delta_s/3.0,
                lateral=-dt*(relative.lateral_error_m/0.5)**2,
                heading=-0.25*dt*(relative.heading_error_rad/0.25)**2,
                speed=-0.25*dt*((relative.forward_speed_m_s-relative.target_speed_m_s)/3.0)**2,
                steering_rate=-0.01*dt*((applied[0]-self._previous_control[0])/(dt*0.5))**2,
                time=-0.05*dt,
            )
        # Early safety failure is still an episode failure, even during settling.
        # Running settling reward remains exactly zero; terminal cost is distinct.
        components['terminal'] = 10.0 if success else (-10.0 if terminated else 0.0)
        observation = self._observation(state, applied, tick, relative)
        self._previous_state, self._previous_control, self._previous_relative = state, applied, relative
        self._tick, self._terminal = tick, terminated or truncated
        return LaneTransition(tick, tick/cfg.physics_hz, observation, relative, components,
                              sum(components.values()), terminated, truncated, reason, success,
                              observation if self._terminal else None)
