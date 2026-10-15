"""Runtime-free contract for the single-car Isaac Lab compatibility task.

Observations are simulator state, not LiDAR or inferred perception. The task is
an interface/reset check, with zero reward; it is not a training objective.
"""
import math
from numbers import Real

ACTION_NAMES = ('steering_rad', 'throttle', 'brake')
ACTION_LOW = (-0.5, 0.0, 0.0)
ACTION_HIGH = (0.5, 1.0, 1.0)
OBSERVATION_NAMES = (
    'x_m', 'y_m', 'z_m', 'yaw_rad', 'vx_m_s', 'vy_m_s', 'vz_m_s',
    'upright_z', 'applied_steering_rad', 'applied_throttle', 'applied_brake',
)


def validate_config(config):
    """Reject unsupported rates and phases before starting an expensive runtime."""
    for name in ('physics_hz', 'control_hz', 'repeats', 'seed', 'command_ttl_ticks', 'max_process_seconds'):
        value = config[name]
        if not isinstance(value, int) or isinstance(value, bool) or value < (0 if name == 'seed' else 1):
            raise ValueError(f'{name} must be an integer in its supported range')
    if config['physics_hz'] != 120 or config['control_hz'] != 60:
        raise ValueError('Lab compatibility probe is fixed to 120 Hz physics and 60 Hz control')
    if not 2 <= config['repeats'] <= 10:
        raise ValueError('repeats must be in [2, 10]')
    if not 2 <= config['command_ttl_ticks'] <= 12:
        raise ValueError('command_ttl_ticks must be in [2, 12]')
    if not 60 <= config['max_process_seconds'] <= 300:
        raise ValueError('max_process_seconds must be in [60, 300]')
    for name, maximum in [('target_speed_m_s', 3), ('drive_torque_per_front_wheel_nm', 700),
                          ('brake_torque_per_wheel_nm', 1500)]:
        value = config[name]
        if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value) or not 0 < value <= maximum:
            raise ValueError(f'{name} must be finite and in (0, {maximum}]')
    if config['phases'] != [['settle', 2.0], ['straight', 4.0], ['brake', 2.0]]:
        raise ValueError('Lab compatibility phases are fixed: settle 2 s, straight 4 s, brake 2 s')
    acceptance = config['acceptance']
    for name in ('repeat_position_tolerance_m', 'repeat_speed_tolerance_m_s',
                 'reset_position_tolerance_m', 'reset_speed_tolerance_m_s',
                 'minimum_forward_progress_m', 'stopped_speed_max_m_s'):
        value = acceptance[name]
        if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'acceptance.{name} must be finite and positive')
    return config


def bounded_action(values):
    """Clip finite physical-unit controls; brake suppresses propulsion.

    Nonfinite/malformed actions raise before physics steps. The caller must
    stop the rollout, rather than keep integrating the previous command.
    """
    if len(values) != 3 or any(
        isinstance(v, bool) or not isinstance(v, Real) or not math.isfinite(v)
        for v in values
    ):
        raise ValueError('Action must contain three finite numbers: steering_rad, throttle, brake')
    steering, throttle, brake = (
        max(low, min(high, float(value)))
        for value, low, high in zip(values, ACTION_LOW, ACTION_HIGH)
    )
    return steering, 0.0 if brake > 0 else throttle, brake


def observation_values(state, control):
    values = (
        *state['position_m'], state['yaw_rad'], *state['velocity_m_s'],
        state['upright_z'], control.steering_rad, control.throttle, control.brake,
    )
    if len(values) != len(OBSERVATION_NAMES) or not all(math.isfinite(v) for v in values):
        raise ValueError('Physics state must produce eleven finite observation values')
    return values


def require_finite_telemetry(value, path='trace'):
    """Reject NaN/Inf anywhere in nested numeric telemetry, including controls."""
    if isinstance(value, dict):
        for key, child in value.items():
            require_finite_telemetry(child, f'{path}.{key}')
    elif isinstance(value, (list, tuple)):
        for index, child in enumerate(value):
            require_finite_telemetry(child, f'{path}[{index}]')
    elif isinstance(value, Real) and not isinstance(value, bool) and not math.isfinite(value):
        raise ValueError(f'Nonfinite numeric telemetry at {path}')
