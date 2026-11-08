"""Experimental CIRCLES longitudinal adapter; steering remains locally owned."""
import math


def measured_leader_input(state, leader, *, now_s):
    """Current chassis-frame bumper gap to one measured stationary lead fixture.

    This bounded probe requires the supplied obstacle to be in front and in the
    ego corridor. It is not a general leader selector or perception pipeline.
    """
    x, y = state['position_m'][:2]
    yaw = state['yaw_rad']
    c, s = math.cos(yaw), math.sin(yaw)
    forward = c * (leader.x_m-x) + s * (leader.y_m-y)
    lateral = -s * (leader.x_m-x) + c * (leader.y_m-y)
    angle = leader.yaw_rad-yaw
    half_length = abs(math.cos(angle))*leader.length_m/2 + abs(math.sin(angle))*leader.width_m/2
    half_width = abs(math.sin(angle))*leader.length_m/2 + abs(math.cos(angle))*leader.width_m/2
    gap = forward-half_length-2.4
    speed = c*state['velocity_m_s'][0]+s*state['velocity_m_s'][1]
    lead_speed = c*leader.vx_m_s+s*leader.vy_m_s
    values = (speed, lead_speed, gap, lateral, now_s, leader.timestamp_s)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Nonfinite measured leader frame')
    if not 0 <= now_s-leader.timestamp_s <= .2 or gap <= 0 or abs(lateral) > .9+half_width:
        raise ValueError('Leader frame stale, overlapping, behind, or outside corridor')
    return [max(0., speed), max(0., lead_speed), gap]


def acceleration_to_pedals(speed, acceleration):
    """CRA's documented T=.6 target, existing gains, explicit 3 m/s test cap."""
    if not all(math.isfinite(v) for v in (speed, acceleration)) or speed < 0:
        raise ValueError('Finite speed and acceleration required')
    target = min(3., max(0., speed+.6*acceleration))
    error = target-speed
    throttle = min(1., max(0., .7*error))
    brake = min(1., max(0., -.35*error))
    return throttle, brake, target
