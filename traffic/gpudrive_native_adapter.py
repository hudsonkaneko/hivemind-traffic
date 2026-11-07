"""Explicit GPUDrive steering -> SUMO lane intent; safety stays in v05/SUMO."""
import math
import numpy as np
from environments.sumo_mappo_env import physical_acceleration_to_action


def native_request(acceleration, steering, speed, lane_width, lookahead_s=1.):
    if not all(math.isfinite(x) for x in (acceleration,steering,speed,lane_width,lookahead_s)):
        raise ValueError('Finite adapter inputs required')
    if speed<0 or lane_width<=0 or lookahead_s<=0:
        raise ValueError('Invalid adapter geometry')
    beta=math.atan(.5*math.tan(steering))
    lateral=speed*math.sin(beta)*lookahead_s
    delta=int(math.copysign(1,lateral)) if abs(lateral)>=.5*lane_width else 0
    limited=float(np.clip(acceleration,-4.5,2.))
    return np.asarray([physical_acceleration_to_action(limited),delta],np.float32),dict(
        raw_acceleration_m_s2=float(acceleration),raw_steering_rad=float(steering),
        adapted_acceleration_m_s2=limited,requested_lane_delta=delta,
        estimated_lateral_displacement_m=lateral,lookahead_s=lookahead_s,
        acceleration_range_clipped=abs(limited-acceleration)>1e-4)
