"""Comfort filter around SUMO ACC requests; SUMO retains final safety authority."""
from dataclasses import asdict, dataclass
import math


@dataclass(frozen=True)
class ComfortConfig:
    max_acceleration: float = 1.5
    comfortable_deceleration: float = 2.5
    max_jerk: float = 2.0
    target_speed: float = 25.0
    cruise_approach_gain: float = .4
    realized_jerk_tolerance: float = .05

    def __post_init__(self):
        if not all(math.isfinite(v) and v>0 for v in asdict(self).values()):
            raise ValueError('Comfort settings must be finite and positive')


def limit_acceleration(requested, realized_previous, speed, dt, config=ComfortConfig()):
    """Limit the next request relative to measured acceleration, not hidden state.

    Stopping at zero speed may make measured jerk differ from this request. SUMO
    safety may also brake harder. Neither is concealed as a smooth realized step.
    """
    if not all(math.isfinite(v) for v in (requested,realized_previous,speed,dt)) or dt<=0 or speed<0:
        raise ValueError('Invalid acceleration-filter input')
    desired=max(-config.comfortable_deceleration,min(config.max_acceleration,requested))
    delta=config.max_jerk*dt
    filtered=max(realized_previous-delta,min(realized_previous+delta,desired))
    return max(-speed/dt,filtered)


def command(conn, vid, dt, config=ComfortConfig(), *, follow_cap=False):
    speed=conn.vehicle.getSpeed(vid)
    previous=conn.vehicle.getAcceleration(vid)
    leader=conn.vehicle.getLeader(vid,150.)
    if leader and leader[0]:
        other,gap=leader
        desired_speed=conn.vehicle.getFollowSpeed(vid,speed,gap,conn.vehicle.getSpeed(other),
            conn.vehicle.getDecel(other),other)
    else:
        desired_speed=conn.vehicle.getFollowSpeed(vid,speed,1e6,config.target_speed,4.5,'')
    desired_speed=min(desired_speed,config.target_speed,conn.vehicle.getAllowedSpeed(vid))
    raw=(desired_speed-speed)/dt
    # Approach the cap with declining acceleration instead of hitting it with
    # positive acceleration and letting SUMO abruptly clip the next step.
    cruise_bound=config.cruise_approach_gain*(min(config.target_speed,conn.vehicle.getAllowedSpeed(vid))-speed)
    raw=min(raw,cruise_bound)
    filtered=limit_acceleration(raw,previous,speed,dt,config)
    applied=min(filtered,raw) if follow_cap else filtered
    conn.vehicle.setAcceleration(vid,applied,dt)
    return dict(requested_acceleration=raw,filtered_acceleration=filtered,
        applied_acceleration=applied,follow_cap_active=follow_cap,
        predicted_speed=speed+applied*dt,previous_realized_acceleration=previous,
        action_source='diagnostic_ACC_follow_cap' if follow_cap else 'SUMO_ACC_followSpeed_with_comfort_filter',
        leader_id=leader[0] if leader and leader[0] else None)
