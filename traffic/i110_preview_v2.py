"""Continuous, path-relevant preview; native SUMO retains final safety authority.

The leader is TraCI's current route-local leader, not a guessed adjacent merge.
The early gap-closing law is informed by SUMO 1.27.1 ACC, with extra headway and
space reserve. It is a request, not a claim to reproduce every native mode.
"""
from dataclasses import asdict, dataclass
import math


@dataclass(frozen=True)
class PreviewV2Config:
    range_m: float = 240.
    headway_s: float = 2.
    reserve_m: float = 10.
    speed_gain: float = .8
    gap_gain: float = .04
    # Matches the native ACC low-speed spacing offset, bounded and defined at rest.
    low_speed_offset_max_m: float = 2.

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in asdict(self).values()):
            raise ValueError('Preview v2 settings must be finite and positive')


def following_request(speed, leader_speed, net_gap, config=PreviewV2Config()):
    """Net gap uses TraCI getLeader semantics (ego minGap already excluded)."""
    if (not all(math.isfinite(v) for v in (speed, leader_speed, net_gap))
            or speed < 0 or leader_speed < 0):
        raise ValueError('Invalid current leader state')
    offset = min(config.low_speed_offset_max_m, max(0., 75./speed-5.)) if speed > 0 else config.low_speed_offset_max_m
    gap_error = net_gap-config.headway_s*speed-config.reserve_m-offset
    return config.speed_gain*(leader_speed-speed)+config.gap_gain*gap_error


class PreviewV2Controller:
    def __init__(self, config=PreviewV2Config()):
        self.config = config
        self.last_time = {}

    def observe(self, conn, vid, states, time_s):
        if time_s <= self.last_time.get(vid, -math.inf):
            raise ValueError('Preview clock must advance; construct a new controller at reset')
        self.last_time[vid] = time_s
        ego = states[vid]
        leader = conn.vehicle.getLeader(vid, self.config.range_m)
        supported = bool(leader and leader[0] in states and leader[1] <= self.config.range_m)
        # Count observed adjacent possibilities only; do not feed their speeds to control.
        excluded = sum(1 for oid, s in states.items() if oid != vid
            and (not leader or oid != leader[0])
            and math.hypot(s['x']-ego['x'], s['y']-ego['y']) <= self.config.range_m)
        ceiling = following_request(ego['speed'], states[leader[0]]['speed'], leader[1], self.config) if supported else None
        known = not leader or not leader[0] or leader[0] in states
        return dict(version=2, active=ceiling is not None, context_known=known,
            requested_ceiling_m_s2=ceiling, limiting_vehicle=leader[0] if supported else None,
            native_leader_net_gap_m=leader[1] if supported else None,
            limiting_leader_speed_m_s=states[leader[0]]['speed'] if supported else None,
            excluded_nonleader_count=excluded, range_m=self.config.range_m,
            recovery='recompute every step; no fixed brake latch or dwell',
            information='current route-local TraCI leader and exact state; no future intent/event/role labels')
