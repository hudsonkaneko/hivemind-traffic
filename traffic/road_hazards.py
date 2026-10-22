"""Pure, seeded road-local hazard schedules; no simulator or pose writer.

The native owner precreates fixed colliders and applies these kinematic targets.
Only measured native frames, never this schedule, belong in actor observations.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import math
from numbers import Real
import random


def number(value, name):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f'{name} must be finite')
    return float(value)


@dataclass(frozen=True)
class MeasuredObject:
    object_id: str
    x_m: float
    y_m: float
    yaw_rad: float
    vx_m_s: float
    vy_m_s: float
    length_m: float
    width_m: float
    timestamp_s: float

    def __post_init__(self):
        if not isinstance(self.object_id, str) or not self.object_id.strip():
            raise ValueError('object_id must be nonempty')
        for key, value in asdict(self).items():
            if key != 'object_id':
                number(value, key)
        if self.length_m <= 0 or self.width_m <= 0 or self.timestamp_s < 0:
            raise ValueError('Positive dimensions and nonnegative timestamp required')


@dataclass(frozen=True)
class HazardSpec:
    object_id: str
    kind: str
    station_m: float
    start_lateral_m: float
    end_lateral_m: float
    onset_s: float
    speed_m_s: float
    length_m: float = 1.0
    width_m: float = 1.0

    def __post_init__(self):
        if self.kind not in ('static', 'crossing'):
            raise ValueError('Unsupported hazard kind')
        if not isinstance(self.object_id, str) or not self.object_id:
            raise ValueError('Nonempty hazard ID required')
        for k, v in asdict(self).items():
            if k not in ('object_id', 'kind'):
                number(v, k)
        if self.station_m <= 0 or self.onset_s < 0 or self.speed_m_s < 0:
            raise ValueError('Invalid station, onset or speed')
        if (self.length_m, self.width_m) != (1.0, 1.0):
            raise ValueError('Hazard v1 uses a fixed 1 x 1 m footprint')
        if self.kind == 'static':
            if self.speed_m_s or self.start_lateral_m or self.end_lateral_m or self.onset_s:
                raise ValueError('Static object is already in the lane at reset')
        elif (self.speed_m_s <= 0 or self.start_lateral_m * self.end_lateral_m >= 0
              or min(abs(self.start_lateral_m), abs(self.end_lateral_m)) < 3.3):
            raise ValueError('Crossing must start/end offroad on opposite sides')

    def local_sample(self, elapsed_s):
        t = number(elapsed_s, 'elapsed_s')
        if t < 0:
            raise ValueError('Negative episode time')
        distance = abs(self.end_lateral_m - self.start_lateral_m)
        moved = min(distance, max(0.0, t - self.onset_s) * self.speed_m_s)
        sign = 1.0 if self.end_lateral_m > self.start_lateral_m else -1.0
        lateral = self.start_lateral_m + sign * moved
        velocity = sign * self.speed_m_s if self.onset_s <= t and moved < distance else 0.0
        return lateral, velocity

    def sample(self, route, elapsed_s):
        lateral, speed = self.local_sample(elapsed_s)
        point = route.evaluate(self.station_m)
        c, s = math.cos(point.yaw_rad), math.sin(point.yaw_rad)
        return MeasuredObject(self.object_id, point.position_xy[0] - s*lateral,
                              point.position_xy[1] + c*lateral, point.yaw_rad,
                              -s*speed, c*speed, self.length_m, self.width_m,
                              float(elapsed_s))


@dataclass(frozen=True)
class HazardSchedule:
    stage: str
    seed: int
    episode_index: int
    hazards: tuple[HazardSpec, ...]
    rejected_draws: tuple[dict, ...] = ()

    def __post_init__(self):
        if self.stage not in ('clear', 'static', 'crossing'):
            raise ValueError('Unknown hazard stage')
        for value in (self.seed, self.episode_index):
            if type(value) is not int or value < 0:
                raise ValueError('Seed/index must be nonnegative integers')
        if self.seed >= 2**32 or len(self.hazards) != (0 if self.stage == 'clear' else 1):
            raise ValueError('Hazard v1 has zero or one active object')
        if self.hazards and self.hazards[0].kind != self.stage:
            raise ValueError('Stage and object kind differ')

    def sample(self, route, elapsed_s):
        t = number(elapsed_s, 'elapsed_s')
        if t < 0:
            raise ValueError('Negative episode time')
        return tuple(h.sample(route, t) for h in self.hazards)

    def as_dict(self):
        return asdict(self)


def feasible_envelope(schedule, *, cruise_speed_m_s=3.0, settle_s=2.0,
                      planning_deceleration_m_s2=1.5, reaction_s=.2,
                      stopping_margin_m=1.0, ego_half_length_m=2.4):
    """Predeclared nominal braking envelope, NOT a native collision guarantee.

    Static objects are visible at reset. A crossing object becomes predictable
    from its measured velocity at motion onset; the reference car is assumed to
    travel at cruise immediately after settling (a conservative distance bound).
    This certificate assumes prompt observation and validated braking authority.
    """
    for name, value in locals().copy().items():
        if name != 'schedule':
            number(value, name)
    if (cruise_speed_m_s < 0 or settle_s < 0 or planning_deceleration_m_s2 <= 0
            or reaction_s < 0 or stopping_margin_m < 0 or ego_half_length_m <= 0):
        raise ValueError('Invalid feasibility parameters')
    required = (cruise_speed_m_s**2/(2*planning_deceleration_m_s2)
                + cruise_speed_m_s*reaction_s + stopping_margin_m)
    details = []
    for h in schedule.hazards:
        available_at_s = h.onset_s if h.kind == 'crossing' else 0.0
        ego_s = max(0.0, available_at_s-settle_s)*cruise_speed_m_s
        gap = h.station_m - h.length_m/2 - ego_half_length_m - ego_s
        details.append(dict(object_id=h.object_id,first_predictable_s=available_at_s,
                            reference_ego_station_m=ego_s,available_gap_m=gap,
                            required_gap_m=required,reserve_m=gap-required,feasible=gap>=required))
    return dict(feasible=all(x['feasible'] for x in details),objects=details,
                definition='nominal_current_velocity_braking_envelope_v1',
                assumption='3 m/s reference after settling; current native state; no sensor delay beyond reaction margin; not a physical proof')


def make_hazard_schedule(stage, seed, episode_index=0, *, route=None, feasible_only=True, side=None):
    """Hash-derived private RNG; bounded rejection happens before an episode.

    Rejected draws are preserved. Held-out seeds and feasibility selection must
    be declared before evaluation; never resample because of a policy outcome.
    Clear has no observed objects; the scene owner parks its reserved pool.
    """
    HazardSchedule(stage, seed, episode_index, () if stage == 'clear' else (
        HazardSpec('hazard_000','static' if stage=='static' else 'crossing',25.,
                   0. if stage=='static' else 3.3,0. if stage=='static' else -3.3,
                   0.,0. if stage=='static' else 1.),))
    if type(feasible_only) is not bool:
        raise ValueError('feasible_only must be bool')
    if side not in (None,'left','right') or side is not None and stage!='crossing':
        raise ValueError('Forced side applies only to crossing: left or right')
    rng = random.Random(int.from_bytes(hashlib.sha256(
        f'road-hazards-v1|{stage}|{seed}|{episode_index}'.encode()).digest(),'big'))
    rejected=[]
    for _ in range(128):
        if stage=='clear':
            return HazardSchedule(stage,seed,episode_index,())
        station=rng.uniform(24.,32.)
        if stage=='static':
            spec=HazardSpec('hazard_000',stage,station,0.,0.,0.,0.)
        else:
            sampled_side=rng.choice((-1.,1.)); speed=rng.uniform(.75,1.25)
            entry_side=sampled_side if side is None else 1. if side=='left' else -1.
            center_time=rng.uniform(9.,12.)
            spec=HazardSpec('hazard_000',stage,station,entry_side*3.3,-entry_side*3.3,
                            center_time-3.3/speed,speed)
        if route is not None and (route.width_m>3.6 or route.length_m<spec.station_m+3.):
            raise ValueError('Schedule needs a <=3.6 m road through the object station')
        schedule=HazardSchedule(stage,seed,episode_index,(spec,),tuple(rejected))
        certificate=feasible_envelope(schedule)
        if not feasible_only or certificate['feasible']:
            return schedule
        rejected.append(dict(draw=asdict(spec),certificate=certificate))
    raise ValueError('No feasible draw within the declared 128-draw cap')
