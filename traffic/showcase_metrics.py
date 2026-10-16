"""Bounded, ground-truth descriptive statistics for the circular showcase.

These metrics never affect motion. A lifetime is a generation-aware logical ID,
not a reusable native slot. Only consecutive ``drive`` observations contribute;
settling, the forced final brake, missing intervals and dormant slots do not.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
import math


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


@dataclass(frozen=True)
class MetricsConfig:
    near_clearance_m: float = 1.0
    near_ttc_s: float = 1.5
    hard_braking_m_s2: float = 3.0
    stopped_speed_m_s: float = 0.1
    leader_horizon_m: float = 150.0
    max_interval_s: float = 0.25
    fairness_min_exposure_s: float = 1.0
    max_active: int = 81
    max_lifetimes: int = 4096

    def __post_init__(self):
        for key, value in asdict(self).items():
            if not _finite(value) or value <= 0:
                raise ValueError(f'{key} must be finite and positive')
        for key in ('max_active', 'max_lifetimes'):
            if type(getattr(self, key)) is not int:
                raise ValueError(f'{key} must be an integer')
        if self.max_lifetimes < self.max_active:
            raise ValueError('Lifetime capacity must cover active capacity')


@dataclass(frozen=True)
class VehicleSample:
    vehicle_id: str
    x_m: float
    y_m: float
    yaw_rad: float
    speed_m_s: float
    length_m: float
    width_m: float
    free_flow_speed_m_s: float
    role: str = 'background'

    def __post_init__(self):
        if not isinstance(self.vehicle_id, str) or not self.vehicle_id.strip():
            raise ValueError('A nonempty generation-aware identity is required')
        if self.role not in ('ego', 'background'):
            raise ValueError('Role must be ego or background')
        values = (self.x_m, self.y_m, self.yaw_rad, self.speed_m_s,
                  self.length_m, self.width_m, self.free_flow_speed_m_s)
        if not all(_finite(v) and abs(v) <= 1e9 for v in values):
            raise ValueError('Vehicle values must be finite numbers within the metrics numeric envelope')
        if self.speed_m_s < 0 or min(self.length_m, self.width_m, self.free_flow_speed_m_s) <= 0:
            raise ValueError('Speed must be nonnegative and dimensions/reference positive')
        if self.free_flow_speed_m_s < 1e-6:
            raise ValueError('Free-flow reference must be at least 1e-6 m/s')
        if math.hypot(self.x_m, self.y_m) <= math.hypot(self.length_m, self.width_m) / 2:
            raise ValueError('A footprint must not include the undefined circle origin')


def _footprint(sample):
    """Exact radial bounds and angular corner extents about the circle origin."""
    x, y, a = sample.x_m, sample.y_m, sample.yaw_rad
    c, s = math.cos(a), math.sin(a)
    hl, hw = sample.length_m / 2, sample.width_m / 2
    radius, angle = math.hypot(x, y), math.atan2(y, x)
    corners = [(x + u*c - v*s, y + u*s + v*c)
               for u in (-hl, hl) for v in (-hw, hw)]
    angles = [(math.atan2(cy, cx)-angle+math.pi) % math.tau-math.pi for cx, cy in corners]
    radial_min = math.hypot(max(abs(x*c+y*s)-hl, 0), max(abs(-x*s+y*c)-hw, 0))
    return radius, angle, radial_min, max(math.hypot(*p) for p in corners), min(angles), max(angles)


def measured_leader(ego, others, config=None):
    """Ego-only geometric gap on a known circle; TTC is a closing-rate proxy.

    Footprint radial intervals must overlap. Arc distances use the ego's actual
    radius, including outer lanes, and angular footprint extrema. This handles
    the seam and partial lane changes without using a requested following gap.
    """
    config = config or MetricsConfig()
    er, ea, elo, ehi, _, efront = _footprint(ego)
    best = None
    for other in others:
        if other.vehicle_id == ego.vehicle_id:
            continue
        radius, angle = math.hypot(other.x_m, other.y_m), math.atan2(other.y_m, other.x_m)
        delta = (angle-ea) % math.tau
        center_gap = er*delta
        if center_gap > config.leader_horizon_m:
            continue
        # Bounding circles are a conservative cheap rejection; exact footprint
        # work is needed only for nearby forward candidates, not all 80 slots.
        if abs(radius-er) > (math.hypot(ego.length_m, ego.width_m)+math.hypot(other.length_m, other.width_m))/2:
            continue
        radius, angle, low, high, rear, front = _footprint(other)
        if min(ehi, high) <= max(elo, low):
            continue
        gap = max(0.0, er*(delta + rear - efront))
        if best is not None and gap >= best['bumper_gap_m']:
            continue
        ego_tangent_speed = ego.speed_m_s*math.cos(ego.yaw_rad-ea-math.pi/2)
        other_tangent_speed = other.speed_m_s*math.cos(other.yaw_rad-angle-math.pi/2)
        closing = ego_tangent_speed-other_tangent_speed*er/radius
        moving = ego.speed_m_s > config.stopped_speed_m_s
        best = dict(leader_id=other.vehicle_id, bumper_gap_m=gap,
                    net_headway_s=gap/ego.speed_m_s if moving else None,
                    front_to_front_headway_s=max(0.0, er*(delta+front-efront))/ego.speed_m_s if moving else None,
                    closing_speed_m_s=closing, ttc_s=gap/closing if closing > 1e-9 else None)
    return best or dict(leader_id=None, bumper_gap_m=None, net_headway_s=None,
                        front_to_front_headway_s=None, closing_speed_m_s=None, ttc_s=None)


@dataclass
class _Moments:
    weight: float = 0.0
    total: float = 0.0
    square: float = 0.0
    absolute: float = 0.0
    minimum: float | None = None
    maximum: float | None = None

    def add(self, value, weight):
        self.weight += weight
        self.total += value*weight
        self.square += value*value*weight
        self.absolute += abs(value)*weight
        self.minimum = value if self.minimum is None else min(self.minimum, value)
        self.maximum = value if self.maximum is None else max(self.maximum, value)

    def report(self):
        return dict(observed_s=self.weight, mean=self.total/self.weight if self.weight else None,
                    mean_absolute=self.absolute/self.weight if self.weight else None,
                    rms=math.sqrt(self.square/self.weight) if self.weight else None,
                    minimum=self.minimum, maximum=self.maximum)


@dataclass
class _Lifetime:
    role: str
    first_s: float
    last_s: float
    previous: VehicleSample
    active: bool = True
    exposure_s: float = 0.0
    distance_m: float = 0.0
    time_loss_s: float = 0.0
    reference_distance_m: float = 0.0
    acceleration: float | None = None
    acceleration_dt: float | None = None
    hard_braking: bool = False
    hard_braking_events: int = 0
    hard_braking_s: float = 0.0
    stopped_s: float = 0.0
    speed: _Moments = field(default_factory=_Moments)
    accel: _Moments = field(default_factory=_Moments)
    jerk: _Moments = field(default_factory=_Moments)


def _fairness(lifetimes, minimum_s):
    selected = [v for v in lifetimes if v.exposure_s >= minimum_s]
    rates = [v.time_loss_s/v.exposure_s for v in selected]
    achieved = [1-r for r in rates]
    squares = sum(v*v for v in achieved)
    return dict(eligible_lifetimes=len(selected), excluded_short_lifetimes=len(lifetimes)-len(selected),
                minimum_exposure_s=minimum_s,
                mean_time_loss_fraction=sum(rates)/len(rates) if rates else None,
                time_loss_fraction_range=max(rates)-min(rates) if rates else None,
                achieved_fraction_jain_index=sum(achieved)**2/(len(achieved)*squares) if squares else None)


class ShowcaseMetrics:
    """O(active vehicles) per tick; explicit bounded lifetime ledger, no samples.

    Capacity exhaustion raises instead of silently dropping identities. The
    default 4096-lifetime bound exceeds the live fleet's 2048 event limit.
    """
    def __init__(self, config=None):
        self.config = config or MetricsConfig()
        self._lifetimes = {}
        self._time = None
        self._phase = None
        self._ego_id = None
        self._near = False
        self._headway = _Moments()
        self._gap = _Moments()
        self._ttc = _Moments()
        self._latest = {}
        self.near_events = 0
        self.near_exposure_s = 0.0
        self.drive_observed_s = 0.0
        self.excluded_gap_s = 0.0
        self.excluded_boundary_s = 0.0
        self.phase_samples = {'settle': 0, 'drive': 0, 'brake': 0}

    def update(self, sim_time_s, phase, vehicles, *, ego_id='ego', nearest_clearance_m=None):
        vehicles = tuple(vehicles)
        if not _finite(sim_time_s) or not 0 <= sim_time_s <= 1e9 or (self._time is not None and sim_time_s-self._time < 1e-9):
            raise ValueError('Simulation time must be finite, nonnegative and strictly increasing')
        if phase not in self.phase_samples:
            raise ValueError('Unknown showcase phase')
        if not vehicles or not all(isinstance(v, VehicleSample) for v in vehicles):
            raise ValueError('Active VehicleSamples including ego are required')
        incoming = {v.vehicle_id: v for v in vehicles}
        if len(incoming) != len(vehicles) or len(incoming) > self.config.max_active:
            raise ValueError('Duplicate IDs or active capacity exceeded')
        if ego_id not in incoming or incoming[ego_id].role != 'ego' or sum(v.role == 'ego' for v in vehicles) != 1:
            raise ValueError('Exactly one ego sample with the requested ID is required')
        if self._ego_id is not None and ego_id != self._ego_id:
            raise ValueError('An accumulator cannot change ego identity')
        if nearest_clearance_m is not None and (not _finite(nearest_clearance_m) or nearest_clearance_m < 0):
            raise ValueError('Clearance must be finite and nonnegative, or None')
        for key, sample in incoming.items():
            old = self._lifetimes.get(key)
            if old and (not old.active or old.role != sample.role):
                raise ValueError('Retired identities cannot reappear or change role; use a new generation')
        if len(self._lifetimes)+sum(key not in self._lifetimes for key in incoming) > self.config.max_lifetimes:
            raise ValueError('Metrics lifetime capacity exceeded')
        leader = measured_leader(incoming[ego_id], vehicles, self.config)
        dt = 0.0 if self._time is None else sim_time_s-self._time
        consecutive = bool(dt and dt <= self.config.max_interval_s+1e-9 and phase == self._phase == 'drive')
        if dt > self.config.max_interval_s+1e-9:
            self.excluded_gap_s += dt
        elif dt and phase != self._phase:
            self.excluded_boundary_s += dt
        # Iterating only active IDs keeps update cost independent of run length.
        prior_active = getattr(self, '_active_ids', set())
        for key in prior_active-incoming.keys():
            self._lifetimes[key].active = False
        ego_acceleration = ego_jerk = None
        for key, sample in incoming.items():
            previous = self._lifetimes.get(key)
            if previous is None:
                self._lifetimes[key] = _Lifetime(sample.role, sim_time_s, sim_time_s, sample)
                continue
            old = previous.previous
            acceleration = jerk = None
            if consecutive:
                previous.exposure_s += dt
                previous.distance_m += (old.speed_m_s+sample.speed_m_s)*0.5*dt
                previous.reference_distance_m += (old.free_flow_speed_m_s+sample.free_flow_speed_m_s)*0.5*dt
                loss0 = max(0.0, 1-old.speed_m_s/old.free_flow_speed_m_s)
                loss1 = max(0.0, 1-sample.speed_m_s/sample.free_flow_speed_m_s)
                previous.time_loss_s += (loss0+loss1)*0.5*dt
                previous.speed.add(old.speed_m_s, dt/2)
                previous.speed.add(sample.speed_m_s, dt/2)
                acceleration = (sample.speed_m_s-old.speed_m_s)/dt
                previous.accel.add(acceleration, dt)
                if previous.acceleration is not None:
                    midpoint_dt = (dt+previous.acceleration_dt)/2
                    jerk = (acceleration-previous.acceleration)/midpoint_dt
                    previous.jerk.add(jerk, midpoint_dt)
                braking = acceleration <= -self.config.hard_braking_m_s2
                previous.hard_braking_events += int(braking and not previous.hard_braking)
                previous.hard_braking_s += dt*braking
                previous.stopped_s += dt*(sample.speed_m_s <= self.config.stopped_speed_m_s)
                previous.hard_braking = braking
            else:
                previous.hard_braking = False
            previous.acceleration = acceleration
            previous.acceleration_dt = dt if acceleration is not None else None
            previous.previous = sample
            previous.last_s = sim_time_s
            if key == ego_id:
                ego_acceleration, ego_jerk = acceleration, jerk
        near = phase == 'drive' and ((nearest_clearance_m is not None and nearest_clearance_m < self.config.near_clearance_m)
                                      or (leader['ttc_s'] is not None and leader['ttc_s'] < self.config.near_ttc_s))
        if not consecutive:
            self._near = False
        self.near_events += int(near and not self._near)
        if consecutive:
            self.drive_observed_s += dt
            self.near_exposure_s += dt*near
            for key, accumulator in (('net_headway_s', self._headway), ('bumper_gap_m', self._gap), ('ttc_s', self._ttc)):
                if leader[key] is not None:
                    accumulator.add(leader[key], dt)
        self._near = near
        self._time, self._phase, self._ego_id = sim_time_s, phase, ego_id
        self._active_ids = set(incoming)
        self.phase_samples[phase] += 1
        ego_lifetime = self._lifetimes[ego_id]
        self._latest = dict(**leader, measured_speed_m_s=incoming[ego_id].speed_m_s,
                            longitudinal_acceleration_m_s2=ego_acceleration, longitudinal_jerk_m_s3=ego_jerk,
                            near_miss_proxy=bool(near), near_miss_events=self.near_events,
                            near_miss_exposure_s=self.near_exposure_s,
                            ego_drive_exposure_s=ego_lifetime.exposure_s,
                            ego_time_loss_s=ego_lifetime.time_loss_s,
                            active_vehicles=len(vehicles), statistics_phase=phase)
        return dict(self._latest)

    def summary(self):
        records = []
        for key, value in sorted(self._lifetimes.items()):
            records.append(dict(vehicle_id=key, role=value.role, active_at_last_sample=value.active,
                observation_end='run_censored' if value.active else 'lifecycle_exit',
                completed_trip=False, first_observed_s=value.first_s, last_observed_s=value.last_s,
                drive_exposure_s=value.exposure_s, speed_integral_distance_m=value.distance_m,
                reference_distance_m=value.reference_distance_m, free_flow_time_loss_s=value.time_loss_s,
                time_loss_fraction=value.time_loss_s/value.exposure_s if value.exposure_s else None,
                stopped_exposure_s=value.stopped_s, hard_braking_events=value.hard_braking_events,
                hard_braking_exposure_s=value.hard_braking_s, speed_m_s=value.speed.report(),
                longitudinal_acceleration_m_s2=value.accel.report(), longitudinal_jerk_m_s3=value.jerk.report()))
        values = list(self._lifetimes.values())
        return dict(schema_version=1, source='privileged_simulator_state', scope='drive_only_closed_loop_showcase',
            thresholds=asdict(self.config), observed_drive_s=self.drive_observed_s,
            last_observed_sim_time_s=self._time, last_observed_phase=self._phase,
            excluded_gap_s=self.excluded_gap_s, excluded_phase_boundary_s=self.excluded_boundary_s,
            phase_sample_counts=dict(self.phase_samples), completed_trips=None,
            near_miss_proxy=dict(events=self.near_events, exposure_s=self.near_exposure_s,
                                monitored_drive_s=self.drive_observed_s),
            ego_net_headway_s=self._headway.report(), ego_bumper_gap_m=self._gap.report(),
            ego_closing_ttc_s=self._ttc.report(),
            fairness=dict(all_lifetimes=_fairness(values, self.config.fairness_min_exposure_s),
                          by_role={role: _fairness([v for v in values if v.role == role], self.config.fairness_min_exposure_s)
                                   for role in ('ego', 'background')}),
            vehicles=records, latest=dict(self._latest))
