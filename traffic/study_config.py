"""Strict configuration and deterministic role assignment for scaling studies."""
from dataclasses import asdict, dataclass
import math
import random


@dataclass(frozen=True)
class StudyConfig:
    vehicles: int = 20
    av_fraction: float = .5
    seed: int = 42
    seconds: float = 20.
    warmup_s: float = 2.
    dt_s: float = .1
    control_hz: float = 2.
    control_mode: str = 'sumo_native'
    schema_version: int = 1

    def __post_init__(self):
        for name in ('vehicles', 'seed', 'schema_version'):
            if type(getattr(self, name)) is not int:
                raise ValueError(f'{name} must be an integer')
        if self.schema_version != 1 or not 1 <= self.vehicles <= 500 or not 0 <= self.seed < 2**31:
            raise ValueError('Unsupported version, vehicle count or seed')
        for name in ('av_fraction', 'seconds', 'warmup_s', 'dt_s', 'control_hz'):
            value = getattr(self, name)
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        if not 0 <= self.av_fraction <= 1 or not 0 < self.seconds <= 120 or not 0 <= self.warmup_s <= 30 or not .01 <= self.dt_s <= 1 or not 0 < self.control_hz <= 1/self.dt_s:
            raise ValueError('Invalid ratio, duration, timestep or control frequency')
        for value in (self.seconds/self.dt_s, self.warmup_s/self.dt_s, 1/self.control_hz/self.dt_s):
            if not math.isclose(value, round(value), abs_tol=1e-8):
                raise ValueError('Duration and control periods must align to SUMO steps')
        if self.control_mode not in ('sumo_native', 'scripted_speed'):
            raise ValueError('Only native and workload-probe scripted control are implemented')

    @property
    def av_count(self):
        return math.floor(self.vehicles * self.av_fraction + .5)

    def roles(self):
        # Shuffle once, then select prefixes: identical demand and nested AV
        # assignments across ratios for a fixed population/seed.
        ids = [f'car_{i:04d}' for i in range(self.vehicles)]
        order = ids.copy()
        random.Random(self.seed).shuffle(order)
        avs = set(order[:self.av_count])
        return {vid: 'av' if vid in avs else 'human_model' for vid in ids}

    def resolved(self):
        return {**asdict(self), 'av_count': self.av_count,
                'realized_av_fraction': self.av_count/self.vehicles, 'roles': self.roles(),
                'observations': 'SUMO ground truth; no LiDAR',
                'purpose': 'capacity probe, NOT coordination-effectiveness evaluation'}
