"""Opt-in speed-domain schedule and raw-action evidence for population v04."""
from __future__ import annotations

import math
import numpy as np

REVISION = 'population-v04-near-limit'
SPEED_CAP = 26.8224
SPEEDS = [12.0, 18.0, 22.352, 26.8224]


def enabled(cfg):
    return cfg.get('revision') == REVISION


def validate_revision(cfg):
    if (cfg['environment'].get('control_target_version') != 'v3'
            or cfg['optimizer'].get('near_limit_coef') != .1
            or cfg['optimizer'].get('near_limit_band_m_s') != 1.
            or cfg.get('objective_version') != 'raw_near_limit_si_mse_v1'
            or cfg.get('output_directory') != 'outputs/highway-population/v04'
            or cfg.get('speed_curriculum', {}).get('ceilings_m_s') != SPEEDS):
        raise ValueError('V04 requires its explicit raw-acceleration objective and speed curriculum')
    rows = cfg['validation_cases']
    if len(rows) != 24 or any('speed_limit_m_s' in row for row in rows[:16]):
        raise ValueError('Preserve sixteen original validation rows and add eight speed cases')
    coverage = {(row.get('speed_limit_m_s'), row['case']['task'], row['controlled_count']) for row in rows[16:]}
    expected = {(speed, task, count) for speed in (12., 18.)
                for task in ('cruise', 'passing') for count in (1, 5)}
    if coverage != expected:
        raise ValueError('Low-speed validation must cover both limits, tasks and populations')
    for row in rows:
        speed = row.get('speed_limit_m_s')
        if speed is not None and (isinstance(speed, bool) or not math.isfinite(speed) or speed not in SPEEDS):
            raise ValueError('Invalid validation speed ceiling')


def speed_for_episode(cfg, episode_index):
    """Every four successive episodes cover all ceilings, independent of task order."""
    if not enabled(cfg):
        return None
    if type(episode_index) is not int or episode_index < 0:
        raise ValueError('Episode index must be a nonnegative integer')
    rng = np.random.default_rng(np.random.SeedSequence([cfg['training']['seed'], episode_index // 4, 714]))
    return float(rng.permutation(cfg['speed_curriculum']['ceilings_m_s'])[episode_index % 4])


class RawNearLimitMetrics:
    """Before-guard requests only; never changes a command or acceptance gate."""
    def __init__(self, band_m_s=1.):
        self.band = band_m_s
        self.count = self.near_count = self.at_limit_count = self.excess_count = self.positive_at_limit = 0
        self.error_squared = self.request_sum = self.target_sum = self.max_excess = 0.

    def add(self, observation, action, teacher):
        speed, limit = float(observation[0])*SPEED_CAP, float(observation[4])*SPEED_CAP
        requested = -4.5 + (float(action[0])+1)*3.25
        target = -4.5 + (float(teacher[0])+1)*3.25
        if not all(math.isfinite(v) for v in (speed, limit, requested, target)):
            raise ValueError('Nonfinite raw-action diagnostic')
        self.count += 1
        if speed < limit-self.band:
            return
        self.near_count += 1
        self.error_squared += (requested-target)**2
        self.request_sum += requested
        self.target_sum += target
        ceiling = max(-4.5, min(2., (limit-speed)/.1))
        excess = max(0., requested-ceiling)
        self.excess_count += excess > .05
        self.max_excess = max(self.max_excess, excess)
        if abs(speed-limit) <= .05:
            self.at_limit_count += 1
            self.positive_at_limit += requested > .05

    def report(self):
        n = self.near_count
        return dict(all_decisions=self.count, near_limit_decisions=n, band_m_s=self.band,
            raw_teacher_rmse_m_s2=math.sqrt(self.error_squared/n) if n else None,
            mean_raw_request_m_s2=self.request_sum/n if n else None,
            mean_teacher_target_m_s2=self.target_sum/n if n else None,
            raw_speed_ceiling_excess_count=self.excess_count,
            raw_speed_ceiling_excess_fraction=self.excess_count/n if n else None,
            maximum_raw_ceiling_excess_m_s2=self.max_excess if n else None,
            at_limit_decisions=self.at_limit_count, positive_at_limit_count=self.positive_at_limit,
            positive_at_limit_fraction=self.positive_at_limit/self.at_limit_count if self.at_limit_count else None,
            diagnostic_tolerance_m_s2=.05, at_limit_band_m_s=.05,
            scope='Raw action before all guards; diagnostic tolerances do not redefine safety accounting')
