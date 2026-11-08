"""Pure, conservative per-car post-merge evidence; no simulator dependency.

Entry, completed lateral motion, and eight seconds of stable driving are distinct
events. A truncated observation window is never promoted to a successful merge.
Lane reversals and neighboring braking are observations, not causal conclusions.
"""
from __future__ import annotations

from copy import deepcopy
import math


METRICS_VERSION = 'highway_post_merge_window_v05'
_EPS = 1e-8


def _finite(value, name, *, minimum=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be finite')
    if minimum is not None and value < minimum:
        raise ValueError(f'{name} must be at least {minimum}')
    return float(value)


def _risk_value(value, name):
    if value is None or value == math.inf:
        return None
    return _finite(value, name, minimum=0.)


def _empty_metrics():
    return dict(samples=0, decisions=0, intervention_decisions=0, policy_filter_changes=0,
                native_realization_changes=0, requested_acceleration_samples=0,
                filtered_acceleration_samples=0, realized_acceleration_samples=0,
                requested_acceleration_sum=0., filtered_acceleration_sum=0.,
                realized_acceleration_sum=0., collisions=0, emergency_steps=0,
                overspeed_steps=0, leader_samples=0, missing_headway_samples=0, missing_ttc_samples=0,
                minimum_headway_s=None, minimum_ttc_s=None,
                observed_neighbor_braking_samples=0, minimum_neighbor_acceleration_m_s2=None,
                elapsed_s=0., progress_m=0., reference_progress_m=0.,
                observed_reversals=0, unnecessary_reversals=0,
                justified_reversals=0, unclassified_reversals=0)


class PostMergeTracker:
    """One vehicle's complete route and first completed-merge stability window.

    Call ``observe`` once per simulation sample, including the sample at lateral
    completion. ``distance_m`` is monotonic odometry, not lane-local position.
    ``leader_present`` distinguishes an absent leader from missing measurements;
    an infinite TTC means the leader is not being approached. Use a stable lane
    identity across road edges. A reversal is A -> B -> A within the horizon.
    Only an evidence-backed ``unnecessary``/``oscillation`` classification fails
    the gate or earns a penalty; unknown reasons are explicitly unclassified.
    """

    def __init__(self, vehicle_id='', *, window_s=8., min_headway_s=1.5, min_ttc_s=3.,
                 max_intervention_fraction=.05, min_progress_ratio=.25, stable_bonus=2.,
                 reversal_horizon_s=8., max_sample_gap_s=.2):
        self.vehicle_id = str(vehicle_id)
        self.window_s = _finite(window_s, 'window_s', minimum=_EPS)
        self.min_headway_s = _finite(min_headway_s, 'min_headway_s', minimum=0.)
        self.min_ttc_s = _finite(min_ttc_s, 'min_ttc_s', minimum=0.)
        self.max_intervention_fraction = _finite(max_intervention_fraction, 'max_intervention_fraction', minimum=0.)
        self.min_progress_ratio = _finite(min_progress_ratio, 'min_progress_ratio', minimum=0.)
        if self.max_intervention_fraction > 1 or self.min_progress_ratio > 1:
            raise ValueError('Fractions must not exceed one')
        self.stable_bonus = _finite(stable_bonus, 'stable_bonus', minimum=0.)
        self.reversal_horizon_s = _finite(reversal_horizon_s, 'reversal_horizon_s', minimum=0.)
        self.max_sample_gap_s = _finite(max_sample_gap_s, 'max_sample_gap_s', minimum=_EPS)
        self.entry_time_s = None
        self.merge_complete_time_s = None
        self.merge_lane = None
        self.status = 'pending'
        self.failure_reasons = set()
        self.full_route = _empty_metrics()
        self.post_window = _empty_metrics()
        self.window_complete = False
        self._window_closed = False
        self.finished = False
        self.finish_reason = None
        self.reversals = []
        self._lane_changes = []
        self._last = None
        self._window_last = None
        self._coverage_gap = False
        self._bonus_consumed = False
        self._penalties_consumed = 0

    def mark_entry(self, time_s):
        """Retain the historical first-through-lane entry event separately."""
        time_s = _finite(time_s, 'time_s', minimum=0.)
        if self.entry_time_s is None:
            self.entry_time_s = time_s
            return True
        return False

    def mark_merge_complete(self, time_s, lane=None):
        """Begin the window only after the caller verifies lateral completion."""
        time_s = _finite(time_s, 'time_s', minimum=0.)
        if self.finished:
            raise ValueError('Cannot complete a merge after tracker finish')
        if self.entry_time_s is None or time_s < self.entry_time_s - _EPS:
            raise ValueError('Completed lateral merge requires a prior entry event')
        if self.merge_complete_time_s is None:
            self.merge_complete_time_s, self.merge_lane = time_s, deepcopy(lane)
            return True
        return False

    def _reversal(self, sample, reason, evidence):
        previous = self._last
        if previous is None or sample['lane'] is None or previous['lane'] is None or sample['lane'] == previous['lane']:
            return None
        event = None
        if self._lane_changes:
            old = self._lane_changes[-1]
            if (old['from'] == sample['lane'] and old['to'] == previous['lane']
                    and sample['time_s'] - old['time_s'] <= self.reversal_horizon_s + _EPS):
                classification = ('justified' if evidence and reason in ('route', 'safety', 'passing')
                                  else 'unnecessary' if evidence and reason in ('unnecessary', 'oscillation')
                                  else 'unclassified')
                event = dict(time_s=sample['time_s'], from_lane=deepcopy(previous['lane']),
                             to_lane=deepcopy(sample['lane']), reason=reason,
                             reason_evidence=bool(evidence), classification=classification)
                self.reversals.append(event)
        self._lane_changes.append(dict(time_s=sample['time_s'], **{'from': deepcopy(previous['lane']), 'to': deepcopy(sample['lane'])}))
        return event

    @staticmethod
    def _accumulate(metrics, sample, previous, reversal, *, max_time=None):
        metrics['samples'] += 1
        decision = int(sample['count_decision'])
        metrics['decisions'] += decision
        metrics['intervention_decisions'] += decision * int(sample['intervention'])
        metrics['native_realization_changes'] += decision * int(sample['native_intervention'])
        requested, filtered = sample['requested_acceleration'], sample['filtered_acceleration']
        metrics['policy_filter_changes'] += decision * int(requested is not None and filtered is not None and abs(requested - filtered) > 1e-4)
        for key in ('requested_acceleration', 'filtered_acceleration', 'realized_acceleration'):
            if sample[key] is not None and decision:
                metrics[key + '_samples'] += 1
                metrics[key + '_sum'] += sample[key]
        metrics['collisions'] += int(sample['collision'])
        metrics['emergency_steps'] += int(sample['emergency'])
        metrics['overspeed_steps'] += int(sample['speed_mps'] > sample['speed_limit_mps'] + 1e-4)
        if sample['leader_present']:
            metrics['leader_samples'] += 1
            metrics['missing_headway_samples'] += int(sample['leader_headway_s'] is None and not sample['headway_infinite'])
            metrics['missing_ttc_samples'] += int(sample['leader_ttc_s'] is None and not sample['ttc_infinite'])
            for source, destination in (('leader_headway_s', 'minimum_headway_s'), ('leader_ttc_s', 'minimum_ttc_s')):
                value = sample[source]
                if value is not None:
                    metrics[destination] = value if metrics[destination] is None else min(metrics[destination], value)
        braking = sample['neighbor_braking_mps2']
        if braking is not None:
            metrics['observed_neighbor_braking_samples'] += int(braking < -.05)
            old = metrics['minimum_neighbor_acceleration_m_s2']
            metrics['minimum_neighbor_acceleration_m_s2'] = braking if old is None else min(old, braking)
        if previous is not None:
            duration = sample['time_s'] - previous['time_s']
            clipped = duration if max_time is None else min(duration, max(0., max_time - previous['time_s']))
            metrics['elapsed_s'] += clipped
            metrics['progress_m'] += max(0., sample['distance_m'] - previous['distance_m']) * clipped / max(duration, _EPS)
            metrics['reference_progress_m'] += previous['speed_limit_mps'] * clipped
        if reversal:
            metrics['observed_reversals'] += 1
            key = {'justified': 'justified_reversals', 'unnecessary': 'unnecessary_reversals', 'unclassified': 'unclassified_reversals'}[reversal['classification']]
            metrics[key] += 1

    def observe(self, time_s, *, speed_mps, speed_limit_mps, distance_m,
                leader_headway_s=None, leader_ttc_s=None, leader_present=False,
                collision=False, emergency=False, intervention=False, lane=None,
                reversal_reason=None, reason_evidence=False, neighbor_braking_mps2=None,
                requested_acceleration=None, filtered_acceleration=None,
                realized_acceleration=None, native_intervention=False, count_decision=True):
        if self.finished:
            raise ValueError('Cannot observe a finished tracker')
        sample = dict(time_s=_finite(time_s, 'time_s', minimum=0.),
            speed_mps=_finite(speed_mps, 'speed_mps', minimum=0.),
            speed_limit_mps=_finite(speed_limit_mps, 'speed_limit_mps', minimum=_EPS),
            distance_m=_finite(distance_m, 'distance_m', minimum=0.),
            leader_headway_s=_risk_value(leader_headway_s, 'leader_headway_s'),
            leader_ttc_s=_risk_value(leader_ttc_s, 'leader_ttc_s'), headway_infinite=leader_headway_s == math.inf,
            ttc_infinite=leader_ttc_s == math.inf,
            leader_present=bool(leader_present), collision=bool(collision), emergency=bool(emergency),
            intervention=bool(intervention), native_intervention=bool(native_intervention),
            count_decision=bool(count_decision), lane=deepcopy(lane))
        for key, value in dict(requested_acceleration=requested_acceleration,
                               filtered_acceleration=filtered_acceleration,
                               realized_acceleration=realized_acceleration,
                               neighbor_braking_mps2=neighbor_braking_mps2).items():
            sample[key] = None if value is None else _finite(value, key)
        if self._last is not None and sample['time_s'] <= self._last['time_s']:
            raise ValueError('Observation timestamps must increase strictly')
        if self._last is not None and sample['distance_m'] < self._last['distance_m'] - _EPS:
            raise ValueError('distance_m must be monotonic route odometry')
        reversal = self._reversal(sample, reversal_reason, reason_evidence)
        self._accumulate(self.full_route, sample, self._last, reversal)
        start = self.merge_complete_time_s
        if start is not None and sample['time_s'] >= start - _EPS and not self._window_closed:
            end = start + self.window_s
            previous = self._window_last
            if previous is None:
                self._coverage_gap |= sample['time_s'] > start + _EPS
            else:
                self._coverage_gap |= sample['time_s'] - previous['time_s'] > self.max_sample_gap_s + _EPS
            self._accumulate(self.post_window, sample, previous, reversal, max_time=end)
            self._window_last = sample
            self._assess(sample['time_s'] >= end - _EPS)
        self._last = sample
        return self.summary()

    def _assess(self, expired):
        metrics = self.post_window
        for key, reason in (('collisions', 'collision'), ('emergency_steps', 'emergency'),
                            ('overspeed_steps', 'overspeed'), ('unnecessary_reversals', 'unnecessary_reversal')):
            if metrics[key]:
                self.failure_reasons.add(reason)
        for key, minimum, reason in (('minimum_headway_s', self.min_headway_s, 'headway'),
                                      ('minimum_ttc_s', self.min_ttc_s, 'ttc')):
            if metrics[key] is not None and metrics[key] < minimum - _EPS:
                self.failure_reasons.add(reason)
        if expired:
            self._window_closed = True
            self.window_complete = not self._coverage_gap and metrics['elapsed_s'] >= self.window_s - _EPS
            if self.window_complete:
                if metrics['decisions'] == 0 or metrics['missing_headway_samples'] or metrics['missing_ttc_samples']:
                    self.failure_reasons.add('missing_observation_evidence')
                if metrics['intervention_decisions'] / max(1, metrics['decisions']) > self.max_intervention_fraction + _EPS:
                    self.failure_reasons.add('intervention_fraction')
                if metrics['progress_m'] < self.min_progress_ratio * metrics['reference_progress_m'] - _EPS:
                    self.failure_reasons.add('insufficient_progress')
                self.status = 'failed' if self.failure_reasons else 'succeeded'
            elif not self.failure_reasons:
                self.status = 'pending'
        if self.failure_reasons:
            self.status = 'failed'

    def finish(self, *, arrived=False, truncated=False):
        self.finished = True
        self.finish_reason = 'arrived' if arrived else 'truncated' if truncated else 'removed'
        return self.summary()

    def consume_stable_bonus(self):
        if self.status != 'succeeded' or self._bonus_consumed:
            return 0.
        self._bonus_consumed = True
        return self.stable_bonus

    def consume_lane_oscillation_penalty(self):
        count = self.full_route['unnecessary_reversals'] - self._penalties_consumed
        self._penalties_consumed += count
        return count

    @staticmethod
    def _summary(metrics):
        row = deepcopy(metrics)
        row['intervention_fraction'] = row['intervention_decisions'] / max(1, row['decisions'])
        row['progress_ratio'] = row['progress_m'] / row['reference_progress_m'] if row['reference_progress_m'] > 0 else None
        for key in ('requested_acceleration', 'filtered_acceleration', 'realized_acceleration'):
            row['mean_' + key] = row[key + '_sum'] / row[key + '_samples'] if row[key + '_samples'] else None
        return row

    def summary(self):
        return dict(schema=METRICS_VERSION, vehicle_id=self.vehicle_id,
            entry_event=self.entry_time_s is not None, entry_time_s=self.entry_time_s,
            lateral_merge_completed=self.merge_complete_time_s is not None,
            lateral_merge_complete_time_s=self.merge_complete_time_s,
            window_s=self.window_s, window_complete=self.window_complete,
            status=self.status, stable_merge_succeeded=self.status == 'succeeded',
            failure_reasons=sorted(self.failure_reasons), observation_coverage_gap=self._coverage_gap,
            finished=self.finished, finish_reason=self.finish_reason,
            full_route=self._summary(self.full_route), post_window=self._summary(self.post_window),
            lane_reversals=deepcopy(self.reversals),
            neighbor_braking_attribution='observed contemporaneous braking; no causal attribution',
            gates=dict(min_headway_s=self.min_headway_s, min_ttc_s=self.min_ttc_s,
                       max_intervention_fraction=self.max_intervention_fraction,
                       min_progress_ratio=self.min_progress_ratio))
