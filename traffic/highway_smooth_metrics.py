"""Uncensored longitudinal diagnostics from measured, consecutive SUMO samples.

This module has no simulator dependency. Command/reference histories are local
to a vehicle and a consecutive decision; absent samples are never filled with
zero. Native startup acceleration is measured, so its first derivative is kept.
"""
from __future__ import annotations

import math
import numpy as np

METRICS_VERSION = 'highway_smooth_metrics_v01'
DT = .1
JERK_KEYS = ('requested', 'filtered', 'realized')


def huber(value):
    """Unit-threshold Huber penalty with an uncapped linear tail."""
    value = abs(float(value))
    if not math.isfinite(value):
        raise ValueError('Huber input must be finite')
    return .5 * value * value if value <= 1. else value - .5


def distribution(values, *, eligible=None):
    values = [float(v) for v in values if v is not None]
    if any(not math.isfinite(v) for v in values):
        raise ValueError('Diagnostic samples must be finite')
    absolute = np.abs(values)
    n = len(values)
    return dict(samples=n, eligible_samples=n if eligible is None else eligible,
                missing_samples=0 if eligible is None else eligible - n,
                mean_abs=float(absolute.mean()) if n else None,
                rms=float(np.sqrt(np.mean(absolute ** 2))) if n else None,
                p95_abs=float(np.percentile(absolute, 95)) if n else None,
                peak_abs=float(absolute.max()) if n else None)


def _summary(rows):
    measured = [r for r in rows if r['speed_m_s'] is not None]
    decisions = [r for r in rows if r['requested_acceleration_m_s2'] is not None]
    out = dict(rows=len(rows), measured_samples=len(measured), decisions=len(decisions),
        command_applied_decisions=sum(r['command_actually_applied'] for r in decisions),
        counterfactual_command_decisions=sum(not r['command_actually_applied'] for r in decisions),
        reference_samples=sum(r['reference_m_s2'] is not None for r in decisions),
        stopped_time_s=sum(r['interval_s'] for r in measured if r['speed_m_s'] < .1),
        measured_time_s=sum(r['interval_s'] for r in measured),
        measured_progress_m=sum(r['progress_m'] for r in measured),
        intervention_decisions=sum(r['intervention'] for r in decisions),
        reference_braking_priority_events=sum(r['reference_braking_priority'] for r in decisions),
        unknown_constraint_samples=sum(r['constraint_unknown'] for r in rows),
        jerk_m_s3={}, acceleration_total_variation_m_s2={})
    for name in JERK_KEYS:
        key = name + '_jerk_m_s3'
        eligible = len(measured) if name == 'realized' else len(decisions)
        out['jerk_m_s3'][name] = distribution([r[key] for r in rows], eligible=eligible)
        out['acceleration_total_variation_m_s2'][name] = sum(
            abs(r[key]) * DT for r in rows if r[key] is not None)
    out['reference_error_m_s2'] = distribution(
        [r['reference_error_m_s2'] for r in rows], eligible=len(decisions))
    out['reference_residual_slew_m_s3'] = distribution(
        [r['reference_residual_slew_m_s3'] for r in rows], eligible=len(decisions))
    out['feasible_next_speed_error_m_s'] = distribution(
        [r['feasible_next_speed_error_m_s'] for r in rows], eligible=len(decisions))
    ratios = [r['legal_speed_ratio'] for r in measured if r['legal_speed_ratio'] is not None]
    out['legal_speed_utilization'] = dict(samples=len(ratios),
        mean=sum(ratios) / len(ratios) if ratios else None)
    return out


def _drawdown(trace, entry, completion, window_s=8.):
    measured = [r for r in trace if r['speed_m_s'] is not None]
    start = entry - 5. if entry is not None else None
    end = completion + window_s if completion is not None else None
    window = [r for r in measured if (start is None or r['time_s'] >= start - 1e-8)
              and (end is None or r['time_s'] <= end + 1e-8)]
    peak = None
    pair = None
    loss = -1.
    for row in window:
        if peak is not None:
            candidate = max(0., peak['speed_m_s'] - row['speed_m_s'])
            if candidate > loss:
                loss, pair = candidate, (peak, row)
        if peak is None or row['speed_m_s'] > peak['speed_m_s']:
            peak = row
    gaps = sum(b['time_s'] - a['time_s'] > DT + 1e-7 for a, b in zip(window, window[1:]))
    coverage = bool(entry is not None and completion is not None and window
                    and window[0]['time_s'] <= start + 1e-7
                    and window[-1]['time_s'] >= end - 1e-7 and not gaps)
    changes = [dict(time_s=b['time_s'], before_m_s=a['legal_speed_m_s'], after_m_s=b['legal_speed_m_s'])
               for a, b in zip(window, window[1:])
               if not math.isclose(a['legal_speed_m_s'], b['legal_speed_m_s'], abs_tol=1e-8)]
    out = dict(coverage_complete=coverage, requested_start_s=start, requested_end_s=end,
        available_start_s=window[0]['time_s'] if window else None,
        available_end_s=window[-1]['time_s'] if window else None,
        samples=len(window), nonconsecutive_gaps=gaps, entry_time_s=entry,
        settled_completion_time_s=completion, legal_limit_changes=changes,
        interpretation='Observed slowdown, not a causal claim; incomplete windows retain available trajectory',
        observed_loss_m_s=max(0., loss) if pair else None,
        fraction_of_preceding_peak=(max(0., loss) / pair[0]['speed_m_s'] if pair and pair[0]['speed_m_s'] > 0 else None),
        peak_time_s=pair[0]['time_s'] if pair else None,
        trough_time_s=pair[1]['time_s'] if pair else None)
    interval = [r for r in window if pair and pair[0]['time_s'] < r['time_s'] <= pair[1]['time_s']]
    out['slowdown_interval'] = dict(samples=len(interval),
        reference_braking_samples=sum(r['reference_m_s2'] is not None and r['reference_m_s2'] < 0 for r in interval),
        unknown_reference_samples=sum(r['reference_m_s2'] is None for r in interval),
        falling_legal_limit=any(c['after_m_s'] < c['before_m_s'] for c in changes
                                if pair and pair[0]['time_s'] < c['time_s'] <= pair[1]['time_s']))
    return out


def nominal_merge_stop(trace, stable_completion_time_s):
    """Longest observed low-speed span through first stable completion, inclusive.

    Duration is last-minus-first timestamp over consecutive measured samples;
    no unobserved onset or arrival interval is invented. The reset sample is
    included. Missing stable completion keeps coverage false and observations.
    """
    rows = [r for r in trace if stable_completion_time_s is None
            or r['time_s'] <= stable_completion_time_s + 1e-8]
    measured = [r for r in rows if r['speed_m_s'] is not None]
    start = previous = None
    longest, best_start, best_end = 0., None, None
    gaps = 0
    for row in measured:
        consecutive = previous is not None and abs(row['time_s'] - previous['time_s'] - DT) < 1e-7
        if previous is not None and not consecutive:
            gaps += 1
        if row['speed_m_s'] < .1:
            if start is None or not consecutive:
                start = row['time_s']
            elapsed = row['time_s'] - start
            if best_start is None or elapsed > longest:
                longest, best_start, best_end = elapsed, start, row['time_s']
        else:
            start = None
        previous = row
    complete = bool(stable_completion_time_s is not None and measured and not gaps
        and measured[-1]['time_s'] >= stable_completion_time_s - 1e-7
        and len(measured) == len(rows))
    return dict(coverage_complete=complete, longest_consecutive_stop_s=longest,
        threshold_speed_m_s=.1, start_time_s=best_start, end_time_s=best_end,
        stable_completion_time_s=stable_completion_time_s,
        observed_start_time_s=measured[0]['time_s'] if measured else None,
        observed_end_time_s=measured[-1]['time_s'] if measured else None,
        measured_samples=len(measured), nonconsecutive_gaps=gaps,
        duration_definition='Elapsed span between consecutive measured speed<0.1 samples, including reset; first/arrival duration is not invented')


class SmoothControlMetrics:
    """All native cars, with command metrics only where a command was recorded.

    ``startup`` is the first measured acceleration interval for each vehicle,
    not an excluded warmup. A reset state supplies the first interval endpoint;
    a newly spawned car lacking a previous state has a missing first derivative.
    """
    def __init__(self, *, task='', merge_vehicle_ids=(), controlled_ids=(), window_s=8.):
        self.task, self.window_s = task, float(window_s)
        self.merge_vehicle_ids = set(merge_vehicle_ids)
        self.controlled_ids = set(controlled_ids)
        self.traces, self.previous_commands, self.native_intervals = {}, {}, {}
        self.entries, self.completions, self.lateral_starts, self.stable_completions = {}, {}, {}, {}
        self.arrived = set()

    @staticmethod
    def _state_row(time_s, state):
        return dict(time_s=float(time_s), speed_m_s=float(state['speed']) if state else None,
            acceleration_m_s2=float(state['acceleration']) if state else None,
            legal_speed_m_s=min(26.8224, float(state['allowed_speed'])) if state else None,
            lane_id=state['lane_id'] if state else None)

    def prime(self, time_s, states):
        for vehicle_id, state in states.items():
            if vehicle_id not in self.traces:
                row = self._state_row(time_s, state)
                row['baseline'] = True
                self.traces[vehicle_id] = [row]

    def observe(self, time_s, before, after, infos, *, merge_vehicle_ids=(), arrived=()):
        self.merge_vehicle_ids.update(merge_vehicle_ids)
        self.arrived.update(arrived)
        self.prime(time_s - DT, before)
        current_commands = {}
        for vehicle_id in sorted(set(after) | set(infos)):
            state, old, info = after.get(vehicle_id), before.get(vehicle_id), infos.get(vehicle_id, {})
            trace = self.traces.setdefault(vehicle_id, [])
            last = trace[-1] if trace else None
            consecutive = bool(old and last and abs(last['time_s'] - (time_s - DT)) < 1e-7)
            native_jerk = (state['acceleration'] - old['acceleration']) / DT if state and consecutive else None
            startup = native_jerk is not None and self.native_intervals.get(vehicle_id, 0) == 0
            if native_jerk is not None:
                self.native_intervals[vehicle_id] = self.native_intervals.get(vehicle_id, 0) + 1
            requested = info.get('requested_acceleration')
            filtered = info.get('filtered_acceleration')
            reference = info.get('feasible_acceleration_target')
            previous = self.previous_commands.get(vehicle_id)
            command_consecutive = previous is not None and abs(previous['time_s'] - (time_s - DT)) < 1e-7
            row = self._state_row(time_s, state)
            row.update(baseline=False, startup=startup, interval_s=DT if state and consecutive else 0.,
                requested_acceleration_m_s2=requested, filtered_acceleration_m_s2=filtered,
                requested_jerk_m_s3=(requested - previous['requested']) / DT if requested is not None and command_consecutive else None,
                filtered_jerk_m_s3=(filtered - previous['filtered']) / DT if filtered is not None and command_consecutive else None,
                realized_jerk_m_s3=native_jerk, reference_m_s2=reference,
                reference_error_m_s2=requested - reference if requested is not None and reference is not None else None,
                reference_residual_slew_m_s3=info.get('command_reference_residual_slew_m_s3'),
                feasible_next_speed_error_m_s=(state['speed'] - min(max(old['speed'] + DT * reference, 0.),
                    min(26.8224, old['allowed_speed']))) if state and consecutive and reference is not None else None,
                legal_speed_ratio=state['speed'] / max(.1, min(26.8224, state['allowed_speed'])) if state else None,
                progress_m=max(0., state['distance'] - old['distance']) if state and consecutive else 0.,
                command_actually_applied=bool(info.get('command_actually_applied', False)),
                intervention=bool(info.get('override', False)),
                reference_braking_priority=bool(info.get('smooth_reference', {}).get('braking_override_soft_slew', False)),
                safety_constrained=bool(info.get('override', False) or info.get('requested_constraint_violations')),
                constraint_unknown=not bool(info),
                maneuver_in_progress=bool(info.get('maneuver', {}).get('in_progress', False)))
            trace.append(row)
            if requested is not None and filtered is not None:
                current_commands[vehicle_id] = dict(time_s=time_s, requested=requested, filtered=filtered)
            if vehicle_id in self.merge_vehicle_ids and state:
                if info.get('stable_merge_status') == 'succeeded':
                    self.stable_completions.setdefault(vehicle_id, time_s)
                kind = state.get('lane_identity', {}).get('kind')
                old_kind = old.get('lane_identity', {}).get('kind') if old else None
                if vehicle_id not in self.entries and (state['lane_id'].startswith(':') or
                        old_kind == 'ramp' and kind in ('auxiliary', 'through')):
                    self.entries[vehicle_id] = time_s
                if row['maneuver_in_progress']:
                    self.lateral_starts.setdefault(vehicle_id, time_s)
                if info.get('merge_completed'):
                    self.completions.setdefault(vehicle_id, time_s)
        # Missing/disappeared or ended decisions cannot bridge command history.
        self.previous_commands = {a: c for a, c in current_commands.items() if a in after}

    def _phase(self, vehicle_id, row):
        if vehicle_id not in self.merge_vehicle_ids:
            return 'route'
        time = row['time_s']
        entry, completion = self.entries.get(vehicle_id), self.completions.get(vehicle_id)
        if completion is not None and time >= completion - 1e-8:
            return 'post_merge'
        if row.get('maneuver_in_progress'):
            return 'lateral_maneuver'
        if entry is not None and time >= entry - 1e-8:
            return 'native_join'
        return 'approach'

    def report(self):
        per_car, phase_rows, all_rows, controlled_rows, background_rows = {}, {}, [], [], []
        for vehicle_id, trace in sorted(self.traces.items()):
            rows = [r for r in trace if not r['baseline']]
            phases = {}
            for row in rows:
                phase = self._phase(vehicle_id, row)
                phases.setdefault(phase, []).append(row)
                phase_rows.setdefault(phase, []).append(row)
            all_rows.extend(rows)
            (controlled_rows if vehicle_id in self.controlled_ids else background_rows).extend(rows)
            per_car[vehicle_id] = dict(all=_summary(rows),
                assigned_controlled=vehicle_id in self.controlled_ids,
                startup=_summary([r for r in rows if r['startup']]),
                later=_summary([r for r in rows if not r['startup']]),
                phases={phase: _summary(values) for phase, values in phases.items()},
                route_completed=vehicle_id in self.arrived,
                first_observed_time_s=trace[0]['time_s'] if trace else None,
                last_observed_time_s=trace[-1]['time_s'] if trace else None)
            if vehicle_id in self.merge_vehicle_ids:
                # Baselines supply real historical speed endpoints for drawdown.
                drawdown_trace = [dict(r, reference_m_s2=r.get('reference_m_s2')) for r in trace]
                per_car[vehicle_id]['merge_slowdown'] = _drawdown(drawdown_trace,
                    self.entries.get(vehicle_id), self.completions.get(vehicle_id), self.window_s)
                per_car[vehicle_id]['nominal_merge_stop'] = nominal_merge_stop(trace,
                    self.stable_completions.get(vehicle_id))
        return dict(schema=METRICS_VERSION, task=self.task, dt_s=DT,
            startup_definition='First consecutive measured acceleration interval per vehicle; included in all-sample metrics and peak',
            missing_definition='Missing first command, arrival state, or nonconsecutive history is not zero; eligible and missing counts retained',
            phase_definition='Approach before first native junction sample; native join before settled completion excluding active lateral motion; post merge after settled completion',
            native_state_scope='All observed vehicles, including post-control native tail when caller advances it; this report alone does not assert a complete horizon',
            controlled_and_native_cars=len(per_car),
            assigned_controlled_ids=sorted(self.controlled_ids),
            controlled=_summary(controlled_rows), background=_summary(background_rows),
            all=_summary(all_rows), startup=_summary([r for r in all_rows if r['startup']]),
            later=_summary([r for r in all_rows if not r['startup']]),
            safety_constrained=_summary([r for r in all_rows if r['safety_constrained']]),
            observed_unflagged=_summary([r for r in all_rows if not r['safety_constrained'] and not r['constraint_unknown']]),
            constraint_unknown=_summary([r for r in all_rows if r['constraint_unknown']]),
            per_phase={phase: _summary(values) for phase, values in phase_rows.items()}, per_car=per_car)
