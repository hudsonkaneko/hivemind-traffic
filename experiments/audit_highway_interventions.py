"""Offline magnitude, duration and target-fit audit of retained Stage 1 journals.

The original override union and thresholds remain unchanged. Native realization
differences and agreement with a logged predictor do not identify SUMO's hidden
controller mode or establish a near-collision.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import gzip
import hashlib
import json
import math
from pathlib import Path
import platform

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
NATIVE_THRESHOLD = .05
FILTER_THRESHOLD = 1e-4
DEFAULT_STAGES = ('validation-after-bc', 'validation-phase-2', 'validation-phase-3')
VALUE_NAMES = ('requested_minus_filtered', 'requested_minus_target', 'filtered_minus_realized',
    'realized_minus_target', 'realized_minus_min_filtered_target', 'paired_request_target_error',
    'paired_native_delta', 'native_only_filtered_minus_realized',
    'native_only_requested_minus_target', 'native_only_realized_minus_target')


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def distribution(values):
    """Finite arithmetic summaries; empty samples remain unknown, not zero."""
    if not values:
        return dict(count=0, mean=None, p50=None, p95=None, maximum=None)
    a = np.asarray(values, dtype=np.float64)
    return dict(count=len(a), mean=float(a.mean()), p50=float(np.quantile(a, .5)),
                p95=float(np.quantile(a, .95)), maximum=float(a.max()))


def magnitude_bins(values):
    a = np.abs(np.asarray(values, dtype=np.float64))
    cuts = (0., .05, .1, .25, .5, 1., float('inf'))
    labels = ('0_to_0p05', 'gt_0p05_to_0p1', 'gt_0p1_to_0p25',
              'gt_0p25_to_0p5', 'gt_0p5_to_1', 'gt_1')
    return {name: int(np.count_nonzero((a >= low if i == 0 else a > low) & (a <= high)))
            for i, (name, low, high) in enumerate(zip(labels, cuts, cuts[1:]))}


class Accumulator:
    def __init__(self, dt=.1):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('Control interval must be finite and positive')
        self.dt = dt
        self.counts = Counter()
        self.causes = Counter()
        self.values = defaultdict(list, {name: [] for name in VALUE_NAMES})
        self.runs = {}
        self.run_lengths = []

    def finish_episode(self):
        self.run_lengths.extend(length for _, length in self.runs.values())
        self.runs.clear()

    def _finish_agent(self, agent):
        if agent in self.runs:
            self.run_lengths.append(self.runs.pop(agent)[1])

    def add_frame(self, frame):
        if frame.get('control_mode') != 'policy':
            self.counts['non_policy_frames_excluded'] += 1
            self.finish_episode()
            return
        time = float(frame['time'])
        if not math.isfinite(time):
            raise ValueError('Nonfinite frame time')
        agents = frame['agents']
        for agent in set(self.runs) - set(agents):
            self._finish_agent(agent)
        for agent, row in agents.items():
            requested = float(row['requested_acceleration'])
            filtered = float(row.get('filtered_acceleration', row['applied_acceleration']))
            realized = row['realized_acceleration']
            realized = None if realized is None else float(realized)
            target = row.get('feasible_acceleration_target')
            target = None if target is None else float(target)
            if not all(math.isfinite(x) for x in (requested, filtered)) or any(
                    x is not None and not math.isfinite(x) for x in (realized, target)):
                raise ValueError('Nonfinite command/target')
            self.counts['agent_decisions'] += 1
            recorded = bool(row['override'])
            filter_change = abs(requested - filtered) > FILTER_THRESHOLD
            native = realized is not None and abs(filtered - realized) > NATIVE_THRESHOLD
            lane_rejected = bool(row.get('lane_rejected', False))
            self.counts['recorded_overrides'] += recorded
            self.counts['recomputed_override_union'] += filter_change or native or lane_rejected
            self.counts['override_union_mismatch'] += recorded != (filter_change or native or lane_rejected)
            self.counts['filter_changes'] += filter_change
            self.counts['native_changes'] += native
            self.counts['lane_rejected'] += lane_rejected
            self.counts['realization_unobserved'] += realized is None
            self.counts['target_unobserved'] += target is None
            lane_request = bool(row.get('lane_request', 0))
            lane_change = bool(row.get('lane_change_observed', False))
            self.counts['lane_request_decisions'] += lane_request
            self.counts['lane_change_decisions'] += lane_change
            self.counts['native_changes_with_lane_request'] += native and lane_request
            self.counts['native_changes_with_observed_lane_change'] += native and lane_change
            reasons = row.get('override_causes')
            if reasons is None:
                self.counts['missing_cause_telemetry'] += 1
            else:
                if not isinstance(reasons, list) or not all(isinstance(x, str) for x in reasons):
                    raise ValueError('Invalid cause telemetry')
                self.causes.update(set(reasons))
            self.values['requested_minus_filtered'].append(requested - filtered)
            if target is not None:
                self.values['requested_minus_target'].append(requested - target)
                self.counts['request_above_target_gt_0p05'] += requested - target > NATIVE_THRESHOLD
                self.counts['request_below_target_gt_0p05'] += target - requested > NATIVE_THRESHOLD
            if realized is not None:
                delta = filtered - realized
                self.values['filtered_minus_realized'].append(delta)
                self.counts['native_reductions'] += delta > NATIVE_THRESHOLD
                self.counts['native_increases'] += delta < -NATIVE_THRESHOLD
                if target is not None:
                    prediction = min(filtered, target)
                    self.values['realized_minus_target'].append(realized - target)
                    self.values['realized_minus_min_filtered_target'].append(realized - prediction)
                    self.values['paired_request_target_error'].append(requested - target)
                    self.values['paired_native_delta'].append(delta)
                    predicted_native = filtered - target > NATIVE_THRESHOLD
                    self.counts['native_predicted_by_target'] += predicted_native
                    self.counts['native_predicted_true_positive'] += predicted_native and native
                    self.counts['native_predicted_false_positive'] += predicted_native and not native
                    self.counts['native_predicted_false_negative'] += native and not predicted_native
                if native:
                    self.values['native_only_filtered_minus_realized'].append(delta)
                    self.counts['native_changes_realized_braking'] += realized < -.05
                    self.counts['native_changes_realized_nonbraking'] += realized >= -.05
                    self.counts['native_changes_requested_accel_realized_braking'] += requested > .05 and realized < -.05
                    if target is not None:
                        self.values['native_only_requested_minus_target'].append(requested - target)
                        self.values['native_only_realized_minus_target'].append(realized - target)
                        self.counts['native_changes_target_matches_realized_within_0p05'] += abs(realized-target) <= NATIVE_THRESHOLD
            if native:
                previous = self.runs.get(agent)
                if previous and not math.isclose(time - previous[0], self.dt, abs_tol=1e-6):
                    self._finish_agent(agent)
                    previous = None
                self.runs[agent] = (time, 1 + (previous[1] if previous else 0))
            else:
                self._finish_agent(agent)

    def report(self):
        self.finish_episode()
        n = self.counts['agent_decisions']
        native = self.counts['native_changes']
        metrics = {}
        for name, values in self.values.items():
            if name.startswith('paired_'):
                continue
            metrics[name + '_m_s2'] = dict(signed=distribution(values),
                absolute=distribution([abs(x) for x in values]), absolute_bins=magnitude_bins(values))
        x = np.asarray(self.values['paired_request_target_error'])
        y = np.asarray(self.values['paired_native_delta'])
        correlation = float(np.corrcoef(x, y)[0, 1]) if len(x) > 1 and x.std() > 0 and y.std() > 0 else None
        return dict(counts=dict(self.counts), explicitly_logged_causes=dict(self.causes),
            override_fraction=self.counts['recorded_overrides']/n if n else None,
            native_change_fraction=native/n if n else None,
            native_target_agreement_fraction=self.counts['native_changes_target_matches_realized_within_0p05']/native if native else None,
            acceleration=metrics,
            native_change_runs=dict(count=len(self.run_lengths), decisions=distribution(self.run_lengths),
                duration_s=distribution([x*self.dt for x in self.run_lengths]),
                runs_at_least_one_second=sum(x*self.dt >= 1. for x in self.run_lengths)),
            request_target_error_vs_native_delta_pearson=correlation)


def audit_reports(report_path, root=ROOT, dt=.1):
    reports = json.loads(Path(report_path).read_text(encoding='utf-8'))
    total = Accumulator(dt)
    cases = []
    for source in reports:
        episode = (root / Path(source['episode_directory'].replace('\\', '/'))).resolve()
        if not episode.is_relative_to(root / 'outputs'):
            raise ValueError('Episode journal must remain inside project outputs')
        path = episode / 'actions.jsonl.gz'
        current = Accumulator(dt)
        with gzip.open(path, 'rt', encoding='utf-8') as stream:
            for line in stream:
                frame = json.loads(line)
                current.add_frame(frame)
                total.add_frame(frame)
        total.finish_episode()
        result = current.report()
        if result['counts']['agent_decisions'] != source['agent_steps'] or result['counts']['recorded_overrides'] != source['overrides']:
            raise ValueError('Retained report and journal counts disagree')
        if result['counts'].get('override_union_mismatch', 0):
            raise ValueError('Recorded metric differs from documented override union')
        cases.append(dict(case_id=source['case_id'], seed=source['seed'], task=source['task'],
            journal=path.relative_to(root).as_posix(), journal_sha256=sha256(path), **result))
    return dict(source_report=Path(report_path).relative_to(root).as_posix(),
        source_report_sha256=sha256(report_path), **total.report(), cases=cases)


def audit_run(run, output, root=ROOT):
    run, output = Path(run).resolve(), Path(output).resolve()
    if not run.is_relative_to(root/'outputs') or not output.is_relative_to(root/'outputs/highway-stage1/v03'):
        raise ValueError('Use project outputs for input and a fresh Stage 1 v03 audit directory')
    if output.exists():
        raise FileExistsError('Never overwrite prior diagnostic evidence')
    cfg = json.loads((run/'config.json').read_text(encoding='utf-8'))
    provenance = json.loads((run/'provenance.json').read_text(encoding='utf-8'))
    selected = json.loads((run/'selection.json').read_text(encoding='utf-8'))['generation']
    if Path(selected).name != selected:
        raise ValueError('Invalid selected-generation name')
    selected_contract = json.loads((run/selected/'policy-contract.json').read_text(encoding='utf-8'))
    if selected_contract['validation_report'] != DEFAULT_STAGES[-1]+'.json':
        raise ValueError('This audit expects the selected actor to match the final phase-3 report')
    if (sha256(run/selected/'validation.json') != selected_contract['validation_sha256'] or
            sha256(run/selected_contract['validation_report']) != selected_contract['validation_sha256']):
        raise ValueError('Selected validation checksum mismatch')
    stages = {name: audit_reports(run/(name+'.json'), root, cfg['contract']['step_s']) for name in DEFAULT_STAGES}
    result = dict(schema='highway_intervention_diagnostic_v1', created_utc=datetime.now(timezone.utc).isoformat(),
        input_run=run.relative_to(root).as_posix(), config_sha256=sha256(run/'config.json'),
        source_run_commit=provenance['git_head'], selected_generation=selected,
        runtime=dict(python=platform.python_version(), numpy=np.__version__),
        script_sha256=sha256(Path(__file__)), optimizer_steps=0, simulator_runs=0, stages=stages,
        semantics={
            'override_metric': 'Unchanged union: lane rejection OR abs(requested-filtered)>1e-4 OR observed abs(filtered-realized)>0.05. Active-agent decision denominator.',
            'native_sign': 'filtered-realized > 0 is a native reduction; < 0 is a native increase.',
            'runs': 'Consecutive native-change decisions for the same vehicle within one episode at configured control dt. Missing vehicles, gaps, non-policy frames and episode boundaries end a run. Duration is decisions*dt.',
            'target': 'The logged feasible target is an analytical/SUMO-ACC prediction computed before action. Association with measured realization is not proof of native controller cause.',
            'prediction': 'Diagnostic min(filtered,target) model only; no action is changed and no acceptance threshold is relaxed.',
            'unknown': 'Journals do not contain full input observations, exact car-following mode, gap/TTC or reasons for native safety intervention. Near-collision labels cannot be inferred.',
            'correlation': 'Pearson correlation across recorded decisions is descriptive, temporally dependent and not causal; it is not a significance test.',
        })
    output.mkdir(parents=True)
    (output/'audit.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if Path.cwd().resolve() != ROOT:
        raise ValueError('Run from the canonical project checkout root')
    result = audit_run(args.run, args.output)
    print(json.dumps({name: dict(counts=s['counts'], override_fraction=s['override_fraction'],
        native_target_agreement_fraction=s['native_target_agreement_fraction']) for name, s in result['stages'].items()}, indent=2))


if __name__ == '__main__':
    main()
