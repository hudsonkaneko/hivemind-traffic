"""Audit stored highway command journals without simulation or learning."""
from __future__ import annotations

import argparse
from collections import Counter
import gzip
import hashlib
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def summarize_journal(path: Path) -> dict:
    counts = Counter()
    causes = Counter()
    acceleration = []
    with gzip.open(path, 'rt', encoding='utf-8') as stream:
        for line in stream:
            frame = json.loads(line)
            if frame.get('control_mode') != 'policy':
                counts['non_policy_frames_excluded'] += 1
                continue
            for row in frame['agents'].values():
                requested = float(row['requested_acceleration'])
                filtered = float(row['applied_acceleration'])
                realized = row['realized_acceleration']
                realized = None if realized is None else float(realized)
                if not all(math.isfinite(v) for v in (requested, filtered)) or (realized is not None and not math.isfinite(realized)):
                    raise ValueError(f'Nonfinite command in {path}')
                counts['agent_decisions'] += 1
                counts['recorded_overrides'] += bool(row['override'])
                counts['requested_filtered_difference'] += abs(requested-filtered) > 1e-4
                counts['filtered_realized_difference'] += realized is not None and abs(filtered-realized) > .05
                counts['realization_unobserved'] += realized is None
                counts['lane_rejected'] += bool(row.get('lane_rejected', False))
                counts['near_max_acceleration_requests'] += requested > 1.8
                # Older journals do not record exact shield causes. Differences
                # alone cannot establish speed/following/native braking causes.
                reasons = row.get('intervention_causes', row.get('override_causes'))
                if reasons is None:
                    counts['decisions_without_cause_telemetry'] += 1
                else:
                    if not isinstance(reasons, (list, tuple)) or not all(isinstance(s, str) for s in reasons):
                        raise ValueError('Invalid intervention-causes list')
                    causes.update(set(reasons))
                if realized is not None:
                    acceleration.append([requested, filtered, realized])
    n = counts['agent_decisions']
    values = np.asarray(acceleration, dtype=np.float64)
    stats = {}
    if len(values):
        stats = {
            'mean_requested_filtered_realized_m_s2': values.mean(axis=0).tolist(),
            'mean_abs_requested_filtered_error_m_s2': float(np.abs(values[:, 0]-values[:, 1]).mean()),
            'mean_abs_filtered_realized_error_m_s2': float(np.abs(values[:, 1]-values[:, 2]).mean()),
            'p95_abs_requested_realized_error_m_s2': float(np.quantile(np.abs(values[:, 0]-values[:, 2]), .95)),
        }
    return dict(journal=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                counts=dict(counts), cause_decision_counts=dict(causes),
                override_fraction=counts['recorded_overrides']/n if n else None,
                near_max_request_fraction=counts['near_max_acceleration_requests']/n if n else None,
                acceleration_statistics_observed_decisions=len(acceleration), **stats)


def summarize(paths) -> dict:
    reports = [summarize_journal(Path(path)) for path in paths]
    if not reports:
        raise ValueError('No action journals found')
    counts = Counter()
    causes = Counter()
    for report in reports:
        counts.update(report['counts'])
        causes.update(report['cause_decision_counts'])
    n = counts['agent_decisions']
    return dict(schema='highway_action_audit_v1', journals=reports, counts=dict(counts),
                cause_decision_counts=dict(causes),
                override_fraction=counts['recorded_overrides']/n if n else None,
                near_max_request_fraction=counts['near_max_acceleration_requests']/n if n else None,
                semantics={
                    'counts': 'Per recorded active-agent decision; repeated episodes remain separate journals.',
                    'causes': 'Only explicitly logged causes. Multiple causes may overlap on a decision.',
                    'native': 'Filtered/realized difference is not proof of a particular native SUMO safety cause.',
                    'legacy': 'Terminal realized values may be synthetic in old journals; original records are preserved.',
                }, optimizer_steps=0, simulator_runs=0)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    source, target = args.input_root.resolve(), args.output.resolve()
    if not source.is_relative_to(ROOT/'outputs') or not target.is_relative_to(ROOT):
        raise ValueError('Read project outputs and save inside this checkout')
    if target.exists():
        raise FileExistsError('Preserve previous audits; choose a new output path')
    result = summarize(sorted(source.rglob('actions.jsonl.gz')))
    for report in result['journals']:
        report['journal'] = str(Path(report['journal']).relative_to(ROOT)).replace('\\', '/')
    result['input_root'] = source.relative_to(ROOT).as_posix()
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k != 'journals'}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
