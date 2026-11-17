"""Measure the insertion lag and speed-limit trims in saved v06 round-7 validations (read-only).

Evidence for the round-8 proposal (documentation/highway-merge-training-plan-v06.md):
- per car, how many decisions pass before |request - measured| first falls to
  0.5 m/s2 (5 m/s3) after SUMO inserts the car at 0 m/s2;
- how many of the 8 validation cases pass the original gates plus the
  raw-request check when each car's first K decisions are skipped;
- speed-ceiling trims (count, largest and median trimmed amount).
No simulation and no ledger charge.

Usage: python scripts/measure_highway_flow_rl_v06_insertion_lag.py --run outputs/highway-flow-rl/v06/round-07 --output <json>
"""
import argparse
import glob
import gzip
import hashlib
import json
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from experiments.run_highway_flow_rl_v05 import development_gates  # noqa: E402

LABELS = ('initial-validation', 'periodic-validation-00041')
STEP_S, LIMIT_M_S3 = .1, 5.
SKIPS = range(9)


def case_steps(run, label, index):
    path = glob.glob(str(run / label / f'case-{index:02d}' / 'episode-*' / 'actions.jsonl.gz'))[0]
    with gzip.open(path, 'rt') as handle:
        for line in handle:
            yield json.loads(line)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = dict(schema='highway_flow_rl_v06_insertion_lag_v01', run=args.run.as_posix(),
                  limit_m_s3=LIMIT_M_S3, validations={})
    for label in LABELS:
        passes, cases = defaultdict(int), []
        for index in range(8):
            report = json.loads((args.run / label / f'{index:02d}.json').read_text(encoding='utf-8'))
            original = development_gates(report)['passed']
            assigned = [a for a, c in report['smooth_control']['per_car'].items() if c.get('assigned_controlled')]
            raw, trims = defaultdict(list), defaultdict(list)
            for step in case_steps(args.run, label, index):
                for agent, decision in step['agents'].items():
                    audit = decision.get('v05_audit')
                    if not audit:
                        continue
                    c = audit['conditioning']
                    raw[agent].append(abs(c['requested'] - c['measured']) / STEP_S)
                    if 'speed_ceiling' in (decision.get('override_causes') or []):
                        trims[agent].append(decision['requested_acceleration'] - decision['acceleration_bounds']['speed_ceiling'])
            for skip in SKIPS:
                passes[skip] += bool(original and all(max(raw[a][skip:] or [0.]) <= LIMIT_M_S3 + 1e-9 for a in assigned))
            cases.append(dict(case=report['case_id'], original_gates_passed=original,
                insertion_lag_decisions={a: next((k for k, v in enumerate(raw[a]) if v <= LIMIT_M_S3 + 1e-9), len(raw[a]))
                                         for a in assigned},
                peak_raw_after_5_decisions_m_s3={a: round(max(raw[a][5:] or [0.]), 2) for a in assigned},
                speed_ceiling_trims={a: dict(count=len(v), max_m_s2=round(max(v), 3), median_m_s2=round(sorted(v)[len(v) // 2], 3))
                                     for a, v in trims.items() if v}))
        result['validations'][label] = dict(
            sha256=hashlib.sha256((args.run / f'{label}.json').read_bytes()).hexdigest(),
            passing_cases_by_skipped_decisions={str(k): v for k, v in passes.items()}, cases=cases)
    args.output.write_text(json.dumps(result, indent=1) + '\n', encoding='utf-8')
    print(json.dumps({k: v['passing_cases_by_skipped_decisions'] for k, v in result['validations'].items()}))


if __name__ == '__main__':
    main()
