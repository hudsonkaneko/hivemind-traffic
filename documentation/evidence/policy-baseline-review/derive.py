"""Reproduce offline baseline diagnostics from retained journals; no learning."""
from pathlib import Path
import gzip
import json
from collections import Counter

ROOT = Path(__file__).resolve().parents[3]
OUTPUT = Path(__file__).with_name('diagnosis.json')
RUNS = ['train-20261104T225405Z-302c4943', 'train-20261105T045039Z-a6db8f1b']


def main():
    result = {}
    for run_name in RUNS:
        run = ROOT / 'outputs/highway-population/v01' / run_name
        for label in ('initial-validation', 'final-validation'):
            reports = json.loads((run / (label + '.json')).read_text(encoding='utf-8'))['reports']
            report = reports[0]
            journal = ROOT / report['episode_directory'] / 'actions.jsonl.gz'
            values = []
            with gzip.open(journal, 'rt', encoding='utf-8') as stream:
                for line in stream:
                    for row in json.loads(line)['agents'].values():
                        if row['realized_acceleration'] is not None:
                            values.append(row)
            result[run_name + '/' + label] = dict(
                sparse_case={key: sum(v[key] for v in values) / len(values)
                    for key in ('requested_acceleration', 'filtered_acceleration', 'realized_acceleration')},
                sparse_case_speed_ratio=report['mean_fraction_of_speed_limit'],
                sparse_case_overrides=report['intervention_fraction'],
                totals={key: sum(r.get(key, 0) for r in reports)
                    for key in ('overrides', 'native_realization_changes', 'policy_filter_changes')})
        counts = Counter()
        coverage = json.loads((run / 'training-coverage.json').read_text(encoding='utf-8'))
        for key, row in coverage.items():
            counts[key.split('|')[1]] += row['actual_agent_transitions']
        result[run_name + '/coverage'] = dict(cells=len(coverage), decisions_by_av_count=dict(counts))
    OUTPUT.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(OUTPUT)


if __name__ == '__main__':
    main()
