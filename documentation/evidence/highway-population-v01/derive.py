"""Derive the compact population pilot report from retained run artifacts."""
from pathlib import Path
import argparse
import hashlib
import json

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def compact(report):
    names = ('case_id', 'seed', 'task', 'population_setting', 'controlled_count',
             'total_vehicles', 'requested_av_fraction', 'agent_steps', 'overrides',
             'intervention_fraction', 'override_causes', 'collisions', 'teleports',
             'overspeed_steps', 'mean_fraction_of_speed_limit', 'passes',
             'successful_merges', 'merge_target_count', 'lane_changes',
             'mean_abs_jerk', 'peak_abs_jerk', 'emergency_braking_observed',
             'minimum_headway_s', 'minimum_ttc_s',
             'population_metrics', 'delivered_messages', 'dropped_messages',
             'delivered_bytes', 'primary_horizon_complete',
             'accounting_comparison_valid', 'episode_directory')
    result = {name: report.get(name) for name in names}
    result['accounting_totals'] = report.get('accounting', {}).get('totals')
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', type=Path, required=True)
    args = parser.parse_args()
    run = args.run.resolve()
    if not run.is_relative_to(ROOT / 'outputs'):
        raise ValueError('Run must be inside project outputs')
    names = ('summary.json', 'provenance.json', 'config.json', 'comparison.json',
             'initial-validation.json', 'final-validation.json', 'native-baseline.json',
             'training-coverage.json', 'policy-contract.json')
    data = {name: json.loads((run / name).read_text(encoding='utf-8')) for name in names}
    before = data['initial-validation.json']['reports']
    after = data['final-validation.json']['reports']
    if len(before) != 16 or len(after) != 16:
        raise ValueError('Expected the complete fixed validation matrix')
    for first, last in zip(before, after):
        for key in ('case_id', 'seed', 'population_setting'):
            if first[key] != last[key]:
                raise ValueError('Initial/final comparison is not matched')
    evidence = dict(run=str(run.relative_to(ROOT)).replace('\\', '/'),
        artifact_sha256={name: digest(run / name) for name in (*names, 'policy.pt', 'last.pt')},
        summary=data['summary.json'], provenance=data['provenance.json'],
        comparison=data['comparison.json'], training_coverage=data['training-coverage.json'],
        initial=[compact(r) for r in before], final=[compact(r) for r in after],
        native_baselines=[compact(r) for r in data['native-baseline.json']['reports']],
        limits='One seed and 16 validation cases; no held-out test, live handoff or physical/sensor transfer. Raw episode journals remain local in the run package.')
    (HERE / 'results.json').write_text(json.dumps(evidence, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    lines = ['# Matched population validation', '',
        'Generated from `results.json`. Overrides use the unchanged combined intervention metric. Both evaluations use the same sixteen cases; neither is a held-out test.', '',
        '| Case | Task | AV / total | Channel | Initial overrides | Final overrides | Initial / final speed ratio | Initial / final passes | Initial / final merges (targets) |',
        '| --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for i, (a, b) in enumerate(zip(before, after)):
        p = b['population_setting']
        channel = f"{p['communication']} / {p['packet_loss']:.0%} loss"
        lines.append(f"| {i} | {b['task']} | {p['controlled_count']} / {p['total_vehicles']} | {channel} | "
            f"{a['intervention_fraction']:.2%} | {b['intervention_fraction']:.2%} | "
            f"{a['mean_fraction_of_speed_limit']:.3f} / {b['mean_fraction_of_speed_limit']:.3f} | "
            f"{a['passes']} / {b['passes']} | {a['successful_merges']} / {b['successful_merges']} ({b['merge_target_count']}) |")
    (HERE / 'comparison.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    brief = lambda s: {k: (len(v) if k == 'failures' else v) for k, v in s.items()}
    print(json.dumps(dict(initial=brief(data['comparison.json']['initial']), final=brief(data['comparison.json']['final']),
        run=evidence['run'], policy_sha256=evidence['artifact_sha256']['policy.pt']), indent=2))


if __name__ == '__main__':
    main()
