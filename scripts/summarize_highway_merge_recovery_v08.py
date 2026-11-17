"""Save compact v08 completion evidence from retained files; no simulation."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import torch
from experiments import run_highway_merge_recovery_v08 as runner


def read(path):
    return json.loads(path.read_text())


def scalars(value):
    return {k: v for k, v in value.items() if not isinstance(v, (dict, list))}


def cases(result):
    rows = []
    for report, gate in zip(result['reports'], result['summary']['case_gates']):
        matches = [q['speed_match_fraction'] for q in report.get('quality_merges', {}).values()]
        proof = report.get('cutin_fixture', {})
        rows.append(dict(case_id=report['case_id'], stress=report['cutin_stress'], passed=gate['passed'],
            merge_match=sum(matches)/len(matches) if matches else None,
            overrides=report['intervention_fraction'],
            realized_peak_jerk=max(c['peak_abs_jerk_m_s3'] for c in gate['per_car'].values()),
            raw_peak_jerk=max(c['peak_raw_requested_jerk_m_s3'] for c in report['v05_metrics']['per_car'].values()),
            command_slew_peak=max(c['peak_command_slew'] for c in report['v08_reward_audit']['per_car'].values()),
            moving_at_handoff=proof.get('moving_at_handoff', False),
            completed_into_target=proof.get('completed_into_target', False),
            handoff_headway_s=proof.get('handoff_headway_s'),
            collisions=report['collisions'], teleports=report['teleports'], failures=gate['failures']))
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    args = parser.parse_args()
    run = args.run.resolve()
    if not run.is_relative_to(ROOT / 'outputs'):
        raise ValueError('Use a retained project run')
    cfg = read(run / 'resolved-config.json')
    result = read(run / 'summary.json')
    checkpoint = torch.load(run / 'last.pt', map_location='cpu', weights_only=False)
    state = checkpoint['state']
    runner.install()
    model, _, _ = runner.base.build_model(cfg, checkpoint=str(run / 'last.pt'), evaluation=True,
        sources=runner.base.hashes(), fingerprint=checkpoint['config_sha256'])
    exported = torch.jit.load(str(run / 'policy.pt'), map_location='cpu')
    probe = torch.zeros(2, 117)
    probe[:, 30:33] = 1.
    with torch.no_grad():
        if not torch.equal(exported(probe), model.act(probe, deterministic=True)[0]):
            raise ValueError('Final exported actor differs from the final checkpoint')
    paths = [run / 'initial-validation.json', *sorted(run.glob('periodic-validation-*.json')),
             run / 'final-validation.json']
    evaluations = {}
    for path in paths:
        value = read(path)
        evaluations[path.stem] = dict(summary=scalars(value['summary']),
            nominal=scalars(value['summary']['nominal']), stress=scalars(value['summary']['stress']),
            cases=cases(value))
    coverage = {group: dict(ppo=0, critic=0) for group in cfg['curriculum_mix']}
    for row in cfg['training_rows']:
        c = state['coverage'].get(row['case']['id'], {})
        coverage[row['curriculum_group']]['ppo'] += c.get('ppo_decisions', 0)
        coverage[row['curriculum_group']]['critic'] += c.get('critic_decisions', 0)
    for field in ('ppo', 'critic'):
        total = sum(v[field] for v in coverage.values())
        for value in coverage.values():
            value[field + '_fraction'] = value[field] / max(total, 1)
    events = dict(stress_episodes=0, moving_at_handoff=0, completed_into_target=0, collisions=0, teleports=0)
    curve = []
    actor_batches = 0
    for path in sorted((run / 'updates').glob('*.json')):
        value = read(path)
        report, opt = value['report'], value['optimization']
        events['stress_episodes'] += int(report.get('cutin_stress', False))
        for key in ('moving_at_handoff', 'completed_into_target'):
            events[key] += int(report.get('cutin_fixture', {}).get(key, False))
        for key in ('collisions', 'teleports'):
            events[key] += report[key]
        actor_batches += int(not value['critic_only'] and opt.get('optimizer_steps', 0) > 0)
        curve.append(dict(episode=path.stem, case_id=report['case_id'], critic_only=value['critic_only'],
            optimizer_steps=opt.get('optimizer_steps', 0), deferred=opt.get('deferred', False),
            approximate_kl=opt.get('approximate_kl'), overrides=report['intervention_fraction']))
    initial = evaluations['initial-validation']['summary']['merge_speed_match']
    final = evaluations['final-validation']['summary']['merge_speed_match']
    evidence = dict(status='completed_experimental_not_promoted', promoted=False,
        run=run.relative_to(ROOT).as_posix(), source_commit=read(run / 'manifest.json')['git_commit'],
        elapsed_seconds=result['elapsed_seconds'], training=scalars(state), actor_update_batches=actor_batches,
        training_events=events, curriculum_actual=coverage, evaluations=evaluations,
        final_below_matched_merge_baseline=final < initial,
        actor_export_probe_matches_checkpoint=True,
        artifacts={p.name: dict(path=p.relative_to(ROOT).as_posix(),
            sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in
            [run / 'last.pt', run / 'policy.pt', run / 'best.pt', run / 'best-policy.pt',
             run / 'resolved-config.json', run / 'manifest.json', *paths]},
        limitations=['No held-out or 3D evaluation; no policy promotion.',
            'Cut-ins are already moving at handoff; this measures recovery, not anticipation.',
            'Nominal and stress gates remain failed; emergency jerk peaks are not hidden.',
            'Periodic merge checks passed, but final validation regressed after the last periodic check.',
            'best-policy.pt remains the initial actor under the safety-first ranking.',
            'Early validation improvements do not imply a retained improved checkpoint.',
            'Historical raw-request jerk includes actuator lag; command-slew is reported separately.',
            'The final two buffered episodes were charged but did not form a four-episode PPO batch.'])
    out = ROOT / 'documentation/evidence/highway-merge-recovery-2026-11-17'
    out.mkdir(parents=True, exist_ok=True)
    (out / 'result.json').write_text(json.dumps(evidence, indent=2, allow_nan=False) + '\n')
    with (out / 'cases.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=['evaluation'] + list(cases(read(paths[0]))[0]))
        writer.writeheader()
        for label, value in evaluations.items():
            for row in value['cases']:
                writer.writerow(dict(evaluation=label, **row))
    with (out / 'learning-curve.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(curve[0]))
        writer.writeheader()
        writer.writerows(curve)
    print(json.dumps(dict(training=evidence['training'], actor_batches=actor_batches,
        events=events, coverage=coverage, initial_match=initial, final_match=final), indent=2))


if __name__ == '__main__':
    main()
