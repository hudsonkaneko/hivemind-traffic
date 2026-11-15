"""Reduce retained v07 results and export the experimental actor; no simulation."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import torch
from experiments import run_highway_merge_cutin_v07 as runner
from policies.mappo import export_actor


def read(path):
    return json.loads(path.read_text())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def cases(result):
    rows = []
    for report, gate in zip(result['reports'], result['summary']['case_gates']):
        matches = [q['speed_match_fraction'] for q in report.get('quality_merges', {}).values()]
        raw = [c['peak_raw_requested_jerk_m_s3'] for c in report['v05_metrics']['per_car'].values()]
        realized = [c['peak_abs_jerk_m_s3'] for c in gate['per_car'].values()]
        audit = report['v07_admission']
        rows.append(dict(case_id=report['case_id'], stress=report['cutin_stress'], passed=gate['passed'],
            merge_match=sum(matches)/len(matches) if matches else None,
            overrides=report.get('intervention_fraction'), realized_peak_jerk=max(realized, default=None),
            raw_peak_jerk=max(raw, default=None), stress_started=audit['stress_started'],
            stress_completed=audit['stress_completed'], ongoing_gap_violations=audit['ongoing_gap_violations'],
            collisions=report['collisions'], teleports=report['teleports'], failures=gate['failures']))
    return rows


def summary(value):
    return {k: v for k, v in value.items() if not isinstance(v, (dict, list))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    args = parser.parse_args()
    run = args.run.resolve()
    if not run.is_relative_to(ROOT / 'outputs'):
        raise ValueError('Use a retained project run')
    runner.install()
    cfg = read(run / 'resolved-config.json')
    saved = torch.load(run / 'last.pt', map_location='cpu', weights_only=False)
    state = saved['state']
    model, _, _ = runner.base.build_model(cfg, checkpoint=str(run / 'last.pt'), evaluation=True,
        sources=runner.base.hashes(), fingerprint=saved['config_sha256'])
    actor = run / 'candidate-policy.pt'
    if not actor.exists():
        export_actor(model, actor)
    # Rerunning reduction preserves an existing actor and verifies its outputs.
    exported = torch.jit.load(str(actor), map_location='cpu')
    probe = torch.zeros(2, 117)
    probe[:, 30:33] = 1.
    with torch.no_grad():
        if not torch.equal(exported(probe), model.act(probe, deterministic=True)[0]):
            raise ValueError('Existing candidate differs from the checkpoint')
    initial = read(run / 'initial-validation.json')
    latest_path = sorted(run.glob('periodic-validation-*.json'))[-1]
    latest = read(latest_path)
    coverage = {group: dict(ppo=0, critic=0) for group in cfg['curriculum_mix']}
    for row in cfg['training_rows']:
        c = state['coverage'].get(row['case']['id'], {})
        coverage[row['curriculum_group']]['ppo'] += c.get('ppo_decisions', 0)
        coverage[row['curriculum_group']]['critic'] += c.get('critic_decisions', 0)
    for field in ('ppo', 'critic'):
        total = sum(v[field] for v in coverage.values())
        for value in coverage.values():
            value[field + '_fraction'] = value[field] / max(total, 1)
    events = dict(stress_episodes=0, stress_started=0, stress_completed=0, collisions=0, teleports=0)
    for path in (run / 'updates').glob('*.json'):
        r = read(path)['report']
        events['stress_episodes'] += int(r.get('cutin_stress', False))
        for key in ('stress_started', 'stress_completed'):
            events[key] += r['v07_admission'][key]
        for key in ('collisions', 'teleports'):
            events[key] += r[key]
    evidence = dict(status='stopped_by_declared_merge_regression_rule', promoted=False,
        run=run.relative_to(ROOT).as_posix(), source_commit=read(run / 'manifest.json')['git_commit'],
        training={k: state.get(k) for k in ('decisions', 'critic_decisions', 'updates', 'optimizer_steps', 'critic_optimizer_steps', 'episodes')},
        training_events=events, curriculum_actual=coverage,
        initial_summary=summary(initial['summary']), latest_summary=summary(latest['summary']),
        initial_nominal=summary(initial['summary']['nominal']), latest_nominal=summary(latest['summary']['nominal']),
        initial_stress=summary(initial['summary']['stress']), latest_stress=summary(latest['summary']['stress']),
        initial_cases=cases(initial), latest_cases=cases(latest),
        artifacts={p.name: dict(path=p.relative_to(ROOT).as_posix(), sha256=digest(p)) for p in
            [run / 'last.pt', actor, run / 'best-policy.pt', run / 'initial-validation.json', latest_path,
             run / 'failure.json', run / 'resolved-config.json']},
        limitations=['No held-out or 3D evaluation.',
            'Latest stress validation had zero realized cut-ins; reduced interventions do not prove improved cut-in recovery.',
            'Original merge-quality and raw-request-jerk gates remain failed.',
            'Baseline uses the same new admission layer; improvement against earlier traffic rules is not attributed to learning.',
            'candidate-policy.pt is experimental; best-policy.pt retains the unpromoted initial actor.'])
    out = ROOT / 'documentation/evidence/highway-merge-cutin-2026-11-15'
    out.mkdir(parents=True, exist_ok=True)
    (out / 'result.json').write_text(json.dumps(evidence, indent=2, allow_nan=False) + '\n')
    with (out / 'cases.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=['evaluation'] + list(evidence['initial_cases'][0]))
        writer.writeheader()
        for label, rows in [('initial', evidence['initial_cases']), ('100k', evidence['latest_cases'])]:
            for row in rows:
                writer.writerow(dict(evaluation=label, **row))
    print(json.dumps(dict(training=evidence['training'], events=events, coverage=coverage), indent=2))


if __name__ == '__main__':
    main()
