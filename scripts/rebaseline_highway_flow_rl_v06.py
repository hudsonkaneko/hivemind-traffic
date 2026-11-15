"""V06 Phase 0c: frozen-v04 baseline under the ORIGINAL gate rules (no road-end exemption).

Offline only: applies the unchanged `development_gates` to saved per-episode
reports of the frozen v04 actor (startup mode, Krauss controlled vehicles). No
simulation and no ledger charge. The development set swaps the historical
ramp120 merge case for the identical row (same demand, seed, controlled car)
already evaluated on its ramp-started network. Each merge result is shown beside
that car's physical ceiling from the Phase 0a feasibility audit (diagnostic only).
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from experiments.run_highway_flow_rl_v05 import development_gates  # noqa: E402

SWAP_FROM = 'v02-development-train-2-lane-ramp120-merge'
SWAP_TO = 'ramp-eval-train-2-lane-ramp120-ramp-started'
ROW_KEYS = ('controlled_count', 'total_vehicles', 'seed', 'speed_limit_m_s', 'communication', 'packet_loss',
            'difficulty', 'traffic_mode', 'horizon_steps')


def load_run(run):
    run = Path(run)
    config = json.loads((run / 'resolved-config.json').read_text(encoding='utf-8'))
    reports = {}
    for path in sorted((run / 'evaluation').glob('[0-9][0-9].json')):
        report = json.loads(path.read_text(encoding='utf-8'))
        reports[report['case_id']] = dict(report=report, path=path,
                                          sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    rows = {r['case']['id']: r for r in config['validation_rows']}
    return config, reports, rows


def summarize(case_id, entry, ceilings):
    report = entry['report']
    gates = development_gates(report)
    cars = gates['per_car']
    merge = report.get('quality_merges') or {}
    return dict(case_id=case_id, source=str(entry['path']).replace('\\', '/'), report_sha256=entry['sha256'],
                passed=gates['passed'], failures=gates['failures'],
                failing_cars={a: [k for k, v in c['checks'].items() if not v] for a, c in cars.items() if not c['passed']},
                cars=len(cars), pooled_intervention_fraction=report.get('intervention_fraction'),
                collisions=report.get('collisions'), teleports=report.get('teleports'),
                merge_quality=merge, physical_ceiling=ceilings.get(case_id))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dev-run', required=True)
    parser.add_argument('--ramp-run', required=True)
    parser.add_argument('--feasibility', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    for run in (args.dev_run, args.ramp_run):
        config = json.loads((Path(run) / 'resolved-config.json').read_text(encoding='utf-8'))
        assert config['controlled_car_follow_model'] == 'Krauss' and config['variant'] == 'startup', run
    _, dev, dev_rows = load_run(args.dev_run)
    _, ramp, ramp_rows = load_run(args.ramp_run)
    a, b = dev_rows[SWAP_FROM], ramp_rows[SWAP_TO]
    assert all(a[k] == b[k] for k in ROW_KEYS), 'swap rows differ in demand'
    assert {k: v for k, v in a['case'].items() if k not in ('id', 'network', 'network_sha256', 'geometry_manifest',
            'geometry_manifest_sha256', 'network_layout')} == \
           {k: v for k, v in b['case'].items() if k not in ('id', 'network', 'network_sha256', 'network_layout')}
    feasibility = json.loads(Path(args.feasibility).read_text(encoding='utf-8'))
    ceilings = {}
    for car in feasibility['cars']:
        if car['run'] in (Path(args.dev_run).name, Path(args.ramp_run).name):
            ceilings.setdefault(car['case_id'], {})[car['agent']] = dict(
                observed=round(car['observed_speed_match'], 3),
                ceiling_vehicle_limits=round(car['ceilings']['gate']['best_fraction'], 3),
                ceiling_comfort_limits=round(car['ceilings']['comfort']['best_fraction'], 3))
    development = [summarize(SWAP_TO if cid == SWAP_FROM else cid, ramp[SWAP_TO] if cid == SWAP_FROM else entry, ceilings)
                   for cid, entry in dev.items()]
    ramp_cases = [summarize(cid, entry, ceilings) for cid, entry in ramp.items()]
    out = dict(schema='highway_flow_rl_v06_rebaseline', rules='original development_gates, no road-end exemption',
               actor='v04 long-round-03/last.pt, v05 startup mode, Krauss controlled vehicles',
               swap=dict(replaced=SWAP_FROM, by=SWAP_TO, same_row_keys=list(ROW_KEYS)),
               development=development, ramp_started=ramp_cases,
               totals={name: dict(cases=len(rows), passed=sum(r['passed'] for r in rows),
                                  cars=sum(r['cars'] for r in rows),
                                  failing_cars=sum(len(r['failing_cars']) for r in rows))
                       for name, rows in (('development', development), ('ramp_started', ramp_cases))})
    Path(args.output).write_text(json.dumps(out, indent=1), encoding='utf-8')
    for name in ('development', 'ramp_started'):
        print(name, out['totals'][name])
        for r in out[name]:
            print(' ', r['case_id'], 'PASS' if r['passed'] else 'FAIL', r['failures'], r['failing_cars'],
                  'ceiling', r['physical_ceiling'])


if __name__ == '__main__':
    main()
