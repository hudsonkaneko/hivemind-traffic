"""v15 Phase 2: the rule expert (Level 1 with the rule Level 2) on the 70 development rows.

Evaluation only, SUMO only (libsumo), IDM background only, charged. The held-out seeds are never built.

User decision, November 25, 2026 (relayed by the main Claude session, recorded in the coordination
master): "Yes, approve 5 runs":
- at most 5 full runs of the rule expert on the 70 development rows and at most 725,000 decisions in
  all (about 145,000 per run); if a resolved estimate is higher, stop and ask;
- clock time charged against the study total; every run on benchmark rows is charged, partial runs
  included; 1-2 row code checks stay uncharged engineering checks, as in Phase 0;
- as in Phase 1: at most 16 workers at below-normal priority, nothing heavy alongside, the shared
  lock, and the resolved configuration and expected charge in the master before each charged run;
- stop if the next run won't fit, the lock is held, a canary or parity check mismatches, tests fail,
  or all 5 runs are used. Phase 3 has no budget, so Phase 2 ends after its runs either way.

usage (from the checkout root, with .venv-traffic's Python):
  python experiments/v15_phase2.py resolve N [--previous OUT]   write configs/evaluation/v15-phase2-run-0N.json
  python experiments/v15_phase2.py run N OUT                     check the pinned configuration, take the lock, run
  python experiments/v15_phase2.py report OUT                    rebuild OUT/report.json and OUT/report.md

A run is 2 batches: development seed 1 (14 expert tasks plus one round-16 parity task), then seeds
2-5 (56 tasks), each with a canary on another worker. Before each batch the ledger session refuses
one whose expected decisions (scaled by the ratio seen so far, plus 5%) would pass what is left of
the phase's 725,000. The parity task is the round-16 policy on one development row; its report and
trace hashes must equal Phase 1's record of that row, or the run stops (the pool changed since).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from experiments.v15_phase1 import _file_sha, _json_sha, _read_jsonl, load_policies  # noqa: E402

SCHEMA = 'highway_merge_v15_phase2_v01'
CONFIG_DIR = ROOT / 'configs/evaluation'
ROUND_INDEX = 24
ROW_SET = 'v15-development'
BACKEND, BACKGROUND = 'libsumo', 'idm'
WORKERS, PRIORITY = 16, 'below_normal'       # the user's CPU limit (see experiments/v15_phase1.py)
ALLOWANCE = 725_000                          # "at most 725,000 decisions" for Phase 2 in all
MAX_RUNS = 5                                 # "at most 5 full runs"
MARGIN = 0.05
RESERVE_SECONDS = 600.
PHASE1_RESULTS = 'outputs/highway-merge-v15/phase-1/run-01/results.jsonl'
ROUND16 = 'policy:f0b11fa577b33763'
FOLLOWER = 'rule:plan-follower-v1'
ONE_CAR_LA = ('v12-val-la-nb-pasadena-model-merge-focused-1of16-medium', 'v12-val-la-sb-adams-model-merge-focused-1of16-medium')
AUTHORIZATION = ('User decision of November 25, 2026, relayed by the main Claude session: "Yes, approve 5 runs" '
                 '(at most 5 full runs of the v15 rule expert on the 70 development rows, at most 725,000 decisions, '
                 'clock time charged; settings as Phase 1). Evaluation only; no held-out rows.')


def config_path(run):
    return CONFIG_DIR / f'v15-phase2-run-{int(run):02d}.json'


def _phase1(rows):
    """Phase 1's first-attempt records by (driver, row) (not in Git; the numbers are copied into the config)."""
    records = {}
    for record in _read_jsonl(ROOT / PHASE1_RESULTS):
        if record['attempt'] == 'first' and record['fields']['row_set'] == ROW_SET:
            records[(record['fields']['driver'], record['fields']['row'])] = record
    if not all((ROUND16, r) in records for r in range(len(rows))):
        raise ValueError('Phase 1 has no round-16 record for every development row')
    return records


def _ledger_phase2():
    from experiments import v15_ledger as L
    ledger = json.loads((L.study_directory() / 'cost-ledger.json').read_text())
    return L.totals(ledger)['v15'].get('2', dict(segments=0, decisions=0, seconds=0., evaluation_episodes=0))


def resolve(run, previous=None):
    """Run N's resolved configuration and expected charge (no simulation)."""
    from traffic import highway_flow_rl_v14_contract as contract
    from traffic import v15_benchmark, v15_rule_expert as expert
    run = int(run)
    if not 1 <= run <= MAX_RUNS:
        raise ValueError(f'Phase 2 has at most {MAX_RUNS} runs')
    cfg = contract.build_config(ROUND_INDEX)
    rows = v15_benchmark.development_rows(cfg)
    phase1 = _phase1(rows)
    round16 = {r: phase1[(ROUND16, r)] for r in range(len(rows))}
    if previous:                 # the expert's own counts from an earlier Phase 2 run
        earlier = {rec['fields']['row']: rec for rec in _read_jsonl(Path(previous) / 'results.jsonl')
                   if rec['attempt'] == 'first' and rec['fields']['driver'].startswith('rule:expert')}
        if set(earlier) != set(range(len(rows))):
            raise ValueError(f'{previous} does not hold an expert record for every development row')
        per_row = {r: earlier[r]['decisions'] for r in earlier}
        ratio, source = 1.0, dict(kind='previous expert run', path=str(Path(previous)), decisions=sum(per_row.values()))
    else:                        # the round-16 policy's Phase 1 counts, scaled by the follower's measured ratio
        per_row = {r: round16[r]['decisions'] for r in round16}
        shared = [r for r in round16 if (FOLLOWER, r) in phase1]
        ratio = max(1., sum(phase1[(FOLLOWER, r)]['decisions'] for r in shared) / sum(round16[r]['decisions'] for r in shared))
        source = dict(kind='Phase 1 round-16 counts x the plan follower\'s ratio on its 60 rows', ratio=round(ratio, 4),
                      path=PHASE1_RESULTS, round16_decisions=sum(per_row.values()))
    estimate = {r: per_row[r] * ratio for r in per_row}
    horizon = {r: rows[r]['horizon_steps'] for r in range(len(rows))}
    shortest = min(horizon.values())
    parity_row = min((r for r in round16 if horizon[r] == shortest), key=lambda r: (round16[r]['decisions'], r))
    seeds = sorted({row['seed'] for row in rows})
    groups = [[seeds[0]], seeds[1:]]
    batches = []
    for index, group in enumerate(groups):
        tasks = [dict(driver='expert', row=r) for r in range(len(rows)) if rows[r]['seed'] in group]
        canary = min((t for t in tasks if horizon[t['row']] == shortest), key=lambda t: (estimate[t['row']], t['row']))
        tasks.remove(canary)
        tasks.insert(0, canary)
        expected = sum(estimate[t['row']] for t in tasks) + estimate[canary['row']]
        if index == 0:
            tasks.append(dict(driver='round-16', row=parity_row))
            expected += round16[parity_row]['decisions']
        batches.append(dict(seeds=group, tasks=tasks, expected_decisions=round(expected),
                            canary=dict(driver='expert', row=canary['row'], expected=round(estimate[canary['row']]))))
    expected = sum(b['expected_decisions'] for b in batches)
    if expected > ALLOWANCE:
        raise ValueError(f'Run {run} expects {expected:,} decisions, above the phase\'s {ALLOWANCE:,}: ask first')
    episodes = sum(len(b['tasks']) + 1 for b in batches)
    budget = cfg['budget']
    return dict(
        schema=SCHEMA, phase='2', run=run, round_index=ROUND_INDEX, backend=BACKEND, workers=WORKERS, priority=PRIORITY,
        backgrounds=[BACKGROUND], mode='validate', row_set=ROW_SET,
        rows=[dict(row=i, case=row['case']['id'], seed=row['seed'], horizon_steps=row['horizon_steps'])
              for i, row in enumerate(rows)],
        benchmark=v15_benchmark.manifest(cfg),
        expert=dict(id=expert.DRIVER, settings=expert.SETTINGS,
                    source=dict(path='traffic/v15_rule_expert.py', sha256=_file_sha(ROOT / 'traffic/v15_rule_expert.py'))),
        baseline=dict(id=ROUND16, source=PHASE1_RESULTS, rows='all 70',
                      passed={str(r): bool(round16[r]['passed']) for r in round16},
                      decisions=sum(round16[r]['decisions'] for r in round16)),
        parity=dict(driver=ROUND16, row=parity_row, report_sha=round16[parity_row]['report_sha'],
                    trace_sha=round16[parity_row]['trace_sha'], source=PHASE1_RESULTS),
        pass_definition='experiments/run_highway_flow_rl_v05.ruled_gates(report)["passed"] (v14 round 24 gates), '
                        'as recorded by experiments/v15_pool.py',
        targets=dict(collisions=0, one_car_la_cut_in_emergencies=0, free_road_stops=0,
                     pass_rate=f'above the round-16 policy\'s {sum(round16[r]["passed"] for r in round16)} of 70 on the same rows',
                     one_car_la_cases=list(ONE_CAR_LA)),
        batches=batches,
        charge=dict(expected_decisions=expected, phase_allowance=ALLOWANCE, max_runs=MAX_RUNS, guard_margin=MARGIN,
                    estimate_source=source, expected_episodes=episodes, expected_clock_seconds=150.,
                    reserve_seconds=RESERVE_SECONDS, total_wall_seconds=budget['total_wall_seconds'],
                    clock_estimate='2 batches (15 and 56 tasks plus a canary each) at 16 workers, below-normal priority; '
                                   'Phase 1 ran a 41-task batch in about 35 s, set by the 2,000-step cruise rows'),
        authorization=AUTHORIZATION, held_out='not built, not run')


class ParityMismatch(RuntimeError):
    """The round-16 parity task differs from Phase 1's record of the same row."""


def run(run_index, out):
    from experiments import v15_ledger as L
    from experiments import v15_pool as P
    out = Path(out).resolve()
    config = json.loads(json.dumps(resolve(run_index, _previous_of(run_index))))
    pinned = json.loads(config_path(run_index).read_text())
    if config != pinned:
        raise ValueError(f'The resolved configuration differs from {config_path(run_index)}; '
                         're-resolve and re-record it first')
    sha = _json_sha(pinned)
    charge = config['charge']
    if config['workers'] > 16:
        raise ValueError('At most 16 workers (the user\'s CPU limit)')
    done = _ledger_phase2()
    if done['segments'] >= MAX_RUNS:
        raise ValueError(f'All {MAX_RUNS} Phase 2 runs are used')
    if done['segments'] != run_index - 1:
        raise ValueError(f'The ledger holds {done["segments"]} Phase 2 runs; this is run {run_index}')
    learner_priority = P.set_priority(config['priority'])
    if learner_priority != config['priority']:
        raise RuntimeError(f'The learner runs at {learner_priority} priority, not {config["priority"]}')
    out.mkdir(parents=True, exist_ok=True)
    (out / 'resolved-config.json').write_text(json.dumps(config, indent=1, sort_keys=True))
    log = out / 'phase2-log.jsonl'
    ids = {'expert': config['expert']['id'], 'round-16': config['parity']['driver']}
    with L.EvaluationSession(phase='2', output=out, config_sha256=sha, expected_decisions=charge['expected_decisions'],
                             decision_allowance=charge['phase_allowance'],
                             expected_clock_seconds=charge['expected_clock_seconds'],
                             reserve_seconds=charge['reserve_seconds'], total_wall_seconds=charge['total_wall_seconds'],
                             intended_episodes=charge['expected_episodes'], workers=config['workers'],
                             priority=config['priority'], authorization=config['authorization'],
                             round_index=config['round_index']) as session:
        session.segment['phase2_run'] = run_index
        with P.WorkerPool(config['workers'], out, backend=config['backend'], round_index=config['round_index'],
                          priority=config['priority']) as pool:
            session.segment['worker_priorities'] = sorted(set(pool.worker_priorities.values()))
            pool.charged_by = f'cost-ledger {L.COMMAND} phase 2 run {run_index}, started_at_unix {session.segment["started_at_unix"]}'
            round16, _ = load_policies(pool.cfg)
            if pool.publish(round16.state_dict()) != ids['round-16']:
                raise ValueError('The round-16 policy published under a different id')
            expected_so_far = charged_so_far = 0
            for index, batch in enumerate(config['batches']):
                ratio = max(1., charged_so_far / expected_so_far) if expected_so_far else 1.
                projected = batch['expected_decisions'] * ratio * (1 + charge['guard_margin'])
                session.check_next(projected)
                tasks = [P.Task(ROW_SET, t['row'], ids[t['driver']], background=BACKGROUND,
                                run=f'phase2-run{run_index}:{t["driver"]}') for t in batch['tasks']]
                try:
                    result = pool.run(tasks)
                except P.CanaryMismatch as error:
                    if error.result is not None:
                        session.add_batch(error.result)
                    else:
                        session.charge_estimate(projected, f'batch {index}: {error}')
                    raise
                except BaseException as error:
                    session.charge_estimate(projected, f'batch {index} failed before reporting: {type(error).__name__}')
                    raise
                session.add_batch(result)
                expected_so_far += batch['expected_decisions']
                charged_so_far += result.charged_decisions
                parity = None
                for task, record in zip(batch['tasks'], result.records):
                    if task['driver'] == 'round-16':
                        parity = (record['report_sha'], record['trace_sha']) == (config['parity']['report_sha'],
                                                                                 config['parity']['trace_sha'])
                entry = dict(batch=index, seeds=batch['seeds'], tasks=len(tasks), simulated=result.simulated_tasks,
                             reused=result.reused_tasks, clock_seconds=result.clock_seconds,
                             worker_seconds=result.worker_seconds, simulated_seconds=result.simulated_seconds,
                             decisions=result.decisions, canary_decisions=result.canary_decisions,
                             expected_decisions=batch['expected_decisions'], canary=result.canary, parity_matched=parity,
                             elapsed_seconds=session.elapsed(), charged_decisions_so_far=session.segment['decisions'])
                with open(log, 'a', encoding='utf-8') as handle:
                    handle.write(json.dumps(entry) + '\n')
                print(json.dumps({k: entry[k] for k in ('batch', 'seeds', 'simulated', 'clock_seconds', 'decisions',
                                                        'expected_decisions', 'parity_matched', 'elapsed_seconds')}),
                      flush=True)
                if result.simulated_tasks and (result.canary is None or not result.canary['matched']):
                    raise P.CanaryMismatch(f'Batch {index}: canary {result.canary}')
                if parity is False:
                    raise ParityMismatch(f'Batch {index}: the round-16 parity row differs from Phase 1')
    segment = session.segment
    summary = {k: segment[k] for k in ('status', 'charged_seconds', 'clock_seconds', 'decisions', 'canary_decisions',
                                       'evaluation_episodes', 'worker_seconds', 'simulated_seconds', 'workers', 'batches',
                                       'decision_allowance', 'expected_decisions', 'started_at_unix', 'config_sha256')}
    summary['ledger'] = str(session.path)
    (out / 'charge.json').write_text(json.dumps(summary, indent=1))
    return report(out)


PREVIOUS = {}      # run -> the earlier run's OUT whose counts size its estimate (set by the CLI; recorded in the config)


def _previous_of(run_index):
    return PREVIOUS.get(int(run_index))


def _car_checks(out, record):
    """Each failed per-car check of one episode (from its report.json and the v14 round-24 gates)."""
    from experiments.run_highway_flow_rl_v05 import ruled_gates
    path = Path(out) / 'episodes' / record['key'][:16] / record['attempt'] / 'report.json'
    gate = ruled_gates(json.loads(path.read_text()))
    failed = {}
    for car in gate['per_car'].values():
        for check, ok in car['checks'].items():
            if not ok:
                failed[check] = failed.get(check, 0) + 1
    return failed


def report(out):
    """The expert's pass rate (Wilson), per case, paired with the round-16 policy, events and targets."""
    from experiments import v15_pool as P
    from traffic import v15_stats as S
    out = Path(out).resolve()
    config = json.loads((out / 'resolved-config.json').read_text())
    rows = config['rows']
    records = {}
    for record in P.read_results(out / 'results.jsonl'):
        f = record['fields']
        if f['row_set'] == ROW_SET and f['background'] == BACKGROUND and f['driver'] == config['expert']['id']:
            records.setdefault(f['row'], record)
    done = sorted(records)
    passes = [records[r]['passed'] for r in done]
    cases, failures, events, checks, by_case_checks = {}, {}, {}, {}, {}
    for r in done:
        record = records[r]
        case = rows[r]['case']
        cases.setdefault(case, []).append(record['passed'])
        for failure in record['failures']:
            failures[failure] = failures.get(failure, 0) + 1
        for event, count in record['events'].items():
            slot = events.setdefault(event, dict(rows=0, count=0))
            slot['rows'] += int(count > 0)
            slot['count'] += int(count)
        for check, count in _car_checks(out, record).items():
            checks[check] = checks.get(check, 0) + count
            by_case_checks.setdefault(case, {})[check] = by_case_checks.setdefault(case, {}).get(check, 0) + count
    baseline = config['baseline']['passed']
    pair = S.paired(passes, [baseline[str(r)] for r in done])
    one_car = [r for r in done if rows[r]['case'] in config['targets']['one_car_la_cases']]
    totals = lambda key, subset: sum(int(records[r]['events'].get(key, 0)) for r in subset)
    targets = dict(collisions=totals('collisions', done), free_road_stops=totals('free_road_stops', done),
                   one_car_la_cut_ins=totals('cut_ins', one_car), one_car_la_emergencies=totals('emergencies', one_car),
                   one_car_la_controlled_emergencies=totals('controlled_emergencies', one_car),
                   passed=sum(passes), baseline_passed=sum(baseline[str(r)] for r in done))
    targets['met'] = dict(collisions=targets['collisions'] == 0, free_road_stops=targets['free_road_stops'] == 0,
                          one_car_la_cut_in_emergencies=targets['one_car_la_cut_ins'] == 0
                          and targets['one_car_la_emergencies'] == 0,
                          pass_rate=targets['passed'] > targets['baseline_passed'])
    result = dict(schema=SCHEMA + '_report', run=config['run'], expert=config['expert']['id'],
                  simulator='SUMO (libsumo); no physics', background=BACKGROUND,
                  rows='v15 development rows; held-out rows not built or run', planned_rows=len(rows), run_rows=len(done),
                  pooled=S.rate(passes), cases={case: S.rate(p) for case, p in cases.items()},
                  baseline_cases={case: S.rate([baseline[str(r)] for r in done if rows[r]['case'] == case]) for case in cases},
                  paired=dict(first='expert', second='round-16', **pair),
                  decisions=sum(records[r]['decisions'] for r in done),
                  gate_failures=dict(sorted(failures.items(), key=lambda kv: -kv[1])), car_check_failures=checks,
                  car_check_failures_by_case=by_case_checks, events=events, targets=targets,
                  failing_rows=[dict(row=r, case=rows[r]['case'], seed=rows[r]['seed'], failures=records[r]['failures'])
                                for r in done if not records[r]['passed']],
                  charge=json.loads((out / 'charge.json').read_text()) if (out / 'charge.json').is_file() else None,
                  caveat='Rows of one case share its network and traffic setting and differ in seed; pooled '
                         'intervals treat the rows as independent.')
    (out / 'report.json').write_text(json.dumps(result, indent=1))
    (out / 'report.md').write_text(_markdown(result))
    return result


def _pct(x):
    return '-' if x is None else f'{100 * x:.0f}%'


def _markdown(result):
    p, q = result['pooled'], result['paired']
    lines = [f'# v15 Phase 2 run {result["run"]} report ({result["expert"]})', '',
             f'{result["simulator"]}; background {result["background"]}; {result["rows"]}.', '',
             f'- Expert: {p["passed"]} of {p["rows"]} rows, {_pct(p["rate"])} ({_pct(p["low"])}-{_pct(p["high"])}).',
             f'- Paired with the round-16 policy (Phase 1): both {q["both"]}, expert only {q["first_only"]}, '
             f'round-16 only {q["second_only"]}, neither {q["neither"]}; {100 * q["difference"]:+.1f} pts '
             f'({100 * q["low"]:+.1f} to {100 * q["high"]:+.1f}), McNemar p = {q["mcnemar_p"]:.3f}.', '',
             '| Case | Expert | Round-16 |', '|---|---|---|']
    for case, c in result['cases'].items():
        b = result['baseline_cases'][case]
        lines.append(f'| {case} | {c["passed"]}/{c["rows"]} | {b["passed"]}/{b["rows"]} |')
    lines += ['', '| Target | Value | Met |', '|---|---|---|']
    t = result['targets']
    for key, value in (('collisions', t['collisions']), ('free_road_stops', t['free_road_stops']),
                       ('one_car_la_cut_in_emergencies', f'{t["one_car_la_cut_ins"]} cut-ins, {t["one_car_la_emergencies"]} emergencies'),
                       ('pass_rate', f'{t["passed"]} vs {t["baseline_passed"]}')):
        lines.append(f'| {key} | {value} | {"yes" if t["met"][key] else "no"} |')
    lines += ['', f'Gate failures: {result["gate_failures"]}', '', f'Per-car check failures: {result["car_check_failures"]}',
              '', f'Events: {result["events"]}', '', f'Caveat: {result["caveat"]}', '']
    return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('command', choices=('resolve', 'run', 'report'))
    parser.add_argument('args', nargs='*')
    parser.add_argument('--previous', help='an earlier Phase 2 run OUT whose expert counts size this run\'s estimate')
    args = parser.parse_args(argv)
    if args.command == 'resolve':
        run_index = int(args.args[0])
        config = resolve(run_index, args.previous)
        path = config_path(run_index)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(config, indent=1, sort_keys=True) + '\n')
        left = ALLOWANCE - _ledger_phase2()['decisions']
        print(json.dumps(dict(config=str(path.relative_to(ROOT)), sha256=_json_sha(config), phase_left=left,
                              **{k: config['charge'][k] for k in ('expected_decisions', 'expected_episodes',
                                                                  'expected_clock_seconds', 'reserve_seconds')})))
    elif args.command == 'run':
        run_index, out = int(args.args[0]), args.args[1]
        pinned = json.loads(config_path(run_index).read_text())
        source = pinned['charge']['estimate_source']
        if source.get('kind') == 'previous expert run':
            PREVIOUS[run_index] = source['path']
        result = run(run_index, out)
        print(json.dumps(dict(pooled=result['pooled'], paired=result['paired'], targets=result['targets'])))
    else:
        report(args.args[0])
    return 0


if __name__ == '__main__':
    sys.exit(main())
