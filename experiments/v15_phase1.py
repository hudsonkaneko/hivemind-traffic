"""v15 Phase 1: baselines on the 70 development rows. Evaluation only, SUMO only, charged.

User decisions, November 25, 2026 (relayed by the main Claude session, recorded in the coordination
master):
- plan v15 decision 6: Phase 1 approved (round-16 policy, round 21's final policy and the plan
  follower on the 70 development rows), after Phase 0 passes and once its resolved configuration and
  expected charge are in the master; clock time charged (decision 5, "use clock time");
- "Yes, all four": (1) commit Phase 0; (2) a decision allowance for Phase 1 only, sized to its
  resolved expected charge and at most 400,000, the time cap unchanged; (3) no Krauss runs in Phase 1,
  IDM background only; (4) the plan follower scored on the 60 rows without cases 0 and 4, the two
  policies on all 70, paired comparisons with the follower on the shared 60 rows;
- "start phjase one after those modifications have been made".

No training and no update. The held-out seeds are never built. Every episode is a SUMO simulation
(libsumo), not physics.

usage (from the checkout root, with .venv-traffic's Python):
  python experiments/v15_phase1.py resolve    write configs/evaluation/v15-phase1.json (no simulation, no lock)
  python experiments/v15_phase1.py run OUT    check the pinned configuration, take the shared lock and run (charged)
  python experiments/v15_phase1.py report OUT rebuild OUT/report.json and OUT/report.md from OUT/results.jsonl

The run goes seed by seed: batch k is every driver on the 14 cases at development seed k (40 tasks).
Before each batch the ledger session refuses one whose expected decisions (scaled by the ratio seen
so far, plus 5%) would pass the allowance, so a stop leaves whole seeds with every driver paired.
Development rows 0-27 are round 24's validation rows 14-41, which Phase 0 ran with the round-16 policy;
the round-16 records on them must match Phase 0's exactly, or the run stops.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

SCHEMA = 'highway_merge_v15_phase1_v01'
CONFIG = ROOT / 'configs/evaluation/v15-phase1.json'
ROUND_INDEX = 24
ROW_SET = 'v15-development'
BACKEND, BACKGROUND = 'libsumo', 'idm'
# User direction, November 25, 2026, relayed by the main Claude session: "don't use all cpu cores i need some to
# do other tasks": at most 16 workers of 32 hardware threads, workers and learner at below-normal priority.
# Phase 0 measured 16 workers as fast as 24 on the 42 validation rows (33.7 s).
WORKERS, PRIORITY = 16, 'below_normal'
ALLOWANCE_CAP = 400_000                  # "at most 400,000"
MARGIN = 0.05                            # the guard's margin on each batch's expected decisions
RESERVE_SECONDS = 900.                   # this run's clock-time reservation
ROUND21_FINAL = 'outputs/highway-flow-rl/v14/round-21/last.pt'
# Phase 0's records of the round-16 policy on round 24's 42 validation rows (pool, 24 workers; they match
# serial TraCI and the trainer's own record). Not in Git: their hashes are copied into the configuration.
PHASE0_RECORDS = 'outputs/test-runs/v15-p0/pool-2/w24/results.jsonl'
PHASE0_ROUND16 = 'policy:f0b11fa577b33763'
# Sources for the decision estimate (not in Git; the numbers are copied into the configuration).
ROUND21_VALIDATIONS = ('outputs/highway-flow-rl/v14/round-21/initial-validation.json',
                       'outputs/highway-flow-rl/v14/round-21/final-validation.json')
FOLLOWER_CHECKS = 'outputs/test-runs/v14-follow-gate-02-{mode}-case{case:02d}/report.json'
AUTHORIZATION = ('User decisions of November 25, 2026, relayed by the main Claude session: plan v15 decision 6 '
                 '(Phase 1 approved, about 210 evaluation episodes), "use clock time", then "Yes, all four" '
                 '(Phase 1 decision allowance at most 400,000, time cap unchanged, IDM only, the follower on 60 rows) '
                 'and "start phjase one after those modifications have been made". Evaluation only; no held-out rows.')
DRIVERS = ('round-16', 'round-21-final', 'plan-follower')


def _json_sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()


def _file_sha(path):
    from traffic.highway_flow_contract import file_digest
    return file_digest(path)


def _read_jsonl(path):
    with open(path, encoding='utf-8') as handle:
        return [json.loads(line) for line in handle if line.strip()]


def load_policies(cfg):
    """The two policies as models: round 24's warm start (the round-16 policy) and round 21's last.pt."""
    import torch
    from experiments import run_highway_flow_rl_v12 as runner
    from experiments import run_highway_flow_rl_v14 as runner14
    runner.install = runner14.install
    runner.install()
    round16, _ = runner.load_warm_start(cfg)
    round16.eval()
    saved = torch.load(ROOT / ROUND21_FINAL, map_location='cpu', weights_only=False)
    round21 = copy.deepcopy(round16)
    round21.load_state_dict(saved['model'])
    round21.eval()
    return round16, round21


def _estimates(cfg, rows):
    """Expected decisions per development row for the round-16 policy, and the other drivers' ratios.

    - Rows 0-27: Phase 0's measured round-16 decisions (the same rows as round 24's validation rows 14-41).
    - Rows 28-69 (seeds not run yet): the case's mean over its three measured seeds (its own v14 seed and
      6952001-6952002).
    - Round 21's final policy: its final validation over its initial one (the round-16 policy), on the
      14 cases at their own seeds.
    - The plan follower: its runs over the round-16 policy's on cases 8-11 (round 23's checks, the cases'
      own seeds); the only cases it has been run on."""
    phase0 = [r for r in _read_jsonl(ROOT / PHASE0_RECORDS)
              if r['fields']['row_set'] == 'v14-validation' and r['fields']['driver'] == PHASE0_ROUND16
              and r['fields']['background'] == 'idm' and r['fields']['mode'] == 'validate']
    if len(phase0) != 42 or len({r['fields']['row'] for r in phase0}) != 42:
        raise ValueError('Expected Phase 0\'s 42 round-16 records')
    measured = {(r['fields']['case'], r['fields']['seed']): r for r in phase0}
    by_case = {}
    for (case, _), record in measured.items():
        by_case.setdefault(case, []).append(record['decisions'])
    per_row = []
    for row in rows:
        known = measured.get((row['case']['id'], row['seed']))
        per_row.append(float(known['decisions']) if known else sum(by_case[row['case']['id']]) / len(by_case[row['case']['id']]))
    initial, final = (json.loads((ROOT / p).read_text())['reports'] for p in ROUND21_VALIDATIONS)
    if [(r['case_id'], r['seed']) for r in initial] != [(r['case_id'], r['seed']) for r in final]:
        raise ValueError('Round 21\'s validations differ in rows')
    round21 = sum(r['agent_transitions'] for r in final) / sum(r['agent_transitions'] for r in initial)
    policy = follower = 0
    for case in (8, 9, 10, 11):
        policy += json.loads((ROOT / FOLLOWER_CHECKS.format(mode='policy', case=case)).read_text())['agent_transitions']
        follower += json.loads((ROOT / FOLLOWER_CHECKS.format(mode='v14', case=case)).read_text())['agent_transitions']
    ratios = {'round-16': 1., 'round-21-final': round21, 'plan-follower': follower / policy}
    sources = dict(round16=f'{PHASE0_RECORDS} (rows 0-27); case means over 3 measured seeds (rows 28-69)',
                   round21=f'{ROUND21_VALIDATIONS[1]} / {ROUND21_VALIDATIONS[0]} agent_transitions',
                   follower='v14-follow-gate-02-v14 / -policy, cases 8-11, agent_transitions',
                   follower_counts=dict(follower=follower, policy=policy))
    parity = {}
    for index, row in enumerate(rows):
        known = measured.get((row['case']['id'], row['seed']))
        if known is not None:
            parity[str(index)] = dict(report_sha=known['report_sha'], trace_sha=known['trace_sha'],
                                      phase0_row=known['fields']['row'])
    return per_row, ratios, sources, parity


def _batches(rows, scored, per_row, ratios):
    """Seed-major batches of (driver, row), the cheapest 800-step task first so it is the canary."""
    batches = []
    seeds = sorted({row['seed'] for row in rows}, key=[row['seed'] for row in rows].index)
    for seed in seeds:
        tasks = []
        for index, row in enumerate(rows):
            if row['seed'] != seed:
                continue
            for driver in DRIVERS:
                if driver != 'plan-follower' or index in scored:
                    tasks.append(dict(driver=driver, row=index, expected=per_row[index] * ratios[driver],
                                      horizon=row['horizon_steps']))
        shortest = min(t['horizon'] for t in tasks)
        canary = min((t for t in tasks if t['horizon'] == shortest), key=lambda t: t['expected'])
        tasks.remove(canary)
        tasks.insert(0, canary)
        batches.append(dict(seed=seed, tasks=[dict(driver=t['driver'], row=t['row']) for t in tasks],
                            expected_decisions=round(sum(t['expected'] for t in tasks) + canary['expected']),
                            canary=dict(driver=canary['driver'], row=canary['row'], expected=round(canary['expected']))))
    return batches


def resolve():
    """Phase 1's resolved configuration and expected charge (no simulation)."""
    from experiments import v15_pool as P
    from traffic import highway_flow_rl_v14_contract as contract
    from traffic import v15_benchmark, v15_plan_follower as follower
    cfg = contract.build_config(ROUND_INDEX)
    rows = v15_benchmark.development_rows(cfg)
    scored = follower.scored_rows(rows)
    round16, round21 = load_policies(cfg)
    ids = {'round-16': f'policy:{P.state_hash(round16.state_dict())}',
           'round-21-final': f'policy:{P.state_hash(round21.state_dict())}', 'plan-follower': follower.DRIVER}
    if ids['round-16'] != PHASE0_ROUND16:
        raise ValueError(f'The round-16 policy hashes to {ids["round-16"]}, not Phase 0\'s {PHASE0_ROUND16}')
    per_row, ratios, sources, parity = _estimates(cfg, rows)
    batches = _batches(rows, set(scored), per_row, ratios)
    expected = sum(b['expected_decisions'] for b in batches)
    if expected > ALLOWANCE_CAP:
        raise ValueError(f'Phase 1 expects {expected:,} decisions, above the {ALLOWANCE_CAP:,} the user approved: ask first')
    allowance = min(ALLOWANCE_CAP, int(math.ceil(expected * (1 + MARGIN) / 1000.) * 1000))
    budget = cfg['budget']
    episodes = sum(len(b['tasks']) + 1 for b in batches)
    return dict(
        schema=SCHEMA, phase='1', round_index=ROUND_INDEX, backend=BACKEND, workers=WORKERS, priority=PRIORITY,
        backgrounds=[BACKGROUND], mode='validate', row_set=ROW_SET,
        rows=[dict(row=i, case=row['case']['id'], seed=row['seed'], horizon_steps=row['horizon_steps'])
              for i, row in enumerate(rows)],
        benchmark=v15_benchmark.manifest(cfg),
        drivers={'round-16': dict(id=ids['round-16'], source=cfg['warm_start'], rows='all 70'),
                 'round-21-final': dict(id=ids['round-21-final'], source=dict(path=ROUND21_FINAL,
                                        sha256=_file_sha(ROOT / ROUND21_FINAL)), rows='all 70'),
                 'plan-follower': dict(id=follower.DRIVER, settings=follower.SETTINGS,
                                       not_scored_cases=list(follower.LANE_CHANGE_CASES), rows=scored)},
        pass_definition='experiments/run_highway_flow_rl_v05.ruled_gates(report)["passed"] (v14 round 24 gates), '
                        'as recorded by experiments/v15_pool.py',
        paired=[['round-21-final', 'round-16', 'all 70 rows'], ['plan-follower', 'round-16', 'the 60 follower rows'],
                ['plan-follower', 'round-21-final', 'the 60 follower rows']],
        batches=batches,
        charge=dict(expected_decisions=expected, decision_allowance=allowance, allowance_cap=ALLOWANCE_CAP,
                    guard_margin=MARGIN, decision_ratios=ratios, estimate_sources=sources,
                    expected_episodes=episodes, expected_clock_seconds=300., reserve_seconds=RESERVE_SECONDS,
                    total_wall_seconds=budget['total_wall_seconds'],
                    clock_estimate='5 batches of 40 tasks at 16 workers, below-normal priority; Phase 0: 42 rows in '
                                   '33.7 s at 16 workers, the 2,000-step cruise rows setting each batch\'s time; '
                                   'the user\'s own load lengthens it'),
        parity=dict(rows=parity, source=PHASE0_RECORDS),
        authorization=AUTHORIZATION, held_out='not built, not run')


class ParityMismatch(RuntimeError):
    """A round-16 record differs from Phase 0's record of the same row."""


def run(out):
    from experiments import v15_ledger as L
    from experiments import v15_pool as P
    out = Path(out).resolve()
    config = json.loads(json.dumps(resolve()))
    pinned = json.loads(CONFIG.read_text())
    if config != pinned:
        raise ValueError(f'The resolved configuration differs from {CONFIG.relative_to(ROOT)}; re-resolve and re-record it first')
    sha = _json_sha(pinned)
    charge = config['charge']
    if config['workers'] > 16:
        raise ValueError('At most 16 workers (the user\'s CPU limit)')
    learner_priority = P.set_priority(config['priority'])     # before the workers start, which inherit it
    if learner_priority != config['priority']:
        raise RuntimeError(f'The learner runs at {learner_priority} priority, not {config["priority"]}')
    out.mkdir(parents=True, exist_ok=True)
    (out / 'resolved-config.json').write_text(json.dumps(config, indent=1, sort_keys=True))
    log = out / 'phase1-log.jsonl'
    ids = {name: driver['id'] for name, driver in config['drivers'].items()}
    with L.EvaluationSession(phase='1', output=out, config_sha256=sha, expected_decisions=charge['expected_decisions'],
                             decision_allowance=charge['decision_allowance'],
                             expected_clock_seconds=charge['expected_clock_seconds'],
                             reserve_seconds=charge['reserve_seconds'], total_wall_seconds=charge['total_wall_seconds'],
                             intended_episodes=charge['expected_episodes'], workers=config['workers'],
                             priority=config['priority'], authorization=config['authorization'],
                             round_index=config['round_index']) as session:
        with P.WorkerPool(config['workers'], out, backend=config['backend'], round_index=config['round_index'],
                          priority=config['priority']) as pool:
            session.segment['worker_priorities'] = sorted(set(pool.worker_priorities.values()))
            pool.charged_by = f'cost-ledger {L.COMMAND} phase 1, started_at_unix {session.segment["started_at_unix"]}'
            round16, round21 = load_policies(pool.cfg)
            published = {'round-16': pool.publish(round16.state_dict()), 'round-21-final': pool.publish(round21.state_dict())}
            for name, driver in published.items():
                if driver != ids[name]:
                    raise ValueError(f'{name} published as {driver}, not {ids[name]}')
            expected_so_far = charged_so_far = 0
            for index, batch in enumerate(config['batches']):
                ratio = max(1., charged_so_far / expected_so_far) if expected_so_far else 1.
                projected = batch['expected_decisions'] * ratio * (1 + config['charge']['guard_margin'])
                session.check_next(projected)
                tasks = [P.Task(ROW_SET, t['row'], ids[t['driver']], background=BACKGROUND, run=f'phase1:{t["driver"]}')
                         for t in batch['tasks']]
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
                differing = []
                for task, record in zip(batch['tasks'], result.records):
                    reference = config['parity']['rows'].get(str(task['row']))
                    if task['driver'] == 'round-16' and reference is not None:
                        if (record['report_sha'], record['trace_sha']) != (reference['report_sha'], reference['trace_sha']):
                            differing.append(task['row'])
                entry = dict(batch=index, seed=batch['seed'], tasks=len(tasks), simulated=result.simulated_tasks,
                             reused=result.reused_tasks, clock_seconds=result.clock_seconds,
                             worker_seconds=result.worker_seconds, simulated_seconds=result.simulated_seconds,
                             decisions=result.decisions, canary_decisions=result.canary_decisions,
                             expected_decisions=batch['expected_decisions'], canary=result.canary,
                             parity_rows=sum(1 for t in batch['tasks'] if t['driver'] == 'round-16'
                                             and str(t['row']) in config['parity']['rows']),
                             parity_differing=differing, elapsed_seconds=session.elapsed(),
                             charged_decisions_so_far=session.segment['decisions'])
                with open(log, 'a', encoding='utf-8') as handle:
                    handle.write(json.dumps(entry) + '\n')
                print(json.dumps({k: entry[k] for k in ('batch', 'seed', 'simulated', 'clock_seconds', 'decisions',
                                                        'expected_decisions', 'parity_differing', 'elapsed_seconds')}),
                      flush=True)
                if result.simulated_tasks and (result.canary is None or not result.canary['matched']):
                    raise P.CanaryMismatch(f'Batch {index}: canary {result.canary}')
                if differing:
                    raise ParityMismatch(f'Batch {index}: round-16 rows {differing} differ from Phase 0')
    segment = session.segment                     # as written to the ledger when the session closed
    summary = {k: segment[k] for k in ('status', 'charged_seconds', 'clock_seconds', 'decisions', 'canary_decisions',
                                       'evaluation_episodes', 'worker_seconds', 'simulated_seconds', 'workers', 'batches',
                                       'decision_allowance', 'expected_decisions', 'started_at_unix', 'config_sha256')}
    summary['ledger'] = str(session.path)
    (out / 'charge.json').write_text(json.dumps(summary, indent=1))
    return report(out)


def report(out):
    """Pass rates with Wilson intervals, paired differences and events, from OUT/results.jsonl."""
    from experiments import v15_pool as P
    from traffic import v15_stats as S
    out = Path(out).resolve()
    config = json.loads((out / 'resolved-config.json').read_text())
    ids = {name: driver['id'] for name, driver in config['drivers'].items()}
    records = {}
    for record in P.read_results(out / 'results.jsonl'):
        f = record['fields']
        if f['row_set'] == ROW_SET and f['background'] == BACKGROUND and f['mode'] == 'validate':
            records.setdefault((f['driver'], f['row']), record)
    rows = config['rows']
    planned = {name: (list(range(len(rows))) if name != 'plan-follower' else config['drivers'][name]['rows'])
               for name in DRIVERS}
    drivers = {}
    for name in DRIVERS:
        done = [r for r in planned[name] if (ids[name], r) in records]
        passes = [records[(ids[name], r)]['passed'] for r in done]
        cases = {}
        for r in done:
            cases.setdefault(rows[r]['case'], []).append(records[(ids[name], r)]['passed'])
        failures, events = {}, {}
        for r in done:
            record = records[(ids[name], r)]
            for failure in record['failures']:
                failures[failure] = failures.get(failure, 0) + 1
            for event, count in record['events'].items():
                slot = events.setdefault(event, dict(rows=0, count=0))
                slot['rows'] += int(count > 0)
                slot['count'] += int(count)
        drivers[name] = dict(id=ids[name], planned_rows=len(planned[name]), run_rows=len(done), pooled=S.rate(passes),
                             cases={case: S.rate(p) for case, p in cases.items()},
                             decisions=sum(records[(ids[name], r)]['decisions'] for r in done),
                             gate_failures=dict(sorted(failures.items(), key=lambda kv: -kv[1])), events=events)
    pairs = []
    for first, second, label in config['paired']:
        shared = [r for r in planned[first] if r in set(planned[second])
                  and (ids[first], r) in records and (ids[second], r) in records]
        pair = S.paired([records[(ids[first], r)]['passed'] for r in shared],
                        [records[(ids[second], r)]['passed'] for r in shared])
        pairs.append(dict(first=first, second=second, rows_label=label, **pair))
    result = dict(schema=SCHEMA + '_report', simulator='SUMO (libsumo); no physics', background=BACKGROUND,
                  rows='v15 development rows; held-out rows not built or run', drivers=drivers, paired=pairs,
                  charge=json.loads((out / 'charge.json').read_text()) if (out / 'charge.json').is_file() else None,
                  caveat='Rows of one case share its network and traffic setting and differ in seed; pooled '
                         'intervals treat the rows as independent.')
    (out / 'report.json').write_text(json.dumps(result, indent=1))
    (out / 'report.md').write_text(_markdown(result, config))
    return result


def _pct(x):
    return '-' if x is None else f'{100 * x:.0f}%'


def _markdown(result, config):
    lines = ['# v15 Phase 1 report', '', f'{result["simulator"]}; background {result["background"]}; {result["rows"]}.', '',
             '| Driver | Rows passed | Pass rate (95% Wilson) | Decisions |', '|---|---|---|---|']
    for name, d in result['drivers'].items():
        p = d['pooled']
        lines.append(f'| {name} | {p["passed"]} of {p["rows"]} | {_pct(p["rate"])} ({_pct(p["low"])}-{_pct(p["high"])}) | '
                     f'{d["decisions"]:,} |')
    lines += ['', '| Case | ' + ' | '.join(DRIVERS) + ' |', '|---' * (len(DRIVERS) + 1) + '|']
    for case in dict.fromkeys(r['case'] for r in config['rows']):
        cells = []
        for name in DRIVERS:
            c = result['drivers'][name]['cases'].get(case)
            cells.append('not scored' if c is None else f'{c["passed"]}/{c["rows"]}')
        lines.append(f'| {case} | ' + ' | '.join(cells) + ' |')
    lines += ['', '| First vs second | Rows | Both | First only | Second only | Neither | Difference (95%) | McNemar p |',
              '|---|---|---|---|---|---|---|---|']
    for p in result['paired']:
        lines.append(f'| {p["first"]} vs {p["second"]} | {p["rows"]} | {p["both"]} | {p["first_only"]} | '
                     f'{p["second_only"]} | {p["neither"]} | {100 * p["difference"]:+.1f} pts '
                     f'({100 * p["low"]:+.1f} to {100 * p["high"]:+.1f}) | {p["mcnemar_p"]:.3f} |')
    lines += ['', f'Caveat: {result["caveat"]}', '']
    return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('command', choices=('resolve', 'run', 'report'))
    parser.add_argument('out', nargs='?')
    args = parser.parse_args(argv)
    if args.command == 'resolve':
        config = resolve()
        CONFIG.parent.mkdir(parents=True, exist_ok=True)
        CONFIG.write_text(json.dumps(config, indent=1, sort_keys=True) + '\n')
        print(json.dumps(dict(config=str(CONFIG.relative_to(ROOT)), sha256=_json_sha(config), **{
            k: config['charge'][k] for k in ('expected_decisions', 'decision_allowance', 'expected_episodes',
                                             'expected_clock_seconds', 'reserve_seconds', 'decision_ratios')})))
    elif args.command == 'run':
        if not args.out:
            parser.error('run needs OUT')
        result = run(args.out)
        print(json.dumps({name: d['pooled'] for name, d in result['drivers'].items()}))
    else:
        if not args.out:
            parser.error('report needs OUT')
        report(args.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
