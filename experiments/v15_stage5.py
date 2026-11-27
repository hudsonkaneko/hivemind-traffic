"""v15 Stage 5 part 1: the held-out check. SUMO only, charged, once.

User decision, November 27, 2026, typed in the Claude side session: "Approve recommended" (plan v15 decision 10,
"Stage 5 proposal"), in answer to "Approve Stage 5 at the recommended size?". Part 1 at the recommended size:
- the Phase 3 policy 'l2:bc7c050f6d3da3ce' (the relabel learner, the only candidate), with expert v3
  ('rule:expert-v3', Phase 2 run 3's qualified rule expert) and the round-16 policy ('policy:f0b11fa577b33763') as
  fixed baselines, each on the 42 reserved held-out rows (the 14 cases at seeds 6953001-6953003), in one batch;
- Stage 5's own ledger allowance (phase '5'): at most 300,000 decisions and 300 s, from the study's time, with no
  raise of the study total;
- the same settings as Phases 1-4: at most 16 workers at below-normal priority, IDM background only, SUMO only, the
  shared lock, and the resolved configuration and expected charge in the coordination master before it runs.
Part 2 (the physics drives on development row 11) is separate and comes after this part.

The batch also holds two development checks, so no canary falls on a held-out row and every held-out row runs once:
- the policy on development row 3 goes first; the pool's canary re-runs it (the shortest horizon, then the first),
  and both must reproduce its Phase 3 record exactly;
- the parity row: 'l2:rule-v3' on development row 52 must reproduce Phase 2 run 3's expert v3 record, as in Phases 3-4.

The held-out rows are built only when the run starts, by a worker pool given the user's authorization
(experiments/v15_pool.WorkerPool(held_out=...), which calls traffic/v15_benchmark.held_out_rows(cfg, authorized=True)).
`resolve` never builds them: their horizons and the decision estimate come from the cases and the development rows
(each driver's mean decisions per case over its 5 development seeds, times 3). The step refuses to run again once it
has a ledger segment or any result: no held-out row is ever re-run.

Stop rules (plan v15, "Stage 5 proposal"):
- before dispatch: stop and ask if the projection doesn't fit the allowance, the lock is held, or a test fails; the
  run is never trimmed to fit;
- a canary, development-check or parity mismatch: the run is reported as failed, nothing is re-run;
- events on held-out rows (collisions, free-road stops, cut-ins, emergencies) are results: reported, nothing retuned.

Reading, set before the run (READING): the policy holds up as well as its teacher if the 95% interval of its paired
difference from expert v3 under the frozen re-baselined gates includes 0, it has no collision, and it shows no gate
failure or event it never had on the development rows; it beats v14's best if the interval of its paired difference
from round 16 (re-baselined) lies above 0. These are the study's final numbers for this policy; nothing is tuned on them.

usage (from the checkout root, with .venv-traffic's Python):
  python experiments/v15_stage5.py resolve held-out   write configs/evaluation/v15-stage5-held-out.json (no simulation)
  python experiments/v15_stage5.py run held-out       check the pinned configuration, take the lock, run (charged, once)
  python experiments/v15_stage5.py report held-out    rebuild the report from the run's records
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from experiments import v15_phase3 as P3                  # noqa: E402
from experiments import v15_phase4 as P4                  # noqa: E402
from experiments.v15_phase1 import _file_sha, _json_sha  # noqa: E402

SCHEMA = 'highway_merge_v15_stage5_v01'
OUT_ROOT = ROOT / 'outputs/highway-merge-v15/stage-5'
STEPS = ('held-out',)
PHASE = '5'
ALLOWANCE, TIME_CAP = 300_000, 300.          # "Approve recommended": part 1's cap
HELD = 'v15-held-out'                        # experiments/v15_pool.HELD_OUT_ROW_SET
CASES, HELD_SEEDS = 14, 3
POLICY, POLICY_STEP = 'l2:bc7c050f6d3da3ce', 'relabel'
EXPERT = P3.EXPERT                           # 'rule:expert-v3'
ROUND16, ROUND16_RUN = 'policy:f0b11fa577b33763', 'outputs/highway-merge-v15/phase-1/run-01'
DRIVERS = (POLICY, EXPERT, ROUND16)          # the candidate, then the two fixed baselines
NAMES = {POLICY: 'policy', EXPERT: 'expert v3', ROUND16: 'round 16'}
DEV_CHECK_ROW = 3
REBASELINE = P3.PHASE2_RUN3 + '/rebaseline/rebaseline.json'
SOURCES = ('traffic/v15_level2.py', 'traffic/v15_rule_expert.py', 'traffic/v15_benchmark.py', 'traffic/v15_stats.py',
           'experiments/v15_pool.py', 'experiments/v15_regrade.py')
AUTHORIZATION = ('Stage 5 part 1, the held-out check: user decision of November 27, 2026, typed in the Claude side '
                 'session: "Approve recommended" (plan v15 decision 10 at the recommended size: the Phase 3 policy '
                 'l2:bc7c050f6d3da3ce, with expert v3 and the round-16 policy as fixed baselines, on the 42 reserved '
                 'held-out rows, once, in one batch with the development checks; Stage 5\'s own allowance of at most '
                 '300,000 decisions and 300 s, with no raise of the study total; at most 16 workers at below-normal '
                 'priority, IDM background only, SUMO only, the shared lock, and the resolved configuration and expected '
                 'charge in the coordination master before it runs; then part 2, the physics drives on development row '
                 '11, at most 500 s, counted as Stage 5 evaluation episodes; not authorized: any raise of the study total, '
                 'Phase 4 step B, Krauss runs, Isaac Lab training, merging to main).')
STOP_RULE = ('before dispatch, stop and ask if the projection does not fit the allowance, the lock is held, or a test '
             'fails, and never trim the run; a canary, development-check or parity mismatch reports the run as failed and '
             'nothing is re-run; events on held-out rows (collisions, free-road stops, cut-ins, emergencies) are results, '
             'reported, and nothing is retuned; no held-out row is ever re-run')
READING = ('the policy holds up as well as its teacher if the 95% interval of its paired difference from expert v3 under '
           'the frozen re-baselined gates includes 0, it has no collision, and it shows no gate failure or event it never '
           'had on the development rows; it beats v14\'s best if the 95% interval of its paired difference from round 16 '
           '(re-baselined) lies above 0; these are the study\'s final numbers for this policy and nothing is tuned on them')


def config_path(step):
    return P3.CONFIG_DIR / f'v15-stage5-{step}.json'


def out_dir(step):
    return OUT_ROOT / step


# --------------------------------------------------------------------------- development records

def _first(path, driver):
    """A driver's first record on each development row (IDM) of a run's results table."""
    from experiments import v15_pool as P
    rows = {}
    for r in P.read_results(Path(path) / 'results.jsonl'):
        f = r['fields']
        if f['driver'] == driver and f['row_set'] == P3.DEV and f['background'] == P3.BACKGROUND and r['attempt'] == 'first':
            rows.setdefault(f['row'], r)
    if sorted(rows) != list(range(70)):
        raise ValueError(f'{path} has no first record of {driver} on every development row')
    return rows


def _development():
    """Each driver's development records and verdicts: recorded, and under the frozen re-baselined gates."""
    rebaseline = json.loads((ROOT / REBASELINE).read_text())
    records = {POLICY: _first(P3.out_dir(POLICY_STEP), POLICY), EXPERT: P3._run3(), ROUND16: _first(ROOT / ROUND16_RUN, ROUND16)}
    start, start_rows = P4._start_rows()
    if start['id'] != POLICY:
        raise ValueError(f'The Phase 3 relabel learner is {start["id"]}, not {POLICY}')
    out = {}
    for driver, rows in records.items():
        recorded = [bool(rows[r]['passed']) for r in range(70)]
        if driver == POLICY:
            rebased = [start_rows[r]['rebaselined'] for r in range(70)]
            failures = sorted({k for f in start['failing_rows'] for k in f['failures']})
        else:
            rebased = list(recorded)
            for change in rebaseline['changed_rows'][driver]:
                rebased[change['row']] = bool(change['rebaselined'])
            if sum(rebased) != rebaseline['pass_rates'][driver]['rebaselined']['passed']:
                raise ValueError(f'{driver}: the re-baselined development verdicts do not reproduce {REBASELINE}')
            failures = None
        out[driver] = dict(records=rows, recorded=recorded, rebaselined=rebased, failures=failures,
                           events=P3._events(list(rows.values())))
    return out


def _estimate(cases, development):
    """Each driver's expected decisions on the held-out rows: its mean per case over the 5 development seeds, times 3
    (with the range from each case's fewest and most), without building a held-out row."""
    per = {}
    for driver in DRIVERS:
        rows = development[driver]['records']
        by_case = {c: [rows[14 * k + c]['decisions'] for k in range(5)] for c in range(CASES)}
        per[driver] = dict(expected=round(sum(HELD_SEEDS * sum(v) / len(v) for v in by_case.values())),
                           low=sum(HELD_SEEDS * min(v) for v in by_case.values()),
                           high=sum(HELD_SEEDS * max(v) for v in by_case.values()),
                           development_decisions=sum(r['decisions'] for r in rows.values()),
                           longest=max(r['decisions'] for r in rows.values()),
                           per_case_mean={str(c): round(sum(v) / len(v), 1) for c, v in by_case.items()})
    return per


def _canary_index(tasks, horizon):
    """The task the pool's canary re-runs: the shortest horizon, then the first (experiments/v15_pool.WorkerPool.run).
    Held-out row r and development row r are both case r % 14, so the horizon comes from the case."""
    return min(range(len(tasks)), key=lambda i: (horizon[tasks[i]['row'] % CASES], i))


def _ledger_stage5():
    from experiments import v15_ledger as L
    ledger = json.loads((L.study_directory() / 'cost-ledger.json').read_text())
    totals = L.totals(ledger)
    return totals['v15'].get(PHASE, dict(segments=0, decisions=0, seconds=0., evaluation_episodes=0)), totals, ledger


def _held_out_tasks():
    return [dict(row_set=HELD, row=r, driver=d) for r in range(CASES * HELD_SEEDS) for d in DRIVERS]


# --------------------------------------------------------------------------- resolve

def resolve(step):
    """The held-out check's resolved configuration and expected charge (no simulation, no held-out row built)."""
    import torch
    from experiments import v15_pool as P
    from traffic import highway_flow_rl_v14_contract as contract
    from traffic import v15_benchmark, v15_level2
    if step not in STEPS:
        raise ValueError(f'Unknown step {step!r}; expected one of {STEPS}')
    if P.HELD_OUT_ROW_SET != HELD or HELD in P.ROW_SETS:
        raise ValueError('The pool\'s held-out row set must be separate from its ordinary row sets')
    cfg = contract.build_config(P3.ROUND_INDEX)
    benchmark = v15_benchmark.manifest(cfg)
    if (benchmark['held_out_rows'], benchmark['held_out_seeds']) != (CASES * HELD_SEEDS, list(v15_benchmark.HELD_OUT_SEEDS)):
        raise ValueError('The benchmark does not reserve 42 held-out rows at 3 seeds')
    rebaseline = json.loads((ROOT / REBASELINE).read_text())
    if _file_sha(ROOT / 'experiments/v15_regrade.py') != rebaseline['module_sha256']:
        raise ValueError('experiments/v15_regrade.py is not the frozen re-baselined regrade (decision 3)')
    cases = v15_benchmark.cases(cfg)
    horizon = {c: cases[c]['horizon_steps'] for c in range(CASES)}
    development = _development()
    estimate = _estimate(cases, development)
    # the drivers' weights: the hash of what is loaded must be the driver id
    sources = {}
    for driver, path in ((POLICY, P3.out_dir(POLICY_STEP) / 'policies' / f'{POLICY[3:]}.pt'),
                         (ROUND16, ROOT / ROUND16_RUN / 'policies' / f'{ROUND16[7:]}.pt')):
        digest = P.state_hash(torch.load(path, map_location='cpu', weights_only=False))
        if digest != driver.split(':', 1)[1]:
            raise ValueError(f'{path} holds {digest}, not {driver}')
        sources[driver] = dict(weights=path.relative_to(ROOT).as_posix(), file_sha256=_file_sha(path))
    sources[EXPERT] = dict(module='traffic/v15_rule_expert.py', file_sha256=_file_sha(ROOT / 'traffic/v15_rule_expert.py'),
                           qualified='Phase 2 run 3')
    check_record = development[POLICY]['records'][DEV_CHECK_ROW]
    dev_check = dict(driver=POLICY, row=DEV_CHECK_ROW, report_sha=check_record['report_sha'],
                     trace_sha=check_record['trace_sha'], decisions=check_record['decisions'],
                     source=f'{P3.out_dir(POLICY_STEP).relative_to(ROOT).as_posix()}/results.jsonl')
    run3 = P3._run3()
    parity = dict(driver=v15_level2.RULE_DRIVER, row=P3.PARITY_ROW, report_sha=run3[P3.PARITY_ROW]['report_sha'],
                  trace_sha=run3[P3.PARITY_ROW]['trace_sha'], decisions=run3[P3.PARITY_ROW]['decisions'],
                  source=f'{P3.PHASE2_RUN3}/results.jsonl, {EXPERT}')
    tasks = [dict(row_set=P3.DEV, row=DEV_CHECK_ROW, driver=POLICY, dev_check=True)] + _held_out_tasks()
    tasks.append(dict(parity, row_set=P3.DEV, mode='validate', parity=True))
    canary = _canary_index(tasks, horizon)
    if canary != 0:
        raise ValueError(f'The canary would re-run task {canary} ({tasks[canary]}), not the development check')
    held = sum(e['expected'] for e in estimate.values())
    checks = 2 * dev_check['decisions'] + parity['decisions']       # the row, its canary and the parity row
    expected = held + checks
    rate = P4._measured_rate()
    longest = max(e['longest'] for e in estimate.values())
    clock = P3.START_S + max(expected / rate['decisions_per_clock_second'], longest / 140.) + 10.
    phase, totals, _ = _ledger_stage5()
    decisions_left, seconds_left = ALLOWANCE - phase['decisions'], TIME_CAP - phase['seconds']
    study_left = cfg['budget']['total_wall_seconds'] - totals['total']
    reserve = min(round(clock * 1.6 + 30.), round(seconds_left, 3), round(study_left, 3))
    return dict(
        schema=SCHEMA, phase=PHASE, step=step, round_index=P3.ROUND_INDEX, backend=P3.BACKEND, workers=P3.WORKERS,
        priority=P3.PRIORITY, backgrounds=[P3.BACKGROUND], benchmark=benchmark,
        held_out=dict(row_set=HELD, rows=CASES * HELD_SEEDS, seeds=list(v15_benchmark.HELD_OUT_SEEDS),
                      row_order='row 14 * k + c is case c at held-out seed k',
                      built='only when the run starts, by a worker pool given the authorization; resolve builds none',
                      runs='once; the step refuses to run again'),
        drivers=[dict(id=d, name=NAMES[d], role='candidate' if d == POLICY else 'fixed baseline', **sources[d],
                      development=dict(rebaselined_passed=sum(development[d]['rebaselined']),
                                       recorded_passed=sum(development[d]['recorded'])))
                 for d in DRIVERS],
        policy_development=dict(failures_rebaselined=development[POLICY]['failures'],
                                events={k: v for k, v in development[POLICY]['events'].items() if v['count']}),
        sources={p: _file_sha(ROOT / p) for p in SOURCES}, regrade=dict(module_sha256=rebaseline['module_sha256'],
                                                                        rules=list(rebaseline['rules']), source=REBASELINE),
        level2=dict(settings_digest=v15_level2.settings_digest(), rule=v15_level2.RULE_DRIVER),
        batches=[dict(name='held-out', drivers=list(DRIVERS), tasks=tasks, expected_decisions=expected)],
        dev_check=dev_check, parity=parity, reading=READING,
        stop_checks=dict(rule=STOP_RULE, one_car_la_cases=list(P3.ONE_CAR_LA)),
        charge=dict(expected_decisions=expected, projected_decisions=round(P4._projected(expected)),
                    expected_episodes=len(tasks) + 1, expected_clock_seconds=round(clock, 1), reserve_seconds=reserve,
                    phase_allowance=ALLOWANCE, phase_time_cap=TIME_CAP, phase_decisions_used=phase['decisions'],
                    phase_seconds_used=round(phase['seconds'], 3), decisions_left=decisions_left,
                    seconds_left=round(seconds_left, 3), study_seconds_left=round(study_left, 3),
                    guard_margin=P3.MARGIN, range=dict(low=sum(e['low'] for e in estimate.values()) + checks,
                                                       high=sum(e['high'] for e in estimate.values()) + checks),
                    estimate_source=dict(
                        per_driver={d: estimate[d] for d in DRIVERS},
                        rule='each driver\'s mean decisions per case over its 5 development seeds, times 3, summed over '
                             'the 14 cases; the range from each case\'s fewest and most',
                        checks=dict(dev_check=dev_check['decisions'], canary=dev_check['decisions'],
                                    parity=parity['decisions']),
                        rate=rate, longest_episode_decisions=longest, decisions_per_worker_second=140,
                        start_seconds=P3.START_S),
                    total_wall_seconds=cfg['budget']['total_wall_seconds']),
        authorization=AUTHORIZATION, simulator='SUMO (libsumo); no physics')


# --------------------------------------------------------------------------- run

def _not_run_yet(step):
    """The held-out check runs once: refuse if its output holds anything or the ledger has a segment for it."""
    out = out_dir(step)
    for name in ('charge.json', 'results.jsonl', 'episodes', 'report.json'):
        if (out / name).exists():
            raise ValueError(f'Stage 5 step {step} has already run ({out / name}); no held-out row is re-run')
    _, _, ledger = _ledger_stage5()
    found = [s for s in ledger['segments'] if s.get('v15_phase') == PHASE and s.get('stage5_step') == step]
    if found:
        raise ValueError(f'Stage 5 step {step} already has {len(found)} ledger segment(s) '
                         f'({[s["status"] for s in found]}); no held-out row is re-run')


class CheckMismatch(RuntimeError):
    """The canary, the development check or the parity row did not reproduce its record: the run failed."""


def _checks(config, tasks, result):
    """The canary must re-run the development check and match; the check and the parity row must match their records."""
    first = result.records[0]
    canary = result.canary
    dev = config['dev_check']
    checks = dict(canary=bool(canary and canary['matched'] and canary['key'] == first['key']),
                  dev_check=(first['report_sha'], first['trace_sha']) == (dev['report_sha'], dev['trace_sha']),
                  parity=None)
    for t, record in zip(tasks, result.records):
        if t.get('parity'):
            checks['parity'] = (record['report_sha'], record['trace_sha']) == (config['parity']['report_sha'],
                                                                               config['parity']['trace_sha'])
    return checks


def run(step):
    import torch
    from experiments import v15_ledger as L
    from experiments import v15_pool as P
    _not_run_yet(step)
    config = json.loads(json.dumps(resolve(step)))
    pinned = json.loads(config_path(step).read_text())
    if config != pinned:
        raise ValueError(f'The resolved configuration differs from {config_path(step)}; re-resolve and record it first')
    sha = _json_sha(pinned)
    charge = config['charge']
    if config['workers'] > 16:
        raise ValueError('At most 16 workers (the user\'s CPU limit)')
    P4._check_fit(charge)
    if P.set_priority(config['priority']) != config['priority']:
        raise RuntimeError(f'The learner does not run at {config["priority"]} priority')
    out = out_dir(step)
    out.mkdir(parents=True, exist_ok=True)
    (out / 'resolved-config.json').write_text(json.dumps(config, indent=1, sort_keys=True))
    log_path = out / 'stage5-log.jsonl'

    def log(entry):
        with open(log_path, 'a', encoding='utf-8') as handle:
            handle.write(json.dumps(entry) + '\n')
        print(json.dumps(entry), flush=True)

    batch = config['batches'][0]
    tasks = batch['tasks']
    session = None
    try:
        with L.EvaluationSession(phase=PHASE, output=out, config_sha256=sha, expected_decisions=charge['expected_decisions'],
                                 decision_allowance=ALLOWANCE, expected_clock_seconds=charge['expected_clock_seconds'],
                                 reserve_seconds=charge['reserve_seconds'], total_wall_seconds=charge['total_wall_seconds'],
                                 intended_episodes=charge['expected_episodes'], workers=config['workers'],
                                 priority=config['priority'], authorization=config['authorization'],
                                 round_index=config['round_index']) as session:
            # training_episodes: experiments/v15_phase3._add moves training-row episodes there (none here)
            session.segment.update(stage5_step=step, v15_work=['evaluation'], training_episodes=0,
                                   held_out_rows=config['held_out']['rows'])
            with P.WorkerPool(config['workers'], out, backend=config['backend'], round_index=config['round_index'],
                              priority=config['priority'], held_out=config['authorization']) as pool:
                session.segment['worker_priorities'] = sorted(set(pool.worker_priorities.values()))
                pool.charged_by = f'cost-ledger {L.COMMAND} phase {PHASE} step {step}, started_at_unix {session.segment["started_at_unix"]}'
                for d in config['drivers']:
                    if 'weights' not in d:
                        continue
                    state = torch.load(ROOT / d['weights'], map_location='cpu', weights_only=False)
                    published = pool.publish_level2(state) if d['id'].startswith('l2:') else pool.publish(state)
                    if published != d['id']:
                        raise ValueError(f'{d["weights"]} published as {published}, not {d["id"]}')
                P3._check_policies(out, tasks)
                projected = P4._projected(batch['expected_decisions'])
                session.check_next(projected)
                pool_tasks = [P3._task(P, t, f'stage5-{step}') for t in tasks]
                try:
                    result = pool.run(pool_tasks)
                except P.CanaryMismatch as error:
                    if error.result is not None:
                        P3._add(session, error.result, pool_tasks)
                    else:
                        session.charge_estimate(projected, f'batch 0: {error}')
                    raise
                except BaseException as error:
                    session.charge_estimate(projected, f'batch 0 failed before reporting: {type(error).__name__}')
                    raise
                P3._add(session, result, pool_tasks)
                checks = _checks(config, tasks, result)
                log(dict(batch=0, name=batch['name'], tasks=len(pool_tasks), simulated=result.simulated_tasks,
                         clock_seconds=result.clock_seconds, worker_seconds=result.worker_seconds,
                         decisions=result.decisions, canary_decisions=result.canary_decisions,
                         expected_decisions=batch['expected_decisions'], canary=result.canary, checks=checks,
                         elapsed_seconds=session.elapsed(), charged_decisions_so_far=session.segment['decisions']))
                if not all(checks.values()):
                    raise CheckMismatch(f'The development checks did not reproduce their records: {checks}')
    except BaseException as error:
        # a failed run is still reported (the stop rule); its rows are never re-run
        if session is not None and session.segment is not None:
            _write_charge(out, session)
        if (out / 'results.jsonl').is_file():
            report(step, failed=f'{type(error).__name__}: {error}')
        raise
    _write_charge(out, session)
    return report(step)


def _write_charge(out, session):
    segment = session.segment
    summary = {k: segment.get(k) for k in ('status', 'charged_seconds', 'clock_seconds', 'decisions', 'canary_decisions',
                                           'evaluation_episodes', 'worker_seconds', 'simulated_seconds', 'workers',
                                           'batches', 'decision_allowance', 'expected_decisions', 'started_at_unix',
                                           'config_sha256', 'failure')}
    summary['ledger'] = str(session.path)
    (out / 'charge.json').write_text(json.dumps(summary, indent=1))


# --------------------------------------------------------------------------- report

class _HeldOutRegrade:
    """Run the frozen regrade (experiments/v15_regrade.py, unchanged) on the held-out row set: its first_records reads
    the row set from the module, so it is pointed at the held-out rows for the call and restored after."""

    def __enter__(self):
        from experiments import v15_regrade as R
        self.R, self.saved = R, R.ROW_SET
        R.ROW_SET = HELD
        return R

    def __exit__(self, *_):
        self.R.ROW_SET = self.saved


def _held_records(records):
    by = {}
    for r in records:
        f = r['fields']
        if f['row_set'] == HELD:
            by.setdefault(f['driver'], {}).setdefault(f['row'], r)
    return by


def report(step, failed=None):
    from experiments import v15_pool as P
    from traffic import highway_flow_rl_v14_contract as contract
    from traffic import v15_benchmark
    from traffic import v15_stats as S
    out = out_dir(step)
    config = json.loads((out / 'resolved-config.json').read_text())
    n = config['held_out']['rows']
    first = {}
    for record in P.read_results(out / 'results.jsonl'):
        first.setdefault(record['key'], record)
    records = list(first.values())
    held = _held_records(records)
    complete = sorted(held) == sorted(DRIVERS) and all(sorted(held[d]) == list(range(n)) for d in DRIVERS)
    result = dict(schema=SCHEMA + '_report', step=step, simulator='SUMO (libsumo); no physics', background=P3.BACKGROUND,
                  run_failed=failed, complete=complete, reading_rule=config['reading'],
                  charge=json.loads((out / 'charge.json').read_text()) if (out / 'charge.json').is_file() else None)
    dev = [r for r in records if r['fields']['row_set'] == P3.DEV]
    result['dev_check'] = [dict(row=r['fields']['row'], attempt=r['attempt'],
                                matched=(r['report_sha'], r['trace_sha']) == (config['dev_check']['report_sha'],
                                                                              config['dev_check']['trace_sha']))
                           for r in dev if r['fields']['driver'] == POLICY]
    result['parity'] = [dict(row=r['fields']['row'], matched=(r['report_sha'], r['trace_sha']) ==
                             (config['parity']['report_sha'], config['parity']['trace_sha']))
                        for r in dev if r['fields']['driver'] == config['parity']['driver']]
    if not complete:
        result['rows_present'] = {d: sorted(held.get(d, {})) for d in DRIVERS}
        _write_report(out, result)
        return result
    cases = v15_benchmark.cases(contract.build_config(P3.ROUND_INDEX))
    case_rows = [cases[r % CASES] for r in range(n)]          # case ids only: no held-out row is rebuilt here
    with _HeldOutRegrade() as R:
        regraded, regrade_parity = R.regrade_run(out, dict(none=(), rebaselined=R.RULES))
    drivers = {}
    verdicts = {}
    for d in DRIVERS:
        mine = [held[d][r] for r in range(n)]
        g = regraded[d]
        recorded = [bool(r['passed']) for r in mine]
        if [g['passed']['none'][r] for r in range(n)] != recorded:
            raise ValueError(f'{d}: the re-grade without rules does not reproduce the recorded verdicts')
        rebased = [g['passed']['rebaselined'][r] for r in range(n)]
        verdicts[d] = dict(rebaselined=rebased, recorded=recorded)
        drivers[d] = dict(name=NAMES[d], role=next(x['role'] for x in config['drivers'] if x['id'] == d),
                          rebaselined=S.rate(rebased), recorded=S.rate(recorded),
                          development=next(x['development'] for x in config['drivers'] if x['id'] == d),
                          decisions=sum(r['decisions'] for r in mine), events=P3._events(mine),
                          stop_rule_events=P3._new_failures(mine, case_rows),
                          failing_rows={r: g['failures']['rebaselined'][r] for r in range(n) if not rebased[r]},
                          failing_rows_recorded={r: mine[r]['failures'] for r in range(n) if not recorded[r]})
    paired = {}
    for a, b in ((POLICY, EXPERT), (POLICY, ROUND16), (EXPERT, ROUND16)):
        paired[f'{NAMES[a]} vs {NAMES[b]}'] = {gate: S.paired(verdicts[a][gate], verdicts[b][gate])
                                               for gate in ('rebaselined', 'recorded')}
    per_case = []
    for c in range(CASES):
        rows = [c + CASES * k for k in range(HELD_SEEDS)]
        per_case.append(dict(case=cases[c]['case']['id'], rows=rows,
                             **{NAMES[d]: dict(rebaselined=sum(verdicts[d]['rebaselined'][r] for r in rows),
                                               recorded=sum(verdicts[d]['recorded'][r] for r in rows))
                                for d in DRIVERS}))
    result.update(
        rows=n, drivers={d: drivers[d] for d in DRIVERS}, paired=paired, per_case=per_case,
        regrade_parity=regrade_parity,
        reading=_reading(paired['policy vs expert v3']['rebaselined'], paired['policy vs round 16']['rebaselined'],
                         drivers[POLICY]['failing_rows'], drivers[POLICY]['events'],
                         config['policy_development']['failures_rebaselined'], config['policy_development']['events']))
    _write_report(out, result)
    return result


def _reading(vs_expert, vs_round16, failing_rows, events, dev_failures, dev_events):
    """READING, set before the run: holds up (the interval against expert v3 includes 0, no collision, no gate failure
    or event the policy never had on the development rows) and beats v14's best (the interval against round 16 above 0).

    failing_rows: {row: [re-baselined gate failures]}; events: {event: dict(rows, count)} on the held-out rows."""
    new_failures = sorted({k for f in failing_rows.values() for k in f} - set(dev_failures))
    new_events = sorted(k for k, v in events.items() if v['count'] and k not in set(dev_events))
    collisions = events.get('collisions', dict(count=0))['count']
    return dict(holds_up=vs_expert['low'] <= 0 <= vs_expert['high'] and not collisions and not new_failures
                and not new_events,
                beats_v14=vs_round16['low'] > 0, vs_expert_interval=[vs_expert['low'], vs_expert['high']],
                vs_round16_interval=[vs_round16['low'], vs_round16['high']], policy_collisions=collisions,
                new_gate_failures=new_failures, new_events=new_events,
                development_failures=sorted(dev_failures), development_events=sorted(dev_events))


def _write_report(out, result):
    (out / 'report.json').write_text(json.dumps(result, indent=1))
    (out / 'report.md').write_text(_markdown(result))


def _pct(rate):
    return f'{rate["passed"]}/{rate["rows"]} ({100 * rate["rate"]:.1f}%, {100 * rate["low"]:.1f}-{100 * rate["high"]:.1f}%)'


def _markdown(result):
    s = result
    lines = ['# v15 Stage 5 part 1: the held-out check', '', f'{s["simulator"]}; background {s["background"]}; the 42 '
             'held-out rows (14 cases at seeds 6953001-6953003), run once.', '']
    if s['run_failed']:
        lines += [f'**The run failed:** {s["run_failed"]}', '']
    lines += [f'Development check (the policy on development row 3, and its canary): {s["dev_check"]}',
              f'Parity (l2:rule-v3 on development row 52): {s["parity"]}', '']
    if not s['complete']:
        lines += [f'Incomplete: {s.get("rows_present")}', '', f'Charge: {s["charge"]}']
        return '\n'.join(lines) + '\n'
    lines += ['| Driver | Role | Re-baselined | Recorded | Development (re-baselined, recorded) | Stop-rule events |',
              '|---|---|---|---|---|---|']
    for d in s['drivers'].values():
        lines.append(f'| {d["name"]} | {d["role"]} | {_pct(d["rebaselined"])} | {_pct(d["recorded"])} | '
                     f'{d["development"]["rebaselined_passed"]}/70, {d["development"]["recorded_passed"]}/70 | '
                     f'{len(d["stop_rule_events"])} |')
    lines += ['', '| Pair | Gates | Difference | 95% interval | McNemar p | Only first | Only second |',
              '|---|---|---|---|---|---|---|']
    for name, gates in s['paired'].items():
        for gate, p in gates.items():
            lines.append(f'| {name} | {gate} | {100 * p["difference"]:+.1f} pts | {100 * p["low"]:+.1f} to '
                         f'{100 * p["high"]:+.1f} | {p["mcnemar_p"]:.3g} | {p["first_only"]} | {p["second_only"]} |')
    lines += ['', '| Case | Rows | Policy | Expert v3 | Round 16 |', '|---|---|---|---|---|']
    for c in s['per_case']:
        cell = lambda name: f'{c[name]["rebaselined"]}/3, {c[name]["recorded"]}/3'
        lines.append(f'| {c["case"]} | {c["rows"]} | {cell("policy")} | {cell("expert v3")} | {cell("round 16")} |')
    r = s['reading']
    lines += ['', f'Reading (set before the run): holds up as well as its teacher: {r["holds_up"]}; beats v14\'s best: '
              f'{r["beats_v14"]}.', f'New gate failures: {r["new_gate_failures"] or "none"}; new events: '
              f'{r["new_events"] or "none"}; policy collisions: {r["policy_collisions"]}.', '',
              f'Charge: {s["charge"]}']
    return '\n'.join(lines) + '\n'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('command', choices=('resolve', 'run', 'report'))
    parser.add_argument('step')
    args = parser.parse_args(argv)
    if args.command == 'resolve':
        config = resolve(args.step)
        path = config_path(args.step)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(config, indent=1, sort_keys=True) + '\n')
        print(json.dumps(dict(config=str(path.relative_to(ROOT)), sha256=_json_sha(config),
                              **{k: config['charge'][k] for k in ('expected_decisions', 'projected_decisions',
                                                                  'expected_episodes', 'expected_clock_seconds',
                                                                  'reserve_seconds', 'decisions_left', 'seconds_left',
                                                                  'study_seconds_left', 'range')})))
    elif args.command == 'run':
        result = run(args.step)
        print(json.dumps(dict(reading=result.get('reading'), dev_check=result['dev_check'], parity=result['parity'])))
    else:
        report(args.step)
    return 0


if __name__ == '__main__':
    sys.exit(main())
