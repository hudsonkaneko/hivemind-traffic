"""v15 Phase 4 step A's probe: scripted Level 2 variants on the starting policy's failing rows. SUMO only, charged.

User decision, November 26, 2026, typed in the Claude side session: "Yes, run step A" (plan v15 decision 9,
"Phase 4 proposal"). Step A is:
- the probe (this file): the 5 scripted Level 2 variants of traffic/v15_level2.PROBES (the rule's choices with a
  constant speed offset of -2, -1, +1 or +2 m/s, or the gap one car later than the rule's) on the development
  rows that the starting policy (the relabel learner, experiments/v15_phase3.py step relabel) and expert v3 both
  fail. It is Phase 4's first ledger segment, with its own allowance: at most 100,000 decisions and 150 s;
- the seed check, experiments/v15_phase3.py step relabel-seeds, from Phase 3's unused allowance.
Together about 360,000 decisions and 290 s, with no raise of the study total. Same settings as Phases 1-3: at
most 16 workers at below-normal priority, IDM background only, SUMO only, development rows only, held-out rows
never built or run, the shared lock, and the resolved configuration and expected charge in the coordination
master before it runs. Not authorized: step B (anchored MAPPO), any raise of the study total, held-out runs,
Krauss runs, merging to main.

The probe trains nothing. It asks whether any simple Level 2 choice passes a row the rule fails: if none does,
the proposal recommends skipping step B. RL could still find a state-dependent choice the constant variants miss,
so a negative probe is a strong hint, not proof. Every run also re-runs 'l2:rule-v3' on development row 52 (parity
with Phase 2 run 3's expert v3) and has a canary on another worker.

usage (from the checkout root, with .venv-traffic's Python):
  python experiments/v15_phase4.py resolve probe     write configs/evaluation/v15-phase4-probe.json (no simulation)
  python experiments/v15_phase4.py run probe         check the pinned configuration, take the lock, run (charged)
  python experiments/v15_phase4.py report probe      rebuild the report (re-baselined verdicts and speed match)
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from experiments import v15_phase3 as P3                       # noqa: E402
from experiments.v15_phase1 import _file_sha, _json_sha   # noqa: E402

SCHEMA = 'highway_merge_v15_phase4_v01'
OUT_ROOT = ROOT / 'outputs/highway-merge-v15/phase-4'
STEPS = ('probe',)
ALLOWANCE, TIME_CAP = 100_000, 150.          # step A's probe: Phase 4's own allowance so far
START_STEP = 'relabel'                       # the starting policy's Phase 3 step
SPEED_MATCH_GATE = 0.9
AUTHORIZATION = P3.AUTHORIZATION + ' ' + P3.STEP_A_AUTHORIZATION
SLOWER = ('l2:probe-gap-p1', 'l2:probe-offset-m1', 'l2:probe-offset-m2')   # can lengthen an episode
STOP_RULE = ('a canary or parity mismatch stops the run; so does any probe collision or free-road stop, or a probe '
             'cut-in or emergency in a one-car LA case: the probes count as drivers for the stop rule, the event is '
             'reported to the user and nothing is retuned (the main session\'s relay of the user\'s "Yes, approve '
             'step A", November 26, 2026). The probe is one batch, so the stop comes after its charge and report')


def config_path(step):
    return P3.CONFIG_DIR / f'v15-phase4-{step}.json'


def out_dir(step):
    return OUT_ROOT / step


def _ledger_phase4():
    from experiments import v15_ledger as L
    ledger = json.loads((L.study_directory() / 'cost-ledger.json').read_text())
    totals = L.totals(ledger)
    return totals['v15'].get('4', dict(segments=0, decisions=0, seconds=0., evaluation_episodes=0)), totals


def _failing_rows():
    """The development rows the starting policy fails (re-baselined), which must be expert v3's failing rows."""
    start = P3._report(START_STEP)['learners'][0]
    rows = sorted(f['row'] for f in start['failing_rows'])
    expert = P3._expert_rebaselined(ROOT / P3.PHASE2_RUN3 / 'rebaseline' / 'rebaseline.json')
    if rows != [r for r in range(70) if not expert[r]]:
        raise ValueError(f'The starting policy fails rows {rows}, not expert v3\'s')
    return start, rows


def resolve(step):
    """The probe's resolved configuration and expected charge (no simulation)."""
    from traffic import highway_flow_rl_v14_contract as contract
    from traffic import v15_benchmark, v15_level2
    if step not in STEPS:
        raise ValueError(f'Unknown step {step!r}; expected one of {STEPS}')
    if not (P3.out_dir('relabel-seeds') / 'report.json').is_file():
        raise ValueError('Step A runs the seed check (experiments/v15_phase3.py relabel-seeds) first')
    cfg = contract.build_config(P3.ROUND_INDEX)
    run3 = P3._run3()
    start, rows = _failing_rows()
    probes = sorted(v15_level2.PROBES)
    parity = dict(driver=v15_level2.RULE_DRIVER, row=P3.PARITY_ROW, report_sha=run3[P3.PARITY_ROW]['report_sha'],
                  trace_sha=run3[P3.PARITY_ROW]['trace_sha'], decisions=run3[P3.PARITY_ROW]['decisions'],
                  source=f'{P3.PHASE2_RUN3}/results.jsonl, {P3.EXPERT}')
    per_row = {str(r): run3[r]['decisions'] for r in rows}      # string keys, as the pinned JSON holds them
    longest = max(per_row.values())
    # Two batches: the variants that can lengthen an episode (slower, or a later gap) first, so the second batch is
    # checked against the measured decisions before it is dispatched (the allowance is a hard cap)
    batches = []
    for name, group in (('probe-slower', SLOWER), ('probe-faster', tuple(p for p in probes if p not in SLOWER))):
        tasks = [dict(row_set=P3.DEV, row=r, driver=d) for r in rows for d in group]
        extra = 800
        if not batches:
            tasks.append(dict(parity, row_set=P3.DEV, mode='validate', parity=True))
            extra += parity['decisions']
        batches.append(dict(name=name, drivers=list(group), tasks=tasks,
                            expected_decisions=round(len(group) * sum(per_row.values()) + extra)))
    if sorted(d for b in batches for d in b['drivers']) != probes:
        raise ValueError('Every probe runs exactly once')
    expected = sum(b['expected_decisions'] for b in batches)
    # 16 workers at about 1,300-2,200 decisions per second of clock time (Phase 3); no batch is shorter than its
    # longest episode, at about 140 decisions per worker-second (DAgger 1's rollouts)
    clock = P3.START_S + sum(max(b['expected_decisions'] * P3.PER_DECISION_S, longest / 140.) + 10. for b in batches)
    phase, totals = _ledger_phase4()
    decisions_left, seconds_left = ALLOWANCE - phase['decisions'], TIME_CAP - phase['seconds']
    study_left = cfg['budget']['total_wall_seconds'] - totals['total']
    reserve = min(round(clock * 1.6 + 30.), seconds_left, study_left)
    return dict(
        schema=SCHEMA, phase='4', step=step, round_index=P3.ROUND_INDEX, backend=P3.BACKEND, workers=P3.WORKERS,
        priority=P3.PRIORITY, backgrounds=[P3.BACKGROUND], benchmark=v15_benchmark.manifest(cfg),
        level2=dict(rule=v15_level2.RULE_DRIVER, probes={p: v15_level2.PROBES[p] for p in probes},
                    offsets=list(v15_level2.OFFSETS), gaps=v15_level2.GAPS, settings_digest=v15_level2.settings_digest(),
                    sources={p: _file_sha(ROOT / p) for p in ('traffic/v15_level2.py', 'traffic/v15_rule_expert.py',
                                                              'experiments/v15_pool.py', 'experiments/v15_regrade.py')}),
        start_policy=dict(id=start['id'], step=START_STEP, rebaselined_passed=start['rebaselined_passed'],
                          recorded_passed=start['recorded_passed']),
        rows=rows, rows_rule='the development rows the starting policy fails under the frozen re-baselined gates; '
                             'expert v3 fails the same rows (Phase 2 run 3 and its rebaseline)',
        expert_decisions=per_row, batches=batches, parity=parity,
        reading=f'a row has room if any probe passes it under the frozen re-baselined gates; the failing cars\' '
                f'speed match is reported against the gate\'s {SPEED_MATCH_GATE} and against expert v3\'s',
        stop_checks=dict(any_driver=STOP_RULE, one_car_la_cases=list(P3.ONE_CAR_LA)),
        charge=dict(expected_decisions=expected, expected_episodes=sum(len(b['tasks']) + 1 for b in batches),
                    expected_clock_seconds=round(clock, 1),
                    reserve_seconds=reserve, phase_allowance=ALLOWANCE, phase_time_cap=TIME_CAP,
                    phase_decisions_used=phase['decisions'], phase_seconds_used=round(phase['seconds'], 3),
                    decisions_left=decisions_left, seconds_left=round(seconds_left, 3),
                    study_seconds_left=round(study_left, 3), guard_margin=P3.MARGIN,
                    estimate_source=dict(per_row='expert v3\'s decisions on each row, Phase 2 run 3',
                                         decisions_per_clock_second=1300, longest_episode_decisions=longest,
                                         decisions_per_worker_second=140, start_seconds=P3.START_S),
                    total_wall_seconds=cfg['budget']['total_wall_seconds']),
        authorization=AUTHORIZATION, held_out='not built, not run', simulator='SUMO (libsumo); no physics')


def run(step):
    from experiments import v15_ledger as L
    from experiments import v15_pool as P
    config = json.loads(json.dumps(resolve(step)))
    pinned = json.loads(config_path(step).read_text())
    if config != pinned:
        raise ValueError(f'The resolved configuration differs from {config_path(step)}; re-resolve and record it first')
    sha = _json_sha(pinned)
    charge = config['charge']
    if config['workers'] > 16:
        raise ValueError('At most 16 workers (the user\'s CPU limit)')
    if charge['expected_decisions'] * (1 + P3.MARGIN) > charge['decisions_left'] or \
            charge['expected_clock_seconds'] > charge['seconds_left']:
        raise L.BudgetExceeded(f'Step {step} will not fit what is left of its allowance')
    out = out_dir(step)
    if (out / 'charge.json').is_file():
        raise ValueError(f'Step {step} has already run ({out})')
    if P.set_priority(config['priority']) != config['priority']:
        raise RuntimeError(f'The learner does not run at {config["priority"]} priority')
    from traffic import highway_flow_rl_v14_contract as contract
    from traffic import v15_benchmark
    dev_rows = v15_benchmark.development_rows(contract.build_config(P3.ROUND_INDEX))
    out.mkdir(parents=True, exist_ok=True)
    (out / 'resolved-config.json').write_text(json.dumps(config, indent=1, sort_keys=True))
    log_path = out / 'phase4-log.jsonl'

    def log(entry):
        with open(log_path, 'a', encoding='utf-8') as handle:
            handle.write(json.dumps(entry) + '\n')
        print(json.dumps(entry), flush=True)

    with L.EvaluationSession(phase='4', output=out, config_sha256=sha, expected_decisions=charge['expected_decisions'],
                             decision_allowance=ALLOWANCE, expected_clock_seconds=charge['expected_clock_seconds'],
                             reserve_seconds=charge['reserve_seconds'], total_wall_seconds=charge['total_wall_seconds'],
                             intended_episodes=charge['expected_episodes'], workers=config['workers'],
                             priority=config['priority'], authorization=config['authorization'],
                             round_index=config['round_index']) as session:
        # training_episodes: experiments/v15_phase3._add moves training-row episodes there (none here); the probe's
        # first attempt failed without it, after its first batch was charged
        session.segment.update(phase4_step=step, v15_work=['evaluation'], training_episodes=0)
        with P.WorkerPool(config['workers'], out, backend=config['backend'], round_index=config['round_index'],
                          priority=config['priority']) as pool:
            session.segment['worker_priorities'] = sorted(set(pool.worker_priorities.values()))
            pool.charged_by = f'cost-ledger {L.COMMAND} phase 4 step {step}, started_at_unix {session.segment["started_at_unix"]}'
            expected_so_far = charged_so_far = 0
            for index, batch in enumerate(config['batches']):
                tasks = batch['tasks']
                P3._check_policies(out, tasks)
                ratio = max(1., charged_so_far / expected_so_far) if expected_so_far else 1.
                projected = batch['expected_decisions'] * ratio * (1 + P3.MARGIN)
                session.check_next(projected)
                pool_tasks = [P3._task(P, t, f'phase4-{step}:{batch["name"]}') for t in tasks]
                try:
                    result = pool.run(pool_tasks)
                except P.CanaryMismatch as error:
                    if error.result is not None:
                        P3._add(session, error.result, pool_tasks)
                    else:
                        session.charge_estimate(projected, f'batch {index}: {error}')
                    raise
                except BaseException as error:
                    session.charge_estimate(projected, f'batch {index} failed before reporting: {type(error).__name__}')
                    raise
                P3._add(session, result, pool_tasks)
                expected_so_far += batch['expected_decisions']
                charged_so_far += result.charged_decisions
                parity = None
                for t, record in zip(tasks, result.records):
                    if t.get('parity'):
                        parity = (record['report_sha'], record['trace_sha']) == (config['parity']['report_sha'],
                                                                                 config['parity']['trace_sha'])
                log(dict(batch=index, name=batch['name'], tasks=len(pool_tasks), simulated=result.simulated_tasks,
                         clock_seconds=result.clock_seconds, worker_seconds=result.worker_seconds,
                         decisions=result.decisions, canary_decisions=result.canary_decisions,
                         expected_decisions=batch['expected_decisions'], canary=result.canary, parity_matched=parity,
                         elapsed_seconds=session.elapsed(), charged_decisions_so_far=session.segment['decisions']))
                if result.simulated_tasks and (result.canary is None or not result.canary['matched']):
                    raise P.CanaryMismatch(f'Batch {index}: canary {result.canary}')
                if parity is False:
                    raise P3.ParityMismatch(f'Batch {index}: l2:rule-v3 on development row {P3.PARITY_ROW} differs from '
                                            f'Phase 2 run 3\'s expert v3')
                found = P3._new_failures([r for t, r in zip(tasks, result.records) if not t.get('parity')], dev_rows)
                if found and index + 1 < len(config['batches']):
                    raise P3.StopAndAsk(f'Batch {index}: probe events under the stop rule (charged; the next batch was '
                                        f'not dispatched; nothing retuned): {found}')
    segment = session.segment
    summary = {k: segment[k] for k in ('status', 'charged_seconds', 'clock_seconds', 'decisions', 'canary_decisions',
                                       'evaluation_episodes', 'worker_seconds', 'simulated_seconds', 'workers', 'batches',
                                       'decision_allowance', 'expected_decisions', 'started_at_unix', 'config_sha256')}
    summary['ledger'] = str(session.path)
    (out / 'charge.json').write_text(json.dumps(summary, indent=1))
    result = report(step)
    _stop_on_probe_events(result)
    return result


def _stop_on_probe_events(result):
    """STOP_RULE for the probes: any collision or free-road stop, or a cut-in or emergency in a one-car LA case."""
    found = {name: p['new_failures'] for name, p in result['probes'].items() if p['new_failures']}
    if found:
        raise P3.StopAndAsk(f'Probe events under the stop rule (charged and reported; nothing retuned): {found}')


def speed_matches(run_dir, records, rules):
    """{row: {agent: speed match fraction}} of each record's graded cars under `rules` (experiments/v15_regrade)."""
    from experiments import v15_regrade as R
    out = {}
    for record in records:
        directory = Path(run_dir) / 'episodes' / record['key'][:16] / record['attempt']
        report = json.loads((directory / 'report.json').read_text(encoding='utf-8'))
        quality = R.episode_quality(R.episode_steps(directory, report), report, rules)
        out[record['fields']['row']] = {agent: round(g['speed_match_fraction'], 4) for agent, g in sorted(quality.items())}
    return out


def report(step):
    from experiments import v15_pool as P
    from experiments import v15_regrade as R
    from traffic import highway_flow_rl_v14_contract as contract
    from traffic import v15_benchmark
    out = out_dir(step)
    config = json.loads((out / 'resolved-config.json').read_text())
    dev_rows = v15_benchmark.development_rows(contract.build_config(P3.ROUND_INDEX))
    first = {}
    for record in P.read_results(out / 'results.jsonl'):
        first.setdefault(record['key'], record)
    records = list(first.values())
    drivers, parity = R.regrade_run(out, dict(none=(), rebaselined=R.RULES))
    rows = config['rows']
    run3 = P3._run3()
    expert_match = speed_matches(ROOT / P3.PHASE2_RUN3, [run3[r] for r in rows], R.RULES)
    probes = {}
    not_run = []
    for probe in sorted(config['level2']['probes']):
        mine = sorted((r for r in records if r['fields']['driver'] == probe), key=lambda r: r['fields']['row'])
        if not mine:
            not_run.append(probe)           # a run that stopped before the probe's batch
            continue
        if [r['fields']['row'] for r in mine] != rows:
            raise ValueError(f'{probe} has no record for every probed row')
        g = drivers[probe]
        match = speed_matches(out, mine, R.RULES)
        probes[probe] = dict(
            change=config['level2']['probes'][probe],
            rebaselined_passed=[r for r in rows if g['passed']['rebaselined'][r]],
            recorded_passed=[r for r in rows if g['passed']['none'][r]],
            rows={r: dict(rebaselined=g['passed']['rebaselined'][r], recorded=g['passed']['none'][r],
                          failures=g['failures']['rebaselined'][r], speed_match=match[r], decisions=rec['decisions'],
                          events={k: v for k, v in rec['events'].items() if v})
                  for r, rec in zip(rows, mine)},
            events=P3._events(mine), new_failures=P3._new_failures(mine, dev_rows), acted=_acted(mine))
        if [g['passed']['none'][r] for r in rows] != [bool(m['passed']) for m in mine]:
            raise ValueError(f'{probe}: the re-grade without rules does not reproduce the recorded verdicts')
    room = sorted({r for p in probes.values() for r in p['rebaselined_passed']})
    best = {}
    for r in rows:
        worst = [(min(p['rows'][r]['speed_match'].values()), name) for name, p in probes.items()
                 if p['rows'][r]['speed_match']]
        best[r] = max(worst) if worst else (None, None)
    parity_rows = [r for r in records if r['fields']['driver'] == config['parity']['driver']]
    result = dict(
        schema=SCHEMA + '_report', step=step, simulator='SUMO (libsumo); no physics', background=P3.BACKGROUND,
        rows=rows, start_policy=config['start_policy'], probes=probes, probes_not_run=not_run, rows_with_room=room,
        expert_speed_match=expert_match, best_probe_speed_match={r: dict(probe=best[r][1], worst_car=best[r][0])
                                                                 for r in rows},
        gate=SPEED_MATCH_GATE, regrade_parity=parity,
        parity=[dict(row=r['fields']['row'], matched=(r['report_sha'], r['trace_sha']) ==
                     (config['parity']['report_sha'], config['parity']['trace_sha'])) for r in parity_rows],
        charge=json.loads((out / 'charge.json').read_text()) if (out / 'charge.json').is_file() else None,
        held_out='not built, not run')
    (out / 'report.json').write_text(json.dumps(result, indent=1))
    (out / 'report.md').write_text(_markdown(result))
    return result


def _acted(records):
    totals = {}
    for record in records:
        for k, v in ((record.get('level2') or {}).get('acted') or {}).items():
            totals[k] = totals.get(k, 0) + v
    return totals


def _markdown(result):
    lines = [f'# v15 Phase 4 step A: the probe', '', f'{result["simulator"]}; background {result["background"]}; '
             f'development rows {result["rows"]} (the starting policy\'s and expert v3\'s failing rows).', '',
             f'Rows a probe passes (re-baselined): {result["rows_with_room"] or "none"}.', '',
             '| Probe | Passes (re-baselined) | Passes (recorded) | New failures |', '|---|---|---|---|']
    for name, p in result['probes'].items():
        lines.append(f'| {name} | {p["rebaselined_passed"]} | {p["recorded_passed"]} | {len(p["new_failures"])} |')
    lines += ['', '| Row | Expert v3 worst car speed match | Best probe | Its worst car |', '|---|---|---|---|']
    for r in result['rows']:
        e = result['expert_speed_match'][r]
        b = result['best_probe_speed_match'][r]
        lines.append(f'| {r} | {min(e.values()) if e else None} | {b["probe"]} | {b["worst_car"]} |')
    lines += ['', f'Parity: {result["parity"]}', f'Charge: {result["charge"]}']
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
                              **{k: config['charge'][k] for k in ('expected_decisions', 'expected_episodes',
                                                                  'expected_clock_seconds', 'reserve_seconds',
                                                                  'decisions_left', 'seconds_left')})))
    elif args.command == 'run':
        result = run(args.step)
        print(json.dumps(dict(rows_with_room=result['rows_with_room'], parity=result['parity'],
                              probes={k: dict(rebaselined=v['rebaselined_passed'], recorded=v['recorded_passed'],
                                              new_failures=v['new_failures']) for k, v in result['probes'].items()})))
    else:
        report(args.step)
    return 0


if __name__ == '__main__':
    sys.exit(main())
