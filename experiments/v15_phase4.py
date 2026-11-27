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

The probe stopped at its stop rule after its first batch (free-road stops by the slower variants). Its second batch
(offsets +1 and +2) runs alone as step 'probe-faster': user decision of November 26, 2026, relayed by the main Claude
session, "Yes, run batch 2". It is charged to what is left of step A's allowance and projected at the first batch's
measured rate; if that projection does not fit, it stops before dispatch (the batch is not trimmed to fit).
  python experiments/v15_phase4.py resolve probe-faster
  python experiments/v15_phase4.py run probe-faster
  python experiments/v15_phase4.py report probe-faster   (also combines the first batch's report)

The offset check (step 'offset-check'): user decision of November 26, 2026, typed in the Claude side session, "Yes,
run the check". The +1 m/s variant (l2:probe-offset-p1) on all 70 development rows, with no training, before the
held-out check (Stage 5), as a new Phase 4 allowance of 140,000 decisions and 100 s with no raise of the study total.
It runs the 62 rows the probe did not, and takes the probe's second batch for the other 8 (same sources, settings and
benchmark, checked), with its own canary and parity row.
  python experiments/v15_phase4.py resolve offset-check
  python experiments/v15_phase4.py run offset-check
  python experiments/v15_phase4.py report offset-check
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
STEPS = ('probe', 'probe-faster', 'offset-check')
ALLOWANCE, TIME_CAP = 100_000, 150.          # step A's probe: Phase 4's first allowance
CHECK_ALLOWANCE, CHECK_TIME_CAP = 140_000, 100.   # the offset check's own allowance (Phase 4's second)
START_STEP = 'relabel'                       # the starting policy's Phase 3 step
SPEED_MATCH_GATE = 0.9
AUTHORIZATION = P3.AUTHORIZATION + ' ' + P3.STEP_A_AUTHORIZATION
SLOWER = ('l2:probe-gap-p1', 'l2:probe-offset-m1', 'l2:probe-offset-m2')   # can lengthen an episode
FASTER = ('l2:probe-offset-p1', 'l2:probe-offset-p2')                       # the probe's second batch
PROBE_SHA = '3c622365fffa03ba8a974d4edd0888d03fbc7c4d37b25c2838c06195d4b7b41a'  # the pinned probe configuration
BATCH2_AUTHORIZATION = ('Phase 4 step A, the probe\'s second batch: user decision of November 26, 2026, relayed by '
                        'the main Claude session: "Yes, run batch 2" (offsets +1 and +2 m/s on development rows 0, 8, '
                        '9, 14, 22, 24, 37 and 38 with its canary and parity row, after an uncharged runner change; '
                        'charged to step A\'s remaining allowance of 41,999 decisions and 102.3 s; if the projection '
                        'does not fit before dispatch, stop and ask, and the batch is not trimmed to fit; same settings '
                        'and stop rule; not authorized: step B, any raise of the study total, the held-out check, '
                        'Krauss runs, merging to main).')
STOP_RULE_FASTER = ('a canary or parity mismatch stops the run; so does any probe collision or free-road stop, or a '
                    'probe cut-in or emergency in a one-car LA case (rows 24 and 38 here): it is reported to the user '
                    'and nothing is retuned. The batch is one pool batch, so the stop comes after its charge and report')
CHECK_DRIVER = 'l2:probe-offset-p1'         # the offset check's variant: the rule's choices, a constant +1 m/s offset
FASTER_SHA = '2bceef41fbeb5a866a1b0be55373102dd1646d63768823f20e96ec7d452c1b83'  # the pinned second-batch configuration
CHECK_RATE_STEP = 'relabel'                 # the Phase 3 step whose measured clock rate sizes the check's clock time
CHECK_AUTHORIZATION = ('Phase 4 offset check: user decision of November 26, 2026, typed in the Claude side session: '
                       '"Yes, run the check" (the constant +1 m/s offset, l2:probe-offset-p1, on all 70 development rows '
                       'with no training, before the held-out check (Stage 5); a new Phase 4 allowance of about 140,000 '
                       'decisions and 100 s, with no raise of the study total; the main Claude session recorded the same '
                       'answer in the coordination master with an allowance of at most 160,000 decisions and 150 s, and '
                       'step B skipped; same settings as Phases 1-3: at most 16 '
                       'workers at below-normal priority, IDM background only, SUMO only, development rows only, the '
                       'shared lock, and the resolved configuration and expected charge in the coordination master '
                       'before it runs; not authorized: step B, any raise of the study total, the held-out check, Krauss '
                       'runs, merging to main).')
STOP_RULE_CHECK = ('a canary or parity mismatch stops the run; any collision or free-road stop by the +1 variant, or a '
                   'cut-in or emergency by it in a one-car LA case, is reported to the user and nothing is retuned. The '
                   'check is one pool batch, so the stop comes after its charge and report')
CHECK_READING = ('the +1 variant joins the held-out check (Stage 5) beside the Phase 3 policy only if it passes every '
                 'development row expert v3 passes under the frozen re-baselined gates and has no stop-rule event; each '
                 'row it gains or loses against expert v3 and against the Phase 3 policy is reported with its failures '
                 'and speed match, and nothing is retuned')
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


def _projected(expected_decisions, measured_ratio=1.):
    """A batch's projected charge: its expected decisions at the measured rate (never below 1) plus the margin."""
    return expected_decisions * max(1., float(measured_ratio)) * (1 + P3.MARGIN)


def _check_fit(charge):
    """Stop before dispatch when the projected charge or the expected clock time doesn't fit what is left."""
    from experiments import v15_ledger as L
    projected = charge.get('projected_decisions', _projected(charge['expected_decisions']))
    if projected > charge['decisions_left'] or charge['expected_clock_seconds'] > charge['seconds_left']:
        raise L.BudgetExceeded(f'Projected {projected:,.0f} decisions and {charge["expected_clock_seconds"]} s; '
                               f'{charge["decisions_left"]:,} decisions and {charge["seconds_left"]} s are left. Stop and '
                               'ask: the batch is not trimmed to fit')


def _probe_segment():
    """The probe's ledger segment (Phase 4's first), which failed after its first batch was charged."""
    from experiments import v15_ledger as L
    ledger = json.loads((L.study_directory() / 'cost-ledger.json').read_text())
    output = str(out_dir('probe').relative_to(ROOT)).replace('\\', '/')
    found = [s for s in ledger['segments'] if s.get('v15_phase') == '4' and s.get('output') == output]
    if len(found) != 1:
        raise ValueError(f'Expected one ledger segment for {output}, found {len(found)}')
    return found[0]


def _resolve_faster():
    """The probe's second batch alone, after the first batch stopped at the stop rule (no simulation).

    Its tasks are the pinned probe configuration's second batch, plus the canary and the parity row. The sources,
    rows and settings must be the first batch's, so the two batches compare.
    """
    from experiments import v15_pool as P
    from traffic import highway_flow_rl_v14_contract as contract
    from traffic import v15_benchmark, v15_level2
    first = json.loads(config_path('probe').read_text())
    if _json_sha(first) != PROBE_SHA:
        raise ValueError(f'{config_path("probe")} is not the probe configuration that ran ({PROBE_SHA[:8]})')
    batch = first['batches'][1]
    if batch['name'] != 'probe-faster' or tuple(batch['drivers']) != FASTER:
        raise ValueError('The probe configuration\'s second batch is not the faster variants')
    done = P.read_results(out_dir('probe') / 'results.jsonl')
    ran = sorted((r['fields']['driver'], r['fields']['row']) for r in done if r['fields']['driver'] in SLOWER)
    if ran != sorted((d, r) for d in SLOWER for r in first['rows']):
        raise ValueError('The probe\'s first batch is incomplete')
    if any(r['fields']['driver'] in FASTER for r in done):
        raise ValueError('The probe\'s second batch has already run')
    segment = _probe_segment()
    if segment['status'] != 'failed' or segment['batches'] != 1:
        raise ValueError(f'The probe\'s segment is {segment["status"]} with {segment["batches"]} batches')
    cfg = contract.build_config(P3.ROUND_INDEX)
    level2 = dict(rule=v15_level2.RULE_DRIVER, offsets=list(v15_level2.OFFSETS), gaps=v15_level2.GAPS,
                  settings_digest=v15_level2.settings_digest(),
                  sources={p: _file_sha(ROOT / p) for p in first['level2']['sources']})
    if {k: first['level2'][k] for k in level2} != level2 or v15_benchmark.manifest(cfg) != first['benchmark']:
        raise ValueError('The Level 2 sources, settings or benchmark differ from the probe\'s first batch')
    level2['probes'] = {p: first['level2']['probes'][p] for p in FASTER}
    parity = first['parity']
    tasks = list(batch['tasks']) + [dict(parity, row_set=P3.DEV, mode='validate', parity=True)]
    expected = batch['expected_decisions'] + parity['decisions']
    measured = segment['decisions'] / first['batches'][0]['expected_decisions']
    longest = max(first['expert_decisions'].values())
    clock = P3.START_S + max(expected * P3.PER_DECISION_S, longest / 140.) + 10.
    phase, totals = _ledger_phase4()
    decisions_left, seconds_left = ALLOWANCE - phase['decisions'], TIME_CAP - phase['seconds']
    study_left = cfg['budget']['total_wall_seconds'] - totals['total']
    reserve = min(round(clock * 1.6 + 30.), round(seconds_left, 3), round(study_left, 3))
    return dict(
        schema=SCHEMA, phase='4', step='probe-faster', round_index=first['round_index'], backend=first['backend'],
        workers=first['workers'], priority=first['priority'], backgrounds=first['backgrounds'],
        benchmark=first['benchmark'], level2=level2, start_policy=first['start_policy'], rows=first['rows'],
        rows_rule=first['rows_rule'], expert_decisions=first['expert_decisions'],
        batches=[dict(name=batch['name'], drivers=list(FASTER), tasks=tasks, expected_decisions=expected)],
        parity=parity, reading=first['reading'],
        continues=dict(step='probe', config_sha256=PROBE_SHA, output=segment['output'],
                       stopped='at the stop rule after its first batch (free-road stops by offsets -1 and -2; the '
                               'segment failed on a KeyError after that batch was charged, fixed in 1f57be7)',
                       first_batch_expected_decisions=first['batches'][0]['expected_decisions'],
                       first_batch_charged_decisions=segment['decisions'], measured_ratio=round(measured, 6)),
        stop_checks=dict(any_driver=STOP_RULE_FASTER, one_car_la_cases=list(P3.ONE_CAR_LA)),
        charge=dict(expected_decisions=expected, projected_decisions=round(_projected(expected, measured)),
                    measured_ratio=round(measured, 6), expected_episodes=len(tasks) + 1,
                    expected_clock_seconds=round(clock, 1), reserve_seconds=reserve, phase_allowance=ALLOWANCE,
                    phase_time_cap=TIME_CAP, phase_decisions_used=phase['decisions'],
                    phase_seconds_used=round(phase['seconds'], 3), decisions_left=decisions_left,
                    seconds_left=round(seconds_left, 3), study_seconds_left=round(study_left, 3),
                    guard_margin=P3.MARGIN,
                    estimate_source=dict(per_row='expert v3\'s decisions on each row, Phase 2 run 3',
                                         rate='the probe\'s first batch: charged decisions over expected',
                                         longest_episode_decisions=longest, decisions_per_worker_second=140,
                                         start_seconds=P3.START_S),
                    total_wall_seconds=cfg['budget']['total_wall_seconds']),
        authorization=AUTHORIZATION + ' ' + BATCH2_AUTHORIZATION, held_out='not built, not run',
        simulator='SUMO (libsumo); no physics')


def _segments(phase, output_dir):
    """The ledger segments of a v15 phase charged to an output directory."""
    from experiments import v15_ledger as L
    ledger = json.loads((L.study_directory() / 'cost-ledger.json').read_text())
    output = str(Path(output_dir).relative_to(ROOT)).replace('\\', '/')
    return [s for s in ledger['segments'] if s.get('v15_phase') == phase and s.get('output') == output]


def _canary_index(tasks, horizon):
    """The task the pool's canary re-runs (experiments/v15_pool.WorkerPool.run): the shortest horizon, then the first."""
    return min(range(len(tasks)), key=lambda i: (horizon[tasks[i]['row']], i))


def _measured_rate():
    """Decisions per second of clock time in Phase 3's slowest one-batch development evaluation (step relabel: 70 rows,
    the parity row and a canary at 16 workers), less its fit time. Its pool started during the fit, so the estimate
    adds the start time back."""
    found = [s for s in _segments('3', P3.out_dir(CHECK_RATE_STEP)) if s['status'] == 'complete']
    if len(found) != 1:
        raise ValueError(f'Expected one complete ledger segment for Phase 3 step {CHECK_RATE_STEP}, found {len(found)}')
    s = found[0]
    seconds = float(s['charged_seconds']) - float(s['fit_seconds'])
    return dict(step=CHECK_RATE_STEP, decisions=s['decisions'], seconds=round(seconds, 3),
                decisions_per_clock_second=round(s['decisions'] / seconds, 1))


def _check_summary(rows, expert, start, new_failures):
    """The offset check's verdicts against expert v3's and the Phase 3 policy's, row by row.

    rows, expert, start: {row: dict(rebaselined=bool, recorded=bool)} for every development row."""
    if sorted(rows) != sorted(expert) or sorted(rows) != sorted(start):
        raise ValueError('The check, expert v3 and the Phase 3 policy must cover the same rows')
    order = sorted(rows)

    def against(other):
        return {f'{change}{suffix}': [r for r in order if rows[r][gate] == (change == 'gained') and
                                      other[r][gate] == (change == 'lost')]
                for change in ('gained', 'lost') for gate, suffix in (('rebaselined', ''), ('recorded', '_recorded'))}

    vs_expert = against(expert)
    return dict(rebaselined_passed=[r for r in order if rows[r]['rebaselined']],
                recorded_passed=[r for r in order if rows[r]['recorded']],
                vs_expert=vs_expert, vs_start=against(start),
                joins_stage5=not vs_expert['lost'] and not new_failures)


def _start_rows():
    """The Phase 3 policy's verdicts on every development row: re-baselined from its report's failing rows, recorded
    from its run's first records."""
    from experiments import v15_pool as P
    start = P3._report(START_STEP)['learners'][0]
    failing = {f['row'] for f in start['failing_rows']}
    records = {r['fields']['row']: r for r in P.read_results(P3.out_dir(START_STEP) / 'results.jsonl')
               if r['attempt'] == 'first' and r['fields']['driver'] == start['id'] and r['fields']['row_set'] == P3.DEV}
    rows = {r: dict(rebaselined=r not in failing, recorded=bool(records[r]['passed'])) for r in sorted(records)}
    if (len(rows), sum(v['rebaselined'] for v in rows.values()), sum(v['recorded'] for v in rows.values())) != \
            (70, start['rebaselined_passed'], start['recorded_passed']):
        raise ValueError('The Phase 3 policy\'s rows do not reproduce its report')
    return start, rows


def _resolve_check():
    """The +1 variant on all 70 development rows (no simulation): the 62 rows the probe did not run, with a canary and
    the parity row; the probe's second batch supplies the other 8, so its sources, settings and benchmark must match."""
    from traffic import highway_flow_rl_v14_contract as contract
    from traffic import v15_benchmark, v15_level2
    faster = json.loads(config_path('probe-faster').read_text())
    if _json_sha(faster) != FASTER_SHA:
        raise ValueError(f'{config_path("probe-faster")} is not the configuration that ran ({FASTER_SHA[:8]})')
    found = _segments('4', out_dir('probe-faster'))
    if [s['status'] for s in found] != ['complete']:
        raise ValueError(f'The probe\'s second batch has segments {[s["status"] for s in found]}, not one complete')
    report_path = out_dir('probe-faster') / 'report.json'
    earlier = json.loads(report_path.read_text())
    mine = earlier['probes'][CHECK_DRIVER]
    reused_rows = sorted(int(r) for r in mine['rows'])
    if reused_rows != faster['rows'] or mine['new_failures'] or not earlier['parity'] or \
            not all(p['matched'] for p in earlier['parity']):
        raise ValueError('The probe\'s second batch does not cover its rows cleanly for the +1 variant')
    cfg = contract.build_config(P3.ROUND_INDEX)
    level2 = dict(rule=v15_level2.RULE_DRIVER, offsets=list(v15_level2.OFFSETS), gaps=v15_level2.GAPS,
                  settings_digest=v15_level2.settings_digest(),
                  sources={p: _file_sha(ROOT / p) for p in faster['level2']['sources']})
    if {k: faster['level2'][k] for k in level2} != level2 or v15_benchmark.manifest(cfg) != faster['benchmark']:
        raise ValueError('The Level 2 sources, settings or benchmark differ from the probe\'s second batch')
    level2['probes'] = {CHECK_DRIVER: faster['level2']['probes'][CHECK_DRIVER]}
    dev = v15_benchmark.development_rows(cfg)
    rows = list(range(len(dev)))
    if len(rows) != 70:
        raise ValueError(f'Expected 70 development rows, found {len(rows)}')
    run_rows = [r for r in rows if r not in reused_rows]
    run3 = P3._run3()
    per_row = {str(r): run3[r]['decisions'] for r in run_rows}      # string keys, as the pinned JSON holds them
    parity = faster['parity']
    tasks = [dict(row_set=P3.DEV, row=r, driver=CHECK_DRIVER) for r in run_rows]
    tasks.append(dict(parity, row_set=P3.DEV, mode='validate', parity=True))
    canary = tasks[_canary_index(tasks, {r: dev[r]['horizon_steps'] for r in rows})]
    canary_decisions = run3[canary['row']]['decisions']
    expected = sum(per_row.values()) + parity['decisions'] + canary_decisions
    # the +1 variant's decisions over expert v3's on the probe's rows (the projection never goes below 1)
    measured = sum(mine['rows'][str(r)]['decisions'] for r in reused_rows) / sum(run3[r]['decisions'] for r in reused_rows)
    rate = _measured_rate()
    longest = max(per_row.values())
    clock = P3.START_S + max(expected / rate['decisions_per_clock_second'], longest / 140.) + 10.
    used = _segments('4', out_dir('offset-check'))
    decisions_left = CHECK_ALLOWANCE - sum(s['decisions'] for s in used)
    seconds_left = CHECK_TIME_CAP - sum(float(s['charged_seconds']) for s in used)
    phase, totals = _ledger_phase4()
    study_left = cfg['budget']['total_wall_seconds'] - totals['total']
    reserve = min(round(clock * 1.6 + 30.), round(seconds_left, 3), round(study_left, 3))
    start, start_rows = _start_rows()
    la_rows = [r for r in rows if dev[r]['case']['id'] in P3.ONE_CAR_LA]
    return dict(
        schema=SCHEMA, phase='4', step='offset-check', round_index=faster['round_index'], backend=faster['backend'],
        workers=faster['workers'], priority=faster['priority'], backgrounds=faster['backgrounds'],
        benchmark=faster['benchmark'], level2=level2,
        start_policy=dict(id=start['id'], step=START_STEP, rebaselined_passed=start['rebaselined_passed'],
                          recorded_passed=start['recorded_passed'],
                          rows_sha256=_json_sha({str(k): v for k, v in start_rows.items()})),
        rows=rows, run_rows=run_rows, reused_rows=reused_rows,
        reused=dict(step='probe-faster', config_sha256=FASTER_SHA, output=found[0]['output'],
                    report_sha256=_file_sha(report_path), driver=CHECK_DRIVER),
        rows_rule='every development row: the 62 the probe did not run here, the other 8 from the probe\'s second batch',
        expert_decisions=per_row,
        batches=[dict(name='offset-check', drivers=[CHECK_DRIVER], tasks=tasks, expected_decisions=expected)],
        parity=parity, reading=CHECK_READING,
        stop_checks=dict(any_driver=STOP_RULE_CHECK, one_car_la_cases=list(P3.ONE_CAR_LA), one_car_la_rows=la_rows),
        charge=dict(expected_decisions=expected, projected_decisions=round(_projected(expected, measured)),
                    measured_ratio=round(measured, 6), expected_episodes=len(tasks) + 1,
                    expected_clock_seconds=round(clock, 1), reserve_seconds=reserve, phase_allowance=CHECK_ALLOWANCE,
                    phase_time_cap=CHECK_TIME_CAP, session_allowance=ALLOWANCE + CHECK_ALLOWANCE,
                    phase_decisions_used=phase['decisions'], phase_seconds_used=round(phase['seconds'], 3),
                    check_decisions_used=CHECK_ALLOWANCE - decisions_left,
                    decisions_left=decisions_left, seconds_left=round(seconds_left, 3),
                    study_seconds_left=round(study_left, 3), guard_margin=P3.MARGIN,
                    estimate_source=dict(per_row='expert v3\'s decisions on each row, Phase 2 run 3',
                                         canary=dict(row=canary['row'], decisions=canary_decisions,
                                                     rule='the shortest-horizon task, then the first'),
                                         ratio='the +1 variant\'s decisions over expert v3\'s on the probe\'s 8 rows',
                                         rate=rate, longest_episode_decisions=longest,
                                         decisions_per_worker_second=140, start_seconds=P3.START_S),
                    total_wall_seconds=cfg['budget']['total_wall_seconds']),
        authorization=CHECK_AUTHORIZATION, held_out='not built, not run', simulator='SUMO (libsumo); no physics')


def resolve(step):
    """The probe's resolved configuration and expected charge (no simulation)."""
    from traffic import highway_flow_rl_v14_contract as contract
    from traffic import v15_benchmark, v15_level2
    if step not in STEPS:
        raise ValueError(f'Unknown step {step!r}; expected one of {STEPS}')
    if step == 'probe-faster':
        return _resolve_faster()
    if step == 'offset-check':
        return _resolve_check()
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


def _not_run_yet(step):
    out = out_dir(step)
    if (out / 'charge.json').is_file() or (out / 'results.jsonl').is_file():
        raise ValueError(f'Step {step} has already run ({out}); the probe\'s second batch runs as step probe-faster')


def run(step):
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
    _check_fit(charge)
    out = out_dir(step)
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
                             decision_allowance=charge.get('session_allowance', ALLOWANCE),
                             expected_clock_seconds=charge['expected_clock_seconds'],
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
                # the measured rate so far; a continuation starts at its earlier batch's measured rate
                ratio = charged_so_far / expected_so_far if expected_so_far else charge.get('measured_ratio', 1.)
                projected = _projected(batch['expected_decisions'], ratio)
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
    if step == 'offset-check':
        return _report_check(out, config)
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
    if config.get('continues'):
        result['combined'] = _combined(json.loads((out_dir(config['continues']['step']) / 'report.json').read_text()),
                                       result)
    (out / 'report.json').write_text(json.dumps(result, indent=1))
    (out / 'report.md').write_text(_markdown(result))
    return result


def _report_check(out, config):
    """The +1 variant on all 70 development rows: this run's 62 and the probe's second batch's 8, re-graded."""
    from experiments import v15_pool as P
    from experiments import v15_regrade as R
    from traffic import highway_flow_rl_v14_contract as contract
    from traffic import v15_benchmark
    dev_rows = v15_benchmark.development_rows(contract.build_config(P3.ROUND_INDEX))
    first = {}
    for record in P.read_results(out / 'results.jsonl'):
        first.setdefault(record['key'], record)
    records = list(first.values())
    drivers, parity = R.regrade_run(out, dict(none=(), rebaselined=R.RULES))
    mine = sorted((r for r in records if r['fields']['driver'] == CHECK_DRIVER), key=lambda r: r['fields']['row'])
    if [r['fields']['row'] for r in mine] != config['run_rows']:
        raise ValueError(f'{CHECK_DRIVER} has no record for every row the check runs')
    g = drivers[CHECK_DRIVER]
    if [g['passed']['none'][r['fields']['row']] for r in mine] != [bool(r['passed']) for r in mine]:
        raise ValueError(f'{CHECK_DRIVER}: the re-grade without rules does not reproduce the recorded verdicts')
    match = speed_matches(out, mine, R.RULES)
    rows = {}
    for rec in mine:
        r = rec['fields']['row']
        rows[r] = dict(rebaselined=g['passed']['rebaselined'][r], recorded=g['passed']['none'][r],
                       failures=g['failures']['rebaselined'][r], speed_match=match[r], decisions=rec['decisions'],
                       events={k: v for k, v in rec['events'].items() if v}, source='offset-check')
    reused_path = out_dir(config['reused']['step']) / 'report.json'
    if _file_sha(reused_path) != config['reused']['report_sha256']:
        raise ValueError(f'{reused_path} is not the report the configuration pinned')
    reused = json.loads(reused_path.read_text())['probes'][CHECK_DRIVER]
    for r in config['reused_rows']:
        rows[r] = dict(reused['rows'][str(r)], source=config['reused']['step'])
    rows = {r: rows[r] for r in sorted(rows)}
    if list(rows) != config['rows']:
        raise ValueError('The check does not cover every development row once')
    new_failures = P3._new_failures(mine, dev_rows) + list(reused['new_failures'])
    run3 = P3._run3()
    expert_rb = P3._expert_rebaselined(ROOT / P3.PHASE2_RUN3 / 'rebaseline' / 'rebaseline.json')
    expert = {r: dict(rebaselined=bool(expert_rb[r]), recorded=bool(run3[r]['passed'])) for r in rows}
    start, start_rows = _start_rows()
    if _json_sha({str(k): v for k, v in start_rows.items()}) != config['start_policy']['rows_sha256']:
        raise ValueError('The Phase 3 policy\'s rows differ from the pinned configuration')
    summary = _check_summary(rows, expert, start_rows, new_failures)
    expert_match = speed_matches(ROOT / P3.PHASE2_RUN3, [run3[r] for r in rows], R.RULES)
    changed = sorted(set().union(*summary['vs_expert'].values(), *summary['vs_start'].values()))
    events = {}
    for row in rows.values():
        for k, v in row['events'].items():
            slot = events.setdefault(k, dict(rows=0, count=0))
            slot['rows'] += int(v > 0)
            slot['count'] += int(v)
    parity_rows = [r for r in records if r['fields']['driver'] == config['parity']['driver']]
    result = dict(
        schema=SCHEMA + '_report', step='offset-check', simulator='SUMO (libsumo); no physics', background=P3.BACKGROUND,
        driver=CHECK_DRIVER, change=config['level2']['probes'][CHECK_DRIVER], rows=config['rows'],
        run_rows=config['run_rows'], reused_rows=config['reused_rows'], start_policy=config['start_policy'],
        **summary, events=events, new_failures=new_failures, acted=_acted(mine), reading=config['reading'],
        changed_rows={r: dict(check=dict(rebaselined=rows[r]['rebaselined'], recorded=rows[r]['recorded'],
                                         failures=rows[r]['failures'], speed_match=rows[r]['speed_match'],
                                         events=rows[r]['events'], source=rows[r]['source']),
                              expert=dict(expert[r], speed_match=expert_match[r]), start=start_rows[r],
                              case=dev_rows[r]['case']['id'])
                      for r in changed},
        worst_car_speed_match={r: dict(check=min(rows[r]['speed_match'].values()) if rows[r]['speed_match'] else None,
                                       expert=min(expert_match[r].values()) if expert_match[r] else None)
                               for r in rows},
        failing_rows={r: rows[r]['failures'] for r in rows if not rows[r]['rebaselined']},
        rows_detail=rows, gate=SPEED_MATCH_GATE, regrade_parity=parity,
        parity=[dict(row=r['fields']['row'], matched=(r['report_sha'], r['trace_sha']) ==
                     (config['parity']['report_sha'], config['parity']['trace_sha'])) for r in parity_rows],
        charge=json.loads((out / 'charge.json').read_text()) if (out / 'charge.json').is_file() else None,
        held_out='not built, not run')
    # the stop rule and main() read the variant's results the way they read the probe's
    result['probes'] = {CHECK_DRIVER: dict(rebaselined_passed=summary['rebaselined_passed'],
                                           recorded_passed=summary['recorded_passed'], new_failures=new_failures)}
    result['rows_with_room'] = summary['vs_expert']['gained']
    (out / 'report.json').write_text(json.dumps(result, indent=1))
    (out / 'report.md').write_text(_markdown_check(result))
    return result


def _markdown_check(result):
    s = result
    lines = [f'# v15 Phase 4: the +1 offset check on all 70 development rows', '',
             f'{s["simulator"]}; background {s["background"]}; {s["driver"]}: {s["change"]}. Rows run here: '
             f'{len(s["run_rows"])}; from the probe\'s second batch: {s["reused_rows"]}.', '',
             f'Passes: {len(s["rebaselined_passed"])}/70 re-baselined, {len(s["recorded_passed"])}/70 recorded '
             f'(the Phase 3 policy: {s["start_policy"]["rebaselined_passed"]} and {s["start_policy"]["recorded_passed"]}).',
             '', f'Against expert v3: {s["vs_expert"]}', f'Against the Phase 3 policy: {s["vs_start"]}', '',
             f'Stop-rule events: {s["new_failures"] or "none"}. Events: {s["events"] or "none"}.',
             f'Joins the held-out check under the pre-set reading: {s["joins_stage5"]}.', '',
             '| Row | Case | Check (re-baselined, recorded) | Expert v3 | Phase 3 policy | Check failures | '
             'Worst-car speed match (check, expert) |', '|---|---|---|---|---|---|---|']
    for r, c in s['changed_rows'].items():
        w = s['worst_car_speed_match'][r]
        lines.append(f'| {r} | {c["case"]} | {c["check"]["rebaselined"]}, {c["check"]["recorded"]} | '
                     f'{c["expert"]["rebaselined"]}, {c["expert"]["recorded"]} | {c["start"]["rebaselined"]}, '
                     f'{c["start"]["recorded"]} | {c["check"]["failures"]} | {w["check"]}, {w["expert"]} |')
    lines += ['', f'Parity: {s["parity"]}', f'Charge: {s["charge"]}']
    return '\n'.join(lines) + '\n'


def _combined(earlier, later):
    """Both batches of the probe together: the variants, the rows with room and the best worst-car speed match."""
    probes = dict(earlier['probes'], **later['probes'])
    best = {}
    for r in later['rows']:
        options = [(b['worst_car'], b['probe']) for b in (earlier['best_probe_speed_match'][str(r)],
                                                          later['best_probe_speed_match'][r]) if b['probe']]
        top = max(options) if options else (None, None)
        best[r] = dict(probe=top[1], worst_car=top[0])
    return dict(probes={k: dict(rebaselined_passed=v['rebaselined_passed'], recorded_passed=v['recorded_passed'],
                                new_failures=v['new_failures']) for k, v in sorted(probes.items())},
                rows_with_room=sorted(set(earlier['rows_with_room']) | set(later['rows_with_room'])),
                best_probe_speed_match=best, not_run=sorted(set(earlier['probes_not_run']) - set(later['probes'])))


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
