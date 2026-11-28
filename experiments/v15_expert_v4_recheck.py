"""Plan v15 decision 14, step 2: the 70-row SUMO re-check of expert v4 and of the policy on v4's executor. Sized and
configured here; NOT run.

User decision of November 27, 2026, relayed by the main Claude session: "Yes, as recommended" (decisions 13-20), with
decision 14 step 2 not authorized: "Size it and present the pinned config and the expected charge for the go-ahead."
So this script only resolves the configuration and its expected charge from recorded files (no simulation) and
writes it in the slim format (experiments/v15_runner.py). `run` refuses. If the user gives the go-ahead, the run and
report code (a pool extension that registers the v4 drivers without editing experiments/v15_pool.py, the no-regression
evaluation) is written and tested first, uncharged, against this configuration.

What would run (SUMO, libsumo, IDM background; development rows only, never the held-out rows):
- 'rule:expert-v4' (traffic/v15_rule_expert_v4.py) and 'l2v4:bc7c050f6d3da3ce' (the Phase 3 policy's Level 2 network
  on v4's executor, traffic/v15_level2_v4.py) on each of the 70 development rows;
- the parity row of every v15 check: 'l2:rule-v3' on row 52 must reproduce expert v3's recorded trace (the pool);
- a v4 parity row: 'l2v4:rule-v4' on row 52 must reproduce 'rule:expert-v4''s trace from the same run (the Level 2
  rule on v4's executor reproduces expert v4, as 'l2:rule-v3' reproduces expert v3);
- the pool's canary (it re-runs the shortest-horizon task).

The expected charge: each row's decisions as recorded for expert v3 (Phase 2 run 3) and for the policy (Phase 3 step
relabel), at the clock rate measured in Phase 3's step relabel. v4's own episode lengths are unmeasured; the offline
replay (experiments/v15_expert_v4_replay.py, post hoc) projects its stream-car crossings within 0.6 s of v3's.

Usage: python experiments/v15_expert_v4_recheck.py resolve   (writes configs/evaluation/slim/v15-expert-v4-recheck.json)
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from experiments import v15_runner as R                       # noqa: E402

SCHEMA = 'highway_merge_v15_expert_v4_recheck_v01'
PHASE, STEP = '7', 'recheck'                     # the next v15 ledger phase (6 is the physics gap)
CONFIG = ROOT / 'configs/evaluation/slim/v15-expert-v4-recheck.json'   # slim configurations sit apart from the 18 recorded full ones
EXPERT_V4 = 'rule:expert-v4'
POLICY, POLICY_V4 = 'l2:bc7c050f6d3da3ce', 'l2v4:bc7c050f6d3da3ce'
RULE_V3, RULE_V4 = 'l2:rule-v3', 'l2v4:rule-v4'
DRIVERS = (EXPERT_V4, POLICY_V4)
PARITY_ROW = 52                                   # experiments/v15_phase3.PARITY_ROW
WORKERS, PRIORITY = 16, 'below_normal'            # the user's CPU limit
START_S, SETTLE_S, PER_WORKER_RATE = 15., 10., 140.   # experiments/v15_phase4._resolve_check's clock estimate
AUTHORIZATION = ('not authorized: decision 14 step 2 needs the user\'s go-ahead ("Size it and present the pinned '
                 'config and the expected charge for the go-ahead", relayed by the main Claude session, November 27, '
                 '2026)')
# Set in the write-up before any run (plan v15, "Ramp driving", option A, "The no-regression rule, set now")
NO_REGRESSION = dict(
    rows=('neither driver loses a development row it passes today, under either gate set (54 recorded, 62 '
          're-baselined)'),
    events='no new event kind, in particular no native emergency and no cut-in',
    movement=('fewer cars fail the movement checks than today\'s 23 of 31, and none of the 7 clear cars does '
              '(M1, M2 and M4, traffic/v15_movement.py; M3 reported alongside)'),
    unchanged='rows where the change never acts reproduce their traces exactly',
    drivers=dict(compared={EXPERT_V4: 'rule:expert-v3', POLICY_V4: POLICY}, matched_rows='every development row'),
    situations='reported separately: clear, stream, blocked, i110 (decision 15)')
READING = ('Option A passes if both drivers keep the no-regression rule. The offline replay (post hoc, open loop) '
           'expects it not to: on the 12 stream cars v4 crosses when v3 does (median 19.0 s) and fails the movement '
           'checks on 12, v3 on 11.')


def tasks_for(rows, horizons):
    """The batch's tasks in dispatch order: per row, expert v4 then the policy on v4; then the two parity rows."""
    tasks = [dict(row_set='v15-development', row=r, driver=d) for r in rows for d in DRIVERS]
    tasks.append(dict(row_set='v15-development', row=PARITY_ROW, driver=RULE_V3, parity='expert v3 recorded'))
    tasks.append(dict(row_set='v15-development', row=PARITY_ROW, driver=RULE_V4, parity=f'{EXPERT_V4} in this run'))
    canary = min(range(len(tasks)), key=lambda i: (horizons[tasks[i]['row']], i))    # experiments/v15_pool's rule
    return tasks, canary


def charge(per_row, tasks, canary, rate, study_left, total_wall):
    """The expected charge from recorded decisions: per_row = {driver: {row: decisions}} with each v4 driver's v3
    counterpart's recorded decisions."""
    expected = sum(per_row[t['driver']][t['row']] for t in tasks) + per_row[tasks[canary]['driver']][tasks[canary]['row']]
    longest = max(max(v.values()) for v in per_row.values())
    clock = START_S + max(expected / rate['decisions_per_clock_second'], longest / PER_WORKER_RATE) + SETTLE_S
    return dict(expected_decisions=expected, projected_decisions=round(R.projected(expected)),
                expected_episodes=len(tasks) + 1, expected_clock_seconds=round(clock, 1),
                reserve_seconds=round(min(clock * 1.6 + 30., study_left), 3), guard_margin=R.MARGIN,
                proposed_allowance=dict(decisions=-(-round(R.projected(expected)) // 10_000) * 10_000, seconds=300.),
                study_seconds_left=round(study_left, 3), total_wall_seconds=total_wall, raise_needed=False,
                estimate_source=dict(per_row='expert v3: Phase 2 run 3; the policy: Phase 3 step relabel (first '
                                             'attempts); each v4 driver at its v3 counterpart\'s decisions',
                                     canary=dict(task=canary, row=tasks[canary]['row'], driver=tasks[canary]['driver'],
                                                 rule='the shortest-horizon task, then the first'),
                                     rate=rate, longest_episode_decisions=longest,
                                     decisions_per_worker_second=PER_WORKER_RATE, start_seconds=START_S))


def _recorded():
    """Recorded per-row decisions: expert v3 (Phase 2 run 3) and the policy (Phase 3 step relabel)."""
    from experiments import v15_phase3 as P3
    from experiments import v15_pool as P
    run3 = P3._run3()
    policy = {x['fields']['row']: x for x in P.read_results(P3.out_dir('relabel') / 'results.jsonl')
              if x['attempt'] == 'first' and x['fields']['driver'] == POLICY and x['fields']['row_set'] == P3.DEV}
    if sorted(run3) != list(range(70)) or sorted(policy) != list(range(70)):
        raise ValueError('Expected recorded expert v3 and policy episodes on all 70 development rows')
    v3 = {r: run3[r]['decisions'] for r in run3}
    pol = {r: policy[r]['decisions'] for r in policy}
    return {EXPERT_V4: v3, POLICY_V4: pol, RULE_V3: v3, RULE_V4: v3}, run3


def resolve():
    """The re-check's resolved configuration (no simulation)."""
    from experiments import v15_ledger as L
    from experiments import v15_phase4 as P4
    from traffic import highway_flow_contract as F
    from traffic import highway_flow_rl_v14_contract as contract
    from traffic import v15_benchmark, v15_level2, v15_level2_v4, v15_rule_expert_v4
    cfg = contract.build_config(24)
    dev = v15_benchmark.development_rows(cfg)
    rows = list(range(len(dev)))
    if len(rows) != 70:
        raise ValueError(f'Expected 70 development rows, found {len(rows)}')
    per_row, run3 = _recorded()
    tasks, canary = tasks_for(rows, {r: dev[r]['horizon_steps'] for r in rows})
    ledger = json.loads((L.study_directory() / 'cost-ledger.json').read_text())
    totals = L.totals(ledger)
    if PHASE in totals['v15']:
        raise ValueError(f'Ledger phase {PHASE} already has segments')
    total_wall = cfg['budget']['total_wall_seconds']
    expected = charge(per_row, tasks, canary, P4._measured_rate(), total_wall - totals['total'], total_wall)
    sources = ('traffic/v15_rule_expert.py', 'traffic/v15_rule_expert_v4.py', 'traffic/v15_level2.py',
               'traffic/v15_level2_v4.py', 'traffic/v15_movement.py', 'experiments/v15_pool.py',
               'experiments/v15_regrade.py', 'experiments/v15_runner.py', 'experiments/v15_expert_v4_recheck.py')
    return dict(
        schema=SCHEMA, phase=PHASE, step=STEP, round_index=24, backend='libsumo', workers=WORKERS, priority=PRIORITY,
        backgrounds=['idm'], benchmark=v15_benchmark.manifest(cfg), drivers=list(DRIVERS),
        expert_v4=dict(driver=EXPERT_V4, settings={k: v15_rule_expert_v4.SETTINGS[k]
                                                   for k in v15_rule_expert_v4.NEW_SETTINGS}),
        level2=dict(policy=POLICY_V4, network_of=POLICY, rule=RULE_V4, settings_digest=v15_level2_v4.settings_digest(),
                    v3_settings_digest=v15_level2.settings_digest()),
        sources={p: F.file_digest(ROOT / p) for p in sources},
        rows=rows, batches=[dict(name=STEP, drivers=list(DRIVERS), tasks=tasks,
                                 expected_decisions=expected['expected_decisions'])],
        parity=dict(row=PARITY_ROW, report_sha=run3[PARITY_ROW]['report_sha'], trace_sha=run3[PARITY_ROW]['trace_sha'],
                    decisions=run3[PARITY_ROW]['decisions']),
        no_regression=NO_REGRESSION, reading=READING,
        charge=expected,
        authorization=AUTHORIZATION, held_out='not built, not run', simulator='SUMO (libsumo); no physics')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('command', choices=('resolve', 'run'))
    args = parser.parse_args(argv)
    if args.command == 'run':
        raise SystemExit(f'Not run: {AUTHORIZATION}')
    resolved = resolve()
    R.check_limits(resolved)
    form = R.write_slim(CONFIG, resolved)
    print(json.dumps(dict(config=CONFIG.relative_to(ROOT).as_posix(), resolved_sha256=form['resolved_sha256'],
                          charge=resolved['charge']), indent=1))


if __name__ == '__main__':
    main()
