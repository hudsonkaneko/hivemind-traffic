"""The sizing of decision 14 step 2 (experiments/v15_expert_v4_recheck.py): its tasks, expected charge and refusal to
run. No SUMO run and no recorded output is read."""
import json
from pathlib import Path

import pytest

from experiments import v15_expert_v4_recheck as C
from experiments import v15_runner as R

ROOT = Path(__file__).resolve().parents[1]


def test_both_v4_drivers_on_every_row_then_the_two_parity_rows():
    horizons = {r: 500 + (r % 7) * 10 for r in range(70)}
    tasks, canary = C.tasks_for(list(range(70)), horizons)
    assert len(tasks) == 142 and [t['driver'] for t in tasks[:2]] == [C.EXPERT_V4, C.POLICY_V4]
    assert {(t['row'], t['driver']) for t in tasks[:140]} == {(r, d) for r in range(70) for d in C.DRIVERS}
    assert [(t['row'], t['driver']) for t in tasks[140:]] == [(52, C.RULE_V3), (52, C.RULE_V4)]
    assert all(t['row_set'] == 'v15-development' for t in tasks)                 # never the held-out rows
    assert canary == 0 and horizons[tasks[canary]['row']] == min(horizons.values())


def test_the_expected_charge_counts_each_task_and_the_canary_at_recorded_decisions():
    rows = [0, 1]
    tasks, canary = C.tasks_for(rows, {0: 100, 1: 50, 52: 90})
    per_row = {C.EXPERT_V4: {0: 1000, 1: 2000, 52: 191}, C.POLICY_V4: {0: 1100, 1: 2100},
               C.RULE_V3: {52: 191}, C.RULE_V4: {52: 191}}
    rate = dict(decisions_per_clock_second=1900.4)
    out = C.charge(per_row, tasks, canary, rate, 1414.764, 64700)
    assert tasks[canary]['row'] == 1 and tasks[canary]['driver'] == C.EXPERT_V4
    assert out['expected_decisions'] == 1000 + 2000 + 1100 + 2100 + 191 + 191 + 2000
    assert out['projected_decisions'] == round(R.projected(out['expected_decisions']))
    assert out['expected_episodes'] == len(tasks) + 1 and not out['raise_needed']
    assert out['expected_clock_seconds'] == round(C.START_S + max(out['expected_decisions'] / 1900.4, 2100 / 140.)
                                                  + C.SETTLE_S, 1)
    assert out['proposed_allowance']['decisions'] % 10_000 == 0
    assert out['proposed_allowance']['decisions'] >= out['projected_decisions']


def test_the_configuration_is_slim_and_unauthorized_and_run_refuses():
    form = R.read_slim(ROOT / 'configs/evaluation/slim/v15-expert-v4-recheck.json')
    settings = form['settings']
    assert settings['authorization'] == C.AUTHORIZATION and settings['authorization'].startswith('not authorized')
    assert settings['workers'] <= R.MAX_WORKERS and settings['priority'] == R.PRIORITY
    assert settings['drivers'] == list(C.DRIVERS) and settings['no_regression'] == json.loads(json.dumps(C.NO_REGRESSION))
    assert settings['held_out'] == 'not built, not run'
    with pytest.raises(SystemExit, match='Not run'):
        C.main(['run'])
