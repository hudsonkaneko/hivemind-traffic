from copy import deepcopy
import json
from pathlib import Path

import pytest

from traffic.highway_selection import RANKING_VERSION, selection_diagnostics, selection_rank


ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / 'outputs/highway-stage1/v02/train-20261103T190020Z-8469072d'


def config():
    return dict(evaluation=dict(validation_seeds=[1, 2, 3, 4], provisional_targets=dict(
        maximum_intervention_fraction=.05, cruise_mean_speed_limit_fraction_min=.9,
        passes_min_per_passing_episode=1, merge_success_fraction_min=.9)))


def reports(override_fraction=.1):
    result = []
    for seed, task in enumerate(('cruise', 'following', 'passing', 'merge'), 1):
        totals = dict(scheduled=0, waiting=0, active=12, teleporting=0, completed=0,
                      removed=0, requested=12, unfinished=12, teleported=0,
                      boundary_time_s=800., external_wait_s=1.)
        result.append(dict(case_id=task, seed=seed, task=task, primary_horizon_complete=True,
            accounting_comparison_valid=True, partial_control_horizon=False, agent_steps=1000,
            overrides=round(override_fraction * 1000), intervention_fraction=override_fraction,
            collisions=0, teleports=0, overspeed_steps=0, mean_fraction_of_speed_limit=.95,
            passes=int(task == 'passing'), successful_merges=2 * int(task == 'merge'), reward=20.,
            requested_trips=12, completed_trips=0, primary_horizon_time=80.,
            accounting=dict(time=80., comparison_valid=True, totals=totals)))
    return result


def change_fraction(report, fraction):
    report['intervention_fraction'] = fraction
    report['overrides'] = round(fraction * report['agent_steps'])


def test_safety_precedes_intervention_minimization():
    safe, unsafe = reports(.2), reports(0.)
    unsafe[0]['collisions'] = 1
    assert selection_rank(safe, config()) > selection_rank(unsafe, config())


def test_parked_actor_cannot_win_on_low_overrides():
    competent, parked = reports(.2), reports(0.)
    for row in parked:
        row.update(mean_fraction_of_speed_limit=0., passes=0, successful_merges=0)
    assert selection_rank(competent, config()) > selection_rank(parked, config())


def test_case_violations_precede_marginal_cruise_speed_and_reward():
    fewer, faster = reports(.03), reports(.03)
    change_fraction(fewer[1], .2)
    change_fraction(faster[1], .2)
    change_fraction(faster[2], .06)
    faster[0].update(mean_fraction_of_speed_limit=.999, reward=100000.)
    assert selection_rank(fewer, config()) > selection_rank(faster, config())


def test_worst_case_precedes_pooled_rate():
    lower_worst, lower_pooled = reports(.1), reports(.06)
    change_fraction(lower_pooled[1], .2)
    assert selection_rank(lower_worst, config()) > selection_rank(lower_pooled, config())


def test_completed_trips_and_boundary_burden_remain_tie_breaks():
    base, completed, shorter = reports(.03), reports(.03), reports(.03)
    completed[0]['completed_trips'] = 1
    completed[0]['accounting']['totals'].update(completed=1, active=11, unfinished=11)
    shorter[0]['accounting']['totals']['boundary_time_s'] = 799.
    assert selection_rank(completed, config()) > selection_rank(shorter, config()) > selection_rank(base, config())


def test_zero_arrivals_in_long_road_fixture_are_not_an_acceptance_failure():
    result = selection_diagnostics(reports(.03), config())
    assert result['quality_accepted'] and result['unfinished_trips'] == 48


@pytest.mark.parametrize('field', ['primary_horizon_complete', 'accounting_comparison_valid'])
def test_incomplete_horizon_cannot_be_ranked(field):
    rows = reports()
    rows[0][field] = False
    with pytest.raises(ValueError, match='complete comparable'):
        selection_rank(rows, config())


def test_partial_control_horizon_cannot_be_ranked():
    rows = reports()
    rows[0]['partial_control_horizon'] = True
    with pytest.raises(ValueError, match='partial/incomplete'):
        selection_rank(rows, config())


@pytest.mark.parametrize('mutation', ['missing', 'nonfinite', 'count_fraction', 'cohort', 'time', 'negative'])
def test_malformed_evidence_is_rejected(mutation):
    rows = reports()
    if mutation == 'missing': del rows[0]['accounting']
    elif mutation == 'nonfinite': rows[0]['reward'] = float('nan')
    elif mutation == 'count_fraction': rows[0]['overrides'] += 1
    elif mutation == 'cohort': rows[0]['accounting']['totals']['active'] -= 1
    elif mutation == 'time': rows[0]['accounting']['time'] = 70.
    else: rows[0]['accounting']['totals']['boundary_time_s'] = -1.
    with pytest.raises(ValueError):
        selection_rank(rows, config())


@pytest.mark.parametrize('mutation', ['empty', 'missing_case', 'duplicate_seed', 'missing_task'])
def test_partial_or_changed_validation_matrix_is_rejected(mutation):
    rows = reports()
    if mutation == 'empty': rows = []
    elif mutation == 'missing_case': rows.pop()
    elif mutation == 'duplicate_seed': rows[-1]['seed'] = rows[0]['seed']
    else: rows[-1]['task'] = 'following'
    with pytest.raises(ValueError):
        selection_rank(rows, config())


def test_ranking_is_order_independent_and_does_not_mutate_reports():
    rows = reports()
    before = deepcopy(rows)
    assert selection_rank(rows, config()) == selection_rank(rows[::-1], config())
    assert rows == before and RANKING_VERSION == 5


def test_nonfinite_aggregate_cannot_enter_rank():
    rows = reports()
    for row in rows:
        row['reward'] = 1e308
    with pytest.raises(ValueError, match='Aggregated'):
        selection_rank(rows, config())


@pytest.mark.skipif(not (RUN / 'validation-phase-2.json').exists(), reason='Local retained run is not distributed in Git')
def test_retained_phase_two_ranks_above_final_without_changing_acceptance():
    cfg = json.loads((RUN / 'config.json').read_text())
    phase_two = json.loads((RUN / 'validation-phase-2.json').read_text())
    final = json.loads((RUN / 'validation-phase-3.json').read_text())
    a, b = selection_diagnostics(phase_two, cfg), selection_diagnostics(final, cfg)
    assert a['rank'] > b['rank']
    assert a['pooled_override_fraction'] == pytest.approx(.12473297556400771)
    assert b['pooled_override_fraction'] == pytest.approx(.18147472642001042)
    assert not a['quality_accepted'] and not b['quality_accepted']
