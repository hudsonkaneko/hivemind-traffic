from copy import deepcopy
import json

import pytest

from experiments.highway_v05_curriculum import CURRICULUM_DEFAULTS, next_case, observe


def config():
    return dict(training=dict(seed=9307, seed_start=6800000, seed_end=6800999),
        population={}, model=dict(num_agents=24), environment=dict(traffic_mode='sustained'),
        training_cases=[dict(id=f'train-{task}-{lane}', task=task, split='train', through_lanes=lane)
                        for task in ('cruise', 'following', 'passing', 'merge') for lane in (2, 3)],
        validation_cases=[dict(seed=6900000)])


def report(row, **updates):
    value = dict(curriculum_training=True, kind='learned', partial_control_horizon=False,
        primary_horizon_complete=True, accounting_comparison_valid=True,
        seed=row['seed'], case_id=row['case']['id'], agent_transitions=100,
        collisions=0, teleports=0, overspeed_steps=0, emergency_braking_observed=0,
        native_emergencies=dict(available=True, unique_events=0), intervention_fraction=.01,
        mean_fraction_of_speed_limit=.95, passes=1,
        stable_merge=dict(targets=2, stable=2, failed=0, pending=0))
    value.update(updates)
    return value


def test_json_resume_reproduces_full_future_rows_and_state_with_retries():
    cfg, first = config(), {}
    for index in range(35):
        row = next_case(cfg, first)
        observe(first, report(row, collisions=int(index % 3 == 0)), row)
    resumed = json.loads(json.dumps(first, allow_nan=False))
    for index in range(45):
        a, b = next_case(cfg, first), next_case(cfg, resumed)
        assert a == b
        result = report(a, collisions=int(index % 7 == 0))
        assert observe(first, result, a) == observe(resumed, result, b)
    assert first == resumed
    assert first['retry_rollouts'] > 0


def test_independent_population_fraction_speed_channel_coverage_and_full_av():
    cfg, state = config(), {}
    rows = [next_case(cfg, state) for _ in range(600)]
    assert rows[0]['case']['task'] == 'cruise' and rows[1]['case']['task'] == 'following'
    assert all(row['difficulty'] == 0 for row in rows)
    for total in (8, 16, 24):
        assert {r['requested_av_fraction'] for r in rows if r['total_vehicles'] == total} == {.125, .25, .5, .75, 1.}
    assert {r['controlled_count'] for r in rows if r['requested_av_fraction'] == 1} == {8, 16, 24}
    assert {r['speed_limit_m_s'] for r in rows} == {12., 18., 22.352, 26.8224}
    assert {(r['communication'], r['packet_loss']) for r in rows} == {('none', 0.), ('local', 0.), ('local', .3)}
    assert all(r['traffic_mode'] == 'initial' for r in rows if r['requested_av_fraction'] == 1.)
    assert all(r['controlled_count'] < r['total_vehicles'] for r in rows if r['case']['task'] in ('following', 'passing'))
    for offset in range(0, 600, 4):
        assert len({r['speed_limit_m_s'] for r in rows[offset:offset + 4]}) == 4


def test_fresh_retries_keep_seed_and_scenario_and_never_exceed_twenty_percent():
    cfg, state, issued = config(), {}, {}
    for _ in range(80):
        row = next_case(cfg, state)
        info = row['curriculum']
        if info['retry']:
            source = issued[info['source_rollout_id']]
            assert {k: v for k, v in row.items() if k != 'curriculum'} == {k: v for k, v in source.items() if k != 'curriculum'}
            assert info['rollout_id'] != info['source_rollout_id'] and info['fresh_rollout_required']
            assert 'transitions' not in row and 'batch' not in row
        issued[info['rollout_id']] = row
        observe(state, report(row, collisions=1), row)
        assert state['retry_rollouts'] <= .2 * state['rollouts_issued'] + 1e-12
    assert state['retry_rollouts'] == 16
    assert state['actual_agent_decisions'] == 8000


def test_grade_advances_from_complete_training_and_all_tasks_remain():
    cfg, state = config(), {}
    for _ in range(120):
        row = next_case(cfg, state)
        observe(state, report(row), row)
    assert state['grade'] == 2
    assert [item['grade'] for item in state['grade_history']][:2] == [1, 2]
    rows = [next_case(cfg, state) for _ in range(120)]
    assert all(row['difficulty'] == 2 for row in rows)
    assert {row['case']['task'] for row in rows} == {'cruise', 'following', 'passing', 'merge'}
    assert all(all(value > 0 for value in weights.values()) for weights in CURRICULUM_DEFAULTS['task_weights'])


@pytest.mark.parametrize('changes', [dict(partial_control_horizon=True),
    dict(primary_horizon_complete=False), dict(accounting_comparison_valid=False),
    dict(curriculum_training=False)])
def test_partial_and_non_learning_reports_never_enter_evidence_or_retry_bank(changes):
    state = {}
    row = next_case(config(), state)
    original = deepcopy(state)
    assert not observe(state, report(row, **changes), row)['eligible']
    assert state == original


def test_validation_cannot_enter_schedule_or_observations():
    cfg = config()
    cfg['training_cases'][0]['split'] = 'validation'
    with pytest.raises(ValueError, match='training case bank'):
        next_case(cfg, {})
    state = {}
    row = next_case(config(), state)
    row['case']['split'] = 'validation'
    with pytest.raises(ValueError, match='Validation/test'):
        observe(state, report(row), row)


def test_safety_and_stable_merge_failures_cannot_advance_grade():
    cfg, state = config(), {}
    for _ in range(80):
        row = next_case(cfg, state)
        result = report(row, stable_merge=dict(targets=2, stable=1, pending=1))
        observe(state, result, row)
    assert state['grade'] == 1
    assert all('stable_merge' in item['failures'] for item in state['failed_cases'])
    for _ in range(50):
        row = next_case(cfg, state)
        observe(state, report(row, native_emergencies=dict(available=True, unique_events=1)), row)
    assert state['grade'] == 0


def test_missing_native_safety_evidence_and_duplicate_report_fail_closed():
    state = {}
    row = next_case(config(), state)
    result = observe(state, report(row, native_emergencies=dict(available=False)), row)
    assert not result['passed'] and 'missing_native_emergencies' in result['failures']
    with pytest.raises(ValueError, match='already observed'):
        observe(state, report(row), row)


def test_resume_domain_change_and_oversized_retry_fraction_rejected():
    cfg, state = config(), {}
    next_case(cfg, state)
    cfg['training']['seed'] += 1
    with pytest.raises(ValueError, match='resume configuration'):
        next_case(cfg, state)
    cfg['curriculum'] = dict(failed_training_resample_fraction=.21)
    with pytest.raises(ValueError, match='retry fraction'):
        next_case(cfg, {})
