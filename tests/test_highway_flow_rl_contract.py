"""CPU-only checks for authorization, teacher removal and cumulative costs."""
from copy import deepcopy
import math

import pytest

from traffic import highway_flow_rl_contract as contract


@pytest.fixture
def cfg():
    return dict(schema=contract.SCHEMA, training_authorized=True,
        initializer_id=contract.INITIALIZER_ID,
        initializer=deepcopy(contract.INITIALIZERS[contract.INITIALIZER_ID]),
        observation_size=117, state_size=2832, action_contract=deepcopy(contract.ACTION_CONTRACT),
        safety=dict(native_guards_unchanged=True, safety_shield_unchanged=True),
        optimizer=deepcopy(contract.OPTIMIZER), reward_weights=deepcopy(contract.REWARD_WEIGHTS),
        round_index=1, budget=dict(per_round_decisions=30000, total_decisions=60000,
            per_round_wall_seconds=1500., inherited_spent_seconds=contract.INHERITED_SPENT_SECONDS,
            total_wall_seconds=5400., final_reserve_seconds=900.))


def test_resolved_reference_free_contract(cfg):
    assert contract.validate_config(cfg) is cfg
    assert cfg['optimizer']['imitation_coef'] == cfg['optimizer']['near_limit_coef'] == 0
    assert cfg['reward_weights']['feasible_acceleration'] == 0


@pytest.mark.parametrize('field,value', [('training_authorized', False), ('observation_size', 99),
    ('state_size', 118), ('initializer_id', 'v05_final'), ('round_index', True), ('round_index', 3)])
def test_contract_drift_rejected(cfg, field, value):
    cfg[field] = value
    with pytest.raises(ValueError):
        contract.validate_config(cfg)


@pytest.mark.parametrize('section,field,value', [('initializer', 'sha256', '0'*64),
    ('optimizer', 'imitation_coef', .04), ('optimizer', 'near_limit_coef', .1),
    ('reward_weights', 'feasible_acceleration', .25), ('reward_weights', 'collision', 0.),
    ('safety', 'native_guards_unchanged', False), ('budget', 'total_wall_seconds', 5401.),
    ('budget', 'inherited_spent_seconds', 0.), ('budget', 'per_round_wall_seconds', 1501.),
    ('budget', 'total_decisions', 60001), ('budget', 'final_reserve_seconds', 899.),
    ('budget', 'per_round_wall_seconds', math.nan)])
def test_no_teacher_safety_or_budget_bypass(cfg, section, field, value):
    cfg[section][field] = value
    with pytest.raises(ValueError):
        contract.validate_config(cfg)


def test_failed_and_revised_segments_keep_their_costs():
    inherited = contract.INHERITED_SPENT_SECONDS
    assert contract.budget_remaining(inherited) == pytest.approx(3481.3150076)
    assert contract.budget_remaining(inherited + 1500., 100.) == pytest.approx(1881.3150076)
    assert contract.budget_remaining(inherited + 1500., 100., final_reserve_seconds=0) == pytest.approx(2781.3150076)
    assert contract.budget_remaining(5400., 1.) == 0.


@pytest.mark.parametrize('history,spent', [(0., 1.), (1018., 0.), (math.nan, 0.),
    (1020., -1.), (1020., math.inf), (1020., True)])
def test_invalid_accounting_cannot_reset_budget(history, spent):
    with pytest.raises(ValueError):
        contract.budget_remaining(history, spent)


def test_preflight_requires_actual_reference_free_zero_update_proof():
    proof = dict(schema='highway_flow_rl_preflight_v01', passed=True, training=False,
        optimizer_steps=0, config_sha256='config', source_hashes={'runner': 'sha'},
        checks=dict.fromkeys(contract.PREFLIGHT_CHECKS, True))
    assert contract.validate_preflight(proof, 'config', {'runner': 'sha'}) is proof
    for field, value in [('schema', 'highway_flow_preflight_v01'), ('optimizer_steps', 1),
                         ('optimizer_steps', False), ('source_hashes', {}), ('config_sha256', 'stale')]:
        changed = deepcopy(proof)
        changed[field] = value
        with pytest.raises(ValueError):
            contract.validate_preflight(changed, 'config', {'runner': 'sha'})
    for key in contract.PREFLIGHT_CHECKS:
        changed = deepcopy(proof)
        changed['checks'][key] = False
        with pytest.raises(ValueError):
            contract.validate_preflight(changed, 'config', {'runner': 'sha'})


def test_second_round_requires_terminal_evidence_and_scoped_changes(cfg):
    revised = deepcopy(cfg)
    revised['round_index'] = 2
    revised['reward_weights']['jerk'] = .03
    revised['optimizer']['actor_lr'] = 5e-5
    report = dict(completed=True, max_peak_jerk_m_s3=6., approximate_kl=.03,
        pooled_intervention_fraction=.1, initial_pooled_intervention_fraction=.1, collisions=0)
    assert contract.validate_revision(cfg, revised, report) is revised
    with pytest.raises(ValueError, match='completed or failed'):
        contract.validate_revision(cfg, revised, {})
    with pytest.raises(ValueError, match='high jerk'):
        contract.validate_revision(cfg, revised, dict(report, max_peak_jerk_m_s3=5.))
    with pytest.raises(ValueError, match='KL or worsened interventions'):
        contract.validate_revision(cfg, revised, dict(report, approximate_kl=.01))
    revised['seed'] = 123
    with pytest.raises(ValueError, match='undeclared'):
        contract.validate_revision(cfg, revised, report)


def test_declared_low_rate_can_follow_measured_failure(cfg):
    revised = deepcopy(cfg)
    revised['round_index'] = 2
    revised['optimizer']['actor_lr'] = 5e-5
    report = dict(failed=True, max_peak_jerk_m_s3=2., pooled_intervention_fraction=.2,
        initial_pooled_intervention_fraction=.1, collisions=0)
    assert contract.validate_revision(cfg, revised, report) is revised


def test_unchanged_objective_may_continue_after_good_measurements(cfg):
    revised = deepcopy(cfg)
    revised['round_index'] = 2
    report = dict(completed=True, max_peak_jerk_m_s3=2., pooled_intervention_fraction=.01, collisions=0)
    assert contract.validate_revision(cfg, revised, report) is revised


@pytest.fixture
def rows_cfg(cfg):
    row = dict(case=dict(id='i110-merge', task='merge'), controlled_count=4,
        total_vehicles=16, communication='local', packet_loss=.1,
        speed_limit_m_s=26.8224, difficulty=1, seed=7300000)
    cfg['training_rows'] = [row]
    cfg['validation_rows'] = [dict(deepcopy(row), seed=6900000)]
    return cfg


def test_runtime_rows_accept_supported_population_and_disjoint_seeds(rows_cfg):
    assert contract.validate_config(rows_cfg) is rows_cfg


@pytest.mark.parametrize('split', ['training_rows', 'validation_rows'])
@pytest.mark.parametrize('field,value', [('total_vehicles', 12), ('total_vehicles', 20),
    ('total_vehicles', 16.), ('controlled_count', 0), ('controlled_count', 17),
    ('controlled_count', True), ('communication', 'full'), ('packet_loss', -.1),
    ('packet_loss', 1.01), ('packet_loss', math.nan), ('speed_limit_m_s', 0.),
    ('speed_limit_m_s', 27.), ('speed_limit_m_s', math.inf), ('difficulty', True),
    ('difficulty', 3), ('seed', -1), ('seed', True)])
def test_invalid_runtime_row_is_rejected_before_simulator_start(rows_cfg, split, field, value):
    rows_cfg[split][0][field] = value
    with pytest.raises(ValueError):
        contract.validate_config(rows_cfg)


@pytest.mark.parametrize('split', ['training_rows', 'validation_rows'])
@pytest.mark.parametrize('task', ['following', 'passing', 'unknown'])
def test_runtime_tasks_require_known_task_and_native_leader(rows_cfg, split, task):
    row = rows_cfg[split][0]
    row['case']['task'] = task
    row['controlled_count'] = row['total_vehicles']
    with pytest.raises(ValueError):
        contract.validate_config(rows_cfg)


def test_training_and_validation_seeds_cannot_overlap(rows_cfg):
    rows_cfg['validation_rows'][0]['seed'] = rows_cfg['training_rows'][0]['seed']
    with pytest.raises(ValueError, match='across both splits'):
        contract.validate_config(rows_cfg)


def test_duplicate_training_seed_cannot_hide_repeated_cases(rows_cfg):
    rows_cfg['training_rows'].append(deepcopy(rows_cfg['training_rows'][0]))
    with pytest.raises(ValueError, match='across both splits'):
        contract.validate_config(rows_cfg)


def test_training_must_retain_i110_merge_exposure(rows_cfg):
    rows_cfg['training_rows'][0]['case']['id'] = 'unrelated-merge'
    with pytest.raises(ValueError, match='I110 merge'):
        contract.validate_config(rows_cfg)
