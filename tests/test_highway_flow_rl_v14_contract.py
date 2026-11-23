"""V14 round-20 contract and the runner's collision regression rule (no SUMO, no training)."""
from copy import deepcopy
import json
from pathlib import Path

import pytest

from experiments import run_highway_flow_rl_v14 as runner14
from experiments.run_highway_flow_rl_v06 import StopRuleTriggered
from traffic import highway_flow_rl_v12_contract as v12c
from traffic import highway_flow_rl_v14_contract as contract
from traffic import highway_flow_rl_v14_rewards as v14
from traffic.road_grade import DRIVE_LIMIT, validate_drive_limit

ROOT = Path(__file__).resolve().parents[1]
DECLARED = {'authorization', 'base_config', 'budget', 'evaluation_interval_decisions', 'execution', 'learning_profile',
            'provenance', 'reward_version', 'round_index', 'schema', 'stop_rules', 'v14_control', 'v14_drive_limit',
            'v14_settings'}


@pytest.fixture(scope='module')
def round20():
    return contract.build_config(20)


@pytest.fixture(scope='module')
def round19():
    return json.loads((ROOT / contract.BASE_CONFIG['path']).read_text(encoding='utf-8'))


def test_round20_is_round19_plus_only_the_declared_changes(round20, round19):
    changed = {key for key in set(round19) | set(round20) if round19.get(key) != round20.get(key)}
    assert changed == DECLARED
    assert round20['schema'] == 'highway_flow_rl_v14' and round20['round_index'] == 20
    assert round20['training_authorized'] is True and round20['held_out_authorized'] is False
    assert round20['reward_version'] == v14.REWARD_VERSION
    assert round20['v14_settings'] == v14.SETTINGS and round20['v14_control'] == v14.CONTROL
    assert validate_drive_limit(round20['v14_drive_limit']) == DRIVE_LIMIT
    assert round20['evaluation_interval_decisions'] == 50000
    assert round20['execution']['output_root'] == 'outputs/highway-flow-rl/v14'
    for key in ('training_rows', 'validation_rows', 'warm_start', 'optimizer', 'v12_training', 'v12_settings',
                'critic_warmup_decisions', 'seed'):
        assert round20[key] == round19[key]


def test_budget_raise_is_decisions_and_two_diagnostic_episodes_only(round20, round19):
    before, after = round19['budget'], round20['budget']
    assert {k for k in before if before[k] != after[k]} == {'total_decisions', 'max_diagnostic_episodes'}
    assert after['total_decisions'] == before['total_decisions'] + 100000 == 6330000
    assert after['max_diagnostic_episodes'] == before['max_diagnostic_episodes'] + 2 == 170
    assert after['per_round_decisions'] == 200000 and after['per_round_wall_seconds'] == 3600
    assert after['total_wall_seconds'] == 61200


def test_collision_rule_is_restated_as_a_regression_check(round20, round19):
    before, after = round19['stop_rules'], round20['stop_rules']
    assert before['any_collision'] is True and after['any_collision'] is False
    assert 'collision_regression' in after
    kept = ('max_pooled_intervention_fraction', 'relative_merge_floor', 'merge_speed_match_floor', 'merge_floor_action')
    assert all(after[key] == before[key] for key in kept)


def test_validate_config_requires_the_declared_round(round20, round19):
    assert contract.validate_config(deepcopy(round20), require_training=True)
    for change in (dict(evaluation_interval_decisions=100000), dict(budget=dict(round20['budget'], total_decisions=7e6)),
                   dict(v14_drive_limit=None), dict(stop_rules=dict(round20['stop_rules'], any_collision=True))):
        with pytest.raises(ValueError):
            contract.validate_config(dict(deepcopy(round20), **change))
    assert contract.validate_config(deepcopy(round19)) == round19      # evaluation and recordings
    with pytest.raises(ValueError, match='trains only declared v14 rounds'):
        contract.validate_config(deepcopy(round19), require_training=True)


def test_written_configuration_is_the_contracts(round20):
    path = ROOT / 'configs/training/highway-flow-rl-v14-round20.json'
    assert json.loads(path.read_text(encoding='utf-8')) == round20


def _report(case, collisions, drive=None):
    report = dict(case_id=case, collisions=collisions)
    if drive:
        report['v14_drive_limit'] = drive
    return report


def _initial(output, collisions):
    output.mkdir(parents=True, exist_ok=True)
    reports = [_report(f'case-{i}', n) for i, n in enumerate(collisions)]
    (output / 'initial-validation.json').write_text(json.dumps(dict(summary={}, reports=reports)))


def _result(collisions):
    return dict(summary={}, reports=[_report(f'case-{i}', n) for i, n in enumerate(collisions)])


def test_collision_regression_stops_only_on_new_or_more_collisions(tmp_path, round20):
    _initial(tmp_path, [0, 2, 0])
    same = _result([0, 2, 0])
    runner14.collision_regression(round20, tmp_path, 'periodic-validation-00010', same)
    assert same['summary']['collision_regression']['regressed'] is False
    fewer = _result([0, 1, 0])
    runner14.collision_regression(round20, tmp_path, 'periodic-validation-00011', fewer)
    assert fewer['summary']['collision_regression']['regressed'] is False
    for collisions in ([1, 2, 0], [0, 3, 0], [0, 0, 1]):
        with pytest.raises(StopRuleTriggered, match='collision regression'):
            runner14.collision_regression(round20, tmp_path, 'periodic-validation-00012', _result(collisions))
    final = _result([0, 0, 1])                                      # recorded, never a stop outside periodic checks
    runner14.collision_regression(round20, tmp_path, 'final-validation', final)
    assert final['summary']['collision_regression']['new_colliding_cases'] == ['02 case-2']


def test_collision_regression_needs_an_initial_validation_and_the_rule(tmp_path, round20, round19):
    result = _result([3])
    runner14.collision_regression(round20, tmp_path, 'periodic-validation-00001', result)
    assert 'collision_regression' not in result['summary']
    _initial(tmp_path, [0])
    runner14.collision_regression(round19, tmp_path, 'periodic-validation-00001', result)
    assert 'collision_regression' not in result['summary']


def test_drive_limit_summary_pools_cases():
    drive = dict(decisions=10, limited=2, max_grade=.07, min_ceiling_m_s2=1.1, settings={}, road_grade={})
    summary = runner14.drive_limit_summary([_report('a', 0, drive), _report('b', 0), _report('c', 0, dict(drive, limited=0))])
    assert summary['decisions'] == 20 and summary['limited'] == 2 and summary['limited_fraction'] == .1
    assert set(summary['per_case']) == {'00 a', '02 c'}
    assert runner14.drive_limit_summary([_report('b', 0)]) is None


def test_runner_sources_include_the_contract():
    assert 'traffic/highway_flow_rl_v14_contract.py' in runner14.SOURCES
    assert 'scripts/prepare_highway_flow_rl_v14.py' in runner14.SOURCES
    assert v12c.ROUNDS[contract.BASE_ROUND]['budget']['per_round_decisions'] == 200000


# --- round 21: the stop fix -----------------------------------------------------------------
ROUND21_DECLARED = {'authorization', 'budget', 'learning_profile', 'provenance', 'reward_version', 'round_index',
                    'stop_rules', 'v14_stop_fix'}


@pytest.fixture(scope='module')
def round21():
    return contract.build_config(21)


def test_round21_is_round20_plus_only_the_stop_fix(round21, round20):
    from traffic import highway_flow_rl_v14_stop_fix as stop_fix
    changed = {key for key in set(round20) | set(round21) if round20.get(key) != round21.get(key)}
    assert changed == ROUND21_DECLARED
    assert round21['round_index'] == 21 and round21['training_authorized'] is True
    assert round21['held_out_authorized'] is False
    assert round21['reward_version'] == stop_fix.REWARD_VERSION != v14.REWARD_VERSION
    assert stop_fix.validate_settings(round21['v14_stop_fix']) == stop_fix.SETTINGS
    assert round21['provenance']['parent_round'] == round20['learning_profile']
    assert round21['authorization'].startswith(round20['authorization'])
    for key in ('training_rows', 'validation_rows', 'warm_start', 'seed', 'optimizer', 'v14_drive_limit',
                'evaluation_interval_decisions', 'v12_settings', 'v06_reward_weights', 'reward_weights'):
        assert round21[key] == round20[key]


def test_round21_budget_raises_training_decisions_only(round21, round20):
    before, after = round20['budget'], round21['budget']
    assert {k for k in before if before[k] != after[k]} == {'total_decisions'}
    assert after['total_decisions'] == before['total_decisions'] + 200000 == 6530000
    assert after['max_diagnostic_episodes'] == 170 and after['total_wall_seconds'] == 61200


def test_round21_adds_the_free_road_stop_rule(round21, round20):
    before, after = round20['stop_rules'], round21['stop_rules']
    assert {k for k in set(before) | set(after) if before.get(k) != after.get(k)} == {
        'free_road_stop_regression', 'checked_at'}
    rule = after['free_road_stop_regression']
    assert rule['minimum'] == 20 and rule['factor'] == 2.


def test_round21_validation_and_written_configuration(round21):
    assert contract.validate_config(deepcopy(round21), require_training=True)
    for change in (dict(v14_stop_fix=None), dict(reward_version=v14.REWARD_VERSION),
                   dict(budget=dict(round21['budget'], total_decisions=7000000))):
        with pytest.raises(ValueError):
            contract.validate_config(dict(deepcopy(round21), **change))
    path = ROOT / 'configs/training/highway-flow-rl-v14-round21.json'
    assert json.loads(path.read_text(encoding='utf-8')) == round21


def _stops(free_road, stopped=None):
    return dict(decisions=100, stopped=free_road if stopped is None else stopped, asking_to_brake=0,
                asking_to_brake_overrides=0, free_road=free_road)


def _stop_initial(output, free_road):
    output.mkdir(parents=True, exist_ok=True)
    (output / 'initial-validation.json').write_text(json.dumps(dict(summary=dict(v14_stops=_stops(free_road)),
                                                                    reports=[])))


def test_free_road_stop_regression_threshold(tmp_path, round21):
    _stop_initial(tmp_path, 0)
    ok = dict(summary=dict(v14_stops=_stops(20)), reports=[])
    runner14.free_road_stop_regression(round21, tmp_path, 'periodic-validation-00031', ok)
    assert ok['summary']['free_road_stop_regression'] == dict(initial_free_road=0, free_road=20, limit=20, regressed=False)
    with pytest.raises(StopRuleTriggered, match='free-road stopped decisions'):
        runner14.free_road_stop_regression(round21, tmp_path, 'periodic-validation-00056',
                                           dict(summary=dict(v14_stops=_stops(25)), reports=[]))
    final = dict(summary=dict(v14_stops=_stops(180)), reports=[])
    runner14.free_road_stop_regression(round21, tmp_path, 'final-validation', final)   # recorded, not a stop
    assert final['summary']['free_road_stop_regression']['regressed'] is True
    _stop_initial(tmp_path, 15)                                                      # factor 2 x initial binds
    runner14.free_road_stop_regression(round21, tmp_path, 'periodic-validation-00091',
                                       dict(summary=dict(v14_stops=_stops(30)), reports=[]))


def test_free_road_stop_rule_needs_the_rule_and_counts(tmp_path, round20, round21):
    _stop_initial(tmp_path, 0)
    result = dict(summary=dict(v14_stops=_stops(500)), reports=[])
    runner14.free_road_stop_regression(round20, tmp_path, 'periodic-validation-00001', result)
    assert 'free_road_stop_regression' not in result['summary']
    (tmp_path / 'initial-validation.json').write_text(json.dumps(dict(summary={}, reports=[])))
    runner14.free_road_stop_regression(round21, tmp_path, 'periodic-validation-00001', result)
    assert 'free_road_stop_regression' not in result['summary']


def test_stops_summary_pools_cases():
    reports = [dict(case_id='a', v14_stops=_stops(3, 5)), dict(case_id='b'), dict(case_id='c', v14_stops=_stops(0, 0))]
    summary = runner14.stops_summary(reports)
    assert summary['free_road'] == 3 and summary['stopped'] == 5 and summary['decisions'] == 200
    assert set(summary['per_case']) == {'00 a'}
    assert runner14.stops_summary([dict(case_id='b')]) is None


def test_runner_sources_include_the_stop_fix():
    assert 'traffic/highway_flow_rl_v14_stop_fix.py' in runner14.SOURCES


# --- round 22: the drive-aware reward target ------------------------------------------------
ROUND22_DECLARED = {'authorization', 'budget', 'learning_profile', 'provenance', 'reward_version', 'round_index',
                    'v14_drive_target'}


@pytest.fixture(scope='module')
def round22():
    return contract.build_config(22)


def test_round22_is_round21_plus_only_the_drive_target(round22, round21):
    from traffic import highway_flow_rl_v14_drive_target as drive_target
    changed = {key for key in set(round21) | set(round22) if round21.get(key) != round22.get(key)}
    assert changed == ROUND22_DECLARED
    assert round22['round_index'] == 22 and round22['training_authorized'] is True
    assert round22['held_out_authorized'] is False
    assert round22['reward_version'] == drive_target.REWARD_VERSION != round21['reward_version']
    assert drive_target.validate_settings(round22['v14_drive_target']) == drive_target.SETTINGS
    assert round22['provenance']['parent_round'] == round21['learning_profile']
    assert round22['authorization'].startswith(round21['authorization'])
    for key in ('training_rows', 'validation_rows', 'warm_start', 'seed', 'optimizer', 'v14_drive_limit',
                'v14_stop_fix', 'stop_rules', 'evaluation_interval_decisions', 'v12_settings',
                'v06_reward_weights', 'reward_weights'):
        assert round22[key] == round21[key]


def test_round22_budget_raises_decisions_and_two_diagnostic_episodes(round22, round21):
    before, after = round21['budget'], round22['budget']
    assert {k for k in before if before[k] != after[k]} == {'total_decisions', 'max_diagnostic_episodes'}
    assert after['total_decisions'] == before['total_decisions'] + 200000 == 6730000
    assert after['max_diagnostic_episodes'] == before['max_diagnostic_episodes'] + 2 == 172
    assert after['total_wall_seconds'] == 61200


def test_round22_validation_and_written_configuration(round22):
    assert contract.validate_config(deepcopy(round22), require_training=True)
    for change in (dict(v14_drive_target=None), dict(reward_version=round22['reward_version'] + 'x'),
                   dict(budget=dict(round22['budget'], max_diagnostic_episodes=180))):
        with pytest.raises(ValueError):
            contract.validate_config(dict(deepcopy(round22), **change))
    path = ROOT / 'configs/training/highway-flow-rl-v14-round22.json'
    assert json.loads(path.read_text(encoding='utf-8')) == round22


def test_drive_target_summary_regrades_only_rolling_merge_quality():
    gates = [dict(passed=False, failures=['native_emergency_or_missing_accounting', 'rolling_merge_quality']),
             dict(passed=False, failures=['rolling_merge_quality']),
             dict(passed=True, failures=[])]
    quality = lambda *flags: {f'agent_{i}': dict(qualified=f, speed_match_fraction=.95 if f else .4)
                              for i, f in enumerate(flags)}
    reports = [dict(case_id='a', task='merge', stable_merge=dict(targets=2), v14_drive_target=dict(quality_merges=quality(True, True))),
               dict(case_id='b', task='merge', stable_merge=dict(targets=2), v14_drive_target=dict(quality_merges=quality(True, True))),
               dict(case_id='c', task='cruise')]
    result = runner14.drive_target_summary(reports, gates)
    assert result['gates_passed'] == 2      # b now passes; a keeps its emergency failure; c keeps its pass
    assert result['per_case']['00 a']['failures'] == ['native_emergency_or_missing_accounting']
    assert result['per_case']['01 b']['passed'] is True and result['per_case']['01 b']['gate_passed'] is False
    reports[1]['v14_drive_target']['quality_merges'] = quality(True, False)
    assert runner14.drive_target_summary(reports, gates)['gates_passed'] == 1
    assert runner14.drive_target_summary([dict(case_id='c', task='cruise')], gates[2:]) is None


def test_runner_sources_include_the_drive_target():
    assert 'traffic/highway_flow_rl_v14_drive_target.py' in runner14.SOURCES
