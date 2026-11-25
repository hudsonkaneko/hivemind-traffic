"""V14 round 24: multi-seed validation, the cut-in charge and drive saturation (no SUMO, no training)."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from environments.highway_flow_rl_v14_env import HighwayFlowRLV14Env
from experiments import run_highway_flow_rl_v14 as runner14
from traffic import highway_flow_rl_v14_contract as contract
from traffic import highway_flow_rl_v14_cut_in as cut_in
from traffic import highway_flow_rl_v14_saturation as saturation

ROUND24_DECLARED = {'authorization', 'budget', 'learning_profile', 'provenance', 'reward_version', 'round_index',
                    'stop_rules', 'v14_cut_in', 'v14_drive_saturation', 'v14_validation', 'validation_rows'}
ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def round23():
    return contract.build_config(23)


@pytest.fixture(scope='module')
def round24():
    return contract.build_config(24)


def test_round24_is_round23_plus_only_the_declared_changes(round24, round23):
    changed = {key for key in set(round23) | set(round24) if round23.get(key) != round24.get(key)}
    assert changed == ROUND24_DECLARED
    assert round24['round_index'] == 24 and round24['training_authorized'] is True
    assert round24['held_out_authorized'] is False and round24['held_out_rows'] == round23['held_out_rows']
    assert round24['reward_version'] == cut_in.REWARD_VERSION != round23['reward_version']
    assert cut_in.validate_settings(round24['v14_cut_in']) == cut_in.SETTINGS
    assert saturation.validate_settings(round24['v14_drive_saturation']) == saturation.SETTINGS
    assert round24['provenance']['parent_round'] == round23['learning_profile']
    assert round24['authorization'].startswith(round23['authorization'])
    assert '"All three"' in round24['authorization'] and '"approve full"' in round24['authorization']
    for key in ('training_rows', 'warm_start', 'seed', 'optimizer', 'v14_drive_limit', 'v14_stop_fix',
                'v14_follow_gate', 'evaluation_interval_decisions', 'v12_settings', 'v06_reward_weights',
                'reward_weights', 'gae', 'v12_training'):
        assert round24[key] == round23[key]


def test_validation_is_each_case_at_its_own_seed_then_at_two_added_seeds(round24, round23):
    rows, own = round24['validation_rows'], round23['validation_rows']
    assert len(own) == 14 and len(rows) == 42
    assert rows[:14] == own
    for block, seed in ((rows[14:28], 6952001), (rows[28:], 6952002)):
        for row, base in zip(block, own):
            assert row['seed'] == seed and dict(row, seed=base['seed']) == base
    assert round24['v14_validation']['seeds_per_case'] == 3 and round24['v14_validation']['cases'] == 14


def test_validation_seeds_are_disjoint_from_training_held_out_and_diagnosis_seeds(round24):
    rows = round24['training_rows']
    training = set()
    for critic in (False, True):
        for cursor in range(400):     # far more episodes than a 200,000-decision round runs
            seed = rows[cursor % len(rows)]['seed'] + 1000 * (cursor // len(rows))
            training.add(seed + 10000 * (round24['round_index'] - 1) + (50000 if critic else 0))
    added = set(contract.ROUND24_VALIDATION_SEEDS)
    assert not added & training
    assert not added & {row['seed'] for row in round24['held_out_rows']}
    assert not added & set(range(6951001, 6951012))
    assert not {row['seed'] for row in round24['validation_rows']} & training


def test_free_road_stop_minimum_scales_with_the_rows_only(round24, round23):
    rule, old = round24['stop_rules']['free_road_stop_regression'], round23['stop_rules']['free_road_stop_regression']
    assert rule['minimum'] == 60 and old['minimum'] == 20 and rule['factor'] == old['factor'] == 2.
    assert {k: v for k, v in round24['stop_rules'].items() if k != 'free_road_stop_regression'} == \
           {k: v for k, v in round23['stop_rules'].items() if k != 'free_road_stop_regression'}


def test_round24_budget_is_the_approved_full_raise_only(round24, round23):
    budget, old = round24['budget'], round23['budget']
    changed = {key for key in set(budget) | set(old) if budget.get(key) != old.get(key)}
    assert changed == {'total_decisions', 'total_wall_seconds', 'per_round_wall_seconds', 'max_diagnostic_episodes'}
    assert budget['total_decisions'] == old['total_decisions'] + 141150 == 6871150
    assert budget['total_wall_seconds'] == old['total_wall_seconds'] + 3500 == 64700
    assert budget['per_round_wall_seconds'] == 4800 and budget['max_diagnostic_episodes'] == 174
    assert budget['per_round_decisions'] == 200000 and budget['final_reserve_seconds'] == 900


def test_written_configuration_is_the_contracts(round24):
    path = ROOT / 'configs/training/highway-flow-rl-v14-round24.json'
    assert json.loads(path.read_text(encoding='utf-8')) == round24


def test_round24_validates_and_rejects_changes(round24):
    assert contract.validate_config(deepcopy(round24), require_training=True)
    for change in (dict(v14_cut_in=dict(round24['v14_cut_in'], weight=1.)), dict(v14_drive_saturation=None),
                   dict(validation_rows=round24['validation_rows'][:14]), dict(v14_validation=None)):
        with pytest.raises(ValueError):
            contract.validate_config(dict(deepcopy(round24), **change))


class Vehicles:
    def __init__(self, leaders):
        self.leaders, self.calls = leaders, []

    def getLeader(self, vehicle, distance):
        self.calls.append((vehicle, distance))
        return self.leaders.get(vehicle)


def test_cut_in_charges_only_background_followers_braking_hard_close_behind_our_cars():
    vehicles = Vehicles({'main_1': ('agent_0', 12.), 'main_2': ('agent_0', 39.9), 'main_3': ('main_9', 5.),
                         'main_4': ('agent_1', 3.), 'main_5': ('agent_0', 2.)})
    states = {'agent_0': dict(acceleration=-6.), 'agent_1': dict(acceleration=0.),
              'agent_2': dict(acceleration=-8.),                 # controlled follower: its own braking
              'main_1': dict(acceleration=-9.), 'main_2': dict(acceleration=-4.),
              'main_3': dict(acceleration=-9.),                  # leader is a background car
              'main_4': dict(acceleration=-2.9),                 # not below -3
              'main_5': dict(acceleration=-5.), 'main_9': dict(acceleration=0.)}
    result = cut_in.charges(SimpleNamespace(vehicle=vehicles), states, ['agent_0', 'agent_1'])
    assert set(result) == {'agent_0'}
    total, events = result['agent_0']
    assert total == pytest.approx(6. + 1. + 2.)
    assert {e['follower'] for e in events} == {'main_1', 'main_2', 'main_5'}
    # only hard-braking background vehicles are looked up, with the 40 m limit
    assert {call[0] for call in vehicles.calls} == {'main_1', 'main_2', 'main_3', 'main_5'}
    assert all(call[1] == 40. for call in vehicles.calls)


def test_cut_in_skips_far_missing_and_inactive_leaders():
    vehicles = Vehicles({'main_1': ('agent_0', 40.5), 'main_2': ('', -1.), 'main_3': None, 'main_4': ('agent_3', 1.)})
    states = {name: dict(acceleration=-7.) for name in ('main_1', 'main_2', 'main_3', 'main_4')}
    assert cut_in.charges(SimpleNamespace(vehicle=vehicles), states, ['agent_0']) == {}


def test_cut_in_settings_are_pinned():
    assert cut_in.SETTINGS == dict(weight=.25, braking_m_s2=-3., max_gap_m=40., followers='background')
    with pytest.raises(ValueError):
        cut_in.validate_settings(dict(cut_in.SETTINGS, max_gap_m=80.))


def test_saturation_is_an_override_caused_only_by_the_drive_bound():
    assert saturation.saturated(dict(override=True, override_causes=['drive_grade']))
    for info in (dict(override=True, override_causes=['drive_grade', 'comfort_rate_limit']),
                 dict(override=True, override_causes=['drive_grade', 'speed_ceiling']),
                 dict(override=True, override_causes=['drive_grade', 'native_realization_unattributed']),
                 dict(override=True, override_causes=[]), dict(override=False, override_causes=['drive_grade'])):
        assert not saturation.saturated(info)
    with pytest.raises(ValueError):
        saturation.validate_settings(dict(intervention=True, causes=['drive_grade']))


def test_saturation_exemption_undoes_the_intervention_counts_and_reward():
    env = SimpleNamespace(stats=dict(reward_terms=dict(intervention=-.6), overrides=3,
                                     override_causes=dict(drive_grade=2, speed_ceiling=1)))
    info = dict(override=True, override_causes=['drive_grade'], requested_acceleration=1.4,
                applied_acceleration=1.02, reward_terms=dict(intervention=-.2, progress=.5))
    rewards = dict(agent_0=.3)
    HighwayFlowRLV14Env._drive_saturation_exempt(env, 'agent_0', info, rewards)
    assert rewards['agent_0'] == pytest.approx(.5)
    assert info['override'] is False and info['override_causes'] == []
    assert info['reward_terms']['intervention'] == 0. and info['reward_terms']['progress'] == .5
    assert info['drive_saturation'] == dict(causes=['drive_grade'], requested_m_s2=1.4, applied_m_s2=1.02)
    assert env.stats['overrides'] == 2 and env.stats['override_causes'] == dict(drive_grade=1, speed_ceiling=1)
    assert env.stats['reward_terms']['intervention'] == pytest.approx(-.4)


def test_drive_saturation_needs_the_drive_limit():
    with pytest.raises(ValueError):
        HighwayFlowRLV14Env('unused.net.xml', 'unused', drive_saturation=saturation.SETTINGS)


def test_seeds_summary_counts_each_case_across_its_seeds(round24):
    cfg = dict(v14_validation=dict(cases=2, seeds_per_case=3))
    reports = [dict(case_id=c, seed=s) for s in (1, 2, 3) for c in ('a', 'b')]
    passed = [True, False, True, False, True, True]
    gates = [dict(passed=p, failures=[] if p else ['stable_merge']) for p in passed]
    result = runner14.seeds_summary(cfg, reports, gates)
    assert result['rows_passed'] == 4 and result['rows'] == 6
    assert result['cases_passed_at_every_seed'] == 1 and result['cases_passed_at_own_seed'] == 1
    assert result['per_case']['00 a'] == dict(passed=3, seeds=3, by_seed={'1': [], '2': [], '3': []})
    assert result['per_case']['01 b']['passed'] == 1
    assert runner14.seeds_summary({}, reports, gates) is None


def test_cut_in_summary_pools_cases():
    reports = [dict(case_id='a', v14_cut_in=dict(charged=2, excess_m_s2=7., cut_in=-1.75,
                                                 events=[dict(follower='main_1'), dict(follower='main_1')])),
               dict(case_id='b', v14_cut_in=dict(charged=0, excess_m_s2=0., cut_in=0., events=[])), dict(case_id='c')]
    result = runner14.cut_in_summary(reports)
    assert result['charged'] == 2 and result['cut_in'] == -1.75
    assert result['per_case'] == {'00 a': dict(charged=2, excess_m_s2=7., followers=['main_1'])}
    assert runner14.cut_in_summary([dict(case_id='c')]) is None


def test_runner_passes_round24_settings_and_records_their_sources():
    for path in ('traffic/highway_flow_rl_v14_cut_in.py', 'traffic/highway_flow_rl_v14_saturation.py'):
        assert path in runner14.SOURCES
