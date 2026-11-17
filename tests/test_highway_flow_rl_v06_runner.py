import json
from copy import deepcopy
from pathlib import Path

import pytest

pytest.importorskip('torch')
pytest.importorskip('gymnasium')

from traffic import highway_flow_rl_v06_contract as contract
from traffic import highway_flow_rl_v06_rewards as v06

CONFIG = Path(__file__).resolve().parents[1] / 'configs/training/highway-flow-rl-v06.json'
ROUND6 = Path(__file__).resolve().parents[1] / 'configs/training/highway-flow-rl-v06-round6.json'
ROUND7 = Path(__file__).resolve().parents[1] / 'configs/training/highway-flow-rl-v06-round7.json'
ROUND8 = Path(__file__).resolve().parents[1] / 'configs/training/highway-flow-rl-v06-round8.json'


@pytest.fixture
def cfg():
    return json.loads(CONFIG.read_text(encoding='utf-8'))


def test_committed_config_is_valid_and_held_out_not_authorized(cfg):
    contract.validate_config(cfg)
    assert cfg["held_out_authorized"] is False and cfg["development_gates_passed"] is False
    contract.validate_config(cfg, require_training=cfg['training_authorized'])
    with pytest.raises(ValueError, match='disabled'):
        contract.validate_config(dict(cfg, training_authorized=False), require_training=True)


def test_config_carries_user_decisions_and_audited_weights(cfg):
    assert cfg['rate_limit'] == 'full' and cfg['controlled_car_follow_model'] == 'Krauss'
    assert cfg['road_end_leader_exit_rule'] is False and cfg['v06_reward_weights'] == v06.WEIGHTS
    merges = [r for r in cfg['training_rows'] if r['case']['task'] == 'merge']
    assert sum(r.get('missed_gap', False) for r in merges) / len(merges) == .25


@pytest.mark.parametrize('change', [
    lambda c: c.update(rate_limit='startup'),
    lambda c: c.update(road_end_leader_exit_rule=True),
    lambda c: c['v06_reward_weights'].update(merge_tracking=.5),
    lambda c: c['budget'].update(per_round_decisions=600000),
    lambda c: c['stop_rules'].update(any_collision=False),
    lambda c: c['training_rows'][0].update(missed_gap=True),          # I-110 4-AV row
    lambda c: c['validation_rows'][0].update(missed_gap=True),
    lambda c: c['training_rows'][1]['case'].update(network='outputs/highway-environment/v05/network-bank/train-4-lane-ramp120/network.net.xml'),
    lambda c: c.update(warm_start=dict(c['warm_start'], sha256='0' * 64)),
])
def test_contract_rejects_unreviewed_changes(cfg, change):
    bad = deepcopy(cfg)
    change(bad)
    with pytest.raises(ValueError):
        contract.validate_config(bad)


@pytest.fixture
def cfg6():
    return json.loads(ROUND7.read_text(encoding='utf-8'))  # the round-6 profile, run as round 7


def test_superseded_round6_config_is_not_runnable():
    old = json.loads(ROUND6.read_text(encoding='utf-8'))
    contract.validate_config(old)
    assert old['training_authorized'] is False and 'Superseded' in old['authorization']


def test_round6_config_is_valid_unauthorized_and_changes_only_the_chosen_fields(cfg, cfg6):
    contract.validate_config(cfg6)
    assert cfg6['held_out_authorized'] is False and cfg6['development_gates_passed'] is False
    contract.validate_config(cfg6, require_training=cfg6['training_authorized'])  # user approval 2026-11-15
    with pytest.raises(ValueError, match='disabled'):
        contract.validate_config(dict(cfg6, training_authorized=False), require_training=True)
    assert cfg6['round_index'] == 7 and cfg6['raw_request_scope'] == 'after_first_decision'
    assert cfg6['optimizer'] == dict(cfg['optimizer'], epochs=10, target_kl=.03, episodes_per_update=4)
    assert cfg6['stop_rules']['merge_speed_match_floor'] == pytest.approx(.4167)
    assert cfg6['stop_rules']['max_pooled_intervention_fraction'] == .05
    changed = {k for k in set(cfg) | set(cfg6) if cfg.get(k) != cfg6.get(k)}
    assert changed == {'round_index', 'learning_profile', 'optimizer', 'raw_request_scope',
                       'acceptance', 'stop_rules', 'provenance', 'missed_gap_seed_check', 'authorization'}


@pytest.mark.parametrize('change', [
    lambda c: c.update(raw_request_scope='all'),
    lambda c: c.pop('raw_request_scope'),
    lambda c: c['optimizer'].update(episodes_per_update=1),
    lambda c: c['optimizer'].update(target_kl=.05),
    lambda c: c['stop_rules'].pop('max_pooled_intervention_fraction'),
    lambda c: c['acceptance'].update(raw_request_scope='all'),
    lambda c: c.update(learning_profile='merge_v06_300k_20epochs'),
    lambda c: c.update(round_index=8),
])
def test_contract_rejects_unreviewed_round6_changes(cfg6, change):
    bad = deepcopy(cfg6)
    change(bad)
    with pytest.raises(ValueError):
        contract.validate_config(bad)


def test_round6_training_seeds_do_not_reach_evaluation_seeds(cfg6):
    held = {r['seed'] for r in cfg6['validation_rows'] + cfg6['held_out_rows']}
    for cycle in range(40):
        for row in cfg6['training_rows']:
            for critic in (0, 50000):
                assert row['seed'] + 1000 * cycle + 10000 * 6 + critic not in held


@pytest.fixture
def cfg8():
    return json.loads(ROUND8.read_text(encoding='utf-8'))


def test_round8_config_is_valid_unauthorized_and_changes_only_the_proposed_fields(cfg, cfg6, cfg8):
    contract.validate_config(cfg8)
    assert cfg8['training_authorized'] is False  # proposed November 17, 2026; awaiting user approval
    assert cfg8['held_out_authorized'] is False and cfg8['development_gates_passed'] is False
    with pytest.raises(ValueError, match='disabled'):
        contract.validate_config(cfg8, require_training=True)
    assert cfg8['round_index'] == 8 and cfg8['raw_request_scope'] == 'after_insertion_lag'
    assert cfg8['optimizer'] == dict(cfg6['optimizer'], target_kl=.1)
    assert cfg8['stop_rules']['merge_floor_action'] == 'report'
    assert cfg8['stop_rules']['max_pooled_intervention_fraction'] == .05 and cfg8['stop_rules']['any_collision']
    changed = {k for k in set(cfg6) | set(cfg8) if cfg6.get(k) != cfg8.get(k)}
    assert changed == {'round_index', 'learning_profile', 'optimizer', 'raw_request_scope', 'acceptance',
                       'stop_rules', 'provenance', 'missed_gap_seed_check', 'authorization', 'training_authorized'}
    for key in ('training_rows', 'validation_rows', 'held_out_rows', 'warm_start', 'budget', 'v06_reward_weights'):
        assert cfg8[key] == cfg[key]


@pytest.mark.parametrize('change', [
    lambda c: c.update(raw_request_scope='after_first_decision'),
    lambda c: c['acceptance'].update(raw_request_scope='after_first_decision'),
    lambda c: c['optimizer'].update(target_kl=.03),
    lambda c: c['optimizer'].update(episodes_per_update=1),
    lambda c: c['stop_rules'].update(merge_floor_action='stop'),
    lambda c: c['stop_rules'].pop('merge_floor_action'),
    lambda c: c['stop_rules'].pop('max_pooled_intervention_fraction'),
    lambda c: c['stop_rules'].update(any_collision=False),
    lambda c: c.update(round_index=9),
])
def test_contract_rejects_unreviewed_round8_changes(cfg8, change):
    bad = deepcopy(cfg8)
    change(bad)
    with pytest.raises(ValueError):
        contract.validate_config(bad)


def test_earlier_rounds_cannot_switch_off_the_merge_floor(cfg6):
    with pytest.raises(ValueError):
        contract.validate_config(dict(cfg6, stop_rules=dict(cfg6['stop_rules'], merge_floor_action='report')))


def test_round8_training_seeds_do_not_reach_evaluation_seeds(cfg8):
    held = {r['seed'] for r in cfg8['validation_rows'] + cfg8['held_out_rows']}
    for cycle in range(40):
        for row in cfg8['training_rows']:
            for critic in (0, 50000):
                assert row['seed'] + 1000 * cycle + 10000 * 7 + critic not in held


def test_round8_seed_check_covers_every_cycle_the_round_cap_can_reach(cfg8):
    rows = cfg8['training_rows']
    cycle_decisions = sum(r['rollout_decisions'] for r in rows)
    reachable = -(-cfg8['budget']['per_round_decisions'] // cycle_decisions)  # PPO cycles 0..reachable-1
    assert cfg8['missed_gap_seed_check']['cycles'] >= reachable
    missed = [i for i, r in enumerate(rows) if r.get('missed_gap')]
    built = {(c['row'], c['seed']) for c in cfg8['missed_gap_seed_check']['rows']}
    for i in missed:
        for cycle in range(reachable):
            assert (i, rows[i]['seed'] + 1000 * cycle + 70000) in built
        assert (i, rows[i]['seed'] + 70000 + 50000) in built


def test_round8_merge_floor_is_reported_not_enforced(cfg8, monkeypatch, tmp_path):
    from experiments import run_highway_flow_rl_v06 as runner
    low = dict(summary=dict(pooled_intervention_fraction=.01),
               reports=[dict(merge_target_ids=['agent_0'], quality_merges={})])
    monkeypatch.setattr(runner, '_evaluate', lambda *a, **k: deepcopy(low))
    result = runner.evaluate(cfg8, tmp_path, 'periodic-validation-00030', None, None)
    assert result['summary']['merge_speed_match'] == 0. and result['summary']['merge_speed_match_below_floor']
    noisy = deepcopy(low)
    noisy['summary']['pooled_intervention_fraction'] = .2
    monkeypatch.setattr(runner, '_evaluate', lambda *a, **k: deepcopy(noisy))
    with pytest.raises(runner.StopRuleTriggered, match='intervention'):
        runner.evaluate(cfg8, tmp_path, 'periodic-validation-00030', None, None)


def test_buffered_trainer_merges_episodes_and_never_duplicates_a_retry(monkeypatch):
    import torch
    from experiments import run_highway_flow_rl_v06 as runner
    calls = []
    monkeypatch.setattr(runner.MappoTrainer, 'update',
                        lambda self, batch, **k: calls.append(batch['x'].shape[0]) or dict(optimizer_steps=7))
    trainer = object.__new__(runner.BufferedMappoTrainer)
    trainer.episodes_per_update, trainer.buffer = 3, []
    batches = [dict(x=torch.zeros(n, 24)) for n in (5, 6, 7)]
    assert trainer.update(batches[0])['deferred'] and trainer.update(batches[0])['buffered_episodes'] == 1
    assert trainer.update(batches[1])['optimizer_steps'] == 0
    stats = trainer.update(batches[2])
    assert calls == [18] and stats['merged_episodes'] == 3 and stats['optimizer_steps'] == 7 and trainer.buffer == []
    trainer.episodes_per_update = 1
    trainer.update(batches[0])
    assert calls == [18, 5]
    with pytest.raises(ValueError):
        runner.BufferedMappoTrainer(None, episodes_per_update=0)


def test_intervention_stop_rule_fires_only_on_periodic_validation(cfg6, monkeypatch, tmp_path):
    from experiments import run_highway_flow_rl_v06 as runner
    noisy = dict(summary=dict(pooled_intervention_fraction=.105),
                 reports=[dict(merge_target_ids=['agent_0'], quality_merges=dict(agent_0=dict(speed_match_fraction=.5)))])
    monkeypatch.setattr(runner, '_evaluate', lambda *a, **k: deepcopy(noisy))
    with pytest.raises(runner.StopRuleTriggered, match='intervention'):
        runner.evaluate(cfg6, tmp_path, 'periodic-validation-00030', None, None)
    assert runner.evaluate(cfg6, tmp_path, 'final-validation', None, None)['summary']['merge_speed_match'] == .5
    calm = deepcopy(noisy)
    calm['summary']['pooled_intervention_fraction'] = .01
    monkeypatch.setattr(runner, '_evaluate', lambda *a, **k: deepcopy(calm))
    runner.evaluate(cfg6, tmp_path, 'periodic-validation-00030', None, None)


def test_runner_install_points_shared_loop_at_v06():
    from experiments import run_highway_flow_rl_v06 as runner
    runner.install()
    base = runner.base
    assert base.env_for is runner.env_for and base.ruled_gates is v06.v06_gates and base.MappoTrainer is runner.BufferedMappoTrainer
    assert base.validate_config is contract.validate_config and base.CHECKPOINT_SCHEMA.endswith('v06')
    assert set(runner.V06_SOURCES) <= set(base.NEW_SOURCES) and base.OUTPUT_ROOT.name == 'v06'


def test_merge_speed_match_counts_missing_stable_merge_as_zero():
    from experiments.run_highway_flow_rl_v06 import merge_speed_match
    reports = [dict(merge_target_ids=['agent_0'], quality_merges=dict(agent_0=dict(speed_match_fraction=.6))),
               dict(merge_target_ids=['agent_0'], quality_merges={})]
    assert merge_speed_match(reports) == pytest.approx(.3)
    assert merge_speed_match([dict(merge_target_ids=[])]) is None


def test_stop_rules_fire(cfg, monkeypatch, tmp_path):
    from experiments import run_highway_flow_rl_v06 as runner
    monkeypatch.setattr(runner, '_episode', lambda *a, **k: (dict(collisions=1), None))
    with pytest.raises(runner.StopRuleTriggered):
        runner.episode(cfg, cfg['training_rows'][0], tmp_path, None)
    low = dict(summary={}, reports=[dict(merge_target_ids=['agent_0'], quality_merges={})])
    monkeypatch.setattr(runner, '_evaluate', lambda *a, **k: deepcopy(low))
    with pytest.raises(runner.StopRuleTriggered):
        runner.evaluate(cfg, tmp_path, 'periodic-validation-00010', None, None)
    assert runner.evaluate(cfg, tmp_path, 'final-validation', None, None)['summary']['merge_speed_match'] == 0.
