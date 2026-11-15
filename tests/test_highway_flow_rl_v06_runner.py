import json
from copy import deepcopy
from pathlib import Path

import pytest

pytest.importorskip('torch')
pytest.importorskip('gymnasium')

from traffic import highway_flow_rl_v06_contract as contract
from traffic import highway_flow_rl_v06_rewards as v06

CONFIG = Path(__file__).resolve().parents[1] / 'configs/training/highway-flow-rl-v06.json'


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


def test_runner_install_points_shared_loop_at_v06():
    from experiments import run_highway_flow_rl_v06 as runner
    runner.install()
    base = runner.base
    assert base.env_for is runner.env_for and base.ruled_gates is v06.v06_gates
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
