from copy import deepcopy
import json

import pytest
import torch

from experiments import run_highway_stage1 as runner


def config():
    return runner.load_config(runner.ROOT / 'experiments/configs/highway_stage1_v3.json')


def test_revision_is_disabled_and_keeps_transition_budget_and_contract():
    cfg = config()
    old = runner.load_config(runner.DEFAULT)
    assert not cfg['training_authorized']
    assert cfg['model'] == old['model']
    assert cfg['environment'] == old['environment']
    assert cfg['reward_weights'] == old['reward_weights']
    assert cfg['training']['max_agent_transitions'] == 500000
    assert cfg['training']['max_wall_seconds'] == 7200
    assert runner.ranking_version(cfg) == 5
    assert runner.ranking_version(old) == 4


def test_retention_is_continuous_and_legacy_schedule_is_unchanged():
    cfg = config()
    assert runner.imitation_weight(cfg, 0) == pytest.approx(.05)
    assert runner.imitation_weight(cfg, 200000) == pytest.approx(.03)
    assert runner.imitation_weight(cfg, 400000) == pytest.approx(.01)
    assert runner.imitation_weight(cfg, 450000) == pytest.approx(.01)
    old = runner.load_config(runner.DEFAULT)
    assert runner.imitation_weight(old, 100000) == pytest.approx(.025)
    assert runner.imitation_weight(old, 200000) == 0.


def test_acceleration_weight_changes_only_its_gradient():
    acceleration = torch.tensor(.1, requires_grad=True)
    lane = torch.tensor(.2, requires_grad=True)
    objective = runner.bc_objective((acceleration + lane, acceleration, lane), config())
    objective.backward()
    assert acceleration.grad.item() == 4.
    assert lane.grad.item() == 1.
    old = runner.load_config(runner.DEFAULT)
    assert runner.bc_objective((acceleration + lane, acceleration, lane), old).item() == pytest.approx(.3)


@pytest.mark.parametrize('change', ['epochs', 'weight', 'floor', 'nan', 'decay', 'version', 'ranking'])
def test_invalid_revision_parameters_rejected(tmp_path, change):
    cfg = config()
    if change == 'epochs': cfg['behavior_cloning']['epochs'] = 13
    elif change == 'weight': cfg['behavior_cloning']['acceleration_loss_weight'] = 5.
    elif change == 'floor': cfg['training']['imitation_floor'] = .06
    elif change == 'nan': cfg['training']['imitation_floor'] = float('nan')
    elif change == 'decay': cfg['training']['imitation_decay_transitions'] = 0
    elif change == 'version': cfg['experiment_version'] = '../elsewhere'
    else: cfg['ranking_version'] = 4
    path = tmp_path / 'config.json'
    path.write_text(json.dumps(cfg))
    with pytest.raises(ValueError): runner.load_config(path)


def test_readiness_fingerprint_binds_learning_revision():
    cfg = config()
    original = runner.preparation_digest(cfg)
    for container, key, value in [('behavior_cloning', 'epochs', 8),
                                   ('behavior_cloning', 'acceleration_loss_weight', 2.),
                                   ('training', 'imitation_floor', .02)]:
        changed = deepcopy(cfg)
        changed[container][key] = value
        assert runner.preparation_digest(changed) != original


def test_v03_selection_dispatches_to_strict_full_matrix(monkeypatch):
    from traffic import highway_selection
    monkeypatch.setattr(highway_selection, 'selection_rank', lambda reports, cfg: (123,))
    assert runner.selection_rank([], config()) == (123,)


def test_new_configuration_cannot_start_training_without_authorization(monkeypatch):
    monkeypatch.chdir(runner.ROOT)
    with pytest.raises(ValueError, match='explicit authorization'):
        runner.main(['--config', str(runner.ROOT / 'experiments/configs/highway_stage1_v3.json'), '--mode', 'train'])
