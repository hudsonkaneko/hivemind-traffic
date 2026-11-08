"""Versioned cumulative training budgets and environment identity."""
from copy import deepcopy
import json
from pathlib import Path

import pytest

from experiments.loop_hazard_training import validate_training_config

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('version', [1, 2, 3])
def test_declared_training_budgets(version):
    cfg = json.loads((ROOT/f'experiments/loop-hazard-rsl-v{version}.json').read_text())
    assert validate_training_config(cfg) is cfg
    prior = cfg.get('budget_prior_transitions', cfg.get('prior_ppo_transitions', 0))
    assert prior + cfg['maximum_ppo_transitions'] == 1048576


def test_full_scene_fresh_model_still_counts_prior_experiment():
    cfg = json.loads((ROOT/'experiments/loop-hazard-rsl-v3.json').read_text())
    assert cfg['environment'] == 'full'
    assert cfg['prior_ppo_transitions'] == 0
    assert cfg['budget_prior_transitions'] == 98304
    assert cfg['behavior_cloning']['gradient_steps'] == 2000
    bad = deepcopy(cfg)
    bad['budget_prior_transitions'] += 2048
    with pytest.raises(ValueError, match='lifetime'):
        validate_training_config(bad)


def test_budget_requires_matching_rollouts():
    cfg = json.loads((ROOT/'experiments/loop-hazard-rsl-v3.json').read_text())
    cfg['maximum_ppo_transitions'] -= 1
    with pytest.raises(ValueError, match='exactly'):
        validate_training_config(cfg)
