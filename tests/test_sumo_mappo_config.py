"""Configuration boundary tests; loading a proposal never starts a simulator."""

import copy
import math

import pytest

from experiments.sumo_mappo_support import config_digest, load_config, validate_config


def test_proposed_config_is_bounded_headless_and_explicit_launch():
    config = load_config()
    assert config["training"]["algorithm"] == "MAPPO"
    assert config["training"]["max_agent_transitions"] == 1_000_000
    assert config["training"]["max_wall_seconds"] == 7200
    assert config["environment"]["speed_limit_m_s"] == 60 * 0.44704
    assert config["execution"] == dict(worker_timeout_seconds=30, log_mode="files_only",
                                       rendering=False, requires_explicit_train_mode=True)
    assert config_digest(config) == config_digest(copy.deepcopy(config))
    changed = copy.deepcopy(config)
    changed["training"]["seed"] += 1
    assert config_digest(config) != config_digest(changed)


@pytest.mark.parametrize("section,key,value", [
    ("model", "communication", "global_unlimited"),
    ("model", "observation_size", 112),
    ("model", "critic_input", "local_only"),
    ("model", "action", "steering_throttle_brake"),
    ("environment", "speed_limit_m_s", 40.0),
    ("environment", "dt", 0.01),
    ("environment", "horizon", 601),
    ("environment", "acceleration_max_m_s2", 20.0),
    ("training", "algorithm", "PPO"),
    ("training", "device", "cuda"),
    ("training", "num_workers", 5),
    ("training", "num_workers", True),
    ("training", "max_agent_transitions", 1_000_001),
    ("training", "max_wall_seconds", 7201),
    ("training", "actor_lr", math.nan),
    ("training", "target_kl", 0.0),
    ("training", "entropy_coef", -0.01),
    ("execution", "rendering", True),
    ("execution", "requires_explicit_train_mode", False),
    ("execution", "log_mode", "console_spam"),
    ("execution", "worker_timeout_seconds", 61),
    ("preflight", "max_wall_seconds", 301),
    ("preflight", "rollout_steps", 65),
])
def test_unsupported_or_unbounded_changes_are_rejected(section, key, value):
    config = load_config()
    config[section][key] = value
    with pytest.raises(ValueError):
        validate_config(config)


@pytest.mark.parametrize("mutation", [
    lambda c: c["evaluation"].update(validation_seeds=[]),
    lambda c: c["evaluation"]["test_seeds"].append(c["evaluation"]["validation_seeds"][0]),
    lambda c: c["preflight"]["seeds"].append(c["evaluation"]["test_seeds"][0]),
    lambda c: c["evaluation"]["validation_seeds"].append(c["training"]["episode_seed_start"]),
    lambda c: c["evaluation"]["test_seeds"].append(-1),
    lambda c: c["evaluation"]["test_seeds"].append(400030.5),
    lambda c: c["evaluation"]["test_seeds"].append(2**31),
    lambda c: c["preflight"]["seeds"].append(2**31),
    lambda c: c["training"].update(episode_seed_end=99999),
    lambda c: c["training"].update(seed=10.5),
    lambda c: c["training"].update(seed=True),
    lambda c: c["training"].update(episode_seed_start=100000.5),
    lambda c: c["training"].update(episode_seed_end=2**31),
])
def test_seed_partitions_are_nonempty_disjoint_and_valid(mutation):
    config = load_config()
    mutation(config)
    with pytest.raises(ValueError):
        validate_config(config)
