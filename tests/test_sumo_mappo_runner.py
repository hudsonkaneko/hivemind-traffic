"""Runner lifecycle checks using synthetic weights and mocks, never SUMO training."""

import copy
import json
import random
import time

import numpy as np
import pytest
import torch

from experiments import run_sumo_mappo as runner
from experiments import sumo_mappo_rollout as rollout
from experiments.sumo_mappo_support import config_digest, load_config
from policies import mappo


@pytest.fixture(autouse=True)
def preserve_random_state():
    threads = torch.get_num_threads()
    states = (random.getstate(), np.random.get_state(), torch.get_rng_state())
    torch.set_num_threads(1)
    yield
    random.setstate(states[0])
    np.random.set_state(states[1])
    torch.set_rng_state(states[2])
    torch.set_num_threads(threads)


def small_config():
    config = load_config()
    config["model"]["actor_hidden_sizes"] = [8]
    config["model"]["critic_hidden_sizes"] = [8]
    return config


def forbid_execution(*args, **kwargs):
    pytest.fail("A configuration-only test attempted to execute a run or start workers")


def test_default_mode_checks_config_without_execution(monkeypatch, capsys):
    monkeypatch.chdir(runner.ROOT)
    monkeypatch.setattr(runner, "execute", forbid_execution)
    monkeypatch.setattr(runner, "SumoWorkerPool", forbid_execution)
    monkeypatch.setattr(runner, "make_model", forbid_execution)
    assert runner.main([]) == 0
    displayed = json.loads(capsys.readouterr().out)
    assert displayed["training_started"] is False
    assert displayed["config_sha256"] == config_digest(displayed["config"])


@pytest.mark.parametrize("extra", [[], ["--approved-config-sha256", "0" * 64]])
def test_train_requires_matching_approved_digest(monkeypatch, extra, capsys):
    monkeypatch.chdir(runner.ROOT)
    monkeypatch.setattr(runner, "execute", forbid_execution)
    with pytest.raises(SystemExit) as error:
        runner.main(["--mode", "train", *extra])
    assert error.value.code == 2
    assert "configuration shown for approval" in capsys.readouterr().err


def test_matching_digest_dispatches_only_explicit_train_mode(monkeypatch):
    monkeypatch.chdir(runner.ROOT)
    calls = []
    monkeypatch.setattr(runner, "execute", lambda *args: calls.append(args) or 17)
    digest = config_digest(load_config())
    assert runner.main(["--mode", "train", "--approved-config-sha256", digest]) == 17
    assert len(calls) == 1 and calls[0][1:] == ("train", None)


def test_checkpoint_roundtrip_preserves_weights_optimizer_and_rng(tmp_path, monkeypatch):
    # Only manually populated synthetic optimizer state; no step or backward call.
    monkeypatch.setattr(runner, "ROOT", tmp_path)
    directory = tmp_path / "outputs"
    directory.mkdir()
    path = directory / "checkpoint.pt"
    config = small_config()
    model = runner.make_model(config)
    trainer = mappo.MappoTrainer(model)
    parameter = next(iter(trainer.optimizer.param_groups[0]["params"]))
    trainer.optimizer.state[parameter] = dict(step=torch.tensor(7.),
        exp_avg=torch.full_like(parameter, 0.2), exp_avg_sq=torch.full_like(parameter, 0.3))
    before = runner.model_hash(model)
    random.seed(90)
    np.random.seed(91)
    torch.manual_seed(92)
    runner.save_checkpoint(path, model, trainer, config, 128, 2, 100010, (1, 0, 0, 10, 2., -0.01), 7)
    expected = random.random(), np.random.random(3), torch.rand(3)
    restored_model = runner.make_model(config)
    restored_trainer = mappo.MappoTrainer(restored_model)
    random.seed(10)
    np.random.seed(11)
    torch.manual_seed(12)
    record = runner.load_checkpoint(path, restored_model, config, restored_trainer)
    actual = random.random(), np.random.random(3), torch.rand(3)
    assert actual[0] == expected[0]
    np.testing.assert_array_equal(actual[1], expected[1])
    torch.testing.assert_close(actual[2], expected[2], rtol=0, atol=0)
    assert runner.model_hash(restored_model) == before
    assert record["agent_transitions"] == 128 and record["optimizer_steps_in_run"] == 7
    assert record["next_episode_seed"] == 100010 and record["update"] == 2
    assert "fresh episodes" in record["resume_boundary"]
    restored_parameter = next(iter(restored_trainer.optimizer.param_groups[0]["params"]))
    restored_state = restored_trainer.optimizer.state[restored_parameter]
    assert restored_state["step"].item() == 7
    torch.testing.assert_close(restored_state["exp_avg"], torch.full_like(parameter, 0.2))
    torch.testing.assert_close(restored_state["exp_avg_sq"], torch.full_like(parameter, 0.3))
    assert not path.with_suffix(".tmp").exists()


def test_checkpoint_rejects_wrong_config_or_outside_outputs(tmp_path, monkeypatch):
    monkeypatch.setattr(runner, "ROOT", tmp_path)
    directory = tmp_path / "outputs"
    directory.mkdir()
    config = small_config()
    model = runner.make_model(config)
    trainer = mappo.MappoTrainer(model)
    path = directory / "checkpoint.pt"
    runner.save_checkpoint(path, model, trainer, config, 4, 1, 100004, (), 0)
    changed = copy.deepcopy(config)
    changed["training"]["seed"] += 1
    with pytest.raises(ValueError, match="configuration differs"):
        runner.load_checkpoint(path, model, changed)
    with pytest.raises(ValueError, match="inside project outputs"):
        runner.load_checkpoint(tmp_path / "outside.pt", model, config)


@pytest.mark.parametrize("remaining", [0, 1, 2, 3])
def test_resume_with_insufficient_fresh_episode_budget_never_starts_workers(tmp_path, monkeypatch, remaining):
    config = small_config()
    config["training"]["max_agent_transitions"] = 16
    output, logs = tmp_path / "output", tmp_path / "logs"
    output.mkdir()
    logs.mkdir()
    monkeypatch.setattr(runner, "SumoWorkerPool", forbid_execution)
    monkeypatch.setattr(runner, "load_checkpoint", lambda *args, **kwargs: dict(
        agent_transitions=16 - remaining, update=1, next_episode_seed=100004))
    with pytest.raises(ValueError, match="insufficient budget for a fresh four-agent step"):
        runner.train(config, output, logs, time.monotonic() + 60, resume=tmp_path / "synthetic.pt")


def test_passing_moving_policy_ranks_above_stalled_zero_intervention_candidate():
    config = load_config()
    common = dict(agent_transitions=2400, collision_vehicles=0, overspeed_steps=0,
                  nominal_end=True, peak_speed_m_s=25.)
    moving = dict(common, interventions=24, team_reward=100., mean_progress_m=900.)
    stalled = dict(common, interventions=0, team_reward=500., mean_progress_m=10.)
    good = rollout.aggregate([moving], config["evaluation"]["acceptance"])
    bad = rollout.aggregate([stalled], config["evaluation"]["acceptance"])
    assert good["accepted"] and not bad["accepted"]
    assert tuple(good["selection_score"]) > tuple(bad["selection_score"])


def test_latest_checkpoint_and_status_saved_before_validation_failure(tmp_path, monkeypatch):
    config = small_config()
    config["training"]["max_agent_transitions"] = 4
    output, logs = tmp_path / "output", tmp_path / "logs"
    output.mkdir()
    logs.mkdir()
    events = []

    class MockTrainer:
        def __init__(self, model, **kwargs):
            self.optimizer = torch.optim.Adam(model.parameters())

        def update(self, batch):
            # Mock bookkeeping only: deliberately no optimizer.step/backward.
            assert batch == {"synthetic": True}
            events.append("mock-update")
            return dict(optimizer_steps=2)

    class MockCollector:
        def __init__(self, *args, **kwargs):
            self.next_seed = 100004
            self.total_agent_transitions = 4

        def collect(self, steps, remaining):
            assert remaining == 4
            return {"synthetic": True}, [], 4

    class MockPool:
        forced_cleanup = []

        def __init__(self, *args, **kwargs):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *args):
            events.append("pool-closed")

    real_save = runner.save_checkpoint
    real_json = runner.atomic_json

    def save(*args, **kwargs):
        real_save(*args, **kwargs)
        events.append("checkpoint-saved")

    def save_status(path, value):
        real_json(path, value)
        if path.name == "status.json":
            events.append("status-saved")

    def fail_validation(*args, **kwargs):
        events.append("validation-started")
        checkpoint = torch.load(output / "checkpoints/latest.pt", map_location="cpu", weights_only=True)
        status = json.loads((output / "status.json").read_text())
        assert checkpoint["agent_transitions"] == status["agent_transitions"] == 4
        assert checkpoint["optimizer_steps_in_run"] == status["optimizer_steps"] == 2
        raise RuntimeError("Synthetic validation failure")

    monkeypatch.setattr(mappo, "MappoTrainer", MockTrainer)
    monkeypatch.setattr(rollout, "RolloutCollector", MockCollector)
    monkeypatch.setattr(runner, "SumoWorkerPool", MockPool)
    monkeypatch.setattr(runner, "save_checkpoint", save)
    monkeypatch.setattr(runner, "atomic_json", save_status)
    monkeypatch.setattr(runner, "evaluate", fail_validation)
    with pytest.raises(RuntimeError, match="Synthetic validation failure"):
        runner.train(config, output, logs, time.monotonic() + 60)
    assert events == ["mock-update", "checkpoint-saved", "status-saved", "validation-started", "pool-closed"]
    assert not (output / "checkpoints/best.pt").exists()
    assert not (output / "policy.pt").exists()
