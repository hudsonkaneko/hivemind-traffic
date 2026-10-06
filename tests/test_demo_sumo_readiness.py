"""CPU-only tests for readiness gating and immutable run packaging."""

import csv
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

from experiments import demo_sumo_readiness as readiness


def record(policy="scripted", seed=2001, *, completed=True, collisions=0, finite=True,
           repeat=False, fingerprint="abc"):
    return {
        "policy": policy,
        "seed": seed,
        "repeat": repeat,
        "status": "completed",
        "completed_both": completed,
        "collision_count": collisions,
        "finite_metrics": finite,
        "metric_fingerprint": fingerprint,
        "wall_time_seconds": 0.1,
        "episode_steps": 12,
        "safety_overrides": {"agent_0": 1, "agent_1": 0},
        "total_rewards": {"agent_0": 2.0, "agent_1": 3.0},
        "error": None,
    }


def test_gates_require_completion_zero_collisions_and_finite_values():
    episodes = [record(policy=policy, seed=seed) for policy in readiness.POLICIES
                for seed in (2001, 2002, 2003)]
    episodes += [record(policy=policy, repeat=True) for policy in readiness.POLICIES]
    status, gates = readiness._status(episodes)
    assert status == "passed"
    assert gates["per_policy"]["trained"]["passed"]
    assert gates["same_seed_repeat_matches"] == {"scripted": True, "trained": True}
    assert gates["per_policy"]["trained"]["expected_episodes_recorded"] is True

    episodes[0]["completed_both"] = False
    episodes[1]["collision_count"] = 1
    episodes[2]["finite_metrics"] = False
    status, gates = readiness._status(episodes)
    assert status == "failed"
    assert gates["per_policy"]["scripted"]["passed"] is False
    assert gates["per_policy"]["trained"]["passed"] is True

    missing_repeat = [e for e in episodes if not (e["policy"] == "trained" and e["repeat"])]
    status, gates = readiness._status(missing_repeat)
    assert status == "failed"
    assert gates["per_policy"]["trained"]["expected_episodes_recorded"] is False

    mismatch = [dict(e) for e in episodes]
    mismatch[-1]["metric_fingerprint"] = "different"
    status, gates = readiness._status(mismatch)
    assert status == "failed"
    assert gates["same_seed_repeat_matches"]["trained"] is False


def test_episode_record_preserves_errors_and_reports_shield_overrides():
    failure = "RuntimeError: SUMO stopped"
    result = readiness._episode_record("trained", 2001, False, [], {}, failure, 0.25)
    assert result["status"] == "failed"
    assert result["error"] == failure
    assert result["finite_metrics"] is False
    assert result["safety_overrides"] == {}

    good = readiness._episode_record(
        "scripted", 2001, False, [{"reward": 1.0}],
        {"completed_agents": ["agent_0", "agent_1"], "total_rewards": {"agent_0": 1.0},
         "safety_overrides": {"agent_0": 4}, "wall_time_seconds": 0.2}, None, 0.2,
    )
    assert good["finite_metrics"] is True
    assert good["safety_overrides"]["agent_0"] == 4


def test_run_package_is_unique_and_keeps_incomplete_episode(tmp_path, monkeypatch):
    checkpoint = tmp_path / "trusted.pt"
    checkpoint.write_bytes(b"checkpoint")
    monkeypatch.setattr(readiness, "_git", lambda root, *args: "commit\n" if args[0] == "rev-parse" else "")
    monkeypatch.setattr(readiness.subprocess, "run", lambda args, **kwargs: SimpleNamespace(
        stdout=("{\"python\":\"3.11\",\"traci\":\"C:/traci.py\"}\n" if "-c" in args
                else "Eclipse SUMO sumo 1.27.1\n"), stderr="", returncode=0))

    calls = []
    def fake_episode(policy, seed, horizon, checkpoint, episode_dir, log_path,
                     runtime, child_env, root):
        calls.append((policy, seed, horizon))
        episode_dir.mkdir(parents=True)
        log_path.write_text("STDOUT\n\nSTDERR\n", encoding="utf-8")
        is_failed = policy == "trained" and seed == 2002
        summary = {
            "completed_agents": [] if is_failed else ["agent_0", "agent_1"],
            "collision_agents": [], "truncated_agents": [],
            "total_rewards": {"agent_0": 1.0, "agent_1": 2.0},
            "safety_overrides": {"agent_0": 2, "agent_1": 0},
            "proposed_lane_changes": {"agent_0": 0, "agent_1": 0},
            "applied_lane_changes": {"agent_0": 0, "agent_1": 0},
            "episode_steps": 5, "wall_time_seconds": 0.1,
        }
        rows = [{"episode_step": 0, "agent": "agent_0", "reward": 1.0}]
        error = "simulated SUMO failure" if is_failed else None
        return rows, summary, error, 0.1

    monkeypatch.setattr(readiness, "_run_episode", fake_episode)
    runtime = Path(sys.executable)
    child_env = {"SUMO_BINARY": "C:/Program Files/Sumo/bin/sumo.exe"}
    first = readiness.create_run(tmp_path / "runs", checkpoint=checkpoint, root=readiness.ROOT,
                                 runtime=runtime, child_env=child_env)
    second = readiness.create_run(tmp_path / "runs", checkpoint=checkpoint, root=readiness.ROOT,
                                  runtime=runtime, child_env=child_env)
    assert first != second
    assert len(calls) == 16
    manifest = json.loads((first / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "failed"
    failed = next(e for e in manifest["episodes"] if e["policy"] == "trained" and e["seed"] == 2002)
    assert failed["error"] == "simulated SUMO failure"
    assert failed["safety_overrides"] == {"agent_0": 2, "agent_1": 0}
    assert (first / "comparison.csv").is_file()
    assert len(list((first / "logs").glob("*.log"))) == 8
    assert (first / "source" / "scenarios" / "curved_two_agent" / "scenario.sumocfg").is_file()
    with (first / "episode-metrics.csv").open(newline="", encoding="utf-8") as handle:
        assert len(list(csv.DictReader(handle))) == 8


def test_fingerprint_rejects_nonfinite_metric():
    assert readiness._finite_values([{"reward": float("nan")}],
                                    {"total_rewards": {"agent_0": 0.0}, "wall_time_seconds": 1}) == ["rows[0].reward"]
    with pytest.raises(ValueError):
        readiness._metric_fingerprint([{"reward": float("nan")}])


def test_step_length_is_read_from_sumo_config(tmp_path):
    config = tmp_path / "scenario.sumocfg"
    config.write_text('<configuration><time><step-length value="0.2"/></time></configuration>',
                      encoding="utf-8")
    assert readiness._step_length(config) == 0.2


def test_episode_timeout_terminates_process_tree_and_records_checkpoint(tmp_path, monkeypatch):
    class TimedOutProcess:
        pid = 12345
        returncode = None
        def wait(self, timeout=None):
            if timeout is not None:
                raise readiness.subprocess.TimeoutExpired("rollout", timeout)
            self.returncode = -9
            return self.returncode
        def poll(self):
            return self.returncode

    process = TimedOutProcess()
    invocation = {}
    def start(command, **kwargs):
        invocation["command"] = command
        return process
    monkeypatch.setattr(readiness, "EPISODE_TIMEOUT_SECONDS", 1)
    monkeypatch.setattr(readiness.subprocess, "Popen", start)
    monkeypatch.setattr(readiness.subprocess, "run", lambda *a, **k: SimpleNamespace(returncode=0))
    checkpoint = tmp_path / "trusted policy.pt"
    rows, summary, error, _ = readiness._run_episode(
        "trained", 2001, 400, checkpoint, tmp_path / "episode", tmp_path / "episode.log",
        Path(sys.executable), {"SUMO_BINARY": "sumo"}, tmp_path,
    )
    assert rows == [] and summary == {}
    assert "1-second wall-time limit" in error
    assert invocation["command"][-2:] == ["--checkpoint", str(checkpoint)]
    assert "EXCEPTION" in (tmp_path / "episode.log").read_text(encoding="utf-8")
