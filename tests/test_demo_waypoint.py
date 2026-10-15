"""CPU-only contract checks for the bounded waypoint demo launcher."""
import hashlib
import json
from pathlib import Path

import pytest

from scripts import demo_waypoint as demo
from scripts.run_record import RunRecord


def fixture_root(tmp_path):
    root = tmp_path / "project with spaces"
    (root / "outputs/training_v2").mkdir(parents=True)
    (root / "outputs/training_v2/policy.zip").write_bytes(b"trusted PPO weights")
    for relative, content in (
        ("scripts/train_waypoint.py", "# trainer\n"),
        ("scripts/waypoint_env.py", 'ASSET_URL = "https://assets.example/leatherback.usd"\n'),
        ("scripts/run_record.py", "# recorder\n"),
        ("scripts/demo_waypoint.py", "# launcher\n"),
        ("experiments/probe_support.py", "# provenance helpers\n"),
        ("hivemind/launcher.py", "# portable runtime discovery\n"),
    ):
        target = root / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(content, encoding="utf-8")
    return root


def parsed(*values):
    return demo.parser().parse_args(list(values))


def complete_summary(**overrides):
    summary = {
        "mode": "evaluate", "num_envs": 1, "requested_steps": 1800,
        "executed_control_steps": 1800, "successes": 2, "failures": 0,
        "timeouts": 0, "completed_episodes": 2, "success_rate": 1.0,
        "elapsed_s": 12.5,
    }
    summary.update(overrides)
    return summary


def test_check_plan_is_fixed_inference_and_uses_unique_output(tmp_path):
    root = fixture_root(tmp_path)
    launcher = tmp_path / "IsaacLab" / "isaaclab.bat"
    launcher.parent.mkdir()
    launcher.touch()
    plan = demo.build_plan(parsed(), root, {"ISAACLAB_PATH": str(launcher.parent)}, windows=True)

    assert plan["config"] == {
        "mode": "evaluate", "steps": 1800, "num_envs": 1, "seed": 43,
        "device": "cuda:0", "gui": False, "capture": False,
        "camera": "overview", "hold_open": False, "deadline_seconds": 300,
        "max_gpu_fraction": 0.90, "checkpoint": str(root / "outputs/training_v2/policy.zip"),
    }
    assert plan["command"][plan["command"].index("--mode") + 1] == "evaluate"
    assert "--headless" in plan["command"]
    assert plan["output"].parent == root / "outputs/waypoint_demo"
    assert plan["asset"]["content_sha256"] is None
    assert plan["asset"]["uri_sha256"] == hashlib.sha256(b"https://assets.example/leatherback.usd").hexdigest()
    assert not plan["output"].exists()


def test_optional_gui_capture_is_bounded_and_headless_by_default(tmp_path):
    root = fixture_root(tmp_path)
    launcher = tmp_path / "IsaacLab" / "isaaclab.bat"
    launcher.parent.mkdir()
    launcher.touch()
    args = parsed("--gui", "--capture", "--steps", "1200", "--deadline-seconds", "240")
    plan = demo.build_plan(args, root, {"ISAACLAB_PATH": str(launcher.parent)}, windows=True)
    assert "--viz" in plan["command"]
    assert "--capture" in plan["command"]
    assert plan["command"][plan["command"].index("--capture") + 1].endswith("waypoint-overview.png")
    with pytest.raises(ValueError, match="bounded"):
        demo.build_plan(parsed("--steps", str(demo.MAX_STEPS + 1)), root,
                        {"ISAACLAB_PATH": str(launcher.parent)}, windows=True)
    with pytest.raises(ValueError, match="requires --gui"):
        demo.build_plan(parsed("--capture"), root, {"ISAACLAB_PATH": str(launcher.parent)}, windows=True)


def test_check_reports_hashes_without_creating_outputs(tmp_path, monkeypatch, capsys):
    root = fixture_root(tmp_path)
    launcher = tmp_path / "IsaacLab" / "isaaclab.bat"
    launcher.parent.mkdir()
    launcher.touch()
    build_plan = demo.build_plan
    monkeypatch.setattr(demo, "ROOT", root)
    monkeypatch.setattr(demo, "build_plan", lambda args: build_plan(
        args, root=root, env={"ISAACLAB_PATH": str(launcher.parent)}, windows=True))
    monkeypatch.setattr(demo.subprocess, "Popen", lambda *args, **kwargs: pytest.fail("started Isaac"))
    assert demo.main(["--check"]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["no_simulator_started"] is True
    assert report["checkpoint_sha256"] == hashlib.sha256(b"trusted PPO weights").hexdigest()
    assert set(report["source_hashes"]) == set(demo.SOURCE_FILES)
    assert not (root / "outputs/waypoint_demo").exists()


def test_source_snapshot_must_still_match_at_end(tmp_path):
    root = fixture_root(tmp_path)
    output = root / "outputs/waypoint_demo/run"
    output.mkdir(parents=True)
    hashes = demo.source_snapshot(root, output)
    assert demo.source_snapshot_matches(root, output, hashes)
    (output / "source/scripts/waypoint_env.py").write_text("changed snapshot", encoding="utf-8")
    assert not demo.source_snapshot_matches(root, output, hashes)


def test_launcher_requires_checkpoint_and_runtime(tmp_path):
    root = fixture_root(tmp_path)
    with pytest.raises(FileNotFoundError, match="Isaac Lab launcher"):
        demo.build_plan(parsed(), root, {"ISAACLAB_PATH": str(tmp_path / "missing")}, windows=True)
    with pytest.raises(FileNotFoundError, match="checkpoint"):
        demo.build_plan(parsed("--checkpoint", "missing.zip"), root,
                        {"ISAACLAB_PATH": str(tmp_path / "missing")}, windows=True)


def test_run_record_override_stays_inside_outputs(tmp_path, monkeypatch):
    root = fixture_root(tmp_path)
    escapes = (tmp_path / "outside", root / "outputs" / ".." / "outside")
    for escaped in escapes:
        monkeypatch.setenv("HIGHWAYSIM_RUN_RECORD_ROOT", str(escaped))
        with pytest.raises(ValueError, match="inside the project outputs"):
            RunRecord(root, {"mode": "evaluate"}, ["--mode", "evaluate"])
        assert not escaped.resolve().exists()

    record_root = root / "outputs/waypoint_demo/test-run/training-record"
    monkeypatch.setenv("HIGHWAYSIM_RUN_RECORD_ROOT", str(record_root))
    record = RunRecord(root, {"mode": "evaluate"}, ["--mode", "evaluate"])
    assert record.directory.is_relative_to(root / "outputs")
    assert (record.directory / "manifest.json").is_file()


def test_assessment_rejects_missing_incomplete_and_failed_summaries():
    missing = demo.assess_evaluation(None, 1800, 0)
    assert not missing["passed"]
    assert any("missing" in error for error in missing["assessment_errors"])

    incomplete = demo.assess_evaluation(
        complete_summary(executed_control_steps=1799), 1800, 0)
    assert not incomplete["passed"]
    assert any("full requested" in error for error in incomplete["assessment_errors"])

    failed_episode = demo.assess_evaluation(
        complete_summary(successes=1, failures=1, completed_episodes=2, success_rate=0.5), 1800, 0)
    assert not failed_episode["passed"]
    assert any("failures or timeouts" in error for error in failed_episode["assessment_errors"])


def test_assessment_requires_requested_capture_and_stable_checkpoint(tmp_path):
    capture = tmp_path / "capture.png"
    missing = demo.assess_evaluation(complete_summary(), 1800, 0, capture_path=capture,
                                     capture_requested=True, checkpoint_unchanged=False)
    assert not missing["passed"]
    assert any("screenshot" in error for error in missing["assessment_errors"])
    assert any("Checkpoint changed" in error for error in missing["assessment_errors"])

    capture.write_bytes(b"png")
    passed = demo.assess_evaluation(complete_summary(), 1800, 0,
                                    capture_path=capture, capture_requested=True)
    assert passed["passed"]
    assert passed["capture"]["sha256"] == hashlib.sha256(b"png").hexdigest()
    assert passed["unfinished_episode_included"] is False
    assert "excluded" in passed["unfinished_episode_note"]


@pytest.mark.parametrize("field,value", [
    ("num_envs", True), ("requested_steps", True), ("executed_control_steps", True),
    ("successes", True), ("failures", 0.0), ("timeouts", False),
    ("completed_episodes", 2.0),
])
def test_assessment_rejects_boolean_or_float_integer_fields(field, value):
    summary = complete_summary()
    summary[field] = value
    result = demo.assess_evaluation(summary, 1800, 0)
    assert not result["passed"]


def test_assessment_rejects_boolean_step_fields_even_when_requested_step_is_one():
    summary = complete_summary(num_envs=True, requested_steps=True, executed_control_steps=True)
    result = demo.assess_evaluation(summary, 1, 0)
    assert not result["passed"]
    assert any("num_envs" in error for error in result["assessment_errors"])
    assert any("requested_steps" in error for error in result["assessment_errors"])
    assert any("full requested" in error for error in result["assessment_errors"])


class FakeChild:
    pid = 31415

    def __init__(self):
        self.returncode = None
        self.terminated = False

    def poll(self):
        return self.returncode

    def terminate(self):
        self.terminated = True
        self.returncode = -15

    def kill(self):
        self.returncode = -9

    def wait(self, timeout=None):
        if self.returncode is None:
            self.returncode = 0
        return self.returncode


def test_stop_child_terminates_posix_process_group(tmp_path, monkeypatch):
    child = FakeChild()
    sent = []
    monkeypatch.setattr(demo, "os", type("FakeOS", (), {
        "name": "posix", "killpg": staticmethod(lambda pid, sig: sent.append((pid, sig)))})())
    result = demo.stop_child(child)
    assert result["stopped"]
    assert sent == [(child.pid, demo.signal.SIGTERM),
                    (child.pid, getattr(demo.signal, "SIGKILL", 9))]


def test_stop_child_bounds_windows_taskkill_and_falls_back(tmp_path, monkeypatch):
    child = FakeChild()
    calls = []
    monkeypatch.setattr(demo, "os", type("FakeOS", (), {"name": "nt"})())
    def timed_taskkill(command, **kwargs):
        calls.append(kwargs.get("timeout"))
        raise demo.subprocess.TimeoutExpired(command, kwargs["timeout"])
    monkeypatch.setattr(demo.subprocess, "run", timed_taskkill)
    result = demo.stop_child(child, terminate_wait_seconds=2)
    assert calls == [2]
    assert child.terminated
    assert result["stopped"] is False
    assert result["errors"]


def test_keyboard_interrupt_writes_failed_run_package(tmp_path, monkeypatch, capsys):
    root = fixture_root(tmp_path)
    lab_launcher = tmp_path / "IsaacLab" / "isaaclab.bat"
    lab_launcher.parent.mkdir()
    lab_launcher.touch()
    output = root / "outputs/waypoint_demo/interrupt-run"
    checkpoint = root / "outputs/training_v2/policy.zip"
    plan = {
        "root": root, "checkpoint": checkpoint, "launcher": lab_launcher,
        "output": output, "command": [str(lab_launcher), "-p", "scripts/train_waypoint.py"],
        "asset": {"uri": "asset://test", "content_sha256": None},
        "config": {"mode": "evaluate", "steps": 1, "num_envs": 1,
                   "checkpoint": str(checkpoint)},
    }
    monkeypatch.setattr(demo, "ROOT", root)
    monkeypatch.setattr(demo, "build_plan", lambda args: plan)
    monkeypatch.setattr(demo, "isaac_sim_version", lambda *args: {"path": "fake", "version": "test"})
    monkeypatch.setattr(demo, "gpu_sample", lambda: {"used_mib": 1.0, "total_mib": 100.0})

    class Monitor:
        rows = [{"used_mib": 1.0, "total_mib": 100.0}]
        errors = []
        def __enter__(self):
            return self
        def __exit__(self, *args):
            return False

    monkeypatch.setattr(demo, "GpuMonitor", Monitor)
    child = FakeChild()
    monkeypatch.setattr(demo.subprocess, "Popen", lambda *args, **kwargs: child)
    monkeypatch.setattr(demo.time, "sleep", lambda _: (_ for _ in ()).throw(KeyboardInterrupt()))
    def stop_and_mutate_snapshot(process):
        (output / "source/scripts/waypoint_env.py").write_text("changed snapshot", encoding="utf-8")
        process.returncode = -9
        return {"stopped": True, "method": "test", "errors": []}
    monkeypatch.setattr(demo, "stop_child", stop_and_mutate_snapshot)

    def fake_run(command, **kwargs):
        if command[1:2] == ["diff"]:
            return type("Completed", (), {"returncode": 0, "stdout": b""})()
        output_text = "commit\n" if command[1:2] == ["rev-parse"] else ""
        return type("Completed", (), {"returncode": 0, "stdout": output_text})()
    monkeypatch.setattr(demo.subprocess, "run", fake_run)

    assert demo.main(["--steps", "1"]) == 130
    result = json.loads((output / "summary.json").read_text(encoding="utf-8"))
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    assert result["interrupted"] is True
    assert result["source_changed_during_run"] == []
    assert result["source_snapshot_matches_at_end"] is False
    assert manifest["status"] == "failed"
    assert "KeyboardInterrupt" in capsys.readouterr().out
