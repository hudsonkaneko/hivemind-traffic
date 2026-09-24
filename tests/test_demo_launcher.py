"""Launcher contract tests require neither SUMO nor an RTX GPU."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from hivemind import launcher as cli


@pytest.fixture
def project(tmp_path, monkeypatch):
    root = tmp_path / "a project with spaces"
    root.mkdir()
    for key in (*cli.KEYS,):
        monkeypatch.delenv("HIVEMIND_" + key.upper(), raising=False)
    monkeypatch.delenv("SUMO_BINARY", raising=False)
    monkeypatch.delenv("SUMO_HOME", raising=False)
    monkeypatch.delenv("ISAAC_SIM_PATH", raising=False)
    for name in (".venv", ".venv-ovrtx"):
        path = cli.venv_python(root, name)
        path.parent.mkdir(parents=True)
        path.touch()
    binary = root / "sumo tool"
    binary.touch()
    monkeypatch.setattr(cli.shutil, "which", lambda name: str(binary))
    monkeypatch.setattr(cli, "ROOT", root)
    return root


def args(*values):
    return cli.parser().parse_args(["demo", *values])


def test_windows_and_posix_layout(tmp_path):
    assert cli.venv_python(tmp_path, ".venv", True) == tmp_path / ".venv/Scripts/python.exe"
    assert cli.venv_python(tmp_path, ".venv", False) == tmp_path / ".venv/bin/python"


@pytest.mark.parametrize("value", [[], {"typo": "x"}, {"checkpoint": 5}])
def test_invalid_config(project, value):
    (project / "hivemind.local.json").write_text(json.dumps(value))
    with pytest.raises(cli.SetupError):
        cli.load_config(project)


def test_bad_json(project):
    (project / "hivemind.local.json").write_text("{")
    with pytest.raises(cli.SetupError, match="Cannot read"):
        cli.load_config(project)


def test_env_override(project, monkeypatch):
    runtime = project / "alternate python"
    runtime.touch()
    monkeypatch.setenv("HIVEMIND_TRAFFIC_PYTHON", str(runtime))
    assert cli.python_runtime(project, {"traffic_python": "missing"}, "traffic") == runtime


def test_missing_explicit_runtime_does_not_fallback(project):
    with pytest.raises(cli.SetupError, match="Missing file"):
        cli.python_runtime(project, {"traffic_python": "missing"}, "traffic")


def test_sumo_plan_spaces_and_environment(project):
    command, env, modules = cli.plan(args("sumo"), project, {})
    assert command[0] == str(cli.venv_python(project, ".venv"))
    assert command[1:5] == ["-m", "experiments.rollout", "--policy", "scripted"]
    assert "--gui" in command
    assert command[-2:] == ["--seed", "42"]
    assert env["SUMO_BINARY"] == str(project / "sumo tool")
    assert "torch" in modules


def test_headless_and_configured_sumo(project):
    path = project / "custom sumo"
    path.touch()
    command, env, _ = cli.plan(args("random", "--headless"), project, {"sumo_binary": "custom sumo"})
    assert "--gui" not in command
    assert env["SUMO_BINARY"] == str(path)


def test_trained_requires_checkpoint(project):
    with pytest.raises(cli.SetupError, match="No checkpoint"):
        cli.plan(args("trained"), project, {})


def test_checkpoint_cli_precedes_config(project):
    path = project / "trusted.pt"
    path.touch()
    command, _, _ = cli.plan(args("trained", "--checkpoint", "trusted.pt"), project, {"checkpoint": "missing"})
    assert command[-4:] == ["--seed", "2001", "--checkpoint", str(path)]


def test_record_overwrite_guard(project):
    out = project / "outputs/sumo_replay"
    out.mkdir(parents=True)
    (out / "recording.json").touch()
    (project / "trusted.pt").touch()
    with pytest.raises(cli.SetupError, match="already exists"):
        cli.plan(args("record"), project, {})
    command, _, _ = cli.plan(args("record", "--overwrite", "--checkpoint", "trusted.pt"), project, {})
    assert command[1:4] == ["scripts/record_sumo_replay.py", "--source-repo", str(project)]


@pytest.mark.parametrize("demo", ["replay", "ovrtx"])
def test_missing_replay_assets(project, demo):
    with pytest.raises(cli.SetupError, match="demo record"):
        cli.plan(args(demo), project, {})


def test_isaac_configuration(project):
    with pytest.raises(cli.SetupError, match="not configured"):
        cli.plan(args("lidar"), project, {})
    runtime = project / "isaac runtime"
    runtime.mkdir()
    (runtime / ("python.bat" if cli.os.name == "nt" else "python.sh")).touch()
    command, _, _ = cli.plan(args("lidar", "--frames", "12"), project, {"isaac_root": "isaac runtime"})
    assert command[1:] == ["scripts/smoke_rtx_lidar.py", "--frames", "12", "--gui"]


def test_ovrtx_plan(project):
    out = project / "outputs/sumo_replay"
    out.mkdir(parents=True)
    for name in ("recording.json", "network.net.xml"):
        (out / name).touch()
    command, _, modules = cli.plan(args("ovrtx", "--time", "15"), project, {})
    assert command == [str(cli.venv_python(project, ".venv-ovrtx")), "scripts/render_ovrtx.py", "--time", "15.0"]
    assert "ovrtx" in modules


def test_check_does_not_launch_or_load_checkpoint(project, monkeypatch):
    (project / "not really a model.pt").touch()
    seen = []
    monkeypatch.setattr(cli, "check_modules", lambda *values: seen.append(values))
    monkeypatch.setattr(cli.subprocess, "run", lambda *a, **kw: pytest.fail("launched child"))
    assert cli.main(["demo", "trained", "--checkpoint", "not really a model.pt", "--check"]) == 0
    assert len(seen) == 1


def test_child_cwd_and_exit_status(project, monkeypatch):
    monkeypatch.setattr(cli, "check_modules", lambda *values: None)
    seen = {}
    def run(command, **kwargs):
        seen.update(kwargs)
        return SimpleNamespace(returncode=7)
    monkeypatch.setattr(cli.subprocess, "run", run)
    assert cli.main(["demo", "sumo", "--headless"]) == 7
    assert seen["cwd"] == project
    assert "shell" not in seen


def test_missing_module_is_actionable(project, monkeypatch):
    monkeypatch.setattr(cli.subprocess, "run", lambda *a, **k: SimpleNamespace(returncode=1, stdout="Missing Python modules: torch", stderr=""))
    with pytest.raises(cli.SetupError, match="torch"):
        cli.check_modules(Path("python"), ("torch",), project)


@pytest.mark.parametrize("option,value", [("--frames", "0"), ("--horizon", "-1"), ("--time", "nan"), ("--time", "-1")])
def test_invalid_numeric_options(option, value):
    with pytest.raises(SystemExit):
        args("sumo", option, value)
