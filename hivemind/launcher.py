"""Portable launcher only: simulation and rendering remain in their own layers."""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
DEMOS = ("sumo", "random", "trained", "record", "lidar", "replay", "ovrtx")
KEYS = {"traffic_python", "ovrtx_python", "isaac_root", "checkpoint",
        "sumo_binary", "sumo_gui_binary"}


class SetupError(Exception):
    """An actionable setup problem rather than an application traceback."""


def load_config(root: Path) -> dict:
    path = root / "hivemind.local.json"
    if not path.exists():
        return {}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise SetupError(f"Cannot read {path}: {exc}") from exc
    if not isinstance(data, dict) or set(data) - KEYS:
        raise SetupError(f"{path.name} must be an object with only: {', '.join(sorted(KEYS))}")
    if any(not isinstance(v, str) for v in data.values()):
        raise SetupError(f"{path.name}: every value must be a path string (or empty string).")
    return data


def setting(config: dict, key: str) -> str:
    return os.environ.get("HIVEMIND_" + key.upper()) or config.get(key, "")


def resolve_path(value: str, root: Path) -> Path:
    path = Path(os.path.expandvars(value)).expanduser()
    return (path if path.is_absolute() else root / path).resolve()


def require_file(path: Path, hint: str) -> Path:
    if not path.is_file():
        raise SetupError(f"Missing file: {path}\n{hint}")
    return path


def venv_python(root: Path, name: str, windows: bool | None = None) -> Path:
    windows = os.name == "nt" if windows is None else windows
    return root / name / ("Scripts/python.exe" if windows else "bin/python")


def python_runtime(root: Path, config: dict, kind: str) -> Path:
    key = f"{kind}_python"
    hint = f"Set {key} in hivemind.local.json; see documentation/portable-demos.md."
    if value := setting(config, key):
        return require_file(resolve_path(value, root), hint)
    candidate = venv_python(root, ".venv" if kind == "traffic" else ".venv-ovrtx")
    if candidate.is_file():
        return candidate
    # An explicitly activated environment also works without the conventional name.
    return require_file(Path(sys.executable), hint)


def isaac_runtime(root: Path, config: dict) -> Path:
    value = setting(config, "isaac_root") or os.environ.get("ISAAC_SIM_PATH")
    if not value:
        raise SetupError("Isaac Sim not configured. Set isaac_root in hivemind.local.json "
                         "or ISAAC_SIM_PATH to its installation directory. "
                         "Install a compatible Isaac runtime/GPU first; it is not in Git.")
    return require_file(resolve_path(value, root) / ("python.bat" if os.name == "nt" else "python.sh"),
                        "isaac_root must contain Isaac's python.bat (Windows) or python.sh (Linux).")


def sumo_runtime(root: Path, config: dict, gui: bool) -> Path:
    key = "sumo_gui_binary" if gui else "sumo_binary"
    program = "sumo-gui" if gui else "sumo"
    hint = f"Install SUMO and add it to PATH, set SUMO_HOME, or configure {key}."
    if value := setting(config, key):
        return require_file(resolve_path(value, root), hint)
    # Match the backend's explicit override. Users must select the correct GUI/headless binary.
    if value := os.environ.get("SUMO_BINARY"):
        return require_file(resolve_path(value, root), hint)
    if value := shutil.which(program):
        return Path(value).resolve()
    executable = program + (".exe" if os.name == "nt" else "")
    candidates = []
    if value := os.environ.get("SUMO_HOME"):
        candidates.append(Path(value) / "bin" / executable)
    if os.name == "nt":
        for key in ("ProgramFiles", "ProgramFiles(x86)"):
            if value := os.environ.get(key):
                candidates.append(Path(value) / "Eclipse/Sumo/bin" / executable)
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise SetupError(hint)


def check_modules(runtime: Path, modules: tuple[str, ...], root: Path) -> None:
    # Probe the selected interpreter, not the launcher's interpreter. No GPU imports.
    code = ("import importlib.util,sys; "
            "missing=[m for m in sys.argv[1:] if importlib.util.find_spec(m) is None]; "
            "print('Missing Python modules: '+', '.join(missing)) if missing else None; "
            "sys.exit(bool(missing))")
    try:
        result = subprocess.run([str(runtime), "-c", code, *modules], cwd=root,
                                capture_output=True, text=True, timeout=60)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise SetupError(f"Could not check interpreter {runtime}: {exc}") from exc
    if result.returncode:
        detail = (result.stdout + result.stderr).strip()[-2000:]
        raise SetupError(f"Dependency check failed for {runtime}\n{detail}\n"
                         "See documentation/portable-demos.md for the separate runtime setup. "
                         "Nothing was installed automatically.")


def plan(args, root: Path, config: dict) -> tuple[list[str], dict, tuple[str, ...]]:
    demo = args.demo
    env = os.environ.copy()
    if demo in ("sumo", "random", "trained", "record"):
        runtime = python_runtime(root, config, "traffic")
        gui = demo != "record" and not args.headless
        env["SUMO_BINARY"] = str(sumo_runtime(root, config, gui))
        modules = ("traci", "sumolib", "numpy", "gymnasium", "pettingzoo", "torch")
        if demo == "record":
            command = [str(runtime), "scripts/record_sumo_replay.py", "--source-repo", str(root)]
            if not args.overwrite and any((root / "outputs/sumo_replay" / f).exists()
                                          for f in ("recording.json", "network.net.xml")):
                raise SetupError("A replay recording/network already exists. Back it up first, "
                                 "then use --overwrite only if replacing it is intended.")
        else:
            command = [str(runtime), "-m", "experiments.rollout", "--policy",
                       "scripted" if demo == "sumo" else demo, "--horizon", str(args.horizon)]
            if gui:
                command += ["--gui", "--gui-delay-ms", str(args.delay)]
        seed = args.seed if args.seed is not None else (2001 if demo in ("trained", "record") else 42)
        command += ["--seed", str(seed)]
        if demo in ("trained", "record"):
            value = args.checkpoint or setting(config, "checkpoint")
            if not value:
                raise SetupError("No checkpoint selected. Pass --checkpoint PATH or set checkpoint "
                                 "in hivemind.local.json. Use a trusted 16-observation checkpoint "
                                 "from this project; see documentation/portable-demos.md. "
                                 "No model is bundled or downloaded automatically.")
            command += ["--checkpoint", str(require_file(resolve_path(value, root),
                        "Choose a compatible local checkpoint; see documentation/portable-demos.md."))]
    else:
        if demo in ("replay", "ovrtx"):
            for name in ("recording.json", "network.net.xml"):
                require_file(root / "outputs/sumo_replay" / name,
                             "Create replay inputs with: python -m hivemind demo record --checkpoint PATH")
        if demo == "ovrtx":
            runtime = python_runtime(root, config, "ovrtx")
            modules = ("ovrtx", "ovstage", "pxr", "numpy", "PIL")
            command = [str(runtime), "scripts/render_ovrtx.py", "--time", str(args.time)]
        else:
            runtime = isaac_runtime(root, config)
            modules = ("isaacsim", "numpy")
            script = "smoke_rtx_lidar.py" if demo == "lidar" else "traffic_replay_3d.py"
            frames = args.frames or (1800 if demo == "lidar" else 2900)
            command = [str(runtime), f"scripts/{script}", "--frames", str(frames)]
            if not args.headless:
                command += ["--gui"]
    return command, env, modules


def positive(value: str) -> int:
    result = int(value)
    if result <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return result


def nonnegative(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result < 0:
        raise argparse.ArgumentTypeError("must be finite and nonnegative")
    return result


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description="Portable demos; run from the repository root.")
    commands = result.add_subparsers(dest="action", required=True)
    run = commands.add_parser("demo", help="check dependencies, then launch one existing demo")
    run.add_argument("demo", choices=DEMOS)
    run.add_argument("--check", action="store_true", help="check setup only; no simulation or rendering")
    run.add_argument("--checkpoint", help="trusted 16-observation PPO checkpoint; relative to repo root")
    run.add_argument("--headless", action="store_true")
    run.add_argument("--seed", type=int, help="default: 2001 for trained/record, otherwise 42")
    run.add_argument("--horizon", type=positive, default=400)
    run.add_argument("--delay", type=positive, default=100, help="SUMO GUI delay in milliseconds")
    run.add_argument("--frames", type=positive, help="Isaac lidar/replay frame budget")
    run.add_argument("--time", type=nonnegative, default=2, help="ovrtx replay time in seconds")
    run.add_argument("--overwrite", action="store_true", help="allow record to replace existing replay inputs")
    return result


def main(argv=None) -> int:
    args = parser().parse_args(argv)
    try:
        command, env, modules = plan(args, ROOT, load_config(ROOT))
        check_modules(Path(command[0]), modules, ROOT)
        print("Repository:", ROOT, flush=True)
        print("Launch arguments:", json.dumps(command), flush=True)
        if args.check:
            print("Setup check passed. No simulation started; GPU, driver and checkpoint compatibility "
                  "are not validated by this check.")
            return 0
        if args.demo in ("trained", "record"):
            print("Only use trusted checkpoints: the existing policy loader uses pickle deserialization.", flush=True)
        if args.demo in ("replay", "ovrtx"):
            print("This refreshes generated replay/render artifacts, not source code or trained weights.", flush=True)
        completed = subprocess.run(command, cwd=ROOT, env=env)
        if completed.returncode:
            print(f"Demo exited with status {completed.returncode}; inspect its output above.", file=sys.stderr)
        return completed.returncode
    except (SetupError, OSError) as exc:
        print(f"Setup needed: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("Demo interrupted.", file=sys.stderr)
        return 130
