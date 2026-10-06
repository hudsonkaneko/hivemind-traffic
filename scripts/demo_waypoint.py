"""Bounded launcher for inference with the existing Isaac Lab waypoint PPO."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import signal
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from experiments.probe_support import GpuMonitor, finish, gpu_sample, write_json
from hivemind.launcher import load_config, resolve_path

CHECKPOINT_DEFAULT = Path("outputs/training_v2/policy.zip")
SOURCE_FILES = (
    "scripts/demo_waypoint.py",
    "scripts/train_waypoint.py",
    "scripts/waypoint_env.py",
    "scripts/run_record.py",
    "experiments/probe_support.py",
    "hivemind/launcher.py",
)
MAX_STEPS = 7200
MAX_DEADLINE_SECONDS = 600


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def asset_reference(root):
    source = (root / "scripts/waypoint_env.py").read_text(encoding="utf-8")
    for line in source.splitlines():
        if line.startswith("ASSET_URL = "):
            uri = line.split("=", 1)[1].strip().strip("\"'")
            return {"uri": uri, "uri_sha256": hashlib.sha256(uri.encode("utf-8")).hexdigest(),
                    "content_sha256": None,
                    "hash_note": "URI reference only; remote USD bytes are not downloaded by this launcher"}
    raise ValueError("Could not find the waypoint USD asset reference in scripts/waypoint_env.py")


def positive_int(value):
    try:
        number = int(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("must be an integer") from error
    if number < 1:
        raise argparse.ArgumentTypeError("must be positive")
    return number


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--check", action="store_true", help="Validate inputs and print the plan without launching Isaac")
    result.add_argument("--checkpoint", type=Path, default=CHECKPOINT_DEFAULT)
    result.add_argument("--steps", type=positive_int, default=1800)
    result.add_argument("--seed", type=int, default=43)
    result.add_argument("--gui", action="store_true", help="Show the Isaac overview window")
    result.add_argument("--capture", action="store_true", help="Save one PNG from the GUI run")
    result.add_argument("--deadline-seconds", type=positive_int, default=300)
    result.add_argument("--max-gpu-fraction", type=float, default=0.90)
    return result


def resolve(root, value):
    path = Path(value).expanduser()
    return (path if path.is_absolute() else root / path).resolve()


def isaaclab_launcher(env, windows=None):
    windows = os.name == "nt" if windows is None else windows
    root_value = env.get("ISAACLAB_PATH")
    if root_value:
        lab_root = Path(os.path.expandvars(root_value)).expanduser()
    else:
        home = Path(env.get("USERPROFILE", str(Path.home())))
        lab_root = home / "Documents" / "IsaacLab"
    return (lab_root / ("isaaclab.bat" if windows else "isaaclab.sh")).resolve()


def build_plan(args, root=ROOT, env=None, windows=None):
    env = os.environ if env is None else env
    windows = os.name == "nt" if windows is None else windows
    if args.steps > MAX_STEPS:
        raise ValueError(f"--steps is bounded to at most {MAX_STEPS}")
    if args.deadline_seconds > MAX_DEADLINE_SECONDS:
        raise ValueError(f"--deadline-seconds is bounded to at most {MAX_DEADLINE_SECONDS}")
    if not 0.1 <= args.max_gpu_fraction <= 0.99:
        raise ValueError("--max-gpu-fraction must be between 0.10 and 0.99")
    if args.capture and not args.gui:
        raise ValueError("--capture requires --gui")

    root = Path(root).resolve()
    checkpoint = resolve(root, args.checkpoint)
    if not checkpoint.is_file():
        raise FileNotFoundError(f"Waypoint PPO checkpoint not found: {checkpoint}")
    launcher = isaaclab_launcher(env, windows)
    if not launcher.is_file():
        raise FileNotFoundError(f"Isaac Lab launcher not found: {launcher}; set ISAACLAB_PATH if needed")
    for relative in SOURCE_FILES:
        if not (root / relative).is_file():
            raise FileNotFoundError(f"Required source file not found: {root / relative}")

    asset = asset_reference(root)
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:8]
    output = root / "outputs" / "waypoint_demo" / run_id
    command = [str(launcher), "-p", "scripts/train_waypoint.py", "--mode", "evaluate",
               "--steps", str(args.steps), "--num_envs", "1", "--seed", str(args.seed),
               "--checkpoint", str(checkpoint), "--output", str(output / "evaluation")]
    command += ["--device", "cuda:0"]
    if args.gui:
        command += ["--viz", "kit"]
    else:
        command += ["--headless"]
    if args.capture:
        command += ["--capture", str(output / "waypoint-overview.png")]
    return {
        "root": root,
        "checkpoint": checkpoint,
        "launcher": launcher,
        "output": output,
        "command": command,
        "asset": asset,
        "config": {
            "mode": "evaluate", "steps": args.steps, "num_envs": 1, "seed": args.seed,
            "device": "cuda:0", "gui": args.gui, "capture": args.capture,
            "camera": "overview", "hold_open": False,
            "deadline_seconds": args.deadline_seconds,
            "max_gpu_fraction": args.max_gpu_fraction,
            "checkpoint": str(checkpoint),
        },
    }


def process_command(command, windows=None):
    windows = os.name == "nt" if windows is None else windows
    if windows:
        # Isaac Lab's public entry point is a batch file, which cmd.exe must interpret.
        return [os.environ.get("COMSPEC", "cmd.exe"), "/d", "/c", "call", *command]
    return command


def source_snapshot(root, output):
    hashes = {}
    for relative in SOURCE_FILES:
        source = root / relative
        hashes[relative] = sha256_file(source)
        target = output / "source" / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(source.read_bytes())
    return hashes


def source_snapshot_matches(root, output, hashes):
    return all(
        (root / relative).is_file()
        and sha256_file(root / relative) == digest
        and (output / "source" / relative).is_file()
        and sha256_file(output / "source" / relative) == digest
        for relative, digest in hashes.items()
    )


def isaac_sim_version(root, env=None):
    env = os.environ if env is None else env
    configured = env.get("ISAAC_SIM_PATH") or load_config(root).get("isaac_root")
    if configured:
        sim_root = resolve_path(configured, root)
    elif os.name == "nt":
        sim_root = Path("C:/isaacsim")
    else:
        return {"path": None, "version": None}
    version_file = sim_root / "VERSION"
    version = version_file.read_text(encoding="utf-8").strip() if version_file.is_file() else None
    return {"path": str(sim_root), "version": version}


def assess_evaluation(summary, requested_steps, exit_code, stop_reason=None,
                      capture_path=None, capture_requested=False,
                      checkpoint_unchanged=True):
    errors = []
    if exit_code != 0:
        errors.append(f"Isaac Lab process exited with code {exit_code}")
    if stop_reason:
        errors.append(stop_reason)
    if not isinstance(summary, dict):
        errors.append("Evaluation summary is missing or is not a JSON object")
        summary = {}
    if summary.get("mode") != "evaluate":
        errors.append("Evaluation summary mode is not evaluate")
    if type(summary.get("num_envs")) is not int or summary["num_envs"] != 1:
        errors.append("Evaluation summary num_envs is not 1")
    if type(summary.get("requested_steps")) is not int or summary["requested_steps"] != requested_steps:
        errors.append("Evaluation summary requested_steps does not match the resolved configuration")
    if (type(summary.get("executed_control_steps")) is not int
            or summary["executed_control_steps"] != requested_steps):
        errors.append("Evaluation did not execute the full requested control-step budget")

    counts = {}
    for name in ("successes", "failures", "timeouts", "completed_episodes"):
        value = summary.get(name)
        if type(value) is not int:
            errors.append(f"Evaluation summary {name} is missing or is not an integer")
        elif value < 0:
            errors.append(f"Evaluation summary {name} is not a nonnegative whole number")
        else:
            counts[name] = value
    if len(counts) == 4:
        if counts["completed_episodes"] != counts["successes"] + counts["failures"] + counts["timeouts"]:
            errors.append("completed_episodes does not equal successes + failures + timeouts")
        if counts["successes"] < 1:
            errors.append("Evaluation completed no successful waypoint episodes")
        if counts["failures"] != 0 or counts["timeouts"] != 0:
            errors.append("Evaluation recorded failures or timeouts")

    success_rate = summary.get("success_rate")
    if isinstance(success_rate, bool) or not isinstance(success_rate, (int, float)) or not math.isfinite(success_rate):
        errors.append("Evaluation summary success_rate is missing or non-finite")
    elif ("completed_episodes" in counts and "successes" in counts
          and counts["completed_episodes"]
          and abs(success_rate - counts["successes"] / counts["completed_episodes"]) > 1e-9):
        errors.append("success_rate does not match the completed episode counts")
    elapsed = summary.get("elapsed_s")
    if isinstance(elapsed, bool) or not isinstance(elapsed, (int, float)) or not math.isfinite(elapsed) or elapsed < 0:
        errors.append("Evaluation summary elapsed_s is missing, negative, or non-finite")

    capture_info = None
    if capture_requested:
        path = Path(capture_path) if capture_path is not None else None
        if path is None or not path.is_file() or path.stat().st_size <= 0:
            errors.append("Requested GUI screenshot is missing or empty")
        else:
            capture_info = {"path": str(path), "bytes": path.stat().st_size,
                            "sha256": sha256_file(path)}
    if not checkpoint_unchanged:
        errors.append("Checkpoint changed or disappeared during the run")

    return {
        "passed": not errors,
        "assessment_errors": errors,
        "completed_episodes": counts.get("completed_episodes"),
        "unfinished_episode_included": False,
        "unfinished_episode_note": (
            "Any episode still active at the control-step budget is excluded from completed_episodes and success_rate; "
            "the summary does not estimate its outcome."
        ),
        "capture": capture_info,
    }


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        plan = build_plan(args)
    except (ValueError, OSError) as error:
        print(f"Waypoint demo setup error: {error}", file=sys.stderr)
        return 2

    if args.check:
        report = {key: str(value) if isinstance(value, Path) else value
                  for key, value in plan.items() if key != "output"}
        report["source_hashes"] = {name: sha256_file(ROOT / name) for name in SOURCE_FILES}
        report["checkpoint_sha256"] = sha256_file(plan["checkpoint"])
        report["no_simulator_started"] = True
        print(json.dumps(report, indent=2))
        return 0

    output = plan["output"]
    output.mkdir(parents=True, exist_ok=False)
    source_hashes = {}
    manifest = {
        "schema_version": 1,
        "run_id": output.name,
        "status": "running",
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "project_root": str(ROOT),
        "command": plan["command"],
        "config": plan["config"],
        "source_hashes": source_hashes,
        "asset": plan["asset"],
        "checkpoint": {"path": str(plan["checkpoint"]), "sha256": sha256_file(plan["checkpoint"]),
                       "bytes": plan["checkpoint"].stat().st_size},
        "python": sys.version,
        "platform": platform.platform(),
        "git_commit": None,
        "git_status": None,
        "local_configuration_contents_captured": False,
        "local_configuration_note": (
            "hivemind.local.json is not copied or hashed; the resolved launcher, "
            "command, checkpoint, and simulator details are recorded separately."
        ),
    }
    checkpoint_hash = manifest["checkpoint"]["sha256"]
    checkpoint_bytes = manifest["checkpoint"]["bytes"]
    capture_path = output / "waypoint-overview.png" if args.capture else None
    result = {"passed": False, "error": "Run did not reach a completed evaluation"}
    child = None
    monitor = None
    started = None
    interrupted = False
    try:
        source_hashes = source_snapshot(ROOT, output)
        manifest["source_hashes"] = source_hashes
        commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True,
                                text=True, timeout=5, check=False)
        manifest["git_commit"] = commit.stdout.strip() if commit.returncode == 0 else None
        status = subprocess.run(["git", "status", "--porcelain"], cwd=ROOT, capture_output=True,
                                text=True, timeout=5, check=False)
        manifest["git_status"] = status.stdout.splitlines() if status.returncode == 0 else None
        diff = subprocess.run(["git", "diff", "HEAD", "--binary"], cwd=ROOT, capture_output=True,
                              timeout=10, check=False)
        manifest["git_dirty_diff"] = {
            "bytes": len(diff.stdout),
            "sha256": hashlib.sha256(diff.stdout).hexdigest(),
            "available": diff.returncode == 0,
        }
        lab_git = subprocess.run(["git", "rev-parse", "HEAD"], cwd=plan["launcher"].parent,
                                 capture_output=True, text=True, timeout=5, check=False)
        manifest["isaaclab_git_commit"] = lab_git.stdout.strip() if lab_git.returncode == 0 else None
        manifest["isaac_sim"] = isaac_sim_version(ROOT)
        if diff.returncode != 0:
            raise RuntimeError("Could not capture the tracked Git diff hash before launch")
        if not source_snapshot_matches(ROOT, output, source_hashes):
            raise RuntimeError("Source snapshot changed while it was being captured")
        write_json(output / "resolved-config.json", plan["config"])
        write_json(output / "manifest.json", manifest)
        gpu_before = gpu_sample()
        manifest["gpu_before"] = gpu_before
        write_json(output / "manifest.json", manifest)
        if gpu_before["used_mib"] / gpu_before["total_mib"] >= args.max_gpu_fraction:
            raise RuntimeError("GPU memory guard: current whole-GPU use is above the configured limit")

        launch = process_command(plan["command"])
        reason = None
        started = time.monotonic()
        print("WAYPOINT_DEMO_RUN=" + str(output), flush=True)
        with (output / "runtime.log").open("w", encoding="utf-8") as log, GpuMonitor() as monitor:
            child = subprocess.Popen(launch, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                     creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0,
                                     start_new_session=(os.name != "nt"),
                                     env={**os.environ, "HIGHWAYSIM_RUN_RECORD_ROOT": str(output / "training-record")})
            while child.poll() is None:
                if time.monotonic() - started > args.deadline_seconds:
                    reason = "Process deadline exceeded"
                elif monitor.rows and monitor.rows[-1]["used_mib"] / monitor.rows[-1]["total_mib"] >= args.max_gpu_fraction:
                    reason = "GPU memory guard"
                if reason:
                    cleanup = stop_child(child)
                    if not cleanup["stopped"]:
                        reason += "; process-tree termination could not be confirmed"
                    break
                time.sleep(0.5)
        summary_path = output / "evaluation" / "summary.json"
        evaluation = json.loads(summary_path.read_text(encoding="utf-8")) if summary_path.is_file() else None
        checkpoint_unchanged = (
            plan["checkpoint"].is_file()
            and plan["checkpoint"].stat().st_size == checkpoint_bytes
            and sha256_file(plan["checkpoint"]) == checkpoint_hash
        )
        result = assess_evaluation(
            evaluation, args.steps, child.returncode, reason, capture_path, args.capture,
            checkpoint_unchanged,
        )
        result.update({
            "exit_code": child.returncode,
            "stop_reason": reason,
            "evaluation_summary": evaluation,
            "wall_process_seconds": time.monotonic() - started,
            "gpu_peak_mib": max((row["used_mib"] for row in monitor.rows), default=None),
            "gpu_scope": "Whole GPU including desktop; sampled approximately once per second",
            "gpu_monitor_errors": monitor.errors,
            "asset_content_hash_available": False,
            "checkpoint_unchanged": checkpoint_unchanged,
            "checkpoint_sha256_after": sha256_file(plan["checkpoint"]) if plan["checkpoint"].is_file() else None,
        })
        write_json(output / "gpu-samples.json", monitor.rows)
        log_text = (output / "runtime.log").read_text(encoding="utf-8", errors="replace")
        result["runtime_error_lines"] = [line for line in log_text.splitlines() if "[Error]" in line]
        result["passed"] = bool(result["passed"] and not result["runtime_error_lines"] and not monitor.errors and monitor.rows)
    except KeyboardInterrupt:
        interrupted = True
        result = {"passed": False, "interrupted": True,
                  "error": "KeyboardInterrupt: evaluation interrupted by user"}
    except Exception as error:
        result = {"passed": False, "error": repr(error)}
    finally:
        cleanup = None
        if child is not None and child.poll() is None:
            cleanup = stop_child(child)
            result["child_cleanup"] = cleanup
            if not cleanup["stopped"]:
                result["passed"] = False
                result.setdefault("assessment_errors", []).append(
                    "Could not confirm that the simulator process tree stopped"
                )
        checkpoint_unchanged = (
            plan["checkpoint"].is_file()
            and plan["checkpoint"].stat().st_size == checkpoint_bytes
            and sha256_file(plan["checkpoint"]) == checkpoint_hash
        )
        result["checkpoint_unchanged"] = checkpoint_unchanged
        result["checkpoint_sha256_after"] = sha256_file(plan["checkpoint"]) if plan["checkpoint"].is_file() else None
        if not checkpoint_unchanged:
            result["passed"] = False
            result.setdefault("assessment_errors", []).append("Checkpoint changed or disappeared during the run")
        result["source_changed_during_run"] = [relative for relative, digest in source_hashes.items()
            if not (ROOT / relative).is_file() or sha256_file(ROOT / relative) != digest]
        result["source_snapshot_matches_at_end"] = bool(
            source_hashes and source_snapshot_matches(ROOT, output, source_hashes)
        )
        result["passed"] = bool(result.get("passed") and not result["source_changed_during_run"]
                                 and result["source_snapshot_matches_at_end"])
        write_json(output / "gpu-samples.json", monitor.rows if monitor is not None else [])
        finish(output, manifest, result)
    print(json.dumps(result, indent=2), flush=True)
    if interrupted:
        return 130
    return 0 if result["passed"] else 1


def stop_child(child, *, terminate_wait_seconds=5, kill_wait_seconds=10):
    """Stop the launcher and its descendants without an unbounded cleanup wait."""
    if child.poll() is not None:
        return {"stopped": True, "method": "already-exited", "errors": []}
    errors = []
    method = "process-group"
    if os.name == "nt":
        method = "taskkill-tree"
        try:
            subprocess.run(["taskkill", "/PID", str(child.pid), "/T", "/F"],
                           capture_output=True, timeout=terminate_wait_seconds)
        except (OSError, subprocess.TimeoutExpired) as error:
            errors.append(f"taskkill failed: {error}")
        if child.poll() is None:
            try:
                child.terminate()
            except OSError as error:
                errors.append(f"terminate failed: {error}")
    else:
        try:
            os.killpg(child.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        except OSError as error:
            errors.append(f"process-group SIGTERM failed: {error}")
            try:
                child.terminate()
            except OSError as fallback_error:
                errors.append(f"terminate fallback failed: {fallback_error}")
    try:
        child.wait(timeout=terminate_wait_seconds)
    except subprocess.TimeoutExpired:
        method += "+kill"
        try:
            if os.name == "nt":
                child.kill()
            else:
                os.killpg(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        except OSError as error:
            errors.append(f"forced kill failed: {error}")
            try:
                child.kill()
            except OSError as fallback_error:
                errors.append(f"process kill fallback failed: {fallback_error}")
        try:
            child.wait(timeout=kill_wait_seconds)
        except subprocess.TimeoutExpired:
            errors.append("launcher process did not exit after forced kill")
    if os.name != "nt":
        # The shell launcher may exit while an Isaac Python child remains in its
        # session. Kill any survivors in the original process group as well.
        try:
            os.killpg(child.pid, getattr(signal, "SIGKILL", 9))
            method += "+group-kill"
        except ProcessLookupError:
            pass
        except OSError as error:
            errors.append(f"process-group final kill failed: {error}")
    return {"stopped": child.poll() is not None and not errors, "method": method, "errors": errors}


if __name__ == "__main__":
    raise SystemExit(main())
