"""Run the bounded, matched-seed SUMO demo readiness comparison.

The study evaluates the existing scripted baseline and a trusted local PPO
checkpoint through the same environment and horizon. It records failed episodes
in place and never reuses an output directory.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import numbers
import os
from pathlib import Path
import platform
import shutil
import signal
import subprocess
import sys
import traceback
from time import perf_counter
import uuid
import xml.etree.ElementTree as ET

from environments.highway_parallel_env import DEFAULT_CONFIG
from hivemind import launcher


ROOT = Path(__file__).resolve().parents[1]
MATCHED_SEEDS = (2001, 2002, 2003)
HORIZON = 400
REPEAT_SEED = 2001
POLICIES = ("scripted", "trained")
EPISODE_TIMEOUT_SECONDS = 120


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _git(root: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", *args], cwd=root, capture_output=True, text=True, check=True
    )
    return result.stdout


def _finite_values(rows: list[dict], summary: dict) -> list[str]:
    invalid: list[str] = []
    for index, row in enumerate(rows):
        for key, value in row.items():
            if isinstance(value, numbers.Real) and not isinstance(value, bool):
                if not math.isfinite(float(value)):
                    invalid.append(f"rows[{index}].{key}")
    rewards = summary.get("total_rewards", {})
    for agent, value in rewards.items():
        if not isinstance(value, numbers.Real) or not math.isfinite(float(value)):
            invalid.append(f"total_rewards.{agent}")
    wall_time = summary.get("wall_time_seconds")
    if not isinstance(wall_time, numbers.Real) or not math.isfinite(float(wall_time)):
        invalid.append("wall_time_seconds")
    return invalid


def _jsonable(value):
    if hasattr(value, "item"):
        return _jsonable(value.item())
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(item) for item in value]
    return value


def _metric_fingerprint(rows: list[dict]) -> str:
    """Hash stable trajectory fields while excluding incidental wall time."""
    payload = json.dumps(_jsonable(rows), sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _episode_record(policy: str, seed: int, repeat: bool, rows: list[dict], summary: dict,
                    error: str | None, elapsed: float) -> dict:
    expected_agents = {"agent_0", "agent_1"}
    completed = set(summary.get("completed_agents", []))
    collisions = set(summary.get("collision_agents", []))
    invalid = _finite_values(rows, summary)
    status = "completed" if error is None else "failed"
    rewards = summary.get("total_rewards", {})
    rewards = {agent: (float(value) if isinstance(value, numbers.Real) and math.isfinite(float(value))
                       else repr(value)) for agent, value in rewards.items()}
    return {
        "policy": policy,
        "seed": seed,
        "repeat": repeat,
        "status": status,
        "episode_steps": summary.get("episode_steps"),
        "completed_agents": sorted(completed),
        "completed_both": completed == expected_agents,
        "collision_agents": sorted(collisions),
        "collision_count": len(collisions),
        "truncated_agents": sorted(set(summary.get("truncated_agents", []))),
        "total_rewards": rewards,
        "safety_overrides": summary.get("safety_overrides", {}),
        "proposed_lane_changes": summary.get("proposed_lane_changes", {}),
        "applied_lane_changes": summary.get("applied_lane_changes", {}),
        "finite_metrics": not invalid,
        "nonfinite_fields": invalid,
        "wall_time_seconds": elapsed,
        "metric_fingerprint": _metric_fingerprint(rows) if rows and not invalid else None,
        "error": error,
    }


def _write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        path.write_text("\n", encoding="utf-8")
        return
    fieldnames = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def _write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
                    encoding="utf-8")


def _step_length(config_path: Path) -> float:
    root = ET.parse(config_path).getroot()
    element = root.find("./time/step-length")
    value = (element.get("value") or element.text) if element is not None else None
    try:
        step_length = float(value) if value is not None else float("nan")
    except (TypeError, ValueError):
        step_length = float("nan")
    if not math.isfinite(step_length) or step_length <= 0:
        raise ValueError(f"Missing or invalid SUMO step-length in {config_path}")
    return step_length


def _status(records: list[dict], seeds: tuple[int, ...] = MATCHED_SEEDS,
            repeat_seed: int = REPEAT_SEED, inputs_unchanged: bool = True) -> tuple[str, dict]:
    per_policy = {}
    for policy in POLICIES:
        group = [record for record in records if record["policy"] == policy]
        expected = [(seed, False) for seed in seeds] + [(repeat_seed, True)]
        observed = [(record["seed"], record["repeat"]) for record in group]
        gates = {
            "expected_episodes_recorded": len(group) == len(expected) and sorted(observed) == sorted(expected),
            "all_episodes_completed_without_error": len(group) == len(expected) and all(
                record["status"] == "completed" and record["error"] is None for record in group),
            "both_cars_complete_every_episode": bool(group) and all(r["completed_both"] for r in group),
            "zero_collisions_every_episode": bool(group) and all(r["collision_count"] == 0 for r in group),
            "finite_metrics_every_episode": bool(group) and all(r["finite_metrics"] for r in group),
            "episodes_recorded": len(group),
        }
        gates["passed"] = all(gates[key] for key in (
            "expected_episodes_recorded", "all_episodes_completed_without_error",
            "both_cars_complete_every_episode", "zero_collisions_every_episode",
            "finite_metrics_every_episode"))
        per_policy[policy] = gates
    repeat_consistent = {}
    for policy in POLICIES:
        group = [r for r in records if r["policy"] == policy and r["seed"] == repeat_seed]
        repeat_consistent[policy] = (len(group) == 2 and group[0]["metric_fingerprint"] is not None
                                     and group[0]["metric_fingerprint"] == group[1]["metric_fingerprint"])
    gates = {
        "per_policy": per_policy,
        "same_seed_repeat_matches": repeat_consistent,
        "same_seed_repeat_is_required": True,
        "input_files_unchanged_during_run": inputs_unchanged,
    }
    all_pass = all(result["passed"] for result in per_policy.values()) and all(repeat_consistent.values()) and inputs_unchanged
    return ("passed" if all_pass else "failed"), gates


def _coerce_csv_value(value: str):
    if value == "":
        return value
    if value.lower() == "true":
        return True
    if value.lower() == "false":
        return False
    try:
        return int(value)
    except ValueError:
        try:
            return float(value)
        except ValueError:
            return value


def _kill_process_tree(process: subprocess.Popen) -> None:
    if os.name == "nt":
        try:
            subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"],
                           capture_output=True, text=True, timeout=15)
        except (OSError, subprocess.TimeoutExpired):
            if process.poll() is None:
                process.kill()
    else:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)


def _run_episode(policy: str, seed: int, horizon: int, checkpoint: Path,
                 episode_dir: Path, log_path: Path, runtime: Path,
                 child_env: dict[str, str], root: Path = ROOT
                 ) -> tuple[list[dict], dict, str | None, float]:
    episode_dir.mkdir(parents=True, exist_ok=False)
    output_root = episode_dir / "rollouts"
    command = [str(runtime), "-m", "experiments.rollout", "--policy", policy,
               "--seed", str(seed), "--horizon", str(horizon),
               "--output-root", str(output_root)]
    if policy == "trained":
        command += ["--checkpoint", str(checkpoint)]
    stdout_path, stderr_path = episode_dir / "stdout.txt", episode_dir / "stderr.txt"
    started = perf_counter()
    error = None
    process = None
    creationflags = subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0
    with stdout_path.open("w", encoding="utf-8") as stdout, stderr_path.open("w", encoding="utf-8") as stderr:
        try:
            process = subprocess.Popen(
                command, cwd=root, env=child_env, stdout=stdout, stderr=stderr,
                creationflags=creationflags, start_new_session=(os.name != "nt"),
            )
            try:
                return_code = process.wait(timeout=EPISODE_TIMEOUT_SECONDS)
            except subprocess.TimeoutExpired:
                error = f"episode exceeded {EPISODE_TIMEOUT_SECONDS}-second wall-time limit; process tree terminated"
                _kill_process_tree(process)
                process.wait()
                return_code = process.returncode
            if error is None and return_code != 0:
                error = f"rollout child exited with status {return_code}"
        except Exception as exc:
            error = "".join(traceback.format_exception(type(exc), exc, exc.__traceback__))
            if process is not None and process.poll() is None:
                _kill_process_tree(process)
                process.wait()
    elapsed = perf_counter() - started
    stdout_text = stdout_path.read_text(encoding="utf-8", errors="replace")
    stderr_text = stderr_path.read_text(encoding="utf-8", errors="replace")
    log_path.write_text(
        "COMMAND\n" + json.dumps(command) + "\nSTDOUT\n" + stdout_text + "\nSTDERR\n" + stderr_text
        + ("\nEXCEPTION\n" + error if error else ""),
        encoding="utf-8",
    )
    rows: list[dict] = []
    summary: dict = {}
    if error is None:
        try:
            manifests = list(output_root.glob(f"{policy}/seed-*/manifest.json"))
            metric_files = list(output_root.glob(f"{policy}/seed-*/metrics.csv"))
            if len(manifests) != 1 or len(metric_files) != 1:
                raise FileNotFoundError(f"Expected one child evidence package; got {len(manifests)} manifests and {len(metric_files)} metrics files")
            child_manifest = json.loads(manifests[0].read_text(encoding="utf-8"))
            summary = child_manifest["summary"]
            if child_manifest.get("status") != "completed":
                error = f"child evidence status was {child_manifest.get('status')!r}"
            with metric_files[0].open(newline="", encoding="utf-8") as handle:
                rows = [{key: _coerce_csv_value(value) for key, value in row.items()}
                        for row in csv.DictReader(handle)]
        except Exception as exc:
            error = f"could not read child evidence: {type(exc).__name__}: {exc}"
    return rows, summary, error, elapsed


def create_run(output_root: Path, *, checkpoint: Path, horizon: int = HORIZON,
               seeds: tuple[int, ...] = MATCHED_SEEDS, repeat_seed: int = REPEAT_SEED,
               root: Path = ROOT, runtime: Path | None = None,
               child_env: dict[str, str] | None = None) -> Path:
    checkpoint = checkpoint.resolve()
    if not checkpoint.is_file():
        raise FileNotFoundError(f"Checkpoint does not exist: {checkpoint}")
    if horizon <= 0:
        raise ValueError("horizon must be positive")
    if repeat_seed not in seeds:
        raise ValueError("repeat_seed must be included in the matched seed set")
    config = Path(DEFAULT_CONFIG).resolve()
    step_length = _step_length(config)
    if runtime is None or child_env is None:
        runtime, child_env, _, checkpoint = resolve_launcher_context(root, checkpoint)
    sumo_binary = Path(child_env["SUMO_BINARY"]).resolve()

    run_id = f"demo-sumo-readiness-{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}-{uuid.uuid4().hex[:8]}"
    run_dir = output_root.resolve() / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    logs_dir = run_dir / "logs"
    logs_dir.mkdir()

    sources = [config, config.parent / "network.net.xml", config.parent / "routes.rou.xml",
              Path(__file__).resolve(), Path(__import__("experiments.rollout", fromlist=["__file__"]).__file__).resolve(),
              Path(__import__("environments.highway_parallel_env", fromlist=["__file__"]).__file__).resolve(),
              (root / "hivemind" / "launcher.py"),
              (root / "policies" / "ppo.py"), (root / "policies" / "baselines.py"),
              (root / "traffic" / "sumo_backend.py")]
    source_hashes = {str(path.relative_to(root)): sha256(path) for path in sources}
    checkpoint_hash = sha256(checkpoint)
    source_dir = run_dir / "source"
    for source in sources:
        target = source_dir / source.relative_to(root)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    status_text = _git(root, "status", "--porcelain")
    sumo_version = subprocess.run([str(sumo_binary), "--version"], capture_output=True,
                                  text=True, check=True).stdout.splitlines()[0]
    runtime_probe = subprocess.run(
        [str(runtime), "-c", "import json,sys,traci; print(json.dumps({'python':sys.version,'traci':traci.__file__}))"],
        cwd=root, env=child_env, capture_output=True, text=True, check=True,
    )
    runtime_info = json.loads(runtime_probe.stdout.splitlines()[-1])
    resolved = {
        "question": "Are the existing scripted and frozen trained SUMO demos ready for a bounded headless run?",
        "variants": list(POLICIES),
        "matched_seeds": list(seeds),
        "repeat_seed": repeat_seed,
        "horizon_steps": horizon,
        "simulation_step_length_seconds": step_length,
        "evaluation_mode": "deterministic; headless; no training",
        "scenario": str(config.relative_to(root)),
        "checkpoint": str(checkpoint.relative_to(root)) if checkpoint.is_relative_to(root) else str(checkpoint),
        "checkpoint_observation_size": 16,
        "predeclared_gates": ["exact episode and seed set recorded", "every child exits successfully",
                              "both cars complete every episode", "zero collisions every episode",
                              "all recorded numeric metrics are finite", "seed-2001 repeat matches",
                              "source and checkpoint hashes remain unchanged"],
        "shield_overrides_are_reported_not_a_zero_override_gate": True,
        "comparison_claim": "descriptive paired-seed readiness only; no superiority claim",
    }
    _write_json(run_dir / "resolved-config.json", resolved)
    manifest = {
        "run_id": run_id,
        "status": "running",
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "exact_command": [sys.executable, "-m", "experiments.demo_sumo_readiness", "--headless",
                          "--horizon", str(horizon), "--checkpoint", str(checkpoint),
                          "--output-root", str(output_root.resolve())],
        "episode_timeout_seconds": EPISODE_TIMEOUT_SECONDS,
        "git_commit": _git(root, "rev-parse", "HEAD").strip(),
        "git_dirty": bool(status_text),
        "git_status_porcelain_sha256": hashlib.sha256(status_text.encode()).hexdigest(),
        "git_diff_sha256": hashlib.sha256(_git(root, "diff", "--binary").encode()).hexdigest(),
        "python": sys.version,
        "platform": platform.platform(),
        "sumo": sumo_version,
        "sumo_binary": str(sumo_binary),
        "traci_module": str(Path(runtime_info["traci"]).resolve()),
        "traffic_python": str(runtime),
        "local_configuration_contents_captured": False,
        "local_configuration_note": (
            "hivemind.local.json is not copied or hashed; resolved Python, SUMO, "
            "and checkpoint paths are recorded separately."
        ),
        "source_snapshot": "source/",
        "source_sha256_start": source_hashes,
        "checkpoint_sha256_start": checkpoint_hash,
        "resolved_config": resolved,
        "episodes": [],
    }
    _write_json(run_dir / "manifest.json", manifest)

    episode_specs = [(policy, seed, False) for policy in POLICIES for seed in seeds]
    episode_specs += [(policy, repeat_seed, True) for policy in POLICIES]
    all_rows: list[dict] = []
    records: list[dict] = []
    for index, (policy, seed, repeat) in enumerate(episode_specs, 1):
        tag = f"{policy}-seed-{seed}" + ("-repeat" if repeat else "")
        rows, summary, error, elapsed = _run_episode(
            policy, seed, horizon, checkpoint, run_dir / "episode_jobs" / tag,
            logs_dir / f"{tag}.log", runtime, child_env, root,
        )
        record = _episode_record(policy, seed, repeat, rows, summary, error, elapsed)
        record["episode_id"] = f"{index:02d}-{tag}"
        records.append(record)
        for row in rows:
            all_rows.append({"episode_id": record["episode_id"], "policy": policy, "seed": seed,
                             "repeat": repeat, **row})
        manifest["episodes"] = records
        manifest["status"] = "running"
        _write_csv(run_dir / "episode-metrics.csv", records)
        _write_csv(run_dir / "metrics.csv", all_rows)
        _write_json(run_dir / "manifest.json", manifest)

    source_hashes_end = {str(path.relative_to(root)): sha256(path) for path in sources}
    checkpoint_hash_end = sha256(checkpoint)
    inputs_unchanged = source_hashes_end == source_hashes and checkpoint_hash_end == checkpoint_hash
    status, gates = _status(records, seeds, repeat_seed, inputs_unchanged)
    manifest["status"] = status
    manifest["finished_utc"] = datetime.now(timezone.utc).isoformat()
    manifest["gates"] = gates
    manifest["source_sha256_end"] = source_hashes_end
    manifest["checkpoint_sha256_end"] = checkpoint_hash_end
    manifest["inputs_unchanged"] = inputs_unchanged
    manifest["metrics_sha256"] = sha256(run_dir / "metrics.csv")
    manifest["episode_metrics_sha256"] = sha256(run_dir / "episode-metrics.csv")
    _write_json(run_dir / "manifest.json", manifest)
    comparison = _comparison(records, seeds)
    _write_csv(run_dir / "comparison.csv", comparison)
    manifest["comparison_sha256"] = sha256(run_dir / "comparison.csv")
    manifest["log_sha256"] = {path.name: sha256(path) for path in sorted(logs_dir.glob("*.log"))}
    _write_json(run_dir / "manifest.json", manifest)
    return run_dir


def _comparison(records: list[dict], seeds: tuple[int, ...] = MATCHED_SEEDS) -> list[dict]:
    output = []
    for seed in seeds:
        scripted = next((r for r in records if r["policy"] == "scripted" and r["seed"] == seed and not r["repeat"]), None)
        trained = next((r for r in records if r["policy"] == "trained" and r["seed"] == seed and not r["repeat"]), None)
        if scripted is None or trained is None:
            continue
        output.append({
            "seed": seed,
            "scripted_completed_both": scripted["completed_both"],
            "trained_completed_both": trained["completed_both"],
            "scripted_collisions": scripted["collision_count"],
            "trained_collisions": trained["collision_count"],
            "scripted_wall_time_seconds": scripted["wall_time_seconds"],
            "trained_wall_time_seconds": trained["wall_time_seconds"],
            "scripted_shield_overrides": sum(scripted["safety_overrides"].values()),
            "trained_shield_overrides": sum(trained["safety_overrides"].values()),
            "scripted_episode_steps": scripted["episode_steps"],
            "trained_episode_steps": trained["episode_steps"],
        })
    return output


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--checkpoint", type=Path,
                        help="trusted local 16-input checkpoint; defaults to hivemind.local.json")
    result.add_argument("--output-root", type=Path, default=ROOT / "outputs" / "demo_sumo_readiness")
    result.add_argument("--horizon", type=int, default=HORIZON)
    result.add_argument("--headless", action="store_true", help="accepted for portable demo command compatibility")
    return result


def resolve_launcher_context(root: Path, checkpoint_override: Path | None):
    """Resolve interpreter, SUMO binary, and checkpoint through the portable launcher."""
    config = launcher.load_config(root)
    argv = ["demo", "trained", "--headless"]
    if checkpoint_override is not None:
        value = str(checkpoint_override if checkpoint_override.is_absolute()
                    else (root / checkpoint_override).resolve())
        argv.extend(("--checkpoint", value))
    launch_args = launcher.parser().parse_args(argv)
    command, env, modules = launcher.plan(launch_args, root, config)
    try:
        checkpoint_index = command.index("--checkpoint") + 1
        checkpoint = Path(command[checkpoint_index]).resolve()
    except (ValueError, IndexError) as exc:
        raise ValueError("The portable launcher did not resolve a trained checkpoint") from exc
    return Path(command[0]).resolve(), env, modules, checkpoint


def configured_checkpoint(root: Path, value: Path | None) -> Path:
    return resolve_launcher_context(root, value)[3]


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        runtime, child_env, modules, checkpoint = resolve_launcher_context(ROOT, args.checkpoint)
        launcher.check_modules(runtime, modules, ROOT)
        run_dir = create_run(args.output_root, checkpoint=checkpoint, horizon=args.horizon,
                             runtime=runtime, child_env=child_env)
        print(run_dir)
        manifest = json.loads((run_dir / "manifest.json").read_text(encoding="utf-8"))
        return 0 if manifest["status"] == "passed" else 1
    except Exception as exc:
        print(f"Readiness run setup failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
