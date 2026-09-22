"""Run and record the deterministic single-vehicle scripted baseline."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys

import traci

from traffic.sumo_backend import SumoBackend, VehicleCommand


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "scenarios" / "single_vehicle" / "scenario.sumocfg"


def scripted_command(simulation_time: float, lane_change_sent: bool) -> tuple[VehicleCommand, bool]:
    if simulation_time < 3.0:
        return VehicleCommand(target_speed=14.0), lane_change_sent
    if simulation_time < 6.0 and not lane_change_sent:
        return VehicleCommand(target_speed=14.0, lane_delta=1), True
    if simulation_time < 9.0:
        return VehicleCommand(target_speed=10.0), lane_change_sent
    return VehicleCommand(target_speed=18.0), lane_change_sent


def run_episode(config: Path, seed: int, max_steps: int) -> tuple[list[dict], dict]:
    rows: list[dict] = []
    lane_change_sent = False
    completed = False
    collisions: set[str] = set()
    with SumoBackend(config) as backend:
        result = backend.reset(seed)
        for episode_step in range(max_steps):
            if "agent_0" not in backend.active_vehicle_ids:
                completed = "agent_0" in result.arrived
                break
            command, lane_change_sent = scripted_command(result.simulation_time, lane_change_sent)
            result = backend.step({"agent_0": command})
            collisions.update(result.collisions)
            state = result.states.get("agent_0", {})
            executed = result.executed_actions.get("agent_0", {})
            rows.append(
                {
                    "episode_step": episode_step,
                    "simulation_time": result.simulation_time,
                    "active": "agent_0" in backend.active_vehicle_ids,
                    "arrived": "agent_0" in result.arrived,
                    "requested_speed": command.target_speed,
                    "executed_speed_command": executed.get("target_speed"),
                    "requested_lane_delta": command.lane_delta,
                    "executed_lane_delta": executed.get("lane_delta"),
                    "x": state.get("x"),
                    "y": state.get("y"),
                    "speed": state.get("speed"),
                    "acceleration": state.get("acceleration"),
                    "lane_index": state.get("lane_index"),
                    "road_id": state.get("road_id"),
                    "distance": state.get("distance"),
                    "waiting_time": state.get("waiting_time"),
                    "time_loss": state.get("time_loss"),
                    "collisions": len(result.collisions),
                }
            )
            if "agent_0" in result.arrived:
                completed = True
                break
    summary = {
        "seed": seed,
        "steps": len(rows),
        "completed": completed,
        "collision_vehicle_ids": sorted(collisions),
        "final_simulation_time": rows[-1]["simulation_time"] if rows else None,
        "final_distance": next((row["distance"] for row in reversed(rows) if row["distance"] is not None), None),
    }
    return rows, summary


def write_run(output_root: Path, config: Path, seed: int, max_steps: int) -> Path:
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = output_root / f"seed-{seed}-{timestamp}"
    run_dir.mkdir(parents=True, exist_ok=False)
    rows, summary = run_episode(config, seed, max_steps)
    metrics_path = run_dir / "metrics.csv"
    with metrics_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]) if rows else ["episode_step"])
        writer.writeheader()
        writer.writerows(rows)

    config_hash = hashlib.sha256(config.read_bytes()).hexdigest()
    network = config.parent / "network.net.xml"
    routes = config.parent / "routes.rou.xml"
    git_commit = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True, capture_output=True, check=True
    ).stdout.strip()
    git_dirty = bool(
        subprocess.run(["git", "status", "--porcelain"], cwd=ROOT, text=True, capture_output=True, check=True).stdout
    )
    manifest = {
        "run_id": run_dir.name,
        "status": "completed" if summary["completed"] else "incomplete",
        "command": " ".join(sys.argv),
        "git_commit": git_commit,
        "git_dirty": git_dirty,
        "python": platform.python_version(),
        "platform": platform.platform(),
        "traci": str(Path(traci.__file__).resolve()),
        "sumo_binary": str(SumoBackend(config).sumo_binary),
        "seed": seed,
        "max_steps": max_steps,
        "inputs": {
            str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (config, network, routes)
        },
        "config_sha256": config_hash,
        "summary": summary,
        "metrics_sha256": hashlib.sha256(metrics_path.read_bytes()).hexdigest(),
    }
    (run_dir / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    (run_dir / "resolved-config.json").write_text(
        json.dumps({"scenario": str(config.relative_to(ROOT)), "seed": seed, "max_steps": max_steps}, indent=2),
        encoding="utf-8",
    )
    return run_dir


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--max-steps", type=int, default=600)
    parser.add_argument("--output-root", type=Path, default=ROOT / "results" / "single_vehicle")
    args = parser.parse_args()
    run_dir = write_run(args.output_root.resolve(), args.config.resolve(), args.seed, args.max_steps)
    print(run_dir)


if __name__ == "__main__":
    main()

