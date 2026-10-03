"""Run visible or headless baseline policies and save an evidence package."""

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
from time import perf_counter

from environments.highway_parallel_env import DEFAULT_CONFIG, HighwayParallelEnv
from policies.baselines import make_policy
from policies.ppo import CheckpointPolicy
from traffic.sumo_backend import find_sumo_binary


ROOT = Path(__file__).resolve().parents[1]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run_rollout(
    *,
    policy_name: str,
    seed: int,
    horizon: int,
    gui: bool = False,
    gui_delay_ms: int = 50,
    checkpoint: Path | None = None,
) -> tuple[list[dict], dict]:
    env = HighwayParallelEnv(horizon=horizon, gui=gui, gui_delay_ms=gui_delay_ms)
    policy = CheckpointPolicy(checkpoint) if policy_name == "trained" else make_policy(policy_name)
    policy.reset(seed)
    rows: list[dict] = []
    summary = {
        "policy": policy_name,
        "seed": seed,
        "completed_agents": [],
        "collision_agents": [],
        "truncated_agents": [],
        "total_rewards": {agent: 0.0 for agent in env.possible_agents},
        "safety_overrides": {agent: 0 for agent in env.possible_agents},
        "proposed_lane_changes": {agent: 0 for agent in env.possible_agents},
        "applied_lane_changes": {agent: 0 for agent in env.possible_agents},
    }
    started = perf_counter()
    try:
        observations, _ = env.reset(seed=seed)
        episode_step = 0
        while env.agents:
            actions = policy.act(observations)
            observations, rewards, terminations, truncations, infos = env.step(actions)
            for agent, info in infos.items():
                proposed = tuple(info["proposed_action"])
                applied = tuple(info["applied_action"])
                components = info["reward_components"]
                summary["total_rewards"][agent] += rewards[agent]
                summary["safety_overrides"][agent] += int(info["safety_intervention"])
                summary["proposed_lane_changes"][agent] += int(proposed[1] != 1)
                summary["applied_lane_changes"][agent] += int(applied[1] != 1)
                if info["termination_cause"] == "route_complete":
                    summary["completed_agents"].append(agent)
                elif info["termination_cause"] == "collision":
                    summary["collision_agents"].append(agent)
                if info["truncation_cause"]:
                    summary["truncated_agents"].append(agent)
                observation = observations.get(agent)
                rows.append(
                    {
                        "episode_step": episode_step,
                        "simulation_time": info["simulation_time"],
                        "agent": agent,
                        "reward": rewards[agent],
                        "reward_progress": components["progress"],
                        "reward_speed": components["speed"],
                        "reward_unsafe_gap": components["unsafe_gap"],
                        "reward_collision": components["collision"],
                        "reward_completion": components["completion"],
                        "reward_safety_intervention": components["safety_intervention"],
                        "reward_lane_change_request": components["lane_change_request"],
                        "proposed_speed_action": proposed[0],
                        "proposed_lane_action": proposed[1],
                        "applied_speed_action": applied[0],
                        "applied_lane_action": applied[1],
                        "safety_intervention": info["safety_intervention"],
                        "safety_reasons": "|".join(info["safety_reasons"]),
                        "terminated": terminations[agent],
                        "truncated": truncations[agent],
                        "next_speed_normalized": observation[0] if observation is not None else "",
                        "next_route_progress": observation[2] if observation is not None else "",
                        "next_front_gap_normalized": observation[3] if observation is not None else "",
                    }
                )
            episode_step += 1
    finally:
        env.close()
    summary["episode_steps"] = episode_step
    summary["wall_time_seconds"] = perf_counter() - started
    summary["completed_agents"] = sorted(set(summary["completed_agents"]))
    summary["collision_agents"] = sorted(set(summary["collision_agents"]))
    summary["truncated_agents"] = sorted(set(summary["truncated_agents"]))
    return rows, summary


def write_evidence_package(
    *,
    policy_name: str,
    seed: int,
    horizon: int,
    gui: bool,
    gui_delay_ms: int,
    checkpoint: Path | None,
    output_root: Path,
) -> Path:
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = output_root / policy_name / f"seed-{seed}-{timestamp}"
    run_dir.mkdir(parents=True, exist_ok=False)
    rows, summary = run_rollout(
        policy_name=policy_name,
        seed=seed,
        horizon=horizon,
        gui=gui,
        gui_delay_ms=gui_delay_ms,
        checkpoint=checkpoint,
    )
    metrics_path = run_dir / "metrics.csv"
    with metrics_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    config = Path(DEFAULT_CONFIG).resolve()
    inputs = [config, config.parent / "network.net.xml", config.parent / "routes.rou.xml"]
    git_commit = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True, check=True
    ).stdout.strip()
    git_status = subprocess.run(
        ["git", "status", "--porcelain"], cwd=ROOT, capture_output=True, text=True, check=True
    ).stdout
    sumo_version = subprocess.run(
        [str(find_sumo_binary()), "--version"], capture_output=True, text=True, check=True
    ).stdout.splitlines()[0]
    resolved = {
        "policy": policy_name,
        "seed": seed,
        "horizon": horizon,
        "gui": gui,
        "gui_delay_ms": gui_delay_ms,
        "scenario": str(config.relative_to(ROOT)),
        "communication": "none",
        "checkpoint": str(checkpoint) if checkpoint else None,
    }
    (run_dir / "resolved-config.json").write_text(
        json.dumps(resolved, indent=2), encoding="utf-8"
    )
    manifest = {
        "run_id": run_dir.name,
        "status": "completed",
        "exact_command": " ".join([sys.executable, "-m", "experiments.rollout", *sys.argv[1:]]),
        "git_commit": git_commit,
        "git_dirty": bool(git_status),
        "dirty_diff_sha256": (
            hashlib.sha256(
                subprocess.run(
                    ["git", "diff", "--binary"], cwd=ROOT, capture_output=True, check=True
                ).stdout
            ).hexdigest()
            if git_status
            else None
        ),
        "python": platform.python_version(),
        "platform": platform.platform(),
        "sumo": sumo_version,
        "inputs": {str(path.relative_to(ROOT)): sha256(path) for path in inputs},
        "resolved_config": resolved,
        "summary": summary,
        "metrics_sha256": sha256(metrics_path),
    }
    (run_dir / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return run_dir


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy", choices=("scripted", "random", "trained"), default="scripted")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--horizon", type=int, default=400)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--gui-delay-ms", type=int, default=50)
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--output-root", type=Path, default=ROOT / "results" / "rollouts")
    args = parser.parse_args()
    if args.policy == "trained" and args.checkpoint is None:
        parser.error("--checkpoint is required when --policy trained")
    run_dir = write_evidence_package(
        policy_name=args.policy,
        seed=args.seed,
        horizon=args.horizon,
        gui=args.gui,
        gui_delay_ms=args.gui_delay_ms,
        checkpoint=args.checkpoint.resolve() if args.checkpoint else None,
        output_root=args.output_root.resolve(),
    )
    print(run_dir)


if __name__ == "__main__":
    main()
