"""Train and evaluate the shared no-communication PPO baseline."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import random
import subprocess
import sys
from time import perf_counter

import numpy as np
import torch

from environments.highway_parallel_env import DEFAULT_CONFIG, HighwayParallelEnv
from policies.baselines import make_policy
from policies.ppo import (
    CheckpointPolicy,
    ObservationNormalizer,
    SharedActorCritic,
    action_masks_from_observations,
)
from traffic.sumo_backend import find_sumo_binary


ROOT = Path(__file__).resolve().parents[1]


def seed_everything(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.use_deterministic_algorithms(True)
    torch.set_num_threads(1)


def compute_advantages(trajectories: list[list[dict]], gamma: float, gae_lambda: float):
    flat: list[dict] = []
    for trajectory in trajectories:
        advantage = 0.0
        next_value = 0.0
        for transition in reversed(trajectory):
            delta = transition["reward"] + gamma * next_value - transition["value"]
            advantage = delta + gamma * gae_lambda * advantage
            transition["advantage"] = advantage
            transition["return"] = advantage + transition["value"]
            next_value = transition["value"]
        flat.extend(trajectory)
    return flat


def collect_rollout(
    model: SharedActorCritic,
    normalizer: ObservationNormalizer,
    config: dict,
    minimum_transitions: int,
    episode_seed_start: int,
):
    trajectories: list[list[dict]] = []
    episode_summaries: list[dict] = []
    raw_observations: list[np.ndarray] = []
    transition_count = 0
    episode_seed = episode_seed_start
    while transition_count < minimum_transitions:
        env = HighwayParallelEnv(horizon=config["horizon"])
        per_agent = {agent: [] for agent in env.possible_agents}
        rewards = {agent: 0.0 for agent in env.possible_agents}
        overrides = {agent: 0 for agent in env.possible_agents}
        completed: list[str] = []
        try:
            observations, _ = env.reset(seed=episode_seed)
            while env.agents:
                agents = sorted(env.agents)
                raw_batch = np.stack([observations[agent] for agent in agents])
                normalized_batch = normalizer.normalize(raw_batch)
                action_masks = action_masks_from_observations(raw_batch)
                with torch.no_grad():
                    actions, log_probs, _, values = model.act(
                        torch.from_numpy(normalized_batch),
                        action_masks=torch.from_numpy(action_masks),
                    )
                action_dict = {
                    agent: action.numpy().astype(np.int64)
                    for agent, action in zip(agents, actions)
                }
                next_observations, step_rewards, terminations, truncations, infos = env.step(
                    action_dict
                )
                for index, agent in enumerate(agents):
                    per_agent[agent].append(
                        {
                            "observation": normalized_batch[index],
                            "raw_observation": raw_batch[index],
                            "action": actions[index].numpy(),
                            "action_mask": action_masks[index],
                            "log_prob": float(log_probs[index]),
                            "value": float(values[index]),
                            "reward": float(step_rewards[agent]),
                        }
                    )
                    rewards[agent] += float(step_rewards[agent])
                    overrides[agent] += int(infos[agent]["safety_intervention"])
                    if infos[agent]["termination_cause"] == "route_complete":
                        completed.append(agent)
                observations = next_observations
            for agent in env.possible_agents:
                if per_agent[agent]:
                    trajectories.append(per_agent[agent])
                    transition_count += len(per_agent[agent])
                    raw_observations.extend(item["raw_observation"] for item in per_agent[agent])
            episode_summaries.append(
                {
                    "seed": episode_seed,
                    "team_reward": sum(rewards.values()),
                    "completed_agents": len(set(completed)),
                    "safety_overrides": sum(overrides.values()),
                }
            )
        finally:
            env.close()
        episode_seed += 1
    transitions = compute_advantages(
        trajectories, config["gamma"], config["gae_lambda"]
    )
    return transitions, np.stack(raw_observations), episode_summaries, episode_seed


def ppo_update(model, optimizer, transitions, config):
    observations = torch.from_numpy(np.stack([item["observation"] for item in transitions]))
    actions = torch.from_numpy(np.stack([item["action"] for item in transitions])).long()
    action_masks = torch.from_numpy(np.stack([item["action_mask"] for item in transitions]))
    old_log_probs = torch.tensor([item["log_prob"] for item in transitions], dtype=torch.float32)
    returns = torch.tensor([item["return"] for item in transitions], dtype=torch.float32)
    advantages = torch.tensor([item["advantage"] for item in transitions], dtype=torch.float32)
    advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
    batch_size = len(transitions)
    losses = []
    for _ in range(config["update_epochs"]):
        indices = torch.randperm(batch_size)
        for start in range(0, batch_size, config["minibatch_size"]):
            batch = indices[start : start + config["minibatch_size"]]
            log_probs, entropies, values = model.evaluate_actions(
                observations[batch], actions[batch], action_masks[batch]
            )
            ratio = (log_probs - old_log_probs[batch]).exp()
            unclipped = ratio * advantages[batch]
            clipped = torch.clamp(
                ratio, 1.0 - config["clip_ratio"], 1.0 + config["clip_ratio"]
            ) * advantages[batch]
            policy_loss = -torch.min(unclipped, clipped).mean()
            value_loss = 0.5 * (returns[batch] - values).pow(2).mean()
            entropy = entropies.mean()
            loss = (
                policy_loss
                + config["value_coefficient"] * value_loss
                - config["entropy_coefficient"] * entropy
            )
            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), config["maximum_gradient_norm"])
            optimizer.step()
            losses.append((float(policy_loss.detach()), float(value_loss.detach()), float(entropy.detach())))
    return {
        "policy_loss": float(np.mean([item[0] for item in losses])),
        "value_loss": float(np.mean([item[1] for item in losses])),
        "entropy": float(np.mean([item[2] for item in losses])),
    }


def evaluate(policy, seeds: list[int], horizon: int):
    episodes = []
    for seed in seeds:
        env = HighwayParallelEnv(horizon=horizon)
        policy.reset(seed)
        rewards = {agent: 0.0 for agent in env.possible_agents}
        overrides = 0
        completed: list[str] = []
        collided: list[str] = []
        try:
            observations, _ = env.reset(seed=seed)
            steps = 0
            while env.agents:
                observations, step_rewards, _, _, infos = env.step(policy.act(observations))
                for agent, reward in step_rewards.items():
                    rewards[agent] += float(reward)
                    overrides += int(infos[agent]["safety_intervention"])
                    if infos[agent]["termination_cause"] == "route_complete":
                        completed.append(agent)
                    elif infos[agent]["termination_cause"] == "collision":
                        collided.append(agent)
                steps += 1
            episodes.append(
                {
                    "seed": seed,
                    "team_reward": sum(rewards.values()),
                    "completed_agents": len(set(completed)),
                    "collision_agents": len(set(collided)),
                    "safety_overrides": overrides,
                    "steps": steps,
                }
            )
        finally:
            env.close()
    return episodes


def save_checkpoint(path, model, optimizer, normalizer, config, transitions):
    payload = {
        "model_state": model.state_dict(),
        "optimizer_state": optimizer.state_dict(),
        "normalizer_state": normalizer.state_dict(),
        "config": config,
        "global_transitions": transitions,
        "torch_rng_state": torch.get_rng_state(),
        "numpy_rng_state": np.random.get_state(),
        "python_rng_state": random.getstate(),
    }
    temporary = path.with_suffix(path.suffix + ".tmp")
    torch.save(payload, temporary)
    temporary.replace(path)


def train(config_path: Path, output_root: Path) -> Path:
    config = json.loads(config_path.read_text(encoding="utf-8"))
    seed_everything(config["training_seed"])
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = output_root / config["study"] / f"seed-{config['training_seed']}-{timestamp}"
    checkpoints = run_dir / "checkpoints"
    checkpoints.mkdir(parents=True, exist_ok=False)
    (run_dir / "resolved-config.json").write_text(json.dumps(config, indent=2), encoding="utf-8")

    observation_size = config["model"]["observation_size"]
    model = SharedActorCritic(
        observation_size=observation_size,
        hidden_sizes=config["model"]["hidden_sizes"],
    )
    optimizer = torch.optim.Adam(model.parameters(), lr=config["learning_rate"])
    normalizer = ObservationNormalizer(size=observation_size)
    global_transitions = 0
    episode_seed = config["training_seed"]
    best_selection_score = -float("inf")
    best_validation_reward = -float("inf")
    best_validation_completion = 0.0
    training_rows = []
    started = perf_counter()
    while global_transitions < config["total_transitions"]:
        transitions, raw_observations, episodes, episode_seed = collect_rollout(
            model,
            normalizer,
            config,
            config["minimum_rollout_transitions"],
            episode_seed,
        )
        losses = ppo_update(model, optimizer, transitions, config)
        normalizer.update(raw_observations)
        global_transitions += len(transitions)
        save_checkpoint(
            checkpoints / "latest.pt",
            model,
            optimizer,
            normalizer,
            config,
            global_transitions,
        )
        latest_policy = CheckpointPolicy(checkpoints / "latest.pt")
        validation = evaluate(latest_policy, config["validation_seeds"], config["horizon"])
        validation_mean = float(np.mean([episode["team_reward"] for episode in validation]))
        validation_completion = float(
            np.mean([episode["completed_agents"] / 2 for episode in validation])
        )
        selection_score = 1000.0 * validation_completion + validation_mean
        if selection_score > best_selection_score:
            best_selection_score = selection_score
            best_validation_reward = validation_mean
            best_validation_completion = validation_completion
            save_checkpoint(
                checkpoints / "best.pt",
                model,
                optimizer,
                normalizer,
                config,
                global_transitions,
            )
        training_rows.append(
            {
                "global_transitions": global_transitions,
                "training_mean_team_reward": np.mean([item["team_reward"] for item in episodes]),
                "training_completion_rate": np.mean([item["completed_agents"] / 2 for item in episodes]),
                "training_mean_safety_overrides": np.mean([item["safety_overrides"] for item in episodes]),
                "validation_mean_team_reward": validation_mean,
                "validation_completion_rate": validation_completion,
                **losses,
            }
        )
        print(json.dumps(training_rows[-1]))

    with (run_dir / "training-metrics.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(training_rows[0]))
        writer.writeheader()
        writer.writerows(training_rows)

    best_policy = CheckpointPolicy(checkpoints / "best.pt")
    comparisons = {
        "shared_ppo": evaluate(best_policy, config["test_seeds"], config["horizon"]),
        "scripted": evaluate(make_policy("scripted"), config["test_seeds"], config["horizon"]),
        "random": evaluate(make_policy("random"), config["test_seeds"], config["horizon"]),
    }
    (run_dir / "evaluation.json").write_text(json.dumps(comparisons, indent=2), encoding="utf-8")
    git_status = subprocess.run(
        ["git", "status", "--porcelain"], cwd=ROOT, capture_output=True, text=True, check=True
    ).stdout
    manifest = {
        "run_id": run_dir.name,
        "status": "completed",
        "exact_command": " ".join([sys.executable, "-m", "experiments.train_ppo", *sys.argv[1:]]),
        "git_commit": subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True, check=True
        ).stdout.strip(),
        "git_dirty": bool(git_status),
        "python": platform.python_version(),
        "torch": torch.__version__,
        "sumo": subprocess.run(
            [str(find_sumo_binary()), "--version"], capture_output=True, text=True, check=True
        ).stdout.splitlines()[0],
        "wall_time_seconds": perf_counter() - started,
        "config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
        "training_metrics_sha256": hashlib.sha256(
            (run_dir / "training-metrics.csv").read_bytes()
        ).hexdigest(),
        "evaluation_sha256": hashlib.sha256((run_dir / "evaluation.json").read_bytes()).hexdigest(),
        "best_validation_mean_team_reward": best_validation_reward,
        "best_validation_completion_rate": best_validation_completion,
        "best_selection_score": best_selection_score,
        "best_checkpoint": "checkpoints/best.pt",
        "latest_checkpoint": "checkpoints/latest.pt",
    }
    (run_dir / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return run_dir


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config", type=Path, default=ROOT / "experiments" / "configs" / "ppo_baseline.json"
    )
    parser.add_argument("--output-root", type=Path, default=ROOT / "results" / "training")
    args = parser.parse_args()
    print(train(args.config.resolve(), args.output_root.resolve()))


if __name__ == "__main__":
    main()
