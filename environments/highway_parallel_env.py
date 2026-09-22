"""Deterministic PettingZoo ParallelEnv backed by SUMO/TraCI."""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path
from typing import Callable

import gymnasium as gym
import numpy as np
from pettingzoo import ParallelEnv

from traffic.sumo_backend import StepResult, SumoBackend, VehicleCommand


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "scenarios" / "curved_two_agent" / "scenario.sumocfg"


class HighwayParallelEnv(ParallelEnv):
    """Two simultaneous tactical agents on a curved SUMO highway."""

    metadata = {"name": "hivemind_highway_parallel_v0", "render_modes": []}

    def __init__(
        self,
        *,
        config_path: str | Path = DEFAULT_CONFIG,
        horizon: int = 400,
        route_length: float = 1074.06,
        backend_factory: Callable[[], SumoBackend] | None = None,
    ) -> None:
        self.possible_agents = ["agent_0", "agent_1"]
        self.agents: list[str] = []
        self.horizon = int(horizon)
        self.route_length = float(route_length)
        self._config_path = Path(config_path).resolve()
        self._backend_factory = backend_factory or (lambda: SumoBackend(self._config_path))
        self._backend: SumoBackend | None = None
        self._result: StepResult | None = None
        self._step_count = 0
        self._previous_distance: dict[str, float] = {}
        self._last_observation = {
            agent: np.zeros(9, dtype=np.float32) for agent in self.possible_agents
        }

    @lru_cache(maxsize=None)
    def observation_space(self, agent: str) -> gym.spaces.Box:
        self._validate_agent(agent)
        return gym.spaces.Box(
            low=np.array([0, 0, 0, 0, -1, 0, 0, -1, 0], dtype=np.float32),
            high=np.array([1, 1, 1, 1, 1, 1, 1, 1, 1], dtype=np.float32),
            dtype=np.float32,
        )

    @lru_cache(maxsize=None)
    def action_space(self, agent: str) -> gym.spaces.MultiDiscrete:
        self._validate_agent(agent)
        # [speed: decelerate/hold/accelerate, lane: right/stay/left]
        return gym.spaces.MultiDiscrete(np.array([3, 3], dtype=np.int64))

    @property
    def state_space(self) -> gym.spaces.Box:
        return gym.spaces.Box(low=-1.0, high=1.0, shape=(20,), dtype=np.float32)

    def reset(self, seed: int | None = None, options: dict | None = None):
        self.np_random, _ = gym.utils.seeding.np_random(seed)
        del options
        self.close()
        self._backend = self._backend_factory()
        resolved_seed = 42 if seed is None else seed
        self._result = self._backend.reset(seed=resolved_seed)
        self._step_count = 0
        self.agents = [
            agent for agent in self.possible_agents if agent in self._backend.active_vehicle_ids
        ]
        self._previous_distance = {
            agent: float(self._result.states[agent]["distance"])
            for agent in self.agents
        }
        observations = self._observations(self.agents)
        infos = {
            agent: {
                "simulation_time": self._result.simulation_time,
                "seed": resolved_seed,
                "active_agents": tuple(self.agents),
            }
            for agent in self.agents
        }
        return observations, infos

    def step(self, actions: dict[str, np.ndarray]):
        if not self.agents:
            raise RuntimeError("step() called with no active agents; call reset().")
        transition_agents = list(self.agents)
        if set(actions) != set(transition_agents):
            raise KeyError(
                f"Actions must match active agents; expected {transition_agents}, got {sorted(actions)}"
            )

        commands: dict[str, VehicleCommand] = {}
        requested: dict[str, tuple[int, int]] = {}
        for agent in transition_agents:
            action = np.asarray(actions[agent], dtype=np.int64)
            if not self.action_space(agent).contains(action):
                raise ValueError(f"Action for {agent} is outside its declared space: {action}")
            speed_action, lane_action = (int(action[0]), int(action[1]))
            state = self._result.states[agent]
            current_speed = float(state["speed"])
            target_speed = max(0.0, current_speed + (-2.0, 0.0, 2.0)[speed_action])
            lane_delta = (-1, 0, 1)[lane_action]
            commands[agent] = VehicleCommand(target_speed=target_speed, lane_delta=lane_delta)
            requested[agent] = (speed_action, lane_action)

        self._result = self._require_backend().step(commands)
        self._step_count += 1
        collided = set(self._result.collisions)
        arrived = set(self._result.arrived)
        horizon_reached = self._step_count >= self.horizon

        rewards: dict[str, float] = {}
        terminations: dict[str, bool] = {}
        truncations: dict[str, bool] = {}
        infos: dict[str, dict] = {}
        for agent in transition_agents:
            state = self._result.states.get(agent)
            previous = self._previous_distance.get(agent, 0.0)
            current = float(state["distance"]) if state else self.route_length
            progress = max(0.0, current - previous)
            speed = float(state["speed"]) if state else 0.0
            unsafe_gap = self._nearest_gap(agent, ahead=True)
            components = {
                "progress": 0.02 * progress,
                "speed": 0.01 * min(speed / 27.0, 1.0),
                "unsafe_gap": -1.0 if unsafe_gap is not None and unsafe_gap < 5.0 else 0.0,
                "collision": -10.0 if agent in collided else 0.0,
                "completion": 5.0 if agent in arrived else 0.0,
            }
            rewards[agent] = float(sum(components.values()))
            terminations[agent] = agent in collided or agent in arrived
            truncations[agent] = horizon_reached and not terminations[agent]
            infos[agent] = {
                "simulation_time": self._result.simulation_time,
                "reward_components": components,
                "requested_action": requested[agent],
                "executed_action": self._result.executed_actions.get(agent, {}),
                "safety_intervention": any(
                    bool(self._result.executed_actions.get(agent, {}).get(key))
                    for key in ("speed_clamped", "lane_clamped")
                ),
                "active_agents": tuple(transition_agents),
                "termination_cause": (
                    "collision" if agent in collided else "route_complete" if agent in arrived else None
                ),
                "truncation_cause": "time_limit" if truncations[agent] else None,
            }
            self._previous_distance[agent] = current

        self.agents = [
            agent
            for agent in transition_agents
            if not terminations[agent]
            and not truncations[agent]
            and agent in self._require_backend().active_vehicle_ids
        ]
        observations = self._observations(self.agents)
        return observations, rewards, terminations, truncations, infos

    def state(self) -> np.ndarray:
        chunks: list[np.ndarray] = []
        for agent in self.possible_agents:
            chunks.append(self._last_observation[agent])
            chunks.append(np.array([1.0 if agent in self.agents else 0.0], dtype=np.float32))
        return np.concatenate(chunks, dtype=np.float32)

    def close(self) -> None:
        backend, self._backend = self._backend, None
        if backend is not None:
            backend.close()
        self.agents = []
        self._result = None

    def _observations(self, agents: list[str]) -> dict[str, np.ndarray]:
        observations: dict[str, np.ndarray] = {}
        for agent in agents:
            state = self._result.states[agent]
            front = self._nearest_neighbor(agent, ahead=True)
            rear = self._nearest_neighbor(agent, ahead=False)
            speed = float(state["speed"])
            observation = np.array(
                [
                    np.clip(speed / 27.0, 0.0, 1.0),
                    np.clip(float(state["lane_index"]), 0.0, 1.0),
                    np.clip(float(state["distance"]) / self.route_length, 0.0, 1.0),
                    min(front[0] / 100.0, 1.0) if front else 1.0,
                    np.clip((front[1] - speed) / 27.0, -1.0, 1.0) if front else 0.0,
                    1.0 if front else 0.0,
                    min(rear[0] / 100.0, 1.0) if rear else 1.0,
                    np.clip((rear[1] - speed) / 27.0, -1.0, 1.0) if rear else 0.0,
                    1.0 if rear else 0.0,
                ],
                dtype=np.float32,
            )
            if not self.observation_space(agent).contains(observation):
                raise ValueError(f"Observation for {agent} is outside its space: {observation}")
            observations[agent] = observation
            self._last_observation[agent] = observation
        return observations

    def _nearest_neighbor(self, agent: str, *, ahead: bool) -> tuple[float, float] | None:
        ego = self._result.states.get(agent)
        if ego is None:
            return None
        ego_distance = float(ego["distance"])
        candidates: list[tuple[float, float]] = []
        for other, state in self._result.states.items():
            if other == agent:
                continue
            delta = float(state["distance"]) - ego_distance
            if (ahead and delta > 0.0) or (not ahead and delta < 0.0):
                candidates.append((abs(delta), float(state["speed"])))
        return min(candidates, default=None, key=lambda item: item[0])

    def _nearest_gap(self, agent: str, *, ahead: bool) -> float | None:
        neighbor = self._nearest_neighbor(agent, ahead=ahead)
        return neighbor[0] if neighbor else None

    def _require_backend(self) -> SumoBackend:
        if self._backend is None:
            raise RuntimeError("Environment is not reset.")
        return self._backend

    def _validate_agent(self, agent: str) -> None:
        if agent not in self.possible_agents:
            raise KeyError(f"Unknown agent: {agent}")


def parallel_env(**kwargs) -> HighwayParallelEnv:
    return HighwayParallelEnv(**kwargs)
