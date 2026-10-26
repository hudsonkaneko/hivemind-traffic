"""Four decentralized longitudinal actors and a separate centralized critic state."""

from __future__ import annotations

from copy import deepcopy
from functools import lru_cache
import math

import gymnasium as gym
import numpy as np
from pettingzoo import ParallelEnv

from traffic.mappo_sumo import AGENT_IDS, MappoSumoBackend

DEFAULT_CONFIG = {
    "dt": 0.1, "horizon": 600, "speed_limit_m_s": 26.8224, "route_length_m": 5000.0,
    "neighbor_range_m": 120.0, "acceleration_min_m_s2": -4.5, "acceleration_max_m_s2": 2.0,
    "min_gap_m": 2.5, "time_headway_s": 1.5, "initial_speed_range_m_s": [12.0, 22.0],
    "background_depart_interval_range_s": [1.5, 3.0],
    "reward_weights": {"progress": 1.0, "headway": -0.5, "jerk": -0.02,
                       "collision": -10.0, "intervention": -0.1},
}
observation_features = (
    "speed_over_cap", "acceleration_over_4p5", "lane_index_over_2", "remaining_route_fraction",
    "speed_limit_over_cap", "previous_requested_acceleration_over_4p5",
    *(f"{lane}_{direction}_{feature}" for lane in ("own", "left", "right")
      for direction in ("front", "rear") for feature in ("gap_over_120", "relative_speed_over_cap", "present")),
    "left_lane_exists", "right_lane_exists", "elapsed_fraction",
)
OBSERVATION_SIZE = len(observation_features)
STATE_SIZE = len(AGENT_IDS) * (OBSERVATION_SIZE + 1)


def action_to_acceleration(action: float) -> float:
    """Affine normalized [-1,1] action to physical [-4.5,2.0] m/s²."""
    value = float(action)
    if not math.isfinite(value) or not -1.0 <= value <= 1.0:
        raise ValueError("Normalized acceleration must be finite and in [-1,1]")
    return -4.5 + (value + 1.0) * 3.25


def physical_acceleration_to_action(acceleration: float) -> float:
    value = float(acceleration)
    if not math.isfinite(value) or not -4.5 <= value <= 2.0:
        raise ValueError("Physical acceleration must be finite and in [-4.5,2.0]")
    return (value + 4.5) / 3.25 - 1.0


class SumoMappoEnv(ParallelEnv):
    metadata = {"name": "sumo_mappo_highway_v01", "render_modes": [], "is_parallelizable": True}
    observation_features = observation_features

    def __init__(self, config=None, backend_factory=None, run_directory=None):
        self.config = deepcopy(DEFAULT_CONFIG)
        if config:
            unknown = set(config) - set(DEFAULT_CONFIG)
            if unknown:
                raise ValueError(f"Unknown environment settings: {sorted(unknown)}")
            self.config.update(deepcopy(config))
        self._validate_config()
        self.dt = float(self.config["dt"])
        self.horizon = int(self.config["horizon"])
        self.possible_agents = list(AGENT_IDS)
        self.agents = []
        self._backend_factory = backend_factory or (lambda: MappoSumoBackend(self.config, run_directory))
        self._backend = None
        self._result = None
        self._step_count = 0
        self._previous_requests = {agent: 0.0 for agent in self.possible_agents}
        self._last_observations = {}
        self.control_mode = "policy"
        self.state_space = gym.spaces.Box(-1.0, 1.0, (STATE_SIZE,), np.float32)

    def _validate_config(self):
        cfg = self.config
        for key in ("dt", "speed_limit_m_s", "route_length_m", "neighbor_range_m", "min_gap_m", "time_headway_s"):
            if not math.isfinite(float(cfg[key])) or cfg[key] <= 0:
                raise ValueError(f"{key} must be finite and positive")
        if not isinstance(cfg["horizon"], int) or cfg["horizon"] < 1:
            raise ValueError("horizon must be a positive integer")
        if cfg["dt"] != 0.1 or cfg["route_length_m"] != 5000.0 or cfg["speed_limit_m_s"] != 26.8224:
            raise ValueError("v01 fixes dt=0.1, route_length_m=5000 and speed_limit_m_s=26.8224")
        if cfg["acceleration_min_m_s2"] != -4.5 or cfg["acceleration_max_m_s2"] != 2.0:
            raise ValueError("v01 acceleration and observation scales are fixed at [-4.5,2.0]")
        if cfg["neighbor_range_m"] != 120.0 or cfg["min_gap_m"] != 2.5 or cfg["time_headway_s"] != 1.5:
            raise ValueError("v01 neighbor range, vehicle minGap and headway are fixed at 120,2.5,1.5")
        for key in ("initial_speed_range_m_s", "background_depart_interval_range_s"):
            value = cfg[key]
            if len(value) != 2 or not all(math.isfinite(float(x)) for x in value) or not 0 < value[0] <= value[1]:
                raise ValueError(f"Invalid range for {key}")
        if cfg["initial_speed_range_m_s"][1] > cfg["speed_limit_m_s"]:
            raise ValueError("Initial speeds exceed the ceiling")
        if set(cfg["reward_weights"]) != set(DEFAULT_CONFIG["reward_weights"]):
            raise ValueError("Reward weights must name every v01 component")
        if not all(math.isfinite(float(x)) for x in cfg["reward_weights"].values()):
            raise ValueError("Reward weights must be finite")

    @lru_cache(maxsize=None)
    def observation_space(self, agent):
        self._validate_agent(agent)
        low = np.zeros(OBSERVATION_SIZE, dtype=np.float32)
        low[[1, 5, 7, 10, 13, 16, 19, 22]] = -1.0
        return gym.spaces.Box(low, np.ones(OBSERVATION_SIZE, dtype=np.float32), dtype=np.float32)

    @lru_cache(maxsize=None)
    def action_space(self, agent):
        self._validate_agent(agent)
        return gym.spaces.Box(-1.0, 1.0, (1,), np.float32)

    def reset(self, seed=None, options=None):
        # PettingZoo permits an arbitrary options dictionary; v01 has no option-dependent reset behavior.
        del options
        seed = 42 if seed is None else int(seed)
        if not 0 <= seed <= 2**31 - 1:
            raise ValueError("SUMO seed must fit a nonnegative signed 32-bit integer")
        self.close()
        self.np_random, _ = gym.utils.seeding.np_random(seed)
        self._backend = self._backend_factory()
        try:
            self._result = self._backend.reset(seed=seed)
            if not set(self.possible_agents).issubset(self._result.states):
                raise RuntimeError("All four controlled agents must exist after initial spawn")
            self.agents = list(self.possible_agents)
            self._step_count = 0
            self._previous_requests = {agent: 0.0 for agent in self.possible_agents}
            observations = self._observations(self.agents)
            return observations, {agent: {"seed": seed, "simulation_time": self._result.simulation_time,
                "active_agents": tuple(self.agents)} for agent in self.agents}
        except BaseException:
            self.close()
            raise

    def step(self, actions):
        if not self.agents:
            if actions:
                raise KeyError("No active agents remain")
            return {}, {}, {}, {}, {}
        transition_agents = list(self.agents)
        if self.control_mode not in ("policy", "native"):
            raise ValueError("control_mode must be policy or native")
        if set(actions) != set(transition_agents):
            raise KeyError("Provide exactly one action for every active agent")
        requested, filtered, reasons = {}, {}, {}
        # Validate the full joint action before modifying either the simulator or history.
        for agent in transition_agents:
            action = np.asarray(actions[agent], dtype=np.float32)
            if not self.action_space(agent).contains(action):
                raise ValueError(f"Action outside declared space for {agent}: {action}")
            requested[agent] = action_to_acceleration(float(action[0]))
            filtered[agent], reasons[agent] = self._safety_filter(agent, requested[agent])
        before = self._result
        self._result = self._backend.step(filtered if self.control_mode == "policy" else {})
        self._step_count += 1
        if not math.isclose(self._result.simulation_time - before.simulation_time, self.dt, abs_tol=1e-7):
            raise RuntimeError("Backend did not advance exactly one control interval")
        self._previous_requests.update(requested if self.control_mode == "policy" else
                                       {agent: 0.0 for agent in transition_agents})
        observations = self._observations(transition_agents)
        any_collision = bool(self._result.collisions)
        any_teleport = bool(self._result.teleported)
        terminations, truncations, infos, local_rewards = {}, {}, {}, {}
        for agent in transition_agents:
            state, prior = self._result.states.get(agent), before.states[agent]
            arrived = agent in self._result.arrived
            missing = state is None
            terminations[agent] = any_collision or any_teleport or arrived or missing
            truncations[agent] = self._step_count >= self.horizon and not terminations[agent]
            progress = max(0.0, float(state["distance"] - prior["distance"])) if state else 0.0
            acceleration = float(state["acceleration"]) if state else None
            neighbor = self._neighbor(agent, 0, True)
            gap = neighbor[0] if neighbor else None
            desired_gap = self.config["min_gap_m"] + self.config["time_headway_s"] * state["speed"] if state else 0.0
            sumo_intervention = (self.control_mode == "policy" and acceleration is not None
                                 and abs(acceleration - filtered[agent]) > 1e-4)
            local_intervention = self.control_mode == "policy" and bool(reasons[agent])
            intervention = local_intervention or sumo_intervention
            components = {
                "progress": float(np.clip(progress / (self.config["speed_limit_m_s"] * self.dt), 0, 1)),
                "headway": float(np.clip((desired_gap - gap) / desired_gap, 0, 1)) if gap is not None else 0.0,
                "jerk": float(np.clip(abs(acceleration - prior["acceleration"]) / self.dt / 10.0, 0, 1)) if state else 0.0,
                "collision": float(agent in self._result.collisions),
                "intervention": float(intervention),
            }
            weighted = {key: self.config["reward_weights"][key] * value for key, value in components.items()}
            local_rewards[agent] = float(sum(weighted.values()))
            infos[agent] = dict(simulation_time=self._result.simulation_time, episode_step=self._step_count,
                speed_m_s=float(state["speed"]) if state else None, acceleration_m_s2=acceleration,
                front_gap_m=gap, nearest_gap_m=gap, progress_m=progress,
                relative_closing_speed_m_s=float(state["speed"] - neighbor[1]) if neighbor else None,
                min_ttc_s=float(gap / (state["speed"] - neighbor[1]))
                    if neighbor and state["speed"] > neighbor[1] else None,
                distance_m=float(state["distance"]) if state else None,
                lane_index=int(state["lane_index"]) if state else None,
                requested_acceleration_m_s2=requested[agent] if self.control_mode == "policy" else None,
                filtered_acceleration_m_s2=filtered[agent] if self.control_mode == "policy" else None,
                action_source="policy" if self.control_mode == "policy" else "SUMO_native",
                realized_acceleration_m_s2=acceleration, local_safety_intervention=local_intervention,
                sumo_safety_intervention=sumo_intervention, safety_intervention=intervention,
                safety_reasons=reasons[agent] if self.control_mode == "policy" else (),
                reward_components=components, weighted_reward_components=weighted,
                local_reward=local_rewards[agent], active_agents=tuple(transition_agents),
                background_vehicle_count=sum(vehicle not in self.possible_agents for vehicle in self._result.states),
                collisions=self._result.collisions, teleported=self._result.teleported,
                termination_cause=("collision" if any_collision else "teleport" if any_teleport else
                                   "route_complete" if arrived else "unexpected_disappearance" if missing else None),
                truncation_cause="time_limit" if truncations[agent] else None,
                observation_available=state is not None)
        # A time limit needs the measured final state for V(s_{t+1}); true terminals have no bootstrap.
        bootstrap_agents = [agent for agent in transition_agents if not terminations[agent]]
        terminal_state = self._state_for(bootstrap_agents)
        shared_reward = float(np.mean(list(local_rewards.values())))
        rewards = {agent: shared_reward for agent in transition_agents}
        for info in infos.values():
            info["team_reward"] = shared_reward
            info["terminal_state"] = terminal_state.copy()
        self.agents = [agent for agent in transition_agents if not terminations[agent] and not truncations[agent]]
        return observations, rewards, terminations, truncations, infos

    def _neighbor(self, agent, lane_delta, ahead):
        ego = self._result.states.get(agent)
        if ego is None:
            return None
        lane = ego["lane_index"] + lane_delta
        if not 0 <= lane < 3:
            return None
        candidates = []
        for other, state in self._result.states.items():
            if other == agent or state["lane_index"] != lane:
                continue
            delta = state["position"] - ego["position"]
            if (ahead and delta >= 0) or (not ahead and delta < 0):
                # SUMO lane positions locate front bumpers. Subtract the vehicle between the bumpers.
                gap = max(0.0, abs(delta) - (state["length"] if ahead else ego["length"]))
                if gap <= self.config["neighbor_range_m"]:
                    candidates.append((gap, state["speed"], other))
        return min(candidates, default=None, key=lambda value: (value[0], value[2]))

    def _observations(self, agents):
        observations = {}
        for agent in agents:
            state = self._result.states.get(agent)
            if state is None:
                observation = np.zeros(OBSERVATION_SIZE, dtype=np.float32)
            else:
                speed = state["speed"]
                cap = self.config["speed_limit_m_s"]
                features = [np.clip(speed / cap, 0, 1), np.clip(state["acceleration"] / 4.5, -1, 1),
                    state["lane_index"] / 2, np.clip(1 - state["position"] / self.config["route_length_m"], 0, 1),
                    np.clip(state["speed_limit"] / cap, 0, 1), self._previous_requests[agent] / 4.5]
                for lane_delta in (0, 1, -1):
                    for ahead in (True, False):
                        neighbor = self._neighbor(agent, lane_delta, ahead)
                        features.extend([neighbor[0] / self.config["neighbor_range_m"],
                                         np.clip((neighbor[1] - speed) / cap, -1, 1), 1.0] if neighbor else [0, 0, 0])
                features.extend([float(state["lane_index"] < 2), float(state["lane_index"] > 0),
                                 min(1.0, self._step_count / self.horizon)])
                observation = np.asarray(features, dtype=np.float32)
                if not self.observation_space(agent).contains(observation):
                    raise ValueError(f"Invalid observation for {agent}: {observation}")
            observations[agent] = observation
            self._last_observations[agent] = observation
        return observations

    def _safety_filter(self, agent, requested):
        speed = self._result.states[agent]["speed"]
        filtered = min(requested, (self.config["speed_limit_m_s"] - speed) / self.dt)
        reasons = ["speed_ceiling"] if filtered < requested - 1e-8 else []
        neighbor = self._neighbor(agent, 0, True)
        if neighbor is not None:
            gap, _, _ = neighbor
            safe_speed = max(0.0, (gap - self.config["min_gap_m"]) / self.config["time_headway_s"])
            # The local guard only uses the same current-lane neighbor available to the actor.
            safe_acceleration = max(-4.5, (safe_speed - speed) / self.dt)
            if safe_acceleration < filtered - 1e-8:
                filtered = safe_acceleration
                reasons.append("headway")
        stopping_bound = -speed / self.dt
        if filtered < stopping_bound - 1e-8:
            filtered = stopping_bound
            reasons.append("zero_speed")
        return float(np.clip(filtered, -4.5, 2.0)), tuple(reasons)

    def _state_for(self, active_agents):
        active = set(active_agents)
        return np.concatenate([np.concatenate((self._last_observations[agent], [1])).astype(np.float32)
            if agent in active else np.zeros(OBSERVATION_SIZE + 1, dtype=np.float32)
            for agent in self.possible_agents])

    def state(self):
        return self._state_for(self.agents)

    def close(self):
        backend, self._backend = self._backend, None
        self.agents = []
        self._result = None
        self._last_observations = {}
        if backend is not None:
            backend.close()

    def _validate_agent(self, agent):
        if agent not in self.possible_agents:
            raise KeyError(agent)


def parallel_env(**kwargs):
    return SumoMappoEnv(**kwargs)
