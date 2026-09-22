from __future__ import annotations

import numpy as np
from pettingzoo.test import parallel_api_test, parallel_seed_test

from environments.highway_parallel_env import HighwayParallelEnv
from traffic.sumo_backend import StepResult


class FakeBackend:
    """Small deterministic backend for environment-only lifecycle tests."""

    def __init__(self):
        self.active_vehicle_ids = ()
        self.step_count = 0

    def reset(self, seed=42):
        del seed
        self.step_count = 0
        self.active_vehicle_ids = ("agent_0", "agent_1")
        return self._result(departed=self.active_vehicle_ids)

    def step(self, commands):
        self.step_count += 1
        arrived = ("agent_0",) if self.step_count == 2 else ()
        if arrived:
            self.active_vehicle_ids = ("agent_1",)
        return self._result(arrived=arrived, commands=commands)

    def close(self):
        self.active_vehicle_ids = ()

    def _result(self, departed=(), arrived=(), commands=None):
        states = {
            agent: {
                "x": float(10 * self.step_count + index),
                "y": 0.0,
                "speed": 10.0,
                "acceleration": 0.0,
                "lane_index": index,
                "lane_id": f"lane_{index}",
                "road_id": "fake_road",
                "distance": float(10 * self.step_count + index),
                "waiting_time": 0.0,
                "time_loss": 0.0,
            }
            for index, agent in enumerate(self.active_vehicle_ids)
        }
        commands = commands or {}
        return StepResult(
            simulation_time=0.2 * (self.step_count + 1),
            states=states,
            departed=tuple(departed),
            arrived=tuple(arrived),
            collisions=(),
            requested_actions={agent: {} for agent in commands},
            executed_actions={
                agent: {
                    "target_speed": command.target_speed,
                    "speed_clamped": False,
                    "lane_delta": command.lane_delta,
                    "lane_clamped": False,
                }
                for agent, command in commands.items()
            },
        )


def hold_actions(env: HighwayParallelEnv) -> dict[str, np.ndarray]:
    return {agent: np.array([1, 1], dtype=np.int64) for agent in env.agents}


def rollout(seed: int, steps: int = 40):
    env = HighwayParallelEnv(horizon=steps)
    trace = []
    try:
        observations, _ = env.reset(seed=seed)
        trace.append({agent: value.tolist() for agent, value in observations.items()})
        while env.agents:
            transition = env.step(hold_actions(env))
            observations, rewards, terminations, truncations, infos = transition
            trace.append(
                {
                    "observations": {agent: value.tolist() for agent, value in observations.items()},
                    "rewards": rewards,
                    "terminations": terminations,
                    "truncations": truncations,
                    "times": {agent: info["simulation_time"] for agent, info in infos.items()},
                }
            )
    finally:
        env.close()
    return trace


def test_spaces_and_global_state():
    env = HighwayParallelEnv(horizon=5)
    try:
        observations, _ = env.reset(seed=42)
        assert env.agents == ["agent_0", "agent_1"]
        for agent, observation in observations.items():
            assert env.observation_space(agent).contains(observation)
            assert env.action_space(agent).contains(np.array([1, 1], dtype=np.int64))
        assert env.state_space.contains(env.state())
    finally:
        env.close()


def test_fake_backend_agent_lifecycle_and_rewards():
    env = HighwayParallelEnv(horizon=5, backend_factory=FakeBackend)
    try:
        env.reset(seed=42)
        env.step(hold_actions(env))
        observations, rewards, terminations, truncations, infos = env.step(hold_actions(env))
        assert env.agents == ["agent_1"]
        assert set(observations) == {"agent_1"}
        assert set(rewards) == {"agent_0", "agent_1"}
        assert terminations["agent_0"]
        assert not truncations["agent_0"]
        assert infos["agent_0"]["termination_cause"] == "route_complete"
        assert infos["agent_0"]["reward_components"]["completion"] == 5.0
    finally:
        env.close()


def test_safety_shield_logs_boundary_and_headway_overrides():
    env = HighwayParallelEnv(horizon=5, backend_factory=FakeBackend)
    try:
        env.reset(seed=42)
        _, _, _, _, infos = env.step(
            {
                "agent_0": np.array([2, 0], dtype=np.int64),
                "agent_1": np.array([1, 2], dtype=np.int64),
            }
        )
        assert infos["agent_0"]["proposed_action"] == (2, 0)
        assert infos["agent_0"]["applied_action"] == (0, 1)
        assert set(infos["agent_0"]["safety_reasons"]) == {
            "emergency_headway",
            "road_boundary",
        }
        assert infos["agent_1"]["applied_action"][1] == 1
        assert infos["agent_1"]["safety_intervention"]
    finally:
        env.close()


def test_joint_action_advances_sumo_once():
    env = HighwayParallelEnv(horizon=5)
    try:
        _, infos = env.reset(seed=42)
        before = infos["agent_0"]["simulation_time"]
        _, _, _, _, infos = env.step(hold_actions(env))
        after = infos["agent_0"]["simulation_time"]
        assert after - before == 0.2
    finally:
        env.close()


def test_fixed_action_rollout_is_deterministic():
    assert rollout(seed=42) == rollout(seed=42)


def test_two_agents_complete_curved_route():
    env = HighwayParallelEnv(horizon=400)
    completed = set()
    try:
        env.reset(seed=42)
        while env.agents:
            actions = {
                agent: np.array([2, 1], dtype=np.int64) for agent in env.agents
            }
            _, _, terminations, truncations, infos = env.step(actions)
            for agent, terminated in terminations.items():
                if terminated and infos[agent]["termination_cause"] == "route_complete":
                    completed.add(agent)
            assert not any(truncations.values())
        assert completed == set(env.possible_agents)
    finally:
        env.close()


def test_parallel_api_contract():
    env = HighwayParallelEnv(horizon=60)
    try:
        parallel_api_test(env, num_cycles=50)
    finally:
        env.close()


def test_parallel_seed_contract():
    parallel_seed_test(lambda: HighwayParallelEnv(horizon=60), num_cycles=50)
