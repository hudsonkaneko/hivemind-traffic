"""No-learning unit/API checks, with SUMO behind a deterministic fake."""

from copy import deepcopy

import numpy as np
import pytest
from pettingzoo.test import parallel_api_test, parallel_seed_test

from environments.sumo_mappo_env import (SumoMappoEnv, action_to_acceleration,
    physical_acceleration_to_action, observation_features)
from traffic.mappo_sumo import AGENT_IDS, MappoStepResult


class FakeBackend:
    def __init__(self, event=None, override=False):
        self.event = event
        self.override = override
        self.closed = False
        self.steps = 0
        self.commands = []

    def reset(self, seed=42):
        self.closed = False
        self.steps = 0
        self.commands = []
        rng = np.random.default_rng(seed)
        self.states = {}
        for index, agent in enumerate(AGENT_IDS):
            self.states[agent] = dict(speed=10 + float(rng.uniform(0, 1)), acceleration=0.0,
                lane_index=(0, 0, 1, 2)[index], position=(220, 130, 175, 190)[index],
                distance=0.0, length=4.8, speed_limit=26.8224)
        self.states["ordinary"] = dict(speed=12.0, acceleration=0.0, lane_index=0, position=310.0,
            distance=0.0, length=5.0, speed_limit=26.8224)
        return MappoStepResult(0.1, deepcopy(self.states))

    def step(self, commands):
        self.commands.append(dict(commands))
        self.steps += 1
        for vehicle, state in self.states.items():
            requested = commands.get(vehicle, 1.0)
            state["acceleration"] = min(requested, -2.0) if self.override and vehicle in commands else requested
            state["speed"] = max(0.0, state["speed"] + state["acceleration"] * 0.1)
            state["position"] += state["speed"] * 0.1
            state["distance"] += state["speed"] * 0.1
        arrived = collisions = teleported = ()
        if self.event == "arrive":
            self.states.pop("agent_0", None)
            arrived = ("agent_0",)
        elif self.event == "disappear":
            self.states.pop("agent_0", None)
        elif self.event == "collide":
            collisions = ("agent_0", "ordinary")
        elif self.event == "teleport":
            teleported = ("ordinary",)
        return MappoStepResult(0.1 + self.steps * 0.1, deepcopy(self.states), arrived=arrived,
                              collisions=collisions, teleported=teleported)

    def close(self):
        self.closed = True


def actions(env, physical=0.0):
    return {agent: np.array([physical_acceleration_to_action(physical)], dtype=np.float32) for agent in env.agents}


def make_env(**kwargs):
    return SumoMappoEnv({"horizon": 17}, backend_factory=lambda: FakeBackend(**kwargs))


def test_action_mapping_is_asymmetric_and_invertible():
    assert action_to_acceleration(-1) == -4.5
    assert action_to_acceleration(0) == -1.25
    assert action_to_acceleration(1) == 2.0
    assert physical_acceleration_to_action(0) == pytest.approx(0.384615384615)
    for physical in np.linspace(-4.5, 2, 101):
        assert action_to_acceleration(physical_acceleration_to_action(physical)) == pytest.approx(physical)
    for invalid in (float("nan"), float("inf"), -1.01, 1.01):
        with pytest.raises(ValueError):
            action_to_acceleration(invalid)


def test_observation_contract_neighbors_and_no_global_actor_leak():
    env = make_env()
    obs, _ = env.reset(seed=1)
    assert len(observation_features) == 27
    assert env.state().shape == (112,)
    assert env.state_space.contains(env.state())
    assert all(env.observation_space(a).contains(o) for a, o in obs.items())
    assert obs["agent_0"][6] == pytest.approx((310 - 220 - 5) / 120)
    assert obs["agent_0"][8] == 1
    assert obs["agent_0"][24:26].tolist() == [1, 0]
    assert obs["agent_3"][24:26].tolist() == [0, 1]
    before = obs["agent_0"].copy()
    env._result.states["distant"] = dict(env._result.states["ordinary"], position=4000, speed=1)
    assert np.array_equal(env._observations(["agent_0"])["agent_0"], before)
    env.close()


def test_batch_atomic_validation_and_one_clock_step():
    env = make_env()
    env.reset()
    backend = env._backend
    invalid = actions(env)
    invalid["agent_3"] = np.array([float("nan")], dtype=np.float32)
    with pytest.raises(ValueError):
        env.step(invalid)
    assert backend.steps == 0
    assert all(value == 0 for value in env._previous_requests.values())
    with pytest.raises(KeyError):
        env.step({"agent_0": np.array([0], dtype=np.float32)})
    obs, rewards, terminated, truncated, infos = env.step(actions(env))
    assert backend.steps == 1
    assert set(obs) == set(rewards) == set(terminated) == set(truncated) == set(infos) == set(AGENT_IDS)
    assert infos["agent_0"]["simulation_time"] == pytest.approx(0.2)
    env.close()


def test_shared_reward_logs_weighted_components_and_native_adjustments():
    env = make_env(override=True)
    env.reset()
    _, rewards, _, _, infos = env.step(actions(env, 1.0))
    expected = np.mean([sum(info["weighted_reward_components"].values()) for info in infos.values()])
    assert all(value == pytest.approx(expected) for value in rewards.values())
    assert all(info["sumo_safety_intervention"] for info in infos.values())
    assert infos["agent_0"]["requested_acceleration_m_s2"] == pytest.approx(1.0)
    assert infos["agent_0"]["realized_acceleration_m_s2"] == -2.0
    env.close()


def test_local_guard_reports_headway_speed_and_stop_bounds():
    env = make_env()
    env.reset()
    ego = env._result.states["agent_0"]
    leader = env._result.states["ordinary"]
    leader["position"] = ego["position"] + 7.5
    filtered, reasons = env._safety_filter("agent_0", 2.0)
    assert filtered == -4.5 and "headway" in reasons
    leader["position"] = 4000
    ego["speed"] = 26.8224
    filtered, reasons = env._safety_filter("agent_0", 2.0)
    assert filtered == 0 and "speed_ceiling" in reasons
    ego["speed"] = 0.01
    filtered, reasons = env._safety_filter("agent_0", -4.5)
    assert filtered == pytest.approx(-0.1) and "zero_speed" in reasons
    env.close()


def test_time_limit_preserves_final_observation_and_bootstrap_state():
    env = SumoMappoEnv({"horizon": 1}, backend_factory=FakeBackend)
    initial, _ = env.reset()
    obs, _, terminated, truncated, infos = env.step(actions(env))
    assert not any(terminated.values()) and all(truncated.values())
    assert not env.agents and not env.state().any()
    assert obs["agent_0"][26] == 1 and not np.array_equal(obs["agent_0"], initial["agent_0"])
    final_state = infos["agent_0"]["terminal_state"].reshape(4, 28)
    assert (final_state[:, -1] == 1).all()
    assert np.array_equal(final_state[0, :27], obs["agent_0"])
    assert env.step({}) == ({}, {}, {}, {}, {})
    env.close()


@pytest.mark.parametrize("event,cause", [("arrive", "route_complete"), ("disappear", "unexpected_disappearance")])
def test_departed_agent_removed_once_without_stale_observations(event, cause):
    env = make_env(event=event)
    env.reset()
    obs, _, terminated, truncated, infos = env.step(actions(env))
    assert terminated["agent_0"] and not truncated["agent_0"]
    assert "agent_0" not in env.agents and not obs["agent_0"].any()
    assert infos["agent_0"]["termination_cause"] == cause
    final_state = infos["agent_1"]["terminal_state"].reshape(4, 28)
    assert not final_state[0].any() and (final_state[1:, -1] == 1).all()
    assert not env.state()[:28].any()
    env.close()


@pytest.mark.parametrize("event", ["collide", "teleport"])
def test_safety_event_terminates_joint_episode(event):
    env = make_env(event=event)
    env.reset()
    _, _, terminated, truncated, infos = env.step(actions(env))
    assert all(terminated.values()) and not any(truncated.values())
    assert not env.agents
    assert not infos["agent_0"]["terminal_state"].any()
    env.close()


def test_native_baseline_sends_no_acceleration_and_close_idempotent():
    env = make_env()
    env.control_mode = "native"
    env.reset()
    backend = env._backend
    _, _, _, _, infos = env.step(actions(env, -4.5))
    assert backend.commands == [{}]
    assert all(info["action_source"] == "SUMO_native" and not info["safety_intervention"] for info in infos.values())
    assert infos["agent_0"]["requested_acceleration_m_s2"] is None
    env.close()
    env.close()
    assert backend.closed


def test_reset_failure_releases_backend():
    backend = FakeBackend()
    original = backend.reset
    def fail(seed):
        original(seed)
        raise RuntimeError("injected reset failure")
    backend.reset = fail
    env = SumoMappoEnv(backend_factory=lambda: backend)
    with pytest.raises(RuntimeError, match="injected"):
        env.reset()
    assert backend.closed and not env.agents


def test_parallel_api_and_seed_contract():
    env = make_env()
    try:
        parallel_api_test(env, num_cycles=100)
    finally:
        env.close()
    parallel_seed_test(make_env, num_cycles=100)


@pytest.mark.parametrize("config", [{"horizon": 0}, {"dt": 0.2}, {"speed_limit_m_s": 30},
    {"initial_speed_range_m_s": [20, 10]}, {"unrecognized": True}])
def test_invalid_contract_rejected(config):
    with pytest.raises(ValueError):
        SumoMappoEnv(config)
