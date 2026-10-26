"""Bounded headless SUMO integration. No model, gradients, or optimization."""

from copy import deepcopy
import json
from pathlib import Path
from uuid import uuid4

import numpy as np
import pytest

from environments.sumo_mappo_env import DEFAULT_CONFIG, SumoMappoEnv, physical_acceleration_to_action
from pettingzoo.test import parallel_api_test, parallel_seed_test

from traffic.mappo_sumo import AGENT_IDS, MappoSumoBackend, ROOT, write_episode
from traffic.sumo_backend import find_sumo_binary


def require_sumo():
    try:
        return find_sumo_binary()
    except FileNotFoundError:
        pytest.skip("SUMO is not installed")


@pytest.fixture
def project_tmp_path(request):
    """Keep runtime artifacts inside the checkout even with pytest's default OS temp settings."""
    directory = ROOT / "outputs" / "sumo_mappo_tests" / f"{request.node.name}-{uuid4().hex[:8]}"
    directory.mkdir(parents=True, exist_ok=False)
    return directory


def test_seeded_demand_is_reproducible_and_changes_with_seed(project_tmp_path):
    tmp_path = project_tmp_path
    one = write_episode(deepcopy(DEFAULT_CONFIG), 500001, tmp_path / "one")
    two = write_episode(deepcopy(DEFAULT_CONFIG), 500001, tmp_path / "two")
    three = write_episode(deepcopy(DEFAULT_CONFIG), 500002, tmp_path / "three")
    assert one == two and one["vehicles"] != three["vehicles"]
    assert (tmp_path / "one/demand.rou.xml").read_bytes() == (tmp_path / "two/demand.rou.xml").read_bytes()
    assert set(AGENT_IDS).issubset({vehicle["id"] for vehicle in one["vehicles"]})
    assert any(vehicle["depart"] > 0 for vehicle in one["vehicles"])
    with pytest.raises(FileExistsError):
        write_episode(deepcopy(DEFAULT_CONFIG), 500001, tmp_path / "one")


def test_backend_real_step_safety_modes_and_close(project_tmp_path):
    require_sumo()
    tmp_path = project_tmp_path
    backend = MappoSumoBackend(dict(DEFAULT_CONFIG, horizon=10), tmp_path / "backend")
    try:
        first = backend.reset(seed=500001)
        assert set(AGENT_IDS).issubset(first.states)
        assert backend.simulation_steps == 1
        for agent in AGENT_IDS:
            assert backend.connection.vehicle.getSpeedMode(agent) == 31
            assert backend.connection.vehicle.getLaneChangeMode(agent) == 512
        second = backend.step({agent: 0.0 for agent in AGENT_IDS})
        assert backend.simulation_steps == 2
        assert second.simulation_time - first.simulation_time == pytest.approx(0.1)
        assert backend.version[1].startswith("SUMO 1.27")
        runtime = json.loads((backend.episode_directory / "runtime.json").read_text(encoding="utf-8"))
        assert "--error-log" in runtime["command"]
        assert Path(runtime["error_log"]).is_relative_to(ROOT / "logs")
        assert Path(runtime["error_log"]).is_file()
        with pytest.raises(ValueError):
            backend.step({"agent_0": float("nan")})
        assert backend.simulation_steps == 2
    finally:
        backend.close()
        backend.close()
    assert not backend.active_vehicle_ids


def rollout(directory, seed, mode="policy", horizon=100):
    env = SumoMappoEnv({"horizon": horizon}, run_directory=directory)
    env.control_mode = mode
    trace = []
    try:
        obs, _ = env.reset(seed=seed)
        initial_lanes = {agent: float(value[2]) for agent, value in obs.items()}
        while env.agents:
            actions = {agent: np.array([physical_acceleration_to_action(1.0)], dtype=np.float32) for agent in env.agents}
            obs, rewards, terminated, truncated, infos = env.step(actions)
            trace.append({agent: {"observation": obs[agent].tolist(), "reward": rewards[agent],
                "speed": info["speed_m_s"], "gap": info["front_gap_m"], "progress": info["progress_m"],
                "intervention": info["safety_intervention"], "background": info["background_vehicle_count"],
                "terminated": terminated[agent], "truncated": truncated[agent]} for agent, info in infos.items()})
            assert all(not value for value in terminated.values())
            assert all(obs[agent][2] == initial_lanes[agent] for agent in obs)
            assert all(info["speed_m_s"] <= 26.8224 + 1e-5 for info in infos.values())
            assert all(env.observation_space(agent).contains(value) for agent, value in obs.items())
        assert all(truncated.values())
        final = infos["agent_0"]["terminal_state"].reshape(4, 28)
        assert (final[:, -1] == 1).all() and not env.state().any()
    finally:
        env.close()
    return trace


def test_real_sumo_replayability_dynamic_background_and_time_limit(project_tmp_path):
    require_sumo()
    tmp_path = project_tmp_path
    first = rollout(tmp_path / "first", 500001)
    second = rollout(tmp_path / "second", 500001)
    different = rollout(tmp_path / "different", 500002)
    assert first == second and first != different
    assert first[-1]["agent_0"]["background"] > first[0]["agent_0"]["background"]
    (tmp_path / "deterministic-scripted-trace.json").write_text(json.dumps(first, indent=2), encoding="utf-8")


def test_real_sumo_parallel_api_and_seed(project_tmp_path):
    require_sumo()
    tmp_path = project_tmp_path
    def factory():
        return SumoMappoEnv({"horizon": 25}, run_directory=tmp_path / "real-api")
    env = factory()
    try:
        parallel_api_test(env, num_cycles=30)
    finally:
        env.close()
    parallel_seed_test(factory, num_cycles=30)


@pytest.mark.parametrize("mode", ["policy", "native"])
def test_full_sixty_second_scripted_and_native_baselines(project_tmp_path, mode):
    require_sumo()
    tmp_path = project_tmp_path
    trace = rollout(tmp_path / mode, 500001, mode=mode, horizon=600)
    assert len(trace) == 600
    assert all(sum(step[agent]["progress"] for step in trace) > 600 for agent in AGENT_IDS)
    if mode == "native":
        assert not any(step[agent]["intervention"] for step in trace for agent in AGENT_IDS)
    (tmp_path / f"{mode}-trace.json").write_text(json.dumps(trace, indent=2), encoding="utf-8")
