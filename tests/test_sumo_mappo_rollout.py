"""Collector protocol tests with in-memory workers; no SUMO or optimization."""

import copy
import time

import numpy as np
import pytest
import torch

from experiments.sumo_mappo_rollout import AGENTS, RolloutCollector


def state(marker):
    value = np.zeros(112, dtype=np.float32)
    value[0] = marker
    return value


def observations(agents, marker):
    result = {}
    for agent in agents:
        value = np.zeros(27, dtype=np.float32)
        value[0] = AGENTS.index(agent)
        value[1] = marker
        result[agent] = value
    return result


def initial(agents=AGENTS, marker=2):
    return dict(observations=observations(agents, marker), state=state(marker), agents=list(agents), infos={})


def reply(before=AGENTS, after=AGENTS, marker=3, terminal_marker=None, reward=1, truncated=False):
    terminal_marker = marker if terminal_marker is None else terminal_marker
    packet = initial(after, marker)
    packet.update(
        rewards={agent: float(reward) for agent in before},
        terminated={agent: agent not in after and not truncated for agent in before},
        truncated={agent: bool(truncated and agent not in after) for agent in before},
        infos={agent: dict(terminal_state=state(terminal_marker), progress_m=1.0,
                          truncation_cause="time_limit" if truncated else None,
                          termination_cause="route_complete" if agent not in after and not truncated else None)
               for agent in before},
    )
    return packet


class FakePool:
    """Queues are per worker; an episode is (reset_packet, step_packets)."""

    def __init__(self, scripts):
        self.processes = [object() for _ in scripts]
        self.scripts = copy.deepcopy(scripts)
        self.episodes = [-1] * len(scripts)
        self.steps = [0] * len(scripts)
        self.calls = []
        self.live = {}

    def request(self, commands):
        self.calls.append(copy.deepcopy(commands))
        responses = {}
        for worker, (operation, value) in commands.items():
            if operation == "reset":
                self.episodes[worker] += 1
                self.steps[worker] = 0
                response = self.scripts[worker][self.episodes[worker]][0]
            else:
                assert operation == "step"
                assert set(value) == set(self.live[worker])
                assert all(action.shape == (1,) for action in value.values())
                response = self.scripts[worker][self.episodes[worker]][1][self.steps[worker]]
                self.steps[worker] += 1
            responses[worker] = copy.deepcopy(response)
            self.live[worker] = list(response["agents"])
        return responses


class ControlledModel:
    """Value equals marker + slot, making bootstrap and identity errors visible."""

    def __init__(self):
        self.actor_inputs = []
        self.critic_inputs = []

    def act(self, local):
        assert local.ndim == 2 and local.shape[1] == 27
        self.actor_inputs.append(local.clone())
        action = torch.zeros((len(local), 1), dtype=torch.float32)
        scalar = torch.zeros(len(local), dtype=torch.float32)
        return action, scalar, scalar, action

    def values(self, joint, identities):
        assert joint.shape == (len(identities), 112)
        assert identities.dtype == torch.long
        self.critic_inputs.append((joint.clone(), identities.clone()))
        return joint[:, 0] + identities


def collector(pool, **kwargs):
    config = {"training": dict(episode_seed_start=100, episode_seed_end=200,
                               gamma=0.9, gae_lambda=0.8)}
    return RolloutCollector(pool, ControlledModel(), config, **kwargs)


def test_budget_selects_only_workers_that_fit_and_counts_live_agents():
    # Four plus one fit; the middle worker's three agents must not be stepped.
    singleton = (AGENTS[2],)
    pool = FakePool([
        [(initial(marker=10), [reply(marker=11)])],
        [(initial(AGENTS[:3], marker=20), [])],
        [(initial(singleton, marker=30), [reply(singleton, singleton, marker=31)])],
    ])
    run = collector(pool)
    batch, episodes, count = run.collect(steps=3, max_agent_transitions=5)
    assert count == run.total_agent_transitions == 5
    assert run.total_environment_steps == 2
    assert episodes == []
    steps = [call for call in pool.calls if next(iter(call.values()))[0] == "step"]
    assert len(steps) == 1 and set(steps[0]) == {0, 2}
    assert batch["active"].tolist() == [True, True, True, True, False, False, True, False]
    assert batch["agent_indices"].tolist() == [0, 1, 2, 3, 0, 1, 2, 3]
    torch.testing.assert_close(batch["old_values"][batch["active"]], torch.tensor([10., 11., 12., 13., 32.]))
    assert batch["local_obs"].shape == (8, 27)
    assert batch["global_states"].shape == (8, 112)
    assert run.model.actor_inputs[0][:, 0].tolist() == [0, 1, 2, 3, 2]


def test_small_remaining_budget_does_not_step_a_full_worker():
    pool = FakePool([[(initial(), [])]])
    run = collector(pool)
    batch, episodes, count = run.collect(steps=10, max_agent_transitions=3)
    assert batch == {} and episodes == [] and count == 0
    assert run.total_environment_steps == run.total_agent_transitions == 0
    assert all(command[0] == "reset" for call in pool.calls for command in call.values())


def test_truncation_bootstraps_terminal_state_and_cannot_leak_reset_reward():
    pool = FakePool([[
        (initial(marker=2), [reply(after=(), marker=0, terminal_marker=10, reward=1, truncated=True)]),
        (initial(marker=100), [reply(after=(), marker=0, terminal_marker=20, reward=1000)]),
    ]])
    run = collector(pool)
    batch, episodes, count = run.collect(steps=2, max_agent_transitions=8)
    assert count == 8 and len(episodes) == 2
    # A cleared terminal packet state (0) or the next reset state (100) is wrong.
    torch.testing.assert_close(batch["next_values"][:4], torch.tensor([10., 11., 12., 13.]))
    torch.testing.assert_close(batch["advantages"][:4], torch.tensor([8., 7.9, 7.8, 7.7]))
    torch.testing.assert_close(batch["returns"][:4], torch.tensor([10., 10.9, 11.8, 12.7]))
    torch.testing.assert_close(batch["returns"][4:], torch.full((4,), 1000.))
    assert batch["truncated"][:4].all() and batch["terminated"][4:].all()
    assert [episode["seed"] for episode in episodes] == [100, 101]
    assert all(episode["nominal_end"] for episode in episodes)


def test_agent_departure_preserves_slots_and_reduces_budget_charge():
    survivor = (AGENTS[3],)
    pool = FakePool([[(initial(), [
        reply(after=survivor, marker=3),
        reply(before=survivor, after=survivor, marker=4, reward=2),
    ])]])
    run = collector(pool)
    batch, episodes, count = run.collect(steps=2, max_agent_transitions=5)
    assert count == 5 and episodes == []
    assert batch["active"].tolist() == [True, True, True, True, False, False, False, True]
    # Departed agents receive no recursive contribution from the survivor's step.
    torch.testing.assert_close(batch["returns"][:3], torch.ones(3))
    assert batch["advantages"][4:7].eq(0).all()
    assert batch["returns"][4:7].eq(0).all()
    assert run.model.actor_inputs[1][:, 0].tolist() == [3]


def test_rollout_cut_bootstraps_and_next_collect_continues_without_reset():
    pool = FakePool([[(initial(marker=2), [reply(marker=3), reply(marker=4)])]])
    run = collector(pool)
    first, _, count = run.collect(steps=1, max_agent_transitions=4)
    assert count == 4
    torch.testing.assert_close(first["returns"], torch.tensor([3.7, 4.6, 5.5, 6.4]))
    second, _, count = run.collect(steps=1, max_agent_transitions=4)
    assert count == 4 and run.total_agent_transitions == 8
    assert len([call for call in pool.calls if next(iter(call.values()))[0] == "reset"]) == 1
    torch.testing.assert_close(second["old_values"], torch.tensor([3., 4., 5., 6.]))


def test_expired_deadline_never_requests_a_worker():
    pool = FakePool([[(initial(), [])]])
    with pytest.raises(TimeoutError, match="wall-time"):
        collector(pool, deadline=time.monotonic() - 1).collect(1, 4)
    assert pool.calls == []


def test_training_seed_exhaustion_never_wraps_into_another_partition():
    pool = FakePool([[
        (initial(), [reply(after=())]),
        (initial(), []),
    ]])
    run = collector(pool, seed_start=199, seed_end=199)
    with pytest.raises(RuntimeError, match="seed interval exhausted"):
        run.collect(2, 8)
    resets = [command[1] for call in pool.calls for command in call.values() if command[0] == "reset"]
    assert resets == [199]
