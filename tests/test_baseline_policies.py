import numpy as np

from policies.baselines import RandomPolicy, ScriptedPolicy


def test_scripted_policy_accelerates_on_clear_road():
    observation = np.array([0.4, 0.0, 0.1, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0], dtype=np.float32)
    action = ScriptedPolicy().act({"agent_0": observation})["agent_0"]
    assert action.tolist() == [2, 1]


def test_scripted_policy_changes_lane_around_close_leader():
    observation = np.array([0.7, 0.0, 0.4, 0.1, -0.2, 1.0, 1.0, 0.0, 0.0], dtype=np.float32)
    action = ScriptedPolicy().act({"agent_0": observation})["agent_0"]
    assert action.tolist() == [0, 2]


def test_random_policy_replays_from_seed():
    observations = {
        "agent_0": np.zeros(9, dtype=np.float32),
        "agent_1": np.zeros(9, dtype=np.float32),
    }
    policy = RandomPolicy()
    policy.reset(42)
    first = {agent: action.tolist() for agent, action in policy.act(observations).items()}
    policy.reset(42)
    second = {agent: action.tolist() for agent, action in policy.act(observations).items()}
    assert first == second
