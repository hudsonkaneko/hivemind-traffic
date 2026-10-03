from pathlib import Path

import numpy as np
import torch

from policies.ppo import (
    CheckpointPolicy,
    ObservationNormalizer,
    SharedActorCritic,
    action_masks_from_observations,
)


def test_shared_actor_produces_two_categorical_actions():
    model = SharedActorCritic()
    observations = torch.zeros((2, 16), dtype=torch.float32)
    actions, log_probs, entropies, values = model.act(observations)
    assert actions.shape == (2, 2)
    assert log_probs.shape == (2,)
    assert entropies.shape == (2,)
    assert values.shape == (2,)
    assert torch.all((actions >= 0) & (actions <= 2))


def test_observation_normalizer_round_trip():
    normalizer = ObservationNormalizer()
    samples = np.array([[0.0] * 16, [1.0] * 16], dtype=np.float32)
    normalizer.update(samples)
    restored = ObservationNormalizer()
    restored.load_state_dict(normalizer.state_dict())
    np.testing.assert_allclose(restored.normalize(samples), normalizer.normalize(samples))


def test_lane_action_masks_respect_road_boundaries():
    observations = np.zeros((2, 16), dtype=np.float32)
    observations[1, 1] = 1.0
    masks = action_masks_from_observations(observations)
    assert masks[0, 1].tolist() == [False, True, True]
    assert masks[1, 1].tolist() == [True, True, False]


def test_lane_action_masks_block_cooldown_and_occupied_target_gap():
    observations = np.zeros((2, 16), dtype=np.float32)
    observations[0, 15] = 0.5
    observations[1, 1] = 0.0
    observations[1, 9] = 0.05
    observations[1, 11] = 1.0
    masks = action_masks_from_observations(observations)
    assert masks[0, 1].tolist() == [False, True, False]
    assert masks[1, 1].tolist() == [False, True, False]


def test_checkpoint_policy_is_deterministic(tmp_path: Path):
    model = SharedActorCritic(hidden_sizes=(16, 16))
    normalizer = ObservationNormalizer()
    checkpoint = tmp_path / "policy.pt"
    torch.save(
        {
            "model_state": model.state_dict(),
            "normalizer_state": normalizer.state_dict(),
            "config": {"model": {"observation_size": 16, "hidden_sizes": [16, 16]}},
        },
        checkpoint,
    )
    policy = CheckpointPolicy(checkpoint)
    observations = {"agent_0": np.zeros(16, dtype=np.float32)}
    first = policy.act(observations)["agent_0"]
    second = policy.act(observations)["agent_0"]
    np.testing.assert_array_equal(first, second)
