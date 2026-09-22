"""Shared PPO actor-critic and checkpoint policy."""

from __future__ import annotations

from pathlib import Path
from typing import Mapping, Sequence

import numpy as np
import torch
from torch import nn
from torch.distributions import Categorical


def action_masks_from_observations(observations: np.ndarray) -> np.ndarray:
    """Return legal [speed, lane] choices derived from local lane observations."""

    observations = np.asarray(observations)
    masks = np.ones((len(observations), 2, 3), dtype=np.bool_)
    rightmost = observations[:, 1] < 0.5
    masks[rightmost, 1, 0] = False
    masks[~rightmost, 1, 2] = False
    return masks


class SharedActorCritic(nn.Module):
    """One local-observation policy shared by every homogeneous vehicle."""

    def __init__(self, observation_size: int = 9, hidden_sizes: Sequence[int] = (128, 128)):
        super().__init__()
        layers: list[nn.Module] = []
        input_size = observation_size
        for hidden_size in hidden_sizes:
            layers.extend((nn.Linear(input_size, hidden_size), nn.Tanh()))
            input_size = hidden_size
        self.encoder = nn.Sequential(*layers)
        self.speed_head = nn.Linear(input_size, 3)
        self.lane_head = nn.Linear(input_size, 3)
        self.value_head = nn.Linear(input_size, 1)
        self._initialize()

    def _initialize(self) -> None:
        for module in self.modules():
            if isinstance(module, nn.Linear):
                nn.init.orthogonal_(module.weight, np.sqrt(2.0))
                nn.init.zeros_(module.bias)
        nn.init.orthogonal_(self.speed_head.weight, 0.01)
        nn.init.orthogonal_(self.lane_head.weight, 0.01)
        nn.init.orthogonal_(self.value_head.weight, 1.0)

    def forward(self, observations: torch.Tensor):
        hidden = self.encoder(observations)
        return self.speed_head(hidden), self.lane_head(hidden), self.value_head(hidden).squeeze(-1)

    @staticmethod
    def _masked_logits(logits: torch.Tensor, masks: torch.Tensor | None, index: int):
        if masks is None:
            return logits
        return logits.masked_fill(~masks[:, index], torch.finfo(logits.dtype).min)

    def act(
        self,
        observations: torch.Tensor,
        *,
        deterministic: bool = False,
        action_masks: torch.Tensor | None = None,
    ):
        speed_logits, lane_logits, values = self(observations)
        speed_logits = self._masked_logits(speed_logits, action_masks, 0)
        lane_logits = self._masked_logits(lane_logits, action_masks, 1)
        distributions = (Categorical(logits=speed_logits), Categorical(logits=lane_logits))
        if deterministic:
            actions = torch.stack((speed_logits.argmax(-1), lane_logits.argmax(-1)), dim=-1)
        else:
            actions = torch.stack(tuple(distribution.sample() for distribution in distributions), dim=-1)
        log_probs = sum(
            distribution.log_prob(actions[:, index])
            for index, distribution in enumerate(distributions)
        )
        entropies = sum(distribution.entropy() for distribution in distributions)
        return actions, log_probs, entropies, values

    def evaluate_actions(
        self,
        observations: torch.Tensor,
        actions: torch.Tensor,
        action_masks: torch.Tensor | None = None,
    ):
        speed_logits, lane_logits, values = self(observations)
        speed_logits = self._masked_logits(speed_logits, action_masks, 0)
        lane_logits = self._masked_logits(lane_logits, action_masks, 1)
        distributions = (Categorical(logits=speed_logits), Categorical(logits=lane_logits))
        log_probs = sum(
            distribution.log_prob(actions[:, index])
            for index, distribution in enumerate(distributions)
        )
        entropies = sum(distribution.entropy() for distribution in distributions)
        return log_probs, entropies, values


class ObservationNormalizer:
    """Running training-only statistics that are frozen in evaluation checkpoints."""

    def __init__(self, size: int = 9, epsilon: float = 1e-4):
        self.mean = np.zeros(size, dtype=np.float64)
        self.variance = np.ones(size, dtype=np.float64)
        self.count = float(epsilon)

    def update(self, observations: np.ndarray) -> None:
        observations = np.asarray(observations, dtype=np.float64)
        if not len(observations):
            return
        batch_mean = observations.mean(axis=0)
        batch_variance = observations.var(axis=0)
        batch_count = len(observations)
        delta = batch_mean - self.mean
        total = self.count + batch_count
        new_mean = self.mean + delta * batch_count / total
        m_a = self.variance * self.count
        m_b = batch_variance * batch_count
        m2 = m_a + m_b + delta**2 * self.count * batch_count / total
        self.mean = new_mean
        self.variance = m2 / total
        self.count = total

    def normalize(self, observations: np.ndarray) -> np.ndarray:
        normalized = (np.asarray(observations, dtype=np.float32) - self.mean) / np.sqrt(
            self.variance + 1e-8
        )
        return np.clip(normalized, -10.0, 10.0).astype(np.float32)

    def state_dict(self) -> dict:
        return {"mean": self.mean, "variance": self.variance, "count": self.count}

    def load_state_dict(self, state: dict) -> None:
        self.mean = np.asarray(state["mean"], dtype=np.float64)
        self.variance = np.asarray(state["variance"], dtype=np.float64)
        self.count = float(state["count"])


class CheckpointPolicy:
    """Deterministic shared PPO policy loaded for evaluation or GUI replay."""

    def __init__(self, checkpoint_path: str | Path):
        checkpoint = torch.load(Path(checkpoint_path), map_location="cpu", weights_only=False)
        hidden_sizes = checkpoint["config"]["model"]["hidden_sizes"]
        self.model = SharedActorCritic(hidden_sizes=hidden_sizes)
        self.model.load_state_dict(checkpoint["model_state"])
        self.model.eval()
        self.normalizer = ObservationNormalizer()
        self.normalizer.load_state_dict(checkpoint["normalizer_state"])

    def reset(self, seed: int) -> None:
        torch.manual_seed(seed)

    def act(self, observations: Mapping[str, np.ndarray]) -> dict[str, np.ndarray]:
        agents = sorted(observations)
        batch = np.stack([observations[agent] for agent in agents])
        tensor = torch.from_numpy(self.normalizer.normalize(batch))
        masks = torch.from_numpy(action_masks_from_observations(batch))
        with torch.no_grad():
            actions, _, _, _ = self.model.act(
                tensor, deterministic=True, action_masks=masks
            )
        return {
            agent: action.cpu().numpy().astype(np.int64)
            for agent, action in zip(agents, actions)
        }
