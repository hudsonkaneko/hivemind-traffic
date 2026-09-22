"""Non-learning policies used to validate and benchmark the environment."""

from __future__ import annotations

from typing import Mapping

import numpy as np


class ScriptedPolicy:
    """Deterministic local rule set with no communication or privileged state."""

    def reset(self, seed: int) -> None:
        del seed

    def act(self, observations: Mapping[str, np.ndarray]) -> dict[str, np.ndarray]:
        actions: dict[str, np.ndarray] = {}
        for agent, observation in observations.items():
            speed, lane = float(observation[0]), float(observation[1])
            own_offset = 3 if lane < 0.5 else 9
            target_offset = 9 if lane < 0.5 else 3
            front_gap = float(observation[own_offset])
            front_present = bool(observation[own_offset + 2])
            target_front_safe = not bool(observation[target_offset + 2]) or bool(
                observation[target_offset] >= 0.08
            )
            target_rear_safe = not bool(observation[target_offset + 5]) or bool(
                observation[target_offset + 3] >= 0.08
            )
            cooldown_active = observation[15] > 0.0
            if front_present and front_gap < 0.15:
                speed_action = 0
                lane_action = (
                    (2 if lane < 0.5 else 0)
                    if target_front_safe and target_rear_safe and not cooldown_active
                    else 1
                )
            else:
                speed_action = 2 if speed < 0.75 else 1
                lane_action = 1
            actions[agent] = np.array([speed_action, lane_action], dtype=np.int64)
        return actions


class RandomPolicy:
    """Seeded random tactical policy used as a weak comparison baseline."""

    def __init__(self) -> None:
        self._rng = np.random.default_rng(0)

    def reset(self, seed: int) -> None:
        self._rng = np.random.default_rng(seed)

    def act(self, observations: Mapping[str, np.ndarray]) -> dict[str, np.ndarray]:
        return {
            agent: self._rng.integers(0, 3, size=2, dtype=np.int64)
            for agent in sorted(observations)
        }


def make_policy(name: str):
    if name == "scripted":
        return ScriptedPolicy()
    if name == "random":
        return RandomPolicy()
    raise ValueError(f"Unknown baseline policy: {name}")
