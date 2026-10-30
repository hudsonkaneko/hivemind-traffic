"""CTDE MAPPO components for the SUMO acceleration task.

The shared actor accepts local, analytically scaled observations only. The
central critic receives the joint state and a stable agent index. Neither a
running normalizer nor critic state is included in the exported actor.

Actions follow a tanh-squashed diagonal Gaussian. Rollouts must retain the
pre-tanh sample: reconstructing it from a saturated float32 action would change
the PPO likelihood ratio. Entropy uses fixed 20-point Gauss-Hermite quadrature
of the squashing Jacobian (a deterministic approximation, not base-Gaussian
entropy).
"""

from __future__ import annotations

import copy
import math
import time
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np
import torch
from torch import nn
from torch.nn import functional as F


def _mlp(input_size: int, hidden_sizes: Sequence[int], output_size: int, gain: float):
    if input_size <= 0 or output_size <= 0 or any(size <= 0 for size in hidden_sizes):
        raise ValueError("Network dimensions must be positive")
    layers: list[nn.Module] = []
    width = input_size
    for hidden in hidden_sizes:
        layer = nn.Linear(width, hidden)
        nn.init.orthogonal_(layer.weight, math.sqrt(2.0))
        nn.init.zeros_(layer.bias)
        layers.extend((layer, nn.Tanh()))
        width = hidden
    head = nn.Linear(width, output_size)
    nn.init.orthogonal_(head.weight, gain)
    nn.init.zeros_(head.bias)
    layers.append(head)
    return nn.Sequential(*layers)


class MessageObservationEncoder(nn.Module):
    """Permutation-invariant masked encoding of six delivered-message slots."""
    def __init__(self, local_size: int = 27, message_size: int = 7):
        super().__init__()
        self.local_size = local_size
        self.message_size = message_size
        self.message_net = _mlp(message_size - 1, (16,), 16, 1.0)

    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        slots = observations[..., self.local_size:].unflatten(-1, (6, self.message_size))
        mask = slots[..., -1:]
        encoded = self.message_net(slots[..., :-1] * mask) * mask
        pooled = encoded.sum(-2) / mask.sum(-2).clamp_min(1.0)
        # Count distinguishes absent messages from messages whose embedding cancels.
        return torch.cat((observations[..., :self.local_size], pooled, mask.sum(-2) / 6.0), dim=-1)


class LocalAccelerationActor(nn.Module):
    """Deployment interface: local observations -> normalized acceleration."""

    def __init__(self, observation_size: int, hidden_sizes: Sequence[int], message_encoder: bool = False):
        super().__init__()
        self.observation_size = observation_size
        if message_encoder:
            if observation_size != 69:
                raise ValueError("Message encoder requires 27 local plus six 7-feature slots")
            self.mean_net = nn.Sequential(MessageObservationEncoder(), _mlp(44, hidden_sizes, 1, 0.01))
        else:
            self.mean_net = _mlp(observation_size, hidden_sizes, 1, 0.01)

    def forward(self, local_obs: torch.Tensor) -> torch.Tensor:
        if local_obs.shape[-1] != self.observation_size:
            raise ValueError("Actor expects local observations of the configured width")
        return torch.tanh(self.mean_net(local_obs))


class LocalDrivingActor(nn.Module):
    """v04 deployment: local state/messages -> acceleration plus lane delta.

    The mask is part of the stored observation. Category order is right, stay,
    left, mapped to -1, 0, +1. An empty malformed mask conservatively permits
    staying only; it never invents permission to cross a lane boundary.
    """
    def __init__(self, observation_size: int, hidden_sizes: Sequence[int]):
        super().__init__()
        if observation_size != 81:
            raise ValueError('Driving actor requires 33 local plus six 8-feature messages')
        self.observation_size = observation_size
        self.mean_net = nn.Sequential(MessageObservationEncoder(33, 8), _mlp(50, hidden_sizes, 4, .01))

    def lane_logits(self, local_obs: torch.Tensor, raw: torch.Tensor) -> torch.Tensor:
        mask = local_obs[..., 30:33] > .5
        fallback = torch.zeros_like(mask)
        fallback[..., 1] = True
        mask = torch.where(mask.any(dim=-1, keepdim=True), mask, fallback)
        return raw[..., 1:4].masked_fill(~mask, -1e9)

    def forward(self, local_obs: torch.Tensor) -> torch.Tensor:
        if local_obs.shape[-1] != self.observation_size:
            raise ValueError('Actor expects local observations of the configured width')
        raw = self.mean_net(local_obs)
        lane = self.lane_logits(local_obs, raw).argmax(-1, keepdim=True).to(raw.dtype) - 1.
        return torch.cat((raw[..., :1].tanh(), lane), dim=-1)


def _log_tanh_jacobian(pre_tanh: torch.Tensor) -> torch.Tensor:
    # log(1-tanh(z)^2), without cancellation when |z| is large.
    return 2.0 * (math.log(2.0) - pre_tanh - F.softplus(-2.0 * pre_tanh))


class MappoActorCritic(nn.Module):
    """Shared local Gaussian actor and agent-conditioned centralized critic."""

    def __init__(
        self,
        observation_size: int = 27,
        state_size: int = 112,
        num_agents: int = 4,
        actor_hidden_sizes: Sequence[int] = (128, 128),
        critic_hidden_sizes: Sequence[int] = (256, 256),
        message_encoder: bool = False,
        hybrid_actions: bool = False,
    ):
        super().__init__()
        if num_agents <= 0 or state_size <= 0:
            raise ValueError("Agent count and centralized-state width must be positive")
        self.observation_size = observation_size
        self.state_size = state_size
        self.num_agents = num_agents
        self.hybrid_actions = bool(hybrid_actions)
        self.action_size = 2 if self.hybrid_actions else 1
        if self.hybrid_actions and (not message_encoder or state_size != 328 or num_agents != 4):
            raise ValueError('Driving actor requires the v04 message/state contract')
        self.actor = (LocalDrivingActor(observation_size, actor_hidden_sizes) if self.hybrid_actions
                      else LocalAccelerationActor(observation_size, actor_hidden_sizes, message_encoder))
        self.log_std = nn.Parameter(torch.zeros(1))
        self.critic = _mlp(state_size + num_agents, critic_hidden_sizes, 1, 1.0)
        nodes, weights = np.polynomial.hermite.hermgauss(20)
        self.register_buffer("entropy_nodes", torch.tensor(nodes, dtype=torch.float32))
        self.register_buffer("entropy_weights", torch.tensor(weights / math.sqrt(math.pi), dtype=torch.float32))

    def _distribution(self, local_obs: torch.Tensor):
        if local_obs.shape[-1] != self.observation_size:
            raise ValueError("Actor only accepts local observations")
        mean = self.actor.mean_net(local_obs)
        if self.hybrid_actions:
            mean = mean[..., :1]
        log_std = self.log_std.clamp(-5.0, 2.0).expand_as(mean)
        return torch.distributions.Normal(mean, log_std.exp())

    def _entropy(self, distribution) -> torch.Tensor:
        points = distribution.loc.unsqueeze(-1) + (
            math.sqrt(2.0) * distribution.scale.unsqueeze(-1) * self.entropy_nodes
        )
        expected_jacobian = (_log_tanh_jacobian(points) * self.entropy_weights).sum(-1)
        return (distribution.entropy() + expected_jacobian).sum(-1)

    def act(self, local_obs: torch.Tensor, deterministic: bool = False):
        """Return action[...,1], log-prob, entropy, and pre-tanh action[...,1]."""
        distribution = self._distribution(local_obs)
        pre_tanh = distribution.loc if deterministic else distribution.sample()
        actions = pre_tanh.tanh()
        log_prob = (distribution.log_prob(pre_tanh) - _log_tanh_jacobian(pre_tanh)).sum(-1)
        entropy = self._entropy(distribution)
        if self.hybrid_actions:
            categorical = torch.distributions.Categorical(logits=self.actor.lane_logits(local_obs, self.actor.mean_net(local_obs)))
            lane = categorical.logits.argmax(-1) if deterministic else categorical.sample()
            log_prob = log_prob + categorical.log_prob(lane)
            entropy = entropy + categorical.entropy()
            delta = (lane - 1).to(actions.dtype).unsqueeze(-1)
            actions = torch.cat((actions, delta), -1)
            # Second stored latent is the exact categorical delta, not a Gaussian.
            pre_tanh = torch.cat((pre_tanh, delta), -1)
        return actions, log_prob, entropy, pre_tanh

    def evaluate_actions(
        self,
        local_obs: torch.Tensor,
        actions: torch.Tensor,
        pre_tanh_actions: torch.Tensor | None = None,
    ):
        """Evaluate the stored latent sample; optional inversion rejects saturation."""
        distribution = self._distribution(local_obs)
        expected = distribution.loc.shape[:-1] + (self.action_size,)
        if actions.shape != expected or not torch.isfinite(actions).all():
            raise ValueError('Actions must be finite with the configured action width')
        lane_log_prob = lane_entropy = 0.
        if self.hybrid_actions:
            delta = actions[..., 1]
            if ((delta < -1) | (delta > 1) | (delta != delta.round())).any():
                raise ValueError('Lane action must be exactly -1, 0 or 1')
            categorical = torch.distributions.Categorical(logits=self.actor.lane_logits(local_obs, self.actor.mean_net(local_obs)))
            lane = (delta + 1).long()
            if (categorical.logits.gather(-1, lane.unsqueeze(-1)) < -1e8).any():
                raise ValueError('Stored lane action violates its observation mask')
            lane_log_prob, lane_entropy = categorical.log_prob(lane), categorical.entropy()
            if pre_tanh_actions is not None:
                if pre_tanh_actions.shape != actions.shape or not torch.equal(pre_tanh_actions[..., 1], delta):
                    raise ValueError('Stored categorical samples disagree')
                pre_tanh_actions = pre_tanh_actions[..., :1]
            actions = actions[..., :1]
        if pre_tanh_actions is None:
            if (actions.abs() >= 1.0).any():
                raise ValueError("Saturated actions require their stored pre-tanh sample")
            pre_tanh_actions = torch.atanh(actions)
        elif (
            pre_tanh_actions.shape != actions.shape
            or not torch.isfinite(pre_tanh_actions).all()
            or not torch.allclose(pre_tanh_actions.tanh(), actions, rtol=1e-5, atol=1e-6)
        ):
            raise ValueError("Stored actions and pre-tanh samples disagree")
        log_prob = (
            distribution.log_prob(pre_tanh_actions) - _log_tanh_jacobian(pre_tanh_actions)
        ).sum(-1)
        return log_prob + lane_log_prob, self._entropy(distribution) + lane_entropy

    def values(self, global_states: torch.Tensor, agent_indices: torch.Tensor) -> torch.Tensor:
        """Joint state [...,112] plus one-hot ID [...,4] -> per-agent value."""
        if global_states.shape[-1] != self.state_size or agent_indices.shape != global_states.shape[:-1]:
            raise ValueError("Critic state width or agent-index shape is incorrect")
        if agent_indices.dtype != torch.long:
            raise ValueError("Agent indices must be torch.long")
        if ((agent_indices < 0) | (agent_indices >= self.num_agents)).any():
            raise ValueError("Agent index is outside the configured stable slots")
        identity = F.one_hot(agent_indices, num_classes=self.num_agents).to(global_states.dtype)
        return self.critic(torch.cat((global_states, identity), dim=-1)).squeeze(-1)


def compute_gae(
    rewards: torch.Tensor,
    values: torch.Tensor,
    next_values: torch.Tensor,
    terminated: torch.Tensor,
    truncated: torch.Tensor,
    active: torch.Tensor,
    gamma: float = 0.99,
    gae_lambda: float = 0.95,
):
    """GAE for [time,...] tensors, including [time,environment,agent].

    ``active[t]`` marks an agent present before the action. True terminations
    bootstrap zero; time-limit truncations bootstrap the genuine final-state
    ``next_values[t]`` supplied by the collector, NEVER a reset observation.
    Both end flags stop the recursive trace. Inactive slots receive zero
    advantages/returns and cannot pass a trace between different lives.
    """
    items = (values, next_values, terminated, truncated, active)
    if rewards.ndim < 1 or any(item.shape != rewards.shape for item in items):
        raise ValueError("All GAE tensors must share shape [time,...]")
    if not 0 <= gamma <= 1 or not 0 <= gae_lambda <= 1:
        raise ValueError("Discount and GAE lambda must be in [0,1]")
    active = active.bool()
    terminated = terminated.bool()
    truncated = truncated.bool()
    if any(not torch.isfinite(item[active]).all() for item in (rewards, values)):
        raise ValueError("Active rewards and values must be finite")
    if not torch.isfinite(next_values[active & ~terminated]).all():
        raise ValueError("Bootstrapped next-state values must be finite")
    advantages = torch.zeros_like(rewards)
    carry = torch.zeros_like(rewards[0]) if rewards.shape[0] else torch.zeros(rewards.shape[1:], device=rewards.device)
    for step in reversed(range(rewards.shape[0])):
        bootstrap = torch.where(terminated[step], 0.0, next_values[step])
        delta = rewards[step] + gamma * bootstrap - values[step]
        continuation = ~(terminated[step] | truncated[step])
        carry = torch.where(active[step], delta + gamma * gae_lambda * continuation * carry, 0.0)
        advantages[step] = carry
    returns = torch.where(active, advantages + values, 0.0)
    return advantages, returns


class MappoTrainer:
    """Clipped PPO updates with local actor batches and centralized values.

    Every tensor may have arbitrary shared leading rollout dimensions. The
    inactive rows are removed BEFORE numerical checks, advantage normalization,
    minibatching and losses. Values use 0.5 * maximum of clipped/unclipped MSE.
    The KL gate runs before each proposed optimizer step, including the first.
    """

    def __init__(
        self,
        model: MappoActorCritic,
        *,
        actor_lr: float = 3e-4,
        critic_lr: float = 3e-4,
        clip: float = 0.2,
        value_clip: float = 0.2,
        entropy_coef: float = 0.01,
        value_coef: float = 0.5,
        max_grad_norm: float = 0.5,
        epochs: int = 4,
        minibatch_size: int = 256,
        target_kl: float = 0.02,
        imitation_coef: float = 0.0,
        imitation_lane_change_weight: float = 10.0,
    ):
        if any(not math.isfinite(v) or v <= 0 for v in (actor_lr, critic_lr, clip, value_clip, max_grad_norm, target_kl)):
            raise ValueError("Optimizer rates, clipping bounds and KL limit must be positive")
        if epochs <= 0 or minibatch_size <= 0 or entropy_coef < 0 or value_coef < 0:
            raise ValueError("Invalid PPO update parameters")
        if (not math.isfinite(imitation_coef) or imitation_coef < 0
                or not math.isfinite(imitation_lane_change_weight) or imitation_lane_change_weight < 1):
            raise ValueError('Invalid imitation loss settings')
        if imitation_coef and not model.hybrid_actions:
            raise ValueError('Imitation assistance requires the hybrid driving actor')
        self.model = model
        self.clip = clip
        self.value_clip = value_clip
        self.entropy_coef = entropy_coef
        self.value_coef = value_coef
        self.max_grad_norm = max_grad_norm
        self.epochs = epochs
        self.minibatch_size = minibatch_size
        self.target_kl = target_kl
        self.imitation_coef = imitation_coef
        self.imitation_lane_change_weight = imitation_lane_change_weight
        self.optimizer = torch.optim.Adam([
            {"params": list(model.actor.parameters()) + [model.log_std], "lr": actor_lr},
            {"params": model.critic.parameters(), "lr": critic_lr},
        ], eps=1e-5)

    def _imitation_loss(self, observations: torch.Tensor, teacher_actions: torch.Tensor):
        """Training-only supervision; never changes deployment action selection.

        Labels must use the observation's legal lane mask. The continuous mean
        is fit in the normalized action space, so saturated teacher actions do
        not require an undefined inverse tanh. Lane-change rows receive the
        declared relative weight to account for their rarity in demonstrations.
        """
        if not self.model.hybrid_actions:
            raise ValueError('Imitation assistance requires the hybrid driving actor')
        if (observations.ndim != 2 or observations.shape[-1] != self.model.observation_size
                or teacher_actions.shape != (observations.shape[0], self.model.action_size)
                or not len(observations)):
            raise ValueError('Imitation observations/actions have incompatible shapes')
        if not torch.isfinite(observations).all() or not torch.isfinite(teacher_actions).all():
            raise ValueError('Imitation observations/actions must be finite')
        delta = teacher_actions[:, 1]
        if (teacher_actions[:, 0].abs() > 1).any() or (
                (delta < -1) | (delta > 1) | (delta != delta.round())).any():
            raise ValueError('Imitation action is outside the hybrid action contract')
        raw = self.model.actor.mean_net(observations)
        logits = self.model.actor.lane_logits(observations, raw)
        labels = (delta + 1).long()
        if (logits.gather(-1, labels.unsqueeze(-1)) < -1e8).any():
            raise ValueError('Imitation lane label violates its observation mask')
        acceleration_loss = F.mse_loss(raw[:, :1].tanh(), teacher_actions[:, :1].detach())
        weights = torch.where(delta == 0, torch.ones_like(delta),
                              torch.full_like(delta, self.imitation_lane_change_weight))
        lane_loss = (F.cross_entropy(logits, labels, reduction='none') * weights).mean()
        return acceleration_loss + lane_loss, acceleration_loss, lane_loss

    def behavior_clone(self, observations: torch.Tensor, teacher_actions: torch.Tensor, *,
                       epochs: int, minibatch_size: int = 512, learning_rate: float = 3e-4,
                       initial_log_std: float = -2., deadline: float | None = None,
                       seed: int = 0) -> dict:
        """Fit only actor parameters on training demonstrations before MAPPO.

        Uses a separate optimizer, leaving critic and PPO optimizer state
        untouched. The caller owns transition/seed accounting and persists this
        stage's dataset, metrics and resulting full checkpoint before PPO.
        """
        if (type(epochs) is not int or epochs <= 0 or type(minibatch_size) is not int
                or minibatch_size <= 0 or not math.isfinite(learning_rate) or learning_rate <= 0
                or not math.isfinite(initial_log_std) or not -5 <= initial_log_std <= 2
                or type(seed) is not int or seed < 0):
            raise ValueError('Invalid behavior-cloning settings')
        observations, teacher_actions = observations.detach(), teacher_actions.detach()
        with torch.no_grad():
            initial_loss = float(self._imitation_loss(observations, teacher_actions)[0])
        if not math.isfinite(initial_loss):
            raise ValueError('Nonfinite initial behavior-cloning loss')
        parameters = list(self.model.actor.parameters())
        optimizer = torch.optim.Adam(parameters, lr=learning_rate, eps=1e-5)
        generator = torch.Generator(device=observations.device).manual_seed(seed)
        totals = np.zeros(3, dtype=np.float64)
        steps = samples_seen = 0
        for _epoch in range(epochs):
            permutation = torch.randperm(len(observations), generator=generator, device=observations.device)
            for start in range(0, len(observations), minibatch_size):
                if deadline is not None and time.monotonic() >= deadline:
                    raise TimeoutError('Experiment wall-time budget reached during behavior cloning')
                indices = permutation[start:start + minibatch_size]
                losses = self._imitation_loss(observations[indices], teacher_actions[indices])
                if not all(torch.isfinite(value) for value in losses):
                    raise ValueError('Nonfinite behavior-cloning loss')
                optimizer.zero_grad(set_to_none=True)
                losses[0].backward()
                nn.utils.clip_grad_norm_(parameters, self.max_grad_norm, error_if_nonfinite=True)
                optimizer.step()
                totals += np.asarray([float(value.detach()) for value in losses]) * len(indices)
                samples_seen += len(indices)
                steps += 1
        with torch.no_grad():
            final_loss = float(self._imitation_loss(observations, teacher_actions)[0])
            if not math.isfinite(final_loss):
                raise ValueError('Nonfinite final behavior-cloning loss')
            self.model.log_std.fill_(initial_log_std)
        means = totals / samples_seen
        return dict(optimizer_steps=steps, examples=len(observations), samples_seen=samples_seen,
                    epochs=epochs, initial_loss=initial_loss, final_loss=final_loss,
                    imitation_loss=float(means[0]), imitation_acceleration_loss=float(means[1]),
                    imitation_lane_loss=float(means[2]), initial_log_std=initial_log_std, seed=seed)

    def update(self, batch: Mapping[str, torch.Tensor]) -> dict:
        widths = {
            "local_obs": self.model.observation_size,
            "global_states": self.model.state_size,
            "actions": self.model.action_size,
            "pre_tanh_actions": self.model.action_size,
        }
        if self.imitation_coef:
            widths['teacher_actions'] = self.model.action_size
        scalar_names = ("agent_indices", "old_log_probs", "old_values", "advantages", "returns")
        leading = batch["active"].shape
        active = batch["active"].bool().reshape(-1)
        samples = int(active.sum().item())
        if samples == 0:
            raise ValueError("MAPPO update needs at least one active transition")
        data = {}
        for name in (*widths, *scalar_names):
            if name not in batch:
                raise ValueError(f'Missing required batch field {name}')
            expected = leading + ((widths[name],) if name in widths else ())
            if batch[name].shape != expected:
                raise ValueError(f"Batch field {name} expected shape {expected}")
            tail = (widths[name],) if name in widths else ()
            data[name] = batch[name].detach().reshape((-1,) + tail)[active]
            if not torch.isfinite(data[name]).all():
                raise ValueError(f"Active batch field {name} is not finite")
        advantages = data["advantages"]
        data["advantages"] = (advantages - advantages.mean()) / (advantages.std(unbiased=False) + 1e-8)
        stats: list[dict] = []
        early_stopped = False
        last_kl = 0.0
        completed_epochs = 0
        for _epoch in range(self.epochs):
            permutation = torch.randperm(samples, device=active.device)
            for start in range(0, samples, self.minibatch_size):
                indices = permutation[start:start + self.minibatch_size]
                part = {name: value[indices] for name, value in data.items()}
                log_probs, entropy = self.model.evaluate_actions(part["local_obs"], part["actions"], part["pre_tanh_actions"])
                log_ratio = log_probs - part["old_log_probs"]
                ratio = log_ratio.exp()
                kl = ((ratio - 1.0) - log_ratio).mean()
                if not torch.isfinite(kl):
                    raise ValueError("Nonfinite PPO likelihood ratio")
                last_kl = float(kl.detach())
                if last_kl > self.target_kl:
                    early_stopped = True
                    break
                actor_loss = -torch.minimum(
                    ratio * part["advantages"],
                    ratio.clamp(1.0 - self.clip, 1.0 + self.clip) * part["advantages"],
                ).mean()
                values = self.model.values(part["global_states"], part["agent_indices"])
                clipped_values = part["old_values"] + (values - part["old_values"]).clamp(-self.value_clip, self.value_clip)
                value_loss = 0.5 * torch.maximum(
                    (values - part["returns"]).square(),
                    (clipped_values - part["returns"]).square(),
                ).mean()
                entropy_mean = entropy.mean()
                loss = actor_loss + self.value_coef * value_loss - self.entropy_coef * entropy_mean
                imitation_stats = {}
                if self.imitation_coef:
                    imitation_loss, acceleration_imitation, lane_imitation = self._imitation_loss(
                        part['local_obs'], part['teacher_actions'])
                    loss = loss + self.imitation_coef * imitation_loss
                    imitation_stats = dict(imitation_loss=float(imitation_loss.detach()),
                        imitation_acceleration_loss=float(acceleration_imitation.detach()),
                        imitation_lane_loss=float(lane_imitation.detach()))
                if not torch.isfinite(loss):
                    raise ValueError("Nonfinite MAPPO loss")
                self.optimizer.zero_grad(set_to_none=True)
                loss.backward()
                grad_norm = nn.utils.clip_grad_norm_(self.model.parameters(), self.max_grad_norm, error_if_nonfinite=True)
                self.optimizer.step()
                stats.append({
                    "actor_loss": float(actor_loss.detach()),
                    "value_loss": float(value_loss.detach()),
                    "entropy": float(entropy_mean.detach()),
                    "approx_kl": last_kl,
                    "clip_fraction": float(((ratio - 1.0).abs() > self.clip).float().mean().detach()),
                    "grad_norm_before_clip": float(grad_norm),
                    **imitation_stats,
                })
            if early_stopped:
                break
            completed_epochs += 1
        result = {name: sum(item[name] for item in stats) / len(stats) for name in stats[0]} if stats else {}
        return dict(result, optimizer_steps=len(stats), completed_epochs=completed_epochs,
                    early_stopped=early_stopped, last_kl=last_kl, active_samples=samples)


def export_actor(model: MappoActorCritic, path: str | Path) -> Path:
    """Save deterministic actor only, with no critic, agent IDs or training state."""
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    actor = copy.deepcopy(model.actor).cpu().eval()
    torch.jit.freeze(torch.jit.script(actor)).save(str(destination))
    return destination
