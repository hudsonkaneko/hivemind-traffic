"""Read-only MAPPO gradient/value measurements, with no optimizer or sampling.

Component gradients are weighted derivatives, not predicted Adam updates. These
helpers neither fill parameter ``.grad`` buffers nor alter caller-owned targets.
"""
from __future__ import annotations

import itertools
import math
from typing import Mapping

import torch


def _parameter_groups(model):
    return list(model.actor.parameters()) + [model.log_std], list(model.critic.parameters())


def current_gradient_norms(model) -> dict:
    """Measure actual existing .grad buffers for actor (including std) and critic."""
    actor, critic = _parameter_groups(model)

    def norm(parameters):
        values = [p.grad.detach().norm() for p in parameters if p.grad is not None]
        value = float(torch.stack(values).norm()) if values else 0.0
        if not math.isfinite(value):
            raise ValueError('Nonfinite diagnostic gradient norm')
        return value

    actor_norm, critic_norm = norm(actor), norm(critic)
    return dict(actor=actor_norm, critic=critic_norm, joint=math.hypot(actor_norm, critic_norm))


def weighted_gradient_report(model, weighted_losses: Mapping[str, torch.Tensor]) -> dict:
    """Differentiate already-weighted scalar losses without touching .grad/RNG.

    Graphs remain available for the caller's ordinary backward pass. Supply
    precisely the weights used in that pass, including negative entropy and
    the value coefficient. Zero/unused/frozen derivatives have zero norms and
    undefined cosines are JSON-safe None. This measures one batch only.
    """
    actor, critic = _parameter_groups(model)
    parameters = actor + critic
    differentiable = [p for p in parameters if p.requires_grad]
    actor_width = sum(p.numel() for p in actor)
    vectors, components = {}, {}
    for name, loss in weighted_losses.items():
        if loss.numel() != 1 or not torch.isfinite(loss).all():
            raise ValueError('Diagnostic component losses must be finite scalars')
        gradients = (torch.autograd.grad(loss, differentiable, allow_unused=True, retain_graph=True)
                     if loss.requires_grad and differentiable else [None] * len(differentiable))
        by_id = {id(p): g for p, g in zip(differentiable, gradients)}
        vector = torch.cat([torch.zeros_like(p).reshape(-1) if by_id.get(id(p)) is None
                            else by_id[id(p)].detach().reshape(-1) for p in parameters])
        if not torch.isfinite(vector).all():
            raise ValueError('Nonfinite diagnostic component gradient')
        vectors[name] = vector
        components[name] = dict(weighted_loss=float(loss.detach()),
            actor=float(vector[:actor_width].norm()), critic=float(vector[actor_width:].norm()),
            joint=float(vector.norm()))
    cosines = {}
    for left, right in itertools.combinations(vectors, 2):
        a, b = vectors[left][:actor_width], vectors[right][:actor_width]
        denominator = float(a.norm() * b.norm())
        value = None if denominator <= 1e-15 else float(torch.dot(a, b)) / denominator
        cosines[left + '__' + right] = None if value is None else max(-1., min(1., value))
    return dict(components=components, actor_cosines=cosines,
                scope='Weighted derivatives on this batch only; not optimizer parameter steps')


@torch.no_grad()
def critic_fit_metrics(model, global_states: torch.Tensor, agent_indices: torch.Tensor,
                       returns: torch.Tensor, active: torch.Tensor | None = None,
                       *, chunk_size: int = 1024) -> dict:
    """Score the current critic against detached, fixed caller rollout returns.

    Active masks are applied before finite checks. Calling before and after an
    update with identical tensors compares that update on the same targets.
    Zero target variance makes explained variance undefined (None), not NaN.
    """
    if type(chunk_size) is not int or chunk_size <= 0:
        raise ValueError('Diagnostic critic chunk size must be positive')
    if (global_states.shape != returns.shape + (model.state_size,)
            or agent_indices.shape != returns.shape
            or (active is not None and active.shape != returns.shape)):
        raise ValueError('Diagnostic critic tensors have incompatible shapes')
    mask = torch.ones_like(returns, dtype=torch.bool) if active is None else active.bool()
    target = returns.detach()[mask]
    states, indices = global_states.detach()[mask], agent_indices.detach()[mask]
    if not torch.isfinite(target).all() or not torch.isfinite(states).all():
        raise ValueError('Active critic diagnostic data must be finite')
    if not len(target):
        return dict(samples=0, mse=None, explained_variance=None, target_variance=None)
    prediction = torch.cat([model.values(states[start:start + chunk_size], indices[start:start + chunk_size])
                            for start in range(0, len(target), chunk_size)])
    if not torch.isfinite(prediction).all():
        raise ValueError('Nonfinite diagnostic critic predictions')
    error = target.double() - prediction.double()
    variance = float(target.double().var(unbiased=False))
    explained = None if variance <= 1e-12 else 1. - float(error.var(unbiased=False)) / variance
    return dict(samples=len(target), mse=float(error.square().mean()), explained_variance=explained,
                target_variance=variance)
