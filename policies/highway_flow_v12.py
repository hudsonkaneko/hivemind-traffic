"""121-field actor and 2928-field critic for v12 (target-speed inputs at 69..72).

Same networks as policies/highway_flow.py with four more local inputs. A 117-field
model migrates by copying every weight and inserting zero columns for the new
inputs, so the migrated actor and critic compute exactly what the parent computed,
whatever the new inputs hold, until training moves those weights.
"""
from __future__ import annotations

import time
from copy import deepcopy

import torch
from torch import nn

from policies import highway_flow as parent
from policies.mappo import (LocalDrivingActor, MappoActorCritic, MaskedPopulationCritic,
    MessageObservationEncoder, _mlp)

INSERTED = 4
INSERT_AT = parent.LOCAL_SIZE                       # 69
LOCAL_SIZE = parent.LOCAL_SIZE + INSERTED           # 73
OBSERVATION_SIZE = parent.OBSERVATION_SIZE + INSERTED  # 121
MAX_AGENTS = parent.MAX_AGENTS
STATE_SIZE = MAX_AGENTS * (OBSERVATION_SIZE + 1)    # 2928


class FlowActorV12(LocalDrivingActor):
    def __init__(self):
        nn.Module.__init__(self)
        self.observation_size = OBSERVATION_SIZE
        self.mask_start = 30
        self.mean_net = nn.Sequential(MessageObservationEncoder(LOCAL_SIZE, 8),
            _mlp(LOCAL_SIZE + 17, (128, 128), 4, .01))


class FlowModelV12(MappoActorCritic):
    def __init__(self):
        # As parent.FlowModel: build the frozen legacy ABI once, then install the v12 heads.
        super().__init__(observation_size=99, state_size=2400, num_agents=MAX_AGENTS,
            message_encoder=True, hybrid_actions=True, driving_local_size=51,
            driving_message_size=8, driving_mask_start=30, population_critic=True)
        self.observation_size = OBSERVATION_SIZE
        self.state_size = STATE_SIZE
        self.actor = FlowActorV12()
        self.critic = MaskedPopulationCritic(OBSERVATION_SIZE, MAX_AGENTS, (256, 256))


def _insert_columns(weight, start):
    """Copy `weight` with INSERTED zero input columns at `start`."""
    rows, columns = weight.shape
    result = torch.zeros(rows, columns + INSERTED, dtype=weight.dtype)
    result[:, :start] = weight[:, :start]
    result[:, start + INSERTED:] = weight[:, start:]
    return result


WIDENED = {'actor.mean_net.1.0.weight': ((128, 86), INSERT_AT),
           'critic.row_encoder.0.weight': ((64, 117), INSERT_AT),
           'critic.value_head.0.weight': ((256, 182), INSERT_AT)}  # focal row first, then pooled + count


def migrate_from_flow117(state_dict):
    """A 117-field FlowModel state dict -> a FlowModelV12 with identical outputs."""
    model = FlowModelV12()
    target = model.state_dict()
    if set(state_dict) != set(target):
        raise ValueError('V12 migration requires the exact flow117 parameter set')
    migrated = {}
    for name, value in state_dict.items():
        if name in WIDENED:
            shape, start = WIDENED[name]
            if tuple(value.shape) != shape:
                raise ValueError('Flow117 layer has an unexpected shape: ' + name)
            migrated[name] = _insert_columns(value, start)
        elif tuple(value.shape) != tuple(target[name].shape):
            raise ValueError('Flow117 parameter does not match the v12 model: ' + name)
        else:
            migrated[name] = value.clone()
    model.load_state_dict(migrated, strict=True)
    return model, dict(from_observation_size=parent.OBSERVATION_SIZE, to_observation_size=OBSERVATION_SIZE,
        from_state_size=parent.STATE_SIZE, to_state_size=STATE_SIZE,
        inserted_local_indices=list(range(INSERT_AT, INSERT_AT + INSERTED)), inserted_weights='zero',
        widened_layers=sorted(WIDENED), actor='copied', critic='copied', log_std='copied', optimizer='fresh')


def critic_update(model, trainer, batch, deadline):
    """policies.highway_flow.critic_update for the v12 state width (actor and log_std stay frozen)."""
    from experiments.run_highway_environment_v05 import actor_digest
    from policies.mappo_diagnostics import critic_fit_metrics, current_gradient_norms
    if model.state_size != STATE_SIZE:
        raise ValueError('V12 critic warmup requires the 2928-field state')
    active = batch['active'].reshape(-1).bool()
    states = batch['global_states'].reshape(-1, STATE_SIZE)[active]
    indices = batch['agent_indices'].reshape(-1)[active]
    returns = batch['returns'].reshape(-1)[active].detach()
    before, frozen = critic_fit_metrics(model, states, indices, returns), actor_digest(model)
    steps, losses, exhausted = [], [], False
    for _ in range(trainer.epochs):
        for start in range(0, len(returns), trainer.minibatch_size):
            if deadline is not None and time.monotonic() >= deadline:
                exhausted = True
                break
            values = model.values(states[start:start+trainer.minibatch_size], indices[start:start+trainer.minibatch_size])
            loss = .5*(values-returns[start:start+trainer.minibatch_size]).square().mean()
            if not torch.isfinite(loss):
                raise ValueError('Nonfinite v12 critic warmup')
            trainer.optimizer.zero_grad(set_to_none=True)
            loss.backward()
            before_clip = current_gradient_norms(model)
            nn.utils.clip_grad_norm_(model.critic.parameters(), trainer.max_grad_norm, error_if_nonfinite=True)
            steps.append(dict(before=before_clip, after=current_gradient_norms(model),
                critic_clip_factor=min(1., trainer.max_grad_norm/(before_clip['critic']+1e-6))))
            trainer.optimizer.step()
            losses.append(float(loss.detach()))
        if exhausted:
            break
    if actor_digest(model) != frozen:
        raise ValueError('V12 critic warmup changed actor or exploration')
    return dict(optimizer_steps=len(steps), value_loss=sum(losses)/max(1, len(losses)), actor_frozen=True,
        actor_sha256=frozen, budget_exhausted=exhausted, gradient_diagnostics=dict(
            phase='critic_warmup', actual_steps=steps, critic_fit_before=before,
            critic_fit_after=critic_fit_metrics(model, states, indices, returns),
            loss_definition='0.5*mean squared value error; critic-only clipping .5'))


def frozen_copy(model):
    """An evaluation-only copy whose parameters never receive gradients (the anchor)."""
    anchor = deepcopy(model).eval()
    for parameter in anchor.parameters():
        parameter.requires_grad_(False)
    return anchor
