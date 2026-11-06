"""Versioned 117-field adapters; legacy MAPPO implementation stays unchanged.

The Gaussian/categorical likelihood, PPO/GAE, optimizer and clipping machinery is
inherited. Only the actor ABI, critic width and qualified supervision selection
are new. No optimizer step is performed by construction or diagnostics.
"""
from __future__ import annotations

from copy import deepcopy
import torch
from torch import nn

from policies.mappo import (LocalDrivingActor, MappoActorCritic, MappoTrainer,
    MaskedPopulationCritic, MessageObservationEncoder, _mlp)

OBSERVATION_SIZE = 117
LOCAL_SIZE = 69
STATE_SIZE = 2832
MAX_AGENTS = 24


class FlowActor(LocalDrivingActor):
    def __init__(self):
        nn.Module.__init__(self)
        self.observation_size = OBSERVATION_SIZE
        self.mask_start = 30
        self.mean_net = nn.Sequential(MessageObservationEncoder(LOCAL_SIZE, 8),
            _mlp(LOCAL_SIZE + 17, (128, 128), 4, .01))


class FlowModel(MappoActorCritic):
    def __init__(self):
        # The inherited implementation has an intentionally frozen ABI guard.
        # Construct its buffers once, then install separately versioned heads.
        super().__init__(observation_size=99, state_size=2400, num_agents=24,
            message_encoder=True, hybrid_actions=True, driving_local_size=51,
            driving_message_size=8, driving_mask_start=30, population_critic=True)
        self.observation_size = OBSERVATION_SIZE
        self.state_size = STATE_SIZE
        self.actor = FlowActor()
        self.critic = MaskedPopulationCritic(OBSERVATION_SIZE, MAX_AGENTS, (256, 256))


def migrate_actor_99(model, parent):
    """Preserve 51 old locals and 17 pooled features; zero 18 new inputs."""
    actor = deepcopy(parent.actor.state_dict())
    key = 'mean_net.1.0.weight'
    previous, target = actor[key], model.actor.state_dict()[key]
    if previous.shape != (128, 68) or target.shape != (128, 86):
        raise ValueError('Flow migration requires the exact 99-to-117 ABI')
    weights = torch.zeros_like(target)
    weights[:, :51] = previous[:, :51]
    weights[:, 69:] = previous[:, 51:]
    actor[key] = weights
    model.actor.load_state_dict(actor, strict=True)
    with torch.no_grad():
        model.log_std.copy_(parent.log_std)
    return dict(from_observation_size=99, to_observation_size=117,
        inserted_local_indices=list(range(51, 69)), inserted_weights='zero',
        critic='fresh', optimizer='fresh', prior_actor_semantics='retained; new environment still requires evaluation')


class FlowTrainer(MappoTrainer):
    """Same losses, with separate qualified acceleration/lane label masks."""
    def __init__(self, model, **options):
        if model.observation_size != OBSERVATION_SIZE or model.state_size != STATE_SIZE:
            raise ValueError('Flow trainer requires the explicit 117/2832 ABI')
        near = options.get('near_limit_coef', 0.)
        super().__init__(model, **dict(options, near_limit_coef=0.))
        if near != .1:
            raise ValueError('Flow retains the reviewed near-limit coefficient .1')
        self.near_limit_coef = near

    def _prepare_batch(self, batch):
        data, samples = super()._prepare_batch(batch)
        active = batch['active'].reshape(-1).bool()
        for name in ('acceleration_supervision_eligible', 'lane_supervision_eligible'):
            value = batch.get(name)
            if not isinstance(value, torch.Tensor) or value.shape != batch['active'].shape or value.dtype != torch.bool:
                raise ValueError('Flow batch requires a boolean ' + name)
            data[name] = value.reshape(-1)[active].detach()
        return data, samples

    def _near_limit_terms(self, observations, teacher_actions, eligibility=None):
        if (observations.ndim != 2 or observations.shape[1] != OBSERVATION_SIZE
                or teacher_actions.shape != (len(observations), 2) or not len(observations)):
            raise ValueError('Flow near-limit shape mismatch')
        if not torch.isfinite(observations).all() or not torch.isfinite(teacher_actions).all():
            raise ValueError('Flow near-limit values must be finite')
        if ((observations[:, 0] < 0) | (observations[:, 0] > 1)
                | (observations[:, 4] <= 0) | (observations[:, 4] > 1)).any():
            raise ValueError('Flow near-limit speed fractions are invalid')
        if (teacher_actions[:, 0].abs() > 1).any():
            raise ValueError('Flow acceleration labels must be normalized')
        selected = observations[:, 0] >= observations[:, 4] - self.near_limit_band_m_s / 26.8224
        if eligibility is not None:
            selected = selected & eligibility
        raw = self.model.actor.mean_net(observations)[:, 0].tanh()
        predicted = -1.25 + 3.25 * raw
        target = -1.25 + 3.25 * teacher_actions[:, 0].detach()
        count = int(selected.sum())
        if count:
            error = predicted[selected] - target[selected]
            loss = error.square().mean()
            prediction_mean = float(predicted[selected].detach().mean())
            target_mean = float(target[selected].detach().mean())
            excess_fraction = float((error.detach() > .05).float().mean())
        else:
            loss = raw.sum() * 0.
            prediction_mean = target_mean = excess_fraction = 0.
        return loss, dict(near_limit_mse_m2_s4=float(loss.detach()), near_limit_samples=count,
            near_limit_batch_samples=len(observations), near_limit_sample_fraction=count / len(observations),
            near_limit_raw_acceleration_mean_m_s2=prediction_mean, near_limit_target_mean_m_s2=target_mean,
            near_limit_above_target_0p05_fraction=excess_fraction)

    def _loss_terms(self, part, ratio, entropy):
        # Preserve legacy PPO arithmetic and supervised formulas. Each masked
        # component averages only its qualified rows; zero evidence means zero
        # auxiliary gradient, never an invented target. PPO retains every row.
        actor_loss = -torch.minimum(ratio * part['advantages'],
            ratio.clamp(1.-self.clip, 1.+self.clip) * part['advantages']).mean()
        values = self.model.values(part['global_states'], part['agent_indices'])
        clipped = part['old_values'] + (values-part['old_values']).clamp(-self.value_clip, self.value_clip)
        value_loss = .5 * torch.maximum((values-part['returns']).square(), (clipped-part['returns']).square()).mean()
        entropy_mean = entropy.mean()
        loss = actor_loss + self.value_coef*value_loss - self.entropy_coef*entropy_mean
        weighted = dict(ppo=actor_loss, value=self.value_coef*value_loss, entropy=-self.entropy_coef*entropy_mean)
        obs, labels = part['local_obs'], part['teacher_actions']
        accel_mask, lane_mask = part['acceleration_supervision_eligible'], part['lane_supervision_eligible']
        zero = self.model.actor.mean_net(obs).sum()*0.
        acceleration = super()._imitation_loss(obs[accel_mask], labels[accel_mask])[1] if accel_mask.any() else zero
        lane = super()._imitation_loss(obs[lane_mask], labels[lane_mask])[2] if lane_mask.any() else zero
        imitation = self.imitation_acceleration_weight*acceleration + lane
        loss = loss + self.imitation_coef*imitation
        weighted.update(acceleration=self.imitation_coef*self.imitation_acceleration_weight*acceleration,
            lane=self.imitation_coef*lane)
        near, near_stats = self._near_limit_terms(obs, labels, accel_mask)
        loss = loss + self.near_limit_coef*near
        weighted['near_limit'] = self.near_limit_coef*near
        if not torch.isfinite(loss):
            raise ValueError('Nonfinite Flow loss')
        return loss, weighted, dict(actor_loss=float(actor_loss.detach()), value_loss=float(value_loss.detach()),
            entropy=float(entropy_mean.detach()), imitation_loss=float(imitation.detach()),
            imitation_acceleration_loss=float(acceleration.detach()), imitation_lane_loss=float(lane.detach()),
            acceleration_supervision_samples=int(accel_mask.sum()), lane_supervision_samples=int(lane_mask.sum()),
            **near_stats)


def critic_update(model, trainer, batch, deadline):
    """Future authorized critic warmup; actor/log_std are never updated."""
    import time
    from experiments.run_highway_environment_v05 import actor_digest
    from policies.mappo_diagnostics import critic_fit_metrics, current_gradient_norms
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
                raise ValueError('Nonfinite Flow critic warmup')
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
        raise ValueError('Flow critic warmup changed actor or exploration')
    return dict(optimizer_steps=len(steps), value_loss=sum(losses)/max(1,len(losses)), actor_frozen=True,
        actor_sha256=frozen, budget_exhausted=exhausted, gradient_diagnostics=dict(
            phase='critic_warmup', actual_steps=steps, critic_fit_before=before,
            critic_fit_after=critic_fit_metrics(model, states, indices, returns),
            loss_definition='0.5*mean squared value error; critic-only clipping .5'))
