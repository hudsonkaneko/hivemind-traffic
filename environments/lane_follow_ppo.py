"""Bounded, auditable SB3 2.9.0 PPO core; importing this module starts nothing.

The native runner owns authorization, the environment, resource supervision and
raw physical traces. ``create_model`` imports Torch/SB3 only when explicitly
called after runtime setup. There is no trainer CLI or automatic resume here.
"""
from copy import deepcopy
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import pickle
import random
import time
import uuid

import numpy as np


PROPOSAL_SHA256 = '5999396ed15fbdd15f82a4f49779da8c2f3458e3118a477ce0219d908681dafd'
TOTAL_STEPS = 65536
ROLLOUT_STEPS = 2048
MAX_UPDATES = 32
MAX_OPTIMIZER_STEPS = 5120
CHECKPOINT_UPDATES = (8, 16, 24, 32)
EXPECTED_TRAINER = {
    'library': 'stable-baselines3', 'version': '2.9.0', 'algorithm': 'PPO',
    'policy': 'MlpPolicy', 'device': 'cpu', 'torch_num_threads': 4,
    'policy_kwargs': {'net_arch': {'pi': [64, 64], 'vf': [64, 64]},
                      'activation_fn': 'torch.nn.Tanh', 'ortho_init': True, 'log_std_init': -1},
    'learning_rate': 0.0003, 'n_steps': 2048, 'batch_size': 128, 'n_epochs': 10,
    'gamma': 0.999, 'gae_lambda': 0.95, 'clip_range': 0.2, 'clip_range_vf': None,
    'ent_coef': 0, 'vf_coef': 0.5, 'max_grad_norm': 0.5,
    'normalize_advantage': True, 'target_kl': 0.02, 'use_sde': False,
}


class TrainingStopped(RuntimeError):
    """A declared finite-state, time or budget guard stopped this single run."""


def _json(value):
    return json.dumps(value, sort_keys=True, ensure_ascii=True, allow_nan=False)


def require_finite(value, name):
    """Validate arrays/tensors/trees without importing Torch or copying tensors."""
    if isinstance(value, dict):
        for key, item in value.items():
            require_finite(item, f'{name}.{key}')
    elif isinstance(value, (tuple, list)):
        for index, item in enumerate(value):
            require_finite(item, f'{name}[{index}]')
    elif hasattr(value, 'detach'):
        # All supported training tensors are CPU. This check preserves gradients.
        if not bool(value.detach().isfinite().all().item()):
            raise TrainingStopped(f'nonfinite:{name}')
    else:
        try:
            valid = np.isfinite(np.asarray(value)).all()
        except (TypeError, ValueError) as exc:
            raise TrainingStopped(f'nonnumeric:{name}') from exc
        if not valid:
            raise TrainingStopped(f'nonfinite:{name}')


def validate_proposal(proposal):
    """Reject changed algorithm/budgets; the runner verifies the whole file hash."""
    _json(proposal)
    if _json(proposal['trainer']) != _json(EXPECTED_TRAINER):
        raise ValueError('Trainer parameters differ from the approved proposal')
    expected = {
        'training_seed': 2101, 'total_control_transitions': TOTAL_STEPS,
        'completed_rollout_updates': MAX_UPDATES,
        'max_minibatch_optimizer_steps': MAX_OPTIMIZER_STEPS,
        'training_wall_cap_s': 3600, 'training_runs': 1,
        'serial_only': True, 'no_automatic_extension_or_resume': True,
    }
    for key, value in expected.items():
        if type(proposal['budget'].get(key)) is not type(value) or proposal['budget'][key] != value:
            raise ValueError(f'Unapproved budget: {key}')
    if (proposal['checkpoints']['after_completed_updates'] != list(CHECKPOINT_UPDATES)
            or proposal['checkpoints']['control_steps'] != [x * ROLLOUT_STEPS for x in CHECKPOINT_UPDATES]):
        raise ValueError('Unapproved checkpoint schedule')
    for key, value in {'num_envs': 1, 'physics_hz': 120, 'control_hz': 60,
                       'training_route': 'left_r60', 'stop_s_m': 60,
                       'target_speed_m_s': 3, 'episode_s': 35, 'settle_s': 2,
                       'success_hold_s': 1}.items():
        if proposal['task'][key] != value:
            raise ValueError(f'Unapproved task: {key}')
    for key, value in {'norm_obs': True, 'norm_reward': False, 'clip_obs': 10,
                       'epsilon': 1e-8}.items():
        if proposal['normalization'][key] != value:
            raise ValueError(f'Unapproved normalization: {key}')


def sb3_logger_values(records, final_epoch_kls, *, explained_variance, policy_std, clip_range):
    """SB3 2.9.0 reporting windows, separate from whole-update audit averages.

    A KL-rejected minibatch participates in these diagnostics as in SB3. KL is
    averaged over the final entered epoch; loss is the last evaluated batch.
    Undefined explained variance is allowed in the native logger only (the
    strict JSON audit represents it as null).
    """
    return {
        'train/entropy_loss': np.mean([r['entropy_loss'] for r in records]),
        'train/policy_gradient_loss': np.mean([r['policy_loss'] for r in records]),
        'train/value_loss': np.mean([r['value_loss'] for r in records]),
        'train/approx_kl': np.mean(final_epoch_kls),
        'train/clip_fraction': np.mean([r['clip_fraction'] for r in records]),
        'train/loss': records[-1]['loss'],
        'train/explained_variance': explained_variance,
        'train/std': policy_std,
        'train/clip_range': clip_range,
    }


@dataclass
class TrainingCounters:
    control_steps: int = 0
    completed_updates: int = 0
    optimizer_steps: int = 0
    completed_episodes: int = 0
    status: str = 'created'
    stop_reason: str | None = None

    def observe_step(self, actual_steps):
        if type(actual_steps) is not int or actual_steps != self.control_steps + 1 or actual_steps > TOTAL_STEPS:
            raise TrainingStopped('control_step_budget_or_sequence')
        self.control_steps = actual_steps

    def before_update(self):
        if (self.completed_updates >= MAX_UPDATES
                or self.control_steps != (self.completed_updates + 1) * ROLLOUT_STEPS):
            raise TrainingStopped('update_requires_exact_complete_rollout')

    def before_optimizer_step(self):
        if self.optimizer_steps >= MAX_OPTIMIZER_STEPS:
            raise TrainingStopped('optimizer_step_budget')


class TrainingAudit:
    """Streaming bounded-memory accounting and durable compact learning logs."""

    def __init__(self, output_dir, deadline_monotonic, *, clock=time.monotonic):
        self.clock = clock
        self.started = clock()
        if not math.isfinite(deadline_monotonic) or not self.started < deadline_monotonic <= self.started + 3600:
            raise ValueError('Deadline must be in the future and at most one hour away')
        self.deadline = deadline_monotonic
        self.output = Path(output_dir)
        self.output.mkdir(parents=True, exist_ok=True)
        self.counters = TrainingCounters()
        self._streams = {}
        self.episode = self._empty_episode()
        self.clipped_transitions = 0
        self.clipped_components = [0, 0, 0]
        self.brake_priority_requests = 0
        self.settling_transitions = 0
        self.checkpoints = []

    @staticmethod
    def _empty_episode():
        return {'steps': 0, 'reward': 0.0, 'reward_components': {},
                'clipped_transitions': 0, 'brake_priority_requests': 0,
                'settling_transitions': 0}

    def check_time(self):
        if self.clock() >= self.deadline:
            raise TrainingStopped('training_wall_deadline')

    def emit(self, name, record):
        if name not in self._streams:
            # Never overwrite evidence from another invocation.
            self._streams[name] = (self.output / f'{name}.jsonl').open('x', encoding='utf-8')
        stream = self._streams[name]
        stream.write(_json(record) + '\n')
        stream.flush()

    def observe(self, local, model, *, count_step=True):
        """Called before SB3 mutates a timeout reward with its value bootstrap."""
        if count_step:
            self.counters.observe_step(int(model.num_timesteps))
        for key in ('actions', 'clipped_actions', 'new_obs', 'rewards', 'values', 'log_probs'):
            require_finite(local[key], key)
        actions = np.asarray(local['actions'])
        clipped = np.asarray(local['clipped_actions'])
        if actions.shape != (1, 3) or clipped.shape != (1, 3):
            raise TrainingStopped('action_shape')
        dones = np.asarray(local['dones'])
        if dones.shape != (1,) or dones.dtype != np.bool_:
            raise TrainingStopped('done_shape_or_dtype')
        infos = local['infos']
        if len(infos) != 1:
            raise TrainingStopped('info_shape')
        if bool(dones[0]):
            terminal = infos[0].get('terminal_observation')
            if terminal is None or np.asarray(terminal).shape != (22,):
                raise TrainingStopped('missing_terminal_observation')
            require_finite(terminal, 'terminal_observation')
        changed = np.not_equal(actions[0], clipped[0])
        clipped_count = int(bool(changed.any()))
        # Requested brake priority is separate from actual settle/slew/failure overrides.
        brake = int(clipped[0, 1] > 0 and clipped[0, 2] > 0)
        self.clipped_transitions += clipped_count
        self.clipped_components = [a + int(b) for a, b in zip(self.clipped_components, changed)]
        self.brake_priority_requests += brake
        ep = self.episode
        ep['steps'] += 1
        ep['reward'] += float(np.asarray(local['rewards']).reshape(-1)[0])
        ep['clipped_transitions'] += clipped_count
        ep['brake_priority_requests'] += brake
        transition = local.get('lane_transition')
        if transition is not None:
            terms = transition['reward_components']
            require_finite(terms, 'reward_components')
            for key, value in terms.items():
                ep['reward_components'][key] = ep['reward_components'].get(key, 0.0) + float(value)
            # The action ending at tick 240 was still issued during settling;
            # the returned observation at exactly 2 s already says settling=0.
            settling = int(transition['episode_tick'] <= 240)
            self.settling_transitions += settling
            ep['settling_transitions'] += settling
        if bool(dones[0]):
            self.counters.completed_episodes += 1
            self.emit('episodes', {**deepcopy(ep), 'episode': self.counters.completed_episodes,
                'control_steps': self.counters.control_steps,
                'reason': None if transition is None else transition['reason'],
                'success': None if transition is None else transition['success'],
                'terminated': None if transition is None else transition['terminated'],
                'truncated': bool(infos[0].get('TimeLimit.truncated', False)),
                'episode_serial': None if transition is None else transition['episode_serial']})
            self.episode = self._empty_episode()

    def optimizer_step(self, optimizer, parameters, *, loss_record):
        """Count only optimizer calls that returned; pre/post checks use real tensors."""
        self.check_time()
        self.counters.before_optimizer_step()
        parameters = list(parameters)
        require_finite(loss_record, 'loss')
        for index, parameter in enumerate(parameters):
            require_finite(parameter, f'parameter[{index}]')
            if parameter.grad is not None:
                require_finite(parameter.grad, f'gradient[{index}]')
        optimizer.step()
        self.counters.optimizer_steps += 1
        # Persist the actual call even if the subsequent parameter check fails.
        self.emit('optimizer-steps', {**loss_record, 'optimizer_step': self.counters.optimizer_steps,
            'rollout_update': self.counters.completed_updates + 1,
            'control_steps': self.counters.control_steps})
        for index, parameter in enumerate(parameters):
            require_finite(parameter, f'updated_parameter[{index}]')
        require_finite(optimizer.state, 'optimizer_state')

    def snapshot(self):
        return {**asdict(self.counters), 'elapsed_wall_s': max(0.0, self.clock() - self.started),
            'training_seed': 2101, 'proposal_sha256': PROPOSAL_SHA256,
            'clipped_transitions': self.clipped_transitions,
            'clipped_action_components': list(self.clipped_components),
            'brake_priority_requests': self.brake_priority_requests,
            'settling_transitions': self.settling_transitions,
            'partial_episode': deepcopy(self.episode), 'checkpoints': list(self.checkpoints),
            'exact_native_mid_episode_resume': False}

    def close(self):
        for stream in self._streams.values():
            stream.flush()
            os.fsync(stream.fileno())
            stream.close()
        self._streams.clear()


def atomic_checkpoint(model, vec_env, directory, state, rng_state):
    """Publish a complete new checkpoint directory using one same-volume rename.

    On failure the hidden partial directory remains as explicit failed evidence;
    no complete-looking checkpoint is published or overwritten.
    """
    destination = Path(directory)
    if destination.exists():
        raise FileExistsError(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.parent / f'.{destination.name}.{uuid.uuid4().hex}.partial'
    temporary.mkdir()
    model.save(temporary / 'model.zip')
    vec_env.save(temporary / 'vecnormalize.pkl')
    (temporary / 'state.json').write_text(_json(state) + '\n', encoding='utf-8')
    with (temporary / 'rng-state.pkl').open('wb') as stream:
        pickle.dump(rng_state, stream, protocol=pickle.HIGHEST_PROTOCOL)
    names = ('model.zip', 'vecnormalize.pkl', 'rng-state.pkl', 'state.json')
    hashes = {}
    for name in names:
        path = temporary / name
        # Windows FlushFileBuffers requires a writable handle.
        with path.open('r+b') as stream:
            hashes[name] = hashlib.file_digest(stream, 'sha256').hexdigest()
            os.fsync(stream.fileno())
    manifest = {'schema_version': 1, 'complete': True, 'files': hashes,
                'exact_native_mid_episode_resume': False}
    with (temporary / 'checkpoint.json').open('w', encoding='utf-8') as stream:
        stream.write(_json(manifest) + '\n')
        stream.flush()
        os.fsync(stream.fileno())
    temporary.rename(destination)
    return manifest


def create_model(vec_env, proposal, *, output_dir, deadline_monotonic, provenance,
                 reset_state, on_step=None, after_update=None):
    """Construct, but do not train, the single approved CPU PPO model.

    ``model.learn(total_timesteps=65536)`` installs the internal callback.
    ``on_step(locals, model)`` may add the raw ``lane_transition`` record and may
    return False to request a stop. ``after_update(model, record)`` runs after
    the completed optimization update and any scheduled atomic checkpoint.
    Ordinary ``PPO.load(model.zip)`` loads inference/optimizer state without
    these live hooks. The runner alone grants authorization and starts learn.
    """
    validate_proposal(proposal)
    _json(provenance)
    if not callable(reset_state):
        raise TypeError('reset_state must be callable')
    if vec_env.num_envs != 1 or tuple(vec_env.observation_space.shape) != (22,):
        raise ValueError('Exactly one environment with 22 observations is required')
    if (tuple(vec_env.action_space.shape) != (3,)
            or not np.array_equal(vec_env.action_space.low, [-0.5, 0, 0])
            or not np.array_equal(vec_env.action_space.high, [0.5, 1, 1])):
        raise ValueError('The approved physical action space is required')
    for key, expected in {'training': True, 'norm_obs': True, 'norm_reward': False,
                          'clip_obs': 10, 'epsilon': 1e-8}.items():
        if getattr(vec_env, key, None) != expected:
            raise ValueError(f'Invalid training VecNormalize.{key}')
    import stable_baselines3 as sb3
    import torch
    from stable_baselines3.common.callbacks import BaseCallback, CallbackList
    from stable_baselines3.common.utils import explained_variance
    if sb3.__version__ != '2.9.0':
        raise ValueError('This core is source-reviewed against installed SB3 2.9.0')
    torch.set_num_threads(4)
    audit = TrainingAudit(output_dir, deadline_monotonic)

    class GuardedPPO(sb3.PPO):
        def _excluded_save_params(self):
            return super()._excluded_save_params() + [key for key in self.__dict__ if key.startswith('_lane_')]

        def training_state(self):
            return self._lane_audit.snapshot()

        def collect_rollouts(self, env, callback, rollout_buffer, n_rollout_steps):
            audit.check_time()
            if n_rollout_steps != ROLLOUT_STEPS or self.num_timesteps + n_rollout_steps > TOTAL_STEPS:
                raise TrainingStopped('rollout_budget')
            result = super().collect_rollouts(env, callback, rollout_buffer, n_rollout_steps)
            if result:
                for key in ('observations', 'actions', 'rewards', 'values', 'log_probs', 'advantages', 'returns'):
                    require_finite(getattr(rollout_buffer, key), f'rollout.{key}')
            return result

        def learn(self, total_timesteps=TOTAL_STEPS, callback=None, **kwargs):
            if type(total_timesteps) is not int or total_timesteps != TOTAL_STEPS:
                raise ValueError('Exactly 65536 transitions were approved')
            if self._lane_learn_called or self.num_timesteps != 0:
                raise ValueError('A second training call or resume is not authorized')
            if kwargs.get('reset_num_timesteps', True) is not True:
                raise ValueError('Resume is not supported')
            self._lane_learn_called = True
            audit.counters.status = 'running'
            internal = AuditCallback()
            callbacks = internal if callback is None else CallbackList([internal, self._init_callback(callback)])
            try:
                result = super().learn(total_timesteps, callback=callbacks, **kwargs)
                if audit.counters.status == 'running':
                    if audit.counters.control_steps == TOTAL_STEPS and audit.counters.completed_updates == MAX_UPDATES:
                        audit.counters.status = 'completed'
                    else:
                        audit.counters.status = 'stopped'
                        audit.counters.stop_reason = 'callback_stop'
                return result
            except TrainingStopped as exc:
                audit.counters.status = 'stopped'
                audit.counters.stop_reason = str(exc)
                return self
            except BaseException as exc:
                audit.counters.status = 'failed'
                audit.counters.stop_reason = f'{type(exc).__name__}: {exc}'
                raise
            finally:
                audit.emit('training-events', audit.snapshot())
                audit.close()

        def train(self):
            # SB3 2.9.0 PPO.train arithmetic/order retained; guards surround the
            # real loss/backward/optimizer calls. _n_updates remains epoch count.
            audit.check_time()
            audit.counters.before_update()
            self.policy.set_training_mode(True)
            self._update_learning_rate(self.policy.optimizer)
            clip_range = self.clip_range(self._current_progress_remaining)
            losses, kls, clips = [], [], []
            optimizer_before = audit.counters.optimizer_steps
            early_kl = False
            for epoch in range(self.n_epochs):
                final_epoch_kls = []
                for batch in self.rollout_buffer.get(self.batch_size):
                    audit.check_time()
                    for name in ('observations', 'actions', 'old_values', 'old_log_prob', 'advantages', 'returns'):
                        require_finite(getattr(batch, name), f'minibatch.{name}')
                    values, log_prob, entropy = self.policy.evaluate_actions(batch.observations, batch.actions)
                    require_finite((values, log_prob, entropy), 'evaluate_actions')
                    values = values.flatten()
                    advantages = batch.advantages
                    if self.normalize_advantage and len(advantages) > 1:
                        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
                    ratio = torch.exp(log_prob - batch.old_log_prob)
                    policy_loss = -torch.min(advantages * ratio,
                        advantages * torch.clamp(ratio, 1 - clip_range, 1 + clip_range)).mean()
                    value_loss = torch.nn.functional.mse_loss(batch.returns, values)
                    entropy_loss = -torch.mean(entropy)
                    loss = policy_loss + self.ent_coef * entropy_loss + self.vf_coef * value_loss
                    with torch.no_grad():
                        log_ratio = log_prob - batch.old_log_prob
                        native_kl = torch.mean((torch.exp(log_ratio) - 1) - log_ratio).cpu().numpy()
                        approx_kl = float(native_kl)
                        clip_fraction = torch.mean((torch.abs(ratio - 1) > clip_range).float()).item()
                    record = {'epoch': epoch + 1, 'loss': loss.item(),
                        'policy_loss': policy_loss.item(), 'value_loss': value_loss.item(),
                        'entropy_loss': entropy_loss.item(), 'approx_kl': approx_kl,
                        'clip_fraction': clip_fraction}
                    require_finite(record, 'loss')
                    losses.append(record)
                    kls.append(approx_kl)
                    final_epoch_kls.append(native_kl)
                    clips.append(clip_fraction)
                    if self.target_kl is not None and approx_kl > 1.5 * self.target_kl:
                        early_kl = True
                        break
                    self.policy.optimizer.zero_grad()
                    loss.backward()
                    for index, parameter in enumerate(self.policy.parameters()):
                        if parameter.grad is not None:
                            require_finite(parameter.grad, f'gradient_before_clip[{index}]')
                    norm = torch.nn.utils.clip_grad_norm_(self.policy.parameters(), self.max_grad_norm,
                                                        error_if_nonfinite=True)
                    record['gradient_norm_before_clip'] = float(norm.item())
                    audit.optimizer_step(self.policy.optimizer, self.policy.parameters(), loss_record=record)
                self._n_updates += 1
                if early_kl:
                    break
            # A KL-shortened train() is a completed update, possibly with zero
            # optimizer calls; record both counts without pretending all ran.
            audit.counters.completed_updates += 1
            update = {**audit.snapshot(), 'optimizer_steps_this_update': audit.counters.optimizer_steps - optimizer_before,
                'epochs_entered': epoch + 1, 'kl_early_stop': early_kl,
                'mean_loss': float(np.mean([r['loss'] for r in losses])),
                'mean_policy_loss': float(np.mean([r['policy_loss'] for r in losses])),
                'mean_value_loss': float(np.mean([r['value_loss'] for r in losses])),
                'mean_entropy_loss': float(np.mean([r['entropy_loss'] for r in losses])),
                'mean_approx_kl': float(np.mean(kls)), 'mean_clip_fraction': float(np.mean(clips))}
            # Undefined explained variance (constant returns) is recorded as
            # null, not confused with a nonfinite training tensor/loss.
            ev = float(explained_variance(self.rollout_buffer.values.flatten(), self.rollout_buffer.returns.flatten()))
            update['explained_variance'] = ev if math.isfinite(ev) else None
            policy_std = torch.exp(self.policy.log_std).mean().item()
            require_finite(policy_std, 'policy_std')
            for key, value in sb3_logger_values(losses, final_epoch_kls,
                    explained_variance=ev, policy_std=policy_std, clip_range=clip_range).items():
                self.logger.record(key, value)
            self.logger.record('train/n_updates', self._n_updates, exclude='tensorboard')
            self.logger.record('train/completed_rollout_updates', audit.counters.completed_updates)
            self.logger.record('train/actual_optimizer_steps', audit.counters.optimizer_steps)
            audit.emit('updates', update)
            if audit.counters.completed_updates in CHECKPOINT_UPDATES:
                audit.check_time()
                checkpoint_name = f'update-{audit.counters.completed_updates:02d}'
                state = {**audit.snapshot(), 'config': deepcopy(proposal), 'provenance': deepcopy(provenance),
                         'reset_state': reset_state(), 'checkpoint_phase': 'after_completed_train_update'}
                rng = {'python': random.getstate(), 'numpy': np.random.get_state(),
                       'torch_cpu': torch.get_rng_state().clone(),
                       'torch_cuda': torch.cuda.get_rng_state_all() if torch.cuda.is_initialized() else []}
                atomic_checkpoint(self, vec_env, audit.output / 'checkpoints' / checkpoint_name, state, rng)
                audit.checkpoints.append(checkpoint_name)
            if after_update is not None:
                after_update(self, deepcopy(update))
            audit.check_time()

    class AuditCallback(BaseCallback):
        def _on_step(self):
            # Count the completed physical step even if a subsequent guard or
            # runner callback detects invalid output and aborts this rollout.
            audit.counters.observe_step(int(self.model.num_timesteps))
            # Raw observation checks precede any action using a potentially
            # clipped-to-finite VecNormalize result, including auto-reset obs.
            require_finite(vec_env.get_original_obs(), 'raw_observation')
            require_finite(vec_env.get_original_reward(), 'raw_reward')
            proceed = True if on_step is None else on_step(self.locals, self.model)
            audit.observe(self.locals, self.model, count_step=False)
            if proceed is False:
                audit.counters.status = 'stopped'
                audit.counters.stop_reason = 'runner_requested_stop'
                return False
            audit.check_time()
            return True

    kwargs = deepcopy(EXPECTED_TRAINER)
    for name in ('library', 'version', 'algorithm', 'torch_num_threads'):
        kwargs.pop(name)
    kwargs['policy_kwargs']['activation_fn'] = torch.nn.Tanh
    model = GuardedPPO(env=vec_env, seed=2101, **kwargs)
    model._lane_audit = audit
    model._lane_learn_called = False
    for index, parameter in enumerate(model.policy.parameters()):
        require_finite(parameter, f'initial_parameter[{index}]')

    def before_policy(_module, inputs):
        audit.check_time()
        require_finite(inputs, 'policy_observation')
        original = vec_env.get_original_obs()
        if original is not None:
            require_finite(original, 'raw_policy_observation')

    def after_policy(_module, _inputs, outputs):
        # Check the Gaussian actions before finite bounds could hide infinity.
        require_finite(outputs, 'policy_action_value_log_probability')

    model._lane_policy_hooks = [model.policy.register_forward_pre_hook(before_policy),
                                model.policy.register_forward_hook(after_policy)]
    return model
