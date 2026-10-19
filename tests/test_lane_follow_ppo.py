"""No PPO training/real optimizer: synthetic audit, budget and publication tests."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import pickle
import subprocess
import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

from environments.lane_follow_ppo import (
    MAX_OPTIMIZER_STEPS, PROPOSAL_SHA256, TrainingAudit, TrainingCounters,
    TrainingStopped, atomic_checkpoint, create_model, require_finite, sb3_logger_values, validate_proposal,
)

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def proposal():
    return json.loads((ROOT / 'experiments/lane-follow-ppo-proposal.json').read_text())


def test_exact_approved_proposal_identity(proposal):
    assert hashlib.sha256((ROOT / 'experiments/lane-follow-ppo-proposal.json').read_bytes()).hexdigest() == PROPOSAL_SHA256
    validate_proposal(proposal)


@pytest.mark.parametrize('section,key,value', [
    ('trainer', 'gamma', .99), ('trainer', 'n_steps', 1024),
    ('trainer', 'target_kl', None), ('trainer', 'device', 'cuda'),
    ('budget', 'training_seed', 2102), ('budget', 'training_runs', True),
    ('budget', 'training_wall_cap_s', 3601), ('budget', 'max_minibatch_optimizer_steps', 5121),
    ('task', 'num_envs', 2), ('task', 'training_route', 'right_r60'),
    ('normalization', 'norm_reward', True),
    ('checkpoints', 'after_completed_updates', [7, 16, 24, 32]),
])
def test_unapproved_parameter_rejected(proposal, section, key, value):
    proposal[section][key] = value
    with pytest.raises(ValueError):
        validate_proposal(proposal)


def test_import_has_no_runtime_or_training_side_effects():
    result = subprocess.run([sys.executable, '-c',
        "import sys; import environments.lane_follow_ppo; "
        "assert not any(x in sys.modules for x in ['torch','stable_baselines3','isaaclab','omni.kit'])"],
        cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_counter_all_32_rollouts_and_maximum_steps_are_exact():
    c = TrainingCounters()
    for step in range(1, 65537):
        c.observe_step(step)
        if step % 2048 == 0:
            c.before_update()
            for _ in range(160):
                c.before_optimizer_step()
                c.optimizer_steps += 1
            c.completed_updates += 1
    assert (c.control_steps, c.completed_updates, c.optimizer_steps) == (65536, 32, 5120)
    with pytest.raises(TrainingStopped, match='control_step'):
        c.observe_step(65537)
    with pytest.raises(TrainingStopped, match='optimizer_step'):
        c.before_optimizer_step()
    with pytest.raises(TrainingStopped, match='update_requires'):
        c.before_update()


@pytest.mark.parametrize('count', [0, 1, 2047, 2049, 4096])
def test_update_requires_next_complete_rollout(count):
    c = TrainingCounters(control_steps=count)
    with pytest.raises(TrainingStopped, match='complete_rollout'):
        c.before_update()


@pytest.mark.parametrize('step', [0, 2, True, 1.0])
def test_nonsequential_or_noninteger_transition_rejected(step):
    with pytest.raises(TrainingStopped):
        TrainingCounters().observe_step(step)


class Parameter:
    """Tiny array/tensor protocol fake: no autodiff or optimizer execution."""
    def __init__(self, values, grad=None):
        self.values = np.asarray(values, dtype=float)
        self.grad = grad

    def detach(self):
        return self

    def isfinite(self):
        return np.isfinite(self.values)


class FakeOptimizer:
    def __init__(self, action=lambda: None):
        self.action = action
        self.calls = 0
        self.state = {'fake': {'counter': 0, 'moment': Parameter([0.0])}}

    def step(self):
        self.calls += 1
        self.action()


def audit(tmp_path):
    return TrainingAudit(tmp_path, 100.0, clock=lambda: 1.0)


def test_actual_optimizer_calls_counted_independently_of_epochs_and_updates(tmp_path):
    a = audit(tmp_path)
    opt = FakeOptimizer()
    p = Parameter([1.0], Parameter([.2]))
    for _ in range(3):
        a.optimizer_step(opt, [p], loss_record={'loss': 1.25})
    a.close()
    assert opt.calls == a.counters.optimizer_steps == 3
    assert a.counters.completed_updates == 0
    records = [json.loads(line) for line in (tmp_path / 'optimizer-steps.jsonl').read_text().splitlines()]
    assert [x['optimizer_step'] for x in records] == [1, 2, 3]
    assert all(x['rollout_update'] == 1 for x in records)


@pytest.mark.parametrize('where', ['loss', 'parameter', 'gradient', 'budget', 'deadline'])
def test_pre_step_guard_never_calls_optimizer(tmp_path, where):
    a = audit(tmp_path)
    opt, p, loss = FakeOptimizer(), Parameter([1.0], Parameter([.2])), {'loss': 1.0}
    if where == 'loss': loss['loss'] = float('nan')
    if where == 'parameter': p.values[0] = float('inf')
    if where == 'gradient': p.grad.values[0] = float('nan')
    if where == 'budget': a.counters.optimizer_steps = MAX_OPTIMIZER_STEPS
    if where == 'deadline': a.clock = lambda: 100.0
    prior = a.counters.optimizer_steps
    with pytest.raises(TrainingStopped):
        a.optimizer_step(opt, [p], loss_record=loss)
    assert opt.calls == 0
    assert a.counters.optimizer_steps == prior


@pytest.mark.parametrize('where', ['parameter', 'optimizer_state'])
def test_post_step_nonfinite_preserves_actual_executed_call(tmp_path, where):
    a, p = audit(tmp_path), Parameter([1.0])
    opt = FakeOptimizer()
    opt.action = lambda: (p.values.__setitem__(0, float('nan')) if where == 'parameter'
                          else opt.state['fake']['moment'].values.__setitem__(0, float('inf')))
    with pytest.raises(TrainingStopped):
        a.optimizer_step(opt, [p], loss_record={'loss': 1.0})
    a.close()
    assert opt.calls == a.counters.optimizer_steps == 1
    assert json.loads((tmp_path / 'optimizer-steps.jsonl').read_text())['optimizer_step'] == 1


def test_optimizer_exception_is_not_falsely_counted_as_returned_step(tmp_path):
    def fail(): raise RuntimeError('fake failure')
    a, opt = audit(tmp_path), FakeOptimizer(fail)
    with pytest.raises(RuntimeError):
        a.optimizer_step(opt, [Parameter([1.0])], loss_record={'loss': 0.0})
    assert opt.calls == 1 and a.counters.optimizer_steps == 0


@pytest.mark.parametrize('value', [float('nan'), [1, float('inf')], {'nested': np.array([-np.inf])}, Parameter([np.nan])])
def test_nested_finite_guard(value):
    with pytest.raises(TrainingStopped, match='nonfinite'):
        require_finite(value, 'root')


def transition(*, done=False, tick=2):
    obs = np.zeros((1, 22), np.float32)
    info = {'TimeLimit.truncated': done}
    if done: info['terminal_observation'] = np.ones(22)
    return {'actions': np.array([[.7, .2, .3]]), 'clipped_actions': np.array([[.5, .2, .3]]),
        'new_obs': obs, 'rewards': np.array([1.5]), 'values': np.array([2.0]),
        'log_probs': np.array([-.3]), 'dones': np.array([done]), 'infos': [info],
        'lane_transition': {'reward_components': {'progress': 2.0, 'time': -.5},
            'observation': [0.0] * 22, 'episode_tick': tick, 'reason': 'time_limit' if done else 'running',
            'success': False, 'terminated': False, 'episode_serial': 7}}


def test_raw_reward_episode_clipping_brake_and_settle_boundary_accounting(tmp_path):
    a = audit(tmp_path)
    first = transition(tick=240)
    a.observe(first, SimpleNamespace(num_timesteps=1))
    # SB3 later bootstraps this array; accumulated task reward must not change.
    first['rewards'][0] += .999 * 100
    a.observe(transition(done=True, tick=242), SimpleNamespace(num_timesteps=2))
    a.close()
    record = json.loads((tmp_path / 'episodes.jsonl').read_text())
    assert record['reward'] == 3.0
    assert record['reward_components'] == {'progress': 4.0, 'time': -1.0}
    assert record['clipped_transitions'] == record['brake_priority_requests'] == 2
    assert record['settling_transitions'] == 1
    assert record['truncated'] is True and record['reason'] == 'time_limit'
    assert a.counters.completed_episodes == 1
    assert a.snapshot()['partial_episode']['steps'] == 0
    assert a.snapshot()['clipped_action_components'] == [2, 0, 0]


def test_invalid_completed_transition_still_counts_physical_exposure(tmp_path):
    a, local = audit(tmp_path), transition()
    local['rewards'][0] = np.nan
    with pytest.raises(TrainingStopped):
        a.observe(local, SimpleNamespace(num_timesteps=1))
    assert a.counters.control_steps == 1


def test_terminal_snapshot_required_before_timeout_bootstrap(tmp_path):
    a, local = audit(tmp_path), transition(done=True)
    local['infos'][0].pop('terminal_observation')
    with pytest.raises(TrainingStopped, match='terminal_observation'):
        a.observe(local, SimpleNamespace(num_timesteps=1))


@pytest.mark.parametrize('deadline', [1.0, 0.0, float('nan'), float('inf'), 3601.01])
def test_deadline_is_bounded_and_finite(tmp_path, deadline):
    with pytest.raises(ValueError):
        TrainingAudit(tmp_path, deadline, clock=lambda: 1.0)


class SavedObject:
    def __init__(self, payload=b'finite fake persisted state', fail=False):
        self.payload, self.fail = payload, fail

    def save(self, path):
        Path(path).write_bytes(self.payload)
        if self.fail: raise RuntimeError('simulated disk/write failure')


def test_checkpoint_is_complete_hashed_and_preserves_rng_metadata(tmp_path):
    target = tmp_path / 'update-08'
    state = {'completed_updates': 8, 'control_steps': 16384, 'optimizer_steps': 801,
             'training_seed': 2101, 'proposal_sha256': PROPOSAL_SHA256,
             'reset_state': {'episode': 3, 'tick': 622}}
    rng = {'python': (1, 2, 3), 'numpy': np.array([4, 5]), 'torch_cpu': b'fake'}
    manifest = atomic_checkpoint(SavedObject(), SavedObject(b'normalizer'), target, state, rng)
    assert {p.name for p in target.iterdir()} == {'model.zip', 'vecnormalize.pkl', 'rng-state.pkl', 'state.json', 'checkpoint.json'}
    assert manifest['complete'] is True and manifest['exact_native_mid_episode_resume'] is False
    for name, expected in manifest['files'].items():
        assert hashlib.sha256((target / name).read_bytes()).hexdigest() == expected
    assert json.loads((target / 'state.json').read_text()) == state
    with (target / 'rng-state.pkl').open('rb') as stream:
        restored = pickle.load(stream)
    np.testing.assert_array_equal(restored['numpy'], rng['numpy'])
    assert restored['python'] == rng['python']
    with pytest.raises(FileExistsError):
        atomic_checkpoint(SavedObject(), SavedObject(), target, state, rng)


def test_failed_checkpoint_never_publishes_complete_directory(tmp_path):
    target = tmp_path / 'update-16'
    with pytest.raises(RuntimeError):
        atomic_checkpoint(SavedObject(), SavedObject(fail=True), target, {}, {})
    assert not target.exists()
    partials = list(tmp_path.glob('.*.partial'))
    assert len(partials) == 1 and not (partials[0] / 'checkpoint.json').exists()


def test_model_rejects_wrong_environment_before_importing_runtime(proposal, tmp_path):
    env = SimpleNamespace(num_envs=2)
    with pytest.raises(ValueError, match='one environment'):
        create_model(env, proposal, output_dir=tmp_path, deadline_monotonic=100,
                     provenance={}, reset_state=lambda: {})


def fake_runtime(monkeypatch):
    """Exercise the real guarded subclass/callback, but no ML library is loaded."""
    class BaseCallback:
        pass

    class FakePolicy:
        def parameters(self): return [Parameter([1.0])]
        def register_forward_pre_hook(self, hook):
            self.pre_hook = hook
            return 'fake_pre_hook_handle'
        def register_forward_hook(self, hook):
            self.post_hook = hook
            return 'fake_post_hook_handle'

    class PPO:
        def __init__(self, **kwargs):
            self.policy = FakePolicy()
            self.num_timesteps = 0
            self.kwargs = kwargs

        def _excluded_save_params(self): return ['policy', 'env']

        def learn(self, total_timesteps, callback, **kwargs):
            # Simulate exactly the external callback boundary following one
            # already completed physical step. No rollout/optimizer is run.
            self.num_timesteps += 1
            callback.model = self
            callback.locals = transition()
            callback._on_step()
            return self

    fake_sb3 = ModuleType('stable_baselines3')
    fake_sb3.__version__, fake_sb3.PPO = '2.9.0', PPO
    fake_callbacks = ModuleType('stable_baselines3.common.callbacks')
    fake_callbacks.BaseCallback = BaseCallback
    fake_callbacks.CallbackList = lambda callbacks: callbacks
    fake_utils = ModuleType('stable_baselines3.common.utils')
    fake_utils.explained_variance = lambda a, b: 1.0
    fake_torch = ModuleType('torch')
    fake_torch.set_num_threads = lambda count: None
    fake_torch.nn = SimpleNamespace(Tanh=type('FakeTanh', (), {}))
    for name, module in [('stable_baselines3', fake_sb3),
                         ('stable_baselines3.common.callbacks', fake_callbacks),
                         ('stable_baselines3.common.utils', fake_utils), ('torch', fake_torch)]:
        monkeypatch.setitem(sys.modules, name, module)
    return SimpleNamespace(num_envs=1,
        observation_space=SimpleNamespace(shape=(22,)),
        action_space=SimpleNamespace(shape=(3,), low=np.array([-.5, 0., 0.]), high=np.array([.5, 1., 1.])),
        training=True, norm_obs=True, norm_reward=False, clip_obs=10, epsilon=1e-8,
        get_original_obs=lambda: np.zeros((1, 22)), get_original_reward=lambda: np.array([1.5]))


def fake_model(monkeypatch, proposal, tmp_path, on_step=None):
    import time
    env = fake_runtime(monkeypatch)
    model = create_model(env, proposal, output_dir=tmp_path,
        deadline_monotonic=time.monotonic() + 100, provenance={'synthetic': True},
        reset_state=lambda: {'episode_index': 0}, on_step=on_step)
    return model, env


def test_actual_callback_counts_native_step_before_failing_runner_hook(monkeypatch, proposal, tmp_path):
    def fail(local, model): raise RuntimeError('native recorder failed')
    model, _ = fake_model(monkeypatch, proposal, tmp_path, on_step=fail)
    with pytest.raises(RuntimeError, match='native recorder'):
        model.learn()
    assert model.training_state()['control_steps'] == model.num_timesteps == 1
    assert model.training_state()['status'] == 'failed'
    assert model.training_state()['optimizer_steps'] == 0


def test_actual_callback_counts_native_step_before_invalid_raw_observation(monkeypatch, proposal, tmp_path):
    model, env = fake_model(monkeypatch, proposal, tmp_path)
    env.get_original_obs = lambda: np.full((1, 22), np.inf)
    assert model.learn() is model
    assert model.training_state()['control_steps'] == model.num_timesteps == 1
    assert model.training_state()['status'] == 'stopped'
    assert model.training_state()['stop_reason'] == 'nonfinite:raw_observation'


def test_second_learn_rejected_and_hooks_excluded_from_ordinary_save(monkeypatch, proposal, tmp_path):
    calls = []
    def hook(local, model):
        calls.append(model.num_timesteps)
        return False
    model, _ = fake_model(monkeypatch, proposal, tmp_path, on_step=hook)
    model.learn()
    assert calls == [1]
    assert model.training_state()['stop_reason'] == 'runner_requested_stop'
    assert {'_lane_audit', '_lane_learn_called', '_lane_policy_hooks'} <= set(model._excluded_save_params())
    with pytest.raises(ValueError, match='second training call'):
        model.learn()


@pytest.mark.parametrize('timesteps,kwargs', [(65537, {}), (65536.0, {}), (65536, {'reset_num_timesteps': False})])
def test_learn_rejects_changed_budget_or_resume_before_any_native_step(monkeypatch, proposal, tmp_path, timesteps, kwargs):
    model, _ = fake_model(monkeypatch, proposal, tmp_path)
    with pytest.raises(ValueError):
        model.learn(timesteps, **kwargs)
    assert model.num_timesteps == 0


def test_forward_guards_raw_observation_and_gaussian_action_before_clipping(monkeypatch, proposal, tmp_path):
    model, env = fake_model(monkeypatch, proposal, tmp_path)
    env.get_original_obs = lambda: np.full((1, 22), np.inf)
    with pytest.raises(TrainingStopped, match='raw_policy_observation'):
        model.policy.pre_hook(model.policy, (np.full((1, 22), 10.0),))
    with pytest.raises(TrainingStopped, match='policy_action'):
        model.policy.post_hook(model.policy, (), (np.array([[np.inf, 0., 0.]]), np.array([1.]), np.array([0.])))


def test_native_logger_uses_last_epoch_kl_last_batch_loss_and_all_batch_other_means():
    records = [dict(loss=1., policy_loss=.2, value_loss=2., entropy_loss=-3., clip_fraction=.1),
               dict(loss=9., policy_loss=.8, value_loss=6., entropy_loss=-5., clip_fraction=.3)]
    logs = sb3_logger_values(records, [np.array(.04, dtype=np.float32), np.array(.06, dtype=np.float32)],
        explained_variance=.75, policy_std=.367, clip_range=.2)
    assert logs['train/approx_kl'] == np.mean(np.array([.04, .06], dtype=np.float32))
    assert logs['train/loss'] == 9.0  # not the whole-update mean of 5
    assert logs['train/policy_gradient_loss'] == .5
    assert logs['train/value_loss'] == 4.
    assert logs['train/entropy_loss'] == -4.
    assert logs['train/clip_fraction'] == pytest.approx(.2)
    assert logs['train/explained_variance'] == .75
    assert logs['train/std'] == .367 and logs['train/clip_range'] == .2
    assert len(logs) == 9


def test_native_undefined_explained_variance_is_not_a_nonfinite_loss():
    records = [dict(loss=1., policy_loss=.2, value_loss=2., entropy_loss=-3., clip_fraction=.1)]
    logs = sb3_logger_values(records, [np.array(.01, dtype=np.float32)],
        explained_variance=float('nan'), policy_std=.367, clip_range=.2)
    assert np.isnan(logs['train/explained_variance'])
    assert np.isfinite(logs['train/loss'])
