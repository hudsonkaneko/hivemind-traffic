"""Inference-mode reset/step regression without importing Isaac or launching Kit."""
from contextlib import ContextDecorator
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]


class InferenceDouble:
    """Models only the real Torch in-place inference-tensor restriction."""
    enabled = False

    def inference_mode(self):
        owner = self
        class Context(ContextDecorator):
            def __enter__(self):
                self.before = owner.enabled
                owner.enabled = True
                return self

            def __exit__(self, *unused):
                owner.enabled = self.before
        return Context()

    def is_inference_mode_enabled(self):
        return self.enabled

    def ones(self, shape, *, requires_grad=False):
        owner = self
        class Tensor:
            def __init__(self):
                self.inference = owner.enabled
                self.requires_grad = requires_grad and not owner.enabled
                self.value = 1.

            def _guard(self):
                if self.inference and not owner.enabled:
                    raise RuntimeError('Inplace update to inference tensor outside InferenceMode')

            def zero_(self):
                self._guard(); self.value = 0.
                return self

            def add_(self, other):
                self._guard(); self.value += other.value
                return self

            def clone(self):
                value = Tensor()
                value.value = self.value
                return value
        return Tensor()


def load_boundary(monkeypatch, torch):
    class Parent:
        def reset(self, seed=None, options=None):
            self.calls.append(('reset', torch.is_inference_mode_enabled(), seed, options))
            self.actions.zero_()
            return {'policy': self.actions.clone()}, {'reset': {'seed': seed}}

        def step(self, actions):
            self.calls.append(('step', torch.is_inference_mode_enabled()))
            self.actions.add_(actions)
            if self.fail_step:
                raise ValueError('synthetic physical failure')
            return {'policy': self.actions.clone()}, 0., False, False, {'raw': 'detached'}

    monkeypatch.setitem(sys.modules, 'torch', torch)
    monkeypatch.setitem(sys.modules, 'isaaclab.utils', SimpleNamespace(configclass=lambda cls: cls))
    monkeypatch.setitem(sys.modules, 'environments.lane_following_lab',
        SimpleNamespace(LaneFollowingLabCfg=object, LaneFollowingLabEnv=Parent, ROOT=ROOT))
    monkeypatch.setitem(sys.modules, 'usd.loop_hazard_scene', SimpleNamespace(author_hazards=None))
    spec = importlib.util.spec_from_file_location('_hazard_inference_boundary', ROOT/'environments/loop_hazard_lab.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    env = module.LoopHazardLabEnv.__new__(module.LoopHazardLabEnv)
    env.calls, env.fail_step = [], False
    with torch.inference_mode():
        env.actions = torch.ones((1, 3))
    return env, Parent


def exercise_boundary(monkeypatch, torch):
    env, parent = load_boundary(monkeypatch, torch)
    assert not torch.is_inference_mode_enabled()
    # This is the exact observed old failure, with the physical tensor allocated
    # in the preceding RSL rollout's inference context.
    with pytest.raises(RuntimeError, match='[Ii]nplace update.*inference tensor'):
        parent.reset(env, seed=31001)
    observation, extras = env.reset(seed=31001, options={'perturb': True})
    assert env.calls[-1] == ('reset', True, 31001, {'perturb': True})
    assert extras == {'reset': {'seed': 31001}}
    assert not observation['policy'].requires_grad
    assert not env.actions.requires_grad
    assert not torch.is_inference_mode_enabled()
    inputs = torch.ones((1, 3), requires_grad=True)
    returned = env.step(inputs)
    assert env.calls[-1] == ('step', True)
    assert not returned[0]['policy'].requires_grad
    assert not env.actions.requires_grad
    assert not torch.is_inference_mode_enabled()
    with torch.inference_mode():
        env.reset(seed=31002)
        env.step(inputs)
        assert torch.is_inference_mode_enabled()
    assert not torch.is_inference_mode_enabled()
    env.fail_step = True
    with pytest.raises(ValueError, match='synthetic physical failure'):
        env.step(inputs)
    assert not torch.is_inference_mode_enabled()


def test_reset_and_step_preserve_inference_physical_boundary_with_cpu_double(monkeypatch):
    exercise_boundary(monkeypatch, InferenceDouble())


def test_real_cpu_torch_inference_tensor_reset_and_step(monkeypatch):
    torch = pytest.importorskip('torch', reason='Optional installed CPU Torch check; never launches Isaac')
    exercise_boundary(monkeypatch, torch)
