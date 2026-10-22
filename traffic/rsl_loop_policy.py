"""Hash-pinned RSL JIT inference; no optimizer, scene stepping or pose ownership.

The trusted exported actor includes its observation normalizer. Inputs are the
new 42-value privileged-current-object contract, never the old 22-value policy.
The caller owns DriverControlGate at 120 Hz and native wheel actuation.
"""
from __future__ import annotations

import hashlib
import io
import math
from numbers import Real
from pathlib import Path
import re

from traffic.driver_control import DriverCommand

ROOT = Path(__file__).resolve().parents[1]
OBSERVATION_SIZE = 42
ACTION_SIZE = 2
SOURCE = 'rsl_rl_jit_embedded_normalization_privileged_current_objects'


def _values(values, length, name):
    if not isinstance(values, (list, tuple)) or len(values) != length:
        raise ValueError(f'{name} must contain exactly {length} values')
    if any(isinstance(v, bool) or not isinstance(v, Real) or not math.isfinite(v) for v in values):
        raise ValueError(f'{name} must contain finite numbers')
    return tuple(float(v) for v in values)


def physical_action(raw):
    """Two normalized actions -> steering radians and exclusive forward/brake."""
    raw = _values(raw, ACTION_SIZE, 'Policy output')
    clipped = tuple(max(-1., min(1., v)) for v in raw)
    return ((.5 * clipped[0], max(0., clipped[1]), max(0., -clipped[1])),
            clipped, tuple(a != b for a, b in zip(raw, clipped)))


class RslLoopPolicy:
    def __init__(self, policy_path, expected_sha256, *, episode_id, vehicle_id='ego', device='cpu'):
        path = Path(policy_path).absolute()
        if (not path.is_relative_to(ROOT / 'outputs') or path.resolve() != path
                or any(p.is_symlink() for p in (path, *path.parents)) or not path.is_file()):
            raise ValueError('Policy must be a regular trusted local file under project outputs')
        if not isinstance(expected_sha256, str) or not re.fullmatch(r'[0-9a-f]{64}', expected_sha256):
            raise ValueError('Expected an explicit lowercase SHA256')
        if device != 'cpu':
            raise ValueError('This deployment profile uses CPU inference')
        payload = path.read_bytes()
        actual = hashlib.sha256(payload).hexdigest()
        if actual != expected_sha256:
            raise ValueError('Policy SHA256 mismatch before deserialization')
        import torch
        self._torch = torch
        # Load the exact verified bytes, avoiding a hash/load path race.
        self._actor = torch.jit.load(io.BytesIO(payload), map_location='cpu').eval()
        self.metadata = dict(source=SOURCE, policy_path=str(path.relative_to(ROOT)),
            policy_sha256=actual, observation_size=OBSERVATION_SIZE, action_size=ACTION_SIZE,
            normalizer='embedded in exported actor; no external normalization',
            action_mapping='steering*.5; positive longitudinal throttle; negative longitudinal brake',
            motion_owner='native PhysX; adapter emits DriverCommand only', device=device)
        self.episode_id = None
        self.reset(episode_id, vehicle_id=vehicle_id)
        self.predict([0.] * OBSERVATION_SIZE)  # Shape/finite smoke without stepping physics.
        self.diagnostics = dict(status='ready', **self.metadata)

    def reset(self, episode_id, *, vehicle_id='ego'):
        if any(not isinstance(v, str) or not v.strip() for v in (episode_id, vehicle_id)):
            raise ValueError('Nonempty episode and vehicle identities required')
        if episode_id == self.episode_id:
            raise ValueError('Reset requires a new episode identity')
        self.episode_id, self.vehicle_id = episode_id, vehicle_id
        self._sequence, self._last_tick = 0, None
        self.diagnostics = dict(status='reset', source=SOURCE)
        if hasattr(self._actor, 'reset'):
            self._actor.reset()

    def predict(self, observation):
        values = _values(observation, OBSERVATION_SIZE, 'Observation')
        torch = self._torch
        tensor = torch.tensor([values], dtype=torch.float32, device='cpu')
        if not bool(torch.isfinite(tensor).all()):
            raise ValueError('Observation exceeds finite float32 range')
        with torch.inference_mode():
            output = self._actor(tensor)
        if not isinstance(output, torch.Tensor) or tuple(output.shape) != (1, ACTION_SIZE):
            raise ValueError('Actor must return one tensor with shape [1,2]')
        if not bool(torch.isfinite(output).all()):
            raise ValueError('Actor returned nonfinite action')
        raw = tuple(float(v) for v in output[0].detach().cpu().tolist())
        physical, clipped, mask = physical_action(raw)
        self.diagnostics = dict(status='inference', source=SOURCE, raw_action=raw,
            clipped_action=clipped, clipped_components=mask, clipping_count=sum(mask),
            physical_action=physical, normalization='embedded_once')
        return physical

    def command(self, observation, *, tick, observation_tick, stop=False):
        if (type(tick) is not int or tick < 0 or tick % 2
                or type(observation_tick) is not int or observation_tick != tick
                or self._last_tick is not None and tick <= self._last_tick
                or type(stop) is not bool):
            raise ValueError('Commands need fresh observations and increasing 60 Hz ticks')
        values = (0., 0., 1.) if stop else self.predict(observation)
        if stop:
            self.diagnostics = dict(status='requested_stop', source=SOURCE,
                                    physical_action=values, clipping_count=0)
        command = DriverCommand(self.episode_id, self.vehicle_id, self._sequence,
                                tick, tick + 12, *values)
        self._sequence += 1
        self._last_tick = tick
        self.diagnostics.update(tick=tick, observation_tick=observation_tick,
                                episode_id=self.episode_id, vehicle_id=self.vehicle_id)
        return command
