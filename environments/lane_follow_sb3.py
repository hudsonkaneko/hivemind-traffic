"""Project-local SB3 terminal-observation correction; importing this starts no Kit.

The installed Lab wrapper builds ``terminal_observation`` from the returned
post-autoreset observation, in both its fast and slow variants. This wrapper
retains SB3's reset observation but replaces terminal metadata with the task's
detached pre-reset snapshot. No optimizer, training, or runtime patch is involved.
"""
from __future__ import annotations

from collections.abc import Mapping, Sequence
import numpy as np


def _array(value):
    """Accept NumPy data or detached Torch-like tensors without importing Torch."""
    if hasattr(value, 'detach'):
        value = value.detach().cpu().numpy()
    return np.asarray(value)


def _boolean_vector(value, name, count=None):
    array = _array(value)
    if array.dtype != np.bool_ or array.ndim != 1 or not len(array):
        raise ValueError(f'{name} must be a nonempty one-dimensional boolean array')
    if count is not None and array.shape != (count,):
        raise ValueError(f'{name} shape differs from the environment batch')
    return array


def terminal_infos(infos, terminated, truncated, extras, *, observation_shape):
    """Return copied SB3 infos with validated terminal snapshots.

    ``observation_shape`` is the single flat policy observation shape. All steps
    must supply the task's final snapshot and flags, including nonterminal steps;
    the mask must exactly equal ``terminated | truncated``. This rejects missing
    or stale terminal data instead of silently bootstrapping from reset state.
    A true termination takes precedence over a simultaneous time limit.
    """
    terminated = _boolean_vector(terminated, 'terminated')
    count = len(terminated)
    truncated = _boolean_vector(truncated, 'truncated', count)
    if (not isinstance(infos, Sequence) or len(infos) != count
            or any(not isinstance(info, Mapping) for info in infos)):
        raise ValueError('infos must contain one mapping per environment')
    if (not isinstance(observation_shape, tuple) or len(observation_shape) != 1
            or not isinstance(observation_shape[0], (int, np.integer))
            or isinstance(observation_shape[0], (bool, np.bool_))
            or observation_shape[0] <= 0):
        raise ValueError('Only a nonempty flat policy observation is supported')
    required = ('final_observation', '_final_observation',
                'final_terminated', 'final_truncated')
    if not isinstance(extras, Mapping) or any(key not in extras for key in required):
        raise ValueError('Task final-observation snapshot or terminal flags are missing')
    mask = _boolean_vector(extras['_final_observation'], '_final_observation', count)
    for key, expected in [('final_terminated', terminated), ('final_truncated', truncated)]:
        if not np.array_equal(_boolean_vector(extras[key], key, count), expected):
            raise ValueError(f'{key} disagrees with this step; stale terminal metadata')
    done = terminated | truncated
    if not np.array_equal(mask, done):
        raise ValueError('_final_observation mask must exactly equal terminated | truncated')
    snapshots = extras['final_observation']
    if not isinstance(snapshots, Mapping) or set(snapshots) != {'policy'}:
        raise ValueError('final_observation must contain exactly the policy observation')
    terminal = _array(snapshots['policy'])
    if terminal.shape != (count, *observation_shape) or terminal.dtype.kind != 'f':
        raise ValueError('Final policy observation must be a correctly shaped floating-point array')
    # Conversion to float32 must not turn a finite larger float into infinity.
    with np.errstate(over='ignore', invalid='ignore'):
        terminal = terminal.astype(np.float32, copy=True)
    if not np.isfinite(terminal).all():
        raise ValueError('Final policy observation must remain finite in float32')
    result = []
    for index, original in enumerate(infos):
        info = dict(original)
        info.pop('terminal_observation', None)
        info['TimeLimit.truncated'] = bool(truncated[index] and not terminated[index])
        if done[index]:
            info['terminal_observation'] = terminal[index].copy()
        result.append(info)
    return result


def make_lane_follow_sb3_wrapper(env):
    """Wrap the one-car task after Kit starts; never create a trainer or optimizer.

    The installed wrapper remains untouched. The fast variant is deliberately
    used so nested task diagnostics are not indexed by its generic slow path.
    The task's raw extras remain available to validation outside this wrapper.
    """
    if env.unwrapped.num_envs != 1:
        raise ValueError('Lane-following SB3 boundary is validated for one environment only')
    from isaaclab_rl.sb3 import Sb3VecEnvWrapper

    class LaneFollowSb3VecEnvWrapper(Sb3VecEnvWrapper):
        def _process_extras(self, obs, terminated, truncated, extras, reset_ids):
            infos = super()._process_extras(obs, terminated, truncated, extras, reset_ids)
            return terminal_infos(infos, terminated, truncated, extras,
                                  observation_shape=self.observation_space.shape)

    return LaneFollowSb3VecEnvWrapper(env, fast_variant=True)
