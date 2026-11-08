"""RSL-RL models and evidence boundary for the versioned native hazard task."""
import time
import torch
from rsl_rl.models import MLPModel
from environments.lane_follow_rsl import signed_vehicle_actions


class FixedRoadMLP(MLPModel):
    """Physical feature scales, identical during PPO, evaluation and JIT export.

    These declared scales avoid a tiny observed variance turning wheel-height
    or rarely active object channels into extreme deployment inputs.
    """
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        if self.obs_dim != 42 or not self.obs_normalization:
            raise ValueError('FixedRoadMLP requires the42-field normalized hazard contract')
        mean = torch.zeros(42)
        mean[4] = mean[10] = 1.0
        scale = torch.tensor([60., 60., .5, .2, .2, 3., .5, 3., .2, 1., .1,
                              .02, .02, .02, 3., 3., .5, 1., 1., 1., 1., 1.]
                             + [1., 30., 5., 5., 5., 1., 1., 1., 1., .2] * 2)
        with torch.no_grad():
            self.obs_normalizer._mean.copy_(mean.unsqueeze(0))
            self.obs_normalizer._std.copy_(scale.unsqueeze(0))
            self.obs_normalizer._var.copy_(scale.square().unsqueeze(0))
        self.obs_normalizer.eps = 0.0

    def update_normalization(self, obs):
        pass  # Fixed engineering scales are part of the versioned model.


def wrap_hazard_rsl(env, *, deadline, on_transition=None):
    import numpy as np
    from gymnasium.spaces import Box
    from tensordict import TensorDict
    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper

    class HazardRslWrapper(RslRlVecEnvWrapper):
        def __init__(self, task):
            super().__init__(task, clip_actions=None)
            self.num_actions = 2
            self.control_steps = 0

        @property
        def action_space(self):
            return Box(-1., 1., shape=(2,), dtype=np.float32)

        def step(self, actions):
            if time.monotonic() > deadline:
                raise TimeoutError('Hazard training deadline reached')
            obs, rewards, dones, extras = super().step(signed_vehicle_actions(actions))
            self.control_steps += 1
            if on_transition is not None:
                on_transition(extras['lane_transition'], actions.detach().cpu(), extras.get('reset'))
            terminal = TensorDict(extras['final_observation'], batch_size=[1]).clone()
            return obs, rewards, dones, {
                'time_outs': extras['final_truncated'] & ~extras['final_terminated'],
                'lane_terminal_observation': terminal,
            }

    return HazardRslWrapper(env)
