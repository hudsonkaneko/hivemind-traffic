"""RSL-RL boundary for the one-car task; import after native runtime startup."""
import torch
from rsl_rl.algorithms import PPO


def signed_vehicle_actions(actions):
    if actions.ndim != 2 or actions.shape[1] != 2 or not bool(torch.isfinite(actions).all()):
        raise ValueError('Expected finite (num_envs, 2) policy actions')
    values = actions.clamp(-1., 1.)
    return torch.stack((values[:, 0] * .5, values[:, 1].clamp_min(0),
                        (-values[:, 1]).clamp_min(0)), dim=1)


class TerminalObservationPPO(PPO):
    """Bootstrap timeouts from their pre-reset state, never from the reset state."""
    def process_env_step(self, obs, rewards, dones, extras):
        extras = dict(extras)
        timeouts = extras.pop('time_outs', None)
        terminal = extras.pop('lane_terminal_observation', None)
        if timeouts is not None and bool(timeouts.any()):
            if terminal is None:
                raise ValueError('Timeout requires a detached terminal observation')
            with torch.no_grad():
                values = self.critic(terminal.to(self.device)).squeeze(-1)
            rewards = rewards + self.gamma * values * timeouts.to(self.device)
        # Removing time_outs prevents RSL-RL's default current-state bootstrap
        # from being added a second time. True terminations receive no bootstrap.
        super().process_env_step(obs, rewards, dones, extras)


def wrap_lane_rsl(env, *, deadline):
    import time
    import numpy as np
    from gymnasium.spaces import Box
    from tensordict import TensorDict
    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper

    class LaneRslWrapper(RslRlVecEnvWrapper):
        def __init__(self, task):
            super().__init__(task, clip_actions=None)
            self.num_actions = 2
            self.control_steps = 0

        @property
        def action_space(self):
            return Box(-1., 1., shape=(2,), dtype=np.float32)

        def step(self, actions):
            if time.monotonic() > deadline:
                raise TimeoutError('Training time limit reached; completed checkpoints retained')
            obs, rewards, dones, extras = super().step(signed_vehicle_actions(actions))
            self.control_steps += 1
            timeouts = extras['final_truncated'] & ~extras['final_terminated']
            terminal = TensorDict(extras['final_observation'], batch_size=[self.num_envs]).clone()
            return obs, rewards, dones, {'time_outs': timeouts,
                                        'lane_terminal_observation': terminal}

    return LaneRslWrapper(env)
