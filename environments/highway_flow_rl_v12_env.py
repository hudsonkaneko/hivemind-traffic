"""V12 merge environment: v11 plus the gate's target speed as actor input and a band reward.

Changes (traffic/highway_flow_rl_v12_rewards.py):
- the actor observation grows from 117 to 121 fields: four target inputs inserted at
  69..72, before the six message slots; the critic state grows to 24 x 122 = 2928;
- one reward term, target_band, on the same target and the same post-decision speed
  as the inherited merge_tracking term.
Dynamics, safety shield, native guards, the lane-change mask (including the v11
merge-speed precondition), action contract, gates and every earlier reward term are
unchanged. The safety guard, mask and journaled observation117 keep the raw
117-field observation; only the actor and critic see the extra inputs.
"""
from copy import deepcopy

import gymnasium as gym
import numpy as np

from environments.highway_flow_rl_v11_env import HighwayFlowRLV11Env
from traffic import highway_flow_rl_v12_rewards as v12
from traffic.highway_flow_rl_v03_rewards import map_evidence
from traffic.highway_flow_rl_v04_rewards import merge_speed_target

OBSERVATION_SIZE = 121
STATE_SIZE = 24 * (OBSERVATION_SIZE + 1)
RAW_SPACE = gym.spaces.Box(-1., 1., (117,), np.float32)
ACTOR_SPACE = gym.spaces.Box(-1., 1., (OBSERVATION_SIZE,), np.float32)


class HighwayFlowRLV12Env(HighwayFlowRLV11Env):
    def __init__(self, *args, v12_settings=None, **kwargs):
        self.v12_settings = v12.validate_settings(v12_settings or v12.SETTINGS)
        super().__init__(*args, **kwargs)
        self.state_space = gym.spaces.Box(-1., 1., (STATE_SIZE,), np.float32)

    def observation_space(self, agent):
        """The 121-field actor space; the inherited raw-vector check inside _observations sees 117."""
        if getattr(self, '_building_raw', False):
            return RAW_SPACE
        return ACTOR_SPACE

    def _observations(self):
        self._building_raw = True
        try:
            return super()._observations()
        finally:
            self._building_raw = False

    def _reset_v06(self):
        super()._reset_v06()
        self._target_inputs, self._augmented = {}, {}
        self.v12_metrics = dict(decisions=0, band_decisions=0, in_band_decisions=0, band_reward=0.,
                                per_car={})

    def reset(self, seed=None, options=None):
        observations, infos = super().reset(seed=seed, options=options)
        return self._augment(observations), infos

    def step(self, actions):
        observations, rewards, terminated, truncated, infos = super().step(actions)
        return self._augment(observations), rewards, terminated, truncated, infos

    def state(self):
        """Critic state from the same (faded, augmented) observations the actors receive."""
        return np.concatenate([np.r_[self._augmented[a] if a in self.agents else np.zeros(OBSERVATION_SIZE),
                                     float(a in self.agents)] for a in self.possible_agents]).astype(np.float32)

    def current_target(self, agent):
        """The gate's merge-speed target for this car's next decision, and whether its merge is pending."""
        state = self.states.get(agent)
        context = getattr(self, 'flow_contexts', {}).get(agent)
        tracker = getattr(self, 'merge_trackers', {}).get(agent)
        pending = bool(tracker and tracker.status == 'pending')
        if state is None or context is None or not pending:
            return None, pending, state
        target, _ = merge_speed_target(state, context, map_evidence(self.graph, state))
        return target, pending, state

    def _augment(self, observations):
        result = {}
        for agent, observation in observations.items():
            target, pending, state = self.current_target(agent)
            raw = v12.target_features(target, state['speed'] if state else None, pending, self.v12_settings)
            if agent in self.agents:
                features = v12.fade(self._target_inputs.get(agent), raw, self.v12_settings)
                self._target_inputs[agent] = features
            else:  # final observation of a car that has left: raw, never faded or stored
                features = raw
            result[agent] = v12.insert(observation, features)
            if not ACTOR_SPACE.contains(result[agent]):
                raise ValueError('Invalid v12 actor observation')
            if agent in self.agents:
                self._augmented[agent] = result[agent]
        return result

    def _extra_rewards(self, agent, info, snapshot, audit, state, merge_active):
        terms = super()._extra_rewards(agent, info, snapshot, audit, state, merge_active)
        speed = state['speed'] if state else None
        terms['target_band'] = v12.target_band(snapshot['target'], speed, merge_active, self.v12_settings)
        totals = self.v06_metrics['reward_terms']
        totals['target_band'] = totals.get('target_band', 0.) + terms['target_band']
        value = v12.active_target(snapshot['target'], merge_active)
        metrics = self.v12_metrics
        metrics['decisions'] += 1
        if value is not None and speed is not None:
            inside = abs(speed - value) <= self.v12_settings['band_tolerance_m_s']
            car = metrics['per_car'].setdefault(agent, dict(band_decisions=0, in_band_decisions=0,
                band_reward=0., max_abs_error_m_s=0., first_time_s=None, last_time_s=None))
            for row in (metrics, car):
                row['band_decisions'] += 1
                row['in_band_decisions'] += int(inside)
                row['band_reward'] += terms['target_band']
            car['max_abs_error_m_s'] = max(car['max_abs_error_m_s'], abs(speed - value))
            if car['first_time_s'] is None:
                car['first_time_s'] = round(snapshot['time_s'], 1)
            car['last_time_s'] = round(snapshot['time_s'], 1)
        info['v12_audit'] = dict(target_m_s=value, speed_m_s=speed, target_band=terms['target_band'],
                                 inputs=self._target_inputs.get(agent, np.zeros(4)).tolist())
        return terms

    def report(self):
        result = super().report()
        metrics = deepcopy(self.v12_metrics)
        for row in (metrics, *metrics['per_car'].values()):
            row['in_band_fraction'] = (row['in_band_decisions'] / row['band_decisions']
                                       if row['band_decisions'] else None)
        result.update(schema='highway_flow_rl_merge_env_v12', reward_version=v12.REWARD_VERSION,
            objective_version=v12.REWARD_VERSION, observation_version=v12.OBSERVATION_VERSION,
            observation_size=OBSERVATION_SIZE, state_size=STATE_SIZE, v12_settings=dict(self.v12_settings),
            v12_metrics=metrics,
            v12_scope='v11 environment and terms; adds four target-speed actor inputs and the target_band term')
        return result
