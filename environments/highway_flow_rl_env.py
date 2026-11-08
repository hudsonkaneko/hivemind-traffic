"""Experimental teacher-free reward on flow117 observations and native v05 control.

This separate environment does not qualify the flow reference or change its
prepared imitation experiment. Native v05 still computes its inexpensive legacy
target internally; that unused diagnostic is removed before rewards are journaled.
The full flow gap planner is never called. No training starts on import.
"""
from __future__ import annotations

from copy import deepcopy
import math

import numpy as np

from environments.highway_environment_v05 import HighwayEnvironmentV05Env, DT
from environments.highway_flow_env import HighwayFlowEnv, OBSERVATION_SIZE
from traffic.highway_flow_context import legacy_observation


SCHEMA = 'highway_flow_rl_teacher_free_v1'
REWARD_VERSION = 'highway_flow_rl_physical_objective_v1'
DEFAULT_JERK_REWARD_WEIGHT = .01
REMOVED_REWARD_TERMS = ('feasible_acceleration',)


class HighwayFlowRLEnv(HighwayFlowEnv):
    """Flow117 policy environment with no teacher labels or target reward.

    Only ``jerk_reward_weight`` may change the otherwise preserved physical
    objective. It weights absolute measured longitudinal jerk without clipping
    emergencies, adding a comfort filter, or changing the hard safety checks.
    ``unsafe_request`` remains: it measures excess over the current analytical
    safety ceiling, independently of the removed teacher target.
    """
    metadata = dict(HighwayFlowEnv.metadata, name=SCHEMA)

    def __init__(self, network, run_directory, *, jerk_reward_weight=DEFAULT_JERK_REWARD_WEIGHT, **kwargs):
        if (isinstance(jerk_reward_weight, bool)
                or not isinstance(jerk_reward_weight, (int, float))
                or not math.isfinite(jerk_reward_weight) or jerk_reward_weight < 0.):
            raise ValueError('jerk_reward_weight must be finite and nonnegative')
        self.jerk_reward_weight = float(jerk_reward_weight)
        # Parent validation pins all other baseline reward coefficients.
        super().__init__(network, run_directory, **kwargs)
        self.reward_weights.update(feasible_acceleration=0., jerk=self.jerk_reward_weight)

    def teacher_actions(self, observations=None):
        raise RuntimeError('Teacher-free flow RL does not provide imitation labels')

    def scripted_actions(self):
        raise RuntimeError('Teacher-free flow RL requires explicit policy or native actions')

    def step(self, actions):
        original = self.obs
        self._flow_before = deepcopy(self.states)
        self._flow_before_context = deepcopy(self.flow_contexts)
        self.obs = {agent: legacy_observation(obs) for agent, obs in original.items()}
        self._flow_step_active = True
        try:
            # Bypass HighwayFlowEnv.step, including its reference planner.
            # Dynamic hooks retain flow observations, native masks and commands.
            result = HighwayEnvironmentV05Env.step(self, actions)
        except BaseException:
            self.obs = original
            raise
        finally:
            self._flow_step_active = False
        observations, rewards, terminated, truncated, infos = result
        observations = {agent: obs if obs.shape == (OBSERVATION_SIZE,)
                        else np.zeros(OBSERVATION_SIZE, np.float32)
                        for agent, obs in observations.items()}
        self.flow_metrics.observe(self.time, self._flow_before, self.states, infos,
            merge_vehicle_ids=self.merge_agents, arrived=self.arrived)
        return observations, rewards, terminated, truncated, infos

    def _update_merge_metrics(self, agents, infos, rewards, terminated, truncated):
        # This hook runs before the base class writes its journal. Retain native
        # route/stable-merge accounting while bypassing the flow-reference hook.
        HighwayEnvironmentV05Env._update_merge_metrics(self, agents, infos, rewards, terminated, truncated)
        for agent in agents:
            info = infos[agent]
            removed = info['reward_terms'].pop('feasible_acceleration')
            rewards[agent] -= removed
            info.pop('feasible_acceleration_target')
            info.pop('feasible_target_source')
            measured = info['realized_acceleration']
            old = self._flow_before[agent]
            info.update(objective_version=REWARD_VERSION, teacher_reference_used=False,
                realized_jerk_m_s3=(measured - old['acceleration']) / DT if measured is not None else None,
                jerk_reward_weight=self.jerk_reward_weight,
                flow_context=self._flow_before_context[agent],
                fixture_events=deepcopy(self.fixture_event_log))
        self.stats['reward_terms'].pop('feasible_acceleration', None)
        # The base class needs this accumulator, but no teacher-error metric is
        # meaningful for this objective. It is explicitly null in report().
        self.stats['acceleration_error_sum'] = 0.

    def report(self):
        result = super().report()
        result.update(schema=SCHEMA, reward_version=REWARD_VERSION,
            objective_version=REWARD_VERSION, control_target_version=None,
            mean_feasible_acceleration_error=None,
            teacher_reference_used=False, imitation_labels_available=False,
            legacy_target_computed_and_discarded=True,
            removed_reward_terms=list(REMOVED_REWARD_TERMS),
            jerk_reward_weight=self.jerk_reward_weight,
            objective_scope='Native progress/events, analytical unsafe-request and measured safety/comfort terms; no teacher alignment or imitation',
            unsafe_request_source='Excess over analytical current-state/map safety ceiling, not teacher target')
        result['reward_weights'] = {key: value for key, value in self.reward_weights.items()
                                   if key not in REMOVED_REWARD_TERMS}
        return result
