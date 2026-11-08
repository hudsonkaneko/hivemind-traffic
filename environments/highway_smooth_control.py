"""Versioned reward-only smooth-control experiment on the frozen v05 dynamics.

The subclass does not change observations, masks, commands, native motion,
demand, safety guards or intervention accounting. Reference changes affect
supervision and reward only. No training is started by importing this module.
"""
from __future__ import annotations

from copy import deepcopy
import math

from environments.highway_environment_v05 import (
    HighwayEnvironmentV05Env, DT, REWARD_VERSION as V05_REWARD_VERSION,
    legacy_observation, teacher_actions as v05_teacher_actions,
)
from environments.highway_stage1_env import DEFAULT_REWARD_WEIGHTS
from traffic.highway_stage1_teacher import longitudinal_target_v3
from traffic.highway_smooth_reference import reference_details, teacher_actions as smooth_teacher_actions
from traffic.highway_smooth_metrics import SmoothControlMetrics, huber

SCHEMA = 'highway_smooth_control_v01'
REWARD_VERSION = 'highway_smooth_alignment_huber_residual_v01'
OBJECTIVE_VERSIONS = ('v05', 'smooth_v01')
REFERENCE_PARAMETERS = dict(legal_speed_buffer_m_s=.1, entry_preview_time_s=2.,
    nominal_reference_jerk_m_s3=2., desired_speed_error_gain_s_inverse=.4)


def smooth_reward_terms(requested, reference, realized_jerk, previous_residual=None):
    """SI-unit reward terms; missing measured/history samples are not fabricated."""
    residual = float(requested) - float(reference)
    residual_slew = None if previous_residual is None else (residual - previous_residual) / DT
    terms = dict(feasible_acceleration=-abs(residual) / 4.5,
        jerk=-.03 * huber(realized_jerk) if realized_jerk is not None else 0.,
        command_reference_residual_slew=-.01 * huber(residual_slew) if residual_slew is not None else 0.)
    return terms, residual_slew


class HighwaySmoothControlEnv(HighwayEnvironmentV05Env):
    """The same v05 constructor plus a pinned reward/reference version.

    ``v05`` retains the exact old objective and labels. ``smooth_v01`` uses
    alignment 1, uncapped Huber jerk .03 and residual-error slew .01. Optional
    reference parameters may repeat the pinned constants but cannot tune them
    without a newly reviewed version. Extra baseline reward overrides are rejected.
    """
    metadata = dict(HighwayEnvironmentV05Env.metadata, name=SCHEMA)

    def __init__(self, network, run_directory, *, objective_version='v05', reference_parameters=None, **kwargs):
        if objective_version not in OBJECTIVE_VERSIONS:
            raise ValueError(f'objective_version must be one of {OBJECTIVE_VERSIONS}')
        if reference_parameters is not None:
            for key, value in reference_parameters.items():
                if key not in REFERENCE_PARAMETERS or isinstance(value, bool) or not isinstance(value, (int, float)) or value != REFERENCE_PARAMETERS[key]:
                    raise ValueError('Reference parameters are pinned for smooth_v01')
        if kwargs.get('reward_weights') and any(
                key not in DEFAULT_REWARD_WEIGHTS or value != DEFAULT_REWARD_WEIGHTS[key]
                for key, value in kwargs['reward_weights'].items()):
            raise ValueError('Smooth experiment requires unchanged baseline reward weights')
        self.objective_version = objective_version
        self.reference_parameters = dict(REFERENCE_PARAMETERS)
        super().__init__(network, run_directory, **kwargs)
        if objective_version == 'smooth_v01':
            self.reward_weights.update(feasible_acceleration=1., jerk=.03,
                command_reference_residual_slew=.01)
        self._reset_smooth_tracking()

    def _reset_smooth_tracking(self):
        self._smooth_step_in_progress = False
        self._previous_reference_residual = {}
        self._smooth_before, self._smooth_references = {}, {}
        self.smooth_metrics = SmoothControlMetrics(task=self.task,
            merge_vehicle_ids=getattr(self, 'merge_agents', ()),
            controlled_ids=self._controlled_agents(),
            window_s=getattr(self, 'stabilization_seconds', 8.))

    def reset(self, seed=None, options=None):
        if options and set(options) & {'objective_version', 'reference_parameters'}:
            raise ValueError('Smooth objective/reference is fixed at construction')
        self._reset_smooth_tracking()
        result = super().reset(seed=seed, options=options)
        self.smooth_metrics.merge_vehicle_ids.update(self.merge_agents)
        self.smooth_metrics.prime(self.time, self.states)
        return result

    def teacher_actions(self, observations=None):
        observations = self.obs if observations is None else observations
        teacher = smooth_teacher_actions if self.objective_version == 'smooth_v01' else v05_teacher_actions
        return teacher(observations)

    def step(self, actions):
        # Only current actor observations enter reference labels. Copies protect
        # measurement endpoints from a native adapter updating a state in place.
        self._smooth_before = deepcopy(self.states)
        self._smooth_before_time = self.time
        self._smooth_references = {
            a: reference_details(self.obs[a]) if self.objective_version == 'smooth_v01'
            else dict(version='existing_v3', acceleration_m_s2=longitudinal_target_v3(legacy_observation(self.obs[a])))
            for a in self.agents}
        self._smooth_step_in_progress = True
        try:
            result = super().step(actions)
        finally:
            self._smooth_step_in_progress = False
        infos = result[4]
        self.smooth_metrics.observe(self.time, self._smooth_before, self.states, infos,
            merge_vehicle_ids=self.merge_agents, arrived=self.arrived)
        if not infos:
            self._previous_reference_residual.clear()
        return result

    def _advance(self):
        # Primary accounting drains through _advance directly after the last
        # controlled car finishes. Record that native tail exactly once; normal
        # step observations wait for command/reward diagnostics above.
        measure_tail = (getattr(self, '_measurement_phase', False)
                        and not getattr(self, '_smooth_step_in_progress', False)
                        and hasattr(self, 'smooth_metrics'))
        before = deepcopy(self.states) if measure_tail else None
        super()._advance()
        if measure_tail:
            self.smooth_metrics.observe(self.time, before, self.states, {},
                merge_vehicle_ids=self.merge_agents, arrived=self.arrived)
            self._previous_reference_residual.clear()

    def _update_merge_metrics(self, agents, infos, rewards, terminated, truncated):
        # This hook runs before v05 serializes the journal. Returned rewards,
        # journal terms and cumulative statistics therefore remain consistent.
        super()._update_merge_metrics(agents, infos, rewards, terminated, truncated)
        new_history = {}
        for agent in agents:
            info, old = infos[agent], self._smooth_before[agent]
            reference = self._smooth_references[agent]
            target = reference['acceleration_m_s2']
            measured = info['realized_acceleration']
            native_jerk = (measured - old['acceleration']) / DT if measured is not None else None
            previous = self._previous_reference_residual.get(agent)
            previous_residual = (previous['residual'] if previous is not None
                and math.isclose(previous['time_s'], self._smooth_before_time, abs_tol=1e-7) else None)
            replacement, residual_slew = smooth_reward_terms(info['requested_acceleration'], target, native_jerk, previous_residual)
            if self.objective_version == 'smooth_v01':
                self.stats['acceleration_error_sum'] += (
                    abs(info['requested_acceleration'] - target)
                    - abs(info['requested_acceleration'] - info['feasible_acceleration_target']))
                for key, value in replacement.items():
                    delta = value - info['reward_terms'].get(key, 0.)
                    info['reward_terms'][key] = value
                    rewards[agent] += delta
                    self.stats['reward_terms'][key] = self.stats['reward_terms'].get(key, 0.) + delta
                info['feasible_acceleration_target'] = target
                info['feasible_target_source'] = 'smooth_v01 current-observation soft reference; labels/reward only'
            info.update(objective_version=self.objective_version,
                smooth_reference=reference, realized_jerk_m_s3=native_jerk,
                command_reference_residual_slew_m_s3=residual_slew,
                residual_history_available=previous_residual is not None)
            if not terminated[agent] and not truncated[agent]:
                new_history[agent] = dict(time_s=self.time, residual=info['requested_acceleration'] - target)
        self._previous_reference_residual = new_history

    def report(self):
        report = super().report()
        report.update(schema=SCHEMA, objective_version=self.objective_version,
            reward_version=REWARD_VERSION if self.objective_version == 'smooth_v01' else V05_REWARD_VERSION,
            reference_parameters=dict(self.reference_parameters),
            smooth_control=self.smooth_metrics.report(),
            objective_scope='Only reward/reference labels differ; v05 actor ABI, native commands, guards and intervention counts unchanged')
        return report
