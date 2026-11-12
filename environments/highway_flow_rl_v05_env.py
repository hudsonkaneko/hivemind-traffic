"""V05 actuator continuity with independently scored legacy acceptance targets."""
from copy import deepcopy

from environments.highway_flow_rl_v04_env import HighwayFlowRLV04Env
from traffic.highway_flow_rl_v04_rewards import gap_plan, merge_speed_target, DEFAULT_WEIGHTS
from traffic.highway_flow_rl_v05_rewards import (REWARD_VERSION, SETTINGS, WEIGHTS,
    condition_acceleration, stable_target, extra_rewards)
from traffic.highway_flow_context import flow_context


# 'startup' (Nov 11 2026): legacy v04 behavior except during control handoff.
# A car enters the episode at its SUMO insertion acceleration (0 m/s2) and the
# actor's first command used to be applied in one 0.1 s step (12 m/s3 jerk).
# The same comfort rate limit as 'continuity' (4 m/s3, max braking and lower
# safety ceilings bypass it) applies only until the realized command catches
# up with the actor's request, or for at most HANDOFF_MAX_STEPS decisions.
HANDOFF_MAX_STEPS = 30
VARIANTS = ('legacy', 'continuity', 'full', 'startup')


class HighwayFlowRLV05Env(HighwayFlowRLV04Env):
    def __init__(self, *args, v05_variant='full', v05_weights=None, **kwargs):
        if v05_variant not in VARIANTS:
            raise ValueError('Unknown v05 variant')
        self.v05_variant = v05_variant
        self.v05_weights = dict(WEIGHTS if v05_weights is None else v05_weights)
        if self.v05_weights != WEIGHTS:
            raise ValueError('Unreviewed v05 reward weights')
        kwargs['v04_reward_weights'] = ({k:self.v05_weights[k] for k in DEFAULT_WEIGHTS}
                                        if v05_variant == 'full' else dict(DEFAULT_WEIGHTS))
        self._reset_v05()
        super().__init__(*args, **kwargs)

    def _reset_v05(self):
        self._planner, self._reference_targets, self._conditioning = {}, {}, {}
        self._v05_seen = set()
        self._handoff = {}  # agent -> handoff decisions so far, or None once complete
        self.v05_metrics = dict(per_car={}, phases={}, decisions=0, conditioning_changes=0,
            safety_or_native_changes=0, combined_changes=0, safety_exceptions=0,
            reward_terms={}, acceptance_target='unchanged_v04_reference',
            handoff=dict(cars=0, conditioned_decisions=0, completed_by_catch_up=0, completed_by_limit=0))

    def reset(self, seed=None, options=None):
        self._reset_v05()
        return super().reset(seed=seed, options=options)

    def _handoff_active(self, agent):
        if agent not in self._handoff:
            self._handoff[agent] = 0
            self.v05_metrics['handoff']['cars'] += 1
        return self._handoff[agent] is not None

    def _condition_acceleration(self, agent, requested, ceiling):
        if self.v05_variant == 'startup':
            enabled = self._handoff_active(agent)
        else:
            enabled = self.v05_variant != 'legacy'
        evidence = condition_acceleration(requested, self.states[agent]['acceleration'], ceiling,
            enabled=enabled)
        evidence['handoff'] = bool(self.v05_variant == 'startup' and enabled)
        if evidence['handoff']:
            handoff = self.v05_metrics['handoff']
            self._handoff[agent] += 1
            handoff['conditioned_decisions'] += int(evidence['changed'])
            if not evidence['changed']:
                self._handoff[agent] = None  # caught up: legacy behavior from here on
                handoff['completed_by_catch_up'] += 1
            elif self._handoff[agent] >= HANDOFF_MAX_STEPS:
                self._handoff[agent] = None
                handoff['completed_by_limit'] += 1
        self._conditioning[agent] = evidence
        return evidence['conditioned']

    def _reward_target(self, agent, state, context, preview):
        if self.v05_variant == 'full':
            target, plan, reference = stable_target(state, context, preview, self._planner.get(agent))
            self._planner[agent] = dict(target=deepcopy(target), plan=deepcopy(plan))
        else:
            target, plan = merge_speed_target(state, context, preview)
            reference = target
        self._reference_targets[agent] = reference
        return target, plan

    def _acceptance_target(self, agent, snapshot):
        return self._reference_targets[agent]

    def _successor_plan(self, agent, state):
        if not state:
            return dict(potential=0.)
        return gap_plan(state, flow_context(self.graph,state,self.states),
            previous_plan=self._planner.get(agent,{}).get('plan') if self.v05_variant == 'full' else None,
            switch_margin=SETTINGS['slot_switch_margin'])

    def _update_merge_metrics(self, agents, infos, rewards, terminated, truncated):
        super()._update_merge_metrics(agents, infos, rewards, terminated, truncated)
        for agent in agents:
            info, snapshot = infos[agent], self._v02_pre_action[agent]
            audit = self._conditioning[agent]
            state, tracker = self.states.get(agent), self.merge_trackers.get(agent)
            merge_active = bool(tracker and tracker.status == 'pending')
            extra = extra_rewards(info['realized_jerk_m_s3'], audit['raw_requested_jerk'],
                snapshot['target'], state['speed'] if state else None, merge_active=merge_active,
                weights=self.v05_weights) if self.v05_variant == 'full' else {}
            rewards[agent] += sum(extra.values())
            info['reward_terms'].update(extra)
            for key,value in extra.items():
                self.stats['reward_terms'][key] = self.stats['reward_terms'].get(key,0.)+value
                self.v05_metrics['reward_terms'][key] = self.v05_metrics['reward_terms'].get(key,0.)+value
            phase = ('startup' if agent not in self._v05_seen else
                     'braking_recovery' if audit['measured'] < -.2 and audit['requested'] > audit['measured'] else
                     'merge' if merge_active else 'normal')
            self._v05_seen.add(agent)
            non_comfort = bool(info['override'] and any(c != 'comfort_rate_limit' for c in info['override_causes']))
            for row in (self.v05_metrics, self.v05_metrics['per_car'].setdefault(agent,{}),
                        self.v05_metrics['phases'].setdefault(phase,{})):
                for key,value in dict(decisions=1, conditioning_changes=int(audit['changed']),
                    safety_or_native_changes=int(non_comfort), combined_changes=int(info['override']),
                    safety_exceptions=int(audit['safety_exception'])).items():
                    row[key] = row.get(key,0)+value
                if info['realized_jerk_m_s3'] is not None:
                    row['peak_abs_jerk_m_s3'] = max(row.get('peak_abs_jerk_m_s3',0.),abs(info['realized_jerk_m_s3']))
                row['peak_raw_requested_jerk_m_s3'] = max(row.get('peak_raw_requested_jerk_m_s3',0.),abs(audit['raw_requested_jerk']))
            info.update(objective_version=REWARD_VERSION, v05_audit=dict(conditioning=deepcopy(audit),
                phase=phase, merge_active=merge_active, legacy_acceptance_target=self._reference_targets[agent],
                reward_target=snapshot['target'], raw_actor_action_preserved=True,
                safety_or_native_change=non_comfort, variant=self.v05_variant))

    def report(self):
        result = super().report()
        result.update(schema='highway_flow_rl_continuity_env_v05', reward_version=REWARD_VERSION,
            objective_version=REWARD_VERSION, v05_variant=self.v05_variant,
            v05_weights=self.v05_weights, v05_settings=SETTINGS, v05_metrics=deepcopy(self.v05_metrics),
            intervention_definition='Unchanged combined raw-request deviations, including comfort conditioning',
            planner_scope='Reward supervision only; frozen actor motion cannot improve from reward changes')
        return result
