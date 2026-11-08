"""Isolated v02 reward hooks; flow117 and native v05 masks/guards are unchanged."""
from __future__ import annotations

from copy import deepcopy

from environments.highway_flow_rl_env import HighwayFlowRLEnv
from traffic.highway_flow_rl_v02_rewards import (REWARD_VERSION, MASK_VERSION,
    validate_weights, map_preview, cruise_speed_component, optional_detour_evidence)

SCHEMA = 'highway_flow_rl_current_evidence_env_v02'


class HighwayFlowRLV02Env(HighwayFlowRLEnv):
    metadata = dict(HighwayFlowRLEnv.metadata, name=SCHEMA)

    def __init__(self, network, run_directory, *, jerk_reward_weight=.03,
                 v02_reward_weights=None, **kwargs):
        self.v02_reward_weights = validate_weights(v02_reward_weights)
        self._reset_v02()
        super().__init__(network, run_directory, jerk_reward_weight=jerk_reward_weight, **kwargs)

    def _reset_v02(self):
        self._v02_pre_action, self._v02_pending = {}, {}
        self.v02_summary = dict(cruise_eligible_decisions=0, cruise_speed_reward=0.,
            route_detour_reward=0., detour_motion_events=0, per_agent={})
        self.v02_mask_audit = dict(version=MASK_VERSION, changed=False, decision_samples=0,
            allowed_actions={'right': 0, 'stay': 0, 'left': 0},
            optional_detour_allowed_decisions=0, additional_blocked_actions=0,
            denominator='One sample per controlled active pre-action decision; independent of executed-action intervention accounting')

    def reset(self, seed=None, options=None):
        self._reset_v02()
        return super().reset(seed=seed, options=options)

    def step(self, actions):
        self._v02_pre_action = {}
        for agent in self.agents:
            state, context = self.states[agent], self.flow_contexts[agent]
            kinds = {row['lane_id']: self.graph.identity(row['lane_id']).get('kind')
                     for row in context['corridors'].values() if row.get('lane_id')}
            value, cruise = cruise_speed_component(state, context,
                map_preview(self.graph, state, lane_change_duration=self.lane_change_duration),
                maneuver_in_progress=agent in self.maneuvers,
                weight=self.v02_reward_weights['cruise_speed'])
            detours = {str(delta): optional_detour_evidence(state, context, delta, kinds,
                       lane_change_duration=self.lane_change_duration) for delta in (-1, 1)}
            self._v02_pre_action[agent] = dict(time_s=self.time,
                observation117=self.obs[agent].tolist(), allowed_actions=self.obs[agent][30:33].tolist(),
                cruise_speed_reward=value, cruise=cruise, detours=detours,
                ego_state=deepcopy(state), context=deepcopy(context), lane_kinds=kinds)
        return super().step(actions)

    def _begin_maneuver(self, agent, delta):
        super()._begin_maneuver(agent, delta)
        snapshot = self._v02_pre_action.get(agent)
        if snapshot is not None:
            self._v02_pending[agent] = dict(started_at=self.maneuvers[agent]['started_at'],
                evidence=deepcopy(snapshot['detours'][str(delta)]),
                pre_action_time_s=snapshot['time_s'])

    def _consume_detour_event(self, agent):
        pending = self._v02_pending.get(agent)
        if pending is None:
            return None
        maneuver = self.maneuvers.get(agent) or self.completed_maneuvers.get(agent)
        if maneuver and maneuver.get('started_at') == pending['started_at'] and maneuver.get('moved'):
            self._v02_pending.pop(agent)
            return dict(pending, first_measured_motion_time_s=self.time)
        if agent not in self.maneuvers:
            self._v02_pending.pop(agent)
        return None

    def _update_merge_metrics(self, agents, infos, rewards, terminated, truncated):
        super()._update_merge_metrics(agents, infos, rewards, terminated, truncated)
        for agent in agents:
            info, snapshot = infos[agent], self._v02_pre_action[agent]
            event = self._consume_detour_event(agent)
            detour = bool(event and event['evidence']['optional_detour'])
            # Realized braking suppresses the positive addition; it never adds
            # a negative slowdown term or changes existing safety penalties.
            cruise = snapshot['cruise_speed_reward']
            braking_veto = info['realized_acceleration'] is None or info['realized_acceleration'] < -.05
            if braking_veto:
                cruise = 0.
            terms = dict(cruise_speed=cruise,
                         route_detour=-self.v02_reward_weights['route_detour'] * detour)
            rewards[agent] += sum(terms.values())
            info['reward_terms'].update(terms)
            for name, value in terms.items():
                self.stats['reward_terms'][name] = self.stats['reward_terms'].get(name, 0.) + value
            info.update(objective_version=REWARD_VERSION,
                v02_reward_audit=dict(pre_action=deepcopy(snapshot), detour_event=event,
                    measured_braking_veto=braking_veto, mask_version=MASK_VERSION,
                    no_actor_input_added=True, no_teacher_reference=True))
            row = self.v02_summary['per_agent'].setdefault(agent,
                dict(decisions=0, cruise_eligible_decisions=0, cruise_speed_reward=0.,
                     route_detour_reward=0., detour_motion_events=0))
            row['decisions'] += 1
            for target in (row, self.v02_summary):
                target['cruise_eligible_decisions'] += int(snapshot['cruise']['eligible'] and not braking_veto)
                target['cruise_speed_reward'] += cruise
                target['route_detour_reward'] += terms['route_detour']
                target['detour_motion_events'] += int(detour)
            self.v02_mask_audit['decision_samples'] += 1
            for name, allowed in zip(('right', 'stay', 'left'), snapshot['allowed_actions']):
                self.v02_mask_audit['allowed_actions'][name] += int(allowed > .5)
            self.v02_mask_audit['optional_detour_allowed_decisions'] += int(any(
                snapshot['allowed_actions'][delta + 1] > .5 and snapshot['detours'][str(delta)]['optional_detour']
                for delta in (-1, 1)))

    def report(self):
        result = super().report()
        result.update(schema=SCHEMA, reward_version=REWARD_VERSION, objective_version=REWARD_VERSION,
            v02_reward_weights=dict(self.v02_reward_weights),
            v02_reward_summary=deepcopy(self.v02_summary), v02_mask_audit=deepcopy(self.v02_mask_audit),
            objective_scope='Teacher-free native objective plus conservative free-cruise bonus and one-time observed optional ending-lane detour penalty; unchanged native guards and masks')
        return result
