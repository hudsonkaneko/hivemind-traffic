"""V03 reward-only hooks: native commands, masks and actor ABI are inherited."""
from copy import deepcopy

from environments.highway_flow_rl_env import HighwayFlowRLEnv
from environments.highway_flow_rl_v02_env import HighwayFlowRLV02Env
from traffic.highway_flow_rl_v03_rewards import (REWARD_VERSION, MASK_VERSION,
    validate_weights, map_evidence, safe_speed_target, optional_detour_evidence,
    speed_error_reward, jerk_reward, BrakingDiagnostic)

SCHEMA = 'highway_flow_rl_safe_speed_huber_env_v03'


class HighwayFlowRLV03Env(HighwayFlowRLEnv):
    metadata = dict(HighwayFlowRLEnv.metadata, name=SCHEMA)
    # Reuse only the independently tested first-motion event lifecycle.
    def _begin_maneuver(self, agent, delta):
        super()._begin_maneuver(agent, delta)
        snapshot = self._v02_pre_action.get(agent)
        if snapshot is not None:
            self._v02_pending[agent] = dict(started_at=self.maneuvers[agent]['started_at'],
                evidence=deepcopy(snapshot['detours'][str(delta)]),
                pre_action_time_s=snapshot['time_s'])

    _consume_detour_event = HighwayFlowRLV02Env._consume_detour_event

    def __init__(self, network, run_directory, *, jerk_reward_weight=0., v03_reward_weights=None, **kwargs):
        if jerk_reward_weight != 0.:
            raise ValueError('V03 replaces the old jerk term; its base weight must be zero')
        self.v03_reward_weights = validate_weights(v03_reward_weights)
        self._reset_v03()
        super().__init__(network, run_directory, jerk_reward_weight=0., **kwargs)

    def _reset_v03(self):
        self._v02_pre_action, self._v02_pending = {}, {}
        self.braking_diagnostic = BrakingDiagnostic()
        self.v03_summary = dict(decisions=0, target_eligible=0, speed_error_reward=0.,
            jerk_reward=0., route_detour_reward=0., detour_motion_events=0, exclusions={}, per_agent={})
        self.v03_mask_audit = dict(version=MASK_VERSION, changed=False, decision_samples=0,
            additional_blocked_actions=0, allowed_actions=dict(right=0, stay=0, left=0))

    def reset(self, seed=None, options=None):
        self._reset_v03()
        return super().reset(seed=seed, options=options)

    def step(self, actions):
        self._v02_pre_action = {}
        for agent in self.agents:
            state, context = self.states[agent], self.flow_contexts[agent]
            kinds = {r['lane_id']: self.graph.identity(r['lane_id']).get('kind')
                     for r in context['corridors'].values() if r.get('lane_id')}
            preview = map_evidence(self.graph, state)
            target = safe_speed_target(state, context, preview)
            detours = {str(d): optional_detour_evidence(state, context, d, kinds,
                       lane_change_duration=self.lane_change_duration) for d in (-1, 1)}
            self._v02_pre_action[agent] = dict(time_s=self.time,
                observation117=self.obs[agent].tolist(), allowed_actions=self.obs[agent][30:33].tolist(),
                target=target, map_preview=preview, detours=detours,
                ego_state=deepcopy(state), context=deepcopy(context), lane_kinds=kinds)
        return super().step(actions)

    def _update_merge_metrics(self, agents, infos, rewards, terminated, truncated):
        super()._update_merge_metrics(agents, infos, rewards, terminated, truncated)
        for agent in agents:
            info, snapshot = infos[agent], self._v02_pre_action[agent]
            event = self._consume_detour_event(agent)
            detour = bool(event and event['evidence']['optional_detour'])
            state = self.states.get(agent)
            next_speed = state['speed'] if state else None
            weights = self.v03_reward_weights
            terms = dict(speed_error=speed_error_reward(snapshot['target'], next_speed, weights['speed_error']),
                jerk=jerk_reward(info['realized_jerk_m_s3'], weights['jerk_huber']),
                route_detour=-weights['route_detour'] * detour)
            # The legacy jerk feature remains measured, but its base weight is zero.
            if info['reward_terms']['jerk'] != 0.: raise ValueError('Jerk would be charged twice')
            rewards[agent] += sum(terms.values())
            info['reward_terms'].update(terms)
            for name, value in terms.items():
                self.stats['reward_terms'][name] = self.stats['reward_terms'].get(name, 0.) + value
            diagnosis = self.braking_diagnostic.observe(agent, snapshot['time_s'], snapshot['target'],
                snapshot['ego_state']['speed'], info['requested_acceleration'], info['feasible_acceleration_ceiling'])
            info.update(objective_version=REWARD_VERSION, v03_reward_audit=dict(
                pre_action=deepcopy(snapshot), measured_speed_m_s=next_speed, detour_event=event,
                avoidable_braking=diagnosis, mask_version=MASK_VERSION,
                no_actor_input_added=True, no_teacher_reference=True))
            row = self.v03_summary['per_agent'].setdefault(agent, dict(decisions=0, target_eligible=0,
                speed_error_reward=0., jerk_reward=0., route_detour_reward=0., detour_motion_events=0))
            for total in (row, self.v03_summary):
                total['decisions'] += 1
                total['target_eligible'] += int(snapshot['target']['eligible'])
                total['speed_error_reward'] += terms['speed_error']
                total['jerk_reward'] += terms['jerk']
                total['route_detour_reward'] += terms['route_detour']
                total['detour_motion_events'] += int(detour)
            for reason in snapshot['target']['reasons']:
                self.v03_summary['exclusions'][reason] = self.v03_summary['exclusions'].get(reason, 0) + 1
            self.v03_mask_audit['decision_samples'] += 1
            for name, allowed in zip(('right', 'stay', 'left'), snapshot['allowed_actions']):
                self.v03_mask_audit['allowed_actions'][name] += int(allowed > .5)

    def report(self):
        result = super().report()
        result.update(schema=SCHEMA, reward_version=REWARD_VERSION, objective_version=REWARD_VERSION,
            v03_reward_weights=dict(self.v03_reward_weights), v03_reward_summary=deepcopy(self.v03_summary),
            v03_mask_audit=deepcopy(self.v03_mask_audit),
            avoidable_braking=deepcopy(self.braking_diagnostic.rows),
            objective_scope='Pre-action safe-speed error, uncapped Huber jerk and first-motion optional detour; native control unchanged')
        return result
