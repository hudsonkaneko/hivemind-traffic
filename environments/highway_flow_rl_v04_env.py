"""V04 merge preparation and complete-maneuver gap mask; unchanged native guards."""
from copy import deepcopy

from environments.highway_flow_rl_env import HighwayFlowRLEnv
from environments.highway_flow_rl_v02_env import HighwayFlowRLV02Env
from traffic.highway_flow_rl_v04_rewards import (REWARD_VERSION, MASK_VERSION,
    validate_weights, merge_speed_target, gap_plan, shaping_reward,
    maneuver_gap_evidence, quality_merge_eligible)
from traffic.highway_flow_rl_v03_rewards import (map_evidence, optional_detour_evidence,
    speed_error_reward, jerk_reward, BrakingDiagnostic)
from traffic.highway_flow_context import flow_context

SCHEMA = 'highway_flow_rl_gap_preparation_env_v04'


class HighwayFlowRLV04Env(HighwayFlowRLEnv):
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

    def __init__(self, network, run_directory, *, jerk_reward_weight=0., v04_reward_weights=None, **kwargs):
        if jerk_reward_weight != 0.:
            raise ValueError('V04 replaces the old jerk term; its base weight must be zero')
        self.v04_reward_weights = validate_weights(v04_reward_weights)
        self._reset_v04()
        super().__init__(network, run_directory, jerk_reward_weight=0., **kwargs)

    def _reset_v04(self):
        self._v02_pre_action, self._v02_pending = {}, {}
        self.quality_history, self.quality_results, self.approach_stops = {}, {}, {}
        self.braking_diagnostic = BrakingDiagnostic()
        self.v04_summary = dict(decisions=0, target_eligible=0, gap_preparation_reward=0., quality_merge_reward=0., speed_error_reward=0.,
            jerk_reward=0., route_detour_reward=0., detour_motion_events=0, exclusions={}, per_agent={})
        self.v04_mask_audit = dict(version=MASK_VERSION, changed=True, decision_samples=0,
            additional_blocked_actions=0, allowed_actions=dict(right=0, stay=0, left=0))

    def reset(self, seed=None, options=None):
        self._reset_v04()
        return super().reset(seed=seed, options=options)

    def step(self, actions):
        self._v02_pre_action = {}
        for agent in self.agents:
            state, context = self.states[agent], self.flow_contexts[agent]
            baseline_mask = HighwayFlowRLEnv._mask(self,agent,self._context(agent))
            self.v04_mask_audit['additional_blocked_actions'] += sum(
                int(old > .5 and new < .5) for old,new in zip(baseline_mask,self.obs[agent][30:33]))
            kinds = {r['lane_id']: self.graph.identity(r['lane_id']).get('kind')
                     for r in context['corridors'].values() if r.get('lane_id')}
            preview = map_evidence(self.graph, state)
            target, plan = merge_speed_target(state, context, preview)
            detours = {str(d): optional_detour_evidence(state, context, d, kinds,
                       lane_change_duration=self.lane_change_duration) for d in (-1, 1)}
            self._v02_pre_action[agent] = dict(time_s=self.time,
                observation117=self.obs[agent].tolist(), allowed_actions=self.obs[agent][30:33].tolist(),
                target=target, gap_plan=plan, map_preview=preview, detours=detours,
                ego_state=deepcopy(state), context=deepcopy(context), lane_kinds=kinds,
                maneuver_gap_checks={label: maneuver_gap_evidence(state['speed'],self._context(agent)['neighbors'][label]) for label in ('left','right')})
        return super().step(actions)

    def _update_merge_metrics(self, agents, infos, rewards, terminated, truncated):
        super()._update_merge_metrics(agents, infos, rewards, terminated, truncated)
        for agent in agents:
            info, snapshot = infos[agent], self._v02_pre_action[agent]
            event = self._consume_detour_event(agent)
            detour = bool(event and event['evidence']['optional_detour'])
            state = self.states.get(agent)
            next_speed = state['speed'] if state else None
            weights = self.v04_reward_weights
            successor = gap_plan(state, flow_context(self.graph,state,self.states)) if state else dict(potential=0.)
            gap_reward = shaping_reward(snapshot['gap_plan'], successor,
                terminated=terminated[agent], weight=weights['gap_preparation']) if agent in self.merge_agents else 0.
            target = snapshot['target']
            context = self._context(agent) if state else None
            neighbor_ids = {n['id'] for neighbors in context['neighbors'].values() for n in neighbors.values() if n} if context else set()
            neighbor_accel = min((self.states[n]['acceleration'] for n in neighbor_ids if n in self.states), default=None)
            sample = dict(time_s=self.time, stopped=next_speed is not None and next_speed < .1,
                jerk=info['realized_jerk_m_s3'], override=info['override'], neighbor_acceleration=neighbor_accel,
                speed_match=bool(next_speed is not None and target['eligible'] and abs(next_speed-target['target_m_s'])<=2.))
            history = self.quality_history.setdefault(agent,[])
            history.append(sample)
            tracker = self.merge_trackers.get(agent)
            completed_at = tracker.merge_complete_time_s if tracker else None
            if tracker and completed_at is None:
                self.approach_stops[agent] = self.approach_stops.get(agent,0.) + .1*sample['stopped']
            history[:] = [r for r in history if r['time_s'] >= (completed_at if completed_at is not None else self.time)-8.]
            stable_old = info['reward_terms'].get('stable_merge',0.)
            quality = bool(stable_old and self.approach_stops.get(agent,0.) <= 1.+1e-8
                           and quality_merge_eligible(history))
            if stable_old:
                self.quality_results[agent] = dict(qualified=quality, samples=len(history),
                    stop_seconds=sum(r['stopped'] for r in history)*.1,
                    full_approach_stop_seconds=self.approach_stops.get(agent,0.),
                    speed_match_fraction=sum(r['speed_match'] for r in history)/len(history),
                    scope='Observed approach and post-merge quality; neighbor braking is not causal attribution')
            rewards[agent] -= stable_old
            self.stats['reward_terms']['stable_merge'] -= stable_old
            info['reward_terms']['stable_merge'] = 0.
            terms = dict(gap_preparation=gap_reward, quality_merge=weights['quality_merge']*quality, speed_error=speed_error_reward(snapshot['target'], next_speed, weights['speed_error']),
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
            info.update(objective_version=REWARD_VERSION, v04_reward_audit=dict(
                pre_action=deepcopy(snapshot), measured_speed_m_s=next_speed, detour_event=event,
                avoidable_braking=diagnosis, mask_version=MASK_VERSION,
                no_actor_input_added=True, no_teacher_reference=True, successor_gap_plan=successor, quality_sample=sample))
            row = self.v04_summary['per_agent'].setdefault(agent, dict(decisions=0, target_eligible=0,
                gap_preparation_reward=0., quality_merge_reward=0., speed_error_reward=0., jerk_reward=0., route_detour_reward=0., detour_motion_events=0))
            for total in (row, self.v04_summary):
                total['decisions'] += 1
                total['target_eligible'] += int(snapshot['target']['eligible'])
                total['gap_preparation_reward'] += terms['gap_preparation']
                total['quality_merge_reward'] += terms['quality_merge']
                total['speed_error_reward'] += terms['speed_error']
                total['jerk_reward'] += terms['jerk']
                total['route_detour_reward'] += terms['route_detour']
                total['detour_motion_events'] += int(detour)
            for reason in snapshot['target']['reasons']:
                self.v04_summary['exclusions'][reason] = self.v04_summary['exclusions'].get(reason, 0) + 1
            self.v04_mask_audit['decision_samples'] += 1
            for name, allowed in zip(('right', 'stay', 'left'), snapshot['allowed_actions']):
                self.v04_mask_audit['allowed_actions'][name] += int(allowed > .5)

    def _mask(self, agent, context):
        mask = super()._mask(agent, context)
        for label, index in (('right',0),('left',2)):
            if mask[index] and not maneuver_gap_evidence(self.states[agent]['speed'], context['neighbors'][label])['permitted']:
                mask[index] = 0.
        return mask

    def _lane_rejection_causes(self, agent, context, delta):
        causes = super()._lane_rejection_causes(agent, context, delta)
        if delta and not maneuver_gap_evidence(self.states[agent]['speed'], context['neighbors']['left' if delta==1 else 'right'])['permitted']:
            causes.append('projected_maneuver_gap')
        return causes

    def report(self):
        result = super().report()
        result.update(schema=SCHEMA, reward_version=REWARD_VERSION, objective_version=REWARD_VERSION,
            v04_reward_weights=dict(self.v04_reward_weights), v04_reward_summary=deepcopy(self.v04_summary),
            v04_mask_audit=deepcopy(self.v04_mask_audit),
            avoidable_braking=deepcopy(self.braking_diagnostic.rows),
            quality_merges=deepcopy(self.quality_results),
            pre_merge_stop_seconds=deepcopy(self.approach_stops),
            objective_scope='Gap preparation potential, merge-speed target, rolling quality and projected native-safe mask')
        return result
