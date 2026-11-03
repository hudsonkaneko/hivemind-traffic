"""Versioned Stage 1 highway task with measurable interventions and merge state.

The historical v1 implementation remains unchanged. All guards use current
exact SUMO state/map in this development stage; this is not sensor-based driving.
"""
from __future__ import annotations

from functools import lru_cache
import json
import math

import gymnasium as gym
import numpy as np

from environments.i110_merge_env import (
    I110MergeEnv, AGENTS, LOCAL_FEATURES as V1_LOCAL_FEATURES, MESSAGE_FEATURES,
)
from environments.sumo_mappo_env import action_to_acceleration
from traffic.highway_merge_context import Stage1LaneGraph, merge_target_context, merge_features, MERGE_FEATURES, CAP
from traffic.highway_stage1_teacher import acc_response_ceiling

SCHEMA = 'highway_stage1_graph_messages_v2'
LOCAL_FEATURES = (*V1_LOCAL_FEATURES, *MERGE_FEATURES)
OBSERVATION_SIZE = 93
STATE_SIZE = 376
REWARD_VERSION = 'highway_stage1_feasible_acceleration_v2'
DEFAULT_REWARD_WEIGHTS = dict(progress=1., merge=5., passing=2., arrival=2.,
    collision=20., intervention=.2, jerk=.01, lane_change=.1,
    feasible_acceleration=.25, unsafe_request=.5, cooperation=.1)


def acceleration_constraints(state, front, merge_distance):
    """Same v1 speed/following/lane-end bounds, explicitly returned for auditing."""
    bounds = dict(speed_ceiling=(min(CAP, state['allowed_speed']) - state['speed']) / .1,
                  following=None, lane_end=None)
    if front:
        leader = max(0., state['speed'] + front['relative_speed_m_s'])
        safe = max(0., leader + .5 * (front['bumper_gap_m'] - 2.5 - 1.5 * state['speed']))
        bounds['following'] = (safe - state['speed']) / .1
    if merge_distance is not None:
        safe = math.sqrt(2 * 4.5 * max(0., merge_distance - 2.5))
        bounds['lane_end'] = (safe - state['speed']) / .1
    ceiling = float(np.clip(min(2., *(v for v in bounds.values() if v is not None)), -4.5, 2.))
    return bounds, ceiling


def feasible_acceleration_target(state, front, merge_distance):
    """Taper toward a feasible target, including a current-input ACC prediction.

    The ACC estimate is the teacher's documented SUMO-response calibration, not
    a native API query or an extra guard. It prevents the reward from requesting
    acceleration above the predicted ordinary following response. Native
    junction/safety behavior remains independently measured and unattributed.
    """
    target_speed = min(CAP, state['allowed_speed'])
    if front:
        leader = max(0., state['speed'] + front['relative_speed_m_s'])
        target_speed = min(target_speed, max(0., leader + .5 * (front['bumper_gap_m'] - 2.5 - 1.5 * state['speed'])))
    if merge_distance is not None:
        target_speed = min(target_speed, math.sqrt(9. * max(0., merge_distance - 2.5)))
    _, ceiling = acceleration_constraints(state, front, merge_distance)
    neighbor = ((front['bumper_gap_m'], max(0., state['speed'] + front['relative_speed_m_s']))
                if front else None)
    predicted_response = acc_response_ceiling(state['speed'], min(CAP, state['allowed_speed']), neighbor)
    return min(ceiling, predicted_response, float(np.clip(.8 * (target_speed - state['speed']), -4.5, 2.)))


class HighwayStage1Env(I110MergeEnv):
    metadata = {'name': SCHEMA, 'render_modes': [], 'is_parallelizable': True}
    observation_size = OBSERVATION_SIZE
    state_size = STATE_SIZE

    def __init__(self, network, run_directory, *, reward_weights=None, **kwargs):
        super().__init__(network, run_directory, **kwargs)
        self.graph = Stage1LaneGraph(self.network, self.graph.through_lanes)
        self.state_space = gym.spaces.Box(-1., 1., (STATE_SIZE,), np.float32)
        self.reward_weights = dict(DEFAULT_REWARD_WEIGHTS)
        if reward_weights:
            if set(reward_weights) - set(self.reward_weights):
                raise ValueError('Unknown Stage 1 reward weight')
            if any(not math.isfinite(v) or v < 0 for v in reward_weights.values()):
                raise ValueError('Reward magnitudes must be finite and nonnegative')
            self.reward_weights.update(reward_weights)

    @lru_cache(None)
    def observation_space(self, agent):
        return gym.spaces.Box(-1., 1., (OBSERVATION_SIZE,), np.float32)

    def reset(self, seed=None, options=None):
        result = super().reset(seed=seed, options=options)
        self.stats.update(override_causes={}, policy_filter_changes=0,
                          native_realization_changes=0, emergency_braking_observed=0,
                          acceleration_error_sum=0., abs_acceleration_sum=0.,
                          abs_jerk_sum=0., minimum_ttc_s=None, minimum_headway_s=None)
        return result

    def _observations(self):
        observations = {}
        for agent in self.agents:
            if agent not in self.states:
                continue
            state = self.states[agent]
            context = self._context(agent)
            distance = self.merge_distance(state)
            _, ceiling = acceleration_constraints(state, context['neighbors']['own']['front'], distance)
            local = context['features'][:26] + [context['lane_count'] / 7.,
                min(1., distance / 240.) if distance is not None else 1.,
                float(distance is not None), max(0., 1 - (self.time - self.last_change[agent]) / 5),
                *self._mask(agent, context)]
            local += merge_features(merge_target_context(self.graph, state, self.states), ceiling)
            slots = []
            if self.communication == 'local':
                for _, message in sorted(self.inbox[agent].items()):
                    age = self.time - message['time']
                    if age > .5 + 1e-8:
                        continue
                    slots.extend([np.clip((state['y'] - message['y']) / 150., -1, 1),
                        message['requires_merge'], np.clip((message['speed'] - state['speed']) / CAP, -1, 1),
                        np.clip(message['acceleration'] / 4.5, -1, 1), message['request'] / 4.5,
                        min(1., age / .5), message['lane_intent'], 1.])
            vector = np.asarray(local + slots + [0.] * (48 - len(slots)), np.float32)
            if not self.observation_space(agent).contains(vector):
                raise ValueError('Invalid Stage 1 graph observation')
            observations[agent] = vector
        return observations

    def state(self):
        return np.concatenate([np.r_[self.obs[a] if a in self.agents else np.zeros(OBSERVATION_SIZE),
                                    float(a in self.agents)] for a in AGENTS]).astype(np.float32)

    def scripted_actions(self):
        from traffic.highway_stage1_teacher import teacher_actions
        return teacher_actions(self.obs)

    def _lane_rejection_causes(self, agent, context, delta):
        if not delta:
            return []
        state = self.states[agent]
        causes = []
        if self.graph.lanes[state['lane_id']]['internal']:
            causes.append('lane_internal_segment')
        if self.time - self.last_change[agent] < 5:
            causes.append('lane_cooldown')
        direction = 'left' if delta > 0 else 'right'
        if not context['adjacent_context_known']:
            causes.append('lane_context_unknown')
        if not context['lane_exists'][direction]:
            causes.append('lane_unavailable')
        if not self._mask(agent, context)[delta + 1] and not causes:
            causes.append('lane_gap')
        return causes

    def step(self, actions):
        agents = list(self.agents)
        if not agents or set(actions) != set(agents):
            raise ValueError('One action for each active agent required')
        before = self.states
        requested, applied, lane_requests, rejected, contexts, diagnostics = {}, {}, {}, {}, {}, {}
        # Validate the complete joint action before sending any commands.
        for agent in agents:
            action = np.asarray(actions[agent], np.float32)
            if not self.action_space(agent).contains(action):
                raise ValueError('Invalid hybrid action')
            requested[agent] = action_to_acceleration(action[0])
            lane_requests[agent] = int(action[1])
        snapshots = {a: dict(time=self.time, x=before[a]['x'], y=before[a]['y'], speed=before[a]['speed'],
            acceleration=before[a]['acceleration'], requires_merge=float(self.merge_distance(before[a]) is not None),
            request=self.previous[a][0], lane_intent=self.previous[a][1]) for a in agents}
        for agent in agents:
            state, context, delta = before[agent], self._context(agent), lane_requests[agent]
            contexts[agent] = context
            distance = self.merge_distance(state)
            bounds, ceiling = acceleration_constraints(state, context['neighbors']['own']['front'], distance)
            causes = self._lane_rejection_causes(agent, context, delta)
            rejected[agent] = bool(causes)
            if self.control_mode == 'policy' and delta and not rejected[agent]:
                self.conn.vehicle.changeLaneRelative(agent, delta, .1)
            applied[agent] = min(requested[agent], ceiling)
            # Causes count constraints violated by the request, not a speculative
            # attribution of native SUMO acceleration differences.
            violations = [key for key, value in bounds.items() if value is not None and requested[agent] > value + 1e-4]
            if abs(applied[agent] - requested[agent]) > 1e-4:
                causes += violations
            diagnostics[agent] = dict(bounds=bounds, ceiling=ceiling, causes=causes,
                violations=violations, target=feasible_acceleration_target(state, context['neighbors']['own']['front'], distance))
            if self.control_mode == 'policy':
                self.conn.vehicle.setAcceleration(agent, applied[agent], .1)
            self.previous[agent] = (requested[agent], delta)
        self._advance()
        self.steps += 1
        for receiver in agents:
            if receiver not in self.states or self.communication == 'none':
                continue
            for sender, message in snapshots.items():
                if sender == receiver:
                    continue
                if math.hypot(message['x'] - before[receiver]['x'], message['y'] - before[receiver]['y']) > 150:
                    continue
                if self.rng.random() < self.packet_loss:
                    self.stats['dropped_messages'] += 1
                    continue
                self.inbox[receiver][sender] = message
                self.stats['delivered_messages'] += 1
                self.stats['delivered_bytes'] += len(json.dumps(message, sort_keys=True, separators=(',', ':')).encode())

        events = {}
        for agent in agents:
            state, old = self.states.get(agent), before[agent]
            same_edge = bool(state and self.graph.lanes[state['lane_id']]['edge'] == self.graph.lanes[old['lane_id']]['edge'])
            changed = bool(same_edge and self.graph.identity(state['lane_id']) != self.graph.identity(old['lane_id']))
            merged = bool(self.task == 'merge' and agent in ('agent_2', 'agent_3') and state and
                          agent not in self.merged and self.graph.identity(state['lane_id']).get('kind') == 'through')
            passed = bool(self.task == 'passing' and agent not in self.passed and state and 'slow_leader' in self.states and
                          state['y'] < self.states['slow_leader']['y'] - 10)
            events[agent] = dict(changed=changed, merged=merged, passed=passed, same_edge=same_edge)
        team_merges = sum(event['merged'] for event in events.values()) / len(AGENTS)
        rewards, terminated, truncated, infos = {}, {}, {}, {}
        for agent in agents:
            state, old, event, diagnostic = self.states.get(agent), before[agent], events[agent], diagnostics[agent]
            completed = agent in self.arrived
            if event['changed']:
                self.last_change[agent] = self.time
                self.stats['lane_changes'] += 1
            if event['merged']:
                self.merged.add(agent)
                self.stats['successful_merges'] += 1
            if event['passed']:
                self.passed.add(agent)
                self.stats['passes'] += 1
            realized = state['acceleration'] if state else None
            policy_filter_changed = abs(applied[agent] - requested[agent]) > 1e-4
            native_changed = state is not None and abs(realized - applied[agent]) > .05
            override = self.control_mode == 'policy' and (rejected[agent] or policy_filter_changed or native_changed)
            causes = diagnostic['causes'] + (['native_realization_unattributed'] if native_changed else [])
            emergency = bool(state and realized < -4.5 - .05)
            if emergency:
                causes.append('emergency_braking_observed')
            jerk = abs(realized - old['acceleration']) / .1 if state else 0.
            progress = max(0., state['distance'] - old['distance']) if state else old['speed'] * .1 if completed else 0.
            raw = dict(progress=progress / 2.68224, merge=float(event['merged']), passing=float(event['passed']),
                arrival=float(completed), collision=-float(agent in self.collisions), intervention=-float(override),
                jerk=-jerk, lane_change=-float(event['changed']),
                feasible_acceleration=-abs(requested[agent] - diagnostic['target']) / 4.5,
                unsafe_request=-max(0., requested[agent] - diagnostic['ceiling']) / 4.5,
                cooperation=team_merges)
            terms = {name: value * self.reward_weights[name] for name, value in raw.items()}
            rewards[agent] = float(sum(terms.values()))
            for name, value in terms.items():
                self.stats['reward_terms'][name] = self.stats['reward_terms'].get(name, 0.) + value
            for name in set(causes) if override else ():
                self.stats['override_causes'][name] = self.stats['override_causes'].get(name, 0) + 1
            self.stats['overrides'] += int(override)
            self.stats['policy_filter_changes'] += int(self.control_mode == 'policy' and policy_filter_changed)
            self.stats['native_realization_changes'] += int(self.control_mode == 'policy' and native_changed)
            self.stats['emergency_braking_observed'] += int(emergency)
            self.stats['agent_steps'] += 1
            self.stats['peak_abs_jerk'] = max(self.stats['peak_abs_jerk'], jerk)
            self.stats['abs_jerk_sum'] += jerk
            self.stats['acceleration_error_sum'] += abs(requested[agent] - diagnostic['target'])
            self.stats['abs_acceleration_sum'] += abs(realized) if state else 0.
            self.stats['overspeed_steps'] += int(state is not None and state['speed'] > min(CAP, state['allowed_speed']) + 1e-4)
            if state:
                self.stats['speed_ratio_sum'] += state['speed'] / max(.1, min(CAP, state['allowed_speed']))
                self.stats['speed_samples'] += 1
            front = contexts[agent]['neighbors']['own']['front']
            if front:
                if old['speed'] > .1:
                    headway = front['bumper_gap_m'] / old['speed']
                    previous = self.stats['minimum_headway_s']
                    self.stats['minimum_headway_s'] = headway if previous is None else min(previous, headway)
                if front['relative_speed_m_s'] < -.01:
                    ttc = front['bumper_gap_m'] / -front['relative_speed_m_s']
                    previous = self.stats['minimum_ttc_s']
                    self.stats['minimum_ttc_s'] = ttc if previous is None else min(previous, ttc)
            terminated[agent] = completed or agent not in self.states or bool(self.collisions) or self.stats['teleports'] > 0
            truncated[agent] = self.steps >= self.horizon and not terminated[agent]
            infos[agent] = dict(reward_terms=terms, requested_acceleration=requested[agent],
                applied_acceleration=applied[agent], filtered_acceleration=applied[agent], realized_acceleration=realized,
                lane_request=lane_requests[agent], lane_rejected=rejected[agent],
                lane_applied_delta=lane_requests[agent] if not rejected[agent] and self.control_mode == 'policy' else 0,
                lane_change_observed=event['changed'],
                observed_lane_index_delta=(self.graph.lanes[state['lane_id']]['index'] - self.graph.lanes[old['lane_id']]['index']) if event['same_edge'] else None,
                override=bool(override), override_causes=sorted(set(causes)) if override else [],
                policy_filter_changed=bool(self.control_mode == 'policy' and policy_filter_changed),
                native_realization_changed=bool(self.control_mode == 'policy' and native_changed),
                command_actually_applied=self.control_mode == 'policy',
                counterfactual_filter_difference=bool(self.control_mode != 'policy' and policy_filter_changed),
                requested_constraint_violations=diagnostic['violations'],
                acceleration_bounds=diagnostic['bounds'], feasible_acceleration_ceiling=diagnostic['ceiling'],
                feasible_acceleration_target=diagnostic['target'],
                bound_source='current SUMO state/map analytical guards',
                feasible_target_source='current-input taper and pure SUMO ACC response prediction; not native API or future state',
                native_cause='unattributed' if native_changed and self.control_mode == 'policy' else None,
                merge_completed=event['merged'], route_completed=completed)
        self.obs = self._observations()
        observations = {a: self.obs.get(a, np.zeros(OBSERVATION_SIZE, np.float32)) for a in agents}
        self.agents = [a for a in agents if not terminated[a] and not truncated[a]]
        self.obs = {a: observations[a] for a in self.agents}
        self.journal.write(json.dumps(dict(time=self.time, control_mode=self.control_mode, agents=infos), allow_nan=False) + '\n')
        return observations, rewards, terminated, truncated, infos

    def report(self):
        report = super().report()
        count = max(1, self.stats['agent_steps'])
        report.update(schema=SCHEMA, control_mode=self.control_mode, reward_version=REWARD_VERSION, reward_weights=self.reward_weights,
            mean_abs_acceleration=self.stats['abs_acceleration_sum'] / count,
            mean_abs_jerk=self.stats['abs_jerk_sum'] / count,
            mean_feasible_acceleration_error=self.stats['acceleration_error_sum'] / count,
            override_cause_fractions={k: v / count for k, v in self.stats['override_causes'].items()},
            override_metric='Same v1 policy-mode union: lane rejected, |filtered-requested|>1e-4, or measured |realized-filtered|>0.05; denominator active-agent steps',
            native_attribution='SUMO realization differences are measured but their underlying native cause is not inferred',
            observation_source='exact current SUMO state/map and delivered messages; no perception input')
        return report
