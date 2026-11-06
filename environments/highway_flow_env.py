"""Flow117: explicit map/corridor inputs and legacy-weight reference alignment.

This separate environment preserves v05 native command/safety authority and
reward magnitudes. It does not enable training or alter historical artifacts.
"""
from __future__ import annotations

from copy import deepcopy
from functools import lru_cache
import json
import math
import xml.etree.ElementTree as ET

import gymnasium as gym
import numpy as np

from environments.highway_environment_v05 import HighwayEnvironmentV05Env, DT
from environments.highway_stage1_env import acceleration_constraints, DEFAULT_REWARD_WEIGHTS
from traffic.highway_merge_context import merge_features, CAP
from traffic.highway_flow_context import (SCHEMA, LOCAL_SIZE, OBSERVATION_SIZE, STATE_SIZE,
    EXTRA_FEATURES, FIELD_INDEX, LEGACY_TO_FLOW, legacy_observation, inject_observation, flow_context)
from traffic.highway_smooth_metrics import SmoothControlMetrics
from traffic.highway_v05_demand import DemandManifest, PROFILES, MAIN_ROUTE, RAMP_ROUTE

REWARD_VERSION = 'highway_flow_legacy_weights_new_reference_v1'


class HighwayFlowEnv(HighwayEnvironmentV05Env):
    metadata = dict(HighwayEnvironmentV05Env.metadata, name=SCHEMA)
    observation_size = OBSERVATION_SIZE
    state_size = STATE_SIZE

    def __init__(self, network, run_directory, *, fixture=None, **kwargs):
        if kwargs.get('reward_weights') and any(
                key not in DEFAULT_REWARD_WEIGHTS or value != DEFAULT_REWARD_WEIGHTS[key]
                for key, value in kwargs['reward_weights'].items()):
            raise ValueError('Flow preparation preserves legacy reward coefficients')
        self.fixture = deepcopy(fixture)
        if self.fixture is not None and self.fixture.get('schema') != 'highway_flow_fixture_v1':
            raise ValueError('Explicit versioned flow fixture required')
        super().__init__(network, run_directory, **kwargs)
        self.state_space = gym.spaces.Box(-1., 1., (STATE_SIZE,), np.float32)
        self._reset_flow()

    @lru_cache(None)
    def observation_space(self, agent):
        return gym.spaces.Box(-1., 1., (OBSERVATION_SIZE,), np.float32)

    def _reset_flow(self):
        self._flow_step_active = False
        self.flow_contexts, self._flow_references = {}, {}
        self.fixture_event_log = []
        self._fixture_fired = set()
        self.flow_metrics = SmoothControlMetrics(task=self.task,
            controlled_ids=self._controlled_agents(), merge_vehicle_ids=())

    def reset(self, seed=None, options=None):
        self._reset_flow()
        result = super().reset(seed=seed, options=options)
        self.flow_metrics.merge_vehicle_ids.update(self.merge_agents)
        self.flow_metrics.prime(self.time, self.states)
        return result

    def _write_demand(self, path, seed):
        if self.fixture is None:
            return super()._write_demand(path, seed)
        if int(seed) != self.fixture['seed']:
            raise ValueError('Fixture seed is immutable')
        rows = deepcopy(self.fixture['vehicles'])
        if len(rows) != self.total_vehicles or {r['vehicle_id'] for r in rows if r['controlled']} != set(self._controlled_agents()):
            raise ValueError('Fixture cohort does not match declared environment')
        root = ET.Element('routes')
        common = dict(length='4.8', minGap='2.5', emergencyDecel='9', maxSpeed=str(CAP), speedDev='0', sigma='0')
        ET.SubElement(root, 'vType', id='controlled', carFollowModel='ACC', tau='1.5', accel='2', decel='4.5', speedFactor='1', **common)
        for name, profile in PROFILES.items():
            ET.SubElement(root, 'vType', id=name, carFollowModel='IDM', **{k: str(v) for k, v in profile.items()}, **common)
        for name, route in (('main', MAIN_ROUTE), ('ramp', RAMP_ROUTE)):
            ET.SubElement(root, 'route', id=name, edges=' '.join(route))
        for row in sorted(rows, key=lambda x: (x['depart'], x['route'], -x['edge'], -x['position'], x['vehicle_id'])):
            lane = self.graph.lanes.get(row['lane_id'])
            if lane is None or not 7.3 <= row['position'] <= lane['length'] - 10. or not 0 <= row['speed'] <= min(self.speed_limit_m_s or CAP, self.graph.speed_limits[row['lane_id']]):
                raise ValueError('Fixture spawn outside declared lane/limit')
            node = ET.SubElement(root, 'vehicle', id=row['vehicle_id'], type='controlled' if row['controlled'] else row['driver_profile'],
                route=row['route'], depart=str(row['depart']), departEdge=str(row['edge']), departLane=str(row['lane']),
                departPos=str(row['position']), departSpeed=str(row['speed']))
            for key, value in (('research.origin', row['route']), ('research.population_slot', row['slot']), ('research.initial_cohort', 'true')):
                ET.SubElement(node, 'param', key=key, value=value)
        ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)
        self.population_manifest = DemandManifest(rows, dict(schema='highway_flow_fixture_v1', fixture=self.fixture['name'],
            physical_seed=seed, through_lanes=self.graph.through_lanes, traffic_mode='initial',
            horizon_seconds=self.horizon * DT, description=self.fixture['description'],
            explicit_counterfactual_events=self.fixture.get('events', []), no_future_actor_observations=True))
        self.merge_agents = tuple(r['vehicle_id'] for r in rows if r['controlled'] and r['route'] == 'ramp')

    def _observations(self):
        result = {}
        for agent in self.agents:
            if agent not in self.states:
                continue
            state, context = self.states[agent], self._context(agent)
            distance = self.merge_distance(state)
            _, ceiling = acceleration_constraints(state, context['neighbors']['own']['front'], distance)
            flow = flow_context(self.graph, state, self.states)
            local = context['features'][:26] + [context['lane_count'] / 7., min(1., distance / 240.) if distance is not None else 1.,
                float(distance is not None), max(0., 1 - (self.time - self.last_change[agent]) / 5), *self._mask(agent, context)]
            local += merge_features(flow['entry'], ceiling) + self._motion_features(agent)
            slots = []
            if self.communication == 'local':
                for _, message in self._selected_messages(agent):
                    age = self.time - message['time']
                    slots.extend([np.clip((state['y'] - message['y']) / 150., -1, 1), message['requires_merge'],
                        np.clip((message['speed'] - state['speed']) / CAP, -1, 1), np.clip(message['acceleration'] / 4.5, -1, 1),
                        message['request'] / 4.5, min(1., age / .5), message['lane_intent'], 1.])
            vector = inject_observation(local + slots + [0.] * (48 - len(slots)), flow['extras'])
            if not self.observation_space(agent).contains(vector):
                raise ValueError('Invalid flow actor observation')
            result[agent] = vector
            self.flow_contexts[agent] = flow
            peers = int(vector[LOCAL_SIZE:].reshape(6, 8)[:, -1].sum())
            fresh = sum(sender in self.states and sender != agent and -1e-8 <= self.time - msg['time'] <= .5 + 1e-8
                        for sender, msg in self.inbox[agent].items())
            self.population_stats['peer_observations'] += 1
            self.population_stats['selected_peer_sum'] += peers
            self.population_stats['maximum_selected_peers'] = max(self.population_stats['maximum_selected_peers'], peers)
            self.population_stats['overflow_peer_observations'] += int(fresh > 6)
            self.population_stats['zero_peer_observations'] += int(peers == 0)
            self.population_stats['selected_peer_histogram'][str(peers)] += 1
        return result

    def state(self):
        return np.concatenate([np.r_[self.obs[a] if a in self.agents else np.zeros(OBSERVATION_SIZE), float(a in self.agents)]
                               for a in self.possible_agents]).astype(np.float32)

    def teacher_actions(self, observations=None):
        from traffic.highway_flow_reference import teacher_actions
        return teacher_actions(self.obs if observations is None else observations)

    def step(self, actions):
        from traffic.highway_flow_reference import reference_details
        original = self.obs
        self._flow_before = deepcopy(self.states)
        self._flow_before_context = deepcopy(self.flow_contexts)
        self._flow_references = {a: reference_details(original[a]) for a in self.agents}
        # The unchanged v05 method calculates its old target before our hook
        # replaces that one term. Its native action/mask path stays identical.
        self.obs = {a: legacy_observation(o) for a, o in original.items()}
        self._flow_step_active = True
        try:
            result = super().step(actions)
        except BaseException:
            self.obs = original
            raise
        finally:
            self._flow_step_active = False
        observations, rewards, terminated, truncated, infos = result
        # v05 terminal padding is99 wide; flow terminals have117 zero fields.
        observations = {a: o if o.shape == (OBSERVATION_SIZE,) else np.zeros(OBSERVATION_SIZE, np.float32) for a, o in observations.items()}
        self.flow_metrics.observe(self.time, self._flow_before, self.states, infos,
            merge_vehicle_ids=self.merge_agents, arrived=self.arrived)
        return observations, rewards, terminated, truncated, infos

    def _update_merge_metrics(self, agents, infos, rewards, terminated, truncated):
        super()._update_merge_metrics(agents, infos, rewards, terminated, truncated)
        for agent in agents:
            info, reference = infos[agent], self._flow_references[agent]
            target = float(reference['acceleration_m_s2'])
            old = info['reward_terms']['feasible_acceleration']
            replacement = -.25 * abs(info['requested_acceleration'] - target) / 4.5
            delta = replacement - old
            info['reward_terms']['feasible_acceleration'] = replacement
            rewards[agent] += delta
            self.stats['reward_terms']['feasible_acceleration'] += delta
            self.stats['acceleration_error_sum'] += abs(info['requested_acceleration'] - target) - abs(info['requested_acceleration'] - info['feasible_acceleration_target'])
            info.update(feasible_acceleration_target=target, feasible_target_source='flow117 current-state reference, legacy .25 coefficient',
                flow_reference=reference, smooth_reference=reference, flow_context=self._flow_before_context[agent],
                objective_version='legacy_weights_flow_reference_v1', fixture_events=deepcopy(self.fixture_event_log))

    def _fixture_commands(self):
        if self.fixture is None or not getattr(self, '_measurement_phase', False):
            return
        elapsed = self.time - self.start_time
        for request in self.fixture.get('speed_controls', []):
            if request['vehicle_id'] in self.states and elapsed >= request.get('from_s', 0.):
                self.conn.vehicle.setSpeed(request['vehicle_id'], request['speed_m_s'])
        for index, event in enumerate(self.fixture.get('events', [])):
            if index in self._fixture_fired or elapsed + 1e-8 < event['at_s']:
                continue
            self._fixture_fired.add(index)
            vid, delta = event['vehicle_id'], event['lane_delta']
            record = dict(event, index=index, requested_at=self.time, source='predeclared external counterfactual',
                          command_issued=False, movement_observed=False, outcome='vehicle_absent')
            if vid in self.states:
                record['source_lane'] = self.states[vid]['lane_id']
                if vid in self._controlled_agents():
                    permitted = bool(self._mask(vid, self._context(vid))[delta + 1])
                    if permitted:
                        self._begin_maneuver(vid, delta)
                else:
                    permitted = self.graph.adjacent(self.states[vid]['lane_id'], delta)[0] is not None
                    if permitted:
                        self.conn.vehicle.changeLaneRelative(vid, delta, self.lane_change_duration + DT)
                record.update(command_issued=permitted, outcome='requested_native_safe_execution' if permitted else 'guard_refused')
            self.fixture_event_log.append(record)

    def _advance(self):
        measure = getattr(self, '_measurement_phase', False) and not getattr(self, '_flow_step_active', False) and hasattr(self, 'flow_metrics')
        before = deepcopy(getattr(self, 'states', {})) if measure else None
        self._fixture_commands()
        super()._advance()
        for event in self.fixture_event_log:
            state = self.states.get(event['vehicle_id'])
            if event['command_issued'] and state and (state['lane_id'] != event['source_lane'] or abs(state['lateral_position']) > .02):
                event['movement_observed'] = True
                event.setdefault('first_observed_time_s', self.time)
        if measure:
            self.flow_metrics.observe(self.time, before, self.states, {}, merge_vehicle_ids=self.merge_agents, arrived=self.arrived)

    def report(self):
        result = super().report()
        result.update(schema=SCHEMA, observation_size=OBSERVATION_SIZE, local_size=LOCAL_SIZE, state_size=STATE_SIZE,
            control_observation_version=SCHEMA, control_target_version='highway_flow_reference_v1',
            reward_version=REWARD_VERSION, observation_extensions=EXTRA_FEATURES,
            legacy_projection_indices=LEGACY_TO_FLOW, flow_control=self.flow_metrics.report(),
            smooth_control=self.flow_metrics.report(), fixture=deepcopy(self.fixture), fixture_events=deepcopy(self.fixture_event_log),
            objective_scope='Legacy formulas/coefficients with flow reference; no Huber/residual-slew bundle',
            counterfactual_events_are_external=bool(self.fixture and self.fixture.get('events')))
        return result
