"""Continuous SUMO lane motion, fixed 24-slot population and explicit maneuver ABI.

The first 45 actor fields retain the shared v3 meanings. Six measured/commanded
motion fields precede the six delivered-message slots. SUMO remains the sole
motion owner; this environment never interpolates or overwrites vehicle poses.
"""
from __future__ import annotations

from functools import lru_cache
import json
import math
from numbers import Integral

import gymnasium as gym
import numpy as np

from environments.highway_population_env import HighwayPopulationEnv, episode_speed_ceiling
from environments.highway_stage1_env import (
    HighwayStage1Env, LOCAL_FEATURES as LEGACY_LOCAL_FEATURES, MESSAGE_FEATURES,
    acceleration_constraints, DEFAULT_REWARD_WEIGHTS,
)
from environments.sumo_mappo_env import action_to_acceleration
from traffic.highway_merge_context import CAP, Stage1LaneGraph, merge_features, merge_target_context_v3
from traffic.highway_stage1_teacher import longitudinal_target_v3, teacher_actions as legacy_teacher_actions

SCHEMA = 'highway_environment_v05_continuous_motion'
CONTROL_OBSERVATION_VERSION = 'highway_environment_v05_actor_99'
REWARD_VERSION = 'highway_environment_v05_stable_merge'
MAX_AGENTS = 24
LOCAL_SIZE = 51
OBSERVATION_SIZE = 99
STATE_SIZE = MAX_AGENTS * (OBSERVATION_SIZE + 1)
DT = .1
MOTION_FEATURES = (
    'native_lateral_position_over_lane_width', 'native_lateral_speed_over_3',
    'committed_lane_direction', 'maneuver_in_progress',
    'maneuver_remaining_distance_fraction', 'time_since_completion_over_5',
)
LOCAL_FEATURES = (*LEGACY_LOCAL_FEATURES, *MOTION_FEATURES)
FEATURES = (*LOCAL_FEATURES, *(f'message_{slot}_{name}' for slot in range(6) for name in MESSAGE_FEATURES))


def legacy_observation(observation):
    """Project actor-visible v05 inputs into the frozen v3 teacher contract."""
    vector = np.asarray(observation, np.float32)
    if vector.shape != (OBSERVATION_SIZE,) or not np.isfinite(vector).all():
        raise ValueError('Expected a finite 99-field v05 observation')
    return np.concatenate((vector[:45], vector[51:]))


def teacher_actions(observations):
    return legacy_teacher_actions({a: legacy_observation(o) for a, o in observations.items()}, version='v3')


class ContinuousMotionLaneGraph(Stage1LaneGraph):
    """Current measured lateral occupancy, mapped onto connected road lanes.

    SUMO changes the primary lane ID at the maneuver midpoint. A noncentered
    vehicle also occupies a conservative corridor on the side of its measured
    lateral offset: the destination before the midpoint, the source afterward.
    Movement toward that corridor is measurable, not a queried native intent.
    A paused offcenter vehicle retains both corridors until it is centered.
    """

    def occupied_lanes(self, state):
        lanes = [state['lane_id']]
        offset = state.get('lateral_position', 0.)
        width = state.get('lane_width', 3.2)
        if abs(offset) <= .02 or abs(offset) > width * .5 + .05:
            return lanes
        adjacent, known = self.adjacent(state['lane_id'], 1 if offset > 0 else -1)
        if known and adjacent is not None:
            lanes.append(adjacent)
        return lanes

    def occupancy_states(self, states, ego_id=None):
        expanded, aliases = dict(states), {}
        for vid, state in states.items():
            if vid == ego_id:
                continue
            for lane in self.occupied_lanes(state)[1:]:
                key = f'{vid}\x00occupied:{lane}'
                position = (state['lane_position'] / self.lanes[state['lane_id']]['length']
                            * self.lanes[lane]['length'])
                expanded[key] = dict(state, id=key, lane_id=lane, lane_position=position)
                aliases[key] = vid
        return expanded, aliases

    def neighbor(self, ego, states, lid, position, *, forward):
        expanded, aliases = self.occupancy_states(states, ego['id'])
        corridors = [lid]
        if lid == ego['lane_id']:
            corridors = self.occupied_lanes(ego)
        candidates = []
        for lane in corridors:
            projected = position / self.lanes[lid]['length'] * self.lanes[lane]['length']
            neighbor = super().neighbor(ego, expanded, lane, projected, forward=forward)
            if neighbor is not None:
                neighbor = dict(neighbor, id=aliases.get(neighbor['id'], neighbor['id']))
                candidates.append(neighbor)
        return min(candidates, key=lambda n: (n['bumper_gap_m'], n['id'])) if candidates else None


class HighwayEnvironmentV05Env(HighwayPopulationEnv):
    metadata = dict(HighwayStage1Env.metadata, name=SCHEMA)
    observation_size = OBSERVATION_SIZE
    state_size = STATE_SIZE

    def __init__(self, network, run_directory, *, controlled_count, total_vehicles,
                 max_agents=MAX_AGENTS, speed_limit_m_s=None, difficulty=0,
                 traffic_mode='initial', lane_change_duration=3., stabilization_seconds=8.,
                 native_warmup_seconds=0.,
                 **kwargs):
        if max_agents != MAX_AGENTS:
            raise ValueError('V05 declares exactly 24 stable critic slots')
        if isinstance(controlled_count, bool) or not isinstance(controlled_count, Integral) or not 0 <= controlled_count <= MAX_AGENTS:
            raise ValueError('controlled_count must be an integer from zero to 24')
        if isinstance(total_vehicles, bool) or total_vehicles not in (8, 16, 24) or controlled_count > total_vehicles:
            raise ValueError('total_vehicles must be 8, 16 or 24 and include all AVs')
        if isinstance(difficulty, bool) or difficulty not in (0, 1, 2):
            raise ValueError('difficulty must be zero, one or two')
        if traffic_mode not in ('initial', 'sustained'):
            raise ValueError('traffic_mode must be initial or sustained')
        if traffic_mode == 'sustained' and controlled_count == total_vehicles:
            raise ValueError('Full-AV scenarios use initial traffic; later native inflow changes that fraction')
        if kwargs.get('task', 'merge') in ('following', 'passing') and controlled_count == total_vehicles:
            raise ValueError('Following/passing requires a native slow leader within the total population')
        if not math.isfinite(lane_change_duration) or lane_change_duration < DT:
            raise ValueError('Lane-change duration must be finite and at least one tick')
        if not math.isfinite(stabilization_seconds) or not 0 < stabilization_seconds <= 30:
            raise ValueError('Post-merge stabilization must be finite, positive and at most 30 seconds')
        if not math.isfinite(native_warmup_seconds) or not 0 <= native_warmup_seconds <= 30:
            raise ValueError('Native warmup must be finite and between zero and 30 seconds')
        if not math.isclose(native_warmup_seconds / DT, round(native_warmup_seconds / DT), abs_tol=1e-8):
            raise ValueError('Native warmup must contain a whole number of 0.1-second ticks')
        if kwargs.get('control_target_version', 'v3') != 'v3':
            raise ValueError('V05 requires the shared v3 longitudinal target')
        kwargs['control_target_version'] = 'v3'
        self.controlled_count, self.total_vehicles = int(controlled_count), int(total_vehicles)
        self.speed_limit_m_s = episode_speed_ceiling(speed_limit_m_s)
        self.difficulty, self.traffic_mode = int(difficulty), traffic_mode
        self.lane_change_duration, self.stabilization_seconds = float(lane_change_duration), float(stabilization_seconds)
        self.native_warmup_seconds = float(native_warmup_seconds)
        self.possible_agents = [f'agent_{i}' for i in range(MAX_AGENTS)]
        self.population_manifest, self.merge_agents = [], ()
        self.native_speed_limit_manifest, self.population_stats = [], {}
        self._speed_limits_applied = False
        self.maneuvers, self.completed_maneuvers = {}, {}
        self.motion_stats, self.merge_trackers = {}, {}
        self._measurement_phase = False
        # Intentionally bypass the old eight-slot population constructor.
        HighwayStage1Env.__init__(self, network, run_directory, **kwargs)
        self.graph = ContinuousMotionLaneGraph(self.network, self.graph.through_lanes)
        self.state_space = gym.spaces.Box(-1., 1., (STATE_SIZE,), np.float32)

    @lru_cache(None)
    def observation_space(self, agent):
        return gym.spaces.Box(-1., 1., (OBSERVATION_SIZE,), np.float32)

    def _sumo_extra_args(self):
        return ['--lanechange.duration', str(self.lane_change_duration),
                '--lanechange-output', str(self.directory / 'lanechanges.xml'),
                '--lanechange-output.started', 'true', '--lanechange-output.ended', 'true']

    def _write_demand(self, path, seed):
        from traffic.highway_v05_demand import write_demand
        self.population_manifest = write_demand(path, seed, self.graph, self.task,
            self.controlled_count, self.total_vehicles, speed_limit_m_s=self.speed_limit_m_s,
            difficulty=self.difficulty, traffic_mode=self.traffic_mode,
            stabilization_seconds=self.native_warmup_seconds, horizon_seconds=self.horizon * DT)
        self.merge_agents = tuple(v['vehicle_id'] for v in self.population_manifest
                                  if v['controlled'] and v['route'] == 'ramp')

    def _spawn_ready(self):
        initial = {r['vehicle_id'] for r in self.population_manifest if r['initial_cohort']}
        return initial.issubset(self.states)

    def reset(self, seed=None, options=None):
        if options and set(options) & {'controlled_count', 'total_vehicles', 'max_agents', 'communication',
                'packet_loss', 'speed_limit_m_s', 'difficulty', 'traffic_mode', 'lane_change_duration', 'stabilization_seconds', 'native_warmup_seconds'}:
            raise ValueError('V05 is configured at construction; live controller handoff is not implemented')
        from traffic.highway_v05_metrics import PostMergeTracker
        self.maneuvers, self.completed_maneuvers = {}, {}
        self._measurement_phase = False
        self.motion_stats = dict(started=0, completed=0, native_refused=0, interrupted=0,
            active_samples=0, nonzero_lateral_samples=0, maximum_abs_lateral_speed_m_s=0.,
            maximum_lateral_displacement_per_tick_m=0., completion_durations_s=[])
        self.merge_trackers = {}
        self.occupancy_samples = []
        try:
            # Reuse population reset initialization, preserving the old class untouched.
            super().reset(seed=seed, options=options)
            spawn_time = self.time
            spawn_population = len(self.states)
            for _ in range(round(self.native_warmup_seconds / DT)):
                self._advance()
                if self.stats['collisions'] or self.stats['teleports']:
                    raise RuntimeError('Native collision or teleport during v05 stabilization')
            missing = set(self._controlled_agents()) - set(self.states)
            if missing:
                raise RuntimeError(f'Fixed AV cohort departed before measurement: {sorted(missing)}')
            self.start_time = self.time
            self.stabilization = dict(post_merge_window_seconds=self.stabilization_seconds,
                native_warmup_requested_seconds=self.native_warmup_seconds, native_warmup_actual_seconds=self.time - spawn_time,
                start_time=spawn_time, measurement_start_time=self.time,
                initial_on_road=spawn_population, measurement_on_road=len(self.states),
                motion_owner='native SUMO; policy begins immediately after initial spawn and any explicitly requested warmup')
            self.route_trackers = {a: PostMergeTracker(vehicle_id=a, window_s=self.stabilization_seconds) for a in self._controlled_agents()}
            self.merge_trackers = {a: self.route_trackers[a] for a in self.merge_agents}
            self._measurement_phase = True
            self.occupancy_samples = []
            self._record_occupancy()
            self.obs = self._observations()
            return self.obs, {a: dict(motion_source='native SUMO', maneuver=self._maneuver_info(a)) for a in self.agents}
        except BaseException:
            self.close()
            raise

    def _record_occupancy(self):
        count = sum(a in self.states for a in self._controlled_agents())
        total = len(self.states)
        self.occupancy_samples.append(dict(time=self.time - self.start_time, total=total,
                                          controlled=count, av_fraction=count / max(1, total)))

    def _advance(self):
        before = getattr(self, 'states', {})
        super()._advance()
        for agent, state in self.states.items():
            state['lateral_position'] = float(self.conn.vehicle.getLateralLanePosition(agent))
            state['lateral_speed'] = float(self.conn.vehicle.getLateralSpeed(agent))
            state['angle'] = float(self.conn.vehicle.getAngle(agent))
            state['lane_width'] = float(self.conn.lane.getWidth(state['lane_id']))
            state['lane_identity'] = self.graph.identity(state['lane_id'])
            if not all(math.isfinite(state[k]) for k in ('x', 'y', 'speed', 'acceleration', 'lateral_position', 'lateral_speed', 'angle', 'lane_width')):
                raise ValueError('Nonfinite native SUMO vehicle motion')
            if state['lane_width'] <= 0:
                raise ValueError('Invalid native lane width')
            if getattr(self, '_measurement_phase', False):
                self.motion_stats['maximum_abs_lateral_speed_m_s'] = max(
                    self.motion_stats['maximum_abs_lateral_speed_m_s'], abs(state['lateral_speed']))
                if agent in before:
                    # Project the actual world displacement onto the old heading's
                    # perpendicular. No rendering position enters this metric.
                    old = before[agent]
                    angle = math.radians(old.get('angle', state['angle']))
                    lateral_step = abs((state['x'] - old['x']) * math.cos(angle)
                                       - (state['y'] - old['y']) * math.sin(angle))
                    self.motion_stats['maximum_lateral_displacement_per_tick_m'] = max(
                        self.motion_stats['maximum_lateral_displacement_per_tick_m'], lateral_step)
        self._update_maneuvers()
        if getattr(self, '_measurement_phase', False):
            self._record_occupancy()

    def _lane_identity_key(self, lane_id):
        return json.dumps(self.graph.identity(lane_id), sort_keys=True)

    def _begin_maneuver(self, agent, delta):
        state = self.states[agent]
        target_lane, known = self.graph.adjacent(state['lane_id'], delta)
        if not known or target_lane is None:
            raise ValueError('Cannot commit a maneuver without a known target lane')
        target_index = self.graph.lanes[target_lane]['index']
        maneuver = dict(source_lane=state['lane_id'], target_lane=target_lane,
            source_identity=self._lane_identity_key(state['lane_id']),
            target_identity=self._lane_identity_key(target_lane), target_index=target_index,
            direction=int(delta), started_at=self.time, moved=False, remaining_fraction=1.)
        previous = self.completed_maneuvers.get(agent)
        context = self._context(agent)
        # Conservative operational definition: an immediate return to the
        # previous lane, with no merge requirement, no slower front car, no
        # delivered merge request, and no passing task. Other reversals remain
        # explicitly unclassified, rather than guessing the driver's intent.
        front = context['neighbors']['own']['front']
        unnecessary = bool(previous and previous['source_identity'] == maneuver['target_identity']
            and self.time - previous['completed_at'] <= 8.
            and self.merge_distance(state) is None and self.task != 'passing'
            and (front is None or front['relative_speed_m_s'] >= -1.)
            and not any(m['requires_merge'] for _, m in self._selected_messages(agent)))
        maneuver.update(reversal_reason='unnecessary' if unnecessary else None,
                        reason_evidence=unnecessary)
        # Command duration is a request lifetime; --lanechange.duration is the
        # physical SUMO movement duration. A single request holds the target.
        self.conn.vehicle.changeLane(agent, target_index, self.lane_change_duration + DT)
        self.maneuvers[agent] = maneuver
        self.motion_stats['started'] += 1

    def _update_maneuvers(self):
        self.completed_this_step = set()
        self.refused_this_step = {}
        self.interrupted_this_step = {}
        for agent, maneuver in list(self.maneuvers.items()):
            state = self.states.get(agent)
            if state is None:
                self.motion_stats['interrupted'] += 1
                del self.maneuvers[agent]
                continue
            elapsed = self.time - maneuver['started_at']
            on_target = self._lane_identity_key(state['lane_id']) == maneuver['target_identity']
            moved = abs(state['lateral_position']) > .02 or abs(state['lateral_speed']) > .02 or on_target
            maneuver['moved'] |= moved
            self.motion_stats['active_samples'] += 1
            self.motion_stats['nonzero_lateral_samples'] += int(abs(state['lateral_speed']) > .02)
            lateral_fraction = state['lateral_position'] / state['lane_width']
            maneuver['remaining_fraction'] = float(np.clip(abs(lateral_fraction) if on_target
                else 1. - maneuver['direction'] * lateral_fraction, 0., 1.))
            if on_target and abs(state['lateral_position']) < .05 and abs(state['lateral_speed']) < .05:
                self.completed_maneuvers[agent] = dict(maneuver, completed_at=self.time, duration_s=elapsed)
                self.last_change[agent] = self.time
                self.completed_this_step.add(agent)
                self.motion_stats['completed'] += 1
                self.motion_stats['completion_durations_s'].append(elapsed)
                del self.maneuvers[agent]
            elif (maneuver['moved'] and elapsed > DT + 1e-8
                  and abs(state['lateral_position']) < .05 and abs(state['lateral_speed']) < .05):
                # Native SUMO can abandon motion when a target lane disappears.
                # Settling on a different corridor is an observed interruption,
                # never completion or a controller-implemented safe abort.
                self.interrupted_this_step[agent] = dict(maneuver,
                    interrupted_at=self.time, reason='native_settled_without_target',
                    settled_lane=state['lane_id'])
                self.motion_stats['interrupted'] += 1
                del self.maneuvers[agent]
            elif elapsed > self.lane_change_duration + 2 * DT and not maneuver['moved']:
                # SUMO rejected a collision-unsafe request: do not invent motion
                # completion or retain a pending command after its expiry.
                self.motion_stats['native_refused'] += 1
                self.refused_this_step[agent] = dict(maneuver)
                del self.maneuvers[agent]

    def _lane_change_runway(self, agent, delta):
        """Bound travel during the complete maneuver using current map/speed.

        Connected lanes with the same semantic identity, including internal
        connectors, form one corridor. A mere edge seam does not end it.
        The bound includes the existing 2 m/s2 acceleration limit, one tick of
        command latency, and the existing 2.5 m standstill clearance.
        """
        state = self.states[agent]
        target, known = self.graph.adjacent(state['lane_id'], delta)
        if not known or target is None:
            return None
        position = (state['lane_position'] / self.graph.lanes[state['lane_id']]['length']
                    * self.graph.lanes[target]['length'])
        target_identity = self.graph.identity(target)
        distances = self.graph.distances(target, position, state['route'])
        corridor = {lane: distance for lane, distance in distances.items()
                    if self.graph.identity(lane) == target_identity}
        ends = [distance + self.graph.lanes[lane]['length'] for lane, distance in corridor.items()
                if not (self.graph.successors[lane] & set(corridor))]
        available = max(0., min(ends)) if ends else math.inf
        duration = self.lane_change_duration + DT
        required = state['speed'] * duration + .5 * 2. * duration ** 2 + 2.5
        return dict(available_m=available, required_m=required,
                    permitted=available + 1e-8 >= required)

    def _mask(self, agent, context):
        if agent in self.maneuvers:
            return [0., 1., 0.]
        mask = super()._mask(agent, context)
        for delta in (-1, 1):
            if mask[delta + 1]:
                runway = self._lane_change_runway(agent, delta)
                if runway is not None and not runway['permitted']:
                    mask[delta + 1] = 0.
        return mask

    def _lane_rejection_causes(self, agent, context, delta):
        if delta and agent in self.maneuvers:
            return ['maneuver_in_progress']
        causes = super()._lane_rejection_causes(agent, context, delta)
        if delta:
            runway = self._lane_change_runway(agent, delta)
            if runway is not None and not runway['permitted']:
                # The parent names an otherwise-unattributed mask rejection
                # lane_gap. Remove that fallback only when its own gap mask
                # would allow this action; simultaneous failures retain both.
                if super()._mask(agent, context)[delta + 1]:
                    causes = [cause for cause in causes if cause != 'lane_gap']
                causes.append('lane_change_runway')
        return causes

    def _motion_features(self, agent):
        state, maneuver = self.states[agent], self.maneuvers.get(agent)
        last = self.completed_maneuvers.get(agent)
        return [float(np.clip(state['lateral_position'] / state['lane_width'], -1., 1.)),
            float(np.clip(state['lateral_speed'] / 3., -1., 1.)),
            float(maneuver['direction']) if maneuver else 0., float(maneuver is not None),
            maneuver['remaining_fraction'] if maneuver else 0.,
            min(1., max(0., (self.time - last['completed_at']) / 5.)) if last else 1.]

    def _maneuver_info(self, agent):
        state = self.states.get(agent)
        return dict(in_progress=agent in self.maneuvers,
            committed_target=dict(self.maneuvers[agent]) if agent in self.maneuvers else None,
            completed_this_step=agent in getattr(self, 'completed_this_step', ()),
            native_refusal=getattr(self, 'refused_this_step', {}).get(agent),
            native_interruption=getattr(self, 'interrupted_this_step', {}).get(agent),
            native_lateral_position_m=state['lateral_position'] if state else None,
            native_lateral_speed_m_s=state['lateral_speed'] if state else None,
            safe_abort_implemented=False)

    def _observations(self):
        observations = {}
        for agent in self.agents:
            if agent not in self.states:
                continue
            state, context = self.states[agent], self._context(agent)
            distance = self.merge_distance(state)
            _, ceiling = acceleration_constraints(state, context['neighbors']['own']['front'], distance)
            local = context['features'][:26] + [context['lane_count'] / 7.,
                min(1., distance / 240.) if distance is not None else 1., float(distance is not None),
                max(0., 1 - (self.time - self.last_change[agent]) / 5), *self._mask(agent, context)]
            occupied, _ = self.graph.occupancy_states(self.states, agent)
            local += merge_features(merge_target_context_v3(self.graph, state, occupied), ceiling)
            local += self._motion_features(agent)
            slots = []
            if self.communication == 'local':
                for _, message in self._selected_messages(agent):
                    age = self.time - message['time']
                    slots.extend([np.clip((state['y'] - message['y']) / 150., -1, 1),
                        message['requires_merge'], np.clip((message['speed'] - state['speed']) / CAP, -1, 1),
                        np.clip(message['acceleration'] / 4.5, -1, 1), message['request'] / 4.5,
                        min(1., age / .5), message['lane_intent'], 1.])
            vector = np.asarray(local + slots + [0.] * (48 - len(slots)), np.float32)
            if not self.observation_space(agent).contains(vector):
                raise ValueError('Invalid v05 continuous-motion observation')
            observations[agent] = vector
            peers = int(vector[51:].reshape(6, 8)[:, -1].sum())
            fresh = sum(sender in self.states and sender != agent and
                        -1e-8 <= self.time - msg['time'] <= .5 + 1e-8 for sender, msg in self.inbox[agent].items())
            self.population_stats['peer_observations'] += 1
            self.population_stats['selected_peer_sum'] += peers
            self.population_stats['maximum_selected_peers'] = max(self.population_stats['maximum_selected_peers'], peers)
            self.population_stats['overflow_peer_observations'] += int(fresh > 6)
            self.population_stats['zero_peer_observations'] += int(peers == 0)
            self.population_stats['selected_peer_histogram'][str(peers)] += 1
        return observations

    def state(self):
        return np.concatenate([np.r_[self.obs[a] if a in self.agents else np.zeros(OBSERVATION_SIZE),
                                    float(a in self.agents)] for a in self.possible_agents]).astype(np.float32)

    def teacher_actions(self, observations=None):
        return teacher_actions(self.obs if observations is None else observations)

    def scripted_actions(self):
        return self.teacher_actions()

    def step(self, actions):
        agents = list(self.agents)
        if not agents and not actions:
            self._advance()
            self.steps += 1
            return {}, {}, {}, {}, {}
        if set(actions) != set(agents):
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
                self._begin_maneuver(agent, delta)
            applied[agent] = min(requested[agent], ceiling)
            # Causes count constraints violated by the request, not a speculative
            # attribution of native SUMO acceleration differences.
            violations = [key for key, value in bounds.items() if value is not None and requested[agent] > value + 1e-4]
            if abs(applied[agent] - requested[agent]) > 1e-4:
                causes += violations
            diagnostics[agent] = dict(bounds=bounds, ceiling=ceiling, causes=causes,
                violations=violations, target=longitudinal_target_v3(legacy_observation(self.obs[agent])))
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
            changed = agent in self.completed_this_step
            merged = bool(self.task == 'merge' and agent in self._merge_agents() and state and
                          agent not in self.merged and self.graph.identity(state['lane_id']).get('kind') == 'through' and
                          agent not in self.maneuvers and abs(state['lateral_position']) < .05 and abs(state['lateral_speed']) < .05)
            passed = bool(self.task == 'passing' and agent not in self.passed and state and 'slow_leader' in self.states and
                          state['y'] < self.states['slow_leader']['y'] - 10)
            events[agent] = dict(changed=changed, merged=merged, passed=passed, same_edge=same_edge)
        team_merges = sum(event['merged'] for event in events.values()) / max(1, len(self._controlled_agents()))
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
            native_lane_refused = agent in getattr(self, 'refused_this_step', {})
            native_lane_interrupted = agent in getattr(self, 'interrupted_this_step', {})
            override = self.control_mode == 'policy' and (rejected[agent] or policy_filter_changed or native_changed or native_lane_refused or native_lane_interrupted)
            causes = diagnostic['causes'] + (['native_realization_unattributed'] if native_changed else [])
            if native_lane_refused:
                causes.append('native_lane_request_refused')
            if native_lane_interrupted:
                causes.append('native_lane_maneuver_interrupted')
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
                feasible_target_source=('shared v3 observation-only teacher/reward target' if self.control_target_version == 'v3'
                    else 'current-input taper and pure SUMO ACC response prediction; not native API or future state'),
                native_cause='unattributed' if native_changed and self.control_mode == 'policy' else None,
                merge_completed=event['merged'], route_completed=completed,
                maneuver=self._maneuver_info(agent), motion_source='native SUMO')
        self._update_merge_metrics(agents, infos, rewards, terminated, truncated)
        self.obs = self._observations()
        observations = {a: self.obs.get(a, np.zeros(OBSERVATION_SIZE, np.float32)) for a in agents}
        self.agents = [a for a in agents if not terminated[a] and not truncated[a]]
        self.obs = {a: observations[a] for a in self.agents}
        self.journal.write(json.dumps(dict(time=self.time, control_mode=self.control_mode, agents=infos), allow_nan=False) + '\n')
        return observations, rewards, terminated, truncated, infos

    def _update_merge_metrics(self, agents, infos, rewards, terminated, truncated):
        for agent in agents:
            tracker = self.route_trackers[agent]
            state, info = self.states.get(agent), infos[agent]
            if state is not None:
                lane = self._lane_identity_key(state['lane_id'])
                if agent in self.merge_trackers and state['lane_identity'].get('kind') == 'through':
                    tracker.mark_entry(self.time)
                    if agent not in self.maneuvers and abs(state['lateral_position']) < .05 and abs(state['lateral_speed']) < .05:
                        tracker.mark_merge_complete(self.time, lane)
                context = self._context(agent)
                front = context['neighbors']['own']['front']
                headway = (front['bumper_gap_m'] / state['speed'] if state['speed'] > .1 else math.inf) if front else None
                ttc = (front['bumper_gap_m'] / -front['relative_speed_m_s']
                       if front and front['relative_speed_m_s'] < -.01 else math.inf if front else None)
                neighbor_ids = {n['id'] for lane_neighbors in context['neighbors'].values()
                                for n in lane_neighbors.values() if n is not None}
                braking = min((self.states[n]['acceleration'] for n in neighbor_ids if n in self.states), default=None)
                maneuver = self.maneuvers.get(agent) or self.completed_maneuvers.get(agent) or {}
                tracker.observe(self.time, speed_mps=state['speed'], speed_limit_mps=min(CAP, state['allowed_speed']),
                    distance_m=max(0., state['distance']), leader_headway_s=headway, leader_ttc_s=ttc,
                    leader_present=front is not None, collision=agent in self.collisions,
                    emergency=state['acceleration'] < -4.55, intervention=info['override'], lane=lane,
                    neighbor_braking_mps2=braking, requested_acceleration=info['requested_acceleration'],
                    filtered_acceleration=info['filtered_acceleration'], realized_acceleration=info['realized_acceleration'],
                    native_intervention=info['native_realization_changed'],
                    reversal_reason=maneuver.get('reversal_reason'), reason_evidence=maneuver.get('reason_evidence', False))
            if terminated[agent] or truncated[agent]:
                tracker.finish(arrived=info['route_completed'], truncated=truncated[agent])
            stable_bonus = tracker.consume_stable_bonus() if agent in self.merge_trackers else 0.
            oscillation_penalty = -.5 * tracker.consume_lane_oscillation_penalty()
            terms = dict(stable_merge=stable_bonus, lane_oscillation=oscillation_penalty)
            info['reward_terms'].update(terms)
            for name, value in terms.items():
                self.stats['reward_terms'][name] = self.stats['reward_terms'].get(name, 0.) + value
            rewards[agent] += sum(terms.values())
            info['stable_merge_status'] = tracker.status if agent in self.merge_trackers else None

    def report(self):
        from traffic.highway_v05_demand import demand_report
        report = super().report()
        summaries = {a: tracker.summary() for a, tracker in self.merge_trackers.items()}
        routes = {a: tracker.summary()['full_route'] for a, tracker in self.route_trackers.items()}
        occupancy = self.occupancy_samples
        steady_start = min(10., self.horizon * DT * .25)
        steady = [row for row in occupancy if row['time'] >= steady_start - 1e-8]
        report.update(schema=SCHEMA, control_observation_version=CONTROL_OBSERVATION_VERSION,
            reward_version=REWARD_VERSION, max_agents=MAX_AGENTS, observation_size=OBSERVATION_SIZE,
            local_size=LOCAL_SIZE, state_size=STATE_SIZE,
            reward_extensions=dict(stable_merge_bonus=2., unnecessary_reversal_penalty=.5),
            stable_merge=dict(targets=len(summaries), entered=sum(s['entry_event'] for s in summaries.values()),
                lateral_completed=sum(s['lateral_merge_completed'] for s in summaries.values()),
                stable=sum(s['status'] == 'succeeded' for s in summaries.values()),
                failed=sum(s['status'] == 'failed' for s in summaries.values()),
                pending=sum(s['status'] == 'pending' for s in summaries.values()),
                oscillation_events=sum(s['full_route']['unnecessary_reversals'] for s in summaries.values()),
                per_agent=summaries),
            per_car_full_route=routes, difficulty=self.difficulty, traffic_mode=self.traffic_mode,
            scenario=demand_report(self.population_manifest), stabilization=self.stabilization,
            measured_steady_window=dict(requested_start_s=steady_start,
                actual_end_s=occupancy[-1]['time'] if occupancy else None, samples=len(steady),
                definition='Measurement after min(10 seconds, one quarter of configured horizon); no samples inferred',
                mean_total_on_road=sum(s['total'] for s in steady) / len(steady) if steady else None,
                mean_controlled_on_road=sum(s['controlled'] for s in steady) / len(steady) if steady else None,
                mean_av_fraction=sum(s['av_fraction'] for s in steady) / len(steady) if steady else None),
            measured_occupancy=dict(samples=len(occupancy),
                minimum_total=min((s['total'] for s in occupancy), default=0),
                maximum_total=max((s['total'] for s in occupancy), default=0),
                mean_total=sum(s['total'] for s in occupancy) / max(1, len(occupancy)),
                mean_controlled=sum(s['controlled'] for s in occupancy) / max(1, len(occupancy)),
                mean_av_fraction=sum(s['av_fraction'] for s in occupancy) / max(1, len(occupancy)),
                minimum_av_fraction=min((s['av_fraction'] for s in occupancy), default=0.),
                maximum_av_fraction=max((s['av_fraction'] for s in occupancy), default=0.),
                trace=occupancy),
            continuous_motion=dict(self.motion_stats, native_lanechange_duration_s=self.lane_change_duration,
                tick_s=DT, source='native SUMO lateral position/speed and world poses; no pose interpolation',
                command='one absolute target lane request for duration+one tick, held until geometric completion',
                completion='target lane identity, |native lateral offset|<0.05 m and |native lateral speed|<0.05 m/s',
                interruption='Previously moving maneuver settles on a non-target corridor: record interruption and release intent, never completion or safe abort',
                runway_mask='Connected target corridor must cover speed*T+0.5*2*T^2+2.5 metres, T=duration+0.1s; same-identity edge seams preserve continuity',
                actor_motion_features=list(MOTION_FEATURES), safe_abort_implemented=False,
                active_maneuvers={a: dict(m) for a, m in self.maneuvers.items()}),
            neighbor_context='V05 keeps field meanings/indices but enriches occupancy: measured displaced adjacent vehicles occupy both source/destination corridors until centered; graph distance and fractional parallel-lane projection, never Euclidean leader gaps or native intent queries',
            population_contract='Controller assignment fixed at reset; 24 stable critic slots; later inflow is native background',
            unnecessary_reversal_rule='Return to previous lane within 8s, no required merge, no slower front car (>1 m/s deficit), no fresh delivered merge request, non-passing task; all other reversals unclassified')
        return report


# Short alias for experiment callers; both names identify the same versioned ABI.
HighwayEnvironmentV05 = HighwayEnvironmentV05Env
