"""Highway merge v15, Phase 0: one flat environment that executes exactly what v14 round 24 executes.

v15 Phase 0 builds tools only (no training, nothing charged to a budget ledger). This environment is
the first of them: the v14 merge environment as configured for round 24 (HighwayFlowRLV14Env with the
drive limit, follow gate and drive saturation on; experiments/run_highway_flow_rl_v14.py env_for),
rewritten as one class that reads top to bottom. It inherits from no v04-v14 / HighwayFlow* /
HighwayEnvironmentV05Env / HighwayPopulationEnv / HighwayStage1Env / I110MergeEnv class and never builds
one. It imports only plain helpers: the lane graph, the demand writer, the road grade, the merge
tracker, the observation feature builders and the v04-v14 target functions.

Given the same per-agent actions it applies the same accelerations and lane changes, reads back the
same vehicle states at the same times, ends cars on the same decisions and returns the same 121-field
actor observations and 2928-field critic state (tests/test_highway_merge_v15_env.py checks this
against the v14 environment). Rewards are 0.0: v15's event-based rewards come later. Of v14's reward
and audit code only the parts that feed execution or observations are kept (the stage comments say
which v14 class each comes from):
- the per-car merge-speed target (v14 ramp and follow-gate reference): actor inputs 69-72, and the
  gate's speed match;
- the speed-match history: v11's merge-speed precondition in the lane mask (inputs 30-32);
- the merge trackers (post-merge window): 'pending' gates the target inputs, and the stable merge
  ends the follow reference;
- the override and its causes, with v14's drive-saturation exemption: the merge window counts
  interventions.

Reset: R1 episode directory, R2 demand, R3 episode state, R4 SUMO start, R5 spawn, R6 native spawn
modes, R7 native warmup, R8 trackers, R9 observe, R10 v14 control modes.
Step: S1 action map, S2 per-car targets, S3 message snapshots, S4 per-car command (bounds, comfort rate
limit, applied, setAcceleration; lane request, rejection causes, changeLane), S5 advance, S6 messages,
S7 outcomes (override, termination), S8 trackers and speed-match history, S9 observe.

SUMO runs through traffic.sumo_coupling.module(): TraCI by default, libsumo after
sumo_coupling.use('libsumo') (one simulation per process).
"""
from __future__ import annotations

from copy import deepcopy
import json
import math
import shutil
import uuid
from numbers import Integral
from pathlib import Path

import gymnasium as gym
import numpy as np
from pettingzoo import ParallelEnv

from environments.highway_environment_v05 import ContinuousMotionLaneGraph
from environments.highway_population_env import episode_speed_ceiling, population_leader_speed
from environments.highway_stage1_env import acceleration_constraints
from environments.sumo_mappo_env import ManeuverActionSpace, action_to_acceleration
from traffic import sumo_coupling
from traffic import highway_flow_rl_v06_rewards as v06
from traffic import highway_flow_rl_v11_rewards as v11
from traffic import highway_flow_rl_v12_rewards as v12
from traffic import highway_flow_rl_v14_rewards as v14
from traffic import highway_flow_rl_v14_follow_gate as follow_module
from traffic import highway_flow_rl_v14_saturation as saturation_module
from traffic.highway_flow_context import flow_context, inject_observation
from traffic.highway_flow_rl_v03_rewards import map_evidence
from traffic.highway_flow_rl_v04_rewards import maneuver_gap_evidence
from traffic.highway_flow_rl_v05_rewards import condition_acceleration
from traffic.highway_merge_context import CAP, merge_features
from traffic.highway_v05_demand import write_demand
from traffic.highway_v05_metrics import PostMergeTracker
from traffic.road_grade import RoadGrade, validate_drive_limit
from traffic.sumo_backend import find_sumo_binary

ROOT = Path(__file__).resolve().parents[1]
SCHEMA = 'highway_merge_v15_flat_env_phase0'
DT = .1
MAX_AGENTS = 24
RAW_SIZE = 117                       # v13/v14 flow observation before the target inputs
OBSERVATION_SIZE = 121               # v12: target inputs inserted at 69..72
STATE_SIZE = MAX_AGENTS * (OBSERVATION_SIZE + 1)
TASKS = ('cruise', 'following', 'passing', 'merge')
SPAWN_MODES = dict(speed=31, lane_change=512)            # i110_merge_env reset, until R10
CONTROL_MODES = dict(speed=v14.CONTROL['speed_mode'], lane_change=v14.CONTROL['lane_change_mode'])
MESSAGE_RANGE_M = 150.
MESSAGE_AGE_S = .5
MESSAGE_SLOTS = 6
SPAWN_ADVANCES = 100
SPEED_MATCH_M_S = 2.                 # v04's gate: |speed - target| <= 2 m/s
QUALITY_WINDOW_S = 8.                # v04's speed-match history window
ACTION_SPACE = ManeuverActionSpace()
RAW_SPACE = gym.spaces.Box(-1., 1., (RAW_SIZE,), np.float32)
ACTOR_SPACE = gym.spaces.Box(-1., 1., (OBSERVATION_SIZE,), np.float32)
CRITIC_SPACE = gym.spaces.Box(-1., 1., (STATE_SIZE,), np.float32)


# --- pure stage helpers (unit-tested without SUMO) --------------------------------------------
def map_action(action):
    """A normalized policy action [acceleration, lane] -> (requested m/s^2, lane request -1/0/1).

    As v05-v14: the action is cast to float32, checked against the hybrid action space, and the
    acceleration mapped affinely from [-1, 1] to [-4.5, 2] (environments/sumo_mappo_env.py).
    """
    action = np.asarray(action, np.float32)
    if not ACTION_SPACE.contains(action):
        raise ValueError('Invalid hybrid action')
    return action_to_acceleration(action[0]), int(action[1])


def acceleration_bounds(state, front, merge_distance, drive_ceiling=None):
    """Stage 1's speed, following and lane-end bounds, plus v14's drive bound when one is given.

    Returns (bounds, ceiling): ceiling = clip(min(2, bounds...), -4.5, 2), as v14's patched
    acceleration_constraints (environments/highway_flow_rl_v14_env.py _drive_limited).
    """
    bounds, ceiling = acceleration_constraints(state, front, merge_distance)
    if drive_ceiling is None:
        return bounds, ceiling
    return dict(bounds, drive_grade=drive_ceiling), float(np.clip(min(ceiling, drive_ceiling), -4.5, 2.))


def decision_override(*, rejected, causes, requested, applied, realized, refused, interrupted, saturation=None):
    """v05's override and its causes for one decision, then v14's drive-saturation exemption.

    causes: the decision's own causes (lane rejection causes, comfort_rate_limit, violated bounds).
    realized: the measured acceleration after the step, None for a car that left.
    Returns (override, sorted causes ([] without an override), drive_saturated, native_changed).
    """
    filter_changed = abs(applied - requested) > 1e-4
    native_changed = realized is not None and abs(realized - applied) > .05
    override = bool(rejected or filter_changed or native_changed or refused or interrupted)
    causes = list(causes) + (['native_realization_unattributed'] if native_changed else [])
    if refused:
        causes.append('native_lane_request_refused')
    if interrupted:
        causes.append('native_lane_maneuver_interrupted')
    if realized is not None and realized < -4.5 - .05:
        causes.append('emergency_braking_observed')
    listed = sorted(set(causes)) if override else []
    saturated = bool(saturation is not None and
                     saturation_module.saturated(dict(override=override, override_causes=listed), saturation))
    if saturated:   # applied as before, not counted as an intervention (v14 round 24)
        override, listed = False, []
    return override, listed, saturated, native_changed


def project_path(value):
    path = (ROOT / value).resolve()
    if not path.is_relative_to(ROOT.resolve()):
        raise ValueError('Artifacts must remain in this checkout')
    return path


class HighwayMergeV15Env(ParallelEnv):
    metadata = {'name': SCHEMA, 'render_modes': [], 'is_parallelizable': True}
    possible_agents = [f'agent_{i}' for i in range(MAX_AGENTS)]
    observation_size = OBSERVATION_SIZE
    state_size = STATE_SIZE

    # === construction ==========================================================================
    def __init__(self, network, run_directory, *, controlled_count, total_vehicles, horizon=800,
                 communication='local', packet_loss=0., through_lanes=4, task='merge', density='medium',
                 speed_limit_m_s=None, difficulty=0, traffic_mode='initial', missed_gap=False,
                 controlled_car_follow_model='ACC', lane_change_duration=3., stabilization_seconds=8.,
                 native_warmup_seconds=0., rate_limit='full', v11_settings=None, v12_settings=None,
                 v14_settings=None, follow_gate=None, drive_limit=None, road_model_manifest=None,
                 drive_saturation=None):
        """The v14 environment's execution inputs (the reward-only ones are not taken).

        density is recorded only: the v05 demand writer, which every v05-v14 environment uses, does not
        read it. rate_limit 'full' (v06: the 4 m/s^3 comfort rate limit on every decision) is the only
        conditioning implemented; v05's 'startup' handoff variant is not.
        """
        self.network = Path(network).resolve()
        self.output = Path(run_directory).resolve()
        if not self.output.is_relative_to(ROOT / 'outputs'):
            raise ValueError('Outputs must stay inside project outputs')
        if communication not in ('none', 'local') or not 0 <= packet_loss <= 1:
            raise ValueError('Unsupported channel')
        if task not in TASKS:
            raise ValueError('Unknown driving task')
        if isinstance(controlled_count, bool) or not isinstance(controlled_count, Integral) \
                or not 0 <= controlled_count <= MAX_AGENTS:
            raise ValueError('controlled_count must be an integer from zero to 24')
        if isinstance(total_vehicles, bool) or total_vehicles not in (8, 16, 24) or controlled_count > total_vehicles:
            raise ValueError('total_vehicles must be 8, 16 or 24 and include all AVs')
        if isinstance(difficulty, bool) or difficulty not in (0, 1, 2):
            raise ValueError('difficulty must be zero, one or two')
        if traffic_mode not in ('initial', 'sustained'):
            raise ValueError('traffic_mode must be initial or sustained')
        if traffic_mode == 'sustained' and controlled_count == total_vehicles:
            raise ValueError('Full-AV scenarios use initial traffic; later native inflow changes that fraction')
        if task in ('following', 'passing') and controlled_count == total_vehicles:
            raise ValueError('Following/passing requires a native slow leader within the total population')
        if not math.isfinite(lane_change_duration) or lane_change_duration < DT:
            raise ValueError('Lane-change duration must be finite and at least one tick')
        if not math.isfinite(stabilization_seconds) or not 0 < stabilization_seconds <= 30:
            raise ValueError('Post-merge stabilization must be finite, positive and at most 30 seconds')
        if not math.isfinite(native_warmup_seconds) or not 0 <= native_warmup_seconds <= 30 or \
                not math.isclose(native_warmup_seconds / DT, round(native_warmup_seconds / DT), abs_tol=1e-8):
            raise ValueError('Native warmup must be a whole number of ticks between zero and 30 seconds')
        if controlled_car_follow_model not in ('ACC', 'Krauss'):
            raise ValueError('Controlled car-following model must be ACC or Krauss')
        if rate_limit != 'full':
            raise ValueError("v15 Phase 0 implements v06's rate_limit='full' only")
        self.controlled_count, self.total_vehicles = int(controlled_count), int(total_vehicles)
        self.horizon, self.communication, self.packet_loss = horizon, communication, packet_loss
        self.task, self.density, self.difficulty, self.traffic_mode = task, density, int(difficulty), traffic_mode
        self.speed_limit_m_s = episode_speed_ceiling(speed_limit_m_s)
        self.missed_gap, self.controlled_car_follow_model = bool(missed_gap), controlled_car_follow_model
        self.lane_change_duration = float(lane_change_duration)
        self.stabilization_seconds = float(stabilization_seconds)
        self.native_warmup_seconds = float(native_warmup_seconds)
        self.rate_limit = rate_limit
        self.v11_settings = v11.validate_settings(v11_settings or {})
        self.v12_settings = v12.validate_settings(v12_settings or v12.SETTINGS)
        self.v14_settings = v14.validate_settings(v14_settings or v14.SETTINGS)
        self.follow_gate = None if follow_gate is None else follow_module.validate_settings(follow_gate)
        self.drive_limit = None if drive_limit is None else validate_drive_limit(drive_limit)
        if self.drive_limit is None and road_model_manifest is not None:
            raise ValueError('A road model for the grade needs the drive limit on')
        self.drive_saturation = (None if drive_saturation is None else
                                 saturation_module.validate_settings(drive_saturation))
        if self.drive_saturation is not None and self.drive_limit is None:
            raise ValueError('Drive saturation needs the drive limit on')
        self.graph = ContinuousMotionLaneGraph(self.network, through_lanes)
        self.road_grade = (None if self.drive_limit is None else
                           RoadGrade(network, road_model_manifest, spec=self.drive_limit['vehicle']))
        self.conn = None
        self.agents = []
        self.state_space = CRITIC_SPACE

    @classmethod
    def from_row(cls, cfg, row, directory):
        """The v15 environment of a v14 configuration row (experiments/run_highway_flow_rl_v14.py env_for)."""
        from traffic.highway_flow_contract import file_digest
        if cfg.get('v14_drive_target') is not None:
            raise ValueError("v15 Phase 0 reproduces round 24; v14's drive-aware target is not implemented")
        case = row['case']
        network = project_path(case['network'])
        if file_digest(network) != case['network_sha256']:
            raise ValueError('Frozen network changed')
        drive_limit, manifest = cfg.get('v14_drive_limit'), None
        if drive_limit is not None and case.get('road_model'):
            manifest = network.parent / 'corridor-manifest.json'
            if not manifest.is_file():
                raise ValueError(f"Case {case['id']} declares a road model but has no corridor manifest")
        env = cls(network, directory, controlled_count=row['controlled_count'], total_vehicles=row['total_vehicles'],
            horizon=row['horizon_steps'], communication=row['communication'], packet_loss=row['packet_loss'],
            through_lanes=case['through_lanes'], task=case['task'], density=case['density'],
            speed_limit_m_s=row['speed_limit_m_s'], difficulty=row['difficulty'], traffic_mode=row['traffic_mode'],
            missed_gap=row.get('missed_gap', False), controlled_car_follow_model=cfg['controlled_car_follow_model'],
            lane_change_duration=3., stabilization_seconds=8., rate_limit=cfg['rate_limit'],
            v11_settings=cfg['v11_settings'], v12_settings=cfg['v12_settings'], v14_settings=cfg.get('v14_settings'),
            follow_gate=cfg.get('v14_follow_gate'), drive_limit=drive_limit, road_model_manifest=manifest,
            drive_saturation=cfg.get('v14_drive_saturation'))
        env.case_id = case['id']
        return env

    def observation_space(self, agent):
        return ACTOR_SPACE

    def action_space(self, agent):
        return ACTION_SPACE

    def close(self):
        if self.conn is not None:
            try:
                self.conn.close()
            finally:
                self.conn = None

    def _controlled_agents(self):
        return tuple(self.possible_agents[:self.controlled_count])

    # === reset ===================================================================================
    def reset(self, seed=None, options=None):
        if options:
            raise ValueError('v15 is configured at construction')
        self.close()
        seed = 0 if seed is None else int(seed)
        controlled = self._controlled_agents()
        # R1 episode directory and the seeded message channel (its only random draws)
        self.rng = np.random.default_rng(seed)
        self.directory = self.output / f'episode-{seed}-{uuid.uuid4().hex[:8]}'
        self.directory.mkdir(parents=True)
        shutil.copyfile(self.network, self.directory / 'network.net.xml')
        # R2 demand: the v05 writer (v06's missed-gap option), ramp cars with an AV id are the merge agents
        self.population_manifest = write_demand(self.directory / 'routes.rou.xml', seed, self.graph, self.task,
            self.controlled_count, self.total_vehicles, speed_limit_m_s=self.speed_limit_m_s,
            difficulty=self.difficulty, traffic_mode=self.traffic_mode,
            stabilization_seconds=self.native_warmup_seconds, horizon_seconds=self.horizon * DT,
            controlled_car_follow_model=self.controlled_car_follow_model, missed_gap=self.missed_gap)
        self.merge_agents = tuple(v['vehicle_id'] for v in self.population_manifest
                                  if v['controlled'] and v['route'] == 'ramp')
        # R3 episode state
        self.previous = {a: (0., 0) for a in controlled}       # (raw request, requested lane delta)
        self.last_change = {a: -5. for a in controlled}
        self.inbox = {a: {} for a in controlled}
        self.steps = self.collision_count = self.teleport_count = 0
        self.states, self.arrived, self.collisions = {}, set(), set()
        self.maneuvers, self.completed_maneuvers = {}, {}
        self.completed_this_step, self.refused_this_step, self.interrupted_this_step = set(), {}, {}
        self._speed_limits_applied = False
        self.flow_contexts, self.trackers = {}, {}
        self.quality_history, self.stable_merged = {}, set()
        self._reference_targets, self._v14_cache, self._ramp, self._follow = {}, {}, {}, {}
        self._faded, self._target_inputs, self._augmented = {}, {}, {}
        self.agents = []
        # R4 SUMO start (i110_merge_env's command plus v05's lane-change settings)
        command = [str(find_sumo_binary()), '-n', str(self.directory / 'network.net.xml'),
            '-r', str(self.directory / 'routes.rou.xml'), '--seed', str(seed), '--step-length', '.1',
            '--no-step-log', 'true', '--duration-log.disable', 'true', '--time-to-teleport', '-1',
            '--collision.action', 'warn', '--collision.check-junctions', 'true',
            '--error-log', str(self.directory / 'sumo-errors.log'),
            '--lanechange.duration', str(self.lane_change_duration),
            '--lanechange-output', str(self.directory / 'lanechanges.xml'),
            '--lanechange-output.started', 'true', '--lanechange-output.ended', 'true']
        coupling = sumo_coupling.module()
        label = 'v15-' + uuid.uuid4().hex
        try:
            coupling.start(command, label=label, verbose=False)
            self.conn = coupling.getConnection(label)
            socket = getattr(self.conn, '_socket', None)
            if socket is not None:
                socket.settimeout(30)
            # R5 spawn: native steps until the whole initial cohort is on the road
            initial = {r['vehicle_id'] for r in self.population_manifest if r['initial_cohort']}
            for _ in range(SPAWN_ADVANCES):
                self._advance()
                if self.collision_count or self.teleport_count:
                    raise RuntimeError('Invalid spawn event')
                if initial.issubset(self.states):
                    break
            else:
                raise RuntimeError('Controlled spawn timeout; no vehicle discarded')
            # R6 SUMO's own checks on at spawn (superseded in R10), the fixed cohort
            for agent in controlled:
                self.conn.vehicle.setSpeedMode(agent, SPAWN_MODES['speed'])
                self.conn.vehicle.setLaneChangeMode(agent, SPAWN_MODES['lane_change'])
            self.agents = list(controlled)
            if len(self.states) != self.total_vehicles:
                raise RuntimeError('Population fixture did not spawn the entire fixed cohort at reset')
            # R7 native warmup (zero ticks unless requested)
            for _ in range(round(self.native_warmup_seconds / DT)):
                self._advance()
                if self.collision_count or self.teleport_count:
                    raise RuntimeError('Native collision or teleport during v05 stabilization')
            missing = set(controlled) - set(self.states)
            if missing:
                raise RuntimeError(f'Fixed AV cohort departed before measurement: {sorted(missing)}')
            self.start_time = self.time
            # R8 one post-merge tracker per merging car
            self.trackers = {a: PostMergeTracker(vehicle_id=a, window_s=self.stabilization_seconds)
                             for a in self.merge_agents}
            # R9 observe
            observations = self._actor_observations(self._raw_observations(self.agents))
            # R10 v14: the policy is the only controller (SUMO speed mode 96, lane change mode 0)
            for agent in controlled:
                if agent in self.states:
                    self.conn.vehicle.setSpeedMode(agent, CONTROL_MODES['speed'])
                    self.conn.vehicle.setLaneChangeMode(agent, CONTROL_MODES['lane_change'])
                    modes = dict(speed=self.conn.vehicle.getSpeedMode(agent),
                                 lane_change=self.conn.vehicle.getLaneChangeMode(agent))
                    if modes != CONTROL_MODES:
                        raise RuntimeError('SUMO did not accept the v14 control modes')
            return observations, {a: {} for a in self.agents}
        except BaseException:
            self.close()
            raise

    # === step ====================================================================================
    def step(self, actions):
        agents = list(self.agents)
        if not agents and not actions:
            self._advance()
            self.steps += 1
            return {}, {}, {}, {}, {}
        if set(actions) != set(agents):
            raise ValueError('One action for each active agent required')
        # S1 action map: the whole joint action is checked before any command
        requested, lane_requests = {}, {}
        for agent in agents:
            requested[agent], lane_requests[agent] = map_action(actions[agent])
        # S2 per-car merge-speed targets for this decision (v04's pre-action hook -> v14's target);
        # the gate's target grades the speed after the step (S8)
        for agent in agents:
            state = self.states[agent]
            self._v14_target(agent, state, self.flow_contexts[agent], map_evidence(self.graph, state))
            self._reference_targets[agent] = self._v14_cache[agent]['gate']
        before = self.states
        # S3 message snapshots: pre-step state with the previous decision's raw request and lane request
        snapshots = {a: dict(time=self.time, x=before[a]['x'], y=before[a]['y'], speed=before[a]['speed'],
            acceleration=before[a]['acceleration'], requires_merge=float(self.merge_distance(before[a]) is not None),
            request=self.previous[a][0], lane_intent=self.previous[a][1]) for a in agents}
        # S4 per-car command
        decisions = {}
        for agent in agents:
            state, context, delta = before[agent], self._context(agent), lane_requests[agent]
            # bounds: speed limit, own-lane following gap, lane end, drive grade
            bounds, ceiling = self._bounds(state, context['neighbors']['own']['front'], self.merge_distance(state))
            # lane request: rejection causes (the lane mask's reasons); an accepted change starts a maneuver
            causes = self._lane_rejection_causes(agent, context, delta)
            rejected = bool(causes)
            if delta and not rejected:
                self._begin_maneuver(agent, delta)
            # comfort rate limit (v05/v06: +-0.4 m/s^2 around the measured acceleration), then the ceiling
            conditioned = condition_acceleration(requested[agent], state['acceleration'], ceiling,
                                                 enabled=True)['conditioned']
            applied = min(conditioned, ceiling)
            if abs(conditioned - requested[agent]) > 1e-4:
                causes.append('comfort_rate_limit')
            violations = [k for k, v in bounds.items() if v is not None and requested[agent] > v + 1e-4]
            if abs(applied - requested[agent]) > 1e-4:
                causes += violations
            self.conn.vehicle.setAcceleration(agent, applied, DT)
            self.previous[agent] = (requested[agent], delta)
            decisions[agent] = dict(applied=applied, rejected=rejected, causes=causes)
        # S5 advance one 0.1 s tick
        self._advance()
        self.steps += 1
        # S6 messages: one tick latency, 150 m range from the receiver's pre-step position, seeded loss
        # (one draw per in-range pair, also at zero loss)
        for receiver in agents:
            if receiver not in self.states or self.communication == 'none':
                continue
            for sender, message in snapshots.items():
                if sender == receiver:
                    continue
                if math.hypot(message['x'] - before[receiver]['x'], message['y'] - before[receiver]['y']) > MESSAGE_RANGE_M:
                    continue
                if self.rng.random() < self.packet_loss:
                    continue
                self.inbox[receiver][sender] = message
        # S7 outcomes: override (with v14's drive saturation), termination, truncation
        terminated, truncated, infos, outcomes = {}, {}, {}, {}
        for agent in agents:
            state, decision = self.states.get(agent), decisions[agent]
            realized = state['acceleration'] if state else None
            override, causes, saturated, native_changed = decision_override(
                rejected=decision['rejected'], causes=decision['causes'], requested=requested[agent],
                applied=decision['applied'], realized=realized, refused=agent in self.refused_this_step,
                interrupted=agent in self.interrupted_this_step, saturation=self.drive_saturation)
            completed = agent in self.arrived
            terminated[agent] = completed or agent not in self.states or bool(self.collisions) or self.teleport_count > 0
            truncated[agent] = self.steps >= self.horizon and not terminated[agent]
            outcomes[agent] = dict(override=override, native_changed=native_changed, realized=realized,
                                   completed=completed)
            infos[agent] = dict(requested_acceleration=requested[agent], applied_acceleration=decision['applied'],
                lane_request=lane_requests[agent], lane_applied_delta=0 if decision['rejected'] else lane_requests[agent],
                lane_rejected=decision['rejected'], override=override, override_causes=causes,
                drive_saturated=saturated)
        # S8 merge trackers, then the speed-match history the lane mask reads
        stable = self._update_trackers(agents, requested, decisions, outcomes, terminated, truncated)
        self._update_speed_match(agents, stable)
        # S9 observe every car of this step that is still on the road; cars that left get zeros
        raw = self._raw_observations(agents)
        raw = {a: raw.get(a, np.zeros(RAW_SIZE, np.float32)) for a in agents}
        self.agents = [a for a in agents if not terminated[a] and not truncated[a]]
        observations = self._actor_observations(raw)
        rewards = {a: 0. for a in agents}     # v15's event rewards come later
        return observations, rewards, terminated, truncated, infos

    # === advance =================================================================================
    def _advance(self):
        # v05 population: lower every native lane limit to the episode ceiling before the first tick
        self._apply_episode_speed_limit()
        self.conn.simulationStep()
        sim, vehicle = self.conn.simulation, self.conn.vehicle
        self.time = sim.getTime()
        self.arrived = set(sim.getArrivedIDList())
        self.collisions = set(sim.getCollidingVehiclesIDList())
        teleports = sim.getStartingTeleportIDList()
        active = vehicle.getIDList()
        self.collision_count += len(self.collisions)
        self.teleport_count += len(teleports)
        self.states = {}
        for a in active:
            x, y = vehicle.getPosition(a)
            self.states[a] = dict(id=a, x=x, y=y, lane_id=vehicle.getLaneID(a), lane_position=vehicle.getLanePosition(a),
                route=vehicle.getRoute(a), speed=vehicle.getSpeed(a), acceleration=vehicle.getAcceleration(a),
                distance=vehicle.getDistance(a), allowed_speed=vehicle.getAllowedSpeed(a), length=vehicle.getLength(a))
        if self.task in ('passing', 'following') and 'slow_leader' in self.states:
            # i110's legacy leader speed, superseded below with a speed ceiling (no tick in between)
            vehicle.setSpeed('slow_leader', 10. if self.task == 'passing' else 16.)
            if self.speed_limit_m_s is not None:
                vehicle.setSpeed('slow_leader', population_leader_speed(self.task, self.speed_limit_m_s))
        for agent, state in self.states.items():
            state['lateral_position'] = float(vehicle.getLateralLanePosition(agent))
            state['lateral_speed'] = float(vehicle.getLateralSpeed(agent))
            state['angle'] = float(vehicle.getAngle(agent))
            state['lane_width'] = float(self.conn.lane.getWidth(state['lane_id']))
            state['lane_identity'] = self.graph.identity(state['lane_id'])
            if not all(math.isfinite(state[k]) for k in ('x', 'y', 'speed', 'acceleration', 'lateral_position',
                                                         'lateral_speed', 'angle', 'lane_width')):
                raise ValueError('Nonfinite native SUMO vehicle motion')
            if state['lane_width'] <= 0:
                raise ValueError('Invalid native lane width')
        self._update_maneuvers()

    def _apply_episode_speed_limit(self):
        """min(native limit, episode ceiling) on every lane incl. connectors, once, read back into the map."""
        if self.speed_limit_m_s is None or self._speed_limits_applied:
            return
        lane_api = self.conn.lane
        lane_ids = sorted(lane_api.getIDList())
        if not lane_ids or set(lane_ids) != set(self.graph.speed_limits):
            raise ValueError('Native lane inventory differs from the frozen map speed contract')
        pending = []
        for lane_id in lane_ids:
            original = float(lane_api.getMaxSpeed(lane_id))
            if not math.isfinite(original) or original <= 0:
                raise ValueError('Invalid native lane speed limit')
            pending.append((lane_id, min(original, self.speed_limit_m_s)))
        for lane_id, speed in pending:
            lane_api.setMaxSpeed(lane_id, speed)
        readback = {}
        for lane_id, speed in pending:
            readback[lane_id] = float(lane_api.getMaxSpeed(lane_id))
            if not math.isclose(readback[lane_id], speed, rel_tol=0, abs_tol=1e-6):
                raise ValueError('Native lane speed readback differs from requested curriculum ceiling')
        self.graph.speed_limits.update(readback)
        self._speed_limits_applied = True

    def _update_maneuvers(self):
        """v05 continuous motion: completion on the target corridor, native interruption or refusal."""
        self.completed_this_step, self.refused_this_step, self.interrupted_this_step = set(), {}, {}
        for agent, maneuver in list(self.maneuvers.items()):
            state = self.states.get(agent)
            if state is None:
                del self.maneuvers[agent]
                continue
            elapsed = self.time - maneuver['started_at']
            on_target = self._lane_identity_key(state['lane_id']) == maneuver['target_identity']
            moved = abs(state['lateral_position']) > .02 or abs(state['lateral_speed']) > .02 or on_target
            maneuver['moved'] |= moved
            lateral_fraction = state['lateral_position'] / state['lane_width']
            maneuver['remaining_fraction'] = float(np.clip(abs(lateral_fraction) if on_target
                else 1. - maneuver['direction'] * lateral_fraction, 0., 1.))
            if on_target and abs(state['lateral_position']) < .05 and abs(state['lateral_speed']) < .05:
                self.completed_maneuvers[agent] = dict(maneuver, completed_at=self.time, duration_s=elapsed)
                self.last_change[agent] = self.time
                self.completed_this_step.add(agent)
                del self.maneuvers[agent]
            elif (maneuver['moved'] and elapsed > DT + 1e-8
                  and abs(state['lateral_position']) < .05 and abs(state['lateral_speed']) < .05):
                self.interrupted_this_step[agent] = dict(maneuver, interrupted_at=self.time,
                    reason='native_settled_without_target', settled_lane=state['lane_id'])
                del self.maneuvers[agent]
            elif elapsed > self.lane_change_duration + 2 * DT and not maneuver['moved']:
                self.refused_this_step[agent] = dict(maneuver)
                del self.maneuvers[agent]

    # === command pieces ==========================================================================
    def _context(self, agent):
        return self.graph.observe(agent, self.states, self.time - self.start_time, self.horizon * .1,
                                  self.previous[agent][0])

    def merge_distance(self, state):
        """Distance to the end of an ending lane on the route, or None when the route needs no merge."""
        distances = self.graph.distances(state['lane_id'], state['lane_position'], state['route'])
        if any(self.graph.lanes[l]['edge'] == state['route'][-1] for l in distances):
            return None
        ends = [d + self.graph.lanes[l]['length'] for l, d in distances.items()
                if not (self.graph.successors[l] & set(distances))]
        return max(0., min(ends)) if ends else 0.

    def _drive_ceiling(self, state):
        if self.road_grade is None:
            return None
        limit = self.drive_limit
        return self.road_grade.ceiling(state['lane_id'], state['lane_position'], state['speed'],
            headroom_m_s2=limit['headroom_m_s2'], lookahead_s=limit['lookahead_s'])['ceiling_m_s2']

    def _bounds(self, state, front, merge_distance):
        return acceleration_bounds(state, front, merge_distance, self._drive_ceiling(state))

    def _lane_identity_key(self, lane_id):
        return json.dumps(self.graph.identity(lane_id), sort_keys=True)

    def _begin_maneuver(self, agent, delta):
        """v05: one absolute target-lane request held for the SUMO lane-change duration plus a tick."""
        state = self.states[agent]
        target_lane, known = self.graph.adjacent(state['lane_id'], delta)
        if not known or target_lane is None:
            raise ValueError('Cannot commit a maneuver without a known target lane')
        target_index = self.graph.lanes[target_lane]['index']
        maneuver = dict(source_lane=state['lane_id'], target_lane=target_lane,
            source_identity=self._lane_identity_key(state['lane_id']),
            target_identity=self._lane_identity_key(target_lane), target_index=target_index,
            direction=int(delta), started_at=self.time, moved=False, remaining_fraction=1.)
        # An immediate unnecessary return to the previous lane fails the merge window (tracker input).
        previous = self.completed_maneuvers.get(agent)
        front = self._context(agent)['neighbors']['own']['front']
        unnecessary = bool(previous and previous['source_identity'] == maneuver['target_identity']
            and self.time - previous['completed_at'] <= 8.
            and self.merge_distance(state) is None and self.task != 'passing'
            and (front is None or front['relative_speed_m_s'] >= -1.)
            and not any(m['requires_merge'] for _, m in self._selected_messages(agent)))
        maneuver.update(reversal_reason='unnecessary' if unnecessary else None, reason_evidence=unnecessary)
        self.conn.vehicle.changeLane(agent, target_index, self.lane_change_duration + DT)
        self.maneuvers[agent] = maneuver

    # --- lane mask: i110 gap check, v05 maneuver/runway, v04 projected gaps, v11 merge-speed gate ---
    def _gap_mask(self, agent, context):
        """i110: no change on a connector or within 5 s of the last change; current gaps both ways."""
        s, mask = self.states[agent], [0., 1., 0.]
        if self.graph.lanes[s['lane_id']]['internal'] or self.time - self.last_change[agent] < 5:
            return mask
        for label, index in (('right', 0), ('left', 2)):
            if not context['lane_exists'][label] or not context['adjacent_context_known']:
                continue
            safe = True
            for direction in ('front', 'rear'):
                n = context['neighbors'][label][direction]
                if n:
                    follower = s['speed'] if direction == 'front' else max(0., s['speed'] + n['relative_speed_m_s'])
                    leader = max(0., s['speed'] + n['relative_speed_m_s']) if direction == 'front' else s['speed']
                    required = 2.5 + max(1.5 * follower, follower * .1 + follower * follower / 9 - leader * leader / 18)
                    safe &= n['bumper_gap_m'] >= required
            if safe:
                mask[index] = 1.
        return mask

    def _lane_change_runway(self, agent, delta):
        """v05: the connected target corridor must cover speed*T + 0.5*2*T^2 + 2.5 m, T = duration + 0.1 s."""
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
        return dict(available_m=available, required_m=required, permitted=available + 1e-8 >= required)

    def _motion_mask(self, agent, context):
        """v05: stay only while a maneuver runs; the gap mask less changes without runway."""
        if agent in self.maneuvers:
            return [0., 1., 0.]
        mask = self._gap_mask(agent, context)
        for delta in (-1, 1):
            if mask[delta + 1]:
                runway = self._lane_change_runway(agent, delta)
                if runway is not None and not runway['permitted']:
                    mask[delta + 1] = 0.
        return mask

    def _projected_mask(self, agent, context):
        """v04: the v05 mask less changes whose current or 3.1 s projected gaps are short."""
        mask = self._motion_mask(agent, context)
        for label, index in (('right', 0), ('left', 2)):
            if mask[index] and not maneuver_gap_evidence(self.states[agent]['speed'], context['neighbors'][label])['permitted']:
                mask[index] = 0.
        return mask

    def _merge_speed_gate(self, agent):
        """v11: a car that must leave its lane changes only after a matched-speed run or near the lane end."""
        state = self.states.get(agent)
        if state is None or not self.v11_settings['merge_dwell_decisions']:
            return None
        flow = flow_context(self.graph, state, self.states)
        route, entry = flow['corridors']['own']['route'], flow['entry']
        if route.get('reachable', True) or not route.get('known') or not entry.get('known') or not entry.get('lane_delta'):
            return None
        gate = v11.merge_speed_precondition(v11.matched_run(self.quality_history.get(agent)),
                                            route['distance_m'], state['speed'], self.v11_settings)
        gate['lane_delta'] = int(entry['lane_delta'])
        return gate

    def _mask(self, agent, context):
        """The lane mask (actor inputs 30-32; v11 on v04 on v05 on i110)."""
        mask = self._projected_mask(agent, context)
        gate = self._merge_speed_gate(agent)
        if gate is not None and not gate['permitted'] and mask[gate['lane_delta'] + 1]:
            mask[gate['lane_delta'] + 1] = 0.
        return mask

    def _lane_rejection_causes(self, agent, context, delta):
        """Why a lane request is refused; [] accepts it. Non-empty exactly where the mask blocks it."""
        if not delta:
            return []
        state = self.states[agent]
        if agent in self.maneuvers:
            causes = ['maneuver_in_progress']       # v05 stops here; v04 and v11 still add their causes
        else:
            causes = []                              # stage 1
            if self.graph.lanes[state['lane_id']]['internal']:
                causes.append('lane_internal_segment')
            if self.time - self.last_change[agent] < 5:
                causes.append('lane_cooldown')
            if not context['adjacent_context_known']:
                causes.append('lane_context_unknown')
            if not context['lane_exists']['left' if delta > 0 else 'right']:
                causes.append('lane_unavailable')
            if not self._mask(agent, context)[delta + 1] and not causes:
                causes.append('lane_gap')
            runway = self._lane_change_runway(agent, delta)    # v05
            if runway is not None and not runway['permitted']:
                if self._gap_mask(agent, context)[delta + 1]:
                    causes = [cause for cause in causes if cause != 'lane_gap']
                causes.append('lane_change_runway')
        if not maneuver_gap_evidence(state['speed'], context['neighbors']['left' if delta == 1 else 'right'])['permitted']:
            causes.append('projected_maneuver_gap')            # v04
        gate = self._merge_speed_gate(agent)                   # v11
        if gate is not None and delta == gate['lane_delta'] and not gate['permitted']:
            if self._projected_mask(agent, context)[delta + 1]:
                causes = [cause for cause in causes if cause != 'lane_gap']
            causes.append('merge_speed_precondition')
        return causes

    # === merge-speed target (v14 ramp and follow gate) ==========================================
    def _v14_target(self, agent, state, context, preview):
        """One target per car and simulation time (v14 _v14_target): returns the actor's target, caches
        the gate's. Called before each decision (S2) and for the actor inputs of a pending merge (S9)."""
        cached = self._v14_cache.get(agent)
        if cached is not None and abs(cached['time'] - self.time) < 1e-9:
            return cached['target'], cached['plan']
        desired, plan, ceiling = v14.merge_speed_target(state, context, preview)
        active = v14.yield_entry(context)
        ramp = self._ramp.get(agent)
        target = desired
        eligible = bool(desired and desired.get('eligible'))
        if not eligible:
            self._ramp.pop(agent, None)
        elif active or ramp is not None:
            # yield entries: the target moves at +1.5 / -2.0 m/s^2 until it meets the plan past the line
            value = v14.reachable(desired['target_m_s'], ceiling, state['speed'], ramp, self.time, self.v14_settings)
            target = deepcopy(desired)
            target['target_m_s'] = value
            if not active and abs(value - desired['target_m_s']) < 1e-9:
                self._ramp.pop(agent, None)
            else:
                self._ramp[agent] = dict(time=self.time, value=value)
        gate = target
        if self.follow_gate is not None:
            gate, target = self._follow_targets(agent, state, desired, ceiling, active, eligible, target)
        self._v14_cache[agent] = dict(time=self.time, target=target, gate=gate, plan=plan)
        return target, plan

    def _follow_targets(self, agent, state, desired, ceiling, active, eligible, fallback):
        """v14 follow gate (round 23): (gate target, actor target) from a bounded-jerk reference that starts
        at the car's speed at its first yield-entry decision and ends after the stable merge."""
        settings = self.follow_gate
        reference = self._follow.get(agent)
        if (reference is None and not (eligible and active)) or not eligible:
            return fallback, fallback
        rise = follow_module.rise_rate(self._drive_ceiling(state), settings)
        acceleration = state.get('acceleration', 0.)
        road = desired['target_m_s']
        if reference is None:
            reference = follow_module.start(state['speed'], acceleration, ceiling, rise, self.time, settings)
        else:
            reference, _ = follow_module.advance(reference, road, ceiling, rise, state['speed'], acceleration,
                                                 self.time, settings)
        lower, upper = follow_module.band(reference['value'], road)
        cap = follow_module.reward_cap(desired, settings)
        actor_value = reference['value'] if cap is None else min(reference['value'], cap)
        if agent in self.stable_merged and abs(reference['value'] - road) <= settings['end_tolerance_m_s']:
            self._follow.pop(agent, None)
        else:
            self._follow[agent] = reference
        band = dict(road_target_m_s=road, reference_m_s=reference['value'], lower_m_s=lower, upper_m_s=upper)
        gate, target = deepcopy(desired), deepcopy(desired)
        gate['target_m_s'], target['target_m_s'] = reference['value'], actor_value
        gate['follow_gate'] = target['follow_gate'] = band
        return gate, target

    def _current_target(self, agent):
        """The actor's target while the car's merge is pending (v12/v14 current_target)."""
        state = self.states.get(agent)
        context = self.flow_contexts.get(agent)
        tracker = self.trackers.get(agent)
        pending = bool(tracker and tracker.status == 'pending')
        if state is None or context is None or not pending:
            return None, pending, state
        target, _ = self._v14_target(agent, state, context, map_evidence(self.graph, state))
        return target, pending, state

    def _acceptance_target(self, agent):
        """The gate's speed-match target; with the follow gate, the band point nearest the car's speed."""
        reference = self._reference_targets[agent]
        band = (reference or {}).get('follow_gate') if self.follow_gate is not None else None
        state = self.states.get(agent)
        if band is None or state is None or not reference.get('eligible'):
            return reference
        return dict(reference, target_m_s=follow_module.acceptance(state['speed'], band['lower_m_s'], band['upper_m_s']))

    # === S8 trackers and the speed-match history ================================================
    def _update_trackers(self, agents, requested, decisions, outcomes, terminated, truncated):
        """v05's post-merge trackers of the merging cars; returns the cars whose stable merge completed now."""
        stable = set()
        for agent in agents:
            tracker = self.trackers.get(agent)
            if tracker is None:
                continue
            state, outcome = self.states.get(agent), outcomes[agent]
            if state is not None:
                lane = self._lane_identity_key(state['lane_id'])
                if state['lane_identity'].get('kind') == 'through':
                    tracker.mark_entry(self.time)
                    if agent not in self.maneuvers and abs(state['lateral_position']) < .05 \
                            and abs(state['lateral_speed']) < .05:
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
                    emergency=state['acceleration'] < -4.55, intervention=outcome['override'], lane=lane,
                    neighbor_braking_mps2=braking, requested_acceleration=requested[agent],
                    filtered_acceleration=decisions[agent]['applied'], realized_acceleration=outcome['realized'],
                    native_intervention=outcome['native_changed'],
                    reversal_reason=maneuver.get('reversal_reason'), reason_evidence=maneuver.get('reason_evidence', False))
            if terminated[agent] or truncated[agent]:
                tracker.finish(arrived=outcome['completed'], truncated=truncated[agent])
            if tracker.consume_stable_bonus():
                stable.add(agent)
        return stable

    def _update_speed_match(self, agents, stable):
        """v04's samples: did the speed after this decision match the gate's target (within 2 m/s)? The
        history keeps the 8 s before the merge completed (or before now); v11's gate counts its trailing run."""
        for agent in agents:
            state = self.states.get(agent)
            speed = state['speed'] if state else None
            target = self._acceptance_target(agent)
            history = self.quality_history.setdefault(agent, [])
            history.append(dict(time_s=self.time, speed_match=bool(
                speed is not None and target['eligible'] and abs(speed - target['target_m_s']) <= SPEED_MATCH_M_S)))
            tracker = self.trackers.get(agent)
            completed_at = tracker.merge_complete_time_s if tracker else None
            start = (completed_at if completed_at is not None else self.time) - QUALITY_WINDOW_S
            history[:] = [r for r in history if r['time_s'] >= start]
            if agent in stable:
                self.stable_merged.add(agent)

    # === observe =================================================================================
    def _selected_messages(self, agent):
        """The nearest six fresh messages from senders still on the road (stable sender-id tie break)."""
        state = self.states[agent]
        eligible = [(sender, message) for sender, message in self.inbox[agent].items()
                    if sender in self.states and sender != agent and
                    -1e-8 <= self.time - message['time'] <= MESSAGE_AGE_S + 1e-8]
        eligible.sort(key=lambda pair: (math.hypot(state['x'] - pair[1]['x'], state['y'] - pair[1]['y']), pair[0]))
        return eligible[:MESSAGE_SLOTS]

    def _motion_features(self, agent):
        state, maneuver = self.states[agent], self.maneuvers.get(agent)
        last = self.completed_maneuvers.get(agent)
        return [float(np.clip(state['lateral_position'] / state['lane_width'], -1., 1.)),
            float(np.clip(state['lateral_speed'] / 3., -1., 1.)),
            float(maneuver['direction']) if maneuver else 0., float(maneuver is not None),
            maneuver['remaining_fraction'] if maneuver else 0.,
            min(1., max(0., (self.time - last['completed_at']) / 5.)) if last else 1.]

    def _raw_observations(self, agents):
        """The 117 raw fields (highway_flow_env): 0-25 graph features, 26-29 lane count / merge distance /
        requires merge / cooldown, 30-32 lane mask, 33-44 entry and acceleration ceiling (incl. the drive
        bound), 45-50 motion, 51-68 corridor extras, 69-116 six message slots. Stores each car's flow context."""
        result = {}
        for agent in agents:
            if agent not in self.states:
                continue
            state, context = self.states[agent], self._context(agent)
            distance = self.merge_distance(state)
            _, ceiling = self._bounds(state, context['neighbors']['own']['front'], distance)
            flow = flow_context(self.graph, state, self.states)
            local = context['features'][:26] + [context['lane_count'] / 7.,
                min(1., distance / 240.) if distance is not None else 1., float(distance is not None),
                max(0., 1 - (self.time - self.last_change[agent]) / 5), *self._mask(agent, context)]
            local += merge_features(flow['entry'], ceiling) + self._motion_features(agent)
            slots = []
            if self.communication == 'local':
                for _, message in self._selected_messages(agent):
                    age = self.time - message['time']
                    slots.extend([np.clip((state['y'] - message['y']) / 150., -1, 1), message['requires_merge'],
                        np.clip((message['speed'] - state['speed']) / CAP, -1, 1), np.clip(message['acceleration'] / 4.5, -1, 1),
                        message['request'] / 4.5, min(1., age / .5), message['lane_intent'], 1.])
            vector = inject_observation(local + slots + [0.] * (48 - len(slots)), flow['extras'])
            if not RAW_SPACE.contains(vector):
                raise ValueError('Invalid flow actor observation')
            result[agent] = vector
            self.flow_contexts[agent] = flow
        return result

    def _actor_observations(self, raw):
        """v06: live cars' merge inputs fade over 1 s; v12: four target inputs (faded likewise) at 69..72.
        A car that left gets its raw fields and raw target inputs, neither faded nor stored."""
        faded = {}
        for agent, observation in raw.items():
            if agent in self.agents:
                faded[agent] = self._faded[agent] = v06.fade_observation(self._faded.get(agent), observation)
        result = {}
        for agent, observation in raw.items():
            target, pending, state = self._current_target(agent)
            inputs = v12.target_features(target, state['speed'] if state else None, pending, self.v12_settings)
            if agent in self.agents:
                inputs = self._target_inputs[agent] = v12.fade(self._target_inputs.get(agent), inputs, self.v12_settings)
            result[agent] = v12.insert(faded.get(agent, observation), inputs)
            if not ACTOR_SPACE.contains(result[agent]):
                raise ValueError('Invalid v15 actor observation')
            if agent in self.agents:
                self._augmented[agent] = result[agent]
        return result

    def state(self):
        """Critic state: each of the 24 slots' current actor observation (zeros when absent) and a live flag."""
        return np.concatenate([np.r_[self._augmented[a] if a in self.agents else np.zeros(OBSERVATION_SIZE),
                                     float(a in self.agents)] for a in self.possible_agents]).astype(np.float32)
