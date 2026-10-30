"""Four decentralized longitudinal actors and a separate centralized critic state."""

from __future__ import annotations

from copy import deepcopy
from functools import lru_cache
import math

import gymnasium as gym
import numpy as np
from pettingzoo import ParallelEnv

from traffic.mappo_sumo import AGENT_IDS, MappoSumoBackend
from traffic.mappo_driving import DrivingMixin, validate_driving

DEFAULT_CONFIG = {
    "communication": None, "wave": None, "driving": None,
    "cruise_speed_m_s": 26.8224,
    "dt": 0.1, "horizon": 600, "speed_limit_m_s": 26.8224, "route_length_m": 5000.0,
    "neighbor_range_m": 120.0, "acceleration_min_m_s2": -4.5, "acceleration_max_m_s2": 2.0,
    "min_gap_m": 2.5, "time_headway_s": 1.5, "initial_speed_range_m_s": [12.0, 22.0],
    "background_depart_interval_range_s": [1.5, 3.0],
    "reward_weights": {"progress": 1.0, "headway": -0.5, "jerk": -0.02,
                       "collision": -10.0, "intervention": -0.1},
}
observation_features = (
    "speed_over_cap", "acceleration_over_4p5", "lane_index_over_2", "remaining_route_fraction",
    "speed_limit_over_cap", "previous_requested_acceleration_over_4p5",
    *(f"{lane}_{direction}_{feature}" for lane in ("own", "left", "right")
      for direction in ("front", "rear") for feature in ("gap_over_120", "relative_speed_over_cap", "present")),
    "left_lane_exists", "right_lane_exists", "elapsed_fraction",
)
OBSERVATION_SIZE = len(observation_features)
STATE_SIZE = len(AGENT_IDS) * (OBSERVATION_SIZE + 1)


def action_to_acceleration(action: float) -> float:
    """Affine normalized [-1,1] action to physical [-4.5,2.0] m/s²."""
    value = float(action)
    if not math.isfinite(value) or not -1.0 <= value <= 1.0:
        raise ValueError("Normalized acceleration must be finite and in [-1,1]")
    return -4.5 + (value + 1.0) * 3.25


def physical_acceleration_to_action(acceleration: float) -> float:
    value = float(acceleration)
    if not math.isfinite(value) or not -4.5 <= value <= 2.0:
        raise ValueError("Physical acceleration must be finite and in [-4.5,2.0]")
    return (value + 4.5) / 3.25 - 1.0


class ManeuverActionSpace(gym.spaces.Box):
    """Hybrid action encoded as two floats; sampled lane requests are discrete."""
    def __init__(self):
        super().__init__(-1.,1.,(2,),np.float32)
    def sample(self,mask=None):
        action=super().sample()
        action[1]=self.np_random.choice([-1.,0.,1.])
        return action
    def contains(self,x):
        return super().contains(x) and float(x[1]) in (-1.,0.,1.)


class SumoMappoEnv(DrivingMixin, ParallelEnv):
    metadata = {"name": "sumo_mappo_highway_v01", "render_modes": [], "is_parallelizable": True}
    observation_features = observation_features

    def __init__(self, config=None, backend_factory=None, run_directory=None):
        self.config = deepcopy(DEFAULT_CONFIG)
        if config:
            unknown = set(config) - set(DEFAULT_CONFIG)
            if unknown:
                raise ValueError(f"Unknown environment settings: {sorted(unknown)}")
            self.config.update(deepcopy(config))
        self._validate_config()
        self.driving = self.config["driving"] is not None
        self.observation_features = observation_features
        if self.driving:
            self.observation_features += ("leader_cooperative", "required_gap_over_120", "lane_cooldown_over_5", "mask_right", "mask_stay", "mask_left")
        self.communication = None
        self.observation_size = len(self.observation_features)
        if self.config["communication"] is not None:
            from traffic.mappo_communication import V2VChannel
            self.communication = V2VChannel(self.config["communication"])
            self.message_slots = int(self.config["communication"].get("max_neighbors", 6))
            names = ("relative_position_over_range", "lane_delta_over_2", "relative_speed_over_cap",
                     "acceleration_over_4p5", "intended_acceleration_over_4p5", "age_over_expiry", "present")
            if self.driving:
                names = names[:-1] + ("previous_lane_intent", "present")
            self.observation_features = self.observation_features + tuple(
                f"message_{i}_{name}" for i in range(self.message_slots) for name in names)
            self.observation_size = len(self.observation_features)
        self.state_size = len(AGENT_IDS) * (self.observation_size + 1)
        self.dt = float(self.config["dt"])
        self.horizon = int(self.config["horizon"])
        self.possible_agents = list(AGENT_IDS)
        self.agents = []
        self._backend_factory = backend_factory or (lambda: MappoSumoBackend(self.config, run_directory))
        self._backend = None
        self._result = None
        self._step_count = 0
        self._previous_requests = {agent: 0.0 for agent in self.possible_agents}
        self._previous_lane_requests = {agent: 0 for agent in self.possible_agents}
        self._last_lane_changes = {}
        self._pass_candidates = {agent: {} for agent in self.possible_agents}
        self._passed_pairs = set()
        self._last_observations = {}
        self.control_mode = "policy"
        self.state_space = gym.spaces.Box(-1.0, 1.0, (self.state_size,), np.float32)

    def _validate_config(self):
        cfg = self.config
        if cfg["driving"] is not None:
            validate_driving(cfg["driving"])
            if cfg["communication"] is None or cfg["communication"].get("max_neighbors") != 6 or cfg["wave"] is not None:
                raise ValueError("v04 requires six-slot communication and no wave fixture")
        for key in ("dt", "speed_limit_m_s", "route_length_m", "neighbor_range_m", "min_gap_m", "time_headway_s"):
            if not math.isfinite(float(cfg[key])) or cfg[key] <= 0:
                raise ValueError(f"{key} must be finite and positive")
        if not isinstance(cfg["horizon"], int) or cfg["horizon"] < 1:
            raise ValueError("horizon must be a positive integer")
        if cfg["dt"] != 0.1 or cfg["route_length_m"] != 5000.0 or cfg["speed_limit_m_s"] != 26.8224:
            raise ValueError("v01 fixes dt=0.1, route_length_m=5000 and speed_limit_m_s=26.8224")
        if cfg["acceleration_min_m_s2"] != -4.5 or cfg["acceleration_max_m_s2"] != 2.0:
            raise ValueError("v01 acceleration and observation scales are fixed at [-4.5,2.0]")
        if cfg["neighbor_range_m"] != 120.0 or cfg["min_gap_m"] != 2.5 or cfg["time_headway_s"] != 1.5:
            raise ValueError("v01 neighbor range, vehicle minGap and headway are fixed at 120,2.5,1.5")
        for key in ("initial_speed_range_m_s", "background_depart_interval_range_s"):
            value = cfg[key]
            if len(value) != 2 or not all(math.isfinite(float(x)) for x in value) or not 0 < value[0] <= value[1]:
                raise ValueError(f"Invalid range for {key}")
        if cfg["initial_speed_range_m_s"][1] > cfg["speed_limit_m_s"]:
            raise ValueError("Initial speeds exceed the ceiling")
        if not 0 < cfg['cruise_speed_m_s'] <= cfg['speed_limit_m_s']:
            raise ValueError('Cruise speed must be positive and no greater than the ceiling')
        required = set(DEFAULT_CONFIG['reward_weights'])
        if not required.issubset(cfg['reward_weights']) or set(cfg['reward_weights']) - required - {'cruise_excess', 'traffic_progress', 'wave_fluctuation', 'passing', 'lane_change', 'cooperative_tracking', 'lane_change_rejected'}:
            raise ValueError("Reward weights must name every v01 component")
        if cfg['driving'] is None and set(cfg['reward_weights']).intersection({'passing','lane_change','cooperative_tracking','lane_change_rejected'}):
            raise ValueError('Maneuver reward components require v04 driving')
        wave = cfg.get("wave")
        if wave is not None:
            expected = {"enabled", "pulse_start_s_range", "pulse_duration_s", "pulse_acceleration_m_s2", "detector_position_m"}
            if set(wave) != expected or wave["enabled"] is not True:
                raise ValueError("Wave settings must explicitly specify the complete enabled pulse contract")
            lo, hi = wave["pulse_start_s_range"]
            if not 0 <= lo <= hi or hi + wave["pulse_duration_s"] >= cfg["dt"] * cfg["horizon"]:
                raise ValueError("Pulse must finish before the episode horizon")
            if not 0 < wave["pulse_duration_s"] or not -4.5 <= wave["pulse_acceleration_m_s2"] < 0:
                raise ValueError("Pulse must have positive duration and feasible braking")
            if not 0 < wave["detector_position_m"] < cfg["route_length_m"]:
                raise ValueError("Detector must lie inside the road")
        if not all(math.isfinite(float(x)) for x in cfg["reward_weights"].values()):
            raise ValueError("Reward weights must be finite")

    @lru_cache(maxsize=None)
    def observation_space(self, agent):
        self._validate_agent(agent)
        low = np.zeros(self.observation_size, dtype=np.float32)
        low[[1, 5, 7, 10, 13, 16, 19, 22]] = -1.0
        low[OBSERVATION_SIZE:] = -1.0
        return gym.spaces.Box(low, np.ones(self.observation_size, dtype=np.float32), dtype=np.float32)

    @lru_cache(maxsize=None)
    def action_space(self, agent):
        self._validate_agent(agent)
        return ManeuverActionSpace() if self.driving else gym.spaces.Box(-1.0, 1.0, (1,), np.float32)

    def reset(self, seed=None, options=None):
        # PettingZoo permits an arbitrary options dictionary; v01 has no option-dependent reset behavior.
        del options
        seed = 42 if seed is None else int(seed)
        if not 0 <= seed <= 2**31 - 1:
            raise ValueError("SUMO seed must fit a nonnegative signed 32-bit integer")
        self.close()
        self.np_random, _ = gym.utils.seeding.np_random(seed)
        self._backend = self._backend_factory()
        try:
            self._result = self._backend.reset(seed=seed)
            if not set(self.possible_agents).issubset(self._result.states):
                raise RuntimeError("All four controlled agents must exist after initial spawn")
            self.agents = list(self.possible_agents)
            self._step_count = 0
            self._previous_requests = {agent: 0.0 for agent in self.possible_agents}
            self._previous_lane_requests = {agent: 0 for agent in self.possible_agents}
            self._last_lane_changes = {}
            self._pass_candidates = {agent: {} for agent in self.possible_agents}
            self._passed_pairs = set()
            self._episode_start_time = self._result.simulation_time
            if self.communication is not None:
                self.communication.reset(seed, episode_id=str(seed))
                self._broadcast()
            observations = self._observations(self.agents)
            return observations, {agent: {"seed": seed, "simulation_time": self._result.simulation_time,
                "active_agents": tuple(self.agents)} for agent in self.agents}
        except BaseException:
            self.close()
            raise

    def step(self, actions):
        if not self.agents:
            if actions:
                raise KeyError("No active agents remain")
            return {}, {}, {}, {}, {}
        transition_agents = list(self.agents)
        if self.control_mode not in ("policy", "native"):
            raise ValueError("control_mode must be policy or native")
        if set(actions) != set(transition_agents):
            raise KeyError("Provide exactly one action for every active agent")
        requested, filtered, reasons = {}, {}, {}
        lane_requests = {}
        # Validate the full joint action before modifying either the simulator or history.
        for agent in transition_agents:
            action = np.asarray(actions[agent], dtype=np.float32)
            if not self.action_space(agent).contains(action):
                raise ValueError(f"Action outside declared space for {agent}: {action}")
            requested[agent] = action_to_acceleration(float(action[0]))
            if self.driving: lane_requests[agent] = int(action[1])
        for agent in transition_agents:
            filtered[agent], reasons[agent] = self._safety_filter(agent, requested[agent])
        before = self._result
        if self.driving:
            lane_filtered,lane_reasons = self._filter_lanes(lane_requests)
            for agent in transition_agents:
                reasons[agent] += tuple(lane_reasons[agent])
                if lane_filtered[agent]:
                    destination_acceleration,destination_reasons=self._driving_safety_filter(
                        agent,filtered[agent],lane_delta=lane_filtered[agent],ordinary=True)
                    filtered[agent]=min(filtered[agent],destination_acceleration)
                    reasons[agent] += tuple('destination_'+reason for reason in destination_reasons)
                neighbor=self._neighbor(agent,0,True)
                if neighbor and neighbor[1]<self.config["cruise_speed_m_s"]-self.config["driving"]["pass_speed_deficit_m_s"]:
                    self._pass_candidates[agent].setdefault(neighbor[2],self._step_count)
            headways={agent:self._headway(agent)[0] for agent in transition_agents}
            for agent in transition_agents:
                if lane_filtered[agent]:headways[agent]=self.config['driving']['ordinary_headway_s']
            self._result=self._backend.step(filtered if self.control_mode=="policy" else {},
                lane_changes=lane_filtered if self.control_mode=="policy" else {},
                headways=headways if self.control_mode=="policy" else {})
            self._previous_lane_requests.update(lane_requests if self.control_mode=="policy" else {a:0 for a in transition_agents})
        else:
            self._result = self._backend.step(filtered if self.control_mode == "policy" else {})
        self._step_count += 1
        if not math.isclose(self._result.simulation_time - before.simulation_time, self.dt, abs_tol=1e-7):
            raise RuntimeError("Backend did not advance exactly one control interval")
        self._previous_requests.update(requested if self.control_mode == "policy" else
                                       {agent: 0.0 for agent in transition_agents})
        self._broadcast()
        if self.driving:
            for agent in transition_agents:
                state=self._result.states.get(agent)
                if state and state["lane_index"]!=before.states[agent]["lane_index"]:
                    self._last_lane_changes[agent]=self._step_count*self.dt
        observations = self._observations(transition_agents)
        wave_metrics = self._wave_metrics(before)
        any_collision = bool(self._result.collisions)
        any_teleport = bool(self._result.teleported)
        terminations, truncations, infos, local_rewards = {}, {}, {}, {}
        for agent in transition_agents:
            state, prior = self._result.states.get(agent), before.states[agent]
            arrived = agent in self._result.arrived
            missing = state is None
            terminations[agent] = any_collision or any_teleport or arrived or missing
            truncations[agent] = self._step_count >= self.horizon and not terminations[agent]
            progress = max(0.0, float(state["distance"] - prior["distance"])) if state else 0.0
            acceleration = float(state["acceleration"]) if state else None
            neighbor = self._neighbor(agent, 0, True)
            gap = neighbor[0] if neighbor else None
            desired_gap = self.config["min_gap_m"] + self.config["time_headway_s"] * state["speed"] if state else 0.0
            cooperative = False
            lane_applied = lane_rejected = False
            if self.driving and state:
                desired_gap,cooperative=self._required_gap(agent,neighbor)
                if self.control_mode != 'policy':cooperative=False
                lane_applied=state["lane_index"] != prior["lane_index"]
                lane_rejected=bool(lane_requests[agent] and (not lane_applied or int(state["lane_index"]-prior["lane_index"])!=lane_requests[agent])) and self.control_mode=="policy"
                if lane_applied:self._last_lane_changes[agent]=self._step_count*self.dt
                if lane_rejected and lane_filtered[agent]:reasons[agent] += ("sumo_lane_rejected",)
            sumo_intervention = (self.control_mode == "policy" and acceleration is not None
                                 and (abs(acceleration - filtered[agent]) > 1e-4 or (self.driving and lane_rejected and bool(lane_filtered[agent]))))
            local_intervention = self.control_mode == "policy" and bool(reasons[agent])
            intervention = local_intervention or sumo_intervention
            components = {
                "progress": float(np.clip(progress / (self.config["cruise_speed_m_s"] * self.dt), 0, 1)),
                "headway": float(np.clip((desired_gap - gap) / desired_gap, 0, 1)) if gap is not None else 0.0,
                "jerk": float(np.clip(abs(acceleration - prior["acceleration"]) / self.dt / 10.0, 0, 1)) if state else 0.0,
                "collision": float(agent in self._result.collisions),
                "intervention": float(intervention),
            }
            if 'cruise_excess' in self.config['reward_weights']:
                target = self.config['cruise_speed_m_s']
                components['cruise_excess'] = float(np.clip(
                    ((state['speed'] if state else 0.) - target) /
                    max(1e-6, self.config['speed_limit_m_s'] - target), 0, 1))
            if "traffic_progress" in self.config["reward_weights"]:
                components["traffic_progress"] = wave_metrics["traffic_progress_normalized"]
            if "wave_fluctuation" in self.config["reward_weights"]:
                components["wave_fluctuation"] = wave_metrics["traffic_acceleration_abs_normalized"]
            pass_count=0
            if self.driving:
                for other in self._pass_candidates[agent]:
                    target=self._result.states.get(other)
                    pair=(agent,other)
                    if state and target and pair not in self._passed_pairs and state["position"]-state["length"]>target["position"]+self.config["driving"]["pass_clearance_m"]:
                        self._passed_pairs.add(pair);pass_count+=1
                additions=dict(passing=float(pass_count),lane_change=float(lane_applied),lane_change_rejected=float(lane_rejected),
                    cooperative_tracking=float(cooperative and gap is not None and desired_gap<=gap<=1.2*desired_gap))
                components.update({k:v for k,v in additions.items() if k in self.config["reward_weights"]})
            weighted = {key: self.config["reward_weights"][key] * value for key, value in components.items()}
            local_rewards[agent] = float(sum(weighted.values()))
            infos[agent] = dict(simulation_time=self._result.simulation_time, episode_step=self._step_count,
                speed_m_s=float(state["speed"]) if state else None, acceleration_m_s2=acceleration,
                front_gap_m=gap, nearest_gap_m=gap, progress_m=progress,
                relative_closing_speed_m_s=float(state["speed"] - neighbor[1]) if neighbor else None,
                min_ttc_s=float(gap / (state["speed"] - neighbor[1]))
                    if neighbor and state["speed"] > neighbor[1] else None,
                distance_m=float(state["distance"]) if state else None,
                lane_index=int(state["lane_index"]) if state else None,
                requested_acceleration_m_s2=requested[agent] if self.control_mode == "policy" else None,
                filtered_acceleration_m_s2=filtered[agent] if self.control_mode == "policy" else None,
                action_source="policy" if self.control_mode == "policy" else "SUMO_native",
                realized_acceleration_m_s2=acceleration, local_safety_intervention=local_intervention,
                sumo_safety_intervention=sumo_intervention, safety_intervention=intervention,
                safety_reasons=reasons[agent] if self.control_mode == "policy" else (),
                reward_components=components, weighted_reward_components=weighted,
                local_reward=local_rewards[agent], active_agents=tuple(transition_agents),
                background_vehicle_count=sum(vehicle not in self.possible_agents for vehicle in self._result.states),
                collisions=self._result.collisions, teleported=self._result.teleported,
                termination_cause=("collision" if any_collision else "teleport" if any_teleport else
                                   "route_complete" if arrived else "unexpected_disappearance" if missing else None),
                truncation_cause="time_limit" if truncations[agent] else None,
                observation_available=state is not None)
            if self.driving:
                infos[agent]["driving_metrics"]=dict(lane_change_requested=int(bool(lane_requests[agent]) and self.control_mode=='policy'),lane_change_applied=int(lane_applied),
                    lane_change_rejected=int(lane_rejected),pass_completed=pass_count,headway_s=gap/state["speed"] if gap is not None and state and state["speed"]>1e-6 else None,
                    cooperative_headway_active=cooperative,required_gap_m=desired_gap,
                    applied_native_headway_s=headways[agent] if self.control_mode=='policy' else self.config['driving']['ordinary_headway_s'],
                    cooperative_headway_applied=self.control_mode=='policy' and headways[agent]==self.config['driving']['cooperative_headway_s'],
                    abs_jerk_m_s3=abs(acceleration-prior["acceleration"])/self.dt if state else 0.,
                    requested_lane_delta=lane_requests[agent],filtered_lane_delta=lane_filtered[agent],
                    actual_lane_delta=int(state["lane_index"]-prior["lane_index"]) if state else 0)
        # A time limit needs the measured final state for V(s_{t+1}); true terminals have no bootstrap.
        bootstrap_agents = [agent for agent in transition_agents if not terminations[agent]]
        terminal_state = self._state_for(bootstrap_agents)
        shared_reward = float(np.mean(list(local_rewards.values())))
        rewards = {agent: shared_reward for agent in transition_agents}
        for info in infos.values():
            info["team_reward"] = shared_reward
            if self.config["wave"] is not None:
                info["wave_metrics"] = dict(wave_metrics)
            if self.communication is not None:
                info["communication"] = self.communication.telemetry()
            info["terminal_state"] = terminal_state.copy()
        self.agents = [agent for agent in transition_agents if not terminations[agent] and not truncations[agent]]
        return observations, rewards, terminations, truncations, infos

    def _broadcast(self):
        if self.communication is None:
            return
        fleet = {agent: dict(position_m=state["position"], lane=state["lane_index"],
                 speed_m_s=state["speed"], acceleration_m_s2=state["acceleration"],
                 intended_acceleration_m_s2=self._previous_requests[agent])
                 for agent, state in self._result.states.items() if agent in self.possible_agents}
        if self.driving:
            for agent in fleet:fleet[agent]["intended_lane_delta"]=self._previous_lane_requests[agent]
        self.communication.broadcast(self._step_count * self.dt, fleet)

    def _wave_metrics(self, before):
        states = self._result.states
        # Spatially upstream followers receive the braking wave; "downstream" denotes its propagation.
        leader = states.get("background_initial_0_1")
        followers = [s for key, s in states.items() if s["lane_index"] == 0
                     and (leader is None or s["position"] < leader["position"])]
        speeds = [s["speed"] for s in followers]
        detector = (self.config["wave"] or {}).get("detector_position_m", 1000.)
        crossings = sum(s["lane_index"] == 0 and key in before.states and before.states[key]["position"] < detector <= s["position"]
                        for key, s in states.items())
        common = [(before.states[key], s) for key, s in states.items() if key in before.states and s["lane_index"] == 0]
        affected = [s for s in states.values() if s["lane_index"] == 0]
        progress = sum(max(0., s["distance"] - prior["distance"]) for prior, s in common)
        return dict(elapsed_s=self.dt, downstream_speed_sum=float(sum(speeds)),
                    downstream_speed_squared_sum=float(sum(v*v for v in speeds)),
                    downstream_speed_count=len(speeds), throughput_count=int(crossings),
                    traffic_progress_m=float(progress),
                    pulse_leader_present=bool(leader is not None),
                    pulse_leader_speed_m_s=float(leader["speed"]) if leader else None,
                    pulse_leader_acceleration_m_s2=float(leader["acceleration"]) if leader else None,
                    traffic_progress_normalized=float(np.mean([min(1., s["speed"] / self.config["cruise_speed_m_s"]) for s in affected])) if affected else 0.,
                    traffic_acceleration_abs_normalized=float(np.mean([min(1., abs(s["acceleration"])/4.5) for s in affected])) if affected else 0.)

    def _neighbor(self, agent, lane_delta, ahead):
        ego = self._result.states.get(agent)
        if ego is None:
            return None
        lane = ego["lane_index"] + lane_delta
        if not 0 <= lane < 3:
            return None
        candidates = []
        for other, state in self._result.states.items():
            if other == agent or state["lane_index"] != lane:
                continue
            delta = state["position"] - ego["position"]
            if (ahead and delta >= 0) or (not ahead and delta < 0):
                # SUMO lane positions locate front bumpers. Subtract the vehicle between the bumpers.
                gap = max(0.0, abs(delta) - (state["length"] if ahead else ego["length"]))
                if gap <= self.config["neighbor_range_m"]:
                    candidates.append((gap, state["speed"], other))
        return min(candidates, default=None, key=lambda value: (value[0], value[2]))

    def _observations(self, agents):
        observations = {}
        for agent in agents:
            state = self._result.states.get(agent)
            if state is None:
                observation = np.zeros(self.observation_size, dtype=np.float32)
            else:
                speed = state["speed"]
                cap = self.config["speed_limit_m_s"]
                features = [np.clip(speed / cap, 0, 1), np.clip(state["acceleration"] / 4.5, -1, 1),
                    state["lane_index"] / 2, np.clip(1 - state["position"] / self.config["route_length_m"], 0, 1),
                    np.clip(state["speed_limit"] / cap, 0, 1), self._previous_requests[agent] / 4.5]
                for lane_delta in (0, 1, -1):
                    for ahead in (True, False):
                        neighbor = self._neighbor(agent, lane_delta, ahead)
                        features.extend([neighbor[0] / self.config["neighbor_range_m"],
                                         np.clip((neighbor[1] - speed) / cap, -1, 1), 1.0] if neighbor else [0, 0, 0])
                features.extend([float(state["lane_index"] < 2), float(state["lane_index"] > 0),
                                 min(1.0, self._step_count / self.horizon)])
                if self.driving:
                    desired,cooperative=self._required_gap(agent)
                    cooldown=max(0.,self.config["driving"]["lane_change_cooldown_s"]-(self._step_count*self.dt-self._last_lane_changes.get(agent,-math.inf)))
                    features.extend([float(cooperative),min(1.,desired/120.),min(1.,cooldown/5.),*self.lane_action_mask(agent)])
                if self.communication is not None:
                    now = self._step_count * self.dt
                    messages = self.communication.receive(agent, now, state["position"])
                    channel_cfg = self.config["communication"]
                    for message in messages:
                        features.extend([np.clip((message.position_m - state["position"]) / channel_cfg.get("range_m", 150.), -1, 1),
                            np.clip((message.lane - state["lane_index"]) / 2, -1, 1),
                            np.clip((message.speed_m_s - speed) / cap, -1, 1),
                            np.clip(message.acceleration_m_s2 / 4.5, -1, 1),
                            np.clip(message.intended_acceleration_m_s2 / 4.5, -1, 1),
                            np.clip((now - message.timestamp_s) / channel_cfg.get("expiry_s", .5), 0, 1)])
                        if self.driving:features.append(float(message.intended_lane_delta or 0))
                        features.append(1.)
                    features.extend([0.] * ((8 if self.driving else 7) * (self.message_slots - len(messages))))
                observation = np.asarray(features, dtype=np.float32)
                if not self.observation_space(agent).contains(observation):
                    raise ValueError(f"Invalid observation for {agent}: {observation}")
            observations[agent] = observation
            self._last_observations[agent] = observation
        return observations

    def _safety_filter(self, agent, requested):
        if self.driving:return self._driving_safety_filter(agent,requested)
        speed = self._result.states[agent]["speed"]
        filtered = min(requested, (self.config["speed_limit_m_s"] - speed) / self.dt)
        reasons = ["speed_ceiling"] if filtered < requested - 1e-8 else []
        neighbor = self._neighbor(agent, 0, True)
        if neighbor is not None:
            gap, _, _ = neighbor
            safe_speed = max(0.0, (gap - self.config["min_gap_m"]) / self.config["time_headway_s"])
            # The local guard only uses the same current-lane neighbor available to the actor.
            safe_acceleration = max(-4.5, (safe_speed - speed) / self.dt)
            if safe_acceleration < filtered - 1e-8:
                filtered = safe_acceleration
                reasons.append("headway")
        stopping_bound = -speed / self.dt
        if filtered < stopping_bound - 1e-8:
            filtered = stopping_bound
            reasons.append("zero_speed")
        return float(np.clip(filtered, -4.5, 2.0)), tuple(reasons)

    def _state_for(self, active_agents):
        active = set(active_agents)
        return np.concatenate([np.concatenate((self._last_observations[agent], [1])).astype(np.float32)
            if agent in active else np.zeros(self.observation_size + 1, dtype=np.float32)
            for agent in self.possible_agents])

    def state(self):
        return self._state_for(self.agents)

    def close(self):
        backend, self._backend = self._backend, None
        self.agents = []
        self._result = None
        self._last_observations = {}
        if backend is not None:
            backend.close()

    def _validate_agent(self, agent):
        if agent not in self.possible_agents:
            raise KeyError(agent)


def parallel_env(**kwargs):
    return SumoMappoEnv(**kwargs)
