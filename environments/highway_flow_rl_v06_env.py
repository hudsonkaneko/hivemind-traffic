"""V06 merge objective on the v05 continuity environment (Phase 1, code only).

Changes against v05 (documentation/highway-merge-training-plan-v06.md, Phase 1):
1. merge speed reward tracks the SAME unsmoothed v04 reference target the
   rolling-merge-quality gate scores, with a larger weight;
2. optional missed-gap demand (traffic/highway_v05_demand.py) and a metric for
   reaching the last 20% of the acceleration lane unmerged;
3. the actor's merge inputs fade over 1 s; the safety guard, mask and lane-change
   permission keep the raw values (the critic sees the actors' faded inputs);
4. stronger jerk_excess on max(requested, realized) jerk, and a per-car raw
   requested jerk check for evaluation (traffic.highway_flow_rl_v06_rewards.v06_gates);
5. the gap-preparation potential fades from 1 to the plan's value over 1 s;
6. a small cost on avoidable braking requests.
Safety shield, native guards, action contract and observation size are unchanged.
"""
from copy import deepcopy

from environments.highway_environment_v05 import DT
from environments.highway_flow_rl_v05_env import HighwayFlowRLV05Env
from traffic.highway_flow_context import flow_context
from traffic.highway_flow_rl_v04_rewards import gap_plan, merge_speed_target, validate_weights
from traffic import highway_flow_rl_v06_rewards as v06

RATE_LIMITS = ('full', 'startup')


class HighwayFlowRLV06Env(HighwayFlowRLV05Env):
    def __init__(self, *args, rate_limit='full', v06_weights=None, missed_gap=False, raw_request_scope='all', **kwargs):
        if rate_limit not in RATE_LIMITS:
            raise ValueError('rate_limit must be full (whole episode) or startup (handoff only)')
        if raw_request_scope not in v06.RAW_REQUEST_SCOPES:
            raise ValueError('Unknown raw-request scope')
        self.raw_request_scope = raw_request_scope
        weights = dict(v06.WEIGHTS if v06_weights is None else v06_weights)
        if weights != v06.WEIGHTS:
            raise ValueError('Unreviewed v06 reward weights')
        if 'v05_variant' in kwargs or 'v05_weights' in kwargs:
            raise ValueError('V06 selects the v05 conditioning through rate_limit')
        self.rate_limit, self.v06_weights, self.missed_gap = rate_limit, weights, bool(missed_gap)
        self._reset_v06()
        # 'full' = whole-episode 4 m/s3 command rate limit; 'startup' = handoff only.
        super().__init__(*args, v05_variant=rate_limit, **kwargs)
        self.v04_reward_weights = validate_weights({k: weights[k] for k in v06.BASE_KEYS})

    def _reset_v06(self):
        self._faded, self._required_run = {}, {}
        self.v06_metrics = dict(per_car={}, decisions=0, faded_input_changes=0,
                                avoidable_braking_charged=0, late_lane_decisions=0, reward_terms={})

    def reset(self, seed=None, options=None):
        self._reset_v06()
        observations, infos = super().reset(seed=seed, options=options)
        return self._fade(observations), infos

    def step(self, actions):
        observations, rewards, terminated, truncated, infos = super().step(actions)
        live = {a: o for a, o in observations.items() if a in self.agents}
        faded = self._fade(live)
        return {a: faded.get(a, o) for a, o in observations.items()}, rewards, terminated, truncated, infos

    def state(self):
        """Critic state from the same (faded) observations the actors receive.

        The MAPPO runner builds the successor critic state from returned
        observations, so the current state must use them too.
        """
        import numpy as np
        from environments.highway_flow_env import OBSERVATION_SIZE
        return np.concatenate([np.r_[self._faded[a] if a in self.agents else np.zeros(OBSERVATION_SIZE),
                                     float(a in self.agents)] for a in self.possible_agents]).astype(np.float32)

    def _fade(self, observations):
        result = {}
        for agent, raw in observations.items():
            value = v06.fade_observation(self._faded.get(agent), raw)
            self.v06_metrics['faded_input_changes'] += int(abs(value - raw).max() > 1e-6)
            self._faded[agent] = value
            result[agent] = value
        return result

    # --- merge speed target and guidance potential -------------------------
    def _reward_target(self, agent, state, context, preview):
        target, plan = merge_speed_target(state, context, preview)  # the gate's own target
        self._reference_targets[agent] = target
        run = self._required_run.get(agent)
        if run is None:
            run = int(bool(plan.get('required')))
            self._required_run[agent] = run
        return target, v06.blend_potential(plan, run)

    def _successor_plan(self, agent, state):
        if not state:
            return dict(potential=0.)
        plan = gap_plan(state, flow_context(self.graph, state, self.states))
        run = self._required_run.get(agent, 0) + 1 if plan.get('required') else 0
        self._required_run[agent] = run
        return v06.blend_potential(plan, run)

    # --- added reward terms -------------------------------------------------
    def _extra_rewards(self, agent, info, snapshot, audit, state, merge_active):
        w = self.v06_weights
        ego = snapshot['ego_state']
        front = (snapshot['context'].get('corridors', {}).get('own', {}) or {}).get('front')
        diagnosis = info.get('v04_reward_audit', {}).get('avoidable_braking') or {}
        terms = dict(
            jerk_excess=v06.jerk_excess(info['realized_jerk_m_s3'], audit['raw_requested_jerk'], w['jerk_excess']),
            merge_tracking=v06.merge_tracking(snapshot['target'], state['speed'] if state else None,
                                              merge_active, w['merge_tracking']),
            avoidable_braking=v06.avoidable_braking(diagnosis, audit['requested'], ego['speed'], front,
                                                    w['avoidable_braking']))
        lane = self.graph.lanes.get(ego['lane_id'], {})
        late = v06.late_in_acceleration_lane(snapshot['context'], lane.get('length'), merge_active)
        car = self.v06_metrics['per_car'].setdefault(agent, dict(decisions=0, late_lane_decisions=0,
            first_late_lane_time_s=None, avoidable_braking_charged=0,
            **{v06.FIRST_DECISION_EXCLUDED_KEY: 0., v06.INSERTION_LAG_EXCLUDED_KEY: 0.}))
        raw = abs(audit['raw_requested_jerk'])
        if agent in self._v05_seen:  # not this car's first decision (insertion at 0 m/s2)
            car[v06.FIRST_DECISION_EXCLUDED_KEY] = max(car[v06.FIRST_DECISION_EXCLUDED_KEY], raw)
        if car['decisions'] >= v06.INSERTION_LAG_DECISIONS:  # past the fixed insertion-lag window
            car[v06.INSERTION_LAG_EXCLUDED_KEY] = max(car[v06.INSERTION_LAG_EXCLUDED_KEY], raw)
        for row in (car, self.v06_metrics):
            row['decisions'] += 1
            row['late_lane_decisions'] += int(late)
            row['avoidable_braking_charged'] += int(terms['avoidable_braking'] != 0.)
        if late and car['first_late_lane_time_s'] is None:
            car['first_late_lane_time_s'] = round(snapshot['time_s'], 1)
        for key, value in terms.items():
            self.v06_metrics['reward_terms'][key] = self.v06_metrics['reward_terms'].get(key, 0.) + value
        info['v06_audit'] = dict(late_acceleration_lane=late, gap_plan=deepcopy(snapshot['gap_plan']),
                                 avoidable_braking_free_road=v06.free_road(ego['speed'], front))
        return terms

    # --- missed-gap demand --------------------------------------------------
    def _write_demand(self, path, seed):
        if not self.missed_gap or getattr(self, 'fixture', None) is not None:
            return super()._write_demand(path, seed)
        from traffic.highway_v05_demand import write_demand
        self.population_manifest = write_demand(path, seed, self.graph, self.task,
            self.controlled_count, self.total_vehicles, speed_limit_m_s=self.speed_limit_m_s,
            difficulty=self.difficulty, traffic_mode=self.traffic_mode,
            stabilization_seconds=self.native_warmup_seconds, horizon_seconds=self.horizon * DT,
            controlled_car_follow_model=self.controlled_car_follow_model, missed_gap=True)
        self.merge_agents = tuple(v['vehicle_id'] for v in self.population_manifest
                                  if v['controlled'] and v['route'] == 'ramp')

    def report(self):
        result = super().report()
        result.update(schema='highway_flow_rl_merge_env_v06', reward_version=v06.REWARD_VERSION,
            objective_version=v06.REWARD_VERSION, control_observation_version=v06.OBSERVATION_VERSION,
            v06_rate_limit=self.rate_limit, v06_weights=dict(self.v06_weights),
            v06_weights_status=v06.WEIGHTS_STATUS, v06_settings=dict(v06.SETTINGS),
            v06_fade_indices=list(v06.FADE_INDICES), v06_missed_gap=self.missed_gap,
            v06_metrics=deepcopy(self.v06_metrics), v06_raw_request_scope=self.raw_request_scope,
            v06_scope='Actor (and critic) merge inputs faded; safety guard, mask, lane permission and journaled observation117 stay raw')
        return result
