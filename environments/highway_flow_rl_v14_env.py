"""V14 merge environment: the policy is the only controller of the controlled cars.

Changes against v12 (traffic/highway_flow_rl_v14_rewards.py):
- control: after reset every controlled car gets SUMO speed mode 96 and lane change mode 0,
  so SUMO executes the environment's filtered command as given. The environment's own
  limits are unchanged (speed limit, following gap, lane end, comfort rate limit, the
  lane-change mask and runway checks). The runner checks both modes at every decision.
- target: at yield-junction entries the gap plan also checks the gaps from crossing the
  line, and the target changes no faster than v05's +1.5 / -2.0 m/s^2 until it meets the
  plan's target after the line. Both target hooks (_reward_target before the decision and
  current_target for the actor inputs) share one computation per car and step.
After the scored horizon, cars still on the road are handed to native SUMO for the drain
accounting, as in every earlier version; v14 restores speed mode 31 for that drain, which
is never counted as a learned merge or route success.
Observation widths, rewards, weights and gate thresholds are v12's.

Optional drive limit (user decision, November 19, 2026: "Yes, build it"; off unless the
constructor gets drive_limit=traffic.road_grade.DRIVE_LIMIT): SUMO's cars ignore grade, the
physics car does not. With the limit on, every controlled car's acceleration ceiling also
holds the force balance of the prepared Americano at full drive torque on the steepest road
grade it reaches within 0.2 s, less 0.1 m/s^2 of headroom (traffic/road_grade.py). The
grade is the road model's, for cases that declare one (road_model_manifest); other
networks are flat. The bound joins the existing ones everywhere they are used: the applied
command, the unsafe-request reward, the override causes ('drive_grade') and the ceiling
feature of the observation, whose width does not change.

Optional stop fix (user decision, November 21, 2026: "Yes, build and train"; off unless the
constructor gets stop_fix=traffic.highway_flow_rl_v14_stop_fix.SETTINGS): two reward terms,
merge_braking (v06's avoidable-braking cost for pending merges whose target needs no slowing) and
target_band_tail (the band's slope past v12's cap), and the reward version
highway_flow_rl_merge_v14_merge_braking. Every v14 environment also measures stopped decisions,
those asking to brake, and free-road stops (report field v14_stops); measuring changes no reward.
"""
from contextlib import contextmanager
from copy import deepcopy

import numpy as np

from environments import highway_environment_v05 as _v05_module
from environments import highway_flow_env as _flow_module
from environments.highway_flow_rl_v12_env import HighwayFlowRLV12Env
from environments.highway_stage1_env import acceleration_constraints as _base_constraints
from traffic.road_grade import RoadGrade, validate_drive_limit
from traffic import highway_flow_rl_v06_rewards as v06
from traffic import highway_flow_rl_v14_rewards as v14
from traffic import highway_flow_rl_v14_stop_fix as stop_fix_module
from traffic.highway_flow_context import flow_context
from traffic.highway_flow_rl_v03_rewards import map_evidence
from traffic.highway_flow_rl_v04_rewards import gap_plan as v04_gap_plan

NATIVE_DRAIN_SPEED_MODE = 31  # SUMO default, the mode every earlier version drained with


class HighwayFlowRLV14Env(HighwayFlowRLV12Env):
    def __init__(self, *args, v14_settings=None, drive_limit=None, road_model_manifest=None, stop_fix=None, **kwargs):
        self.v14_settings = v14.validate_settings(v14_settings or v14.SETTINGS)
        self.stop_fix = None if stop_fix is None else stop_fix_module.validate_settings(stop_fix)
        self.drive_limit = None if drive_limit is None else validate_drive_limit(drive_limit)
        if self.drive_limit is None and road_model_manifest is not None:
            raise ValueError('A road model for the grade needs the drive limit on')
        self.road_grade = (None if self.drive_limit is None else
                           RoadGrade(args[0], road_model_manifest, spec=self.drive_limit['vehicle']))
        self._drive_patched = False
        self.drive_stats = dict(decisions=0, limited=0, max_grade=-np.inf, min_ceiling_m_s2=np.inf)
        super().__init__(*args, **kwargs)

    # --- optional drive limit -------------------------------------------------------------
    def _drive_bound(self, state):
        limit = self.drive_limit
        info = self.road_grade.ceiling(state['lane_id'], state['lane_position'], state['speed'],
                                       headroom_m_s2=limit['headroom_m_s2'], lookahead_s=limit['lookahead_s'])
        stats = self.drive_stats
        stats['max_grade'] = max(stats['max_grade'], info['limiting_grade'])
        stats['min_ceiling_m_s2'] = min(stats['min_ceiling_m_s2'], info['ceiling_m_s2'])
        return info['ceiling_m_s2']

    @contextmanager
    def _drive_limited(self):
        """The v05 step and the flow observations read acceleration_constraints from their modules;
        while this env runs, both see the same constraints plus the drive bound."""
        if self.road_grade is None or self._drive_patched:
            yield
            return
        modules = (_v05_module, _flow_module)
        if any(module.acceleration_constraints is not _base_constraints for module in modules):
            raise RuntimeError('acceleration_constraints is already replaced')

        def constraints(state, front, merge_distance):
            bounds, ceiling = _base_constraints(state, front, merge_distance)
            drive = self._drive_bound(state)
            return dict(bounds, drive_grade=drive), float(np.clip(min(ceiling, drive), -4.5, 2.))

        self._drive_patched = True
        for module in modules:
            module.acceleration_constraints = constraints
        try:
            yield
        finally:
            for module in modules:
                module.acceleration_constraints = _base_constraints
            self._drive_patched = False

    def _observations(self):
        with self._drive_limited():
            return super()._observations()

    def step(self, actions):
        with self._drive_limited():
            result = super().step(actions)
        if self.road_grade is not None:
            for info in result[4].values():
                if isinstance(info, dict) and 'drive_grade' in (info.get('acceleration_bounds') or {}):
                    self.drive_stats['decisions'] += 1
                    self.drive_stats['limited'] += int('drive_grade' in (info.get('override_causes') or ()))
        return result

    # --- control: one controller ------------------------------------------------------
    def reset(self, seed=None, options=None):
        result = super().reset(seed=seed, options=options)
        modes = {}
        for agent in self._controlled_agents():
            if agent in self.states:
                self.conn.vehicle.setSpeedMode(agent, v14.CONTROL['speed_mode'])
                self.conn.vehicle.setLaneChangeMode(agent, v14.CONTROL['lane_change_mode'])
                modes[agent] = dict(speed=self.conn.vehicle.getSpeedMode(agent),
                                    lane_change=self.conn.vehicle.getLaneChangeMode(agent))
        if any(mode != dict(speed=v14.CONTROL['speed_mode'], lane_change=v14.CONTROL['lane_change_mode'])
               for mode in modes.values()):
            raise RuntimeError('SUMO did not accept the v14 control modes')
        self.v14_control = dict(modes=modes, scope='every controlled car from the first decision')
        return result

    def finish_accounting(self):
        for agent in self._controlled_agents():
            if agent in self.states:
                self.conn.vehicle.setSpeedMode(agent, NATIVE_DRAIN_SPEED_MODE)
        return super().finish_accounting()

    # --- target ---------------------------------------------------------------------------
    def _reset_v06(self):
        super()._reset_v06()
        self.drive_stats = dict(decisions=0, limited=0, max_grade=-np.inf, min_ceiling_m_s2=np.inf)
        self._v14_cache, self._ramp = {}, {}
        self.v14_metrics = dict(yield_decisions=0, crossing_changed_target=0, crossing_limited_plans=0,
                                ramp_decisions=0, ramp_limited=0, hook_mismatch=0, per_car={})
        self.v14_stops = dict(decisions=0, stopped=0, asking_to_brake=0, asking_to_brake_overrides=0,
                              free_road=0, per_car={})
        self.v14_stop_fix_metrics = dict(merge_braking_charged=0, merge_braking=0., band_tail_charged=0,
                                         target_band_tail=0., per_car={})

    def _v14_target(self, agent, state, context, preview):
        """One target per car and simulation time, shared by both hooks."""
        desired, plan, ceiling = v14.merge_speed_target(state, context, preview)
        cached = self._v14_cache.get(agent)
        if cached is not None and abs(cached['time'] - self.time) < 1e-9:
            if (desired or {}).get('target_m_s') != (cached['desired'] or {}).get('target_m_s'):
                self.v14_metrics['hook_mismatch'] += 1
            return cached['target'], cached['plan']
        active = v14.yield_entry(context)
        ramp = self._ramp.get(agent)
        target = desired
        eligible = bool(desired and desired.get('eligible'))
        if not eligible:
            self._ramp.pop(agent, None)
        elif active or ramp is not None:
            value = v14.reachable(desired['target_m_s'], ceiling, state['speed'], ramp, self.time,
                                  self.v14_settings)
            target = deepcopy(desired)
            target['target_m_s'] = value
            target['v14'] = dict(desired_m_s=desired['target_m_s'], ceiling_m_s=ceiling,
                                 ramp_started=ramp is None, yield_entry=active,
                                 crossing_deficit=plan.get('crossing_deficit'),
                                 crossing_time_s=plan.get('crossing_time_s'))
            if not active and abs(value - desired['target_m_s']) < 1e-9:
                self._ramp.pop(agent, None)    # past the line and caught up: v12's target from here
            else:
                self._ramp[agent] = dict(time=self.time, value=value)
            self._count(agent, state, context, desired, plan, value, active)
        self._v14_cache[agent] = dict(time=self.time, desired=desired, target=target, plan=plan)
        return target, plan

    def _count(self, agent, state, context, desired, plan, value, active):
        car = self.v14_metrics['per_car'].setdefault(agent, dict(yield_decisions=0, crossing_changed_target=0,
            crossing_limited_plans=0, ramp_decisions=0, ramp_limited=0))
        changed = limited = False
        if active:
            old = v04_gap_plan(state, context)
            changed = bool(old.get('known') and plan.get('known') and
                           abs(old.get('target_m_s', 0.) - plan.get('target_m_s', 0.)) > 1e-9)
            limited = bool((plan.get('crossing_deficit') or 0.) > 1e-9)
        for row in (self.v14_metrics, car):
            row['yield_decisions'] += int(active)
            row['crossing_changed_target'] += int(changed)
            row['crossing_limited_plans'] += int(limited)
            row['ramp_decisions'] += 1
            row['ramp_limited'] += int(abs(value - desired['target_m_s']) > 1e-9)

    def _reward_target(self, agent, state, context, preview):
        target, plan = self._v14_target(agent, state, context, preview)
        self._reference_targets[agent] = target   # the gate's speed match grades the same target
        run = self._required_run.get(agent)
        if run is None:
            run = int(bool(plan.get('required')))
            self._required_run[agent] = run
        return target, v06.blend_potential(plan, run)

    def _successor_plan(self, agent, state):
        if not state:
            return dict(potential=0.)
        plan = v14.gap_plan(state, flow_context(self.graph, state, self.states), settings=self.v14_settings)
        run = self._required_run.get(agent, 0) + 1 if plan.get('required') else 0
        self._required_run[agent] = run
        return v06.blend_potential(plan, run)

    def current_target(self, agent):
        state = self.states.get(agent)
        context = getattr(self, 'flow_contexts', {}).get(agent)
        tracker = getattr(self, 'merge_trackers', {}).get(agent)
        pending = bool(tracker and tracker.status == 'pending')
        if state is None or context is None or not pending:
            return None, pending, state
        target, _ = self._v14_target(agent, state, context, map_evidence(self.graph, state))
        return target, pending, state

    def _extra_rewards(self, agent, info, snapshot, audit, state, merge_active):
        terms = super()._extra_rewards(agent, info, snapshot, audit, state, merge_active)
        info['v14_audit'] = deepcopy((snapshot.get('target') or {}).get('v14'))
        speed = snapshot['ego_state']['speed']
        front = ((snapshot['context'].get('corridors') or {}).get('own') or {}).get('front')
        sample = stop_fix_module.stop_sample(speed, audit['requested'], front, snapshot['target'],
                                             info.get('native_realization_changed'))
        car = self.v14_stops['per_car'].setdefault(agent, dict(stopped=0, asking_to_brake=0,
                                                              asking_to_brake_overrides=0, free_road=0))
        self.v14_stops['decisions'] += 1
        for row in (self.v14_stops, car):
            for key, value in sample.items():
                row[key] += value
        info['v14_stop'] = sample
        if self.stop_fix is not None:
            extra = dict(
                merge_braking=stop_fix_module.merge_braking(snapshot['target'], speed, audit['requested'],
                    info['feasible_acceleration_ceiling'], front, snapshot.get('gap_plan'), merge_active,
                    self.stop_fix),
                target_band_tail=stop_fix_module.target_band_tail(snapshot['target'], state['speed'] if state else None,
                    merge_active, self.v12_settings, self.stop_fix))
            if extra['merge_braking'] != 0. and terms.get('avoidable_braking', 0.) != 0.:
                raise ValueError('Braking would be charged twice')
            metrics = self.v14_stop_fix_metrics
            fixed = metrics['per_car'].setdefault(agent, dict(merge_braking_charged=0, merge_braking=0.,
                                                              band_tail_charged=0, target_band_tail=0.))
            for row in (metrics, fixed):
                row['merge_braking_charged'] += int(extra['merge_braking'] != 0.)
                row['merge_braking'] += extra['merge_braking']
                row['band_tail_charged'] += int(extra['target_band_tail'] != 0.)
                row['target_band_tail'] += extra['target_band_tail']
            totals = self.v06_metrics['reward_terms']
            for key, value in extra.items():
                totals[key] = totals.get(key, 0.) + value
            terms.update(extra)
        return terms

    def report(self):
        result = super().report()
        version = v14.REWARD_VERSION if self.stop_fix is None else stop_fix_module.REWARD_VERSION
        result.update(schema='highway_flow_rl_merge_env_v14', reward_version=version,
            objective_version=version, v14_settings=dict(self.v14_settings),
            v14_stops=deepcopy(self.v14_stops), v14_stop_settings=dict(stop_fix_module.STOP_SETTINGS),
            v14_control=dict(v14.CONTROL, **deepcopy(getattr(self, 'v14_control', {}))),
            v14_metrics=deepcopy(self.v14_metrics),
            v14_scope=('v12 environment and terms; SUMO checks off for controlled cars (the policy and '
                       "the environment's own limits drive them); yield-entry targets check crossing gaps "
                       'and change at most +1.5/-2.0 m/s^2'))
        if self.stop_fix is not None:
            result['v14_stop_fix'] = dict(settings=dict(self.stop_fix, obligations=list(self.stop_fix['obligations'])),
                                          **deepcopy(self.v14_stop_fix_metrics))
        if self.road_grade is not None:
            stats = {k: (None if isinstance(v, float) and not np.isfinite(v) else v) for k, v in self.drive_stats.items()}
            result['v14_drive_limit'] = dict(settings=deepcopy(self.drive_limit), road_grade=deepcopy(self.road_grade.info),
                                             **stats)
        return result
