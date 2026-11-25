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

Optional drive target (round 22; user decision, November 22, 2026: "Yes, raise it"; off unless the
constructor gets drive_target=traffic.highway_flow_rl_v14_drive_target.SETTINGS): at yield entries
the reward target starts at the car's speed and rises no faster than min(1.5 m/s^2, the drive
ceiling), on its own ramp. It feeds the actor target inputs and the band, tracking, speed-error,
merge_braking and band-tail terms. The gate's speed match, the merge-quality payments graded on
the gate's window (v04's quality_merge, v10's graded quality) and the free-road stop measurement
keep v14's target. The report (v14_drive_target) also grades each merging car's rolling merge
quality on the drive-aware target, as a diagnostic only.

Optional follow gate (round 23; user decision, November 24, 2026: "yes redesign"; off unless the
constructor gets follow_gate=traffic.highway_flow_rl_v14_follow_gate.SETTINGS; not with the drive
target): from a car's first yield-entry decision, the gate grades a reference that starts at the
car's speed, moves toward the plan/road target at bounded acceleration and jerk, restarts from the
car's speed after a yield, and lasts through the merge window. The speed match accepts speeds within
2 m/s of the span between the reference and the road target (_acceptance_target). The reward target
and the actor target inputs are the reference capped at once by the road target's leader and
lane-end bounds. The merge-quality payments follow the gate, as they did before round 22. The report
(v14_follow_gate) also grades each merging car's rolling merge quality on v14's target, as a
diagnostic, so gate counts stay comparable with rounds 20-22.

Optional cut-in charge (round 24; user decision, November 25, 2026: "All three"; off unless the
constructor gets cut_in=traffic.highway_flow_rl_v14_cut_in.SETTINGS): after each step, every background
vehicle braking below -3 m/s^2 whose SUMO leader within 40 m is one of our cars charges that car 0.25 per
m/s^2 of the excess (reward term cut_in; report field v14_cut_in).

Optional drive saturation (round 24, the same decision; off unless the constructor gets
drive_saturation=traffic.highway_flow_rl_v14_saturation.SETTINGS, with the drive limit on): an override
whose only cause is drive_grade is applied as before but not counted as an intervention (reward term,
override counts, intervention fractions, merge samples); it is counted in v14_drive_limit.saturated.
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
from traffic import highway_flow_rl_v14_drive_target as drive_target_module
from traffic import highway_flow_rl_v14_follow_gate as follow_module
from traffic import highway_flow_rl_v14_cut_in as cut_in_module
from traffic import highway_flow_rl_v14_saturation as saturation_module
from traffic.highway_flow_context import flow_context
from traffic.highway_flow_rl_v03_rewards import map_evidence
from traffic.highway_flow_rl_v04_rewards import gap_plan as v04_gap_plan, quality_merge_eligible

NATIVE_DRAIN_SPEED_MODE = 31  # SUMO default, the mode every earlier version drained with


class HighwayFlowRLV14Env(HighwayFlowRLV12Env):
    def __init__(self, *args, v14_settings=None, drive_limit=None, road_model_manifest=None, stop_fix=None,
                 drive_target=None, follow_gate=None, cut_in=None, drive_saturation=None, **kwargs):
        self.v14_settings = v14.validate_settings(v14_settings or v14.SETTINGS)
        self.cut_in = None if cut_in is None else cut_in_module.validate_settings(cut_in)
        self.drive_saturation = (None if drive_saturation is None else
                                 saturation_module.validate_settings(drive_saturation))
        if self.drive_saturation is not None and drive_limit is None:
            raise ValueError('Drive saturation needs the drive limit on')
        self.stop_fix = None if stop_fix is None else stop_fix_module.validate_settings(stop_fix)
        self.drive_target = None if drive_target is None else drive_target_module.validate_settings(drive_target)
        self.follow_gate = None if follow_gate is None else follow_module.validate_settings(follow_gate)
        if self.drive_target is not None and self.follow_gate is not None:
            raise ValueError('The drive target and the follow gate are separate rounds')
        self.drive_limit = None if drive_limit is None else validate_drive_limit(drive_limit)
        if self.drive_limit is None and road_model_manifest is not None:
            raise ValueError('A road model for the grade needs the drive limit on')
        self.road_grade = (None if self.drive_limit is None else
                           RoadGrade(args[0], road_model_manifest, spec=self.drive_limit['vehicle']))
        self._drive_patched = False
        self.drive_stats = dict(decisions=0, limited=0, saturated=0, max_grade=-np.inf, min_ceiling_m_s2=np.inf)
        self._cut_in_step = {}
        self.v14_cut_in_metrics = dict(decisions=0, charged=0, excess_m_s2=0., cut_in=0., per_car={}, events=[])
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
                    saturated = info.get('drive_saturation') is not None
                    self.drive_stats['decisions'] += 1
                    self.drive_stats['limited'] += int(saturated or 'drive_grade' in (info.get('override_causes') or ()))
                    self.drive_stats['saturated'] += int(saturated)
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
        self.drive_stats = dict(decisions=0, limited=0, saturated=0, max_grade=-np.inf, min_ceiling_m_s2=np.inf)
        self._cut_in_step = {}
        self.v14_cut_in_metrics = dict(decisions=0, charged=0, excess_m_s2=0., cut_in=0., per_car={}, events=[])
        self._v14_cache, self._ramp = {}, {}
        self.v14_metrics = dict(yield_decisions=0, crossing_changed_target=0, crossing_limited_plans=0,
                                ramp_decisions=0, ramp_limited=0, hook_mismatch=0, per_car={})
        self.v14_stops = dict(decisions=0, stopped=0, asking_to_brake=0, asking_to_brake_overrides=0,
                              free_road=0, per_car={})
        self.v14_stop_fix_metrics = dict(merge_braking_charged=0, merge_braking=0., band_tail_charged=0,
                                         target_band_tail=0., per_car={})
        self._drive_ramp, self._drive_quality_history, self._drive_quality_results = {}, {}, {}
        self.v14_drive_target_metrics = dict(decisions=0, below_gate=0, above_gate=0, gate_minus_target_m_s=0.,
                                             per_car={})
        self._follow, self._v14_old_gate = {}, {}
        self._old_gate_quality_history, self._old_gate_quality_results = {}, {}
        self.v14_follow_gate_metrics = dict(decisions=0, started=0, restarts=0, ended=0, reward_capped=0,
                                            road_target_below=0, per_car={})

    def _v14_target(self, agent, state, context, preview):
        """One target per car and simulation time, shared by both hooks.

        Returns the reward target: v14's, the drive-aware one (drive target on) or the capped follow
        reference (follow gate on). The gate's target (v14's, or the follow reference) is cached
        alongside it (_v14_gate_target), and so is v14's own (gate_v14).
        """
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
        gate = gate_v14 = target
        if self.drive_target is not None:
            target = self._drive_aware(agent, state, desired, plan, ceiling, active, eligible, gate)
        elif self.follow_gate is not None:
            gate, target = self._follow_targets(agent, state, desired, ceiling, active, eligible, gate_v14)
        self._v14_cache[agent] = dict(time=self.time, desired=desired, target=target, gate=gate,
                                      gate_v14=gate_v14, plan=plan)
        return target, plan

    def _follow_targets(self, agent, state, desired, ceiling, active, eligible, fallback):
        """The follow gate's (gate target, reward target) of a car (traffic.highway_flow_rl_v14_follow_gate).

        Cars that have not reached a yield entry keep v14's target for both; so does a decision whose
        road target is not eligible (the reference is kept and moves on at the next eligible one).
        """
        settings = self.follow_gate
        reference = self._follow.get(agent)
        if (reference is None and not (eligible and active)) or not eligible:
            return fallback, fallback
        drive = None
        if self.road_grade is not None:
            limit = self.drive_limit
            drive = self.road_grade.ceiling(state['lane_id'], state['lane_position'], state['speed'],
                                            headroom_m_s2=limit['headroom_m_s2'],
                                            lookahead_s=limit['lookahead_s'])['ceiling_m_s2']
        rise = follow_module.rise_rate(drive, settings)
        acceleration = state.get('acceleration', 0.)
        road = desired['target_m_s']
        started, restarted = reference is None, False
        if started:
            reference = follow_module.start(state['speed'], acceleration, ceiling, rise, self.time, settings)
        else:
            reference, restarted = follow_module.advance(reference, road, ceiling, rise, state['speed'],
                                                         acceleration, self.time, settings)
        lower, upper = follow_module.band(reference['value'], road)
        cap = follow_module.reward_cap(desired, settings)
        reward_value = reference['value'] if cap is None else min(reference['value'], cap)
        graded = agent in self.quality_results
        ended = graded and abs(reference['value'] - road) <= settings['end_tolerance_m_s']
        if ended:
            self._follow.pop(agent, None)
        else:
            self._follow[agent] = reference
        audit = dict(road_target_m_s=road, reference_m_s=reference['value'], lower_m_s=lower, upper_m_s=upper,
                     slope_m_s2=reference['slope'], rise_m_s2=rise, drive_ceiling_m_s2=drive, ceiling_m_s=ceiling,
                     reward_cap_m_s=cap, yield_entry=active, started=started, restarted=restarted, ended=ended,
                     v14_gate_m_s=fallback['target_m_s'] if fallback.get('eligible') else None)
        gate, target = deepcopy(desired), deepcopy(desired)
        gate['target_m_s'], target['target_m_s'] = reference['value'], reward_value
        gate['follow_gate'] = target['follow_gate'] = audit
        target['v14'] = deepcopy(fallback.get('v14')) or dict(desired_m_s=road, ceiling_m_s=ceiling, yield_entry=active)
        metrics = self.v14_follow_gate_metrics
        car = metrics['per_car'].setdefault(agent, dict(decisions=0, started=0, restarts=0, ended=0, reward_capped=0,
                                                        road_target_below=0))
        for row in (metrics, car):
            row['decisions'] += 1
            row['started'] += int(started)
            row['restarts'] += int(restarted)
            row['ended'] += int(ended)
            row['reward_capped'] += int(reward_value < reference['value'] - 1e-9)
            row['road_target_below'] += int(road < reference['value'] - 1e-9)
        return gate, target

    def _drive_aware(self, agent, state, desired, plan, ceiling, active, eligible, gate):
        """The drive-aware reward target (traffic.highway_flow_rl_v14_drive_target), on its own ramp."""
        ramp = self._drive_ramp.get(agent)
        if not eligible:
            self._drive_ramp.pop(agent, None)
            return gate
        if not (active or ramp is not None):
            return gate
        drive = None
        if self.road_grade is not None:
            limit = self.drive_limit
            drive = self.road_grade.ceiling(state['lane_id'], state['lane_position'], state['speed'],
                                            headroom_m_s2=limit['headroom_m_s2'],
                                            lookahead_s=limit['lookahead_s'])['ceiling_m_s2']
        rise = drive_target_module.rise_rate(drive, self.drive_target)
        value = drive_target_module.reachable(desired['target_m_s'], ceiling, state['speed'], ramp, self.time,
                                              rise, self.drive_target)
        if not active and abs(value - desired['target_m_s']) < 1e-9:
            self._drive_ramp.pop(agent, None)
        else:
            self._drive_ramp[agent] = dict(time=self.time, value=value)
        target = deepcopy(gate)
        target['target_m_s'] = value
        target['v14'] = dict(gate.get('v14') or dict(desired_m_s=desired['target_m_s'], ceiling_m_s=ceiling,
                                                       yield_entry=active,
                                                       crossing_deficit=plan.get('crossing_deficit'),
                                                       crossing_time_s=plan.get('crossing_time_s')),
                             drive_target=dict(gate_target_m_s=gate['target_m_s'], rise_m_s2=rise,
                                               drive_ceiling_m_s2=drive, ramp_started=ramp is None))
        metrics = self.v14_drive_target_metrics
        car = metrics['per_car'].setdefault(agent, dict(decisions=0, below_gate=0, above_gate=0,
                                                        gate_minus_target_m_s=0.))
        difference = gate['target_m_s'] - value
        for row in (metrics, car):
            row['decisions'] += 1
            row['below_gate'] += int(difference > 1e-9)
            row['above_gate'] += int(difference < -1e-9)
            row['gate_minus_target_m_s'] += difference
        return target

    def _v14_gate_target(self, agent, fallback):
        cached = self._v14_cache.get(agent)
        if cached is not None and abs(cached['time'] - self.time) < 1e-9:
            return cached['gate']
        return fallback

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
        # The gate's speed match grades v14's target (the same as the reward target unless the drive
        # target is on), or with the follow gate on, the follow reference's band.
        self._reference_targets[agent] = self._v14_gate_target(agent, target)
        if self.follow_gate is not None:
            cached = self._v14_cache.get(agent)
            self._v14_old_gate[agent] = (cached['gate_v14'] if cached is not None and
                                         abs(cached['time'] - self.time) < 1e-9 else target)
        run = self._required_run.get(agent)
        if run is None:
            run = int(bool(plan.get('required')))
            self._required_run[agent] = run
        return target, v06.blend_potential(plan, run)

    def _acceptance_target(self, agent, snapshot):
        """The gate's speed-match target. With the follow gate on and a follow band, the point of the band
        nearest the car's speed after the decision, so v04's |speed - target| <= 2 m/s is the band check."""
        reference = super()._acceptance_target(agent, snapshot)
        audit = (reference or {}).get('follow_gate') if self.follow_gate is not None else None
        state = self.states.get(agent)
        if audit is None or state is None or not reference.get('eligible'):
            return reference
        return dict(reference, target_m_s=follow_module.acceptance(state['speed'], audit['lower_m_s'],
                                                                   audit['upper_m_s']))

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
        gate = self._reference_targets.get(agent, snapshot['target'])   # v14's target, as in round 21
        sample = stop_fix_module.stop_sample(speed, audit['requested'], front, gate,
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
        if self.cut_in is not None:
            excess, events = self._cut_in_step.get(agent, (0., []))
            value = -self.cut_in['weight'] * excess
            metrics = self.v14_cut_in_metrics
            car = metrics['per_car'].setdefault(agent, dict(decisions=0, charged=0, excess_m_s2=0., cut_in=0.))
            for row in (metrics, car):
                row['decisions'] += 1
                row['charged'] += int(excess > 0.)
                row['excess_m_s2'] += excess
                row['cut_in'] += value
            for event in events:
                metrics['events'].append(dict(event, time_s=round(self.time, 2), car=agent))
            totals = self.v06_metrics['reward_terms']
            totals['cut_in'] = totals.get('cut_in', 0.) + value
            info['v14_cut_in'] = dict(excess_m_s2=excess, followers=[e['follower'] for e in events]) if events else None
            terms['cut_in'] = value
        return terms

    def _drive_saturation_exempt(self, agent, info, rewards):
        """An override caused only by the drive bound: applied as before, not counted as an intervention."""
        term = info['reward_terms'].get('intervention', 0.)
        rewards[agent] -= term
        info['reward_terms']['intervention'] = 0.
        stats = self.stats
        stats['reward_terms']['intervention'] = stats['reward_terms'].get('intervention', 0.) - term
        stats['overrides'] -= 1
        for name in info['override_causes']:
            stats['override_causes'][name] -= 1
            if stats['override_causes'][name] <= 0:
                del stats['override_causes'][name]
        info['drive_saturation'] = dict(causes=list(info['override_causes']),
                                        requested_m_s2=info['requested_acceleration'],
                                        applied_m_s2=info['applied_acceleration'])
        info['override'], info['override_causes'] = False, []

    def _update_merge_metrics(self, agents, infos, rewards, terminated, truncated):
        """Diagnostic only: each merging car's rolling merge quality graded on the drive-aware target,
        with v04's own samples, window and rule (rewards and the gate are unchanged).

        Round 24 first, before any inner metric reads the step's infos: drive saturations stop counting as
        interventions, and the cut-in charges of the step just simulated are found (paid in _extra_rewards)."""
        if self.drive_saturation is not None:
            for agent in agents:
                if saturation_module.saturated(infos[agent], self.drive_saturation):
                    self._drive_saturation_exempt(agent, infos[agent], rewards)
        if self.cut_in is not None:
            controlled = [agent for agent in agents if agent in self.states]
            self._cut_in_step = cut_in_module.charges(self.conn, self.states, controlled, self.cut_in)
        diagnostic = self.drive_target is not None or self.follow_gate is not None
        before = dict(self.quality_results) if diagnostic else None
        super()._update_merge_metrics(agents, infos, rewards, terminated, truncated)
        if self.drive_target is not None:
            self._diagnostic_quality(agents, before, lambda agent: self._v02_pre_action[agent]['target'],
                                     self._drive_quality_history, self._drive_quality_results)
        elif self.follow_gate is not None:
            self._diagnostic_quality(agents, before, lambda agent: self._v14_old_gate.get(agent),
                                     self._old_gate_quality_history, self._old_gate_quality_results)

    def _diagnostic_quality(self, agents, before, target_of, histories, results):
        """Diagnostic only: rolling merge quality of each merging car with the speed match graded on
        target_of(agent) (|speed - target| <= 2 m/s), with v04's own samples, window and rule."""
        for agent in agents:
            history = self.quality_history.get(agent)
            if not history:
                continue
            target = target_of(agent)
            state = self.states.get(agent)
            speed = state['speed'] if state else None
            sample = dict(history[-1], speed_match=bool(speed is not None and target and target.get('eligible')
                                                        and abs(speed - target['target_m_s']) <= 2.))
            own = histories.setdefault(agent, [])
            own.append(sample)
            tracker = self.merge_trackers.get(agent)
            completed_at = tracker.merge_complete_time_s if tracker else None
            own[:] = [r for r in own if r['time_s'] >= (completed_at if completed_at is not None else self.time) - 8.]
            result = self.quality_results.get(agent)
            if result is not None and result is not before.get(agent):
                results[agent] = dict(
                    qualified=bool(self.approach_stops.get(agent, 0.) <= 1. + 1e-8 and quality_merge_eligible(own)),
                    samples=len(own), speed_match_fraction=sum(r['speed_match'] for r in own) / len(own),
                    gate_qualified=result['qualified'], gate_speed_match_fraction=result['speed_match_fraction'])

    def report(self):
        result = super().report()
        version = (cut_in_module.REWARD_VERSION if self.cut_in is not None else
                   drive_target_module.REWARD_VERSION if self.drive_target is not None else
                   follow_module.REWARD_VERSION if self.follow_gate is not None else
                   v14.REWARD_VERSION if self.stop_fix is None else stop_fix_module.REWARD_VERSION)
        result.update(schema='highway_flow_rl_merge_env_v14', reward_version=version,
            objective_version=version, v14_settings=dict(self.v14_settings),
            v14_stops=deepcopy(self.v14_stops), v14_stop_settings=dict(stop_fix_module.STOP_SETTINGS),
            v14_control=dict(v14.CONTROL, **deepcopy(getattr(self, 'v14_control', {}))),
            v14_metrics=deepcopy(self.v14_metrics),
            v14_scope=('v12 environment and terms; SUMO checks off for controlled cars (the policy and '
                       "the environment's own limits drive them); yield-entry targets check crossing gaps "
                       'and change at most +1.5/-2.0 m/s^2'))
        if self.drive_target is not None:
            result['v14_drive_target'] = dict(settings=dict(self.drive_target),
                quality_merges=deepcopy(self._drive_quality_results), **deepcopy(self.v14_drive_target_metrics),
                scope=('reward target and actor target inputs at yield entries; quality_merges is a diagnostic '
                       'grading of the gate window on this target, not the gate'))
        if self.follow_gate is not None:
            result['v14_follow_gate'] = dict(settings=dict(self.follow_gate, reward_caps=list(self.follow_gate['reward_caps'])),
                v14_gate_quality_merges=deepcopy(self._old_gate_quality_results), **deepcopy(self.v14_follow_gate_metrics),
                scope=("the gate's speed match, the reward target and the actor target inputs from each car's first "
                       'yield-entry decision; v14_gate_quality_merges is a diagnostic grading of the gate window on '
                       "v14's target, not the gate"))
        if self.stop_fix is not None:
            result['v14_stop_fix'] = dict(settings=dict(self.stop_fix, obligations=list(self.stop_fix['obligations'])),
                                          **deepcopy(self.v14_stop_fix_metrics))
        if self.road_grade is not None:
            stats = {k: (None if isinstance(v, float) and not np.isfinite(v) else v) for k, v in self.drive_stats.items()}
            result['v14_drive_limit'] = dict(settings=deepcopy(self.drive_limit), road_grade=deepcopy(self.road_grade.info),
                                             **stats)
            if self.drive_saturation is not None:
                result['v14_drive_limit']['saturation'] = dict(
                    settings=dict(self.drive_saturation, causes=list(self.drive_saturation['causes'])),
                    scope=('overrides caused only by the drive bound: applied, counted in saturated, not counted as '
                           'interventions; limited counts every drive-bound clip, saturated or not'))
        if self.cut_in is not None:
            result['v14_cut_in'] = dict(settings=dict(self.cut_in), **deepcopy(self.v14_cut_in_metrics),
                scope=('per decision: background vehicles braking below braking_m_s2 whose SUMO leader within '
                       'max_gap_m is one of our cars; that car pays weight x the excess (reward term cut_in)'))
        return result
