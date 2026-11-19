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
"""
from copy import deepcopy

from environments.highway_flow_rl_v12_env import HighwayFlowRLV12Env
from traffic import highway_flow_rl_v06_rewards as v06
from traffic import highway_flow_rl_v14_rewards as v14
from traffic.highway_flow_context import flow_context
from traffic.highway_flow_rl_v03_rewards import map_evidence
from traffic.highway_flow_rl_v04_rewards import gap_plan as v04_gap_plan

NATIVE_DRAIN_SPEED_MODE = 31  # SUMO default, the mode every earlier version drained with


class HighwayFlowRLV14Env(HighwayFlowRLV12Env):
    def __init__(self, *args, v14_settings=None, **kwargs):
        self.v14_settings = v14.validate_settings(v14_settings or v14.SETTINGS)
        super().__init__(*args, **kwargs)

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
        self._v14_cache, self._ramp = {}, {}
        self.v14_metrics = dict(yield_decisions=0, crossing_changed_target=0, crossing_limited_plans=0,
                                ramp_decisions=0, ramp_limited=0, hook_mismatch=0, per_car={})

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
        return terms

    def report(self):
        result = super().report()
        result.update(schema='highway_flow_rl_merge_env_v14', reward_version=v14.REWARD_VERSION,
            objective_version=v14.REWARD_VERSION, v14_settings=dict(self.v14_settings),
            v14_control=dict(v14.CONTROL, **deepcopy(getattr(self, 'v14_control', {}))),
            v14_metrics=deepcopy(self.v14_metrics),
            v14_scope=('v12 environment and terms; SUMO checks off for controlled cars (the policy and '
                       "the environment's own limits drive them); yield-entry targets check crossing gaps "
                       'and change at most +1.5/-2.0 m/s^2'))
        return result
