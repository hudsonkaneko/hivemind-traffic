"""V11 merge environment: the v10 environment plus the merge-speed precondition mask.

Only the lane-change mask changes (traffic/highway_flow_rl_v11_rewards.py), and it
only removes permissions. Dynamics, safety shield, native guards, observation
layout, action contract, gates and every v04, v06, v09 and v10 reward term are
unchanged. The actor sees the rule through the existing mask inputs (30-32).
"""
from copy import deepcopy

from environments.highway_flow_rl_v10_env import HighwayFlowRLV10Env
from traffic import highway_flow_rl_v11_rewards as v11
from traffic.highway_flow_context import flow_context


class HighwayFlowRLV11Env(HighwayFlowRLV10Env):
    def __init__(self, *args, v11_settings=None, **kwargs):
        self.v11_settings = v11.validate_settings(v11_settings or {})
        super().__init__(*args, **kwargs)

    def _reset_v06(self):
        super()._reset_v06()
        self._merge_gate, self._merge_gate_counted = {}, set()
        self.v11_metrics = dict(blocked_decisions=0, per_car={})

    def _merge_speed_gate(self, agent):
        """Evidence for the merge lane change of a car that must leave its lane, else None."""
        state = self.states.get(agent)
        if state is None or not self.v11_settings['merge_dwell_decisions']:
            return None
        flow = flow_context(self.graph, state, self.states)
        route, entry = flow['corridors']['own']['route'], flow['entry']
        if route.get('reachable', True) or not route.get('known') or not entry.get('known') or not entry.get('lane_delta'):
            return None
        gate = v11.merge_speed_precondition(v11.matched_run(self.quality_history.get(agent)),
                                            route['distance_m'], state['speed'], self.v11_settings)
        gate.update(lane_delta=int(entry['lane_delta']), time_s=round(self.time, 1))
        return gate

    def _mask(self, agent, context):
        mask = super()._mask(agent, context)
        gate = self._merge_speed_gate(agent)
        self._merge_gate[agent] = gate
        if gate is None:
            return mask
        index = gate['lane_delta'] + 1
        gate['earlier_mask_allowed'] = bool(mask[index])
        if not gate['permitted'] and mask[index]:
            mask[index] = 0.
            key = (agent, gate['time_s'])
            if key not in self._merge_gate_counted:  # the mask is built more than once per decision
                self._merge_gate_counted.add(key)
                car = self.v11_metrics['per_car'].setdefault(agent, dict(blocked_decisions=0,
                    first_blocked_time_s=None, last_blocked_time_s=None))
                car['blocked_decisions'] += 1
                self.v11_metrics['blocked_decisions'] += 1
                if car['first_blocked_time_s'] is None:
                    car['first_blocked_time_s'] = gate['time_s']
                car['last_blocked_time_s'] = gate['time_s']
        return mask

    def _lane_rejection_causes(self, agent, context, delta):
        causes = super()._lane_rejection_causes(agent, context, delta)
        gate = self._merge_speed_gate(agent)
        if delta and gate is not None and delta == gate['lane_delta'] and not gate['permitted']:
            if super()._mask(agent, context)[delta + 1]:  # only this rule blocks it
                causes = [cause for cause in causes if cause != 'lane_gap']
            causes.append('merge_speed_precondition')
        return causes

    def _begin_maneuver(self, agent, delta):
        gate = self._merge_gate.get(agent)
        super()._begin_maneuver(agent, delta)
        if gate is not None and delta == gate['lane_delta']:
            car = self.v11_metrics['per_car'].setdefault(agent, dict(blocked_decisions=0,
                first_blocked_time_s=None, last_blocked_time_s=None))
            car.update(merge_started_time_s=round(self.time, 1), matched_run_at_start=gate['matched_run'],
                       started_by='matched_speed' if gate['matched'] else 'lane_end_release',
                       speed_at_start_m_s=self.states[agent]['speed'], remaining_at_start_m=gate['remaining_m'])

    def _extra_rewards(self, agent, info, snapshot, audit, state, merge_active):
        terms = super()._extra_rewards(agent, info, snapshot, audit, state, merge_active)
        info['v11_audit'] = dict(merge_gate=deepcopy(self._merge_gate.get(agent)))
        return terms

    def report(self):
        result = super().report()
        result.update(schema='highway_flow_rl_merge_env_v11', reward_version=v11.REWARD_VERSION,
            objective_version=v11.REWARD_VERSION, v11_settings=dict(self.v11_settings),
            v11_mask_version=v11.MASK_VERSION, v11_metrics=deepcopy(self.v11_metrics),
            v11_scope='v10 environment and terms; adds only the merge-speed precondition to the lane-change mask')
        return result
