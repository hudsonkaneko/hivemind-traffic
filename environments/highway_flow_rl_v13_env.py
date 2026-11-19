"""V13 merge environment: v12 with the merge target following SUMO's yield decision.

Only the target changes (traffic/highway_flow_rl_v13_rewards.py): at a yield-junction
entry, while SUMO reports the link closed, the target is capped by a stop at the line.
Both target hooks apply the same cap, at the same simulation time and with one TraCI
query per car and step:
- _reward_target: the pre-action target used by every reward term and by the gate's
  speed match (_reference_targets);
- current_target: the target behind the four actor inputs.
Observation widths, dynamics, the safety shield, native guards, masks, gate thresholds
and reward weights are v12's. getNextLinks is a read-only query; a replay with it
reproduced round 19's validation result exactly.
"""
from copy import deepcopy

import traci

from environments.highway_flow_rl_v12_env import HighwayFlowRLV12Env
from traffic import highway_flow_rl_v13_rewards as v13

HARD_BRAKING_M_S2 = -3.  # the v04 gate's neighbor-braking threshold, reused


class HighwayFlowRLV13Env(HighwayFlowRLV12Env):
    def __init__(self, *args, v13_settings=None, **kwargs):
        self.v13_settings = v13.validate_settings(v13_settings or v13.SETTINGS)
        super().__init__(*args, **kwargs)

    def _reset_v06(self):
        super()._reset_v06()
        self._links, self._links_time = {}, None
        self.v13_metrics = dict(yield_decisions=0, link_missing=0, link_closed=0, link_open=0,
                                bounded_decisions=0, open_while_braking=0, per_car={})

    def _yield_link(self, agent, entry):
        """SUMO's link state for this car's yield entry, queried once per car and step."""
        if self._links_time != self.time:
            self._links, self._links_time = {}, self.time
        if agent not in self._links:
            try:
                links = self.conn.vehicle.getNextLinks(agent)
            except traci.TraCIException:
                links = ()
            self._links[agent] = v13.yield_link(links, entry.get('target_lane'))
        return self._links[agent]

    def _yield_target(self, agent, target, context):
        entry = (context or {}).get('entry') or {}
        if not entry.get('entry_active') or not target or not target.get('eligible'):
            return target, None, False
        link = self._yield_link(agent, entry)
        bounded, applied = v13.apply_yield(target, entry, link, self.v13_settings)
        return bounded, link, applied

    def _reward_target(self, agent, state, context, preview):
        target, potential = super()._reward_target(agent, state, context, preview)
        bounded, link, applied = self._yield_target(agent, target, context)
        if link is not None:
            self._reference_targets[agent] = bounded  # the gate's speed match uses it too
            lowered = applied and bounded['target_m_s'] < target['target_m_s'] - 1e-9
            braking = bool(link['open'] and state.get('acceleration', 0.) <= HARD_BRAKING_M_S2)
            car = self.v13_metrics['per_car'].setdefault(agent, dict(yield_decisions=0, link_missing=0,
                link_closed=0, link_open=0, bounded_decisions=0, open_while_braking=0))
            for row in (self.v13_metrics, car):
                row['yield_decisions'] += 1
                row['link_missing'] += int(not link['found'])
                row['link_closed'] += int(link['found'] and not link['open'])
                row['link_open'] += int(bool(link['found'] and link['open']))
                row['bounded_decisions'] += int(lowered)
                row['open_while_braking'] += int(braking)
        return bounded, potential

    def current_target(self, agent):
        target, pending, state = super().current_target(agent)
        if target is None:
            return target, pending, state
        bounded, _, _ = self._yield_target(agent, target, getattr(self, 'flow_contexts', {}).get(agent))
        return bounded, pending, state

    def report(self):
        result = super().report()
        result.update(schema='highway_flow_rl_merge_env_v13', reward_version=v13.REWARD_VERSION,
            objective_version=v13.REWARD_VERSION, v13_settings=dict(self.v13_settings),
            v13_metrics=deepcopy(self.v13_metrics),
            v13_scope='v12 environment and terms; the merge target follows SUMO yield-link state at yield entries')
        return result
