"""V09 merge objective on the unchanged v06 environment (Claude proposal, November 17, 2026).

Only the added reward terms change (traffic/highway_flow_rl_v09_rewards.py):
unsaturated merge-speed tracking, a graded merge-quality payment, and no raw-request
jerk charge during the fixed insertion lag. Dynamics, safety shield, native guards,
observation fading, rate limit, action contract, gates and the v04/v06 weights are
exactly v06's, so validation metrics stay comparable with round 9.
"""
from copy import deepcopy

from environments.highway_flow_rl_v06_env import HighwayFlowRLV06Env
from traffic import highway_flow_rl_v06_rewards as v06
from traffic import highway_flow_rl_v09_rewards as v09


class HighwayFlowRLV09Env(HighwayFlowRLV06Env):
    def __init__(self, *args, v09_settings=None, **kwargs):
        settings = dict(v09.SETTINGS if v09_settings is None else v09_settings)
        if settings != v09.SETTINGS:
            raise ValueError('Unreviewed v09 reward settings')
        self.v09_settings = settings
        super().__init__(*args, **kwargs)

    def _reset_v06(self):
        super()._reset_v06()
        self._graded_paid = {}

    def _extra_rewards(self, agent, info, snapshot, audit, state, merge_active):
        w, s = self.v06_weights, self.v09_settings
        ego = snapshot['ego_state']
        front = (snapshot['context'].get('corridors', {}).get('own', {}) or {}).get('front')
        diagnosis = info.get('v04_reward_audit', {}).get('avoidable_braking') or {}
        car = self.v06_metrics['per_car'].setdefault(agent, dict(decisions=0, late_lane_decisions=0,
            first_late_lane_time_s=None, avoidable_braking_charged=0,
            **{v06.FIRST_DECISION_EXCLUDED_KEY: 0., v06.INSERTION_LAG_EXCLUDED_KEY: 0.}))
        graded, details = 0., None
        if agent in self.quality_results and agent not in self._graded_paid:  # set this decision at the stable merge
            graded, details = v09.graded_quality(self.quality_history.get(agent, []),
                                                 self.approach_stops.get(agent, 0.), s)
            self._graded_paid[agent] = dict(details, payment=graded, time_s=round(snapshot['time_s'], 1))
        terms = dict(
            jerk_excess=v09.jerk_excess(info['realized_jerk_m_s3'], audit['raw_requested_jerk'], w['jerk_excess'],
                                        car['decisions'], s),
            merge_tracking=v09.merge_tracking(snapshot['target'], state['speed'] if state else None,
                                              merge_active, w['merge_tracking'], s),
            avoidable_braking=v06.avoidable_braking(diagnosis, audit['requested'], ego['speed'], front,
                                                    w['avoidable_braking']),
            quality_merge_graded=graded)
        lane = self.graph.lanes.get(ego['lane_id'], {})
        late = v06.late_in_acceleration_lane(snapshot['context'], lane.get('length'), merge_active)
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
        if details is not None:
            info['v09_quality_payment'] = dict(details, payment=graded)
        return terms

    def report(self):
        result = super().report()
        result.update(schema='highway_flow_rl_merge_env_v09', reward_version=v09.REWARD_VERSION,
            objective_version=v09.REWARD_VERSION, v09_settings=dict(self.v09_settings),
            v09_graded_quality=dict(self._graded_paid),
            v09_scope='v06 environment and weights; only tracking, graded quality and lag-aware jerk excess differ')
        return result
