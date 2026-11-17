"""V10 merge objective: the v09 environment plus a gate-aligned post-merge override cost.

Only one reward term is added (traffic/highway_flow_rl_v10_rewards.py). Dynamics,
safety shield, native guards, observation, action contract, gates and every v04,
v06 and v09 term are unchanged, so validation metrics stay comparable with round 9
and round 11.
"""
from environments.highway_flow_rl_v09_env import HighwayFlowRLV09Env
from traffic import highway_flow_rl_v10_rewards as v10


class HighwayFlowRLV10Env(HighwayFlowRLV09Env):
    def __init__(self, *args, v10_settings=None, **kwargs):
        settings = dict(v10.SETTINGS if v10_settings is None else v10_settings)
        if settings != v10.SETTINGS:
            raise ValueError('Unreviewed v10 reward settings')
        self.v10_settings = settings
        super().__init__(*args, v09_settings=dict(v10.V09_SETTINGS), **kwargs)

    def _reset_v06(self):
        super()._reset_v06()
        self._window_interventions = {}

    def _new_window_interventions(self, agent):
        """Override decisions the stable-merge tracker added to its post-merge window this decision."""
        tracker = getattr(self, 'merge_trackers', {}).get(agent)
        count = tracker.post_window['intervention_decisions'] if tracker is not None else 0
        new = count - self._window_interventions.get(agent, 0)
        self._window_interventions[agent] = count
        return new

    def _extra_rewards(self, agent, info, snapshot, audit, state, merge_active):
        terms = super()._extra_rewards(agent, info, snapshot, audit, state, merge_active)
        terms['post_merge_override'] = v10.post_merge_override(self._new_window_interventions(agent), self.v10_settings)
        totals = self.v06_metrics['reward_terms']
        totals['post_merge_override'] = totals.get('post_merge_override', 0.) + terms['post_merge_override']
        return terms

    def report(self):
        result = super().report()
        result.update(schema='highway_flow_rl_merge_env_v10', reward_version=v10.REWARD_VERSION,
            objective_version=v10.REWARD_VERSION, v10_settings=dict(self.v10_settings),
            v10_post_merge_interventions=dict(self._window_interventions),
            v10_scope='v09 environment and terms; adds only the post-merge override cost')
        return result
