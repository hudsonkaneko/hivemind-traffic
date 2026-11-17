"""V10 merge reward: v09 plus a post-merge override cost aligned with the stable-merge gate.

Pure functions only; the environment wiring is environments/highway_flow_rl_v10_env.py.
Evidence: documentation/highway-merge-training-plan-v10.md (round-11 diagnosis).

Round 11 (v09) was stopped by its merge floor because two of four validation
merging cars failed the stable-merge gate. That gate allows at most 5% override
decisions in the 8 s after the lateral merge (4 of 81). The training episodes show
the same thing: 6 of 17 merging cars failed it in episodes 16-43 (round 9: 2 of 17),
four of them with 47-54 of 81 post-merge decisions where the native vehicle model
did not realize a small positive acceleration request behind a slower leader. The
v06 intervention term charges 0.2 per override everywhere, so 5 extra overrides
cost about 1 while the gate result they break is worth the whole graded quality
payment (up to 5).

V10 keeps every v09 term and adds one: each decision the stable-merge tracker
counts as an intervention inside its post-merge window costs an extra 1.0 (total
1.2 with the unchanged intervention term). Five such decisions, the first count
that fails the gate, then cost 6.0, more than the largest graded payment. The count
is taken from the tracker itself, so the charged decisions are exactly the gate's
numerator (policy filter changes, including speed-ceiling clamps, and native
realization changes). Nothing else changes.
"""
from traffic import highway_flow_rl_v09_rewards as v09

REWARD_VERSION = 'highway_flow_rl_merge_v10'
V09_SETTINGS = dict(v09.SETTINGS)
SETTINGS = dict(V09_SETTINGS,
    post_merge_override_weight=1.,   # extra cost per gate-counted override in the 8 s post-merge window
)


def post_merge_override(new_window_interventions, settings=SETTINGS):
    """Extra cost for override decisions the stable-merge gate counts in its post-merge window."""
    if type(new_window_interventions) is not int or new_window_interventions < 0:
        raise ValueError('Post-merge intervention count must be a non-negative integer')
    return -settings['post_merge_override_weight'] * new_window_interventions
