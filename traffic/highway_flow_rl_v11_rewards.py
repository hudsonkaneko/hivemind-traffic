"""V11 merge objective: v10 plus a merge-speed precondition in the lane-change mask.

Pure functions and reviewed settings only; the environment wiring is
environments/highway_flow_rl_v11_env.py. Evidence:
documentation/highway-merge-training-plan-v11.md.

Round 9's saved validation journals show the policy requests the merge lane change
on the first decision the mask allows: 0.1 s after entering the auxiliary lane on
the ramp-start cases, at 13.0 m/s into 22.35 or 24.59 m/s lanes. The rolling
merge-quality gate scores the 8 s before the lateral merge completes, so that
timing caps the merge-speed match near 0.55 whatever the acceleration is. V09 and
v10 changed rewards on speed requests, which do not decide the timing.

V11 adds one rule. While a car must leave its lane (its own lane does not reach
its route and the merge target lane is known), the lane change toward the target
lane stays masked until the car has matched the gate's own speed target (within
2 m/s, the gate's tolerance) for `merge_dwell_decisions` consecutive decisions.
The rule releases near the end of the lane, so it can never strand a car: once
the remaining distance is at most max(merge_release_min_m, speed *
merge_release_time_s) the earlier mask applies unchanged. The rule only removes
permissions; every earlier mask rule, the safety shield and the native guards
still apply. Later rounds add reward terms here, each as a reviewed settings row.
"""
from traffic import highway_flow_rl_v10_rewards as v10

REWARD_VERSION = 'highway_flow_rl_merge_v11'
MASK_VERSION = 'native_v05_plus_projected_maneuver_gaps_v04_plus_merge_speed_dwell_v11'
V10_SETTINGS = dict(v10.SETTINGS)
RULE_KEYS = ('merge_dwell_decisions', 'merge_release_time_s', 'merge_release_min_m')

# One reviewed settings row per training-study round (the contract pins the row to
# the round). The environment refuses any settings that are not one of these rows.
SETTINGS_BY_ROUND = {
    # Round 13: rule test with the frozen round-9 policy (no training).
    # 40 decisions = 4.0 s. The gate window is the 8 s before and after the lateral
    # merge completes (161 samples, at most 16 may miss the target). A lane change
    # lasts 3 s, so 50 window samples precede the request; 4.0 s of matched speed
    # leaves at most 10 of them unmatched.
    13: dict(merge_dwell_decisions=40, merge_release_time_s=10., merge_release_min_m=150.),
    # Round 14: second rule test, frozen round-9 policy. Round 13 passed two of the
    # three ramp-start cases (match 0.938 and 0.944) but on I-110, whose ending lane
    # is about 360 m long, the rule held the car through its only gap (15.4-17.5 s);
    # the release came at 19 s, the gap was gone, and the car stopped at the lane
    # end. A 15 s release hands I-110 back to the earlier mask before that gap opens
    # (220 m at 14.7 m/s) and still leaves the 800 m ramp-case lanes their dwell.
    14: dict(merge_dwell_decisions=40, merge_release_time_s=15., merge_release_min_m=150.),
    # Round 15: first training round, same rule as round 14 (5 of 8 gates with the
    # frozen round-9 policy; I-110 merge, I-110 following and one ramp case left).
    15: dict(merge_dwell_decisions=40, merge_release_time_s=15., merge_release_min_m=150.),
    # Round 16: shield projection from round 9 (no PPO), same rule.
    16: dict(merge_dwell_decisions=40, merge_release_time_s=15., merge_release_min_m=150.),
    # Round 17: guided projection from the round-16 actor, same rule.
    17: dict(merge_dwell_decisions=40, merge_release_time_s=15., merge_release_min_m=150.),
}


def validate_settings(settings):
    """Return the settings if they are a reviewed row; raise otherwise."""
    settings = dict(settings)
    if settings not in SETTINGS_BY_ROUND.values():
        raise ValueError('Unreviewed v11 settings')
    return settings


def matched_run(samples):
    """Trailing count of consecutive quality samples whose speed matched the gate target."""
    run = 0
    for sample in reversed(samples or []):
        if not sample.get('speed_match'):
            break
        run += 1
    return run


def merge_speed_precondition(run, remaining_m, speed_m_s, settings):
    """Whether a required merge lane change may start now.

    `run` is the trailing count of matched decisions, `remaining_m` the distance
    left on the car's own (ending) lane. Released near the lane end so the rule
    can only delay a merge, never prevent it.
    """
    if type(run) is not int or run < 0:
        raise ValueError('Matched run must be a non-negative integer')
    if not all(isinstance(v, (int, float)) and v == v and v >= 0 for v in (remaining_m, speed_m_s)):
        raise ValueError('Remaining distance and speed must be finite and non-negative')
    release_m = max(settings['merge_release_min_m'], speed_m_s * settings['merge_release_time_s'])
    released = remaining_m <= release_m
    matched = run >= settings['merge_dwell_decisions']
    return dict(permitted=bool(matched or released), matched=bool(matched), released=bool(released),
                matched_run=run, required_run=settings['merge_dwell_decisions'],
                remaining_m=float(remaining_m), release_distance_m=float(release_m))
