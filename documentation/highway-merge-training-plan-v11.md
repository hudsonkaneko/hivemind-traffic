# Highway merge training v11: merge-speed rule and iterated rounds

Started by Claude on November 17, 2026, after
[v10 round 12](highway-merge-training-plan-v10.md#round-12-result-november-17-2026-stopped-by-the-merge-floor-at-100k).

**User decision (November 17, 2026):** "Test merge rule. also i want you to use
some autonomy. continue monitoring and modifting training and changing rewards
or weights to achieve acceptable conditions. keep iterating until you get to
that point."

So v11 runs as a series of rounds. Claude declares each round in
[`traffic/highway_flow_rl_v11_contract.py`](../traffic/highway_flow_rl_v11_contract.py)
in a reviewed commit before it runs, and records each result here. "Acceptable
conditions" is read as the existing development gates: all 8 validation cases
pass. Held-out evaluation was not part of the decision and is not authorized.

## Why rounds 11 and 12 did not help

Read from the round-9 policy's saved validation journals
(`outputs/highway-flow-rl/v10/round-12/initial-validation`). No simulation.

- **The policy merges on the first decision the mask allows.** On the three
  ramp-start cases it requests the lane change at 3.2 s, 0.1 s after entering
  the auxiliary lane, at 13.0 m/s (29 mph), into 22.35 or 24.59 m/s lanes
  (50 or 55 mph). On I-110 it requests it at 15.4 s, also the first allowed
  decision. It never declines a permitted merge.
- **The auxiliary lane is about 800 m long on the ramp cases and is hardly used.**
- **That timing caps the score.** The rolling merge-quality gate scores the 8 s
  before and after the lateral merge completes and needs 90% of those samples
  within 2 m/s of the target. A kinematic estimate with the vehicle's limits
  (2.0 m/s² cap, +0.4 m/s² per decision) gives at most about 0.58, 0.55 and
  0.55 with round 9's merge timing, even at full throttle. Round 9 scores
  0.461, 0.397 and 0.369. With the merge completed near 14–16 s instead of
  6.2 s, 0.90 or more is reachable.
- **Rounds 11 and 12 changed rewards on speed requests.** Those do not decide
  when the lane change starts.

The estimate is a calculation from saved records, not a simulator run. A check
of the same model against the recorded requests reads 0.02 high on the ramp
cases. Rounds 13 and 14 below are the simulator test.

## The rule (v11 environment)

Code: [`traffic/highway_flow_rl_v11_rewards.py`](../traffic/highway_flow_rl_v11_rewards.py)
and [`environments/highway_flow_rl_v11_env.py`](../environments/highway_flow_rl_v11_env.py),
a subclass of the v10 environment.

A car that must leave its lane (its own lane does not reach its route, and the
merge target lane is known) may start that lane change only after it has
matched the gate's own speed target, within the gate's 2 m/s tolerance, for
`merge_dwell_decisions` consecutive decisions.

- **Release near the lane end.** Once the remaining lane is at most
  `max(merge_release_min_m, speed × merge_release_time_s)`, the earlier mask
  applies unchanged. The rule can delay a merge but never prevent one.
- **It only removes permissions.** Every earlier mask rule, the safety shield
  and the native guards still apply. The observation layout, actions, rewards
  and gates are unchanged. The actor sees the rule through the existing mask
  inputs.
- **Why 40 decisions.** The gate window has 161 samples and allows 16 misses. A
  lane change lasts 3 s, so 50 window samples come before the request. 4.0 s of
  matched speed leaves at most 10 of them unmatched.

`mask_version` in the configuration stays the v04 value, because the v04 mask
audit is unchanged. The addition is reported as `v11_mask_version`.

## Bounds for the iterated rounds

Set by Claude in `STUDY_BUDGET` for this decision, on the shared ledger:

| Limit | Before | v11 |
| --- | --- | --- |
| Training decisions | 5,230,000 (5,123,695 used) | 6,230,000 |
| Charged wall clock | 46,800 s (43,781 s used) | 61,200 s |
| Diagnostic episodes | 128 (all used) | 168 |

Raising any of these again needs a new note to the user.

## Rounds

| Round | What | Result |
| --- | --- | --- |
| 13 | Rule test, 10 s release, frozen round-9 policy | 5 of 8 gates; I-110 regressed (stopped at the lane end) |
| 14 | Rule test, 15 s release, frozen round-9 policy | 5 of 8 gates; I-110 as round 9 |
| 15 | Training: round-14 rule plus shield-consistency loss | See below |

### Round 13: rule test with a 10 s release (no training)

The frozen round-9 policy on the 8 validation cases, 90.6 s. Evidence:
[round-13-rule-test.json](evidence/highway-merge-training-v11/round-13-rule-test.json).
For comparison, the same policy without the rule:
[round-09-policy-without-rule.json](evidence/highway-merge-training-v11/round-09-policy-without-rule.json).

| Merging car | Without the rule | With the rule | Note |
| --- | --- | --- | --- |
| Ramp 120 m, 2 lanes | 0.461, gate fails | **0.938, gate passes** | Lane change requested at 12.4 s at 22.1 m/s (was 3.2 s at 13.0 m/s) |
| Ramp 180 m, 4 lanes | 0.369, gate fails | **0.944, gate passes** | Requested at 14.6 s at 24.2 m/s |
| Ramp 180 m, 2 lanes | 0.397, gate fails | Speed match 0.938, but the stable merge fails | 6 speed-ceiling clamps in the 8 s after merging; the gate allows 4 of 81 |
| I-110 | 0.590, gate fails | **Worse:** 0.441, stopped 6.6 s | See below |

Gates: 5 of 8 (3 of 8 without the rule). No collisions.

**I-110 failure.** The I-110 ending lane is about 360 m long. The car was at
14.5 m/s with a target of 16.9–18.0 m/s, so it was not matched. Its only gap
opened from 15.4 s to 17.5 s and the rule blocked it for 22 decisions. The
release came at 19 s (150 m left), the gap was gone, and the car braked to a
stop at the lane end (peak jerk 43.9 m/s³) and merged from standstill at
36.8 s. This run is kept as a recorded failure of the 10 s release.

### Round 14: rule test with a 15 s release (no training)

Same test with `merge_release_time_s = 15`. At 14.7 m/s that hands I-110 back to
the earlier mask with 220 m left, before its gap opens. The ramp cases keep
their dwell because their lanes are 800 m long. 89.5 s. Evidence:
[round-14-rule-test.json](evidence/highway-merge-training-v11/round-14-rule-test.json).

- Gates: **5 of 8**. Pooled merge-speed match 0.618 (0.454 without the rule).
- Stable merges 3 of 4, no collisions, pooled interventions 0.28%, peak jerk
  4.0 m/s³.
- The three ramp cars are exactly as in round 13. I-110 is back to round 9's
  behavior (0.590, requested at 15.4 s).

Three failures remain:

1. **Ramp 180 m, 2 lanes: stable merge.** Right after merging the car sits at
   24.587 m/s under a 24.59 m/s limit and keeps requesting +0.2 to +0.001 m/s².
   Each is clamped by the speed ceiling and counted as an override.
2. **I-110 following: raw request jerk 6.1 m/s³ (gate 5).** One controlled car's
   leader leaves the road end at 33.1 s. Its request steps from −0.06 to
   +0.56 m/s² in one decision.
3. **I-110 merge: rolling merge quality.** The car holds about 14.7 m/s while
   the gate's target rises from 12.4 to 16.9 m/s, and is 0.2 m/s outside the
   band for 4.5 s before its merge. Five window samples also have a neighbor
   braking at −4.5 m/s², 5 s before the merge.

### Round 15: training under the rule with a shield-consistency loss

Failures 1 and 2 are requests that the command filters change. The environment
caps every request at the feasible ceiling (speed limit, following, lane end)
and rate-limits commands to 0.4 m/s² per decision around the measured
acceleration. The gates count each such change as an override, and the
raw-request check allows 0.5 m/s² per decision.

**Shield-consistency loss** (`ShieldConsistentTrainer` in
[`experiments/run_highway_flow_rl_v11.py`](../experiments/run_highway_flow_rl_v11.py)):
the squared distance of the actor's mean request from the band

`[measured − 0.35, min(measured + 0.35, ceiling − 0.05)]` (m/s²),

added to the PPO loss with weight 50.

- **Not a teacher.** Both band inputs are fields the actor already receives:
  its measured acceleration (input 1) and the feasible ceiling (input 43). A
  check against the round-14 journals found both equal to the environment's own
  values to 1e-7 on all 17,995 decisions.
- **Zero inside the band.** There the loss and its gradient are zero, so PPO
  alone decides the behavior.
- **Braking is never held back.** If the ceiling is below the band, the band
  collapses onto the ceiling.
- **How often it acts.** The round-9 policy is outside the band on 64 of 17,995
  validation decisions (0.36%), 62 of them above the upper bound.

Everything else is as in round 12 and round 14: the round-9 warm start, v10
rewards, the round-14 rule, rows, optimizer, acceptance. Up to 200,000 decisions.

Stop rules: any collision; pooled interventions above 5%; pooled merge-speed
match below 0.30. The floor is lower than before on purpose. One ramp car
flipping its stable merge moves the pooled score from 0.62 to 0.38, and that
kind of single-episode flip stopped rounds 11 and 12.

Failure 3 (I-110 merge) has no dedicated change in this round.
