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
| 15 | PPO training: round-14 rule plus shield-consistency loss | **Regressed:** 5 → 3 → 2 of 8 gates; nothing kept |
| 16 | Shield projection from round 9 (supervised, no PPO) | **7 of 8 gates**; only I-110 merge fails |
| — | Scripted tracking probes on the I-110 merge case (no training) | The last gate is reachable; see below |
| 17 | Guided projection from the round-16 actor | 5, 6, 6 of 8 gates; not kept |
| — | Probe 07: scripted merge approach with the newest actor, all 8 cases | 7 of 8; all four merges qualify |

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

#### Round 15 result: regressed, nothing kept

The run completed its 200,000-decision budget (197,345 decisions, 1,696 s, no
collisions). Evidence:
[round-15-results.json](evidence/highway-merge-training-v11/round-15-results.json).
Run outputs stay in Claude's worktree, `outputs/highway-flow-rl/v11/round-15`.

| Validation | Gates | Pooled merge match | Stable merges | Interventions | Peak jerk |
| --- | --- | --- | --- | --- | --- |
| Initial (round 9 under the rule) | 5 of 8 | 0.618 | 3 of 4 | 0.28% | 4.0 |
| After 100k decisions | 3 of 8 | 0.796 | 4 of 4 | 0.30% | 5.2 |
| Final (197k) | 2 of 8 | 0.409 | 3 of 4 | 2.30% | 39.5 |

- **At 100k some things had improved.** Speed-ceiling clamps were gone from
  every case (only comfort-rate changes remained), all four merges were stable,
  and I-110 matched 0.851 after merging into a later gap. But three ramp cars
  had dropped to 0.67–0.84, and raw request jerk had risen to 9–12 m/s³ on two
  of them.
- **By the end the policy was broken.** The I-110 car and the ramp 120 m car
  both ran to the lane end and stopped (6.6 s and 6.5 s stopped, jerk 39.5 and
  37.3 m/s³). Raw request jerk reached 22 m/s³. Six of eight cases failed.
- **The updates were erratic.** 13 of 16 PPO updates stopped early on the KL
  limit, most after 8 to 93 optimizer steps, with a last KL up to 0.35. In
  training, 24 of 66 PPO episodes had a jerk above 10 m/s³, most from cars that
  reached a lane end under exploration noise.
- **Conclusion.** At this scale PPO changes far more than the term it is given.
  The best validation of the run is the initial one, so the round-9 policy
  under the rule stays the reference. The round-15 checkpoints are kept but
  not used.

### Round 16: shield projection from round 9 (no PPO)

Runner: [`experiments/run_highway_flow_rl_v11_projection.py`](../experiments/run_highway_flow_rl_v11_projection.py).
It makes the smallest change to the round-9 actor that removes requests the
filters would alter.

1. Collect the 20 training rows with the current actor in the v11 environment.
2. Fit the actor on every observation collected so far with
   `50 × shield consistency + (request − round-9 request)² + (lane outputs − round-9 lane outputs)²`.
3. Run the 8 validation cases and save the actor. Then repeat once, so the
   second fit also sees the states the changed actor visits.

- No reward, advantage, critic or teacher label is used. The critic and the
  action noise are not touched; a check fails the run if anything outside the
  actor changes.
- Inside the filters' band the first term is zero, so the fit only holds the
  actor to round 9 there. With weights 50 and 1, a request that starts 1.35
  m/s² outside the band (a car just inserted) ends about 0.03 m/s² outside,
  inside the filters' 0.05 margin.
- Budget: at most 203,200 decisions (two passes over the 20 rows).
- It targets failures 1 and 2 above. Failure 3 (I-110 merge) is left for the
  next round.

#### Round 16 result: 7 of 8 gates

111,609 decisions, 40 collection episodes, 876 s, no collisions in validation.
Evidence: [round-16-results.json](evidence/highway-merge-training-v11/round-16-results.json).
Run outputs: `outputs/highway-flow-rl/v11/round-16` (Claude's worktree).

| Validation | Gates | Pooled merge match | Stable merges | Interventions | Peak jerk |
| --- | --- | --- | --- | --- | --- |
| Round 9 without the rule | 3 of 8 | 0.454 | 4 of 4 | 0.26% | 4.0 |
| Round 9 under the rule (round 14) | 5 of 8 | 0.618 | 3 of 4 | 0.28% | 4.0 |
| Round 16, after pass 1 | 4 of 8 | 0.571 | 4 of 4 | 0.23% | 9.4 |
| **Round 16, after pass 2** | **7 of 8** | **0.876** | 4 of 4 | **0.06%** | 4.8 |

After pass 2 (`candidates/checkpoint-02.pt`, `651d20fe…53bed7`):

- **All three ramp cases pass**, each with a speed match of 0.938 and no
  override in the 8 s after merging. The speed-ceiling clamps are gone.
- **I-110 following passes.** Raw request jerk is 3.96 m/s³ (was 6.1).
- **Cruise and passing cases still pass.** Raw request jerk fell to 1.2–1.7.
- **I-110 merge still fails `rolling_merge_quality`:** match 0.689, and 9 window
  samples have a neighbor braking at −4.5 m/s².

**Pass 1 was worse than the start.** Its fit was made only on states the
round-9 actor visits. The fitted actor then drove the ramp cars slowly enough
that two of them merged at the lane-end release near 56–59 s (match 0.39 and
0.44). Pass 2 added the states that actor visits and corrected it. One pass is
not enough for this method, and the result rests on one seed per case.

The fit itself: the share of requests outside the band fell from 1.79% to
0.63% of collected decisions, and the mean squared distance from the band fell
from 0.0061 to 0.00004 (m/s²)². The mean squared change from the round-9
request is 0.0086 (m/s²)².

### Scripted probes on the I-110 merge case (no training)

Script: [`scripts/probe_highway_flow_rl_v11_tracking.py`](../scripts/probe_highway_flow_rl_v11_tracking.py).
For a car with a pending merge, the acceleration of the frozen round-16 actor is
replaced by `0.6 × (target − speed)` on the gate's own target, kept inside the
filters' band. Lane decisions and every other car stay with the actor. Nine
diagnostic episodes. Evidence:
[tracking-probes.json](evidence/highway-merge-training-v11/tracking-probes.json).

| Probe | Speed aim while waiting | Lane change requested | Speed match | Result for the merging car |
| --- | --- | --- | --- | --- |
| 01 | The target | 20.7 s | 0.714 | Fails: the gap opens late, and near the lane end the target itself drops and then jumps |
| 02 | Target − 1.3 m/s | 14.5 s | 0.969 | Fails: 9 neighbor-braking samples in the window; 59 post-merge overrides from the law itself |
| 03 | Target − 0.5 m/s | 19.1 s | 0.900 | **Qualifies** (just) |
| 04 | Target − 0.9 m/s | 16.5 s | 0.994 | Speed and neighbors pass; 61 post-merge overrides from the law itself |
| 05 | Target − 0.9 m/s, actor drives again after the lateral merge | 16.5 s | 0.994 | **Qualifies**, stable merge |
| 06 | Target − 0.7 m/s, same hand-back | 18.0 s | 0.994 | **Qualifies**, stable merge |

Probe 05 also ran the three ramp cases: all three pass (match 1.000, 0.938,
0.938).

What the probes show:

- **The I-110 merge gate is reachable** by speed tracking alone, with the actor's
  own lane decisions. The round-16 actor holds about 14.7 m/s while the target
  is 15.8–16.9 m/s; that is the whole speed-match failure.
- **Waiting slightly under the target matters.** A faster car is ahead in the
  target lane. At the target speed the gap behind it opens too late (probe
  01); a little under it, the gap opens in time.
- **The neighbor-braking samples are not caused by the controlled car.** At
  10.5–11.4 s an uncontrolled ramp car merges in front of a mainline car, which
  brakes at −4.5 m/s². The gate counts it whenever the controlled car's merge
  completes before about 19.4 s. Probes 03–06 request the lane change at
  16.5 s or later and so miss it. This makes the case sensitive to timing on
  this one seed.
- **In probes 05 and 06 the case still fails, on a different car.** A cruising
  car (`agent_6`) reaches the 18.0 m/s limit while still accelerating at
  0.6 m/s² and is clamped to 0.04 in one decision: jerk 5.63 m/s³ (gate 5). In
  round 16's own validation the same event measured 4.8.

### Round 17: guided projection from the round-16 actor

The same projection runner, starting from and held to the round-16 actor
(`checkpoint-02.pt`), with three additions:

1. **Guidance labels for pending merges.** For a car whose merge is pending and
   not yet laterally complete, the fit uses `10 × (request − label)²` instead of
   the term that holds it to round 16. The label is the probes' law: gain 0.6
   on the gate's target, 0.7 m/s under it while the car waits, at the target
   once the lane change has started, always inside the filters' band. After the
   lateral merge the actor is held to round 16 again.
   **This is a scripted teacher on those decisions.** Rounds 5–12 used no
   teacher labels. It is the reason this round can change merge-approach
   speed without PPO, and it should be judged with that in mind.
2. **A band that anticipates the speed limit.** The upper bound also includes
   `sqrt(2 × 0.35 × margin / 0.1) − 0.175` m/s², where margin is the speed
   limit minus the speed (both actor inputs). A car inside it can always ramp
   its acceleration to zero at 0.35 m/s² per decision before the limit.
3. **More I-110 merge rows.** Each pass adds 5 extra cycles of `i110-merge` and
   `i110-merge-focused-8` (10 episodes) to the 20 rows, with the seeds of later
   curriculum cycles. Only 2 of the 20 rows contain a gap-limited merge, which
   is too few states to fit the label from. A test checks that no collection
   seed is a validation or held-out seed.

Three passes of 30 episodes. In the first pass the pending merges are driven by
the label, so that the states of a correct merge are in the data; in the second
and third the actor drives and the label only supervises. Budget: at most
364,800 decisions.

#### Round 17 result: worse than round 16, not kept

194,435 decisions, 90 collection episodes, 1,517 s, no collisions. Evidence:
[round-17-results.json](evidence/highway-merge-training-v11/round-17-results.json).

| Validation | Gates | Pooled merge match | Stable merges | Interventions | Peak jerk |
| --- | --- | --- | --- | --- | --- |
| Start (round 16, pass 2) | 7 of 8 | 0.876 | 4 of 4 | 0.06% | 4.8 |
| After pass 1 (labels drive the merges in collection) | 5 of 8 | 0.720 | 4 of 4 | 0.03% | 29.2 |
| After pass 2 | 6 of 8 | 0.706 | 3 of 4 | 0.13% | 4.0 |
| After pass 3 | 6 of 8 | 0.825 | 4 of 4 | 0.03% | 4.0 |

- **The actor did not learn the waiting behavior.** On the I-110 validation case
  the car was at 13.7–15.9 m/s when it requested its lane change, at 13.4–15.5
  s, with a match of 0.565, none (stable merge failed) and 0.671. The label
  would have had it near 16.2 m/s at about 18 s.
- **The fit to the labels stayed loose.** The root-mean-square distance from
  the label on guided decisions ended at 0.12–0.16 m/s² (from 0.22–0.31). The
  gap-limited target is computed from the merge neighbors; the actor has those
  inputs (faded over 1 s) but not the target itself, and 20,271 guided
  decisions, most from ramp rows, were not enough to recover it.
- **It disturbed a case that passed.** Ramp 120 m 2 lanes went from 0.938 to a
  lane-end-release merge (0.435 and 0.752 in passes 1 and 3).
- **Two things did carry over.** Peak jerk is 4.0 and pooled interventions are
  0.03% after pass 3, so the speed-limit-aware band works. The two 180 m ramp
  cases and all four non-merge cases pass after pass 3.

The round-16 pass-2 actor remains the best learned policy.

### Probe 07: scripted merge approach with the round-17 actor (no training)

All 8 validation cases, round-17 pass-3 actor, with the probe law driving cars
whose merge is pending (0.7 m/s under the target while waiting, hand-back after
the lateral merge). Eight diagnostic episodes. Evidence: the last entry of
[tracking-probes.json](evidence/highway-merge-training-v11/tracking-probes.json).

- **7 of 8 gates.** All four merging cars qualify: I-110 0.994, ramp 120 m
  1.000, both 180 m ramps 0.938. Pooled merge-speed match 0.967, 4 of 4 stable
  merges, no collisions, pooled interventions 0.02%, peak jerk 4.0 m/s³.
- **The one failure is a different car.** In the I-110 merge case a cruising
  car (`agent_2`) starts its own lane change at 10.7 s and its request steps by
  −0.61 m/s² in one decision: raw request jerk 6.12 (gate 5). The law does not
  drive that car.

## Where v11 stands

| Policy | Gates (8 development validation cases) |
| --- | --- |
| Round 9, as trained | 3 of 8 |
| Round 9 under the merge rule | 5 of 8 |
| **Round 16 pass 2 under the merge rule (best learned policy)** | **7 of 8** |
| Round 17 pass 3 plus the scripted merge-approach law | 7 of 8 (a different case fails) |

What is established:

- **The merge rule is worth two gates with no training,** and fixed the cause
  of the low ramp scores: merging 16–21 mph slower than traffic.
- **The projection is worth two more** (clamps after merging, and the request
  step when a leader leaves) and is stable where PPO was not.
- **The last gate, I-110 merge, is reachable** by speed control in the merge
  approach; the scripted law shows it. The learned actor has not reproduced it.

What is not established:

- **Every result is one seed per case.** Two unrelated cars failed a jerk limit
  by small margins in different runs of the I-110 case (5.63 and 6.12 against
  5). Gate counts near the limits can move by one between runs that differ
  only slightly.
- **No held-out evaluation was run.** The rule and both fits were chosen with
  the validation cases in view.
- **Nothing is promoted.**

Open choices for the last gate, none taken:

1. Accept a scripted merge-approach speed law in the loop (as the rule and the
   filters already are) and tune the hand-back.
2. Give the actor the gate's target as an input, which changes the 117-input
   layout shared with Codex's work.
3. Keep the learned policy at 7 of 8 and spend the remaining budget on a
   multi-seed check of what already passes.

### Ledger after round 17 and probe 07

- Training decisions: 5,627,084 of 6,230,000.
- Diagnostic episodes: 162 of 168.
- Charged wall clock: 48,250.8 s of 61,200 s.
- Google engineering guide sync is still pending.
