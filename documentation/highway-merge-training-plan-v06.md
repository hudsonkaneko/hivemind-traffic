# Highway merge training plan v06

Written on November 13, 2026. **Status: proposed. Nothing in this plan is
implemented, run or authorized for training.** Training starts only after the
user approves the resolved configuration in Phase 4 (AGENTS.md).

v06 is the next step after the v05 jerk diagnosis
([highway-jerk-diagnosis-v05.md](highway-jerk-diagnosis-v05.md)). v05 improved
how commands are executed and scored. It did not change the learned policy:
every v05 result uses the frozen v04 actor.

## Problems this plan addresses

Measured with the frozen v04 actor on the six development cases, **original
gate rules** (no road-end exemption): 2/6 cases pass and 4/27 cars fail a
per-car check.

| Problem | Evidence | Proposed fix |
| --- | --- | --- |
| Merge speed matching about 42-48% vs 90% required | Both stable merges fail `rolling_merge_quality` | Reward tracking of the same target the gate scores |
| No recovery after a missed gap | Corrected 2-lane ramp120: the actor rides the acceleration lane to its end | Training episodes that start with a missed gap |
| 7-8 m/s3 spikes when merge inputs switch | I-110 merge focal car, 8.39 m/s3 | Real jerk penalty, faded merge inputs, continuous merge-guidance potential |
| 60 mph cruise car brakes without need | `agent_5`, `avoidable_braking` | Small cost tied to the existing avoidable-braking diagnosis |

## What already exists in the code

The v05 `full` variant (`environments/highway_flow_rl_v05_env.py`,
`traffic/highway_flow_rl_v05_rewards.py`) already contains part of the proposed
fix. It has never been trained to completion: the November 10 draft's
`comparison-01/full-final` run is incomplete.

- `merge_tracking` (weight 0.12): per-step cost of up to 0.12 for being more
  than 2 m/s from the reward target during a pending merge.
- `jerk_excess` (weight 0.10): quadratic cost on the larger of the requested and
  realized jerk above 5 m/s3.
- `stable_target`: a merge reward target with slot hysteresis and bounded
  acceleration (+1.5 / -2.0 m/s2).
- A 4 m/s3 command rate limit for the whole episode. Maximum braking and lower
  safety ceilings bypass it.

Three findings from reading this code change the proposal:

1. **The jerk penalty is far too weak.** At the 8.39 m/s3 peak,
   `jerk_excess` costs 0.10 x ((8.39-5)/5)^2 = 0.046. In the same step,
   `gap_preparation` (weight 2.0) can change by more than 1.
2. **The reward and the gate use different merge targets.** In `full`, the
   reward tracks the smoothed `stable_target`, but the merge-quality gate
   (`speed_match` in `environments/highway_flow_rl_v04_env.py`) measures speed
   within 2 m/s of the unsmoothed v04 reference target. That target re-picks its
   gap slot every step without hysteresis, so it can jump. A smooth driver may
   be unable to reach 90% against it. This has not been measured.
3. **Correction to the v05 diagnosis.** That note said the merge-guidance reward
   "teaches the jumps". `gap_preparation` is potential-based shaping, which in
   theory does not change the best policy. Its 0-to-1 potential jump when a
   merge becomes required mainly adds noise to the critic's value estimates. The
   main causes of the spikes are the one-step input flips and the near-zero
   jerk cost.

## Phase 0 - fix measurement and environment first (no training)

**0a. Merge-target feasibility audit.** For the recorded merges, compute how
fast the gate's reference target changes, and the best speed-match fraction a
car could reach while respecting +1.5 / -2.0 m/s2 and 4 m/s3. Use saved
per-step traces if they contain the reference target; otherwise up to 2
diagnostic episodes.
- Reachable ceiling at or above 90%: keep the gate. The policy is the problem.
- Ceiling below 90%: the gate partly measures target jitter. **User decision**
  before training: keep the gate, or score against the smoothed target. Any
  change is reported both ways, like the road-end rule.

**0b. Environment fixes** (from the v05 diagnosis):
- **Controlled car-following model (user decision).** Recommended: return to
  ACC and add ACC's follow-speed bound to the actor's safety guard and mask, so
  SUMO no longer needs to trim allowed commands. Krauss produced a 0.34 s
  headway cut-in on the corrected 2-lane ramp.
- **Road end:** extend the final segment past the measured section so leaders
  do not vanish while followers are still scored. The original gate rules are
  then used unchanged.
- **Ramp networks:** use the ramp-started layouts for training and
  development. Keep the historical ramp120 layout only as a reference.

**0c. Re-baseline.** Evaluate the frozen v04 actor after 0b, under the original
rules: the six development cases (historical ramp120 swapped for its
ramp-started version) and the six ramp-started layouts. This is the baseline
that v06 training must beat.

Budget: 0a-0c need about 14 diagnostic episodes. 78/80 are used. **User
decision:** raise `max_diagnostic_episodes` to 100.

## Phase 1 - implement the fixes (code and tests only)

Each change gets unit tests and a v06 reward/observation version string. Safety
shield, native guards, action contract (acceleration + lane intent) and
observation size (117) stay unchanged, so a v04 warm start remains possible.

1. **Merge speed reward.** Track the target the gate scores (per 0a). Raise
   `merge_tracking` from 0.12. The weight is set in Phase 2 so that, over a
   typical merge, a 90% match earns clearly more than v04's 42-48%.
2. **Missed-gap recovery.**
   - New demand mode in `traffic/highway_v05_demand.py`: a mainline car is
     placed alongside the ramp car where the acceleration lane begins, so the
     first gap is closed and a usable gap exists behind. About 25% of merge
     training episodes.
   - The existing `gap_plan` already scores the `behind_rear` slot, so the
     reward target for falling back exists.
   - New metric (and, if Phase 2 supports it, a small cost): reaching the last
     20% of the acceleration lane without merging.
3. **Faded merge inputs.**
   - The binary merge inputs (`requires_merge`, `merge_context_known`,
     `merge_target_lane_delta`, `target_front_present`, `target_rear_present`)
     and the target-neighbor numeric fields fade in and out over 1 s (10
     decisions) instead of switching in one step.
   - Only the **actor's inputs** fade. The safety guard, mask and lane-change
     permission keep using the raw current values, so no safety reaction is
     delayed.
4. **Jerk.**
   - Raise `jerk_excess` so that an 8 m/s3 request costs about as much as one
     step of merge guidance changes. The starting point is a weight of about
     1-2, confirmed in Phase 2.
   - **User decision:** use the `full` command rate limit for the whole episode
     (recommended), or only `startup`'s handoff limit.
   - Either way, evaluation adds a stricter per-car check: the actor's **raw
     requested** jerk must also stay under 5 m/s3. This stops the rate limit
     from hiding a jerky policy.
5. **Continuous merge-guidance potential.** When a merge becomes required, blend
   the `gap_plan` potential from 1 to the plan's potential over the same 1 s
   fade, instead of jumping to 0 or 1/(1+error).
6. **Avoidable braking.**
   - A small per-step cost using the same definition as the existing
     `avoidable_braking` diagnosis.
   - Charged only when no safety bound applies: no binding ceiling, no leader
     inside the 1.5 s headway, and no lower speed target.

## Phase 2 - offline reward audit (no simulation)

- Score saved v04 trajectories with the new reward functions, extending
  `scripts/audit_highway_flow_rl_v03_rewards.py`.
- Choose the weights for items 1, 4 and 6 so that:
  - a smooth merge with 90% matching earns clearly more than v04's recorded
    merges;
  - an 8 m/s3 request outweighs the noise from step-to-step shaping;
  - no term can be farmed by lingering, stopping, or refusing to merge.
- Record the weights and the audit evidence before asking for training approval.

## Phase 3 - training proposal for approval

The resolved configuration is presented to the user before any launch. The
expected shape:

| Item | Proposal |
| --- | --- |
| Algorithm | MAPPO, v05 pilot optimizer unchanged (actor lr 1e-4, critic lr 3e-4, clip 0.2, 20 epochs) |
| Warm start | v04 `long-round-03/best.pt` (sha `79e36b54...`) |
| Observations | 117 fields, merge inputs faded |
| Actions | Unchanged: acceleration + lane intent |
| Rewards | v04 terms + v06 changes with Phase 2 weights |
| Training rows | Ramp-started training layouts, I-110 merge/following, cruise/passing, missed-gap mode |
| Budget | One round: 300k decisions / 5400 s (the existing v05 pilot round size), second round not authorized |
| Stop rule | Any collision, or merge quality below the Phase 0c baseline at the 100k checkpoint: stop and diagnose |

## Phase 4 - evaluation

- **Development set:** the six development cases (ramp-started ramp120) and the
  ramp-started validation layouts.
- **Rules:** original gate rules plus the raw-request jerk check. Any
  exemption is reported separately and never counted as a pass.
- **Comparison:** the Phase 0c frozen-v04 baseline, using the same seeds.
- **Held-out bank:** run once, frozen, only after development gates pass.
  Results are never used for tuning.

## Decisions needed from the user

1. Controlled car-following model: ACC with an ACC-aware guard (recommended),
   or Krauss.
2. Raise the diagnostic episode cap from 80 to 100 for Phase 0.
3. After 0a: what merge quality is scored against, if the current target is
   unreachable.
4. Command rate limit for the whole episode (recommended), or startup only.
5. Training configuration and budget, at Phase 3.
