# Highway merge training plan v06

Written on November 13, 2026. Phase 0 started November 14 (see the progress
section at the end). **Status: proposed. Nothing in this plan is
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

## Phase 0 progress (November 14, 2026)

**User decisions (November 14):** use Krauss for controlled cars, and raise
the diagnostic episode cap. The cap was raised from 80 to 100 in the pinned
contract (`traffic/highway_flow_rl_v05_contract.py`) and the pilot config. No
other budget value changed. No episodes were used for the work below.

### 0b. Krauss cut-in: no Krauss setting prevents it

The cut-in happened at 56.9 s on the ramp-started 2-lane ramp120 layout. The
native car `main_3` (19.05 m/s) moved 3.0 m in front of the AV, which was doing
8.75 m/s while still stuck after missing its merge. SUMO's lane changer asks
the car behind (the AV) how much gap it needs. The answer comes from the AV's
own car-following model. Asked directly through
`traci.vehicle.getSecureGap` ([probe](../scripts/probe_highway_flow_rl_v06_secure_gap.py)):

| Situation (AV speed, new leader speed) | ACC tau 1.5 | Krauss tau 1.5 | Krauss tau 2.0 | Krauss tau 3.0 |
| --- | ---: | ---: | ---: | ---: |
| The cut-in (8.75, 19.05) | 10.0 m | 0 m | 0 m | 0 m |
| Same speed (20, 20) | 30.0 m | 30.0 m | 40.0 m | 60.0 m |
| AV slower (15, 20) | 21.0 m | 5.5 m | 13.0 m | 28.0 m |
| AV faster (20, 15) | 31.5 m | 47.0 m | 57.0 m | 77.0 m |

At equal speeds Krauss and ACC require the same gap. Only when the AV is much
slower than the car cutting in does Krauss require nothing: that car's own
braking distance covers the AV's. Raising Krauss's tau does not change that
case, and would make the AV more cautious everywhere else. As far as we can
tell, the native lane-change caution settings (such as `lcAssertive`) scale
this required gap, so they cannot raise a zero either; this was not tested. **Conclusion:** keep Krauss as decided and accept this cut-in as a
known side effect. It occurs only when an AV crawls more than 10 m/s below
traffic, which is the missed-gap failure Phase 1 item 2 targets. The headway
gate still counts it. Evidence:
[krauss-secure-gap.json](evidence/highway-merge-training-v06/krauss-secure-gap.json)
(SHA-256 `92a7b974e78231fbd5cf32fa2f49b0be90d5ba3b7edd9d7d1a02d8f88a5d821f`).

### 0a. Merge-target feasibility: 90% is unreachable on most layouts

The new offline audit
([script](../scripts/audit_highway_flow_rl_v06_merge_target.py),
[tests](../tests/test_highway_flow_rl_v06_merge_target_audit.py)) rebuilds
each merge's quality window from the saved traces. It reproduces the recorded
speed-match fractions exactly. It then finds the best fraction any car could
reach against the same recorded target sequence, starting from the car's real
speed. Two limit sets: the vehicle limits with the 5 m/s3 gate jerk
(-4.5 to +2.0 m/s2), and comfort limits (-2.0 to +1.5 m/s2, 4 m/s3).

| Layout (frozen v04, Krauss + startup run) | Observed | Best possible (vehicle limits) | Best possible (comfort) |
| --- | ---: | ---: | ---: |
| I-110 merge | 0.48 | **0.99** | 0.99 |
| 2-lane ramp120, historical network | 0.42 | 0.70 | 0.68 |
| 2-lane ramp120, ramp-started (unstable merge, not gated) | 0.65 | 0.69 | 0.67 |
| 3-lane ramp240, ramp-started | 0.28 | 0.49 | 0.26 |
| 4-lane ramp120, ramp-started | 0.49 | 0.65 | 0.48 |
| 5-lane ramp240, ramp-started | 0.27 | 0.49 | 0.26 |
| 2-lane ramp180 validation, ramp-started | 0.39 | 0.57 | 0.37 |
| 4-lane ramp180 validation, ramp-started | 0.38 | 0.57 | 0.37 |

- **I-110: a policy problem.** A car could match 99% of the window. The actor
  instead holds about 13.5 m/s for 6 s while the target is 15.8-18 m/s.
- **Ramp-started layouts: a layout problem.** The merge completes about 6 s
  after the car is inserted on the ramp at about 8 m/s, so the 8 s window
  reaches back to insertion. When the car enters the acceleration lane, the
  target rises to the 26.8 m/s mainline speed. Even at the 2 m/s2 limit the car
  needs about 6 s to get within 2 m/s. The gate's target itself does not jitter
  there (no step changes over 0.2 m/s, apart from one 22 m/s switch on ramp120).
  The 60 m parallel acceleration lane chosen for the v05 ramp-started builder is
  too short for a 60 mph mainline.
- Open-loop caveat: a different trajectory would see a somewhat different
  target, and could merge at a different time. The ceilings are estimates for
  the recorded sequences, not proofs.

**Consequence for the plan:** training cannot pass merge quality on the
current ramp-started layouts. Before re-baselining, they should be rebuilt with
an acceleration lane and ramp long enough to reach mainline speed (a network
change, not a gate change). The reward-target mismatch (finding 2 above)
matters less than expected: the gate's target is mostly steady.

**User decision (November 14, 2026): do not rebuild.** Use only the road
geometry already in GitHub; the policy must work on any scenario and on-ramp,
including short acceleration lanes. So the rebuild proposed above is withdrawn,
and:

- The gate stays as it is. On the ramp-started layouts the 90% merge
  speed-match rule is likely unreachable (ceiling 0.49-0.69), so failures there
  under the original rules are expected and will be reported as such.
- Every merge-quality result is reported next to that car's physical ceiling
  from this audit, so "policy fell short" and "the road made it impossible" stay
  separate. The ceiling is a diagnostic only, never a pass rule.
- Training targets generalization: the curriculum mixes all existing layouts
  (short and long acceleration lanes, 2- and 4-lane, I-110), and the Phase 1
  merge speed reward pushes toward the target as fast as the acceleration and
  jerk limits allow, rather than assuming a long lane.
- Success on these layouts is measured as closing the gap to the ceiling (for
  example I-110 0.48 toward 0.99), alongside the original pass/fail.

Evidence:
[merge-target-feasibility.json](evidence/highway-merge-training-v06/merge-target-feasibility.json)
(SHA-256 `44701e95f62604101d31e4639b9855dccffb3a5a58a3ee302109335be5e12313`).

### Road end: not changed yet

Extending the road so lead cars keep driving still leaves someone vanishing at
the new end. Unless scoring stops before that end, this is a measurement-window
choice, equivalent to not scoring the final stretch. It needs a user decision
and must be reported beside the original-rules result.

### Not run yet

The frozen-v04 re-baseline (0c) waits only for the road-end decision. It runs
on the existing layouts (no rebuild, per the user decision above).

### Road end: keep the original rules (user decision, November 14, 2026)

No road-end exemption. `road_end_leader_exit_rule` stays off for the baseline
and for v06 evaluation; end-of-road jerk counts as a failure.

### 0c. Frozen-v04 baseline under the original rules

No new episodes were needed (still 78/100). The frozen v04 actor (startup mode,
Krauss) had already run every baseline case; only the scoring differed.
[`scripts/rebaseline_highway_flow_rl_v06.py`](../scripts/rebaseline_highway_flow_rl_v06.py)
applies the unchanged `development_gates` to the saved per-episode reports of
`dev-native-fixes-01` and `ramp-native-fixes-01`. The historical ramp120 merge
case is swapped for the ramp-started run of the same row; the script asserts
that demand, seed and controlled car are identical and only the network
differs.

| Case | Original rules | Failing check | Merge match: observed / ceiling |
| --- | --- | --- | --- |
| I-110 merge | fail | focal peak jerk 8.4; merge quality | 0.48 / 0.99 |
| Cruise, 1 AV | pass | | |
| Passing, 1 AV | pass | | |
| I-110 following | fail | 1 car peak jerk 5.34 (road end) | |
| ramp120 merge, ramp-started | fail | jerk 28.9, interventions 10.5%, headway 0.34 s, no stable merge, merge quality | 0.65 / 0.69 |
| 60 mph cruise | fail | 1 car avoidable braking | |

**Development: 2/6 cases pass, 4/27 cars fail.** Zero collisions and teleports.

| Ramp-started layout | Original rules | Merge match: observed / ceiling (vehicle limits) |
| --- | --- | --- |
| train 2-lane ramp120 | fail (as above) | 0.65 / 0.69 |
| train 3-lane ramp240 | fail, merge quality only | 0.28 / 0.49 |
| train 4-lane ramp120 | fail, merge quality only | 0.49 / 0.65 |
| train 5-lane ramp240 | fail, merge quality only | 0.27 / 0.49 |
| validation 2-lane ramp180 | fail, merge quality only | 0.39 / 0.57 |
| validation 4-lane ramp180 | fail, merge quality only | 0.38 / 0.57 |

**Ramp-started: 0/6 pass, and all fail the 90% rule, which no car could pass
there.** On five layouts every per-car check passes. This is the baseline v06
must beat: more cases passing under the original rules, and on every merge a
smaller gap between observed and ceiling.

Evidence: [rebaseline-original-rules.json](evidence/highway-merge-training-v06/rebaseline-original-rules.json)
(SHA-256 `c789523991729742590f099e1dcf71e897ae156f44a08733050c1389489a5343`).
The source runs are under `outputs/highway-flow-rl/v05/` (not in Git); the
evidence records each report's SHA-256. Phase 0 is complete. Next: Phase 1
(code and tests only).

## Phase 1 progress (November 14, 2026): implemented, not trained

Code and tests only. No training, no policy episodes, no ledger charge.
Environment `environments/highway_flow_rl_v06_env.py` (subclass of the v05
environment); pure functions in `traffic/highway_flow_rl_v06_rewards.py`;
reward version `highway_flow_rl_merge_v06`, observation version
`highway_flow_actor117_v1_faded_merge_v06`. The safety shield, native guards,
action contract and 117-field observation size are unchanged, so a v04 warm
start still loads. The only v05 change is a behavior-preserving refactor: the
v05 extra reward terms moved into an overridable `_extra_rewards` method.

| Plan item | Implemented as |
| --- | --- |
| 1. Merge speed reward | `merge_tracking` now tracks the unsmoothed v04 reference target, the same one the gate's `speed_match` scores, with the gate's 2 m/s tolerance. Weight 0.50 (was 0.12). `speed_error` also uses that target. |
| 2. Missed-gap recovery | `write_demand(..., missed_gap=True)`: the rightmost through-lane platoon is shifted as one, keeping every draw, spacing and speed, so one mainline car is predicted level with the front ramp car where the acceleration lane begins. Default demand is byte-identical (tested). Metric: decisions in the last 20% of the acceleration lane without merging. No cost added yet. |
| 3. Faded merge inputs | Actor indices 27, 28 and 33-42 move at most 0.1 per decision, so a 0-to-1 flip takes 10 decisions (1 s). The mask (30-32), acceleration ceiling (43), safety guard and lane permission use raw values. The critic sees the same faded inputs as the actors, because the MAPPO runner builds the successor critic state from returned observations. |
| 4. Jerk | `jerk_excess` weight 1.50 (was 0.10): an 8.39 m/s3 request now costs 0.69, comparable to a step of merge guidance. Evaluation check `v06_gates` = the original gates plus a per-car raw requested jerk check (at most 5 m/s3; missing evidence fails). Both rate-limit options exist: `rate_limit='full'` (whole episode, recommended) or `'startup'` (handoff only). |
| 5. Continuous guidance potential | When a merge becomes required, the gap-preparation potential fades from 1 to the plan's value over 10 decisions. It is a function of state plus a consecutive-required counter, so shaping stays potential-based; a test checks the pre-action and successor potentials agree. |
| 6. Avoidable braking | Cost 0.05 x min(1, (-0.2 - request)/2) on the actor's own request, only when the existing diagnosis finds no binding ceiling and no lower target, and no leader is inside a 1.5 s headway. |

**All weights are provisional.** The environment refuses any others; Phase 2
sets them.

### Checks

- 16 new tests in `tests/test_highway_flow_rl_v06.py`; 374 passed and 1
  skipped across the v04/v05/v06/demand tests (`-k "v04 or v05 or v06 or demand"`).
- A bounded 400-step SUMO smoke run of the v06 environment (constant command,
  2-lane ramp120 ramp-started, normal and missed-gap) had no collisions. Input
  fading was active and the report fields were present. This only checks the
  wiring, not driving quality.
- **Missed-gap check in native SUMO**
  ([`scripts/check_highway_flow_rl_v06_missed_gap.py`](../scripts/check_highway_flow_rl_v06_missed_gap.py)).
  Every car is SUMO-driven; six ramp-started layouts x 8 non-evaluation seeds.
  A mainline car is level with the ramp car (within 7.3 m) when it reaches the
  acceleration lane in **33/47** missed-gap runs, against **0/48** with normal
  demand. One seed could not be built (no rightmost-lane car). A policy-driven
  car moves differently, so the measured rate under training will differ.
  Evidence: [missed-gap-native-check.json](evidence/highway-merge-training-v06/missed-gap-native-check.json)
  (SHA-256 `f6d203f319282754971adff336571b15c5788194e5203aa1469641294faa2e29`).

### Known limits

- Messages from other cars (`requires_merge` in message slots) are not faded.
- In rows with several AVs, the missed-gap blocker can itself be an AV.
  Training should use it in one-AV merge rows.
- The v05 runner still builds v05 environments. Wiring v06 into a runner and
  config is part of the Phase 3 proposal.
- The smoke run's constant command never merges. The new costs then add up to
  about -190 for `merge_tracking` per 400 decisions, against +5 for a quality
  merge. Phase 2 must check this balance.

**Rate limit (user decision, November 15, 2026): whole episode.** v06 uses
`rate_limit='full'` (the environment default): the 4 m/s3 command rate limit
applies for the whole episode, with maximum braking and lower safety ceilings
bypassing it. The raw-request jerk check still applies in evaluation. Next:
Phase 2, the offline reward audit on saved v04 trajectories.

