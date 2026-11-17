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

## Phase 2 - offline reward audit (November 15, 2026)

No simulation. [`scripts/audit_highway_flow_rl_v06_rewards.py`](../scripts/audit_highway_flow_rl_v06_rewards.py)
recomputes every v06 reward term for every decision of the 12 saved
Phase 0c baseline episodes (34 cars, 21,770 decisions). These were frozen v04
runs in startup mode, whose base terms and speed target are the ones v06 keeps.
The gap-preparation term is re-scored with the blended potential. Merge tracking
is also scored for the best acceleration- and jerk-limited trajectory (4 m/s3,
the v06 rate limit) against the same recorded target, which gives the reward a
smooth, well-matched merge would earn. Open-loop caveat as in Phase 0a.

**Change found by the audit: the jerk cost is now capped at 10 m/s3** (per-step
cost at most 1 x weight). Uncapped, emergency braking (raw requested jerk of
18-50 m/s3) cost 141 to 879 unit-weight points per episode. One emergency would
outweigh a whole episode of every other term and could teach the actor not to
brake. Emergencies stay with the safety layer. The 5-10 m/s3 range, where the
actor's spikes are (7-12 m/s3), keeps the full quadratic signal.

### Weights and the rules that set them

| Term | Weight | Rule | Evidence |
| --- | ---: | --- | --- |
| `merge_tracking` | 0.25 (was 0.50 provisional) | (a) A smooth merge must earn clearly more: weight x median tracking improvement >= the 5-point quality bonus, so >= 0.15. (b) The per-step cost must stay at most half the median per-step progress reward (0.80), so a car waiting for a gap still earns positive reward per step and gains nothing by ending the episode: <= 0.40. | Best-vs-observed improvement at 0.25: I-110 +25.4; ramp-started layouts +7.1 to +9.3; stuck ramp120 car +2.3 inside the window. |
| `jerk_excess` | 1.50 (unchanged), capped | An 8 m/s3 request (unit cost 0.36) must outweigh (a) the 99th-percentile per-step guidance shaping (0.072), needing >= 0.2; and (b) one step of tracking gain (0.25), needing >= 0.7. | Cost: 0.54 at 8 m/s3, 1.50 at 10 m/s3 and above. Per episode: about -3.7 for the startup spike on every ramp car, -5.0 for the I-110 focal car, -80.5 for the stuck ramp120 car (was -212 uncapped). |
| `avoidable_braking` | 0.25 (was 0.05 provisional) | At most one step of tracking cost: unneeded free-road braking at full depth costs as much as being off the merge target. At 0.05 the 60 mph cruise car's cost (-0.02) was negligible. | Charged on 7 cars, 64 decisions in total, all of them braking requests on a free road. |

### Farming and balance checks

- All three new terms are never positive in any of the 21,770 decisions, so none
  can be farmed by lingering, stopping or refusing to merge.
- Guidance shaping stays potential-based. Blending spreads the merge-onset jump
  over 1 s: the per-step 99th percentile is 0.072 (0.044 unblended), the episode
  total is unchanged within 0.6, and the 2.0 maximum is the terminal step.
- **Worst case, the stuck car** (ramp120 ramp-started, 553 pending decisions):
  tracking -116.6 and jerk -80.5 against base +495. Its return stays positive,
  so ending the episode early does not pay, but it is clearly worse than a
  timely merge.
- The stuck car's merge-window gain (+2.3) is below the quality bonus. Its loss
  is mostly the long pending phase before the window, which the missed-gap
  episodes target.

Evidence: [reward-audit.json](evidence/highway-merge-training-v06/reward-audit.json)
(SHA-256 `157737e642f1d35515dce79f0032daef65128230f39e8017fcc3c65718741661`);
the source traces' SHA-256 values are inside. 374 tests pass, including a test
for the jerk cap. Next: Phase 3, the training proposal for user approval.

## Phase 3 - resolved training proposal (November 15, 2026): awaiting approval

**Not authorized.** [`configs/training/highway-flow-rl-v06.json`](../configs/training/highway-flow-rl-v06.json)
has `training_authorized: false`, and the contract
([`traffic/highway_flow_rl_v06_contract.py`](../traffic/highway_flow_rl_v06_contract.py))
refuses to train until a reviewed commit sets it to true. The config is
generated by [`scripts/prepare_highway_flow_rl_v06.py`](../scripts/prepare_highway_flow_rl_v06.py).
The runner [`experiments/run_highway_flow_rl_v06.py`](../experiments/run_highway_flow_rl_v06.py)
reuses the v05 MAPPO loop unchanged: shared ledger, complete-episode rollouts,
exact resume, periodic validation and export. It swaps in the v06 environment,
contract, gates and stop rules.

| Item | Resolved value |
| --- | --- |
| Algorithm | MAPPO, v05 optimizer unchanged: actor lr 1e-4, critic lr 3e-4, clip 0.2, 20 epochs, minibatch 256, target KL 0.02, entropy 0.001, GAE gamma 0.99 / lambda 0.95. Seed 9811. |
| Warm start | v04 `long-round-03/best.pt` (SHA-256 `79e36b54...`), actor and critic; fresh optimizer. |
| Critic warmup | 20,000 critic-only decisions first, inside the 300k. The new costs shift returns (tracking alone is -20 to -117 per merge), so the inherited critic must re-learn values before actor updates. |
| Observations | 117 fields unchanged; actor and critic merge inputs faded over 1 s; safety guard, mask and lane permission use raw values. |
| Actions | Unchanged: acceleration plus lane intent. |
| Rewards | v04 base terms plus merge tracking 0.25 (gate target), jerk excess 1.50 capped at 10 m/s3, avoidable braking 0.25, blended guidance potential (Phase 2). |
| Control | Whole-episode 4 m/s3 command rate limit; Krauss controlled vehicles; original road-end rules (all user decisions). |
| Training rows | 20 rows, the v05 pilot curriculum with merges moved to the ramp-started layouts. 12 merge rows: I-110 x2, ramp-started 2/3/4-lane plus a new 5-lane layout, and **3 one-AV missed-gap rows (25%)**. 8 cruise/passing/following rows. One cycle is 101,600 decisions (35,000 merge, 4,800 missed-gap); about 3 cycles per round. All 33 missed-gap seeds the round can reach (10 cycles plus critic warmup) were pre-built successfully. |
| Budget | One round (ledger round 5): at most 300,000 decisions and 5,400 s charged; no second round. Expected about 45-60 min (v04 ran at about 130 decisions/s, plus 5 validations x 8 episodes). Study ledger before launch: 31,334 s of 41,400 s and 3,975,087 of 4,630,000 decisions used. |
| Validation | The Phase 0c rows: six development cases (ramp-started ramp120) plus the 2- and 4-lane ramp180 validation layouts, at start, every 100k decisions and at the end. Best checkpoint by the v05 validation ranking. |
| Stop rules | Any collision in any training or validation episode; or periodic-validation merge speed matching below **0.3128**, the frozen v04 value on the same rows (a missed stable merge counts as 0). Either stops the run for diagnosis with the checkpoint kept. |
| Acceptance | Original gates, no road-end exemption, plus the raw-request jerk check (at most 5 m/s3 per car). Baseline on these 8 rows: 2/8 pass. On ramp layouts merge quality is also reported against the physical ceiling. |
| Held-out | Not run. Only after development gates pass, with separate user approval; never used for tuning. |

**Preflight (zero-update, not training):** `outputs/highway-flow-rl/v06/preflight-01`
in the main checkout, 1 episode / 64 decisions / 3.3 s charged as preparation
(diagnostic episodes now 79/100). All checks passed (API, shapes, native
guards, finite gradient, export, reward and mask version), with zero collisions
and zero optimizer steps. `preflight.json` SHA-256
`e43900efa7019ddbec3c01bd02ee96f798638ccb7abfc74eeda47a0eb07cd5b8`.
Approval changes the config hash, so one more preflight episode is needed
before launch.

Tests: 388 passed and 1 skipped (`-k "v04 or v05 or v06 or demand"`),
including contract rejections, runner wiring and both stop rules.

**Launch steps after approval:** set `training_authorized` to true in a
reviewed commit; copy the runtime files to the main checkout after checking it
matches; run the preflight again; then
`experiments/run_highway_flow_rl_v06.py train --config configs/training/highway-flow-rl-v06.json --output outputs/highway-flow-rl/v06/round-05 --preflight <new preflight.json>`
in the background, and report when it exits.

## Round 5 training result (November 15, 2026): no improvement, nothing promoted

User approved the Phase 3 configuration ("Approve and launch"). Authorized in
commit `4b64751`; preflight repeated (`preflight-02`, 1 diagnostic episode,
80/100); trained in the main checkout at `outputs/highway-flow-rl/v06/round-05`.
The run completed normally in 2,489 s: 289,563 decisions (22,062 critic-only),
109 updates, no collisions or teleports in any episode, and no stop rule fired.
It ended when the remaining budget could not hold another complete episode.

| Validation (8 rows) | Original gates | + raw-request jerk | Merge speed match | Stable merges | Interventions | Peak jerk |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Initial (v04 actor, v06 env) | **4/8** | 1/8 | 0.437 | 4/4 | 0.27% | 4.8 |
| 100k (update 42) | 1/8 | 0/8 | 0.463 | 4/4 | 9.47% | 33.3 |
| 200k (update 78) | 4/8 | 0/8 | 0.468 | 4/4 | 1.96% | 36.5 |
| Final | 1/8 | 0/8 | **0.254** | **2/4** | 10.47% | 30.9 |

**Outcome: training did not beat its own starting point.** The v05 validation
ranking selected the initial (warm-start) policy as best. Merge speed matching
improved only slightly at 100k/200k (0.44 to 0.47) and collapsed at the end,
when both ramp180 validation merges were lost. The actor's requests became
jumpier, not smoother: the I-110 focal car's peak raw requested jerk rose from
13.9 to 20.8 m/s3 and the ramp cars' from 13.8 to 16.3, while interventions
(which include the comfort rate limit) rose to 10%. **No policy is promoted and
held-out evaluation is not run.** The v04 policy remains the reference.

**What the run shows:**
1. **The whole-episode rate limit alone helped the old policy.** The unchanged
   v04 actor passes 4/8 under the original gates in the v06 environment, against
   2/8 in the Phase 0c baseline (startup handoff only). Under the new
   raw-request check it passes 1/8: 15 of 29 cars request steps above 5 m/s3,
   which the rate limit then smooths.
2. **Learning was throttled.** Most actor updates hit the 0.02 KL target after
   1-4 optimizer steps (of up to 20 epochs). With one complete episode per
   update, every update was small and noisy, and only about 99 actor updates
   happened.
3. **The jerk cost did not shape requests.** Under a rate limit, the requested
   jerk is measured against the already-smoothed acceleration, so a stepped
   request is charged again on later steps. Above the 10 m/s3 cap there is no
   extra gradient. The policy found no smoother way to request commands.
4. **Missed-gap recovery did not improve.** Late-lane decisions in missed-gap and
   focused ramp episodes did not trend down (for example 142-256 per episode
   through training).
5. **The stop-rule floor was too lenient.** 0.3128 came from the startup-mode
   baseline, but the same actor already scores 0.437 under the v06 rate limit.
   The final drop to 0.254 was below the floor, but the floor applies only at
   periodic checks.

Evidence: [round-05-results.json](evidence/highway-merge-training-v06/round-05-results.json)
(SHA-256 `60c2169987632156c45e38a4fcb5aeb1d46334428f484c64d7aace85b489f864`),
with the hashes of the summary and the four validation files. Study ledger
after the run: 33,825 s of 41,400 s and 4,264,650 of 4,630,000 decisions used.
No further training is authorized.

## Round 5 diagnosis (November 15, 2026)

Read-only analysis of the saved round-5 validation episodes (per-decision action
logs) and the 99 PPO update records. No simulation, training or ledger charge.
Script: [`scripts/diagnose_highway_flow_rl_v06_round05.py`](../scripts/diagnose_highway_flow_rl_v06_round05.py).

**Corrections to the round-5 summary above** (kept unchanged as the
first-pass record):
- "Requests became jumpier": mostly wrong. Almost every raw-request check
  failure is the car's **first decision** (finding 1). Outside that first
  decision, 1 car fails at the start and 2 at the end.
- "1-4 optimizer steps per update": wrong. The median was 11 steps (63 max,
  1,593 in total). The accurate statement: KL early stopping ended 98 of 99
  updates, usually within the **first epoch** (median 1 of 20).
- "Lost 2 of 4 stable merges": the ramp180 cars did merge (lateral merge done
  at 6.2 s, as before). Their post-merge window failed only on
  `intervention_fraction`, caused by the speed-limit guard (finding 3).

### Findings

1. **The raw-request jerk check mostly measures vehicle insertion.** SUMO
   inserts each car at 0 m/s2. The actor's first command (typically 0.6-1.7
   m/s2) is compared with that 0, which gives 6-17 m/s3 on one decision. Of the
   failing cars, 14 of 15 at the start and 18 of 20 at the end fail only on
   that first decision. With the first decision excluded (a diagnostic count,
   not a rule change), the starting policy passes 3/8 instead of 1/8, and the
   200k checkpoint 3/8 instead of 0/8. Training made the first command larger,
   which is why the failing count rose from 15 to 20. During driving, the
   actor's own command changes stay small: the median per-car 95th percentile
   is 0.2-0.26 m/s2 per decision, below the 0.5 limit.
2. **The large realized jerk (31-36 m/s3) is one car, a late merge.** In the
   100k, 200k and final policies, the I-110 focal car reaches the end of its
   acceleration lane unmerged. The lane-end guard brakes it from 0 to about
   -3 m/s2 in one step, it stops for 6.6 s, and native SUMO restarts it. The
   starting policy merged without stopping. All other realized peaks are
   16 m/s3 or below, from the speed-limit guard. This is a merge-timing
   regression, not a smoothness one.
3. **Most of the 10% interventions are the speed-limit guard.** Speed-ceiling
   interventions rose from 3 (start) to 1,644 (final), plus 148 lane-end
   decisions from the stopped I-110 car. The final policy asks for about +0.05
   m/s2 while already at the speed limit, and the guard trims it every step.
   On the one-AV cruise case that is 86% of decisions, at 99.8% of the limit
   (start: 99.0%, zero interventions). Physically harmless, but it fails the
   5% intervention gate on cruise, passing and I-110 following, and the ramp180
   post-merge windows. The reward already charges -0.2 per intervention, so this
   drift ran against the reward signal (finding 5).
4. **Merge quality was never passable on 3 of the 4 merge rows.** Every
   checkpoint fails `rolling_merge_quality` on all four merge rows. On the
   ramp-started layouts the measured ceiling is below 0.90 (ramp120 0.69,
   ramp180 0.57), as recorded in Phase 0c. So the best possible score on these
   8 rows is 5/8, and the starting policy already passed the 4 non-merge rows.
   The only real room was the I-110 merge (ceiling 0.99) and closing the gap to
   the ceiling elsewhere. The 200k checkpoint did close it a little (ramp120
   0.45 to 0.50, ramp180 0.39/0.37 to 0.43/0.43), while I-110 fell from 0.53 to
   0.50 with the lane-end stop.
5. **The learning signal was weak and noisy.** With one complete episode per
   update (median 1,042 samples), the KL check stopped 98 of 99 updates, and
   33 never finished one epoch. Mean value loss rose from 5.6 to 9.8 in the
   middle of training before falling to 6.6. Gradient norms averaged about 18
   against a 0.5 clip. A small systematic drift (finding 3) can then win over a
   -0.2 per-step cost. Training rows are mostly merges, so the cruise-at-limit
   behavior gets little correction. This is an inference from the statistics,
   not a tested cause.

### What this changes

- The jerk diagnosis that motivated an acceleration-change action is weaker
  than reported. Requests are mostly smooth; the check is dominated by the
  insertion step. An acceleration-change action is **not** indicated by this
  evidence.
- The concrete regressions to fix are (a) speed-limit riding and (b) the I-110
  late merge, both under a noisy one-episode-per-update optimizer.
- Options for user decision (none started):
  1. Score the raw-request check from the second decision, or seed each car's
     insertion acceleration from its first command. Either is a rule or
     environment change and needs a decision; results would be reported beside
     the current rule.
  2. Larger, steadier updates: several episodes per update and/or a looser KL
     target, within the remaining budget (7,575 s, about 365,000 decisions).
  3. Stop speed-limit riding: clamp a positive request at the ceiling before
     it counts as an intervention, or add cruise rows to the curriculum.
  4. Do not train further in this study; keep v04 with the whole-episode rate
     limit as the reference (4/8 under the original rules).

Evidence: [round-05-diagnosis.json](evidence/highway-merge-training-v06/round-05-diagnosis.json)
(SHA-256 `dac359ede09fc30fba651d8ea0069257eb4dfa2bd8af895d7bdeb318600f966d`),
with the input hash of each validation report.

## Round 6 proposal (November 15, 2026): fix scoring and updates

User decision after the round-5 diagnosis: "Fix scoring + updates" (options 1
and 2). Speed-limit riding gets no objective change; it is guarded by a stop rule
instead. Code, configuration and tests are ready. `training_authorized` is
**false**, and no preflight has been run for this configuration yet. The preflight
must follow authorization anyway, because the configuration hash changes.

| Item | Round 5 | Round 6 (proposed) |
| --- | --- | --- |
| Raw-request jerk check | every decision | from each car's **second** decision. The first decision is compared with SUMO's 0 m/s2 insertion value; its realized jerk stays under the original per-car gate. Every validation also reports the every-decision result. |
| Episodes per PPO update | 1 | **4** complete episodes, one on-policy batch, advantages normalized once |
| KL target / epochs | 0.02 / 20 | **0.03 / 10** |
| Stop: merge speed match | below 0.3128 (startup-mode baseline) | below **0.4167** (this actor's round-5 initial validation in the v06 environment, 0.4367, minus 0.02) |
| Stop: interventions | none | periodic-validation pooled interventions above **5%** (the per-case gate value; round 5 reached 9.5-10.5%) |
| Round index / seeds | 5 | 6 (every training seed shifts; all 33 missed-gap seeds for 10 cycles rebuilt) |
| Unchanged | | rows, reward and weights, faded inputs, whole-episode rate limit, Krauss, original road-end rules, safety, warm start (v04 best), critic warmup 20,000, 300,000 decisions / 5,400 s, validation rows, held-out not authorized |

**Expected effect and limits.** About 99 PPO episodes become about 25
updates of roughly 10,000 samples. Each KL check then sees about 4 times more
data per epoch before stopping, but there are about 4 times fewer updates.
Up to 4 episodes are collected but never learned from: on interruption, or when
the budget ends mid-group. They show as trailing `deferred` update records and
their decisions stay charged. The scoring change cannot affect training: the reward
still charges the first decision's requested jerk, as in round 5. Merge quality
on the three ramp validation rows still cannot reach 90% (ceilings 0.57-0.69),
so the best possible result on the 8 rows is 5/8.

**Verification (no ledger charge).** 557 tests pass, 3 skipped, across the
highway-flow suites. New tests cover the scope rule, the gates reporting both
scopes, the buffered trainer (merge, no duplicate on retry, exact round-5
behavior at 1 episode), the intervention stop rule, the round-6 contract
rejections, and round-6 seed separation. A SUMO smoke run in the worktree (2
short episodes, sampled actions) confirmed:
- the new per-car metric and scope reach the report;
- the first episode was deferred, and the second triggered one real update on
  the merged 300 samples that changed the weights.

Remaining budget: 7,575 s and 365,350 decisions. Round 5 used 2,489 s for its
300,000-decision round, so one round fits. Diagnostic episodes: 80/100 (the
preflight needs 1).

Files: [`configs/training/highway-flow-rl-v06-round6.json`](../configs/training/highway-flow-rl-v06-round6.json)
(generated by [`scripts/prepare_highway_flow_rl_v06_round6.py`](../scripts/prepare_highway_flow_rl_v06_round6.py)),
`traffic/highway_flow_rl_v06_contract.py` (`ROUNDS[6]`),
`experiments/run_highway_flow_rl_v06.py` (`BufferedMappoTrainer`, stop rule),
`environments/highway_flow_rl_v06_env.py` and `traffic/highway_flow_rl_v06_rewards.py` (scope).

### Launch correction: runs as round 7 (November 15, 2026)

The user approved the round-6 profile ("Approve and launch"). Before launch,
the shared study ledger showed that **round index 6 was already charged** by the
v07 cut-in run from another session (`documentation/highway-merge-cutin-2026-11-15.md`):
preparation episodes plus 100,728 training decisions and 1,133 s, stopped at its
merge-regression rule. The runner refuses a fresh round on a charged index, and the
budget quoted above was out of date. Actual remaining budget at launch: **6,400 s and
264,622 decisions**; diagnostic episodes 90/100.

User decision: "Round 7, all remaining". The same profile is
`contract.ROUNDS[7]`, with config
[`highway-flow-rl-v06-round7.json`](../configs/training/highway-flow-rl-v06-round7.json).
Every training seed shifts by one more round, and all 33 missed-gap seeds were
rebuilt. The runner stops at the smaller of the 300,000-decision / 5,400 s round
limit and the study total, so the study budget will be essentially used up.
The round-6 config stays as the approval record, marked superseded and not
authorized.

## Round 7 result (November 15, 2026): stopped by the merge stop rule at 100k

The approved round-6 profile ran as round 7 (`outputs/highway-flow-rl/v06/round-07`).
Preflight passed (`preflight-r07`). The first periodic validation (100,250
decisions, 912 s charged, 31 PPO episodes in 7 merged updates plus 10
critic-only episodes) triggered the declared stop rule: merge speed match
**0.343**, below the 0.4167 floor. No collisions or teleports. Nothing is
promoted, and held-out evaluation is not run.

| Validation (8 rows) | Original gates | + raw check from 2nd decision | + raw check, every decision | Merge speed match | Stable merges | Interventions | Peak jerk |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Initial (v04 actor) | 4/8 | 1/8 | 1/8 | 0.437 | 4/4 | 0.27% | 4.8 |
| 100k (update 41) | 4/8 | 2/8 | 1/8 | **0.343** | 3/4 | 0.46% | 4.0 |

**What happened:**
1. **The stop came from one car.** I-110 speed matching rose from 0.534 to
   0.565. The ramp120 and ramp180 4-lane merges were nearly unchanged (0.454 to
   0.447; 0.369 to 0.362). On the 2-lane ramp180 the car merged as before (6.2
   s), but its post-merge window failed `intervention_fraction`: the speed-limit
   guard trimmed it 40 times. A failed stable merge counts as 0, so the mean
   fell from 0.437 to 0.343. With that car at its earlier value, the mean would
   be about 0.44. This is the same speed-limit riding as round 5, now in one
   case (pooled interventions only 0.46%).
2. **The original-rule result held (4/8).** No realized jerk exceeded 4.0
   m/s3 (round 5's 100k checkpoint: 33 m/s3), and the I-110 car did not stop at
   the lane end.
3. **Correction to the round-5 diagnosis, finding 1.** The insertion effect is
   not one decision. A ramp car's first command is about 1.6-1.7 m/s2. The
   whole-episode rate limit raises the realized command by only 0.4 m/s2 per
   decision, so |request - measured| stays above 0.5 m/s2 for about 3-4
   decisions (the second decision still reads 12-13 m/s3). The diagnosis
   attributed that whole lag run to "first decision" and counted it out. The
   implemented rule (skip only the first decision) therefore passes 1/8 at the
   start, not the predicted 3/8. The raw-request check stays dominated by the
   insertion handoff, not by the actor's behavior once driving.
4. **Updates were still KL-limited, now by a known cause.** All 7 merged
   updates (4,053-16,992 samples) stopped on KL (last KL 0.030-0.037 against
   0.03), 5 of them inside the first epoch (236 optimizer steps in total). The
   warm-start policy's action standard deviation is 0.04 in normalized units
   (log std -3.22, about 0.13 m/s2). For a Gaussian, a KL of 0.03 is reached
   when the mean moves by only about 0.01 normalized (about 0.03 m/s2). So each
   update can shift commands by only about 0.03 m/s2. Larger batches cannot
   change that. The same scale matches the +0.05 m/s2 drift that trips the
   speed-limit guard.

Evidence: [round-07-results.json](evidence/highway-merge-training-v06/round-07-results.json)
(SHA-256 `71aa3a6145014b4ef3e81f906bec0e699d99727ee1f15e7dcf55d10451e3d2ce`), with validation hashes, per-case failures and update
statistics. Study ledger after the run: 35,914 s of 41,400 s, 4,465,628 of
4,630,000 decisions, and 91 of 100 diagnostic episodes used. That leaves 5,486 s
and 164,372 decisions, which is less than one full round. No further training is
authorized.

## Round 8 proposal (November 17, 2026): final round on the remaining budget

The user asked for a final-round proposal after round 7. The resolved
configuration is
[`highway-flow-rl-v06-round8.json`](../configs/training/highway-flow-rl-v06-round8.json)
(SHA-256 `b876e2468a94b28dc9e8574a1454de7be15a49579716b33880a8c14f4eb207f9`),
profile `contract.ROUNDS[8]`, with **`training_authorized=false`**. Nothing
runs until the user approves it.

**Budget.** The study has 5,486 s, 164,372 decisions and 9 diagnostic episodes
left. One curriculum cycle (20 training rows) is 101,600 decisions. Of the
164,372, 20,000 go to critic warm-up and about 144,000 to PPO: roughly 35
episodes, or 9 merged updates. The run gets one periodic validation at 100k and
a final validation. Preflight uses 1 diagnostic episode. Round 7 charged 912 s
per 100k decisions, so time is not the limit.

**What stays the same as round 7:**
- training/validation/held-out rows;
- frozen v04 warm start;
- reward weights, observation, whole-episode rate limit and safety settings;
- 4 complete episodes per update and 10 epochs;
- merge-floor value 0.4167;
- collision and pooled-intervention (5%) stop rules.

**Changes:**

1. **Raw-request jerk scored after a fixed 5-decision insertion lag.**
   - [round-08-insertion-lag.json](evidence/highway-merge-training-v06/round-08-insertion-lag.json)
     (SHA-256 `3215c801cce6650246cb1c2e1578d22264b1253baf1a2934453e634257a00a6a`,
     from `scripts/measure_highway_flow_rl_v06_insertion_lag.py`) re-scores
     round 7's saved validations.
   - The insertion lag lasts at most 4 decisions.
   - Cases passing the original gates plus the raw check are:
     - 1/8 with no skip or one skipped decision;
     - 3/8 with 2 to 8 skipped decisions.
   - The round-8 window is a fixed 5 decisions (0.5 s): the measured lag plus
     one decision. It does not depend on the policy, and realized jerk in those
     decisions still has to stay under the original 5 m/s3 gate.
   - Starting value: 3/8, since validation is deterministic and the warm start
     is the same. Ceiling: 4/8 (5/8 if the I-110 merge also passes).
   - One failure that remains is a genuine actor jump: car `agent_2` in
     I-110 following reaches 6.0 m/s3 after the window.

2. **KL target 0.1 instead of 0.03.**
   - For a Gaussian, the mean can move about sigma * sqrt(2 * KL) before the
     per-minibatch early stop fires. With the warm-start sigma of 0.04
     normalized, that is:
     - 0.0098 normalized (about 0.03 m/s2) per update at a KL target of 0.03;
     - 0.018 normalized (about 0.06 m/s2) at 0.1.
   - In round 7, every merged update stopped on KL at 0.03, mostly within the
     first epoch.
   - The critic shares that early stop, so it was also under-trained.
   - Risk: larger steps can degrade the policy, as in round 5. The intervention
     and collision stops stay, and checkpoint selection keeps the warm start
     unless a validation ranks better.

3. **Merge-speed floor reported, not enforced, at periodic validation**
   (`stop_rules.merge_floor_action='report'`).
   - Round 7 stopped at 100k because one ramp180 car failed its stable window,
     although I-110 was improving.
   - The remaining budget cannot fund another round, so stopping early saves
     nothing usable.
   - The floor result is still recorded (`merge_speed_match_below_floor`).
   - Earlier rounds' profiles cannot switch it off.

**Not changed: speed-limit trims.**
- The ramp180 car's 40 trims were tiny: median 0.028 m/s2, largest 0.21 m/s2.
- A cost graded by trim size would be close to zero.
- Each trim already costs the reward's −0.2 intervention penalty.
- This round relies on larger update steps to let that penalty act. If the
  trims persist, the case still fails its gate.
- The intervention gate, the guard and the reward weights are unchanged.

**Seed check.** The missed-gap seeds were rebuilt for every cycle that the
300,000-decision round cap can reach (PPO cycles 0-3 and critic cycle 0): 15
seeds, all built. One seed does not build: row 16 (ramp240 missed-gap), seed
9177010 at round-8 cycle 7. No round can reach that cycle.

**Expected outcome (not a promise).**
- The realistic target is the I-110 merge passing the original gates (5/8).
  It rose from 0.534 to 0.565 in round 7, against a layout ceiling of 0.99.
- The ramp layouts cannot pass merge quality (ceilings 0.69 and 0.57).
- Held-out evaluation stays unauthorized.

### Launch correction: runs as round 9 under the shared study ceilings (November 17, 2026)

The user approved the round-8 profile ("Approve and launch"). Before launch, the
shared study ledger showed two changes:

- **Round index 8 was already charged** by another session's v08
  merge-recovery run
  ([highway-merge-recovery-2026-11-17.md](highway-merge-recovery-2026-11-17.md),
  branch `2026-11-17-codex-merge-recovery`): 298,361 training decisions and
  3,752 s, plus preparation episodes.
- That run used the shared study ceilings first approved for v07 (4.93M
  decisions / 46,800 s), so the ledger (4,763,989 decisions, 39,692 s) is now
  past the v06 ceilings (4.63M / 41,400 s). Under v06's own budget nothing
  remained.

User decision: "Round 9, shared ceilings". The same profile is
`contract.ROUNDS[9]`, with the shared ceilings as its only budget change
(`SHARED_STUDY_BUDGET`). The config is
[`highway-flow-rl-v06-round9.json`](../configs/training/highway-flow-rl-v06-round9.json)
(SHA-256 `91947d58d48feabe1efa97bb0a678909a27ba419ffab86b124e506cf2da99217`, `training_authorized=true`).

**Remaining budget at approval:**
- 166,011 decisions and 7,108 s;
- 98 of 100 diagnostic episodes used.

The runner caps the round at the smaller of the 300,000-decision / 5,400 s
round limit, the study totals and the 900 s final reserve.

**Seeds.** Every training seed shifts by one more round (+80,000). The 15
missed-gap seeds for the reachable cycles were rebuilt, and all built.

**Shared ledger.** Further v08 training would compete for the same shared
remaining budget. The round-8 config stays as the approval record, marked
superseded and not authorized.
