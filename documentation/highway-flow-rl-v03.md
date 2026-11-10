# Highway Flow RL v03: pre-action speed recovery and smoothness rewards

**Current status:** the longer continuation is complete. Fleet metrics improved, but all six case acceptance gates still fail; no promotion. [Latest results](#longer-continuation-result-and-spawn-repair). Earlier preparation and pilot sections are historical.

November 9, 2026. Implemented and checked offline, **not trained**. No new SUMO,
Isaac or rendering runs. This implements the approved
[reward plan](highway-smooth-reward-plan-v03.md); it does not promote a new actor.

## Final review configuration

The executable [configuration](../configs/training/highway-flow-rl-v03.json) has
`training_authorized: false`, held-out evaluation disabled and the second round
unauthorized. The [runner](../experiments/run_highway_flow_rl_v03.py) requires
explicit training authorization plus a fresh native, zero-update preflight tied
to the exact configuration and source hashes. Offline rescoring is not that proof.

| Item | Resolved value |
| --- | --- |
| Algorithm | Shared actor MAPPO, centralized critic; no imitation |
| Initialization | Frozen flow117 v01 round-two actor and critic; fresh optimizer |
| Inputs/actions | Actor 117, critic 2,832, maximum 24 agents; unchanged normalized acceleration and discrete lane intent |
| Control interval | 0.1 seconds |
| Speed ceiling | 26.8224 m/s (60 mph), subject to lower local/scenario limits |
| Training coverage | 12 rows, 2–5 through lanes; cruise, following, passing and I-110 merging; existing AV mixes and communication variants |
| Scenario speed caps | 18, 22.352 and 26.8224 m/s |
| Learning rates | Actor 0.0001; critic 0.0003 |
| PPO | Clip 0.2; four epochs; minibatch 256; target KL 0.02; entropy 0.001; max gradient norm 0.5 |
| Returns | Existing GAE defaults: gamma 0.99, lambda 0.95 |
| Pilot | At most 30,000 active-agent decisions and 900 seconds, including development evaluations |
| Rollout | Default 2,048 decisions; existing per-case limits retained |
| Seed | 9708; existing disjoint training/development/held-out demand seeds |
| Safety | Native speed mode 31, lane-change mode 512, analytical guards and action masks unchanged |

The shared study budget remains 2,760 seconds. **121.328 seconds** are now charged
in the existing `outputs/highway-flow-rl/v02/cost-ledger.json`, including both
offline audit revisions (about 5.594 seconds total). No reset/refund. Seven of
eight native diagnostic allocations remain used; offline audits add zero native
episodes. The final allocation is still reserved for the new preflight.
The 600-second final evaluation reserve and separately unauthorized second pilot
remain as before. No automatic loop of additional training is enabled.

## Reward calculation

| Component | Signed contribution |
| --- | --- |
| Route progress | `+1.0 * distance_this_step / 2.68224` |
| Safe-target speed error | `-0.30 * min(1, max(0, abs(next_speed - target) - 0.5) / max(target, 5))` |
| Smoothness | `-0.09 * Huber(jerk / 2)`; quadratic below magnitude one, linear above; uncapped |
| Optional ending-lane detour | `-2.0` once, on first measured lateral motion |
| Merge / stable merge / pass / arrival | Existing `+5 / +2 / +2 / +2` event terms |
| Collision / intervention / unsafe request | Existing `-20 / -0.2 / -0.5 * normalized excess` |
| Lane change / reversal / cooperation | Existing `-0.1 / -0.5 / +0.1 * team merge signal` |

The old absolute-jerk coefficient is **zero** in v03 to prevent double charging.
The v02 positive cruise bonus is removed. There is no generic penalty on all
acceleration/braking and no action smoother. Actual arithmetic lives in
[pure reward helpers](../traffic/highway_flow_rl_v03_rewards.py), with per-step
accounting in the [environment](../environments/highway_flow_rl_v03_env.py).

Huber examples: magnitude 1 / 2 / 5 / 10 / 20 m/s3 costs
0.01125 / 0.045 / 0.18 / 0.405 / 0.855. This penalizes abrupt changes more than
the old `0.03 * abs(jerk)`, while tolerating small smooth adjustments. It is
deliberately **not a larger penalty at every jerk magnitude**. The persistent
speed-error term addresses gradual unnecessary slowing.

The target is frozen before the action and is independent of requested or
previous acceleration. It considers occupied lanes, current leaders, lower
upcoming speed limits, required merge front/rear gaps, yield visibility and
ending-lane runway. It uses a 1.5 s headway floor, 2.5 m minimum gap, 0.3 s
reaction allowance, follower/leader braking estimates 4.5/9 m/s2, 2 m/s2 preview
deceleration and a three-second constant-speed merge projection. These are
explicit reward assumptions, not a future-state oracle or a safety certificate.
They are not fed to the actor as additional observations or used to change masks.

Unknown evidence, unresolved neighboring lateral motion and incompatible merge
intervals abstain from the speed-error term and retain all other reward/safety
terms. A required slowdown therefore does not automatically target highway speed.
Large errors saturate at -0.30; safety still comes from the unchanged guards and
safety costs. The reward alone is not a collision-avoidance guarantee.

## Saved-trace results

[Metrics and hashes](evidence/highway-flow-rl-v03/saved-trace-metrics.json) retain
the five frozen development cases. Bulk per-car CSVs and the resolved configuration
are in `outputs/highway-flow-rl/v03/offline-reward-audit-02/`. The first audit is
also retained. The [offline audit script](../scripts/audit_highway_flow_rl_v03_rewards.py)
never calls the simulator or optimizer and never consumes held-out data.

- **18,492 decisions** rescored; target evidence qualifies for **17,669 (95.55%)**.
- **18,466** measured post-speed linkages checked against recorded acceleration;
  maximum discrepancy below 1.4e-17 m/s. Missing final/arrived post-states are
  explicitly excluded from the speed term, not reconstructed as successful motion.
- The I-110 focal car has **396/545 qualified targets (72.66%)**. Its 149 excluded
  decisions include 147 incompatible current merge intervals and two unknown
  required targets. This limits coverage; it is not a driving pass.
- First dip, 6.6–10.2 s: **zero flagged avoidable-braking decisions**. A tight
  target-lane gap makes blanket punishment of that deceleration inappropriate.
- Later dip, 17.4–21.1 s: **14 flagged decisions, one persistent event**. The speed
  term costs **2.223** across the window; the old cruise addition was zero.
  The optional ending-lane move now costs **2.0**, previously 0.5.
- Across the full focal route: speed-error cost **26.948**; Huber jerk cost
  **11.384**, versus old jerk cost **12.224**. Total reward changes from
  **185.225 to 155.682** on exactly the same trajectory. Lower revised reward is
  an accounting change, not a performance decline or improvement.

The avoidable-braking diagnostic requires three consecutive 0.1 s requests below
-0.2 m/s2 with a qualified pre-action target at least 0.5 m/s above current speed,
safe ceiling at least +0.2 m/s2, and no yield/ending-lane obligation. Discontinuous
samples or new obligations break the episode of evidence. It reports exclusions,
coverage and subsequent reacceleration without altering vehicle commands.

## Validation and next gate

**89 focused CPU tests pass**, including existing v02 regressions. Tests cover
the Huber scale, acceleration-independent targets, recovery versus stalling,
equal-progress oscillation cost, close leaders, required merges and rear gaps,
yielding, lower limits, ending lanes, missing evidence, first-motion accounting,
unchanged native commands/actor shapes/masks, reward/journal sums, launch denial,
stale-proof rejection, exact-resume provenance and retained ledger charges.
[Validation record](evidence/highway-flow-rl-v03/validation.json).

Keep the existing per-car/full-route collision, overspeed, intervention <=5%,
uncensored peak jerk <=5 m/s3, headway >=1.5 s, closing TTC >=3 s, merge and passing
gates. Cruise cases additionally require measured clear-road coverage, >=90%
legal-speed utilization there, and no flagged persistent avoidable braking.
Missing coverage cannot pass. Emergency safety interventions remain enabled.

Next: review this final configuration. After authorization, run the fresh native
zero-update preflight before the bounded pilot. Save checkpoints and metrics
automatically; inspect only completion or failure while training runs. No driving
improvement, accepted checkpoint, held-out result or native v03 validation is
claimed here. Google guide incremental synchronization remains pending; its
generated Markdown mirror was not edited.


## Round-one training result

The user authorized the presented v03 configuration on November 9. Only its
authorization flag and authorization provenance changed. Native zero-update
preflight passed against the exact launch config/source hashes before training.
No language-model analysis or metric inspection occurred during the run.

The pilot completed **29,999 active-agent decisions**, 18 rollout updates and
444 optimizer steps in **294.859 seconds** including initial/final development
evaluation. The runner stopped below the 30,000-decision ceiling with no error.
Both six-case evaluation sets covered their declared horizons.

| Development metric | Before | After |
| --- | ---: | ---: |
| Pooled safety overrides | 4.74% | 10.21% |
| Maximum uncensored jerk, m/s3 | 57.10 | 59.73 |
| Collisions / teleports | 0 / 0 | 0 / 0 |
| Stable merges | 2 / 2 | 1 / 2 |
| Cases passing every acceptance check | 0 / 6 | 0 / 6 |
| I-110 focal avoidable-braking events | 1 | 0 |
| I-110 focal optional ending-lane detours | 1 | 0 |
| I-110 focal intervention fraction | 21.47% | 31.37% |
| I-110 focal peak jerk, m/s3 | 27.86 | 40.57 |

The targeted focal behaviors improve, but **overall driving acceptance regresses**.
Do not promote this actor. The existing accepted actor remains untouched.
There was no held-out test, automatic second round or parameter revision.
[Compact per-case/per-car comparison and hashes](evidence/highway-flow-rl-v03/round-01-results.json).

Local artifacts under `outputs/highway-flow-rl/v03/round-01/` include `policy.pt`,
`last.pt` (full model/optimizer/RNG/training state), `summary.json`, complete
initial/final validation, source snapshots, resolved configuration and raw
action journals. Console output is `logs/highway-flow-rl-v03-round-01.log`.
Weights are saved locally, not uploaded as release assets in this milestone.

The shared ledger now retains **418.687 seconds** including preflight and both
offline audits. All eight native preparation allocations are consumed. The
second round and held-out evaluation remain unauthorized. Next: inspect the
saved per-car command/override traces and the failed short-ramp merge before
proposing another reward or training change. The authorization flag records
this completed round; the ledger prevents silently starting it again. Google
guide incremental synchronization remains pending.


## Authorized longer continuation

The user requested longer learning and more epochs after the rejected pilot.
[Separate resolved configuration](../configs/training/highway-flow-rl-v03-long.json):
**300,000 additional active-agent decisions, up to eight PPO epochs per update,
5,400 seconds (90 minutes) including development evaluation**. The KL limit
remains 0.02, so updates can stop before all eight epochs. Actor/critic learning
rates, minibatch size, rewards, safety, masks, observations and scenario rows
are unchanged. Continue from the saved v03 round-one actor and critic with a
fresh optimizer; this is a new configured round, not an exact optimizer resume.

95 focused tests and a fresh zero-update native preflight pass.
[Launch evidence](evidence/highway-flow-rl-v03/long-launch.json). The budget
contract now explicitly recognizes this one authorized extended profile; the
original pilot's limits still apply to its original configuration. One new
native preflight allocation was added, with all previous time and decisions
retained in the same ledger. The study total cap is now 6,600 seconds (including
prior work and the unchanged 600-second evaluation reserve), and total learning
is capped at 330,000 decisions across both rounds. No third round or held-out
evaluation is authorized. Save checkpoints automatically and inspect results
only at completion/failure. Existing accepted actors remain unchanged.


## Longer continuation result and spawn repair

The extended run completed **298,400 actual learning decisions**, with **300,000
charged decisions** including the 1,600-decision failed-rollout reservation.
177 successful updates performed 7,130 optimizer steps. Of those updates, 106
completed all eight epochs; the remainder ended earlier under the retained KL
guard (zero completed epochs can include a partial first epoch). Training runtime
was **1,738.281 seconds, about 29 minutes**, across the failed and resumed segments.
No intermediate metrics were inspected while either training process ran.

At seed 8319007, SUMO rejected native `ramp_0` at 9.697 m/s because the short ramp
lacked stopping room before the unprioritized junction. The old placement rule
considered braking distance but omitted cautious-driver reaction distance.
[SUMO's insertion rules](https://sumo.dlr.de/docs/Simulation/VehicleInsertion.html)
require the departing vehicle to be able to brake for an upcoming yield junction.

Demand schema v2 now caps the common ramp-platoon departure speed using 2.0 s
reaction time plus one 0.1 s tick, 4.5 m/s2 normal braking and a 2.5 m margin.
The failed fixture's speed becomes 8.422 m/s. Random draws, counts, positions,
spacing and profiles are preserved. The cap precedes controller assignment,
so changing the AV fraction does not change physical demand. No vehicle is
discarded and no safety or reward rule is relaxed.

127 focused regression tests pass. A fresh native preflight on the actual failed
seed passes with the saved learned weights and zero optimizer updates. The
checkpoint migration explicitly records old/new source/config hashes and
preserves actor, critic, optimizer, RNG and learning counters. It is a documented
source-compatible recovery, not a claim of bitwise exact replay across a code
change. The original checkpoint and failure remain immutable. The failed rollout
is discarded on recovery and its full reservation remains charged.
[Migration provenance](evidence/highway-flow-rl-v03/spawn-repair-migration.json).

Offline old/new demand comparison found five development fixtures physically
unchanged. The short-ramp development fixture alone changed its starting speed
from 8.924 to 8.315 m/s, so that baseline case was rerun using the same frozen
round-one actor. The migrated initial evaluation replaces only that case,
retaining the original result in the failed-run evidence. This explains why the
corrected pooled baseline is 10.50%, versus 10.21% in the earlier pilot report.
Two additional native preparation allocations (baseline repair and failing-seed
preflight) were recorded, raising the explicit allocation cap to 11 while
retaining the same 6,600-second study and 90-minute round limits.

| Matched development metric | Corrected starting baseline | Longer-trained actor |
| --- | ---: | ---: |
| Pooled interventions | 10.50% | 4.33% |
| Stable merges | 1 / 2 | 2 / 2 |
| Collisions / teleports | 0 / 0 | 0 / 0 |
| Maximum uncensored jerk, m/s3 | 59.74 | 55.61 |
| Cases passing every gate | 0 / 6 | 0 / 6 |
| I-110 focal interventions | 31.37% | 32.79% |
| I-110 focal mean absolute jerk, m/s3 | 1.216 | 0.547 |
| I-110 focal p95 absolute jerk, m/s3 | 4.989 | 1.613 |
| I-110 focal peak jerk, m/s3 | 40.57 | 40.71 |
| I-110 focal mean legal-speed fraction | 61.65% | 58.98% |
| I-110 focal avoidable-braking events | 0 | 1 |
| I-110 focal optional ending-lane detours | 0 | 0 |

The longer run helps fleet averages and restores the second merge, but the
main car still relies heavily on interventions and is slower. Its typical jerk
improves, while the peak remains unacceptable. Passing a pooled 5% threshold
does not satisfy the per-car gate. **Do not promote this candidate.** Existing
accepted policies are unchanged; no held-out evaluation or third round ran.
[Per-case/per-car metrics, epoch counts, provenance and hashes](evidence/highway-flow-rl-v03/long-round-02-results.json).

Final experimental actor: `outputs/highway-flow-rl/v03/long-round-02-resumed/policy.pt`.
Full checkpoint: `last.pt` in the same folder. Retain both `long-round-02/` and
`long-round-02-resumed/` for the complete learning history, plus `spawn-repair/`,
`short-ramp-baseline-repair/` and `repaired-preflight-01/`. Runtime logs are under
`logs/`. All costs remain in the shared ledger, now totaling **2,169.579 seconds**.
The next useful diagnosis is the focal car's intervention causes and speed
recovery, using saved traces before another learning change. Guide sync pending.


## November 9 - completed offline policy audit

The [full audit](highway-policy-audit-v01.md) replaces the pending main-car
diagnosis above. Its dominant override is the lane-end guard (173 decisions),
while the speed-target term abstains on 344/549 focal decisions and no left
merge is allowed until near-stop. Training coverage and maneuver-feasibility
mismatches are documented with actual rewards, gradients and verified actor
replay. No reward/configuration/runtime changes or further training were made.
