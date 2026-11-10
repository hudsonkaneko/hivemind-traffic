# Highway v03 policy audit

November 9, 2026. **Offline audit only: no training, simulator runs, or runtime
changes.** Candidate remains rejected. This audits the completed long v03 run,
not every historical policy. Scope: its reward and target functions, actor and
critic, optimizer, rollout collection, observations/messages, masks, safety
filters, SUMO execution, demand, evaluation and checkpoint provenance.

The clearest failure chain is: **no early admissible merge gap -> speed-target
reward abstains -> car reaches the ending-lane braking guard -> prolonged stop
before a merge becomes executable**. The guard's role and reward abstention are
directly measured. Whether a particular early speed adjustment would avoid the
stop still needs a matched closed-loop test; saved traces cannot prove that
counterfactual. Increasing the jerk weight or training longer alone does not
address this chain.

## Evidence and reproducibility

- Actor: `outputs/highway-flow-rl/v03/long-round-02-resumed/policy.pt`; full
  checkpoint: `last.pt` beside it. The [prior result](highway-flow-rl-v03.md#longer-continuation-result-and-spawn-repair)
  records the failed/resumed run and source-compatible spawn repair.
- [Audit script](../scripts/audit_highway_policy_v03.py),
  [compact evidence](evidence/highway-policy-audit-v01/audit-summary.json),
  [focal timeline](evidence/highway-policy-audit-v01/focal-timeline.csv).
- Full audit, per-case CSVs and focal JSON:
  `outputs/highway-flow-rl/v03/policy-audit-01/`. Checkpoints and raw journals stay
  local; the compact evidence includes their SHA-256 hashes, source fingerprints
  and the exact resolved configuration.
- Replayed every recorded observation through both the exported actor and full
  checkpoint: **17,230 decisions**, identical action tensors, matching lane
  choices, maximum decoded acceleration error **3.43e-7 m/s2**. All observations
  finite and within the recorded normalized range. Runtime source hashes match
  the final run's manifest. This rules out export/action-decoding drift on these
  states, not every possible inference defect.
- Audit computation used about **2.9 seconds** in the shared cost ledger, zero
  simulator episodes and zero optimizer updates. Documentation time is separate.
  To reproduce with the retained local inputs, run
  `python scripts/audit_highway_policy_v03.py` in the traffic environment. It
  refuses to overwrite an existing audit directory; preserve existing evidence.

## What the main car actually does

Times below are pre-action state times; the corresponding resulting state is
0.1 seconds later. A stopped sample means speed below 0.1 m/s.

| Observation | Measured result | Meaning |
| --- | ---: | --- |
| Fleet interventions | 746 / 17,230 = 4.33% | Decision-weighted across all six cases and cars |
| I-110 focal interventions | 180 / 549 = 32.79% | The headline fleet figure hides this car's problem |
| Lane-end cause | 173 affected focal decisions | Dominant intervention; causes can overlap |
| Other focal causes | 5 speed ceiling, 3 native lane refusals, 2 native realization | These counts are not disjoint |
| Policy filter changed acceleration | 178 decisions | Our filter is the main direct longitudinal constraint |
| Native acceleration realization changed | 2 decisions | Generic SUMO speed suppression is not the dominant cause here |
| First left permission | 26.4 s, ego speed 0.407 m/s | No permitted left action before near-stop |
| Permitted/requested left changes | 4 / 4 | No observed ignored left opportunity |
| Stopped interval | 26.5-40.0 s, 136 samples = 13.6 s | Completion does not mean a smooth merge |
| Target reward eligible | 205 / 549 = 37.34% | Most decisions get no speed-target term |
| Eligible while still in ending/unreachable lane | 55 / 399 = 13.78% | Very sparse guidance during the critical phase |

Before the first permitted left action, the mask rejects the current rear gap on
210 decisions and the front gap on 18; reasons can overlap. These are actual gap
constraints, not evidence that masks should simply be disabled. The focal car
already receives nearly six peer messages on average during this interval.

The local legal limit is **13.41 m/s (30 mph) on the ramp**, then **18 m/s
(40.3 mph)** in this I-110 test. The global 60 mph ceiling does not make this a
60 mph merge test. Network metadata saying 60 mph is not the effective local
vehicle/lane limit; the trace records the latter.

## Rewards: actual values and coverage

The [resolved config](../configs/training/highway-flow-rl-v03-long.json) has the
following active terms. Positive weights are shown with their reward sign.
Totals are undiscounted contributions for the same 549-decision focal episode,
not comparable coefficients or isolated causal effects.

| Term | Current calculation / weight | Focal total |
| --- | --- | ---: |
| Progress | +1.0 * distance / 2.68224 m per 0.1 s step | +213.147 |
| Merge | +5 on merge event | +5.000 |
| Stable merge | +2 on stability event | +2.000 |
| Arrival | +2 on arrival | +2.000 |
| Passing | +2 per accepted pass | 0 |
| Cooperation | +0.1 * team merges / controlled population | +0.0125 |
| Intervention | -0.2 per overridden decision | -36.000 |
| Unsafe acceleration request | -0.5 * positive excess over safe ceiling / 4.5 | -17.581 |
| Safe-speed error | -0.30 * capped normalized error; conditional eligibility | -16.163 |
| Jerk | -0.09 * Huber(abs(realized jerk) / 2); uncapped | -8.807 |
| Lane change | -0.1 on actual change | -0.100 |
| Optional ending-lane detour | -2, one-shot event | 0 |
| Unnecessary reversal | -0.5 on event | 0 |
| Collision | -20 on collision | 0 |

The old absolute-jerk weight and feasible-acceleration reward are zero. No
imitation or near-limit teacher loss is active. The Huber function is `0.5*x*x`
up to x=1, then `x-0.5`. The speed term is
`-0.30 * min(1, max(0, abs(v_next-v_target)-0.5) / max(v_target,5))`.
It is zero when target evidence is ineligible or the next speed is unavailable.

The key defect is **coverage**, not just 0.09 versus 1.0. Of 344 ineligible focal
decisions, 340 report `no_current_admissible_merge_gap`, two have unknown target
lane evidence, and two have unresolved neighbor lateral motion. The target
helper intersects front and rear safe-speed intervals projected three seconds
ahead. If that intersection is empty, it discards the entire speed objective,
even when present information could support an earlier gap-formation maneuver.
An impossible immediate merge and genuinely missing information need distinct
handling. Safe stopping constraints remain required in both cases.

During the 3.5-22.5 s gap-formation phase, the focal car receives **+124.11
progress**, **-1.96 speed error**, and **-1.70 jerk**; the target is eligible on
only 12 of 190 decisions. Progress rewards motion but does not describe which
neighbor to merge behind. Arrival/merge bonuses are late. For a fixed completed
route, raw progress is largely fixed; discounting affects its time preference.
Episode sums alone therefore do not justify blindly shrinking progress or
increasing every braking penalty. A smooth early yield may be the right action.

## Safety, masks and native execution

The longitudinal guard in [highway_stage1_env.py](../environments/highway_stage1_env.py)
uses 2.5 m minimum gap, 1.5 s following headway, a 60 mph/global-and-local speed
bound and a lane-end stopping envelope with 4.5 m/s2 braking. Each 0.1 s command
is clipped to the minimum bound and the physical range [-4.5,+2] m/s2. A car
that fails to prepare a merge can reach this hard fallback while still moving
fast. Keep the fallback; change the behavior before it is needed.

The inherited lane mask checks current front/rear gaps, route connectivity,
internal segments and a 5 s cooldown. V05 adds target-lane runway and a committed
maneuver lock. Its current-gap floor uses 0.1 s reaction, 1.5 s headway and
4.5/9 m/s2 follower/leader braking. The reward's projected-gap target instead
uses 0.3 s reaction and a 3 s projection. This is not one shared feasibility
model. Native physical lane movement lasts 3 s; the TraCI request lifetime is
3.1 s, and no-motion requests become refusals after about 3.2 s.

At the first permitted request, rear gap is 41.13 m and closing speed 16.19 m/s.
A constant-speed three-second projection gives **-7.44 m**, despite the
instantaneous mask permitting a request. Native refusal is consistent with a
future-gap problem, but this projection is not a native refusal reason code or
a collision prediction under actual braking. Three requests are refused before
the merge proceeds. Audit a shared, conservative full-maneuver feasibility model
and log native reasons where available; do not just loosen the gap check.

Speed mode **31** and lane-change mode **512** preserve SUMO safety constraints;
512 disables autonomous lane selection while retaining collision and safety-gap
checks. Consequently the learner must prepare and request the merge itself.
These are documented behaviors, not a hidden training controller.
[SUMO command documentation](https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html#lane-change-mode-0xb6).

Peak jerk has several sources: about -31.79 m/s3 when the lane-end guard begins
hard braking, +40.71 m/s3 when deceleration ends at rest, -17.65/+16.42 m/s3
around the ramp speed-limit transition, and +13.94 m/s3 on initial engagement.
The stopped-state peak must remain in the uncensored comfort metric. Diagnose
initial engagement, local limit changes, hard-stop recovery and ordinary
oscillation separately; a global mean or p95 hides their different causes.

## Training and model variables

| Area | Current setting / measured evidence | Assessment and next check |
| --- | --- | --- |
| Training budget | 298,400 learned decisions; 300,000 charged including failed rollout; 177 updates / 7,130 optimizer steps; about 29 min | Longer run completed; duration alone is not the missing objective |
| Role balance | 31,366 merge-car samples = 10.51%; I-110 merging cars 7,156 = 2.40% of all samples | Balance merging roles as well as scenario names; background cruising dilutes exposure |
| Episode coverage | 103/177 rollouts truncate and close/reset the environment | Bootstrapping is present, but unseen later states remain unseen. Continue episodes across chunks or budget by simulated time/completion |
| Population-dependent horizon | At 2,048 active-agent decisions, 16 cars get 12.8 s; 8 get 25.6 s; 4 get 51.2 s while all remain active | More AVs shorten experience per vehicle; avoid this interaction with the AV slider |
| Scenario coverage | 12 rows, 2-5 through lanes, cruise/follow/pass/merge; roughly 14-15 episodes per row | This is not thousands of independently varied scenarios |
| Discount and GAE | gamma .99, lambda .95 at 10 Hz, currently defaults in GAE | Reward 30 s later is weighted .049; direct GAE trace weight at 10 s is .00217. Critic bootstrapping can carry longer credit, so this is not a hard cutoff |
| Optimizer | Actor LR 1e-4, critic 3e-4, Adam epsilon 1e-5, batch 256 | Preserve baseline and change one factor at a time |
| PPO reuse | 8 epochs maximum, clip .2, target KL .02 | 106/177 updates complete eight epochs, 71 stop earlier; 11 have zero full epochs but may have partial updates. Do not disable the guard to force eight |
| Losses | Value coefficient .5, value clip .2, entropy .001; no imitation, no critic warmup | Small ongoing exploration after warm-start; assess with task-specific diagnostics |
| Gradient clipping | Joint actor/critic norm cap .5; every one of 7,130 steps clipped; median multiplier .094 | Coupled clipping is a tuning concern, not proof that learning is broken. Critic norm exceeds actor only 26.47% of steps; Adam updates are not simply scaled by this ratio |
| Critic fit | Median post-update explained variance .895; 5th percentile .020 | Usually fits on-policy returns; some batches remain difficult. Fit does not establish correct targets or long-horizon planning |
| Continuous exploration | Final latent Gaussian std .09797; deterministic evaluation | Latent std is not acceleration std after squashing. Compare exploration only after coverage fixes |
| Actor | Shared feedforward MAPPO; 69 local inputs + six 8-field message slots = 117; message encoder 16, masked mean/count; local MLP 128/128 | No recurrent state, explicit trajectory or agreed-gap plan |
| Critic | 24 * (117 observations + presence) = 2,832 input, masked population representation; value MLP 256/256 | Training-only centralized critic, not the deployed actor; population padding is not proof of every population being trained |
| Action decoding | Acceleration = -1.25 + 3.25*u for u in [-1,1]; lane intent -1/0/+1 | u=0 means -1.25 m/s2, not coast; mapping is internally consistent and replay verified, so no demonstrated mapping bug |
| Sensors and state | Simulator-derived structured ego, neighbors, route/map, previous request, actual acceleration and delivered V2V | Camera/LiDAR estimates, real sensor errors and physical-car control are outside this training run |
| Communication | Up to six peers within 150 m, 0.5 s freshness, one-tick delivery delay; configured loss 0 or .1, plus no-message rows | Contains states/previous intent, not negotiated reservations. Same-state message zeroing changes acceleration but no lane choice here; not a closed-loop benefit test |
| Population variation | Training controlled counts 1,4,8,12,16; total traffic 8,16,24; fixed initial cohorts | No within-episode slider changes, no proof of arbitrary AV ratios or 24 simultaneous learned cars |
| Demand and background | Deterministic seeded initial traffic; native cautious/balanced/attentive profiles with tau 2/1.75/1.5 s, accel 1.6/2/2.2, speedFactor .94/.97/1; speedDev/sigma zero | Neighbor speeds and profiles materially affect merge feasibility. No sudden obstacle or continuous-flow validation claim |
| Geometry and routing | Explicit route continuation, target-lane context, finite acceleration lane; engineering network, synthetic 100 m visibility | Effective speeds/runway matter more than scenario title. Shared road geometry across splits is not unseen-geometry validation |
| Initialization/recovery | Warm-started v03 actor/critic; fresh optimizer for long round; optimizer/RNG/counters retained on documented resume | No detected accidental reset. One changed short-ramp validation fixture was re-baselined; five were unchanged |
| Snapshot selection | Final exported checkpoint evaluated; rolling last.pt during training | Save periodic candidate snapshots and evaluate automatically; a final checkpoint need not be the best |
| Acceptance | Six development cases, no new held-out evaluation, zero collisions/teleports under guards; all six fail case gates | Zero protected collisions does not establish unshielded safety or deployment readiness |

Raw update-by-update statistics and all 12 per-case exposure counts are in the
compact evidence. The MAPPO paper independently motivates testing implementation
and hyperparameter choices; it does not establish an optimal epoch count or
reward scale for this driving task.
[Primary MAPPO study](https://arxiv.org/abs/2103.01955).

## Measurement gaps

The 60 mph cruise case contributes **6,977/17,230 = 40.49%** of evaluated
decisions and zero overrides. Report per-car/per-task distributions beside the
fleet figure. Do not use the pooled 5% result as a substitute for the main-car
gate.

"Stable merge" checks completion and eight seconds of post-merge behavior; it
does not exclude the 13.6 s stop before merging. Add feasible-merge delay,
pre-merge stop duration, speed recovery, downstream braking/queue impact and
matched trip-time metrics. Retain legitimate yielding and emergency braking;
do not label every slowdown unnecessary. Keep uncensored peak jerk, safety
interventions, refusals and failed episodes visible.

## Recommended correction order — proposed, not implemented

1. **Correct the gap-formation objective before retuning scalar weights.**
   Separate missing evidence from no immediate merge opening. Choose a feasible
   front/rear traffic slot and evaluate gentle early acceleration/yielding over
   the remaining usable lane distance. Reward reduction of a bounded,
   physically justified gap/timing error without paying for waiting or oscillating.
   Retain safe stopping when a merge is genuinely unavailable. Validate rankings
   on saved blocked, clear, emergency and speed-limit traces first; static
   re-scoring cannot prove the resulting closed-loop behavior.
2. **Fix experience coverage.** Keep unfinished episodes alive across rollout
   chunks or explicitly allocate full physical-time horizons. Increase merging
   car representation while preserving cruise, follow and pass cases. Report
   late-merge states actually learned, not just total samples. Compare balanced
   episodes without invalidating PPO's on-policy probabilities.
3. **Align mask, reward and execution timing.** Use compatible gap assumptions
   over the full three-second maneuver, preserve fallback guards, and expose
   usable gap/closing-time context consistently. Test difficult front/rear gaps,
   connector transitions and native refusals before another full run.
4. **Then tune smoothness and temporal credit in controlled comparisons.**
   Keep current progress 1.0, speed .30 and Huber .09 as the attribution baseline
   while fixing coverage. Do not claim a new coefficient is optimal from one
   trace. Predeclare a small sweep only after objective normalization is defined;
   evaluate legal-speed progress, early gap formation and uncensored jerk
   together. Compare gamma/lambda in seconds of useful credit, not defaults
   copied across step rates. Separate clipping and exploration are secondary
   ablations, not simultaneous unmeasured changes.
5. **Gate on useful driving and keep compute/assistant usage low.** Run a small
   matched development pilot, then automatic periodic evaluations/checkpoint
   retention with fixed budgets and failure criteria. Inspect only terminal
   results or failure. Keep held-out seeds untouched until development gates
   pass; test varied AV proportions and message failures separately. Present
   the final resolved training configuration before launching a new run.

No runtime reward weights, masks, safety limits, training configuration or
policy weights changed in this audit. Documentation and reproducible analysis
are the only changes. Google engineering-guide synchronization remains pending;
its generated Markdown mirror was not edited.
