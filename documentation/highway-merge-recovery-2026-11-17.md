# November 17: merge recovery and verified cut-in training

User approved fixing the merge regression, ensuring real cut-ins, and retraining
with completion/failure-only review. Branch: `2026-11-17-codex-merge-recovery`.
This preparation checkpoint does not claim a successful learned policy.

## Diagnosis and changes

The retained v07 candidate improved I-110 but slowed the corrected-road cases.
Its merge-speed cost saturates at a 2 m/s error, providing no further distinction
between, for example, 5 and 6 m/s error. Offline rescoring shows the new signal
penalizes the slower candidate more on each of the three regressed ramp cases.
This is objective qualification, not evidence of successful retraining.

- Replace the saturated merge tracking term with
  `-0.40 * min(2, abs(next_speed - safe_target)/8)`.
- Add merge acceleration alignment:
  `-0.35 * min(4, (raw_request - desired_acceleration)^2)`;
  desired acceleration is `(safe_target - current_speed)/2`, bounded to
  [-2, +2] m/s2 and then limited by the current safety ceiling. It uses the
  existing pre-action local target, never future traffic or privileged actor
  inputs. It changes rewards, not applied vehicle commands.
- Add `-0.50 * min(1, max(0, raw_request-speed_ceiling)/0.2)` to discourage
  small positive requests that repeatedly hit the legal-speed guard.
- The excess-jerk reward now uses consecutive raw commands and realized jerk.
  Previously it called `(request - measured_acceleration)/dt` raw jerk, which
  confounds the startup actuator lag with command changes. No first-command
  difference is invented. The old every-decision metric and original strict
  gate remain visible and unchanged; the new command-slew metric is separate.
  V08's actual reward audit supersedes the intermediate v06 term accounting.
- Other audited rewards, safety guards, 4 m/s3 routine conditioning, road
  geometry, 117 actor inputs, actions and nominal acceptance rules stay intact.

Cut-in recovery no longer waits for a chance encounter. A seeded eight-car
mainline scene positions a native cutter in the adjacent lane, slightly faster
and approximately 0.95-1.15 s ahead. Native SUMO starts the lane change before
handoff; reset only succeeds when lateral motion and a measured 0.5-1.5 s gap
are present. Both native safety modes remain active (31/512). Vehicles are
initialized through demand files, never teleported during a running episode.
Completion into the intended lane is also required before the episode can be
accepted. Missing cut-ins fail explicitly rather than silently count as stress
training. These scenes train recovery from a cut-in already starting; they do
not qualify anticipation of a previously hidden vehicle. Nominal scenarios
retain v07's admission layer. Stress safety failures remain reported separately.

## Resolved training configuration

[Configuration](../configs/training/highway-merge-recovery-2026-11-17.json):
round 8, v04 best warm-start, fresh optimizer, seed 10111, 20,000 critic-only
decisions inside the 300,000-decision cap, 5,400-second round cap. Existing
shared study ledger/lock and 4.93M / 46,800-second study ceilings retained.
No reset of prior charges. No held-out test or automatic additional round.

MAPPO: four complete episodes per actor update, actor/critic LR 3e-5/3e-4,
20 maximum epochs, minibatch 256, clip 0.2, target KL 0.02, entropy 0.001,
gamma/lambda 0.99/0.95. The inherited actor's latent Gaussian std (~0.04) is
initialized to 0.12 for exploration; deterministic actor means are unchanged.
This addresses the documented very small exploration and KL-limited updates.
The critic and actor architectures are unchanged.

Actual-decision curriculum targets remain 30% normal merge, 20% missed gap,
10% verified cut-in recovery, 25% cruise/follow, 15% passing. Cut-in scenes are
now explicitly mainline recovery tasks on the same frozen highway networks.
The initial and trained policies see identical seeded validation conditions.
Collisions, missing cut-in exposure, periodic merge regression against the
matched initial nominal baseline, or pooled interventions over 5% stop the run.

## Verification before launch

- 151 targeted tests pass. Cut-in demand is byte-repeatable across 12 sampled
  seed/layout combinations; all retain eight vehicles and a real adjacent pair.
- Zero-update preflight passes in 3.0 charged seconds, including a measured
  cut-in starting at 0.2 s, 1.114 s headway, and completed native lane change.
- Three additional 64-decision native checks on 2-, 5- and 3-lane layouts all
  start and complete the intended cut-in, with zero collisions/teleports.
  Handoff headways: 1.114, 1.119 and 1.022 s. These are bounded qualification
  checks, not complete-route policy acceptance.
- [Saved-trace reward audit](evidence/highway-merge-recovery-2026-11-17/offline-reward-audit.json)
  retains source hashes and per-case costs. Native evidence is under
  `outputs/highway-flow-rl/v08/preflight-01` and `cutin-qualification-01`.

The launcher retains logs, source-bound preflight, full checkpoints, raw episode
metrics and failures. It waits for process completion without interactive
progress inspection. Technical documentation is updated in GitHub; the Google
engineering-guide synchronization is pending, and its generated mirror is
unchanged.

## Completed retrain: cut-in coverage fixed, merge regression remains

Run `outputs/highway-flow-rl/v08/train-20261117-01/train` completed on source
commit `0efc28a`, using 3,751.58 charged seconds (62.5 minutes) and 298,361
decisions. This includes 23,119 critic-only decisions. It stopped because the
remaining decision allowance could not reserve another complete episode.
There were 233 episodes, 54 actual four-episode actor updates and 21,840 actor
optimizer minibatch steps; the historical `updates=233` counter counts episodes,
not actor batches. The last two buffered episodes remained charged but did not
form another four-episode update. No interactive progress inspection occurred.

**Not accepted or promoted. The merge regression is not solved.** The final
actor and all metrics are retained, and the prior deployed policy is untouched.
The safety-first checkpoint ranking retained the initial actor as `best-policy.pt`;
the final experimental actor is `policy.pt`. A deterministic probe verified the
final export against `last.pt`. Hashes are in the evidence JSON.

| Nominal validation metric | Initial | About 100k | About 200k | Final |
| --- | ---: | ---: | ---: | ---: |
| Merge-speed matching | 43.52% | 51.44% | 47.70% | 39.36% |
| Stable merges | 4/4 | 4/4 | 4/4 | 4/4 |
| I-110 merge-speed matching | 52.80% | 89.44% | 45.96% | 12.42% |
| Original strict case gates | 1/8 | 1/8 | 1/8 | 0/8 |
| Peak realized jerk, m/s3 | 4.76 | 4.00 | 30.02 | 28.38 |

Final nominal pooled overrides were 0.448% versus 0.265% initially; the low
pooled value does not erase per-car comfort and merge-quality failures.
The training 2-lane ramp's match improved from 45.39% to 72.67%, while the
validation 2-lane ramp fell from 39.01% to 35.46% and the 4-lane ramp remained
36.88%. This is uneven learning, not a general merge improvement. Both periodic
checks stayed above the initial aggregate merge floor. The final evaluation
fell below it after the last periodic check; the existing stop applies to
periodic checks only. The final summary correctly says
`training_acceptance=experimental_not_promoted` and `quality_accepted=false`.
No acceptance rule was relaxed and no extra round was launched.

Cut-ins actually started and completed in **34/34 training stress episodes**
and **2/2 stress cases at each of the four validation evaluations**. Cut-in
recovery supplied 10.08% of PPO-phase collected decisions. Across all 233
training episodes and all validation cases: zero collisions and teleports.
The final stress overrides were 1.875%, essentially unchanged from 1.874%;
stress peak jerk remained 45.0 m/s3 and both stress cases failed the original
comfort/raw-request checks. Exposure is fixed; smooth recovery is unproven.

Retained review artifacts:

- [Result, checkpoint hashes and all evaluation summaries](evidence/highway-merge-recovery-2026-11-17/result.json).
- [Paired per-case measurements](evidence/highway-merge-recovery-2026-11-17/cases.csv).
- [Per-episode optimizer and intervention curve](evidence/highway-merge-recovery-2026-11-17/learning-curve.csv).
- Full checkpoints, resolved config, raw actions and evaluation files in the
  run directory above; runtime logs in `logs/train-20261117-01-*.log`.
  These bulk artifacts remain local, not in GitHub.

The next revision should preserve every evaluated checkpoint, apply the merge
comparison at final evaluation too, and constrain per-scenario regressions so a
gain on one ramp cannot hide deterioration on I-110. Diagnose the retained
I-110 requested/applied actions before changing weights again. These are next
steps, not completed fixes or authorization for another run. No held-out or 3D
claims. Google engineering-guide synchronization remains pending.
