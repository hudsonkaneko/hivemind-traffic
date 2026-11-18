# Highway flow RL tuning history

What each highway flow round changed and what it showed, as recorded in the
repository's plan documents and evidence (November 2026). Use it to avoid
repeating a change that already failed, and to estimate what a lever can do at
this budget. The numbers are from single validation seeds per case; read them
as directions, not precise effects.

Gate counts use the 8 development validation cases unless stated otherwise.
"Raw check" is the raw-request jerk gate after the 5-decision insertion lag.

## Rounds

| Round | Change | Result | Lesson |
| --- | --- | --- | --- |
| v03 round 1 (30k decisions, 6 cases) | Speed-error and Huber smoothness rewards, ending-lane detour penalty | I-110 focal braking and detours fixed; pooled overrides 4.74% to 10.21%, peak jerk up, stable merges 2 to 1 of 2, 0 of 6 cases | A term can pass its own metric while acceptance regresses. Always score every gate |
| v04 long run (3.64M decisions, 7.8 h, 6 cases) | Future gap preparation, one merge-speed objective, projected-gap lane mask | I-110 interventions 31% to 0%, stopped time 11.7 s to 0; still 0 of 6 full cases (peak jerk, merge quality) | Long PPO fixes gross failures, not strict quality gates |
| 5 (v06, 290k) | Whole-episode rate limit, jerk cost, one episode per update, `target_kl` 0.02 | 4 to 1 of 8 (original gates); KL stopped 98 of 99 updates, usually in the first epoch; speed-ceiling trims 3 to 1,644 | Read the update records. A per-step cost (-0.2 per intervention) lost to drift. The raw jerk check mostly measured insertion |
| 6 (index used by another session's v07 cut-in run) | Cut-in coverage | Stopped by its merge-regression rule at about 100k | Check the ledger for charged round indexes before declaring a round |
| 7 (v06, 100k) | Four complete episodes per update; raw check from the second decision | Stopped by the merge floor (0.343 against 0.4167): one ramp car's post-merge window failed on 40 speed-limit trims | One car can trip a pooled floor. Look per car |
| 8 (index used by another session's v08 retrain, 298k) | Cut-in recovery retrain | I-110 match 52.8% to 89.4% at 100k, then 12.4% at the end; 1 to 0 of 8 | The best point can be mid-run. Validate periodically and keep the best |
| 9 (v06, 158k) | Final-round profile: 4 episodes per update, 10 epochs, `target_kl` 0.1 | Updates train (11 of 13 ran all epochs, 4,608 steps); merge match 0.437 to 0.454; 3 of 8 with the raw check, unchanged | Healthy updates still barely move merge matching. Progress is about 97% of the per-decision signal; tracking saturates at 2 m/s |
| 11 (v09, 100k) | Unsaturated tracking, graded quality payment, no raw jerk charge during insertion lag | Stopped by the merge floor (0.211); the I-110 car merged 6.1 s later with 32 of 81 post-merge overrides | Reward changes moved speed requests, not the timing of the merge |
| 12 (v10, 100k) | Adds -1.0 per post-merge override | Stopped by the merge floor (0.240); matching on stable cars +0.000; failures from tiny requests clamped at the speed limit | Same. Pricing a filter action does not stop the policy from triggering it |
| 13 (v11, evaluation only) | Merge-speed dwell mask rule, 10 s release | The I-110 car missed its only gap and stopped at the lane end | Check the gap timeline before setting a release distance |
| 14 (v11, evaluation only) | Same rule, 15 s release | 3 to 5 of 8 with no training | A mask rule that only removes permissions fixed the timing cause |
| 15 (v11, 197k) | PPO under the rule plus a shield-consistency loss (weight 50) | 5 to 3 of 8 at 100k, 2 of 8 at the end; 13 of 16 updates stopped on KL (up to 0.35); cars reached lane ends | At this scale PPO changes far more than the term it is given |
| 16 (v11, 112k) | Supervised projection of the round-9 actor onto the filter band, 2 passes, no PPO | 4 of 8 after pass 1, **7 of 8** after pass 2; interventions 0.06%, peak jerk 4.8, no collisions | Fitting to a computable band works. One pass is not enough |
| Probes 01 to 07 | Scripted law on the I-110 merge car only | Qualifies (0.994) when waiting 0.7 to 0.9 m/s under the target, then matching it; another car then failed a jerk limit (6.12) | The last gate is reachable. Probes are cheaper than training |
| 17 (v11, 194k) | Projection plus scripted teacher labels on pending merges | 5, 6 and 6 of 8; label fit stalled at 0.12 to 0.16 m/s² RMS | Labels do not transfer when the actor cannot observe what the label depends on (the gate's target speed) |

## Patterns across rounds

- **Diagnosis came from journals.** The insertion-jerk finding (round 5), the
  saturated tracking term (round 9) and the merge-timing cause (v11) were all
  found by reading saved per-decision journals, not by running more training.
- **Rewards moved continuous requests, not discrete timing.** Rounds 11 and 12
  changed merge rewards; the merge moment did not improve. A mask rule did.
- **Filters and the policy fight at the edges.** Speed-ceiling clamps and
  rate-limit changes caused most intervention failures. Penalties did not
  remove them; a band-aware supervised fit did.
- **PPO at this size is noisy.** About 8 updates per 100,000 decisions with
  narrow exploration. Small tuning effects are below the noise of one seed per
  case.
- **Records of failure were kept.** Every failed round stays in the plan
  documents with its evidence, which is what made the later diagnosis possible.

## Sources in the repository

- `documentation/highway-flow-rl-v03.md`
- `documentation/highway-merge-rewards-v04.md`
- `documentation/highway-merge-training-plan-v06.md` (rounds 5 to 9)
- `documentation/highway-merge-cutin-2026-11-15.md` (round index 6)
- `documentation/highway-merge-recovery-2026-11-17.md` (round index 8)
- `documentation/highway-merge-training-plan-v09.md` (round 11)
- `documentation/highway-merge-training-plan-v10.md` (round 12)
- `documentation/highway-merge-training-plan-v11.md` (rounds 13 to 17, probes)
