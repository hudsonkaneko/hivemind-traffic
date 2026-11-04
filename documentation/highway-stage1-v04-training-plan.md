# Stage 1 v04 training improvement plan

November 4, 2026, America/Los_Angeles. **Proposed; implementation and training have not started.**
Parent: [living sensor-driven roadmap](sensor-driven-training-roadmap.md).
Evidence: [v03 training result](highway-stage1-training-v03.md).

## Objective and measured starting point

Reduce persistent acceleration corrections during following and passing without losing cruise speed, merge completion or passing ability. Keep the 93-field actor, 376-field centralized critic, four controlled cars, exact SUMO/map observations, delivered messages, rewards, 60 mph ceiling and existing 1.5 s following/stopping protections.

V03 selected validation achieved 6/6 merges and 12 passes, but 14.97% pooled and 51.47% worst-case overrides; five of twelve cases failed. Its earlier phase-0 checkpoint had 11.34% pooled and 32.625% worst-case overrides, with six failing cases. Better supervised acceleration fit did not establish better driving: after BC, overrides rose to 29.02%.

Hypothesis: retaining corrections from the actor's own difficult training states will reduce recurring errors more effectively than increasing BC epochs. The runner already computes teacher labels on every current PPO rollout. The proposed change retains past training examples for an auxiliary supervised loss; it is not the introduction of teacher supervision.

## 1. Diagnose before implementing the learning change

Use retained phase-0 and final journals to compare requested, filtered and realized acceleration, teacher/feasible targets, gaps, relative speeds, lane changes and mismatch duration. Prioritize five-lane following and four-lane passing, which regressed, while reviewing all twelve cases.

Report analytical limit interventions, rejected lane changes and measured native acceleration differences separately, alongside the unchanged combined override count. Native differences are not automatically emergencies; leave their internal causes unknown when the logs do not establish them. Do not lower tolerances, remove difficult cases or relabel differences out of the acceptance metric.

Use validation records only for diagnosis and reporting. They must never enter the correction buffer, optimizer, normalization statistics or teacher-fitting data. If timing, labels or the diagnostic target are inconsistent, resolve that and review the plan before learning.

Deliverables: reproducible per-case audit, representative acceleration/gap plots, and a short evidence-backed hypothesis.

## 2. Prepare one narrow comparison

Both arms start from the same retained phase-0 full model:

- Run: outputs/highway-stage1/v03/train-20261103T230506Z-04ba1761.
- Checkpoint: selected-00036-d97376b9/best.pt.
- Checkpoint SHA-256: b77f55ae7262178ce0fa15729fe844e18693cd168905eea8b1bbd579c6913ab1.
- Actor SHA-256: 8526f6134b7eae90d3370bd12cc0487926a6ec4356c6c6ea2dd231e8aa8b1bb8.

Implement an explicit audited warm-start importer for model weights, critic and exploration log standard deviation. This is a new, separately budgeted experiment, not resume of the completed v03 attempt. Reset optimizer state, RNG and new-run counters identically for A and B; preserve parent provenance. Do not patch old checkpoints. The current runner does not yet support this mode.

| Arm | Acceleration imitation | Lane imitation |
| --- | --- | --- |
| A: comparison baseline | Current rollout only | Current rollout, unchanged |
| B: correction memory | 50% current rollout loss + 50% retained-example loss | Current rollout, unchanged |

Keep the total auxiliary coefficient, PPO update schedule and all other settings identical. Use the v03 decay with its historical 99,998-PPO-decision offset: approximately 0.04 at the new start and 0.03 after 100k new decisions, with the existing 0.01 floor. Resetting the new budget counter must not silently restart that schedule. No fresh BC stage is proposed.

### Correction memory proposal

- Start empty; capacity 20,000 examples, with 5,000 slots per task: cruise, following, passing and merge.
- Within each task, reserve half for flagged errors and half for uniformly sampled remaining training examples. Flag an override or an absolute requested-versus-teacher acceleration difference above 0.05 m/s². This sampling rule does not alter evaluation thresholds.
- Use deterministic reservoir sampling, unique sample IDs and explicit underfilled-stratum fallback. Log the actual task/error mix.
- Admit only complete new training transitions, storing actor observation, teacher label, task/layout/seed, action diagnostics, policy version and source hashes. Validation/test flags must fail admission.
- Add a rollout to memory after its PPO update. Historical samples contribute only to the supervised acceleration term, never PPO probability ratios, advantages, critic targets or value updates.
- Use 256 retained examples per PPO minibatch when at least 1,000 prior examples exist; before then, use current-rollout imitation. Log when retention begins.
- Save buffer contents, seen-counts, RNG and cursor atomically with resume state. Test exact recovery and sample provenance.

This is a bounded adaptation of dataset aggregation, not a claim that our teacher or implementation inherits DAgger's guarantees. [DAgger research](https://proceedings.mlr.press/v15/ross11a.html) motivates learning on states induced by the policy. Keeping historical data outside the PPO objective respects its [on-policy design](https://spinningup.openai.com/en/latest/algorithms/ppo.html).

## 3. Proposed screening budget and configuration

| Setting | Both arms |
| --- | --- |
| New training allocation | Up to 100,000 active-agent PPO decisions per arm |
| Wall cap | 30 minutes per arm, including validation and recovery |
| Total first comparison | Up to 200,000 new training decisions / 60 minutes; sequential CPU runs |
| Preparation cap | Separately bounded no-learning preflight, up to 15 minutes |
| Device | Existing traffic environment; CPU, one worker, one Torch thread |
| Training seed | 9206, paired between A and B; separate deterministic sampler RNG |
| Environment seeds | Fixed manifest in training-only range 6102000–6102999; no validation/test seeds |
| Task quotas | 25k cruise, 25k following, 25k passing, 25k merging, interleaved |
| Roads/demand | Existing training layouts including varied lane counts and I-110; light/dense merge coverage |
| PPO actor / critic LR | 0.0001 / 0.0003 |
| PPO epochs / batch / clip / KL | 4 / 256 / 0.2 / 0.02 |
| Fresh demonstration collection / BC | None |
| Evaluation | Existing twelve validation cases at start, 50k charged-budget boundary and quota exhaustion |

Use the exact shared training-case order and seed manifest for both arms, with counters based on completed active-agent decisions and conservative charging of failures. Small unusable quota remainders may remain; record them. Evaluation decisions are recorded separately from the training allocation, while their time counts against the wall cap.

Resolve and hash the two runnable configurations only after implementation, tests and preflight. Present those final configurations before launch; this proposed table is not executable configuration or launch approval.

## 4. Evaluate progress without hiding regressions

Keep the original v03 selections immutable. For new runs, predeclare selection order: safety events and existing driving-task failures first, then worst-case override rate, pooled override rate, failing-case count and efficiency tie-breaks. Incomplete horizons, invalid trip ledgers or nonfinite/missing metrics remain disqualifying.

Compare A and B at corresponding predetermined allocation milestones: the 50k charged-budget boundary and exhaustion of the task quotas. Actual completed decisions can differ because of unusable remainders or conservatively charged failures; report actual and charged counts separately. The primary screening comparison is the final common completed milestone, not whichever historical checkpoint makes B look best. If either arm times out, report the last shared completed milestone and the incomplete endpoint; do not extend automatically.

Proposed screening success requires all of the following:

- B reduces worst-case overrides by at least 20% relative to matched A, and does not exceed the starting checkpoint's 32.625% worst-case rate.
- B's pooled rate and failing-case count do not exceed either A or the starting checkpoint (11.344% pooled, six failing cases).
- No collisions, teleports or overspeed; preserve 6/6 merges, at least one pass in every passing case and all existing cruise-speed gates.
- No case loses more than 0.02 in speed/limit ratio relative to either A or the frozen start. Per-case complete-cohort boundary time and external wait must not worsen by more than 2% relative to either comparison; use absolute 0.1-second numerical tolerance when a reference is zero.
- Report all per-case overrides, speeds, pass counts, lane changes, jerk, headway/TTC where available, unfinished trips and complete cohort burden.

Bind all baseline comparisons to the exact frozen phase-0 report, not rounded prose: pooled overrides are 4,352/38,363 = 0.11344264004379219; worst-case rate is 0.32625. Baseline report SHA-256: f06a005166c157dd928bd4460655deed398e0b4458692a6ec02fabd0f9c947e2. Read per-case speed/cohort references from that same report.

The 20% improvement and efficiency tolerances are proposed experiment decisions to freeze before launch, not previously validated safety thresholds. Passing this screen justifies a repeated-seed study; it does not accept the policy. Full nominal acceptance still requires the existing driving gates and no more than 5% overrides in every validation case, followed by a frozen held-out evaluation.

## 5. Low-usage execution and recovery

The runner handles logging, predetermined validation, hashing and checkpoints. The chat waits for process exit without reading learning curves or reasoning during training. No extra agent runs, repeated status reports or live documentation work while it learns.

Before launching, add bounded handling for transient Windows checkpoint replacement failures: unique temporary files, preservation of the previous valid checkpoint and a short retry limit. Test both transient and persistent failures; never turn retries into an unbounded loop.

After a failure, inspect once, retain the failed package and resume only from a verified compatible checkpoint within the remaining arm budget. Changes to runtime/configuration require versioned provenance and fresh readiness; never bypass fingerprint checks. No silent extra attempts or budget resets.

After both arms finish, retain the comparison, per-case evidence and all failure records. Publish any new frozen actor with an experimental label and update the living roadmap, changelog, Google guide and Markdown mirror.

## 6. Decision after the screen

If B passes the screening conditions, propose a fixed repeated-seed budget before scaling; do not automatically launch it. If it fails, use the retained errors to choose one subsequent change, such as loss balance or a bounded correction around the scripted controller. Do not simultaneously retune reward, architecture and curriculum.

Keep the held-out partition untouched during tuning. After nominal validation acceptance, freeze the policy and evaluate the held-out road/seed matrix once under a declared protocol. Sensor uncertainty and physical camera/LiDAR integration remain later roadmap gates.

## Preparation checklist

- [ ] Complete saved-journal diagnosis and freeze the hypothesis.
- [ ] Implement warm-start import, correction memory and versioned selection.
- [ ] Verify leakage rejection, fresh PPO data, memory loss scaling and deterministic recovery.
- [ ] Verify failure-safe checkpoint writes and run bounded no-learning preflight.
- [ ] Present exact A/B configs, hashes, budgets and comparison rules.
- [ ] Obtain explicit training launch instruction.
- [ ] Run the approved pair, then report results and update project memory.

Only this plan and its documentation links were created for this milestone; no learner, runnable v04 config, new weights or simulator run is claimed.
