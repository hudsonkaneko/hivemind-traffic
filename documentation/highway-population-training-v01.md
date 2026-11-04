# Variable autonomous-vehicle population pilot v01

November 4, 2026, America/Los_Angeles. Completed pilot; final policy regressed and is not accepted.

## Plan and scope

Train one shared MAPPO actor to drive with a variable number of controlled cars and communicating peers. This is a separate population experiment from the proposed [v04 correction-memory comparison](highway-stage1-v04-training-plan.md); that comparison remains unimplemented. The user explicitly requested a plan and training on November 4. Present the resolved pilot configuration before launching.

1. Replace fixed-four assumptions in a separate population environment. Keep the existing four-car experiments compatible. Hold total requested traffic fixed while changing which cars are controlled, record the actual fractions, and use explicit presence masks for departing agents.
2. Preserve the actor's 93 fields and acceleration/lane-intent outputs. Select at most six valid nearby messages deterministically; use masked empty slots when fewer arrive. Noncommunicating cars remain observable through local traffic state. Use a separately initialized, masked set critic during training, with no global state entering inference.
3. Warm-start only actor weights and exploration scale from the retained v03 phase-0 checkpoint. Reset the new critic, optimizers, RNG and experiment budget explicitly. Do not call this resume, reuse held-out data, replay old PPO transitions, or change the safety protections.
4. Mix 1, 2, 4, 6 and 8 controlled cars with 8, 16 and 24 total requested vehicles, subject to task validity. Vary communication independently: local delivery, packet loss and no-message episodes. Reserve intermediate controlled counts for validation. Report fleet fraction, total traffic and actual received-neighbor distributions separately.
5. Verify API/lifecycle, deterministic seeds, padding, message truncation, no-data behavior and checkpoint recovery. Run bounded no-learning SUMO episodes before optimization. Reject invalid spawn/accounting evidence rather than silently dropping cases.
6. Run one bounded CPU pilot: at most 100,000 new active-agent training decisions or 30 minutes, including scheduled validation. Use one worker and one Torch thread. Retain initial/final matched validation, per-case journals, configurations, source/runtime hashes, failures and checkpoints. Training is exploratory, not acceptance or a broad robustness study.
7. Wait for completion/failure without chat analysis or polling of learning metrics. Inspect once at exit; compatible recovery must preserve consumed budget. Publish evidence and update the living roadmap after the run.

## Starting point and unchanged boundaries

Parent full checkpoint: `outputs/highway-stage1/v03/train-20261103T230506Z-04ba1761/selected-00036-d97376b9/best.pt`.
SHA-256: `b77f55ae7262178ce0fa15729fe844e18693cd168905eea8b1bbd579c6913ab1`.
Parent actor SHA-256: `8526f6134b7eae90d3370bd12cc0487926a6ec4356c6c6ea2dd231e8aa8b1bb8`.
This parent is experimental: its old four-car validation had 11.344% pooled and 32.625% worst-case interventions. Those numbers do not serve as the baseline for newly changed traffic populations; the new matched initial evaluation does.

The actor still receives exact structured SUMO/local map observations and delivered V2V messages, not camera/LiDAR perception. SUMO owns lane centering and lateral execution. Retain the 60 mph ceiling, acceleration bounds [-4.5, 2] m/s², 1.5 s following rule, lane-gap checks, native collision protections and original intervention accounting. No very-short-gap or physical-driving claim follows from this experiment.

## Slider contract

The first supported experimental setting changes autonomous allocation at episode reset. One motion/controller owner per car remains required. An all-human setting needs no actor inference; its native baseline must be labeled separately from controlled-agent learning. Passing scenarios with a scripted slow leader must count that leader in total demand and must not be labeled 100% autonomous.

After this pilot, implement and separately test live changes: preserve position/velocity and route; transfer ownership once; clear stale messages and recurrent/controller state; distinguish instantaneous requested percentage from actual admitted percentage. A policy trained on a maximum of eight controlled cars is not yet evidence for arbitrary fleet sizes, even when its shared actor accepts larger inference batches.

## Evaluation and decision

Predeclare cases before launch and retain all results. Compare initial and final frozen actors on identical population, geometry, seeds, horizon and channel conditions. Report collisions, teleports, overspeed, combined overrides and causes, speed/limit ratio, passing, merge success relative to actual merge roles, lane changes, jerk, headway/TTC when observed, complete-cohort burden, received neighbors and message traffic. Incomplete horizons and invalid accounting cannot establish a pass.

Keep the existing driving gates: zero safety events, no more than 5% interventions per case, cruise speed ratio at least 0.9, merge success at least 0.9 and at least one pass in each passing case. Passing a small validation matrix is not held-out acceptance. Reserve test roads/seeds; no automatic second training attempt or expansion to physical sensors.

## Resolved configuration and evidence

Implemented configuration: [highway_population_v1.json](../experiments/configs/highway_population_v1.json). The tracked configuration stays disabled; the launch uses a separate authorized copy bound to successful preflight evidence.

| Setting | Resolved pilot |
| --- | --- |
| Controlled count / total requested vehicles | 1, 2, 4, 6, 8 / 8, 16, 24; invalid all-AV slow-leader tasks excluded |
| Communication | None, local with no loss, local with 30% independent packet loss |
| Message contract | Six nearest fresh received snapshots; 150 m range, 10 Hz, 0.1 s delay, 0.5 s expiry |
| Training tasks / roads | Cruise, following, passing, merging; 20 existing training cases including I-110 and 2–5 through lanes |
| Actor / centralized critic | Existing 93-input shared hybrid actor; new 8-slot × 94-field masked critic state (752 values) |
| Critic structure | Shared 93→64→64 row encoder, masked mean + focal 93 fields + count; 158→256→256→1 value head |
| Warm start | Parent actor and exploration only; fresh critic and optimizer |
| Warmup | First 4,096 active-agent decisions train critic only, included in the 100k allocation; actor hash checked unchanged |
| PPO | Actor LR 0.0001, critic LR 0.0003, four epochs, batch 256, clip 0.2, KL stop 0.02, entropy 0.001 |
| Teacher assistance | Current-rollout imitation coefficient 0.04, lane-change label weight 10; no retained memory or fresh BC |
| Budget | 100,000 active-agent decisions or 1,800 seconds total; stop learning by 1,200 seconds to reserve up to 600 seconds for final validation |
| Runtime | CPU, one worker, one Torch thread; seed 9207; train environment seeds 6500000–6500999 |
| Validation | Sixteen fixed 800-step (80 s simulated) cases, seeds 6510000–6510015, before and after learning |
| Validation populations | Counts 1, 3, 5, 7, 8; unseen intermediate counts 3/5/7, 1/24 and 8/8 fraction endpoints |
| Native baseline | Three all-human cruise cases, total cohorts 8/16/24, no actor decisions |
| Export | Fixed final endpoint, experimental label; no best-case checkpoint selection |

The demand is a fixed initial cohort with 60 m nominal within-lane row spacing and 11–13 m/s mainline starting speeds, not continuous inflow or measured I-110 demand. Ramp starts are 35/95 m before the junction (clipped to at least 5 m along the ramp), at 7–9 m/s. A route position near an edge seam moves at most 10 m to 5 m into the next edge; initial placement is identical across control assignments. Different total counts vary occupancy on the fixed road; they do not independently span every local headway distribution. The sixteen validation cases combine different roads, populations and channels and therefore provide a screening matrix, not isolated communication ablations. The 100k budget cannot cover the full factorial schedule; actual coverage is saved and must be checked before claiming exposure to a setting.

Preparation review found and corrected the population merge denominator, partial critic-update accounting, missing nonfinite-metric rejection and final-evaluation failure closure before launch. The parent actor artifact checksum was independently verified. The final 157-test targeted suite passed, including legacy behavior, absent/permuted messages, critic padding, current-state truncation bootstrap, actor export, actor-frozen warmup and bounded atomic replacement failures. Four new fault-injection tests verify conservative failed-episode charging, exact model/optimizer/RNG recovery after an interrupted update, preserved evaluation time without retraining, and rejection of configuration drift.

The first spawn matrix passed 132/170 cases; native ramp insertion and cropped-road edge seams required fixture repairs. A subsequent collector seed still failed in dense passing traffic. Lowering speed alone did not resolve it; larger initial spacing plus lower starting speeds did. All sixteen fixed validation fixtures then passed actual reset checks. The original preflight also exposed a secondary reporting exception during failed reset; failure evidence now preserves the original exception even before control statistics initialize. No vehicles were discarded, seeds silently retried or safety bounds weakened. Failed runs and intermediate checks remain under `outputs/highway-population/v01/`.

The second preflight reached the standard Parallel API test and exposed rejection of generic reset options; generic metadata is now accepted while population-changing options remain explicitly rejected. The final no-learning preflight passed in 27.453 seconds, including Parallel API/seed tests, exact repeated teacher results, finite collector batches, native zero-AV accounting and actor export. See [preparation evidence](evidence/highway-population-v01/preparation.json).

Disabled configuration SHA-256: `e99060ca8cfee87c6333649f57b61fcef7411d115a5b162a369733b5211d72ef`.
Authorized launch configuration SHA-256: `9917f0b103e97671492fc4dd0e63aaf6b73b0f17cdd2387fce67883ad545033c`.
Verified preflight summary SHA-256: `5fa4e584a90e7fdd7cf3ba3928dc42ef28d54f40a2acd1142e754890426e0bde`.
Launch configuration: `outputs/highway-population/v01/launch-20261104T225147Z/authorized-config.json`.

Training launch command (from the project root, existing traffic environment):

```text
.venv-traffic/Scripts/python.exe -m experiments.run_highway_population --mode train --config outputs/highway-population/v01/launch-20261104T225147Z/authorized-config.json --approved-config-sha256 9917f0b103e97671492fc4dd0e63aaf6b73b0f17cdd2387fce67883ad545033c
```

The completed result is recorded below. The Google guide was updated with revision-guarded scoped edits and verified native readback. All 42 tabs and historical content were preserved; the 40 unaffected tab bodies were unchanged. The Markdown mirror regenerated and passed its check from `outputs/doc-sync/highway-population-v01/verified-source.json`. Native pagination was not inspected.


## Completed pilot and decision

Run: `outputs/highway-population/v01/train-20261104T225405Z-302c4943`. Runtime source commit: `07482466f4d85805d931e4f9c639acb9600aafa0`. The run completed with exit 0 and no runtime-source drift, failure recovery, automatic extension or held-out execution. Source provenance truthfully records unrelated dirty workspace files; the training sources were committed and hash-verified.

The process consumed 99,999 active-agent decisions (4,096 critic-only and 95,903 PPO), 41 rollout updates, 76 critic warmup optimizer steps and 1,392 PPO optimizer steps. Charged elapsed time was 474.296 seconds. One decision remained unusable for the next active cohort. The chat waited for process exit without reading metrics, changing code or running agents while learning proceeded.

| Matched 16-case validation | Frozen start | Final |
| --- | --- | --- |
| Pooled interventions | 11,926 / 57,783 = 20.6393% | 18,266 / 57,619 = 31.7013% |
| Worst case | 44.875% | 75.125% |
| Cases failing existing gates | 15 / 16 | 15 / 16 |
| Completed controlled merge targets | 4 / 4 | 4 / 4 |
| Passes | 13 | 14 |
| Final collisions / teleports / overspeed | — | 0 / 0 / 0 under SUMO protections |

All sixteen initial and final accounting windows were complete and valid. Thirteen cases worsened in intervention rate; three improved. The worst final case was one AV among 24 cars with no messages on the two-lane cruise fixture. Intermediate counts 3/5/7 also regressed as a group: 20.5668% to 31.2406% pooled interventions across nine matched cases. The all-AV 8/8 cruise case passed, but that isolated result does not establish slider robustness. See [all per-case results](evidence/highway-population-v01/comparison.md) and [machine-readable evidence](evidence/highway-population-v01/results.json).

**Native emergency warning discrepancy:** sampled `emergency_braking_observed` counters were zero, but SUMO warning logs contain four controlled-car emergency stops at lane ends during training and one during initial validation. Final validation has no such warning. The native warnings and their paths/hashes are retained in the [independent audit](evidence/highway-population-v01/audit.json); do not describe the entire run as emergency-free or assume the sampled counter covers every native event.

**Decision: reject promotion.** The final actor is retained in [the experimental policy catalog](../policies/versions/highway-population-v01/README.md), SHA-256 `8d2e8793072e8a575acba03391da6783705b87521e0848ba14401345d4616e59`. It does not supersede the parent or any accepted policy. Full resume checkpoints, raw journals, native logs and per-case ledgers stay local under the run directory; the frozen actor, exact configuration, compact metrics and checksums are published.

Next action: diagnose acceleration/native-response mismatch and the missed lane-end warning events using the saved journals. Establish a scripted baseline on the same population matrix before proposing a targeted correction and another bounded run. Do not increase the budget, combine the unfinished correction-memory study or enable live slider handoffs automatically. Camera/LiDAR integration and physical transfer remain separate gates.
