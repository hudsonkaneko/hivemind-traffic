# Living roadmap: from structured traffic learning to sensor-driven cars

Revision **v07**, updated **November 4, 2026, America/Los_Angeles**.
Status: **Stage 1 remains unaccepted; variable-population pilot completed with regression**.
Parent: [hybrid roadmap](hybrid-roadmap.md). Persistent update instruction:
[AGENTS.md](../AGENTS.md#persistent-sensor-driven-training-plan).

## Outcome and current next action

Build a controller that cruises, follows, passes and merges on varied highways,
using observations available to a real vehicle. Keep ego speed/odometry,
localization, map/route guidance and actually delivered V2V messages alongside
camera/LiDAR perception. Do not require every useful input to come from pixels.
Privileged simulator state can supervise a teacher, train the centralized critic
and score evaluation; it must not silently enter the deployed actor or its masks.

**Next action:** diagnose the failed [population pilot](highway-population-training-v01.md) using saved journals: interventions increased from 20.64% to 31.70%, and native lane-end emergency warnings escaped the sampled counter. Prepare a matched scripted baseline and a targeted correction before any new training proposal. The v04 correction-memory comparison remains separate and unimplemented; no further learning is authorized by this completed pilot.

| Stage | Current status | Exit evidence needed |
| --- | --- | --- |
| 1. Learn driving with structured observations | Population v01 completed; 4/4 merge targets and 14 passes, 15/16 cases fail; interventions regressed | Per-task driving gates with low intervention and a frozen held-out evaluation |
| 2. Learn with imperfect observations | Planned; existing message delay is only part of the problem | Paired clean/noisy results, leakage audit and reliable stale/missing-data behavior |
| 3. Replace ideal fields with sensor estimates | Frozen segmentation/fusion previews and dataset/trainer preparation exist; policy integration open | Perception/tracking accuracy and latency, then physical closed-loop validation |

The target data path is **cameras + LiDAR + ego motion + map/route + received V2V
messages -> calibrated perception and tracking -> structured local observations
with confidence/age -> MAPPO actor -> acceleration/lane intent -> physical
trajectory and vehicle controller**. Each interface needs its own acceptance gate.
SUMO remains the inexpensive learning environment; Isaac/Omniverse supplies
selected sensor and physical validation, with lightweight background traffic.

## Verified starting point

| Component | What exists | Limit that determines the plan |
| --- | --- | --- |
| [Highway pilot](highway-curriculum-training-v01.md) | 499,998 active-agent transitions; 81-input graph/message actor; acceleration and lane-change actions | Final validation: 98.76% cruise speed ratio, eight passes, **0/6 merges**, **96.44% overrides**. Not accepted; no held-out test used |
| Current actor inputs | SUMO state, map/route features and delayed messages; 60 mph ceiling | No camera/LiDAR input. SUMO owns lane centering/lateral execution. Current 1.5 s following rule and stopping bounds are not permission for very short gaps |
| [Saved-capture prediction](segmentation-refinement-preview-v02.md) | Frozen SegFormer B0 refinement: 86.06% shared-class mIoU, 93.74% projected predicted-label accuracy | Generic obstacle semantic recall is zero. 89.28% geometric obstacle point recall covers eligible returns, not all obstacles or tracked objects |
| [Ground-truth fusion](semantic-lidar-fusion-v01.md) | Calibrated static and scripted moving fixtures; about 99.95% assigned-label accuracy | Ground-truth projection accuracy is not learned perception accuracy; coverage, ego motion and highway-speed sensing remain separate |
| [Segmentation preparation](segmentation-training-preparation-v02.md) | 3,600 RGB frames in 180 groups; 2,880/360/360 train/validation/test split; trainer and zero-update CUDA preflight | Six-class head is untrained. New corpus has no LiDAR or motion vectors, shares car geometry and uses static human poses |
| [Physical vehicle foundation](physics-vehicle-foundation.md) | Bounded low-speed physical-car evidence and sensor assets | No accepted highway-speed sensor-driven MAPPO transfer. Pose playback is not physical closed-loop control |

The failed pilot and diagnostic candidate remain under
`outputs/i110-merge/v01/`; their exact hashes and source/runtime provenance are in
the linked evidence note. Accepted v04r2 remains separate and unchanged. Bulk
weights, captures and logs are local artifacts, not automatically backed up by Git.

## Stage 1 â€” driving competence before sensor uncertainty

1. **Make intervention causes measurable.** Preserve requested, filtered and
   realized acceleration; separate speed ceiling, following, lane-gap and emergency
   interventions. Count affected decisions per cause and per task. Saturating the
   accelerator against a safety guard is not successful speed control.
2. **Audit merge information and action masks.** Include ramp distance/end,
   target lane, route connectivity, projected conflicting-car gaps and relative
   speeds across connected road segments, including background cars. Retain
   delivered message age/validity. Version the observation ABI, masks, normalization,
   checkpoint metadata and export together; old weights cannot load silently.
3. **Establish a feasible teacher, then warm-start MAPPO.** Validate a scripted
   speed/follow/pass/merge controller using train-only scenarios. Collect
   demonstrations and behavior-clone the actor. Test closed-loop teacher behavior
   before treating its labels as targets. If necessary, label student-visited
   training states and use a separately logged, decaying imitation loss; do not
   mix demonstration transitions into PPO's on-policy importance ratios.
4. **Use a mixed curriculum.** Begin with cruise/following, add slower-car passing,
   then easy-to-dense merges. Retain earlier tasks throughout and vary lane count,
   speed limit, density, driver behavior, gap placement and communication delivery.
   Resample failed training scenarios for fresh on-policy rollouts; cap their
   share so ordinary traffic remains represented. Do not replay stale PPO samples;
   stored teacher labels belong only in the separate imitation loss. Validation
   failures may guide development but validation trajectories must not become
   demonstration/training samples.
5. **Fix incentives without weakening guards.** Reward feasible acceleration,
   speed tracking, route progress and completed safe merges/passes. Penalize
   unnecessary lane changes, unsafe requests and excessive acceleration/jerk.
   Add bounded cooperation rewards only when they improve the team outcome.
   Use focused ablations to separate observation, imitation and reward effects.
   Keep the 60 mph ceiling and current headway/stopping protections.

**Proposed pilot allocation, not launch authorization:** at most two attempts,
each capped at **500,000 active-agent transitions and two hours**, whichever
limit is reached first. Total experiment ceiling: **1,000,000 transitions/four
hours**, counting failed attempts and resumes. Per-attempt allocation: up to
50k demonstration transitions, 100k foundational driving, 100k passing and 250k
merge/mixed transitions. Demonstrations count toward that total; up to four BC
epochs and all validation count toward the wall-time cap. Preserve cohort/joint
step accounting instead of relabeling agent transitions as simulator steps.
These are allocation ceilings, not a promise that competence fits the budget.
Freeze stage-advance/plateau criteria and allocation accounting in the final config.
Only spend the second attempt after a documented diagnosis and scoped revision.

**Proposed quality gates:** zero collisions, teleports and overspeed steps;
cruise speed ratio at least 0.90 in each cruise case; at least one valid pass per
passing case; at least 90% successful merges overall and by scenario group; safety
override fraction at most 0.05 in every nominal case. With only six merge vehicles,
90% requires all six: enlarge the evaluation cohort before drawing reliability
conclusions. Report numerator/denominator, per-seed outcomes, confidence intervals,
time-to-collision, headway, comfort, throughput and requested/completed/unfinished
trip accounting. Do not hide a failed task behind an aggregate reward.

Select checkpoints using the fixed validation matrix and competence-first ranking.
Run the untouched geometry/speed test set only after freezing a candidate. Test
failures remain evidence; any later tuning requires a new untouched confirmation
set. Repeat accepted candidates across independent training seeds under a separately
reviewed budget before making repeatability claims. These are research acceptance
criteria, not proof of road safety. Closer cooperative following is a later,
separately evaluated change, conditioned on braking, delay and fallback evidence.

## Stage 2 â€” robustness to realistic imperfect observations

1. **Define a deployable observation contract.** Every object estimate and message
   needs frame/time, age, validity and uncertainty; specify units, frames, range,
   slot ordering, clipping and unknown values. Carry ego motion, route intent and
   localization uncertainty. Missing or occluded objects must not mean empty road.
2. **Calibrate corruptions from data.** Measure range/velocity/lane errors and
   latency from available captures; collect missing paired temporal evidence later.
   Model correlated bias, occlusion, missed/false detections, ID switches, stale
   tracks, map/localization error, V2V delay/jitter/loss and receiver-observable age.
   Freeze nominal and severe-stress envelopes before evaluation. Do not invent
   numerical sensor accuracy from SUMO ground truth or tune noise on the test set.
3. **Close privileged-information leaks.** Actor masks and deployable safety
   decisions must use estimated gaps and legally available map information.
   V2V payloads must eventually contain sender estimates, not oracle state.
   Label the existing oracle simulator shield separately from a deployable guard;
   report both intervention streams. Training critic/teacher and offline scoring
   may retain explicit privileged inputs. Audit the whole actor decision path.
4. **Compare before adding complexity.** Use paired scenario seeds for clean
   actor/clean observations, frozen actor/corrupted observations and robust
   actor/corrupted observations. Start with timestamped history and consistent
   noise training. Add a GRU with age/action history only if delayed-state failures
   justify it; version sequence handling, resets and inference export together.

**Proposed budget:** initial frozen robustness evaluation uses zero optimizer
steps. If justified, propose one robustness fine-tuning attempt capped at **250k
active-agent transitions/60 minutes including evaluation**, requiring its own
reviewed config and authorization. This is separate from Stage 1, not an automatic
extension. Resume only compatible actor/normalizer/optimizer state.

**Exit:** retain Stage 1 competence and intervention targets inside the declared
nominal envelope, with predeclared degradation limits for speed, merging and
comfort. Severe occlusion/dropout cases are scored for collision avoidance,
stale-data handling and controlled fallback, not forced to maintain cruise speed
or a low override rate. Measure availability and safe progress separately.
Numeric corruption ranges and allowable degradation remain open until calibrated.

## Stage 3 â€” perception, tracking and physical closed-loop transfer

1. **Finish the compact semantic model first.** Reuse the prepared SegFormer B0
   and six-class [disabled configuration](../configs/perception/segmentation_training_v02.json).
   Recheck data hashes, splits and GPU preflight before requesting supervised
   training. Existing weighted CE, foreground Dice and boundary Dice losses are
   the starting point, not a new architecture search. Separate road, lane, vehicle,
   pedestrian and obstacle quality. Measured unclassified occupancy remains
   occupied; absent or rejected observations remain unknown. Add asset diversity
   and moving people in a later dataset version rather than claiming current data
   covers them.
2. **Build paired temporal sensing and tracking.** The existing 3,600-frame RGB
   corpus cannot train or validate full RGB/LiDAR temporal fusion. Plan synchronized
   RGB, LiDAR, calibration, ego pose/time and evaluator-only object truth in whole
   sequences. Project with visibility checks and ego-motion compensation; estimate
   occupancy and object position/size/heading/velocity with uncertainty. Start with
   geometric clustering, association and a motion filter; compare a suitable
   CenterPoint checkpoint only after checking class coverage, point format,
   coordinate convention, license, domain mismatch and local inference cost.
   A semantic mask alone is not velocity estimation or object tracking.
3. **Match the actor contract.** Generate lanes/gaps, tracked neighbors and
   confidence/age from perception plus map/localization. Do not use simulator actor
   IDs or exact velocities in deployment. Preserve observations/actions in a
   timestamped journal and measure drift against evaluator-only truth. Use Stage 2
   calibration/adapter training to handle the predicted-input distribution gap.
4. **Validate in increasing scope.** Saved frames -> saved temporal sequences ->
   3D shadow inference -> bounded low-speed physical control -> mixed highway
   scenarios. Shadow inference does not actuate the car. In physical tests the
   main car receives acceleration/lane intent through a trajectory and steering/
   torque controller, with one motion owner; backgrounds remain lightweight.
   No SUMO pose teleport or visual replay counts as this acceptance test.
5. **Fine-tune only the demonstrated gap.** Freeze/evaluate perception and planner
   separately first. Sensor-policy or physical-control fine-tuning needs a new
   resolved config and budget after a measured failure diagnosis. Do not launch
   end-to-end camera/LiDAR RL from scratch as the first local experiment.

**Segmentation gates to freeze before supervised learning:** class IoU, boundary
F1, false-drivable area and obstacle/pedestrian precision/recall, stratified by
image size and by distance where evaluation depth supports it. The later temporal
fusion/tracking gates add range/velocity error, ID switches, temporal flicker,
coverage and uncertainty calibration; the current RGB-only corpus cannot measure
those. Split entire scene/drive/asset families, not adjacent frames. Keep existing
inspected preview frames as regressions, not new held-out evidence. Set numerical
thresholds from stopping/control needs and a measured baseline before each stage.

**Runtime target to benchmark:** a 10 Hz policy loop implies a 100 ms end-to-end
deadline, including sensing age, perception, tracking and control. Report p50/p95
and worst latency, deadline misses, stale-data fallback and peak VRAM. This is a
proposed target, not measured performance. Local RTX 5070 has about 12 GB VRAM;
benchmark batch-one inference alongside Isaac, and serialize competing GPU jobs.
Do not assume BEVFusion or a world model fits the remaining memory. Keep traffic,
perception and Isaac dependencies isolated and follow [USD discipline](../usd/SCENE_STRUCTURE.md).

**Budget status:** the existing supervised segmentation proposal remains disabled
and must be shown again before launch. New paired capture, tracking experiments,
3D runs and policy fine-tuning need explicit scoped budgets; none are allocated
or started by this roadmap. End-to-end acceptance requires physical closed-loop
driving with sensor estimates, deployable safety controls, latency and retained
per-task metrics; a good mask or video alone cannot satisfy it.

## Research informing the choices

Primary sources reviewed November 2, 2026. These motivate experiments; none prove
that this repository already has equivalent models, data scale or performance.

| Work | Applicable finding and planned use | Boundary |
| --- | --- | --- |
| [Waymo: imitation plus RL](https://waymo.com/research/imitation-is-not-enough-robustifying-imitation-with-reinforcement-learning/) | Warm-start useful driving behavior, then improve closed-loop recovery | Their algorithm/result is not our MAPPO validation |
| [Waymo Driver/Simulator/Critic](https://waymo.com/blog/2025/12/demonstrably-safe-ai-for-autonomous-driving/) | Separate driving, simulation, critique and failure mining; retain independent validation | Public architecture is not a complete reproducible training recipe |
| [NVIDIA Alpamayo post-training](https://developer.nvidia.com/blog/how-to-post-train-autonomous-vehicle-models-in-closed-loop-with-nvidia-alpamayo) | Start from useful behavior and target measured closed-loop failures | Published default GRPO differs from our MAPPO; no algorithm migration proposed |
| [DriveAdapter](https://arxiv.org/abs/2308.00398) | Predicted perception and privileged planner inputs have a distribution gap; test adapters/robustness | Better segmentation alone does not guarantee good driving |
| [DAROM](https://arxiv.org/abs/2403.11852) | Delay-aware history and recurrent encoding motivate Stage 2 ablations | Different training setup; reported delay success is not a local guarantee |
| [SegFormer](https://github.com/NVlabs/SegFormer), [paper](https://arxiv.org/abs/2105.15203) | Reuse the compact B0 semantic baseline already prepared here | It supplies image semantics, not tracked 3D state; retain license/runtime constraints |
| [CenterPoint](https://github.com/tianweiy/CenterPoint), [paper](https://arxiv.org/abs/2006.11275) | Candidate LiDAR detection/tracking benchmark for boxes, heading and motion | Requires compatible scans, labels, calibration and domain validation |
| [BEVFusion](https://github.com/mit-han-lab/bevfusion), [paper](https://arxiv.org/abs/2205.13542) | Optional later shared-BEV camera/LiDAR comparison | Upstream repo archived January 2025; old CUDA stack and memory cost make it a later experiment |
| [nuPlan evaluation](https://github.com/motional/nuplan-devkit/blob/master/docs/competition.md) | Distinguish open-loop, nonreactive closed-loop and reactive traffic evaluations | Reactive backgrounds are needed to assess cooperation |
| [Cosmos interactive simulation](https://developer.nvidia.com/blog/build-world-models-for-interactive-simulation-with-nvidia-cosmos/), [Sim Auto model card](https://huggingface.co/nvidia/Cosmos3-Nano-Sim-Auto) | Possible later weather/appearance expansion and interactive sensor simulation | Not a replacement for our traffic physics; published AV setup uses multiple GPUs, and custom-rig transfer needs validation. No local fit, cloud purchase or upload assumed |

Large-scale systems such as [Gigaflow](https://arxiv.org/abs/2502.03349) also show
self-play can work without human demonstrations at substantial scale. Imitation
is our practical small-budget starting hypothesis, not a universal requirement.
Keep SUMO first; changing simulators is not necessary to test the current failure.

## Low-usage execution and retained evidence

- Freeze config, seeds/splits, source hash, runtime versions, observation/action
  ABI, reward version, budgets and gates before each launch. Distinguish proposed,
  authorized, executed and quality-accepted states.
- Put checkpointing, finite-value checks, failure handling and evaluation in the
  runner. Proposed schedule: a four-case sentinel every 50k transitions and the
  full 12-case validation matrix at stage boundaries/completion; deduplicate when
  they coincide. Include all of that time in the cap and abort cleanly on expiry.
- While training runs, the agent does no analysis or progress polling. Wait for
  completion/failure and inspect the retained report then. This is automated run
  instrumentation, not continuous chat/model monitoring or a heartbeat task.
- Resume mechanical failures from a compatible checkpoint within the remaining
  aggregate budget, preserving optimizer/RNG/curriculum/accounting state. A material
  reward/observation change is a new version, not an invisible resume. Do not
  endlessly retry or extend budgets; document remaining limits before a revision.
- Keep immutable attempts in `outputs/`, logs in `logs/`, and compact per-seed
  results/provenance in `documentation/evidence/`. Save actor hashes, requested/
  filtered/executed commands, intervention causes, safety/task/comfort/throughput
  metrics and latency. Retain failures and rejected candidates. Record artifact
  backup/release status rather than claiming ignored weights are on GitHub.

## Project memory and update procedure

This file is the persistent plan read through `AGENTS.md` during future relevant
project work. It is not an unattended watcher of GitHub, Docs or training jobs.
After each meaningful implementation, evaluation, failed attempt or decision:

1. Update the revision/date, stage status and single next action above.
2. Link the actual source/config/run/checkpoint and measured evidence; record
   accepted, rejected, blocked and unknown separately. Never rewrite old results
   to match a new interpretation or weaken an acceptance gate silently.
3. Append a dated progress entry below with changes, results, remaining risks and
   the next gate. Add the relevant technical note and CHANGELOG entry; link major
   milestones from the parent hybrid roadmap.
4. Commit/push the scoped tested change on its feature branch with PR review.
   Update the native engineering guide and regenerate its complete Markdown mirror
   using [the sync procedure](learning-guide-sync.md#chat-managed-markdown-mirror).
   If either service is unavailable, record exactly which copy is pending.

## Progress history

### v01 â€” November 2, 2026: research-backed plan and persistent memory

- Audited highway pilot and perception preparation with sub-agents; recorded the
  actual failed driving gates and untrained six-class model status above.
- Established the three-stage plan, proposed bounded learning allocations,
  sensor/privilege boundary, evaluation rules and low-usage execution procedure.
- Added the persistent project instruction and linked the overall roadmap.
- Planning/documentation only: no trainer, sensor, simulator or runtime changed;
  no model downloaded and no training started. Next gate is Stage 1 preparation
  followed by a concrete configuration review. Guide sync evidence is recorded in
  the [sync ledger](learning-guide-sync.md).

### v02 â€” November 2, 2026: Stage 1 implemented and prepared

- Added intervention diagnostics, a versioned 93-field merge contract, feasible
  observation-only teacher, BC/MAPPO runner, mixed curriculum and bounded recovery.
- The first full preflight passed 27/30 teacher cases and exposed crossing
  connectors in generated ramps. A separate right-entry v02 geometry bank fixes
  those fixtures while preserving original networks, imported I-110 and failures.
- Final preflight: 30/30 cases pass, 30/30 ramp merges, passing in all five passing
  cases, zero collisions/teleports/overspeed, maximum case overrides 4.39%. These
  are scripted results on corrected scenarios, not learned-policy acceptance.
- 243 regression tests pass. No BC/PPO learning, held-out traffic, new sensor
  captures or physical simulator runs occurred. Existing protection thresholds
  remain unchanged. [Results/config/evidence](highway-stage1-preparation-v02.md).
- Next gate: approve the resolved disabled configuration for one attempt capped
  at 500k active-agent transitions/two hours. No automatic second attempt; evaluate
  learned driving before Stage 2. Agent waits while future training runs; the
  runner retains metrics and signals completion/failure.

### v03 â€” November 3, 2026: authorized Stage 1 learning completed

- User approved the presented configuration. Ran one bounded CPU attempt with
  no progress inspection or analysis until process exit. Runtime sources unchanged.
- 499,992 transitions (49,998 demonstrations + 449,994 PPO), 162 updates, 59m24s.
  Completed normally at the transition cap; no mechanical recovery needed.
- Selected final actor: 6/6 validation merges, 12 passes, cruise speed
  96.59â€“97.42% of limit, zero collisions/teleports/overspeed under protections.
  Overrides 6,965/38,380 (18.15%); worst case 53.56%; eight of twelve cases fail.
- Phase 2 had lower pooled overrides (12.47%) than the selected final actor, but
  also failed acceptance. Preserved phase results, hashes, sources and checkpoints.
- [Full result and policy location](highway-stage1-training-v02.md). No held-out
  test, sensor/physical transfer or second attempt. Next gate is a documented
  intervention diagnosis and reviewed revision; Stage 1 remains unaccepted.

- User-requested publication: [policy versions](../policies/versions/README.md) now
  retain the frozen accepted v04r2 and experimental Stage 1 v02 actors with short
  change notes, exact configs/contracts, measured results and checksums. Newer
  publication does not imply acceptance or compatible observations.

### v04 â€” November 3, 2026: diagnosis and v03 preparation

- Offline audit: 94.90% of final native-change decisions realize the logged
  feasible acceleration target within 0.05 m/sÂ². Native internal causes remain
  unlogged; merge discrepancies are less well explained.
- Frozen after-BC actor: training-label acceleration RMSE 0.2013 m/sÂ² versus
  99.886% lane-label agreement. No fitting occurred during this audit.
- Teacher on the same twelve validation cases: 1.168% pooled overrides, worst
  4.156%, all gates passed; two-lane following/passing is materially slower.
  No validation trajectories enter learning and no held-out traffic ran.
- Implemented ranking v5 and disabled v03 proposal: 12 BC epochs, 4Ã— BC
  acceleration loss, imitation 0.05â†’0.01 over 400k PPO decisions. Fixed 500k/two-hour
  attempt budget, observations, rewards and safety protections remain unchanged.
- 98 tests and frozen full 30-case no-learning preflight pass. Original selected
  actors/catalog remain intact; phase 2 is favored only in the recorded reranking.
- [Diagnosis, exact config and evidence](highway-stage1-v03-preparation.md). Next
  gate is configuration approval before the second bounded learning attempt.


### v05 â€” November 3, 2026: v03 learning and checkpoint recovery

- User approved the reviewed configuration. Only the authorization flag changed in a retained launch copy; the tracked proposal remains disabled for future launches.
- One Windows checkpoint replacement failed; the valid closed checkpoint resumed with identical sources/configuration and retained consumed budget. No runtime change or repeated failure occurred.
- Total 499,990 transitions, 162 PPO updates and 3,544.125 seconds charged runtime. The chat waited for exit; automated metrics and journals remain preserved.
- Selected validation: 14.97% pooled/51.47% worst overrides, five failing cases, 6/6 merges and 12 passes; zero collisions/teleports/overspeed under protections. Four-lane passing regressed; older phase-2 v02 still has lower pooled interventions.
- [Run, failure and comparison evidence](highway-stage1-training-v03.md); [frozen experimental actor](../policies/versions/highway-stage1-v03/README.md). No held-out traffic or physical transfer was run.
- Stage 1 is still open. Diagnose the remaining per-case failures and present a revised setup before another learning attempt; existing safety/acceptance criteria stay fixed.


### v06 â€” November 4, 2026: improved training plan

- Saved the [v04 training plan](highway-stage1-v04-training-plan.md): diagnose following/passing errors, then compare current-only imitation with retained acceleration corrections from training states.
- Proposed shared warm start from verified v03 phase 0; explicit import is required because ordinary resume cannot reuse selected checkpoints for a new experiment. Keep observations, reward, safety guards and lane imitation unchanged.
- Proposed paired screen: 100k new PPO decisions or 30 minutes per arm; deterministic 20k task-balanced memory for B, no validation/test data or historical PPO replay. Predeclare worst-case-first selection and compare fixed endpoints with efficiency protections.
- Screening success is distinct from the unchanged per-case 5% acceptance gate. A repeated-seed study and held-out evaluation require later scoped proposals; sensors remain a subsequent stage.
- Research basis: dataset aggregation and on-policy PPO, linked in the plan. No runtime changes, new simulator runs or learning occurred. Next deliverable is implementation/preflight plus exact disabled configurations for review.


### v07 â€” November 4, 2026: variable-population preparation

- User authorized a separate bounded population-adaptation pilot; v04 correction memory is not included.
- Implemented reset-based mixed cohorts, nearest-six messages, a masked eight-slot critic and an unchanged 93-field actor.
- 157 targeted tests plus frozen no-learning API/seed/collector/export preflight pass; spawn and reset-option failures remain recorded.
- [Resolved configuration and evidence](highway-population-training-v01.md). Next action: launch the authorized 100k/30-minute run, wait for exit, retain results and synchronize the guide. Live handoff, held-out acceptance and sensor transfer remain open.


### v07 result â€” November 4, 2026: population pilot completed, not accepted

- Ran the approved configuration once: 99,999 decisions, 474.296 charged seconds, no runtime failure/source drift or monitoring during learning.
- Matched validation regressed from 20.6393% pooled/44.875% worst interventions to 31.7013%/75.125%; both endpoints fail 15/16 cases. Final 4/4 merge targets and 14 passes do not outweigh this regression.
- All initial/final ledgers are complete; final collision/teleport/overspeed counts are zero under protections. Four training and one initial-validation native emergency lane-end warnings were missed by sampled counters; none appear in final validation.
- [Result, raw-evidence locations and next action](highway-population-training-v01.md). Preserve the final experimental actor without promoting it. No new run, live handoff, held-out or sensor/physical evaluation follows automatically.


### November 4: population v02 authorized correction

Matched scripted baseline measured 1.51% pooled overrides; prepare acceleration-only supervision weight 50 with unchanged guards and bounded matched retraining. See [v02](highway-population-training-v02.md). Results pending.


### November 4: population v02 retraining completed

[Population v02](highway-population-training-v02.md): 99,998 decisions in 514.516 seconds. Pooled overrides 29.97%, worst 86.75%; twelve failing cases. Rejected for promotion. Metrics, native warnings and experimental weights retained. Next: diagnose acceleration errors before another revision. Google guide sync pending.
