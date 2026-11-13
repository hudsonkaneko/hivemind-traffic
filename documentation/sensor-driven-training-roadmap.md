# Living roadmap: from structured traffic learning to sensor-driven cars

Revision **v42**, updated **November 10, 2026, America/Los_Angeles**.
Status: **V04 long run completed: final I-110 focal car has zero stops/overrides; fleet overrides 0.42%. Comfort/quality gates still fail; no promotion.**
Parent: [hybrid roadmap](hybrid-roadmap.md). Persistent update instruction:
[AGENTS.md](../AGENTS.md#persistent-sensor-driven-training-plan).

## Outcome and current next action

Build a controller that cruises, follows, passes and merges on varied highways,
using observations available to a real vehicle. Keep ego speed/odometry,
localization, map/route guidance and actually delivered V2V messages alongside
camera/LiDAR perception. Do not require every useful input to come from pixels.
Privileged simulator state can supervise a teacher, train the centralized critic
and score evaluation; it must not silently enter the deployed actor or its masks.

Latest offline comparison: [adjacent native traffic](highway-flow-rl-v02.md#november-8---speed-comparison-with-adjacent-native-traffic) accelerates more steadily than the frozen actor; no additional simulation or training.

**Next action:** the [startup handoff ramp](highway-jerk-diagnosis-v05.md#startup-handoff-ramp-november-11-2026) fixes startup jerk on 5/6 ramp-started layouts. Remaining blockers are merge quality (27-49% vs 90%) and fragile merge decisions (a missed borderline gap window leads to riding the acceleration lane to its end). Both need a training change; agree on it before launching. Then run the frozen actor once on the held-out merge geometry bank. [Ramp-started network results](highway-jerk-diagnosis-v05.md#frozen-actor-on-ramp-started-networks-november-11-2026): 0% overrides, ~12 m/s3 peak (startup only) on all six layouts. The `mlk_on_0` braking is [explained](highway-jerk-diagnosis-v05.md#native-braking-on-mlk_on_0-explained-november-11-2026). [V05 jerk diagnosis](highway-jerk-diagnosis-v05.md) (November 11): 11/16 failing cars exceed 5 m/s3 only at startup; the 50.76 m/s3 maximum is native braking and its release, not the actor. Previous v04 terminal context: [V04 terminal results](highway-merge-rewards-v04.md#completed-long-run---november-10-2026): 3,643,488 learning decisions, 1,298 updates, about 7h47m. Final focal stopped time and interventions are both zero; fleet interventions 0.42%, stable merges 2/2 and no collisions/teleports. Final actor passes 0/6 whole-case gates; selected best update 1078 passes 1/6. Both actors retained; no promotion, held-out evaluation or new run.

Latest GPUDrive diagnostic: [v05](gpudrive-i110-v05.md) improves longitudinal comfort with an explicit filter, but the crossing audit finds an unconnected early ramp exit in both v04 and v05. Mainline occupancy/goal arrival must not be treated as verified proper merging. Next transfer work is the intended merge corridor and steering validation; no training or policy promotion.

| Stage | Current status | Exit evidence needed |
| --- | --- | --- |
| 1. Learn driving with structured observations | Environment v05 pilot complete and rejected; 8/9 stable merges but 19.83% interventions and 19/24 failed cases | Per-task driving gates with low intervention, complete native safety accounting and a frozen held-out evaluation |
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
| [Physical adapter](mappo-physical-adapter-v01.md) | Calibrated native acceleration/lane execution at 6/12/18 m/s; frozen actor cruises and passes one scripted car | Actor intervention gates fail. Straight road, simulator truth and surrogate dynamics only; no accepted sensor-driven transfer |

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


### v08 — November 4: checkpoint review and published AV comparison

Verified and preserved the exact Stage 1 v03 phase-0 actor as a reference, not accepted deployment. Same-population overrides 20.64% pooled/44.875% worst outperform both retrains, while v02 passes more individual cases. Offline journals reveal over-requesting without extra progress and sparse-case underexposure. [Comparison](policy-baseline-av-comparison.md) distinguishes PDM/PlanT planning, Wayformer forecasting, SMART sim agents, GPUDrive infrastructure and TransFuser perception. Prioritize control calibration/objective diagnosis before controller, history or attention ablations. No simulator or training ran. Google guide and Markdown mirror synchronized with verified full readback.

### v09 — November 4: frozen acceleration and objective diagnosis

Completed steps 1 and 2 of the comparison using the frozen phase-0 actor, two implementation agents and independent review. Fourteen training-only acceleration probes (7,200 decisions) found synchronized acceleration accurate to 2.57e-13 m/s² and identical 0.1/0.5-second duration traces. Native following limits constrain over-requesting; the scripted teacher reduces interventions without resolving every efficiency limitation. Twelve no-update rollouts yielded 1,536 balanced loss-diagnostic rows: PPO/imitation gradient cosine -0.420, teacher/reward target difference 0.3002 m/s², and fresh-critic gradients that dominate global clipping before warm-up. These are measured mechanisms and hypotheses, not historical post-warm-up proof or accepted driving. All weights and runtime policy/environment code remain unchanged. [Results, plots, native-warning reconciliation and retained artifacts](population-control-diagnostics-v01.md). Next: consistent target design and post-warm-up objective telemetry, then an explicitly reviewed bounded training configuration. No further learning is authorized.

### v10 — November 4: population v03 control correction prepared

Implemented opt-in near-limit taper, first minor-entry conflict observations, source-backed stop-line approach calibration and a shared observation-only teacher/reward acceleration target. Explicitly versioned changed 93-field merge semantics; v2 controls remain available. Added separate actual actor/critic gradient and fixed-return value diagnostics without changing optimization, plus complete native-event accounting/gating for v3. Across 32,400 bounded train-fixture decisions, the teacher reached zero overrides in tested passing/merge cases with tasks retained and similar speeds; the frozen actor's merge overrides improved 26%→11.17% but remain unacceptable. Retained both development attempts, including the initial missing stop-offset correction and provenance limits. 150 CPU tests and a 32.719-second zero-update final preflight passed; all existing weights remain unchanged. [Final configuration, results and measured plots](population-control-v03.md). Next: user review of the disabled 100k-decision/30-minute configuration before a new launch. No training, held-out acceptance or physical/sensor transfer occurred.

### v11 — November 5, 2026: calibrated physical execution with frozen MAPPO

- Used Robot Modeling and RL with sub-agents to implement an acceleration/lane-intent adapter, exact 93-field observation bridge and bounded native runner. No training or checkpoint changes.
- First fit failed: full-brake grip saturation distorted ordinary-brake response (0.66694 m/s² validation RMSE). Preserved the failure; restricted ordinary linear fitting/inversion to brake ≤0.5 and retained full braking separately.
- Corrected response RMSE 0.02131 m/s²; fresh 0.4-brake confirmation 0.01634. Scripted acceleration, left/return lane changes and stopping pass at 6/12/18 m/s; worst settled lane error 0.06473 m.
- Frozen reference cruises and passes a scripted slower car, but intervention gates fail at 312/380 (82.11%) and 161/580 (27.76%). All corrections are speed-ceiling related. No contacts, analytic overlaps, overspeed or adapter/driver fallbacks in these nominal cases.
- 209 targeted CPU tests and six fresh-process composed-scene reopen checks pass. Full traces, screenshots, source/config hashes and both attempts retained; [report and compact evidence](mappo-physical-adapter-v01.md).
- Next gate remains configuration review before any learning, followed by matched SUMO/physical evaluation. Curves, merges, reactive cooperation, native fault injection and sensor estimates remain unqualified. No real-car dynamics identification or policy promotion claimed.


### v12 — November 5: population v04 near-limit correction prepared

Three sub-agents implemented/reviewed the localized SI acceleration objective, speed curriculum and frozen diagnostic. Added four episode ceilings (12/18/22.352/26.8224 m/s), exact native lane/connector readback, eight additional lower-limit validation cases and raw-request diagnostics. Lower-limit coverage contributes to the mismatch, but synthetic highway-limit failures and previous-request sensitivity show it is not the sole cause.

171 tests and the final 27.657-second SUMO preflight passed with zero optimizer steps, unchanged weights and exact export/repeat checks. A background emergency in the teacher I-110 case remains in native evidence; no nominal acceptance is implied. [Final disabled configuration, diagnosis and evidence](population-near-limit-v04.md) retain the 100k-decision/30-minute pilot ceiling and unchanged safety gates. Review before learning; then evaluate matched initial/final cases before physical confirmation. The best actor, historical failures and held-out partition remain preserved.

### v13 — November 5: frozen I-110 merge visualization

Selected final Stage 1 v03 by its lowest intervention rate across the three retained matched merge cases (6/6 completed, 3.3194% pooled). This merge-specific choice does not replace the phase-0 overall population reference. One predeclared I-110 test, seed 6400019 with original v2 semantics, completed both merges at 3.0/9.7 seconds and all four controlled trips. Full 80-second accounting records no collisions, teleports, overspeed steps or native emergency warnings. However, 131/1240 active-agent decisions (10.56%) had native realization interventions, above the unchanged 5% target.

[The visual test](i110-merge-visual-v01.md) retains an 11-second Isaac-rendered replay, four screenshots, the complete metrics and provenance. All 110 frames decoded, recorded positions matched exactly and authored USD layers remained unchanged; six geometry/composition tests passed. The failed initial preview export is preserved. This is reconstructed provisional SUMO road geometry and recorded kinematic motion, not a physical merge or sensor-driven validation. No training, actor edits or acceptance promotion occurred. The next action remains review of population v04's disabled final configuration before separately authorized learning. Google guide and generated Markdown mirror were synchronized with verified full native readback.


### v14 — November 5: continuous-maneuver environment v05

User authorized implementation and training. Three agents implemented native three-second lane changes, 99 actor fields/24 critic slots, varied demand and ramp lengths, stable-merge/full-route metrics, and a resumable competence curriculum. A teacher fixture exposed 14/81 post-merge acceleration corrections because neighboring lateral motion was missing from lane context. Projecting measured motion into the affected corridor reduced that same fixture to 0/81 without weakening the gate; both attempts are preserved. The 1.5-second headway, 3-second TTC, speed limits and guards remain.

256 targeted CPU tests pass. Final source-matched SUMO preflight passed in 64.750 seconds with zero optimizer steps and unchanged sources/weights; no training has started at this documentation checkpoint. I-110 teacher completes a stable merge but retains 75/1404 (5.34%) interventions, above the unchanged 5% gate. Prepared a frozen recorder and SimReady main-car replay option, keeping background proxies lightweight. See [configuration, evidence and limits](highway-environment-v05.md). Google engineering-guide synchronization is pending the completed milestone, not claimed current.


V14 launch recovery: the first attempt failed during initial validation with zero optimized/collected training steps. Departure timestamp precision and native interrupted-maneuver lifecycle/runway checks are corrected; the exact frozen failing cases complete full horizons with no native warnings, while their policy intervention failures remain. 265 targeted tests pass. A reviewed strict migration preserves all starting state and 125.344 seconds charged; fresh preflight is required before resuming. [Preserved failure and repair](highway-environment-v05.md#first-launch-preserved-pre-learning-failure).


V14 second recovery: native ACC insertion feasibility required more initial clearance than collision safety alone. All 132 reset cases pass after the demand-only correction; counts/speeds/guards remain fixed, initial spatial density changes explicitly. 269 tests pass. Both failed launches occurred before learning; 512.048 seconds remain charged. Fresh readiness and a second strict zero-update migration precede resume.


### v15 — November 5: v05 training completed, candidate rejected

Completed `train-20261106T024828Z-90b25880` from source `02948d7` after two preserved pre-learning failures and explicit zero-update checkpoint migrations. The run used 249,999 active-agent decisions (4,096 critic warmup and 245,903 PPO), 50 update batches, 2,025 joint PPO optimizer steps and 68 critic-only steps. Its 2,455.032 total charged seconds include 512.048 from failures. The chat waited for process exit without progress monitoring or parallel thinking during training.

Matched 24-case validation: stable merges improved 2/9 to 8/9; passes increased 55 to 59; pooled interventions worsened 13.3749% to 19.8287%; worst-case interventions changed 73.125% to 69.3339%; failing cases decreased 23 to 19. All horizons completed, with no recorded collisions, teleports, overspeed or emergency events under protections. I-110 remains 0/1 merged; the ramp actor stalls before entry in both initial and final evaluation. The curriculum remains at grade 0. No held-out test or physical/sensor transfer ran. The latest actor is a separately labeled rejected candidate, not an accepted replacement.

[Results, plots and next diagnostic](highway-environment-v05-results.md); [frozen policy and exact contract](../policies/versions/highway-environment-v05/README.md). Preserve the existing references and diagnose training scenarios before another reviewed configuration. Two SimReady replay examples retain the I-110 failure and easier synthetic success, with 502 decoded frames, exact recorded poses and unchanged source assets. Both guide copies are synchronized from verified full readback; all 42 tabs are preserved.


### v16 — November 6: I-110 junction blockage diagnosed and corrected

Three sub-agents separated native junction behavior, network geometry and recording semantics. The original ramp request yielded to all five mainline lanes despite sharing only auxiliary lane 0. Native link plans show overlapping predicted passage windows for unrelated through-lane traffic. Acceleration-lifetime changes and a focal native-longitudinal handoff did not resolve the block. The new versioned network enables lane-specific conflicts and verifies that the true auxiliary conflict/yield remains; source geometry and demand are unchanged.

An independent ordinary recorder, without the diagnostic follow-speed getter, confirms the same frozen actor and seed now enter the auxiliary lane at 4.4 s, complete the lateral merge at 10.1 s and pass the eight-second stable window at 18.1 s. Focal stopped samples fall from 752/800 to zero. All eight controlled trips finish; fleet interventions fall from 1,007/3,298 (30.53%) to 244/2,864 (8.52%), still above the unchanged 5% gate. Full 80-second evidence, background trip accounting, unsuccessful probes and the potentially stateful getter caveat are preserved. The getter was removed from future probes.

The [technical note](i110-merge-blocking-v01.md) retains native diagnostics, exact hashes, before/after screenshots and a 251-frame SimReady v14 main-car replay with lightweight background cars. The video decodes, poses match exactly and authored/source assets remain unchanged. Reporting now distinguishes native junction entry, auxiliary occupancy, actual commanded lateral completion and stable through-lane occupancy. No training or policy promotion occurred; sensor/physical transfer remains unqualified. Diagnose the remaining requested-versus-realized acceleration mismatch on training fixtures before reviewing any new launch configuration.


### v17 — November 6: remaining overrides and merge comfort diagnosis

Three agents independently audited native equations, map/trajectory behavior and the training objective. All 244 corrected-I-110 intervention samples match the current target within 0.0001 m/s²: 233 following responses and 11 approach-braking responses. The actor requests too much acceleration; no evidence justifies changing native car-following, safety modes or the 5% definition. Its severe 11.12 to 6.41 m/s slowdown occurs before entry, followed by 58.93 m/s³ jerk; speed rises through and after the lateral merge.

A fixed training-only seed 6800019 pair retains identical demand and full 80-second accounting. Frozen actor: 557/3183 interventions (17.50%), peak jerk 59.96 m/s³. Existing scripted teacher: 0/3059 interventions, peak jerk 65.00 m/s³. Both complete one stable merge and 33/55 trips with zero recorded safety events. The teacher's abrupt -4.5 to +2 m/s² release makes it unsuitable as an unquestioned stronger imitation reference. These teacher results are not learned-policy improvement or acceptance.

[The diagnostic note](highway-smooth-control-v01.md) retains native/source hashes, all failures, scientific plots and an exactly repeated 801-frame validation trajectory, plus a 251-frame SimReady main-car visual with zero pose error and unchanged sources. [The proposed plan](highway-smooth-training-plan-v01.md) first separates a smooth anticipatory desired reference from mandatory constraints, then proposes stronger alignment, uncapped Huber jerk and reference-error slew rewards. It preserves progress, necessary braking and every safety gate. Its two 50k-decision arms and one-hour total ceiling remain unapproved; reference/comfort preflight and a resolved configuration precede explicit training authorization. No learning occurred, and the current frozen policy remains at 8.52% in the corrected I-110 case.


### v18 — November 6: smooth-control preparation implemented, readiness failed

Three subagents implemented the pure 99-field anticipatory reference, measured-jerk and command-residual rewards/metrics, and matched MAPPO schedule/budget/resume/acceptance contracts. Safety protections, native modes, demand and the legacy v05 objective remain intact. Retained v4 passes six original safety/task/5% intervention checks; I-110 focal merge completes at 20.0 s and stabilizes at 28.0 s without stopping. The full controlled-car peak is 2.000001 m/s³, compared with 65 m/s³ for the previous teacher. This is scripted reference performance, not a trained actor improvement.

Cruise/following/passing/generic-merge peaks remain 21.9587/45.0000/22.9691/10.7391 m/s³. The rejected adjacent-preview v5 reduced passing from seven to one and introduced a 14 s stop for another controlled I-110 car; source was restored byte-for-byte to v4. All 44 full native runs and actual raw/filtered/native traces remain local. The generic frozen-actor post-merge baseline is censored; a documented one-sided maximum bound supports only the corresponding slowdown comparison, not complete window coverage. No failure is removed from scoring.

195 CPU checks plus eight subtests verified across the full run and targeted metadata correction; 11 actual USD tests pass. Frozen-gradient inspection used identical real batches and zero optimizer updates. Two fresh 30-second SimReady v14 main-car replays retain exact SUMO motion, lightweight backgrounds and unchanged authored layers. They do not establish PhysX control or sensor perception. [The technical note](highway-smooth-preparation-v01.md) links all evidence and the disabled final configuration. Guide/native Markdown sync verified all 42 tabs. Next action is remaining peak diagnosis, passing readiness, then explicit training approval; no unattended training or policy promotion is authorized by this milestone.


### v19 — November 6: step back from the scripted slowdown

User requested a complete relevant-file/weight review and a new plan before learning. Three read-only audits cover the active native control chain, learner/reward math and checkpoint history; seven actors match their trusted finite checkpoint tensors and 70 local weight files were inventoried. The repository catalogue covers 1,039 tracked files but distinguishes cataloguing from actual content review. Saved raw traces expose unchanged controlled mean trip time 39.8875 s: v4 slows the main car earlier and sends another car into an ending auxiliary lane, then nearly stops on return. All four comfort failures have precise offline ACC/occupancy fits; a farther slow destination leader can be lost by nearest-only reduction. Native minor-entry visibility 4.5 m and conditional stopping/gap choice must be addressed before stronger teacher imitation. Historical v05 trained only difficulty 0, four merge episodes, no I-110; speed and merges improved while interventions worsened.

[The replacement training proposal](highway-training-rethink-v01.md) prioritizes correct road semantics, actor-visible route utility and joint gap/acceleration targets; it keeps prior safety/5% checks and adds full-cohort progress/stop gates. It separates imitation, comfort and optimizer ablations, retains foundation exposure, requires generic/I-110 and harder cases, and tests varied AV participation/communication loss after competence. Proposed 63 diagnostic runs have a 1,800 s cap; proposed first learning study has two 30k-decision arms / 5,400 s including preparation and a 900 s final-evaluation reserve. A later route/merge study is separately proposed at two 100k-decision arms / 7,200 s. These need new reviewed contracts and explicit approval; existing 3,600 s contract/config remains disabled. Raw historical costs are not erased. Actual saved-sample plots and audit/evidence hashes are retained. No new simulator, optimizer, actor, reward or runtime-source changes occurred. Google guide and Markdown mirror synchronized after verified full native readback.


### v20 — November 6: flow preparation implemented, qualification remains open

Three subagents implemented the 117-field current-state/map ABI, explicit topology/visibility fixtures, route-aware reference and paired MAPPO contract. The synthetic added-lane ramp now has a physically tangent entry; imported I-110 is unchanged. Both invalid native spawn attempts, JSON diagnostics failure, source-change rejection and deadline repair remain retained. All 81 allocated diagnostic recordings preserve complete 80-second accounting; 54 reused records have exact effective-input/control-source checks, 26 are fresh and one is an explicit deadline repair. Three frozen farther-leader prescribed events remain incomplete.

The reference passes 17/27 combined cases. All 27 retain raw safety/5% override checks, but four nominal measured-jerk peaks are 21.959/45.000/10.927/22.412 m/s³ and six counterfactual task gates fail. I-110 main-car minimum 3.421 m/s and 17.9 seconds below half the local limit remain unacceptable flow limitations despite its passing stress-case row. Agent_4 minimum improves from v4's 1.456 to 14.323 m/s after removal of the ending-lane detour. Four actual 60-second SimReady replays, full-route plots and compact metrics are retained in [the technical note](highway-flow-preparation-v01.md). This is kinematic replay, not sensor or physical qualification.

The final prepared configuration remains disabled, selected initializer/readiness null. Two proposed 30k-decision arms compare acceleration imitation 1 versus 5 only, keeping legacy rewards under the new reference; the 5,400-second shared ceiling includes 1,800 seconds of preparation and a 900-second final evaluation reserve. An independent warmup cursor preserves the first 18 single-AV PPO foundation cases, followed by varied participation. Candidate-only frozen gradients used matched 308-sample batches, the first 256 diagnostic rows and zero optimizer steps; this is not final initializer selection. Preparation cost of 978.186 seconds is retained. Reference qualification, persistent pre-maneuver gap identity, future internal-limit preview and the fixed 48-case initializer comparison remain open. No training or policy promotion.


### v21 — November 6: bounded approach ablations rejected

Two same-seed 80-second I-110 checks (39.282 seconds combined wall time) removed premature adjacent-lane braking and then recomputed future entry response. Both worsened main-car arrival from 49.3 to 63.2 s and introduced 12.3 s stopped exposure despite zero safety overrides. Original source/test files were restored exactly; 27 tests plus 3 subtests pass. [Failure evidence and speed plot](highway-flow-preparation-v01.md#focused-approach-ablation--november-6-2026) are retained with source hashes. Next action is a focused gap-window/longitudinal slot-alignment correction, not stronger imitation of the current reference. No training, weights, config or 3D render changes. Guide sync for this incremental note is pending.


### v22 — November 6: frozen GPUDrive transfer evaluated, rejected

Implemented a checksum-pinned CPU inference port with exact upstream-logit parity and a continuous-control SUMO bridge. One native baseline reached the route goal; the frozen pretrained main car oversped (45.81 m/s peak) and crossed the mapped road boundary at 13.4 s. No stable merge or goal was completed. This is unshielded cross-simulator transfer, not a native GPUDrive or PhysX benchmark. [Metrics, adapter limits and actual SimReady replay](gpudrive-i110-v01.md) preserve all evidence. Thirteen tests and 134-frame replay decode/composition checks pass; 0 optimizer updates. Existing policies remain unchanged. Main-car slot alignment remains open; any further pretrained-policy test first needs a justified speed-limit/map-goal adapter. Incremental guide synchronization pending.


### v23 — November 6: GPUDrive with native safety and 60 mph ceiling

[The guarded v02 transfer](gpudrive-i110-v02.md) uses the existing v05/SUMO safety stack through an explicit steering-to-lane adapter. Two matched 80-second runs preserve identical demand; native baseline merges, GPUDrive does not. No collisions, teleports or overspeed were recorded. GPUDrive stops for 71.9 s and requires changes on 92.25% of actions including adapter saturation (existing-stack metric91.125%); it fails the5% gate. Weights unchanged; no training. Seventeen focused tests pass; actual SimReady replay and metrics retained. Next action remains policy/adapter competence, not removing guards or treating this as an accepted policy. Guide sync pending.


### v24 — November 6: continuous GPUDrive steering repaired, route transfer still open

[The bounded v03 diagnostic](gpudrive-i110-v03.md) restores continuous steering without changing weights, retains a hard 60 mph/lower current limit and explicit longitudinal/footprint checks. Twenty-three tests plus four subtests pass. All 10 applied steering values equal actor requests; the car travels 8.269 m without stopping or overspeed, then reaches an unsupported route association at 1.1 s when it enters upstream sb1 from the ramp. An initial graph exception and the explicit termination retry are retained (1.437 s combined SUMO-loop wall time). No merge, policy acceptance, training or new render. Native SUMO safety equivalence is not claimed for external motion. The next GPUDrive decision would be deliberate continuous route reassociation; avoid further tuning campaigns until that scope is justified. Existing SUMO-native training priorities remain unchanged. Incremental guide synchronization pending.


### v25 — November 6: continuous route repair completes the short merge test

[GPUDrive v04](gpudrive-i110-v04.md) allows SUMO to associate the actual external pose with another road, then restores a connected route to the unchanged destination. One repair at 1.1 s removes the v03 blocker. The same-seed test meets 3-second mainline occupancy at 8.3 s and reaches its goal at 28.5 s, taking 3.859 s in the SUMO loop. No stops, overspeed, contact or route errors; all 284 steering requests preserved. Thirty tests plus 14 subtests pass. However, 255/284 acceleration requests change (89.79%), jerk peaks at 60 m/s³ and three background emergency-braking warnings remain: this is not policy qualification or promotion. No training or rendering. Next GPUDrive work is acceleration-contract compatibility/smoothness before acceptance; SUMO-native priorities remain unchanged. Metrics, logs, source snapshots and motion recording retained. Guide incremental sync pending.


### v26 — November 7: crossing audit and bounded longitudinal comfort filter

[GPUDrive v05](gpudrive-i110-v05.md) implements a network-connection audit, explicit acceleration-stage accounting, anticipatory following/speed targets and 3 m/s³ routine longitudinal jerk limiting before the unchanged final guard. One new same-seed CPU run reaches its goal at28.4 s without stops/contact/overspeed; peak/RMS longitudinal jerk improves60/15.00→3/1.17 m/s³. Forty tests plus16 subtests pass. However, the audit reveals that v04 and v05 leave the ramp early without a graph connection (32.23/33.41 m remaining). Prior occupancy/arrival evidence is not a verified proper merge. Raw command modification rises89.79→99.29% despite zero additional final-guard acceleration changes after conditioning, so this is controller comfort improvement, not policy qualification. All raw evidence/comparison metrics retained. Next action is the intended merge corridor and steering validation before further learning. No training or render. Incremental guide sync pending.


### v27 — November 7: plan to apply GPUDrive lessons to MAPPO

Saved [plan v01](mappo-gpudrive-lessons-plan-v01.md), anchored to the current flow117 preparation rather than reverting to the 93-field Stage1 actor. Transfer strict merge verification, command-stage accounting, anticipatory gap/speed alignment and measured comfort; do not transplant GPUDrive weights, steering or its heavy external filter. Keep native SUMO motion, current vehicle/safety limits, reference qualification, explicit ABI migration, paired initializer selection and the existing reviewed-training boundary. Proposed first screen is at most two 80-second CPU episodes with a shared 180-second wall ceiling, charged to the existing ledger. Preserve the planned two 30k-decision arms and 5,400-second shared ceiling; no fresh budget or readiness claim. Include actual curriculum-event coverage, communication/participation limits and quiet script-managed training after a future approved launch. Documentation-only milestone: no config/code/weights changes or simulator/training runs. Google guide incremental sync pending.


### v28 — November 8: revised plan, trained MAPPO, terminal review and restart

User authorized revising and launching training, checking only terminal results, then adjusting/restarting. Implemented a separate teacher-free flow117 environment/runner/contract with no imitation, near-limit teacher auxiliary or teacher-alignment reward; native guards and physical action limits unchanged. The failed reference-assisted study remains intact and disabled. After runtime/migration/finite-gradient/native-mode checks, two rounds collected 59,995 decisions (2,048 critic warmup) with actual I-110 merge PPO exposure. Initial→round1→round2 pooled interventions: 24.94→16.44→7.61%; stable targets 0→0→1 of one. Round2 follows a measured .01→.03 jerk-penalty revision with fresh optimizer; not a causal ablation. Zero evaluation collisions/teleports/overspeed, but three cases fail interventions and all four fail peak comfort. Both runs were left uninspected until completion. New runtime 375.969 s; inherited-plus-new ledger 1,394.654 s. Export/parity and 88 focused tests pass; final actor is published as experimental, not promoted. Last scheduled I-110 cruise row has only four decisions, so complete curriculum coverage is not claimed. [Results and artifacts](highway-flow-rl-v01.md). Guide incremental sync pending.


### v29 - November 8: SimReady video of final flow117 pilot

Recorded and rendered the same frozen round-two I-110 merge development case. All ten compared evaluation metrics exactly reproduce, including stable-merge details. The 25-second, normal-speed clip uses the SimReady main car with lightweight backgrounds. Decode, pose and immutable-source checks passed. No new training, sensors or physical validation; acceptance status and next diagnostic action are unchanged. [Video and evidence](highway-flow-rl-v01.md#november-8-frozen-policy-merge-video). Guide incremental sync pending.


### v30 - November 8: offline velocity and policy-command diagnosis

Analyzed all 399 measured focal steps from the saved video replay. Exact speed/acceleration identity and command-stage decomposition pass; policy/realized correlation is 0.6745 overall and 1.0 without interventions. Initial 25.2-to-14.1 mph drop comes from SUMO while the actor requests acceleration; 4.5 m minor-junction visibility is strongly indicated. Later policy braking accompanies growing leader gaps and a detour into an ending lane, including a 39.9-to-27.8 mph drop outside the 25-second clip. Corrected plots use the actual 18 m/s episode cap. Saved CSV, plots, hashes and source diagnosis. No simulator, learning or parameter changes. [Analysis](highway-flow-rl-velocity-analysis.md). Guide sync pending.


### v31 - November 8: staged ramp and policy improvement plan

Saved [flow117 v02 plan](highway-flow-rl-v02-plan.md). Separates frozen-actor network diagnosis, route-aware lane choices, cruise/comfort reward design, full-route validation and a future bounded training pilot. Requires conflicting-traffic counterexamples, preserves native safety modes and existing acceptance gates, and distinguishes the 60 mph ceiling from lower local/training limits. Proposed preparation and two conditional learning budgets are explicit; no automatic budget carryover or launch. Quiet terminal-only learning review remains the preference. Plan only; guide sync pending.


### v32 - November 8: v02 implementation and bounded preparation

Implemented the [v02 setup](highway-flow-rl-v02.md) with a versioned 100 m synthetic visibility candidate, bounded cruise reward and one-time optional-detour penalty. Original safety modes, masks and actor ABI remain unchanged. Four initial network allocations include a preserved logging failure; one charged recovery completes the occupied pair. Empty-approach native hard-brake ticks fall from 11 to 0; occupied braking remains, but full-route policy behavior regresses. 158 tests and zero-update native preflight pass. Five additional frozen development cases have zero collisions but fail quality gates. Preparation CPU cost is 115.734 seconds. Seven of eight additional allocations are used; the final slot is reserved for the authorization-refresh preflight. Full 54.6-second SimReady replay verified. Training is disabled for review of the 30,000-decision / 900-second pilot. No promotion or held-out evaluation. Guide synchronization pending.


### v33 - November 8: adjacent native-car speed comparison

Analyzed the saved v02 replay without running SUMO or training. Geometry selects native IDM car main_8 alongside agent_0 at 3.7 s. Over 3.7-12.9 s, native speed increases 33.42 to 37.07 mph while MAPPO ends at 26.80 mph after a 20.76 mph dip; mean absolute jerk is 0.045 versus 1.176 m/s3. Saved raw traces, unsmoothed plots, metrics and provenance with finite/cadence/kinematic checks. Different starts, route duties and driver profiles prevent a causal matched-policy claim. [Evidence](highway-flow-rl-v02.md#november-8---speed-comparison-with-adjacent-native-traffic). Configuration review remains next; training disabled and guide synchronization pending.


### v34 - November 8: researched reward revision and scale audit

Two sub-agents reviewed primary AV-RL sources and actual reward code/journals. [Plan](highway-flow-rl-reward-revision-plan.md) addresses cruise eligibility before coefficients, a context-dependent speed target, normalized Huber jerk and a stronger optional-ending-lane penalty. The saved focal audit finds zero cruise payment in both speed dips, including nine rows excluded solely by its own braking; 69/545 decisions receive the existing addition. One-time merge/pass guards are present; the asymmetric action mapping is preserved, not declared causal. Candidate coefficients are unvalidated and the exact target helper needs counterexample qualification. No implementation, configuration, policy, simulator or training changes. Existing budget retained with an explicit evaluation-reserve reallocation proposal; terminal-only review remains the preference. Guide synchronization pending.


### v35 - November 9: review-ready highway reward weights

[Plan v03](highway-smooth-reward-plan-v03.md) records the actual flow117 baseline, signed candidate reward scales, Huber examples and pre-action safe-target eligibility. It adds sequence-level reward qualification and a proposed avoidable-braking classifier whose coverage must be validated. Safety constraints, held-out separation, complete-route metrics and the existing ledger remain. No code/configuration/actor changes, simulator runs or training. Next: user review, offline target/reward qualification and presentation of the resolved configuration before learning. Google guide incremental sync pending.


### v36 - November 9: implemented rewards and offline trace checks

[Flow RL v03](highway-flow-rl-v03.md) implements the reviewed pre-action target,
speed-error 0.30, Huber jerk 0.09 (2 m/s3 scale), and detour -2.0. The legacy
jerk coefficient is zero to prevent double charging. 89 focused tests pass;
18,492 saved development decisions rescored with 95.55% qualified target
coverage (72.66% for the I-110 focal car). Later focal dip: 14 flagged
avoidable-braking decisions, one persistent event; first tight-gap dip: none.
Both audits, per-car CSVs and hashes retained. No new simulator run, training,
held-out evaluation or performance promotion. Shared costs total 121.328 s.
Next: final config review, then fresh exact-source native preflight before any
authorized pilot. Google guide incremental sync remains pending.


### v37 - November 9: authorized v03 pilot completed and rejected

Fresh native zero-update preflight passed, then 29,999 decisions / 18 updates /
444 optimizer steps completed without a runtime error in 294.859 seconds.
No analysis or metric inspection while learning ran. Six matched development
cases retain zero collisions/teleports; pooled interventions worsen to 10.21%,
peak jerk to 59.73 m/s3, stable merges to 1/2. All six case gates fail. Focal
I-110 avoidable braking and optional detour disappear, but interventions rise.
Actor/checkpoint/raw metrics saved, candidate not promoted, held-out and second
round untouched. Shared costs total 418.687 s. Next: saved-trace failure analysis
before a new proposal. [Evidence](highway-flow-rl-v03.md#round-one-training-result).
Guide synchronization pending.


### v38 - November 9: longer continuation authorized and preflighted

User requested more learning and epochs. Prepared a separate strict extended
profile (300,000 decisions, eight epochs, 90 minutes) starting from the rejected
pilot weights with a fresh optimizer. Same rewards and safety. 95 tests plus
fresh native zero-update preflight pass. Shared ledger retained; old pilot
limits preserved; extended study cap explicitly raised to 6,600 s. Review only
terminal results; no third round or held-out test. Guide sync remains pending.


### v39 - November 9: extended training completed after spawn repair

The longer continuation finished 298,400 actual learning decisions and 177
updates (7,130 optimizer steps); 1,600 failed reserved decisions remain charged.
A ramp insertion failure was corrected with an assignment-independent stopping
bound. Explicit migration preserved model, optimizer and RNG; native failing-seed
preflight passed and the one changed development baseline was rerun. 127 CPU
tests pass. Fleet interventions improve 10.50% to 4.33%, stable merges 1/2 to 2/2,
with zero collisions/teleports; all six per-case gates still fail and focal
interventions remain 32.79%. Source/metric/checkpoint history preserved. Study
costs total 2,169.579 s. No promotion, held-out run or third round. Next: saved
focal override/speed diagnosis. [Evidence](highway-flow-rl-v03.md#longer-continuation-result-and-spawn-repair). Guide sync pending.


### v40 - November 9: comprehensive saved-trace policy audit

Audited rewards and their actual contributions, target coverage, masks, native
execution, rollout population/time coupling, observations/messages, PPO/GAE,
gradients, checkpoint identity and acceptance metrics. The focal car attempts
every permitted left merge but receives no such permission until near-stop;
most speed-target reward is absent during gap formation. Retained full traces,
source/input hashes and compact audit evidence. Actor replay and arithmetic
consistency checks pass; about 2.9 s offline computation, zero new simulator
episodes or optimizer steps. Prioritize gap-formation guidance and complete
merge experience, then controlled weight/credit-horizon comparisons. Candidate
remains rejected. [Audit](highway-policy-audit-v01.md). Guide sync pending.


### v41 - November 10: implemented merge-focused v04 and authorized long run

Added gap preparation potential/speed targets, +5 gated rolling-quality bonus,
full-maneuver projected masks, complete physical-time rollout reservations and
merge-focused curriculum rows. Native guards remain enabled. 116 tests pass;
zero-update preflight and seeded trace repeatability pass after a recorded
serialization fix. Full frozen-actor merge completes but still stops 11.7 s;
quality gate rejects it. User authorized long training: 4M additional decisions,
eight-hour cap, 20 maximum epochs, automatic evaluation/checkpoint retention.
[Resolved configuration, evidence and unattended command](highway-merge-rewards-v04.md).
Training result pending; no promotion or held-out run. Guide sync pending.


### v42 - November 10: v04 training completed and terminal metrics reviewed

Normal wall-budget completion; final interrupted reservation preserved and
charged. Main-car merge stopping/interventions are eliminated on the development
case, with improved average smoothness and speed. Strict peak-jerk, rolling
quality and a few per-car checks still fail. Saved final and selected-best
checkpoint metrics/checksums and confirmed the best full checkpoint matches its
retained candidate. No automatic acceptance, deployment or further training.
[Terminal evidence](highway-merge-rewards-v04.md#completed-long-run---november-10-2026).
Next: saved-trace comfort/failure diagnosis. Guide sync pending.


### v43 - November 11: v05 peak-jerk diagnosis from saved traces

New step, handled November 11. Startup handoff explains 11/16 jerk-failing cars;
the 50.76 m/s3 maximum is unattributed native SUMO braking and release on
`mlk_on_0`, not the actor. Policy-owned peaks are at most 8.46 m/s3. A request
rate limiter alone cannot clear the gate. No simulation or training.
[Diagnosis](highway-jerk-diagnosis-v05.md). Next: agree on the startup jerk
rule and investigate the native braking. Guide sync pending.

### v44 - November 11: native ramp braking explained

The 50.76 m/s3 peak is SUMO's minor-link approach rule: a ramp car on a yield
link must stay able to stop until within 4.5 m visibility, with no conflicting
car present. Exact open-loop replay (39/39 steps). Clearing only the
right-of-way bit removes it. Shared by all six ramp networks in the v05 bank.
Options are recorded for decision; nothing implemented, no training.
[Explanation](highway-jerk-diagnosis-v05.md#native-braking-on-mlk_on_0-explained-november-11-2026).

### v45 - November 11: ramp-started acceleration lane networks

User decision: ramp cars should merge from an acceleration lane that the ramp
itself becomes, not yield at the gore. Built corrected copies of all six ramp
networks with a parallel final approach; the ramp link is priority with no
foes. Replay of the saved commands: no SUMO braking (lowest 0.0 m/s2, was
-4.5). Only the ramp legal speed cap remains, an open-loop artifact. No
closed-loop run or training; needs sb1 lane-ID integration first.
[Details](highway-jerk-diagnosis-v05.md#corrected-ramp-networks-acceleration-lane-starts-at-the-ramp-november-11-2026).

### v46 - November 11: frozen actor on ramp-started networks

Ramp-started networks need no environment code change; the lane-identity
regression test was added. The frozen v04 actor ran on six layouts,
historical vs corrected, with one seed each: on the corrected networks,
overrides were 0% and peak jerk about 12 m/s3, the startup handoff only. All
cases merged with no collisions, but the gates still fail (startup jerk,
merge quality 46%). Generalization to other merge designs is not
demonstrated: held-out rows are seeds, not geometry. No training.
[Details](highway-jerk-diagnosis-v05.md#frozen-actor-on-ramp-started-networks-november-11-2026).

### v47 - November 11: held-out merge geometry bank

The project's first held-out geometry: eight frozen ramp-started merge layouts
outside the trained family (short and long acceleration lanes, six lanes,
strong curve, 45 mph, wide lanes, short ramp, fast ramp). Structural checks
pass, and the environment runs all of them natively without the policy.
Unsupported real designs are listed. The policy has not been evaluated; do
not tune on this bank. [Details](highway-jerk-diagnosis-v05.md#held-out-merge-geometry-bank-november-11-2026).

### v48 - November 11: startup handoff ramp

Opt-in `startup` mode: a 4 m/s3 comfort ramp only during control handoff;
max braking and safety ceilings bypass it. The draft's conditioning hook is now
committed. Frozen v04 actor: peak jerk from about 12.5 to 4.0 m/s3 on 5/6
ramp-started layouts, with all per-car checks passing. One layout regressed: a
borderline projected-gap mask closed, and the actor rode the acceleration lane
to its end. That is a fragile merge decision, needing a training change.
Merge quality still fails everywhere. The dev-case run was blocked by the
episode budget. No training. [Details](highway-jerk-diagnosis-v05.md#startup-handoff-ramp-november-11-2026).

### v49 - November 13: startup mode on development cases

The diagnostic episode cap was raised from 60 to 80 by the user. Frozen v04
actor with the startup handoff on the six v04 development cases: 2/6 cases
pass (v04 final 0/6), and failing cars fell from 18 to 7, with no collisions.
Remaining: historical gore braking, I-110 mid-episode jerk, merge quality.
No training. [Details](highway-jerk-diagnosis-v05.md#startup-mode-on-the-v04-development-cases-november-13-2026).

### v50 - November 13: remaining jerk peaks explained

The 10 peaks in the startup dev run come from: gore yield (historical network,
4); SUMO ACC law vs actor guard (2); a leader leaving at the road end (1); and
policy step reactions to switching merge inputs (2), which the training reward
barely penalizes. Fixes are recorded for decision (network, environment,
evaluation rule, training). [Details](highway-jerk-diagnosis-v05.md#remaining-jerk-peaks-explained-november-13-2026).
