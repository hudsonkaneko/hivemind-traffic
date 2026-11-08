# Change log

This is a curated guide to meaningful changes, not a replacement for Git history.
Dates below use America/Los_Angeles. Existing experiment reports remain the
authority for historical measurements and failures.

## 2026-10-26 - MAPPO cruise retraining passes the override gate

- Diagnosed all prior learned overrides as speed-ceiling clamps. Added a separate
  25 m/s cruise-reward configuration with stronger override penalties and lower
  exploration; retained the original safety checks and 5% acceptance threshold.
- Retrained for one million decisions. All 20 fresh held-out episodes passed:
  zero collisions, overspeed or overrides, with 1,445.453 m mean progress per car.
  Preserve the speed/progress tradeoff and different seed-set qualification in
  [the v02 training record](sumo-mappo-v2.md). Google guide sync remains pending.
## October 26, 2026: Near-field sensing and perception preparation

- Versioned local source v12-v14 preserves seven body cameras and adds four compact perimeter LiDARs. Independent writer registrations support five concurrent scanners.
- Final bounded native test detects all 12 small near-field targets and six tall targets; 595 complete scans and 59 images per camera, stable mounts and zero contacts. Earlier coverage failures are preserved.
- Prepared and validated 60 labeled frames in six isolated scenes (40/10/10 split), with hashed RGB/masks/raw scans, transforms and explicit evaluation-only truth. Pedestrian examples and full training diversity remain missing.
- 141 focused CPU tests, 28 USD tests and four additional capture-config tests pass. Actual screenshots, source hashes and limits: [perception preparation v01](perception-preparation-v01.md).
- Proposed segmentation config is disabled; no training or weights download. Google guide and generated Markdown synchronized through verified native readback.

## 2026-10-26 - Body-mounted seven-camera source v11

- Removed camera roof stalks and seated compact housings against measured body
  surfaces on the fascia/hood, fenders, rear quarter panels and tailgate.
  Retained central roof LiDAR and historical source versions.
- Corrected cross-view body occlusion after inspecting actual images. Combined
  capture passed with 59 images per camera, 119 scans and stable moving mounts.
  Source checks and two exterior-intersection tests pass. No training started.
- See [body-camera results and screenshots](body-camera-v01.md).

## 2026-10-26 - Roof LiDAR v10 and combined capture

- Renamed the new source mount to LidarRoof and raised it 20 cm above the prior
  roof location. Preserved earlier versions and migrated v10 paths/metadata.
- Combined seven-camera/LiDAR test passed: 119 scans, 59 images per camera,
  all six tall surrounding targets, stable mounts and no contacts. 94 CPU and
  22 USD tests pass. Small side targets at 2–6 m were missed and are documented.
- Saved actual screenshots and synchronized the guide. No training or actor
  changes. See [roof LiDAR results](roof-lidar-v01.md).

## 2026-10-26 - Seven-camera source rig

- Added seven source-authored perspective cameras to local vehicle v09 while
  preserving v08, the LiDAR path and vehicle dynamics. Fixed RTX mount handling.
- Passed 44 capture tests, 21 USD tests and a six-second native test: 59 valid
  frames per camera, all target views, 0.63 m travel, zero contacts, unchanged
  source and stable mounts. Saved actual screenshots and all failed attempts.
- No learned surround perception or new training. See [results](seven-camera-v01.md).

## 2026-10-26 - Repair native SUMO GUI recording

- Added paced, native-size SUMO viewport capture with bounded socket/process
  cleanup after reproducing the unpaced stall. Headless training is unchanged.
- Recorded and decoded 600 native frames into a 60-second MP4; all vehicle
  trajectories exactly match the prior headless replay. All 26 targeted tests
  pass. No training ran. See [native capture evidence](sumo-mappo-recording.md).

## 2026-10-25 - Forward sensor pipeline and reusable v08 source rig

- Audited exact actor inputs and privileged control shortcuts across SUMO and
  physical-car policies. Existing checkpoint contracts remain unchanged.
- Added source-authored camera/LiDAR mounts in the local v08 vehicle package,
  RGB lane evidence, noisy ego estimation, validated sensor packet boundaries
  and a bounded scripted native probe with fail-closed camera fault injection.
- Final native validation passed: 8.52 m travel, 2.57 m obstacle clearance,
  no contacts or lane departures, and blank-camera braking/recovery. Fixed source
  camera schema/pitch, capture startup and brake-release creep; preserved failed
  runs and disclosed the revised 3 m stopping gate. 163 CPU and 18 USD tests pass.
  Published real screenshots and synchronized the guide; no training started. See
  [the sensor milestone](sensor-perception-v01.md) and [input audit](sensor-input-audit.md).

## 2026-10-25 - Repair SUMO MAPPO delayed spawn before checkpoint resume

- Preserve safe SUMO insertion delays with bounded native warm-up rather than
  requiring all cars in the first step. The failed training seed 100277 now
  resets reproducibly in two steps; 37 targeted tests pass.
- Preserve the failed 659,456-decision run and checkpoints. Resume is authorized
  under the unchanged one-million-decision configuration. See the
  [training failure and repair record](sumo-mappo-setup.md).

## 2026-10-25 - SUMO MAPPO v01 setup, training awaiting approval

- Added a separate four-car fixed-lane SUMO environment, local 27-feature shared
  actor, centralized training critic and acceleration-only MAPPO learner with
  terminal-aware rollout collection and actual active-agent budget accounting.
- Added a reviewed 60 mph / one-million-decision CPU proposal, explicit config
  digest launch guard, isolated workers, quiet file logging, resumable atomic
  checkpoints and disjoint validation/test partitions. No driving training ran.
- Verified a zero-optimizer native preflight, exact repeat/export checks and
  90 final targeted tests; broad regression passed 2,724 tests with 215 skips.
  The scripted baseline's 9.25% override rate fails the unchanged 5% criterion.
- Synchronized the Google guide and its complete Markdown mirror. See the
  [configuration, evidence and physical-transfer limits](sumo-mappo-setup.md).

## 2026-10-25 - Prepared CIRCLES highway test with PPO lane traffic

- Added a separate three-physical-car inference probe with a 60 mph ceiling,
  CIRCLES longitudinal control on the main car and two frozen PPO lane actors
  retaining their 3 m/s trained target. No new training is configured.
- Preserved a zero-step native failure and repaired scene export without
  invalidating live vehicle bindings. All 55 selected CPU/OpenUSD checks pass;
  the user deferred the repaired native run, so driving results are pending.
- Published the required fleet helpers and isolated binding/composition tests;
  unrelated pair/endurance work remains local. Updated the Google guide and
  verified its complete Markdown mirror. See [the prepared protocol and launch
  instructions](highway-circles-60mph.md).

## 2026-10-22 - CIRCLES controller actuation smoke

- Tested seven pinned public ONNX exports on the existing physical-car loop task
  with native state inputs, existing steering and an explicit low-speed pedal
  adapter. Six passed the limited actuation check; strict stopping success was
  0/7. One faster export contacted the obstacle. No training ran.
- Preserved the original log-classification failure and confirmed all seven cases
  in a fresh bounded run after fixing output chronology. Added two adapter tests,
  source/model hashes, local raw traces and compact published measurements.
- Updated the Google lane-task guide and verified the generated Markdown mirror.
  See [the control smoke record](circles-control-smoke.md) for scope and rerun.

## 2026-10-22 - Google engineering guide mirrored on GitHub

- Converted the complete 42-tab Google guide to `engineering-guide.md`, preserving
  14 tables, 182 command blocks, screenshot hyperlinks and historical wording.
- Added a document/revision/tab/hash manifest, conflict-safe refresh script,
  README links and persistent instructions to synchronize future chat-driven
  guide updates with the existing native document. Added a reciprocal GitHub link
  in its start tab; no background watcher or external-edit auto-sync is claimed.
- Verified exact native note text and preservation of the other 41 tab bodies.
  Ten targeted tests pass; GitHub-rendered output retains all 1,742 nonempty source
  paragraphs and all tab anchors. See `learning-guide-sync.md` for the procedure.

## 2026-10-22 - Hazard training recovery and full-scene integration (v02/v03)

- Fixed the native inference-tensor reset boundary and resumed the saved RSL
  optimizer. The isolated curriculum passed all three 10-case development gates
  after 98,304 cumulative PPO transitions plus an explicit imitation warm start.
- Retained the failed full-scene policy transfer (lane departure after 6.659 m).
  Added full-highway training with the actual twelve background-car inputs,
  independently replayed observations and seeded scenario compatibility checks.
  Four scripted full-scene cases and a continuous 180 m baseline passed.
- Verified a separate two-physical-car foundation: independent actuation/reset,
  repeatable motion, support/containment and deliberate car-to-car contacts.
  Learned main-car integration remains the first acceptance target; expansion
  follows it. Full-scene training and held-out policy acceptance are in progress.
- Preserved failed attempts and source snapshots. See
  [the versioned record](rsl-road-hazard-integration.md) for scope and evidence.

## 2026-10-22 - RSL-RL native road-hazard foundation (v01)

- Added a separate 42-observation task, collidable seeded crossing objects,
  shared RSL inference adapter and bounded curriculum runner. Existing lane
  and full-showcase behavior remains preserved.
- Four scripted native probes and 40 demonstration episodes passed independent
  geometry/contact checks. A deliberate non-braking control triggered a native
  contact. The original RSL lane policy evaluated at 0/15 successes (timeouts).
- Recorded versioned configuration, training/evaluation boundaries and compact
  baseline evidence in [the integration record](rsl-road-hazard-integration.md).
  Learned hazard behavior and full-road integration remain in progress.

## 2026-10-19 - Approved first lane PPO pilot

- Added the guarded PPO runner, post-update checkpoints, immutable evidence and
  fixed checkpoint-selection/held-out evaluation. The user approved one seed-2101
  run; it completed exactly 65,536 transitions and 5,120 optimizer steps.
- Native zero-optimizer compatibility checks passed. All 31 complete training
  episodes timed out. The fixed selection rule chose update 24; held-out PPO
  achieved 0/15 successful drive-and-stop episodes against scripted control's
  15/15. Mean progress was 0.040 m versus 59.877 m. No further training ran.
- Published the complete selection/paired comparison, compact hash evidence,
  measured-outcome plot and original native screenshots. The frozen illustrative
  replay is excluded from evaluation totals and performed zero optimizer steps.
- Full local validation passed 2,292 tests, 187 optional skips and 116 subtests;
  subsequent preview-only checks passed 48 tests and chart checks passed five.
  Independent agents audited training, all evaluation packages and native capture
  completion. See the [pilot record](lab-lane-ppo-pilot.md) for results and limits.

## 2026-10-19 - Lane-task guide synchronization

- Added the lane task, original screenshot links and unapproved training proposal
  to a new engineering-guide tab. All 191 native verification checks passed;
  the 41 existing tabs remain unchanged. The [sync record](lab-lane-following.md#engineering-guide-synchronization-and-github-checkpoint)
  includes source checkpoint, readback hashes and scope.
- Implementation checkpoint `4166234` passed both Windows CPU/SUMO GitHub jobs.
  Training remains unapproved with zero optimizer steps.

## 2026-10-19 - Native Isaac Lab lane-following task

- Added a separate one-car DirectRLEnv with seeded native resets, 22 privileged
  observations, steering/throttle/brake actions, time-scaled rewards and explicit
  failure/success/timeout behavior. Existing SUMO and compatibility demos remain
  available. The task cruises at 3 m/s and stops at station 60 m.
- Preserved referenced vehicle/road assets and named scene layers. Fixed the
  installed Lab configuration boundary, enabled and verified native contact
  reporting with cleanup, and corrected the project-local SB3 terminal snapshot
  before automatic reset. Installed runtimes were not modified.
- The frozen 12-package matrix passed: 13 successful drive/stop episodes and
  three expected collision, departure and timeout diagnostics. All three fresh
  process repeats had zero measured position/speed/yaw differences. Worst driving
  lateral RMS was 0.043482 m; every successful stop held for a measured second.
- Preserved construction, contact-reporting and process-isolation failures,
  superseded attempts and serial reruns. Independent raw-trace/hash checks and
  original viewport/hierarchy/layer screenshots are linked in the
  [implementation and evidence note](lab-lane-following.md).
- Full CPU regression passed 2,037 tests and 116 subtests, with 187 optional
  runtime skips. Native validation and scene portability are recorded separately.
- Prepared a concrete [PPO training proposal](lab-lane-following-training-proposal.md):
  one route/environment, 65,536 control transitions, fixed selection and paired
  held-out evaluation. It remains UNAPPROVED; no policy or optimizer was created
  and no training was launched. Measured scripted throughput is not training
  throughput or highway/sensor validation.

## 2026-10-19 - Engineering-guide synchronization

- Appended the validated native results, six screenshot links and matched
  benchmark findings to the existing engineering-guide tab. All 207 native
  checks passed; existing content, styles and all 41 tab identities/order remain
  preserved. Record exact scope and local evidence hashes in the
  [synchronization note](showcase-validation.md#october-19-engineering-guide-synchronization).
- Both Windows CPU/SUMO clean-clone checks passed for implementation checkpoint
  `7940a9757356fa2dcd6790cc8c1769e0d0ba1fbc`. Dense strict timing remains open.

## 2026-10-19 - Matched display matrix and evaluator correction

- Completed all 11 declared native cases on one frozen source. All physical,
  scene and cleanup checks passed. Only the two light-traffic cases passed
  strict display timing; all nine dense variants failed their unchanged limits.
- Repaired a comparator mismatch caused solely by the public scene's correct
  per-run assembly directory. Validate the exact owning run before normalizing
  that field; asset hashes, physics, geometry, motion and timing remain strict.
- Preserved the original rejected comparison package. A separately versioned
  reassessment accepts all eight matched pairs with zero measured motion
  differences; no native captures were rewritten or rerun for this correction.
- Full CPU validation passed 1,767 tests and 116 subtests, with 181 optional
  runtime skips. Independent review verified the repair, input hashes and all
  timing outcomes. The [matrix report](showcase-benchmark-20261019.md) records
  directional observations and the limits of single samples.
- Dense driving has sustained lag and recurring mid-run stalls. Lower display
  cadence and the minimal app improve observed wall time but do not grant
  strict qualification. Targeted timing diagnosis is the next performance step.

## 2026-10-19 - Bounded gap creation and repeated endpoint acceptance

- Added a v09 slowdown/recovery supervisor with a 21-second attempt limit,
  50 m added-lag budget and measured rear/tracking guards. The ordinary planner
  retains its full-cruise forecast, front safety and lane-change authority.
- The original 180-second endpoint and repeat passed all 33 gates, each with
  11 passes and five lane changes. The settled tail added eight passes and four
  lane changes. The 90-second legacy live-control regression also passed.
- Full CPU validation passed 1,750 tests and 116 subtests, with 181 optional
  runtime skips. Independent review covered stop/tracking handoffs and durable
  failure diagnostics before native execution.
- Preserved earlier failures and added a sixth original screenshot, bounded
  controller design and native evidence to the [October 19 report](showcase-validation-20261019.md)
  and [gap-creation note](showcase-gap-creation.md).
- Strict display timing still fails. The matched 11-case matrix is the next
  measurement; physical acceptance does not grant real-time qualification.

## 2026-10-19 — Native validation and remote traffic admission

- Three fresh public-car startup smokes passed and produced matching viewport
  captures. The native 90-second live-control regression also passed.
- Fixed dormant slots that could remain permanently inside the protected entry
  region. Entries now use deterministic safe remote gaps when needed, with
  40 m front/rear spacing and exact 0.8 m footprint clearance. Active cars,
  static assets, 160 m ego exclusion and 30-tick transition limits are preserved.
- The repaired endpoint reached all four declared targets (4/80/12/80), but
  subsequent passing still failed. Both failed endpoint attempts are retained;
  a separate controller improvement remains in progress.
- Verification: 1,724 CPU tests and 100 subtests passed, 181 optional skips;
  two OpenUSD adapter tests passed and independent review found no blocker.
  Strict display qualification remains open; the matched matrix is pending.
- Added actual unmodified screenshots, source hashes and measured evidence in
  the [October 19 report](showcase-validation-20261019.md).

## 2026-10-16 — Showcase validation implementation

- Added bounded native window/viewport/sidebar readiness with retained failure
  evidence, plus an opt-in live endpoint, rapid-request, pause/resume, transition
  and subsequent-passing validation scenario.
- Added measured drive-only gap/headway, acceleration/jerk, near-miss proxies,
  per-lifetime time loss and exposure-qualified fairness. Undefined quantities
  remain null; laps are not completed trips.
- Added eleven serial public-car benchmark cases and eight matched comparisons
  for panel, display, recording and editor costs, with source/evidence integrity,
  physical-equivalence checks and separate strict display qualification.
- Frozen-source regression: 1,712 tests passed, 180 optional skips, 98 subtests;
  independent agent review found no remaining actionable defects. Native runs
  are blocked by competing simulator work; no new runtime or performance pass
  is claimed. Existing demonstrations and private notes remain preserved.
- See [implementation, protocol and evidence](showcase-validation.md).

## 2026-10-15 — Clean-clone readiness

- Added an explicit public procedural PhysX car option, leaving the prepared
  Americano default and private assets unchanged. Setup checks now reject missing
  required inputs instead of printing success with an unavailable vehicle.
- Added Windows/Python 3.11 CPU/SUMO CI, matching binary dependencies, venv-local
  SUMO discovery, clone-relative contributor instructions and a fresh setup guide.
- Removed pytest's shared/deleting temporary directory and an accidental test
  assumption that temporary modules always lived inside this checkout. Published
  highway bytes are protected from Git line-ending conversion.
- See [clean-clone evidence and limits](clean-clone.md) for actual validation
  outcomes, including retained failed attempts. No runtime upgrade or training.

## 2026-10-14 — Main integration checkpoint

- Joined the published collector-v02 and showcase histories for the requested
  main promotion; retained both sides of documentation conflicts and all commits.
- Fresh checks: 1,599 general tests passed, 180 optional skips, 34 subtests;
  134 USD checks, four route checks and v09 launch preflight passed.
- No simulation/asset changes, installation, new runtime benchmark or training.
  Private local notes/assets remain excluded; existing acceptance limits remain.
- See [review, invocation failures and merge scope](main-integration-2026-10-14.md)
  and PR 10 for the GitHub merge outcome.

## 2026-10-12 — Repository-local project skills

- Added eight domain skills under `.agents/skills`, their UI metadata and two
  reusable vehicle-preparation references, with source hashes and usage guidance.
- Scoped older SUMO-first instructions to the applicable baseline; current PhysX
  ownership, local-only asset boundaries and user-led training gates remain.
- Excluded private asset case-study history and bundled/unrelated personal skills.
  Installed skill originals, simulation code, assets and results are unchanged.
- See [inventory, validation and publication scope](project-skills.md).
- Six bundle checks and all eight skill-creator validations passed. Google guide
  tab 38 synced and native-readback verified; all 38 older tabs preserved.

## 2026-10-12 — Native sidebar and live traffic tuning (v08/v09)

- Added a docked regular-editor sidebar with live viewing controls, validated
  settings and 26 disabled, explicitly display-only future sliders. Preserved
  v08's safe-stop/restart workflow as a separate milestone.
- v09 adds live 4–80 background-car targets, bounded speed/following-gap ramps,
  remote rate-limited entry/exit, generation-aware identity and an 80-slot native
  pool without rebuilding USD or physics bindings. Rendering settings still
  restart explicitly. Existing versions, source assets and results are preserved.
- Two native live-change runs passed and repeated exactly across 12,240 physics
  samples; population reached 80→12→40→12 without relaunch. Final general suite:
  1,593 passed, 180 optional skips, 18 subtests; separate USD suites: 122 passed.
  GUI docking/interaction were checked separately. Smoothness and exhaustive
  parameter combinations remain unqualified; failed trials are preserved.
- See [implementation, evidence and limits](showcase-live-controls.md). No
  training, dependency installation, SUMO migration or sensor changes.
- Google guide tab **37 Live showcase controls** synced and native-readback
  verified, including command styling; all 37 older tab bodies unchanged.

## 2026-10-10 — Regular editor UI with v07 optimizations

- Made the regular Isaac Sim editor the v07 default at the user's request;
  the minimal interface remains available with `--ui-profile showcase`.
  Retained visual instancing, cached updates, 5 Hz statistics and every
  physical/control/safety setting. Earlier versions and saved configs remain valid.
- Verified 247 targeted tests and a default-editor paced 90-second driving run,
  `20261011T032144Z-ca68b556`: 16 passes, 8 lane changes, no contacts, safe
  stop/hold, editor panels enabled and clean source/process checks.
- Regular UI measured 0.9513x real time and 28.54 renderer notifications/s;
  strict smoothness remains open. See [evidence](showcase-responsiveness.md).
  No claim of unchanged FPS or measured click latency; no runtime installation.
- Updated portable commands and the existing v07 milestone, not a new physical
  version. Google guide tab 36 synced and verified; command styling and all
  36 older tabs preserved.

## 2026-10-10 — Instanced v07 responsive showcase

- Added a project-owned single-viewport Kit experience, immutable OpenUSD
  visual instancing, cached camera/path metadata, and 5 Hz statistics updates.
  All 80 background colliders, editable wheel pivots, 120 Hz physics/safety,
  60 Hz controls and 10 Hz planning remain; prior versions/assets are preserved.
- Added native application/delivered-frame timing and opt-in bounded CPU/GPU
  diagnosis, with explicit exclusion of profiling/paused runs from qualification.
- Two 30-Hz GUI drives held real-time speed with exact 30,240-sample physical
  repeats (46 passes, 23 changes, zero contacts). Rare 126–128 ms intervals keep
  strict smoothness open; 60 Hz remained below real time at about 50 completion
  notifications/s. See [v07 responsiveness](showcase-responsiveness.md).
- Final general suite: 1,575 passed, 179 optional skips, 18 subtests; 115 USD
  fleet/view checks passed. Review fixes also exclude headless/paused/profiling
  runs from GUI qualification and keep profiling bounded while paused.
- Full evidence, command-expiry braking, blocked following, contact-positive
  detection and the optional editor launch passed. Google guide tab **36
  Responsive showcase** is synced with native readback and all older tabs intact.
- No training, runtime installation, persistent settings or raw-asset publication.

## 2026-10-10 — Profiled v06 showcase presentation

- Added exclusive phase timings, shared native fleet snapshots, single-pass
  camera/path updates, distant decorative-wheel reduction and exact-math reuse.
  Full-rate physics, controls, safety guards and all 80 colliders remain intact.
- Added explicit presentation/full evidence profiles and opt-in display rates.
  Presentation samples optional audits and buffers records; full mode keeps
  every-tick/every-display audits and durable checkpoints. v04 stays default.
- Two full GUI drives repeated exactly across 30,240 samples: 46 passes,
  23 changes, zero contacts, safe braking/hold and clean shutdown. Full evidence
  and command-loss, blocked-lane and contact-positive checks passed separately.
- Strict smooth playback remains unqualified despite approximately 1x average
  speed: lag and maximum frame intervals exceeded unchanged gates. Retained
  timing failures, competing-GPU run and short blocked-fixture failure.
- Verified 1,483 general tests (140 optional skips, 18 subtests) and 76 USD
  fleet/view tests. See [performance evidence](showcase-performance.md) and v06.
  Google guide tab **35 Showcase performance** added with native readback;
  all 35 older tabs preserved. No training, installation or asset publication.

## 2026-10-10 — Denser 80-car loop showcase

- Added opt-in v05 with 80 slower block cars in staggered pairs, keeping open
  passing opportunities and preserving v04's lighter twelve-car default.
- Full GUI physical/visual test passed: 46 overtakes, 23 lane changes, zero
  contacts/overlaps, 1.5693 m minimum clearance and a safe final stop/hold.
  The 252 simulated seconds took 335.4184 active wall seconds (0.7513x).
  **v05 is not real-time qualified**; physics and control rates are unchanged.
- Added exact clearance pruning, batched referenced visuals, native state
  readback and explicit fleet rundown. A process-local legacy renderer fallback
  avoids a dense-stage crash; the vendor root cause remains unproven. Failed
  experiments and original vehicle/road assets are preserved.
- Isolated an unused sensor-icon stage-close callback after a repeated shutdown
  crash. Two fresh 90-second runs then passed and matched exactly across 12,240
  physical samples and 1,020 visual frames; failed attempts remain recorded.
- Verified 1,384 CPU tests (119 optional skips, 18 subtests) and 100 USD tests.
  Final command-loss, blocked-lane and intentional-contact checks passed.
  See [dense-showcase evidence](dense-loop-showcase.md) and version folder v05.
- Synced Google guide tab **34 Loop traffic showcase** with v04/v05 explanations,
  highlighted definitions and portable command blocks; older tabs preserved.
  No dependency installation, training, roadmap replacement or asset publication.

## 2026-10-09 — Versioned loop traffic showcase

- Integrated the supplied Americano v07 as a native-PhysX main car with twelve
  slower referenced block cars, scripted moving-object passing, follow/overhead
  cameras, cyan planned path and live status controls. Sources stay unchanged
  and local-only; Git contains generic code and versioned milestone notes.
- Qualified the corrected v04 GUI: 252 simulated seconds in 252.0003424 live
  wall seconds, twelve complete passes, five lane changes, zero contacts,
  1.5808 m minimum clearance and safe final braking/hold. Clean native shutdown.
- Two fresh 110-second drives matched exactly across 14,640 physical samples,
  with complete visual/native pose checks and source/evidence hash verification.
- Fixed stale kinematic visuals using measured-pose render proxies, normalized
  display rotations, and a guarded process-local workaround for an unused
  scripting extension's stage-close crash. Failed attempts are preserved and
  their historical physical-versus-visual scope is explicitly corrected.
- Verified 1,333 CPU tests (92 optional tests skipped; 15 subtests passed) and
  67 USD tests. See [showcase evidence and commands](loop-traffic-showcase.md).
  Existing experiments and the research roadmap remain intact. Isaac Lab work
  is postponed; no training or installation. Google guide sync is pending the
  required Windows trusted-reader failure, confirmed again for this milestone.

## 2026-10-08 — Continuous physical highway mentor branch

- Preserved the existing V02 road/navigation source and added a portable,
  referenced highway assembly with one physics scene and finite-road support.
- Added periodic scripted driving, true-lap accounting, expiring wheel commands,
  follow/overview previews and immutable run evidence; existing demos are unchanged.
- Passed 3 m/s, 6 m/s and 35 mph driving/braking checks, command-loss fallback,
  two matched full-lap runs, a three-lap / 9.93 km run and a paced 35 mph GUI
  preview. The 28,200 aligned
  samples from the full-lap pair matched exactly on the recorded runtime.
- Fixed a visually detected stale follow camera using explicit session-layer
  ownership and post-render pose checks. Earlier visual failures remain recorded.
- Verified 1,165 CPU tests and 18 USD-specific tests. See the
  [implementation/evidence note](continuous-highway-loop.md) and separate
  [mentor roadmap](mentor-loop-roadmap.md). Main roadmap, vehicle preparation,
  SUMO baselines and prior results remain unchanged by this task.
- No dependency installation or training; user involvement is required before
  the first new Isaac Lab training run. Google guide sync remains pending the
  Windows trusted-reader issue, explicitly recorded in the sync note.

## 2026-10-08 — Bounded 35 mph physical-car profile

- Added a named 15.6464 m/s cruise profile with a 550 m road, gentler 70 m
  lane shifts, longer detection/stopping preview and denser 20-Hz LiDAR scans.
- Preserved the original 3 m/s fixture as `--profile low-speed`, default
  low-speed controller guards and historical resolved configuration support.
- Added sustained-speed, high-speed fault braking, endpoint and full-road
  checks. [A full dropout run passed](physical-35mph.md), but overlapping work
  excludes it from isolated acceptance. Obstacle-pass validation is pending.
- Two partial GPU runs were retained after stopping only this task's children
  when unrelated vehicle tests launched concurrently. No training or dependency
  changes; Google guide sync pending. This is a development-preview checkpoint.

## 2026-10-08 — Referenced one-car physical scene

- Migrated the scripted LiDAR bypass demo to named Environment, Vehicles,
  Physics, Lighting, Cameras and Debug branches with referenced assets.
- Added separate vehicle geometry/physics and scene layers, versioned paths,
  exact Factory-contract checks, static-asset guards and portable scene export.
- Preserved legacy SUMO, low-level physics and Lab fixtures. No installation or
  training; new learned control still requires the user's involvement.
- Retained setup failures and a native shutdown crash alongside subsequent
  successful runs. See [scope, evidence and remaining checks](physical-scene-composition.md).
- Google learning-document sync is pending a Windows trusted-reader issue.

## 2026-10-06 — Required OpenUSD scene-authoring conventions

- Recorded the user's clean-hierarchy requirement in AGENTS.md and the workflow.
- Added [scene/asset conventions](../usd/SCENE_STRUCTURE.md): stable descriptive
  names, explicit component boundaries, references for reusable assets, payloads
  where selective loading helps, and appropriate layer/instancing boundaries.
- Documented the existing physical-prototype gaps and the replay exporter's
  existing references; defined migration and physics/sensor regression checks.
- Policy/documentation only: no runtime hierarchy changes, scene execution,
  dependency installation, asset conversion or training. Existing results and
  unrelated highway/vehicle work are preserved. Google guide sync is pending.

## 2026-10-06 — Measured preview pacing (GUI timing still open)

- Added optional `--real-time` 1x-target pacing, lighter RGB settings and 20-Hz
  rendering. Physics/control/planning/LiDAR rates remain 120/60/10/20 Hz.
- Added playback-rate/lag HUD, per-frame timing evidence, full-episode gates,
  cached overview/path geometry and a transformed steering-target marker.
- Preserved all 15 run packages, including a startup crash. GUI runs still
  fail the timing gate; a headless pass is not a live-window guarantee.
- Verified 880 CPU tests (11 skips), 31 Isaac USD tests, physical/sensor gates
  for blocked/stale-scan cases, and the original-profile runtime regression.
- See [commands, measurements and remaining work](realtime-preview.md).
  No training or dependency installation; historical results are unchanged.

## 2026-10-06 — Measured demo speedup and frozen-policy rehearsals

- Added bounded, matched-seed SUMO readiness checks: eight completed episodes,
  zero collisions, exact repeat fingerprints, saved source/checkpoint hashes,
  child deadlines and stdout/stderr capture. Both SUMO GUI modes also completed.
- Profiled the physical obstacle demo before optimizing it. Immutable evidence
  chunks replace repeated full-log rewrites; NumPy batches the unchanged safety
  predicates. The measured headless loop fell from 92.58 to 45.49 wall seconds
  per 50 simulated seconds. GUI loops remain 64.65–67.19 seconds, not real time.
- Rechecked all pass/blocked/stale-scan gates, wheel geometry and sensor timing.
  Three optimized pass-mode starts and both failure cases passed. Preserved all
  earlier evidence and full final JSON outputs; added fresh follow/overview PNGs.
- Added an inference-only, bounded launcher for the existing Isaac Lab
  Leatherback PPO. Three starts each completed 20 waypoint episodes with no
  failures/timeouts. No training, simulator installation, or environment change.
- Final CPU verification: **870 passed, 10 skipped**. See the
  [demo handoff](demo-readiness.md), [SUMO evidence](demo-sumo-results.md), and
  [timing/equivalence analysis](demo-physics-performance.md). New training still
  requires the user's participation; shared-loop driving and physical fleets
  are not claimed complete. The temporary presentation checklist was not added
  as a new long-term roadmap or memory goal.
- Added and verified Google guide tab 32 with eight shaded commands, six
  highlighted definitions, matched heading/link typography and all 32 prior
  tabs unchanged. [Sync evidence](learning-guide-sync.md) records the Windows
  transport workaround and the unverified pagination limit.

## 2026-10-06 — Outer collector highway USD v02

- Added a separate `_v02` with 500 m innermost main-lane radius, 535 m collector,
  four gradual exits and four returns. V01 files remain unchanged.
- Added seeded exit/re-entry plans and a four-choice route diagram; geometry
  stays fixed while choices vary. Minimum connector radius is 379.386 m.
- All 28 USD validators, 1,280 seeded trips and four route-policy tests passed.
  All 47 stationary contact checks passed, but the strict smoke gate remains
  failed on an unresolved stage-close reference-count warning.
- Saved top, angled and exit-detail RTX screenshots with hashes. See the
  [v02 evidence and limitations](highway-usd-v02.md).

## 2026-10-06 — Versioned four-leaf highway USD draft

- Added `highway_usd/_v01` with a reproducible generator, circular four-lane
  highway, continuous outer auxiliary lane, and four symmetric return petals.
- Added lane/route data, spawn and merge/diverge metadata, a welded road collider,
  box ground, static validator, regression checks, and version documentation.
- All 28 USD validators and three corruption checks passed; every ramp rejoins
  with zero endpoint gap. Both 26-sphere contact probes passed, but their strict
  overall gates remain failed on the known class of stage-close warning.
- Added actual USD/RTX screenshots and a capture workflow for every version.
  See [scope, evidence and limitations](highway-usd-v01.md).

## 2026-10-04 — LiDAR-triggered physical obstacle bypass

- Added a separate one-car, two-lane fixture with a sensed static-obstacle
  detour and return. Cyan now previews the actual driver route; simple visuals,
  corrected wheels, follow/overview cameras and the old stop demo remain.
- Added explicit extent priors, two-scan confirmation, reference-road footprint
  bounds, fresh point-cloud body-envelope braking, blocked-road and frozen-scan
  modes. PhysX owns movement; no SUMO connection, training or runtime installation.
- Retained failed detection, short-ribbon USD conversion and ground-plane versus
  3D-axle comparison attempts. Corrected the latter measurement without changing
  wheel geometry or the 0.1-degree threshold; records identify the new reference.
- **823 CPU tests passed, 10 skipped; 37 Isaac CPU view/wheel tests passed.**
  See [explanation, commands and limits](obstacle-bypass.md) for runtime evidence
  and the distinction between a bounded static fixture and general autonomy.
- Accepted headless/GUI passes retained 2.14471 m body clearance. Blocked-road
  and frozen-scan stops passed, with dropout braking at control tick 1452.
  The old 40-s curved braking demo also passed. [Ten retained attempts](obstacle-bypass-results.json)
  include five failures; one was a native GUI startup crash, followed by a passing
  unchanged-code retry. GUI RTF was 0.358, not real time.
- Added verified guide tab 31: five shaded portable commands, six highlighted
  definitions and preserved earlier tabs. No unattended training was launched.

## 2026-10-04 — Correct physical-car wheel orientation

- Fixed the horizontal/tumbling-wheel visual by authoring an explicit cylinder
  axle basis before simulation. PhysX still owns all wheel and chassis movement;
  no changes to driver control, torque, tire or suspension tuning.
- Added full wheel-geometry evidence and 0.1-degree alignment/pose gates. Final
  40-s test passed with identical 4,800-step physical/control traces to the prior
  obstacle baseline, zero contacts and unchanged 4.24599-m final stop gap.
- Preserved the original visual failure and one failed diagnostic-frame test.
  **786 CPU tests passed, 4 skipped; 4 Isaac wheel tests passed.**
  See [cause and reproduction](wheel-orientation.md) and
  [four retained attempts](wheel-geometry-results.json).
- Added verified Google guide tab 30 with three styled portable commands and
  four highlighted definitions; all 30 earlier tab bodies remain unchanged.

## 2026-10-04 — Visible physical car with RTX emergency braking

- Added a separate SUMO-free GUI demo with a marked curved road, overview/follow
  cameras, cyan reference path, pink pursuit target, green LiDAR points and pause
  controls. Simple vehicle visuals remain; no asset imports or training occurred.
- Added a 120-Hz physics clock with render-only checks, 60-Hz scripted control,
  20-Hz RTX capture, explicit acquisition/delivery/frame metadata and fail-safe
  braking. Map/odometry steering remains distinct from LiDAR emergency stopping.
- Corrected scan timing without relaxing the 0.20-s age gate, rejected malformed
  successful hits, fixed writer-registration order, and disabled a duplicate
  WORLD-to-world transform in the point display. All failed attempts remain local.
- **784 CPU tests passed**, plus all 23 view tests in Isaac's Python, including
  two USD tests skipped by the traffic environment. Accepted headless, dropout,
  GUI and overview checks passed with no runtime errors or scene-reference warning.
- Curve RMS error was 0.02120 m; final barrier gap 4.24599 m; zero contacts.
  Frozen scans caused full braking at the exact expected tick 1,452. Independent
  review recomputed decisions from retained raw clouds and verified identical
  headless/GUI physical traces, plus source/artifact hashes.
- Preserved nine attempts in [main evidence](physics-lidar-view-results.json) and
  [overview evidence](physics-lidar-overview-results.json). One numerical GUI pass
  was rejected by visual QA before the corrected capture was accepted.
- The headless full case achieved RTF 0.675, not real time. Full sensing/reset,
  hybrid traffic, multi-car communication and endurance gates remain open.
  See the [learning and reproduction note](physics-lidar-view.md).
- Added Google guide tab 29 with four shaded portable commands and refreshed
  the overview. Native verification preserved all 28 earlier milestone bodies;
  see the [sync record](learning-guide-sync.md).

## 2026-10-04 — Physical lane-following subset verified

- Added analytic straight/left/right 100 m route fixtures with shared sampling,
  coordinate transforms and explicit chassis-center versus rear-axle references.
- Added separate scripted behavior, timestamped route selection and pure-pursuit
  steering/speed/brake control. This uses known maps and simulator state, not
  LiDAR, learned control, a new road mesh or a new GUI.
- Added strict lane/stop/footprint/support/repeat gates and copied PhysX contact
  reports, including a barrier positive control. Review hardened wrong-spawn,
  malformed wheel-support and settling-departure false positives before running.
- **603 CPU tests passed.** Six physical cases passed: curve RMS 0.01841 m,
  max error 0.04866 m, zero normal-route contacts/departures, five-second holds,
  identical recorded local repeats. Overall supervisor remains **failed** because
  one retained scene-reference warning persists; no thresholds were relaxed.
- Source-captured attempt `20261004T194416Z-42992541` and earlier experiments are
  preserved. Independent review verified raw metrics and artifact/source hashes.
  See [learning note](lane-following.md) and [compact evidence](lane-following-results.json).
- Saved the user's preference to participate directly before Isaac Lab policy
  training. Agree observations/actions/rewards/algorithm/budget/evaluation together.
  No training, dependency installation or runtime edits occurred.
- Added Google guide tab 28 with three shaded portable commands, definitions,
  results and limitations; updated the overview. Native verification confirmed
  29 tabs and preserved all 27 prior milestone bodies. See [sync record](learning-guide-sync.md).

## 2026-10-03 — Hybrid physics foundation: in progress

Next validation increment (same feature branch, not merged to main):

- Added strict, portable stage supervisors, complete raw evidence and compact
  hash-verified reports for resets, expanded dynamics and Isaac Lab compatibility.
- Added a one-environment native-PhysX `DirectRLEnv`, using a `RigidObject` for
  chassis reset and explicit wheel commands for motion. Lazy imports keep SUMO
  dependencies out of this Isaac-only task.
- **434 tests passed.** Twelve dynamics cases passed their physical/repeatability
  subchecks, including 1/3/6 m/s braking, both turn directions and five-second holds.
- Three real Lab episodes passed action/observation/reset checks with identical
  recorded position/speed repeats. No training or LiDAR was added.
- Two ten-reset attempts passed physical/cache/provisional-memory subchecks but
  failed the overall zero-warning gate. Disabling authoring tools and draining
  stage-open events did not eliminate the first-stage reference warning. Dynamics
  also remains failed overall on that warning; the Lab probe did not emit it.
- Preserved all five attempts and the original foundation results. See the
  [validation note](physics-validation.md) and [evidence](physics-validation-results.json).
- Added Google learning-guide tab 27 with shaded portable commands, definitions,
  measurements, failed attempts and source links; refreshed its overview and
  verified all 26 earlier milestone tab bodies remained unchanged.

Original foundation checkpoint:

- Repaired the documentation index: removed missing historical-page links and
  distinguished the active traffic experiment from the preserved RC-scale
  Leatherback prototype.
- Added the [20-step hybrid roadmap](hybrid-roadmap.md), with staged physical-car,
  two-car, mixed-fleet, learning, and visualization acceptance gates.
- Recorded the [motion-authority decision](decisions/001-hybrid-motion-authority.md):
  Isaac moves evaluated physical cars; SUMO moves economical background cars;
  each vehicle has exactly one owner.
- Added the [development workflow](workflow.md), reusable iteration record, and
  pull-request template so code, tests, evidence, GitHub changes, and the Google
  learning document can be kept aligned.

The first physical-car implementation is a bounded low-speed foundation, not a
replacement for the working two-car demo. Its runtime results and completion
status must be recorded with the implementation evidence before closing roadmap
gates. Documentation changes alone do not validate dynamics or hybrid traffic.

Verified implementation subset:

- Added a simulator-independent driver command gate, Ackermann steering and
  explicit wheel torque/brake adapter; PhysX owns the car's motion.
- Added a portable headless supervisor with a deadline, GPU-memory stop,
  per-run source snapshots, partial-failure telemetry and source-change checks.
- **331 tests passed.** Two physical runs, each with two fresh-stage episodes,
  passed the frozen 3 m/s smoke gates. Published run
  `20261003T092703Z-7806d7c0` stopped from 2.99127 m/s in 0.67187 m; its two
  recorded position/speed traces matched. See the
  [foundation report](physics-vehicle-foundation.md) and
  [generated evidence](physics-vehicle-results.json).
- Preserved the first attempt and an unresolved USD stage-reference warning.
  Ten resets, long-run memory, full turn-radius/speed characterization, lane
  keeping, physical LiDAR and Isaac Lab compatibility remain open gates.
- Updated the Google guide with roadmap/workflow and physical-car milestone tabs,
  portable styled commands, evidence and limits; refreshed its start page and
  verified preservation of historical tabs. See [sync record](learning-guide-sync.md).

## 2026-10-03 — Capacity and shared-state foundations

Baseline reference: commit `76c9c197c0be6b36991240ba25a05d0b1484d529`.
See [scaling foundations](scaling-foundations.md) and [capacity results](capacity-results.md).

- Added explicit SUMO state/time/identity rules, seeded fleet assignments, and
  bounded SUMO/RTX capacity probes with source and configuration provenance.
- Measured background traffic and independent LiDAR costs separately; these
  isolated probes do not establish physical-vehicle fleet capacity.
- Refreshed stationary sensor data after startup work before enabling live
  control. The reported 120-second headless run passed its bounded timing gate;
  GUI memory growth and indefinite runtime reliability remain unproven.

## 2026-10-02 — Workspace consolidation

The [consolidation record](consolidation-2026-10-02.md) and
[transfer verification](transfer-verification.md) preserve the details.

- Established `Documents/highwaysim` as the sole active checkout connected to
  `hudsonkaneko/hivemind-traffic`.
- Preserved the earlier SUMO/PettingZoo/PPO work, replay and LiDAR experiments,
  cooperative demonstrations, and separate Leatherback physics prototype.
- Retired competing working directories into recovery archives; external Isaac
  runtimes remain dependencies rather than project copies.

## Earlier work

Consult the [documentation index](README.md), original experiment reports, and
Git history. This new log does not invent missing historical entries or recast
earlier SUMO-motion results as evidence for the new physical-vehicle architecture.

## 2026-10-26 — MAPPO v03 communication and wave setup

Added explicit seeded V2V transport, a masked shared message encoder, fixed-lane braking-pulse scenario, affected-lane rewards/metrics and matched no-communication config. 124 targeted tests and native no-learning preflight pass; zero optimizer steps and unchanged actor weights. No communication-benefit or trained-wave claim. See [configuration and evidence](sumo-mappo-v3.md). Google guide append/native readback and all42-tab Markdown regeneration verified; prior native guide updates reconciled without changing unrelated source work.

## 2026-10-27 — Frozen MAPPO v02 3D replay

Validated a 60-second seed-410001 SUMO rollout and recorded 3D replay. Original seed metrics reproduced exactly; zero collisions, overspeed or safety overrides. Retained action/trajectory/timing evidence and real simulator screenshots. Added structured export and a session-only follow viewer. This is kinematic visualization, not physical control. See [evidence and limitations](mappo-v02-3d.md). Google guide update pending.

## 2026-10-27 — Prepared sensor car in frozen v02 replay

Completed the full 60-second recorded replay with v14 main car, seven cameras and five LiDARs; background traffic remains lightweight. Saved 840 RGB images and 600 complete scans with clocks, transforms, checksums and timing metrics. Zero replay root error, unchanged sources, fresh-process scene audit and 94 sensor tests pass. See [sensor evidence](mappo-v02-sensors.md). This is kinematic sensing, not physical policy control.

## 2026-10-29 — MAPPO v04 cooperative-passing setup

Implemented hybrid acceleration/lane actions, delivered lane intent, guarded cooperative headways, real passing rewards and per-car driving metrics. 145 tests and full native no-learning preflight passed: three scripted passes, seven lane changes, zero collisions/overspeed and one override across 4,800 decisions. Independent review corrected cooldown, destination-lane filtering and applied-vs-eligible telemetry. Final configuration awaits review; no v04 training started. See [setup and evidence](sumo-mappo-v4.md). Google guide and Markdown mirror synchronized, including the earlier v02 capture/release updates.

## 2026-10-29 — MAPPO v04 completed pilot (not accepted)

Completed the approved 1,000,000-agent-transition run in 21 min 24 s without runtime errors or monitoring. Exported update 175 failed held-out acceptance: zero collisions/overspeed, 7.7275% interventions and zero passes. Later validation checkpoints learned passing but failed other gates; no accepted v04 replaces v02. Retained checkpoints, raw metrics, exact held-out evaluation and per-seed CSV. Independent review found no evident recovery bug; the approved budget is exhausted. See [results](sumo-mappo-v4.md#completed-pilot-not-accepted). Google guide and full Markdown mirror synchronized.

## 2026-10-29 — MAPPO v04r1 retraining preparation

Added training-only scripted imitation initialization and on-policy auxiliary teacher labels, within a single one-million-transition budget. Preserved safety/reward/action contracts and made checkpoint selection prioritize driving targets and intervention rate over reward. Fresh validation/test seeds, 167 passing tests and successful native no-learning preflight are recorded in [the refinement note](sumo-mappo-v4-refinement.md). The user authorized retraining; learned results and Google guide synchronization follow completion.

## 2026-10-29 — MAPPO v04r1 passing-capable result

The authorized retraining completed without errors in 23 min 31 s, using 48,000 demonstration plus 952,000 PPO transitions. Frozen neural inference achieved 82 passes across all 50 fresh test episodes, 4.9583% aggregate overrides and zero collisions/overspeed/gap deficits. One episode exceeded the six-changes/car/minute limit (8), so full acceptance remains false. Retained all checkpoints, exact per-seed metrics and learning curves; no teacher runs at inference. See [results and limits](sumo-mappo-v4-refinement.md). Google guide and complete Markdown mirror synchronized.

## 2026-10-29 — MAPPO v04r2 lane-stability preparation

Prepared the authorized retraining with a stronger applied-lane-change cost, stronger auxiliary guidance, lower PPO maneuver-label weighting and reduced exploration. Initial rare-maneuver imitation remains weighted. Safety, inference, cooldown and acceptance limits are unchanged. 178 tests and native no-learning preflight pass. [Declared configuration and evidence](sumo-mappo-v4-stability.md); results and Google guide update follow completion.

## 2026-10-29 — MAPPO v04r2 accepted SUMO passing actor

Authorized retraining completed in 21 min 31 s without runtime errors or agent monitoring. Selected update 175 passes all declared gates across 50 fresh test scenarios: 74 passes, 3.175% aggregate interventions, at most two lane changes/car/minute and zero collisions/overspeed/gap deficits. Latest update 233 regressed and is not the accepted actor. All metrics/checkpoints retained, independently audited, and [documented](sumo-mappo-v4-stability.md). Google guide and Markdown mirror synchronized. This is SUMO acceptance, not physical or sensor-driven validation.


## 2026-10-30 — Semantic mask and LiDAR fusion v01

Implemented calibrated front-camera mask projection onto five native RTX LiDARs,
with explicit time/geometry rejection and independent per-actor evaluation.
Static, 2 m/s and 10 m/s fixtures pass in run `20261030T223941Z-65f18896`:
99.95% assigned-label accuracy and 92.58–93.17% in-image return coverage.
Absolute-clock scheduling and render-only barriers preserve the 50 ms freshness
limit. All 44 focused tests pass; raw captures, failures and metrics are retained.
No training or MAPPO observation/policy change. Masks are simulator ground truth;
parked ego and scripted traffic are bounded sensing tests.
[Results, limitations and actual screenshots](semantic-lidar-fusion-v01.md);
[draft PR 31](https://github.com/hudsonkaneko/hivemind-traffic/pull/31).


## 2026-10-30 — Pretrained segmentation preview v01

Added frozen SegFormer B0 RGB-only inference on all 24 saved fusion captures,
with RGB/prediction/ground-truth/error panels, explicit unsupported classes and
retained native masks/metrics. Shared-class accuracy is 86.89%, mean IoU 71.52%;
vehicle precision 63.36% on the shared region and 59.94% over the full image.
The model visibly overpredicts drivable space; quality is not accepted for driving.
26 focused tests and independent artifact/panel audits pass. No training, new
simulator run, policy change or predicted-mask-to-LiDAR integration occurred.
[Actual previews, provenance and limitations](pretrained-segmentation-preview-v01.md).


## October 31: Saved-capture segmentation refinement v02

Added frozen aspect-preserving preprocessing comparisons, two-pixel boundary
metrics and geometry-only sparse LiDAR occupancy on 24 saved captures. All 98
focused tests pass. Shared mIoU improves from 71.52% to 86.06%; non-road false-road
rate falls from 28.48% to 6.25%, but boundaries and generic obstacle semantics
remain unaccepted. Three agents implemented/reviewed the work; 72 predictions,
raw metrics and screenshots retained in [the evidence note](segmentation-refinement-preview-v02.md).
A disabled six-class training proposal is included; no training/new simulation
started. Google guide and Markdown verified synchronized.

## 2026-10-31 — Segmentation training preparation v02

Prepared 3,600 native six-class captures in 180 split scenes, the guarded SegFormer
trainer and isolated CUDA runtime using three sub-agents. Reviewed dataset readiness
passed; original manifests and three smoke attempts remain intact. RTX 5070 no-update
preflight passed at batch4 x accumulation4, 1,475.64 MiB peak allocated; zero optimizer
steps and restored state. 139 CPU tests pass. Training remains disabled pending final
config review. Shared car geometry, static human poses and deferred advanced metrics
limit interpretation. [Results, screenshots and config](segmentation-training-preparation-v02.md).
Native engineering guide and full 42-tab Markdown mirror verified in sync.

## 2026-11-02 — Broad highway curriculum with I-110

Selectively imported Sam's I-110 road/graph/accounting source and added a distinct
graph-aware MAPPO environment. Twenty training cases cover cruising, dense
following, passing and merging on 2–5 through-lane layouts plus I-110; separate
validation/test layouts include unseen six-lane geometry. Final no-learning
preflight passes, with zero collisions/teleports but high dense-traffic interventions.
129 tests pass; training remains disabled for review of the 500,000-transition /
two-hour pilot. Existing v04r2 and segmentation work are preserved.
[Configuration, measured results and limits](highway-curriculum-v01.md).
Engineering guide and full 42-tab Markdown mirror synchronized and verified.


## November 2, 2026: highway curriculum pilot completed, not accepted

- Completed the approved MAPPO pilot: 499,998 transitions, 177 updates and 8,184
  optimizer steps in 1 h 52 min 39 s. No monitoring or analysis while learning ran.
- Preserved the original slow selected actor; corrected validation ranking to
  prioritize driving competence and strengthened bounded completion/resume,
  ranking migration and actor integrity checks. 137 regression tests pass.
- Final frozen validation: 98.76% cruise speed ratio, eight counted passes,
  zero of six merges and 96.44% safety overrides. No quality acceptance or held-out
  test claim. Faster diagnostic actor and all original evidence retained locally.
- Recorded [results and artifact hashes](highway-curriculum-training-v01.md),
  and synchronized the native engineering guide and complete Markdown mirror.


## November 2, 2026: persistent sensor-driven training roadmap

Saved a research-backed three-stage plan for structured highway driving, robust
imperfect observations and camera/LiDAR perception with physical closed-loop
validation. Added an AGENTS.md instruction to maintain status, evidence, next
actions and append-only progress during future relevant work. Proposed budgets
are bounded and unapproved; training will use script-managed metrics while the
agent waits for completion/failure. Existing failed pilot and perception limits
remain explicit. Two sub-agents audited evidence/research and reviewed the plan.
[Living plan and sources](sensor-driven-training-roadmap.md).
Documentation-only change: no training, simulator run, model download or runtime
change. Native guide and full 42-tab Markdown mirror synchronized and verified.


## November 2, 2026: Stage 1 structured highway preparation v02

- Added a separate 93-field graph/message actor contract, connected-road merge observations and causal action journals; historical runners and policies remain unchanged.
- Added an observation-only feasible teacher, BC plus fresh on-policy MAPPO preparation, fixed mixed curriculum, bounded resume accounting and immutable selected actor generations. Training remains disabled.
- Corrected generated ramps in a separate right-entry v02 geometry bank after the first full preflight exposed crossing through-lane connectors. Preserve failed tests and original networks; imported I-110 is unchanged.
- Results, exact disabled configuration and retained evidence: [Stage 1 preparation v02](highway-stage1-preparation-v02.md). The [living plan](sensor-driven-training-roadmap.md) records the next training-review gate.


## November 3, 2026: Stage 1 training completed; not accepted

- Completed the explicitly approved BC/MAPPO run: 499,992 transitions, 162 updates, 59m24s, no runtime failure or source drift.
- Final validation achieved 6/6 merges, 12 passes and zero collisions/teleports/overspeed, but overrides remain 18.15% pooled (53.56% worst case); eight of twelve cases fail the unchanged 5% gate.
- Preserved all phase results, policy/checkpoint hashes and metrics. No held-out test or second attempt. [Results and policy location](highway-stage1-training-v02.md).

- Added the requested [versioned policy catalog](../policies/versions/README.md), intentionally publishing the accepted v04r2 actor and experimental highway-stage1-v02 actor with exact configs, contracts, evaluations and hashes. Historical actor files are preserved.


## November 3, 2026: Stage 1 v03 diagnostics and disabled revision

- Measured retained acceleration mismatch magnitude/duration and target agreement; the after-BC actor has 0.2013 m/s² training-label acceleration RMSE despite 99.886% lane-label agreement.
- Matched scripted teacher passes all twelve validation cases at 1.168% pooled overrides, with a documented two-lane speed tradeoff. No validation data enters learning.
- Added versioned ranking that prioritizes per-case interventions after safety/task gates and rejects invalid/incomplete evidence. Offline ranking favors retained phase 2; original selections and published weights are preserved.
- Prepared twelve-epoch, acceleration-weighted BC and longer retained imitation within the existing 500k/two-hour budget. Training remains disabled. [Diagnosis, configuration and verification](highway-stage1-v03-preparation.md).


## November 3, 2026: Stage 1 v03 learning completed after recovery

- Ran the approved twelve-epoch, acceleration-weighted BC/MAPPO revision: 499,990 transitions, 162 PPO updates and 59m04s aggregate runtime. One Windows checkpoint access failure resumed from the valid checkpoint without changing sources or resetting budget.
- Selected validation improves pooled overrides from 18.15% to 14.97% and failing cases from eight to five; 6/6 merges and 12 passes remain. Four-lane passing regressed, and retained phase 0 has fewer pooled overrides than final; the actor remains unaccepted.
- Preserved failed/resumed logs, intermediate metrics and exact provenance. Published the frozen experimental actor in the versioned catalog and verified CPU inference/previous-package preservation. No held-out traffic or further learning ran. [Results](highway-stage1-training-v03.md).


## November 4, 2026: Stage 1 v04 improved training plan

- Saved a proposed matched A/B comparison of current-rollout imitation and retained training-only acceleration corrections, starting from the verified v03 phase-0 model.
- Defined explicit warm-start provenance, buffer/PPO separation, fixed evaluation milestones, worst-case-first selection, efficiency protections and unchanged acceptance gates. Proposed screen: 100k decisions/30 minutes per arm; no automatic scale-up.
- Updated the living roadmap and synchronized the engineering guide. This is planning only; no learner/config implementation, simulator run or training occurred. [Complete plan](highway-stage1-v04-training-plan.md).


## November 4, 2026 — variable-population MAPPO preparation

- Added reset-based 0–8 AV population fixtures, six-message selection and a masked population critic; old four-car behavior remains covered.
- Added bounded 100k/30-minute runner with verified actor warm start, critic-only warmup, fixed validation, atomic recovery and retained coverage.
- 157 targeted tests and final no-learning SUMO preflight pass. Failed spawn/API checks and repairs are retained.
- [Plan and resolved configuration](highway-population-training-v01.md). User authorized this pilot; result and engineering-guide milestone sync follow process exit. No training result is claimed yet.


## November 4, 2026 — population pilot result, no promotion

- Completed the authorized run once: 99,999 decisions in 474.296 charged seconds, no runtime failure or source drift. No metric monitoring or agent work ran during learning.
- Matched validation worsened from 20.6393% pooled/44.875% worst interventions to 31.7013%/75.125%; 15/16 cases fail at both endpoints. Final 4/4 merge targets and 14 passes are retained alongside regressions.
- Native logs reveal four training and one initial-validation emergency lane-end stops missed by sampled counters; final validation has none. Added independent hashed audit, exact per-case results and coverage.
- Published the frozen experimental actor and exact configuration without replacing older policies. [Result and next action](highway-population-training-v01.md).
- Google guide updated with revision guard and verified native readback; all 42 tabs/history preserved, 40 unaffected tabs unchanged. Generated Markdown and manifest passed synchronization checks.


### November 4: population v02 retraining completed

[Population v02](highway-population-training-v02.md): 99,998 decisions in 514.516 seconds. Pooled overrides 29.97%, worst 86.75%; twelve failing cases. Rejected for promotion. Metrics, native warnings and experimental weights retained. Next: diagnose acceleration errors before another revision. Google guide sync pending.


## November 4: restored analysis baseline and AV research comparison

Pinned exact Stage 1 v03 phase-0 actor as the lowest-intervention population-tested reference; no acceptance or runtime rollback. Added reproducible offline comparisons, public AV-policy differences and ranked improvements. Preserved all later candidates. No simulator/training ran. Verified 42-tab Google guide sync includes pending v02 outcome; generated mirror checked. See [review](policy-baseline-av-comparison.md).


## November 4, 2026: frozen population control and loss diagnostics

- Added bounded no-learning acceleration and gradient diagnostics using the preserved phase-0 actor, with independent review and 28 passing focused checks.
- Measured 7,200 acceleration decisions: synchronized measurements agree within 2.57e-13 m/s² and matched 0.1/0.5-second command durations are identical. Native following saturation, near-limit/merge response gaps and differing teacher/reward targets remain actionable.
- Sealed 1,536 balanced loss rows from twelve training-only rollouts. Measured opposing PPO/imitation gradients and fresh-critic coupling through global clipping; these do not reconstruct historical post-warm-up updates.
- Reconciled twelve earlier controlled-car lane-end emergency stops missed by sampled acceleration counters (9 training, 1 initial validation, 2 final validation). Corrected diagnostic parser; preserved original logs and summaries.
- Retained metrics, hashes, plots and offline reproduction tools; all weights and production runtime code unchanged. Living roadmap v09 and verified 42-tab engineering guide updated. No training or policy promotion. [Results](population-control-diagnostics-v01.md).


## November 4, 2026: population v03 control correction and diagnostics prepared

- Added opt-in near-limit taper, first minor-entry conflict observations and source-backed stop-line approach calibration. V3 teacher/reward share one current-observation target; v2 controls remain available and changed merge semantics are versioned.
- Matched teacher passing/merge fixtures now have zero overrides with tasks retained and similar speeds. The frozen actor improves from 26% to 11.17% merge overrides but remains unaccepted. Retained both development attempts: 32,400 decisions, no recorded safety events, exact repeat traces and explicit provenance limits.
- Added separate actor/critic gradient, clipping and fixed-return value diagnostics without changing optimizer behavior. V3 evaluation requires complete native emergency-event accounting; legacy thresholds remain unchanged.
- 150 CPU tests and the 32.719-second zero-update final preflight passed. Prepared the disabled 100k-decision/30-minute configuration, retained metrics/plots and updated living roadmap v10. Google guide and 42-tab Markdown mirror synchronized and verified. No experiment training or policy promotion. [Configuration and results](population-control-v03.md).

## November 5, 2026: frozen MAPPO physical adapter and calibration

- Added explicit 10 Hz MAPPO → 60 Hz acceleration/lane adapter → 120 Hz PhysX control, with exact 93-field input parity, smooth lane changes, calibrated pedal mapping and stale-command fallback.
- Preserved the failed full-range brake fit, corrected the ordinary braking domain, and obtained 0.02131 m/s² validation / 0.01634 fresh-confirmation response RMSE. Scripted 6/12/18 m/s acceleration, lane changes and stopping pass.
- Unchanged best actor cruises and passes, but remains unaccepted: 82.11% cruise and 27.76% passing speed-ceiling interventions. Zero nominal contacts/overlaps/overspeed/fallbacks; zero optimizer steps.
- 209 CPU tests and six fresh-process USD reopen checks pass. [Results, exact scope, screenshots and both attempts](mappo-physical-adapter-v01.md). Living roadmap updated; guide synchronization recorded in the sync ledger.


## November 5, 2026: population v04 near-limit correction prepared

- Added opt-in raw deterministic acceleration supervision in physical units near the local speed limit, using the existing shared v3 teacher target. Kept all guards, inference behavior and historical defaults.
- Added shuffled 12/18/22.352/26.8224 m/s episode ceilings, native lane/connector readback, coherent departure/slow-leader speeds and eight additional matched validation cases.
- Retained raw requests, conditional loss/exposure metrics, gradient components and repeatable boundary diagnostics. Frozen audit records previous-request sensitivity and both low/high-speed errors.
- Three sub-agents contributed implementation and review. 171 tests and the 27.657-second zero-update SUMO preflight passed; one teacher-fixture background emergency remains disclosed. No experiment training or new Isaac run.
- [Final configuration and evidence](population-near-limit-v04.md); living roadmap v12 updated. Guide synchronization is recorded in the sync ledger. The actor remains unchanged and unaccepted.


## November 5, 2026 — I-110 frozen-policy merge visual test

- Added a bounded deterministic recorder and layered 3D replay renderer for final Stage 1 v03, selected by matched merge-specific results; original v2 observation/control semantics retained.
- Seed 6400019 completed both ramp merges (3.0/9.7 seconds) with no collisions, teleports, overspeed or native emergency warnings. Full-run interventions 131/1240 (10.56%) still fail the 5% target; no acceptance or training claim.
- Saved an 11-second/110-frame MP4 and actual event screenshots; video decode, zero pose error and unchanged authored layers verified. Six CPU/OpenUSD tests pass. Fixed/preserved the first preview's premature Isaac shutdown failure.
- [Results, provenance, screenshots and replay commands](i110-merge-visual-v01.md); living sensor roadmap and 42-tab Google/Markdown engineering guide updated. Video and bulk trajectories remain local outputs.


## November 5, 2026 — continuous-maneuver highway environment v05

Implemented the user-authorized environment revision: native three-second lane changes, measured moving-neighbor corridor context, 99 actor inputs, 24 controlled slots, eight-second stable-merge/full-route metrics, varied ramp/demand generation and a resumable competence curriculum. Preserved the failed 14/81 post-merge teacher corrections and paired 0/81 fix without weakening the 5% gate. 256 targeted tests and a 64.750-second zero-update final preflight pass; I-110 teacher still fails the overall intervention gate at 75/1404 (5.34%), so readiness is not driving acceptance. Prepared frozen recording and the existing SimReady car as the focal replay asset. The bounded 250k-decision/3,600-second pilot is authorized and queued; training results are pending. [Configuration and evidence](highway-environment-v05.md). Guide synchronization is pending the completed milestone.


## November 5, 2026 — v05 pre-learning failure repair

The first authorized launch stopped during initial validation, before any training decisions or optimizer updates. Preserved the 125.344-second failed attempt and checkpoint. Corrected sustained-departure millisecond serialization, native interrupted-maneuver cleanup, and map-based lane-change runway masks. The exact failed departure/ending-lane cases now complete 80 seconds with frozen weights and no native warnings; their 34.62%/29.535% intervention failures remain. 265 tests pass. A guarded, reviewed zero-update migration preserves weights, optimizer, RNG and consumed budget; fresh readiness precedes resume. [Diagnosis and evidence](highway-environment-v05.md#first-launch-preserved-pre-learning-failure).


## November 5, 2026 — v05 native insertion repair

The second launch stopped before learning: native ACC delayed an initial closing follower, allowing later inflow to mix with the fixed cohort. Preserved both failed checkpoints and 512.048 cumulative seconds. Added source-backed ACC insertion clearance before AV assignment; counts/speeds/guards remain fixed, while initial positions and density change where needed. All 132 native reset cases pass with all initial vehicles departing at zero and present at 0.1 seconds, no collisions/teleports/warnings. 269 tests pass. [Failure, density comparison and recovery](highway-environment-v05.md#second-launch-native-insertion-feasibility).


## November 5, 2026 — v05 pilot completed and rejected

Completed 249,999 active-agent decisions,50 update batches and 2,455.032 cumulative seconds including both failures, without progress monitoring during training. Stable merges improve2/9→8/9 and passes 55→59, but pooled interventions regress 13.37%→19.83%; 19/24 cases fail and I-110 stays 0/1 merged. No recorded collisions/teleports/overspeed/emergency events under protections; no held-out promotion or physical/sensor qualification. Published the small frozen 99-input candidate with exact config, contract, checksums and per-seed evaluation; older policies stay intact. [Results and measured plots](highway-environment-v05-results.md); [policy](../policies/versions/highway-environment-v05/README.md).


## November 6, 2026 — I-110 merge blockage correction v01

- Fixed false ramp-junction conflicts with four parallel through lanes; the genuine auxiliary-lane conflict and yield remain. New versioned output preserves the original network, geometry, demand, policy and safety settings.
- Frozen seed 6900019 replay: focal stopped samples 752 to 0; stable merges 0/1 to 1/1; all eight controlled trips complete; fleet interventions 30.53% to 8.52%, still failing the unchanged 5% target. Zero recorded collisions, teleports, overspeed or native emergency warnings.
- Added native junction-entry/progress diagnostics, honest maneuver labels, readiness provenance and regression tests. Retained failed duration/native-control probes and the potentially stateful diagnostic-getter caveat; ordinary recording independently confirms the result.
- Verified a 251-frame SimReady v14 main-car replay with lightweight backgrounds, exact poses, unchanged source assets and screenshots. No training or policy promotion.
- [Results, evidence and reproduction](i110-merge-blocking-v01.md). Both engineering-guide copies synchronized from a full revision-guarded native readback; all 42 tabs preserved.


## November 6, 2026 — remaining overrides and smooth-control proposal v01

- Audited all 244 remaining corrected-I-110 interventions: 233 native following responses and 11 minor-entry braking responses. Frozen MAPPO still fails the unchanged 5% limit at 8.52%; no safety setting, threshold or learned weight changed.
- Added saved-record diagnostics and a bounded zero-update train-only actor/teacher comparison, with five regression tests. The existing teacher reaches 0% interventions on that separate case but peaks at 65 m/s³ jerk, so stronger imitation requires a smoother verified reference first.
- Preserved raw metrics, hashes, objective/gradient evidence, scientific plots and a fresh 251-frame SimReady visual. The ordinary 801-frame replay exactly repeats the prior corrected trajectory.
- Published a disabled reference/reward proposal: anticipate entry, recover from measured acceleration smoothly, strengthen alignment/imitation after preflight, and use uncapped Huber jerk plus reference-error slew penalties. Proposed paired pilot is 100k decisions/one hour total, pending preparation and training approval.
- [Diagnostic and visual evidence](highway-smooth-control-v01.md); [proposed weights and gates](highway-smooth-training-plan-v01.md). Living roadmap v17 and both engineering-guide copies updated. No training or policy promotion.


## November 6, 2026 — Smooth-control preparation v01, no training

Implemented the observation-only v4 acceleration reference, measured-jerk/command-residual reward bundle, full-horizon per-car metrics and strict paired-MAPPO preparation/launch contracts. Three subagents contributed code, tests and review. Retained all 44 native full runs and the rejected v5 trial; unchanged v05 actor and safety guards remain intact.

All six retained reference fixtures pass the original safety/task/5% override checks (0–0.25%). I-110 merges at 20.0 s, stabilizes at 28.0 s, with no focal stop and 2.000001 m/s³ controlled peak jerk. Four other nominal comfort gates fail, so readiness is false and training remains disabled. These are scripted-reference results, not learned-policy improvement.

195 CPU checks and eight subtests verified across the full run plus one corrected metadata recheck; 11 actual USD/renderer checks pass. Frozen-gradient checks preserve parameters and optimizer state. Fresh 30-second SimReady main-car replays compare the frozen actor and reference with exact SUMO poses and lightweight backgrounds. [Implementation, config, screenshots, failures and metrics](highway-smooth-preparation-v01.md). Google guide and generated Markdown mirror synchronized from verified full native readback; raw videos/checkpoints remain local.


## November 6, 2026 — Highway training rethink v01 (audit and plan only)

Three read-only audits inspected the native control/learning chain, reward scales, saved trajectories and existing weights. Seven actors exactly match trusted finite checkpoints; 70 local weight files inventoried. Saved I-110 data shows the scripted main car travels 55 m less during the 30-second replay and another car slows to 1.456 m/s after an unnecessary ending-lane pass. Controlled mean boundary time is identical to frozen MAPPO. Four comfort peaks have exact offline ACC/occupancy fits, including a farther slow leader missed by nearest-only corridor reduction. Last v05 training covered only difficulty 0 and no I-110 merge episode.

[The new plan and measured figure](highway-training-rethink-v01.md) prioritize roadway/visibility semantics, route-useful targets and occupied-lane anticipation before stronger imitation or comfort penalties. It separates pilot factors, adds all-car progress gates and guarantees broader training exposure. Proposed budgets require new reviewed contracts and explicit launch approval. No runtime source, weights, rewards or disabled training config changed; no new simulator or optimizer run. Living roadmap v19 and Google/GitHub guide synchronized. Compact audits/evidence published; full local catalogue retained.


## November 6, 2026 — Highway flow preparation v01

Implemented the separate 117-field environment/reference, versioned counterfactual bank, qualified-label MAPPO adapters, paired imitation-only configuration and strict budget/resume/readiness gates with three subagents. Preserved all historical runtime sources and weights. The 81-recording allocation uses verified compatible cached evidence plus fresh runs and an explicit deadline repair; failures and 978.186 seconds of preparation cost remain retained. Reference passes 17/27 combined cases: all raw safety/5% checks pass, but four nominal jerk gates and six task gates fail. I-110 main-car flow regresses while the ending-lane detour is removed. Initializer selection/readiness remain null and training disabled. Four verified 60-second SimReady replays, raw metrics, compact evidence, candidate-only frozen gradients and the prepared configuration are linked in [the technical note](highway-flow-preparation-v01.md).


## November 6, 2026 — bounded I-110 approach ablations

Two short candidates failed: arrival 49.3 -> 63.2 s, stopped exposure 0 -> 12.3 s; zero overrides in all runs. Restored reference code/tests, retained failed source snapshots/metrics and measured speed plot. No training/config changes. [Evidence](highway-flow-preparation-v01.md#focused-approach-ablation--november-6-2026). Incremental guide sync pending.


## November 6, 2026 — GPUDrive frozen I-110 transfer v01

Added pinned CPU actor inference and a continuous-action bridge, with 13 passing tests. One native baseline and one pretrained test ran; pretrained transfer oversped and crossed the road boundary at 13.4 s. Saved raw observations/actions, source hashes, metrics, actual SimReady replay and screenshots. Existing policies unchanged; no training. [Evidence](gpudrive-i110-v01.md). Incremental Google guide sync pending.


## November 6, 2026 — GPUDrive guarded I-110 v02

Added a documented steering-to-lane adapter to reuse native SUMO/v05 safety and a60mph ceiling. Paired80-second tests: no collisions/overspeed; frozen actor fails to merge, stops71.9s and needs92.25% combined interventions.17focused tests pass. Saved metrics, native commands and SimReady replay. No training. [Evidence](gpudrive-i110-v02.md); guide sync pending.


## 2026-11-06 — GPUDrive continuous steering repair (v03)

Restored continuous steering instead of tactical lane conversion, with opt-in longitudinal/speed/footprint safeguards and exact action audit. 23 tests plus 4 subtests pass. A bounded CPU test and error-handling retry retain all metrics: 1.1 s simulated, 8.269 m, no stop/overspeed/contact, but unsupported upstream-mainline route association ends the test. Adapter fix verified; transfer remains unqualified. No training or new render. [Evidence and limitations](gpudrive-i110-v03.md). Google guide incremental sync pending.


## 2026-11-06 — GPUDrive destination-preserving route repair (v04)

Fixed off-route continuous-pose association and rebuilt the edge route to the original goal; validated readback and rejected false truncated-route arrivals. One short CPU test completes the merge criterion at 8.3 s and goal at 28.5 s, with no stops/contact/overspeed. Thirty tests plus 14 subtests pass. Acceleration interventions remain 89.79%, so no policy acceptance. [Metrics and limits](gpudrive-i110-v04.md). No training or rendering; guide sync pending.


## 2026-11-07 — GPUDrive crossing audit and comfort filter (v05)

Implemented network-transition verification, separate range/comfort/guard accounting, anticipatory following and routine jerk limiting before unchanged safety controls. Forty tests plus 16 subtests pass. One short run reaches goal in 28.4 s; peak longitudinal jerk60→3 m/s³, no stop/contact/overspeed. Audit exposes an early unconnected ramp exit in both v04/v05; raw-command modification remains99.29%, so neither proper merging nor policy acceptance is established. [Evidence, settings and limits](gpudrive-i110-v05.md). No training/render; guide sync pending.


## 2026-11-07 — GPUDrive lessons applied to a MAPPO plan

Saved a staged [transfer plan](mappo-gpudrive-lessons-plan-v01.md) for the prepared flow117 path: valid merge corridors and stage-wise control metrics, persistent gap alignment/native-response qualification, bounded CPU checks, then the existing initializer and MAPPO comparison gates. Keeps current safety/action limits and budgets, explicitly distinguishes 93/99/117 input generations, and requires actual merge-event exposure rather than scheduled coverage. No implementation, config/weights change, simulation or training. Living roadmap v27 updated; guide incremental sync pending.


## 2026-11-08 — Teacher-free flow117 MAPPO pilot, two completed rounds

Implemented separately versioned training without unqualified teacher rewards/losses, preserving native SUMO safety and the original disabled imitation study. Fixed runtime population validation and independent critic/PPO curriculum cursors. User-authorized learning and terminal-only review collected 59,995 decisions in two rounds; round2 increased jerk penalty after measured failure. Pooled development interventions fell 24.94%→16.44%→7.61%; I-110 merge is 0.67% with one stable target passed. Comfort and three per-case intervention gates still fail; no promotion. [Full evidence](highway-flow-rl-v01.md) and [downloadable experimental actor](../policies/versions/highway-flow-rl-v01/README.md). 88 focused tests, source-bound preflight and exact CPU export parity pass. All failed/runtime costs retained; Google guide incremental sync pending.


## 2026-11-08 - Final flow117 merge video

- Added frozen117 recorder and saved a 25-second I-110 replay using the SimReady main car. Same seed and ten compared metrics match the completed evaluation exactly; 250-frame encode/decode and unchanged asset/pose checks pass. No training or promotion. [Evidence](highway-flow-rl-v01.md#november-8-frozen-policy-merge-video). Guide sync pending.


## 2026-11-08 - Offline MAPPO slowdown diagnosis

- Saved speed/velocity/command traces and graphs from the existing SimReady replay. Identified native junction braking despite positive policy acceleration, later policy-requested speed dips and an ending-lane detour. Verified exact speed-change decomposition across 399 steps; actual episode cap is 18 m/s. [Analysis](highway-flow-rl-velocity-analysis.md). No simulation/training or control changes. Guide sync pending.


## 2026-11-08 - Proposed flow117 v02 plan

- Saved the staged [ramp and policy improvement plan](highway-flow-rl-v02-plan.md), with frozen-policy network comparisons, route-aware lane-choice checks, cruise/comfort reward design and full-route acceptance. Defined bounded diagnostic and proposed training budgets with terminal-only review. Documentation only; no controller changes, simulator runs or training. Guide sync pending.


## 2026-11-08 - Flow117 v02 preparation implemented

- Added a versioned visibility candidate, cruise/detour rewards, full 117-field observation journals, a bounded runner with resume, and a training/preflight contract. Original controls and weights are preserved.
- 158 tests and zero-update native preflight pass. The matched screen removes empty-approach braking while retaining conflict braking; five development evaluations still fail driving gates. Failed attempts and metrics are retained; preparation CPU cost is 115.734 seconds.
- Full 54.6-second SimReady replay, configuration and evidence saved. Training is disabled for review; no new actor or promotion. [Details](highway-flow-rl-v02.md). Google guide synchronization pending.


## November 8, 2026 - Offline adjacent traffic comparison

Compared frozen MAPPO agent_0 with geometry-selected native IDM main_8 in the saved v02 replay. The native car accelerates more steadily; retained speed/acceleration plots, raw traces, hashes and explicit unequal-start/route limitations. Finite/cadence/kinematic checks passed. No simulator runs, training or parameter changes. See [analysis](highway-flow-rl-v02.md#november-8---speed-comparison-with-adjacent-native-traffic). Google guide sync remains pending.


## November 8, 2026 - Research-backed reward revision proposal

Saved a [reward revision plan](highway-flow-rl-reward-revision-plan.md) and source-hashed offline audit. The v02 cruise addition pays zero during both focal speed dips; proposed changes prioritize target/eligibility design, normalized Huber comfort and optional-ending-lane cost. Two sub-agents supplied research and code review. Candidate weights and reserve reallocation require review; runtime configuration, weights and training remain unchanged. No simulator runs. Google guide synchronization pending.


## November 8, 2026 - Repair clean-clone initialization test

[CI diagnosis and repair](ci-clean-clone-checkpoint-fixture-fix.md): replace an ignored local checkpoint dependency with real deterministic test artifacts, retain production checks and add corruption/fresh-critic assertions. All 67 module tests pass locally and in a clean checkout. No training, policy, runtime or workflow changes. Remote full-suite verification remains separate.


## November 8, 2026 - Remove branch prefixes without losing work

[Branch naming cleanup](branch-naming-cleanup.md): renamed all 62 repository branches locally and on GitHub, preserving their commits. Recreated 48 affected PRs with original metadata and historical-discussion links; teammate fork PR #16 remains open. Active work is mappo-flow-rl-v02 / PR #108. Future names omit codex/ and push CI covers all branch names. The earlier checkpoint-fixture repair passed GitHub full CI (4,105 tests). No policy or training changes.


## November 8, 2026 - Repair CI across historical open PRs

[Open-PR audit](ci-open-pr-audit-2026-11-08.md): inspected all failure groups and backported the portable checkpoint fixture to ten historical branches; added the missing Pillow CI dependency to semantic-lidar-fusion. Local oldest-branch module: 67 passed. Semantic tests: seven passed; full collection: 3,169 tests. Exact repair commits and logs retained; per-head GitHub checks remain the remote acceptance evidence. No policy or training changes.
