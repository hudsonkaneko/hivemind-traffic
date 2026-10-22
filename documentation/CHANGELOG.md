# Change log

This is a curated guide to meaningful changes, not a replacement for Git history.
Dates below use America/Los_Angeles. Existing experiment reports remain the
authority for historical measurements and failures.

## 2026-10-22 — Sam's supplied-car visual replay foundation

- Added a scoped [development plan](sam-development-plan.md) under the master
  research boundaries, without adopting the broader physical-fleet roadmap.
- Added private-source asset composition and a layered SUMO replay builder,
  explicit fallback materials, stable vehicle identities and shortest-arc heading
  interpolation. The original collection and baseline inputs remain unchanged.
- Added an Isaac viewer with playback, scrub, selection, telemetry and camera
  controls plus a portable launch wrapper. See [usage and evidence limits](visual-twin.md).
- Three public-fixture tests pass for pose/heading/lifecycle, input immutability,
  invalid times, output preservation and heading wrap. Local full-recording checks
  passed 1,350 pose/heading/lifecycle checks; actual runtime evidence is recorded
  separately. Private assets, installed runtime and raw outputs are excluded.

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
