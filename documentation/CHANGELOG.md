# Change log

This is a curated guide to meaningful changes, not a replacement for Git history.
Dates below use America/Los_Angeles. Existing experiment reports remain the
authority for historical measurements and failures.

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
