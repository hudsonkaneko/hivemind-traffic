# Change log

This is a curated guide to meaningful changes, not a replacement for Git history.
Dates below use America/Los_Angeles. Existing experiment reports remain the
authority for historical measurements and failures.

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
