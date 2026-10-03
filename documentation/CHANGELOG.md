# Change log

This is a curated guide to meaningful changes, not a replacement for Git history.
Dates below use America/Los_Angeles. Existing experiment reports remain the
authority for historical measurements and failures.

## 2026-10-03 — Hybrid physics foundation: in progress

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
