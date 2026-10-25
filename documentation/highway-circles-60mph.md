# CIRCLES main car and PPO lane traffic — v01

October 25, 2026. **Prepared; highway driving validation is pending.** The user
chose to defer execution while a separate project uses Isaac. No background
launch, monitor or new training is scheduled.

## Scenario and controls

The [fixed protocol](../experiments/highway-circles-60mph-v1.json) places three
dynamic physical cars on the V02 highway's adjacent inner circular lanes. PhysX
owns all motion. Each car has its own four wheel controllers, command gate,
contact monitor and native body view. The existing low-speed demo keeps its
3 m/s default cap; only this separate experiment requests **60 mph (26.8224 m/s)**.

| Setting | Prepared value |
| --- | --- |
| Main car | CIRCLES longitudinal output; existing scripted PathFollower steering |
| Background cars | Two copies of the frozen 42-feature RSL-RL PPO actor |
| Lane radii | 500, 503.7 and 507.4 m; initial angles 0, +0.04, -0.04 rad |
| PPO target speed | 3 m/s (6.71 mph), matching the trained task |
| Physics / commands / CIRCLES | 120 / 60 / 20 Hz |
| Each case | 35 simulated seconds including 2 seconds of brake settling |
| Cases | One explicitly scripted baseline, then all seven pinned CIRCLES exports |
| Budget | Seed 51001; 600 seconds maximum process time; zero optimizer steps |

PPO receives measured own-car state and a known lane reference. Its 20 object
features are deliberately empty for **nonreactive lane-only traffic**. The route
reference advances every 60 m without moving or resetting the vehicle. This
does not establish learned car following or collision avoidance.

CIRCLES receives `[own forward speed, leader forward speed, bumper gap]` in
m/s, m/s and metres. Adjacent-lane cars do not count as leaders. With no same-lane
leader, this probe supplies the declared surrogate `[own speed, 26.8224, 1000]`.
That is an experimental input convention, not an upstream-verified absent-leader
contract or a sensor measurement. Other exports retain the interpretation caveat
documented in the [original CIRCLES smoke](circles-control-smoke.md).

The longitudinal adapter uses `target = clamp(speed + 0.6 * output, 0, 26.8224)`,
throttle gain 0.7 and brake gain 0.35. At the ceiling the runner requests full
braking; measured speed above the ceiling terminates the case. This is a control
limit and acceptance guard, not a velocity teleport. Reaching 60 mph is neither
required nor assumed; actual peak speeds must be reported after execution.

The pass rule requires the entire case duration, more than 5 m progress for
every car, and no contact, full-footprint lane departure, excessive tilt, support
loss, reverse motion, command fallback or overspeed. Failed driving cases remain
in the result. An `evidence_valid` result only means execution/evidence checks
passed; inspect each case's `passed` field separately.

## What was checked

**55 CPU tests passed** using Python 3.12.13 and OpenUSD 25.11, without launching
Kit or physics. They cover the 60 mph conversion and legacy default, configuration
bounds, leader selection, PPO observation shape, guards, independent wheel/body
bindings, relationship and collision-group closure, three-car portable scene
export, and engineering-guide conversion. All selected tests ran without skips.

The first native attempt, `v01-20261025T211416Z-ee14a001`, failed before physics
started: rewriting relative sublayer paths in the live anonymous stage invalidated
wheel schema handles. It recorded **0 physics steps, 0 driving cases and no
screenshot**. Cleanup and contact-setting restoration passed, but the evidence
gate correctly failed. The attempt remains under `outputs/highway_circles/` and
its log under `logs/highway_circles/`.

The export now copies opinions into a separate file-backed layer and makes only
that saved layer's paths relative. CPU regression checks preserve all 12 live
wheel prims and reopen a portable three-car scene with valid composition.
**The repaired native run has not been performed.** No highway speed, PPO lane
keeping, interaction or screenshot result is claimed.

## Run later

From the checkout root on the configured Windows workstation, after the other
Isaac job finishes:

```powershell
.\.venv-traffic\Scripts\python.exe experiments/run_highway_circles.py
```

The supervisor refuses an existing Kit job, saves unique source/input hashes,
runs a bounded headless native experiment and checks shutdown. It will attempt
an actual viewport capture in the CRA case; opening the saved initial USD alone
does not run any controller. Outputs go to `outputs/highway_circles/v01-<run-id>/`
and runtime logs to `logs/highway_circles/v01-<run-id>/`. There is no new policy.

Required local inputs, deliberately excluded from Git:

- Existing checkpoint: `outputs/rsl_integration/v02-train-20261022T203726Z-61afe5d7/policy.pt`.
  SHA-256: `f2e51984bce3a13d43e20f146b4b00f5d406f6ce1aa48167bb384773317123a5`.
- Seven ONNX exports under `outputs/circles_control/models/3b3ee3b91afdae93075c8b90334d65748cd9ff74/`,
  fetched using the existing [pinned downloader](../scripts/fetch_circles_controllers.py).
- Existing project-local ONNX Runtime at `runtimes/circles_onnx312`, configured
  Isaac runtime and V02 road assets. No runtime installation is performed.

The runner reuses the earlier physical-fleet binding and scene-authoring helpers.
Their relevant CPU tests are published separately from the unfinished local
pair/endurance probes. Other existing work remains outside this change.

This is an exploratory control compatibility test, not a matched traffic study
or a validated self-driving system. Throughput, merging, sudden obstacles,
sensor perception, high-speed PPO traffic and repeated-seed performance remain
outside its acceptance claim.
