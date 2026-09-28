# Obstacle avoidance validation — 2026-09-28

The completed maneuver changes to the neighboring lane, passes the stationary
obstacle, and returns. If that lane contains an observed static obstacle, it stays
in the original lane and stops. **67 automated tests pass.** Five completed GPU
runs passed all driving gates; two earlier failed attempts are retained below.

| Run ID | Start speed / gap | Case | Minimum box separation | Final speed | Result |
|---|---|---|---:|---:|---|
| 20260928T173127Z-avoid-03cd76 | 6 m/s / 40 m | GUI pass and return | 1.1114 m | 6.0 m/s | PASS |
| 20260928T173439Z-avoid-113840 | 6 m/s / 40 m | Pass and return | 1.1114 m | 6.0 m/s | PASS |
| 20260928T173637Z-avoid-e462fd | 6 m/s / 40 m | Pass and return | 1.1114 m | 6.0 m/s | PASS |
| 20260928T173833Z-avoid-ddcf29 | 8 m/s / 55 m | Pass and return | 1.1553 m | 6.0 m/s | PASS |
| 20260928T174315Z-avoid-be5a40 | 6 m/s / 40 m | Blocked lane | 1.1800 m | 0.0 m/s | PASS |

All runs were 250 traffic steps / 25 traffic seconds, with no collisions or ego
speed overrides above 0.15 m/s. Maximum lateral movement was 0.064 m per 0.1 s;
the blocked case had none. Oriented proxy footprints remained inside the road.
The two matched headless 6 m/s runs had zero differences in recorded x/y/speed,
phase sequence and lane requests (declared tolerance 0.01 m / 0.01 m/s).
This does not imply raw noisy sensor scans or other machines are bitwise deterministic.

The GUI run requested the left lane at step 0, entered the passing phase at step
50, requested return at step 96, and completed return at step 146. Its closest
conservative separation was 1.1114 m. The blocked-lane run stopped roughly 5.2 m
short of the forward obstacle; its 1.18 m minimum separation in the table is to the
other box in the adjacent lane, not the distance ahead.

## Evidence

Every listed run was independently re-audited from raw scans: rebuild occupancy,
recompute controller speed and lane requests, verify timestamps and state
continuity, check artifact hashes, and recompute geometric safety metrics.
[Machine-readable results and manifest hashes](lidar-avoidance-results.json).

GUI captures inspected under `outputs/live_lidar/20260928T173127Z-avoid-03cd76/`:
`preview.png` (left lane before passing), `preview_0090.png` (passing), and
`preview_0130.png` (returning). Raw evidence stays local and ignored by Git.
All runs used SUMO 1.27.1, the installed Isaac 6.0.1 runtime, RTX 5070 / driver
610.74, and the native noisy Example_Rotary profile. No runtime binaries were edited.

## Failures retained

1. `20260928T172851Z-avoid-7b1563`: safe stop but incomplete maneuver. The heading
   settling condition prevented leaving the outbound phase. Corrected and covered
   by regression tests; original source/scans/summary retained.
2. `20260928T174028Z-avoid-11d1a6`: native access violation in
   `rtx.rtxsensor.plugin.dll` after 169 scans. No final result, so not a pass.
   The batch aborted and did not write an aggregate report. Successful earlier
   runs were recovered through independent audits of their intact evidence.
   The unchanged blocked-lane retry `20260928T174315Z-avoid-be5a40` passed.
   This does **not** establish that the native runtime crash is fixed.

The harness now retains per-case checkpoints and explicit missing-result records,
and the capture script writes completed steps incrementally to `telemetry.jsonl`.
A unit test simulates a native crash and verifies preceding successes are preserved.
The original crash log is `logs/avoid-suite-20260928T173439Z-3.log`; its incomplete
run has a recovery note rather than a falsely completed manifest.

## Launch and scope

```bat
.venv\Scripts\python.exe scripts\demo_live_lidar.py --mode avoid --speed 6 --gap 40
```

Run from `C:\Users\hudso\Documents\highwaysim`. See the
[learning guide](lidar-avoidance.md) and [validation plan](lidar-avoidance-plan.md).
The SUMO and sensor skills informed the single motion authority, coordinate
transforms and safety checks; the reproducibility skill informed saved raw scans,
fixed test cases, independent audits and retained failures.

This remains stationary-obstacle avoidance on a known straight two-lane road,
using a scripted controller and SUMO lane-change kinematics—not PPO, moving-traffic
negotiation, physics steering, or wall-clock real time. Current local-only workspace
instructions mean the learning guide is in this repository, not the Google Doc.
