# GPUDrive v05: crossing audit and smoother longitudinal control

November 7, 2026. **Steps 1–3 implemented; one short SUMO run; no training.**

The frozen actor, action decoding, continuous steering, 60 mph/lower legal cap,
route repair, and final safety guard are unchanged. A new opt-in comfort filter
conditions longitudinal requests before that guard. This is an explicit hybrid
controller change, not a learned improvement or a promoted policy.

## What changed

1. Audited recorded lane transitions against the original network connections.
   The v04 car left `mlk_on_0` for `sb1_0` with **32.23 m of ramp remaining**.
   This has no matching graph connection. Three seconds of mainline occupancy
   did not verify a proper merge. The audit records crossing coordinates and
   missing/unknown mappings; no painted-line, gore or barrier masks are verified.
2. Kept physical acceleration bounds at **−4.5 to +2 m/s²**, preserving raw
   actor values and reporting range clipping separately from comfort and safety
   changes. No unit rescaling, larger vehicle limit or hidden actor replacement.
3. Added anticipatory following and speed targets, then limited routine
   longitudinal acceleration changes to **3 m/s³**. Desired following gap is
   `2.5 + 1.8 × speed` metres; gap gain is 0.15/s², relative-speed gain 0.8/s,
   and speed-limit approach gain 0.8/s. The previous acceleration is the final
   applied value. The original guard runs last and can bypass smoothing for
   required braking. Raw/conditioned/applied commands and selected leaders are saved.

## Saved comparison

Same seed/network/demand/weights and final guard. v04 uses saved evidence; only
v05 was newly run, with the same 40-second maximum horizon.

| Measurement | v04 | v05 |
|---|---:|---:|
| Goal reached / simulation time | Yes / 28.5 s | Yes / 28.4 s |
| Peak longitudinal jerk | 60.00 m/s³ | 3.00 m/s³ |
| RMS longitudinal jerk | 15.00 m/s³ | 1.17 m/s³ |
| 95th-percentile absolute longitudinal jerk | 31.15 m/s³ | 3.00 m/s³ |
| Stops / contacts / overspeed steps | 0 / 0 / 0 | 0 / 0 / 0 |
| Maximum speed | 26.820 m/s | 26.439 m/s |
| Background emergency-braking warnings | 3 | 2 |
| Range-clipped commands | 251/284 | 262/283 |
| Raw commands changed overall | 255/284 (89.79%) | 281/283 (99.29%) |
| Guard acceleration changes after preceding control stage | 137 after range clipping | 0 after comfort filter |
| Rejected unsafe proposals | 0 | 0 |
| Unconnected ramp-to-mainline transition | 1 | 1 |

The v05 crossing occurred at 0.8 s with **33.41 m of ramp remaining**. The early
crossing remains unresolved, even though the car reaches the goal. This is not a
proper-merge or 5% raw-intervention pass. Lower downstream guard activity must
not be presented as less dependence on external control: overall raw-command
modification increased to 99.29%. Guard reason flags describe the conditioned
`guard_input_acceleration`, not the raw actor request.

Jerk above is the finite difference of the applied scalar longitudinal
acceleration at 0.1-second steps, **not lateral or full occupant jerk**. The new
trace's graph-selected leader minimum gap was 11.89 m, minimum time headway
1.875 s and minimum closing TTC 31.10 s; these are not all-direction conflict
metrics. No native collisions or teleports. The guard itself remains unchanged;
its existing continuous-motion and constant-velocity prediction limitations apply.

## Verification and retained evidence

**40 tests plus 16 subtests passed**, including five crossing-audit tests and
five comfort tests covering emergency-braking bypass and the final legal cap.
An independent subagent implemented the topology audit and reviewed guard order.
All 283 applied steering values equal the actor requests. Maximum pose readback
error was 1.14e-13 m. The new SUMO loop took **3.813 seconds**, excluding startup,
tests and development. No new renderer, simulator campaign or optimizer updates.

- [Full comparison metrics and provenance](evidence/gpudrive-i110-v05/results.json)
- [Final configuration](../configs/evaluation/gpudrive-i110-v05.json)
- Local raw artifacts: `outputs/gpudrive-i110/v05/policy-01` (recording, raw
  observations/actions, source snapshots, runtime log, resolved config, metrics).
- Local before/after summary: `outputs/gpudrive-i110/v05/comparison.json`.

Next unresolved work is to represent/enforce the intended merge corridor and
validate steering there, then reduce dependence on the comfort controller with
compatible learning. No training is authorized by this result. Google engineering
guide incremental synchronization remains pending; generated mirror unchanged.
