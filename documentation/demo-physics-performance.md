# Headless physics demo performance

## Runs and scope

This is a matched before/after measurement of the scripted, single-car obstacle-bypass demo. Both runs passed all demo gates and used seed 101, the same resolved configuration, 50 simulated seconds, and 6,000 physics ticks. The settings were physics 120 Hz, control 60 Hz, planner 10 Hz, render 30 Hz, and LiDAR 20 Hz. Both were headless and unpaced (`gui=false`, `paced=false`); braking limits, planner settings, controller, and safety gates were unchanged.

| Run | Variant | Startup wall time | Profiled loop wall time | Wall / simulation | Simulation / wall |
|---|---|---:|---:|---:|---:|
| [Baseline, `20261006T083017Z-0bff7a1b`](../outputs/vehicle_obstacle_bypass/20261006T083017Z-0bff7a1b/probe-result.json) | scalar safety predicates; growing full-file evidence rewrites | 9.9770 s | 92.5782 s | 1.8516 | 0.5401× |
| [Optimized, `20261006T083434Z-d4ac9399`](../outputs/vehicle_obstacle_bypass/20261006T083434Z-d4ac9399/probe-result.json) | NumPy safety predicates; immutable per-second evidence chunks | 9.8147 s | 45.4892 s | 0.9098 | 1.0992× |

Startup is reported separately from the simulation loop. The loop time fell by 47.0890 s (50.86%). The wall/simulation ratio is wall seconds divided by 50 simulated seconds; the reciprocal is the simulation rate shown above. The optimized result ran faster than simulation time in this headless configuration. This is not a real-time claim for the GUI, other machines, highway speeds, or broader scenarios.

## GUI preview evidence

Both GUI runs passed all gates for 50 simulated seconds at the same 120/60/10/30 Hz physics/control/planner/render rates. The follow-camera run was unpaced; the overview run used pacing. Both used the same safety and controller settings.

| Run | Camera / pacing | Startup | Profiled loop | Wall / simulation | Render total | View update total | CPU-suite overlap |
|---|---|---:|---:|---:|---:|---:|---|
| [Follow, `20261006T084058Z-22b1b4af`](../outputs/vehicle_obstacle_bypass/20261006T084058Z-22b1b4af/probe-result.json) | follow / unpaced | 10.2305 s | 64.6531 s | 1.2931 | 40.3755 s | 4.1687 s | final 22.5 s |
| [Overview, `20261006T084441Z-6c11d5c6`](../outputs/vehicle_obstacle_bypass/20261006T084441Z-6c11d5c6/probe-result.json) | overview / paced | 10.1464 s | 67.1864 s | 1.3437 | 39.1073 s | 8.8057 s | none |

Both GUI runs saved seven previews at 1, 9, 14, 18, 23, 30, and 40 simulated seconds. The [14-second follow preview](../outputs/vehicle_obstacle_bypass/20261006T084058Z-22b1b4af/preview-14s-follow.png) was visually inspected: the cyan route passes left of the orange obstacle, and wheel geometry is upright. The overview files include [1 s](../outputs/vehicle_obstacle_bypass/20261006T084441Z-6c11d5c6/preview-01s-overview.png) and [40 s](../outputs/vehicle_obstacle_bypass/20261006T084441Z-6c11d5c6/preview-40s-overview.png). These are still-image fallbacks, not a recorded movie.

The full CPU test suite ran for 22.5 seconds during the latter portion of the follow-camera run, so its timing is confounded. The isolated overview run avoids that overlap, but changes both camera and pacing relative to the follow run. Both GUI profiles took longer than the 50 simulated seconds; the faster-than-simulation result applies only to the headless, unpaced run and does not establish interactive real-time performance.

## Blocked and sensor-dropout checks

Two additional headless runs exercised the stopped-obstacle and LiDAR-dropout branches with the optimized code. Both ran the full 50 simulated seconds (6,000 physics ticks), passed every configured gate, recorded no sensor/runtime errors, and had no source changes during execution. Their manifests record the source hashes.

| Run | Mode | Startup | Profiled loop | Minimum clearance | Final stop X | Barrier X | Result |
|---|---|---:|---:|---:|---:|---:|---|
| [Blocked, `20261006T084603Z-c3fa2a45`](../outputs/vehicle_obstacle_bypass/20261006T084603Z-c3fa2a45/summary.json) | blocked obstacle | 9.6464 s | 41.4683 s | 4.4618 m | 37.0396 m | 45 m | passed |
| [Dropout, `20261006T084658Z-9b067a37`](../outputs/vehicle_obstacle_bypass/20261006T084658Z-9b067a37/summary.json) | LiDAR dropout at 12 s | 9.9372 s | 43.0682 s | 13.4338 m | 28.0284 m | 45 m | passed |

The recorded safety gates include observing the obstacle fault, full braking, and stopping before the barrier in blocked mode; dropout mode also requires the vehicle to be moving before sensor loss and to brake promptly after dropout. Three optimized pass-mode starts are now represented across the headless, follow-camera, and overview runs above. These are successful fresh starts for this fixture, not a long-soak reliability result.

## Stage timings

Times are the sums of non-overlapping measured scopes inside the loop. Each scope ran the same number of calls in both runs. `loop_other` is the per-iteration residual, so the stage totals cover the profiled loop. The evidence scope includes per-second checkpoints and event evidence; final full JSON files are written after the loop and are not included in this stage.

| Stage | Calls | Baseline total | Optimized total | Change |
|---|---:|---:|---:|---:|
| Scan conversion | 3,000 | 0.652900 s | 0.620250 s | −5.00% |
| Planner | 500 | 2.758668 s | 0.179491 s | −93.49% |
| Safety and route-envelope checks | 3,000 | 22.113997 s | 2.556780 s | −88.44% |
| Control | 3,000 | 0.433327 s | 0.412418 s | −4.83% |
| Physics step and actuation | 6,000 | 9.899003 s | 9.336441 s | −5.68% |
| State sampling | 6,000 | 1.615120 s | 1.578483 s | −2.27% |
| Contact and collision geometry | 6,000 | 1.231477 s | 0.983847 s | −20.11% |
| Wheel geometry | 1,500 | 1.946082 s | 1.929906 s | −0.83% |
| View update | 1,500 | 4.089394 s | 4.255846 s | +4.07% |
| Render | 1,500 | 20.830109 s | 21.465623 s | +3.05% |
| Sensor evidence processing | 1,500 | 0.109197 s | 0.107530 s | −1.53% |
| Evidence writes in loop | 55 | 26.516683 s | 1.697436 s | −93.60% |
| Progress reporting | 10 | 0.001254 s | 0.001168 s | −6.81% |
| Loop residual | 6,000 | 0.381037 s | 0.363986 s | −4.47% |

The median/p95 safety call fell from 5.297/15.042 ms to 1.030/1.238 ms. Planner median/p95 fell from 3.669/14.454 ms to 0.141/1.548 ms. Evidence-write median/p95 fell from 477.053/995.894 ms to 33.084/36.384 ms. Render median time was similar (13.456 vs 13.389 ms); its p95 was 15.732 vs 20.871 ms, so rendering remains a major cost and showed run-to-run tail variation.

The optimized profiler estimated 1.107 μs of instrumentation overhead per measured scope, or about 0.0372 s over 33,565 scopes (approximately 0.08% of loop time). The baseline run’s profiler was calibrated before it knew the final scope count, so its stored total-scope estimate is zero and is not compared here.

## Behavior and evidence checks

Both runs produced 6,000 trajectory rows and 500 planner records, with identical tick sequences and no differences in planner tick, status, reason, selected path, or source. Per-row categorical planner, safety status/reason, phase, control tick, and route-adoption decisions matched. The run-specific episode identifier differs as expected. `lidar.nearest_gap_m` was null in one run and numeric in the other on four rows, while safety status and braking decisions still matched.

Continuous telemetry stayed close: the largest per-component position difference was 7.63×10⁻⁶ m, speed difference 1.68×10⁻⁶ m/s, yaw difference 4.24×10⁻⁷ rad, and tracking-error difference 1.62×10⁻⁶ m. Planner sensed-bound lateral extrema differed by at most 0.003671 m. LiDAR return timing varied between unpaced runs: oldest-return age differed by up to 0.05 s, corridor point count by up to three points, and nearest-gap values by up to 0.001419 m when both were present. These small scan-timing differences explain why all floating telemetry is not bitwise identical; the discrete plan and safety decisions remained the same.

The optimized run’s 50 immutable `hivemind-traffic-evidence-chunk-v1` files cover ticks 1–6,000 at 120 ticks per chunk. Concatenating their trajectory and planner records reproduced the final `trajectory.json` and `planner.json` exactly. The final full files remain available for existing consumers; the loop no longer rewrites their growing contents every simulated second.

The saved baseline scans `obstacle-observed`, `path-adopted`, `planner-scan-00912`, and `planner-scan-00924` were also checked against scalar predicate references. Each contained 2,569–2,589 valid returns. The vectorized straight-brake decisions and route-envelope hit predicates matched the scalar results on all four scans. The 11 CPU tests in `tests/test_braking_vectorization.py` passed.

## Waypoint inference replay check

Three saved Isaac Lab evaluation traces were compared: [headless, `20261006T083935Z-7e8f09a7`](../outputs/waypoint_demo/20261006T083935Z-7e8f09a7/evaluation/trace.json), [GUI overview, `20261006T084240Z-dbe28c5d`](../outputs/waypoint_demo/20261006T084240Z-dbe28c5d/evaluation/trace.json), and [hardened-launcher headless repeat, `20261006T085125Z-1f22039f`](../outputs/waypoint_demo/20261006T085125Z-1f22039f/evaluation/trace.json). All used seed 43, 1,800 control steps, one environment, and the same checkpoint SHA-256 (`f67f1fa0…cdeb427`). Each passed with 20/20 completed episodes, no failures or timeouts; the GUI run also produced its requested screenshot. The hardened repeat reports unchanged checkpoint and source snapshot.

The traces each contain 180 samples (one every ten control steps). Step indices and all sampled position, target, raw-action, and applied-action values matched exactly across all three traces: zero categorical mismatches and zero numeric difference. This supports repeatability for the sampled trace on this machine and simulator build, not cross-machine determinism. The remote USD asset is recorded by URI hash only; its downloaded content was not hashed.

The final CPU suite reported 870 passed and 10 skipped. The successful demo starts and this suite do not establish long-soak reliability or broader scenario coverage.

## What the result supports

In this fixed, headless 50-second pass-mode run, NumPy batching removed most measured planner and route-safety time, while append-only one-second chunks removed repeated serialization of the growing trajectory and planner histories from the loop. Final full JSON evidence was preserved. Physics, render, sensor cadence, speed limits, and safety gates were unchanged. Both outputs passed; neither run used SUMO or training. The result is a demo-readiness performance comparison, not evidence of a learned policy, general traffic performance, or a safety guarantee beyond the tested fixture.
