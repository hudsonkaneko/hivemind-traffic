# Workstation capacity and roadmap foundations

Generated from recorded probes, October 3, 2026.

Hardware: AMD Ryzen 9 9950X3D 16-Core Processor; OS-reported 61.6 GiB RAM; NVIDIA GeForce RTX 5070, 12227 MiB VRAM, driver 610.74.

## Measured SUMO workload

Three predeclared seeds per row. These are sparse straight-road workloads, not proof of coordination benefits. AV tags select deterministic roles; only native driving and independent scripted speed commands are implemented. No LiDAR, learning or V2V runs in this table.

| Cars | Control | Hz | Runs passed | Mean step ms | Mean p95 ms (seed range) | CPU cores needed at real time* | Sampled combined RAM MiB |
| ---: | --- | ---: | --- | ---: | --- | ---: | ---: |
| 2 | sumo_native | 2 | 3/3 | 0.165 | 0.250 (0.242–0.263) | 0.002 | 66.4 |
| 2 | scripted_speed | 2 | 3/3 | 0.170 | 0.264 (0.261–0.268) | 0.002 | 67.0 |
| 2 | scripted_speed | 10 | 3/3 | 0.186 | 0.285 (0.267–0.296) | 0.002 | 67.2 |
| 20 | sumo_native | 2 | 3/3 | 0.512 | 0.597 (0.579–0.623) | 0.005 | 67.4 |
| 20 | scripted_speed | 2 | 3/3 | 0.563 | 0.763 (0.730–0.811) | 0.006 | 68.0 |
| 20 | scripted_speed | 10 | 3/3 | 0.718 | 0.852 (0.838–0.863) | 0.008 | 68.4 |
| 50 | sumo_native | 2 | 3/3 | 1.033 | 1.124 (1.106–1.136) | 0.011 | 68.5 |
| 50 | scripted_speed | 2 | 3/3 | 1.141 | 1.594 (1.575–1.605) | 0.012 | 68.8 |
| 50 | scripted_speed | 10 | 3/3 | 2.186 | 2.658 (1.940–3.133) | 0.024 | 68.8 |
| 100 | sumo_native | 2 | 3/3 | 2.916 | 3.140 (3.078–3.205) | 0.029 | 69.3 |
| 100 | scripted_speed | 2 | 3/3 | 3.163 | 4.584 (4.221–4.888) | 0.034 | 69.9 |
| 100 | scripted_speed | 10 | 3/3 | 4.551 | 4.909 (4.694–5.051) | 0.050 | 70.1 |
| 200 | sumo_native | 2 | 3/3 | 5.508 | 5.887 (5.865–5.921) | 0.056 | 70.8 |
| 200 | scripted_speed | 2 | 3/3 | 6.171 | 8.771 (8.744–8.786) | 0.062 | 70.9 |
| 200 | scripted_speed | 10 | 3/3 | 8.144 | 8.945 (8.901–9.012) | 0.089 | 70.8 |

*CPU-seconds (Python + SUMO) divided by simulated seconds: a real-time arithmetic estimate, not an observed whole-PC utilization percentage. Windows CPU counters are coarse for short runs. Timings include subscriptions, state validation, serialization and trajectory hashing. Mean p95 is the average of per-run percentiles, not a pooled percentile or confidence interval.

All 45 cases retained; same-seed repeat matched: True. No failed runs were excluded. Startup/network generation/warmup and final artifact writes are outside steady-loop timing.

## Measured RTX workload

Unpaced headless simple boxes, 30 simulated render steps/s, 32-emitter LiDARs at 7200 firing patterns/s. Viewport updates disabled. CPU readback copies included; perception, SUMO, GUI, dynamics and raw-scan disk recording excluded. One run per case plus a repeated 2-car/2-sensor case: exploratory, not a robust capacity limit.

| Cars | LiDARs | Sensor gate | Median frame ms | p95 frame ms | Worst frame ms | Sampled whole-GPU peak MiB |
| ---: | ---: | --- | ---: | ---: | ---: | ---: |
| 2 | 0 | True | 1.71 | 1.94 | 3.85 | 4539 |
| 2 | 1 | True | 8.70 | 9.96 | 11.47 | 4523 |
| 2 | 2 | True | 12.83 | 15.79 | 17.13 | 4521 |
| 20 | 2 | True | 13.98 | 16.09 | 17.54 | 4520 |
| 50 | 2 | True | 15.17 | 16.78 | 19.35 | 4520 |
| 20 | 4 | True | 23.25 | 30.44 | 38.78 | 4759 |
| 2 | 2 | True | 13.50 | 16.65 | 18.48 | 4499 |

The zero-sensor row is a scene-update baseline with no active camera viewport, NOT a visual-render benchmark. GPU telemetry is sampled about once per second across startup and measurement and includes desktop applications; peaks may be missed. Sensor callback counts and errors are retained per run. A passing sensor gate does not mean every frame met 33.3 ms.

The separate exploratory smoke peak was 7,343 MiB global GPU memory; suite peaks ranged 4,499–4,759 MiB. Cache/desktop/startup effects prevent treating these differences as exact per-car memory. Reserve headroom; do not extrapolate a maximum fleet from these short samples.

## Arithmetic: what additional cars actually multiply

Nominal single-return sample opportunities per LiDAR-second = 32 × 7200 = 230,400. This is configured workload, not measured valid hits. Full-return recording projection assumes 20 bytes/sample, no compression, headers or duplicate scans. Actual archive size depends on hit rate, fields and capture behavior.

| Cars | All cars sensed: nominal samples/s | Two sensed cars: nominal samples/s | All-sensed payload GB/hour | All-to-all receiver deliveries/s | Eight-neighbor deliveries/s |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 2 | 460,800 | 460,800 | 33.18 | 20 | 20 |
| 20 | 4,608,000 | 460,800 | 331.78 | 3,800 | 1,600 |
| 50 | 11,520,000 | 460,800 | 829.44 | 24,500 | 4,000 |
| 100 | 23,040,000 | 460,800 | 1658.88 | 99,000 | 8,000 |

Communication calculation assumes every car is an AV publishing at 10 Hz: A(A−1)f logical deliveries, or A·min(8,A−1)f with eight neighbors. This is receiver-processing work, NOT physical radio broadcast bandwidth. Neighbor limiting is a recommendation, not implemented in the existing all-to-all V2V bus. Its 4096-pending-delivery bound is another reason not to attach a large fleet unchanged.

## Recommended operating tiers

1. **Sensor-critical AVs:** initially two cars retain full LiDAR, 10 Hz control, tracking and explicit V2V. Preserve safety/freshness gates.
2. **Background cars:** SUMO native car-following/lane-changing at the same simulation timestep; no individual RTX sensor or Python controller required. They remain visible geometry and obstacles to nearby AV sensors.
3. **Optional lightweight AVs:** lower-frequency scripted or policy decisions with SUMO maintaining motion between decisions. The probe implements 2 Hz scripted speed commands, not a new autonomous-driving policy. Declare this fidelity difference in results.

First integrated target: two fully sensed cars plus 18 native background cars; then test 50 total. These are recommended next validation targets, not a claim that the existing two-car controller already controls an arbitrary fleet. Four sensors already approach the 30 Hz frame budget before controller/recording overhead. Benchmark 4+ sensors explicitly before promoting them.

Further mitigations: instance repeated assets; bound nearby communication; keep one displayed camera; disable point-cloud drawing for statistics runs; use documented lower-detail geometry without hiding sensor-relevant obstacles; log scalar metrics routinely and retain raw scans only for bounded validation or explicit sampling. Do not lower sensor fidelity silently or let the camera position change the underlying driving experiment.

At 100 AVs, reducing decision rate from 10 Hz to 2 Hz cuts speed-command count from 1000/s to 200/s (80%). It does not reduce SUMO physics cadence, LiDAR work, or the size of individual command batches; p95 may improve much less than average cost. Scene complexity and perception algorithms may become the next bottlenecks.

## Startup refresh validation

- 25 s: passed=True; integrity=True; first freshness={'ego': True, 'peer': True}; late >100 ms=0; behavior=True; `outputs\cooperative_lidar\20261003T083245Z-ideal-5fac60`.
- 25 s: passed=True; integrity=True; first freshness={'ego': True, 'peer': True}; late >100 ms=0; behavior=True; `outputs\cooperative_lidar\20261003T083324Z-ideal-07faa9`.
- 120 s: passed=True; integrity=True; first freshness={'ego': True, 'peer': True}; late >100 ms=0; behavior=True; `outputs\cooperative_lidar\20261003T083403Z-ideal-841c43`.

The implementation refreshes stationary scans AFTER expensive GC preparation, then anchors sensor time. It does not advance SUMO during refresh or relax stale-sensor limits. It is not proof of exact RTX repeatability or a fix for GUI memory growth.

## Reproduce and evidence

Run from the canonical project root with the traffic environment:

```text
python -m pytest -q
python -m experiments.benchmark_traffic --suite
python -m experiments.benchmark_rtx
python -m experiments.verify_cooperative_refresh --long
```

Traffic suite: `outputs\traffic_scaling_suites\20261003T082422Z-2ddca072`. RTX suite: `outputs\rtx_scaling_suites\20261003T082837Z-f3c27406`. Full generated values and run references: [capacity-results.json](capacity-results.json). Each run preserves resolved configuration, source copies/hashes, Git state, metrics, and failures.

Original failed CPU smoke (incomplete insertion at 12 m spacing) is retained under `outputs/traffic_scaling/20261003T082342Z-33b8ef3e`. The final sparse fixture uses 25 m spacing and checks that the requested population actually departed. It is not a congestion study.

Guidance sources: [SUMO subscriptions/libsumo](https://sumo.dlr.de/docs/TraCI/Interfacing_TraCI_from_Python.html) and [NVIDIA performance handbook](https://docs.isaacsim.omniverse.nvidia.com/latest/reference_material/sim_performance_optimization_handbook.html). Skill guidance informed provenance, matched seeds, sensor/traffic separation, and shared state conventions.
