# Scaling foundations: what changed and why

Date: October 3, 2026. Active workspace: `highwaysim` only.

## Purpose

Measure the cost of additional vehicles before extending the two-car cooperative
demo. Traffic population, number of independent sensors, decision frequency,
communication fan-out, and visual detail are different workload controls.
Increasing the number of cars does not require giving every car a LiDAR.

## Changes in this increment

| Change | Purpose | Technology and boundary |
| --- | --- | --- |
| Versioned state/time/identity contract | Prevent mismatched positions, reused IDs, and stale updates between components | Python dataclasses, explicit SUMO front-bumper coordinates, reversible USD-safe identities; adopted by the new probe, not yet all legacy adapters |
| Seeded fleet configuration | Reproduce AV/human assignments and vary population and decision rate | Validated JSON configuration and deterministic role selection; tags are not evidence of coordination |
| SUMO capacity harness | Separate native-driver cost from external control overhead | TraCI subscriptions, one simulation step per action batch, native driving versus independent 2/10 Hz speed commands |
| RTX capacity harness | Separate geometry count from independent LiDAR count | Headless Isaac Sim, simple USD boxes, RTX LiDAR callbacks, bounded process supervision and GPU-memory guard |
| Startup sensor refresh | Avoid treating scans aged during startup garbage collection as current | Stationary sensor updates after GC setup, fresh-packet validation, then sensor/traffic time alignment |
| Workload arithmetic and report | Make scaling assumptions inspectable | Sample-rate, recording-volume and communication-delivery calculations plus measured CPU/RAM/frame-time results |

## Key terms to be able to explain

- **SUMO native driver:** SUMO's built-in car-following and lane-changing models.
  This is suitable for background traffic without running a separate Python
  decision controller or RTX sensor on each car. It is a model of traffic, not
  a proven replica of every human driver's behavior.
- **Decision frequency:** how often the external controller chooses an action.
  SUMO keeps advancing motion between decisions. Reducing this frequency does
  not automatically reduce LiDAR capture or rendering cost.
- **Sensor fidelity:** the detail and timing of an observation. A vehicle reading
  perfect SUMO state must not be presented as equivalent to a LiDAR-only vehicle.
- **p95 frame time:** 95% of measured frames took no longer than this value.
  At 30 Hz the budget is about 33.3 ms; p95 below budget still permits late frames.
- **Real-time factor:** simulated seconds divided by elapsed wall seconds.
  Near 1 means paced real time; a fast isolated probe is not a complete live demo.
- **Communication fan-out:** how many receivers process each publication.
  All-to-all delivery grows quadratically; bounded neighbors can reduce processing.
  Logical deliveries are not the same thing as physical radio transmissions.

## Demonstrations and reruns

Run these from the repository root using the configured traffic Python environment.
The commands contain no username-specific paths. RTX probes require an installed
Isaac runtime configured through the project's existing runtime discovery.
Windows process counters and NVIDIA GPU telemetry are machine-specific instrumentation;
the current measured report is not a cross-platform performance guarantee.

```text
python -m pytest -q
python -m experiments.benchmark_traffic --config experiments/configs/scaling_probe.json
python -m experiments.benchmark_traffic --suite
python -m experiments.benchmark_rtx --single 20 2
python -m experiments.verify_cooperative_refresh --long
```

The probes intentionally run without a viewer to isolate costs. To show the
existing two-car cooperative simulation instead:

```text
python scripts/demo_cooperative_lidar.py --check
python scripts/demo_cooperative_lidar.py
```

The check command resolves the Isaac launch command without opening the GUI;
it is not a complete dependency or sensor validation.
New results and manifests stay under `outputs/`; console logs belong in `logs/`.
The [capacity report](capacity-results.md) records the actual tested cases,
measurements, failures, assumptions, and evidence locations.

## Next integration gates

1. Connect the shared state contract to the live fleet and scene adapters.
2. Add 18 native SUMO background cars around the two independently sensed AVs.
   Preserve all cars as sensor-visible obstacles and handle spawn/removal explicitly.
3. Validate the integrated 20-car scene, including collisions, scan age, command
   timing, frame-time tails, and memory growth; then consider 50 total cars.
4. Add bounded-neighbor V2V and explicit sensed/lightweight/background counts.
   AV market share and LiDAR-equipped share must remain separate parameters.
5. Run matched-seed human/native, independent AV, and communicating AV comparisons
   across fleet ratios. Report travel time, throughput, safety, fairness and compute
   cost; do not infer traffic benefits from capacity benchmarks.

## Remaining limitations

The 120-second headless two-car run passed with zero control-loop delays over
100 ms. It produced 1,197 saved sensor frames per car across 1,200 control steps,
not one unique scan at every step. Observed process RAM increased by approximately
41.2 MiB over that bounded run. Neither long-run memory stability nor GUI
performance is established by this test. Raw RTX output is not bitwise repeatable.

This increment does not implement arbitrary-fleet LiDAR control, neighbor-limited
communication, learned coordination, realistic vehicles, or the teammate's road
art. The source and learning notes are in the repository; Google Docs has not
been synchronized during this increment.

The SUMO, OpenUSD, viewer, and reproducible-experiment skills informed the
separation of responsibilities, explicit state contract, bounded tests,
matched seeds, and retained source/configuration evidence.
