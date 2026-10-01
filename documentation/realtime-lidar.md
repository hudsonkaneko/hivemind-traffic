# Continuous lidar: implementation and learning guide

Status: bounded real-time prototype; consult [the results report](realtime-lidar-results.md) for actual validation outcomes and retained failures.

## What changed

The previous controller froze SUMO while waiting at least 18 renderer updates after every 0.1-second traffic step. It also compressed a large scan on that critical path. Profiling a five-second baseline measured 20.86 wall seconds: 16.91 seconds in acquisition and 3.67 seconds in scan compression. That short run was a performance probe, not a completed obstacle maneuver.

The opt-in `--realtime` path instead advances SUMO once per 0.1-second control interval and interpolates its two authoritative poses over three 30 Hz renderer ticks. RTX scans accumulate while the visual/sensor pose moves. A controller decision consumes the most recently completed scan; it never reads obstacle truth. Uncompressed raw NPZ evidence avoids compression latency. A wall-clock scheduler paces the run, and lateness is recorded rather than hidden by changing simulation time.

The original launcher without `--realtime` retains sample-and-hold behavior.

The newer [two-car cooperation demo](cooperative-lidar.md) builds on this baseline with independent sensors, local tracking and timestamped messages.

## Timing contract

Each run saves `timing-contract.json`, raw packets, vehicle states, receipt ages, timing profiles, and a source snapshot. After stationary sensor warm-up:

`traffic_time = traffic_origin + (sensor_time - sensor_origin)`

The traffic and sensor clocks must remain aligned within 0.1 ms. The renderer tick is 1/30 second; the controller and SUMO step are 0.1 second. Isaac's `RenderingManager.set_dt` configures the run loop and timeline coherently before play. Setting only the timeline target FPS was insufficient on this installed runtime.

**A scan is not a render frame.** The GMO frameStart/frameEnd metadata in our captures describes the latest render interval, whereas a complete rotary scan covers approximately 0.1 second. Acquisition times come from `timestampNs + timeOffsetNs` for each ray. Complete scan duration must be 90–110 ms, with no future rays beyond a 2 ms numeric tolerance. Oldest-ray simulation age and wall-clock packet receipt age each have a 250 ms limit. Stale observations request braking and no new lane change; a maneuver already accepted by SUMO may continue laterally.

Correction from the two-car milestone (2026-09-30): the 90–110 ms check above is the older fixture's return-span proxy, not a reliable general test of scan completion. Missing hits can shorten the span even when acquisition completed. The fleet path instead validates the native `scanComplete` flag and configured scan period and uses a conservative scan-origin age; the legacy caller remains unchanged for reproducibility.

## Sensor and motion choices

- Motion BVH remains enabled to trace a moving sensor.
- The new path explicitly requests Cartesian world-space output. It does not transform a whole moving scan using the car's current pose.
- `--world-motion COMPENSATED` and `NONCOMPENSATED` are diagnostic output modes, not a toggle for Motion BVH. They are compared using static obstacle surface alignment, independently of control.
- `--sensor-quality native` retains Example_Rotary's 128 emitters and 36,000 Hz firing pattern.
- `--sensor-quality demo` retains its first 32 emitters (the full original elevation sweep) and reduces pattern frequency to 7,200 Hz: 0.5-degree horizontal steps at 10 rotations per second. Native noise remains enabled. This is a synthetic lightweight profile, not a calibrated commercial lidar.
- Neither output mode nor a passing demo proves general sensor fidelity. The fixture has static boxes and a known straight two-lane map, not dynamic traffic or physics steering.

## Portable commands

Run from the repository root with its Python environment active, or replace `python` with `.venv\Scripts\python.exe` on Windows. External Isaac Sim and SUMO are still required; `ISAAC_SIM_PATH` and `SUMO_BINARY` allow portable runtime discovery.

Baseline:

```text
python scripts/demo_live_lidar.py --mode avoid --speed 6 --gap 40
```

Experimental continuous mode, explicit settings:

```text
python scripts/demo_live_lidar.py --mode avoid --speed 6 --gap 40 --seconds 60 --realtime --world-motion NONCOMPENSATED --sensor-quality demo
```

Add `--headless` for a sensor-only benchmark, `--blocked-lane` for stopping, or `--dropout-step 20` to stop accepting new packets after two seconds. `--unpaced` measures maximum throughput; it is not a real-time validation run. Close other GPU-heavy applications for controlled performance comparisons.

The realtime defaults select the demo density, NONCOMPENSATED world output, and deferred cyclic garbage collection for the <=60-second run. Ordinary Python reference counting remains active. A full collection before the timed work and restoration afterward avoid a measured 176 ms full-collection pause inside the control loop. The original GC state is restored even on Python exceptions. This bounded policy is not yet validated for an indefinitely running viewer. `--gc-mode normal` reproduces the original collector behavior; `gc-profile.json` records collection timings and summaries record process memory.

The script owns the timeline. Let the demo close itself; using the Kit stop/pause/scrub controls invalidates the timing contract and aborts the run.

Audit a completed run:

```text
python scripts/audit_realtime_lidar.py outputs/live_lidar/RUN_ID
```

## What counts as success

The combined gate requires safe pass-and-return (or the declared blocked/fault fallback), real-time factor between 0.95 and 1.05, zero cumulative control deadline lateness events above 100 ms, and maximum per-scan 95th-percentile static point-to-box alignment error of 30 cm. Runtime startup, warm-up, evidence hashing, and shutdown are excluded from steady-state real-time factor, not silently included as simulation progress. This is a desktop soft-real-time experiment, not a hard-real-time guarantee.

Alignment is scored only for fresh new packets admitted to the controller. Rejected stale packets remain recorded but are not reinterpreted as new observations from the car's later pose. The dropout case must demonstrate braking to zero with no new lane requests, while maintaining road/obstacle clearance.

## References and workflow

- [NVIDIA rendering-rate API](https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.core.rendering_manager/docs/api.html): coherent run-loop timing; checked against the installed runtime source.
- [NVIDIA lidar output conventions](https://docs.omniverse.nvidia.com/kit/docs/omni.sensors.nv.lidar/latest/lidar_extension.html): output frame and compensation differ from Motion BVH.
- [RTX performance considerations](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/sensors/isaacsim_sensors_rtx.html): sensor workload and motion processing costs.

The SUMO, viewer, and reproducible-experiment skills informed single motion authority, separate update rates, preserved failure evidence, and explicit performance/safety gates. No external runtime files were modified. This implements only the timing/pose portion of the broader future state/time/identity contract.
