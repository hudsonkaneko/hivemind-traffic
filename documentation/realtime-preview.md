# Live obstacle-bypass preview: measured 1x target

**Status: usable preview; consistent real-time GUI playback is not verified.**
The optional mode targets 1x and exposes timing honestly. A headless run passed
the soft-real-time gate; visible-window runs still lag during driving, even
when their whole-episode average approaches 1x. The flag is not a guarantee.

This is the existing **scripted**, one-car PhysX/RTX LiDAR experiment, not a
recording, SUMO-driven movement, a trained driving policy, or highway-speed test.
The full-size car still travels at its validated maximum of 3 m/s (10.8 km/h).
Real time means one simulated second per wall-clock second, not a faster car.

## Launch

From the canonical repository root in the configured traffic environment:

```text
python scripts/demo_obstacle_bypass.py --real-time
```

On this workstation, in Command Prompt (each line is a complete command):

```bat
cd /d C:\Users\hudso\Documents\highwaysim
.venv-traffic\Scripts\python.exe scripts\demo_obstacle_bypass.py --real-time
```

Use the in-window **Overview**, **Follow car**, **Pause / resume**, path and
LiDAR-point buttons. Use these pause controls rather than the Kit timeline.
The HUD reports measured playback rate and accumulated lag. The bounded scene
runs for 50 simulated seconds, including settling and the final stopped hold,
then closes after writing evidence. Startup and final serialization take extra
time; they are not part of the 50-second live segment.

```text
python scripts/demo_obstacle_bypass.py --real-time --camera overview
python scripts/demo_obstacle_bypass.py --real-time --points
python scripts/demo_obstacle_bypass.py --real-time --capture
python scripts/demo_obstacle_bypass.py --real-time --check
```

`--capture` adds seven viewport PNGs and one GUI-window capture, not a movie.
`--check` resolves the runtime/settings
without starting Isaac. Run one simulator at a time; background GPU/CPU work,
window changes and point drawing can affect timing. `--unpaced` is incompatible
with `--real-time`. Omit `--real-time` to retain the original graphics profile.
With `--real-time`, a physically valid completed episode can still return a
nonzero exit status because its separate timing gate failed. Inspect
`summary.json` rather than interpreting every failed run as a driving failure.

## What changes and what does not

The optional process-local preview profile uses Isaac's MinimalRendering,
textured-diffuse shading, a 960 x 600 RGB viewport, FXAA, 8 CPU worker threads,
and VSync disabled (Kit's normal sync-to-present behavior is retained). Settings
are supplied for the launched process; no installed runtime, Windows/driver
preferences, dependency versions or other demo implementations are edited.
The default profile remains
1280 x 800 RealTimePathTracing/DLSS with the installed default thread limit.

Physics remains 120 Hz, control 60 Hz, planning 10 Hz and RTX LiDAR acquisition
20 Hz. The optional preview renders at **20 Hz instead of 30 Hz**, reducing
visual smoothness to leave more time for computation. It does not guarantee 1x.
No physics/control/sensor steps are skipped. Motion BVH,
sensor geometry, timestamps, pose validation and all driving/safety gates remain.
Rendering must still advance zero physics ticks. No asynchronous rendering is
enabled. The controller still uses LiDAR XYZ, known road geometry, privileged
odometry and the existing conservative maximum-obstacle-size prior.

The overview camera is cached for an unchanged road. In the lightweight mode,
the cyan ribbon shows the full adopted route (including the traveled part),
rebuilt only when the planner changes it. The pink target ring moves with a USD
transform instead of rebuilding vertices every frame. The original mode keeps
its rolling path ribbon. These are visual updates, not route/controller changes.
Debug meshes may enter raw LiDAR returns; they remain below the existing
braking-height region. They are not claimed to be sensor-invisible.

The quality/thread choices follow the installed SimulationApp API and NVIDIA's
[performance handbook](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/reference_material/sim_performance_optimization_handbook.html).
They are one tested profile, not a general optimal-thread recommendation.

## Timing contract and acceptance

The monotonic wall clock starts at episode tick zero. Every completed render
iteration includes its six physics steps (four in the original profile), controller work, sensor evidence,
optional capture requests and due evidence checkpoint writes. If ahead, it waits
until the 1x deadline; if behind, it reports lag and continues every required
step. Explicit user pause time is excluded; the clock origin is never reset to
hide a stall. End-of-frame intervals are measured, not monitor scanout latency.
The complete expected duration/frame count is required; an early aborted prefix
cannot pass the episode timing gate.

The predeclared soft-real-time gate requires overall simulation/wall ratio
**0.99–1.01**, 95th-percentile accumulated lag **<= 0.10 s**, and maximum lag
**<= 0.50 s**. Startup is excluded, but the entire episode, including its initial
settling and recording, is included. The result also requires every existing
physical/sensor gate. A long stall cannot disappear merely by catching up later.
This is not hard-real-time scheduling or a 20-FPS minimum for every frame.

Each completed `outputs/vehicle_obstacle_bypass/<run-id>/` package retains
`preview-timing.json`, profile and actual renderer settings, all 6,000 state
rows, planner/sensor/wheel evidence, source snapshots and runtime log. Early
failures retain whatever evidence was available; a startup crash has no trajectory.
Bulk local evidence is ignored by Git; the implementation and compact report
are version controlled. An overloaded machine may fail the timing gate even
when its physical episode is valid.

## Validation evidence

Recorded workstation: Ryzen 9 9950X3D, RTX 5070 (12 GB), about 64 GB RAM,
Windows, Isaac Sim `6.0.1-rc.7+release.42383.32955d8d.gl`. All runs below retain
their own source hashes/configurations in `outputs/vehicle_obstacle_bypass/`.
Every completed exploratory run passed all physical, wheel, contact, clock and
sensor gates. **Every GUI run failed the timing gate.**

| Run ID | Variant | Active seconds / 50 simulated | RTF | p95 / maximum accumulated lag, seconds |
| --- | --- | ---: | ---: | ---: |
| `20261006T181411Z-f87bc47c` | Minimal RGB, original 30 Hz / 32 threads | 58.861 | 0.849 | 10.906 / 11.077 |
| `20261006T181610Z-27b0d862` | 30 Hz / 16 threads, VSync off | Startup crash | n/a | n/a |
| `20261006T181749Z-81876710` | Unchanged retry | 61.852 | 0.808 | 12.971 / 13.113 |
| `20261006T182005Z-4e182d90` | Extra display waits disabled, 30 Hz | 63.865 | 0.783 | 14.723 / 14.916 |
| `20261006T182234Z-cfd3b3d5` | 20 Hz / 16 threads | 52.445 | 0.953 | 5.967 / 6.143 |
| `20261006T182556Z-34a48942` | Same, UI hidden | 56.665 | 0.882 | 7.667 / 8.025 |
| `20261006T182827Z-c5dc3817` | 20 Hz / 8 threads, UI restored, camera cache | 50.997 | 0.980 | 5.184 / 5.327 |
| `20261006T183146Z-73d82492` | Full-route and target-transform cache | 50.416 | 0.992 | 4.897 / 5.062 |
| `20261006T183344Z-af62d6b0` | Same profile, headless, no capture | 50.013 | 1.000 | 0.015 / 0.293 |
| `20261006T183525Z-8c44b3ce` | GUI 800 x 500, smaller window, UI hidden | 50.361 | 0.993 | 4.789 / 4.844 |
| `20261006T184201Z-f0aefd72` | Retained GUI profile, no capture | 57.350 | 0.872 | 9.914 / 10.090 |
| `20261006T184340Z-edf13b36` | Extra display waits disabled again, 20 Hz | 60.669 | 0.824 | 12.818 / 13.069 |

The startup crash occurred before scene creation and provides no motion/timing
evidence. It is preserved, not overwritten by the retry. Removing display waits
did not establish a benefit; those overrides were not retained. The final trial
read back their effective values after launch, after physics startup and after
the episode; they stayed disabled, with asynchronous rendering still disabled.
Hiding the UI also hid the demonstration controls and did not solve timing, so
the usable profile retains the editor and custom controls.

These are exploratory workstation measurements, not isolated causal benchmarks:
other desktop work was present, document transport overlapped early trials, and
GPU usage varied. No user applications were closed. A better overall average
must not hide driving-segment lag followed by catch-up while stopped.

For example, inclusive render/update time was 30.00 seconds with the window
versus 13.86 seconds headless in the two adjacent full-route-cache runs. This
localizes much of the difference to the window/render path, but does not prove
which native GUI, presentation, GPU-wait or desktop-compositor stage caused it.
A native CPU/GPU trace and matched foreground/background-load repeats are the
next profiling step. No physics timestep changes, sensor-density reductions,
asynchronous scheduling or lowered acceptance thresholds were used to hide it.

The inspected [GUI capture](../outputs/vehicle_obstacle_bypass/20261006T183146Z-73d82492/preview-window.png)
shows the controls and a real 0.89x / 1.67-second-lag reading during the detour.
This is a still from the live run, not evidence of real-time success.

Automated verification: **880 passed, 11 skipped** in the traffic environment;
**31 passed** for road-view geometry tests in Isaac's Python, including full-path
cache invalidation and switching between transformed and original target-ring
rendering. The configuration check passed. A read-only subagent review identified
the aborted-prefix timing edge case; it was fixed and regression-tested.
Final failure-mode/default-profile runtime regressions are recorded below.

### Final regression runs

- Blocked road `20261006T184735Z-0b2440f0`: every physical/sensor gate passed,
  including full brake and stopping before the barrier. Timing failed:
  50.02293 active seconds, p95/max lag 0.38347/0.57312 seconds.
- Frozen scan `20261006T184925Z-d0d611d4`: every physical/sensor gate passed,
  including timely braking after dropout and stopping before the barrier.
  Timing failed: 50.01757 active seconds, p95/max lag 0.11274/0.35386 seconds.
- Original 30-Hz headless profile `20261006T185049Z-21831b7d`: all existing
  physical/sensor gates and the overall default-mode result passed. Timing is
  diagnostic only without `--real-time`: 50.03136 active seconds, p95/max lag
  2.27485/2.31919 seconds, so it is not a verified real-time run.

Both failure-mode runs used the retained 20-Hz headless profile and completed all 6,000 physics
ticks. This demonstrates why even headless timing must be evaluated per run,
not inferred from the earlier passing run or a rounded 1.000x average.
