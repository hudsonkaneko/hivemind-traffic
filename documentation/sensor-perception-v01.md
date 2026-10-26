# Sensor perception v01 — source rig and forward measurement prototype

Implementation draft, October 25, 2026 (America/Los_Angeles).
**Native results and screenshot evidence are pending. No new policy training
has started for this milestone.** This document describes the implemented
interfaces visible during review; it does not claim driving acceptance.

The [input audit](sensor-input-audit.md) records the old SUMO/physical policy
contracts and privileged shortcuts. Its baseline is the actual checked-out
commit `6b14eb0ccfea1c7970ce920ccc1a5422303a635b` observed during audit. New sensor
files are milestone changes on top of that baseline, not features attributed to
that historical commit.

## Reusable vehicle source

The prepared source is
`vehicles/sim_ready/americano-i7-ev/v08/assets/vehicle.usda`, with package metadata
in `vehicles/sim_ready/americano-i7-ev/v08/model.json` and the standalone stage at
`vehicles/sim_ready/americano-i7-ev/v08/world.usda`. This is a new prepared version
of v07; the parent is preserved. The actual reusable entry composes
`assets/vehicle-sensors.usda` alongside its geometry and physics layers. Loading
that entry through a payload brings the camera and mounts with it: the addition
is not confined to a live scene overlay.

[vehicle_sensor_rig.py](../usd/vehicle_sensor_rig.py) authors the following
source hierarchy under `/Vehicle/Chassis/Sensors`:

- `CameraFront`: a USD perspective camera, chassis translation `(0.8, 0, 1.12)` m.
- `LidarFront`: mount at `(0, 0, 1.25)` m; reserved child `Scan` is created by the
  RTX runtime, not baked into a captured runtime stage.
- `CameraHousing` and `LidarHousing`: visual-only housings with no collision,
  rigid-body or mass changes.

The composed standalone chassis path is
`/World/Vehicles/vehicle_ego/Chassis`; sensor paths are derived from the actual
chassis path rather than a fixed scene root. Camera optical forward maps to
chassis +X, optical up to chassis +Z. Optics are provisional: 18 mm focal length,
36 × 20.25 mm aperture, 0.1–250 m clipping, 1280 × 720 resolution, and authored
10 Hz metadata. Actual delivered cadence requires native measurement; custom
frequency metadata does not itself schedule rendering.

The asset is marked friend-supplied/local-use-only with redistribution rights
unverified. Source code and evidence summaries can be published independently;
do not assume the vehicle binary package is licensed for GitHub distribution.

## Measurement and estimator boundary

[sensor_pipeline.py](../traffic/sensor_pipeline.py) separates measurement packets,
ego-motion estimation and geometric perception. Headers carry sensor/frame IDs,
calibration ID, episode ID, acquisition/delivery times and validity. Truth is permitted at
the simulated sensor generator; downstream perception receives measurement
objects, not an environment or exact object list.

| Component | Implemented prototype behavior | Limitation |
| --- | --- | --- |
| `SeededSensorModel` | Deterministic noisy initial position/yaw and wheel-speed/yaw-rate measurements | Noise settings are engineering assumptions, not calibrated hardware. |
| `EgoEstimator` | Integrates measured speed and yaw rate from an initial localization fix, with growing position uncertainty | Dead reckoning; no map correction, GNSS fusion, slip model or complete six-axis IMU. |
| `detect_lane_rgb` | Searches lower RGB image for two narrow bright neutral boundaries; reports pixel center offset | A paint heuristic, not semantic lane detection. Pixel offset is not metres; confidence is uncalibrated. |
| `SensorPipeline` | Freshness/calibration/frame checks, chassis corridor clearance, local-ID geometric tracking and stop-required output | Forward-only low-speed prototype; no surrounding free-space guarantee, learned detector or complete planner. |

Prototype measurement settings are wheel-speed sigma 0.03 m/s, yaw-rate sigma
0.002 rad/s, episode gyro bias sigma 0.001 rad/s, initial localization sigma
0.15 m and yaw sigma 0.01 rad. These make the estimator imperfect explicitly;
they do not establish real-world accuracy.

Freshness is bounded to 0.2 s. Ego and LiDAR acquisition times must agree within
0.05 s. Empty, malformed, stale, duplicated, misframed or uncalibrated LiDAR
frames fail closed. Missing camera lane evidence also requires stopping. Tracker
IDs are generated locally by [lidar_tracking.py](../traffic/lidar_tracking.py).
Estimated ego pose transforms detections into the estimated local map frame.

Forward clearance uses raw local returns rather than requiring a successful
object classification. The corridor has configurable body dimensions; defaults
are 2.6 m ego half-length and 1.2 m half-width. The runtime must use the actual
prepared vehicle envelope. Its braking threshold is
`v² / 3 + v × (0.2 + scan_age) + 1` metres, a provisional 1.5 m/s² braking model
plus latency/margin, not a validated stopping guarantee. A fresh cloud without
corridor hits means no observed obstacle in the tested corridor, not verified
absence everywhere.

## Native capture plumbing

[native_sensor_capture.py](../traffic/native_sensor_capture.py) wraps the
installed experimental RTX camera API to capture RGB and depth. RGB enters the
measurement pipeline; depth is retained separately for evaluation. No depth,
semantic label, simulator object ID or exact object box enters the RGB lane
heuristic. Camera capture relies on a synchronous render/read barrier; native
verification must prove frame freshness and synchronization rather than infer
it from an image's shape.

The native LiDAR adapter adds prototype 1 cm independent XYZ noise and 1% point
dropout. Successful complete returns are converted from the existing WORLD
RTX packet into chassis coordinates before crossing the measurement boundary.
This conversion still uses simulator pose internally to express the simulated
rays in sensor/body coordinates. It must be distinguished from supplying perfect
world localization to downstream perception. Existing returns are
NONCOMPENSATED; a single frame conversion does not deskew moving actors or the
entire scan. Dynamic/high-speed sensing accuracy remains unverified.

[physical_lidar.py](../traffic/physical_lidar.py) accepts an explicit mount
translation so the runtime sensor can use the source-authored mount rather than
apply the legacy roof offset twice. Existing default mounting behavior is
preserved for old callers.

## Verification and evidence status

The new CPU tests in [test_sensor_pipeline.py](../tests/test_sensor_pipeline.py)
cover deterministic noise, dead reckoning, RGB heuristic failures, stop/gap
behavior, tracking transforms, stale/invalid/duplicate measurements, calibration
and frame rejection, timestamp alignment and configurable vehicle envelope.
These are synthetic fixtures; they do not prove a native camera worked.

[test_vehicle_sensor_rig.py](../tests/test_vehicle_sensor_rig.py) checks camera
axes, chassis-following transforms, visual-only housings, duplicate/missing
chassis rejection, payload load/unload behavior and composed-stage errors. These
are plain USD checks; they do not prove an RTX render or physical drive passed.

CPU validation: 160 targeted tests passed across the pipeline, native adapters,
legacy physical LiDAR/tracking, clock ownership, runner and acceptance checks.
Seventeen separate plain-USD tests passed for the sensor rig and prepared vehicle
composition. Native results remain pending while an unrelated ARACHNID job owns
the GPU; the user requested waiting for that job to finish.

```text
.venv-traffic/Scripts/python.exe -m pytest tests/test_sensor_pipeline.py tests/test_native_sensor_capture.py tests/test_physical_lidar.py tests/test_lidar_tracking.py tests/test_rendered_physics_session.py tests/test_sensor_perception_runner.py tests/test_sensor_perception_probe.py -q
runtimes/simready-validation/venv/Scripts/python.exe -m unittest tests.test_vehicle_sensor_rig tests.test_showcase_vehicle -q
.venv-traffic/Scripts/python.exe -m experiments.verify_sensor_perception --check
.venv-traffic/Scripts/python.exe -m experiments.verify_sensor_perception
```

The native command runs a bounded scripted test, not reinforcement learning:
15 seconds, target 2 m/s, seed 17, physics 120 Hz, rendering/LiDAR 20 Hz,
camera/control 10 Hz and noisy ego estimation 60 Hz. A blank camera is injected
at 3–3.5 seconds. It must command full braking; nominal perception must recover.
Acceptance requires movement, at least 80% valid nominal control frames,
full-body lane containment, no rigid contacts, and a sustained valid obstacle
stop with positive measured physical clearance. A stop caused by perception loss
does not satisfy the obstacle-stop gate. The supervisor rejects concurrent Kit
processes and stops only its own child on timeout.

Local source preparation evidence is retained under
`outputs/sensor-perception/v01/source-preparation.json` and
`source-reopen-validation.json`; v07's byte-identical backup is in
`source-backup-v07/`. Native runs use unique timestamped subdirectories and logs
under `logs/sensor-perception/`. Required native evidence is:

1. Composed hierarchy and layers showing the payload source camera/mounts.
2. Actual front RGB, with separately labeled depth/evaluation views if saved.
3. Delivered LiDAR data and estimator/control traces with timestamp provenance.
4. A scripted low-speed response to measured obstacle clearance and fail-safe
   response to missing/stale input, with physical progress and stopping results.
5. Reset isolation and acquisition freshness checks; failure packages remain
   preserved if a native attempt fails.

Until those results are recorded, native camera capture, sensor-controlled
physical driving and milestone acceptance remain **pending**.

## Scope and remaining boundaries

The old27/22/42 actor contracts are retained for historical reproducibility.
This prototype does not convert existing checkpoints into sensor-trained
policies. It does not add radar, surrounding cameras, GNSS fusion, a semantic
vehicle detector, a complete lane-steering planner, Cosmos generation or new
reinforcement learning. No source camera addition alone supplies these abilities.

Review reset/episode identity at the complete runtime boundary, not merely
freshness within one episode. Validate malformed native packet rejection and
report any remaining simulator-assisted steering, speed targets, emergency
intervention or frame conversion. Actual mounted sensor dimensions, optical
occlusion, native API compatibility and delivered rates require runtime evidence.
A later training proposal must explicitly review observations, actions, rewards,
algorithm, budget and evaluation before any learning run starts.
