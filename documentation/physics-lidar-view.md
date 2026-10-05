# Visible physics car and LiDAR braking

This separate fixture makes the physical-car work visible without changing the
preserved SUMO demos. One simple full-size sample car follows a marked curved
road; a chase/overview camera, planned-path ribbon and pursuit target explain
its scripted control. An RTX point cloud supplies emergency-braking inputs for
a static orange barrier. PhysX alone moves the chassis.

## Try it

From the clone root, activate the configured traffic environment. Configure the
external Isaac installation with `isaac_root` in ignored `hivemind.local.json`
or `ISAAC_SIM_PATH`. The launcher discovers Isaac's separate Python; do not
install traffic dependencies into that runtime.

```text
python scripts/demo_physics_lidar.py --check
python scripts/demo_physics_lidar.py --points
```

`--check` resolves configuration without opening a simulator. The second command
opens the bounded 40-s episode, including a 2-s settling period, and closes it
after saving evidence. Startup and simulation can take longer in wall time.
Use the **Highway Sim | Physics + LiDAR** panel to select overview/follow mode,
pause/resume, show/hide the planned path and show/hide LiDAR points. Do not use
the Kit timeline to control this explicitly stepped fixture.

Cyan shows the upcoming map reference, pink marks the pure-pursuit steering
target, and green dots show ray returns. These are not learned trajectories or
semantic object classifications. The point display is optional to reduce visual
clutter. Detailed assets, realistic highway art and ovrtx remain separate work.

Reproduce bounded headless checks one at a time:

```text
python scripts/demo_physics_lidar.py --headless --mode smoke --capture --unpaced
python scripts/demo_physics_lidar.py --headless --mode obstacle --unpaced
python scripts/demo_physics_lidar.py --headless --mode dropout --unpaced
```

The dropout fixture freezes controller delivery at 12 simulated seconds. The
sensor itself continues recording, making the stale-data fault auditable.
Each attempt has a new immutable directory under `outputs/vehicle_visual_lidar/`.
Raw scenes, trajectories, captures and logs remain local, not in a GitHub clone.

## Responsibility and observation boundaries

| Component | What it does | What it does not do |
| --- | --- | --- |
| Scripted behavior and planner | Select the known route and 3 m/s target; issue expiring route references | Infer lane geometry from LiDAR or learn a policy |
| Pure-pursuit/speed controller | Read map and simulator odometry; command steering and wheel drive/brakes | Set or replay chassis poses |
| LiDAR emergency brake | Read time-checked XYZ returns in a forward corridor; override propulsion with braking | Read object labels, barrier pose or obstacle truth |
| PhysX | Integrate wheel forces, suspension, contacts and chassis movement | Decide a route or infer driver intent |
| View | Render the same map, reference and observed car state; move the camera | Move the vehicle or redefine the driving map |

The brake uses a short, straight corridor ahead of the 2.4 m front-bumper
offset, with lateral half-width 1.15 m and chassis-relative height from −0.6 to
+0.6 m. Stopping range includes speed-squared braking distance, scan age and a
margin. A detected obstacle latches braking until a new episode; invalid,
missing, future, wrong-identity, malformed or stale scans also request full
braking. This is a testable low-speed emergency stop, not general perception,
moving-object tracking, road reconstruction or obstacle-passing autonomy.

WORLD-frame returns are reprojected with current privileged chassis odometry.
Per-ray acquisition bounds, delivery tick, episode/vehicle ID and coordinate
reference tick stay explicit. This static-barrier assumption is not moving-object
motion compensation. Ground-level visual overlays may appear in raw returns;
they are below this brake's height filter, not claimed to be sensor-invisible.

## Clock and lifecycle

Physics runs at 120 Hz, control at 60 Hz, planning at 10 Hz, rendering at 30 Hz
and rotary scans at 20 Hz. Both the sensor tick rate and rotary scan rate are
authored and read back. The sensor profile uses 32 emitters and 7,200 firing
patterns/s: doubling scan rate shortens acquisition and halves angular samples
per revolution, rather than doubling the configured ray workload.

The new rendered session uses the installed SimulationManager clock and checks
every physics step. Rendering pumps the application with automatic physics
disabled and must leave the physics counter and time unchanged. The manager's
two initialization steps are recorded as an origin, before episode tick zero.
UI pause withholds physics steps instead of resetting the timeline.

An initial 10 Hz attempt revealed periodic stale-data braking: acquisition plus
delivery and hold time exceeded the unchanged 0.20-s oldest-sample limit. Its
result writer also exposed a NumPy boolean serialization bug. Both failures are
retained; neither is hidden by overwriting a run or relaxing the age threshold.

The first point-cloud GUI attempt failed before movement because the debug
writer extension was loaded after constructing the sensor. The installed sensor
copies its writer aliases at construction, so enabling the extension afterward
is too late. The corrected order enables it before creating the sensor. This
failure is preserved separately from the passing no-point-cloud runs.

A subsequent GUI run passed every driving check but failed visual inspection:
the debug writer's default sensor-to-world transform was applied to XYZ that was
already in WORLD coordinates. The corrected display explicitly uses
`doTransform=False`, matching the installed NVIDIA ROS2 helper's WORLD-frame
branch. Braking parses raw GMO data separately and was not affected. This is why
a numerical pass is not, by itself, a visualization pass.

## Validation status

**Verified bounded subset, not the full roadmap gate.** The headless obstacle
and frozen-delivery tests each completed 4,800 physics ticks (40 s). The final
GUI/point-cloud run repeated the obstacle case and passed the same numerical
checks. The recorded trajectories for headless and GUI movement match exactly
on this workstation; this is not cross-machine determinism.

| Measurement | Recorded result |
| --- | --- |
| Curved-road lane error | RMS 0.02120 m; maximum 0.02717 m |
| Obstacle stop | Final progress 62.22481 m; final barrier clearance 4.24599 m; minimum clearance 4.23066 m |
| Braking trigger | Tick 2,826; LiDAR gap 4.86433 m against computed stopping range 4.88082 m |
| Holding | Final 5-s maximum speed 1.09e-7 m/s; no reported rigid contacts or lane departure |
| Sensor alignment | Maximum recorded mount-position error 0.001016 m; every recorded post-settle frame matched |
| Freshness | Obstacle-run maximum accepted oldest-sample age 0.16667 s, below 0.20 s |
| Dropped delivery | First stale-scan braking at tick 1,452 (12.10 s), exactly the expected control tick; stopped at 28.23182 m |
| Render isolation | 1,200 scheduled render checks per full case; native physics count advanced exactly 4,800 |
| Automated tests | 784 CPU tests passed; two USD tests skipped there and separately passed in Isaac's Python (23 view tests total) |

Passing runtime attempts had no `[Error]` log lines, no scene-reference warning,
no resource-guard stop and no mid-run source changes. Other startup warnings are
retained. These fresh rendered-session results do **not** retrospectively clear
the older stopped-timeline reset/scene-reference failures.

Performance is not yet real-time ready: the unpaced headless obstacle loop took
59.28 wall seconds for 40 simulated seconds (RTF 0.675), plus startup. The final
interactive GUI attempt took 91.95 s end to end, including 10.91 s startup and
extra pause renders. Sampled whole-GPU usage peaked at 5,328 MiB (desktop included);
process peak RAM was about 6.17 GiB. This short one-car run is not a fleet-capacity
or memory-endurance result. No sensor fidelity or freshness gate was lowered to
make timing appear better.

The [eight main attempts](physics-lidar-view-results.json) and
[final overview framing check](physics-lidar-overview-results.json) preserve all
nine attempts and their artifact/source hashes. Two attempts failed numerically;
one further numerical pass was rejected for its misaligned point display.
The final overview smoke additionally verified that the whole road and starting
car fit in frame. The GUI run also retains bounded raw
`moving-clear` and `obstacle-latch` NPZ/JSON snapshots for independent recomputation
of the sensor-based decision. Preview captures were visually inspected: the
road/path are visible and WORLD point-cloud display aligns with scene surfaces.
Historical passing numerical summaries do not certify their earlier misaligned
point-cloud visualization.

Accepted full cases: headless obstacle `20261004T202000Z-aca7bb26`, dropout
`20261004T202122Z-f9bf4799`, GUI with corrected WORLD points
`20261004T202619Z-1a2e35b2`. Final overview: `20261004T202959Z-37b5aef8`.
Independent review recomputed raw-scan corridor hits, braking distance, lane
geometry, timestamps and exactly matching physical traces. All retained artifact
and captured-source hashes verified. GUI latch-cloud nearest distance was
4.86418 m (slightly different from the headless scan); applied physical controls
were still identical. Acquisition/render delivery is not claimed bit-identical.

## Source map and next gates

Follow-up: [wheel orientation correction](wheel-orientation.md) fixes tumbling
tires and adds composed-geometry checks. The older runs above did not contain
that regression gate; their original physical/sensor measurements are preserved.

- `scripts/demo_physics_lidar.py`: portable launcher, deadline/GPU guards and evidence.
- `scripts/physics_lidar_drive.py`: visible fixture, multi-rate loop, UI and recording.
- `traffic/rendered_physics_session.py`: single physics clock and render-only checks.
- `traffic/physical_lidar.py`: native RTX capture, frame conversion and packet validation.
- `traffic/lidar_braking.py`: independent emergency-stop contract and safety rules.
- `traffic/visual_lidar_validation.py`: predeclared pass/fail checks.
- `visualization/physics_road_view.py`: separate static-road and changing-overlay USD layers.

This advances the physical-sensing and visualization subsets of roadmap steps
8 and 20. The full 60-s sensing/reset gate, 120-s integrated scheduler, hybrid
bridge, multi-car interaction, communications and endurance still require their
own evidence. No training is started. Before Isaac Lab policy training, involve
the user in observations, actions, rewards, algorithm, budget and evaluation.

Official references: [multi-tick rendering](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/sensors/isaacsim_sensors_multitick_rendering.html)
and [RTX LiDAR configuration](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/sensors/isaacsim_sensors_rtx_lidar.html).
