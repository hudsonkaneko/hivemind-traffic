# Going around a stationary obstacle

Verified results, repeatability and retained failures: [validation report](lidar-avoidance-results.md).

From the project root, run:

```bat
.venv\Scripts\python.exe scripts\demo_live_lidar.py --mode avoid --speed 6 --gap 40
```

The cyan vehicle measures the orange obstacle, moves into the other lane, passes
it, and returns to its original lane. The viewer closes after 25 simulated seconds.
This is live feedback, not a recorded replay. `python` can replace the local venv
executable; external runtime discovery is unchanged from the live-lidar launcher.

To demonstrate the conservative fallback with a second stationary obstacle in the
neighboring lane:

```bat
.venv\Scripts\python.exe scripts\demo_live_lidar.py --mode avoid --speed 6 --gap 40 --blocked-lane
```

## Why these changes were needed

The original controller only asked “how much room is ahead?” Avoidance also needs
lateral space and memory of an obstacle after it leaves the forward field of view.
The new `traffic/lidar_avoidance.py` filters ground and self hits, converts the
remaining sensor points to road coordinates using ego position/heading, and keeps
a small 25 cm grid of observed stationary obstacles. Persistent memory prevents a
previously seen obstacle from vanishing when the ego body temporarily occludes it.

The planner moves through `approach → outbound → passing → returning → complete`.
It checks observed space in the neighboring lane from 30 m behind to 45 m ahead.
It returns only when the original-lane obstacle map is at least 12 m behind the
front bumper, covering the 5 m ego body plus a margin. Forward braking remains
active, including during lateral movement; cruise is capped at 6 m/s.

**RTX lidar supplies obstacle observations. TraCI carries commands. SUMO owns
vehicle motion. OpenUSD represents the scene.** Lane centres are known from the
road map; this is not lidar-based lane-marking detection. Obstacle truth is used
only after decisions for evaluation. No semantic labels, PPO, or object IDs are
required by this controller.

`--lanechange.duration 5` enables continuous SUMO lane changes. TraCI's
`changeLane(..., duration)` argument controls how long the lane choice is requested;
it is not a substitute for configuring continuous motion. Existing SUMO speed and
lane safety settings remain enabled. The USD bridge now updates heading as well
as translation, so both the vehicle proxy and lidar turn with SUMO.

## Tests and evidence

```bat
.venv\Scripts\python.exe -m pytest -q
.venv\Scripts\python.exe experiments\run_lidar_avoidance_suite.py
.venv\Scripts\python.exe scripts\audit_lidar_avoidance.py outputs\live_lidar\YOUR_RUN_ID
```

The suite covers repeated clear-lane avoidance, another speed/gap/seed, and a
blocked-lane stop. Run configurations, source snapshots, raw scans, telemetry,
captures, versions and hashes live under unique `outputs/live_lidar/` directories.
Logs live under `logs/`. The auditor reconstructs the map and every decision from
raw scans, then independently recomputes conservative geometric clearances.

See [the predeclared plan](lidar-avoidance-plan.md) for acceptance gates.

## Issue found and fixed

The exploratory run `20260928T172851Z-avoid-7b1563` safely stopped in the adjacent
lane instead of continuing. The transition required the heading to return almost
exactly to 90 degrees. SUMO's heading offset settles gradually after lateral motion
ends and stopped settling when the car stopped. Arrival is now recognized by lane
position instead. Actual heading is still used in rendering, sensor transforms,
and geometric evaluation. A unit test preserves this regression case. The failed
run and its source snapshot are retained, not overwritten.

The first audit-dispatch test also exposed old test fixtures without a mode field;
dispatch now keeps the original audit behavior when that field is absent.

One blocked-lane attempt (`20260928T174028Z-avoid-11d1a6`) then suffered a native
access violation in `rtx.rtxsensor.plugin.dll` after 169 raw scans. The vehicle had
stopped, but the process died before a final report; this attempt is **not a pass**.
The crash log and incomplete evidence remain, with a recovery note. A native crash
bypasses Python cleanup, so new runs now flush each completed step to
`telemetry.jsonl` and save initialized sensor metadata early. The suite preserves
per-case reports and records missing-result/failed-audit cases rather than losing
the preceding successful results. A successful retry does not prove the runtime
crash fixed; do not treat this prototype as production-stable.

Research found [NVIDIA's multi-tick sensor known issues](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/sensors/isaacsim_sensors_multitick_rendering.html)
and an [Isaac 6.0.1 sensor-plugin crash report](https://forums.developer.nvidia.com/t/isaac-sim-6-0-1-unbounded-hang-from-null-data-pointer-dereference-in-librtx-rtxsensor-plugin-so-0x2c40d/380481).
Those reports have different conditions/platforms and do not establish this
crash's exact cause. No binary patch, driver change, or external runtime edit was made.

## Limits to explain accurately

This is a two-lane, straight-road **stationary-obstacle** demonstration. The map
deliberately retains hits indefinitely and therefore is unsuitable for moving
traffic. It cannot prove unknown occluded space clear, negotiate approaching cars,
or detect arbitrary road layouts. Unhealthy/stale scans brake without a new lane
request; an acquisition timeout freezes traffic and fails the run. A lane maneuver
already accepted by SUMO may continue laterally while braking.

Vehicles remain 5x2 m kinematic proxies, not wheel-driven PhysX cars. The safety
metric conservatively bounds their oriented footprints between consecutive steps;
this is not continuous collision detection or physical vehicle validation. Lidar
acquisition still holds each traffic pose and runs slower than real time.
The old `stop` and `follow` modes retain their original behavior.

References: [TraCI vehicle commands](https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html),
[continuous lane-change output](https://sumo.dlr.de/docs/Simulation/Output/Lanechange.html),
[SUMO heading-offset implementation](https://sumo.dlr.de/doxygen/d6/d19/_m_s_abstract_lane_change_model_8cpp_source.html).
