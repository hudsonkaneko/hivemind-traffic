# One physical car on the V02 continuous highway

Status: implementation and qualification in progress. This note belongs to the
[mentor branch](mentor-loop-roadmap.md); it does not replace the main roadmap.

## Purpose and responsibility

The new fixture tests a deliberately simple prerequisite: a physical car can
stay on a continuous highway loop at a requested speed, cross the map seam,
and brake safely. It does **not** add learned driving or claim that traffic is
optimized. SUMO remains available in its existing demos but is not required here.

The data flow is:

`V02 known-map circle + PhysX chassis state → scripted driver → command gate → wheel steering/torques → PhysX motion`

The driver uses **pure pursuit**: it looks ahead along a periodic path and asks
for a road-wheel steering angle. A proportional speed controller asks for
normalized throttle or brake. **Ackermann steering** converts the bicycle-style
angle into separate left/right front-wheel angles. **PhysX**, not the controller
or USD animation, determines resulting movement, tire support and collisions.

**Unwrapped progress** accumulates signed movement around the circle. A seam
crossing merely passes map station zero. A completed MainLane1 lap requires
3,141.59265 m of net progress from the driving start, regardless of spawn angle.
**TTL** is command lifetime in simulated ticks: an expired command causes zero
propulsion and full brake. It is not a wall-clock network guarantee.

The car consumes privileged simulator odometry and known road geometry. The cyan
path is a map-based control/debug overlay, not LiDAR perception or learned intent.

## Immutable inputs and scene composition

Original V01/V02 files are not regenerated or edited. V02 is checked against its
manifest before per-run packaging. MainLane1 has radius 500 m and width 3.7 m;
its 65 mph design hint is not a validated operating-speed setting. This fixture
requests 3 m/s, 6 m/s or 15.6464 m/s (35 mph) explicitly.

Every run creates a portable `scene/world.usda` assembly referencing reusable
vehicle and highway interfaces. Highway collision is a required physics sublayer;
large visual/navigation geometry is a `.usdc` payload that must stay loaded while
driving. The original all-in-one V02 stage remains a preserved authoring source.

```text
/World
  Environment/Highway
    RoadNetwork, Navigation, SpawnPoints, DivergeZones, MergeZones
    Colliders/RoadSurface, Looks, Ground
  Vehicles/vehicle_ego/Chassis
    Visuals/Body
    FrontLeftWheel, FrontRightWheel, RearLeftWheel, RearRightWheel
  Physics/Scene
  Physics/Resources
  Lighting/{Sun,Sky}
  Cameras/{Follow,Overview}
  Debug/Route/ProjectedPath
```

Exact wheel paths follow `usd/physical_scene.py`. Factory's infinite support plane is disabled,
and the V02 landscape is visual-only. The actual finite road mesh supplies tire
support with the existing Factory tarmac material and collision/query filtering.
There is one physics scene and one physical movement authority. Spawn pose is
authored once before simulation; subsequent movement uses wheel commands only.

## Portable commands

Run from the repository root using the configured traffic Python environment.
The launcher resolves Isaac's runtime from existing local configuration; no
machine-specific runtime path is embedded in these commands.

```text
python scripts/demo_highway_loop.py --check
python scripts/demo_highway_loop.py --profile seam3
python scripts/demo_highway_loop.py --profile seam6
python scripts/demo_highway_loop.py --profile seam35
python scripts/demo_highway_loop.py --profile dropout35
python scripts/demo_highway_loop.py --profile lap35
python scripts/demo_highway_loop.py --profile three-laps35
```

GUI previews use light graphics and best-effort 1× pacing by default, with
follow/overview and pause controls. `--headless --unpaced` requests a bounded
qualification run without pacing. `--capture` saves preview PNGs. Close other
Isaac/GPU experiments before running: resource measurements are not isolated
if another job starts during the run. Startup is reported separately from the
physics loop. Saved USD is an initial-state scene, not a replay or controller.

## Predeclared qualification gates

These gates are defined in `experiments/highway_loop_config.py` before simulator
qualification, not fitted afterward to make a failing result pass.

- 120 Hz physics, 60 Hz control, 20 Hz rendering; no skipped physics steps.
- 2 s settling, 20 s short driving / 225 s one-lap / 640 s three-lap driving,
  followed by 8 s braking/hold. Start 20 m before the map seam.
- Lane-centre RMS ≤0.20 m and peak ≤0.50 m; all four 4.8 × 1.8 m chassis corners
  stay inside the 3.7 m lane. No unintended rigid contacts.
- All four raycast wheels supported after settling; upright-Z ≥0.98 and chassis
  height 0.5–1.5 m. The run aborts early for large lane/attitude/height failures.
- After the first 10 driving seconds, speed RMS error ≤0.25 m/s and peak error
  ≤0.50 m/s. No seam-triggered stop or controller fallback while driving.
- Stop at ≤0.05 m/s. Braking-distance envelope is `v²/(2×3) + 0.1v + 2` metres,
  measured using actual XY travel; the envelope is a conservative engineering
  gate, not a calibrated real-world vehicle claim. Hold still for at least 5 s
  within 0.05 m. Command dropout must brake at actual command expiry (12 ticks).
- Full-lap claims require the requested true distance; seam smoke alone cannot
  pass a full-lap claim. Compare fresh matched runs at aligned ticks, positions
  within 0.02 m and speed within 0.02 m/s; do not call that in-process reset testing.
- Render-only calls must not advance the physics clock. Package files and static
  layers remain unchanged; clean child exit, no runtime errors/reference warnings.
- After every render, composed camera position/direction must match the requested
  view within 1e-4 m / 1e-4 unit-vector difference. Inspect actual screenshots too:
  valid authored camera properties alone cannot prove that the renderer follows.
- Soft real-time is a separate measured result: RTF 0.99–1.01, p95 lag ≤0.10 s,
  maximum lag ≤0.50 s. Physical acceptance alone does not certify real-time playback.
  Timing ends at the final rendered frame; it excludes the final checkpoint,
  full trajectory export, final validation, cleanup and startup. Earlier periodic
  recording work is included. Total process time is reported separately.

## Qualification results and retained iterations

All run IDs below are UTC-named children of `outputs/highway_loop/`. Tests use the
installed Isaac Sim `6.0.1-rc.7+release.42383.32955d8d.gl` on the RTX 5070 12 GiB.
Project code, original V02 sources and generated asset files remained unchanged
during each completed run. There were no unintended rigid contacts, all four
wheels stayed supported after settling, and children exited cleanly without
runtime errors or the old stage-close reference-count warning. Nonfatal Kit/
Fabric startup warnings are retained in the logs; this is not a warning-free claim.

| Run ID | Check | Lane RMS / peak (m) | Brake distance (m) | Outcome |
| --- | --- | --- | --- | --- |
| `20261009T015731Z-ce18ddb6` | 3 m/s seam | 0.00264 / 0.00325 | 0.663 | Physical pass; first camera framing inadequate |
| `20261009T015808Z-27fb08a7` | 6 m/s seam | 0.00464 / 0.00559 | 2.663 | Physical pass; pre-camera-fix version |
| `20261009T015852Z-139681c4` | 35 mph seam | 0.03963 / 0.05803 | 18.044 | Physical pass; follow framing still inadequate |
| `20261009T020018Z-15bad5ee` | Paced GUI, 35 mph | 0.03963 / 0.05803 | 18.044 | Physical/timing pass, **visual follow failed**; retained, not demo acceptance |
| `20261009T020427Z-d6013f56` | Corrected paced GUI, 35 mph | 0.03963 / 0.05803 | 18.044 | Physical, camera, screenshot and soft-real-time pass |
| `20261009T020522Z-95214b37` | Lost driver commands, 35 mph; overview | 0.17101 / 0.34479 | 19.338 | Exact-expiry fallback, stopping/hold and footprint gates pass |
| `20261009T020618Z-93d951a0` | 225 s driving, full lap A | 0.05058 / 0.05791 | 18.044 | Pass; 3,452.318 m, one complete circuit |
| `20261009T020827Z-0310bc55` | Identical full lap B, fresh process | 0.05058 / 0.05791 | 18.044 | Pass; 3,452.318 m, one complete circuit |

The corrected GUI run completed 30 simulated seconds through its final rendered
frame in 30.000106 wall seconds: RTF 0.999996, p95 lag 0.000520 s and peak lag
0.006773 s. Startup was 11.79 s. Peak whole-GPU allocation was 4,213 MiB (includes
desktop use). This is one measured run, not an all-machines or multi-car promise.

The dropout controller deliberately recentres steering while braking because
fresh steering commands no longer arrive. Its greater lane deviation is measured
and still inside this gentle-radius fixture's gate; it does not establish a safe
fallback for tighter bends. Explicit-stop runs keep following the route while braking.

### Camera investigation and fix

The first GUI screenshots showed stationary road framing as the car receded.
Installed Kit transform commands can route individual transform properties to
the session layer, so repeated helper calls in a weaker layer were not a reliable
camera-ownership contract. That run did not log post-render property stacks, so
the exact competing writer was not proven.

The fix authors a double-precision look-at matrix and full transform order directly
in the strongest session layer, mirrors it into the portable initial-preview layer,
and checks the composed world pose after rendering. Six USD-only regression tests
cover competing opinions, follow poses, overview switching, saved preview poses
and chassis framing. Fresh screenshots verify that the fix reaches the renderer.
This follows NVIDIA's [direct viewport-camera matrix example](https://docs.omniverse.nvidia.com/dev-guide/latest/programmer_ref/viewport/camera.html)
and the documented [session-layer role](https://docs.nvidia.com/learn-openusd/latest/glossary.html).

### Fresh-process repeatability

After two identical full-lap runs, compare without modifying either input:

```text
python -m experiments.highway_loop_repeatability --run-a outputs/highway_loop/RUN_A --run-b outputs/highway_loop/RUN_B
```

Replace `RUN_A`/`RUN_B` with actual full-lap output folder names. The comparator
requires passing runs, independent IDs, identical settings/project source hashes,
matching Isaac/Factory/GPU-driver and recorded machine identity, valid evidence
hashes, contiguous 120 Hz trajectories and independently measured full circuits.
It records a new report and compares every aligned XYZ position/speed, not just
the final screenshot. Same-process reset reliability remains a later Lab gate.

Report `outputs/highway_loop_repeatability/20261009T021015Z-95370ec2` passed for
lap A/B above. All 28,200 aligned samples matched exactly: maximum XYZ position
difference **0 m**, maximum speed difference **0 m/s**, against 0.02 tolerances.
The two trajectory files have identical SHA-256
`b2bb8a2316f6cbf92c70c046939e1ddc46d447a903d594a7deb74f35dc6353d6`.
This verifies these two fresh runs on this recorded software/hardware, not
cross-machine determinism or randomized traffic robustness.

## Automated verification and files

- Complete traffic-environment suite: **1,165 passed, 58 skipped**. The skips
  include dependencies/runtimes deliberately absent from the traffic environment.
- Authoring-environment USD suite: **18 passed** (12 scene + 6 camera tests).
- Preserved V02 independently revalidated: **28 USD validators**, **1,280 seeded
  route plans**; source stage SHA-256 remains
  `8ab855aa553e9ab4576ee8266140553312a004b3616b3ea29ad6400eadbf95a8`.
- Three subagents divided periodic-control implementation, USD packaging/camera
  ownership, and acceptance/review work. Runtime tests were run serially by the
  main task. The review tightened braking-distance, stationary-hold, expiry,
  camera ownership and runtime-provenance checks before final qualification.

| Responsibility | File |
| --- | --- |
| Periodic map, progress and scripted driver | `traffic/loop_driving.py` |
| Fixed profiles and physical acceptance | `experiments/highway_loop_config.py` |
| Portable supervisor and provenance | `scripts/demo_highway_loop.py` |
| Single-owner physics lifecycle and recording | `scripts/physics_highway_loop.py` |
| Referenced environment, spawn and composition checks | `usd/highway_loop_scene.py` |
| Live camera/known-path overlay and GUI | `visualization/highway_loop_view.py` |
| Matched fresh-run comparison | `experiments/highway_loop_repeatability.py` |

The corresponding `tests/test_*loop*.py` files include negative controls: invalid
time/identity, seam versus true lap, wrong-way travel, stale commands, fabricated
braking-distance counters, insufficient stationary hold, modified source assets,
missing payloads, competing camera opinions and mismatched runtime provenance.

## Evidence and limitations

Each new immutable run under `outputs/highway_loop/` includes resolved settings,
source snapshots/hashes, Git state/diff, runtime version, initial composed scene,
per-tick trajectory, immutable per-second checkpoints, contacts, screenshots if
requested, GPU/process samples, clock/lifecycle logs and final acceptance results.
Failed attempts are retained. Full outputs are local and ignored by Git; the
evidence summary in this note is the Git-tracked record.

The installed Factory raycast-tire vehicle is not a calibrated highway car.
There is one car, no moving traffic, LiDAR, merging, V2V or trained policy.
Passing this fixture does not qualify arbitrary highway speeds or tighter radii.
No Isaac Lab training starts until observations, actions, rewards, budget and
evaluation are reviewed with the user.

Google guide sync: pending. The required file-backed Google Docs reader rejected
the canonical Windows workspace with `workspaceRoot must be an absolute path`.
No cloud document edits were made or claimed. Repository documentation is current;
the Google copy must be synchronized once the reader supports this workspace.
