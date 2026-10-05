# Adaptive obstacle path: one physical car

This separate low-speed fixture adds an actual detour to the existing physical
car. RTX LiDAR detects a static obstacle in the nominal lane; a scripted planner
selects a left bypass and return, and the cyan overlay shows that same route.
PhysX moves the car from wheel steering, drive torque and brakes. No SUMO, pose
teleporting, dependency installation or training is used by this demo.

## Show it

Run from the clone root with the configured traffic Python active. The launcher
discovers the external Isaac runtime through the existing local configuration.
Commands are single-line and do not contain a username or installation path.

```text
python scripts/demo_obstacle_bypass.py
python scripts/demo_obstacle_bypass.py --camera overview --points
python scripts/demo_obstacle_bypass.py --headless --capture --unpaced
python scripts/demo_obstacle_bypass.py --mode blocked
python scripts/demo_obstacle_bypass.py --mode dropout
```

The bounded run lasts 50 **simulated** seconds. It is not guaranteed to finish
in 50 wall-clock seconds. Use the demo's Overview/Follow car, Pause/resume,
planned-path and LiDAR-point controls. Restart an old window to load new code.
The original stop-before-obstacle demo remains `scripts/demo_physics_lidar.py`.

## What is adaptive, and what is assumed?

- **Sensing:** fresh, valid XYZ returns are transformed from WORLD to the current
  chassis frame using explicitly privileged simulator odometry. Two distinct
  scans must confirm the obstacle. The planner accumulates a bounded front-face
  cluster, not a hidden object ID, semantic label or barrier pose.
- **Planning:** the known straight two-lane road supplies the allowed corridor.
  An explicit maximum obstacle extent prior (4 m long, 3 m wide) bounds the
  unseen portion of this fixture. A route shifts left to Y=3.8 m, passes, then
  returns to Y=0 and stops near X=90 m. Its placement depends on sensed bounds;
  after confirmation it stays fixed for this static-obstacle episode.
- **Control:** pure pursuit follows that route with steering; a speed controller
  supplies drive/brake commands capped at 3 m/s. The driver gate enforces command
  age, identity, limits and steering slew. Only PhysX updates vehicle motion.
- **Display:** cyan is the next 35 m of the selected driver path, pink is the
  pursuit target, and optional green points are actual RTX returns. The static
  dashed white line marks the lane divider, not the controller's future path.
- **Fallback:** invalid/stale scans brake. Fresh points in the immediate corridor
  or sampled future body envelope latch a stop. An infeasible/late detour also
  stops. `blocked` covers both lanes; `dropout` freezes the controller's packet
  at 12 s without pretending the acquisition time is new.

The 7.2-m-wide `blocked` barrier deliberately violates the planner's 3-m size
prior. The planner may initially select a detour from partial face returns;
route-envelope braking must then stop it. Passing this adversarial fixture does
not prove that the planner recognizes every full-road blockage before moving.

This is **not** general free-space perception, a moving-obstacle planner, a
traffic-rule negotiation system or learned autonomy. Empty adjacent space and
the declared static-object size prior are scenario assumptions. Sparse LiDAR
returns cannot prove unseen space clear. Known-map/privileged-state observations
must remain explicit in any future learned comparison.

## Implementation boundaries

| Component | Files and role |
| --- | --- |
| Fixture and portable launcher | `experiments/physics-obstacle-bypass.json`, `experiments/obstacle_bypass_config.py`, `scripts/demo_obstacle_bypass.py`: frozen settings, runtime discovery, fresh output directory, time/GPU guards and source hashes |
| Physical episode | `scripts/physics_obstacle_bypass.py`: 120-Hz physics, 60-Hz control, 10-Hz planner, 30-Hz render, existing 20-Hz RTX profile, telemetry and bounded cleanup |
| Scripted planner | `traffic/obstacle_bypass.py`: valid scan/identity contract, distinct-scan confirmation, extent prior, analytic detour and certified reference-footprint bounds |
| Control and safety | `traffic/path_following.py`, `traffic/bypass_validation.py`: explicit sensed-path source, privileged odometry, geometric predicted-body braking, full-footprint evaluation and fail-safe gates |
| Display | `visualization/physics_road_view.py`: optional lane divider and dynamic route ribbon in its own layer; old defaults preserved |

The footprint certificate samples the analytic reference with a conservative
bound between samples. It certifies the ideal planar rectangle, not tracking
errors or physical reachable sets. Every physical tick separately checks actual
road footprint, barrier clearance, contact, support, uprightness and path error.
The barrier's ground-truth box is used for scene authoring, evaluation and
aborting failed runs; it is not passed to the planner or wheel controller.

## Predeclared acceptance

All cases require 6,000 physics steps, zero contacts/road departures, at least
0.35 m measured full-body clearance, speed no greater than 3 m/s, upright Z
above 0.99, four supported wheels, path error at most 0.50 m, and a final
five-second hold below 0.05 m/s. Render-only calls must not advance physics.
At least 100 post-settle sensor frames must match historical chassis pose within
0.05 m; wheel orientation stays within the existing 0.1-degree gate.

The passing case additionally requires early sensed adoption, lateral travel
over 3 m, final X at least 85 m, final lateral offset below 0.25 m and no
post-settle safety override. Fault cases must brake continuously after detection
and stay before the barrier. Dropout must occur while moving and command full
braking by the next 60-Hz control opportunity after the 0.20-s oldest-return age
limit. Run failures and incomplete runs remain evidence, not hidden retries.

## Debugging lessons and evidence

The first exploratory run stopped safely: a 3.6-m passing offset and conservative
3-m hidden-width prior required more lateral coverage than the sparse returns
provided. The revised path uses 3.8 m within the second lane and checks the full
reference footprint; the extent prior and clearance thresholds were not reduced.

A second run physically passed and returned but failed at 32.77 s when the
shrinking cyan mesh had four faces. A Python four-tuple was interpreted as a
vector instead of an integer array. Explicit `Vt.IntArray`/`Vt.Vec3fArray` writes
and short-preview regression tests fix that type ambiguity. The
[OpenUSD mesh schema](https://openusd.org/release/api/class_usd_geom_mesh.html)
specifies integer arrays for face counts and indices.

Two complete exploratory runs then failed only the inherited wheel-axis gate.
That gate compared the tilted 3D axle with a ground-plane tire-force direction.
At the worst sample the axle tilt was about 0.131 degrees, but the rendered and
native 3D axle directions agreed. The corrected gate compares native
body-times-wheel lateral axis against the rendered cylinder axle and keeps the
ground-plane difference as a separate diagnostic. The 0.1-degree tolerance is
unchanged, old results retain their failed status, and the reference type is now
explicit in new evidence. This distinction matches NVIDIA's
[PhysX tire-direction definition](https://nvidia-omniverse.github.io/PhysX/physx/5.3.1/_api_build/group__vehicle2.html).

Every run retains a resolved configuration, source snapshots and hashes, raw
candidate/adoption scans, sensor acquisition and delivery times, trajectory,
planner states, contacts, wheel geometry, screenshots when requested, runtime
log and resource samples. Bulk evidence stays in ignored
`outputs/vehicle_obstacle_bypass/`; a clone includes source and compact reports,
not those raw local artifacts.

The accepted headless run `20261005T042001Z-d6aac490` completed all 6,000 steps
with minimum body clearance **2.14471 m**, maximum path error **0.15458 m**,
final position X=89.87754 m/Y=0.00212 m and zero contacts/road departures. All
959 post-settle sensor frames matched within 0.000848 m; native axle and full
pose errors stayed below 0.000036 degrees. Loop time was 99.77 s for 50 simulated
seconds (RTF 0.501); startup was 11.53 s. Whole-GPU peak, including other desktop
applications, was 6,719 MiB. This is not a real-time performance pass.

One subsequent GUI process crashed during Isaac startup before creating the
scene or a probe result. Its original log and failed summary remain local;
startup reliability cannot be inferred from the accepted headless run.

The unchanged-code GUI retry `20261005T042723Z-89b6cbf4` passed every gate and its
fresh approach/pass/return screenshots were inspected. It reproduced the same
physical outcome and clearance. Its loop took 139.85 wall seconds (RTF 0.358),
with 6,548 MiB whole-GPU peak; a working visual window is not a real-time claim.
No external runtime setting or dependency was changed to obtain that retry.
All 6,000 recorded positions, velocities, speeds, headings and clearances were
bit-identical between the accepted GUI and headless runs; their captured source
hashes were identical. Delivered sensor-frame counts still differed (956/959),
so this is not a claim that all RTX delivery timing is deterministic.

The blocked-road run `20261005T042235Z-f99b290a` passed all checks, stopping at
X=37.03959 m with 4.46184 m minimum clearance. The frozen-scan run
`20261005T042427Z-0e7e7d64` also passed, including the oldest-return braking
deadline, stopping at X=28.02843 m with 13.43378 m clearance. Neither fault case
had contact or left the road, and each completed the full 50-s trace and hold.

An independent raw-scan replay reproduced the accepted planner's two decisions
and adopted route exactly without barrier ground truth. Source/artifact hashes
verified. Its 6,000 positions, velocities, speeds, headings and clearances also
exactly matched the previous complete run before the wheel-measurement repair;
only the wheel evidence module changed. This is a measured repeat for these
runs, not a universal guarantee of bitwise RTX determinism.

The preserved old curved-road stop demo also passed its full 40-s regression
(`vehicle_visual_lidar/20261005T042604Z-5458527f`): no contact, final barrier gap
4.19742 m, lane RMS 0.02120 m and all 760 sensor poses matched. Its timing-related
stop distance need not be bit-identical to an older RTX run to meet the existing
acceptance criteria. No prior evidence was rewritten.

[Ten hash-verified attempts](obstacle-bypass-results.json) include all nine new
bypass attempts (four accepted, five failed) and the accepted old-demo regression.
The five failures are one conservative detection stop, one short-ribbon error,
two incorrect axle-comparison failures, and one native GUI startup crash.
Final code checks: **823 CPU tests passed, 10 skipped; all 37 view/wheel tests
passed under Isaac's Python.** Native Google guide tab 31 mirrors this milestone;
its prior 31 tabs are preserved.

## Next boundary

Add moving traffic and occupancy checks before claiming a safe occupied-lane
change; then validate two physical cars and V2V fallback. Preserve matched demand,
driver assignments and observation sources in later comparisons. The user must
participate in observation/action/reward and training-budget choices before
Isaac Lab policy training begins.
