# Live showcase controls — v08/v09

October 12, 2026. Canonical implementation: this repository. Purpose: explore
traffic density and driving targets without repeatedly starting Isaac Sim.
This is an opt-in presentation milestone, not training or a replacement roadmap.

## What changed

v08 adds a native `omni.ui` sidebar to the regular Isaac editor, with live camera
and overlay controls, validated next-run driving/rendering settings, and a
supervised safe-stop/restart. v09 preserves that version and makes density,
main-car target speed, background speed and following gap **live requests**.
Earlier version profiles, prepared assets, SUMO demos and evidence are preserved.

| Control | v09 behavior |
| --- | --- |
| Background cars | Target 4–80, excluding the one main car; gradual entry/removal |
| Main-car target speed | 5–35 mph; requested target ramps at 1 m/s² |
| Background speed | 5–25 mph at the inner lane; ramps at 0.5 m/s² |
| Following gap | Preferred 1.8–3 seconds; ramps at 0.25 s per second |
| Camera, path, statistics, wheel animation distance | Live display changes; not physics quality settings |
| Rendering rate/resolution | Explicit supervised restart; discrete supported choices |
| Future AV/hivemind/LiDAR/driver controls | Disabled, DISPLAY ONLY / Not implemented; no effect |

Background speed cannot exceed the main-car target: lower background speed
first when reducing both. Outer lanes scale speed for common angular spacing.
Targets are not guarantees of measured speed or minimum physical headway.
Requests wait through committed lane changes. New lane changes wait while
background speed or population is transitioning. A protected population can
remain above the requested count until safe removal is possible; the controller
continues lane following during that wait. Paused changes apply on resume.
The sidebar distinguishes requested density from actual on-road population.

## Why an object pool

PhysX tensor views bind a fixed body set. Rebuilding that set while a controller
is running risks stale handles, frame stalls and changed scene ownership.
v09 preallocates **80 pool slots** with the existing referenced/instanced visual
assets. Entry/exit changes membership, not USD hierarchy or tensor bindings.
At most four cars transition per simulated second, outside a 160 m circular
arc-distance exclusion around the main car. Changes wait when no safe remote
slot is eligible; low targets may take longer. Existing traffic is not shuffled.

Dormant bodies remain loaded and park at X≥10,000 m, Y=10,000 m, Z=20 m with
their render proxies hidden. They are not invisible colliders left on the road.
Explicit remote entry/exit uses native pose teleportation; ordinary road motion
continues through kinematic targets. Direct velocity writes are invalid for
these kinematic bodies and are not used. See NVIDIA's
[kinematic actor documentation](https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/docs/RigidBodyDynamics.html#kinematic-actors).

**Identity generations** distinguish a re-entered vehicle from an earlier car
using the same USD slot. The controller accepts only explicit lifecycle events;
unannounced membership changes still fail. Physics/control/planning remain
120/60/10 Hz. Source assets and published static layers remain immutable;
live visibility/transform opinions stay in the runtime layer.

## Performance and evidence boundaries

Callbacks only update a bounded latest-value mailbox. Physics consumes requests
at simulation boundaries; it does not execute native work on every slider pixel.
Statistics remain rate-limited. Pooling avoids load/rebind churn but does **not**
remove all 80-body memory/physics costs at low density. Event logs are bounded;
exhausting them ends the run with an explicit error, not silent evidence loss.

Main car: native PhysX steering/wheel-torque controller. Background: scripted,
nonreactive kinematics with infinite effective mass, not human behavior. Object
tracks are privileged simulator state, not LiDAR detections. No new LiDAR, V2V,
PPO or Isaac Lab training is enabled. Future controls cannot enter configuration.
Live tuning is exploratory: initial settings, applied requests, actual counts,
speed ramps and lifecycle events are recorded, but a changed-demand run is not a
matched fixed-configuration experiment or display-performance qualification.

## Demonstrate it

From the repository root with the configured traffic Python:

```text
python scripts/demo_loop_showcase.py --version v09
```

On this workstation, from `C:\Users\hudso\Documents\highwaysim`:

```text
.venv-traffic\Scripts\python.exe scripts\demo_loop_showcase.py --version v09
```

Select Light / Medium / Dense, or drag Background cars. The population changes
gradually in the same run; nearby cars are protected. Change speed/gap targets
and watch requested versus actual values. The normal editor remains available.
The default demo ends after four minutes of driving plus settle/brake time;
changing a slider does not reset that clock. Only rendering changes use Restart
rendering. Prepared Americano v07 and the configured Isaac runtime remain local
prerequisites; a GitHub clone alone does not include that vehicle asset.

Automated native callback/lifecycle test, hidden to isolate desktop input
(not mouse-latency, visible docking or FPS qualification):

```text
python scripts/demo_loop_showcase.py --version v09 --drive-seconds 90 --control-smoke --headless --panel-hidden --unpaced
```

## Validation record

General regression: **1,593 passed, 180 optional skips, 18 subtests passed**
(`.venv-traffic`, full pytest suite). Separate OpenUSD runtime: **122 passed**
(fleet, pool, loop-view and highway-view unittest suites).

### Native live-change acceptance

Runs `20261013T002704Z-70c5c8fc` and `20261013T002856Z-6b410449` passed all
physical, scene, process and live-smoke gates. Their complete 12,240-row
trajectories are identical (maximum position and speed differences both zero),
as are lifecycle events, control events, resolved configs and source hashes.
Seed 101, Isaac
`6.0.1-rc.7+release.42383.32955d8d.gl`, Windows, RTX 5070 12 GiB; PhysX CPU,
120 Hz physics, 60 Hz control, 10 Hz planning, 30 Hz rendered headless capture
loop (no image files requested). Local runtime packages were not changed.
Base Git `9aba0513`, dirty source snapshots/hashes and diff retained per run.

| Gate / measurement | Observed |
| --- | --- |
| Actual counts at 34 / 64 / 89 simulated seconds | 12 / 40 / 12; one process, no restart |
| First time each requested population was reached | 21.7583 / 41.7583 / 71.7583 seconds (post-step observations) |
| Lifecycle | 96 exits and 28 entries; generation IDs preserved |
| Minimum planar clearance | 1.607123 m; required ≥0.6 m |
| Contacts / overlaps / off-road | 0 / 0 / 0 |
| Braking distance / allowed distance | 1.524326 / 5.863391 m |
| Stationary hold / drift | 9.333333 s / 0.006402 m; required ≥5 s / ≤0.05 m |
| Completed passes / lane changes | 0 / 0; following safely during this tuning sequence, not an overtaking benchmark |
| Native/visible poses, static sources, runtime errors, cleanup | Passed |

Physics was never reset; each changing-population trace contains 12,240 physics
samples. The first hidden run took 88.0833 active wall seconds for 102 simulated
seconds. This is **not a GUI performance result**. Pool occupancy, ramp and
identity checks do not qualify every setting combination or a long-duration soak.
The repeat overlapped the final CPU-only regression suite and is not a matched
performance sample; its 96.5333 active wall seconds must not be used to estimate
panel overhead. Final regression rerun: 1,593 passed / 180 skips / 18 subtests.
Low/high speed endpoints, a matched visible/hidden overhead pair, mouse latency
and dedicated post-transition overtaking coverage remain future checks.

### Retained development failures

Failed evidence is retained, including:

- `20261013T001152Z-c550e310`: physical gates passed but native
PhysX rejected direct kinematic velocity writes, so the supervisor correctly
rejected the overall run. That API misuse was removed; the fake native body
test now rejects any attempt to call it. No error filters or safety gates were
weakened to obtain a pass.
- `20261013T001545Z-b048018e`: stopped at 85 s on camera ownership mismatch;
  recorded camera shifted 7.478 m during rendering. No claim that this proves a
  cause. The guard was retained.
- `20261013T001743Z-4e4df4c5`: all physical/scene/process checks passed, including
  10 passes, 5 lane changes and 1.572843 m minimum clearance, but the scripted
  smoke gate failed. Additional density requests at ticks 3232–3252 changed the
  requested population beyond the automated script. Actual count at 34 s was
  23 rather than 12. Kept as interactive evidence, not a repeatability pass;
  extra inputs do not by themselves establish the cause of the missed checkpoint.
  The sidebar was docked and all 26 future controls stayed disabled. Native
  repeats now run hidden from desktop input, with the same widget-model callbacks.
- `20261013T002148Z-dcfc2277`: the first hidden test exposed that the runner
  skipped panel construction when headless. Its physical run passed but its
  live-smoke gate correctly failed (no changes applied). The smoke path now
  constructs the actual hidden native models; ordinary headless runs still
  avoid the panel. The acceptance gate was not relaxed.
- `20261013T002420Z-ed23561f`: isolated callbacks applied all three requests,
  but density was still 23 at 34 s because repeated lane changes delayed remote
  removal. The controller now holds NEW lane changes while population is
  transitioning, preventing repeated maneuvers from starving density requests.
  Existing committed maneuvers finish normally. The 12/40/12 checkpoints remain.

v08 retained predecessor evidence: `20261012T235404Z-230b01dc` rejected a
deprecated docking API error; `20261012T235652Z-e92126d7` exposed an incorrect
supervisor exit-code expectation. `20261012T235846Z-19cfad34` →
`20261012T235928Z-1ffe3c5a` restarted successfully but did not prove actual docking.
`20261013T000115Z-f62a2c3e` → `20261013T000156Z-ebbf51e5` verified native docking,
disabled future controls, the selected settings, safe braking and clean shutdown.
The latter completed 2 passes/1 lane change with 1.656097 m minimum clearance.

The visible interactive run above averaged 0.999999x, with 35.992 ms p95 and
224.908 ms maximum display-loop intervals. Accumulated lag and hitch thresholds
failed; **strict smoothness/60 FPS are not qualified**. This is a single
interactive run, not a matched panel-overhead comparison.

## Implementation map

- `experiments/showcase_controls.py`: supported settings, strict ranges,
  latest-value mailbox, event bounds, safe restart snapshot.
- `visualization/showcase_sidebar.py`: native docked widgets, live/next-run
  distinction, disabled future controls and actual-model smoke actions.
- `traffic/live_showcase.py`: deterministic membership, generation IDs, ramps,
  explicit controller admission and smoke-result gate.
- `usd/live_showcase_fleet.py`: fixed native pool, parking, visibility and
  indexed remote pose changes; shared base fleet preserves earlier profiles.
- `scripts/physics_loop_showcase.py`: command-boundary application, physical
  checks, native lifecycle, measurement and shutdown.
- `scripts/demo_loop_showcase.py`: portable runtime discovery, source manifest,
  runtime-error rejection and optional safe-restart supervisor.

Google guide: tab **37 Live showcase controls** in the existing engineering
guide, synchronized and native-readback verified. All 37 older tab bodies are
unchanged; new text, heading roles, highlighted definitions and shaded Consolas
command paragraphs were verified. No exported-PDF page-fit claim is made.
The viewer skill informed the bounded mailbox/runtime-layer separation; the
experiment skill informed immutable failed-run records and same-source repeats.
Source review remains
[draft PR 10](https://github.com/hudsonkaneko/hivemind-traffic/pull/10); not merged.
Raw packages are local ignored `outputs/loop_showcase/<run-id>/` evidence, not
assets uploaded with the GitHub source checkpoint.
