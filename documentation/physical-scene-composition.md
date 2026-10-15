# Referenced physical scene — migration v1

Status: one-car migration implemented; bounded runtime validation below.
October 8, 2026. Interactive Stage-panel inspection remains pending.

## Purpose and boundary

Migrate the current one-car scripted LiDAR obstacle-bypass demo to an organized,
composed USD scene without changing its dynamics, controller, sensor profile,
obstacle, or acceptance thresholds. SUMO demonstrations and legacy low-level
physical/Lab fixtures retain their existing paths. No training or dependency
installation is part of this milestone.

The scene interface is `usd/physical_scene.py`; the adapter opts in explicitly
with `scene_directory`. Factory authors a disposable, unattached stage first.
Normalization happens there, not in a running simulation. The public car root
is `/World/Vehicles/vehicle_ego`; the actual moving rigid body is its `Chassis`.
The sensor remains a descendant of that chassis with unchanged extrinsics.

```text
/World
    Environment
        Ground                      referenced support-plane asset
        Highway                     referenced decorative road asset
        Props/Barrier_001           referenced collision/visual prop
    Vehicles/vehicle_ego            referenced, non-instanceable component
        Chassis
            Visuals/Body
            Colliders/Chassis
            FrontLeftWheel/Collision
            FrontRightWheel/Collision
            RearLeftWheel/Collision
            RearRightWheel/Collision
            Sensors/LidarFront
        Physics                     wheel/tire/suspension parameters
    Physics                         scene, friction tables, collision groups
    Lighting                        original key light and dome
    Cameras                         Follow and Overview
    Debug/Route                     path, target, debug materials
```

## Composition and ownership

Each unique run contains `scene/world.usda`, composed from initial-state, debug,
physics, road-layout and layout layers, in strongest-to-weakest order. The car
entry layer composes separate vehicle-physics and vehicle-geometry layers.
Road, support plane and barrier are referenced assets. Paths are relative to
the authoring layer, not the user's working directory. Small required assets
use references; adding payloads indiscriminately would add no useful loading
boundary here. Required simulation/sensor targets are never unloaded.

Cross-asset bindings belong in the scene physics layer. Wheel suspension/tire
connections internal to a car remain in its asset. Collision-group and friction
connections outside that asset are rebound in the scene, including the inverse
collider collection links. Copying only the car geometry would lose these.

Before runtime setup, an exact comparator verifies the normalized Factory source
against the composed assets: 31 prims, 173 authored attributes, 28 relationships
and 36 targets in the installed fixture. It checks API schemas, property values,
binding metadata and ordered targets, not merely whether a target exists.
The live scene type is declared before sublayers are attached so the installed
SimulationManager receives a per-prim addition event; solver settings still come
from the physics layer. The exact one-scene registration guard remains enabled.

During simulation, PhysX/controller/sensor opinions go to the live root layer;
the path display writes only its debug layer. Source assets are not saved during
stepping. The saved scene is an **initial authored scene**, not an animation or a
standalone initialized controller. Run the Python launcher for motion and RTX
writer setup. Runtime-owned `/Render`, `/Replicator`, `/OmniverseKit_*`, and
writer/action graphs are not scene assets and are excluded from the export.

The saved package includes the already loaded, single-layer NVIDIA rotary LiDAR
profile as `assets/lidar-profile.usda`, referenced relatively in the saved copy.
The live sensor still uses its original profile and API setup. Export fails if a
future profile requires unhandled external dependencies. Reopening the saved USD
offline does not provide Isaac's executable sensor plugins or controller code.
These run-local generated assets are evidence fixtures, not a promoted vehicle
library release. Promotion to `vehicles/sim_ready/<name>/vNN/` is a later task.

## Validation protocol (declared before runtime tests)

1. CPU suite and USD-only structure tests: references, layer ownership, relative
   paths, binding closure, unchanged geometry/physics values, invalid-content
   rejection, fresh-process reopen from a different directory.
2. Full headless `pass` fixture, 6,000 physics ticks / 50 simulated seconds,
   120 Hz physics, 60 Hz control, 30 Hz rendering, 20 Hz LiDAR. Keep the existing
   wheel, collision, path, stopping, sensor-pose and timing checks unchanged.
3. Repeat the same pass and compare physical trajectories at aligned ticks;
   predeclare 0.001 m / 0.001 m/s tolerance. Report RTX-driven divergence rather
   than implying bitwise sensor repeatability.
4. Run blocked-corridor and frozen-scan variants with existing fallback gates.
5. Check published file hashes and in-memory static-layer contents before/after
   each run. A structure pass cannot replace physical or sensor evidence.

Keep all failed attempts and unique run IDs. GUI hierarchy inspection and
interactive camera controls require separate verification; headless screenshots
alone do not prove the Stage panel or controls were manually inspected.

## Reproduce

From the canonical repository root, with the documented environment configured:

```text
python scripts/demo_obstacle_bypass.py --check
python scripts/demo_obstacle_bypass.py --headless --capture --unpaced
python scripts/demo_obstacle_bypass.py --headless --mode blocked --unpaced
python scripts/demo_obstacle_bypass.py --headless --mode dropout --unpaced
python scripts/demo_obstacle_bypass.py
```

The normal preview command remains unchanged. This migration does not claim to
fix the separate real-time GUI lag or validate multi-car physics/Lab cloning.
Google learning-document synchronization is pending: the required trusted-read
helper rejects a Windows absolute workspace path before reading the document.
No Google Doc content was changed or claimed synced in this milestone.

## Observed failures and fixes

All attempts are preserved under `outputs/vehicle_obstacle_bypass/`.

| Run ID | Outcome / lesson |
| --- | --- |
| `20261008T213631Z-640f0f38` | Setup rejected a collision-group self-reference already remapped by CopySpec; made path mapping idempotent and added a regression. |
| `20261008T214021Z-58a00e96` | Composed physics scene existed but was not registered; no motion. |
| `20261008T214501Z-d253735e` | Public scene wrapping did not repair registration; removed that unsuccessful fallback. |
| `20261008T214548Z-8339fbb5` | Completed 6,000 ticks but native Fabric stage-close crashed; overall failed, trajectory/screenshots retained. |
| `20261008T214737Z-64ea0959` | Full physical/sensor/asset gates and clean shutdown passed after returning the viewport to Kit's camera before close. |
| `20261008T214933Z-ae9d3122` | Full pass with the offline sensor-profile package; fresh-process ordinary USD reopen from `usd/` had zero composition errors and retained 20-Hz sensor configuration. |
| `20261008T215042Z-aa8e1afe` | Blocked corridor, overview camera: all gates and clean shutdown passed; stopped at X=37.0396 m, minimum clearance 4.4618 m. |
| `20261008T215224Z-2617024a` | Frozen-scan fault: all gates and clean shutdown passed; stopped at X=28.0284 m, minimum clearance 13.4338 m. |
| `20261008T215353Z-f7c0c3ee` | Final-source pass with stricter model-kind/identity checks: all gates and clean shutdown passed. |

The Fabric crash stack identifies shutdown, not a proven unique root cause.
Camera cleanup is a defensive lifecycle change; subsequent successes do not
prove that all native shutdown failures are eliminated. `pre-close-result.json`
preserves provisional gates; only the final probe plus supervisor can pass a run.
The pre-runtime comparator does not substitute for runtime motion/sensor checks.

The two successful pass trajectories above match at all 6,000 ticks (maximum
XYZ-position and speed differences both zero). The second adds only saved-profile
packaging, not a dynamics change. They achieved 2.1447 m minimum obstacle clearance,
0.1546 m maximum path-tracking error and 0.000848 m maximum sensor-pose error.
This is a low-speed, one-fixture result, not general sensor determinism.

The earlier original-profile run `20261006T185049Z-21831b7d` also matches the
migrated `20261008T214933Z-ae9d3122` XYZ positions and speeds exactly at all 6,000
ticks. That historical run used wall pacing and the new run did not, so this is
a motion regression comparison, **not** a matched runtime-performance benchmark.
The saved package was also copied to
`outputs/usd_scene_portability/20261008T214933Z-ae9d3122/scene/` and validated in
a fresh ordinary USD process from a different working directory: no composition
errors, all required references/model kinds/identities present. No scene files
were removed or rewritten to make the relocation test pass.

The final-source repeat differs from `20261008T214933Z-ae9d3122` by at most
0.00000815 m in XYZ position and 0.00000168 m/s in speed across 6,000 aligned
ticks, passing the predeclared 0.001 tolerances. Additional stricter validation
does not establish identical raw RTX samples. Every one of the five successful
runs retained identical on-disk asset hashes after shutdown. Static in-memory
layer checks passed through the simulation; they do not inspect destroyed layers
after close. Successful runs used about 4.6–5.1 GiB sampled whole-GPU memory,
including desktop applications; this is not a fleet-capacity benchmark.

Final automated checks: **883 passed, 40 skipped** in the traffic environment;
**71 passed** in Isaac's USD-only test suite (physical scene, contract comparison,
clean and legacy road view, wheel geometry). The traffic environment intentionally
lacks Isaac/USD modules. `git diff --check` passed. Full failed-attempt evidence
and raw results remain local/ignored; this document and code are versioned.

Inspected headless Follow and Overview captures show the car, road, obstacle and
planned path. They are not screenshots of an interactively inspected Stage panel.
Remaining gates: interactive hierarchy/camera/point-toggle inspection, GUI real-time profiling,
physical multi-car validation and Isaac Lab training remain separate work.

## Sources

- [Project scene conventions](../usd/SCENE_STRUCTURE.md)
- [OpenUSD references](https://openusd.org/release/api/class_usd_references.html)
- [NVIDIA asset structure principles](https://docs.omniverse.nvidia.com/usd/latest/learn-openusd/independent/asset-structure-principles.html)
