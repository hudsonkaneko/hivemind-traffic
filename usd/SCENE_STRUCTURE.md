# OpenUSD scene and asset conventions

Status: required for new and modified scene-authoring work. The one-car LiDAR
obstacle-bypass demo now opts into the composed scene described in
[migration evidence](../documentation/physical-scene-composition.md).
Legacy low-level, Lab and vehicle-specific fixtures have not been migrated.

The user's requirement is legible hierarchies, properly named components and
reusable OpenUSD/Omniverse composition, even with simple development visuals.
This project convention follows OpenUSD concepts; the exact folder names below
are ours, not a universal hierarchy mandated by OpenUSD.

## Scene hierarchy

```text
/World                                  Xform; assembly; stage defaultPrim
    Environment                         Xform; group
        Highway                         referenced environment assembly
        Props                           Xform; group
            Barrier_001                 referenced prop component
    Vehicles                            Xform; group
        vehicle_000001                  referenced vehicle component
        vehicle_000002                  referenced vehicle component
    Lighting                            Scope; lights and lighting rigs
    Cameras                             Scope; Overview and Follow rigs
    Physics                             Scope; scene settings and shared physics resources
    Debug                               Scope; paths, sensor displays and telemetry geometry
```

Only create branches that contain useful content. Avoid both root-level clutter
and redundant wrapper chains. Use Xform when a transform is meaningful and Scope
for non-transforming organization. Simulation-owned paths such as `/Render` or
`/Replicator` may be required by the runtime: register and document exceptions,
do not move them merely to satisfy this tree. Do not author project content into
those runtime namespaces.

If an organizational branch contains model assets, give it the appropriate group
kind so model ancestry stays contiguous; an untagged Scope must not interrupt
that ancestry. Non-model camera/light/debug containers do not need model kinds.

Use descriptive PascalCase component names such as `Chassis`, `FrontLeftWheel`,
`LidarFront`, `Overview`, and `RoadSurface`. Use legal, deterministic
`vehicle_<stable-key>` IDs for placements; preserve original external IDs and
asset/driver/backend identity in metadata or the scene manifest. Do not rename
or reparent a car when its controller mode, AV assignment or visibility changes.
Creation-order suffixes without a stable mapping are not an identity contract.

Model kinds describe asset boundaries, not every mesh: assemblies/groups must
form a contiguous model hierarchy; a component is a leaf model. Wheels or other
important internal parts may be subcomponents, not nested component models under
a component. Keep `Looks` and geometry containers below the asset boundary.

## Vehicle boundary

Publish reusable vehicle entry points under the existing
`vehicles/sim_ready/<vehicle-name>/vNN/` convention. Simple procedural cars must
also become reusable assets when promoted out of a narrowly scoped test fixture.
Do not move or overwrite source imports or previously validated versions.

Expose a documented vehicle root and paths for the actual chassis rigid body,
named wheel attachments, visual geometry, collision shapes, materials and sensor
mounts. For example, a `Chassis` may contain `Visuals`, `Colliders`, wheel
attachments and a `Sensors/LidarFront` mount. The exact internal structure must
match the selected PhysX model, not a cosmetic one-size-fits-all template.
Sensors must inherit the correct moving rigid body's frame. Do not parent a
sensor under a stationary wrapper just because its name looks organized.

Centralize path construction/lookup in an asset or scene interface rather than
duplicating string literals across controllers, tests and viewers. Record the
source ID to prim-path mapping, motion authority, position reference, forward
axis and schema version. A hierarchy edit must not change who controls motion.

## Composition, packaging and loading

| Mechanism | Project usage |
| --- | --- |
| References | Place reusable vehicle, road and prop assets through stable public entry layers. Keep source assets separate from placements. |
| Payloads | Defer substantial geometry or environment sections where selective loading is useful. Not mandatory for every cube or small prototype. |
| Sublayers | Separate authoring responsibilities and overrides sharing the same scene namespace: layout, physics, look development, motion and debug. |
| Session/dynamic layer | Live poses, per-instance state and temporary overlays. Do not repeatedly save or mutate static asset files during stepping. |
| Instancing | Share repeated immutable visual subtrees where compatible. Preserve editable individual vehicle roots and wheel/sensor/control access. |

A useful asset pattern is a lightweight `vehicle.usda` interface with a valid
defaultPrim, asset metadata and public configuration, composing physics/sensor
layers and, when worthwhile, a payload containing heavier geometry/materials.
Binary `.usdc` suits substantial geometry; readable `.usda` suits small assembly
and interface layers. Do not generate a separate file for every trivial prim.
Document sublayer strength order and which layer owns each runtime edit.

Geometry, materials, physics and sensor definitions need clear ownership; they
do not require arbitrarily many files. Keep only meaningful variants, such as
appearance or a validated sensor configuration, and record the selected variant
in experiment manifests. Do not quietly swap collision models between treatments.

Use portable relative asset paths anchored to their owning layer or a documented
resolver, never personal-machine paths in published assets. Package textures and
other dependencies too. Author valid defaultPrim, metersPerUnit, upAxis and
assetInfo on published entry points; verify transform conventions when composing
assets. Our physical-scene contract is metres, Z-up, with a documented +X-forward
vehicle convention. Merely setting stage metadata does not convert imported data.
Scene entry points also document timeCodesPerSecond and the conversion between
simulation seconds and replay time samples; do not confuse these with wall time.

Load all required collision geometry, road support, vehicle internals and sensor
targets before starting physics/sensing. Fail readiness checks if required
payloads or references are missing. No runtime unload of required content in a
validated experiment. Optional scenery may be unloadable only when that choice
does not silently change sensor observations or experimental treatment.

Do not mark an entire dynamic car instanceable merely because its mesh repeats.
Start with immutable visual subtrees; expand instancing only after proving wheel,
joint, sensor and physics behavior in the installed runtime. Instance proxies
cannot be individually authored like ordinary prims.

Keep authored entry stages composed; preserve references/payloads when saving.
Flattening is permitted for explicitly named diagnostic/delivery artifacts with
their composed source and dependency manifest retained. A flattened snapshot is
not the editable source of truth. References and layers do not automatically
make names/hierarchy clean, and payloads do not guarantee a real-time speedup.

## Current implementation and migration order

- `traffic/physx_vehicle.py` keeps its original low-level default and adds an
  explicit composed-scene option. `usd/physical_scene.py` owns versioned paths,
  asset extraction, scene bindings, export and structure checks.
- `scripts/physics_obstacle_bypass.py` uses that option: referenced vehicle,
  road, ground and barrier, named cameras/lights, separate physics and debug.
  It saves `scene/world.usda` with composition intact, not a flattened stage.
- `visualization/physics_road_view.py` preserves its legacy default while the
  migrated demo uses `/World/Environment/Highway` and `/World/Debug/Route`.
- `scripts/replay_export.py` already references `vehicle.usda` and instances the
  Model subtree, with separate road/motion layers. The whole project is not
  reference-free. Preserve its SUMO/front-bumper contract during any migration.

Next authoring milestones: interactive Stage/layer inspection, reusable-library
promotion, and scoped migration of additional consumers when needed.
Do not retrofit published evidence or globally rename live-stage prims. Imported
vendor content may remain internally intact behind a documented asset wrapper
until a dedicated asset-normalization migration is tested.

## Completion checks for scene work

1. Inspect both the composed Stage tree and layer/composition views in Kit/USD
   tooling. Names and groups convey purpose; exceptions have a stated owner.
2. Validate legal/stable paths, defaultPrim, model-kind ancestry, units/axes,
   relationship targets and exact expected vehicle/sensor identities.
3. Open the composed stage from a fresh process and a different working directory;
   references, payloads, materials and textures must resolve without warnings.
4. Confirm repeated assets use composition, not copied meshes. Test intended
   payload load/unload boundaries and required-content readiness failures.
5. Verify live updates only touch intended dynamic opinions and leave published
   assets/static layers unchanged. Preserve references on save/reopen.
6. Re-run wheel orientation, steering/braking/contacts, controller binding,
   LiDAR identity/frame/timestamp, reset and repeatability checks after migration.
   Capture an expanded hierarchy screenshot and retain before/after manifests.

These are required acceptance checks. The linked migration report distinguishes
implemented automated validators and actual runtime evidence from remaining
interactive checks; it does not imply all historical scenes now comply.

## Primary guidance

- [NVIDIA asset structure principles](https://docs.omniverse.nvidia.com/usd/latest/learn-openusd/independent/asset-structure-principles.html): modular workstreams, public interfaces and appropriate composition granularity.
- [OpenUSD model hierarchy](https://openusd.org/release/glossary.html#model-hierarchy): kinds and contiguous model ancestry.
- [OpenUSD scenegraph instancing](https://openusd.org/release/api/_usd__page__scenegraph_instancing.html): shared prototypes and authoring restrictions.
- [NVIDIA payload creation](https://docs.omniverse.nvidia.com/dev-guide/latest/programmer_ref/usd/references-payloads/create-payload.html): explicit loadable asset composition.
