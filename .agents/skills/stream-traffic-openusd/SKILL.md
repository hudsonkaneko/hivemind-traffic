---
name: stream-traffic-openusd
description: Stream live or recorded SUMO/TraCI traffic state into a layered OpenUSD scene with stable prims, reusable vehicle assets, and deterministic replay. Use for coordinate conversion, USD scene composition, vehicle transform updates, or traffic replay export; do not use it to train policies or replace SUMO as the traffic solver.
---

# Stream Traffic to OpenUSD

## Highway Sim repository scope

Read the repository's `AGENTS.md` and `documentation/project-skills.md` before
applying this workflow. The active roadmap determines the selected backend:
SUMO authority statements below apply to SUMO-backed experiments, not to native
PhysX-controlled vehicles. Preserve one motion owner per vehicle. Repo-local
skills do not authorize training, dependency installation, or publication of
local-only vehicle assets; follow the current task and project authorization.

Build a bridge in which SUMO owns traffic dynamics and OpenUSD represents the composed visual scene. Preserve this authority boundary: USD edits must never feed back as a second, independent vehicle controller unless the user explicitly defines that interface.

## Establish the contract

Before implementation, record:

- The source tuple for every step: simulation time, vehicle ID/type, SUMO x/y position, heading, speed, and optional agent/debug state.
- Stage `metersPerUnit`, `upAxis`, time codes per second, world origin, and the vehicle asset's local forward axis.
- A stable, reversible mapping from SUMO IDs to legal USD prim names. Keep the original ID as metadata.
- Whether the result is a live view, a saved replay, or both.

## Map coordinates explicitly

Create one tested conversion function rather than spreading sign changes through the application.

1. Select at least two known road-network anchor points and one known heading.
2. Apply translation, scale, axis mapping, and rotation in a documented order.
3. For a common Z-up stage with +X east and +Y north, SUMO's clockwise heading from north maps to a USD yaw of `90 degrees - sumo_heading`, plus any vehicle-model forward-axis offset. Treat this as a convention to verify, not an assumption.
4. Test position and orientation visually and numerically before streaming a fleet.

## Compose the stage

Prefer this separation:

```text
world.usda
  static/environment.usd       # road, signals, buildings
  dynamic/live-or-replay.usda  # transforms and changing state
  assets/vehicles/*.usd        # reusable vehicle prototypes
```

- Give each vehicle asset a valid `defaultPrim` and reference it from a stable path such as `/World/Vehicles/Agents/vehicle_001`.
- Use instanceable references for repeated models when each vehicle still needs a prim path. Use `UsdGeomPointInstancer` only when indexed instances are acceptable and individual prim identity is unnecessary.
- Keep geometry and materials in referenced asset layers. Author transforms, visibility, traffic-light state, and debug attributes in the dynamic layer.
- Keep roads and other static data unchanged during simulation updates.

## Handle live state and replay differently

For live display, author changing opinions to an anonymous or session layer on an in-memory stage. Update transforms in batches at a chosen display rate; do not rewrite a `.usda` file every simulation step.

For replay, export a separate dynamic layer with time samples. Define the conversion between simulation seconds and USD time codes, set the stage time range, and preserve the seed, scenario/config identity, policy checkpoint, coordinate-map version, and source timestep as layer or root metadata. A viewer should be able to load the same static scene with either the live layer or a replay layer.

## Deliverables

- Coordinate-map implementation plus anchor-point tests.
- Composed root stage, static layer, dynamic layer, and referenced vehicle assets.
- Live stage updater and/or replay exporter, according to the request.
- A short manifest recording units, axes, origin, timing, scenario, and software versions.

## Acceptance tests

- The stage opens without unresolved references, invalid prim paths, or composition errors.
- Two anchor positions and one heading match the expected USD values within the declared tolerance.
- Every source vehicle ID maps to exactly one stable prim or one stable instancer ID.
- Updating a live frame changes only the intended dynamic opinions; static layers remain byte-for-byte unchanged.
- A saved replay reaches the expected first and last poses at the expected times and can be scrubbed without running SUMO.
- Repeated vehicle geometry is referenced or instanced rather than copied into every vehicle prim.

## Official references

- [OpenUSD introduction](https://openusd.org/release/intro.html)
- [USD references API](https://openusd.org/release/api/class_usd_references.html)
- [UsdGeomPointInstancer API](https://openusd.org/release/api/class_usd_geom_point_instancer.html)
- [UsdGeomXformCommonAPI](https://openusd.org/release/api/class_usd_geom_xform_common_a_p_i.html)
