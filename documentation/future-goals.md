# Future implementation goals

This file is durable project memory. These are planned goals, not claims of completed implementation and not instructions to start work automatically.

## Shared state / time / identity contract

Recorded: 2026-09-28. Requested by the user based on the supplied photo titled “Research background and paper skeleton.”

Status: **Backlog — implement in a future integration milestone.** Some conventions may already exist in individual components; audit and consolidate them rather than assuming they are absent or consistently enforced.

### Purpose

Define a reusable, versioned agreement for how SUMO/TraCI, controller/environment code, Isaac Sim/OpenUSD, recorded datasets, and the future ovrtx viewer exchange information. Each component must interpret the same record the same way. A visually plausible scene is not sufficient evidence of correct spacing, timing, or identity.

### Requirements to implement

- **Vehicle identity:** stable IDs throughout each episode, with explicit episode identity and ID-reuse rules. Maintain a reversible mapping to USD prim paths, preserving source IDs as metadata.
- **Time:** simulated seconds from episode start, explicit step index and timestep. Distinguish traffic time, sensor acquisition time, and wall-clock time; define synchronization, freshness, ordering, and missing/stale-frame behavior. Do not imply that the current lockstep sensor and traffic clocks are identical.
- **Position:** meters, documented origin, axis directions, handedness, and conversions between coordinate frames; declare OpenUSD stage units and up axis.
- **Position reference:** explicitly identify whether a pose is measured at the front bumper, vehicle center, asset pivot, or sensor origin. Define and test conversions using vehicle dimensions and sensor mounting transforms.
- **Direction:** explicit heading units, zero direction, rotation convention, and vehicle asset forward axis. Centralize conversion logic rather than scattering offsets through consumers.
- **Lifecycle:** explicit vehicle spawn/update/removal events. Remove or hide the corresponding visual object when the source vehicle leaves; define reset, duplicate-event, and out-of-order-event handling.
- **Authority and versioning:** retain one motion authority per mode (SUMO for the current demo); define schema versions, required fields, validation, and compatibility behavior. A viewer is not a second traffic solver.

### Proposed implementation sequence

1. Audit the existing live bridge, replay/export paths, telemetry, and sensor records; list current conventions and discrepancies.
2. Write the contract and example records, including vehicle state and lifecycle events.
3. Implement shared schemas/validators and tested coordinate, reference-point, and clock adapters.
4. Migrate producers and consumers incrementally, preserving evidence and documenting format changes.
5. Require contract tests before expanding to multiple autonomous vehicles or integrating another viewer/runtime such as ovrtx.

### Completion evidence

- Two known road anchor positions and cardinal headings transform correctly within explicit tolerances.
- Front-bumper/center/asset-pivot and sensor offsets remain correct after rotation, not only on a straight road.
- The same source vehicle maps to the same visual object through an episode; reset and ID reuse cannot mix episodes.
- Spawn/removal and repeated or delayed events do not leave ghost vehicles or duplicate prims.
- Sensor observations can be traced to their pose and traffic step without confusing sensor time with traffic time.
- Live and replay consumers agree on the same recorded poses and lifecycle events.
- Invalid units, unsupported schema versions, missing required fields, and stale/out-of-order records are rejected or handled through an explicit documented policy.

### Why it matters

A front-bumper pose interpreted as a center pose can shift a vehicle without crashing either program. Similar silent errors can corrupt lidar alignment, vehicle gaps, labels, and training/evaluation data. This goal makes those assumptions visible and testable.

Planning guidance: the stream-traffic-openusd skill informed the identity mapping, single-authority boundary, coordinate checks, and live/replay acceptance criteria. No controller or runtime behavior changed when recording this goal.
