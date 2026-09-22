# SUMO-to-OpenUSD traffic snapshot contract

This contract is the boundary between traffic simulation and Isaac Sim rendering.
SUMO remains authoritative in playback/sensor mode.

Each snapshot contains:

| Field | Meaning |
|---|---|
| `schema_version` | Version of this contract |
| `simulation_time_s` | SUMO simulation time in seconds |
| `frame_index` | Monotonic source frame number |
| `scenario_id` | Stable scenario/configuration identifier |
| `seed` | SUMO scenario seed |
| `vehicles` | Vehicle states keyed by original SUMO vehicle ID |

Each vehicle state contains:

- SUMO ID and vehicle type.
- World `x` and `y` in meters.
- SUMO heading in degrees.
- Speed and acceleration in SI units.
- Road ID, lane ID, and lane index.
- Lifecycle state: active, arrived, or removed.
- Optional policy action, communication, and safety-intervention telemetry.

The OpenUSD implementation must additionally define and test:

- Z-up stage and `metersPerUnit = 1`.
- World origin and at least two road-network anchor points.
- The exact SUMO-heading-to-USD-yaw conversion.
- Vehicle asset forward axis and sensor-frame transforms.
- A reversible SUMO-ID-to-legal-USD-prim-name mapping.
- Source timestep, display rate, lidar cadence, and interpolation behavior.

Lidar output must record its parent vehicle, sensor prim path, pose, calibration,
simulation timestamp, frame index, and source snapshot. Static USD layers must not
be modified by replay or live updates.

