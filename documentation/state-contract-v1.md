# State/time/identity contract v1

Implementation: `traffic/state_contract.py`. First producer and replay fixture:
`experiments/benchmark_traffic.py`. Existing replay and cooperative runner formats
are retained; migrating all adapters is a later step, not claimed complete here.

- A vehicle ID is a nonempty Unicode string, stable inside one episode. IDs may
  not be reused after removal in the same episode. Reset allows a new episode.
- Prim paths encode UTF-8 IDs as hexadecimal with legal identifier prefixes.
  Both episode and vehicle identity are reversible; punctuation cannot collide.
- `time_s` is episode-relative, finite, and equals `step * dt_s`. Steps strictly
  increase. Gaps are allowed for viewers that drop frames; duplicate/out-of-order
  frames are rejected. Changing episode or dt requires an explicit reset.
- `frame`: meters, +X east, +Y north, +Z up, origin is the SUMO network origin.
- `heading_deg`: SUMO clockwise from north, [0,360). Asset forward is +X; yaw is
  `90 - heading`. Source XY is front-bumper center, NOT vehicle center.
- Center pose subtracts half vehicle length along the rotated forward direction.
  Vehicle dimensions must be positive. Asset pivot and sensor mounting transforms
  still need their own explicit adapters; this function does not guess them.
- SUMO remains sole motion authority. A state stream is a view/record boundary,
  not a second motion controller or a sensor observation.
- `StateStream.accept` produces sorted spawned/updated/removed ID sets from
  snapshots. A snapshot retains currently active vehicles only. Removal reason
  (arrival versus other removal) is not inferred from absence.
- Schema/version/units/reference mismatches, non-finite values, duplicate IDs,
  invalid speeds, and unsupported clocks are rejected.

`snapshot-fixture.json` in each traffic probe can be serialized/deserialized with
`from_dict` and consumed by the same StateStream as live snapshots. This does not
turn SUMO ground-truth state into LiDAR-derived observations. Sensor acquisition
clocks and provenance remain separate in the cooperative timing contract.
