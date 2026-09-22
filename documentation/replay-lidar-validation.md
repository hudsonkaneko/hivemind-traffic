# Curved traffic replay lidar validation

## Purpose and scope

Apply the controlled sensor test to the existing seeded PPO traffic recording,
including turns, lane changes, four other vehicles, and vehicle appearance/removal.
SUMO's recorded poses remain the sole motion authority. This does not retrain PPO,
add lidar observations, change the normal replay viewer, or modify Isaac's runtime.

The validation copies the recording and network to a unique output directory and
exports its own portable USD layers. It never overwrites the original replay.
The existing controlled two-box validation remains unchanged.

## Independent geometry reference

The pure NumPy checker reads the source recording, not the measured lidar ranges,
to interpolate front-bumper positions and `90 - heading` yaw. Visibility is held
between snapshots, matching the exporter. Scalar yaw interpolation intentionally
matches the current exporter; it is not a general shortest-arc heading solver.

Vehicle bodies match the current block asset: center `(-length/2, 0, 0.65)` in
vehicle coordinates, size `(length, width, 1.1)` meters. The sensor mount is
`(-2.5, 0, 1.8)` on agent_0. Each returned spherical ray is rotated by the ego
heading, transformed into each visible vehicle's coordinates, and intersected
with its oriented box. The nearest intersection wins, including the ego body.
This handles inter-vehicle occlusion; road geometry is not independently tested.
Vehicle intersections lie above the road surface in this scene.

Full auxiliary output and `--/rtx-transient/stableIds/enabled=true` let the capture
resolve actual return IDs to vehicle prim paths. Unmapped/nonvehicle IDs remain
`-1`, not an invented vehicle identity. Actual and independently expected vehicle
labels are compared over their union. Both raw arrays and the map are retained.
See NVIDIA's [RTX sensor API](https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.sensors.experimental.rtx/docs/api.html)
and installed `resolve_lidar_object_ids.py` example for ID semantics and limits.

## Frozen acceptance protocol

The configuration in `experiments/configs/replay_lidar_validation.json` declares
a 48.3-second replay, ending before the ego disappears. It requires at least 470
evaluated scans after three startup scans, 1,000 checked vehicle returns, all four
other vehicles represented, and at least 99% identity agreement. Every scan with
predicted vehicle intersections must meet p95 range error ≤10 cm and maximum
error ≤20 cm. These strict limits are not relaxed after seeing outliers.

Per-ray time is scan timestamp plus nanosecond offset, mapped to USD time by the
previously measured 2/60-second offset. That offset is checked against both GMO
sensor-frame endpoint positions throughout this replay with a 5 cm tolerance;
it is not merely assumed to transfer. Scan cadence must remain 0.1 seconds within
100 ns, offsets must fall within a scan, frame-end metadata must agree, and output
must be spherical, sensor-relative, and non-motion-compensated. Callback delivery
time is recorded separately. Pose timing here checks translation, not quaternion
orientation agreement; rotation affects the independent ray geometry check.

## Reproduce (Command Prompt, repository root)

```cmd
C:\isaacsim\python.bat scripts\validate_replay_lidar.py
.venv\Scripts\python.exe scripts\analyze_replay_lidar.py outputs\replay_lidar_validation\YOUR_RUN_ID
.venv\Scripts\python.exe -m pytest -q
```

Use the appropriate external Isaac installation path on another machine. The
default inputs are the local `outputs/sumo_replay/recording.json` and network.
They are ignored artifacts, not included in a Git clone. Regenerate them using
the recording instructions in `ovrtx-replay.md` and a compatible checkpoint.
`--frames 600` is a diagnostic short run and intentionally fails full coverage.

Each run stores source snapshots, resolved configuration, recording/network,
source checkpoint hash, seed, Git commit/dirty state/diff, runtime versions, raw
scans, per-scan metrics, shared USD layers and artifact hashes. The offline
analyzer verifies hashes and recomputes geometry, then tests a 250 ms timestamp
shift and reversed azimuth. A negative control failing never turns a failing
original into a pass. The process exits nonzero when acceptance fails.
These corruption controls perturb the geometric reference time/direction; captured
frame metadata is unchanged, so their timing-gate results are not a new timing
corruption test.

## Interpreting results

See the machine-generated `replay-lidar-results.json` for complete run IDs,
metrics, gates, and corruption controls. Coverage, geometry, and timing are
reported separately. An animation that looks correct is not a numerical pass.
Scans with no predicted vehicle intersections are counted explicitly. No rays
are selected because their measured error happens to be small.

The first full capture passed timing and coverage but **failed strict geometry
and identity acceptance**. Its supplied StableIdMap remained unchanged and
contained only the two vehicles present at initialization, not the later
background vehicles. After agent_1 disappeared, one late portion had zero
identity agreement. This is an observed mapping limitation to investigate;
unresolved IDs are not replaced by geometry-derived labels to manufacture agreement.
The second full capture additionally retains the unmodified per-point ID bytes
so future mapping diagnostics can be performed offline. This extra saved channel
does not change the sensor configuration, scene, timing, or acceptance rules.

Earlier attempts are retained: a runtime VERSION-file lookup failure and a
10-second diagnostic without the renderer's stable-ID setting (zero identity
agreement and deliberately insufficient full-route coverage). These are not
successful confirmation runs.

The selected Example_Rotary sensor has a nonzero azimuth noise standard deviation
(approximately 0.015 degrees in the installed configuration). At silhouettes,
a small angular perturbation can change a car hit into a road hit, creating a
large range difference. This is a plausible explanation to investigate, not a
reason to silently remove those returns or relax the strict maximum-error gate.
Matched-identity p95 and counts over 20 cm are diagnostic additions only.

## Limitations and learning exercise

These checks validate returned-ray consistency against block-car geometry, not
full firing-pattern recall, road accuracy, real lidar calibration, physical
steering, or safe autonomous driving. Stable-ID mapping can have unresolved
procedural subprimitive IDs. GPU sensor noise is not guaranteed bitwise repeatable.
This single seed/checkpoint recording is not general traffic robustness evidence.
Keep the lidar-to-policy stage gated on resolving or explicitly modeling failures.

Exercise: explain why transforming a ray into a rotated box's coordinate frame
reduces the intersection problem to the earlier axis-aligned test. Then explain
why a tiny direction error at a silhouette can create a very large range error.

The Isaac validation skill guided motion ownership and test scope, the OpenUSD
skill guided coordinate/layer invariants, and the reproducible-experiments skill
guided immutable evidence and explicit failed gates.
