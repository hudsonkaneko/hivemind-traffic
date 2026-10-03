# Controlled moving RTX lidar validation

## Question and scope

Does the installed RTX rotary lidar report ranges and directions consistent with
known moving geometry, at the correct acquisition times? This is a deterministic
two-box kinematic fixture, not a live SUMO run or an Isaac Lab dynamics task.
One USD timeline owns both vehicle poses; no physics engine competes for control.
The existing PPO checkpoint and traffic replay are untouched.

The reproducible-experiments skill guided the frozen configuration, separate
exploratory/confirmation runs, immutable evidence, and deliberate failure tests.
The Isaac validation skill guided the explicit pose authority and limited scope.

## Geometry and clock model

The ego moves along X at 1 m/s; the target moves at 3 m/s. The target is a
4 × 2 × 2 m box initially centered at (12, 2, 1) m. Lidar is mounted at (0, 0, 1)
relative to the ego. Both orientations are fixed. A static variant sets both
velocities to zero without changing geometry. The lateral offset makes reversing
azimuth a detectable error. USD uses meters, Z up, and 60 time codes/second.

The unit direction is `(cos(el) cos(az), cos(el) sin(az), sin(el))`, with angles
converted from degrees. A slab ray/box intersection computes the first surface
distance at each returned ray's acquisition time. Checks select rays by their
reported angles and expected geometry, never by small measured range errors.
The ROI is ±40° azimuth and ±3° elevation. Validity uses the installed GMO flag
value 64, asserted against the runtime enum.

NVIDIA defines acquisition time as GMO `timestampNs + timeOffsetNs`; see the
[GMO field reference](https://docs.isaacsim.omniverse.nvidia.com/latest/py/docs/source/generic_model_output/generic_model_output.html).
Callback time is recorded separately, not substituted for acquisition time.

Exploratory frame-start/end sensor poses exposed a 2/60-second difference between
the sensor clock and authored USD pose time in this installed runtime. The frozen
mapping is `usd_time = sensor_time - 2/60`. It is explicitly calibrated, not an
assertion that the clocks are inherently synchronized. Confirmation runs check
the mapping against sensor frame poses. Do not copy this offset into another
runtime or the full traffic replay without validating it there.

## Predeclared acceptance rules

`experiments/configs/lidar_validation.json` fixes 300 frames, 60 Hz, 0.1-second
scans, and three startup scans excluded before evaluation. Every subsequent scan
must meet the thresholds; no scan is removed because its error is large.

- At least 20 evaluated scans and 20 expected target hits per scan.
- At least 99% valid returned samples among expected target intersections, and
  99% geometric direction consistency among valid returned ROI samples.
- Per-scan 95th-percentile range error ≤0.10 m; maximum error ≤0.20 m.
- Strictly increasing scan timestamps with 0.1-second cadence within 100 ns.
- Point offsets lie within one scan, and scan/frame-end metadata agree.
- Callback time is recorded as a delivery diagnostic, not a gate on acquisition.
- Sensor-frame poses match the declared clock map within 0.02 m.
- Spherical, sensor-frame, non-motion-compensated output is required.

GMO here contains returns, not the complete emitted ray pattern. The fractions
above are internal returned-sample consistency, **not detection recall**. This
test cannot count rays which should have fired but were omitted from output.

## Reproduce (Command Prompt, repository root)

```cmd
C:\isaacsim\python.bat scripts\validate_moving_lidar.py --variant static
C:\isaacsim\python.bat scripts\validate_moving_lidar.py --variant moving
C:\isaacsim\python.bat scripts\validate_moving_lidar.py --variant moving
.venv\Scripts\python.exe -m pytest -q
```

The Isaac path is a machine-specific external dependency; substitute your own
installation path. Add `--gui` to inspect the fixture, but quantitative headless
results are the acceptance evidence. Each run prints its unique output directory.
To recompute metrics and run the two corruption tests without launching Isaac:

```cmd
.venv\Scripts\python.exe scripts\analyze_lidar_validation.py outputs\lidar_validation\YOUR_RUN_ID
```

Every run preserves resolved configuration, source snapshots, Git commit/dirty
state/diff, version and GPU information, authored USD, raw scans, per-scan metrics,
summary, and hashes. The analyzer verifies artifact hashes first. Generated data
remains ignored under `outputs/lidar_validation/`; logs are under `logs/`.
Raw arrays are azimuth degrees, elevation degrees, range meters, per-ray offsets,
flags, and scan timestamp—not Cartesian XYZ.

## Evidence and failure interpretation

See [machine-generated confirmation results](moving-lidar-results.json) for
exact run IDs and recomputed summaries. A 250 ms timestamp shift must fail the
moving geometry and timing gates. Reversing azimuth must fail geometry/direction
checks. These negative controls show the validator can reject plausible-looking
but incorrectly interpreted data.

Early exploratory attempts are retained: one USD vector-constructor mismatch,
one enum-import mismatch, and a diagnostic run before clock calibration and flag
decoding were finalized. They are not counted as successful confirmations.
Protocol revision 1 also rejected a static run because one callback's timeline
value differed from sensor frame-end time by two frames instead of one. That
delivery-time assumption was inappropriate: it is not a sensor acquisition
timestamp. Protocol revision 2 removes only that callback gate, retains cadence,
frame-end, per-ray offset and frame-pose gates, and is rerun on fresh data.
Small scan differences across repeated runs are possible because GPU sensor
noise is not pinned to a documented random seed; do not claim bitwise repeatability.

## Limits and next gate

No curved trajectories, rotations, changing occlusions, full firing-pattern
recall, real sensor calibration, learned perception, or steering dynamics are
validated here. ovrtx remains the separate RGB-rendering path. Before lidar is a
policy observation, apply these ideas to the actual traffic replay with its own
clock checks and object identities, then define and test an observation encoder.

Learning exercise: predict the front-face distance change after one second, then
explain why a ray's measured range is not identical to the center-to-center gap.
Interview explanation: “I built a sensor validation fixture using analytic
ground truth, exposed a clock offset, and verified that timing and coordinate
mistakes are rejected before using lidar as a learning input.”
