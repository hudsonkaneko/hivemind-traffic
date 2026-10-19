# Descriptive statistics for the loop showcase

Updated October 16, 2026. This increment adds a bounded streaming statistics
collector and hand-computed CPU checks. Runtime integration, GUI acceptance and
matched timing evidence are recorded in the parent showcase validation note;
unit tests alone do not qualify the running viewer or establish an improvement.

The source remains privileged simulator state and the known circular map. The
scene contains one native PhysX car controlled by a script and scripted,
nonreactive kinematic backgrounds. These statistics are not LiDAR measurements,
human-driver behavior, a learned policy evaluation or a mixed-traffic fairness
experiment. They do not change steering, propulsion, braking or safety gates.

## Inputs and observation window

`traffic/showcase_metrics.py` accepts one `VehicleSample` for each **active**
logical vehicle, with collision-box center, yaw, nonnegative measured scalar
speed, actual box dimensions, an unconstrained reference speed and role.
The main car's native scalar speed is planar velocity magnitude. Backgrounds
use their measured forward speed in their declared forward circular motion.
Negative speeds are rejected by the collector, not interpreted as reversal.
Wrong-way and native-motion validation remain independent safety checks.

The runner supplies samples at 120 Hz. Requested sidebar refresh frequency does
not change the statistics sample rate. Only intervals with two consecutive
`drive` samples contribute to drive summaries. Initial settling and the final
forced-braking phase are excluded. The first sample, first sample after a phase
change and first sample of a new vehicle lifetime establish a baseline without
inventing an elapsed interval. Phase-boundary excluded time is reported. A pause
does not advance simulation time and therefore adds no vehicle exposure.

Current integration uses the post-step `tick + 1` sample and its corresponding
simulation time. Main-car geometry comes from `body_box(state, metadata)`, which
applies the prepared/public model's collision-center offset and dimensions;
scalar speed comes from the native chassis state's planar velocity magnitude.
The fleet adapter supplies active native transform/velocity readback at the
same tick, with speed obtained by projecting planar velocity onto the body's
forward direction. This preserves the difference between collision-box center
geometry and the chassis state's velocity reference. TTC therefore remains the
stated scalar/tangent approximation if the collision center is offset or the
car moves laterally.

The command-line launcher enables statistics by default for v09 and accepts
`--no-statistics` to disable them. V07/v08 opt in with `--statistics`; the
programmatic `make_config()` default stays false to preserve historical config
snapshots. The enabled flag is saved in the resolved configuration, and the
metrics module is included in the run's source hashes. Collection does not add
the extra sidebar text to the v07 comparison layout. The v08/v09 sidebar shows
measured net time gap, accumulated ego time loss and near-miss proxy events.
The detailed quantities are in each trajectory row's `traffic_statistics`
record, the run result and `traffic-statistics.json`. A partial report saved
during failure is not a successful run; read it together with the enclosing
probe result and its safety/process gates.

The positive-control `contact-check` mode intentionally contains a stationary
obstacle without a positive free-flow target; statistics are rejected for that
mode instead of inventing congestion delay. During drive the collector receives
the signed projected background speed and rejects a negative value. During the
excluded settling/final-braking phases the runner may pass its magnitude to
keep small stopped-body readback noise from blocking cleanup; those values add
no drive distance, delay or derivative evidence. The saved summary identifies
its last observed simulation time and phase, including `null` for an empty run.

A gap greater than 0.25 simulated seconds breaks derivative continuity; its
duration is reported as excluded gap time. Duplicate/backward times, malformed
IDs, nonfinite values and invalid dimensions are rejected before changing the
collector. Numeric inputs are bounded to magnitude 1e9, time resolution must be
at least 1e-9 s, and free-flow references must be at least 1e-6 m/s to keep
derived values finite. These bounds greatly exceed the showcase envelope.

Re-entry uses the fleet's generation-aware logical ID, for example a slot's
`_g001` and `_g002` lifetimes remain separate. A retired identity cannot silently
reappear. Dormant native slots are absent from the input and accrue no duration,
distance, delay or fairness weight. A disappearing vehicle's last interval ends
at its last observed sample; the collector does not extrapolate to an unseen
exit time. The report distinguishes lifecycle exits from vehicles still active
at the last sample, whose observations are right-censored by the run ending.

## Definitions

| Quantity | Definition and interpretation |
| --- | --- |
| Speed | Time-weighted scalar speed using endpoint trapezoids; per-lifetime mean, RMS, minimum and maximum. RMS uses endpoint-weighted squares, not an assumption of exact continuous motion between samples. |
| Speed-integral distance | Integral of scalar speed by trapezoids. This includes lateral travel and is not route progress, completed-trip distance or the existing position-distance counter. |
| Acceleration | Difference of successive scalar speeds divided by elapsed simulated seconds. The output retains `longitudinal_acceleration_m_s2` as its field name, but this is a tangential approximation, not signed body-frame acceleration or a lateral-comfort measurement. No smoothing or differentiation of throttle commands is used. |
| Jerk | Difference between successive acceleration estimates divided by the separation of their interval midpoints. For unequal sample intervals this separation is half their summed durations. The same scalar-speed limitation applies. Its reported observation duration is the midpoint duration, which differs from speed exposure. Finite differencing may amplify native-state noise. |
| Hard braking | A sampled acceleration interval at or below -3.0 m/s². Contiguous qualifying intervals count as one event; their durations sum to braking exposure. This is a declared project diagnostic threshold, not a regulatory or validated comfort criterion. |
| Stopped exposure | Sum of eligible interval durations whose ending scalar speed is at most 0.1 m/s. This is a sampled estimate with interval-size boundary uncertainty. |
| Measured leader | The closest eligible forward vehicle within 150 m of center-to-center arc distance on the known circle, whose collision footprint's radial interval overlaps the ego footprint. Angular extents come from actual box corners; radial bounds include the closest point on box edges. This permits partial lane-change overlap without rounding to a lane label. |
| Bumper gap | Forward angular distance from the ego footprint's front angular extent to the leader's rear angular extent, multiplied by the ego's actual circular radius and clamped to zero. This is a geometric arc gap, not minimum Euclidean clearance. Outer lanes do not incorrectly use the base 500 m radius. |
| Net headway | Bumper gap divided by measured ego speed. This net time gap differs from front-to-front headway, which is also reported. Both are undefined at ego speeds at or below 0.1 m/s; a stopped vehicle still has a measurable distance gap. Neither is copied from the requested 1.8–3 s following preference. |
| Closing TTC proxy | Bumper gap divided by positive closing arc speed. Forward scalar speeds are projected onto each vehicle's local circle tangent; the leader's speed is converted to the ego radius using the radius ratio. Equal-speed or separating vehicles have undefined TTC, not zero or a serialized infinity. This constant-angular-motion proxy ignores future steering and changes of footprint yaw/radius; it is not a general collision prediction during lane changes. |
| Near-miss proxy | An ego-level observed episode when exact nearest clearance supplied by the existing safety calculation is below 1.0 m **or** eligible leader closing TTC is below 1.5 s. Events are false-to-true transitions; exposure is the sum of eligible interval durations whose ending sample is true. The two conditions are one union, so simultaneous triggers do not double-count. |
| Free-flow time loss | Sum of `max(0, 1 - measured_speed / reference_speed) × dt`, using endpoint trapezoids. Overspeed earns no negative delay credit. This is time lost relative to the declared instantaneous speed reference, not actual travel-time delay between an origin and destination. |

The near-miss thresholds are exploratory project definitions. A close stationary
pair can trigger the clearance proxy; a close adjacent pass can also trigger it
without a forward leader. This is why the output is explicitly a **proxy**, not
a count of proven avoided crashes. It does not replace collision/contact tests.
An event already true at the first drive sample counts as one observed episode
with zero prior exposure. After an excluded time gap, a still-true condition
starts a new observed episode; continuity across missing data is unknown.

Gap, headway and TTC are **ego-only** quantities. Their weighted summaries
include only eligible samples, with each denominator reported. No leader within
the stated horizon produces `null`; no-leader time is not folded into the mean
as an enormous gap. The near-miss report separately gives monitored drive time.
The collector does not claim background-to-background collision observation.

The main-car reference is its applied unconstrained target, before the
controller reduces it for obstacles. For static background fixtures it is the
declared per-car nominal speed. For live traffic it is the current applied
reference speed scaled by that vehicle's actual radius divided by 500, matching
the fleet's common angular motion. Requested slider targets and applied ramps
are different: this definition measures loss relative to the applied ramp and
does not label a deliberate target reduction as congestion delay. It also does
not estimate traffic-caused delay separately from acceleration capability or
controller choices.

## Fairness and incomplete observations

Each lifetime retains drive exposure, reference-distance integral, speed-distance
integral, free-flow time loss, speed/acceleration/jerk moments and braking counts.
Fairness uses each lifetime's `time_loss / observed_drive_time` fraction so a
longer-observed car does not automatically look less favored. Lifetimes below
1.0 s of drive exposure are excluded from fairness and counted explicitly; they
remain in the ledger. Exits are not silently discarded.

The report gives the mean and range of per-lifetime time-loss fractions and
Jain's index of the achieved fractions `1 - time_loss_fraction`:

```text
J = (sum(achieved_fraction))² / (n × sum(achieved_fraction²))
```

The index is `null` for no eligible lifetime or an all-zero achieved fraction;
an entirely stopped population is not labeled perfectly fair. Results are
provided both across all lifetimes and separately for `ego` and `background`.
The across-role number mixes different motion models and reference speeds and
is descriptive only. A single eligible ego necessarily has index 1 unless its
achieved fraction is zero; that is not evidence of a coordinated benefit.
Repeated generations can contribute multiple lifetime observations, so this is
not a person-level fairness claim. Matched demand, assignments, seeds, observation
windows and backend/model composition are required before comparisons.

No destination-based trips exist in this fixture. The report's aggregate
`completed_trips` is `null`, and each lifetime is explicitly marked as not an
observed completed trip. Laps, detector passages, lane changes and overtakes
remain separate quantities and must not be renamed as completed trips.

## Storage, cost and validation

The collector stores running moments and one summary per observed lifetime,
not a growing per-tick sample list. Default limits are 81 simultaneous vehicles
and 4096 lifetimes; exceeding either rejects the update instead of silently
dropping evidence. The latter exceeds the existing fleet's 2048 lifecycle-event
bound. Per-tick work is linear in active count and independent of prior lifetime
count. Cheap angular/radial bounds reject far objects before exact footprint
work. Final summaries iterate the bounded lifetime ledger.

`update()` returns a compact current ego record for evidence rows and the
sidebar. `summary()` returns JSON-safe aggregate and per-lifetime evidence; all
undefined values are `null`, never NaN or infinity. The bounded collector itself
does not persist every background measurement, so an aggregate alone cannot
reconstruct every 120 Hz input. Reproduction requires the recorded configuration,
source version, sampling contract and a new matched run, or an external complete
input recording. It must not be presented as raw full-fleet sensor data.

CPU acceptance on October 16, 2026: **27 tests passed** using the existing
traffic environment:

```text
.venv-traffic\Scripts\python.exe -m unittest discover -s tests -p test_showcase_metrics.py -v
```

Tests use hand-computed circular bumper gaps, outer-lane and seam geometry,
partial radial overlap, nonclosing/stationary cases, acceleration and midpoint
jerk for unequal intervals, explicit time-loss and Jain examples, phase
exclusion, lifecycle censoring, generation reuse rejection, finite JSON,
malformed-input atomicity and fixed storage across repeated samples. The
acceleration example uses speeds 10, 8 and 2 m/s at times 0, 1 and 3 s: interval
accelerations -2 and -3 m/s², then jerk -2/3 m/s³. The fairness example has
achieved fractions 1 and 0.5, giving Jain's index 0.9.

These checks establish the definitions and implementation independently of the
controller. They do not establish physical realism, sensor accuracy, improved
driving, 30 Hz display acceptance or 60 FPS. Runtime timing must include the
collector when reporting statistics-enabled performance.

October 19 native evidence: independent reductions passed 86 consistency checks
on the first startup smoke and 88 on the original failed endpoint. Ego metrics,
every membership/lifetime boundary, exposure and saved-summary consistency were
checked against retained telemetry. Background derivatives are not fully
reconstructible from decimated planner records. The [measured report](showcase-validation-20261019.md)
preserves these limits and separates metric correctness from driving and timing
acceptance; the endpoint's driving failure remains a failure.
