# Scripted lane following in Isaac physics

This increment implements the analytic geometry part of roadmap step 6 and the
low-speed straight/curved-route subset of step 7. It is **known-map, simulator-state
control**, not LiDAR autonomy, a trained policy, traffic interaction or a highway
dynamics certification. Existing SUMO/PettingZoo/PPO and Leatherback experiments
and their evidence remain unchanged. No dependencies or runtime files were changed.

## Purpose and implementation

We need a repeatable physical driving baseline before asking learning or sensors
to solve the same task. The car must respond to wheel commands, not have its pose
copied along the route. Detailed road art is not required to measure this.

| Boundary | Responsibility and source |
| --- | --- |
| Lane map | [Route fixture](../scenarios/physics-road/routes.json) and [geometry](../traffic/lane_geometry.py): immutable 100 m straight, left and right routes, 3.6 m width; curved routes are 20 m straight + 60 m radius-60 arc + 20 m straight. |
| Behavior | [ScriptedCruiseBehavior](../traffic/path_following.py): select the known-clear route and request a target speed, initially zero for settling and then 3 m/s. No obstacle decisions or lane changes. |
| Path planning | `PathPlanner`: turn that intention into an episode/vehicle-specific immutable route reference, target speed and chassis-center stop station with an expiry. This is route selection, not a general trajectory optimizer. |
| Low-level control | `PathFollower`: rear-axle pure pursuit for steering and a proportional speed/brake servo. Reduce requested speed near the endpoint and latch full braking at station 99.8 m. |
| Command safety | [DriverControlGate](../traffic/driver_control.py): identity, ordering, expiry, finite numbers, steering clamp/slew and braking precedence. |
| Movement | [PhysxVehicle](../traffic/physx_vehicle.py): steering angles and wheel drive/brake torques; PhysX determines position, orientation and speed. No moving chassis pose/velocity writes. |
| Evidence | [Probe](../scripts/probe_lane_following.py), [frozen config](../experiments/physics-lane-following.json), [acceptance](../traffic/lane_validation.py), [contact reports](../traffic/vehicle_contacts.py) and the existing bounded supervisor. |

Geometry uses metres in XY, Z up, radians counterclockwise, and positive lateral
offset to the left. Route progress and stop distance refer to the chassis root;
pure pursuit uses the rear axle 1.6 m behind it. The 3.2 m wheelbase is checked
against the physical model. SUMO's existing front-bumper contract is not changed.

Analytic projection finds the closest route station. Shared samples include
segment joins and have at most 0.5 m spacing along each sampled center/boundary
curve. Endpoint tangents extend the lane corridor; the car may straddle the
start/end station without falling off a road cliff. A 5 m padding convention is
recorded for future road rendering. The current surface is an **infinite collision
plane**; the sample visible ground is only 30 x 30 m. These are numeric driving-map
fixtures, not a new road mesh, GUI or completed SUMO map exporter. Cross-backend
route agreement within 0.05 m remains an open part of step 6.

## Control loop and safety

Behavior and planning run at 10 Hz; low-level commands at 60 Hz; command gating
and physics at 120 Hz. The first two initialization ticks only hold the brake
while native state becomes available. Thereafter the prior physical sample is
acquired at exactly the current decision time. Diagnostics on intervening physics
ticks describe the latest control decision, not a newly calculated decision.

Behavior intentions expire within 36 physics ticks (0.30 s), route references
within 24 (0.20 s), and commands within 12 (0.10 s); downstream references never
outlive the input that authorized them. Wrong identity/frame/source, stale/future
state, invalid references and excessive tracking error request zero propulsion
and full brake. Steering is limited to ±0.5 rad and 0.5 rad/s by the existing gate.
The fixture uses up to 700 N m per front drive wheel and 1500 N m per braking wheel.

Pure pursuit selects a point ahead of the rear axle and geometrically chooses a
turn toward it. Lookahead is `3 + 0.6 * speed` metres. The approach-speed envelope
is `sqrt(2 * 1.5 * max(0, remaining - 0.2))` m/s, capped by the target speed.
The 1.5 m/s² term is a planning assumption, **not measured deceleration** or a
guaranteed emergency stopping distance. The actual torque/brake response is
measured separately. Method reference: [Coulter's pure-pursuit report](https://publications.ri.cmu.edu/implementation-of-the-pure-pursuit-path-tracking-algorithm).

## Predeclared acceptance and evidence safeguards

Each route runs twice, using seed 101, in a fresh scene: 2 s settling + 48 s
driving/endpoint hold, 6000 physical samples each. The suite has a 420 s external
deadline, an internal deadline 15 s earlier and a 90% whole-GPU memory stop guard.
Every attempt gets a new output folder, copied source hashes, config, initial
scenes, sampled routes, per-tick trajectories, contacts and lifecycle events.

The unchanged pre-run gates require:

- Initial station within 0.05 m of zero and observed forward progress of at least
  99.5 m; starting parked at the end cannot pass.
- Driving-phase lane-center RMS ≤0.20 m and maximum absolute error ≤0.50 m.
- Conservative chassis-envelope containment inside the 3.6 m lane over the whole
  episode, including settling; not just a center-point or four-corner shortcut.
  The bound accounts for all eight box corners, orientation and maximum route
  curvature, and applies only to these simple non-self-intersecting corridors.
- No reported undesired rigid-body contact. A separate intentional static-barrier
  approach must first produce a decoded, nonempty contact event involving the
  car and barrier. Raycast tire support is distinct from rigid chassis contact.
- Throughout the final 5 s: station within 0.50 m of 100 m, speed ≤0.05 m/s and
  position drift ≤0.05 m. Combined tilt ≤5°, four wheels supported in ≥99% of
  post-settling samples, and unsupported gaps ≤0.10 s.
- Complete ordered finite telemetry, exactly four boolean wheel-support values,
  no driver/command fallback during driving, and repeated position/speed/yaw
  differences ≤0.001 m, m/s and rad at every matched tick.
- The supervisor retains its separate zero-USD-reference-warning check. Passing
  physical metrics alone does not override a failed lifecycle gate.

CPU tests cover geometry, control/state expiry and malformed data, contact report
decoding and acceptance false positives. The controller's CPU bicycle fixture is
an algorithm test, **not** evidence about real PhysX behavior. Physical fault
injection, displaced starting poses and multiple evaluation seeds remain later
robustness work; ordinary nominal-route passes do not prove general safe driving.

## Recorded outcome — October 4, 2026

**Physical lane subset verified; overall lifecycle gate still failed.** Run
`20261004T194416Z-42992541` completed all six lane episodes (300 simulated seconds)
plus the detector control. The supervisor returns exit 1 because one existing
USD reference-count warning occurs when closing the first control scene. The
physics child exited 0. No acceptance threshold was changed after running.

| Route (both repeats) | Lane RMS, m | Maximum error, m | End station, m |
| --- | ---: | ---: | ---: |
| Straight | 0.00000156 | 0.00000511 | 99.877388 |
| Left radius 60 m | 0.01840588 | 0.04865949 | 99.883338 |
| Right radius 60 m | 0.01840552 | 0.04865743 | 99.883348 |

All normal-route cases had zero reported rigid contacts, no departure and no
command/driver fallback. The conservative lateral envelope peaked at 1.07085 m,
inside the 1.8 m half-lane. Worst endpoint error during hold was 0.122612 m;
five-second hold speed stayed below 0.000288 m/s and maximum drift was 0.00000416 m.
Every post-settling sample had four supported wheels. Peak achieved speed was
2.97030 m/s. Position, speed and yaw were identical across matched local repeats;
this is not a promise of cross-platform determinism.

The contact control produced four chassis/barrier contact points at tick 566
(4.7167 s). This first report is within PhysX contact margins (positive separation
about 0.0322 m, zero impulse). It proves the reporting path detects proximity
contact; it does **not** validate a realistic crash/impact response. All seven
stage closes emptied the cache/context, but the warning still prevents a clean
lifecycle claim. Logs contained no `[Error]` lines in this attempt; other warnings
remain in the raw log. Historical failed attempts were not edited.

**603 CPU tests passed.** Independent agent review recomputed line/arc projections
from separate equations, verified all 63 artifact and 15 captured-source hashes,
and confirmed the physical metrics and failed overall status. See the
[compact hash-verified report](lane-following-results.json); raw evidence is local
under `outputs/vehicle_lane_following/20261004T194416Z-42992541`.

On the installed Isaac 6.0.1 RC / RTX 5070 setup, startup was 41.23 s and the full
process took 205.11 s. Process peak RAM was about 5.23 GiB; sampled whole-GPU peak
was 3765 MiB, including the desktop. This unpaced headless one-car run includes
trace recording and no LiDAR/rendered GUI. It is **not** a paced real-time or fleet
capacity measurement, and it does not resolve previous GUI memory-growth concerns.

## Portable numerical demonstration

From your clone's repository root with the traffic Python environment active,
configure `isaac_root` in ignored `hivemind.local.json` or `ISAAC_SIM_PATH` as in
the [portable setup](portable-demos.md). These commands contain no user-specific
paths; the supervisor selects the separate installed Isaac Python. Run one GPU
job at a time. No training starts and no GUI opens.

```text
python -m experiments.verify_vehicle_stage --stage lane-following --check
python -m experiments.verify_vehicle_stage --stage lane-following
python -m json.tool documentation/lane-following-results.json
```

`--check` only validates local configuration/runtime discovery. The second command
runs the deliberate contact-detector check followed by six lane episodes, then
prints the unique evidence directory and overall result. A clone contains the
compact report, not the large ignored raw traces; rerun locally to produce them.
Expect the recorded run's six physical cases to be passing while overall status
is false on the retained reference-warning gate. Do not treat that nonzero exit as
permission to suppress the warning or remove failed evidence.

## Next gates and your involvement

Next, mount LiDAR on this physics-owned chassis and validate extrinsics,
acquisition/delivery timestamps, freshness fallback and obstacle stopping. Keep
known-map/state observations distinct from sensor-derived inputs. The SUMO map
adapter, two-way background-traffic bridge, two physical cars, mixed ratios,
V2V failure tests and matched efficiency/safety/fairness studies remain on the
[active roadmap](hybrid-roadmap.md). Investigate historical lifecycle warnings
without rewriting their failed evidence.

**Before Isaac Lab policy training, involve the user directly.** Explain and
agree on observations, actions, rewards, algorithm, training budget and held-out
evaluation together. Scripted validation is authorized now; unattended policy
training is not. The prior Lab step/reset compatibility test is not training and
does not automatically make this lane-following task a complete Lab environment.
