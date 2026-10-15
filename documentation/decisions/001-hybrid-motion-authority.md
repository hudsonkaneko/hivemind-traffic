# ADR 001 — One motion owner per car in a hybrid traffic simulation

Date: October 3, 2026. Status: accepted development direction; implementation
and validation are staged in the [active roadmap](../hybrid-roadmap.md).

## Context and evidence

The current two-car experiment uses SUMO for movement and Isaac Sim for geometry
and RTX LiDAR. The [cooperative report](../cooperative-lidar-results.md) records
scripted avoidance/V2V results, repeat differences, and reliability failures.
The separate [Leatherback prototype](../vehicle.md) proves RC-scale wheel/joint
control in Isaac, not full-size highway dynamics or LiDAR-driven learning.

The [capacity study](../capacity-results.md) shows that inexpensive background
traffic and independent RTX sensors have different costs. It excludes full-size
physical vehicles and cannot determine their maximum count. The project therefore
needs a small physical research fleet before scaling or visual polish.

## Decision

Use **Isaac Sim/PhysX** as the sole motion authority for designated physical
research cars. Explicit driver controllers issue steering, propulsion, and brake
commands; the physics engine integrates their motion and contacts. Use **SUMO**
as the sole motion authority for economical background traffic, while preserving
the existing all-SUMO-motion demos as baselines.

The bridge operates in both directions: SUMO background state becomes Isaac
sensor-visible geometry/kinematic proxies; Isaac physical state updates SUMO
shadow vehicles so background drivers can react. Shadows do not command the
real physical car. Fix ownership for each whole episode initially. Do not add
dynamic handoffs until identity, timing, contacts, and synchronization pass.

A separate small all-Isaac validation mode will have no SUMO dependency. The
hybrid fleet will continue to require SUMO by design. Isaac Lab is the future
training/evaluation framework, not a second physics engine or pose authority.

## Responsibility transfer

The physical-car column is proposed implementation, not existing functionality.

| Responsibility currently supplied by SUMO | Physical research-car replacement | SUMO background and boundary |
| --- | --- | --- |
| Longitudinal motion, acceleration, speed integration | PhysX vehicle dynamics driven by explicit propulsion/brake commands | SUMO native or declared external background controller integrates background motion |
| Lane following | Shared lane geometry, trajectory tracker, steering controller | SUMO continues native lane following |
| Car following and speed choice | Scripted human-like or AV behavior model, then planner and low-level controller | Native SUMO human-like model or genuine lightweight AV controller; identify the model in results |
| Lane-change decisions and lateral motion | Behavior chooses intent; planner checks feasible maneuver; controller tracks it through physics | SUMO owns background lane changes; bridge publishes their actual intermediate poses |
| Routes and lane connectivity | Backend-neutral route/lane representation and progress projection | Route adapter maps common routes to SUMO edges/lanes |
| Traffic demand and departures | Seeded demand manifest and safe entry queue shared by backends | Native insertion is observed and reconciled; requested demand is not assumed inserted |
| Human variability | Bounded seeded reaction delay, headway, speed preference, perception/control error | Corresponding declared SUMO driver parameters; not assumed behaviorally identical |
| Vehicle lifecycle and arrivals | Episode manager creates/removes bodies at safe boundaries and records reason | Explicit bridge events create/remove shadows and background proxies; no ID reuse |
| Collisions and overlap checks | PhysX contact data plus common geometric safety checks for physical pairs | Native SUMO collision information plus cross-boundary geometry checks; reject duplicate counting |
| Road support and tire contact | Collision geometry, mass/inertia, suspension, tire friction, solver configuration | Kinematic backgrounds do not share physical momentum exchange |
| Simulation clock and ordering | One fixed-step scheduler with explicit physics, control, sensor, and bridge boundaries | Exactly one SUMO step per scheduled traffic tick; no free-running competing clock |
| Vehicle identity and state lookup | Versioned episode/vehicle IDs, owner, body-center pose and velocity | Convert SUMO front-bumper pose explicitly; retain v1 semantics |
| Trip completion, travel time, delay, distance | Backend-neutral event/trajectory metric collector with defined free-flow reference | Consume native records only when definitions match; otherwise recompute consistently |
| Queueing, throughput, stops, lane changes | Shared route-based detectors and threshold/event definitions | Same detector definitions for SUMO and physical states where comparable |
| Ground-truth neighborhood observations | Explicit privileged-state observation adapter | Never present simulator state as LiDAR inference |
| Sensor-visible vehicle placement | Actual physical chassis pose and sensor extrinsics | Background proxies remain visible regardless of whether equipped with sensors |
| Seeds, repeatable resets, and termination | Common study configuration, seed streams, reset/termination rules, immutable evidence | Preserve SUMO version/input/seed provenance and numeric tolerance differences |

## Driver stack and interface

Separate **behavior** (what to do), **trajectory planning** (a feasible route
through space and time), and **low-level control** (actuator commands that track
it). Begin with testable scripted components. Learned policies may later replace
one layer at a time without silently changing the others.

A command identifies episode, vehicle, sequence, issue time/tick, and expiry.
Steering is in radians with positive left; propulsion and braking have bounded
normalized values with an explicitly documented physical mapping. Reject
non-finite, wrong-owner, old-episode, out-of-order, or expired commands. Braking
takes precedence over propulsion; expired control cannot leave persistent drive
torque. A bounded safe-stop fallback is tested under sensing and communication
failures. Physics state writes are allowed for reset, not normal driving.

The command contract does not assert that all vehicles have identical dynamics.
Every vehicle configuration records mass/inertia, geometry, contact/friction,
actuator parameters, rate limits, and the speed envelope actually validated.

## Sensors, V2V, and observation provenance

Preserve vehicle identity, episode-relative simulation time, coordinate frames,
sensor extrinsics, acquisition intervals, and packet delivery time. Sensor clocks
are not inferred from wall time. A new contract must distinguish SUMO
front-bumper coordinates, physical body-center coordinates, and sensor frames.

Keep LiDAR-derived observations separate from simulator-state observations and
ground-truth evaluation labels. Labels may supervise perception or evaluate it;
that does not authorize giving them to a sensor-only controller. V2V messages
carry identity, sequence, timestamp/expiry, and declared intent/state provenance.
No, ideal, and degraded communication modes retain matched local-safety rules.

## Experimental validity

Treat driver type, physics backend, and sensor allocation as separate factors.
For an AV-ratio or communication comparison, freeze movement-backend assignment,
sensor budget, demand, role ordering, seeds, routes, horizon, and safety policy.
All-human may still include physics-driven human-like cars. All-AV cannot mean
native human SUMO drivers whose labels were changed to AV.

Existing throughput/travel-time/progress definitions can be retained where
their units and event meanings match. Implement equivalent collision, road
departure, braking/jerk, trip accounting, and unfinished-trip metrics from the
new physical trajectories. Model-specific fuel/emissions or SUMO intervention
metrics are not automatically valid for PhysX. Declare differences explicitly.

Changing dynamics changes the task. Existing SUMO policy results and weights
are baselines, not proof of transfer; re-evaluate or retrain. Compare matched
new-physics scripted and learned controllers before attributing benefits to V2V.
Report per-seed outcomes, failures, sample counts, dispersion, group fairness,
and computing cost. RTX/PhysX repetition uses documented tolerances rather than
an unsupported promise of universal bitwise determinism.

## Risks, mitigations, and rejected shortcuts

- **Kinematic collision boundary:** a SUMO proxy does not react physically to an
  impact with a research car. Detect contact/overlap and fail or stop the trial;
  do not claim crash realism. Validate physical interactions separately with
  two physics-driven vehicles.
- **Stale shadows:** delayed physical state can cause unrealistic background
  reactions. Bound age, record synchronization error, and stop on broken state
  delivery rather than allowing independent clocks to drift.
- **Vehicle-model mismatch:** RC-scale Leatherback results do not calibrate a
  full-size car. Test torque, braking, turning, friction, and reset repeatability
  at 1–3 m/s before increasing the envelope.
- **Training integration:** installed native PhysX vehicle APIs may not expose
  the same Lab interfaces as an articulation. Verify step/reset/state access
  before training; keep an articulated-vehicle alternative as a decision gate.
- **Resource limits:** start with one physical car, then two, with at most two
  independent LiDARs initially. Benchmark each integrated tier and GUI separately;
  short simple-box probes cannot justify a large physical fleet.
- **Pose teleporting instead of physics:** rejected for evaluated research cars
  because it bypasses actuator and contact behavior. It remains intentional for
  background render proxies under their separate SUMO authority.
- **Moving every background car into PhysX immediately:** deferred until measured
  cost and scientific need justify it. It is not necessary for the hybrid goal.

## Consequences

The bridge and metric layer add complexity, but preserve a scalable baseline and
make fidelity explicit. First implement the smallest physics/control fixture,
then shared geometry and LiDAR, then two-way traffic interaction. Keep old
experiments runnable, unique evidence packages intact, and the learning document
aligned with verified repository changes through the [workflow](../workflow.md).
