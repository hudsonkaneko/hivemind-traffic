# Active roadmap: physical research cars, economical background traffic

Updated: October 4, 2026. Status: physical lane-following subset verified;
scene-lifecycle reliability and full shared-map integration **in progress**.
This is the active development order, replacing the old immediate-next-step
sections in the preserved [implementation roadmap](../experiments/ROADMAP.md)
and [capacity study](scaling-foundations.md). Historical evidence is unchanged.

## Destination and boundaries

Isaac Sim/PhysX will determine research-car motion from explicit steering,
propulsion, and braking commands. SUMO will remain the inexpensive background
traffic backend and a preserved baseline. A two-way bridge will publish SUMO
background poses into Isaac and physical-car state into SUMO shadow vehicles.
Each real vehicle has one motion authority for the entire episode initially.
See the [architecture decision](decisions/001-hybrid-motion-authority.md).

Driver type (human-like, independent AV, communicating AV), movement backend, and
sensor allocation are separate configuration fields. Preserve exact AV shares of
0%, 25%, 50%, 75%, and 100%; do not quietly change dynamics or sensor budgets when
changing the experimental treatment. A four-car fleet is the smallest fleet that
represents all five ratios exactly. AV labels alone do not implement an AV driver.

The final research goal remains reproducible evidence about efficiency, safety,
and fairness of coordinated traffic. Simple visuals are sufficient while that
foundation is built. Detailed road/environment art is the teammate's scope;
prepared vehicle assets, overview/follow cameras, path overlays, and ovrtx remain
later integration work. A separate small all-Isaac scenario must ultimately run
without SUMO; the scalable hybrid mode need not eliminate SUMO.

## Existing foundation, not yet a physical traffic fleet

- The current [two-car cooperative experiment](cooperative-lidar-results.md)
  demonstrates scripted LiDAR/V2V decisions with SUMO-owned motion. Historical
  failed long runs and GUI memory growth remain relevant reliability evidence.
- The [Leatherback prototype](vehicle.md) demonstrates articulated steering and
  driven wheels at RC scale. Its waypoint PPO uses simulator state, not LiDAR.
  Its geometry, gains, brake behavior, and success rates do not validate a
  full-size highway vehicle.
- [Capacity measurements](capacity-results.md) isolate SUMO and RTX work. The
  20-car/four-LiDAR probe reached 30.44 ms p95 and 38.78 ms worst frame time before
  vehicle dynamics, perception, GUI, and recording. The 30 Hz budget is 33.3 ms.
  These measurements justify beginning with one physical car and adding sensors
  cautiously; they are not a proven full-physics fleet limit.
- [State contract v1](state-contract-v1.md) specifies SUMO front-bumper poses.
  Preserve it and its tests; add a versioned contract for physical center poses,
  authority, command age, and acquisition timestamps instead of redefining v1.

## Twenty steps and exit gates

Every threshold below is a **proposed acceptance target**, not a reported pass.
Before running a stage, freeze its exact scenario/configuration, tolerances,
seeds, hardware, and failure rule. Changes after a failed exploratory run require
a new versioned configuration; retain the failed attempt. A smoke-test subset
does not close the entire stage.

### Stage A — One physics-driven car

Steps 1–5 are the current implementation batch; their combined exit gate is open.

The first bounded physics/control subset now has passing evidence: two fresh-stage
28-second episodes at a 3 m/s target, physical propulsion/braking/turning, and
matching recorded trajectory repeats. See the [foundation report](physics-vehicle-foundation.md)
and [recorded results](physics-vehicle-results.json). This does not close the full
ten-reset, turn-radius, later 6 m/s, Isaac Lab compatibility, or reset/memory
reliability gates at that historical checkpoint.

The [next validation increment](physics-validation.md) now verifies braking at
1/3/6 m/s, left/right turn radius, five-second holds, two matching dynamics
repeats, ten-repeat final-state/cache/memory subchecks, and three real single-env
Isaac Lab episodes with automatic resets. All twelve dynamics cases passed, but
the standalone reset/dynamics supervisors remain failed overall because one USD
reference-count warning persists. Two candidate lifecycle fixes did not remove
it. The Lab task passed without that warning. Stage A is still not fully closed;
do not label scene rebuilding, highway speeds, GUI endurance or sensors clean.
The [lane-following increment](lane-following.md) now adds analytic geometry and
a scripted driver: all six 100 m straight/curved physical cases passed, with
curved-route RMS about 0.01841 m and max error 0.04866 m. The same lifecycle warning
still fails the separate overall gate. Next is physical-car LiDAR, while retaining
the lifecycle investigation and unimplemented SUMO/Isaac map-correspondence check.

| Step | Deliverable | Measurable exit gate |
| ---: | --- | --- |
| 1 | Preserve the working SUMO/LiDAR and Leatherback baselines | Record baseline commit, commands, versions, and evidence; run the existing CPU test suite; keep historical outputs and source snapshots unchanged. Record any current-runtime regression separately. |
| 2 | Versioned state and control contracts | Unit tests cover identity, episode reset, meter/radian conventions, pose reference, one owner, sequence ordering, expiry, non-finite values, and invalid commands. Transform round trips are within 1 mm and 0.01 degrees on fixed fixtures. |
| 3 | Simple full-size physical vehicle | Record chassis dimensions, mass/inertia, wheelbase, wheel radius, suspension, friction, collision shapes, and solver rates. During a 10 s settled test the chassis stays supported without persistent ground penetration or tipping; report contact and pose traces. |
| 4 | Steering/propulsion/brake adapter and runtime compatibility | Commands obey configured limits and steering slew rate. Brake overrides propulsion; stale/invalid/wrong-identity commands cannot produce drive torque. No episode-time chassis pose or velocity writes bypass physics. Demonstrate one installed Isaac Lab step/reset path or keep Lab compatibility explicitly open. |
| 5 | Low-speed dynamics and repeat suite | At 1 and 3 m/s, measure acceleration, coast, turn, and full braking. Proposed 3 m/s stop distance is at most 4 m from command issue, then speed below 0.05 m/s for 5 s. Steady low-speed turn radius is within 10% of the declared reference, with roll/pitch below 5 degrees. Ten same-config resets agree within 0.10 m final position and 0.05 m/s final speed. |

The first milestone excludes lane changes, traffic, LiDAR-based control, V2V, PPO,
asset polish, and highway-speed claims. The initial exploratory spike used a
3 m/s target. Complete the 1 and 3 m/s characterization before extending to 6 m/s.
Highway speeds need a later explicit validation envelope; do not scale the RC
vehicle's gains.

### Stage B — Road following, sensing, and two interacting cars

Step 6's analytic geometry and step 7's nominal 3 m/s lane-following subset now
have [recorded evidence](lane-following-results.json). SUMO route correspondence,
physical fault-injection/perturbed-start robustness, lane changes and steps 8–13
remain unverified. The driver uses known maps and simulator state, not LiDAR.

| Step | Deliverable | Measurable exit gate |
| ---: | --- | --- |
| 6 | Shared lane and route geometry, independent of road art | Centerlines, widths, curvature, and coordinate transforms are fixture-tested. SUMO/Isaac route correspondence is within 0.05 m on sampled reference points; static road rendering does not redefine the driving map. |
| 7 | Scripted behavior, trajectory planning, and low-level control | At up to 3 m/s, complete straight and curved 100 m routes with lane-center RMS error at most 0.20 m, maximum at most 0.50 m, no road departure, and no contact. Only then test planned lane changes under a declared acceleration/jerk envelope. |
| 8 | LiDAR on the physical chassis | Preserve extrinsics, vehicle/episode ID, acquisition interval, frame, and delivery time. In a 60 s moving fixture no future or wrong-episode scan is accepted; timestamp alignment error is at most one physics tick. Scans older than the configured 0.20 s limit trigger fallback within one control tick. Validate obstacle stopping without using hidden simulator labels as controller inputs. |
| 9 | Fixed-step multi-rate scheduler | Use one simulation clock. Over 120 simulated seconds, planned physics/control/SUMO/sensor counts match their schedule, drift is at most one physics tick, no step is applied twice, and late data never rewinds motion. Record wall-time p95/p99/worst delays separately from simulated time. |
| 10 | Two-way SUMO/Isaac bridge | For one physical car and one background car, record exactly one owner each. Shadows match physical state at synchronization boundaries within 0.05 m and 0.5 degrees. Spawn/remove events occur once; SUMO speed commands never overwrite the physical car. Inject stale state and verify bounded braking or a stopped trial. |
| 11 | One physical car interacting with one SUMO car | Matched following, sudden-braking, and cut-in fixtures complete without contact under declared feasible initial gaps. Background traffic must react to the physical car's shadow. A deliberate impossible-stop/contact case is correctly counted as failure, not a physically realistic impact. |
| 12 | Two physical cars, with optional SUMO background | Repeat braking/following/obstacle fixtures with two physics-owned cars and sensor mounts. The two-car fixture must also start, run, and produce metrics with SUMO absent and no TraCI connection. Repeat tolerances and contact criteria remain explicit. |
| 13 | Communicating-driver mode and failure tests | Compare no, ideal, delayed/lossy, duplicate, out-of-order, stale, and disconnected V2V. A stale message cannot override local safety; one car losing communications still follows its independent fallback. Log every delivered/dropped/expired message and finish or safely stop each case with no unreported contact. |

### Stage C — Matched mixed-traffic evidence and measured scaling

| Step | Deliverable | Measurable exit gate |
| ---: | --- | --- |
| 14 | Human-like, independent AV, and communicating AV drivers | A four-car fixture realizes exactly 0/1/2/3/4 AVs. The same seeded ordering assigns roles across ratios; fixed physics and sensor allocations are recorded independently. Human variation samples bounded reaction time, headway, speed preference, and control error without making unsafe spawn states. SUMO-background AVs must run an actual declared AV controller. |
| 15 | Seeded demand, route, entry, and removal manager | Repeat demand produces identical requested departures/routes/role draws. Track queued, inserted, completed, failed, removed, and still-active trips so accounting sums to requested demand. Reject unsafe insertions, retain waiting demand, and never reuse an ID in an episode. |
| 16 | Backend-neutral metrics | Hand-checked fixtures validate time, distance, throughput, travel delay, queueing, contact/near-contact, braking/jerk, overrides, communication cost, and compute. Include unfinished trips and group-level fairness. SUMO and physical metrics use the same definitions where comparable; model-specific quantities are labeled. |
| 17 | Matched scripted comparison study | Freeze at least five evaluation seeds before running the four-car ratio/mode matrix. Hold demand, assignments, physics/sensor allocation, horizons, safety gates, and observation provenance fixed. Publish all attempts, paired per-seed differences, dispersion, safety outcomes, and unfinished trips; no success claim from reward alone. |
| 18 | Scale total fleet 4, then 10, then 20 | Begin with no more than two independent LiDARs and a fixed small physical subset. Each tier must pass a 120 s headless test, then a separately measured GUI endurance case before promotion. Target paced RTF 0.95–1.05, p99 control completion within 100 ms, no stale-data safety violation, and whole-GPU memory below the configured 90% stop guard. Report startup time, peak RAM/VRAM, and fitted memory growth; sustained unexplained growth blocks longer operation. |

For the endurance case, predeclare a bounded 10-minute target after short tests
pass, save periodic state/telemetry, and stop on the resource guard or timing/safety
failure. Do not disable freshness checks to reach real time. Record skipped or
early-stopped endurance tests as incomplete. Large-car-count probes remain
separate from claims about the number of validated physical agents.

### Stage D — Learning and presentation

**User participation gate:** before starting Isaac Lab policy training, work with
the user to explain and agree on observations, actions, rewards, algorithm,
training budget and held-out evaluation. Scripted plumbing/validation may proceed;
do not launch unattended training under the current authorization. Existing
single-environment Lab compatibility evidence is not a trained driving policy.

| Step | Deliverable | Measurable exit gate |
| ---: | --- | --- |
| 19 | Isaac Lab training and held-out evaluation | Pass task reset/step/space/termination checks and single-environment smoke before vectorized training. Train independent control before coordinated control. Freeze checkpoint selection and compare to scripted drivers on unseen matched seeds, with safety/fairness/compute reports and explicit privileged-state versus sensor-derived observations. Old SUMO policy weights are not claimed transferable without re-evaluation or retraining. |
| 20 | Assets, teammate environment, cameras, overlays, ovrtx | Swap detailed assets without changing the declared physical model or observation interfaces unnoticed. Overview/follow controls do not change the experiment state. Path overlays identify planned versus executed paths. Re-run collision/sensor/performance gates after each material geometry change; verify ovrtx independently instead of equating it with Isaac RTX. |

## Planned controller boundaries and rates

Initial design targets, to be resolved in each fixture configuration:

- **Behavior:** approximately 10 Hz; selects lane/route/speed intentions from an
  explicit observation source. It does not write the vehicle pose.
- **Trajectory planning:** approximately 10–20 Hz; emits timestamped world-frame
  position, heading, curvature, and speed references with a validity interval.
- **Low-level control:** 60 Hz initially; tracks the reference and commands
  steering in radians (positive left), normalized propulsion and braking in
  `[0, 1]`. The adapter converts these to a documented physical actuator model.
- **Physics:** 120 Hz initially. SUMO remains at its configured fixed traffic
  step, initially 10 Hz. Sensor acquisition/render cadence is explicit and need
  not equal control cadence; record acquisition and delivery separately.
- **Fallback:** invalid/expired commands stop propulsion and apply bounded
  braking; invalid observations do not become zero-distance or clear-road data.
  Steering hold/centering rules and braking limits are declared per vehicle and
  tested, not guessed by the bridge.

The lane fixture implements the 10/10/60/120 Hz subset with timestamped immutable
route references (not a general time-parameterized trajectory optimizer).
The integrated SUMO/sensor scheduler and its step-9 endurance gates remain planned.
Never hide control rate or sensor fidelity changes inside an AV-ratio experiment.

## First useful implementation and open decisions

Create a simulator-independent command contract and unit tests, an Isaac-only
physical-car fixture, and a bounded launcher that saves unique immutable results.
Keep them separate from `scripts/cooperative_lidar_drive.py` and the preserved
Leatherback environment. Expected boundaries are `traffic/` for control contracts,
`scripts/` for the Isaac runner, `experiments/` for configuration/supervision,
`tests/` for CPU tests, and `documentation/` for the verified learning note.

Use installed Isaac/PhysX first, without altering the runtime installation.
Evaluate the installed native PhysX vehicle implementation for an explicit
wheel-torque/brake/steer fixture; verify its compatibility with the installed
Isaac Lab version before committing to large-scale training. An articulated-wheel
alternative remains available if that compatibility fails. The choice must be
documented with actual test results, not inferred from the Leatherback prototype.

Remaining decisions include calibrated chassis/suspension/tire parameters,
actuator units and torque allocation, per-axis coordinate adapters, Lab vehicle
state/reset access, the maximum validated speed, and the initial fallback braking
profile. These are bounded engineering gates, not reasons to add detailed assets
or train a controller before a physical baseline is trustworthy.

Use the [workflow](workflow.md) and [iteration record](templates/iteration.md) to
link each accepted gate to source, configuration, measurements, failures, GitHub
review, and the Google learning-document update.
