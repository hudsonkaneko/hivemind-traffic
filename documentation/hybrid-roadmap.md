# Active roadmap: physical research cars, economical background traffic

Updated: October 25, 2026. Status: physical lane following, the bounded visible
LiDAR-braking subset and a separate one-car Isaac Lab lane task are verified
within their stated fixtures. Broader lifecycle/endurance, shared-map and hybrid
integration remain **in progress**. The first bounded Isaac Lab PPO pilot and its
fixed evaluation are **complete, with failed learned driving**: PPO achieved 0/15
held-out successes against the scripted controller's 15/15. See the
[pilot record](lab-lane-ppo-pilot.md). Further training needs a reviewed experiment.
This is the active development order, replacing the old immediate-next-step
sections in the preserved [implementation roadmap](../experiments/ROADMAP.md)
and [capacity study](scaling-foundations.md). Historical evidence is unchanged.

October 25 sensor milestone: the [input audit](sensor-input-audit.md) identifies
privileged policy inputs and safety-controller shortcuts. A separate forward
RGB/RTX LiDAR and noisy ego-estimation prototype, plus source-authored v08 camera
and LiDAR mounts, is implemented. The final seeded 15-second, 2 m/s scripted
native test passed: 8.52 m travel, a 2.57 m obstacle clearance, no contacts or lane
departures, and braking/recovery during a blank-camera fault. 163 CPU and 18 USD
tests pass. This does not establish generalization, repeated native reset,
moving-hazard performance or sensor-trained policies. See [sensor perception v01](sensor-perception-v01.md).
No policy training or replacement of existing policy observation contracts is
authorized by this sensing milestone.

October 25 SUMO/MAPPO setup: a separate headless four-car fixed-lane environment,
shared local actor, centralized critic and bounded quiet training runner are
implemented. No-learning native preflight passes with unchanged actor weights
and zero optimizer updates. Training awaits user approval of the final config;
merging, jump-in hazards and physical transfer remain future gates. See
[the configuration, measured baselines and limitations](sumo-mappo-setup.md).

October 25 CIRCLES highway preparation: a separate bounded three-physical-car
runner uses CIRCLES for the main car and the frozen 3 m/s PPO lane policy for
two adjacent-lane cars, with a 60 mph ceiling. CPU and OpenUSD checks pass.
The first native attempt failed before driving; its export bug is repaired,
but the user deferred the native retry. No driving, speed or interaction gate
advances. See [the prepared protocol and pending status](highway-circles-60mph.md).

October 22 CIRCLES smoke: seven public ONNX exports actuated the existing
physical car through a longitudinal adapter, with local scripted steering.
Six passed the limited non-contact actuation check; all seven failed the strict
stopping task, including one obstacle collision. This was a low-speed isolated
fixture with privileged state, not full-traffic integration. No training ran.
See [the measured results and limitations](circles-control-smoke.md).

October 22 RSL-RL increment: the user authorized full hazard training and
integration, with subagents and versioned evidence. The isolated 42-feature
clear/static/crossing curriculum passed its three 10-case development gates
after 98,304 PPO transitions and an explicit imitation warm start. Its first
full-scene transfer failed lane containment after 6.659 m; v03 therefore trains
with actual background-car observations. A scripted 180 m continuous drive
and separate two-physical-car actuation/reset/contact foundation passed native
checks. Learned full-scene driving, learned interaction between cars and final
held-out evaluation remain in progress. See
[the versioned integration record](rsl-road-hazard-integration.md); these checks
do not establish sensing, SUMO coupling or highway-speed autonomy.

October 6 environment increment: the separate [four-leaf highway USD draft](highway-usd-v01.md)
now has verified static geometry and a closed navigation graph, plus versioned
screenshots. Stationary contact checks passed; the strict simulator gate remains
failed on a stage-close warning. This does not advance vehicle-driving, shared-map
integration, training, or runtime lifecycle gates below.

October 6 collector iteration: [highway USD v02](highway-usd-v02.md) enlarges
the inner circle, adds a gentle outer collector and implements seeded re-entry
planning. Static geometry/routes and stationary contacts pass; moving-car
handling and the runtime lifecycle gate remain unverified. V01 is preserved.

October 8 speed increment: the separate [35 mph physical profile](physical-35mph.md)
extends the one-car LiDAR bypass road, turn geometry and sensing horizon while
preserving the 3 m/s baseline. Validation is in progress; this does not advance
mixed-traffic, physical-fleet, new-training or real-time GUI acceptance gates.

## Destination and boundaries

October 19 Lab task increment: [the one-car lane task](lab-lane-following.md)
passed 12 native packages covering 16 episodes: 13 successful 60 m drive-and-stop
episodes plus contact, departure and timeout controls. Three fresh-process
nominal repeat pairs passed; bounded signed reset corners, native terminal-state
handling and composed scene exports were checked. This advances the task/reset
plumbing part of step 19. It does not complete the 100 m step-7 protocol, authorize
training, validate cloning or add a physical fleet. The
[training proposal](lab-lane-following-training-proposal.md) was subsequently
approved for one bounded pilot; its [run record](lab-lane-ppo-pilot.md) tracks the
later implementation and measured outcomes without changing this task-validation
evidence.

October 19 validation continuation: [native results and screenshots](showcase-validation-20261019.md)
record three startup passes, a verified remote-admission repair and the first
successful original endpoint and repeat after bounded gap creation. All four
targets and subsequent passing now pass on that source. The [matched display
matrix](showcase-benchmark-20261019.md) completed 11 physical passes and eight
valid comparisons after a reviewed evaluator correction. Two light cases pass
strict timing; all nine dense cases fail. Dense runtime cost and periodic stalls
remain open. This does not advance the research integration or training gates.

October 16 validation increment: [showcase validation](showcase-validation.md)
adds bounded native-window readiness, opt-in endpoint/pause/transition tests,
descriptive traffic statistics and a matched display-cost protocol. CPU checks
are passing; native execution is pending an available GPU. This does not advance
sensor-driving, mixed-traffic, training or strict real-time acceptance gates.

October 15 portability increment: [clean-clone readiness](clean-clone.md) adds
Windows CPU/SUMO CI and an opt-in procedural public car for the loop showcase.
This verifies installation and demo entry points separately from private assets;
it does not advance sensing, mixed-traffic research or learned-control gates.

October 12 interactive increment: [v08/v09 live controls](showcase-live-controls.md)
adds a regular-editor sidebar and a fixed native pool for live traffic density,
speed and following-gap requests. Rendering changes still restart explicitly;
prospective human/AV/V2V/LiDAR controls remain disabled. This exploratory showcase
does not advance learned-control, sensing or matched mixed-traffic research gates.

October 10 responsiveness increment: [v07](showcase-responsiveness.md) adds
an optional small showcase app and shared immutable fleet visuals. The regular
editor is now the v07 default at the user's request, retaining the optimizations;
its fresh 90-second driving check passed physical/scene/process gates at 0.9513x
real time, with strict smoothness still open. Two original minimal-app 30-Hz GUI drives
held average real time and repeated exactly; rare hitches and 60 FPS remain
unqualified. Outcomes are recorded in that evidence note. Physics,
sensing/training boundaries and roadmap order below
remain unchanged; this does not qualify LiDAR or mixed-traffic research.

October 10 showcase performance: [v06](showcase-performance.md) preserves the
80-car presentation with reduced redundant work and separate evidence modes.
Physical/repeat/failure checks passed; strict smooth-playback timing is still
open. This isolated presentation branch does not advance research-fleet,
sensor-driving or training gates and does not replace the roadmap below.

Demo-readiness checkpoint: [SUMO matched-seed checks](demo-sumo-results.md),
[physical-demo profiling](demo-physics-performance.md), and the
[existing Isaac Lab waypoint policy rehearsal](demo-readiness.md) are recorded
separately. These harden existing demos without changing this roadmap's stage
order. They do not close the shared-highway, physical-fleet, or new-training
gates. The temporary presentation checklist is not a new long-term roadmap.

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
prepared detailed vehicle assets and ovrtx remain later integration work.
Basic overview/follow cameras and path overlays now exist in the separate
one-car physical demo. It already runs without SUMO; the two-physical-car target
still needs validation. The scalable hybrid mode need not eliminate SUMO.
The [wheel orientation fix](wheel-orientation.md) adds visual-geometry regression
checks without changing the driver or closing any additional fleet/highway gate.

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
general perturbed-start robustness and lane changes remain unverified. The later
[60 m Lab task](lab-lane-following.md) checks only its declared ±0.15 m/±0.035 rad
reset corners; it does not replace the historical 100 m route gate. The
[visible physical-car fixture](physics-lidar-view.md) advances a bounded subset
of step 8: known-map steering plus RTX-return emergency braking, an explicit
physics/render clock and sensor-loss testing. Overview/follow cameras and planned
path overlays bring forward a small part of step 20 without detailed assets.
The full step-8 60-s sensing/reset gate and steps 9–13 remain open; this is not
LiDAR road perception, general avoidance or a hybrid fleet.

The [adaptive obstacle-path increment](obstacle-bypass.md) adds a separate
single-car scripted bypass fixture: fresh RTX bounds select a left detour and
return, and the cyan preview is that actual driver path. It relies on a known
straight road, privileged odometry, an empty adjacent corridor and explicit
static-object extent priors. Its bounded checks do not close moving-obstacle,
occupied-lane-change, multi-car or real-time gates.

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

The October 19 lane-task matrix now closes the bounded single-environment
scripted task/space/reset/termination prerequisite, with actual terminal metadata
checked through the local SB3 wrapper. There are no optimizer steps, trained
checkpoints or learned-policy outcomes in that milestone. The proposal's
algorithm, reward, observations, training budget and held-out evaluation must be
agreed before proceeding; vectorized cloning remains a separate validation gate.

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

## Preview timing checkpoint (October 6)

The [optional lightweight preview](realtime-preview.md) now exposes 1x-target
pacing and measured lag. Physics and safety regressions pass, but consistent
real-time GUI playback remains **in progress**. The next performance task is a
native window/render trace with matched desktop load, not larger timesteps or
weaker sensor/safety gates. This does not advance physical-fleet, shared-map,
highway-speed or new-training milestones above.

## Scene-authoring requirement (October 6)

All future scene/asset work follows [OpenUSD conventions](../usd/SCENE_STRUCTURE.md).
Before expanding the current physical prototype, migrate its generated hierarchy
to explicit reusable asset boundaries and a versioned path interface. Preserve
the current demo/evidence and verify wheels, controller paths, collision/sensor
relationships and timing after restructuring.

October 8: the [one-car bypass migration](physical-scene-composition.md) now has
referenced assets, a versioned path interface, exact pre-runtime preservation
checks and bounded physical/sensor evidence. Other physical/Lab fixtures retained
their legacy contracts at that checkpoint; historical results are untouched.
Interactive Stage/layer inspection and camera/point controls remained open at
that checkpoint.

October 19: the new Lab lane task uses the same stable composed vehicle paths,
one Lab-owned `/World/Physics/Scene`, reusable asset references and separate
runtime opinions. Actual exported ordinary/contact scenes pass fresh-process
portability and static-hash checks; original screenshots show the native hierarchy
and layer stack. The legacy compatibility probe is preserved separately.
Plain OpenUSD audits through the installed runtime still use a `kit.exe` Python
process and must run outside native process-isolation measurement windows.

For the Lab task, the approved pilot, frozen checkpoint selection and matched
held-out evaluation are complete. Review the observed stationary-policy failure
and action clipping/conflicts before choosing a new longitudinal-action or
warm-start experiment; no extension or second training run is authorized.
Broader scene work still includes live-window timing under matched desktop load
and reuse of the scene/control boundary for a two-car/hybrid fixture. Library
promotion and cloning/reset validation are required before expanding beyond the
tested single-car task. This scene milestone does not advance highway-speed,
physical-fleet or hivemind-effectiveness claims.


October 26: [MAPPO v03 communication/wave setup](sumo-mappo-v3.md) adds explicit delivered messages and a reproducible SUMO braking pulse. 124 targeted tests and no-learning native preflight pass; policy training awaits reviewed configuration. This advances SUMO communication plumbing only, not physical V2V, merging, shorter headways or learned wave suppression.

### 2026-10-27 checkpoint: frozen v02 in 3D

Completed the bounded recorded-motion gate for v02: four actors, 60 seconds, seed 410001, exact original metrics and visible vehicles in Isaac captures. Evidence: [MAPPO v02 3D validation](mappo-v02-3d.md). Physical closed-loop transfer remains open; no training was started.

### 2026-10-27 checkpoint: prepared-car replay sensing

The v14 main-car sensor replay gate passed for 60 seconds: seven cameras, five LiDARs, 120 saved capture sets. Data and performance are retained in [sensor evidence](mappo-v02-sensors.md). Measured LiDAR lag is 66.7 ms; moving-object deskew and physical closed-loop control remain open.

### 2026-10-29 checkpoint: v04 passing/cooperative-following setup

[MAPPO v04](sumo-mappo-v4.md) now supports acceleration plus masked lane requests, guarded short-headway eligibility and explicit passing metrics. Native no-learning setup gates pass; the pilot config is ready for review. Learned passing/cooperation, achieved short highway gaps, staged curriculum and physical policy transfer remain unverified. The stopping-distance floor dominates the 0.8-second target at highway speeds.

## October 29: v04r1 SUMO passing refinement

Imitation-assisted MAPPO now passes slower cars in all 50 fresh held-out episodes (82 passes), with 4.9583% aggregate safety interventions and zero collisions/overspeed/required-gap deficits. One episode exceeds the lane-change-frequency limit, so full v04 acceptance remains open and accepted v02 is preserved. Training used the declared one-million-transition budget without runtime errors or agent monitoring. Checkpoints, per-seed metrics and optimization curves are retained; see [v04r1 evidence](sumo-mappo-v4-refinement.md). Next behavior gate is unnecessary lane switching. No new 3D, physical-steering or causal communication benefit is established.

## October 29: v04r2 passes declared SUMO checks

The selected v04r2 neural actor passes all acceptance gates on 50 fresh scenarios: 74 passes, at least one per episode; 3.175% aggregate overrides; maximum two lane changes/car/minute; zero collisions, overspeed or required-gap deficits. Use the frozen policy/best checkpoint at update 175; the final checkpoint regressed. The declared one-million-transition run completed without runtime errors or monitoring, and all evidence is retained in [v04r2 results](sumo-mappo-v4-stability.md). Existing v02 integrations remain intact; physical steering, sensor inputs, 3D transfer and causal communication benefits remain separate gates.


## October 30 sensor milestone — semantic mask/LiDAR fusion v01

Verified calibrated front-camera semantic projection with all five RTX scanners:
static then moving 2/10 m/s fixtures pass, with per-actor support and retained
metrics/screenshots. See [fusion v01 evidence](semantic-lidar-fusion-v01.md).
This adds an evaluated sensor-data product; MAPPO still uses its existing
observation contract. Learned segmentation, ego-motion compensation, robust
visibility, lane-marking/pedestrian validation and policy observation adaptation
remain open. No training was started.


## October 30 perception preview — frozen pretrained RGB segmentation

A pinned Cityscapes SegFormer B0 now produces real predictions on the existing
24 captures, shown alongside RGB, GT and errors. Scores and screenshots are in
[the preview evidence](pretrained-segmentation-preview-v01.md). Road spillover,
oversized vehicle masks and unsupported lane/generic-obstacle classes remain
visible limitations. This is offline inference, not accepted driving perception.
Predicted-mask-to-LiDAR evaluation, more varied saved-scene comparisons and
policy adaptation remain separate work; no training or simulator run was started.


## October 31: Frozen perception refinement on saved captures

[Refinement v02](segmentation-refinement-preview-v02.md) compares three input
transforms and adds boundary metrics plus measured sparse occupancy. The 1024
variant has 86.06% shared mIoU; geometric generic-obstacle point recall is 89.28%
among eligible returns, while learned generic-obstacle semantics remain zero.
This is exploratory sensing evidence, not policy integration or driving
acceptance. A disabled [training proposal](segmentation-refinement-training-proposal.md)
requires varied data, a trainer and CUDA preflight before authorization.
No new training or simulator runs occurred.

## 2026-10-31 checkpoint — Segmentation training preparation v02

Dataset/trainer/runtime preparation is verified: 3,600 six-class captures, 180 scene
groups, reviewed split/hash checks, 139 CPU tests and a zero-update CUDA preflight.
[Final configuration and evidence](segmentation-training-preparation-v02.md) retain
training disabled. Next gate is user review and explicit supervised-training launch;
then validation-only checkpoint selection and frozen held-out evaluation. No trained
six-class quality, new-corpus LiDAR fusion or driving-policy integration is claimed.
Instance/distance/temporal/calibration gates and broader scene diversity remain open.

## 2026-11-02 checkpoint — General highway curriculum, including I-110

[New graph-policy preparation](highway-curriculum-v01.md) now covers cruise, dense
following, passing and merges across 2–5 through-lane training roads. Six-lane
and changed-speed/geometry conditions remain held out. Native no-learning
preflight and 129 CPU tests pass; high scripted dense-traffic intervention rates
are retained as a limitation. The new 81-field ABI is not compatible with v04r2
weights. Next gate is final-config review and the bounded MAPPO pilot, followed
by per-task validation and untouched transfer tests. No learned adaptation,
physical steering, sensor-policy integration or arbitrary-road competence is claimed.


## November 2 follow-up: completed highway curriculum pilot

The [first broad highway pilot](highway-curriculum-training-v01.md) completed its
approved 500k-transition budget (499,998 usable joint transitions). It is not an
accepted controller: final validation passes the cruise and passing targets,
but merges are 0/6 and interventions are 96.44%. Ranking/recovery fixes are tested;
the final diagnostic export and original selected actor are both preserved.
No held-out seeds, new training budget or physical transfer were used. Next work
is a reviewed acceleration-tracking/merge-learning revision with training-only
demonstrations, retained guards and the same validation gates. Accepted v04r2
remains separate.


## November 2: living plan for sensor-driven training

The [persistent three-stage roadmap](sensor-driven-training-roadmap.md) now tracks
structured highway competence, robustness to imperfect observations and eventual
camera/LiDAR-driven physical control. AGENTS.md requires updates after relevant
progress and failures. The next gate is Stage 1 diagnostics/merge observations/
teacher preparation, followed by final-config review. All new learning budgets
are proposals; no training or simulator run started for this planning milestone.
The overall project roadmap remains this file.


## November 2, 2026: sensor-driven roadmap Stage 1 preparation

[Structured highway preparation v02](highway-stage1-preparation-v02.md) adds the
93-field merge observation contract, feasible teacher, intervention diagnostics,
BC/MAPPO runner and corrected right-entry generated geometry. Historical failures
remain evidence. Preparation is distinct from learned driving acceptance: no new
training has started. The [living sensor-driven plan](sensor-driven-training-roadmap.md)
tracks exact results and the final configuration review before a launch.


## November 3, 2026: Stage 1 learning result

The [authorized Stage 1 run](highway-stage1-training-v02.md) completed 499,992
transitions in 59m24s. The final actor merges and passes in validation, but the
18.15% pooled override rate fails driving acceptance. No held-out test or sensor/
physical transfer was run. The living plan now prioritizes intervention diagnosis
and a reviewed revision before a second attempt; all metrics and weights are retained.


## November 3, 2026: Stage 1 intervention revision prepared

[Stage 1 v03 preparation](highway-stage1-v03-preparation.md) quantifies the
acceleration-fitting gap, validates the teacher on the same roads and adds
intervention-aware checkpoint ranking. Teacher conservatism remains a measured
speed/throughput tradeoff. The new BC/teacher-retention configuration is disabled
pending review; Stage 1 driving remains unaccepted and sensor uncertainty is next
only after that gate. The published policy catalog remains frozen.


## November 3, 2026: Stage 1 v03 learning result

The [approved v03 attempt](highway-stage1-training-v03.md) completed after one
checkpoint-file access failure and recovery. Selected validation retains 6/6
merges and 12 passes, with 14.97% pooled overrides and five failing cases.
Per-case regressions and lower-intervention intermediate candidates remain in
the evidence. Stage 1 is still unaccepted; diagnose following/passing errors
before proposing more training or advancing to sensor uncertainty. The new
[frozen catalog version](../policies/versions/highway-stage1-v03/README.md) is
experimental and preserves prior packages.


## November 4, 2026: Stage 1 v04 training plan

The [new training plan](highway-stage1-v04-training-plan.md) starts with saved-data
diagnosis, then proposes a paired comparison of current-only imitation and
retained acceleration corrections. Each arm is capped at 100k new training
decisions or 30 minutes, with a shared frozen warm start and fixed evaluation.
Screening progress is separate from nominal acceptance. Stage 1 remains open;
implementation, final configuration review and launch authorization are next.


## November 4, 2026 — population adaptation pilot

The user requested training for variable autonomous-car allocation. [Population v01](highway-population-training-v01.md) now has tested reset-based cohorts, a shared 93-input actor, masked critic and bounded 100k/30-minute configuration. No live slider handoff or arbitrary fleet-size capability is claimed. The separate v04 correction-memory comparison remains planned. Next action is the authorized pilot and retained per-population comparison.


## November 4, 2026 — variable-population result

The [population pilot](highway-population-training-v01.md) completed 99,999 decisions but regressed from 20.64% to 31.70% pooled interventions. It remains unaccepted despite 4/4 controlled merge targets and 14 passes. Save this candidate as evidence, preserve older policies and diagnose lane-end emergency warnings plus acceleration corrections before another proposal. Actual training covered 1/2/4/6/8 AVs, total cohorts 8/16/24 and 0–6 received peers; this does not establish live slider adaptation.


### November 4: population v02 retraining completed

[Population v02](highway-population-training-v02.md): 99,998 decisions in 514.516 seconds. Pooled overrides 29.97%, worst 86.75%; twelve failing cases. Rejected for promotion. Metrics, native warnings and experimental weights retained. Next: diagnose acceleration errors before another revision. Google guide sync pending.


## November 4: population analysis checkpoint

The [AV comparison](policy-baseline-av-comparison.md) restores the phase-0 reference and prioritizes action-interface calibration and objective diagnostics before more learning. Population v02 remains rejected. Follow the living sensor-driven roadmap; no new learning or physical-transfer capability.


## November 4: frozen control and loss diagnosis completed

[Steps 1 and 2](population-control-diagnostics-v01.md) now have training-only acceleration calibration, identical-observation loss gradients, retained metrics/plots and independent review. No timing/duration bug was found; ordinary native following saturation, near-limit/merge response gaps, target disagreement and shared clipping need deliberate correction. Twelve historical native lane-end emergencies were missed by sampled counters. Keep the phase-0 parent frozen and Stage 1 unaccepted. Next: target consistency and post-warm-up actor/critic telemetry, followed by a reviewed configuration before any new training. The Google guide and Markdown mirror are synchronized.


## November 4: population v03 control setup ready for configuration review

[Population v03](population-control-v03.md) implements near-limit/first-entry corrections, a shared teacher/reward target, separate actor/critic diagnostics and native emergency accounting. Bounded teacher passing/merge checks reach zero overrides, while the unchanged actor remains unaccepted at 11.17% merge overrides. 150 tests and a zero-update final preflight passed; the 100k-decision/30-minute configuration is prepared with training disabled. Review the configuration before a new launch. No held-out qualification, arbitrary fleet-size guarantee or sensor/physical transfer follows from these development checks.

## November 5, 2026: physical policy execution calibrated

The [frozen MAPPO adapter](mappo-physical-adapter-v01.md) now translates acceleration/lane intent into physical wheel controls. Ordinary actuator calibration and scripted 6/12/18 m/s lane changes/stopping pass. The unchanged actor cruises and passes a scripted car but fails intervention gates (82.11%/27.76%, speed-ceiling corrections). Preserve both attempts and the frozen weights; no training or policy promotion occurred. The living sensor roadmap records the remaining near-limit, curved-road, reactive-traffic and sensor-transfer gates. Physical truth-based execution is demonstrated only in this bounded straight-road surrogate fixture.


## November 5, 2026: near-limit learning correction ready for review

[Population v04](population-near-limit-v04.md) adds a localized raw-acceleration objective, four episode speed ceilings and eight lower-limit validation cases. Frozen diagnosis confirms meaningful at-limit acceleration errors and previous-request sensitivity. 171 tests and a zero-update SUMO preflight pass, with the teacher-fixture background emergency retained. Review the disabled 100k-decision/30-minute configuration before learning. Stage 1 and physical transfer remain unaccepted; no policy weights changed or training launched.


## November 5, 2026 — I-110 merge visualization checkpoint

[Final Stage 1 v03 replay](i110-merge-visual-v01.md) completes two ramp merges in a predeclared I-110 test and provides an 11-second merge-focused 3D video with actual screenshots. Final v03 was selected for merge-specific performance; the phase-0 population reference remains separate. Full-run interventions are still 10.56%, above the 5% target, despite no recorded collisions, teleports, overspeed or native emergency warnings. SUMO owns motion on provisional reconstructed road geometry; this is not physical or sensor-driven validation. No learning or checkpoint changes. The [living sensor plan v13](sensor-driven-training-roadmap.md) preserves review of the disabled population v04 setup as the next training action.


## November 5, 2026 — environment v05 milestone

[Continuous-maneuver environment v05](highway-environment-v05.md) implements the next structured-driving step: continuous native lane changes, observed overlapping traffic, stable-merge/full-route metrics, independent AV participation and traffic density, varied ramp geometry and competence-based task mixtures. 256 targeted checks and final source-matched SUMO readiness pass. The I-110 scripted baseline retains its 5.34% intervention failure; no driving acceptance or physical/sensor transfer is implied. The user-authorized 250k-decision/one-hour pilot is queued after saving this checkpoint. Track results and next actions in the [living sensor-driven roadmap](sensor-driven-training-roadmap.md).


## November 5, 2026 — v05 learned candidate remains unaccepted

The [v05 pilot](highway-environment-v05-results.md) completed 249,999 active-agent decisions within the original budget, retaining both failures and consumed time. Stable merging improves 2/9→8/9 but intervention dependence worsens 13.37%→19.83%; I-110 remains unmerged. The separately versioned 99-field actor is rejected, not a replacement for accepted historical fixtures. [Living roadmap v15](sensor-driven-training-roadmap.md) now calls for a train-only native-response/entry diagnostic before another reviewed training configuration. No held-out or physical/sensor acceptance is claimed.


## November 6, 2026 checkpoint: I-110 entry blockage corrected

The [matched frozen-policy diagnostic](i110-merge-blocking-v01.md) isolates an overbroad junction conflict matrix: the ramp yielded to four non-crossing through lanes. A versioned lane-specific rebuild retains the actual auxiliary-lane yield and unchanged geometry/demand/guards. The same v05 actor now completes a stable merge, with zero focal stopped samples and all eight controlled routes finished. Fleet interventions improve 30.53% to 8.52% but still fail 5%; the original rejected 24-case results remain intact.

The SimReady main-car visual rerun is complete, with full 80-second metrics and actual screenshots retained. No training or physical/sensor qualification occurred. [Living roadmap v16](sensor-driven-training-roadmap.md) prioritizes remaining acceleration-realization differences on training fixtures before a separately reviewed configuration. Google guide and Markdown mirror are synchronized from verified full readback.


## November 6, 2026 checkpoint: control mismatch and smoothness proposal

The [completed control audit](highway-smooth-control-v01.md) attributes the corrected-I-110 case's remaining 244 interventions to ordinary following and entry responses. Current MAPPO remains at 8.52%, above the unchanged 5% gate. A separate training-only reference comparison shows zero interventions from the existing teacher but a 65 m/s³ peak jerk; neither that teacher nor the frozen actor demonstrates the desired smooth motion.

A fresh SimReady main-car replay and full metrics are saved. [Living roadmap v17](sensor-driven-training-roadmap.md) now prioritizes the [disabled smooth-reference and reward proposal](highway-smooth-training-plan-v01.md): prepare and test anticipatory entry and recovery, then present the resolved configuration before explicit training approval. The proposed matched pilot is bounded at 100k total decisions/one hour. No learning, policy acceptance, native safety changes or physical/sensor qualification occurred.


## November 6, 2026 — Smooth-control preparation v01

The [implementation and six-fixture results](highway-smooth-preparation-v01.md) complete preparation code and native/visual testing. The retained scripted reference meets every original safety/task/5% intervention check, including a continuous I-110 merge, but four strict comfort gates remain failed. The frozen learned actor is unchanged. No optimizer steps, held-out promotion or training launch occurred.

The [living sensor-driven roadmap](sensor-driven-training-roadmap.md) is now v18. Next: diagnose the remaining native-response peaks and establish a reference that meets comfort and progress gates before reviewing/authorizing the disabled matched 100k-decision MAPPO pilot. Metrics, failed revisions and two SimReady replays are retained; selected screenshots and compact evidence are in Git. The Google engineering guide and Markdown mirror were updated together.


## November 6, 2026 — Audit replaces the next training recommendation

[Highway training rethink v01](highway-training-rethink-v01.md) explains why fewer overrides and an earlier focal merge did not improve the scripted traffic: early low-speed travel, an unnecessary ending-lane pass, and delayed braking after leader/occupancy changes. Seven actor/checkpoint identities are intact; narrow curriculum exposure and misaligned references matter more than a blanket weight-corruption theory.

[Living roadmap v19](sensor-driven-training-roadmap.md) now prioritizes a reviewed zero-learning roadway/reference diagnostic before a new training contract. Preserve all historical failed results and the disabled smooth bundle. The new proposal separates longitudinal learning, route/merge behavior and communication/population robustness, with every car's progress and full-cohort delay checked. Only documentation and saved-file analysis were produced; no new simulation or training.


## November 6 checkpoint: flow preparation tested, training not ready

[Flow preparation v01](highway-flow-preparation-v01.md) implements the new local/map contract, route-aware reference and one-factor paired MAPPO setup. The full diagnostic allocation retains failed attempts, compatibility checks, all-car outcomes and four SimReady replays. Low overrides do not qualify the reference: nominal comfort/task gates and the main-car flow limitation remain unresolved. The [living sensor roadmap](sensor-driven-training-roadmap.md) is v20; no training or policy promotion occurred. Requalify the target before initializer selection and final launch approval.


### Flow approach follow-up — November 6, 2026

[Two focused approach ablations](highway-flow-preparation-v01.md#focused-approach-ablation--november-6-2026) were rejected and baseline restored. Main-car slot alignment remains the next prerequisite; training remains disabled. Metrics and failed source snapshots are preserved.


### GPUDrive pretrained transfer — November 6, 2026

[One frozen I-110 transfer test](gpudrive-i110-v01.md) failed road-boundary/speed criteria; retained as external-policy evidence, not an accepted replacement. CPU inference and a SimReady replay are verified. No training; map/goal transfer and speed enforcement remain unresolved.


### GPUDrive native-safety follow-up — November 6, 2026

[Guarded v02 evaluation](gpudrive-i110-v02.md) applies60mph/lower-road limits and the existing native safety stack. Zero recorded collisions/overspeed, but failed merge/progress and intervention checks. Existing accepted policies unchanged; no training.


### 2026-11-06 — bounded continuous GPUDrive repair

[GPUDrive v03](gpudrive-i110-v03.md) fixes the steering-to-lane reinterpretation and retains explicit limits; its 1.1-second diagnostic still ends at an unsupported route association, not a completed merge. 23 tests plus 4 subtests pass. Metrics and initial failure preserved; no training or rendering. SUMO-native MAPPO remains the primary path. See living sensor roadmap v24; guide sync pending.


### 2026-11-06 — continuous route repair validated

[GPUDrive v04](gpudrive-i110-v04.md) removes the route-mapping blocker while keeping the fixed destination, continuous steering and safety limits. The short CPU run merges and reaches the goal in 28.5 simulated seconds; acceleration interventions and jerk still fail qualification. Tests and full metrics retained. No training, rendering or policy promotion. See living roadmap v25; incremental guide sync pending.


### 2026-11-07 — early crossing found; longitudinal comfort improved

[GPUDrive v05](gpudrive-i110-v05.md) reduces scalar longitudinal jerk with a transparent external filter while retaining all final guards. One short CPU run and saved baseline metrics show continued arrival but an unconnected early ramp exit in both versions. Mainline occupancy was insufficient to certify proper merging. Raw actions are still changed99.29%; no acceptance/promotion/training. Forty tests plus 16 subtests pass. See living roadmap v26; next transfer work is merge-corridor/steering validation. Guide sync pending.


### 2026-11-07 — MAPPO plan from GPUDrive findings

[The saved plan](mappo-gpudrive-lessons-plan-v01.md) transfers merge validity audits, transparent control-stage metrics and anticipatory smooth gap alignment to flow117/SUMO MAPPO. Reference qualification remains blocked; training stays disabled and no initializer is selected. Proposed cheap checks precede existing full gates and the paired pilot within its reconciled budget. No runtime changes or runs this milestone. See living roadmap v27; guide sync pending.


### 2026-11-08 — Teacher-free MAPPO learning and bounded restart completed

[Flow RL v01](highway-flow-rl-v01.md) is the first trained flow117 experimental actor, with native SUMO motion and no teacher losses/alignment reward. Two sequential rounds with terminal review reduce four-case pooled interventions to 7.61%, including a stable I-110 merge at 0.67%; comfort and broader acceptance remain open. Original teacher-assisted qualification failures are preserved. [Actor/contract](../policies/versions/highway-flow-rl-v01/README.md) published without promotion; living roadmap v28 records next actions. No third run, held-out test or physical validation. Guide incremental sync pending.


## November 8 - Recorded flow117 SimReady merge replay

[Final pilot video](highway-flow-rl-v01.md#november-8-frozen-policy-merge-video) shows the round-two actor on its matched I-110 development case, with SimReady main car and lightweight traffic. Exact metric reproduction, 25 seconds at normal speed, verified video/poses/assets. Visualization only; no training, sensors, physical validation or acceptance promotion.


## November 8 - MAPPO velocity diagnosis

[Offline speed and command analysis](highway-flow-rl-velocity-analysis.md) separates native entry braking from learned speed/route problems. Next: isolate minor-junction behavior, then ending-lane detours and cruise behavior, retaining full-route/per-car metrics. Saved CSV and charts; no new simulation, training or promotion.


## November 8 - Next flow117 improvement plan

[Flow117 v02 proposal](highway-flow-rl-v02-plan.md) separates native junction modeling from learned route/speed behavior. Keep collision/right-of-way protection, test counterexamples and whole-route performance, then present a resolved configuration before learning. Planning only; existing results and actors preserved.


## November 8 - Flow117 v02 prepared for review

[Implementation and configuration](highway-flow-rl-v02.md) separates native visibility behavior from learned speed/route choices. Zero-update runtime checks and 158 tests pass; the frozen policy remains unaccepted and its full-route replay exposes the ending-lane stop. New cruise and detour rewards are ready for a bounded 30,000-decision / 900-second pilot after configuration approval. No training or held-out evaluation occurred; metrics and failures are preserved.


## November 8 - Adjacent native traffic comparison

[Saved-replay comparison](highway-flow-rl-v02.md#november-8---speed-comparison-with-adjacent-native-traffic) shows the frozen MAPPO car still has speed oscillations while its native neighbor accelerates steadily. Metrics and unsmoothed plots are retained; different starts and route duties are explicit. No new simulation or learning. Next action remains review of the v02 training configuration.


## November 8 - Reward revision plan after native comparison

[Proposed reward revision](highway-flow-rl-reward-revision-plan.md) replaces the ineffective-in-dip cruise addition with a context-valid speed objective, reshapes comfort and strengthens the optional-detour penalty. Saved-state audit and target qualification come before final configuration or training. Research informs the design; candidate weights are not validated. Existing safety gates, policy ABI and study budget are preserved. No new simulation or learning.
October 26: [seven-camera v09 source](seven-camera-v01.md) passed bounded native
capture, projected-target visibility, composition and moving-mount checks. This
is a camera-rig milestone only; learned surround perception, Cosmos, sensor
fusion and camera-policy training remain open. No new training was launched.

October 26: [v10 roof LiDAR](roof-lidar-v01.md) passed combined capture with all
seven cameras and tall targets around the car. Small side targets at 2–6 m were
missed; near-field sensing remains open. No new training or actor change.
