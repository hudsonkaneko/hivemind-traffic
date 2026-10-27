# Highway disruptions as a separate scenario layer

27 October 2026 · Sam-requested architecture; specification, not runtime implementation.

Later same-day implementation: [MLK–Vernon build](i110-mlk-vernon-build.md)
now provides deterministic event schedules, independent USD event assets and
session-only collision/visibility/pose toggles, with a scripted inspection-car
response. Live SUMO and native physical-driving integration remain pending.

The base road owns permanent lanes, ramps, grade, signs and normal access rules.
Normal traffic owns demand and calibrated driver behavior. An optional versioned
event manifest owns temporary disruptions. A no-event run uses the same road and
traffic configuration. Recurrent bottlenecks caused by permanent merges or demand
are measured outcomes of that baseline, not incidents that must be inserted.

| Event | Required traffic effect | Required scene/physical effect |
|---|---|---|
| Crash/disabled vehicle | Occupied road space, affected lanes, incident start/clearance, measured or synthetic speed/route response | Stable-ID stopped/damaged actor and responders; collision geometry where physics owns motion |
| Construction | Scheduled lane/ramp closures, taper, temporary speed/access restrictions, reopening | Referenced cones/barriers/signs and work-zone boundaries; visual and collider agreement |
| Police traffic break | Moving controlled obstruction across specified lanes; following/no-passing response and gradual release | Patrol actor with prescribed path, speed and lights; sweeps remain inside the specified road geometry |
| Debris/temporary obstruction | Local occupancy and avoidance/stop response | Visible and collidable object with defined dimensions and lifetime |

CHP's [incident glossary](https://cad.chp.ca.gov/htm.net/glossary.htm) identifies
officer-led traffic breaks. The [CHP-hosted driver handbook](https://www.chp.ca.gov/siteassets/images/cadriverhandbook.pdf)
describes slowing while moving across lanes in a serpentine pattern. This
supports the event category, not an inferred speed/path/duration distribution.
Use a declared synthetic path until observed parameters are available.

## One event manifest, explicit backend adapters

The [intake template](templates/highway-event-intake.json) records ID, type,
provenance, source timestamps, uncertainty, coordinate frame, stable lane/road
IDs, footprint, onset/end/clearance stages, affected actors, speed/path or closure
schedule, seed and source/config hashes. Null fields are unresolved and make an
event non-executable. Source truth must be labeled `observed`, `reconstructed`
or `synthetic`; a live report is not automatically a calibrated reconstruction.

Use separate adapters for SUMO and OpenUSD/Isaac. SUMO must receive closures,
stops or moving-obstruction behavior; a USD overlay alone cannot change traffic
flow. PhysX must receive compatible collision/actor control in the bounded
execution case. Each car has one motion owner. Do not teleport a physical vehicle
or have SUMO and PhysX simultaneously write its transform.

Compose optional `events/<event-id>.usda` opinions above base environment assets,
with a stronger runtime/session layer for transient poses/state. Props live under
`/World/Environment/Props/Events/<EventId>`; vehicles retain stable identities
under `/World/Vehicles`. Event provenance maps to those prims without reparenting
cars. Preserve the [scene conventions](../usd/SCENE_STRUCTURE.md). Collision and
sensor-relevant content stays loaded during a run. Reset clears runtime event
state and restores lane permissions without modifying source road assets.

One scenario toggle must enable/disable the event across behavior, rendering,
collision and sensors together. Reject disagreement. Deterministic scheduling
uses simulation time and a seeded generator, not wall-clock polling. Overlapping
closures compose explicitly; clearing one event must not reopen a lane still
closed by another. Record requested/effective actions and safety overrides.

Keep future event times/hidden incident metadata out of a local driving actor.
Only its allowed sensor/map inputs and legitimate delivered messages may inform
decisions. Evaluators can retain full event truth. A controller-caused collision
is an endogenous outcome and must be logged as such, not rewritten to match a
pre-scripted crash schedule.

## Sources and experiments

[Caltrans LCS](https://cwwp2.dot.ca.gov/documentation/lcs/lcs.htm) publishes current
and next-seven-day planned closures every five minutes, including District 7
JSON. [QuickMap](https://quickmap.dot.ca.gov/faq) distinguishes incidents and lane
closures. These are source leads for timestamped intake; no feed adapter or
automatic historical archive was built here. Planned closure times do not prove
actual setup/clearance, and current feeds do not establish past event histories.
Review publisher terms and archive provenance before reuse.

Start with baseline/no-event, fixed disabled-car obstruction, planned lane
closure, then moving traffic break. Use identical exogenous event schedules,
demand, seeds and driver/safety versions for paired controller comparisons.
Evaluate delay, discharge/recovery, queue length, collisions, near misses,
interventions, human/ramp burden and unfinished trips, including vehicles queued
outside the rendered tile. Separate observational calibration from synthetic
stress tests and report uncertainty in incident severity/clearance time.

Required implementation checks: event-off equivalence, repeat/reset determinism,
closure overlap/reopening, moving obstruction containment, consistent collision
and sensor effects, no hidden metadata leakage, and boundary-complete accounting.
These are future acceptance checks, not tests run during this documentation review.
