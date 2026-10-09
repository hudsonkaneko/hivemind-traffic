# Loop traffic showcase — implementation and evidence

Date: October 9, 2026. Status: in progress. Branch: `codex/loop-traffic-showcase`.
This presentation branch does not replace the research/hybrid roadmap.

## Objective and boundaries

Use the existing circular highway, one user-supplied prepared physics vehicle,
and slower block/wheel cars. The main car should visibly pass moving traffic
using steering and propulsion/braking. No runtime main-car pose teleportation.
Background traffic uses deterministic kinematic targets, not calibrated human
drivers; it does not yield or react to the main car. All original assets and
earlier results are preserved. No Lab installation or training is required.

The selected local vehicle entry is
`vehicles/sim_ready/americano-i7-ev/v07/world.usda`. Its redistribution rights
are unverified; assets and per-run copies stay local. A clone needs a compatible,
locally provided prepared vehicle. Do not mistake a generic launcher on GitHub
for distribution of the vehicle.

## Ownership and timing

| Concern | Owner / contract |
| --- | --- |
| Main-car motion | Isaac PhysX; controller applies steering radians and normalized throttle/brake through wheel torques |
| Slower-car motion | Scripted kinematic bodies, fixed-speed CCW lanes, real colliders |
| Decisions and path | Scripted adjacent-lane selection and smooth radial transition, known circular road |
| Observations | Privileged simulator chassis/object positions with stable IDs and integer physics ticks; **not sensor-derived** |
| Rates | Physics 120 Hz, low-level control 60 Hz, gap decisions 10 Hz, view 20 Hz |
| Display | Session/dynamic USD opinions, referenced immutable assets, Z up/metres/+X car forward |
| Training | Postponed; no policy competence, sensor-driving or coordination claim |

Kinematic targets use the installed native tensor API, isolated to background
bodies; they must not share a body view with the dynamic main car. See
[NVIDIA's kinematic-target API](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/108.0/extensions/runtime/source/omni.physics.tensors/docs/api/python.html).

## Predeclared acceptance

- Every owned tick advances exactly once; render-only calls advance zero ticks.
- No chassis rigid contacts, no planar body overlap, at least **0.6 m** minimum
  measured ego-to-obstacle body clearance. Full main-car footprint remains in
  the four main lanes; upright Z at least 0.98 and all wheels supported.
- Finite physical states/commands, observed maximum speed at most 17 m/s;
  command expiry produces zero throttle/full braking within its 12-tick TTL.
- Background native target error at most 0.02 m; stable IDs/count/frames;
  source and saved static package files unchanged during stepping.
- v01: 30-second 6 m/s integration drive and brake/hold.
- v02: 90-second passing drive, at least 3 distinct fully cleared vehicles and
  2 completed lane changes. Count a pass only when the ego rear has cleared
  the peer front by 2 m, not merely when center positions cross.
- Explicit final braking: reach <=0.05 m/s, hold >=5 s with <=0.05 m drift;
  stopping XY distance <= v_initial^2/(2*3) + 0.1*v_initial + 2 m.
- Blocked-lane fixture: no unsafe escape, sustained following at traffic speed,
  no contact/overlap. Separate intentional-contact positive control must detect
  the background collider; this fixture is expected to collide, not a demo.
- Repeat: same configuration and captured inputs, aligned position/speed
  difference <=0.02 m / 0.02 m/s. This is scoped fresh-process repeatability.
- GUI soft real time: measured RTF 0.99–1.01, p95 lag <=0.10 s and maximum lag
  <=0.50 s, excluding startup and explicit pauses; report failures honestly.
- Actual screenshots must show the requested car, wheel orientation, traffic,
  and useful follow/path framing. USD pose checks alone are insufficient.

## Progress record

- Plan presented before implementation; three agents assigned disjoint vehicle,
  fleet and controller work. Main agent owns integration, evidence and docs.
- Inspected the supplied v07: composed asset with moving wheel fixtures and
  prior bounded low-speed validation. Its evidence does not qualify this new
  higher-speed traffic experiment; affected checks are rerun here.
- v01 integration passed in `20261009T173729Z-dce58c1e`: supplied physical car,
  twelve measured moving backgrounds, 6 m/s drive, 2.6553 m stop and 9.125 s
  stationary hold; all contact, support, road, command and source-preservation
  gates passed. Actual image confirms the requested detailed car.
- Preserved failed attempt `20261009T173505Z-e3db9ad7`: ancestor USD copying
  did not produce the typed scene-addition notice needed by the installed
  manager. Explicitly creating that scene before copying fixed registration;
  a USD notice regression covers it. No vendor runtime edits were made.
- v01 unpaced rendering was 0.878x including a 7 s first-frame stall. Physical
  success is distinct from real-time success. Renderer warm-up and live timing
  will be measured separately for the presentation stage.
- v02 physical passing passed in `20261009T173945Z-0a63009b`: 7 distinct passes,
  2 completed lane changes, minimum measured body clearance 1.6594 m, no contact,
  intact road/support/command/source gates. 102 simulated seconds in 98.9794 s
  live window, unpaced. Four render-only preparation frames now precede that
  window and are reported separately; they add no physics ticks.

## Commands and evidence

Pending implementation and measured runs. Do not treat proposed commands as
verified. Each attempt gets a fresh directory under `outputs/loop_showcase/`.
Version folders under `showcases/loop_traffic/` record meaningful progressions;
source versioning remains Git, not duplicated project copies.

## Learning document synchronization

Repository record is authoritative. Google guide sync is pending: the preceding
attempt was blocked by the connector's Windows `workspaceRoot` trusted-read
error. No cloud update is claimed for this milestone.
