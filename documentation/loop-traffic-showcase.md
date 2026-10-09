# Loop traffic showcase — implementation and evidence

Date: October 9, 2026. Status: verified within the bounded v04 showcase scope.
Branch: `codex/loop-traffic-showcase`.
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

## How to explain the demonstration

1. **Observe:** read actual car positions and speeds from PhysX with a stable
   vehicle ID and physics tick. This is ground-truth object tracking, not LiDAR
   perception; the controller already knows the road's circular geometry.
2. **Decide:** compare the slower lead vehicle and adjacent-lane traffic. Check
   predicted gaps throughout a candidate lane change, not just at its endpoint.
   If no adjacent lane is clear, follow at a reduced speed instead of forcing a pass.
3. **Plan:** freeze a smooth radial transition over 80 m. The cyan line displays
   this actual driving reference; it is not a separate decorative animation.
4. **Control:** a rear-axle pure-pursuit controller requests steering; speed error
   produces throttle/brake requests. The command gate limits steering changes
   and brakes on expired commands. PhysX determines the main car's movement.
5. **Present and verify:** measured obstacle poses drive separate display proxies.
   Contacts, clearance, support, timestamps and visible/native pose agreement
   are checked independently of the controller's own success counters.

The twelve slower cars travel roughly 15–19 mph; the main car requests 35 mph
when safe. It need not weave continuously: staying in a clear lane is an allowed,
safer result. This demonstrates scripted motion planning/control and integration,
not learned driving, sensor competence, communicating AVs or traffic optimization.

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
- v03 safety probes passed: a deliberate collision in
  `20261009T174337Z-1dc5b510` produced native contact reports; this is an expected
  failure-injection fixture, not a safe-driving demonstration. Four-lane blocked
  traffic in `20261009T174407Z-bee78c8a` produced zero lane changes, following
  at traffic speed and minimum clearance 10.4050 m. Command loss in
  `20261009T174525Z-33fc5235` triggered gate braking and stopped safely.
- Background wheel visual writes were reduced from 120 to 20 Hz while every
  native kinematic collision target still updates at 120 Hz. Cached USD handles
  are released at shutdown; no stale state cache or skipped physics ticks.
  Follow framing now retains the whole main car and nearby traffic, with a
  separate traffic-overhead view, whole-loop overview and a path-visibility toggle.
- The final scene uses readable `vehicle_background_000` names, avoiding a
  duplicate background prefix. Identity mapping is explicit; prior run scenes
  remain unchanged. New composed scenes use the same referenced collision asset.

## Commands and evidence

From the repository root in the configured traffic environment:

```text
python scripts/demo_loop_showcase.py --check
python scripts/demo_loop_showcase.py --version v04
```

The launcher discovers Isaac. Allow roughly 12 seconds for startup on this
machine. The default follows the Americano for four minutes of driving, then
brakes and closes. Use the showcase window for follow/traffic-overhead/whole-loop,
pause and path controls. Add `--camera traffic` for an overhead starting view or
`--drive-seconds 90` for a shorter demonstration. See
[portable demos](portable-demos.md#prepared-car-loop-traffic-showcase).

### Rejected visual qualification and v04 correction

The v03 fresh-process pair (`20261009T174736Z-e4f828b5`,
`20261009T174856Z-d6a48d83`) matched exactly across 12,240 samples:
comparison `20261009T175035Z-e187a731`. **That is a physics result, not a
finished visual result.** Image/native-state review found the background
visuals stayed at spawn while the measured colliders moved. At t=20 the nearest
native peer was 3.79 m from ego, but its visible block was not there.

v04 separates a hidden collidable kinematic chassis from a referenced,
nonphysical `RenderProxy`. Before each display frame, measured native poses are
published only to those display transforms. After rendering, composed USD and
native positions/headings must match within 0.002 m / 0.0001 rad. Neither proxy
poses nor camera edits can command the physical ego or background bodies.
Native motion still advances at 120 Hz; visualization at 20 Hz. Original failed
images and their historical pass flags are retained, with this scope correction.

The corrected short v04 run `20261009T175702Z-57369b07` passed: 37 simulated
seconds, one full pass, one lane change, 1.6908 m minimum clearance, no contacts,
and 740 displayed-frame synchronization checks. Its t=20 overhead image confirms
the visible side-by-side pass. Full-duration GUI qualification is recorded below;
the fresh-repeat check is separate. Each attempt gets a fresh directory under
`outputs/loop_showcase/`.
Version folders under `showcases/loop_traffic/` record meaningful progressions;
source versioning remains Git, not duplicated project copies.

The first long GUI attempt `20261009T175756Z-b08d6609` is **failed**, retained:
at t=101.35 the unchanged visual-heading gate detected 0.00011182 rad error
(position error zero). Near-zero yaw amplified float32 quaternion scalar rounding
in USD's rotation conversion. Rendering now uses a double-precision normalized
quaternion; native motion and controller observations are unchanged. A regression
reproduces the failure and verifies <0.000001 rad with perturbed native float32
components; the acceptance tolerance was not widened.

That attempt also crashed during stage teardown in the unused installed Behavior
ScriptManager's Fabric-stage destructor. Its pre-close files and partial timing
remain diagnostic evidence, not completion. The supervisor rejects missing final
results and fatal logs even if the runtime batch wrapper returns zero. Its partial
101.3-second 1x window does not qualify the intended 252-second GUI run.

## Qualified full GUI presentation

Corrected source `fa8eff9`, run `20261009T180436Z-70fff812`, passed the entire
252-second window (240 driving + 2 settling + 10 braking/hold). Twelve distinct
complete passes and five completed lane changes; distance 3,698.0942 m, over one
full circuit. No contacts/overlap, minimum body clearance 1.5808 m, full footprint
inside the road and wheels supported. Maximum speed 15.60664 m/s (about 34.91 mph).
Stopping distance 18.0196 m against the 44.1508 m limit; stopped hold 7.7083 s
with 0.00294 m drift. Both native stage and child process closed cleanly.

All 5,040 composed visual checks passed: maximum position mismatch zero,
maximum heading mismatch 0.000000239 rad. Actual follow/overhead screenshots
confirm visible moving traffic. Independent USD-only review of the saved scene
found 18 resolved dependency layers, all inside the run scene, no absolute asset
paths, zero composition errors, one physics scene and 12 referenced backgrounds
with nonphysical geometry-only proxies. All nine original v07 files remain
hash-identical. The saved `scene/world.usda` is an initial assembly, not a replay.

Performance on the recorded RTX 5070 / Isaac 6.0.1-rc.7 setup:

| Measurement | Observed result |
| --- | --- |
| Live simulation / active wall time | 252 s / 252.0003424 s |
| Real-time factor | 0.99999864; soft-real-time gate passed |
| Render rate / physics rate | 20 Hz / 120 Hz, no skipped physics ticks |
| p95 / maximum scheduling lag | 0.000510 s / 0.187459 s |
| Startup including warm-up | 11.7661 s, excluded from live pacing |
| Whole-desktop GPU peak | 2,722 MiB, sampled approximately once per second |
| Child peak working-set memory | 5,570,404,352 bytes (about 5.19 GiB) |

This is a measured 13-car lightweight showcase, not a fleet-capacity limit or
hard-real-time guarantee. Adding LiDAR, more fully dynamic vehicles, resolution
or detailed background assets requires new measurements. A single four-minute
pass does not establish an indefinite GUI memory-soak result.

The process-local scripting isolation disabled `omni.behavior.scripting.core`
and its installed dependents `isaacsim.replicator.behavior`, `isaacsim.exp.base`
and `isaacsim.exp.base.python`. Required PhysX/simulation/rendering extensions
remained enabled. Source and composed-stage checks reject embedded behaviors;
this is a scoped compatibility workaround, not a vendor-file change or a
general-purpose setting to apply to other Isaac projects.

## Repeated runs and automated verification

Fresh corrected runs `20261009T180927Z-9edacf1b` and
`20261009T181057Z-d2fbb54a` each drove for 110 seconds plus settling/braking.
Comparison `outputs/loop_showcase_repeatability/20261009T181248Z-29b478bb`
verified input/output hashes, equal configurations and runtime identity,
14,640 aligned physics samples and 2,440 visual records per run. Position and
speed differences were exactly zero; nine passes and three changes in each.
The comparator checks full cadence, publication sequence, times, visibility,
fleet count and finite pose tolerances rather than trusting a summary flag.
Independent inspection of the actual t=20 overhead capture confirms the pass.
This is same-machine fresh-process repeatability, not cross-platform or reset-soak
determinism. GUI timing was measured separately, without concurrent CPU tests.

Final automated suite: **1,333 passed, 92 skipped, 15 subtests passed** in the
traffic environment; **67 USD-specific/regression tests passed** in the USD
environment. These include original highway composition/view checks and new
vehicle/fleet/view/runtime checks. GPU evidence remains separately required.
Actual GUI follow and offscreen traffic-overhead views were inspected; camera
controls and path visibility have automated tests, not a manual click-through audit.

### Corrected-version failure tests

| Run under `outputs/loop_showcase/` | Result |
| --- | --- |
| `20261009T181228Z-6e5d47fc` — intentional contact | Native ego/background collider contact at tick 705, four points and a nonzero impulse; confirms hiding chassis visuals did not disable collision. Expected collision, not a safe-driving result |
| `20261009T181244Z-daf21212` — blocked lanes | No passes or lane changes, sustained following at traffic speed, minimum clearance 10.4050 m, safe braking and 9.0167 s hold |
| `20261009T181325Z-197921ce` — command loss | Zero throttle/full braking after expiry; 19.3166 m stop, 7.6167 s hold, 0.00648 m drift, no contact and minimum clearance 1.6908 m |

All three completed with clean runtime logs/shutdown and unchanged source/static
files. Test modes keep independent evidence directories; they do not overwrite
the presentation result. No training was launched.

## Learning document synchronization

Repository record is authoritative. Google guide sync is pending: a fresh
required trusted-read attempt again failed with `workspaceRoot must be an absolute
path` for the canonical Windows path. No cloud update is claimed for this milestone.

## Implementation map

| File | Why it exists |
| --- | --- |
| `scripts/demo_loop_showcase.py` | Portable runtime discovery, bounded child process, memory guard and durable evidence supervisor |
| `scripts/physics_loop_showcase.py` | Compose the local car/road/fleet; fixed-clock control, physics, rendering and independent checks |
| `traffic/loop_showcase_control.py` | Timestamped object contract, adjacent-lane planning, smooth path and low-level steering/speed requests |
| `traffic/driver_control.py` | Existing actuator limits, steering-rate limit and expiring-command braking |
| `usd/showcase_vehicle.py` | Generic validation/copy/composition of the supplied prepared vehicle without changing its publication |
| `usd/showcase_fleet.py` | Reusable block-car references, collidable kinematic bodies and measured-pose display proxies |
| `traffic/showcase_runtime.py` | Guard against embedded behaviors and isolate the unused installed scripting extension in this child process |
| `visualization/loop_showcase_view.py` | Follow/overhead/overview cameras, actual path overlay and presentation controls |
| `experiments/loop_showcase_config.py` | Frozen profiles, explicit test modes and independently assessed physical acceptance |
| `experiments/loop_showcase_repeatability.py` | Verify evidence hashes, identical inputs, physical repeatability and complete v04 visual synchronization records |

Existing runtime, contact, scene-structure and immutable-checkpoint helpers are
reused. No package installation or trained-policy file is needed. The main
vehicle and its copied dependencies are deliberately excluded from Git because
redistribution rights have not been established; public source alone cannot
reconstruct those local asset bytes.
