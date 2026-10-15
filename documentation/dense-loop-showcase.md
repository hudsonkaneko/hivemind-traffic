# Denser loop showcase — v05

User objective: a semi-crowded highway with room for the prepared main car to
pass slower traffic. This is a presentation fixture, not a mixed-traffic study
or an increase in the number of learned/physics-driven drivers.

## Design and unchanged boundaries

- 80 slower block cars, 20 in each of four lanes, plus the Americano v07.
- Forty staggered pairs around the 500 m reference-radius loop. Inner pairs
  point their gap outward; outer pairs point inward. This leaves recurring
  adjacent-lane opportunities, not a permanently empty passing lane.
- Background lane speeds are approximately 15–15.33 mph, with shared angular
  speed so nonreactive cars do not catch one another over successive laps.
- Main cruise target remains 35 mph, with the same steering, torque, contact,
  road-containment and clearance limits. It follows/slows when no safe gap exists.
- Main car: dynamic PhysX; backgrounds: collidable kinematic PhysX. Timestamped
  simulator object tracks, not LiDAR perception, RL or human-driver behavior.
- Physics 120 Hz, controller 60 Hz, planning 10 Hz, rendering 10 Hz in v05
  (20 Hz in v04). This trades animation smoothness for live wall-clock speed;
  it does not accelerate a recording or reduce physics/safety checks. All original
  vehicle assets and prior experiments remain unchanged; v04 is still selectable.

## Performance and rendering work

`traffic/showcase_clearance.py` uses circumscribed-circle lower bounds to prune
distant candidates before exact oriented-box checks. Every car participates in
the bound; the returned minimum clearance and overlap are still exact. This is
not a reduced perception radius or a skipped safety tick. Tests compare against
exhaustive polygon calculations on 150 seeded scenes with 80 rotated boxes.

The dense preset uses batched Sdf edits in a disposable visual-motion layer.
Composed values/handles are collected before each `Sdf.ChangeBlock`; only Sdf
writes happen inside it. The main car and native background targets are never
driven by these display edits. Original publication behavior remains available
for v01–v04. Tests exercise both paths, source immutability and pose agreement.
The v05 physical background reference contains only its native body/collider;
the visible wheel/body geometry is referenced once under the render proxy.
This removes duplicate hidden meshes, not collision shapes or traffic cars.
The dense runner reads main-car pose, wheel support and velocity directly from
native physics. It publishes PhysX-to-USD transforms only before a displayed
frame, rather than during every physics/control read. An identity-checked
rigid-body velocity view prevents stale USD velocities between frames. The
fixed simulation clock and control rates are unchanged; standalone probes and
older versions retain their original publication defaults.

At the fixed driving endpoint, v05 requests a coordinated **demo rundown**:
all backgrounds decelerate at 3 m/s2 on the base lane, scaled by lane radius.
The main car brakes through its existing actuator interface while keeping its
path steering. Native targets integrate the deceleration continuously; neither
poses nor velocities jump to a parked location. The stop-only object contract
allows bounded deceleration, but still rejects acceleration, identity changes
and off-circle motion. Driving retains the original constant-speed prediction.
All backgrounds must remain stopped through the final five seconds. This is
scenario shutdown, not learned cooperation, V2V or reactive human braking.

Dense stages exposed a native `usdrt.population.plugin` first-render crash in
the installed Isaac 6.0.1-rc.7 / driver 610.74 setup. The current exploration
uses the documented process-local `/app/useFabricSceneDelegate=false` fallback.
This selects the legacy OmniHydra delegate; it does not change installed files,
persistent preferences, physics, safety thresholds or source geometry. NVIDIA
documents the switch and the diagnostic population options in its
[FSD configuration reference](https://docs.omniverse.nvidia.com/kit/docs/usdrt.scenegraph/latest/fabricsd/configuration.html).
The exact vendor root cause is not established; a working fallback is not proof
of a diagnosed race or of a universal 80-car limitation.

## Predeclared acceptance

Before marking the final v05 ready: at least six complete overtakes and four
completed lane changes in 90 seconds, zero native contact/overlap, at least
0.6 m body clearance, supported wheels, full road containment, successful final
braking and at least five seconds stationary hold. Require complete 120-Hz
traces, source/static-layer preservation and every rendered frame synchronized
with measured native background poses. Inspect actual screenshots. Run fresh
identical repeats and qualify GUI timing separately; do not equate unpaced
physical success with real-time success.

## Retained exploratory runs

All IDs are under local ignored `outputs/loop_showcase/`. No failed evidence was
overwritten. These attempts are not final capacity measurements.

| Run | Purpose and outcome |
| --- | --- |
| `20261010T204214Z-26ad8692` | Initial 80-car layout: native first-render population crash, no live trajectory |
| `20261010T204258Z-09d3fecb` | Same dense launch: same crash; task-owned stuck child was terminated |
| `20261010T204328Z-1b31cb3a` | v04 control launch briefly overlapped the crashed child; canceled during viewport startup, excluded from qualification |
| `20261010T204517Z-247275b8` | Fresh isolated v04 15-second smoke: physical/visual/braking gates passed, unchanged sources |
| `20261010T204556Z-a9977223` | Serial-population diagnostic did not prevent native crash; option not retained |
| `20261010T204622Z-838e0817` | Legacy-delegate exploration completed safely, three passes/two changes, minimum clearance 1.5836 m. Became boxed in after ~30 s. 102 simulation seconds in 214.484 active seconds (0.476x); concurrent CPU tests mean this is not a clean comparative performance benchmark. Does not meet the strengthened v05 presentation gate |
| `20261010T205038Z-2df19941` | Reversed outer-pair stagger, optimized-diffing disabled: first-render crash; option not retained |
| `20261010T205212Z-7f5642d5` | Batched visual edits with Fabric delegate: still crashed; batching alone is not a renderer-crash fix |
| `20261010T205308Z-ab3d1ebd` | Improved stagger: 16 passes/eight changes by 90 s. Failed at ~97.65 s when a nonreactive background reached the stopped ego. This exposed the need for an explicit whole-fleet rundown; it is not a passing validation run |
| `20261010T205851Z-095ac81a` | Removed hidden duplicate visuals and added rundown, but Fabric still exited without a completed probe; not accepted |
| `20261010T210018Z-e6563df1` | First complete safe v05: 16 passes/eight changes, minimum clearance 1.5693 m, all physical/visual/rundown gates passed. Unpaced 102 s / 133.816 active wall s (0.762x) before duplicate-publication optimization; not real-time qualified |
| `20261010T210333Z-6bac071b` | Avoided duplicate USD publication: same safe behavior, 102 s / 130.324 active wall s (0.783x). Not real-time qualified |
| `20261010T210628Z-ad519ed9` | Final native-state/display-decoupled implementation: same 16 passes/eight changes, minimum clearance 1.5693 m, 102 s / 130.012 active wall s (0.785x). Full physical/visual/rundown and source-preservation gates passed; screenshots inspected. This optimization did not establish a meaningful timing improvement over the preceding run |
| `20261010T211810Z-408c14ab` | Repeat reproduced 16 passes/eight changes and every physical/visual gate, but crashed during scene closure. The supervisor correctly rejected it despite the native wrapper returning exit code zero; NOT a qualified repeat |
| `20261010T212122Z-0c0dc0ef` | Command-loss braking worked, but the same scene-close crash invalidated the run; retained as a failure |
| `20261010T212252Z-d70a35d5` | Pre-icon-isolation blocked-lane check passed, no lane changes, minimum clearance 10.4039 m |
| `20261010T212326Z-2608e403` | Pre-icon-isolation intentional-contact check passed: real native contact was detected; not an avoidance success |

## Qualified scope and performance limit

Full GUI run `20261010T211050Z-5d35f8f5` passed the physical, visual, rundown,
composition and unchanged-source gates: 240 seconds driving plus 12 seconds
settling/stopping, 46 complete passes, 23 lane changes, 3,537.1658 m traveled,
zero contacts/overlaps and 1.5693 m minimum body clearance. Maximum measured
speed was 15.6065 m/s. The final stop took 10.8503 m against a speed-dependent
27.4770 m limit, followed by 8.225 seconds of hold with 0.00296 m drift.

**This dense preset is NOT real-time qualified.** The paced GUI simulated
252 seconds in 335.4184 active wall seconds (0.7513x); p95 lag was 82.598 s,
well outside the 0.1 s limit. The summary's general `passed` field describes
physical/scene acceptance; `preview_timing.soft_realtime_passed` is separately
false. A 10-Hz simulation display cadence yields about 7.5 displayed frames per
wall second at that speed. No dropped physics ticks or accelerated replay are
used to conceal the timing shortfall. Whole-desktop peak GPU allocation was
3,869 MiB; this is not an isolated process GPU-memory measurement.

Follow screenshots at 20 and 241 seconds and a short-run overhead screenshot
were inspected. Read-only inspection of the saved composed stage found 80
referenced backgrounds, 80 nonphysical render proxies, exactly one physics
scene and no unresolved external layer references. Source vehicle preservation
was also checked by the supervised run. Camera UI buttons have automated tests;
this was not a manual click-through audit.

The unchanged v04 remains the default and lighter real-time-qualified option.
Use the configured traffic environment from the repository root:

```text
python scripts/demo_loop_showcase.py --version v05
python scripts/demo_loop_showcase.py --version v05 --camera traffic
python scripts/demo_loop_showcase.py --version v04
```

Do not run these simultaneously. The launcher discovers the configured Isaac
runtime. The supplied Americano v07 asset is a local prerequisite, not a Git
download. Outputs remain unique under ignored `outputs/loop_showcase/`.

## Automated regression and documentation

### Scene-close callback diagnosis

The failed repeat and dropout runs reached all driving/braking gates, then
crashed in `isaacsim.gui.sensors.icon`'s installed `model.py:212`, releasing its
USDRT stage handle in `_on_stage_closed`. This identifies the observed callback,
not the underlying native allocator defect. Reading the installed extension
manifest confirms it only draws sensor-identification icons in viewport/stage
UI; it does not compute sensor measurements.

v05 now disables that unused display extension via the public extension manager
before constructing the owned stage. It records actual dependency changes,
rejects loss of any other extension or essential runtime, and checks that the
icon helper stays disabled before playback and after the run. This is a scoped
workaround, not a vendor patch or a sensor/physics bypass. v04 is unchanged.
The prior long-GUI measurement above predates this callback isolation; final
post-workaround smoke/repeat/fault evidence is recorded separately below.

Post-workaround GUI smoke `20261010T212411Z-84e78c6d` passed a 15-second drive,
braking/hold, displayed-pose checks and clean stage/process shutdown. Only
`isaacsim.gui.sensors.icon-2.2.1` was disabled; all required physics/rendering
extensions remained enabled. Its shorter timing window achieved 0.807x, still
not real-time. It is not a substitute for the longer passing/repeat checks.

First post-workaround 90-second run `20261010T212511Z-6495f5ba` passed all
physical/visual/source gates and clean shutdown: 16 passes, eight lane changes,
1.5693 m minimum clearance, 102 simulated seconds in 128.1477 active seconds
(0.796x). All 1,020 displayed frames were checked against native motion.

The second fresh run, `20261010T212738Z-aa5d3a93`, also passed with clean
shutdown (0.783x). Comparator
`outputs/loop_showcase_repeatability/20261010T213056Z-99828d04` accepted the
pair: identical captured source/asset/configuration hashes and recorded runtime,
12,240 aligned physics samples, **zero position and speed difference**, the same
16 pass identities and eight lane changes. Both had complete 1,020-frame,
80-car visible/native checks, zero position discrepancy and maximum yaw error
2.39e-7 rad (limit 1e-4). This is same-machine fresh-process repeatability, not
cross-platform determinism or a guarantee against every future native failure.

```text
python -m experiments.loop_showcase_repeatability --first outputs/loop_showcase/20261010T212511Z-6495f5ba --second outputs/loop_showcase/20261010T212738Z-aa5d3a93
```

Final post-workaround fault checks all passed with clean shutdown, no runtime
errors and unchanged sources:

| Run | Result |
| --- | --- |
| `20261010T213006Z-26eb3977` | Command loss, 80 backgrounds: expired commands produced zero throttle/full braking; 15.1089 m stop, 7.90 s hold, 0.00663 m drift, no contacts and 1.5723 m minimum clearance |
| `20261010T213131Z-e291be75` | Four-across blocked fixture: no passes/changes, followed traffic, 10.4039 m minimum clearance, 3.3465 m final stop and 9.0167 s hold |
| `20261010T213205Z-f22c58e2` | Intentional contact fixture: one native contact report detected. Expected collision proves detection remains active; not an avoidance result |

The lightweight fault fixtures are not 80-car capacity benchmarks. The brief
offline repeat comparator ran during the final dropout test, whose timing is
therefore not used as an isolated performance measurement.

The final traffic-environment suite passed **1,384 tests**, with 119 optional
skips and 18 passing subtests (`python -m pytest -q`). The increased skip count
includes USD-dependent tests; the separate USD environment passed **100 tests**:

```text
python -m unittest tests.test_showcase_fleet tests.test_showcase_vehicle tests.test_loop_showcase_view tests.test_showcase_runtime tests.test_highway_loop_scene tests.test_highway_loop_view -q
```

Tests cover exact clearance pruning, stable 80-car spacing, unchanged v04 and
fault fixtures, stop-only deceleration contracts, final-fleet-stop rejection
cases, native ego identity/velocity shape, render-decoupled clock ownership,
source-preserving Sdf batching, referenced composition and v05 repeat-evidence
integrity. Unit tests alone do not qualify runtime physics or real-time speed.

The Google guide's new **34 Loop traffic showcase** tab covers v04 and v05 with
definitions, portable shaded command blocks, evidence and limitations. A
revision-guarded write and full native readback verified text/styles and that
all older tabs remained unchanged; see [sync record](learning-guide-sync.md).
The main roadmap, prior runs, source assets and local runtime installations
were not replaced. No Isaac Lab training was started.
