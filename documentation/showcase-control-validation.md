# Bounded live-control validation

October 16, 2026. This extends v09's opt-in validation without changing the
existing `--control-smoke` sequence. It tests the actual native widget-model
callbacks when Isaac is running. CPU fixtures test the harness and controller
contracts only; they do not establish native UI, physics or performance results.

## Declared sequence and acceptance

The `endpoints` scenario starts with **12 background cars, 10 mph main-car
target, 5 mph inner-lane background speed, and 2.4 s preferred following gap**.
All times below are absolute simulation seconds, including the two-second
settle. Keep 120 Hz physics, 60 Hz control, 10 Hz planning and all existing
contact, clearance, road, support, braking and shutdown gates.

| Request time | Target cars | Main target | Traffic target | Following gap | Must be applied by |
| --- | --- | --- | --- | --- | --- |
| 5 s | 4 | 5 mph | 5 mph | 3 s | 14 s |
| 15 s, while paused | 80 | 35 mph | 25 mph | 1.8 s | 50 s |
| 55 s | 12 | 35 mph | 5 mph | 3 s | 100 s |
| 105 s | 80 | 35 mph | 15 mph | 1.8 s | 135 s |

The pause uses the sidebar's normal pause/resume callback. During four UI-only
frames the driver issues a burst of 64 population edits and nine speed/gap
edits. The pending mailbox must retain only the latest values; applied settings,
event count, native physics step/time, chassis position/speed and traffic state
must stay unchanged until resume. Rendering may advance its own frame count.
The normal boundary must then consume exactly one event for the complete burst.
An attempted 25 mph background target while the main-car target is still 5 mph
must be rejected and its native model reset to the accepted value.

Actual population and all three ramped driving values must meet every deadline;
successful callbacks alone are insufficient. Lifecycle evidence includes the
circular arc separation computed from the actual runtime phase for every
entry/removal, with native pose and physical-clearance audits kept separate. Changes must remain
at least 160 m from the main car and at least 30 physics ticks apart. Stable
generation identities, physical pose checks and existing native pool checks
remain in force. No hierarchy changes or physics reset are requested.

The 135–180 s tail must retain 80 cars and settled traffic speed, and complete
at least one **additional pass and lane change** after the last transition.
Earlier passes cannot satisfy that gate. The run must not restart. The scenario
is separate from the legacy three-stage 12/40/12 smoke and requires at least
180 driving seconds. Every failed checkpoint remains an explicit failed gate.

## Scope and protected low-density requests

The 4-car endpoint is reached from 12; the dense removal exercises 80 to 12.
This is deliberate. At density 80, more than four cars can lie within the
160 m protection zone. If the main car follows at their speed, a target of four
can remain unreachable until relative geometry changes. A CPU negative fixture
checks that such a request remains pending rather than deleting protected cars.
This does not claim every 80-to-4 transition completes within a fixed deadline.
Neither the exclusion distance nor the safety checks are relaxed to make a
population target pass. The sidebar continues to show actual and requested
population separately.

The scenario covers the stated endpoint sequence, callback coalescing,
pause/resume, bounded lifecycle and passing afterward. It does not qualify
mouse-to-display latency, every combination of sliders, extended endurance,
reset/reload, or 30/60 FPS. The one main car is scripted PhysX control; the
background cars remain nonreactive scripted kinematics. No sensors, SUMO
drivers, learned policies or training are introduced.

## Reproduce and inspect

From the repository root in the configured traffic environment:

```text
python -m pytest -q tests/test_showcase_control_validation.py tests/test_live_showcase.py tests/test_showcase_controls.py
python scripts/demo_loop_showcase.py --version v09 --vehicle public --control-validation endpoints --drive-seconds 180 --headless --panel-hidden --unpaced
```

The native command must construct hidden native widget models, even in headless
mode. Use the public car consistently; results cannot be pooled with prepared
vehicle runs as if they shared a physical model. Retain the resolved initial
configuration, source hashes, control events, lifecycle records, trajectory,
pause snapshots, deadlines, safety gates and clean-shutdown evidence in the
new run's ignored output folder. Changing controls makes this an exploratory
validation run, not a fixed-demand performance comparison.

Implementation: `experiments/showcase_control_validation.py` owns the declared
schedule and assessment; the runner supplies fresh native observations and
consumes settings only after the pause loop returns. `traffic/live_showcase.py`
records actual lifecycle arc distances and admission geometry. Native outcomes
are recorded below only when runtime evidence is available.

## Evidence

The October 16 harness-only checkpoint passed the three targeted CPU suites above:

**30 tests and 16 subtests** in the configured traffic environment. They include
preflight-only CLI/configuration checks with no runtime launch, adversarial pause
and missing-evidence fixtures, endpoint/deadline assessment, measured lifecycle
distance/rate checks and the protected low-density case. A CPU pass is not a
native endpoint acceptance.

### First native endpoint attempt and admission repair — October 19

Run `20261019T162155Z-fb9cb018` completed all 23,040 physics samples and clean
shutdown, with no runtime errors or USD reference-count warnings. Its physical
checks passed, including 1.844801 m minimum measured clearance, but **overall
acceptance failed**. Actual populations at 14 / 50 / 100 / 135 s were
**4 / 75 / 12 / 77**, so both 80-car deadlines failed. The entire 135–180 s tail
remained at 77 cars, with no additional pass or lane change. The one earlier pass
cannot satisfy the tail gate. Pause/mailbox stability, callback coalescing,
invalid-pair rejection and bounded remote lifecycle checks passed.

Three remaining dormant slots stayed inside the 160 m exclusion zone. Their
fixed phases moved at the same angular rate as the traffic the main car followed,
so they remained protected. Population-pending mode correctly inhibited new lane
changes. Read-only geometry analysis found empty remote same-lane gaps; the
fixed nominal-phase admission rule could not use them. This is a demonstrated
admission limitation, not a reason to remove the exclusion or force an overtake.
The failed run and its original source snapshots remain unchanged.

The repair in `traffic/live_showcase.py` preserves safe existing-phase priority
and adds a deterministic fallback for **dormant slots only**. Its runtime phase
mapping leaves immutable fleet specs, static USD, active poses and native body
bindings intact. Every entry requires at least 160 m main-car separation,
40 m front/rear base-circle center spacing, and 0.8 m exact analytic background
footprint clearance. The 40 m bound covers the maximum configured 25 mph and
3 s gap plus body length and clearance, independently of current speed ramps.
Membership and failed-search attempts remain at least 30 physics ticks apart;
committed maneuvers and stopping freeze changes. No admissible bounded candidate
means the request remains pending. Exits use actual relocated phases.

Lifecycle evidence records the selected station, old/new phase, placement method,
front/rear spacing and minimum background footprint clearance. Empty-neighbor
gaps are explicit `null` values. Geometry is calculated from analytic current
target poses; it does not replace measured native pose, contact or clearance
checks. The [live-controls implementation note](showcase-live-controls.md#remote-entry-positions--october-19-admission-repair)
describes the thresholds, event fields and unchanged native admission path.

The repair's focused CPU check passed **112 tests and 18 subtests** across live
controls, endpoint validation, the planner and clearance. Separate plain Isaac
Python/OpenUSD tests passed **two tests with fake native views**, including a
protected nominal entry with 79 active cars, selected-index-only teleportation,
continuous next target/step, generation accounting, visibility, stable native
paths and unchanged static layers. These tests did not launch `SimulationApp`
or exercise actual PhysX dynamics. They establish software/composition contracts.

Native legacy regression `20261019T164220Z-7fd95c16` passed all 33 run gates and
reached the unchanged 12/40/12 checkpoints. Repaired endpoint
`20261019T164433Z-563caac3` reached all four population, speed and gap deadlines
(4/80/12/80), with all physical/scene/process gates passing and 1.831394 m minimum
clearance. Its settled 135–180 s tail added no passes or lane changes, so overall
endpoint acceptance remains **failed**. Three earlier passes and one earlier
lane change do not satisfy that tail gate. Both failures remain in the
[measured report](showcase-validation-20261019.md).

The admission repair does not change the planner's conservative lane-change
forecast. A separate bounded gap-creation improvement is under investigation;
the original endpoint schedule and acceptance requirements remain unchanged.
