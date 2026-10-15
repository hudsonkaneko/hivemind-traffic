# v08 — native sidebar controls

Opt-in interactive exploration, preserving v07 and all earlier evidence.
The regular Isaac editor remains; the new panel docks alongside Properties.

## Scope

- Live: follow-camera distance/height, path preview distance, statistics refresh,
  nearby decorative-wheel animation, camera selection, path visibility, pause.
- Apply/restart: 4–80 background cars, 5–35 mph ego target, 5–25 mph inner-lane
  background speed, 1.8–3 s following gap, 10/20/30 Hz display and 50/75/100%
  render resolution. Outer-lane traffic speed scales to preserve angular spacing.
- Future controls are disabled, labeled DISPLAY ONLY / Not implemented, and
  excluded from settings. Their illustrative numbers are not active parameters.

The physics car still uses timestamped simulator object tracks, not LiDAR/RL.
Backgrounds remain constant-speed nonreactive kinematics, not human drivers.
Lower density changes actual car count, not visibility. Live controls do not
change vehicle motion. Physics/control/planning remain 120/60/10 Hz.

## Launch

From repository root with the configured traffic Python:

```text
python scripts/demo_loop_showcase.py --version v08
python scripts/demo_loop_showcase.py --version v08 --traffic-count 12 --ego-mph 25 --traffic-mph 12
```

Restart is supervised, not a hot scene rebuild: finish at least 15 s of driving,
brake/hold for 10 s, close Isaac, then open a fresh process with validated pending
settings and a unique run directory. An intentionally shortened run is NOT a
completed scenario or a passing performance trial. Restart is allowed only
after safe-stop, source, runtime and cleanup checks. Maximum ten restarts per
launcher invocation. An unsafe/failed run stops the session, never auto-retries.

Run records retain initial settings, applied display events and next-run requests.
UI changes/restarts exclude a run from fixed-display performance qualification.
Variable scenarios do not inherit v07's minimum overtaking targets: zero passes
can be appropriate at low density or low ego speed. All safety gates remain.

## Acceptance defined before runtime trials

1. CPU tests: immutable historical defaults, all 4–80 counts, range validation,
   constant-angular-speed placement, bounded latest-value mailbox, disabled
   prospective controls and fail-closed restart validation.
2. GUI smoke: native slider models affect actual pending/live settings; sources,
   scene ownership, contact/road/brake/hold and clean shutdown pass. A requested
   restart produces a second package using the chosen values. This automated
   model test is not an OS mouse/keyboard latency qualification.
3. Matched 90-second drives with panel visible/hidden: same physical settings,
   compare position/speed and p95 app/frame intervals, active wall time and memory.
   Proposed panel overhead target: <5% active-wall increase; retain misses.
   Do not claim 60 FPS or full real time from unchanged UI work alone.
4. Test low/high parameter endpoints separately; report untested combinations.

Native safe-stop/restart evidence: `20261013T000115Z-f62a2c3e` followed by
`20261013T000156Z-ebbf51e5`. The second process passed physical/scene/process
checks with 12 cars, 25 mph ego, 12 mph background and 2.4 s preferred gap;
the native panel reported docked and all 26 future controls stayed disabled.
Later user changes to pending settings were recorded, so this is not a matched
performance trial. The proposed visible/hidden overhead pair and exhaustive
parameter combinations remain unqualified. Failed predecessors are retained
and described in the [combined learning/evidence note](../../../documentation/showcase-live-controls.md).

Prefer v09 for live driving/density changes without restarting. v08 remains
the explicit next-run-settings milestone; no historical evidence is replaced.
