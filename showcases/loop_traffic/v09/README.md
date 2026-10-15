# v09 — live traffic tuning

Preserves v08's restart-based option. Traffic count, main-car target speed,
background speed and following gap now request live changes without relaunching
Isaac. Future controls remain disabled and display-only. Rendering cadence and
resolution still use the explicit restart button; those are not traffic knobs.

## Acceptance declared before simulator testing

- Pure tests: all 4–80 initial counts, remote rate-limited entry/removal, logical
  identity generations, bounded speed ramps, controller membership admission,
  and unchanged v08 defaults/behavior.
- Native 90 s drive: reduce 80→12, increase toward 40, reduce toward 12, change
  both speed targets and following gap in one process. Check actual counts and
  applied ramps, collision/clearance/road/brake/hold, visibility/native alignment,
  source preservation and clean shutdown. Repeat the same change script hidden
  from desktop input (headless with a hidden but instantiated native panel).
  GUI docking/interaction are checked separately; a hidden pass does not prove
  visible smoothness or OS input latency.
- Physical/control/planning stay 120/60/10 Hz. Rendering timing and overhead
  are separate gates, not inferred from a successful physical run.
- No USD prim creation/deletion, scene rebuild, tensor rebind, or process restart
  for traffic edits. An 80-body pool bounds memory; dormant bodies remain loaded
  in a remote parking area. Reduced density does not remove all physics overhead.

## Behavior

The count is a target, not an instant jump. Up to four cars enter/exit per
simulated second, outside a 160 m arc-distance exclusion around the main car.
Existing cars never get redistributed. A new logical ID is assigned on re-entry
while its USD pool slot stays fixed. Lifecycle and control events are recorded.

Background speed ramps at at most 0.5 m/s²; the main-car target ramps at 1 m/s²
(actual acceleration still depends on the driver and physics). Following-gap
target ramps at 0.25 s per second. Tuning waits through committed lane changes;
new lane changes wait while background speed or population transitions. A
protected population can remain above its requested target until safe removal
is possible; lane following continues during the wait. Backgrounds remain scripted,
nonreactive kinematics—not human drivers. Their speed cannot exceed the main-car
target. Safety gates still apply; these bounds do not prove every combination.

Launch from the configured traffic environment at the repository root:

```text
python scripts/demo_loop_showcase.py --version v09
```

Status: bounded native live-change test verified twice: `20261013T002704Z-70c5c8fc`
and `20261013T002856Z-6b410449`. Actual counts 12/40/12 at 34/64/89 s; no restart,
contacts, overlap or road departure; safe braking/hold and clean shutdown.
All 12,240 trajectory rows repeated exactly, as did sources/configs and events.
The controller followed rather than overtook in this tuning sequence (0 passes,
0 lane changes); this does not qualify every setting combination or smooth GUI
performance. GUI docking and interaction were checked separately before the
final transition-scheduling fix. See the [full evidence](../../../documentation/showcase-live-controls.md),
including failed trials and pending performance/endpoint tests.

No LiDAR, V2V or training added. Live tuning is exploratory, not a matched
fixed-configuration traffic experiment. Existing v04/v07 showcase profiles remain.
