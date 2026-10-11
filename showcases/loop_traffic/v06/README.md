# v06 — lighter 80-car presentation

Status: physical, repeatability and failure-control checks passed;
**strict smooth real-time gate remains open**. Preserves v04/v05 and the supplied Americano
v07. No training, sensors, dependency installation or vehicle asset changes.

This version shares measured fleet reads, avoids duplicate camera/path updates,
skips distant decorative wheel spin, and reuses exact road-position arithmetic.
It retains all 80 moving collision bodies, 120 Hz physics/safety sampling,
60 Hz control, 10 Hz planning, 35 mph target, and the original safety thresholds.

Run from the repository root in the configured traffic Python environment:

```text
python scripts/demo_loop_showcase.py --version v06
```

The opt-in default is a 10 Hz, paced GUI presentation. Follow, traffic overhead,
whole-loop, pause and planned-path visibility controls are available. Display
rate and real-time factor are different: 1x simulation at 10 Hz is not 60 FPS.

```text
python scripts/demo_loop_showcase.py --version v06 --evidence full --drive-seconds 90
python scripts/demo_loop_showcase.py --version v06 --camera traffic
python scripts/demo_loop_showcase.py --version v06 --check
```

Presentation mode samples optional native/display agreement audits at 1 Hz,
plus the first sample; full mode retains every-tick/every-display checks and
crash-safe per-second evidence chunks. Presentation still checks contacts,
clearance, road containment, wheel support, attitude and command expiry at full
cadence, but a native crash may lose its buffered records. All modes write
unique local evidence folders; never overwrite an older run.

20/30 Hz are explicit experimental display overrides, not qualified defaults.
The initial 30 Hz trial ran slower than real time. See the
[performance record](../../../documentation/showcase-performance.md) for actual
timings, retained trials, physical versus display gates, and repeatability.

The showcase is scripted and reads simulator state, not LiDAR detections or
learned policies. Background cars follow nonreactive kinematic targets. Full
evidence does not change those limitations or turn this into a research result.
The original no-version launcher still selects the lighter twelve-car v04.

Two full GUI drives completed 46 passes/23 changes each without contact and
matched exactly over 30,240 physics samples. Both finished 252 simulated seconds
in approximately 252 wall seconds, but p95 accumulated lag was 1.652/0.194 s
(required <=0.1 s) and maximum frame interval exceeded 0.2 s. Do not describe
these as strict smooth-playback passes. Optional audits were sampled as above.
Full-evidence mode and command-loss, blocked-lane and contact-positive fixtures
were validated separately; all old and failed evidence remains available.
