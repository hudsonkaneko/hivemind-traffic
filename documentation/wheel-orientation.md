# Physics-car wheel orientation correction

The simple wheels now roll around lateral axles instead of tumbling like discs.
This corrects the separate PhysX car, not the preserved SUMO or Leatherback demos.
No driver tuning, torque limits, suspension parameters, training, dependency
installation, or external-runtime edits were made.

## Cause and correction

The car uses X forward, Y left, Z up. The installed NVIDIA Factory authored
Y-axis cylinders, initially correct. After simulation started, the USD wheel
attachment acquired a +90-degree Z basis rotation absent from the native wheel
pose. The visible cylinder axle was consequently perpendicular to the tire's
lateral direction: about 90 degrees of error throughout the diagnostic smoke.
This is evidence of an installed runtime's analytic-cylinder pose/writeback
basis mismatch, not a claim about the internal vendor source-code defect.

`traffic/wheel_geometry.py` now authors X-axis cylinders with an explicit local
Z+90-degree rotation before simulation. Radius, width, center, world bounds and
collision APIs remain equivalent at initialization; the local extent is
recomputed. The runtime folds that rotation into the attachment and resets the
child transform, so the **composed shape pose** is the meaningful comparison.
We do not override wheel/body poses during the simulation. PhysX remains the
only motion authority, including spin, steering and suspension motion.

The cylinder is both visible geometry and a collision shape, so this is not
merely a camera trick. Measured free-driving behavior is unchanged; this work
does not establish arbitrary tire/obstacle collision accuracy or highway fidelity.

## Regression checks and evidence

At every 30-Hz render boundary, all four wheels are sampled. The demo requires
complete, unique coverage and errors at most 0.1 degrees for both:

- World-space cylinder axle versus PhysX tire lateral direction.
- Composed world-space cylinder orientation versus chassis orientation, native
  wheel-local orientation and the explicit geometry-basis rotation combined.

The second check includes wheel spin as well as axle direction. Missing,
duplicate, non-finite or misaligned evidence fails the demonstration.

Four source-captured attempts are preserved in
[compact hash-verified evidence](wheel-geometry-results.json):

| Run suffix | Scope | Interpretation |
| --- | --- | --- |
| `012351Z-16b02157` | Original 8-s diagnostic | Old driving checks pass; wheels are visually wrong, about 90-degree axle error. Not an accepted visual result. |
| `012456Z-1d561f65` | Corrected 8-s smoke | Max axle error 0.000020 degrees; all 960 physical/control states exactly match diagnostic baseline. |
| `012621Z-391b18cd` | First corrected 40-s run | Retained failure: a newly added test incorrectly compared raw attachment/native rotations in different frames. Driving/axle checks pass. |
| `012905Z-a261f043` | Final 40-s corrected check | All checks pass: 4,800 wheel samples; max axle error 0.070414 degrees, composed-orientation error 0.027559 degrees. |

All four run IDs begin `20261005T` (UTC); work occurred October 4 locally.
The final run's 4,800 physical states and applied controls exactly match the
earlier obstacle baseline `20261004T202000Z-aca7bb26`. Lane RMS remains
0.0212006 m, final barrier gap 4.24599 m, with zero contacts. All 760 recorded
post-settle sensor frames matched their poses; max mount error 0.001016 m.
No runtime Error lines or USD reference warnings occurred. Follow-camera
captures at 6/12/28 s were visually inspected across corrected runs.

CPU suite: **786 passed, 4 skipped**. Isaac's Python: **4 wheel tests passed**,
including the two USD tests skipped by the traffic environment. Raw artifacts
remain local under `outputs/vehicle_visual_lidar`; the JSON report verifies
captured sources and artifacts. Raw scans/pictures are not included in a clone.

The full loop took 62.91 wall seconds for 40 simulated seconds (RTF about 0.636).
This test is still a low-speed single-car demonstration, not real-time or
highway-speed validation. Prior evidence remains unchanged.

## Show it

From the clone root with the configured traffic environment active:

```text
python scripts/demo_physics_lidar.py --points
```

Close/restart an already-running demo to load the corrected code. Use Follow car
to inspect the rolling wheels, then Overview to see the planned route.

```text
python scripts/demo_physics_lidar.py --headless --capture --unpaced
python -m pytest tests/test_wheel_geometry.py -q
```

The first command records the complete 40-s turn/brake check with wheel evidence.
The second is a lightweight regression test; USD authoring checks need Isaac's
Python. Runtime discovery still uses local configuration, not hardcoded paths.

Learning terms: **axle** is the wheel's spin axis; **coordinate frame** describes
which directions a rotation refers to; **transform composition** combines
parent/child placement; **regression test** checks that a fixed bug stays fixed.
Lesson: compare geometry in a common frame and test visuals as well as motion.

See NVIDIA's [wheel attachment contract](https://docs.omniverse.nvidia.com/kit/docs/usdrt.scenegraph/7.5.0/api/file_usdrt_scenegraph_usd_physxSchema_physxVehicleWheelAttachmentAPI.h.html)
for wheel/child transform semantics; observed runtime behavior is recorded above.
