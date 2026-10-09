# v02 — moving-object passing

Status: verified in the bounded 90-second driving fixture. Use the same asset/road/traffic ownership as v01. Add 10 Hz
adjacent-lane gap selection, smooth quintic lane changes and 60 Hz steering/speed
control through the existing 120 Hz command gate. Target 35 mph, subject to
physical checks. A 90 s drive must finish at least three distinct passes and two
lane changes while satisfying the safety gates in the technical record.

Run `20261009T173945Z-0a63009b` passed all declared physical gates: **7 complete
passes, 2 completed lane changes, minimum body clearance 1.6594 m**, no contacts,
all wheels supported and the full footprint inside the four main lanes.
Maximum measured speed was near the 35 mph target; exact metrics, source copies,
and screenshots are in the local run's manifest/summary. The adaptive path and
requested detailed car were visually inspected.

Renderer preparation now happens before the live clock (zero added physics
ticks). This unpaced run achieved 1.0305x across 102 simulated seconds. A paced
GUI test is still required for v03; unpaced speed alone is not that test.

```text
python scripts/demo_loop_showcase.py --version v02 --capture
```
