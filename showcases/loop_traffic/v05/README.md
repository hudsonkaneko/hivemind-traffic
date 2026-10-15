# v05 — semi-crowded loop

Status: physical/visual GUI acceptance passed; **not real-time qualified**.

User request: a busier highway with usable passing gaps. Preserve v04's twelve
background cars and add a v05 profile with 80 slower cars across the full ring.
Forty staggered pairs alternate between inner and outer lane pairs; there is
no permanently empty passing lane and no four-across fixed roadblock. Shared
angular speed keeps nonreactive background gaps stable across repeated laps.
Main-car dynamics, cruise target and safety margins are unchanged.

Acceptance: existing zero-contact/overlap, >=0.6 m body clearance, road support,
braking/hold, command, static-source and visible/native synchronization gates.
Require >=6 passes and >=4 lane changes in a 90-second drive. Inspect real
images for visibly denser nearby traffic; qualify paced GUI separately. Repeat
identical runs and keep failed attempts. No training or sensor-driving claim.

Dense validation uses circumscribed-circle distance bounds to avoid expensive
polygon checks on clearly distant cars. The minimum distance remains EXACT;
seeded geometry tests compare the result against checking every car.

```text
python scripts/demo_loop_showcase.py --version v05
```

No asset changes or dependency installations. The supplied Americano remains
local-only. Earlier version folders and evidence stay available.

Full GUI evidence `20261010T211050Z-5d35f8f5`: 46 passes, 23 lane changes,
zero contact/overlap, 1.5693 m minimum clearance, safe final braking and hold.
The main car drove 3,537.17 m in the bounded preview. Two final 90-second drives
achieved 16 passes/eight changes each, exceeding this version's presentation
gate, and matched exactly across 12,240 physical samples and 1,020 checked
visual frames. Both completed with clean shutdown after the icon workaround.

Important tradeoff: the full GUI achieved **0.751x real time**, with rendering
at 10 Hz of simulation time. Keep v04 for the lighter real-time demo. v05 uses
a process-local legacy renderer fallback after dense-stage startup crashes;
there are no vendor-file or permanent preference edits. The final whole-fleet
deceleration is demo shutdown, not learned coordination or reactive humans.
An unused sensor-icon GUI helper is also disabled only in the v05 process after
its scene-close callback crashed two otherwise completed trials. The full GUI
measurement predates that workaround; a fresh GUI smoke and paired 90-second
runs passed afterward. Failed runs remain recorded, not reclassified as passes.

See the [complete evidence and retained failures](../../../documentation/dense-loop-showcase.md)
for repeat/fault checks, timing, composition, CPU/USD tests and Google guide sync.
