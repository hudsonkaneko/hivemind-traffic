# Highway Stage 1 v03

**Experimental — not accepted.** Trained November 3, 2026 (America/Los_Angeles).

[Download policy.pt](policy.pt?raw=true) · [Contract](contract.json) ·
[Training parameters](training-config.json) · [Measured results](evaluation.json) ·
[Hashes and provenance](manifest.json)

| Detail | Value |
| --- | --- |
| Policy type | Shared MAPPO actor, behavior-cloning warm start; deterministic TorchScript export |
| Inputs | 93 floats: 45 local/map fields + six delayed 8-field V2V messages; exact order in contract |
| Outputs | Normalized acceleration in [−1,1] and lane intent −1/right, 0/stay, +1/left |
| Acceleration conversion | `-4.5 + (action[0] + 1) * 3.25` m/s², followed by environment guards |
| Network | Message encoder + 128/128 actor; 256/256 centralized critic used only in training |
| Control and limits | 10 Hz, 60 mph ceiling, existing 1.5 s following/stopping guards |
| Training | Four agents; CPU; 49,998 teacher + 449,992 PPO transitions; 12 BC epochs; 162 PPO updates |
| Learning rates | BC 0.0003; actor 0.0001; critic 0.0003; PPO four epochs, batch 256, clip 0.2 |

**Changes from v02:** BC increased from four to twelve epochs with 4× acceleration
loss; teacher guidance tapers from 0.05 to 0.01 over 400k PPO transitions and keeps
that floor. Ranking v5 prioritizes per-case interventions after safety/task gates.
Observations, rewards and protection thresholds are unchanged.

**Intended behavior:** cruise, follow, pass slower cars and merge across varied
highways. SUMO handles motion and lateral execution; the actor is not a steering,
throttle or raw-sensor controller.

**Measured:** 6/6 merges, 12 passes, zero collisions/teleports/overspeed across
12 validation cases; cruise speed 96.44–96.97% of limit. Overrides are
14.97% pooled (51.47% worst); five cases exceed the 5% limit.
This improves on selected v02's 18.15% pooled/eight failing cases, but is still
unaccepted. No held-out test, camera/LiDAR policy inputs or physical-car validation.
It does not supersede the accepted straight-road v04r2 baseline.

A Windows checkpoint replacement failed during BC. Training resumed from the
valid closed checkpoint with the same config/source and preserved budget; both
run records remain. Total recorded training time is 3544.125 seconds.

Selected generation: `selected-00162-80ce2ebf`.
Source: `cf6ae5096ebac9f165e9811ed3916195227f3146`.
Policy SHA-256: `a6d59d3acf58fcca9d7fc33406cb61ca3863d1a3dffad6dd316b195a66c7166f`.

The export accepts float32 `[batch,93]` and returns `[batch,2]`. Use the matching
environment and contract; the recorded zero-input CPU smoke is only a loading
check. Safety filtering, critic and resume optimizer are outside this export.
The historical authorized config does not authorize another training run.
See the [training note](../../../documentation/highway-stage1-training-v03.md).
