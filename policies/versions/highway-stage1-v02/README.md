# Highway Stage 1 v02

**Experimental — not accepted.** Trained November 3, 2026.

[Download policy.pt](policy.pt?raw=true) · [Contract](contract.json) ·
[Training parameters](training-config.json) · [Measured results](evaluation.json) ·
[Hashes and provenance](manifest.json)

| Detail | Value |
| --- | --- |
| Policy type | Shared MAPPO actor, behavior-cloning warm start; deterministic TorchScript export |
| Inputs | 93 floats: 45 local/map fields + six delayed 8-field V2V messages; exact order in contract |
| Outputs | Normalized acceleration in [−1,1] and lane intent −1/right, 0/stay, +1/left |
| Acceleration conversion | `-4.5 + (action[0] + 1) * 3.25` m/s²; then apply environment guards |
| Network | Message encoder + 128/128 actor layers; 256/256 centralized critic used only in training |
| Control and limits | 10 Hz, 60 mph ceiling, existing 1.5 s following/stopping guards |
| Training | Four agents; CPU; 49,998 teacher + 449,994 PPO transitions; four BC epochs; 162 PPO updates |
| Learning rates | BC 0.0003; actor 0.0001; critic 0.0003; PPO four epochs, batch 256, clip 0.2 |

**Intended behavior:** cruise, follow, pass slower cars and merge across generated
2–5-lane training roads plus provisional I-110. SUMO handles lateral motion.

**Measured:** 6/6 merges, 12 passes and zero collisions/teleports/overspeed across
12 validation cases; cruise speed 96.59–97.42% of limit. Safety overrides are
18.15% pooled (53.56% worst case); eight cases fail the 5% limit. No held-out
test was run. No camera/LiDAR inputs, physical steering or real-car validation.

**Changes:** new graph-connected merge context (93 fields), feasible teacher/BC
warm start, feasibility rewards and a mixed highway curriculum. The generated
ramp geometry was corrected in a separate version. This actor does not replace
the accepted straight-road v04r2 baseline; their observations are incompatible.

Selected generation: `selected-00162-cfaa8b0a`.
Source: `38c3b41b1dbc6a5a6d4417e8a5f60a958d193bd9`.
Policy SHA-256: `07ef91a2e7591becd235862c9b2ebb0dd5707ef364546ec251f8bf8964e1a6b3`.

The export accepts float32 tensors shaped `[batch,93]` and returns `[batch,2]`.
Feed observations from the matching environment; zeros are only a loading smoke
test. The actor does not include the safety filter, critic or resume optimizer.
The supplied config records this historical authorized run, not permission to
launch new training. See the [full training note](../../../documentation/highway-stage1-training-v02.md).
