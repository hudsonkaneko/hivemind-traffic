# MAPPO v02: reduce speed-ceiling overrides

October 26, 2026. The user authorized correcting the override problem and
retraining, with no ongoing chat analysis or monitoring while training runs.

The prior held-out action journal contains 48,000 learned decisions. All 36,296
overrides were local speed-ceiling clamps; none were SUMO or headway overrides.
During overridden decisions, mean requested acceleration was 0.166946 m/s² and
mean realized acceleration 0.000465 m/s². The policy kept asking to accelerate
at the ceiling; it was not primarily failing to respond to nearby cars.

The separate `experiments/configs/sumo_mappo_v2.json` retains the 60 mph ceiling,
raw acceleration action, 27 local observations, centralized critic and every
safety/intervention check. The 5% override acceptance threshold is unchanged.
Training now rewards progress up to a cruise target of 25 m/s (55.923 mph),
with no further progress reward above it. An additional `cruise_excess` term is
`clip((speed - 25) / (26.8224 - 25), 0, 1)`, weighted -2.0. The binary intervention
weight changes from -0.1 to -1.0; entropy coefficient from 0.01 to 0.001.
V01 configurations retain their original reward behavior.

This is fresh MAPPO training, not an optimizer resume under changed rewards or
a hidden scripted action governor. Budget: 1,000,000 decisions / 7,200 seconds,
four CPU workers, original network and optimizer settings otherwise unchanged.
Runtime seed 5102; training episode seeds 200000–299999; validation 310001–310010;
test 410001–410020; preflight 520001/520002. The new test set is disjoint from
the old held-out set used to diagnose the problem. Later comparison of rewards
between v01/v02 is invalid without accounting for the changed reward function;
progress, speed, collisions and the unchanged intervention metric remain useful.

Configuration digest:
`172171c4eaf04105c616b8ab430176eb6b6df61d5a8a5a9034151f35de9be0b8`.
The no-learning preflight `preflight-20261026T185849Z-d23d0453` passed with zero
optimizer steps. Existing tests passed; the new reward ordering test initially
used an exact-zero assertion affected by float32 action quantization. It now
uses a small physical deceleration and checks that cruise outranks the ceiling
and that a positive request at the ceiling still records an override. All 19
environment tests pass after this correction; the other 57 selected checks
passed in the preceding suite. Existing failures and raw artifacts are retained.

Authorized launch from traffic Python:

```text
python experiments/run_sumo_mappo.py --config experiments/configs/sumo_mappo_v2.json --mode train --approved-config-sha256 172171c4eaf04105c616b8ab430176eb6b6df61d5a8a5a9034151f35de9be0b8
```

The historical `outputs/sumo_mappo/v01/` runner directory name also contains
v02 packages; use each package's resolved config and study name to identify its
version. No result is claimed before the run completes. Google guide training
updates remain pending; this repository note is the current experiment record.

## Completed result

Run `train-20261026T190003Z-9d133a8c` completed all 1,000,000 decisions and
15,554 optimizer steps in 737.625 seconds. Sources remained unchanged during
execution. The assistant waited for process completion without inspecting
progress logs or performing ongoing analysis while learning ran.

The selected actor passed all 20 held-out episodes on the new test partition:
zero collisions, zero overspeed decisions and **zero safety overrides across
48,000 decisions**. Mean progress was 1,445.453 m per car over 60 seconds
(about 53.9 mph average), with peak speed 26.066 m/s (58.3 mph). The unchanged
5% override and 600 m progress acceptance gates pass. This result is limited to
the fixed-lane SUMO fixture, not merging, unexpected obstacles or physical cars.

Matched native traffic averaged 1,580.629 m per car, about 8.55% more progress
relative to native's distance; the learned policy trades speed for ceiling
margin. Scripted control averaged 1,581.755 m with 11.8875% overrides. V01's
75.6167% override result used a different held-out seed partition, so it is
historical context rather than a paired-seed improvement estimate. Reward
comparisons across versions are invalid because v02 changed the reward.

The new best actor is `outputs/sumo_mappo/v01/train-20261026T190003Z-9d133a8c/policy.pt`.
Best/latest full checkpoints and raw evaluation evidence remain alongside it.
The old policies and failures remain unchanged. See the compact
[v02 evidence](evidence/sumo-mappo-v2-20261026.json). No further training was
launched after this passing result. Google guide synchronization remains pending.
