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
