# What currently drives, and what still needs validation

27 October 2026 · Codex for Sam · Read-only source and committed-evidence audit.

**We have a passing learned highway-cruise prototype in SUMO, but not yet a
validated ordinary physical highway driver for the LA twin.** Newer October
25–26 branches materially advance the October 22 status. They do not establish
general sensor-based driving, lane changes, merging, or robust incident response.

## Fresh repository scope

Read upstream branch heads, fetched the relevant refs without switching or
merging, and inspected source/contracts plus committed evidence summaries:

| Branch / pinned head | Current evidence |
|---|---|
| `codex/mappo-override-retraining` / `d12c168c72c3a7441fb274af3f483de1609dac19` | October 26 completed SUMO MAPPO v02 evaluation |
| `codex/mappo-communication-waves` / `d8f2dc8c7bfd1b19176aac145cb33bf987c80583` | Includes v02; adds untrained v03 communication/wave setup |
| `codex/highway-60mph-ppo-traffic` / `ff5aebc72fba6525cf5dafc6e3da3d58abcc8a08` | October 25 prepared three-physical-car probe; repaired native execution pending |
| `codex/rsl-road-hazard-integration` / `1fb5d1d82e434798ec9996262bf4dd34279da73b` | October 22 low-speed physical hazard curriculum and integration results |

At audit start, Sam's `codex/sam-visual-twin` was `636e872`, one documentation
commit ahead of its fork remote `b23f24c`. Upstream `main` was `edb7125`; it is not
a complete picture of development on these branches. No branch was merged or
model deployed by this audit. Exact upstream source snapshots/hashes are local in
`outputs/la-corridor-review-20261027-a/upstream/` and `repo-audit-manifest.json`.

The numbers below are supported by committed experiment records; raw training
journals/checkpoints on the other workstation were not independently reproduced
here. Neither of the two documented checkpoint paths exists in this checkout.
This is a scoped path check, not a claim about all of Sam's machine or other
unpublished work. No GPU/Kit launch or training occurred during this audit.

## Capability by layer

| Capability | Evidence-supported status | What it does not establish |
|---|---|---|
| Learned ordinary-speed cruise in traffic | SUMO MAPPO v02: four controlled cars, fixed lanes, acceleration-only action; 20/20 reported held-out episodes pass | Steering, lane selection, merges, physical tires/actuators or sensor perception |
| Physical lane/hazard PPO | Low-speed 3 m/s task with simulator-state observations and engineered braking-target assistance; development curriculum successes reported | One frozen candidate passing full-scene highway driving; prior transfer failed after 6.659 m |
| Physical highway-speed integration | Three-car CIRCLES/PPO probe configured with 60 mph ceiling; CPU checks pass; repaired native run deferred | Actual 60 mph driving, reactive learned traffic, or validated following |
| Forward camera/lidar perception | Separate scripted native probe reported 8.52 m travel, 2.57 m obstacle clearance, no contact/departure, camera-fault braking/recovery | A sensor-trained PPO/MAPPO policy or full side/rear situational awareness |
| Explicit communicating policy | v03 channel, actor and matched no-message configuration implemented; no-learning preflight verified | Training completed, reduced traffic waves, or communication benefit |

### SUMO cruise is the strongest current learned-driving result

The [v02 record](https://github.com/hudsonkaneko/hivemind-traffic/blob/d12c168c72c3a7441fb274af3f483de1609dac19/documentation/sumo-mappo-v2.md)
and its compact evidence JSON report 1,000,000 training decisions and 15,554
optimizer steps. Twenty new held-out 60-second episodes had zero collisions,
overspeed decisions and safety overrides across 48,000 controlled decisions.
Mean progress was 1,445.453 m per car, about 53.9 mph average; peak 26.066 m/s,
about 58.3 mph. The target is 25 m/s (55.9 mph), with a 60 mph ceiling.

Matched native traffic traveled 1,580.629 m per car, so the learned controller
made about 8.55% less progress. Passing the safety/override gate is useful,
but not evidence of better traffic efficiency. V01 and v02 used different test
partitions; do not present their difference as a paired-seed causal estimate.

Source inspection corroborates the narrow interface: 27 local features,
acceleration in [-4.5, +2.0] m/s², exact TraCI ego/neighbor state, fixed lanes,
local filtering and SUMO safety support. “No intervention” in these trials is
not proof of safety outside the fixture. This policy controls longitudinal
traffic motion; SUMO supplies lane tracking and vehicle-motion abstraction.

Documented policy path (not present locally):
`outputs/sumo_mappo/v01/train-20261026T190003Z-9d133a8c/policy.pt`.
The directory retains a historical `v01` name; identify the actual version by
its resolved configuration/hash, not that directory name.

### Physical ordinary driving is still the integration gap

The [physical hazard record](https://github.com/hudsonkaneko/hivemind-traffic/blob/1fb5d1d82e434798ec9996262bf4dd34279da73b/documentation/rsl-road-hazard-integration.md)
contains development successes for clear/static/crossing cases using successive
checkpoints, alongside the full-scene learned lane departure after 6.659 m. A
scripted full-scene controller completed 180 m. Preserve both results; a scripted
pass does not repair the learned transfer failure. The actor takes 42 simulator
features and steering/pedal actions; source `safe_target_speed` supplies an
engineered obstacle-aware target. That is not a raw-sensor learned driving stack.

The [October 25 highway probe](https://github.com/hudsonkaneko/hivemind-traffic/blob/ff5aebc72fba6525cf5dafc6e3da3d58abcc8a08/documentation/highway-circles-60mph.md)
uses CIRCLES longitudinal control plus scripted steering on the main car. Two
PPO cars retain their trained **3 m/s** target and empty object slots, hence
nonreactive lane-only behavior. The original native attempt ran zero physics
steps; the repaired run is pending. Its 60 mph label is a configured ceiling,
not observed achievement. Earlier CIRCLES physical checks had 6/7 limited
actuation passes but **0/7 strict stopping successes**, including one contact.

The [sensor input audit](https://github.com/hudsonkaneko/hivemind-traffic/blob/d8f2dc8c7bfd1b19176aac145cb33bf987c80583/documentation/sensor-input-audit.md)
explicitly separates perfect simulator state from estimated sensing. Its newer
scripted perception probe is useful groundwork. It has not replaced current
trained policy observations. A front-only rig cannot truthfully supply every
side/rear neighbor feature; an unseen sector cannot be encoded as an empty road.

### Communication is prepared, not proven

The [v03 record](https://github.com/hudsonkaneko/hivemind-traffic/blob/d8f2dc8c7bfd1b19176aac145cb33bf987c80583/documentation/sumo-mappo-v3.md)
adds 10 Hz messages, 150 m range, 0–300 ms delay, 10% loss, expiration and masked
slots, with 69 actor inputs and a same-shaped no-message counterpart. Both new
training runs are unstarted in the audited published state. Preflight controller
results do not establish a learned communication benefit. This is still a small
four-car braking-pulse fixture, not an empirical LA merge study.

## Route to a realistically regular car

For background traffic realism, start with heterogeneous calibrated SUMO human
drivers. RL is not required for every regular car. Fit speed/headway/lane-change
and gap distributions and queue dynamics to local observations; retain human
and ramp outcomes. A homogeneous fleet of copied PPO weights is not evidence
of realistic human traffic.

For a physical controlled car, the next milestone is **one frozen driver passing
a specified ordinary-driving envelope**: lane/speed tracking, braking, following
a changing-speed leader, cut-ins and stopped traffic, followed by ramp merging.
Measure completion, lane containment, collisions, gap/headway, acceleration/jerk,
interventions and controller latency on independent cases. Expand speeds only
after calibrated actuator/steering response and lower-speed acceptance.

Keep navigation/map access legitimate and sensor inputs explicit. Separate
perception, planning/behavior, steering/speed tracking and safety arbitration;
learning can occupy only the modules where it helps. A coordinated acceleration
policy can sit above a verified ordinary driver without learning every low-level
skill from scratch. Preserve requested, filtered and realized actions in tests.

Use the selected LA geometry with a scripted drive first to diagnose road/contact
errors. Then evaluate frozen learned control. Preserve the approved SUMO traffic
owner and bounded Isaac response model; these branches do not justify replacing
the whole traffic study with an all-physical fleet. Agree the training protocol
and budget with Sam before launching any further learning.
