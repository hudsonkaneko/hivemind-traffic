# Highway merge training v14: the policy is the only controller

Started by Claude on November 19, 2026, after the
[v13](highway-merge-training-plan-v13.md) development check. It replaces v13's
approach.

**User direction (November 19):**
- "what do you mean after sumo releases the car. we should have one controller on
  the car at all times and not be swapping".
- Then "Yes, policy only": turn SUMO's checks off for the controlled cars, and keep
  the environment's own limits (speed limit, following gap, lane end, jerk) as part
  of the car's controller.

No training round is declared yet.

## What was swapping

From v02 to v13, every controlled car got the policy's command every 0.1 s, but
SUMO's own checks ran on top of it. The cars used speed mode 31, which SUMO's
documentation (`doc/userdoc/TraCI/Change_Vehicle_State.html`, SUMO 1.27.1) calls
"default (all checks on)":
- regard safe speed;
- regard maximum acceleration and deceleration;
- regard right of way at intersections;
- brake hard to avoid passing a red light.

They also used lane change mode 512: no SUMO lane changes of its own, but SUMO
enforces safety gaps on the policy's lane requests.

At the LA yield merges, SUMO's right-of-way check braked the car to the line and held
it there until SUMO judged the gap clear, whatever the policy asked for. In round 19's
starting validation, SUMO changed the realized acceleration on 575 of 26,507
decisions. 524 of them were in the four LA merges, 156 of 394 decisions in the
focused northbound merge alone. The other 51 were in the dense LA following case.
v13 then made the merge target follow SUMO's yield decision. That builds on the
override instead of removing it.

There was no other hand-over during an episode:
- **No warm-up.** The policy commands every car from the first decision (native
  warm-up is 0 s, `stabilization.native_warmup_actual_seconds = 0.0`).
- **Drain only after the horizon.** Cars still on the road at the scored horizon are
  handed to native SUMO to leave the network (`finish_accounting`). That drain is
  never counted as a learned merge or route success.

## The v14 control rule

[`environments/highway_flow_rl_v14_env.py`](../environments/highway_flow_rl_v14_env.py):
- **SUMO's checks off.** After reset, every controlled car gets speed mode 96 ("all
  checks off") and lane change mode 0 (no SUMO lane changes, no SUMO checks on the
  policy's requests). SUMO then moves the car exactly as commanded.
- **The environment's own limits are unchanged:**
  - the speed-limit, following-gap and lane-end acceleration ceilings
    (`acceleration_constraints`);
  - the comfort rate limit;
  - the [-4.5, 2] m/s² range;
  - the lane-change mask and runway checks.

  They use only the car's own state, map and neighbors.
- **Checked at every decision.** The runner refuses any controlled car whose modes are
  not the declared ones. The v12 runner's fixed `31/512` check is now a named setting
  (`NATIVE_MODES`, same default).
- **The drain keeps native SUMO.** v14 restores speed mode 31 before the drain, as
  every earlier version drained.

## Probe: the current policy with SUMO's checks off

A development probe outside the ledger, with no training. It ran round 19's 14
validation cases, the v12 target, round 19's starting policy, and modes 96/0.

First run (`outputs/test-runs/policy-only-probe1-stopped`): it stopped at the
first collision, in the focused northbound LA merge. From its journal:
- The gap plan chose "behind_rear", falling in behind `main_13`, the mainline car
  closing from behind. Its target was 16–20 m/s.
- The car crossed the yield line at 13.3 m/s, 0.3 m ahead of `main_13` in the
  shared stop-line projection. `main_13` was 9.4 m/s faster.
- `main_13` braked at 9 m/s², and the two collided in the junction at t = 3.5 s.

The second run (`outputs/test-runs/policy-only-probe2`) measured all 14 cases
without stopping, compared with round 19's starting validation:

| | Round 19 (SUMO checks on) | Policy only |
|---|---|---|
| SUMO overrides of the command | 575 | 0 |
| Pooled interventions | 2.32% | 0.10% |
| Peak jerk in the four LA merges (m/s³) | 43.3–58.5 | 4.0 |
| Gates passed | 8 of 14 | 9 of 14 (LA dense following now passes) |
| Pooled merge speed match | 0.381 | 0.516 |
| Collisions | 0 | 2, both in the focused northbound LA merge (one crash, two cars) |

The eight non-LA cases are unchanged in interventions and jerk. No lane request was
refused.

## Why the target had to change: it ignored the yield line

v04's gap plan (`traffic/highway_flow_rl_v04_rewards.py`) scores each candidate
speed by the gaps at the end of its 3, 6 or 10 s horizon. That assumes the car can
wait beside the slot, as it can on an acceleration lane. At a yield junction, the car
enters the mainline the moment it crosses the line. With SUMO holding the car, this
never mattered. With the policy in control, it gives a target that sends the car
through the line in front of `main_13`.

## The v14 target rule

[`traffic/highway_flow_rl_v14_rewards.py`](../traffic/highway_flow_rl_v14_rewards.py).
It applies only at yield-junction entries; every other merge keeps v12's target
exactly.

1. **Crossing check.** Each gap-plan candidate must also keep its gaps from the moment
   it crosses the line through v04's 3.1 s maneuver window, checked every 0.1 s with
   v03's `required_gap`. A candidate that stops before the line never crosses and
   needs no crossing gap. Away from conflicts, the plan is v04's.
2. **Reachable target.** Once a car's target comes from a yield entry, the target
   changes no faster than v05's stable-target limits (+1.5 / −2.0 m/s²). That
   continues until it meets the plan's target after the line. Map and occupied-lane
   stop bounds still apply at once. This fixes v13's "after release" gap: a car
   starting from the line gets a target it can reach.
   - **A rise starts from the larger of the previous target and the car's speed.**
     v05 started it from the previous target alone. The first smoke check
     (`outputs/test-runs/v14-smoke1-ramp-from-target`) showed why that is wrong. The
     southbound focused car crossed at 15.7 m/s, but its target then crawled up from
     about 6 m/s. For 10 s on the mainline the target sat up to 13 m/s below the car,
     which would reward braking on the mainline.
   - A slow car cannot hold the target down, because the rise also starts from the
     previous target.

Everything else is v12's: observation widths, rewards, weights and gate thresholds.
The capped target feeds the four actor target inputs, the band and tracking rewards,
and the gate's speed match. Both target hooks share one computation per car and
step. Every input is the car's own map and neighbor context. SUMO's yield-link
state, which v13 used, is not an input.

**Offline replay of the logged crash** (v04 plan → v14 plan at the same logged
contexts):
- t = 0.2 s: 16.3 m/s → 3.8 m/s, crossing at about 4.2 s, after `main_13` has
  passed.
- t = 0.5 s: 17.6 m/s → 2.5 m/s.
- t = 0.8 s: 18.8 m/s → 1.3 m/s.
- From t = 2.0 s (11.7 m from the line at 11.8 m/s), every candidate conflicts: the
  car can no longer stop before the line. The target can only prevent this if the car
  follows it from the start, which is what training has to teach.

## Development smoke check

Outside the ledger, as v12's and v13's were; no training, not a validation. It ran
round 19's 14 validation cases with the v14 environment and round 19's starting
policy, without stopping at a collision (`outputs/test-runs/v14-smoke2`): 25,423
decisions, 120 s. The comparison is with the policy-only probe, which had the same
control and v12's target.

- **Driving is identical in all 14 cases.** The starting policy has zero weight on
  the four target inputs, so only the target and the grading differ.
- **Control verified.** Every controlled car held modes 96/0 at every decision. The
  two target hooks never disagreed (0 mismatches).
- **Unchanged where it should be.** Nine cases are identical in every target, reward
  and gate value: the seven non-LA cases without a yield entry, plus the LA following
  and passing cases. In the I-110 merge (a yield
  entry), the crossing check never changed the plan. The ramp limited the target on
  10 of its 36 yield decisions, so its reward sums differ slightly. Its band share
  (0.61), speed match (0.69) and gate record are unchanged.
- **The LA merges** (policy-only probe → v14, same driving):

| Case | In band | Gate speed match per merging car | Crossing check changed the plan |
|---|---|---|---|
| NB merge 8 of 16 | 0.42 → 0.64 | 0.18, 0.29, 0.61, 0.50 → 0.18, 0.77, 0.62, 0.77 | 84 of 269 yield decisions |
| SB merge 8 of 16 | 0.34 → 0.59 | 0.29, 0.50 → 0.29, 0.99 | 23 of 89 |
| NB focused merge | 0.00 → 0.51 | none (collision) | 17 of 28 |
| SB focused merge | 0.29 → 0.89 | 0.31 → 0.90 | 14 of 30 |

- **Totals.** The pooled merge speed match rose from 0.516 to 0.668, and the pooled
  in-band share from 0.47 to 0.61. Gates stay at 9 of 14 and pooled interventions at
  0.10%, because the driving is the same.
- **Still failing, all four LA merges, on the driving itself:**
  - all four fail the rolling merge-quality gate;
  - in both southbound merges a SUMO background car made an emergency stop
    (`native_emergency_or_missing_accounting`);
  - the northbound focused merge still crashes.

  The untrained policy still crosses in front of mainline cars. Only training on the
  new target can change that.

The first smoke check (`outputs/test-runs/v14-smoke1-ramp-from-target`), with the ramp
rising from the previous target alone, is the one that showed the mainline-braking
problem described under the v14 target rule.

## 3D recording with the speed/target graph

**User request (November 19):** "make a new visual recoridng like the top down sumo one
u made earlier but in 3d with the sim car. make sure to keep the graph though".

**How it was made:**
- **Recording.** [`scripts/record_highway_flow_rl_v14_episode.py`](../scripts/record_highway_flow_rl_v14_episode.py)
  is v12's recorder with the v14 environment installed. It ran one deterministic
  episode of round 19's southbound focused LA merge
  (`v12-val-la-sb-adams-model-merge-focused-1of16-medium`, seed 6950004). The policy is
  round 19's best checkpoint (`outputs/highway-flow-rl/v12/round-19/best.pt`, the
  migrated round-16 policy), not retrained.
- **Ledger.** The recording was charged to the shared ledger as one preparation
  episode and 4.8 s. The ledger now stands at 6,129,132 of 6,230,000 decisions,
  52,831.4 of 61,200 s, and 166 of 168 episodes.
- **3D render.** `scripts/stage_highway_flow_isaac_replay.py` and
  `scripts/render_i110_merge_replay.py` replayed the recorded motion on the LA model
  (`la_10_110_flat.usdc`):
  - follow camera on the sim car (the prepared Americano), 0.1–18.0 s, 10 fps;
  - no lanes drawn from the network, because every lane of the model-exact road is
    the model's own. A test frame showed the car on the model's ramp surface.
- **Graph.** [`scripts/compose_highway_flow_3d_graph.py`](../scripts/compose_highway_flow_3d_graph.py)
  puts the top-down video's graph under each 3D frame: speed against the merge target
  with the ±2 m/s band, the 8 s stable-merge window, and the merge events.
- **Where it is.** The video is outside Git:
  `outputs/highway-flow-rl/v14/replay-01/3d-with-graph/3d-with-graph.mp4`.

**What it shows:**
- **0.1–1.5 s: the car does not follow the target.**
  - The v14 target asks the car to slow to 2.6–4.6 m/s and fall in behind `main_9`.
    It would have crossed the line at about 4.1 s, after `main_9` passed.
  - The policy accelerates at about +1.7 m/s² instead. It has zero weight on the
    target inputs and has not been trained on them.
- **From 1.6 s: too late to yield.** At 13.3 m/s the car can no longer stop before the
  line, and every option conflicts. The target then rises from the car's own speed.
- **3.1 s: it cuts in front of `main_9`.**
  - The car enters the junction at 15.8 m/s, about 16 m ahead of `main_9` (24.2 m/s).
  - `main_9`, a SUMO car, brakes at 9 m/s² from 3.2 s to 4.1 s, slowing to 15.2 m/s.
- **After the line, the merge itself is clean:**
  - 0 collisions, peak jerk 4.0 m/s³, 1.4% interventions;
  - in the band on 119 of 133 target decisions;
  - gate speed match 0.90;
  - stable merge passed at 13.6 s.
- **The case still fails its gates.** The recorder does not grade gates. In the smoke
  check, the same case and seed failed two: native emergency braking
  (`native_emergency_or_missing_accounting`, here `main_9`'s stop) and rolling merge
  quality. The cut-in is the behavior that training on the v14 target has to change.

### The cars jumped in the first video (fixed in the renderer)

**User question (November 19):** "why are the cars like jumping".

**Cause: SUMO's flat merge junction, not the driving.** The renderer placed each car at
SUMO's recorded height (`z`).
- SUMO holds the merge junction at one height for its whole 43 m: 5.56 m, the
  model's height at the merge point.
- The model's road climbs about 5% through that stretch, from 3.9 m.
- So the SUMO lanes step up to the junction height over their last 0.7 m (mainline) or
  4.6 m (ramp).
- In this recording, cars rose up to 1.71 m in one 0.1 s frame. They then floated up
  to 1.65 m above the model's road until the merge point. The sim car was 1.6 m up at
  t = 3.1 s; its shadow shows the gap.
- About 90% of car-frames were within 0.1 m of the model's road. The steps were all
  at the merge.

The driving is unaffected:
- SUMO's documentation lists road slope only for the KraussPS car-following model;
  these cars use Krauss and IDM.
- The sim cars' acceleration is set directly.

Only the drawn height was wrong. The round-19 notes on the model-exact roads had
recorded this height step, but the first video did not account for it.

**Fix:** `scripts/render_i110_merge_replay.py --road-surface <corridor-manifest.json>`.
- **Where the heights come from.** The car heights come from the model lane-graph lanes
  that the corridor manifest names as the corridor's source.
- **Hash checks.** The manifest must match the recording's network hash, and the lane
  graph must match the manifest's hash.
- **Height rule.** A car takes the height of the two nearest lane centerlines,
  inverse-square weighted. So a car on a lane sits at that lane's height, and a car
  changing lanes moves smoothly between the two.
- **Off-road guard.** A car more than 4 m from every model lane stops the render.
- **Unchanged:** cars get no pitch or roll, as before. Renders without the option are
  unchanged.
- **Result.** The largest one-frame height change in this recording falls from 1.71 m
  to 0.20 m. That 0.20 m is a car at about 26 m/s on the model's own climb of about 7%
  just past the merge point.

The re-rendered video is outside Git:
`outputs/highway-flow-rl/v14/replay-01/3d-with-graph-model-road/3d-with-graph-model-road.mp4`.
