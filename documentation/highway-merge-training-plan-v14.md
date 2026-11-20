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

## Physics drive: the policy drives a PhysX car (development)

**User decision (November 19):** "Yes, policy drives it". The 3D videos above are
kinematic replays: each car is drawn at its recorded SUMO pose. Here the policy's car
is driven by rigid-body physics on the LA model instead. Training stays in SUMO.

**How the loop works** (two processes on this machine, one 0.1 s decision at a time):
- **SUMO side (the traffic venv).** SUMO, the v14 environment and the policy run as in
  training. [`environments/highway_flow_rl_v14_physical_env.py`](../environments/highway_flow_rl_v14_physical_env.py)
  intercepts the v14 commands for the physics car (`setAcceleration`, `changeLane`).
  Each decision it sends the commanded acceleration and a reference path to the
  physics side. The path is the lane centerline, or a smoothstep blend between the two
  lanes during a lane change ([`traffic/physical_drive.py`](../traffic/physical_drive.py)).
- **Physics side (Isaac Sim's Python).** [`scripts/physics_drive_server.py`](../scripts/physics_drive_server.py)
  runs PhysX at 120 Hz, 12 steps per decision.
  - **Car.** The prepared Americano (`vehicles/sim_ready/americano-i7-ev/v14`, provisional
    physics: 1,800 kg, front-wheel drive, raycast suspension).
  - **Road.** The LA model `la_10_110_flat.usdc`, referenced unchanged. The stage adds
    three overrides on top and never writes to the model file:
    - the model's own physics scene is switched off;
    - its 154 asphalt deck colliders are added to the car's ground group;
    - asphalt gets a friction entry in the car's tire table (0.75).
  - **Controller.** A low-level controller ([`traffic/physical_controller.py`](../traffic/physical_controller.py))
    turns the decision into drive torque, brake torque and steering:
    - speed: feedforward plus feedback, with a grade term and a learned resistance bias;
    - steering: Stanley-style at the front bumper, plus curvature feedforward with
      understeer and rear-slip terms computed from the asset's tire stiffness.
  - **Limits.** 700 Nm drive per front wheel and 1,500 Nm brake per wheel. These are the
    values the project's earlier Americano probe scripts used. The vehicle package
    itself declares no torque limit.
- **Shadow.** The car's physics pose goes back into SUMO each decision (`moveToXY`,
  readback error 0 m). So the background traffic, the policy's observations and v14's
  checks all see where the physics car really is.
- **Link.** A local socket on 127.0.0.1 with a per-run token and length-prefixed JSON
  ([`traffic/physics_link.py`](../traffic/physics_link.py)).
- **What is not physics.** Background cars are SUMO's and are drawn at their SUMO
  poses. Only the policy's car is physics-driven.

### Stage 1: scripted commands on the empty road

[`scripts/run_physics_drive_stage1.py`](../scripts/run_physics_drive_stage1.py) runs the
southbound Adams corridor (`mlk_on` to `sb5`) with no traffic. A fixed schedule of
accelerations (+2, hold 60 mph, −3, +1, 0 m/s²) and three lane changes goes through the
same path builder, link and controller. Run `outputs/test-runs/physics-stage1-06-rendered`
(uncharged development check):

| Measure | Result |
|---|---|
| Length | 261 decisions, to 26.1 s; top speed 26.81 m/s (60 mph) |
| Acceleration error, decisions under the torque limit | mean 0.066 m/s² |
| Acceleration error, all decisions | mean 0.167, max 1.39 m/s² |
| Decisions at the torque limit | 91 of 261 |
| Lateral error outside lane changes | mean 0.046 m, max 0.20 m |
| Lane changes (v14 completion rule) | 3 of 3: 3.5 s, 4.8 s, 4.7 s (SUMO's own: 3 s) |
| Path error during lane changes | max 0.14 m |
| Shadow placement error | 0 m |
| Wheels off the deck | 4 decisions, 10.2–10.5 s (all four at 10.3 s) |

Rendering at 10 Hz did not change the physics: rendered and unrendered runs gave the
same lane-change times and statistics.

**What Stage 1 found:**
- **The car cannot reach +2 m/s² on the ramp.** The ramp climbs about 5%. With 700 Nm
  per front wheel, the car measured about 1.3 m/s² at 23 m/s, with the drive torque at
  its limit.
- **The ramp-to-mainline join has a crest.** The physics pitch and vertical speed show
  the deck going from about +7.5% to −7.5% within about 11 m (not measured on the mesh
  itself). At 26.6 m/s the car left the deck for about 0.2 s.
- **The tires lag about one decision on command reversals.** For example, −3 m/s²
  commanded gave −1.8 in the first decision. The asset's tire longitudinal stiffness is
  low (5,000).
- **Largest steady lateral error: 0.20 m.** It occurs at a 3° corner in SUMO's ramp
  polyline (t ≈ 6 s).

### Stage 2: the policy drives, with traffic, and the video

[`scripts/record_highway_flow_v14_physics.py`](../scripts/record_highway_flow_v14_physics.py)
runs the same episode as the 3D recording above:
- round 19's southbound focused LA merge, seed 6950004, 16 cars, 1 controlled;
- round 19's best checkpoint, deterministic, not retrained;
- the policy's car physics-driven.

An uncharged development run without rendering (`outputs/test-runs/physics-stage2-01`)
and the charged recording with 10 Hz rendering (`outputs/highway-flow-rl/v14/physics-01`)
gave identical results.

| | SUMO drives (3D recording above) | PhysX drives |
|---|---|---|
| Collisions | 0 | 0 |
| Merge entry / lateral merge | 5.5 s / 5.5 s | 5.8 s / 6.1 s |
| Stable merge | passed (13.6 s) | **failed** at 14.1 s: 9 of 81 window decisions flagged, over the 5% cap |
| Target band | 119 of 133 decisions | 122 of 139 decisions (0.878) |
| Interventions | 1.4% | **33.6%** (76 of 226 decisions) |
| Peak jerk | 4.0 m/s³ | **14.1 m/s³** (at 6.5 s, the crest) |
| `main_9`'s hardest braking | −9.0 m/s² at 4.1 s | −9.0 m/s² at 4.3 s |
| Car's speed at 3.0 s | 15.7 m/s | 14.7 m/s |

Physics side of the PhysX run:
- acceleration error, mean 0.090 m/s² overall and 0.028 under the torque limit;
- 59 decisions at the drive-torque limit;
- lateral path error at most 0.12 m;
- no wheel left the deck;
- pitch up to 0.087 rad.

**Why the v14 numbers differ: the car cannot do what SUMO's car can.** v14 flags a
decision (`native_realization_unattributed`) when the car's realized acceleration
differs from the applied command. Under physics, those 74 flags break down as:
- **59: at the torque limit, 0.6–6.4 s.** The policy asks for 1.6–1.7 m/s² up the
  ramp. The car gives about 1.37, with the drive torque at 700 Nm per front wheel.
  From 5.6 to 6.4 s the climb to the mainline steepens to about 8.7% (physics pitch
  0.087 rad), and the car drops to 0.74–1.2 m/s².
- **5: the crest, 6.5–6.9 s.** Over the crest the front (driven) wheels unload, with
  front suspension compression down to 0.028 m. They then grip again:
  - measured acceleration goes 0.74 → 2.15 → 1.30 m/s² against about 1.1–1.75
    commanded;
  - that swing is the 14.1 m/s³ jerk.
- **10: small mismatches of 0.05–0.35 m/s² at other times** (0.2, 0.5, 14.9, 15.3 and
  19.1–19.6 s).

SUMO's road is flat to its car model, and its cars reach whatever acceleration is
commanded up to their limit, so the policy never met the grade.

**The cut-in is unchanged.** The policy still enters in front of `main_9`, and
`main_9` still stops hard. That is a policy behavior, not a physics one.

**Video** (outside Git): `outputs/highway-flow-rl/v14/physics-01/physics-with-graph/physics-with-graph.mp4`.
- The title says PHYSICS, and that the other cars are drawn at SUMO poses.
- Below the speed/target graph, a second strip shows the commanded acceleration (after
  v14's limits) against the acceleration PhysX measured, with ticks where the torque
  limit was reached.

**Ledger.** The recording was charged as one preparation episode and 108.2 s. Totals now:
- 167 of 168 episodes;
- 52,939.6 of 61,200 s;
- decisions unchanged at 6,129,132 of 6,230,000.

The development runs (Stage 1 and the Stage 2 check) were not charged: they test the
physics integration and train or select nothing.

**What this means for running the policy on the 3D road:**
- the car's real acceleration limit depends on grade and speed, and the policy does not
  know it;
- the ramp's crest into the mainline unloads the drive wheels.

Training can stay in SUMO if SUMO's acceleration ceiling for the controlled car follows
the model's grade and the car's torque. That needs the grade along each corridor, which
the model's lane graph has, and a grade-aware ceiling in the v14 limits. It has not
been built.

### Drive limit: SUMO's car gets the physics car's grade-limited acceleration

**User decision (November 19):** "Yes, build it" (a grade-aware acceleration limit).

**What it is.** [`traffic/road_grade.py`](../traffic/road_grade.py) adds a drive ceiling
to v14's acceleration bounds. It is off by default, so earlier v14 runs are unchanged.
It is turned on by `drive_limit=DRIVE_LIMIT` on the environment, or by `v14_drive_limit`
in a configuration.
- **The ceiling** is the force balance of the prepared Americano at full drive torque:

  a_max = (2·700 N·m / r − 4·0.25·v / r² − m·g·sin θ) / (m + 4·1.225 / r²)

  with m = 1,800 kg and r = 0.401728 m. The mass, radius, wheel inertia and wheel
  damping are the vehicle package's values. The 700 N·m is the physics drive's torque
  limit. The ceiling uses the steepest grade the car reaches within 0.2 s, and keeps
  0.1 m/s² of headroom below it.
- **The grade** is the road model's height under the two axles.
  - It comes from the same hash-checked lane-graph lanes as the `--road-surface`
    renderer, sampled every 0.5 m along every lane of the corridor network once.
  - Profiles match the renderer's heights to 0.6 mm.
  - Inside every lane, the SUMO centerlines are within 0.03 m of the model's lanes.
  - Networks without a road model count as flat. The synthetic and curriculum
    networks have no elevation at all.
  - The old I-110 network has SUMO heights but no road model to check them against.
    It also counts as flat. Its SUMO heights are not used, because SUMO holds each
    junction at one height.
- **Where the bound applies.** It joins the existing ceilings everywhere they are used:
  - the applied command;
  - the unsafe-request reward;
  - the override causes (`drive_grade`);
  - the ceiling feature the policy observes. The observation width does not change.

**Checked against PhysX.** Over 137 decisions where the PhysX car was at its drive
limit for the whole 0.1 s, with all wheels down (Stage 1 and the physics recording):
- measured minus predicted was −0.017 ± 0.042 m/s²;
- the model's grade matched the PhysX chassis pitch on the collider mesh within
  +0.0011 ± 0.0034 rad;
- with the 0.1 m/s² headroom, one of the 137 fell short by more than v14's 0.05 m/s²
  realization tolerance (5.8 s, where the ramp steepens into the junction).

**Same southbound episode, round 19's policy (not retrained), uncharged development runs:**

| | SUMO | SUMO + drive limit | PhysX | PhysX + drive limit |
|---|---|---|---|---|
| Collisions | 0 | 0 | 0 | 0 |
| Interventions | 1.4% | 27.3% | 33.6% | 32.9% |
| `drive_grade` / unattributed realization flags | — / 0 | 60 / 0 | — / 74 | 59 / 23 |
| Stable merge | passed | failed (intervention fraction) | failed | failed |
| Peak jerk (m/s³) | 4.0 | 4.0 | 14.1 | 9.8 |
| PhysX acceleration error, mean | — | — | 0.090 | 0.026 |
| PhysX decisions at the torque limit | — | — | 59 | 7 |

**What the table shows:**
- **SUMO with the limit now predicts the physics outcome.** The interventions are of
  the same size and the stable merge fails for the same reason. The difference is that
  the limited requests are now attributed (`drive_grade`) and done as filtered, instead
  of the car silently falling short.
- **Under physics, the commands are now ones the car can carry out.** On the whole climb
  (0.6–5.5 s) there is no unattributed shortfall left.
- **The other LA corridor barely changes.** On the northbound 8-car merge the limit
  binds on 8 of 1,968 decisions. Interventions go from 0.3% to 0.6%, and all four
  merging cars still pass.
- **The northbound focused merge** crashes at 3.5 s with or without the limit (the
  crash recorded above).

**What it does not cover: the crest at the southbound junction.** The model's lane graph
was checked along the car's path:
- through the junction lane the road climbs at 3.2–3.6%;
- at the start of `sb2` the grade between the axles steps to 7.5% and holds about 7% for
  about 11 m;
- within about 8 m it then turns to −5%.

At 19 m/s, cresting a 12-point grade change over about 8 m takes roughly half the
weight off the wheels (about 5 m/s² of upward curvature acceleration). The driven front
wheels unload and then grip again, which causes the remaining shortfalls (5.6–7.2 s) and
the 9.8 m/s³ jerk peak. At Stage 1's 26.6 m/s the same crest exceeds g, and the car left
the deck. A grade limit cannot remove this. It would take a speed limit over the crest
(a vertical-curvature limit) or traction control in the low-level controller. Neither
is built.

**Training.** No v14 configuration exists yet. A v14 round would turn the limit on with
`v14_drive_limit` and still needs the user's budget decision.
