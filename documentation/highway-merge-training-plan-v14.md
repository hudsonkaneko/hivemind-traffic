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

**Training.** Round 20 (below) turns the limit on with `v14_drive_limit`.

## Round 20: v14 with the drive limit

**User decision (November 19, 2026):** "Yes, raise it": raise the study budget so a v14
round can train with the drive limit on.

[`traffic/highway_flow_rl_v14_contract.py`](../traffic/highway_flow_rl_v14_contract.py)
declares it, and [`scripts/prepare_highway_flow_rl_v14.py`](../scripts/prepare_highway_flow_rl_v14.py)
writes `configs/training/highway-flow-rl-v14-round20.json`. **Round 20 is round 19's
configuration** (pinned by checksum) with only these changes:
- **Environment:** v14. The policy is the only controller, and yield-entry targets
  check crossing gaps.
- **Drive limit on.** LA rows take the grade from the road model. Every other network
  has no road model and counts as flat, where the drive ceiling is 1.71–1.80 m/s².
- **Collision rule.**
  - Round 19 stopped at any collision in any episode. The v14 starting policy already
    collides in validation case 10 (the focused northbound LA merge), so that rule would
    stop round 20 at its initial validation.
  - Round 20 instead stops at a periodic validation that is a **safety regression**
    against its own initial validation: a case collides that did not collide at the
    start, or the validation has more collisions in total.
  - Training-rollout collisions keep their −20 reward and are recorded in
    `updates/*.json`; they do not stop the run by themselves.
- **Periodic validation every 50,000 decisions** instead of 100,000. Round 19's
  acceleration-lane merges had already broken by its first check at 106,915 decisions,
  so checking twice as often keeps a checkpoint from before such a break.
- **Budget:**
  - total decisions raised by 100,000 to 6,330,000, so a round-19-sized round of
    200,000 decisions fits (100,868 remained);
  - diagnostic episodes raised by 2 to 170, for this round's preflight and one physics
    recording of its result;
  - wall time is not raised. 8,260.4 s remain, enough for the preflight, a 3,600 s
    round and the recording.

Everything else is round 19's: the 20 training rows (with round 20's seed offset), the
14 validation cases, the warm start (the round-16 policy), the rewards, the anchor and
shield losses, the optimizer and the intervention and merge-floor stop rules.

**Preflight:** passed (`outputs/highway-flow-rl/v14/round-20-preflight`, 2.8 s). The
drive limit was live (5 of 64 decisions limited, steepest grade 6.5%), and every car
held SUMO modes 96/0.

### Round 20 result

**Completed with no stop rule triggered, and no improvement over the start.**
- **Training:** 194,856 decisions and 2,148.2 s, in 112 episodes (33,763 of them
  critic-only warm-up). Learning stopped when the next complete episode no longer
  fit the 200,000-decision budget.
- **Best checkpoint:** still the initial validation, i.e. the migrated round-16
  policy. Nothing was promoted.

| Validation | Gates (of 14) | Pooled interventions | Merge speed match | In band | Drive-limited decisions | Collisions |
|---|---|---|---|---|---|---|
| Initial | 8 | 0.86% | 0.485 | 0.53 | 147 | 2 (case 10) |
| Update 31 (~50k) | 8 | 0.70% | 0.381 | 0.38 | 108 | 2 (case 10) |
| Update 56 (~100k) | 8 | 1.54% | 0.319 | 0.29 | 306 | 2 (case 10) |
| Update 91 (~150k) | 8 | 1.60% | 0.337 | 0.23 | 305 | 2 (case 10) |
| Final | 7 | 1.54% | 0.245 | 0.15 | 204 | 2 (case 10) |

**What the drive limit changed at the start.** With the limit on, the starting policy
fails the 120 m ramp-start case (case 4), which it passed with the limit off. The road
there is flat, but the Americano's drive gives at most about 1.73 m/s² at that speed.
The car runs out of ramp and brakes for the lane end (53 overrides). It fails on jerk,
interventions and headway.

**Better by the end:**
- **Southbound focused LA merge (case 11).**
  - worst-car interventions 27.3% → 18.5%;
  - merge speed match 0 → 0.70;
  - `drive_grade` filters 60 → 43.
- **120 m ramp (case 4).** Interventions 12.7% → 6.0%. The `drive_grade` filters
  fell from 54 to 0: the policy now asks only for what the drive gives. The case still
  fails.

**Worse by the end:**
- **I-110 merge (case 0).** Speed match 0.48 → 0.12; interventions 1.3% → 10.7%; it
  now also fails on peak jerk.
- **4-lane 180 m ramp start (case 7).** Pass → fail, on peak jerk; speed match 0.94 →
  0.47.
- **LA 8-car merges.**
  - Northbound (case 8): speed match 0.62 → 0.18.
  - Southbound (case 9): 0.49 → 0.00. Its `drive_grade` filters rose from 25 to 153,
    so on that case the policy learned to ask for more than the drive gives, not less.

**Safety.**
- Validation case 10 (the focused northbound LA merge) collides at every validation,
  as at the start: 2 collisions. No other case collided, so the regression rule never
  fired.
- 2 of the 112 training episodes had a collision:
  - update 7, in the critic-only warm-up, on the focused northbound merge;
  - update 75, on the northbound 8-of-24 merge.

**Physics check** (`outputs/highway-flow-rl/v14/physics-02-round20`; charged: 1
episode, 54.9 s). Round 20's final policy drove the PhysX Americano on the same
southbound case and seed as `physics-01`, with the drive limit on as trained.

| Southbound focused merge, seed 6950004, PhysX | Round-16 policy, no limit (`physics-01`) | Round-16 policy + limit (development run) | Round 20 final, limit on |
|---|---|---|---|
| Collisions | 0 | 0 | 0 |
| Interventions | 33.6% | 32.9% | 25.5% |
| Peak jerk (m/s³) | 14.1 | 9.8 | 5.5 |
| Commanded vs PhysX acceleration, mean error (m/s²) | 0.090 | 0.026 | 0.020 |
| Decisions at the torque limit | 59 | 7 | 0 |
| Stable merge | failed | failed | failed |

- **The stable merge still fails.** It is marked failed at the end of its 8 s window,
  not earlier, so it did not fail on a collision, headway or emergency.
- **Likely cause (inferred).** 13 of the 83 decisions after the merge entry had
  overrides (16%; the window allows 5%). All 13 are unattributed acceleration
  shortfalls at 5.9–7.3 s, on the crest described above. The recorder does not save
  the window's failure reason, so this is inferred, not read.
- The video is `physics-with-graph/physics-with-graph.mp4` in the run folder.

**What it means.**
- **Where it trained on the southbound ramp, the policy learned to ask for what the car
  can do.** Under physics, that merge is now much smoother: peak jerk 5.5 against the
  gate's 5, and no torque saturation.
- **But PPO again made other merges worse.** This time the I-110 merge, one ramp-start
  case and both LA 8-car merges degraded; in round 19 it was the ramp-start cases. Two
  rounds in a row, this training setup improved the case it targeted and degraded
  others.
- **Not established: why the multi-car merges degrade.**
  - One mismatch was found. The v14 yield-entry target rises at +1.5 m/s² whatever
    the drive ceiling is, and on the southbound ramp that ceiling is about
    1.1–1.4 m/s². So the target pulls ahead there, by about 0.1–0.4 m/s each second.
  - That is too small, by itself, to explain the southbound 8-car in-band share
    falling from 0.46 to 0.01.

**Budget after round 20.** The shared ledger has 6,323,988 of 6,330,000 decisions
(6,012 left), 55,145.6 of 61,200 s and 169 of 170 diagnostic episodes. No further round
fits without another budget decision.

### Why round 20's cars stopped (diagnosis, no training)

**User questions (November 19–21, 2026):** "why did interventions go up by so much",
then "Yes, design a fix". Pooled interventions went 0.86% → 1.54% (222 → 416). Almost all
of the rise comes from cars that braked to a full stop on open road:
- **at standstill, still asking to brake:** `native_realization_unattributed` 1 → 141
  (SUMO cannot slow a stopped car, so the command is not realized);
- **climbing back from a stop up the southbound ramp:** `drive_grade` in case 9 25 → 153.

Method: round 20's saved validation observations were fed back through its saved
actors. They reproduce the recorded requests (2,456 of 2,550 decisions in case 9, and
every decision of the braking onset).

**Cause: a feedback loop in the actor.**
- Each acceleration request is mostly the car's own previous request and acceleration
  (actor inputs 1 and 5; the request is 0.5–1.0 × the previous one) plus a small
  correction from what the car sees. With those two inputs zeroed, the southbound lead
  car (case 9) asks for only −0.1 to −0.3 m/s², instead of building up to −4.2.
- On that ramp the round-16 actor has two settling points from a neutral start:
  **+1.13 m/s²** (it accelerates) and −3.46. From update 56 on, only about −3.8 to −4.0
  is left. A −0.25 m/s² shift in the correction moved it across.
- In the I-110 merge (case 0) and the 4-lane ramp (case 7), the settling point moved
  from −0.02…+0.28 m/s² to −0.36…−1.74 m/s².
- The cars braked while their merge target was 14–20 m/s and nothing was ahead.

**Why training did not correct it:**
- v06's avoidable-braking cost never applies during a pending merge: its diagnostic
  requires a target with no obligations, and every pending merge carries
  `merge_or_yield` (acceleration lanes also `ending_lane`).
- The v12 band cost is capped at 4 per decision once a car is 9 m/s below target. That
  cap explains weak recovery once stopped, not the onset. (The actor does see its own
  speed separately, so it can tell a stop from a crawl.)
- The v12 target inputs are nearly unused. After round 20 their first-layer weights
  are 0.03–0.09, against a median of 1.43 for the other inputs. Round 20 started from
  the round-16 actor, where they were zero.

**Not established:** why the correction drifted downward. One untested candidate is the
drive limit in southbound training (1,217 of 40,882 decisions clipped, each charged as
an intervention).

## Round 21: the stop fix

**User decision (November 21, 2026):** "Yes, build and train". The choice noted that it
implies raising the decision budget.

**The fix** ([`traffic/highway_flow_rl_v14_stop_fix.py`](../traffic/highway_flow_rl_v14_stop_fix.py);
optional in the v14 environment, off unless `stop_fix=` is given; reward version
`highway_flow_rl_merge_v14_merge_braking`):
- **`merge_braking`.** v06's avoidable-braking cost, with its formula and thresholds,
  for the decisions v06 leaves out:
  - a pending merge whose target carries only `merge_or_yield` and/or `ending_lane`;
  - the target at least 0.5 m/s above the car's speed (the target already slows the car
    for merge gaps, the ending-lane runway, the unseen-yield stop and v14's crossing
    check);
  - a safety ceiling of at least 0.2 m/s², no own-lane leader within 1.5 s, and no
    crossing deficit in the yield-entry plan;
  - cost 0.25 × min(1, (−0.2 − request) / 2) per decision, for requests below −0.2 m/s².
  Decisions without obligations stay v06's, so nothing is charged twice.
- **`target_band_tail`.** Past v12's cap the band keeps a quarter of its slope. A stopped
  car with a 20.1 m/s target costs 5.54 per decision instead of 4.
- **Stop counts in every v14 report (`v14_stops`).** Stopped decisions, those asking to
  brake (and their overrides), and free-road stops: below 0.1 m/s, no own-lane leader
  within 20 m, and an eligible target of at least 5 m/s. The intervention metric is
  unchanged.

**Offline and development checks (uncharged):**

| Check | Result |
|---|---|
| `merge_braking` replayed over round 20's initial validation | 0 charges |
| …over its final validation | 588 charges (−55.9), mostly in the decisions before the stops |
| Case 8 (northbound 8-car), which never stops | 83 new charges. They are the same drift: a car braking 10.5 → 4.2 m/s on the ramp, and later 14.4 → 2.6 m/s, while its target is 11–20 m/s |
| Live SUMO run of round 20's case 9, with the fix on and round 20's final actor | reproduces the trace (198 overrides, same causes); 98 `merge_braking` and 314 tail charges, 57 free-road stopped decisions, as in the replay |

**Round 21 is round 20's configuration** (same rows, seeds, warm start, optimizer,
drive limit and validation cases), with only:
- the stop fix on (`v14_stop_fix`) and its reward version;
- **a free-road stop rule** at every periodic validation. It stops when free-road
  stopped decisions exceed both 20 and twice this round's initial validation. Round 20
  had 0, 0, 25, 64 and 180, so this rule would have stopped it at update 56;
- **budget:** total decisions raised by 200,000, to 6,530,000 (6,012 remained). Wall time
  (6,054.4 s left: a 3,600 s round and the 900 s final reserve) and diagnostic episodes
  (one left, for the preflight) are not raised. No physics recording of the result fits
  without another episode.

**Risks:**
- The merge target can be wrong at a yield line. The new cost would then discourage a
  real yield. It is at most 0.25 per decision, against 20 for a collision, and plans
  with a crossing deficit are exempt.
- The feedback loop itself stays in the actor; the cost pushes against the drift.

### Round 21 result

**Completed with no stop rule triggered. It is the first v14 round that did not make
other merges worse.**
- **Training:** 193,714 decisions and 2,126.8 s, in 112 episodes (32,312 decisions of
  critic-only warm-up). Learning stopped when the next complete episode no longer fit
  the 200,000-decision budget.
- **Free-road stop rule:** never fired. Free-road stopped decisions were 0, 0, 3, 0
  and 0 against a limit of 20. Round 20 had 0, 0, 25, 64 and 180.

| Validation | Gates (of 14) | Pooled interventions | Merge speed match | In band | Stable merges (of 12) | Free-road stops | Collisions |
|---|---|---|---|---|---|---|---|
| Initial | 8 | 0.86% | 0.485 | 0.53 | 9 | 0 | 2 (case 10) |
| Update 32 (~50k) | 8 | 0.77% | 0.464 | 0.42 | 9 | 0 | 2 (case 10) |
| Update 56 (~100k) | 8 | 0.88% | 0.354 | 0.29 | 9 | 3 | 2 (case 10) |
| Update 91 (~150k) | 8 | 0.61% | 0.471 | 0.47 | 9 | 0 | 2 (case 10) |
| Final | 8 | 0.46% | 0.461 | 0.34 | 10 | 0 | 2 (case 10) |
| *Round 20 final* | *7* | *1.54%* | *0.245* | *0.15* | *8* | *180* | *2 (case 10)* |

The initial validation is round 20's exactly: same starting policy, rows and seeds.
The largest peak jerk across cases fell from 44.7 to 30.4 m/s³. Drive-limited decisions
fell from 147 to 64.

**By case, start → round 21 final (round 20 final in brackets):**

| Case | Change | Gate |
|---|---|---|
| 0, I-110 merge | Speed match 0.48 → 0.45 (0.12); interventions 4 → 4 (68) | Still fails on rolling merge quality |
| 4, 120 m ramp start | Interventions 111 → 46 (53); `drive_grade` filters 54 → 3 (0); peak jerk 44.7 → 30.4 | Still fails |
| 7, 4-lane 180 m ramp start | Unchanged: passes, speed match 0.94 (round 20 broke it: fail, 0.47) | Passes |
| 8, northbound LA 8-car merge | Interventions 12 → 0 (17); speed match 0.62 → 0.50 (0.18) | Still fails |
| 9, southbound LA 8-car merge | Worst-car interventions 9.6% → 1.7% (20.8%); `drive_grade` filters 25 → 7 (153); speed match 0.49 → 0.23 (0.00) | No longer fails on comfort and control. Still fails on rolling merge quality and on one background car's emergency braking |
| 10, focused northbound LA merge | Unchanged | Still collides (2 cars), as at the start |
| 11, focused southbound LA merge | Speed match 0 → 0.74 (0.70); worst-car interventions 27.3% → 24.0% (18.5%); `drive_grade` filters 60 → 54 (43) | Its merge is now stable. It still fails on interventions, rolling merge quality and native-emergency accounting |

**Update 56 dip.** At update 56, case 9 relapsed:
- 107 interventions, 3 free-road stopped decisions, speed match 0 and peak jerk 57.1;
- it recovered by update 91.

**Best checkpoint: update 56, by the runner's rank.**
- **What the rank uses:** safety events come first. Then come failed cases, completed
  routes, quality merges, seconds stopped before merging, and worst-car interventions.
- **Why update 56 ranks first:** case 9's one background emergency braking did not
  happen there. That event is `main_11` on `sb1_0` at 6.3–6.6 s, at 9 m/s². It is
  missing exactly when that case was at its worst.
- **The final policy ranks second.** It still ranks above the starting policy:
  - stopped before merging, 0.4 s against 1.1 s;
  - worst-car interventions, 24.0% against 27.3%.
- **The final policy is better than update 56 on everything in the table above, and on
  peak jerk (30.4 against 57.1).** Their worst-car interventions are about the same:
  24.0% against 23.6%.
- Neither is promoted. This is development validation only, and there is no held-out
  evaluation.

**Training rollouts.**
- **Free-road stops still occur.** The rollouts use sampled actions and had 1,707
  free-road stopped decisions in 112 episodes. 615 of them were on one training case,
  `train-4-lane-ramp120-merge-focused-1-ramp-started` (6 episodes).
- **Not diagnosed:** why. Round 20 did not measure stops in training, so there is
  nothing to compare.
- **New terms charged in training:** `merge_braking` −460.5 and `target_band_tail`
  −4,852.0. In validation they were small: up to 34 `merge_braking` decisions per
  validation.
- **One training episode collided** (update 12, during the critic-only warm-up, on the
  southbound LA 4-of-16 merge): `agent_0` and `agent_1`, twice.

**What it means.**
- **The fix did what it was for.** Deterministic validation had no free-road stops.
  The cases round 20 broke kept most of their starting behavior: cases 0, 7 and 8.
  Case 9 kept part of it.
- **Not established: how much of this comes from the stop fix.** The two rounds differ
  only in the stop fix and the stop rule, and the rule never fired. But this is one run
  per round, and PPO trajectories diverge once the rewards differ, so run-to-run
  variance is not ruled out.
- **Still failing:** every LA merge case. The gate count did not move (8 of 14), and
  merge speed match ended a little below the start. Speed match was lost on the 8-car
  merges (cases 8 and 9) and gained on the focused southbound merge (case 11).
- **No physics check.** No diagnostic episode is left to record round 21's policy on
  the PhysX car.

**Budget after round 21.** The shared ledger has:
- decisions: 6,517,702 of 6,530,000 (12,298 left);
- wall time: 57,275.2 of 61,200 s (3,924.8 s left);
- diagnostic episodes: 170 of 170.

No further round or recording fits without another budget decision.

## Round 22: a drive-aware reward target

**User decision (November 22, 2026):** "Yes, raise it", to "Raise the study budget for
round 22 and a physics check?".

### Why the LA merges fail rolling merge quality

Every case that fails in round 21's final validation fails `rolling_merge_quality`. That
gate needs at least 90% of a case's merging cars to qualify. A car qualifies when, among
other conditions, its speed is within 2 m/s of the merge target for 90% of the 16 s window
around its lateral merge. In the LA 8-car merges (cases 8 and 9), the misses are almost all
cars below their target. Read from round 21's final action journals
([`scripts/summarize_highway_flow_rl_v11.py`](../scripts/summarize_highway_flow_rl_v11.py)):

| Case, car | Window samples | Below target | Above | Speed match |
|---|---|---|---|---|
| 8, `agent_6` | 124 | 111 | 0 | 0.10 |
| 8, `agent_3` | 161 | 80 | 0 | 0.50 |
| 9, `agent_5` | 142 | 128 | 0 | 0.10 |
| 9, `agent_0` | 161 | 98 | 4 | 0.36 |

**The target runs away from the car.** At a yield entry, v14's reachable target
(`traffic.highway_flow_rl_v14_rewards.reachable`):
- starts at min(plan, speed + 1.5 m/s² × 3 s), so up to 4.5 m/s above the car;
- then rises at 1.5 m/s², from the larger of its previous value and the car's speed.

With the drive limit on, the car gives 1.0–1.4 m/s² on the southbound ramp and up to
2.0 m/s² (the environment's cap) on the northbound one. A car 4.5 m/s below a target rising
at 1.5 m/s² closes the gap at best at (a − 1.5) m/s², where a is its acceleration:
- at the 2.0 cap, it stays outside the 2 m/s band for at least the first 5 s;
- at 1.0–1.4 m/s², the gap grows until the target stops rising.

**Two cars cannot qualify, whatever the policy does.** Case 8's `agent_6` and case 9's
`agent_5` start 4.5 m/s below their targets. Their targets never fall back to the car's
speed, so they do not depend on how the car drives. A car that accelerated from the first
decision would match at most:

| Car | At the 2.0 m/s² cap the whole time | At the logged drive ceilings (approximate) | Needed |
|---|---|---|---|
| Case 8, `agent_6` | 63% | about 56% | 90% |
| Case 9, `agent_5` | 79% | about 37% | 90% |

The ceiling column replays the ceilings logged along the policy's own path. A faster car
would be elsewhere on the ramp, so that column is approximate. Each of these cases has
fewer than ten merging cars, so one car that cannot qualify fails the case. **Cases 8 and
9 cannot pass rolling merge quality at these seeds**, whatever the policy does.

**The policy also asks for less than the car can do.** On the southbound ramp, case 9's
`agent_5` asked for 0.1–0.7 m/s² against a 1.3 m/s² drive ceiling. On the northbound ramp,
case 8's `agent_6` asked for about 1.5 against 2.0. A target the car cannot follow gives the
band cost no in-band reference: on that ramp the car is out of the band whatever it asks for.

### The change

[`traffic/highway_flow_rl_v14_drive_target.py`](../traffic/highway_flow_rl_v14_drive_target.py),
optional in the v14 environment (off unless `drive_target=` is given; reward version
`highway_flow_rl_merge_v14_drive_target`). Only at yield entries; every other merge keeps
v12's target exactly.
- **Start:** the reward target starts at min(plan, the car's speed), with no lead.
- **Rise:** min(1.5 m/s², the drive ceiling at the car's lane position and speed). That is
  the same `traffic.road_grade` bound the applied command gets. The rise starts from the
  larger of the previous target and the car's speed, so a car slower than its drive allows
  still falls behind.
- **Fall:** v14's 2.0 m/s². The occupied-lane and map ceilings still apply at once.

**It feeds:** the four actor target inputs; the band, tracking and speed-error rewards; and
the stop fix's `merge_braking` and band tail.

**It does not feed the gate.** These keep v14's target:
- the gate's speed match (rolling merge quality);
- the merge-quality payments graded on the gate's window (v04's `quality_merge` and v10's
  graded quality, 5.0 each);
- the free-road stop measurement.

So gate counts stay comparable with rounds 20 and 21. Each report also grades every merging
car's rolling merge quality on the drive-aware target (`v14_drive_target.quality_merges`).
Each validation summary adds that diagnostic gate count (`v14_drive_target`) next to the
gate. Whether the gate itself should use the drive-aware target is a decision for the user.

**Development check (uncharged):** `outputs/test-runs/drive-target-check-01-case09`. It ran
case 9 in SUMO with the round-22 configuration and the round-16 starting actor:
- **Gate:** identical to round 21's initial validation of the same case: the same failures,
  overrides, quality merges, stable merges, stops and native emergencies. The starting
  actor's target-input weights are zero, so its actions do not depend on the target.
- **`agent_5`:** the reward target starts at 11.99 m/s, the car's speed, against the gate's
  16.49, and rises at 1.28–1.35 m/s². Speed match is 0.94 on the drive-aware target and
  0.19 on the gate's.
- **`agent_0`:** 0.89 against 0.80.
- **Band cost:** −6.6 against −209.9.

### Round 22's configuration

Round 22 is round 21's configuration: the same rows, seeds, warm start (the round-16
policy), optimizer, drive limit, stop fix, stop rules and validation cases. Only these change:
- the drive-aware target and its reward version;
- **budget:** total decisions +200,000, to 6,730,000 (12,298 remained), and diagnostic
  episodes +2, to 172, for this round's preflight and one physics recording of its result.
  Wall time is not raised: 3,924.8 s remain. That covers the preflight, a round of round
  21's length (2,126.8 s), the recording and the 900 s final reserve.

**Risks:**
- **The official gate count can hardly move.** Cases 8 and 9 cannot pass rolling merge
  quality at these seeds, so a better gate count has to come from other cases. The
  diagnostic count shows what the drive-aware target would grade.
- **Less band pressure where the car already follows its drive.** On the southbound ramp,
  the starting actor is nearly in band with the new target. A car slower than its drive
  still falls behind the target and pays the band cost.
- **Following the new target can cost a borderline car the gate.**
  - On the northbound ramp the drive allows up to 2.0 m/s², but the new target rises at
    1.5. A car that follows it stays as far below the gate's target as it started.
  - Case 8's `agent_3` also starts 4.5 m/s below the gate's target. It can qualify only by
    accelerating near the cap: its bound is 91% at 2.0 m/s², and about 89% at the logged
    ceilings.
  - The merge-quality payments still grade on the gate's target, so they still reward
    closing that gap.

### Round 22 result

**Stopped by the free-road stop rule at its first periodic validation.** At update 31
(50,678 decisions, 615.4 s), validation had 21 free-road stopped decisions, against a
limit of 20 (the initial validation had 0). The rule did its job. The round used about a
quarter of its decision budget.

| Validation | Gates (of 14) | Pooled interventions | Merge speed match | Stable merges (of 12) | Free-road stops | Collisions |
|---|---|---|---|---|---|---|
| Initial | 8 | 0.86% | 0.485 | 9 | 0 | 2 (case 10) |
| Update 31 (~50k) | 7 | 1.54% | 0.265 | 7 | 21 | 2 (case 10) |
| *Round 21, update 32 (~50k)* | *8* | *0.77%* | *0.464* | *9* | *0* | *2 (case 10)* |

- **The initial validation is rounds 20 and 21's exactly**, case by case: the same
  overrides, collisions, quality merges, stable merges, override causes and peak jerk. The
  starting actor ignores its target inputs, so the change touched only the rewards.
- **Diagnostic gate count on the drive-aware target:** 8 at the start and 7 at update 31,
  the same as the gate. At the start, case 8's `agent_6` and `agent_3` match their
  drive-aware targets 100% of the window (0.15 and 0.83 on the gate's), but case 8 still
  fails through `agent_2` (0.77) and `agent_0` (0.73).
- **The target changed only on yield entries:** cases 0 and 8–11. Cases 4 and 7 changed
  only through the shared policy's training.

**By case, update 31 against round 21's update 32:**
- **Case 9 (southbound 8-car) broke.**
  - `agent_5` and `agent_0` brake from their first decisions, building to about −4 m/s².
  - They stop on the open ramp at 4.5–5.8 s while both targets are 11–20 m/s (21
    free-road stopped decisions), then climb back up the ramp at the drive limit.
  - 222 overrides, 197 of them `drive_grade`; speed match 0; peak jerk 28.0.
- **Case 7 (4-lane 180 m ramp start) broke:** pass → fail.
  - The car ran out of ramp and stopped at the lane end.
  - 84 stopped decisions (none free-road); 26 `lane_end` and 82 unattributed overrides;
    peak jerk 32.4.
- **Case 4 (120 m ramp start) improved.**
  - Overrides 111 → 0; `drive_grade` filters 54 → 0; peak jerk 44.7 → 3.5.
  - It now fails only on rolling merge quality (speed match 0.28).
  - Its target did not change, so this is the shared policy, not the new target.
- **Case 8 (northbound 8-car):** speed match 0.62 → 0.38.
- **Cases 10 and 11:** unchanged from the start.

**Why case 9 broke: not established.** It is round 20's failure again: from spawn the
actor's request settles into its braking branch.
- **One candidate:** at spawn the drive-aware target equals the car's speed, so the first
  second of braking costs little:
  - the band is free within ±1.8 m/s;
  - `merge_braking` needs the target 0.5 m/s above the car's speed. In case 9 at update 31
    it started charging at 0.7 s.
  - With v14's target, 4.5 m/s above the car, that braking was charged from the first
    decision.
- **Against it:** the training rollouts of updates 1–31 were alike in both rounds:
  - 265 against 249 free-road stopped decisions;
  - `merge_braking` charged on 1,589 against 1,335 decisions.
- This is one run per round, so run-to-run variance is not ruled out.

**Best checkpoint:** still the initial validation, the round-16 policy. Update 31 ranks
lower (7 failed cases). Nothing is promoted, and there is no held-out evaluation.

**Physics check: not recorded.**
- **Nothing new to record.** The round produced no policy better than its start. That
  start, the round-16 policy with the drive limit, already drove the PhysX car on the
  southbound focused merge in the round-20 development run.
- **The last diagnostic episode is unused.**
- **Recording round 21's final policy needs a recorder change.** A recording is charged
  under its checkpoint's own round configuration, and round 21's episode cap (170) is
  already spent.

**Budget after round 22.** The shared ledger has:
- decisions: 6,568,380 of 6,730,000 (161,620 left);
- wall time: 57,893.3 of 61,200 s (3,306.7 s left);
- diagnostic episodes: 171 of 172.

**What it means.**
- **The drive-aware target did not get a clean test.** The round stopped at a quarter of
  its decisions, and the break was in a case where the new target applies.
- **Two findings stand regardless:**
  - cases 8 and 9 cannot pass rolling merge quality at these seeds under the gate's
    current target;
  - the actor's braking branch is still there: a round-16 start can fall into it within
    50,000 decisions.
- **Whether the gate should grade the drive-aware target is still the user's decision.**

## After round 22: troubleshooting (November 23)

**User decision (November 23, 2026):** "uhh save it then continue troubleshooting". The
last diagnostic episode is kept for a round-23 preflight. No round 23 has started.

Every check below is a development check: SUMO only, no training, not charged to the
study ledger. Outputs are under `outputs/test-runs/` in the Claude worktree:
`r22-target-input-check-*`, `r22-basin-check-01-case09` and `v14-follower-check-01` to `-05`.

### Round 22's case-9 stop is the round-16 actor's own braking loop

- **The new target inputs are not the trigger.** Round 22's update-31 actor (`last.pt`)
  was run on case 9 twice:
  - with round 22's inputs it reproduces the validation exactly: 222 overrides, and both
    cars stop;
  - with round 21's v14 inputs it brakes the same way. `agent_5` stops at 4.7 s, and the
    run has 257 overrides.
- **The round-16 actor is already at the tipping point.** Here update 31 drives, and the
  round-16 actor is scored on the same observations at every decision. Its requests are
  within 0.1–0.3 m/s² of update 31's. At 2.2 s, for example, it asks for −3.75 m/s² where
  update 31 asks for −3.89. These are round 20's two settling points again.
- **The weights hardly moved.** Each actor layer is less than 1% (relative norm) from the
  round-16 actor, which is less than round 21's final. Why training moved them toward
  braking is still not established.

### The LA native emergencies are cut-ins by our merging cars

Each LA merge case (8–11) has one native emergency. Every time, it is a background car on
the mainline right lane braking at 9 m/s².
- **In cases 10 and 11 (one car) the merging car had just reached the junction:**
  358.7 m along the northbound ramp, and 192.2 m along the 192.85 m southbound ramp.
- **In case 9 a merging car was 3.5 m into the merge lane.** Case 8 was not traced.
- **Case 11 at round 21's final** (the start policy behaves the same way):
  - from 0.2 to 1.6 s the v14 target says yield (5.0 → 2.4 m/s), because `main_9` is
    approaching;
  - the actor accelerates at the drive limit (+1.3 m/s²) instead;
  - at 1.7 s stopping is no longer possible within the plan's braking allowance, so the
    target switches to crossing (13 m/s, crossing deficit 1.8);
  - the car enters 10.6 m ahead of `main_9`, which is closing at 9.4 m/s, and `main_9`
    emergency-brakes.
- **The reward does not see it.**
  - No term charges a forced emergency brake: after the switch the merging car's
    tracking cost is about 0.05 per decision, and it earns the +5 merge payment.
  - The per-step neighbor-braking sample misses `main_9` inside the junction. It reads
    +0.27 m/s² while SUMO logs −9.
- **Controlled cars do not yield natively.** They run with SUMO speed mode 96, so only
  the policy can yield.

### A car that follows the v14 target (measurement only)

A scripted follower drove every controlled car to check whether the gates can be reached.
It is a measurement only and drives nothing in training.
- **Its rule:** request k × (target − speed), kept within ±0.35 m/s² of the measured
  acceleration and below the feasible ceiling the actor sees; k = 1 or 3 /s.
- **Target:** v14's gate target, or round 22's drive-aware target where one exists.
- **Lanes:** it always keeps its lane.

| Case | Round 21 final fails | Follower fails | Worst merging car's speed match (90% needed) |
|---|---|---|---|
| 8 (NB 8-car) | native emergency, merge quality | merge quality; native emergency and 1–2 overrides in some runs | 0.21–0.46 |
| 9 (SB 8-car) | native emergency, merge quality | native emergency (`main_11`, 6.0 s), merge quality | 0.29–0.34 |
| 10 (NB 1-car) | 2 collisions, native emergency, comfort, stable merge, merge quality | merge quality only | 0.31–0.66 |
| 11 (SB 1-car) | native emergency, comfort, merge quality | merge quality only | 0.15–0.18 |

- **Following the plan's yield removes the collisions and native emergencies in cases 10
  and 11.**
  - In case 11 the follower brakes at about −3 m/s² to 6.6 m/s and lets `main_9` pass.
  - It then enters behind `main_9` with 69 m to the next car.
- **Case 9 keeps one emergency even when the plan is followed.** So its crossing check is
  not enough there.
- **Rolling merge quality is out of reach in all four cases, even for the follower.**
  Graded on the drive-aware target instead, case 9's cars match 100%, but cases 8, 10 and
  11 still fail.
- **Why the gate's target cannot be followed in the yield cases:**
  1. At spawn it is 3.8–5.0 m/s while the car is at 9–11 m/s.
  2. When the mainline car passes (1.2–2.2 s), it jumps up while the car is still braking
     hard.
  3. It then rises at 1.5 m/s² from its own value rather than the car's speed.

  On the southbound ramp, where the drive gives 1.3 m/s², it stays 3–4 m/s ahead until it
  reaches 26.8 m/s. On the northbound ramp the car catches up after about 5.5 s.
- **Not tested:** cases 0 and 4 need a lane change that the follower does not make.

### What it means

- **No policy that follows its target can pass the merge-quality gate in the four LA
  merge cases.** The gate's target is not followable there within the comfort and drive
  limits. So a policy that learns to follow its target cannot raise the count above 10 of
  14 under this gate. I have not shown that no driving at all could pass.
- **Even so, the merge plan's yield works.** Following it removes the collisions and
  emergencies in the one-car LA cases. The policy does not follow it, and no reward term
  charges for forcing a mainline car to brake hard.
- **Changing how the gate grades merge speed is the user's decision.** No round 23 has
  been started.
