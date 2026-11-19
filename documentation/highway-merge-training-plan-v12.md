# Highway merge training v12: target-speed input, band reward, LA I-10/I-110 corridors

Started by Claude on November 18, 2026, after
[v11 round 16](highway-merge-training-plan-v11.md#round-16-result-7-of-8-gates)
(7 of 8 development gates; only the I-110 merge fails, on rolling merge quality).

**User requests (November 18, 2026):**
- "lets add the target input then add a reward with decent weight to focus on
  matching that target speed. it should have some tolerance around it tho so it
  could be between like 60-70 mph instead of only 65 etc. do some training on that
  and report back"
- "the new 110 3d model should be the primary model used in the training and
  simulation. upload a copy of that to the github too once its implemented in the
  latest version"

The November 17 autonomy decision still covers the rounds. Held-out evaluation is
not authorized. Each round is declared in
[`traffic/highway_flow_rl_v12_contract.py`](../traffic/highway_flow_rl_v12_contract.py)
in a commit before it runs.

## Why a target input

Round 16 fails the I-110 merge because the merging car is too slow while the merge
is pending: agent_0 matched the gate's target speed on 0.689 of its window (the gate
needs 0.90), and 49 of 161 samples were too slow. At its lane-change request
(14.4 s) it was at 14.72 m/s against an 18.00 m/s target. The gate grades against a
number the actor never saw: `merge_speed_target`
([`traffic/highway_flow_rl_v04_rewards.py`](../traffic/highway_flow_rl_v04_rewards.py))
computes it from the gap plan and the occupied-lane and map limits. The actor had to
infer it from gaps and speeds.

## The v12 environment

[`environments/highway_flow_rl_v12_env.py`](../environments/highway_flow_rl_v12_env.py)
is the v11 environment with two additions
([`traffic/highway_flow_rl_v12_rewards.py`](../traffic/highway_flow_rl_v12_rewards.py)).
Nothing else changes: dynamics, safety shield, native guards, lane-change mask
(including the v11 merge-speed rule), action contract, gates and every earlier
reward term.

**Inputs.** Four actor inputs are inserted after the 69 local inputs and before the
six message slots. The actor grows from 117 to 121 inputs and the critic state from
2832 to 2928 values.

| Index | Input | Value |
|---|---|---|
| 69 | target_active | 1 while the car's merge is pending and the gate's target is eligible, else 0 |
| 70 | target_speed | target / 26.8224 m/s (the speed input's own scale) |
| 71 | target_error | (target − speed) / 9 m/s, clipped to ±1 |
| 72 | target_in_band | 1 when \|speed − target\| ≤ 1.8 m/s |

All four are 0 when no target applies. They fade like the v06 merge inputs, so a
0-to-1 change takes 10 decisions (1 s). The safety guard, the mask and the
journaled `observation117` keep the raw 117 inputs.

**Reward (`target_band`).** The cost is zero inside ±1.8 m/s of the gate's target.
Outside that band it rises linearly by 1 per band width, capped at 4 per decision:

    target_band = −1.0 × min(4, max(0, |speed − target| − 1.8) / 1.8)

It uses the same pre-decision target and post-decision speed as the inherited
`merge_tracking` term, and only while the merge is pending.

The tolerance is ±1.8 m/s (±4.0 mph). For a 65 mph target the band is 61–69 mph,
close to the 60–70 mph the user described. It sits just inside the gate's own
±2.0 m/s (±4.5 mph) match window, so a car resting at the band's edge still counts
as matched.

The weight is 1.0, four times the inherited `merge_tracking` weight (0.25):

| Speed error | target_band (per decision) | merge_tracking (inherited) |
|---|---|---|
| 3.6 m/s | −1 | −0.45 |
| 9 m/s or more | −4 | −1.0 |

## Primary road model: LA I-10/I-110

The user's model is committed at [`highway_usd/la_10_110`](../highway_usd/la_10_110)
([provenance](../highway_usd/la_10_110/PROVENANCE.md)). It is generated from
OpenStreetMap; keep the ODbL attribution.

The environments need the corridor layout described in
[`traffic/highway_v05_demand.py`](../traffic/highway_v05_demand.py): through lanes,
an on-ramp, an auxiliary lane, and the lane drop.
[`scripts/build_la_10_110_corridors.py`](../scripts/build_la_10_110_corridors.py)
cuts that layout from the model's lane graph at two single-lane CA-110
on-ramps:

| Corridor | Direction | Through lanes | Length | Ramp |
|---|---|---|---|---|
| `la-110-nb-pasadena-onramp` | CA-110 northbound | 3 | 814 m | 376 m |
| `la-110-sb-adams-onramp` | CA-110 southbound | 3 | 877 m | 231 m |

**Taken from the model, not checked against the real road:**
- the through-lane centerlines, in the model's own ENU frame, so SUMO coordinates
  are USD coordinates and replays line up with the 3D scene;
- the 3.66 m lane spacing, which the model applies uniformly (its README calls it
  the Caltrans standard, not a measurement);
- the deck elevation, which the model's solver computed from OSM bridge/layer tags,
  not measured;
- the ramp alignment.

The compiled through lanes stay within 1.4 cm of the model's lanes. That shows the
corridors match the model, not that the model matches the road.

**Invented, unverified (correction, November 18):** the model joins each ramp
straight into the right lane. Without checking the real ramps, a 250 m auxiliary
lane was added beside the right lane, and the last 80 m of the ramp were blended
onto it. The speeds (mainline 60 mph, ramp 30 mph) were copied from the earlier
I-110 corridor, not looked up for these sites. These values were assumptions, and
round 18 was trained on them. They must be verified before further LA training.

Each corridor's `corridor-manifest.json` lists what is taken from the model and
what is modeled. Its word "real" means "taken from the model".

The networks are built deterministically: a rebuild reproduces the checksums pinned
in the contract.

## Round 18

| Setting | Value |
|---|---|
| Warm start | round-16 pass-2 actor (`outputs/highway-flow-rl/v11/round-16/candidates/checkpoint-02.pt`), migrated by [`policies/highway_flow_v12.py`](../policies/highway_flow_v12.py) |
| Learning | buffered PPO, 4 episodes per update, actor lr 5e-5 (round 15: 1e-4), target KL 0.05 (round 15: 0.1) |
| Losses | v11 shield-consistency loss (50) with the speed-limit-aware band; anchor to the warm start |
| Critic warmup | 30,000 decisions (actor frozen) |
| Budget | 400,000 decisions, 5,400 s wall |
| Validation | every 100,000 decisions; best kept by the shared validation rank |
| Training rows | 20: 12 on the LA corridors (8 merges), 8 kept from round 16 |
| Validation rows | 14: round 16's 8, unchanged, plus 6 on the LA corridors (4 merges) |

**Warm start.** Every weight is copied, with zero weights on the new inputs. The
migrated actor and critic therefore start with exactly the round-16 outputs,
whatever the new inputs hold.

**Anchor.** On every decision with no active merge target (target_active = 0), the
mean request and lane outputs are held to the frozen warm start:

    10 × (request − warm request)² + 1 × mean (lane − warm lane)²

PPO is free only where the new reward applies. Round 15 showed what happens
without an anchor: at this scale PPO drifts everywhere (5 of 8 gates fell to 2 of 8).

**Stop rules:**
- Any collision stops the run.
- A periodic validation stops the run if its pooled intervention fraction exceeds 5%.
- A periodic validation also stops the run if its merge speed match falls more than
  0.25 below this round's own initial validation (never below a 0.1 floor). The LA
  rows have no earlier score, so the static 0.3 floor is only recorded.

**Budget.** The shared ledger before v12 stood at 5,627,084 training decisions,
48,250.8 s and 162 diagnostic episodes. The v11 study budget allows 6,230,000
decisions, 61,200 s and 168 episodes. Round 18 fits inside it, and no budget is
raised.

### Round 18 result

**No improvement. The best checkpoint stays the starting point:** the migrated
round-16 policy, scored by the initial validation. Training used 395,133 decisions
and 3,628 s. There were no collisions, and no stop rule fired.

| Validation | Gates passed (of 14) | Merge speed match | In band | Interventions | Stable merges |
|---|---|---|---|---|---|
| Initial | 8 | 0.567 | 0.389 | 1.84% | 12 of 12 |
| Update 56 | 7 | 0.577 | 0.402 | 1.97% | 12 of 12 |
| Update 116 | 7 | 0.570 | 0.427 | 1.56% | 12 of 12 |
| Update 176 | 6 | 0.523 | 0.416 | 1.93% | 11 of 12 |
| Final | 5 | 0.473 | 0.419 | 2.36% | 10 of 12 |

**Round 16's 8 cases:**
- **I-110 merge.** Its speed match moved between 0.50 and 0.89 (0.689 at the start;
  the gate needs 0.90). At updates 56 and 176 it reached 0.869 and 0.889, but
  another car's jerk failed the case.
- **Lane-end stop.** In the final policy the I-110 car waits and stops at the lane
  end: it starts its merge at 0 m/s with 2 m left, and has 36 lane-end overrides.
  At the start it merged at 14.7 m/s with 216 m left.
- **Ramp-start cases.** The three ramp-start cases each picked up a jerk failure.

**The LA corridors:**
- **All four LA merges fail** at every validation.
- **The starting policy already stopped at the lane end.** Six of the eight LA
  merging cars wait until the end of the 250 m auxiliary lane, stop, and merge
  from 0–2 m/s.
- **Training fixed one of them.** The southbound focused case now merges at 18.9 m/s
  with 111 m left: no overrides, peak jerk 3.9. Only rolling merge quality fails
  (match 0.447).
- **The LA passing case still fails.**

**Likely cause: the target is a stopping speed near the lane end.** This is a
hypothesis: the journal values below are measured, but the mechanism has not been
tested. It rests on the invented 244 m lane length.
- `merge_speed_target` keeps the `ending_lane_runway` bound
  ([`highway_flow_rl_v03_rewards.py`](../traffic/highway_flow_rl_v03_rewards.py)):
  the car must be able to finish a 3.1 s maneuver and then stop before its lane
  ends, so the target is at most √(4 × (d − 3.1 v − 2.5)).
- **The bound applies until the lane change completes**, because the ending lane
  stays occupied during the maneuver.
- **At 60 mph it needs 266 m of runway.** The modeled LA auxiliary lane is
  244 m long, so in that lane the target always falls below mainline speed, and
  toward zero near the end.
- **An example from the recorded case.** Throughout the lane change (13.0–15.0 s,
  at 19.4–20.7 m/s), the journaled target was 0–8 m/s, and target_band paid its
  full −4 on every decision.
  - From 13.8 s the ending-lane bound of the still-occupied auxiliary lane
    accounts for the fall from 8.0 to 0.
  - The zeros before 13.8 s come from another bound, not yet isolated.
  - Once the lane change completed (15.2 s), the target jumped to 25.9 m/s.
- **What the reward pays for.** The band reward therefore pays the car to slow
  down near the lane end instead of taking a gap. This fits what happened: the
  in-band fraction rose while the merge match fell, and the I-110 car learned to
  stop at the lane end.

Proposed directions:
- Grade and reward only the traffic-matching part of the target, the gap plan
  without the ending-lane bound.
- Or apply the band only while that bound is not binding.
- Measure the real acceleration-lane lengths before more LA training (see below).

**Video.** `scripts/record_highway_flow_rl_v12_episode.py` re-ran validation row 11
(southbound focused merge) with the final policy as one diagnostic episode. Its
result equals the final validation of that row. Rendered outputs, not in Git:
- top-down: `outputs/highway-flow-rl/v12/replay-01/topdown/topdown.mp4`;
- 3D on the LA model: `outputs/highway-flow-rl/v12/replay-01/isaac-render/la-110-sb-adams-merge.mp4`.

The 3D render uses `scripts/render_i110_merge_replay.py --road-model
highway_usd/la_10_110/la_10_110_flat.usdc --modeled-lanes sb2_0 sb3_0 mlk_on_0`. It
is a replay of SUMO motion, not physics, and the modeled lanes are drawn as a lighter
overlay.

**Budget after round 18.** The shared ledger has 6,022,217 of 6,230,000 decisions,
51,888 of 61,200 s and 164 of 168 diagnostic episodes. A further 400,000-decision
round does not fit, so the next round needs a budget decision from the user.

**Open question on the road model.** The user asked that the 3D model be based on
satellite images of the real road. The current model is built from OpenStreetMap,
and has no acceleration lane at either corridor's on-ramp. Whether the real ramps
have one, and how long it is, has not been checked. That length decides whether
merging at mainline speed is possible under the ending-lane rule above.

## Round 19: the LA roads exactly as the model has them

**User decision (November 18, 2026):** "dont need to verify, just use that road for
the training" ... "the exusting 3d model we have".

[`scripts/build_la_10_110_corridors_exact.py`](../scripts/build_la_10_110_corridors_exact.py)
rebuilds both corridors from the model's lane graph alone. The v1 builder and its
networks are unchanged, so round 18 stays reproducible.

**From the model (checked in the build):**
- **Through lanes.** The three CA-110 lanes and their 3.66 m width. The compiled
  lanes are within 1.4 cm of the model's lanes.
- **The ramp.** Its centerline, unchanged (within 6 mm). It ends where the model
  ends it: at the start of the merge way's right lane, which the model lists as the
  successor of both the ramp lane and the right lane. **The model has no
  acceleration lane, so neither do these roads.**
- **Speed limits.** The model's `speed` values: 29.1 m/s (65 mph) mainline and
  20.1 m/s (45 mph) ramp. Round 18 had used 60 and 30 mph.

**What SUMO needs that the model does not state:**
- **The merge.** It is a SUMO junction at the model's merge point. The ramp link
  yields to the right lane (state `m`), as on the existing I-110 corridor.
- **Visibility.** The ramp link declares 100 m, the I-110 corridor's value. The
  environments require one, and the model has no sight-distance data.
- **Edge split.** The road after the merge is split into four equal edges, because
  the environments use the edge names sb2–sb5. This changes no geometry.
- **Speed cap.** Every vehicle in these environments is capped at 60 mph (vType
  `maxSpeed`), so mainline traffic drives at most 60 mph on the 65 mph road.
- **Flat junction.** SUMO keeps the junction area at one height (the merge point's).
  The ramp and mainline climb about 1.7 m into it, so recorded heights step at the
  junction edge. The traffic models used here ignore grade.
- **Ramp start.** The traffic generator is unchanged. It places ramp cars 25–55 m
  before the end of the ramp (`traffic/highway_v05_demand.py`).

**Development smoke check (outside the ledger; no training, not a validation).**
The round-18 warm start ran on the new roads:
- **All rows.** Every one of the 18 LA rows reset without error.
- **Merge episodes.** Four full merge episodes had no collisions.
- **Merges completed, poorly.** Every merging car completed its merge, but the
  single-car merges had 35–41% interventions and peak jerk above 50 m/s³.
- **Why.** The car brakes hard to the yield line, waits there (11 s in the
  southbound case), then accelerates from a stop. Its speed match was 0.

The policy has only ever merged from an acceleration lane.

**Round 19 = round 18 with the LA rows on these roads.** It keeps the same rows,
seeds, warm start, rewards, losses, stop rules and validation cases.
- **Budget:** 200,000 decisions and 3,600 s.
- **The shared ledger after round 18:** 6,022,217 of 6,230,000 decisions,
  51,887.8 of 61,200 s, 164 of 168 episodes. Round 19 fits, and nothing is raised.

### Round 19 result

Pending.
