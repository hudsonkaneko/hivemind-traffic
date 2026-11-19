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
cuts that layout from the model's lane graph at its two real single-lane CA-110
on-ramps:

| Corridor | Direction | Through lanes | Length | Ramp |
|---|---|---|---|---|
| `la-110-nb-pasadena-onramp` | CA-110 northbound | 3 | 814 m | 376 m |
| `la-110-sb-adams-onramp` | CA-110 southbound | 3 | 877 m | 231 m |

**Real:**
- the through-lane centerlines, in the model's own ENU frame, so SUMO coordinates
  are USD coordinates and replays line up with the 3D scene;
- the 3.66 m lane spacing;
- the deck elevation;
- the ramp alignment.

The compiled through lanes stay within 1.4 cm of the model's lanes.

**Modeled:** the model joins each ramp straight into the right lane, but the merge
task needs an acceleration lane. So a 250 m auxiliary lane is added beside the
right lane, and the last 80 m of the ramp are blended onto it. Speeds are those of
the earlier I-110 corridor (mainline 60 mph, ramp 30 mph). Each corridor's
`corridor-manifest.json` lists what is real and what is modeled.

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

Pending.
