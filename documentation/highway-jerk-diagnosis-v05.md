# V05 step: peak-jerk diagnosis from saved v04 traces

**Handled on November 11, 2026.** This is a new step, separate from v04
training. It diagnoses why the final v04 actor still fails the 5 m/s3 peak-jerk
gate, using only the saved final-validation traces. No simulation, training,
policy promotion or held-out evaluation was performed in this step.

Branch: `highway-jerk-diagnosis-v05`, based on `highway-merge-rewards-v04`
(commit `5b8e2c1`). Historical v04 code, checkpoints and results are unchanged.

## Description

The [v04 terminal review](highway-merge-rewards-v04.md#completed-long-run---november-10-2026)
left one stated next action: find the cause of the remaining jerk peaks (fleet
maximum 50.76 m/s3) and the specific failed cars before any further training.
This step answers that question for the six saved development cases.

[`scripts/diagnose_highway_flow_rl_v05_jerk.py`](../scripts/diagnose_highway_flow_rl_v05_jerk.py)
reads every `actions.jsonl.gz` journal under
`outputs/highway-flow-rl/v04/long-round-03/final-validation/` and labels each
sample whose |realized jerk| exceeds 5 m/s3 with exactly one primary cause, in
priority order: `startup` (the car's first controlled decision),
`safety_override`, `policy_filter`, `native_realization`, `override_release`
(control returns the step after an override), `lane_change`,
`policy_request` (the actor's own request swung) and `other`. The labels describe
saved trajectories. They are not causal or counterfactual claims.

```text
python scripts/diagnose_highway_flow_rl_v05_jerk.py --run outputs/highway-flow-rl/v04/long-round-03 --output outputs/highway-flow-rl/v05/jerk-diagnosis-01/jerk-diagnosis.json
```

Evidence: [jerk-diagnosis.json](evidence/highway-jerk-diagnosis-v05/jerk-diagnosis.json)
(SHA-256 `879e52bd9274ab9c23787cabef7b66c9b852eefc9aa3907692b07a3200872443`),
with input trace hashes per case. Tests:
[`tests/test_highway_flow_rl_v05_jerk_diagnosis.py`](../tests/test_highway_flow_rl_v05_jerk_diagnosis.py).

## Findings

26 samples across 16 cars exceed 5 m/s3.

| Primary cause | Samples | Largest abs jerk (m/s3) |
| --- | ---: | ---: |
| Startup (first controlled decision) | 16 | 12.11 |
| Safety override (native realization, unattributed) | 4 | 37.40 |
| Policy request swing | 3 | 8.46 |
| Lane change in progress | 2 | 8.37 |
| Release after override | 1 | 50.76 |

1. **Startup is the most common failure.** 11 of the 16 failing cars exceed the
   limit *only* on their first controlled decision. The trace's jerk at that step
   equals the first realized acceleration divided by 0.1 s: the car goes from the
   insertion state (0 m/s2) to the actor's first command in one step. This is a
   real one-step change in SUMO, but it comes from the control handoff, not from
   driving behavior.
2. **The 50.76 m/s3 fleet maximum is SUMO's own braking, not the actor.** In
   `v02-development-train-2-lane-ramp120-merge`, focal car `agent_0` on on-ramp
   `mlk_on_0` requested about +1.6 m/s2 at 1.7 s. SUMO instead realized -0.76 and
   then -4.5 m/s2 (maximum braking) for 1.1 s, with no follower and no lane-end
   bound nearby, and flagged it `native_realization_unattributed`. Speed fell from
   10.7 to 6.2 m/s. At 2.8 s control returned and acceleration jumped from -4.5 to
   +0.58 m/s2 in one step (50.76 m/s3). The entry and release (-23.6, -37.4 and
   +50.8 m/s3) are the three largest peaks in the fleet. Two I-110 merge cars
   (`agent_2`, `agent_5`, -13.3 and -12.1 m/s3 at 0.3 s) show the same
   unattributed native braking right after startup.
3. **Policy-owned peaks are moderate.** Only 3 failing cars have a peak caused by
   the actor's own request or a lane change, at most 8.46 m/s3. These include the
   I-110 focal car (-7.0 m/s3 at 3.8 s, +8.4 m/s3 during its lane change at 13.5 s).

## What this means for the next step

- Rate-limiting the actor's requests alone cannot clear the fleet maximum,
  because the largest peaks happen while SUMO overrides the request. The
  pre-existing draft below confirms this: with its rate limiter, the frozen v04
  actor still reaches 51.05 m/s3.
- The startup handoff needs a defined policy: either a ramped first command or a
  gate that measures jerk only after the first controlled decision. Either
  choice changes the acceptance rule, so it needs the user's agreement first.
- The unattributed native braking on `mlk_on_0` is explained below: it is a SUMO
  junction rule, not a policy failure.

## Native braking on `mlk_on_0` explained (November 11, 2026)

**Cause:** SUMO's approach rule for a non-priority junction link. In the saved
network, ramp `mlk_on_0` (104.66 m) ends at priority junction `n1` and enters
`sb2_0` through connection `:n1_3_0` with link state `m` (minor, must yield).
SUMO requires a car on a minor link to stay able to stop at the stop line until
it is within the link's visibility distance (default 4.5 m; the networks set no
`visibility` attribute). This holds even when the link is open and no conflicting
car exists.

**Evidence:** a bounded, headless, open-loop SUMO 1.27.1 replay
([`scripts/replay_highway_flow_rl_v05_native_braking.py`](../scripts/replay_highway_flow_rl_v05_native_braking.py))
fed the saved applied accelerations back into the saved network/routes with
seed 8400004. No policy, training or reward was involved.

- The replay reproduces the saved realized acceleration exactly at all 39
  steps up to the first lane request (|difference| <= 0.001 m/s2).
- From 1.4 to 3.0 s the link is open (`isOpen=True`, `hasFoe=False`), there are
  no junction foes, and no highway car is in `sb1_0`, the only conflicting lane.
  Every main-road car is in `sb1_1` or `sb1_2`.
- Braking starts at 1.7 s, the first step where the car's stopping distance at
  4.5 m/s2 (12.62 m) exceeds its distance to the stop line (12.19 m). SUMO then
  holds maximum normal braking (-4.5 m/s2), keeping stopping distance just above
  the remaining distance.
- Braking ends at 2.8 s, when the remaining distance (3.39 m) first falls inside
  the 4.5 m visibility distance. The full request returns in one step, giving
  the 50.76 m/s3 peak.
- **Counterfactual:** with only speed-mode bit 3 (right of way at junctions)
  cleared (mode 23 instead of 31), the same commands produce no braking at all.
  The car enters the junction at 11.5 m/s instead of 6.2 m/s. This mode was
  used only for diagnosis, never for control.

Evidence: [native-braking-replay.json](evidence/highway-jerk-diagnosis-v05/native-braking-replay.json)
(SHA-256 `93d81c299964d77aad5d2ff11d5811b2763e83bd935378beeeeb26a0037b0bbb`).
Test: [`tests/test_highway_flow_rl_v05_native_braking_replay.py`](../tests/test_highway_flow_rl_v05_native_braking_replay.py).

```text
python scripts/replay_highway_flow_rl_v05_native_braking.py --episode outputs/highway-flow-rl/v04/long-round-03/final-validation/case-04/episode-8400004-67e660a5 --seed 8400004 --output outputs/highway-flow-rl/v05/native-braking-01/replay.json
```

**Scope:** all six ramp networks in `outputs/highway-environment/v05/network-bank/`
(train 2/3/4/5-lane, validation 2/4-lane) join the ramp through the same `m`
connection with default visibility. Those networks supply 10 of the 20 v04
training rows. Any ramp car that reaches the last ~13 m of the ramp faster than
it could stop there gets this braking. The actor's analytical guards and
masks do not model the rule, which is why the trace logged it as
`native_realization_unattributed`. Not checked here: the I-110 `candidate`
network, and the 0.3 s braking of I-110 cars `agent_2`/`agent_5`, which ran with
a `following` bound active and may be a different mechanism.

**Options for the user to decide on** (none implemented; each changes the
environment or the actor contract):

1. Change the network so ramp cars can see the junction earlier: a larger
   connection `visibility`, or a zipper/acceleration-lane merge instead of a
   priority junction. This changes the training/evaluation networks.
2. Give the actor the rule: add a minor-link approach bound to the analytical
   guards/mask, so the policy learns to arrive slowly enough and brakes smoothly.
3. Keep the network and gates, but attribute this event to the native junction
   rule in the jerk and intervention metrics, so it is not counted as a policy
   comfort failure.

## Draft material absorbed into v05

An undocumented, uncommitted v05 draft ("continuity v05") was found in the main
checkout. Its files are dated November 10, 2026, 23:43-23:51, before this step
began. With the user's agreement on November 11 it is kept as starting material
for v05, not as a result of this step:

- `traffic/highway_flow_rl_v05_rewards.py`, `traffic/highway_flow_rl_v05_contract.py`,
  `environments/highway_flow_rl_v05_env.py`, `experiments/run_highway_flow_rl_v05.py`,
  `scripts/prepare_highway_flow_rl_v05.py`, `scripts/run_highway_flow_rl_v05_job.py`,
  `tests/test_highway_flow_rl_v05.py` and `configs/training/highway-flow-rl-v05-pilot.json`
  (`training_authorized: false`).
- Opt-in hooks in `traffic/highway_flow_rl_v04_rewards.py` (gap-target
  hysteresis) and `environments/highway_flow_rl_v04_env.py` (overridable target
  methods). Default v04 behavior is unchanged; the v04 tests pass.
- Its acceleration conditioning limits normal jerk to 4 m/s3 per 0.1 s step and
  lets maximum braking bypass the limit.
- Its saved outputs under `outputs/highway-flow-rl/v05/` (offline audit, preflight
  and a frozen comparison) used no optimizer steps. The frozen comparison of the
  v04 final actor gave: legacy 50.76 m/s3 maximum, 0.42% interventions, 0/6
  cases; continuity 51.05 m/s3 maximum, 0.79% interventions, 2/6 cases. The
  `full-final` variant has no summary and appears incomplete.

The draft was never reviewed or authorized for training. Training any v05
configuration still needs the user to agree on the configuration first.

Google engineering-guide synchronization is pending for this step.

## Corrected ramp networks: acceleration lane starts at the ramp (November 11, 2026)

**Why:** on a real on-ramp the ramp becomes the acceleration lane and the car
merges into traffic from that lane; it never has to yield at the gore. The v05
bank instead starts the right auxiliary lane at the beginning of the road
(`sb1_0`, 375+ m upstream), so the ramp joins an existing lane as a yield link.
That layout, not the driving task, caused the gore braking above.

**What changed:** [`scripts/build_highway_flow_rl_v05_ramp_networks.py`](../scripts/build_highway_flow_rl_v05_ramp_networks.py)
reuses the historical geometry (same lane counts, lengths, bends, speeds, ramp
start points and lane drop at `n3`) with two differences:

1. `sb1` carries only the through lanes. The auxiliary lane `sb2_0` is fed by
   the ramp alone, and `sb1` lane i continues into `sb2` lane i+1.
2. The ramp's last stretch runs parallel to the road on the auxiliary lane's
   line (up to 60 m, 5 m short of the junction). Without this, removing the
   yield exposed a second artifact: a ~24 degree corner where netconvert capped
   the short connector at 11.24 m/s and SUMO braked the replayed car at -4.5
   m/s2 (first corrected build, `ramp-networks-01`, 44.99 m/s3).

All six layouts compile with the ramp link at state `M` (priority), no foes and
no yield. Ramp connectors allow 17.88-20.12 m/s (19.00 m/s on validation
roads), at least as much as the historical 17.88 m/s connector. Ramps are longer
because of the parallel approach (e.g. 133.4 m instead of 104.7 m on
`train-2-lane-ramp120`). Bank checks: [ramp-started-bank.json](evidence/highway-jerk-diagnosis-v05/ramp-started-bank.json)
(SHA-256 `80046001fbce880437c75fc1f136d5c677ca09b3501f2c52be896e8cfe64937f`).
The historical bank, builder and checks are unchanged.

```text
python -m scripts.build_highway_flow_rl_v05_ramp_networks --output outputs/highway-flow-rl/v05/ramp-networks-02
python scripts/replay_highway_flow_rl_v05_native_braking.py --episode outputs/highway-flow-rl/v04/long-round-03/final-validation/case-04/episode-8400004-67e660a5 --seed 8400004 --window 0.2 14 --network outputs/highway-flow-rl/v05/ramp-networks-02/train-2-lane-ramp120/network.net.xml --main-lane-shift -1 --output outputs/highway-flow-rl/v05/native-braking-03-ramp-started/replay.json
```

**Replay test** (same saved commands, seed and traffic; open loop up to the
car's first lane request at 10.7 s; mainline departures moved down one lane
index to match the new `sb1` numbering):

| Network | SUMO overrides | Lowest realized accel | Largest jerk |
| --- | --- | ---: | ---: |
| Historical v05 bank | 1.7-2.7 s, gore yield braking | -4.50 m/s2 | 50.76 m/s3 |
| Ramp-started, sharp corner (`-01`) | 3.4-4.0 s, connector turn-speed braking | -4.50 m/s2 | 44.99 m/s3 |
| Ramp-started, parallel approach (`-02`) | 4.0-5.0 s, held at ramp speed limit | 0.00 m/s2 | 17.95 m/s3 |

On the final corrected network SUMO never brakes the car. The only remaining
difference is that the replayed commands keep accelerating past the ramp's
30 mph (13.41 m/s) legal limit, so SUMO holds the car at the limit. This is an
open-loop artifact: the saved commands were chosen for a car that was
travelling at 6 m/s after the old braking. In closed loop the actor sees the limit
through its existing speed-ceiling guard. Evidence:
[ramp-started-replay.json](evidence/highway-jerk-diagnosis-v05/ramp-started-replay.json)
(SHA-256 `027262abf2378439c851cd0e9cbe58864a72199e9bf91f1e9cc1639a178f7b79`).
Tests: [`tests/test_highway_flow_rl_v05_ramp_networks.py`](../tests/test_highway_flow_rl_v05_ramp_networks.py).

**Not done yet:**

- No closed-loop run. Evaluating the frozen v04 actor on these networks needs
  integration first: `sb1` lane IDs shift by one (`sb1_k` becomes `sb1_{k-1}`),
  which touches the demand generator's mainline `departLane`, the lane-graph
  identities and any fixture that names `sb1_0` as the auxiliary lane.
- Results from v04 and earlier were measured on the historical layout and stay
  historical evidence; they do not transfer automatically.
- The I-110 `candidate` network was not changed or checked.
- Training on the corrected networks is not authorized and has not started.

## Frozen actor on ramp-started networks (November 11, 2026)

**Integration:** no environment code change was needed. Lane identity is
derived from each edge's lane count minus the through-lane count, so on the
new networks every `sb1` lane is a through lane and `sb2_0` is the auxiliary
lane fed by the ramp. The demand generator resolves mainline lanes through
those identities. Regression test:
[`tests/test_ramp_started_lane_identity.py`](../tests/test_ramp_started_lane_identity.py).
Using the new networks is a config change (network path and hash per row).

**Evaluation:** the frozen v04 final actor (`last.pt`, SHA-256 `20777e23...`)
ran in the draft runner's `legacy` mode, which reproduced the v04 final
validation exactly. It ran from the canonical checkout, whose working tree
matches the draft comparison sources. Each of the six ramp layouts ran twice
with the same single-AV merge row and seed 8400004: historical network vs
ramp-started network. No training, held-out rows or promotion. 12 episodes,
90 s, charged to the shared study ledger.
Config: [highway-flow-rl-v05-ramp-eval-01.json](../configs/evaluation/highway-flow-rl-v05-ramp-eval-01.json)
(generated by [`scripts/prepare_highway_flow_rl_v05_ramp_eval.py`](../scripts/prepare_highway_flow_rl_v05_ramp_eval.py)).

```text
python experiments/run_highway_flow_rl_v05.py evaluate --config outputs/highway-flow-rl/v05/ramp-eval-01-config.json --output outputs/highway-flow-rl/v05/ramp-eval-01 --checkpoint outputs/highway-flow-rl/v04/long-round-03/last.pt --max-wall-seconds 1500
```

| Layout | Network | Overrides | Peak jerk (m/s3) | Mean speed / limit | Pre-merge stop |
| --- | --- | ---: | ---: | ---: | ---: |
| train 2-lane, 120 m ramp | historical | 1.32% | 50.8 | 0.90 | 0 s |
| | ramp-started | 0.00% | 12.1 | 0.96 | 0 s |
| train 3-lane, 240 m ramp | historical | 17.35% | 52.5 | 0.71 | 9.0 s |
| | ramp-started | 0.00% | 12.4 | 0.94 | 0 s |
| train 4-lane, 120 m ramp | historical | 1.25% | 50.9 | 0.94 | 0 s |
| | ramp-started | 0.00% | 12.0 | 0.96 | 0 s |
| train 5-lane, 240 m ramp | historical | 1.36% | 50.7 | 0.92 | 0 s |
| | ramp-started | 0.00% | 12.5 | 0.94 | 0 s |
| validation 2-lane, 180 m ramp | historical | 1.26% | 50.7 | 0.76 | 0 s |
| | ramp-started | 0.00% | 12.6 | 0.95 | 0 s |
| validation 4-lane, 180 m ramp | historical | 1.29% | 50.8 | 0.93 | 0 s |
| | ramp-started | 0.00% | 12.6 | 0.95 | 0 s |

All 12 episodes merged once with zero collisions and teleports. On every
ramp-started network the merging car's only jerk sample above 5 m/s3 is the
startup handoff at 0.2 s (12.0-12.6 m/s3). Every case still fails the strict
gates, for two reasons: that startup sample, and the rolling merge-quality
check, where target-speed agreement around the merge is 46% against the
required 90%. Evidence:
[ramp-eval-01-summary.json](evidence/highway-jerk-diagnosis-v05/ramp-eval-01-summary.json)
(SHA-256 `d918a7bcac9007b5406833ff6073fcad0062bcef357700c87054f837da6569d6`).

**Ledger reconciliation:** the draft's `comparison-01/full-final` segment had
been left `running`, with a stale `active.lock` and no process, after writing 3
of 6 cases. The runner refuses to start until such segments are reconciled. On
November 11 the lock was removed and the segment closed as `failed`, keeping
its full 600 s reservation, because the actual elapsed time is unknown. Backup:
`outputs/highway-flow-rl/v02/cost-ledger.before-2026-11-11-reconciliation.json`.

## Will this policy work on other highway and merge designs?

Not demonstrated, and probably not reliably yet.

- **Coverage so far:** training and evaluation use one road family: straight or
  gently bent 2-5 lane highways with a single right-side on-ramp feeding an
  auxiliary lane (120-240 m ramps, 50-60 mph), plus one imported I-110 segment.
  The v04 `held_out_rows` are held-out demand seeds on the same networks. The
  config states explicitly that they are not held-out geometry.
- **Positive evidence today:** the two `validation-*-ramp180` layouts were never
  trained on, and the corrected ramp-started roads are new geometry (longer,
  parallel ramps, no upstream auxiliary lane). The frozen actor merged on all of
  them with no collisions or overrides. That is within-family transfer from one
  seed per layout, at medium density with one controlled car.
- **Not yet passing at home:** the actor still fails its own development gates
  (startup jerk, merge quality) on the roads it was trained on.
- **Untested designs:** left-side entries, short or stop-controlled ramps with
  no acceleration lane, lane drops, weaving sections (entry followed by exit),
  loop or curved ramps, consecutive ramps, congested or dense traffic, other
  speed limits, several controlled cars merging together, and noisy or
  sensor-based observations.
- **Why some transfer is plausible:** the actor observes structured local
  map/route and neighbor features (gaps, speeds, lane identities, distances to
  lane ends), not a fixed road layout. That gives it a chance on new geometry,
  but only a held-out geometry test can show it.

A proper answer needs a held-out geometry bank with those merge types. It
should be built and frozen before any tuning, then the frozen actor evaluated
on it, after the startup-jerk rule and the merge-quality gate are settled.

## Held-out merge geometry bank (November 11, 2026)

**Purpose:** answer whether the policy works on highway and merge designs it
has never seen. Until now there was no held-out *geometry*, only held-out
demand seeds on training roads. This bank is built once and frozen. Use it only
for frozen-actor evaluation; never tune rewards, gates or training data on its
results.

Builder: [`scripts/build_highway_flow_rl_v05_heldout_bank.py`](../scripts/build_highway_flow_rl_v05_heldout_bank.py).
Frozen bank: `outputs/highway-flow-rl/v05/heldout-bank-01/`, with a manifest copy at
[heldout-bank-01.json](evidence/highway-jerk-diagnosis-v05/heldout-bank-01.json)
(SHA-256 `224e3e3cd56a953cd07637cc61450e30449eb361d013cad384544a5adc40d781`),
which records every network hash. All layouts use the realistic ramp-started
design: the ramp becomes the acceleration lane, joins with priority and has a
parallel final approach.

```text
python -m scripts.build_highway_flow_rl_v05_heldout_bank --output outputs/highway-flow-rl/v05/heldout-bank-01
```

Trained family, for comparison: 2-5 through lanes, 3.2 m lanes, 50-60 mph,
30 mph ramp, ramp start 120-240 m upstream, acceleration lane ending at n3
(~800 m), bend <= 30 m, uniform segments.

| Layout | What is new | Lanes | Road / ramp speed | Ramp length | Acceleration lane |
| --- | --- | ---: | --- | ---: | --- |
| `heldout-short-accel-3lane` | Short 180 m acceleration lane | 3 | 55 / 30 mph | 168 m | sb2 only |
| `heldout-long-accel-2lane` | Long acceleration lane to n4 (~1,260 m) | 2 | 50 / 30 mph | 159 m | sb2-sb4 |
| `heldout-6lane` | Six through lanes | 6 | 60 / 30 mph | 206 m | sb2-sb3 |
| `heldout-tight-curve-3lane` | Merge on a strong curve (150 m bend) | 3 | 55 / 30 mph | 168 m | sb2-sb3 |
| `heldout-45mph-4lane` | Slower road and ramp | 4 | 45 / 25 mph | 150 m | sb2-sb3 |
| `heldout-wide-lanes-2lane` | 3.65 m (US-standard) lanes | 2 | 55 / 30 mph | 168 m | sb2-sb3 |
| `heldout-short-ramp-4lane` | Short ramp, 60 m upstream | 4 | 60 / 30 mph | 85 m | sb2-sb3 |
| `heldout-fast-ramp-3lane` | Long 45 mph ramp, 300 m upstream | 3 | 55 / 45 mph | 303 m | sb2-sb3 |

**Structural checks** (all eight pass): the ramp link is priority (`M`) with no
foes; the auxiliary lane is fed only by the ramp and spans exactly the declared
edges; every through lane continues; lane widths match; and the ramp connector
speed is at least the ramp speed limit, so there is no hidden corner braking.
netconvert reported no warnings.

**Environment compatibility smoke** (no policy loaded): every layout was loaded
into the v04 flow-RL environment with a single-AV merge row, reset (demand,
lane identities, speed limits), and driven for 60 s by SUMO itself (native
handoff, no commands applied). All eight reset and ran without errors, with
zero collisions and teleports, and SUMO merged the car on each. This shows the
environment can represent the layouts; it says nothing about the trained
policy. Script: [`scripts/smoke_highway_flow_rl_v05_heldout_bank.py`](../scripts/smoke_highway_flow_rl_v05_heldout_bank.py).
Evidence: [heldout-bank-01-smoke.json](evidence/highway-jerk-diagnosis-v05/heldout-bank-01-smoke.json)
(SHA-256 `96ca759ae9a2df1f4156781072a5b04a26b8c729f30feda586e4693e34c2e341`).
Tests: [`tests/test_highway_flow_rl_v05_heldout_bank.py`](../tests/test_highway_flow_rl_v05_heldout_bank.py).

**Not representable without environment changes** (listed in the builder,
deliberately not approximated): left-side entries; ramps with no acceleration
lane; mainline lane drops; weaving sections with an exit; consecutive ramps;
loop ramps. These are common real designs, so a policy that passes this bank
still would not be shown to handle all highway merges.

**Not done:** the trained policy has not been evaluated on this bank. That
evaluation is a one-time held-out measurement. It is most informative after the
startup-jerk rule and merge-quality gate are settled, since results must not
feed back into tuning.
