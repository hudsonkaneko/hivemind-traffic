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

## Startup handoff ramp (November 11, 2026)

**Problem:** each controlled car enters at its SUMO insertion acceleration
(0 m/s2), and the actor's first command was applied in one 0.1 s step. That
caused the 12.0-12.6 m/s3 peaks, the only jerk failures left on the corrected
networks.

**Fix:** a new opt-in environment mode, `startup`, in
[`environments/highway_flow_rl_v05_env.py`](../environments/highway_flow_rl_v05_env.py).
It keeps legacy v04 behavior except during control handoff. There it applies
the draft's comfort rate limit: 4 m/s3, i.e. at most 0.4 m/s2 change per step
from the measured acceleration. Maximum braking and any lower safety ceiling
bypass the limit. The ramp ends permanently once the realized command catches
up with the actor's request, or after 30 decisions (3 s). Conditioned steps are
still counted as interventions (`comfort_rate_limit`). The acceptance gates are
unchanged, and the actor itself is unchanged. Tests:
[`tests/test_highway_flow_rl_v05_startup_handoff.py`](../tests/test_highway_flow_rl_v05_startup_handoff.py).

The draft's conditioning hook in `environments/highway_environment_v05.py` existed
only as an uncommitted edit in the main checkout, so on the branch the draft's
`continuity` mode silently did nothing. The hook is now committed, identical to
that edit. Its default returns the request unchanged, so legacy behavior is
identical. The two changed files (`highway_flow_rl_v05_env.py`,
`highway_flow_rl_v05_contract.py`) were also copied over the untracked draft
copies in the main checkout so the evaluation could charge the shared ledger.
The original draft versions are preserved in commit `e98cc7e`.

**Frozen v04 actor, same 12 ramp rows as before (seed 8400004), startup vs legacy:**

| Layout (ramp-started) | Peak jerk | Overrides | Speed / limit | Per-car checks | Result |
| --- | ---: | ---: | ---: | --- | --- |
| train 3-lane, 240 m ramp | 12.4 -> 4.0 | 0.00% -> 0.58% | 0.94 -> 0.93 | all pass | only merge quality fails |
| train 4-lane, 120 m ramp | 12.0 -> 4.0 | 0.00% -> 0.47% | 0.96 -> 0.96 | all pass | only merge quality fails |
| train 5-lane, 240 m ramp | 12.5 -> 4.0 | 0.00% -> 0.53% | 0.94 -> 0.94 | all pass | only merge quality fails |
| validation 2-lane, 180 m ramp | 12.6 -> 4.0 | 0.00% -> 0.54% | 0.95 -> 0.95 | all pass | only merge quality fails |
| validation 4-lane, 180 m ramp | 12.6 -> 4.0 | 0.00% -> 0.49% | 0.95 -> 0.95 | all pass | only merge quality fails |
| train 2-lane, 120 m ramp | 12.1 -> 28.9 | 0.00% -> 17.89% | 0.96 -> 0.79 | jerk, interventions fail | regressed (see below) |

The handoff lasted 4 decisions (0.2-0.5 s) per car. Historical-network rows are
essentially unchanged: the gore braking still dominates there, at about 51
m/s3. All 12 episodes merged with zero collisions. Merge quality (target-speed
agreement around the merge) is 27-49% against the 90% gate everywhere.
Evidence: [startup-handoff-eval.json](evidence/highway-jerk-diagnosis-v05/startup-handoff-eval.json)
(SHA-256 `3c12cc374a735c1886a46e6389b2e1d1795accd2669b06a0f64201cbd93fc7e5`).

**Why one layout regressed:** the handoff left the car about 0.4 m/s slower
at 3.2 s. At that moment, the actor-input flag for a left lane change was open
in the legacy run but closed in the startup run. That flag is the v04
projected-gap mask, which checks the closing rear car `main_6` (69 m back, 7.4-7.6
m/s faster) over a 3.1 s maneuver. The policy never accelerated enough to
reopen the window. It made no lane request until 42.7 s (legacy: 3.2 s), rode
the acceleration lane to its end, braked for the lane end (`lane_end`
overrides, then SUMO), stopped, and merged at 50.7 s. The handoff ramp did what
it was designed to do. The regression exposes a fragile merge decision in the
frozen actor: a borderline gap check plus no learned strategy for creating a
new gap. That needs a policy or training change, not a handoff change.

**Development cases:** first blocked by the 60-episode diagnostic budget; run
on November 13 after the user raised the budget (see below).

## Startup mode on the v04 development cases (November 13, 2026)

**Budget change (user decision, November 13):** the v05 diagnostic episode cap
was raised from 60 to 80, in the pinned contract
(`traffic/highway_flow_rl_v05_contract.py`) and the pilot config. No other
budget value changed, and training remains unauthorized.

The frozen v04 final actor ran in `startup` mode on the same six development
cases and seeds as the v04 final validation. These are the original networks,
including I-110 and the historical ramp120 layout. It used 6 episodes and 79 s.
Config: [highway-flow-rl-v05-dev-startup-01.json](../configs/evaluation/highway-flow-rl-v05-dev-startup-01.json).

| Case | Peak jerk, v04 -> startup | Failing cars | Case gate |
| --- | ---: | ---: | --- |
| I-110 merge (8 AVs) | 13.3 -> 10.5 | 5 -> 3 (jerk) | fail (+ merge quality) |
| Cruise, 1 AV | 5.7 -> 4.0 | 1 -> 0 | **pass** |
| Passing, 1 AV | 7.1 -> 4.0 | 1 -> 0 | **pass** |
| I-110 following (8 AVs) | 8.2 -> 5.3 | 4 -> 2 (jerk 5.3; interventions) | fail |
| 2-lane ramp120 merge (historical network) | 50.8 -> 51.1 | 1 -> 1 (gore braking) | fail (+ merge quality) |
| 60 mph cruise (8 AVs) | 10.0 -> 4.0 | 6 -> 1 (avoidable braking) | fail |

Overall: 2/6 cases pass every gate (v04 final 0/6; v04 selected best 1/6), and
failing cars fell from 18 to 7. Zero collisions and teleports; 2/2 stable merges.
Pooled interventions rose from 0.42% to 0.60%, because the handoff steps count
as interventions. The remaining jerk failures are not startup peaks: the
historical ramp120 gore braking (fixed by the ramp-started networks), and
mid-episode peaks of 5.3-10.5 m/s3 among the I-110 cars. Evidence:
[dev-startup-eval.json](evidence/highway-jerk-diagnosis-v05/dev-startup-eval.json)
(SHA-256 `7839a66dea73c68a4cd684ac574095ae456f70d57600c694bba2fd8a51a5e47f`).
Development measurement only; no held-out evaluation or promotion.

## Remaining jerk peaks explained (November 13, 2026)

Source: the `startup` development run (`outputs/highway-flow-rl/v05/dev-startup-01`).
Each of the 10 samples above 5 m/s3 was checked against three things: the
actor's own requests (policy output), the 117 observation fields the actor saw
before deciding (policy input), and the SUMO state (leaders, bounds, realized
acceleration). The observation layout is 51 legacy local fields, then 18 flow
extras, then 48 message fields. Attribution:
[dev-startup-jerk-peaks.json](evidence/highway-jerk-diagnosis-v05/dev-startup-jerk-peaks.json)
(SHA-256 `01c9bfd743bea1afe3f2728d26db645fb99d7cdec9189903f54cab6eafcdba44`).

| # | Case / car | Time | Jerk (m/s3) | Cause |
| --- | --- | ---: | ---: | --- |
| 1-4 | ramp120 merge (historical network), focal | 1.8-2.9 s | -51.1, -10.0, +50.7, +8.5 | SUMO gore yield braking, release, and the actor resuming |
| 5 | I-110 merge, `agent_5` | 0.3 s | -10.5 | SUMO's ACC car-following law |
| 6 | I-110 merge, `agent_2` | 0.3 s | -7.1 | SUMO, during a lane change already in progress at handoff |
| 7 | I-110 merge, focal `agent_0` | 3.9 s | -7.2 | Actor reacts to merge inputs switching on |
| 8 | I-110 merge, focal `agent_0` | 13.3 s | +8.4 | Actor reacts to merge inputs switching off |
| 9 | I-110 following, `agent_2` | 33.1 s | +5.3 | Leader leaves the end of the road |

**1. SUMO's ACC model overrides commands that the actor's guard allows (peaks
5-6).** The controlled vehicle type uses SUMO's `ACC` car-following model
(tau 1.5 s), and speed mode 31 makes SUMO cap every command at that model's
"safe" speed. For ACC this is a comfort control law, not a collision limit.
`agent_5` had a leader 32 m ahead, 1.2 m/s slower. The actor's analytical
following bound allowed +31.7 m/s2 and the actor applied 0.0, but SUMO's ACC
follow speed allowed only -0.55 m/s2 (realized -0.65). So the actor's guard and
SUMO's model disagree about what is allowed. `agent_2` started mid lane change,
and its realized -0.31 m/s2 likely reflects SUMO also following the target-lane
leader (not verified).

**2. The actor reacts in one step to inputs that switch on or off (peaks 7-8,
and 9).** The policy maps inputs to acceleration with no memory of its previous
output beyond one observation field. When groups of binary inputs flip in a
single step, its request jumps:

- At 3.9 s the focal car enters `sb2_0` from the ramp connector. In one step,
  `merge_context_known`, `merge_target_lane_delta`, `target_front_present`,
  `target_rear_present`, `left_lane_exists` and `left_rear_present` go from 0
  to 1. The request drops from +1.80 to +1.08 m/s2.
- At 13.3 s the lane change completes. `requires_merge` and the target fields
  go from 1 to 0, and `right_lane_exists` from 0 to 1. The request jumps from
  -0.07 to +0.77 m/s2.
- At 33.1 s the following car's leader `slow_leader` (43.5 m ahead) reaches the
  end of the 64 m final road segment and leaves the simulation, with the follower
  49 m from its own route end. `own_front_present` goes from 1 to 0, and the
  request jumps from -0.05 to +0.49 m/s2. That is an end-of-road artifact, not
  real traffic.

**3. The training reward hardly penalizes these jumps.** v04 trained with
`jerk` weight 0.0 and `jerk_huber` 0.09. At the 7.2 m/s3 peak the jerk term was
-0.28, while the potential-shaping term `gap_preparation` swung from +0.97 to
-0.01 between the same two steps. The shaping rewards flip with the same
discrete inputs, so they teach the jumps rather than discourage them. The 5 m/s3
limit exists only in the acceptance gate, which training never sees.

**What would fix each cause** (each changes the environment, the evaluation
rules or training, so each needs a decision):

| Cause | Peaks | Fix | Type |
| --- | --- | --- | --- |
| Gore yield on historical networks | 1-4 | Use the ramp-started networks (already built) | network |
| SUMO ACC law vs actor guard | 5-6 | Give controlled cars a collision-only car-following model, or put SUMO's ACC limit into the actor's guard and mask | environment |
| Leader leaves at road end | 9 | Stop measuring jerk/headway in the final stretch, or extend the last segment | evaluation rule / network |
| Step reactions to input flips | 7-8 | Train with a real jerk penalty matched to the 5 m/s3 gate (the v05 `full` variant's `jerk_excess`), and/or smooth the switching inputs | training |

No simulation, training or code change was made for this analysis.

## Native car-following and road-end fixes (November 13, 2026)

Both fixes are opt-in config keys. With them absent, behavior and gates are
identical to before: the v05 `development_gates` function stays byte-identical
to v04's, as an existing test requires.

**1. `controlled_car_follow_model: "Krauss"`** (default `"ACC"`). Written into
the controlled vType by `traffic/highway_v05_demand.py`. SUMO's Krauss model
caps commands only at its reaction-time collision-avoidance speed. At the two
I-110 states where ACC trimmed the actor (shown earlier), Krauss allows +2.0
m/s2. In close or fast-closing situations it still brakes hard: -4.5 to -13.1
m/s2 in a direct `getFollowSpeed` comparison.

**2. `road_end_leader_exit_rule: true`** (default off). The v05 environment
records when a car's own leader leaves the simulation at its route end
(`getArrivedIDList`). It excludes exactly the next decision from that car's
per-car jerk peak and lists every exemption. The gate wrapper `ruled_gates`
substitutes that peak and then calls the unchanged gates. The uncensored peak
stays in the report.

Tests: [`tests/test_highway_flow_rl_v05_native_fixes.py`](../tests/test_highway_flow_rl_v05_native_fixes.py).
The four changed runtime files were copied over the main checkout only after
confirming they matched the branch's previous commit. Configs:
[dev](../configs/evaluation/highway-flow-rl-v05-dev-native-fixes-01.json),
[ramp](../configs/evaluation/highway-flow-rl-v05-ramp-native-fixes-01.json).

**Frozen v04 actor, `startup` mode plus both fixes:** 12 episodes, 106 s;
diagnostic episodes used 78/80.

| Development case | Before fixes | After fixes |
| --- | --- | --- |
| I-110 merge | 3 cars fail jerk (max 10.5) | 1 car fails: focal, 8.4 m/s3 policy reaction |
| Cruise, 1 AV | pass | pass |
| Passing, 1 AV | pass | pass |
| I-110 following | 2 cars fail (jerk 5.3; interventions) | **pass** |
| ramp120 merge, historical network | gore braking, 51.1 | unchanged (network issue) |
| 60 mph cruise | 1 car avoidable braking | unchanged |

3/6 development cases now pass every gate (before fixes 2/6; v04 final 0/6),
and failing cars fell from 7 to 3. Pooled interventions fell from 0.60% to
0.26%, with zero collisions and teleports and 2/2 stable merges. The road-end
rule exempted 17 decisions across both runs, all genuine leader exits. Only one
exceeded 5 m/s3: the 5.34 m/s3 following-case peak diagnosed earlier.

On the six ramp-started layouts, five are unchanged: every per-car check passes
and only merge quality fails. On `train-2-lane-ramp120` (already failing, as
diagnosed above), interventions fell from 17.9% to 10.5%. But it now also fails
headway, at 0.34 s. Cause: the native car `main_3` made a `keepRight` lane
change into the AV's lane 3.0 m ahead. SUMO logged the AV's required follower
gap as 0.00, because under Krauss a slower follower needs no secure gap. TTC
stayed 38 s, since `main_3` was faster. This cut-in did not occur with ACC.
**Side effect: with Krauss, native drivers cut in closer in front of AVs.**
Whether to accept that, lengthen Krauss's gap, or keep ACC and instead teach the
actor ACC's limit is a decision for the user.

Evidence: [native-fixes-eval.json](evidence/highway-jerk-diagnosis-v05/native-fixes-eval.json)
(SHA-256 `434bd053271a53f0479f42a286f30711acfab06f8a6d5419c1163f20e50a364f`).
No training.
