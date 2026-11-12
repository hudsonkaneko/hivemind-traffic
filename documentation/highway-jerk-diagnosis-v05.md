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
