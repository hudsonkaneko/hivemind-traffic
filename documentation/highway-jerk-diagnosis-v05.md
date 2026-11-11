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
- The unattributed native braking on `mlk_on_0` needs its own investigation
  (likely an internal SUMO junction/foe or safe-speed rule) before it is treated
  as a policy failure.

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
