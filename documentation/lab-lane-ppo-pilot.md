# First one-car Isaac Lab PPO pilot

The user approved this single pilot on October 19, 2026 with “yes start training”.
The immutable [reviewed proposal](../experiments/lane-follow-ppo-proposal.json)
is bound by SHA-256 in the [approval record](../experiments/lane-follow-ppo-pilot.json).
Its historical `UNAPPROVED` field describes the review-stage artifact. The
resolved launch configuration applies the later approval without rewriting it.

The approved training run completed in **385.97 seconds** with exactly **65,536
transitions, 32 updates and 5,120 optimizer calls**. All four checkpoints were
saved. Its 31 complete training episodes timed out; a final 436-step partial
episode is retained separately. This is a completed experiment, not a successful
learned driver. Held-out evaluation and the illustrative screenshot replay are
in progress.

The native zero-optimizer smoke passed all eight inference, normalization,
scripted-completion and reload checks. The final local software suite passed
**2,292 tests**, with 187 optional skips and 116 passing subtests. Independent
agents reviewed the PPO arithmetic against installed SB3 2.9.0, checkpoint
provenance, raw episode reduction and screenshot completion boundaries.

## Approved experiment

| Setting | Frozen value |
|---|---|
| Simulator | One CPU PhysX car in Isaac Lab; physics 120 Hz, control 60 Hz |
| Training road | `left_r60`, target speed 3 m/s, stop at station 60 m |
| Episode | 35 s including 2 s forced-brake settling |
| Reset | Seed 2101; lateral ±0.15 m and yaw ±0.035 rad |
| Observation | 22 privileged simulator/known-map values |
| Action | Steering ±0.5 rad, throttle and brake each [0,1]; existing brake priority |
| Reward and ending | Existing verified task contract, unchanged |
| Algorithm | Stable-Baselines3 2.9.0 PPO on CPU, four Torch threads |
| Networks | Separate [64,64] Tanh actor and critic; initial log standard deviation −1 |
| Optimizer | Learning rate 0.0003; 2,048-step rollouts; batch 128; at most ten epochs |
| PPO | Gamma 0.999; GAE 0.95; clip 0.2; target KL 0.02; entropy coefficient 0 |
| Normalization | Observations only, clip 10, epsilon 1e−8; corrected terminal observations |
| Budget | One run, 65,536 control transitions, 32 updates, at most 5,120 optimizer calls |
| Limits | One-hour training process cap; stop at 90% whole-GPU memory |
| Checkpoints | After completed updates 8, 16, 24 and 32 |

The checkpoints contain actor, critic, optimizer state, observation normalization,
random-generator states, counters and configuration/provenance. They do not claim
exact restoration of a native PhysX episode in progress. A durable exclusive claim
prevents a second training run, and each native worker consumes its run once.
Failures and partial records remain in their original output package.

## Selection and evaluation

All four checkpoints receive three deterministic `left_r60` episodes using seeds
3101–3103. Rank by most driving successes, fewest physical failures, greatest mean
progress, lowest mean lateral RMS, then earliest update. This decision uses no
held-out result.

The selected checkpoint and the existing scripted controller each receive seeds
4101–4105 on `straight100`, `left_r60` and `right_r60`: 15 episodes per controller,
with identical seeded initial conditions. Observation statistics are frozen.
Every episode is retained, including early failures. Evaluation package success
means evidence and execution are valid; it does **not** mean the car drove well.

All 12 selection episodes completed with valid evidence: zero successful drives,
11 timeouts and one lane departure. The declared rule selected **update 24**.
It had no physical failures and mean progress of just **0.001234 m**. Update 16
averaged 4.242 m but had one lane departure, so it ranked below the three
candidates with no physical failures. Selecting an almost stationary candidate
is a consequence of the fixed failure-first rule, not evidence of learned lane
following. The rule was not changed after seeing these results.

The independent reducer replays native guard decisions, two physics substeps per
control, rewards, terminal-state handling and resets. It reports progress,
tracking error, stopping, contact and other failures, action clipping, brake
priority and steering slew. Acceleration and jerk are finite differences of
scalar planar speed at 120 Hz; they are not comfort certification.

## Reproduction and evidence

Run from the repository root using the traffic environment. The supervisor starts
the configured Isaac runtime in a separate process; do not install traffic
dependencies into Isaac Python.

```powershell
python -m experiments.run_lane_follow_ppo --mode train --check
python -m experiments.run_lane_follow_ppo --mode smoke
python -m experiments.run_lane_follow_ppo --mode train --smoke-evidence outputs/lab_lane_ppo/<passing-smoke> --approved-config-sha256 <approval-file-sha256>
python -m experiments.run_lane_follow_ppo --mode evaluate --stage selection --checkpoint outputs/lab_lane_ppo/<training-run>/checkpoints/update-08
python -m experiments.run_lane_follow_ppo --mode evaluate --stage heldout --route straight100 --checkpoint outputs/lab_lane_ppo/<training-run>/checkpoints/<selected-update>
python -m experiments.run_lane_follow_ppo --mode evaluate --stage heldout --route straight100 --controller scripted
```

The train command consumes the one-run authorization. Do not rerun it to obtain a
different seed, replace a failed run, extend the budget or resume a checkpoint.
Run native jobs serially. Repeat selection for the other three declared updates
and held-out evaluation for all three declared routes.

Raw trajectories, checkpoints and frozen run packages live under
`outputs/lab_lane_ppo/`; runtime logs live under `logs/lab_lane_ppo/`. These ignored
local artifacts are not included in a Git clone. Public documentation contains
compact evidence and actual rendered screenshots, with raw artifact hashes and
source identities. Failed driving remains visible in the results.

Checkpoint loading verifies the entire completed training package and all 36
captured source files before deserializing a model. `.gitattributes` pins each
file's checkout line endings; an isolated checkout with `core.autocrlf=false`
matched all 36 recorded hashes. These are hashes of checked-out source bytes,
not Git's normalized blob representation. Original proposal and prior evidence
contents remain unchanged.

This pilot measures low-speed physical control with privileged state. Broader
training, sensor-based driving, vehicle cloning, traffic coordination and highway
speed remain outside its scope.
