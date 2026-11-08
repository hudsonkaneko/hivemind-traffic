# One-car Isaac Lab training proposal

Historical review-stage proposal. The user subsequently approved this exact
single pilot on October 19, 2026. See the [approved run and results](lab-lane-ppo-pilot.md)
and [approval record](../experiments/lane-follow-ppo-pilot.json) for current status.
The review-stage statement below and machine-readable proposal are preserved.

**UNAPPROVED — training has not launched for this task. No policy or optimizer has
been created, no optimizer steps have run, and there is no trainer entrypoint.** This document
is the proposed setup for the user's review after bounded native validation.
Passing environment tests does not authorize training.

The machine-readable proposal is
[`experiments/lane-follow-ppo-proposal.json`](../experiments/lane-follow-ppo-proposal.json).
The new task preserves the existing compatibility adapter and scripted controller.
It is a low-speed physical-control pilot using privileged simulator state and a
known map. It does not establish perception, traffic coordination, highway
performance, or a safe learned driving policy.

## Task and validation boundary

The task uses one CPU-physics vehicle at 120 Hz with decisions at 60 Hz, without
Fabric or cloned environments. The existing roads are 100 m long; the task stops
at station **60 m**, at a target cruise speed of 3 m/s. Each episode has a 35 s
cap including 2 s of forced-brake settling. Success requires net progress of at
least 59.5 m, station 60 ± 0.5 m, speed at most 0.05 m/s, and one continuous
measured second satisfying the stopped condition. Failure takes precedence over
success; timeout occurs only if neither applies.

The proposed first training run uses `left_r60` only. Each reset samples lateral
offset uniformly within ±0.15 m and heading error within ±0.035 rad, with zero
initial velocity and native wheel/rest state reset. The straight and right-hand
routes in final evaluation measure transfer to different route geometry.

Before training approval, the frozen validation protocol in
[`lab-lane-following-validation.json`](../experiments/lab-lane-following-validation.json)
requires 12 serial native packages:

| Packages | Scripted evidence |
|---|---|
| Three nominal runs and three fresh-process repeats | `straight100`, `left_r60`, `right_r60`, seed 101 |
| Three perturbed runs, two episodes each | Signed corner offsets (+0.15 m, +0.035 rad) and (−0.15 m, −0.035 rad), seeds 1101 and 1102 on each route |
| One diagnostic run on `straight100` | Seeded and signed physical resets, invalid-action rejection without advancing physics, finite clipping/brake priority, timeout, and lane-departure termination |
| One contact positive-control run on `straight100` | Verify detection in the newly composed task scene |
| One visible `left_r60` run | Actual rendered captures during driving and a composed-scene inspection after automatic reset |

Successful driving episodes require driving lateral RMS ≤ 0.20 m and maximum
absolute error ≤ 0.50 m, full conservative chassis-footprint containment, no
undesired rigid contacts, finite ordered state, tilt ≤ 5°, and post-settling
four-wheel support fraction ≥ 0.99 with no unsupported interval over 0.10 s.
The stopped hold may drift at most 0.05 m. Fresh-process repeat tolerances are
0.001 m position, 0.001 m/s speed, and 0.001 rad yaw. The deliberate contact and
departure cases must produce their expected failures rather than being counted
as successful driving episodes.

Every native job has a declared bounded deadline (300 s in the current protocol;
never over 600 s) and a stop at 90% whole-GPU memory usage. No other Kit process
may be active. The two exact UI startup errors observed in the compatibility
baseline are retained and allowed at most once each before the task-ready marker;
duplicates, later occurrences, new errors, and USD reference-count warnings fail
qualification. A successful older compatibility run does not qualify the new task.

**The final native matrix passed all 12 packages and all three fresh-process
repeat comparisons on October 19, 2026.** Its 16 declared episodes comprise 13
successful stops and the three expected contact, lane-departure and timeout
diagnostics. The report rechecks saved native traces and package/source hashes;
it does not treat the deliberate failures as driving successes. The earlier
construction/contact-reporting failures remain preserved separately in the
[task evidence history](lab-lane-following.md).

Across the 13 successful stops, progress was 59.876984–59.878113 m, worst driving
lateral RMS was 0.043482 m and worst absolute lateral error was 0.150409 m.
All had zero rigid contacts and post-settling four-wheel support fraction 1.0.
The largest endpoint error was 0.123016 m and largest one-second hold drift was
0.006162 m. Each hold was independently checked over 121 native states. All
three repeat pairs had zero measured position, speed and yaw differences within
their recorded traces. This bounded scripted result does not authorize training
or qualify a learned policy.

The official local report is
`outputs/lab_lane_following/review/matrix-report.json` (SHA256
`ed8640266b89d852b1e5b12409248ef6cc11716eb7668a36c7824985d89dfe0d`).
The raw packages remain under ignored `outputs/`.

## Proposed observations and actions

The actor and critic receive the same 22 float32 fields (`[1, 22]`) from the
simulator and analytic route:

| Group | Fields |
|---|---|
| Route-relative pose | Progress, remaining distance, lateral error, sine and cosine of heading error |
| Motion and support pose | Route-forward speed, route-lateral speed, planar speed, vertical speed, height, upright z |
| Known map and targets | Curvature now, 3 m ahead and 6 m ahead; cruise and endpoint-approach target speeds |
| Applied control and episode state | Steering, throttle, brake, elapsed fraction, settling flag, stopped-hold fraction |

The exact field order is `OBSERVATION_NAMES` in
[`lane_following_contract.py`](../environments/lane_following_contract.py).
No camera or other sensor measurement is used. Wheel and suspension dynamics and
angular velocity are omitted, so this is not a fully observed vehicle-state claim.

The `[1, 3]` actions are physical steering in [−0.5, +0.5] rad, throttle in [0, 1],
and brake in [0, 1]. Finite out-of-range values are clipped. A positive brake
suppresses throttle; steering slew is limited to 0.5 rad/s and commands expire
after 12 physics ticks. Malformed or nonfinite actions are rejected before
physics advances. The first 2 s force braking regardless of requested policy
action. Those explicit settling transitions remain in the proposed rollout with
zero running reward.

Independent Gaussian throttle and brake outputs are redundant under brake
priority. The pilot must report clipping and overrides so a policy cannot appear
to use direct unconstrained actuation. Changing this action design would require
a separately reviewed contract and proposal change.

## Proposed reward and episode endings

With `dt = 1/60 s`, the running terms are:

| Term | Proposed contribution |
|---|---|
| Progress | Signed change in route station / 3, once per transition; zero for impossible state jumps |
| Lateral error | `−dt × (lateral_error / 0.5)^2` |
| Heading error | `−0.25 × dt × (heading_error / 0.25)^2` |
| Speed error | `−0.25 × dt × ((forward_speed − target_speed) / 3)^2` |
| Applied steering rate | `−0.01 × dt × (steering_change / (dt × 0.5))^2` |
| Time | `−0.05 × dt` |
| Terminal | +10 success, −10 declared failure, zero for timeout |

Settling contributes no running terms, but a safety failure during settling still
receives −10. Signed progress prevents repeated back-and-forth displacement from
earning positive net progress. Reward is never a substitute for physical acceptance.
These weights remain proposed even though the scripted environment implements
them for checking transition semantics.

Success and task/physical failure are true terminations. The external 35 s cap is
a truncation only if no termination already occurred. Only a timeout-only
truncation may bootstrap, and it must use the **pre-reset final observation**.
This follows the [SB3 vector-environment contract](https://stable-baselines3.readthedocs.io/en/master/guide/vec_envs.html)
and [Gymnasium time-limit guidance](https://gymnasium.farama.org/tutorials/gymnasium_basics/handling_time_limits/).
The live documentation may describe newer releases; the installed and inspected
SB3 version is 2.9.0.

The installed Isaac Lab SB3 wrapper's fast and slow paths both used the returned
post-reset observation as `terminal_observation`. The project-local
[`lane_follow_sb3.py`](../environments/lane_follow_sb3.py) corrects that boundary
using the native task's detached `final_observation`, terminal flags and mask.
Ordinary returned observations keep the automatic-reset semantics. The helper
rejects missing, stale, nonfinite or mismatched terminal data and copies buffers
so later resets cannot overwrite the terminal state.

The 34 focused CPU tests passed. A separate synthetic check against the installed
wrapper and installed SB3 `VecNormalize` passed all 10 checks without a
`SimulationApp`, physics, rendering, model or optimizer. A hand-computed example distinguishes the
true terminal value from the reset value; the normalized terminal observation is
processed exactly once and frozen evaluation statistics remain unchanged. Evidence:
`outputs/lab_lane_following_review/terminal-normalization-check.json`. The final
native matrix also passed terminal-versus-reset integration for successful stops,
contact/departure failures and timeout, including the SB3 terminal snapshot and
timeout-only bootstrap flag. These checks exercised wrappers and scripted
actions, without constructing a PPO policy or performing optimization.

## Proposed PPO configuration and exact budget

The installed runtime inventory is Python 3.12.13, Isaac Lab 6.1.18,
`isaaclab_rl` 0.5.7, SB3 2.9.0, Gymnasium 1.2.1, PyTorch 2.10.0+cu128 and
NumPy 2.3.1. Metadata and source inspection did not alter those external runtimes.
The synthetic check imported the trainer library's wrappers but constructed no
trainer or policy. Package presence alone does not establish native compatibility.

| Setting | Proposal |
|---|---|
| Algorithm and device | SB3 PPO `MlpPolicy`, CPU, four PyTorch threads, one native environment |
| Actor and critic | Separate [64, 64] layers, Tanh, orthogonal initialization, `log_std_init = −1` |
| Rollout and minibatches | 2,048 transitions/update; batch 128; 10 epochs |
| Learning and discounting | Learning rate 0.0003; gamma 0.999; GAE lambda 0.95 |
| Loss controls | Clip 0.2; no value clip; entropy coefficient 0; value coefficient 0.5; gradient norm 0.5 |
| Other controls | Normalize advantages; target KL 0.02; no state-dependent exploration |
| Observation normalization | `VecNormalize` outside the corrected wrapper; observation clip 10, epsilon 1e−8; no reward normalization |
| Training seed and length | Seed 2101; 65,536 control transitions; 32 completed rollout updates |
| Hard bounds | At most 5,120 minibatch optimizer steps; 3,600 s training wall cap; 90% whole-GPU memory stop; one serial run |

Observation statistics update only during training and are saved with every
checkpoint. Evaluation freezes them. Early stopping on nonfinite data/loss,
contract violation, resource/time limits or user request preserves the actual
partial counts. There is no automatic extension, seed replacement or resume.

The maximum transition budget corresponds to 1,092.27 simulated seconds. The nine
successful **headless driving packages** in the final matrix recorded 17,588
control transitions. Their per-package step-call rates were 258.05–318.24
transitions/s and whole episode-loop rates were 203.41–247.94 transitions/s.
The pooled whole-loop rate was 226.37 transitions/s. This selection includes
nominal runs, fresh-process repeats and both perturbations on all three routes;
it excludes the deliberately short contact case, stationary timeout/departure
diagnostics and visible capture run.

| Proposed training route: `left_r60` | Step-call transitions/s | Whole episode-loop transitions/s |
|---|---:|---:|
| Nominal | 258.05 | 203.41 |
| Fresh-process repeat | 277.07 | 215.84 |
| Two corner episodes together | 317.44 | 244.78 |

Using the slowest observed whole-loop rate gives
`65,536 / 203.412981 = 322.18 s`, approximately **5.37 minutes of rollout-only
reference time**. This is conservative within the measured scripted sample; it
is not a training-time guarantee. Process startup, PPO policy/value inference,
optimizer work, checkpointing and evaluation were not measured by this estimate.
A learned policy's failures and resets can also change throughput. The proposed
3,600 s training wall cap remains unchanged and is a stop, not a prediction.

The visible run recorded 63.16 whole-loop transitions/s with rendering and capture;
its process time also includes inspection. It is excluded from the headless
estimate. Whole-GPU peaks were 3,318–3,328 MiB for the nine headless driving
packages and 5,150 MiB for the visible package, including other desktop GPU use.
These are control-transition rates, not render FPS. They do not establish PPO
training throughput or support extrapolation to cloned environments. Exact rates
and source-summary hashes are recorded in the proposal's additive
`measured_validation_throughput` object; the proposed learning parameters are
unchanged.

## Checkpoints, selection and held-out evaluation

Save atomic checkpoints after completed updates 8, 16, 24 and 32, at transitions
16,384, 32,768, 49,152 and 65,536. Each includes actor, critic, optimizer,
normalization statistics, resolved configuration, update/environment counters,
Python/NumPy/Torch RNG states, reset seed and episode index, and source/runtime/
asset hashes. This does not promise exact native continuation from a mid-episode
snapshot.

After training ends, evaluate those four saved checkpoints in separate no-update
jobs on `left_r60` using selection seeds 3101, 3102 and 3103. Choose by most
successes, then fewest physical/task failures, highest mean net progress and
lowest driving lateral RMS, breaking ties in favor of the earlier checkpoint.
Neither reward nor held-out outcomes select the checkpoint. If all checkpoints
fail to complete, report no learned completion; a relative best label is not
acceptance.

Final held-out seeds 4101–4105 are reserved for each of the three routes: 15
episodes per controller, comparing the selected deterministic policy with the
unchanged scripted follower using identical reset specifications. Freeze the
normalizer and policy, disable exploration noise, run zero optimizer steps, and
retain every failure without replacing seeds. The maximum selection exposure is
25,200 transitions and final paired evaluation exposure is 63,000 transitions,
separate from the training budget. Each evaluation process is bounded to 600 s.

Publish successes with their denominators by route, failure reasons and timeout
exposure, progress, lateral RMS/max, contacts and footprint, endpoint and hold
drift, support and tilt, clipping and brake overrides, and measured resources and
throughput. Any acceleration/jerk measures must identify scalar-speed derivatives
and their sampling convention. Five seeds per route provide descriptive pilot
evidence, not a broad generalization or safety guarantee.

## Required decision before any training

The frozen scripted native matrix and repeat comparisons are complete. Review
the task evidence, original screenshots and measured resource/throughput scope,
then obtain the user's agreement on observations, actions, rewards, PPO, budget,
selection and evaluation. A separately reviewed trainer entrypoint must enforce
those bounds and require an explicit training launch. **This proposal supplies
no training command and grants no training authorization.**
