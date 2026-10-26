# SUMO MAPPO v01: setup complete, training awaiting approval

Date: October 25, 2026. The headless environment, learner, worker runner and
no-learning preflight are implemented. **No driving-policy training has started
and no trained MAPPO policy exists.** Synthetic unit tests exercise optimizer
math; the SUMO preflight collects transitions with zero optimizer updates.

## Purpose and boundaries

The pilot asks whether four cars sharing a local acceleration policy can make
progress in ordinary traffic with few safety overrides. SUMO owns all motion.
The task is fixed-lane car following on a straight, three-lane, 5 km road. It has
no learned steering, lane changes, merging, destination planner, LiDAR/camera
perception, communication, jump-in obstacle curriculum or physical-car adapter.
Existing SUMO/PPO demos and physical-car work are preserved; the earlier CIRCLES
physical highway test remains deferred independently of this setup.

Stable agents `agent_0` through `agent_3` occupy lanes `(0, 0, 1, 2)`. Background
cars start nearby and enter each lane at seeded 1.5–3.0 second intervals. Initial
speeds are 12–22 m/s, with seeded ±10 m position offsets. The ceiling is 26.8224
m/s (60 mph). SUMO's Krauss background model uses length 4.8 m, acceleration
2.0 m/s², normal/emergency deceleration 4.5/9.0 m/s², minimum gap 2.5 m and
headway 1.5 s. Autonomous lane changes are disabled.

## Policy contract

Each shared actor sees **27 local features** with fixed scales and clipping.
These are simulator-derived estimates, not real sensor measurements. Missing
neighbors use three zeros, including the presence mask. Controlled and ordinary
vehicles can fill the six nearest-neighbor slots within 120 m.

| Zero-based indices | Features and scaling |
| --- | --- |
| 0 | Ego speed / 26.8224 |
| 1 | Ego longitudinal acceleration / 4.5 |
| 2 | Lane index / 2 |
| 3 | Remaining route fraction: `1 - lane_position / 5000` |
| 4 | Local speed limit / 26.8224 |
| 5 | Previous requested acceleration / 4.5 |
| 6–23 | Own-lane front/rear, left front/rear, right front/rear; each slot supplies bumper gap / 120, `(neighbor_speed - ego_speed) / 26.8224`, presence mask |
| 24–25 | Left/right lane exists |
| 26 | Elapsed steps / episode horizon |

The critic receives 112 values: four ordered 27-feature observations, each with
an alive mask. A four-value agent one-hot makes **116 effective critic inputs**.
Only the actor's own 27 features are used at deployment. This implements
centralized training with decentralized execution (MAPPO).

One output `u ∈ [-1, 1]` maps to acceleration:
`a = -4.5 + (u + 1) × 3.25` m/s², ranging from -4.5 to +2.0. Normalized zero
therefore means -1.25 m/s²; physical zero is approximately 0.384615 normalized.
Training uses a tanh-squashed Gaussian; evaluation uses the tanh of its mean.
Stored pre-tanh samples preserve action likelihoods near saturation.

A local filter limits speed, headway and braking below zero speed. SUMO safety
checks remain enabled (`speedMode=31`); this is not a collision-free guarantee.
Proposed, filtered and realized acceleration, override causes and reward
components are logged. A realized-versus-filtered difference above 0.0001 m/s²
counts as a SUMO intervention, including simulator constraints beyond collision
avoidance. The native baseline makes no external acceleration requests, so its
external intervention rate is zero by definition.

## Reward and episode semantics

Every active controlled car receives the mean of active cars' local rewards:

| Term | Component before weight | Weight |
| --- | --- | ---: |
| Progress | `clip(distance_delta / (26.8224 × 0.1), 0, 1)` | +1.0 |
| Headway deficit | `clip((desired_gap - actual_gap) / desired_gap, 0, 1)` with `desired_gap = 2.5 + 1.5 × speed`; zero without a visible leader | -0.5 |
| Jerk | `clip(abs(acceleration_delta) / 0.1 / 10, 0, 1)` | -0.02 |
| Collision | Controlled vehicle appears in SUMO collision IDs | -10.0 |
| Intervention | Local filter or SUMO changed the requested command | -0.1 |

One joint SUMO step follows each action batch at 10 Hz. Episodes have at most
600 steps (60 simulated seconds after bounded native spawn warm-up). Any collision or
teleport ends the episode; arrival or disappearance ends the affected agent.
Time limits truncate and bootstrap from the measured final state. True terminals
have zero bootstrap. Inactive agents are masked out of losses and do not consume
the vehicle-decision budget. GAE cannot cross a reset boundary.

## Final proposed training configuration

Source of truth: [`sumo_mappo_v1.json`](../experiments/configs/sumo_mappo_v1.json).
Its canonical configuration SHA-256 is
`0c8e70ef98827a27ed5c77dc680ea35739811cdbbca045e8fb76af9fd2d3265c`.
The matching launch digest prevents accidental configuration changes; it does
not substitute for user approval. No training command has been executed.

| Parameter | v01 value |
| --- | --- |
| Algorithm | MAPPO, shared local actor and centralized critic, no communication |
| Actor / critic hidden layers | `[128, 128]` / `[256, 256]`, Tanh |
| Hardware | CPU, one Torch thread, four isolated SUMO worker processes |
| Rollout | 256 steps per worker, up to 4,096 vehicle decisions per update |
| Budget | 1,000,000 actual vehicle decisions; 7,200-second run deadline including evaluation |
| Runtime seed | 5101 for Python, NumPy and Torch |
| Training episode seeds | Sequential 100000–199999; no wrap into evaluation |
| Discount / GAE lambda | 0.99 / 0.95 |
| Actor / critic learning rate | 0.0003 / 0.0003, Adam |
| PPO clip / value clip | 0.2 / 0.2 |
| Entropy / value coefficient | 0.01 / 0.5 |
| Gradient norm limit | 0.5 |
| Epochs / minibatch | 4 / 256 |
| Target KL | 0.02, early stopping within an update |
| Validation | Seeds 300001–300010, first update, every 25 updates and final update |
| Held-out tests | Seeds 400001–400020, selected actor plus scripted and SUMO-native baselines |
| Worker reply timeout | 30 seconds |

Selection first prefers acceptance, then fewer collision vehicles, fewer
overspeed decisions, more nominal episodes, higher mean team reward and fewer
interventions. Acceptance requires zero colliding vehicles, zero decisions above
the ceiling plus 0.00001 m/s tolerance, at least 600 m mean progress per car,
at most 5% overridden decisions and nominal completion of every evaluation
episode. Collision counts include ordinary vehicles. The progress threshold
is a 10 m/s average over 60 seconds, not continuous driving at 60 mph. Acceptance
does not prove superiority to the baselines.

The wall-time deadline is checked between bounded worker operations/updates;
shutdown can extend slightly beyond it. Timeout preserves the latest saved
checkpoint and reports incomplete/failed evaluation. Resume restores matching
model, optimizer and RNG states but starts fresh episodes; mid-episode SUMO state
is not restored. A new run package preserves the parent and selects its own best
snapshot using fresh validation. Fewer than four remaining decisions cannot
start a fresh four-agent episode and are rejected at resume.

## Verified no-learning preflight

Run `preflight-20261026T002632Z-df0749c6` (October 25 local time) retains resolved
config, source copies/hashes, versions, exact command, generated demand, metrics
and compressed action traces under ignored `outputs/sumo_mappo/v01/<run-id>/`.
Logs are under `logs/sumo_mappo/`. A compact
[evidence summary](evidence/sumo-mappo-setup-20261025.json) is published.

Each controller completed two full episodes on seeds 500001 and 500002. An extra
scripted repeat exactly matched its first episode metrics. Four-worker collection
produced 512 vehicle decisions in 128 environment steps, all finite, with
unchanged-policy log-probability error at most 0.000000476837. Actor export
matched its source model, parameters stayed unchanged and workers closed
without forced cleanup. Total preflight wall time was 24.969 seconds.

| Controller, two episodes each | Mean progress per car | Overrides | Colliding vehicles / overspeed decisions | Driving acceptance |
| --- | ---: | ---: | --- | --- |
| SUMO-native | 1,581.756 m | 0% external, by definition | 0 / 0 | Pass in this fixture |
| Scripted | 1,580.709 m | 9.25% | 0 / 0 | Fail: interventions |
| Random actions | 122.264 m | 46.667% | 0 / 0 | Fail: progress and interventions |
| Untrained actor | 109.862 m | 77.979% | 0 / 0 | Fail: progress and interventions |

This verifies setup plumbing, not learned driving. Preflight seeds are separate
from final tests. Native reward comparisons need care because native control
never receives external-override penalties. The acceptance criterion was not
loosened after seeing the scripted failure. This headless experiment produces
no screenshots and makes no new 3D scene or visual claim.

## Commands, artifacts and quiet execution

Run from the checkout root in isolated traffic Python, never Isaac's Python.
Existing pinned traffic requirements contain the dependencies. On the
maintainer's Windows checkout, use `.\.venv-traffic\Scripts\python.exe`:

```text
python experiments/run_sumo_mappo.py
python experiments/run_sumo_mappo.py --mode preflight
```

The default command displays configuration only. Preflight is bounded and never
constructs an optimizer. **After approval only**, actual training uses:

```text
python experiments/run_sumo_mappo.py --mode train --approved-config-sha256 0c8e70ef98827a27ed5c77dc680ea35739811cdbbca045e8fb76af9fd2d3265c
```

An approved run saves `outputs/sumo_mappo/v01/train-<UTC>-<id>/policy.pt` (best
validation actor), `checkpoints/best.pt`, `checkpoints/latest.pt`, config,
episode/update metrics, action traces, status and results. The actor is
TorchScript with 27 local inputs; Torch deprecates that export API, but the
roundtrip passes. Checkpoints/logs remain local and are not in GitHub. Preflight's
`untrained-actor-interface-test.pt` is not a usable trained driving policy.

The runner prints only the run location and final status. Progress, worker output
and SUMO warnings go to files. Collection, optimization, checkpointing, validation,
held-out tests and shutdown are self-contained. No chat poller or automatic
retraining loop is installed. After a later approved launch, wait for the process
to finish and inspect results on completion/failure; continuous chat analysis
during optimization is unnecessary.

## Review and next gate

Subagents implemented the environment and learning math; another independently
reviewed actor/critic separation, rollout semantics and lifecycle. Review fixed
acceptance ranking, save-before-validation ordering, native test temporary
paths, quiet SUMO logging, collision naming and owned process cleanup. POSIX
forced process-group cleanup was code-reviewed, not exercised on this Windows
host. Final targeted tests: **90 passed**. Broad regression: **2,724 passed,
215 skipped, 116 subtests passed**; two unrelated pre-existing untracked private
test files were excluded. Optional Isaac/USD runtime skips remain explicit.
The final targeted pass covers the seed-bound and resume-budget guards added
after preflight. Detailed source hashes and evidence are in the linked summary.

After user approval, run the bounded pilot and compare held-out results against
the baselines. A later transfer study must reconstruct the same observations,
translate acceleration through a calibrated physical actuator controller, keep
a lane controller and validate one physical car before expanding. Replaying
SUMO positions in Omniverse proves visualization only. Controller transfer,
merging, jump-in obstacles, sensor noise, communication ablations and a matched
IPPO comparison remain subsequent work. No coordination gain is claimed.

## October 25: first training attempt and spawn repair

The approved run `train-20261026T041059Z-5ae53d79` stopped after 360.062 seconds
with 659,456 checkpointed decisions and 10,304 optimizer steps. Seed 100277
triggered safe delayed insertion: one controlled car needed a second SUMO step,
but reset incorrectly required all four cars on the first step. The failure and
its saved best/latest checkpoints remain intact. Best validation at update 150
had zero collisions/overspeed but 81.279% interventions, failing acceptance;
final held-out evaluation did not run.

Reset now permits at most 100 native warm-up steps (10 simulated seconds),
retaining insertion safety and the original seed/demand. It rejects collisions,
teleports, controlled arrivals or exhausted warm-up. The episode clock and
learning budget start only after all four agents exist. Runtime evidence records
the actual warm-up duration. The failed seed now resets identically twice in
two steps (0.2 seconds). All 37 targeted backend/environment/runner tests passed.
The approved training configuration and digest are unchanged. Resume uses the
latest checkpoint and only the remaining original decision budget, with fresh
episodes as documented. The historical single-step preflight remains unchanged.
