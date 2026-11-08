# Population v04: raw near-limit acceleration correction

November 5, 2026. **Preparation verified; training is disabled.** This is population
v04, distinct from the earlier SUMO driving-policy v04/v04r2. The best phase-0
actor has not changed and still fails intervention acceptance. This milestone
prepares learning to remove its unnecessary acceleration at the speed limit;
it does not claim that the unchanged weights have learned that behavior.

## Evidence and diagnosis

The [physical calibration](mappo-physical-adapter-v01.md) measured speed-ceiling
corrections on **312/380 cruise decisions (82.11%)** and **161/580 passing
decisions (27.76%)**. The actuator tracks ordinary acceleration accurately; the
actor requests acceleration that cannot be applied at the limit.

The [frozen audit](evidence/near-limit-v01/diagnostic-findings.md) found:

- Training layouts used 50/60 mph, while physical fixtures used 12/18 m/s
  (26.8/40.3 mph). Explicit lower-limit coverage was missing. Positive requests
  also occur at 50/60 mph, so this is not solely a speed-domain problem.
- Previous raw acceleration requests strongly affect the next request. With
  otherwise identical 12 m/s at-limit inputs, previous requests of -0.5, 0,
  +0.5 and +1.0 m/s² produce -0.486, +0.093, +0.589 and +0.983 m/s². This suggests
  persistence of acceleration requests; no closed-loop causal ablation is claimed.
- SUMO and the physical bridge both supply the previous **raw** request. No
  raw-versus-filtered mismatch was found. The v3 teacher already targets
  -0.080 m/s² at the exact limit in the empty-road probes.

The actor export remains `policies/versions/highway-stage1-v03-phase0/policy.pt`,
SHA-256 `8526f6134b7eae90d3370bd12cc0487926a6ec4356c6c6ea2dd231e8aa8b1bb8`.
The full warm-start checkpoint hash is
`b77f55ae7262178ce0fa15729fe844e18693cd168905eea8b1bbd579c6913ab1`.

## Implemented correction

1. Added an opt-in actor loss on **raw deterministic acceleration**, using the
   existing observation-only v3 teacher target. Select active rows whose speed
   is at least `local allowed limit - 1.0 m/s`, including overspeed rows.
   The additional loss is `0.1 * mean((a_raw - a_teacher)^2)` over selected rows,
   with acceleration in m/s². Empty selections contribute differentiable zero.
   Existing following and merge targets remain applicable; the new term does
   not replace them with a constant zero target.
2. Added episode speed ceilings of **12, 18, 22.352 and 26.8224 m/s**. Every four
   successive episodes cover all four, independently shuffled from the existing
   population/task schedule. Apply `min(native lane limit, episode ceiling)` to
   every lane and connector **before the first simulator step**, verify native
   readback, and update graph speed fields. Departure and slow-leader speeds
   follow the same ceiling. A ceiling never raises an existing lower limit.
3. Retained separate raw-action, teacher-error, speed-ceiling-excess and at-limit
   positive-request metrics. PPO updates record near-limit selected-row exposures
   (including repeated epochs), exposure-weighted physical errors, and a separately
   labeled conditional per-step mean. Native traffic receives no fictitious
   policy-request metric. Initial/final learned evaluations repeat the same
   synthetic boundary probes.

The existing safety guards, 60 mph cap, 1.5 s headway, action mapping and inference
code remain unchanged. No correction is hidden inside the physical adapter.
The loss defaults to disabled, preserving previous configurations. Existing v3
teacher/reward alignment and actor/critic gradient diagnostics remain enabled.

Physical-unit MSE is 10.5625 times normalized-action MSE. The new coefficient
0.1 therefore must not be compared numerically with imitation coefficient 0.04
without accounting for units and conditional sampling. Global gradient clipping
remains 0.5; its effect must be assessed after actual critic warm-up. Shared actor
features can change lane behavior even though this loss has no direct lane-logit
derivative, so passing and merge checks remain mandatory.

## Verification performed without learning

- **171 tests pass** across objective derivatives, speed curriculum, runner,
  diagnostics, legacy control, population masking and resume accounting. Three
  TorchScript deprecation warnings were retained.
- **352 synthetic developmental probes**, with 296 selected rows, confirm every
  nonzero direct acceleration derivative points toward its teacher target.
  Outside-band and direct lane-logit derivatives are zero. Weights, actions,
  checkpoint bytes and RNG remain unchanged. These probes and saved physical
  traces are diagnostic evidence, not training samples or held-out qualification.
- Final SUMO preflight passed in **27.657 seconds**, with **zero optimizer steps**.
  It covers teacher/random cohorts of 1/2/4/6/8 cars, all four ceilings, finite
  128-row frozen collection, exact teacher repeat, PettingZoo API/seed checks,
  zero-AV native traffic and exact frozen actor export. Every recorded lane-limit
  readback matched the requested capped value. No source drift occurred.
- The frozen 128-row batch exercises the new objective: weighted loss 0.165302,
  actor gradient norm 1.426064, PPO/near-limit gradient cosine 0.877400. These are
  pre-warm-up derivatives, not parameter updates or evidence of eventual learning.
- The teacher's eight-AV I-110 merge fixture records **one background emergency
  braking event**, `ramp_0` at 5.2 s, 9.0 m/s². It is retained by native log
  accounting. Preflight is a plumbing gate; it does not assert nominal driving
  acceptance, zero emergencies or completion of full-horizon evaluation.

The final preflight is
`outputs/highway-population/v04/preflight-20261105T214741Z-d8274460`.
[Compact evidence](evidence/near-limit-v01/preflight.json) includes per-case raw
metrics, native events, runtime/source hashes and the frozen gradient report.
Full logs, road readbacks and source snapshots remain in that local output.
Runtime: Python 3.11.9, PyTorch 2.14.0+cpu, SUMO/TraCI 1.27.1.
No new Isaac run or model training was launched.

## Final configuration for review

Source: [highway_population_v4.json](../experiments/configs/highway_population_v4.json).
[Evidence copy](evidence/near-limit-v01/reviewed-config.json) is identical.

| Setting | Resolved value |
| --- | --- |
| Initialization | Frozen best phase-0 actor and exploration log standard deviation; fresh masked critic and optimizer |
| Policy | Shared MAPPO; 93 actor inputs, 752 critic inputs, eight maximum controlled cars |
| Actions / rate | Acceleration -4.5 to +2 m/s² and masked lane delta -1/0/+1 at 10 Hz |
| Roads / tasks | Training-only 2–5-lane roads and I-110; cruise, follow, pass, merge |
| Episode ceilings | 12 / 18 / 22.352 / 26.8224 m/s, limited further by native lane limits |
| Population | 1/2/4/6/8 controlled cars, 8/16/24 total; fixed cohort within an episode |
| Communication | None, local, or local with 30% packet loss; six nearest received slots, 150 m, 0.1 s delay, 0.5 s expiry |
| Learning budget | At most 100,000 active-agent decisions, including 4,096 critic warm-up decisions |
| Wall-time ceiling | 1,800 s total including validation; 600 s reserved for final evaluation |
| Episode length | 800 steps / 80 simulated seconds |
| Compute | CPU, one worker, one Torch thread |
| Learning rates | Actor 0.0001; critic 0.0003 |
| PPO | Four epochs; minibatch 256; policy/value clip 0.2; target KL 0.02; global gradient norm 0.5 |
| Objective weights | Value 0.5; entropy 0.001; imitation 0.04 with acceleration/lane relative weights 1/10; new near-limit SI MSE 0.1 within 1 m/s of limit |
| Seeds | Training 9307; scenario seeds 6600000–6600999; new validation seeds 6700000–6700007 |
| Evaluation | Original 16 cases preserved exactly, plus eight 12/18 m/s cruise/passing cases with 1/5 AVs; matched deterministic initial/final runs |
| Training authorization | **false** |

Reviewed canonical-JSON SHA-256:
`a34f3aa17ce055037aad1510b8171b977a6c310e675fd2ea60e9d2ecadca8f3f`.
Preparation SHA-256, excluding authorization/readiness:
`d593349b383e7298577f2a75195828489e7507989ee403aeea4991d0286bf0f9`.
Readiness binds the exact preflight summary and runtime source hashes. A future
authorized launch must retain a launch copy and its resolved checksum; the
tracked proposal stays disabled. Failed reservations and resumed time remain
charged to the same bounded run.

## Next gate and retained metrics

After approval, run one bounded pilot and let its process retain metrics without
assistant polling while learning. Inspect only completion or failure; repair a
runtime fault and resume a valid checkpoint within the existing budget, never
silently change objectives or start another attempt. Save `last.pt`, initial/final
validation, native logs, gradient/clipping diagnostics, sampled coverage, raw
boundary reports and per-case comparisons under `outputs/highway-population/v04/`.

Acceptance remains per case: no collisions, teleports, overspeed or native
emergency events, complete traffic accounting, overrides at most 5%, cruise at
least 90% of limit, at least one pass in passing cases, and at least 90% of
controlled merge targets. Raw error/positive-request metrics supplement these
gates and do not redefine intervention counts. Collection metrics reflect sampled
actions; deterministic evaluation and auxiliary-loss means are labeled separately.
Clipped 60 mph speed features cannot quantify above-cap overspeed magnitude;
native safety accounting remains authoritative.

This pilot may not cover the full population/task/speed factorial. It is a
single-seed developmental comparison, not held-out acceptance. If learning
improves the candidate, repeat matched physical cruise/passing with the calibrated
adapter before making transfer claims. Sensor inputs and live fleet handoff remain
separate work. No trained policy is promoted by this preparation milestone.
