# Stage 1: structured highway training preparation v02

November 2, 2026 local (run identifiers use November 3 UTC).
Parent: [living sensor-driven roadmap](sensor-driven-training-roadmap.md).

This milestone prepares the next learning attempt. No real behavior cloning or
MAPPO training has started. The historical v1 runner/checkpoints and accepted
v04r2 remain separate. SUMO owns motion; this is not physical steering or
camera/LiDAR-driven control.

## Problem established from saved evidence

The previous final actor requested more than 1.8 m/s² on all **38,400** validation
decisions. **37,034 (96.44%)** decisions had an override, including 22,825 requested
versus filtered differences and 16,126 filtered versus realized differences;
those categories overlap. The old logs do not identify native causes, so the
offline audit does not invent them. See the [historical pilot](highway-curriculum-training-v01.md).

`experiments.summarize_highway_actions` audits retained compressed journals without
simulation or learning, preserves per-journal hashes, and handles genuinely
unobserved terminal acceleration separately. Audit artifacts live under
`outputs/highway-stage1/v02/`.

## Implementation

- A separate `HighwayStage1Env` and schema
  `highway_stage1_graph_messages_v2` preserve the original 33 local fields, add
  12 merge/feasibility fields and retain six eight-field delivered-message slots:
  **93 actor observations; 376 centralized critic fields for four agents**.
- The added fields describe a known connected target lane, index/delta, distance
  to its start, projected front/rear gaps and relative speeds with presence flags,
  current feasible acceleration ceiling and target speed limit. Background cars
  approaching on another incoming road are included using graph distance, not
  unrelated lane-position origins. No future vehicle state is queried.
- Lane masks remain at indices 30–32. Projection can expose an upcoming target
  before it is adjacent; that does not authorize a premature lane change.
  Fixed analytic scaling, schema, model layout and actor-only export are explicit.
  An old 81-field checkpoint cannot silently load into this actor.
- Journals retain requested, filtered and measured acceleration, lane requests,
  accepted commands, observed changes, individual analytical bounds, constraint
  violations and actual override causes. Native SUMO changes remain unattributed
  unless separately diagnosed. Native-baseline counterfactual requests are labeled.
- A deterministic teacher consumes only the declared actor observation. It tapers
  acceleration, follows feasible gaps, passes only when another lane offers a
  useful speed advantage, and aligns with the projected merge gap. Teacher quality
  is checked separately from finite observations and API correctness.
- Rewards retain progress/arrival/pass/merge, collision, intervention, jerk and
  lane-change terms; add feasible-acceleration tracking, unsafe-request penalty
  and a bounded team merge bonus. The 60 mph cap, existing 1.5 s headway/stopping
  constraints, and original override thresholds remain in force.
- The new runner separates demonstration/BC data from fresh on-policy PPO
  rollouts. Any teacher labels on student-visited states enter a separate decaying
  imitation loss. Failed demonstration episodes are retained and not admitted to
  BC. It reserves transition budgets before episodes, saves optimizer/RNG/curriculum
  state and immutable demonstration versions, and enforces aggregate resume limits.

The current acceleration ceiling uses exact SUMO state and analytical guards.
It is a Stage 1 development input; Stage 2 must replace its inputs with estimates
and audit masks/guards for privileged-information leakage.

## First development probe, preserved

Run: `outputs/highway-stage1/v02/teacher-probe-20261103T033241Z/`.
Eight training-layout cases, fixed diagnostic seeds, 61.92 seconds, zero optimizer
steps, unchanged runtime hashes. No held-out test was used. These scripted results
are not a matched-seed learned-policy improvement over the previous pilot.

| Layout/task | Overrides | Speed / limit | Passes | Merges |
| --- | ---: | ---: | ---: | ---: |
| Generated 2-lane cruise | 0.00% | 98.90% | 0 | n/a |
| Generated 2-lane following | 28.25% | 84.70% | 0 | n/a |
| Generated 2-lane passing | 15.75% | 67.86% | 2 | n/a |
| Generated 2-lane merge | 2.79% | 96.75% | 0 | 2/2 |
| I-110 cruise | 0.00% | 93.14% | 0 | n/a |
| I-110 following | 44.04% | 83.09% | 0 | n/a |
| I-110 passing | 36.17% | 82.21% | 4 | n/a |
| I-110 merge | 10.75% | 87.88% | 0 | 2/2 |

All eight had zero collisions, teleports and overspeed. Nevertheless, this teacher
failed the 5% intervention gate. All 2,527 overrides in 17,198 decisions came from
measured native acceleration reductions; there were no rejected lane requests
or analytical-filter changes. The original probe and action audit are preserved.

SUMO ACC uses distinct speed, gap and gap-closing behavior with hysteresis. That
means an acceleration allowed by the analytical safety bound can still be reduced
by the native car-following controller. Diagnostic queries must be separated from
deployable actor inputs. [Official SUMO ACC documentation](https://sumo.dlr.de/docs/Car-Following-Models/ACC.html).

The next teacher version predicts the configured ACC response using only its
existing speed/gap observations, including a conservative choice when hysteresis
state is unavailable. Three matched probes reduced following overrides from
28.25% to 0.3125%, I-110 passing from 36.17% to 2.749%, and I-110 merging from
10.75% to 2.366%. Speed and task outcomes were essentially preserved. This fixes
command feasibility against the configured simulator model; it is not proof of
transfer to a different vehicle controller. An initial probe-harness active-agent
lifecycle error and its subsequent rerun are preserved. Earlier development
probes retain hashes and journals, but not complete original source snapshots;
the full preflight saves actual runtime sources.

## Full preflight exposed a road-fixture error

`preflight-20261103T034423Z-9b8be95f` completed in 402.34 seconds with zero optimizer
steps and no source drift. API, seed, exact teacher repeat, finite PPO collector
and actor export checks passed. The teacher passed 27/30 cases; all 30 had zero
collisions, teleports and overspeed, and all 30 ramp vehicles merged. Three
generated three-lane merge variants failed the intervention gate (17.03–23.24%).
The summary explicitly records `teacher_ready=false` despite passing plumbing.

Inspection found the generated ramp approaching from east/left while joining
the west/rightmost auxiliary lane, so its internal path crossed the through
lanes. SUMO correctly required junction yielding. The teacher's target-lane gap
inputs do not describe that unintended intersection. A separate v02 generator now approaches from the road's right side, using its
local tangent/normal and lane extent. It also asks netconvert for lane-level foe
checks, avoiding false whole-edge conflicts while preserving actual auxiliary
merge yielding. Ten layouts passed structural checks, and two independent builds
produced byte-identical normalized networks. Original v01 networks/results and
imported I-110 remain unchanged.

The three previously failing merge probes now show 1.87–1.90% overrides, all
six ramp vehicles merged, and zero collisions, teleports or overspeed. Their
scenario geometry changed; these are not controller-only comparisons on identical
networks. The first failed geometry compile and every driving failure remain
recorded. See the [geometry verification](evidence/highway-stage1-v02/geometry-verification.json)
and [corrected probe](evidence/highway-stage1-v02/right-ramp-probe.json).

## Proposed training configuration

The executable configuration is
[`highway_stage1_v2.json`](../experiments/configs/highway_stage1_v2.json).
The complete current-source preflight and teacher-quality evidence passed.
Training remains disabled pending user approval of this resolved configuration.

| Setting | Proposed value |
| --- | --- |
| Algorithm | Shared hybrid actor, centralized critic MAPPO; BC warm start |
| Device | CPU, one worker, one Torch thread |
| Observations/actions | 93 observations; acceleration −4.5 to +2 m/s² and right/stay/left |
| Budget per attempt | 500,000 active-agent transitions or 7,200 seconds, first reached |
| Demonstrations | Up to 50,000 transitions, included in the total; train-only scenarios/seeds |
| Behavior cloning | Four epochs, batch 512, learning rate 0.0003, initial log std −2 |
| PPO allocation | 100k cruise/follow; 100k passing mixed with earlier tasks; 125k easy merge/mixed; 125k dense merge/mixed |
| PPO optimizer | Actor LR 0.0001; critic LR 0.0003; four epochs; batch 256; clip 0.2; KL 0.02 |
| Imitation assistance | Initial weight 0.05, decays over 200k PPO transitions; lane-change label weight 10 |
| Evaluation | Four-case sentinel each 50k transitions; full 12-case validation after BC and phase boundaries; validation-only selection |
| Test | Existing reserved test layouts/seeds remain untouched until a frozen quality-accepted candidate |

BC, validation and failed/resumed work count toward the attempt budget. A second
attempt is not launched automatically. Training-time metrics/checkpointing live
in the runner, allowing the agent to wait for completion/failure without polling.
Stopping at a budget is runtime completion, not driving acceptance.

From the project root, print the resolved proposal without training:

```powershell
.\.venv-traffic\Scripts\python.exe -m experiments.run_highway_stage1 --mode config
```

Bounded no-learning validation uses `--mode preflight`. Future checkpoints and
metrics belong under `outputs/highway-stage1/v02/train-<timestamp>-<id>/`.
There is no new trained Stage 1 policy to replay at this milestone.

## Final preparation result

The final full preflight `preflight-20261103T040408Z-52a6d162` passed in
412.84 seconds with **zero BC/PPO optimizer steps**, no source drift and no
held-out traffic evaluation. All 30 scripted teacher cases passed the unchanged
per-case gates.

| Measure | Observed result |
| --- | --- |
| Collisions / teleports / overspeed | 0 / 0 / 0 |
| Override decisions | 1,004 / 82,454 (1.22% pooled); worst case 4.39%, below 5% |
| Cruise speed ratio | Minimum 94.87% across five cruise cases |
| Passing | All five passing cases successful; 16 passes total, at least two per case |
| Ramp merging | 30/30 across 15 cases, including planned light/dense variants |
| Environment/actor plumbing | Parallel API, seeded repeat, exact teacher repeat, finite collector and actor-only export passed |
| Regression suite | 243 passed; 21 PyTorch deprecation warnings |

See [per-seed results](evidence/highway-stage1-v02/preflight-final.json),
[verification and runtime versions](evidence/highway-stage1-v02/verification.json)
and the [disabled configuration snapshot](evidence/highway-stage1-v02/final-config.json).
The raw summary, journals, actual source snapshots, configuration and working-tree
diff are retained in the local run directory. Compact evidence is committed; bulk
outputs and the generated network bank are not a GitHub artifact release.

The final disabled configuration canonical SHA-256 is
`7b53e370d1b3c3a04d0ac2498974662414a7b939a3a740ab3654c5b595125494`. Readiness validation passes against the exact runtime
sources and preparation scope. Authorization remains false; enabling it later
changes the full configuration digest and requires reviewing that launch digest.

These are scripted development results, not a trained-policy success or a
communication-benefit claim. The teacher predicts the configured SUMO ACC response;
its conservatism and simulator calibration may not transfer to a physical car.
The actor still sees exact local simulator/map state. Learned per-task validation,
untouched held-out testing, communication ablations, independent training seeds
and sensor/physical validation remain open. Stage 1 is prepared, not complete.

When a reviewed training run is launched, `last.pt` preserves resume state.
`selection.json` points to an immutable `selected-<update>-<id>/` generation
containing `policy.pt`, `best.pt`, `policy-contract.json` and `validation.json`.
No selected actor exists until full validation has produced one. Automated
selection distinguishes ranking from actual quality acceptance.
