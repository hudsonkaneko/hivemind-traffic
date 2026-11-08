# Stage 1 v03: acceleration diagnostics and training revision

November 3, 2026. Parent: [living roadmap](sensor-driven-training-roadmap.md).
The prior [v02 learning run](highway-stage1-training-v02.md) finished but failed
intervention acceptance. This milestone implements diagnosis, checkpoint ranking
and a disabled training proposal. No v03 policy has been trained.

## What the saved evidence establishes

The offline audit reconciles the original metric exactly: 6,965 final overrides,
including 6,962 native acceleration reductions. Of those, **94.90%** realized
accelerations match the existing logged feasible target within 0.05 m/s². Native
reductions have median 0.175, mean 0.334 and p95 0.767 m/s²; 257 exceed 1 m/s².
Runs of mismatch last up to 79.5 seconds. This is often a sustained control
disagreement rather than isolated rounding.

6,072 native changes still realize nonbraking acceleration (at least −0.05 m/s²),
while 890 realize braking. The logs do not identify internal native control modes
or prove near-collision causes. Merge changes have lower feasible-target agreement
(about 59–63%) than following; preserve them as a distinct unresolved mechanism.
Thresholds and original override classifications are unchanged.

The frozen after-BC actor was also evaluated on its 49,998 retained **training**
demonstrations, without learning or simulation. It agrees with 99.886% of lane
labels, but acceleration RMSE is 0.2013 m/s², MAE 0.0956 m/s², and 47.17% of
labels differ by more than 0.05 m/s². Training-label error is not a driving
override rate; it exposes an acceleration-fitting weakness despite good lane fit.

Evidence: [override audit](evidence/highway-stage1-v03/override-audit.json),
[BC fit](evidence/highway-stage1-v03/bc-fit.json). Both preserve input hashes and
scopes; neither fits a model. The audit CLI refuses overwrite and reconciles
per-case counts before reporting derived distributions.

## Teacher on the same validation matrix

A bounded 152.891-second run used the exact twelve validation cases/seeds from
v02, with the original runtime sources frozen and zero optimizer steps. All
primary horizons/accounting checks completed, with no source drift.

| Measure | Scripted teacher | Selected v02 actor |
| --- | ---: | ---: |
| Pooled overrides | 448/38,353 = 1.168% | 6,965/38,380 = 18.147% |
| Worst case | 4.156% | 53.563% |
| Ramp merges | 6/6 | 6/6 |
| Total passes | 10 | 12 |
| Collisions / teleports / overspeed | 0 / 0 / 0 | 0 / 0 / 0 |
| Two-lane following speed / limit | 69.51% | 81.02% |
| Two-lane passing speed / limit | 67.10% | 94.95% |

The teacher passes every unchanged gate on the same roads, supporting a learning
gap hypothesis. It is also slower in the two-lane cases; copying its behavior
more closely can trade efficiency for fewer interventions. Review per-case
speed and complete cohort burden alongside the intervention metric. No validation
trajectory is admitted as demonstration or PPO training data. The untouched test
partition has not been run. [Matched evidence](evidence/highway-stage1-v03/matched-teacher.json).

## Implemented selection and learning changes

Ranking version 5 rejects incomplete horizons, invalid/inconsistent trip ledgers,
missing/nonfinite metrics and incomplete validation matrices. Its order is:
actual safety events; existing task failures; number of override-failing cases;
worst and pooled override rate; unfinished trips/cohort time; speed/pass/reward
tie-breaks. Thus marginal cruise speed cannot outrank materially worse overrides
after the driving requirements are met. No arbitrary following-speed or arrival
threshold was introduced; following slowdown still requires review.

Offline reranking selects the retained **phase-2** candidate (12.473% pooled,
46.625% worst) over the final actor (18.147%, 53.563%). Both have eight failing
cases and remain unaccepted. Original run selection and published policy versions
remain immutable. [Reranking evidence](evidence/highway-stage1-v03/reranking.json).

The runner selects v5 ranking only for the explicit v03 configuration. Existing
v02 settings retain v4 behavior. v03 increases BC acceleration-loss weight and
epochs, retains teacher assistance later, logs physical acceleration fit error,
binds readiness to those settings and writes into a separate v03 run directory.
Source changes invalidate earlier readiness/resume fingerprints as intended;
old run source snapshots remain available for exact historical reproduction.

## Final proposal for review

| Setting | Previous v02 | Proposed v03 |
| --- | --- | --- |
| Actor/critic inputs | 93 / 376 | 93 / 376 |
| BC epochs | 4 | 12 |
| BC acceleration loss weight | 1 | 4 |
| Imitation assistance | 0.05 → 0 by 200k PPO transitions | 0.05 → 0.01 by 400k; retain 0.01 |
| Checkpoint ranking | Competence, then marginal speed before overrides | Existing task gates, then per-case intervention failures |
| Demonstrations | Up to 50k fresh train-only decisions | Same allocation and paired seeds |
| PPO allocation | 100k foundation, 100k passing, 125k easy merge, 125k dense merge | Same fixed mixed curriculum |
| Total attempt cap | 500k agent decisions or 7,200 seconds | Same, including demos/evaluation/recovery |
| Device | One CPU worker / one Torch thread | Same |
| Protection/gates | 60 mph; existing following/stopping guards; ≤5% overrides in every case | Same |

BC LR stays 0.0003 and batch 512; PPO actor/critic LR 0.0001/0.0003, four epochs,
batch 256, clip 0.2 and KL 0.02. The sparse lane-label weighting remains 10.
The increased acceleration weight applies to BC, while PPO retains its existing
auxiliary loss composition with the revised retention schedule. Rewards and
observations are unchanged. This is one proposed combined pilot; effects of BC
epochs, weighting and retention are not separately attributed causal ablations.

Config: [highway_stage1_v3.json](../experiments/configs/highway_stage1_v3.json).
Training authorization remains false. Print the resolved configuration from the
project root with:

```powershell
.\.venv-traffic\Scripts\python.exe -m experiments.run_highway_stage1 --config experiments/configs/highway_stage1_v3.json --mode config
```

If approved later, the runner owns logging/checkpointing and fixed evaluations;
the chat waits for completion/failure. A revised run is a new attempt, not an
invisible resume of the exhausted v02 budget. No new weights are added to the
policy catalog until such a run actually produces a frozen actor.

## Verification and retained artifacts

98 relevant regression checks passed (12 PyTorch deprecation warnings). The
complete frozen-source preflight `preflight-20261103T210301Z-7c9a12cd` passed
all 30 teacher cases in 380.265 seconds, with zero BC/PPO optimizer steps and
no source drift. Worst-case overrides remain 4.386%, below the unchanged 5%
limit; API, seed/repeat, finite collector, actor export and geometry checks pass.
The separate twelve-case matched validation above also passed. Neither run
trained a v03 actor or exercised the held-out test partition.

Readiness validates against the exact current runtime. Disabled config canonical
SHA-256: `7c6cb1922727f73b62c502deeb8c1417f769ad7ec3aca6363ad782b8c884428b`. [Final config snapshot](evidence/highway-stage1-v03/final-config.json),
[preflight](evidence/highway-stage1-v03/preflight.json) and
[verification/runtime versions](evidence/highway-stage1-v03/verification.json).
Compact evidence is committed; complete journals, source snapshots, harnesses
and raw reports remain in their unique `outputs/highway-stage1/v03/` directories.
The catalog has no new v03 weights because learning has not started.
