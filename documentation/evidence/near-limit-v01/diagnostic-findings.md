# Frozen near-limit diagnostic

November 5, 2026. This is a CPU-only, zero-update audit. No new simulator run or
training occurred. The existing physical validation traces were read for
diagnosis only and were not converted into training samples.

The frozen phase-0 export remains
`policies/versions/highway-stage1-v03-phase0/policy.pt`, SHA-256
`8526f6134b7eae90d3370bd12cc0487926a6ec4356c6c6ea2dd231e8aa8b1bb8`.
The unchanged actor still needs improvement; these results verify the proposed
loss direction and record a baseline, not learned improvement.

## Saved physical evidence

Source: `outputs/mappo-physical-adapter/v01/20261105T204908Z-32331979`.
All recorded interventions in these two fixtures were speed-ceiling corrections.

| Fixture | Decisions | Corrected decisions | Corrections above 0.05 m/s² | Median correction | Maximum correction |
| --- | ---: | ---: | ---: | ---: | ---: |
| Cruise | 380 | 312 (82.11%) | 312 | 0.944 m/s² | 1.166 m/s² |
| Passing | 580 | 161 (27.76%) | 146 | 0 m/s² | 0.844 m/s² |

Cruise had 311 requests above +0.05 m/s² while the observed speed was within
0.05 m/s of, or above, its limit; passing had 151. Tiny numerical interventions
do not explain the cruise failure. Counting the raw requested action separately
from the filtered and realized action is essential.

## Coverage and observation contract

The best checkpoint's saved training configuration names 50/60 mph training
layouts, 55 mph validation layouts, and 45/60 mph test layouts. The physical
fixtures use 12/18 m/s (26.8/40.3 mph). This is missing explicit lower-limit
curriculum coverage, not proof that no lower-limit connector was ever visited.
It is also not the sole cause: positive at-limit requests occur at 50/60 mph in
the probes below.

Both SUMO and the physical bridge expose the previous **raw requested**
acceleration. SUMO stores `requested[agent]` in its previous-action state, so no
raw-versus-filtered mismatch was found in this field. The frozen actor is
sensitive to it: at 12 m/s on lane 0, with all other probe fields fixed, previous
requests -0.5, 0, +0.5 and +1.0 m/s² produce new requests -0.486, +0.093, +0.589
and +0.983 m/s². This is a diagnostic sensitivity, not an explanation established
by a closed-loop ablation.

## Synthetic boundary probes

`experiments/diagnose_near_limit.py` constructs 352 current-state observations
through the same 93-field physical bridge: 328 empty-road combinations of speed
limit, speed offset, lane and previous request, plus 24 slower-leader states.
They are developmental sentinels, neither held-out evaluation nor a training
dataset. Probes above the universal 60 mph speed-feature ceiling are omitted
because that feature clips and would not represent distinct speeds.

At-limit rows below include two lanes and four previous requests each. A positive
request means greater than +0.05 m/s². The actual v3 teacher target at the exact
limit is -0.080 m/s² in these states: its existing v2 baseline is more restrictive
than the v3 speed-margin cap of -0.040 m/s².

| Limit (m/s) | Mean raw request (m/s²) | Raw range (m/s²) | Positive requests | Near-limit teacher MSE ((m/s²)²) |
| ---: | ---: | --- | ---: | ---: |
| 6 | +0.358 | -0.427 to +1.063 | 6/8 | 0.753 |
| 9 | +0.308 | -0.489 to +1.026 | 6/8 | 0.698 |
| 12 | +0.256 | -0.551 to +0.983 | 5/8 | 0.642 |
| 15 | +0.202 | -0.611 to +0.937 | 4/8 | 0.586 |
| 18 | +0.147 | -0.667 to +0.888 | 4/8 | 0.533 |
| 22.352 | +0.069 | -0.741 to +0.812 | 4/8 | 0.463 |
| 26.8224 | -0.008 | -0.806 to +0.732 | 4/8 | 0.310 |

The MSE column includes empty-road probes from one m/s below the limit through
0.1 m/s above it (the above-limit 60 mph rows are omitted). The six and nine m/s
levels are additional low-speed diagnostic probes, not proposed training caps.

## Exact loss-direction check

The final v4 configuration supplies coefficient **0.1** and band **1.0 m/s**.
The tool differentiates the actual `MappoTrainer._near_limit_loss` at frozen
weights against v3 teacher targets. It never calls an optimizer step or
`backward()`. The selected-row predicate exactly matches the trainer's float32
comparison at the inclusive boundary.

- 296 selected rows had a nonzero error; all 296 derivatives point toward their
  teacher target under gradient descent.
- Weighted loss: **0.0656389222 (m/s²)²**.
- Direct lane-logit derivatives and all outside-band derivatives were zero.
- Model parameters, inference actions, checkpoint bytes and RNG were unchanged;
  parameter gradients remained unset and optimizer state remained empty.
- The physical-unit MSE scale is **3.25² = 10.5625** times normalized-action MSE.
  Because it is a conditional mean over selected rows, its relative weighting
  also depends on the selected fraction of a future minibatch.

Zero direct lane-logit derivative does **not** prove lane behavior will be
preserved after future shared-trunk updates. Matched complete cruise, following,
passing and merge evaluation remains required. This audit also does not measure
interactions with the PPO, imitation or critic gradients over real training data;
those remain separately logged diagnostics.

Four diagnostic tests passed. The reusable `boundary_report(actor)` function is
available to the runner before and after a future authorized training run.

## Reproduction and retained evidence

Run from the project root using the isolated traffic environment:

```powershell
.venv-traffic/Scripts/python.exe experiments/diagnose_near_limit.py --config experiments/configs/highway_population_v4.json
.venv-traffic/Scripts/python.exe -m pytest -q tests/test_near_limit_diagnostics.py
```

The final offline run is
`outputs/near-limit-v01/diagnostic-20261105T214424Z-042c0809`; its complete rows
remain local. [Frozen diagnostic JSON](frozen-diagnostic.json) retains aggregate
metrics and source/config/trace hashes. The config hash is the raw file SHA-256,
not the runner's canonical-JSON checksum. Source commit identifies the pre-edit
parent; individual source hashes identify the working code tested.

Earlier local diagnostic folders are retained. One intermediate diagnostic used
an algebraically equivalent but differently rounded float32 band comparison;
that produced a false outside-band-derivative flag. The diagnostic predicate was
corrected to match the trainer exactly, and both the audit and tests were rerun.
No model or optimizer update occurred during that correction.
