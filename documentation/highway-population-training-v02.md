# Population v02 acceleration supervision

November 4, 2026. User authorized correction and retraining.

Diagnosis: v01 final had 18,266 combined interventions, 16,697 native-realization differences and 1,710 policy-filter changes (overlapping counts). The matched sixteen-case scripted baseline achieved 1.5105% pooled / 8.25% worst overrides; one cruise-speed and one override gate still failed. Validation is diagnosis only, never learning data. Raw baseline: outputs/highway-population/v02/diagnosis/teacher-validation.json.

Targeted change: optional acceleration imitation weight, default 1 preserves existing behavior; v02 sets 50. Current-rollout auxiliary objective becomes 0.04*(50*acceleration MSE + existing lane loss). No historical PPO replay, reward changes, threshold relaxation or controller substitution. All other v01 settings, seeds, starting actor and fresh critic match. This is an experimental hypothesis, not a verified fix.

Budget: 100,000 active-agent decisions including 4,096 critic warmup; CPU one thread; 1,800 s including initial/final sixteen-case validation with 600 s reserved for final. Tracked config experiments/configs/highway_population_v2.json remains disabled; authorized launch outputs/highway-population/v02/launch/authorized-config.json SHA256 024ed58ecdb2d1e094c95fde4d96a5256c658594b7f68120cfa703e3fe9f5c88. Existing runner stores runs under outputs/highway-population/v01; configuration revision distinguishes this experiment.

57 targeted tests passed. No-learning preflight preflight-20261105T044931Z-106cb333 passed in 29.625 seconds. Final endpoint compared with same frozen parent and v01 final; preserve speed/pass/merge/cohort results and original 5% per-case override gate. No held-out evaluation or live slider handoff. Native emergency warnings must be inspected separately from sampled counters.

Training results pending. Google guide synchronization pending.


## Completed result

Run `train-20261105T045039Z-a6db8f1b`: 99,998 decisions, 514.516 seconds, no runtime error/source drift. Final pooled overrides 29.9716% versus previous v01 31.7013% and matched parent 20.6393%. Worst 86.75% versus v01 75.125%; twelve of sixteen cases fail versus fifteen previously. All four controlled merges and thirteen passes, zero final validation collisions/teleports/overspeed under protections. Twelve native emergency-warning lines across run logs are retained individually; no emergency-free claim. Stronger acceleration supervision is insufficient and this actor is rejected for promotion. No held-out test or automatic extra attempt.

[Metrics](evidence/highway-population-v02/results.json), [comparison](evidence/highway-population-v02/comparison.json), [experimental actor](../policies/versions/highway-population-v02/README.md). Raw journals/checkpoints remain local in `outputs/highway-population/v01/train-20261105T045039Z-a6db8f1b`. Google guide sync pending.
