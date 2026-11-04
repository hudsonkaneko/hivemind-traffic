# Highway population v01

**Experimental — not accepted.** Trained November 4, 2026 (America/Los_Angeles).
The final policy regressed on the matched validation cases and does not replace
its parent or the accepted straight-road v04r2 policy.

[Download policy.pt](policy.pt?raw=true) · [Runtime contract](contract.json) ·
[Ordered inputs](observation-interface.json) · [Training parameters](training-config.json) ·
[Measured results](evaluation.json) · [Hashes and provenance](manifest.json)

| Detail | Value |
| --- | --- |
| Policy type | Shared hybrid MAPPO actor; deterministic TorchScript export |
| Inputs | 93 floats: 45 local/map fields + six delayed, presence-masked 8-field V2V messages |
| Outputs | Normalized acceleration in [−1,1] and lane intent −1/right, 0/stay, +1/left |
| Acceleration conversion | `-4.5 + (action[0] + 1) * 3.25` m/s², followed by environment guards |
| Architecture | Same 128/128 message actor; new masked population critic, used only during training |
| Population | Training: 1/2/4/6/8 controlled cars among 8/16/24 vehicles; validation: 1/3/5/7/8 controlled cars |
| Communication | None, local, or local with 30% packet loss; nearest six fresh received peers, 150 m range |
| Control and limits | 10 Hz; 60 mph ceiling; existing 1.5 s following/stopping guards |
| Training | 99,999 active-agent decisions: 4,096 frozen-actor critic warmup + 95,903 PPO; 474.328 seconds |
| Optimizer | Actor LR 0.0001, critic LR 0.0003; four epochs, batch 256, PPO clip 0.2; current-rollout imitation 0.04 |

**Changes:** warm-started the actor and exploration from Stage 1 v03's phase-0
checkpoint, initialized a new critic with eight padded slots (752 fields), and
varied the controlled population, total cohort and communication channel.
The actor receives local observations and delivered messages; centralized critic
inputs do not enter deployment. No correction-memory buffer or new BC was used.

**Intended behavior:** cruise, follow, pass slower cars and merge despite different
numbers of communicating vehicles. Controller assignment is fixed at reset.
This is a finite-cohort SUMO pilot; it does not implement a live slider handoff,
continuous traffic inflow, physical steering, or camera/LiDAR policy inputs.

**Measured, same 16 validation cases:** pooled overrides worsened from **20.64%
to 31.70%**, and worst-case overrides from **44.875% to 75.125%**. Both the parent
and final policy failed 15/16 cases. Thirteen cases had more overrides; three
improved. Final validation had 4/4 controlled merges, 14 passes (parent: 13), and
zero collisions, teleports or overspeed under protections. These results do not
establish population robustness; no held-out test or 3D transfer ran.

Native SUMO logs also recorded four controlled lane-end emergency stops during
training and one during initial validation; final validation recorded none.
The sampled emergency counter missed those events, so it must not be interpreted
as run-wide evidence of zero emergencies. See the
[independent log audit](../../../documentation/evidence/highway-population-v01/audit.json)
and [training note](../../../documentation/highway-population-training-v01.md).

The export accepts float32 `[batch,93]` and returns `[batch,2]`; CPU loading and
finite bounded outputs were checked for batches of 1, 3 and 8. This is a loading
check, not a driving evaluation. Use the exact ordered interface and matching
population environment; unchanged tensor width alone does not establish semantic
compatibility. Full resume weights and journals remain in local `outputs/`.
The historical authorized configuration does not authorize another training run.

Run: `train-20261104T225405Z-302c4943`.
Source: `07482466f4d85805d931e4f9c639acb9600aafa0`.
Policy SHA-256: `8d2e8793072e8a575acba03391da6783705b87521e0848ba14401345d4616e59`.
