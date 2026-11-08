# Highway flow RL v01 — final round-two experimental actor

**Not accepted or promoted.** Trained November 8, 2026 in native SUMO. Shared
MAPPO with 117 actor inputs, six received eight-field message slots and a
training-only 2,832-input masked critic. Export accepts float32 `[batch,117]` and
returns `[batch,2]`: normalized acceleration and masked right/stay/left intent.
The matching environment applies −4.5…+2 m/s², 10 Hz, 60 mph/lower limits and native
safety. No steering, raw sensors or physical-car control.

[Download policy.pt](policy.pt?raw=true) · [Input/output contract](contract.json) ·
[Training config](training-config.json) · [Evaluation](evaluation.json) ·
[Hashes/provenance](manifest.json)

Changes: explicit 93→99→117 initializer migration; no teacher imitation,
near-limit auxiliary or teacher-alignment reward; actual I-110 merge exposure;
independent critic-warmup/PPO schedule. Two sequential rounds used 59,995 active
decisions total. After terminal review, round 2 raised the measured-jerk reward
coefficient .01→.03 with fresh optimizer and pinned round 1 model weights.

Four matched development cases: pooled interventions 24.94→16.44→7.61%; I-110
merge case 0.67% final and 1/1 stable target passed. Zero observed collisions,
teleports or overspeed. Three cases still exceed 5% interventions; all four
fail peak comfort. Not held-out or generalization evidence, and not a replacement
for the accepted original straight-road baseline. No live AV slider handoff.

SHA-256: `888a380b7c6741aea623e9b99ad155f7cb9a2ef8fdc2e590823b7e2629031819`.
CPU export/shape/finite-output parity passed; this loading check is distinct from
the driving evaluations. [Full changes and limitations](../../../documentation/highway-flow-rl-v01.md).
