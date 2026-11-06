# Highway environment v05 final pilot endpoint

**Not accepted for deployment.** This frozen diagnostic candidate completed the
bounded v05 MAPPO pilot, but matched validation regressed from **13.3749% to
19.8287% pooled interventions**. Stable merges improved from **2/9 to 8/9** and
passes from **55 to 59**; **19/24 final cases still fail acceptance**. No held-out
test, physical driving or camera/LiDAR policy validation was performed.

Artifacts: [frozen actor](policy.pt), [policy contract](policy-contract.json),
[resolved run configuration](training-config.json). The full checkpoint remains local at
`outputs/highway-environment/v05/train-20261106T024828Z-90b25880/last.pt`.
See [the complete results note](../../../documentation/highway-environment-v05-results.md)
for all 24 matched cases, failures, plots and recovery history.

## Contract

- Shared MAPPO actor with **99 inputs**: 45 structured local/map/merge/mask fields,
  six measured/committed motion fields, and six received eight-field message slots.
  Neighbor context includes measured occupancy across both lane corridors during
  native continuous motion. Observations are current SUMO/map state and actually
  delivered V2V, not learned perception.
- Deterministic exported outputs: normalized acceleration mapped to
  **[-4.5, 2] m/s²** and masked lane intent **-1/0/+1**. SUMO owns execution,
  including 3-second lane changes and the unchanged speed/following guards.
  This actor does not command steering or provide a physical trajectory.
- Training-only masked critic: **24 slots, 2,400 inputs**. It is absent from the
  actor export. Controlled cohort membership is fixed per episode; there is no
  live controller handoff. Following/passing retain a native leader, and 100% AV
  cases use initial-only traffic.
- Warm start: v03 phase-0 actor, with six zero-initialized motion input columns
  inserted and a fresh critic/optimizer. This is an explicit semantic migration;
  old 93-field observations are not accepted by the new actor.

## Measured scope

Training collected **249,999 active-agent decisions**: 4,096 critic warmup and
245,903 PPO. There were 50 rollout updates, 2,025 joint PPO optimizer steps and
68 critic-only steps. All collected scenarios remained at curriculum difficulty
0. There was no I-110 merge/passing training episode, and the final I-110 merge
target remained pending after stalling before entry.

Initial/final deterministic evaluation retained all 24 predeclared rows. The
final endpoint has **38,852/195,938 intervention decisions**, worst-case
**69.3339%**, eight completed/stable merges out of nine targets, and 59 observed
passes. All reports have complete primary-horizon accounting and zero observed
collisions, teleports, overspeed samples, measured emergency samples and native
emergency events. These sample results do not qualify the controller as safe.

## Identity and preservation

| Item | Value |
| --- | --- |
| Frozen actor SHA-256 | `fd639c39f0c514256a458b2b85d35baba242125f44dcc3d9542d767eb2dd46ec` |
| Full checkpoint SHA-256 | `9279b6103d13d2be3fd18fe05e9bae72462e581a16d881fc9e4ff87b0b2ac02d` |
| Semantic configuration SHA-256 | `a9db9895d18a9b3760977603a152cd5d4c52ad82d7e3a0dd8e4d9b9628855f78` |
| Source Git commit | `02948d76f16f4d2c56d839ad72c1266777b373c6` |
| Runtime | Python 3.11.9; PyTorch 2.14.0+cpu; SUMO/TraCI 1.27.1 |
| Training seed | 9507 |
| Validation seeds | 6900000–6900023 |

The run retained source hashes, scoped snapshots, complete logs and a dirty-tree
record. Both pre-learning failures remain in the charged budget and recovery
history. Previous versioned weights remain unchanged, including the separate
accepted MAPPO v04r2 actor. Retain v05 as a reproducible research endpoint, not
as a replacement for an accepted controller.
