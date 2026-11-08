# Stage 1 structured highway training v02

November 3, 2026, America/Los_Angeles. Preparation: [v02](highway-stage1-preparation-v02.md).

The authorized run completed its transition budget without a runtime error.
**Driving quality is not accepted:** eight of twelve validation cases exceed
the unchanged 5% intervention limit. No second attempt or held-out evaluation
was launched. This remains exact-state SUMO driving, not camera/LiDAR or physical
vehicle control.

## Frozen run and budget

- Run: `outputs/highway-stage1/v02/train-20261103T190020Z-8469072d`.
- Source commit: `38c3b41b1dbc6a5a6d4417e8a5f60a958d193bd9`; actual source snapshots and working-tree diff retained; no source drift.
- Authorized config canonical SHA-256: `1fa2582534c201cdf623a877a0ac03c70da554842e9cd998e066bb7116185d39`. Only the authorization flag changed from the reviewed disabled proposal.
- 499,992 active-agent transitions: 49,998 teacher demonstrations + 449,994 fresh PPO transitions. Four BC epochs, 392 BC optimizer steps; 162 PPO updates, 6,886 PPO optimizer steps.
- Runtime: 3,563.906 seconds (59m24s), below the two-hour cap; stop reason `transition_budget`. Small unusable remainders remain within the fixed phase allocations.
- Python 3.11.9, Torch 2.14.0+cpu, TraCI 1.27.1; SUMO 1.27.1. CPU/one worker. No dependencies or training sources changed.
- The chat waited for exit without inspecting training progress. The runner retained checkpoints, training/BC metrics, scheduled validation and complete journals automatically.

## Selected actor and validation

Selection points to `selected-00162-cfaa8b0a`, whose validation is the final
phase-3 evaluation. All checksums match the saved contract.

- Actor: `outputs/highway-stage1/v02/train-20261103T190020Z-8469072d/selected-00162-cfaa8b0a/policy.pt`.
- SHA-256: `07ef91a2e7591becd235862c9b2ebb0dd5707ef364546ec251f8bf8964e1a6b3`.
- Resume state: `outputs/highway-stage1/v02/train-20261103T190020Z-8469072d/last.pt`.
- Contract: 93 actor inputs (45 local fields plus six eight-field message slots),
  acceleration in −4.5 to +2 m/s² and masked lane intent; 60 mph ceiling and the
  existing 1.5-second following/stopping guards. Centralized critic stays out of
  exported inference. The complete ordered fields are in the
  [saved contract](evidence/highway-stage1-training-v02/policy-contract.json).

Across the fixed twelve validation cases: zero collisions, teleports or overspeed;
6/6 ramp merges; four passes in each passing case (12 total); cruise speed
96.59–97.42% of the limit. Overrides total **6,965/38,380 = 18.15%**, with a
53.56% worst case. All following/passing cases and two merge cases fail the
per-case override gate. The four cases meeting the override limit are the three cruise cases and the
two-lane merge. Zero collisions under protections do not establish independent
safe driving. All primary accounting horizons are complete.

| Case | Seed | Speed / limit | Passes | Merges | Overrides |
| --- | ---: | ---: | ---: | ---: | ---: |
| 5-lane-cruise | 5210000 | 96.59% | 0 | — | 0/3200 (0.00%) |
| 5-lane-following | 5210001 | 92.17% | 0 | — | 511/3200 (15.97%) |
| 5-lane-passing | 5210002 | 93.32% | 4 | — | 1084/3200 (33.88%) |
| 5-lane-merge | 5210003 | 93.84% | 0 | 2/2 | 194/3200 (6.06%) |
| 2-lane-cruise | 5210004 | 97.05% | 0 | — | 42/3200 (1.31%) |
| 2-lane-following | 5210005 | 81.02% | 0 | — | 1714/3200 (53.56%) |
| 2-lane-passing | 5210006 | 94.95% | 4 | — | 314/3200 (9.81%) |
| 2-lane-merge | 5210007 | 93.42% | 0 | 2/2 | 140/3180 (4.40%) |
| 4-lane-cruise | 5210008 | 97.42% | 0 | — | 1/3200 (0.03%) |
| 4-lane-following | 5210009 | 93.90% | 0 | — | 1656/3200 (51.75%) |
| 4-lane-passing | 5210010 | 92.62% | 4 | — | 1138/3200 (35.56%) |
| 4-lane-merge | 5210011 | 93.89% | 0 | 2/2 | 171/3200 (5.34%) |

## Within-run progression

| Validation point | Merges | Passes | Pooled overrides | Worst case |
| --- | ---: | ---: | ---: | ---: |
| after-bc | 6/6 | 11 | 18.01% | 55.19% |
| phase-0 | 6/6 | 12 | 18.07% | 51.25% |
| phase-1 | 6/6 | 12 | 15.06% | 50.19% |
| phase-2 | 6/6 | 12 | 12.47% | 46.63% |
| phase-3 | 6/6 | 12 | 18.15% | 53.56% |

The selected final actor follows the frozen competence-first ranking. It is not
the lowest-intervention candidate: phase 2 reached 12.47% pooled overrides, still
above the gate. Neither candidate is quality-accepted. Historical v1 used
different generated ramp geometry, so its 96.44% overrides and 0/6 merges are
context, not a controlled estimate of this revision's causal improvement.

The journal attributes 6,962 final overrides to measured native realization
differences and three to the analytical speed ceiling. It does not identify the
underlying native causes. The next scoped step is to compare requested and realized
acceleration in the failing following/passing cases, check feasibility learning
and validation ranking, and present any revised training setup before another
launch. Guards and acceptance thresholds remain unchanged.

## Retained evidence

[Per-seed metrics and phase progression](evidence/highway-stage1-training-v02/validation.json),
[summary](evidence/highway-stage1-training-v02/summary.json),
[resolved authorized config](evidence/highway-stage1-training-v02/config.json) and
[provenance/checksums](evidence/highway-stage1-training-v02/provenance.json) are
committed. Full local run files include immutable selected generations, last.pt,
BC metrics, demonstration versions, episode reports, journals and source snapshots.
Console log: `logs/highway-stage1-training-20261103T190009Z.log`.
The frozen actor, contract, config and compact results are intentionally published
in [the versioned policy catalog](../policies/versions/highway-stage1-v02/README.md).
Resume checkpoints and bulk outputs remain local; no release asset is claimed.
