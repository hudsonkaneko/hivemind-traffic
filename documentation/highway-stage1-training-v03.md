# Stage 1 highway training v03

November 3, 2026, America/Los_Angeles. [Preparation and reviewed configuration](highway-stage1-v03-preparation.md).

The authorized attempt completed its transition allocation after one checkpoint-file access failure and a successful resume. **The actor remains experimental: five of twelve validation cases exceed the unchanged 5% override gate.** No held-out traffic evaluation or additional learning attempt ran.

## What ran

- Source commit: `cf6ae5096ebac9f165e9811ed3916195227f3146`. Runtime sources remained unchanged across both processes; exact snapshots and the unrelated dirty-worktree diff are retained.
- Reviewed configuration was copied to `outputs/highway-stage1/v03/launch-20261103T230059Z/authorized-config.json`; only `training_authorized` changed to true. Authorized canonical SHA-256: `4df2efa0955d84de1bca101b1dd45d5e72e4a8709a4596165f0f36e6281ea5a8`.
- 12 BC epochs with 4× acceleration-loss weighting; teacher assistance tapers 0.05→0.01 over 400k PPO decisions. Observations, rewards, scenario seeds and protections match the reviewed proposal.
- Aggregate 499,990 active-agent transitions: 49,998 teacher demonstrations and 449,992 PPO transitions; 162 PPO updates. Small unusable phase remainders stay within the 500k cap.
- Aggregate charged runtime 3,544.125 seconds (59m04s), including the failed process; final stop reason `transition_budget`. The two-hour allowance was not reset on recovery.
- CPU, one worker and one Torch thread; Python 3.11.9, Torch 2.14.0+cpu, TraCI 1.27.1. No runtime or dependency changes were made.
- The chat waited for process exit without reading progress. The existing runner retained BC/PPO metrics, fixed scheduled validation, checkpoint generations and complete journals.

## Failure and recovery

Initial run `train-20261103T230113Z-dbea1564` stopped after 199.781 seconds when Windows denied replacing `last.tmp` with `last.pt` (WinError 5). The specific external lock owner is unknown. Its exception path restored the last atomic state and saved a readable, closed recovery checkpoint at BC epoch 2/cursor 8192, after 212 committed BC optimizer steps and no PPO updates. One later BC update was discarded during recovery; it is not represented as retained model progress.

Resume run `train-20261103T230506Z-04ba1761` used that exact checkpoint and identical configuration/source hashes. It restored demonstration data, optimizer/RNG state and consumed budget, then completed without another error in 3,344.375 process seconds. No checkpoint was patched and no checkpoint-writing code changed. The transient failure is preserved; this recovery does not establish a permanent filesystem fix.

Logs: `logs/highway-stage1-v03-launch-20261103T230059Z.log` and `logs/highway-stage1-v03-launch-20261103T230059Z-resume1.log`. Authorization and completion manifests remain in the launch directory. Both complete run folders remain under `outputs/highway-stage1/v03/`.

## Selected validation result

Ranking v5 selected final generation `selected-00162-80ce2ebf`. Selected artifacts match their recorded checksums; no held-out test was used for selection.

| Metric | Selected v02 | Selected v03 |
| --- | ---: | ---: |
| Overrides / decisions | 6,965 / 38,380 | 5,745 / 38,380 |
| Pooled override rate | 18.15% | 14.97% |
| Worst-case override rate | 53.56% | 51.47% |
| Cases failing override gate | 8 / 12 | 5 / 12 |
| Ramp merges | 6 / 6 | 6 / 6 |
| Passes | 12 | 12 |
| Cruise speed / limit | 96.59–97.42% | 96.44–96.97% |
| Collisions / teleports / overspeed | 0 / 0 / 0 | 0 / 0 / 0 |

Pooled interventions improve by 3.18 percentage points versus the selected v02 actor, but this is one paired-seed combined revision, not a multi-seed causal ablation. The retained v02 phase-2 candidate had lower pooled overrides (12.47%, eight failing cases); v03 is not uniformly better than every prior candidate.

Failures remain in 5-lane following (38.94%) and passing (22.06%), 2-lane following (17.91%), and 4-lane following (34.28%) and passing (51.47%). In particular, 4-lane passing regressed from 35.56% to 51.47%. Lower aggregate overrides do not mean every task improved. All cruise and merge cases, and 2-lane passing, meet the override limit.

The actor still consumes exact SUMO/map state and delivered V2V messages: 93 actor fields, 376 critic fields used only during training, normalized acceleration and masked lane intent. SUMO owns motion and the existing 60 mph ceiling, 1.5 s following and stopping protections remain active. This is not camera/LiDAR control or validated physical driving.

## Retained learning progression

| Point | Override-failing cases | Pooled overrides | Worst case |
| --- | ---: | ---: | ---: |
| after-bc | 12 | 29.02% | 54.53% |
| phase-0 | 6 | 11.34% | 32.63% |
| phase-1 | 6 | 17.14% | 46.19% |
| phase-2 | 6 | 15.02% | 51.06% |
| phase-3 | 5 | 14.97% | 51.47% |

Phase 0 has lower pooled and worst-case overrides than the selected final actor, but six failing cases versus five. Ranking v5 prioritizes failing-case count; it behaved as configured. All retained candidates remain unaccepted, and no original selection was rewritten.

Frozen after-BC training-label acceleration RMSE improved from 0.20134 to 0.14399 m/s² on the same 49,998 demonstration examples. However, after-BC closed-loop overrides worsened to 29.02%, with twelve override failures and two cruise task failures. Improved supervised fit alone did not improve driving. The final PPO actor restored the task gates. See [fit](evidence/highway-stage1-training-v03/bc-fit.json) and [phase/paired comparison](evidence/highway-stage1-training-v03/comparison.json).

## Retained policy and next action

Actor SHA-256: `a6d59d3acf58fcca9d7fc33406cb61ca3863d1a3dffad6dd316b195a66c7166f`.

[Versioned actor and exact contract/configuration](../policies/versions/highway-stage1-v03/README.md). Original accepted v04r2 and experimental v02 packages remain intact; publishing v03 is not an acceptance promotion. Full resume state is `outputs/highway-stage1/v03/train-20261103T230506Z-04ba1761/last.pt`; the selected folder contains `policy.pt`, `best.pt`, `validation.json` and `policy-contract.json`.

[Compact result evidence](evidence/highway-stage1-training-v03/) retains per-case and phase metrics, checksums, provenance and the failed/resumed attempt. Bulk journals and resume checkpoints remain local. Catalog loading checks are inference smoke tests, not new driving evaluations.

Next: diagnose acceleration errors on the actor's visited following/passing states and the 4-lane passing regression, compare the retained intermediate candidates, then present a scoped configuration before another attempt. Do not advance to sensor uncertainty or weaken the override criterion on the basis of this run.
