# Common frozen-actor evaluation: November 17, 2026

Completed 30/30 serial episodes in 293.91 charged seconds (294.19 seconds from
parent launch to exit), below the 900-second cap. No optimizer or critic was
constructed, no training decisions were collected, and no policy was promoted.
Source commit: `55af9f6`; clean tree at launch. Run:
`outputs/highway-shared-evaluation/compare-20261117-01` in the Codex worktree.

All three frozen actors used the same v08 environment, lane admission,
117-field local observation contract, safety modes 31/512, rate conditioning,
network hashes, seeds and populations. Eight nominal cases and two cut-in cases
per actor were evaluated. Primary scores include every decision; first-1 and
first-5 exclusions are retained separately as diagnostics.

| Candidate | Merge-speed match | Overrides | Peak realized jerk | Strict nominal cases |
| --- | ---: | ---: | ---: | ---: |
| Frozen v04 baseline | 43.52% | 0.265% | 4.76 m/s3 | 1/8 |
| Claude v06 round 9 | 48.84% | 0.255% | 4.00 m/s3 | 1/8 |
| Codex v08 | 39.36% | 0.448% | 28.38 m/s3 | 0/8 |

All candidates completed 4/4 stable nominal merges, with zero collisions and
teleports across all 30 episodes. Actual cut-in onset and completion were
verified in all 6/6 stress episodes. Cut-in peak realized jerk was 45.0 m/s3 for
every candidate, so none passes the stress comfort criteria. Merge quality also
remains failed. Claude round 9 is the strongest candidate on this development
suite: +5.32 percentage points in merge-speed matching versus the frozen baseline,
with lower nominal peak jerk. This does not establish held-out or 3D capability.

## Reporting correction and retained evidence

The initial scorer looked for SUMO's warning log in the case parent instead of
its unique `episode-*` child and therefore marked native evidence missing.
After completion, native log paths were corrected and the 30 saved reports were
re-scored offline. No episode was rerun, no missing log was assumed empty, and no
original report or summary was overwritten. Only native-warning evidence and
dependent acceptance gates changed. The original run source is preserved;
future evaluator code now reads the environment's actual runtime directory.
The path regression test passes with nine other focused tests (10 total).

- [Corrected compact result, provenance and hashes](evidence/shared-highway-evaluation-2026-11-17/result.json).
- [Per-case comparison and action/jerk diagnostics](evidence/shared-highway-evaluation-2026-11-17/cases.csv).
- Raw per-car observations, requested actions, applied/realized acceleration,
  override reasons and startup diagnostics: each run case's `actor-actions.jsonl.gz`
  and `report.json`; native logs remain in each unique episode child.
- Corrected full reports: run subdirectory `native-log-rescore-v01`; raw originals
  and source snapshots remain unchanged. Bulk artifacts are local, not on GitHub.

One ledger preparation segment records 30 attempted/completed episodes and zero
training decisions; the existing lock was released. The single evaluation
authorization is consumed. Current config is disabled to prevent accidental
relaunch; its exact authorized version remains in the run's resolved spec.
No further simulation or training was started. Google guide sync is pending.
