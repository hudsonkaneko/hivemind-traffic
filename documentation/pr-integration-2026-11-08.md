# Ordered PR integration — November 8, 2026

User requested merging the passing PRs into main and resolving conflicts.
All 48 repository-owned PR heads had successful Clean clone checks before
integration. Teammate fork PR #16 has no passing checks and remains outside
this merge operation. Branches and historical PR discussions are retained.

## Order and conflict resolution

Merge commits preserve original ancestry. Each PR is retargeted to main after
its prerequisites are integrated. The main chain is #61–70, #72, #76–108;
it merged without conflicts. #64 also includes the existing console-training
prerequisite commits from lane-training-console. The sensor chain follows in
order #71, #73, #74, #75, with main integrated into its source branches first.

The sensor overlaps affect CHANGELOG, hybrid-roadmap and the generated Google
guide/manifest pair. Changelog and roadmap entries from both histories are
retained. The newer main guide already contains all sensor-guide content;
only the obsolete start-tab branch link differs. Preserve that newer guide
and its manifest byte for byte, with a verified SHA-256 match. No generated
mirror is hand-edited and no fresh Google synchronization is claimed.

All 48 original heads are ancestors of the combined validation tree. No
runtime code conflicts required manual resolution. Existing policy acceptance
criteria and documented experiment limitations still apply after integration.
The unrelated dirty files in the canonical working checkout remain untouched.

## Validation and retained evidence

A clean validation checkout under outputs/pr-merge-2026-11-08/validation was
used so local uncommitted files cannot influence the result. Initial full CPU
run: 4,115 passed, 248 skipped, 165 subtests passed, six failures and 12 setup
errors (240.86 s). Every failure/error was caused by a missing outputs parent
in this local harness; CI creates outputs/ci before its full suite. After
creating that directory, both affected test modules pass: 17 tests and two
subtests (1.51 s). No assertions or production behavior were changed to obtain
this result. The portable SUMO setup check also passed; it starts no simulation.

The original failed log and the corrected run are retained separately:
logs/pr-merge-combined-tests.log and logs/pr-merge-setup-corrected-tests.log.
JUnit, before/after PR snapshots and the merge journal are under
outputs/pr-merge-2026-11-08/. Final GitHub checks on the revised sensor heads
and merged main are the full clean-clone acceptance evidence; do not infer a
remote pass from this local report. No new training or simulator run was made.

[Exact original heads, merge order and conflict resolution evidence](evidence/pr-integration-2026-11-08/integration-evidence.json).
