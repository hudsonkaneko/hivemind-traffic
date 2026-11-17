# Shared Claude / Codex training plan

Updated November 17, 2026. User requested coordination, comparison and low usage;
do not interrupt active training. This plan does not authorize a new run.

## One shared plan, explicit ownership

This tracked file is the coordination source. The canonical checkout's
`TRAINING_COORDINATION.md` points here while the documentation branch is pending
review. Keep one source rather than independently editing duplicate plans.
Read this file and the linked evidence before related work. Re-read it before
editing so another agent's changes are preserved. No background watcher or
automatic cross-app messaging is installed. Claude acknowledgement is pending;
an already running chat may need to read the plan explicitly.

| Work | Proposed owner | State / next handoff |
| --- | --- | --- |
| Existing training job | Its original launcher | Leave process, config, sources, checkpoints and ledger untouched until completion/failure |
| Next policy candidate and optimizer experiment | Claude | Confirm actual branch/run; no new launch authorized by this plan |
| Scenario validity, cut-in proof, evaluation comparison | Codex | Saved comparison complete; no simulator work launched |
| Shared plan and published result table | Codex initially | Claude records acknowledgement and its exact run/branch here before adopting a task |
| Common evaluation configuration and promotion decision | Both review | Freeze one evaluation contract first; do not automatically promote either candidate |

## Located work and attribution

- Claude's earlier work is positively identified by coauthor trailers on
  `e98cc7e` (jerk diagnosis), `a21038e` (corrected ramp networks), and `daa2440`
  (startup handoff). It is contained in branch `highway-jerk-diagnosis-v05` and
  inherited by the newer branches; it is not a separate active worktree now.
- The latest separate saved v06 run is canonical-root
  `outputs/highway-flow-rl/v06/round-07`. Its manifest records execution from
  the dirty `mappo-flow-rl-v02` checkout at `7367b80`. The config's publication
  branch is `highway-jerk-diagnosis-v05`; subsequent results were committed in
  `5dcf619` on the lineage now retained by `2026-11-15-codex-merge-cutin`.
  The manifest does not identify the assistant that launched it. Do not label
  its launcher as verified Claude attribution.
- The v06 round-8 proposal, commit `183479c`, is already in our current branch.
  `configs/training/highway-flow-rl-v06-round8.json` says
  `training_authorized=false`. No corresponding local run was found.
- Codex v08: branch `2026-11-17-codex-merge-recovery`, training source
  `0efc28a`, results commit `148818d`, [PR 112](https://github.com/hudsonkaneko/hivemind-traffic/pull/112).
  Run: `outputs/highway-flow-rl/v08/train-20261117-01/train` inside the active
  worktree `outputs/worktrees/highway-jerk-diagnosis-v05`.
- At the inspection recorded in [comparison.json](evidence/training-coordination-2026-11-17/comparison.json),
  no local Python/SUMO training process or active shared lock was present.
  Claude app processes were present, which does not prove a running training
  job. An external folder/machine or newer job remains unverified. No process
  was stopped, restarted, attached to, or reprioritized.

## Saved results: descriptive, not a controlled policy ranking

| Latest saved nominal result | v06 round 7 | Codex v08 |
| --- | ---: | ---: |
| Training decisions | 100,250 | 298,361 |
| Initial -> latest merge-speed match | 43.67% -> 34.34% | 43.52% -> 39.36% |
| Stable merges | 3/4 | 4/4 |
| Pooled safety overrides | 0.457% | 0.448% |
| Peak realized jerk | 4.00 m/s3 | 28.38 m/s3 |
| Collisions / teleports | 0 / 0 | 0 / 0 |
| Strict cases passed under each run's scoring | 2/8 | 0/8 |
| Actual cut-in exposure qualification | No dedicated cut-in set | 34/34 training; 8/8 validation exposures |
| Outcome | Stopped at merge regression gate | Finished, failed acceptance; not promoted |

Both share the same frozen v04 warm-start hash, 117 actor inputs, 2,832 critic
inputs, normalized acceleration plus lane-intent actions, Krauss vehicles,
full-episode conditioning and four-episode PPO batches. Their eight nominal
validation network hashes, seeds, horizons, counts, limits and communication
settings match. However, v08 adds lane-change admission checks and verified
cut-in scenes, and raw-request scoring differs (v06 excludes the first decision;
v08 includes every decision). Thus these numbers do not prove which policy is
better under one common environment. Neither is an accepted replacement.

V06's completed round 7 used actor LR 1e-4, 10 epochs, KL 0.03 and inherited
action spread about 0.04. Its unlaunched round-8 proposal instead uses KL 0.1,
five-decision insertion-lag scoring and a report-only merge floor. Codex v08
used LR 3e-5, 20 epochs, KL 0.02, initial spread 0.12, true command-slew rewards,
merge acceleration alignment and unsaturated merge-speed tracking. These are
alternative experiments, not settings to combine in a live run.

## Coordination rules for the next work

1. **A live run owns its inputs.** No changes to its source files, resolved
   config, checkpoints, environment, ports or working branch. Do not switch the
   shared worktree's branch while another agent uses it. Preserve the canonical
   checkout's unrelated dirty files. Use an isolated branch/worktree under the
   project root for future concurrent code edits; do not create one for an
   already active job or move that job.
2. **One launcher owns the compute reservation.** Before any future authorized
   launch, resolve the shared ledger/lock to the canonical Git common directory:
   `outputs/highway-flow-rl/v02/cost-ledger.json` and `active.lock`. Use the
   existing atomic lock; do not delete it because a PID looks stale without
   reconciling its owner. Do not add a second competing lock. Check Isaac/GPU
   ownership too when relevant. Parallel code review is fine; this workstation's
   measured training/evaluation jobs run serially by default.
3. **Allocate fresh run IDs and reconcile budgets.** Both old proposals refer
   to round 8; Codex v08 has now used it. Old remaining-budget notes and the v06
   round-8 authorization flag are not permission to launch. Recompute completed
   ledger charges, select the next unused round and present a resolved budget
   before a new run. Never reset prior charges or overwrite an output folder.
4. **Merge code only after the active job ends.** Candidate reusable pieces:
   Claude's corrected roads, startup-lag diagnosis and complete-episode batching;
   Codex's actual cut-in onset/completion proof and separate command-slew audit.
   Some are already shared ancestors. Review a scoped diff before cherry-picking
   to avoid duplicating them. Do not average model weights or combine PPO
   optimizer states/rollouts collected under different policies.
5. **Freeze a common development evaluation.** Use the existing eight matched
   nominal rows and report the two verified cut-in rows separately. Freeze one
   source/config hash, native safety modes, lane admission, observation/action
   semantics and scoring for all candidates. Report full-route realized jerk,
   raw request-versus-actuator difference and consecutive-command slew separately;
   report startup-window diagnostics without hiding the full-route metrics.
   Validate actor contracts before scheduling serial checkpoint evaluation.
6. **Preserve checkpoints at every evaluation.** Keep each evaluated actor and
   its source/config/normalization hashes. Check per-scenario and aggregate merge
   regressions at periodic AND final evaluation; record all failures. Keep the
   5% override and existing safety/comfort criteria. Do not choose a checkpoint
   from reward alone or tune against held-out test scenarios.
7. **Wait quietly while training.** The launching agent blocks for completion
   or failure, with no progress polling, new agents or repeated analysis. After
   termination, publish one compact handoff: run path, branch/commit and dirty
   snapshot, config/checkpoint hashes, exit status, elapsed cost, actual scenario
   coverage, per-case metrics, remaining budget and next decision.

## Handoff queue

- [x] Codex: locate local branch lineage, terminal runs and compare saved metrics.
- [x] Codex: publish shared plan and discovery pointers for both assistants.
- [ ] Claude: acknowledge this plan; supply the actual current branch, worktree,
  run path, process/remote-job identifier and status (or confirm preparing only).
- [ ] Both: reconcile the now-used round 8 and agree on isolated ownership for
  the next changes. No training, source merge or policy promotion started here.
- [ ] Codex: prepare one reviewed common evaluation spec after Claude's candidate
  is identified; run only within an agreed evaluation budget.
- [ ] Both: choose one next correction from the saved I-110 action traces and
  common results, then present its final configuration before training.

Progress history is append-only. Google engineering-guide synchronization is
pending; this plan and the saved comparison are published in GitHub.
