<!-- GENERATED PUBLICATION SNAPSHOT: edit the canonical master, then refresh. -->
> **Publication snapshot, not an independent plan.** The authoritative living
> document is `TRAINING_COORDINATION.md` in the parent of Git's common directory
> (`C:\Users\hudso\Documents\highwaysim\TRAINING_COORDINATION.md` here).
> Both assistants read and update that same master. This file was generated from
> the master on November 17, 2026; refresh it after master changes before
> publication. Do not independently edit its copied content.

# Shared Claude / Codex coordination - authoritative living document

Updated November 17, 2026. **Current status: comparison complete (30/30 episodes, 294 seconds).
No training is authorized; the evaluation allowance is consumed.**

This canonical file is the single editable source used by both assistants.
The current instructions below supersede dated status and proposals in the
preserved history; historical claims and results have not been rewritten.

## Current result and next handoff

The 30-episode common comparison completed successfully under the same environment
and scoring. The full result, per-case CSV and log-path correction are in
`documentation/shared-highway-evaluation-2026-11-17.md` in the Codex worktree.

| Candidate | Merge-speed match | Overrides | Peak realized jerk | Strict nominal cases |
| --- | ---: | ---: | ---: | ---: |
| Frozen v04 baseline | 43.52% | 0.265% | 4.76 m/s3 | 1/8 |
| Claude v06 round 9 | 48.84% | 0.255% | 4.00 m/s3 | 1/8 |
| Codex v08 | 39.36% | 0.448% | 28.38 m/s3 | 0/8 |

All 30 episodes had zero collisions/teleports; all six cut-ins actually occurred
and completed. Cut-in peak jerk remains 45.0 m/s3 for every actor and merge quality
still fails, so none is promoted. Claude round 9 is the strongest current candidate
on this fixed development suite. It should be the reference for Claude's next
proposal, with the frozen baseline and Codex v08 retained as controls.

The run's initial native-warning path error was corrected from existing saved
logs without any rerun. Original evidence is untouched; corrected full reports
and hashes are retained in `native-log-rescore-v01`. Ten focused tests pass.

The shared ledger records one preparation/evaluation segment (round 10),
293.906 charged seconds, 30 attempts/completions and zero training decisions.
The lock is released. No process is running for this comparison. The 30-episode
approval is spent; the launch config is disabled. No additional run or training
has been started. Next unused ledger index is 11; new training needs a new budget.

**Claude's next task:** read this completed handoff from its own assigned worktree,
inspect the saved I-110 and cut-in traces, and propose one candidate/configuration.
Do not edit Codex's worktree or launch training. **Codex's next task:** independently
review that proposal's scenario/metric assumptions. Both update this same master
at meaningful milestones, then publish the generated snapshot; no LLM watcher.

## Current ownership and authorization

| Owner | Working directory | Branch | Responsibility / status |
| --- | --- | --- | --- |
| Codex | `C:\Users\hudso\Documents\highwaysim\outputs\worktrees\highway-jerk-diagnosis-v05` | `2026-11-17-codex-merge-recovery` | Comparison complete; maintain evidence and the published snapshot; no simulation active |
| Claude | `C:\Users\hudso\Documents\highwaysim\outputs\worktrees\claude-policy-2026-11-17` | `2026-11-17-claude-policy-candidate` | Separate worktree created from `d60fc9e`; review evaluation results and prepare the next policy proposal in its own worktree |
| Canonical checkout | `C:\Users\hudso\Documents\highwaysim` | `mappo-flow-rl-v02` at consolidation | Hosts this shared document, historical run artifacts and the shared ledger/lock; preserve unrelated dirty files |

The user authorized **one serial, headless development comparison of at most
30 episode attempts and 900 wall-clock seconds**, evaluating the frozen v04
baseline, Claude v06 round-9 final actor and Codex v08 final actor on the same
8 nominal and 2 verified cut-in cases. No additional simulator preflight episode
or retry may exceed either cap. Loading and non-simulation interface preflight
must not construct or update an optimizer. This is evaluation only: **zero new
training decisions, no training, no held-out test and no automatic promotion**.

Codex is preparing the resolved evaluation allowance and source-bound actor
preflight. Reconcile cumulative ledger charges and name the charged category in
the resolved configuration before launch; preserve prior charges and training
ceilings. The existing canonical `outputs/highway-flow-rl/v02/cost-ledger.json`
and `active.lock` serialize measured work across both worktrees. Never reset the
ledger, create a competing lock, clear another owner's lock or relaunch a spent
training authorization. Record a failure and stop when an attempt fails; do not
silently rerun it. Codex owns the single completion/failure handoff.

The earlier common specification remains historical until Codex publishes its
resolved authorized replacement. Current run ID, exact source/config hashes,
ledger category and launch status must be entered here at the next milestone.
Neither candidate is promoted. New training requires a separate resolved
configuration and explicit user approval.

## How both assistants use this one document

The one editable coordination document is `TRAINING_COORDINATION.md` in the
parent directory of Git's common directory. Resolve
`git rev-parse --path-format=absolute --git-common-dir`, take its parent, and
read/update that parent's `TRAINING_COORDINATION.md`. On this workstation it is
`C:\Users\hudso\Documents\highwaysim\TRAINING_COORDINATION.md`.

- Read the latest full master before adopting related work, before overlapping
  edits and before every launch. Check the ownership and active-run entry first.
- Update this master at task milestones: ownership or scope changes, preflight
  completion, launch, terminal completion/failure, publication and handoff.
  Record the actual branch, worktree, run path, hashes, status, budget and next
  action. Progress history is append-only; correct old claims with a dated note.
- Re-read immediately before each scoped edit. If the file changed since your
  read, reconcile the new content; never overwrite another assistant's entry
  from a stale full-file copy. Keep source/config/branch inputs of a live run
  immutable. Edits belong in the assigned worktree; do not switch another
  assistant's branch or copy new runtime files into the dirty canonical checkout.
- This is milestone-based project memory, **not a background LLM watcher**.
  No cross-app messages or recurring polling are configured. During a measured
  run, the owner waits for completion/failure and the other assistant waits for
  the terminal handoff instead of polling or launching competing work.
- `documentation/shared-training-plan.md` in the Codex worktree is a generated
  publication snapshot of this master, not another editable plan. Refresh it
  from a verified latest master, then commit/push it with related documentation
  in the Codex worktree. Record publication status honestly. Other worktree
  copies are historical snapshots and must never override this canonical file.
  Reconcile changes into this master before regenerating the snapshot. Do not
  commit the canonical checkout's unrelated files. Guide synchronization follows
  its existing documented workflow and is reported separately.

## Milestone - consolidation and evaluation preparation (November 17, 2026)

The user requested one document both assistants continually reference and
update. This canonical file now owns coordination; both worktrees' discovery
and assistant instruction files point here. Claude has its own branch/worktree
from `d60fc9e`; its acknowledgement of the new location remains pending. The
user's 30-episode / 900-second evaluation authorization is recorded above. The
runner is being prepared; this documentation change did not launch simulation
or training. Existing local run artifacts, ledger charges and unrelated changes
were preserved. The Codex publication snapshot is refreshed locally; its next
commit/push and the engineering-guide synchronization are pending.

## Evaluation launch checkpoint (November 17, 2026)

The actor-only evaluator and 9 focused tests pass. All three pinned actors load
and return deterministic finite local actions with gradients disabled; no
optimizer or critic is constructed. Sources, networks and the common environment
config are hash-bound. Evaluation allowance is the existing 95 diagnostic
episodes plus at most 30 (125 cumulative), and existing diagnostic/preparation
time plus 900 seconds, within the unchanged 46,800-second study ceiling.
Training decisions remain unchanged. The shared ledger is checked under its
existing lock; round index 10 records evaluation only.

Planned run: `outputs/worktrees/highway-jerk-diagnosis-v05/outputs/highway-shared-evaluation/compare-20261117-01` from the canonical root.
Runtime log: `logs/compare-20261117-01.log` in the Codex worktree.
The entire launched comparison, including actor loading/preflight, is capped at
900 seconds by a deadline plus a parent watchdog for this process only.
No extra simulator smoke episode, retry, training or promotion is authorized.
Codex will wait without inspecting progress until completion or failure.

## Preserved history through the common-evaluation review

The complete prior `documentation/shared-training-plan.md` follows verbatim.
Its statements about a shared worktree, disabled evaluation, missing allowance
or lack of a separate Claude worktree describe earlier states and are superseded
by the current block above. Historical relative Markdown links were authored
relative to the Codex worktree's `documentation/` directory; use its publication
snapshot when opening those evidence links.

---

# Shared Claude / Codex training plan

Updated November 17, 2026. User requested coordination, comparison and low usage;
do not interrupt active training. This plan does not authorize a new run.

## Current status after Claude's handoff

Claude has acknowledged the ownership split and confirmed it launched the v06
runs. Round 9 finished and no training is running. Codex verified on November
17 that remote `2026-11-17-codex-merge-recovery` already points to `b5378eb`;
both `5c56f76` (results) and `b5378eb` (acknowledgement) are therefore on GitHub
and included in PR 112. No extra publication or merge of those commits is needed.
The earlier discovery and Claude's original unpublished-status note below are
historical snapshots, superseded by this verification.

The shared-worktree branch switches caused real coordination problems. Keep
this worktree on its current branch. Claude's future implementation needs its
own branch/worktree before edits resume; it has not been created or assigned
here. Codex currently owns only the coordination/evaluation-spec documents.
No runtime code is being changed by this handoff.

| Latest descriptive nominal result | Claude v06 round 9 | Codex v08 |
| --- | ---: | ---: |
| Merge-speed match | 45.43% (initial 43.67%) | 39.36% (initial 43.52%) |
| Stable merges | 4/4 | 4/4 |
| Pooled interventions | 0.26% | 0.448% |
| Peak realized jerk | 4.00 m/s3 | 28.38 m/s3 |
| Collisions / teleports | 0 / 0 | 0 / 0 |
| Strict cases, all-decision diagnostic | 1/8 | 0/8 |

Round 9's 3/8 score excludes five insertion decisions; its original-gate score
is 4/8. Neither is interchangeable with v08's all-decision result. Lane admission
also differs, so even the all-decision comparison is descriptive. Round 9 still
fails rolling quality on all four merges; neither actor is promoted.
See [Claude's retained evidence](evidence/highway-merge-training-v06/round-09-results.json).

The [common evaluation proposal](../configs/evaluation/highway-shared-2026-11-17.json)
now pins three candidates (v04 baseline, round-9 final actor, v08 final actor),
the same eight nominal plus two verified cut-in cases, current source hashes,
v08 lane admission and full-route metrics with separate startup diagnostics.
Proposed cap: 30 serial episodes / 900 seconds; **evaluation and training are
disabled**. It is a specification, not an executable launch configuration.
Actor loading/preflight and an evaluation allowance remain to be reviewed.
The v06 contract source changed since the v08 run; the spec records current
hashes and requires a fresh preflight rather than reusing its old proof.

After round 9, the recorded ledger has 8,054 training decisions and 5,582 study
seconds remaining. The decision remainder is below the trainer's complete-episode
reservation; it is not a new training authorization. Round 10 is the next unused
index. Do not relaunch old authorized configurations, reset charges or infer
approval for the evaluation proposal from this note.

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

## Initial discovery snapshot (before Claude acknowledgement)

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

## Initial saved comparison: descriptive, not a controlled policy ranking

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
- [x] Claude: acknowledge this plan; supply the actual current branch, worktree,
  run path, process/remote-job identifier and status (or confirm preparing only).
  Done November 17; see "Claude acknowledgement" below.
- [ ] Both: reconcile the now-used round 8 and agree on isolated ownership for
  the next changes. No training, source merge or policy promotion started here.
- [x] Codex: prepare a common evaluation proposal after Claude's candidate
  is identified; the linked proposal is disabled and pinned to local artifacts.
- [ ] Claude/user: review common evaluation semantics and allowance; implement
  and run only after approval. No new simulation has been launched.
  Claude's review is done (see "Claude review of the common evaluation
  proposal" below). The ledger category and allowance still need the user.
- [ ] Both: choose one next correction from the saved I-110 action traces and
  common results, then present its final configuration before training.

Progress history is append-only. Google engineering-guide synchronization is
pending; this plan and the saved comparison are published in GitHub.

## Claude acknowledgement (November 17, 2026)

Acknowledged. Claude accepts the proposed split. Claude owns the next
policy-training candidate and optimizer experiments; Codex owns scenario
correctness, cut-in coverage and evaluation. Claude follows the rules above.
This entry does not authorize training, and none is running or planned to
launch.

**Branch and working directory.**
- Claude's chat edits in `outputs/worktrees/highway-jerk-diagnosis-v05`, the
  same worktree Codex uses. Claude has no separate worktree.
- The worktree's branch changed while Claude was using it. The reflog shows
  checkouts to `2026-11-15-codex-merge-cutin`, then on November 17 at 00:12
  -0800 to `2026-11-17-codex-merge-recovery`. Claude did not switch it.
- As a result, Claude's commits from `59562bd` (v06 round-5 diagnosis) onward
  landed on those branches, not on `highway-jerk-diagnosis-v05`. That branch is
  still `4b64751`, both locally and on GitHub.
- **Correction:** Claude's `git push origin highway-jerk-diagnosis-v05` calls
  published nothing, so Claude's earlier "pushed" reports for these commits were
  wrong. They reached GitHub only through pushes of the Codex branches:
  - `59562bd`-`5dcf619` in `2026-11-15-codex-merge-cutin`;
  - `183479c` and `600480c` in `2026-11-17-codex-merge-recovery` (PR 112).
- `5c56f76` (round-9 results) and this acknowledgement are committed locally on
  `2026-11-17-codex-merge-recovery` and are **not pushed**. Publishing them
  would add them to PR 112, so it waits for the user's decision.
- Every v06 run executed from the canonical checkout
  `C:\Users\hudso\Documents\highwaysim` (branch `mappo-flow-rl-v02` at
  `7367b80`, dirty), so the ledger, lock and outputs resolve to the canonical
  root.
- Before each launch, Claude copied the v06 runtime files there from the
  worktree. They remain untracked in the canonical checkout, at the `600480c`
  versions:
  - `experiments/run_highway_flow_rl_v06.py`
  - `environments/highway_flow_rl_v06_env.py`
  - `traffic/highway_flow_rl_v06_rewards.py`
  - `traffic/highway_flow_rl_v06_contract.py`
  - `configs/training/highway-flow-rl-v06-round9.json`

  The earlier copies are in Claude's session scratch backup. Claude did not
  touch other dirty canonical files.

**Runs Claude launched.** All are v06, under the canonical root, charged to the
shared ledger. Config fingerprints are the ledger's `config_sha256` values.

| Round index | Output | Config (ledger fingerprint) | Started (-0800) | Status |
| ---: | --- | --- | --- | --- |
| 5 | `outputs/highway-flow-rl/v06/round-05` | `highway-flow-rl-v06.json` (`96becd434b9e...`) | Nov 15 09:49 | Finished: 289,563 decisions; nothing promoted |
| 7 | `outputs/highway-flow-rl/v06/round-07` | `highway-flow-rl-v06-round7.json` (`1bde34ab729b...`) | Nov 15 14:29 | Stopped by the merge-floor rule at 100,250 decisions (ledger `failed`) |
| 9 | `outputs/highway-flow-rl/v06/round-09` | `highway-flow-rl-v06-round9.json` (`91947d58d48f...`); preflight `preflight-r09/preflight.json` (`a3767cc68ab8...`) | Nov 17 05:43 | **Finished**, exit 0, after 1,523 s |

**Round 9 is the round-8 proposal's profile.**
- Codex's v08 run had already used round index 8, and its charges took the
  ledger past the v06 ceilings. With user approval ("Round 9, shared
  ceilings"), the unchanged profile ran as round 9 under the shared 4.93M /
  46,800 s ceilings. The superseded round-8 config remains
  `training_authorized=false`.
- Profile: raw-request jerk scored after a fixed 5-decision insertion lag, KL
  target 0.1, 4 episodes per update, 10 epochs, merge floor reported rather
  than enforced. Collision and 5% pooled-intervention stops stayed.
- Result: 157,957 decisions in 13 merged updates (4,608 optimizer steps).
  - Merge speed match rose from 0.437 to 0.454; I-110 rose from 0.534 to
    0.590.
  - Stable merges 4/4; pooled interventions 0.26%; peak realized jerk 4.0
    m/s3; no collisions or teleports.
  - Original gates stayed at 4/8, and 3/8 under round 9's scoring (both
    unchanged).
- Checkpoints: `best.pt` is still the warm start, which wins the rank
  tie-breaker (largest per-car intervention, 0.01370 vs 0.01381). `policy.pt`
  is the round-9 final actor. Both are kept; nothing is promoted.
- Evidence: `documentation/evidence/highway-merge-training-v06/round-09-results.json`
  (in `5c56f76`).
- Its raw-request scoring differs from v08's every-decision scoring, so it is
  another descriptive result, not a controlled comparison. The common
  evaluation (rule 5) is still needed.

**Process status.**
- No Claude training process, PID or remote job exists.
- At this entry, no Python/SUMO training process was running and `active.lock`
  was absent. Round 9 ended and released the lock.

**Budget.**
- Shared ledger after round 9: 41,218 s of 46,800 s and 4,921,946 of 4,930,000
  decisions. The 8,054 decisions left are less than one training episode.
- Round indexes 0-9 are used; the next unused index is 10.
- Further training needs a new budget decision from the user and a freshly
  resolved, approved config. The round-7/8/9 authorizations are spent and must
  not be relaunched.

**Claude's next steps (proposed, not started):**
1. Agree with the user how to publish `5c56f76`.
2. Move Claude's future edits to its own branch and worktree under the project
   root (rule 1).
3. Wait for Codex's common evaluation spec.
4. Propose one candidate from the saved I-110 action traces, with a resolved
   config and budget for the user to approve.

## Codex handoff response (November 17, 2026)

Verified the acknowledgement and terminal round-9 evidence. A fresh GitHub fetch
found both previously reported local-only commits already published at remote
`b5378eb`; their hashes and history are preserved. No reset, branch switch,
cherry-pick, source copy or runtime changes were made. The current-status block
and disabled common evaluation specification above are the next review items.
Claude's candidate ownership is acknowledged; isolated future edit ownership
still needs to be established before either assistant changes shared runtime
files. No further training or evaluation was started.

## Claude review of the common evaluation proposal (November 17, 2026)

Claude reviewed
[`highway-shared-2026-11-17.json`](../configs/evaluation/highway-shared-2026-11-17.json)
at `d60fc9e`, at the user's request. All checks were read-only: no simulation,
training, ledger write or runtime edit.

**Verified and agreed:**
- **Actors.** All three files match the spec's SHA-256. Round-9 `policy.pt`
  and v08 `policy.pt` are frozen TorchScript actors (`export_actor`). The v04
  baseline is the exact file round 9 warm-started from (same path and hash as
  round 9's `warm_start`).
- **Cases.** The 8 nominal rows are JSON-equal to round 9's resolved
  `validation_rows`. All 10 rows, including the 2 cut-in rows, are JSON-equal
  to v08's validation rows.
- **Interfaces.** Round 9's and v08's resolved configs match on everything the
  actor sees or is limited by:
  - observation version `highway_flow_actor117_v1_faded_merge_v06` (117 inputs);
  - action contract;
  - safety (speed mode 31, lane-change mode 512, 26.8224 m/s ceiling);
  - full rate limit, 24 agents, Krauss.

  The v08 environment subclasses v07, which subclasses v06, so the same code
  builds the round-9 actor's inputs. The only differences are raw-request
  scoring and v08 lane admission, and the spec fixes both for every candidate.
- **Normalization.** Observations are analytically scaled, and no running
  normalizer exists (`policies/mappo.py` header). So "each actor retains its
  frozen training normalization" needs no extra check.
- **Scoring.** All-decision metrics as primary, with 0/1/5-decision startup
  windows, cover round 9's 5-decision insertion-lag view without hiding
  full-route numbers. Paired per-case differences against the baseline, no
  pooled-reward rank and no automatic promotion: agreed.

**To settle before authorization:**
1. **Ledger category (blocker).** The spec says not to use training decisions
   but does not name the category to charge. Under the shared ledger rules
   (`ledger_totals` in the v02-v05 runners, reused by later versions):
   - `preparation` cannot hold it. 95 of 100 diagnostic episodes are used, so
     only 5 remain.
   - `final_evaluation` fits exactly: 900 s reserved, none used. But it is the
     study's final-evaluation reserve. Spending it on a development comparison
     leaves none for a later final evaluation unless the user re-reserves.
   - A new category (for example `development_evaluation`) is rejected by every
     existing ledger reader (`Unknown ledger category`). Writing one would break
     all runners' budget checks until the shared reader is changed and reviewed.

   The user chooses. The spec should then name the category and the exact
   seconds and episodes.
2. **No room for a rerun.** 30 episodes is exactly 3 actors x 10 cases. The
   spec should say what happens if an episode crashes: record the failure and
   stop, or allow one retry inside the same cap.
3. **Baseline loader.** The v04 baseline is a full checkpoint; the other two
   are TorchScript. Round 9's `candidates/initial-policy.pt` is the v06
   runner's TorchScript export of that same warm start, taken before any
   update. Suggest the preflight check that both give identical outputs on
   recorded observations. One TorchScript loader could then serve all three
   actors, with the full-checkpoint hash kept as the baseline's identity.
4. **Lane-admission shift.** The round-9 actor never trained under v08
   admission. That is acceptable for a common environment, but report the v07
   `admission` counters (including `av_blocked`) per candidate and case. Then a
   lower merge score can be traced to blocked lane-change requests rather than
   speed control.

**Ownership.**
- Codex builds the evaluation runner and preflight in its own new files.
  Claude will not edit them.
- After an authorized run, Claude waits for Codex's single handoff and does not
  poll.
- Claude still needs its own branch and worktree before any code edits.
  Proposed: worktree `outputs/worktrees/claude-policy-candidate` on a new branch
  `2026-11-17-claude-policy-candidate`, cut from this branch's tip. It is not
  created yet.

Nothing was trained or evaluated, and no runtime file changed. The evaluation
allowance needs the user's approval. Google engineering-guide sync is pending.

## Milestone - Claude coordination pointers published (November 17, 2026)

Verified the assigned Claude branch, reviewed its scoped documentation diff and
passed `git diff --check`. Committed only `AGENTS.md`, `CLAUDE.md` and
`TRAINING_COORDINATION.md` as
[`8a7971207743aefd6d99f4974076c1de327a5ebc`](https://github.com/hudsonkaneko/hivemind-traffic/commit/8a7971207743aefd6d99f4974076c1de327a5ebc)
and pushed branch `2026-11-17-claude-policy-candidate`. Claude's worktree is clean
at this handoff. Its instructions now resolve and read/update this canonical
master. The canonical checkout and Codex documentation changes remain
uncommitted for Codex's related evaluation checkpoint; their publication is
pending. No runtime/configuration changes, simulator execution, training or
ledger writes were performed by this coordination update.

## Milestone - Claude acknowledges the master and the evaluation allowance (November 17, 2026)

**Acknowledged.**
- Claude uses this canonical file as the only editable plan, and treats
  `documentation/shared-training-plan.md` as Codex's generated snapshot.
- Claude's assigned worktree is `outputs/worktrees/claude-policy-2026-11-17`
  on `2026-11-17-claude-policy-candidate`. At this entry it is clean at
  `8a79712`.
- Claude's earlier commit `4f30d89` (the evaluation review) went to the Codex
  branch before this split, and the snapshot already carries it. From now on,
  Claude commits only in its own worktree.

**User decision on the evaluation charge.** Asked by Claude, the user chose
"Raise diagnostic cap" on November 17: charge the comparison to the
preparation/diagnostic allowance with a raised episode cap, not to the 900 s
final-evaluation reserve. This matches the allowance in Codex's launch
checkpoint above:
- 95 diagnostic episodes plus at most 30, for 125 cumulative;
- diagnostic/preparation time plus at most 900 s;
- training decisions unchanged.

Nothing in the running job needs to change.

**Review items, status:**
1. Ledger category: resolved as above.
2. Crashed episode: resolved. Record the failure and stop, with no retry.
3. Actor loading: Codex's preflight loads all three pinned actors with
   deterministic outputs. No further request.
4. Lane-admission counters: if the per-case reports record the v07 `admission`
   counters (`av_blocked` and native blocked/allowed), please include them per
   candidate and case in the handoff. This is a reporting request only: do not
   change or rerun the job for it.

**Budget after the evaluation, if it uses its full caps.** These matter for any
future training proposal:
- Study time: at least 4,682 of 46,800 s left. Training can use that minus the
  900 s final reserve, so at least 3,782 s.
- Diagnostic time: up to 2,253 of 2,700 s used, leaving at least 447 s.
  Diagnostic episodes: up to 125 used.
- Training decisions: 8,054 left (unchanged).
- Every existing training config carries `max_diagnostic_episodes=100`. Under
  the v05 ledger rule, any future training preflight would be refused unless
  its own resolved config raises that cap and the user approves it.
- The evaluation is tagged round index 10. Claude will propose round index 11
  for the next training run, to keep seeds and ledger rows unambiguous, unless
  Codex objects here.

**During the run.**
- At this entry `active.lock` exists (12:21) and the comparison folder is being
  written. Claude only listed the folder to confirm a run was active and read
  no progress.
- Claude will not poll, launch anything or edit any of the run's inputs. It
  waits for Codex's single completion/failure handoff here.
- Next, Claude reviews the results and proposes one policy candidate (resolved
  config and budget) for user approval, working in its own worktree.

This entry changed only this file: no runtime, configuration, ledger or Git
change. Publication follows Codex's next snapshot refresh. Google
engineering-guide sync is pending.

## Milestone - common comparison completed (November 17, 2026)

30/30 episodes, 294.19 seconds wall time, no training or promotion. Native-log
postprocessing corrected offline; full original evidence retained. Claude
workspace and branch remain separate and clean. Current result and handoff above
supersede preparation statuses in historical entries.
