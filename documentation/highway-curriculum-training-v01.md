# Highway curriculum pilot v01: completed, not accepted

November 2, 2026 local / November 3 UTC. [Draft PR 35](https://github.com/hudsonkaneko/hivemind-traffic/pull/35).

The approved MAPPO pilot completed **499,998 active-agent transitions**, 177
updates and 8,184 optimizer steps in **6,758.797 seconds** (1 h 52 min 39 s).
No runtime failure occurred. The remaining two transitions cannot form the next
complete joint step. The agent waited for process completion without inspecting
progress or performing analysis during learning. No Isaac or rendering job ran.

This trained the [20-case highway curriculum](highway-curriculum-v01.md):
cruise, following, passing and merging across generated 2/3/4/5-through-lane
roads plus the provisional I-110 layout. It does **not** establish adaptation
to arbitrary highways, learned physical steering, or a replacement for accepted
v04r2. SUMO still owns lateral motion and the safety protections.

## Frozen configuration and source

The exact [authorized configuration](evidence/highway-curriculum-training-v01/training-config.json)
was the reviewed 500,000-transition / 7,200-second CPU pilot, with authorization
enabled. Its canonical config SHA-256 is
`38fc212de8f7d2c20bb5a8c2524e77d909f31686ec83ab351ca9935714ac1730`.
The repository default remains disabled to prevent accidental new learning.

Runtime: Python 3.11.9, Torch 2.14.0+cpu, TraCI 1.27.1; one worker and one Torch
thread. The actor contract is `highway_curriculum_graph_messages_v1`, with 81
observations and two actions (normalized acceleration and legal lane delta).
Inputs are SUMO state, map/route features and delayed messages; camera/LiDAR
inputs are not used. This contract is incompatible with the v04 weights.

The launch started from Git HEAD `3dbcdaa5edaecc65c4536ec30a00a9ff86ff22de`
plus the recovery changes described below. The ignored launch package retains
authorization, source snapshots, diffs, exact command and completion status:
`outputs/i110-merge/v01/launch-20261103T003430Z/`.
[Provenance](evidence/highway-curriculum-training-v01/training-provenance.json)
records actual runtime source/network hashes; no tracked runtime source changed
during learning. The source commit alone is not the complete launch snapshot.

## Validation results

The original ranking selected update 10: it drove at just **7.64% of the speed
limit** averaged across the 12 validation cases. It won because lower intervention
burden was ranked ahead of driving competence after tied merge/cohort scores.
Later matrices had eight passes and much higher speeds. That is a selection
defect; the original `best.pt` and `policy.pt` are preserved unchanged as evidence.

After completion, the retained final checkpoint (update 177) was evaluated on
the same 12 validation cases, covering independent 2/4/5-through-lane layouts at
55 mph. This was a frozen evaluation with **zero optimizer steps**:

| Task | Cases | Mean fraction of applicable speed limit | Safety override fraction | Task outcome |
| --- | ---: | ---: | ---: | --- |
| Cruise | 3 | 98.76% | 94.46% | All three exceed the 90% cruise target |
| Following | 3 | 85.66% | 100.00% | Motion depends on safety intervention |
| Passing | 3 | 78.30% | 99.81% | 3, 2 and 3 counted passes |
| Merge | 3 | 75.17% | 91.50% | 0 of 6 ramp cars successfully merged |

Across all cases: **0 collisions, 0 teleports, 0 overspeed steps**, but **96.44%
overrides** versus the proposed maximum of 5%. The two merge failures per case
also fail the 90% merge target. These are shielded results, not evidence that
the raw actor is safe. All 504 requested cohort trips remained unfinished at
the fixed primary horizon; they were retained in accounting rather than dropped.

[Per-seed comparison](evidence/highway-curriculum-training-v01/validation-comparison.json),
[per-task metrics](evidence/highway-curriculum-training-v01/final-task-metrics.json)
and [17 validation matrices](evidence/highway-curriculum-training-v01/validation-matrices.json)
are retained. **No held-out test seeds were run.** Unseen six-lane adaptation
remains untested. The runtime `passed=true` means execution completed, not that
the quality gates passed.

## Revisions and diagnosis

Before launch, recovery handling was corrected so an elapsed wall budget is
normal bounded completion, while genuine timeouts remain errors. Resume carries
the previous selected actor forward rather than losing it when no later actor
improves. These recovery paths were tested; this run did not need recovery.

After the run, validation ranking was revised to prioritize safety, per-task
cruise/passing/merge competence, merges/passes and minimum cruise speed before
intervention minimization. Ranking is versioned; resuming an older checkpoint
recomputes its score from its saved validation reports and preserves the original
source checkpoint. Missing evidence fails closed. Acceptance targets were not
weakened. The revised ranking was **not** used during the historical training run.

A read-only sub-agent audit found persistent acceleration saturation, not an
optimizer crash. In late training merge seed 5100175, 90.6% of acceleration
requests exceeded 1.8 m/s²; mean requested/filtered/realized acceleration was
1.915/0.309/0.199 m/s². Safety guards repeatedly limit requests even after the
car reaches its speed or following constraint. See the
[action audit](evidence/highway-curriculum-training-v01/action-saturation.json).

The revised comparison prefers the final actor to the original parked selection.
It does not retrospectively recover intermediate weights: only update 10 and
update 177 model states were retained. The final actor was exported separately
as a **diagnostic candidate, quality_accepted=false**, with a successful
actor-only inference round trip. No old artifact was overwritten.

## Retained artifacts and next revision

- Training run: `outputs/i110-merge/v01/train-20261103T003534Z-eaf909f7/`.
  Original `best.pt`/`policy.pt` and resumable final `last.pt` remain here.
- Final validation: `outputs/i110-merge/v01/evaluate-20261103T023055Z-8204e1e5/`.
- Faster diagnostic actor: `outputs/i110-merge/v01/postrun-review-20261103-v01/policy.pt`,
  with `policy-contract.json` explicitly marking it unaccepted.
- Full metrics, trip ledgers, compressed action journals and source snapshots
  remain local under `outputs/`; launch log is
  `logs/highway-training-launch-20261103T003430Z.log`.
- [Artifact hashes](evidence/highway-curriculum-training-v01/artifact-hashes.json)
  and [candidate contract](evidence/highway-curriculum-training-v01/candidate.json)
  identify the retained files. Weights and raw logs are not in GitHub.

The approved pilot budget is spent; no additional learning was started. The
next experiment should address acceleration tracking and deterministic merging:
separate speed-cap/following/lane-gap intervention causes, use feasible cruise
and merge demonstrations for training-only initialization, and assess a staged
curriculum against these same validation gates. Safety guards should stay on.
Those learning changes and a new budget require a reviewed configuration.

Final verification includes focused recovery/ranking tests, existing MAPPO and
SUMO regression tests, and the frozen validation run. See
[verification](evidence/highway-curriculum-training-v01/verification.json).
