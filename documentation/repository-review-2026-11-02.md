# Repository catch-up: Hudson and Sam

2 November 2026 · Codex for Sam · read-only inspection and planning.

Hudson now has a passing-capable, communication-enabled **v04r2 SUMO actor**,
more complete sensor fusion/segmentation experiments, and a prepared six-class
segmentation training pipeline. These advances supersede the October 29
assumption that v02 fixed-lane cruise is his newest accepted traffic actor.
They do not establish I-110 transfer, comfortable driving, sensor-driven policy
control, physical highway autonomy or causal communication benefit.

Sam's recommended next work is the shared I-110 integration/evaluation testbed:
make the existing contribution reviewable, qualify ordinary traffic and inputs,
and define a truthful bridge to the new actor. See the actionable
[Sam roadmap](sam-roadmap-2026-11-02.md). This recommendation is not a new team
assignment, a training authorization or a change to the approved research claim.

## What was checked

- Both configured public repositories: `hudsonkaneko/hivemind-traffic` and
  `samfrausto/hivemind-traffic`; the existing local `codex/sam-visual-twin` checkout.
- All **37 Hudson branches and 20 Sam branches**, matching each GitHub API SHA
  against fetched Git objects, with complete file-name trees retained.
- All **34 upstream PR records**: 23 open, of which 22 are drafts; Sam's fork has
  no own PR records. Sam's cross-fork PR is upstream PR 16. Both issue lists
  contain no separate non-PR issues at this read.
- Complete published release lists and all **97 upstream Actions runs** available
  at the read; the fork's Actions list is empty. Raw CI artifacts were not downloaded.
- The latest five feature branches, their reports, configurations, compact
  evidence and relevant observation/action/safety/training code. Older physical,
  v02 replay and communication milestones were reconciled with their recorded limits.
- Both live Google documents: the research master still has its October 22
  checkpoint; Hudson's engineering guide was modified October 31 Pacific
  (November 1 UTC) and corroborates the latest v04r2/perception records.

This is broad repository/state coverage with focused source/evidence review,
not a line-by-line correctness audit of every historical file. Private or
uncommitted work on Hudson's machine is outside the visible evidence. No
simulation, model execution, training, merge, push or shared-document write ran.

The initial normal fetch exposed a real discovery limitation:
`remote.origin.fetch` is restricted to `main`. Existing older remote branches
were present locally, but five newer heads were missing. An explicit
`git fetch origin 'refs/heads/*:refs/remotes/origin/*' --no-prune --no-tags`
retrieved them. The initial incomplete inventory is retained; the authoritative
`all-published-branches.json` matches all 57 API heads. No persistent fetch
configuration or local branch history was changed.

## Published state and integration

Both `main` branches remain at `edb71259`, the October 15 clean-clone merge.
Newer work is in stacked feature branches and open PRs, so inspecting `main`
alone gives a misleading picture of current capability.

| Track | Pinned head | Review |
|---|---|---|
| MAPPO passing and lane stability | `5f2f31a6` | [PR 30](https://github.com/hudsonkaneko/hivemind-traffic/pull/30) |
| Ground-truth semantic/LiDAR fusion | `9641d92f` | [PR 31](https://github.com/hudsonkaneko/hivemind-traffic/pull/31) |
| Frozen pretrained RGB segmentation | `b7288c3b` | [PR 32](https://github.com/hudsonkaneko/hivemind-traffic/pull/32) |
| Segmentation preprocessing and occupancy | `5c40bcd1` | [PR 33](https://github.com/hudsonkaneko/hivemind-traffic/pull/33) |
| Six-class data/trainer/GPU preparation | `f03f4abc` | [PR 34](https://github.com/hudsonkaneko/hivemind-traffic/pull/34) |
| Sam's published feature branch | `b23f24c0` | [PR 16](https://github.com/hudsonkaneko/hivemind-traffic/pull/16) |
| Sam's local implementation at review start | `e8cdbee` | 11 unpublished commits ahead of the feature remote |

PRs 30–34 are stacked in that order after PR 29's v02 replay. No merge is
implied. PR 16 still describes the older I-80/exiD direction; Sam's I-110
construction and subsequent diagnostics are local. A collaborator following
that public PR will not see the current I-110 work.

Relative to common base `edb71259`, Sam changed 129 paths and the latest Hudson
stack changed 329. Their modified-path intersection is only `AGENTS.md`,
`README.md`, `documentation/CHANGELOG.md` and `documentation/hybrid-roadmap.md`.
This suggests limited same-file overlap, **not proven mergeability or interface
compatibility**. No merge was attempted. Preserve Sam's newer I-110 scope,
research accounting and shared-write hold when reconciling these documents.

Latest head CI succeeds for PRs 30, 32, 33 and 34. PR 31's head has two failed
Clean clone runs; the inspected job failed the full CPU suite. Its public
annotation only says exit code 1, so the specific test failure is unresolved
here. Downstream green CI does not retroactively make that head green. CI and
native experiment acceptance are separate evidence.

## Hudson's control work

The [v04r2 report](https://github.com/hudsonkaneko/hivemind-traffic/blob/5f2f31a6b747dd35a070ece13d58459b4727d691/documentation/sumo-mappo-v4-stability.md)
records imitation-assisted MAPPO trained within a one-million-transition budget.
There is no teacher/scripted fallback at inference. It controls acceleration
and right/stay/left lane requests for four cars on a straight three-lane,
5 km SUMO fixture. Its **81 observations = 33 local fields + six eight-field
delivered-message slots**. This differs from both the v02 27-feature contract
and Sam's current 30-feature I-110 graph observations.

The first v04 pilot failed passing; v04r1 passed vehicles but failed the lane
change frequency gate in one test episode. V04r2 changes the training objective
and exploration, retains those failures, and selects update 175 using validation.
The later update 233 regressed; `latest.pt` is not the accepted deployment actor.

I recomputed the following from the committed 150-row per-controller/per-seed
CSV, confirmed ordered seeds 3210001–3210050 in all three arms, and compared
the learned summary with the exact committed evaluation JSON:

| 50 episodes per arm | Selected learned actor | Scripted reference |
|---|---:|---:|
| Actual slower-car passes | 74 | 84 |
| Episodes with at least one pass | 50 | 50 |
| Aggregate interventions | 3.175% | 0.2367% |
| Mean per-car progress | 1,446.908 m | 1,487.599 m |
| Maximum lane changes / car / minute | 2 | 2 |
| Collisions / overspeed / required-gap deficit steps | 0 / 0 / 0 | 0 / 0 / 0 |
| Maximum realized absolute jerk | 42.0074 m/s³ | 31.8426 m/s³ |

Both pass the declared task gates. Eight learned episodes individually exceed
5% interventions; acceptance is explicitly aggregate. Nominal endings are
60-second episodes, not completion of the 5 km road. The scripted reference
still has better progress and fewer interventions. Peak jerk is present in the
committed metrics but is not the v04 acceptance criterion: this new success
does **not** close Sam's 2.05 m/s³ comfort diagnostic. Different fixtures also
preclude treating these peaks as a paired comparison with Sam's I-110 runs.

All **23 recorded training-source hashes** were recovered and verified from
`bd59d2f`, accepting documented LF/CRLF byte representations only. The canonical
configuration digest matches `8d926dc8…`. This establishes published-source and
summary consistency, not a new independent policy rollout or raw-journal audit.

The accepted export identity is
`4cf4898df997a444624552515ea987e7e994b1ca40bb002cf91f0647be8a7245`.
Only the older v02 has a published release in the inspected release list, and
none of the 57 branch trees contains tracked `.pt/.pth/.ckpt/.onnx/.safetensors`
weights. V04r2 is described as a local output on Hudson's machine. Its export,
runtime/config bundle and preferably action journal are the next handoff inputs;
this review did not request them from Hudson or inspect private assets.

V04 retains `lane_index / 2`, lane bounds `< 3`, edge-local longitudinal
position, the 5 km remaining-distance scale and same-road message offsets.
Its lane masks and joint maneuver guard also assume that topology. A 27→30
adapter discussion is now incomplete: the current integration question is
**the full v04 observation/message/action/shield contract versus the I-110
lane graph**. Padding, truncation or hiding extra lanes is not a valid bridge.

The no-message configuration remains untrained in the reviewed records.
Communication-enabled passing is therefore established only within the reported
task; causal communication benefit is not. A future comparison must match the
selected refinement's architecture, training/teacher budget, scenario, critic
opportunity and safety contract, rather than comparing unrelated actor versions.

## Sensors, perception and physical driving

The [v02 3D replay milestone](https://github.com/hudsonkaneko/hivemind-traffic/pull/29)
already recorded the prepared vehicle's seven cameras and five LiDARs: 840 RGB
images and 600 scans over a 60-second replay. That is useful infrastructure for
Sam to reuse. Motion was recorded/kinematic, sensor lag was documented, and it
was not closed-loop physical or sensor-driven MAPPO control.

The [fusion milestone](https://github.com/hudsonkaneko/hivemind-traffic/blob/9641d92f4ba61234b44ca4924bce3bbac7ddcc55/documentation/semantic-lidar-fusion-v01.md)
records about 99.95% assigned-point accuracy with 92.58–93.17% in-image coverage
on static, 2 m/s and 10 m/s moving-traffic fixtures. **Those labels are simulator
ground-truth camera masks.** Ego was parked, and each case spans only 0.7 seconds
of captures. Timing rejection and independent visibility diagnostics are explicit;
about 1.7% of assigned points remain behind the camera-visible surface.

The [frozen-model refinement](https://github.com/hudsonkaneko/hivemind-traffic/blob/5c40bcd199c04c41c3fb3b855c759490995d5d4f/documentation/segmentation-refinement-preview-v02.md)
uses real RGB predictions from a pretrained Cityscapes SegFormer B0 on the same
24 previously inspected captures. Aspect-preserving 1024 preprocessing improves
shared-class mean IoU from 71.52% to 86.06%, and reduces non-road pixels falsely
called road from 28.48% to 6.25%. Predicted semantic labels projected onto LiDAR
reach 93.74% accuracy, distinct from the 99.95% GT-mask result. Generic-obstacle
semantic recall remains zero; geometry-only occupancy detects 89.28% of eligible
obstacle returns. Those are point metrics, not object recognition or safe driving.
This exploratory, correlated sample is not independent generalization evidence.

The latest [six-class preparation](https://github.com/hudsonkaneko/hivemind-traffic/blob/f03f4abc114bf9e078805cde8651e04fef53e82b/documentation/segmentation-training-preparation-v02.md)
records a 3,600-frame corpus across 180 scene groups, split 2,880/360/360, with
background/road/lane/vehicle/pedestrian/obstacle labels. A trainer, reviewed-manifest
checks, checkpointing and an isolated RTX 5070 CUDA preflight exist. Both
`launch_enabled` and `training_authorized` remain false in the inspected config;
GPU preflight performed zero optimizer steps. There is **no trained six-class
result yet**. Procedural scenery, static human poses, shared vehicle geometry,
absence of LiDAR in this new corpus and deferred object/temporal evaluation
remain material limits. Do not treat its 3,600 images as an I-110 factual audit.

The earlier physical RSL hazard track is separate: isolated low-speed development
checks passed, but the documented full-scene learned transfer departed its lane
after 6.659 m. Its v03 progress is still described as in progress without a newer
completed acceptance record in the inspected branch. The 60 mph CIRCLES/PPO
native retry remains deferred in its report. New SUMO passing and perception
work do not establish completion of either physical track.

## What changes for Sam

1. Preserve v02 as the reproduced compatibility baseline, but use **v04r2** as
   the current candidate for a future passing/cooperation handoff. Its own
   original-fixture reproduction must precede claims about transfer.
2. Continue the I-110 anticipation/comfort problem. Hudson's latest result
   strengthens the available controller library but does not solve that gap.
3. Reuse the existing sensor/replay pipelines once traffic and coordinate checks
   pass. Sam's useful complementary work is an independent corridor-based
   validation suite, rather than assuming synthetic perception scores transfer.
4. Bring the public handoff up to date when publication is authorized. The
   11 unpublished implementation commits and stale PR 16 are a concrete
   collaboration gap, independent of algorithm quality.
5. Keep the scientific gates: empirical lane/merge/queue evidence, independent
   human gap opportunities, protected factual audit, complete trip burdens,
   matched local/communication learning and bounded physical response validation.
   P01–P05/P07 are not closed by any newly inspected milestone.

The older v02 numerical discrepancy (34 values beyond frozen 1e-9 tolerance)
is retained as a separate reproduction issue. It is not evidence that v04 fails,
and cannot be erased by changing versions. A matched-runtime/action-journal
investigation can remain bounded while independent corridor work proceeds.

## Review evidence and reproduction

Local immutable package: `outputs/repo-review-20261102/`.
`review-plan.json` records Sam's request before the review; API responses,
initial/final Git inventories, pinned source copies and source inventories
capture what was inspected. `verify-records.py` generates
`consistency-review-v2.json`; the first terminal-generated summary remains
retained as `consistency-review.json`. The source copies were read, not imported
or executed. `api-followup-access.json` records complete CI coverage and the
limited failed-job annotations. Live document access has separate records.

The final activity registry links this review and roadmap to those inputs.
Recorded upstream test totals/independent audits are Hudson's published evidence;
this session's fresh checks are branch/source identity and committed-result
consistency only. No upstream tests or simulator runs were silently claimed as
newly executed. No new teammate assignment or team-wide adoption is inferred.
