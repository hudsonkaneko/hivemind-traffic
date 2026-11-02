# Sam's next-work roadmap

2 November 2026 · Proposed by Codex for Sam after the
[fresh two-repository review](repository-review-2026-11-02.md).

**Your highest-value next contribution is a qualified I-110 integration and
evaluation testbed.** You already have geographic construction, traffic accounting,
replay and diagnostic work. Hudson now has stronger learned traffic control and
perception infrastructure. Connecting those pieces with explicit acceptance
tests is a useful next direction. This is a recommendation for your work, not
an inferred permanent division of labor or a newly adopted team commitment.

## Immediate checkpoint: make the collaboration state accurate

Prepare one current handoff containing your I-110 entry point, scene/network
versions, tested capabilities, known comfort failures, complete accounting,
local-artifact prerequisites and the current controller candidates. Your public
PR 16 still says I-80/exiD; your 11 local implementation commits are not in it.
The review is already prepared locally; publishing/updating the PR is a separate
action under the existing no-push boundary.

Pin Hudson's candidate to **v04r2 selected update 175**, not `latest.pt` and not
the older v02 actor. Prepare a precise handoff request for the existing export
hash `4cf4898d…`, matching configuration/source/runtime, original evaluation and
preferably action/state journal. No request has been sent. His source and compact
metrics are already recoverable, so missing weights need not block your corridor
work. Keep v02 available as the previously reproduced baseline.

Done when a collaborator can identify one current entry point, exact versions,
what to obtain locally, and what is still unaccepted without reading chat history.
Also make all-head branch discovery explicit: the current normal fetch only
updates `main`. Review the dependency stack and exact-head CI before integrating
features; PR 31's standalone CI failure is unresolved despite green downstream CI.

## Work sequence

| Order | Your concrete work | Deliverable and exit check | Dependency |
|---|---|---|---|
| 1 | Diagnose I-110 ramp convergence and feasible braking | Current-state incoming-path/conflict-point records; no scripted future truth. Show warning lead versus a jerk-limited response envelope, then a bounded matched intervention only if supported. Retain unchanged smooth ACC, safety authority and all failures. | Existing local traces and graph; can start now |
| 2, in parallel | Qualify the corridor's evidence | Station/lane uncertainty ledger for lane count, taper, ramps, widths, grade and epoch; separate inferred geometry from independently checked evidence. Inventory observational boundary/ramp/queue coverage and protect an untouched audit partition before calibration. | Existing imagery/lidar; optional offline PeMS exports if available |
| 3 | Specify and reproduce the new controller handoff | Hash-verified v04r2 original-fixture evaluation, then an explicit observation/message/action/reset/shield support matrix against the I-110 graph. Fail closed on unknown/unsupported topology; no padding/truncation. | Delivered v04r2 export; original source/config already available |
| 4 | Produce one traceable integrated I-110 demonstration | Accepted traffic trace replayed into the existing layered USD scene, with stable vehicle/event IDs, coordinate/heading/time checks and unchanged source layers. Add the prepared sensor rig only after basic replay alignment. | Traffic/geometry checks and reconciled interfaces |
| 5 | Freeze the actual research comparison | Empirically supported human gap models, matched local/communicating actors with equal critic/training opportunity, competitive scripts, dispersed/clustered placement, full external/ramp/unfinished burdens and independent audit. Explicit training budget/approval before launch. | Qualified site/data and supported competent controllers |
| 6 | Validate the consequential physical/sensing assumption | One bounded physical ego response experiment feeding a justified surrogate back into traffic; separately score predicted perception on independent corridor cases if sensors enter scope. | A measurable traffic comparison and explicit motion ownership |

Orders 1 and 2 should progress together. Orders 3 and 4 can advance in preparation
while waiting for a delivered actor or observational data, but their acceptance
must not be inferred from a visually convincing scene or a passing unit test.
These are dependency stages, not promised dates or a commitment to finish all
research before a particular deadline.

## First concrete engineering task

**Preview implementation checkpoint, 2 November:** Sam authorized the controller
and matched test. [Preview v1](i110-preview-2026-11-02.md) now has 15 completed
runs, exact baseline/repeat checks and a paired Isaac replay. It is rejected as
the new baseline: five primary comfort failures and 21–31% cut-in cohort cost.
Next is path relevance and native-ACC/recovery diagnosis before predeclaring v2.
The [portable team handoff](i110-team-handoff-2026-11-02.md) is prepared locally;
GitHub still has the older b23f24c feature head. Corridor qualification and v04
delivery/support remain independent work, not blocked by this failed candidate.

**Continuation checkpoint, 2 November:** the offline convergence and stopping
diagnostic is now complete. See [results](i110-convergence-2026-11-02.md).
At 120 m, the two late ramp targets are exposed 0.6/1.8 s before leader entry,
but their first constant-speed stopping margins are −10.564/+5.708 m; the latter
also fails the 2 s headway or hard-braking assumption. Expanded 240 m context
appears earlier but requires extra available state. All six derivations repeat,
37 targeted tests pass, and no trajectory changed. Next: predeclare the preview
controller's observation range, conditional-path rule, braking reserve and
recovery before a bounded matched intervention. Geometry qualification remains
an independent next track. The original task specification below is preserved.

Start from the existing six development traces. The current adjacent margin
warns before the scripted cut-in but excessively in controls; it does not see
the critical ramp car until it is already a leader. Therefore:

1. Enumerate incoming paths that converge with the ego's supported route. Record
   signed distance to a defined conflict point, current speed, path identity and
   explicit unknowns. Do not use the event actor's name or future command time.
2. Predeclare a current-state warning rule and a physically stated, jerk-limited
   braking envelope. Measure whether it would allow enough lead time; a warning
   one 0.1 s step before hard braking is not established comfort feasibility.
3. Retain lateral-off, unshifted-ramp and exposure controls. Report warning
   exposure as well as misses, then measure actual motion if an intervention is
   justified. Requested smooth commands alone do not establish realized comfort.
4. Keep complete cohorts, primary horizon and separate drain, ramp/human/external
   burden, collisions, removals, teleports and realized jerk. Use development
   cases first and freeze fresh verification cases before examining outcomes.

The existing 2.05 m/s³ diagnostic bound remains disclosed; failure does not
justify weakening safety. Hudson's v04r2 peak is about 42.007 m/s³ on its own
fixture, so neither controller's existing acceptance substitutes for this test.

## What the v04 handoff needs to resolve

V04r2 uses 81 fields and an acceleration-plus-lane action. I-110 currently has a
30-field graph diagnostic and four/five lanes with ramps/internal links.
Resolve lane identity, graph-continuous neighbor/message distances, boundary and
route progress, previous requests, message age/intent, lane masks/cooldown, joint
requests, resets/delayed spawns and shield/intervention accounting. A three-lane
straight-road actor cannot be silently relabeled as a full-corridor merge actor.

If no truthful restricted adapter is supported, document that result and prepare
a graph-compatible training proposal with matched baselines. Training remains
a separate approval decision. Do not spend the original actor's held-out seeds
on repeated tuning, and retain the separate unresolved v02 numerical parity issue.

## Perception work to coordinate rather than duplicate

Hudson already has the 3,600-frame six-class preparation, trainer and GPU preflight;
both launch flags are currently false. Your complementary deliverable would be
an independent I-110 evaluation specification: unseen road geometry and vehicles,
distance/size and occlusion strata, false-drivable area, boundary/object recall,
timestamp/dropout failures and moving-ego alignment. Define the partition before
inspection and keep it out of tuning. New corpus LiDAR and temporal/deskew tests
are still missing; GT semantic fusion accuracy cannot stand in for prediction
quality. Sensor outputs are not yet MAPPO observations.

Reusable crash/closure/police-break SUMO adapters remain a bounded side track
after ordinary traffic is credible. Keep them separate from the main recurrent
merge research comparison. Optional offline PeMS can inform traffic calibration,
but aggregate detector counts alone do not identify human accepted/rejected gap
opportunities or prove complete ramp/external queue visibility.

## Research and communication boundaries

The compelling project question remains whether credible human traffic-model
uncertainty changes the local-versus-communicating controller decision. An
accepted passing policy, attractive digital twin or high segmentation score
does not answer it alone. The learned v04r2 actor still trails the scripted
reference on recorded progress/interventions, and a matched trained no-message
arm is absent. Preserve that strong script in the comparison.

The shared research master is older than the current engineering guide; keep
their dates and roles explicit. Prepare local document-ready updates under Sam's
existing shared-write hold. No teammate messages, new assignments, public push,
training or implementation beyond the requested review was performed here.

Evidence: [repository audit](repository-review-2026-11-02.md),
`documentation/evidence/repo-review-20261102.json`, and the immutable
`outputs/repo-review-20261102/` inventory/source/consistency records.
