# I-110 transfer design and ordered continuation

**29 October status:** targeted ramp/cut-in diagnostics are complete with comfort
failures retained. Hudson's exact actor is delivered; original acceptance passes
and strict numerical parity fails. The semantic comparison below still applies.
Use the [current handoff](continuation-2026-10-29.md) for the updated next actions;
older missing-actor and untested-ramp statements below describe October 28.

28 October 2026 · Codex for Sam · engineering decisions proposed; no training.

This updates the remaining tracks after the first ACC/event tranche. Hudson's
27-feature actor has not run here. Its original frozen-source copy under
`outputs/hudson-continuation-20261028/native-scaffold-v2/source/` is the inspection
source for the comparison below; source identity is recorded in that package.
The live master and Sam refresher were read in this session. Their October 22
checkpoint is historical where Sam's newer I-110 decisions supersede it.

## Transfer is a semantic decision

| Input/action | Frozen v02 | I-110 graph_v1 | Transfer consequence |
|---|---|---|---|
| First six scalars | Speed/cap; acceleration/4.5; lane index/2; 1−position/5000; speed limit/cap; previous requested acceleration/4.5 | Similar speed/acceleration; index/(lane count−1); graph remaining/(traveled+remaining); vehicle allowed speed/cap; clipped prior request/4.5 | Matching feature names or positions do not establish matching values or meaning |
| Six front/rear neighbor triples | Three-lane index and single-road longitudinal positions; 120 m range, bumper gap, relative speed, presence | Directed connected lane paths across junctions/route branches; 120 m range and comparable triples | Branches/internal edges and missing adjacency change information semantics |
| Left/right existence and time | Hardcoded three-lane bounds; step/600 | Current graph adjacency and time/60 s | Auxiliary lane/branch uncertainty cannot be represented by legacy flags alone |
| Additional fields | None | Lane count/5, adjacent-context-known, remaining-route-known | Dropping these hides uncertainty; padding/truncating is prohibited |
| Action | Scalar normalized acceleration to [−4.5,+2] m/s²; original external headway/speed/zero-speed filters | Candidate interface only; current ACC filter +1.5/−2.5 comfort and 2 m/s³ target jerk | Different guards/caps must be explicit in any matched comparison |
| Information source | Exact simulator state, no communications or steering | Exact simulator state, no learned merge/lane-change action | Neither contract is perception or communication capability |

The original environment clips several inputs; the current graph implementation
also clips the previous request. During ACC logging that previous raw request may
exceed the original policy's action range. It is diagnostic plumbing, not an
already accepted policy training distribution. Even a three-lane restriction in
the I-110 network would alter geography, boundaries and topology; it must be a
separately named diagnostic, never a full-corridor transfer result.

Proposed decision: reproduce the exact actor in its original environment first.
Then evaluate whether a semantically exact, explicitly restricted support region
exists. If it does not, a new graph-compatible policy requires a separately
approved training protocol. Before any comparison freeze common network, demand,
seed, horizon, information access, action limits, shield and intervention metrics.
Do not tune on the original 20 fixed held-out seeds or reinterpret shape validity
as transfer acceptance. Preserve the existing hash gate and Python/Torch build
differences in the actor handoff.

## Next bounded tracks, in order

1. **Ramp interaction.** Existing eight-ramp-request cohorts and lane-change logs
   are instrumentation, not a targeted merge-conflict test. Predeclare ramp
   arrival offsets relative to agent_0, using exact demand manifests and a
   development seed. Distinguish realized merges, delayed departures, blocked
   opportunities and downstream auxiliary-lane exit choices. Keep a matched
   ordinary control, full ramp/external/unfinished burden and safety authority.
   Do not label all eight completions proof of an AV-created acceptable gap.
2. **Event adapters.** Extend the reviewed contract from leader/cut-in to explicit
   stopped occupancy, closure/reopening and police-break motion. Use independent
   event IDs/specifications; test permission restoration/overlap and realized
   extents before coupling to existing visual presets.
3. **Geometry uncertainty.** Independently review current lane count, taper/ramp
   connectivity and width against traceable imagery/survey evidence. Keep May
   2024 NAIP interpretation, lidar source epoch/datum, occluded bridge fits and
   provisional auxiliary-lane semantics separate. Record uncertainty by station
   and lane with source, evidence class, reviewer and acceptance status. Existing
   surrogate tire support does not verify lane accuracy or current as-built truth.
4. **Optional offline PeMS.** Intake District 7 station metadata, health/missingness,
   direction/lanes, timestamps/timezone and a bounded exported interval. Record
   rights and file hashes; inspect boundary/ramp queue coverage and reserve
   independent groups before calibration. Credentials/live access are not runtime
   dependencies, and synthetic arrival rates remain uncalibrated until then.
5. **Policy transfer.** Use the semantic comparison above and original actor
   reproduction gates. Exact checkpoint and preferably per-seed evaluation file
   are still needed; no further unchanged public-availability polling is useful.
6. **USD integration.** After targeted traffic acceptance, replay timestamps,
   stable vehicle/event IDs and graph states into the existing layered scene.
   Check coordinate/heading/time alignment and immutable layers. In any bounded
   physical response test, hand motion ownership explicitly to PhysX and record
   requested/realized response. A SUMO replay is not physical fleet validation.
7. **Evidence backfill.** Register current runs/results and retained failures with
   exact generators/inputs; then recover historical links only from manifests or
   original records. Keep `needs_backfill` where the motivation or generator is
   unavailable. Generated catalogs do not replace raw immutable evidence.

These tracks preserve P01–P05/P07, independent factual audit, matched CTDE and
complete burden accounting. They are not team-wide assignments, an empirical
site admission, permission to train, or a forecast of communication benefit.
