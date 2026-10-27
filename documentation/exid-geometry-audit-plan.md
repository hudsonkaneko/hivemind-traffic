# exiD geometry acceptance and protected audit intake

27 October 2026 · Codex for Sam · Prepared checks, not executed acceptance.

This implements the existing site contract and research A/B/C separation. It
does not select numerical tolerances, independent blocks or a model family from
missing data. See the [current evidence sheet](exid-qualification-2026-10-27.md).

## Before opening traffic observations

1. Verify recipient and permitted uses from the actual grant. Keep credentials,
   source data and reconstructable restricted derivatives outside public Git.
2. Inventory filenames, sizes, hashes, release identifiers and non-outcome
   date/location/continuity metadata. Do not run a whole-dataset loader,
   visualizer or exploratory statistics. Retain the received archive unchanged.
3. Establish independent day/session/interaction groups. Adjacent clips from one
   continuous traffic episode are not independent just because IDs differ.
   Keep shared vehicles and interactions in one group; record boundary buffers
   and their justification. If independence cannot be established from metadata,
   ask for continuity evidence before opening candidate audit observations.
4. Freeze explicit A (fit), B (development/admission) and C (final factual audit)
   group/file IDs in the intake manifest, with dated reviewer/custodian decision
   and hashes. No percentage split or sample count is invented here. Document
   the generalization level supported by the available blocks.
5. Exclude C from development paths/loaders and log every access. Hashing and
   filename inventory are not analytical inspection but must still be logged.
   Use directory/access separation or a second custodian where available; no
   reviewer or protection mechanism is currently assigned/implemented. Open one
   designated development recording only after this freeze. Prior analytical
   exposure disqualifies a block from being described as untouched.

**Current reservation status: NOT POSSIBLE YET — no identified package/block IDs.**
No trajectories have been opened by this session. This plan protects the next
step; it does not claim an existing reserved audit set. If data are insufficient
for independent A/B/C, report the insufficiency before narrowing the claim.

## Geometry acceptance record

For every check retain input IDs/hashes, method, uncertainty, measured error,
predeclared tolerance and rationale, reviewer, and pass/fail/unresolved outcome.
Set tolerances from source uncertainty and lane/contact needs before evaluating
withheld geometry checks. No current tolerance or acceptance result exists.

| Check | Evidence to prepare | Acceptance basis |
|---|---|---|
| Epoch and correspondence | Recording, map, background and 3D dates; paired features and change log | Same road epoch or documented compatible context; no modern replacement for missing historical lanes |
| Coordinates and anchor | Horizontal CRS/UTM zone/datum, vertical datum, units, local origin, axis/yaw convention; centroid-to-front-bumper conversion | Round-trip poses and hand-worked checks agree; no guessed EPSG or vertical offset |
| Topology | Lane count, ramp/mainline connections, merge taper, usable lane widths, shoulders and exits | Independent capture-era evidence supports connectivity and feasible merging region; no invented lane connections |
| Horizontal shape | Alignment control points plus separate check points/cross sections across ramp/mainline/taper | Report metre errors and topology disagreements; points used to align cannot also establish independent accuracy |
| Elevation/contact | Ground/deck distinction, grade, crossfall and cross sections with provenance | Height/contact errors satisfy frozen limits; flat or assumed sections explicitly classified and not called measured |
| Assets and conversion | One small licensed map/asset, texture dependencies, versioned conversion and USD composition | Consistent scale/position, no unresolved assets; topology/contact/visual/sensor layers remain separate |
| Existing-viewer integration | One development replay segment with source/date/debug overlays, stable IDs and all camera modes | Inspect render and source overlay, wheel/road contact and lifecycle; saved sources unchanged |

A texture, supplied map and enrichment derived from that same map are not three
independent geometry validations. Record shared provenance and obtain genuinely
independent checks where required. If none exists, state that limit. Do not
claim road reconstruction complete on a visual overlay alone.

## Boundary and opportunity qualification

Draw the actual visible polygon and boundary lines from development evidence;
record entry/exit time coverage, initial cohort and occlusion/tracking censoring.
Distinguish observed exits, track loss and confirmed trip completion. Record
whether a queue tail remains visible and for what intervals. When it leaves the
view, report the censored interval and a justified bound or unknown value.
Never set external waiting to zero by default. Identify independent requested
demand and downstream supply evidence before choosing the traffic domain.

Prepare an event ledger for each ramp decision window: recording/interaction ID,
times, target lane and feasible merge zone, candidate front/rear vehicles,
distance/time gaps and relative motion, observed maneuver, visibility and
uncertainty, accepted/rejected/ambiguous label, and reviewer evidence. A skipped
instant is not automatically a rejected opportunity: establish that the gap was
reachable and merging was feasible. Never turn missing neighbors into unlimited
gaps. Declare censoring/exclusion rules on development data before fitting or
admission; independently review a development sample. Keep B/C role boundaries.

Check continuity and supported traffic regimes without inspecting C outcomes
to select favorable blocks. Metadata stratification and its exposure must be
logged. Insufficient queue, opportunity or independent-block support leaves
the original study unadmitted; a narrower claim requires an explicit amendment.

Carry all requested, waiting, active, completed, removed and unfinished trips
into the existing fixed-horizon ledger, with the drain reported separately and
human/ramp groups retained. Geometry acceptance cannot substitute for traffic
admission, a fitted gap model, learned-driver acceptance or policy evaluation.
