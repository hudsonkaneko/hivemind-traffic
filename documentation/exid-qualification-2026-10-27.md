# Aachen-Brand access and site evidence sheet

27 October 2026 · Codex for Sam · Public-source qualification; no site admission.

**Status:** no exiD grant or restricted package was identified in the scoped local
search. Sam has been asked whether access is unapplied, pending or granted; no
answer is recorded at this checkpoint. The 22 October recommendation remains
conditional. P01–P05 and P07 remain open.

The live shared master and Sam's refresher were read successfully today. They
contain the authorized 22 October closeout, superseding older local statements
that it was only drafted. Today's shared Doc was not edited. Initial checkout:
`codex/sam-visual-twin`, clean at `1884f8a`. See the
[handoff](team-handoff-2026-10-22.md) for verified viewer/accounting capabilities.

## Evidence and missing fields

| Field | Evidence as of this checkpoint | Needed before acceptance |
|---|---|---|
| Access/application | Only the unsent draft and prior public source copies found; Sam's application/grant status unknown | Grant/package location and authorized recipient; actual included products |
| Candidate identity | A44 Aachen-Brand is the prior recommendation. Autobahn authority confirms the interchange is on A44. Prior Hou article identification is recorded in the site decision; today's direct article read returned 403 | Publisher location ID and matching capture-era map/background; interchange identity alone is insufficient |
| Exact location/recording IDs | UNKNOWN; no restricted metadata inspected | ID crosswalk, recording list, footprint polygon and source hashes |
| Travel direction | UNKNOWN | Carriageway destination/bearing verified against matched map and a development trajectory |
| Acquisition dates | UNKNOWN for traffic, map, imagery and 3D asset | Calendar dates, clock/time zone, intervals, asset acquisition versus release dates, road-change check |
| Map/3D edition | Current public link resolves to `exiD-Format_2_1.pdf`; actual package release and per-site editions UNKNOWN | Package inventory and map/asset version relationship |
| Rights | Publisher requires an application and limits distribution; a team member's grant cannot be assumed to cover others | Intended conversion, shared compute, road derivatives, screenshots and release permissions documented from the actual grant |
| Geometry and height | No Aachen road/mesh inspected. New format document describes flat terrain in OpenDRIVE | Independently supported grade/crossfall/deck heights; visual 3D assets are not surveyed pavement |
| Observed boundaries | UNKNOWN; dataset-wide coverage claims are not this site's footprint | Visible mainline/ramp entry and downstream exit lines, occlusions and time coverage |
| Upstream/ramp queues | UNKNOWN; absence of observations cannot mean zero queue | Queue-tail coverage or independent synchronized evidence and defensible bounds; distinguish requested departures from observed entries |
| Accepted/rejected gaps | UNKNOWN; no event extraction or model fit | Physically available opportunities, decision windows, front/rear partners, censoring and uncertainty, not just completed lane changes |
| Independent blocks/regimes | UNKNOWN; no A/B/C IDs assigned and no audit data actually reserved | Dates/session continuity, interaction correlation groups, independent blocks and support for both proposed regimes |

Sources: [publisher/application and terms](https://levelxdata.com/exid-dataset/),
[format 2.1](https://levelxdata.com/wp-content/uploads/2024/03/exiD-Format_2_1.pdf),
[Autobahn interchange context](https://www.autobahn.de/planen-bauen/projekt/a544-ersatzneubau-der-haarbachtalbruecke-bei-aachen),
and the [22 October site decision](highway-twin-site-decision-2026-10-22.md).

The newer public schema is materially relevant: map-based enrichment is
package-dependent; metadata supplies weekday/hour but not a calendar date;
position fields describe centroids with UTM origin offsets. Approximate location
coordinates are not alignment controls. The format's OpenDRIVE version is 1.4,
which is a schema version, not the site's asset edition. These are public format
statements, not verified contents of Sam's future package. Publisher clarification
is needed on measured elevation and included enrichment before building.

## Immediate action

If not applied, Sam should complete the identity/institution fields in the
[prepared request](exid-access-request-draft.md), review the application and
terms, and submit through the publisher when ready. If pending, record that
status; if granted, supply the local package/grant location. No application,
message or license acceptance was performed here. Add the new questions below
to an explicitly authorized request; the original draft remains preserved.

- Does the academic grant include the map enrichment and matching 3D files?
- Is the supplied Aachen-Brand road elevation measured, inferred or flat? Which
  source, capture date, vertical datum and accuracy support it?
- Can the publisher provide a metadata-only date/continuity inventory so complete
  independent audit blocks can be reserved before trajectory inspection?

Use the [acceptance and audit plan](exid-geometry-audit-plan.md) and
[blank intake manifest](templates/exid-site-intake.json) next. Unknowns remain
null; the template is not evidence that a partition exists. The proposed F0 date
in the research plan is today, but its evidence conditions are not satisfied.
Record this slippage; do not backdate a freeze or assume the December target.

## Search, provenance and limits

Filename searches covered project paths including ignored files for exiD,
Aachen and recording metadata names; documentation/output text references and
relevant archive/map/grant filenames were inspected. Matches were the draft,
public source copies, handoff and related notes. Runtime/cache directories were
excluded; one unrelated packman runtime path denied traversal. Arbitrarily named
archives, external storage, email and teammate machines were not searched. This
is a scoped negative finding, not proof that Sam has no access.

Local evidence: `outputs/exid-qualification-20261027-a/`, including successful
public retrieval hashes, live-master extract, PR metadata, scan/access notes and
verification manifest. Web access to the article returned 403 and to the PR page
failed; GitHub API status was obtained separately. Initial sandbox network
attempts failed before a permitted bounded retry succeeded. Historical
22 October source copies and research editions remain intact.

Geography provider is **Geobasis NRW** (North Rhine-Westphalia); previous I3S
metadata is context evidence only. No new NRW tile, Aachen mesh, traffic payload,
simulator session or training run was acquired/launched here. Reuse the existing
free/follow/overview viewer and complete-trip ledger after qualification. Keep
SUMO traffic ownership, bounded Isaac response, matched local/communicating CTDE,
human/ramp/external/unfinished burden and independent uncertainty. Hudson's
reported 42-state-input, assisted-braking, two-action/60 Hz policy still lacks
accepted full-scene learned-driver and following/merge evidence.
