# I-80/Powell qualification decision — 22 October 2026

Prepared by Codex for Sam under the approved Steps 4–5 research design. This
continues the [initial screening](merge-accounting-2026-10-22.md), using the
existing viewer and complete-trip evaluator. No training or partner-branch merge.

**Decision: usable for historical reconstruction and development; NOT ADMITTED
for the full factual-admissibility/communication experiment.** Do not fit the
human gap model or freeze the confirmatory experiment on the current evidence.
This is a documented qualification outcome, not a rejection of I-80 forever.

The evidence package is `outputs/i80-qualification-20261022-b/` (local/ignored).
Its manifest, scripts, request/range logs, source hashes, diagnostics and scene
preserve how this decision was reached. Raw third-party files and private vehicle
geometry are not included in a Git clone or this PR.

## What changed since the first screening

An older **official DOT attachment** remains readable. Its ZIP directory contains
the original GIS, CAD, detector and trajectory packages. Selective byte requests
retrieved GIS/CAD/detector support and only the first trajectory period's original
TXT; both later trajectory contents remain unopened and unretrieved. Directory
listing is recorded separately from inspecting observations.

- [Historical I-80 attachment](https://data.transportation.gov/api/views/8ect-6jqj/files/10ed9f47-ab08-4df2-b46a-29f1683ceffa?download=true&filename=I-80-Emeryville-CA.zip):
  441,781,993 bytes advertised; not downloaded in full.
- First-period original TXT: 179,300,362 bytes, ZIP CRC checked;
  SHA256 `a169ccbfd41bcb48e46da832857a0bac48bd96465423ceade2967bce2c8366df`.
- All **93,618 early-window API rows match the original across all 18 fields**,
  including exactly matching keys, positions, speeds and lane labels. This checks
  conversion/source identity, not measurement truth.
- The original first-period file contains 1,262,678 rows and 2,052 vehicle IDs.
  Its epoch range is 15:58:55.3–16:15:36.5 PDT on 13 April 2005, a 1,001.2-second
  span. Thus the earlier offset from the rounded published 16:00 label is already
  in the original file. No guessed clock correction was applied.

`qualification-protocol.json` was saved before first-period retrieval. It amends
the earlier acquisition cap: first-period source checks are now allowed, while
behavioral/coordinate diagnostics remain within the previously exposed 120
seconds. Neither the 17:00 nor 17:15 trajectory file was accessed. No final A/B/C
partition is asserted from filenames alone.

## Admission matrix

| Requirement | Evidence and disposition |
|---|---|
| Reproducible source access | **Pass for acquired files.** Original support packages and first-period TXT have hashes and validated ZIP reads. Current catalog archive and older attachment differ; do not substitute silently. |
| Historical site and geographic context | **Partial.** Parsed 582 historical GIS line records, six camera polygons and 52 sign/signal points. Modern official Caltrans centerlines are accessible through ArcGIS. Independent anchors, lane graph, merge taper and road-surface checks remain open. |
| Requested demand, initial cohort and external queues | **Fail for current evidence.** Original trajectory documentation omits boundary-incomplete trips. Available detector stations do not directly measure upstream Powell demand or the ramp outside queue. An accounting implementation cannot recover unobserved arrivals. |
| Gap-acceptance evidence | **Not qualified.** All 12 candidate 7→6 label changes occur between local longitudinal positions 202.985 and 203.840 m. That narrow band suggests a lane-end/label convention could be involved; it does not establish physical merge timing. |
| Independent factual audit | **Partial.** Ten detector dates exist, but trajectories cover one day. Other-day downstream aggregates do not provide other-day merge interactions. Later trajectory contents are protected; interaction buffers and usable A/B/C support remain unqualified. |
| Measurement quality | **Partial.** Early keys and sample spacing pass. Position-derived longitudinal speed differs from reported speed by as much as 17.93 m/s in a single step; this is a diagnostic residual, not a collision or estimated real acceleration. No correction/smoothing was fitted. |
| Two supported demand regimes and one dominant merge | **Open.** Published congestion descriptions are insufficient. Downstream restrictions, HOV behavior and boundary supply must be separated from the focal merge mechanism. |
| File rights and release | **Open for raw redistribution.** Current catalog CC BY-SA and legacy PDF restrictions differ; preserve original metadata and keep raw files local. |

This assessment leaves P01–P05 and P07 open where their requirements are not met.
No decision threshold was relaxed to make the candidate pass.

## Detector and merge-event findings

The detector archive contains 172,800 rows: six stations (1, 3, 4, 5, 6, 7), ten
weekdays from 7–20 April, 2,880 timestamps per station/date and no duplicate
station/time keys. These are five-lane, 30-second records. The original study-site
diagram places the available stations downstream of Powell. It depicts a station
8 upstream, but station 8 is absent from this package. No direct ramp queue
observation is established. A six-lane trajectory section must not silently be
equated with a five-lane detector section.

All detector dates/station keys were inventoried; numerical health diagnostics
used **7 April only**. The whole detector ZIP was acquired, so it is not described
as never accessed or never downloaded. Later-date traffic values were not
analyzed. On 7 April, adjusted volumes differ from the provided maximum-volume
columns in 7,390–8,354 of 17,280 rows per lane. These processed/imputed values need
an explicit handling rule; this observation alone does not prove invalid counts.

Of the 12 early merge-label candidates, 11 have both longitudinal lane-6 neighbors
visible, and 11 have one second of stable labels on each side. Three produce a
negative naive front or rear net longitudinal gap. Neither a longitudinal overlap
nor a lane-ID transition proves a physical conflict or gap acceptance. Curved
lane-relative geometry, lateral overlap, vehicle footprints, rejected
opportunities and video/source review are needed before using these events for
the principal uncertainty. No gap distribution or behavior parameter was fitted.

## Exactly how Esri contributes

Use the publisher-owned [Caltrans State Highway Network Lines item](https://www.arcgis.com/home/item.html?id=77f2d7ba94e040a78bfbe36feb6279da)
and its [ArcGIS feature layer](https://caltrans-gis.dot.ca.gov/arcgis/rest/services/CHhighway/SHN_Lines/FeatureServer/0).
The item owner is `California_Department_of_Transportation`; the current layer
description identifies a June 2025 TSN extraction. A bounded query returned EB
and WB Route 80 features (OBJECTIDs 4566 and 860), without a transfer-limit flag.
This replaces the republished OPR item as the preferred modern context source.
The service's embedded secondary item ID did not resolve; that failure is retained.

The layer supplies **directional highway centerlines**, not individual lanes,
ramp taper, pavement height, deck geometry or 2005 traffic conditions. The checked
conversion is State Plane NAD83 California zone III survey feet (EPSG:2227),
subtract origin `(6042500, 2132800)` feet, then multiply by `1200/3937` to metres;
X east, Y north, Z up. ArcGIS returned legacy WKID 102643 with latestWKID 2227.
Historical shapefiles have no PRJ file; their CRS comes from the original metadata
and remains subject to independent anchor verification. No vertical datum or
elevation is invented. Modern data is visual context; historical linework governs
the future empirical road reconstruction.

The [FHWA historical diagram](https://www.fhwa.dot.gov/publications/research/operations/06137/images/image137b.jpg)
was visually inspected: it identifies the Powell on-ramp and Ashby off-ramp just
beyond the trajectory study boundary. Downstream supply and nearby route choices
therefore require explicit treatment before asserting one isolated bottleneck.

The local geographic overlay separates original linework, camera footprints,
observed tracks and modern centerlines. Sharing coordinates and looking plausible
does not certify survey accuracy or the lane graph.

## Existing viewer foundation

`scripts/build_i80_inspection.py` composes a separate inspection scene using the
same referenced car, stable vehicle paths, motion-layer authoring, camera rigs,
playback controls and source-sample telemetry as the accounted merge viewer.
The viewer's title and explanatory labels now come from the scene manifest;
existing synthetic scenes retain their defaults.

This scene has 212 observed identities, 1,200 source frames and 93,618 measured
vehicle positions. Gray lines are historical GIS; orange lines are modern
Caltrans/ArcGIS context; green lines are camera coverage. The ground is explicitly
a flat inspection background, **not a reconstructed road surface**. Heading is a
centered up-to-one-second position secant for display, with a north fallback when
stationary. One supplied visual car stands in for differing observed dimensions
and classes. No physics, controller inference, demand reconstruction or trip
completion is implied. The original accounted synthetic merge remains separate.

Source/sample composition verification passed 254,400 lifecycle/pose/heading
checks with zero position error, 1,272 mesh instances and no active unresolved
assets. Two coordinate anchors and an eastward heading pass synthetic conversion
tests; these are software tests, **not independent surveyed control points**.
Runtime and repository test results are recorded in the evidence manifest.

Full repository retry: **1,797 passed, one private-asset skip, 36 subtests passed**
in 227.75 seconds, exit 0. Two warnings concern the pre-existing missing
pytest-timeout plugin. The external retry supervisor bounded execution at 450
seconds. The initial run reached past 83% without a printed assertion failure
before its 300-second timeout; its log is retained. The replay viewer was active
for part of the initial run and closed for the retry, but no causal throughput
claim is made. Focused conversion/replay/accounting tests also passed (10).

RTX inspection produced visually checked follow/overview images over 360 updates;
all seven state/control callbacks passed and saved layers were unchanged. The
Isaac processes were subsequently observed absent and shutdown was logged, but
the PowerShell launch wrapper remained pending. Its verified orphan console and
wrapper were stopped; **natural shell exit 0 is not verified**. This qualification
does not claim a clean shutdown regression pass or manual GUI interaction.
Unrelated Kit work was subsequently observed on the workstation and left alone;
no exclusive-GPU timing or rendering-throughput result is claimed. See
`render-completion.json` and the retained render log.

Build from the checkout with the traffic Python, using acquired local paths:

```text
python -m scripts.build_i80_inspection --sample SAMPLE.json --gis PARSED_GIS.json --esri CALTRANS_2227.json --vehicle VEHICLE.usda --output outputs/NEW_INSPECTION
python scripts/check_visual_twin.py outputs/NEW_INSPECTION/world.usda --report outputs/NEW_CHECK.json
scripts\open_visual_twin.cmd outputs\NEW_INSPECTION\world.usda
```

The GIS JSON is produced by the evidence package's scoped shapefile/DBF reader;
its source and output hashes are retained there. Raw files and that local analysis
package must be obtained separately for empirical regeneration. A clone supplies
the inspection builder and public-fixture tests, not the private scene.

## Next decision and bounded closure task

Proceed with historical geometry inspection and a development baseline only.
Before full site admission:

1. Independently verify the 2005 ramp/mainline lane graph, two geographic anchors,
   vertical assumptions and observation boundaries against original CAD/imagery.
2. Reconstruct and review accepted **and rejected** gap opportunities from lateral
   motion/footprints and development video; resolve the fixed-position label
   transitions. Set measurement uncertainty before fitting.
3. Establish a bounded arrival/queue model with supporting observations or
   defensible bounds. Existing downstream detectors alone do not close this.
4. Freeze usable A/B/C interaction blocks, buffers and regime support without
   inspecting final-period outcomes. Keep claims restricted to historical blocks
   unless stronger independent evidence exists.

If the demand/queue or gap evidence cannot be recovered, Sam/adviser must choose
an explicitly narrower historical/synthetic sensitivity claim or another
qualified case. Do not label assumed demand as measured, turn replay into driver
validation, or quietly replace the approved research question. Hudson's driver
acceptance remains a separate milestone under the
[driver contract](hudson-driver-research-contract.md).

The live master and Sam refresher were read on 22 October; its latest progress
entry remains the 20 October repository confirmation. New evidence here is local
and artifact-backed. Shared Google Doc writes remain on hold.
