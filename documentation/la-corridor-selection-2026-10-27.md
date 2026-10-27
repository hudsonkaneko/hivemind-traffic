# LA corridor comparison and first reconstruction segment

27 October 2026 · Codex for Sam · Development recommendation, not empirical admission.

## Decision

Build **I-110 southbound general-purpose lanes from the Exposition Boulevard /
37th Street crossing to the Vernon Avenue crossing**, approximately one mile,
first. Within that footprint, prioritize the **Martin Luther King Jr. Boulevard
southbound entrance, Vernon exit, and auxiliary-lane reduction**. This replaces
the earlier unspecified I-110 search area with a concrete reconstruction target.
Sam requested the comparison; this exact selection is Codex's recommendation,
not a recorded team decision or proof of adequate calibration data.

Use these retrieved OpenStreetMap anchors to locate the development footprint:

| Feature | Latitude, longitude (WGS84) | Source identity / interpretation |
|---|---|---|
| Northern crossing reference | 34.0180716, -118.2802731 | Junction of mapped southbound ways 1010971476 / 148249762, Exposition area |
| MLK entrance joins mainline | 34.0083998, -118.2813432 | Link 399277061 joins ways 399276942 / 399276944 |
| Vernon exit diverges | 34.0067173, -118.2812409 | Link 148385755, signed destination Vernon Avenue |
| Mapped five-to-four lane transition | 34.0041593, -118.2812662 | Ways 1119770688 / 1119770689; outer lane tagged merge_to_left |
| Southern crossing reference | 34.0035972, -118.2812689 | Ways 1119770689 / 148252622, Vernon area |

These are map anchors, **not surveyed gore tips, lane boundaries or final mesh
cut lines**. A current imagery/as-built check must resolve exact crossing edges,
tapers, lane counts, widths, shoulders, barriers and access rules. OSM's numeric
precision does not establish that accuracy. Retain source IDs and map date.

The first detailed tile should cover MLK through the Vernon lane reduction;
the full mile adds useful upstream context and recognizable USC surroundings.
Do not make the render footprint the traffic boundary. Include full ramp queues
and the Vernon entrance downstream in a larger traffic domain, with upstream and
downstream boundaries moved beyond observed spillback. The historical Vernon
bottleneck's average reported extent is 3.30 miles, already larger than this tile.
No boundary-complete calibration domain has been established today.

## Comparison using inspected sources

All three areas have retrieved public road-map extracts and actual USGS lidar
points. No corridor has a verified PeMS station-health or contemporaneous demand
advantage yet. Sam confirmed **no PeMS account or District 7 data** on 27 October.

| Candidate | Congestion lead | Geometry/data inspected | First-build assessment |
|---|---|---|---|
| I-110 south, Exposition–MLK–Vernon | Vernon ranked 9th, PM, in the report's 2024 table | Mapped entrance/exit/lane reduction; lidar includes multiple road elevations; five Caltrans camera listings around Vernon, MLK and Exposition | **First choice for combined access, observable features and USC relevance.** Elevated center Transitway and crossing bridges need separate surfaces; not a flat slab. |
| I-10 west, Robertson / National | Robertson ranked 7th, AM | Lidar, mapped westbound links, official ramp inventory; Robertson Place westbound entrance has a 2021 count, not 2024 | **First fallback.** Useful merge/exit geometry; no demonstrated detector or lane-map accuracy superiority. |
| I-405 south, Howard Hughes toward La Tijera | Howard Hughes ranked 2nd, PM | Lidar, mapped southbound entrance/exit, official ramp inventory; southbound entrance latest listed count is 2018 | Strong congestion candidate; **later expansion** because severity does not establish observability or easier calibration. No evidence that its lidar is worse. |
| I-110 north, Adams terminus | Adams ranked 4th, AM | Official HOV plan identifies signal-constrained HOT terminus | Defer as first target: model signal/HOT interactions explicitly before attributing its queue to a local merge. |

Ranking is an engineering judgment, not a weighted quantitative score. It favors
a bounded useful build near USC. If healthy detector coverage and suitable
independent observations prove materially better on I-10 or I-405, change the
empirical study site before expensive detailing. More congestion can increase
interaction opportunities but does not guarantee an AV improvement; test onset,
recovery and lower-demand conditions as well as sustained congestion.

Congestion source: [Caltrans District 7 Q2 2025 report, 2024 bottleneck table](https://dot.ca.gov/-/media/dot-media/programs/traffic-operations/documents/mpr/2025/2025--d7-q2-a11y.pdf).
This is not a 2026 ranking. [Caltrans's 2026 HOV plan](https://dot.ca.gov/-/media/dot-media/programs/traffic-operations/documents/hov/2026-ca-hov-facilities-degradation-action-plan-a11y.pdf),
printed page 44, supports the Adams caveat; do not extend its HOT diagnosis to
all general-purpose lanes.

## What the acquisition actually established

Public source receipts, hashes, extraction scripts and inspected plots are in
`outputs/la-corridor-review-20261027-a/` (local, ignored evidence). No simulator
or training process ran. The initial oversized I-405 map request failed HTTP 400;
a smaller request succeeded. An initial I-405 lidar neighborhood sample was
off the highway; map inspection relocated it onto the carriageway. Both source
selection records are preserved; only the corrected samples below support this
comparison. The full lidar report exceeded the 18 MB per-source bound and was
not retained/read. No claim relies on reading that full report.

A portable [compact evidence record](evidence/la-corridor-review-20261027.json)
retains source hashes, sample counts, pinned repository identities and limitations.

### Geometry

[USGS/NOAA LA 2023–24 catalog](https://www.fisheries.noaa.gov/inport/item/80274/full-list)
links the public `CA_LosAngeles_1_B23` EPT collection. Retrieved EPT metadata says
EPSG:3857 horizontally. The source project identifies NAVD88, but the EPT SRS
does not encode a vertical CRS: reconcile original LAS/tile metadata and datum
before meshing. Source acquisition era is not a guarantee of the road's present
configuration. Selected-tile acquisition times remain to be established.

The corrected reconnaissance read 34 EPT nodes through depth 10 (872,011 points
before clipping), retaining 79,373 points across three 200-by-200 Web Mercator
metre boxes. At this latitude each is about 166 m across on the ground.

| Sample center (lon, lat) | All retained points | Class 2 ground | Class 17 bridge deck |
|---|---:|---:|---:|
| I-110 (-118.2812700, 34.0057000) | 28,227 | 8,747 | 1,852 |
| I-10 (-118.3904659, 34.0312661) | 23,710 | 8,347 | 2,794 |
| I-405 (-118.3868080, 33.9762752) | 27,436 | 8,734 | 420 |

These are depth-limited samples, not full-resolution point density, complete
corridor coverage, lane extraction or independent accuracy tests. The inspected
plot shows road/context surfaces, with I-110's elevated central facility clearly
distinct from adjoining lower surfaces. Point counts do not rank source quality.
Class 17 alone does not identify which named deck a point belongs to. Occluded
pavement underneath bridges still needs another source or declared uncertainty.

**Use classified point clouds for deck/road reconstruction and the bare-earth
DEM for surrounding terrain.** USGS explicitly removes bridges from the
[bare-earth DEM](https://www.usgs.gov/ngp-standards-and-specifications/lidar-base-specification-digital-elevation-model-surface).
Do not drape all highway lanes onto that DEM. Reconstruct each carriageway,
Transitway, bridge and ramp surface independently; preserve height separation,
grade and crossfall. Validate a smooth collision surface separately from the
render mesh. Local lane-marking imagery/as-built rights and independent surveyed
checks remain unresolved. USC affiliation does not establish a LARIAC license.

Road topology comes from bounded [OpenStreetMap API](https://wiki.openstreetmap.org/wiki/API_v0.6)
extracts, not aerial image scraping. Attribution: © OpenStreetMap contributors,
[ODbL](https://www.openstreetmap.org/copyright). It is a topology seed, not an HD
lane map; original edit timestamps and versions are retained in the extracts.

### Traffic and observation limits

Read the official [2024 District 7 ramp workbook](https://dot.ca.gov/-/media/dot-media/programs/traffic-operations/documents/census/2024/2024-d7-ramp-adt-a11y.xlsx)
at worksheet XML `sheet2`, named fields `LOCATION_DESC`, `POSTMILE`, `YR_2015`
through `YR_2024`. Relevant observations:

| Ramp | Workbook row | Latest nonblank listed year/value | Consequence |
|---|---:|---|---|
| I-110 SB on from MLK | 1711 | 2020 / 9,399 | Not present-day time-varying demand |
| I-110 SB off to Vernon | 1709 | 2024 / 3,541 | Annual ramp volume, not peak-period queue evidence |
| I-110 SB on from Vernon | 1706 | 2018 / 11,276 | Downstream demand source is stale |
| I-10 WB on from Robertson Place | 471 | 2021 / 13,907 | Missing 2024 entry is not zero flow |
| I-405 SB on from Howard Hughes | 2363 | 2018 / 9,415 | Missing later entries; no current advantage established |

These are annual ramp ADT inventory values, not hourly injections or directly
comparable same-day demand. Preserve postmile prefixes and distinguish Caltrans
county postmiles from PeMS absolute postmiles. Blank entries mean missing data.

[Caltrans camera directory](https://cwwp2.dot.ca.gov/vm/streamlist.htm) lists
819 Vernon entrance, 177 MLK, 820 MLK entrance, 178 Exposition and 822 south of
Exposition. Listing is not verified uptime, useful camera angle, calibration,
permission to record, or historical video: Caltrans says it does not retain or
archive this footage. No camera images were downloaded. Do not treat camera
counts as equivalent to lane-resolved detector coverage or complete trajectories.

Next acquire [PeMS access](https://dot.ca.gov/programs/traffic-operations/mpr/pems-source)
and District 7 station metadata, lane configuration/health, observed-versus-imputed
fractions, and matched mainline/ramp speed/flow/occupancy periods. Obtain upstream
requested demand, queues and downstream supply; identify event-free and event
periods before fitting. Detector aggregates alone do not identify human merging
gap acceptance. Prospective observations or other licensed trajectories are needed.

Following [FHWA calibration guidance](https://ops.fhwa.dot.gov/publications/fhwahop18036/chapter5.htm),
fit representative conditions and time-dependent congestion, and compare held-out
speed, flow, travel time, queue formation/recovery and lane/ramp behavior. Fix
local acceptance tolerances and independent geometry anchors before tuning; do
not invent centimetre accuracy from project-wide lidar specifications. Preserve
independent A/B/C observation groups and training-seed uncertainty. P01–P05/P07
remain open. No factual site or held-out audit set is admitted/reserved today.

## Separate disruptions layer

Sam explicitly requests crashes, construction and police zigzags as a separate
twin layer. The architecture is now specified in
[the event-layer contract](highway-disruption-layer.md), with a blank
[event intake](templates/highway-event-intake.json). The police behavior is a
CHP traffic break. This is a design specification; no event runner, USD incident
layer, LA road scene or trained event response has been implemented by this review.

## Immediate build and acceptance order

1. Acquire complete road/deck point coverage for the selected mile; validate
   imagery/as-built epoch and rights. Derive a lane graph, measured tapers, grade,
   crossfall, shoulders and barriers. Start the focal MLK–Vernon tile first.
2. Check independent road anchors, bridge clearances and tire contact. Use the
   existing viewer/free camera and source identity/accounting conventions.
   A scripted drive can isolate road/contact errors from learned-driver errors.
3. Expand and calibrate the traffic boundary using observed queues and healthy
   detectors. Model regular human traffic with calibrated heterogeneous behavior;
   it does not need every vehicle to have an RL policy.
4. Accept a frozen ordinary-driving controller before testing incidents or
   communication benefits. Current repository evidence is summarized in the
   [fresh driving audit](driving-readiness-2026-10-27.md).
5. Activate reproducible incident scenarios separately, preserving the normal
   traffic baseline and complete human/ramp/external/unfinished accounting.

This sequence is authorized research/design work and a build recommendation,
not an unattended training launch or a replacement for the approved bounded
SUMO/Isaac study. Shared-master updates remain local drafts under Sam's hold.
