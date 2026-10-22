# Highway twin site decision and build contract

22 October 2026 · Codex for Sam · Evidence reviewed through this date.

## Decision

**Select the exiD Aachen-Brand merging area on the A44 in Germany as the first
target for qualification and reconstruction. Reconstruct the traffic recording
epoch.** This is my recommendation under Sam's delegated site-selection request,
not a finding that the site already passes the research admission gate. Exact
recording IDs, direction, capture dates, footprint and boundaries remain open.
Do not invent those values from an interchange name.

The publisher advertises trajectories, OpenDRIVE/Lanelet2 maps, corresponding
FBX/OSGB environments and aerial backgrounds for exiD's seven sites. The observed
segments are approximately 420 m long; access requires a manually reviewed
application. We have acquired the public format description, not the restricted
traffic/maps/3D package. Asset availability and permitted conversion must be
confirmed in the grant. [Publisher and application](https://levelxdata.com/exid-dataset/).

The geographic identification combines two sources: an original research paper
names the Aachen Brand merging area in exiD, and the highway authority locates
Aachen-Brand on the A44. This establishes a candidate at interchange level, not
the precise recorded carriageway or an independently surveyed lane graph.
[Hou et al., 2025, available article excerpt](https://www.sciencedirect.com/science/article/abs/pii/S0001457525000521),
[Autobahn authority](https://www.autobahn.de/planen-bauen/projekt/a544-ersatzneubau-der-haarbachtalbruecke-bei-aachen).

Sam's latest request authorizes this broader comparison and selection, superseding
the earlier bounded I-80-first search order. It does not alter the approved study:
one open merge, uncertain human gap acceptance, matched local/communicating
policies, whole-traffic accounting, independent audit and bounded Isaac response
validation. No teammate assignment or team-wide architecture change is inferred.

## Why the present highway looks wrong

`scripts/build_i80_inspection.py` produces historical GIS linework and measured
track replay over a flat slab, with modern centerlines for context. It does not
reconstruct pavement surfaces, grade, shoulders, lane paint, barriers or bridge
decks. It is a useful inspection scene, but cannot support a realistic-road or
physics-ready-twin claim. Camera freedom fixes navigation, not this geometry gap.

Keep the existing viewer, supplied car references, stable identities, replay
layers, camera controls and complete-accounting fixtures. Replace the environment
through versioned assets. Preserve I-80 as a historical inspection/regression case.

## Candidate comparison

These are qualitative engineering judgments, not numerical rankings or claims
of exhaustive global coverage.

| Candidate | Evidence in its favor | Material unresolved issue | Decision |
|---|---|---|---|
| exiD / A44 Aachen-Brand | Merge-specific observations and a publisher-supported lane-map/3D route; public regional geography | Package access, exact epoch/direction, requested demand/queues, rejected gaps, independent partitions | Preferred reconstruction target; qualification first |
| I-80/Powell, 2005 | Original trajectories/support recovered; existing viewer and data checks | Modern terrain cannot establish 2005 pavement; full factual admission already fails current boundary/gap support | Retain earned engineering evidence; stop detailed reconstruction as the default |
| I-24 MOTION, Tennessee | Much longer corridor and repeated observations; attractive for traffic-wave work | Current complete ramp coverage and matching merge assets not verified here | Reserve candidate if it can satisfy the same merge contract |
| US-101 NGSIM, 2005 | Historical trajectory alternative already considered in research | Does not remove the historical geometry/boundary problem; weave complexity needs screening | No evidence-based advantage established over the preferred target |
| highD | Useful highway trajectories for other tasks | Dataset-introduction literature identifies missing merging scenarios | Not the primary merge case |

I-24's initial publication describes 6.75 km of coverage, but places trajectories
from additional ramp cameras in future work and documents tracking artifacts.
That is a limitation of the cited release, **not proof about every current
release**. Current GitHub documentation was retrieved; the current data website
failed TLS verification, so present ramp support remains unverified.
[Initial instrument paper](https://arxiv.org/html/2301.11198v2),
[current documentation](https://github.com/I24-MOTION/I24M_documentation).
The exiD introduction motivates its merge coverage against highD:
[Moers et al., 2022](https://ieeexplore.ieee.org/document/9827305).
See the existing [I-80 qualification report](i80-powell-qualification-2026-10-22.md)
for original-source checks and the reasons it is not admitted.

## Source stack and date policy

| Layer | Proposed source and use | What is verified now / what remains |
|---|---|---|
| Lane graph and traffic | Licensed exiD release, same site and epoch | Public format PDF read; actual files and calendar dates not acquired |
| Road visual starting asset | Corresponding licensed exiD 3D environment | Advertised; topology, elevations, textures, conversion and collision suitability untested |
| Terrain and surface measurements | Geobasis NRW elevation/laser products | Public catalog located; exact site tiles, capture dates, classes and accuracy still to qualify |
| Surrounding appearance | Geobasis NRW orthophotos and textured integrated mesh | SceneServer metadata successfully downloaded/decoded; no Aachen mesh tile imported |
| Historical change check | Dated orthophotos and capture-era source imagery | Needed before modern surroundings enter the historical scene |
| Google Maps | Human geographic orientation or separately licensed runtime visualization | No Google-derived geometry, textures or colliders extracted |

The actual NRW endpoint reports an I3S 1.7 IntegratedMesh, horizontal EPSG:25832,
vertical EPSG:7837, gravity-related heights in metres and Data licence Germany
Zero. This is a concrete Esri-compatible source, not a requirement to buy a
particular GIS license. Its photogrammetric appearance is not a validated road
contact or sensor model. Use a site subset; regional SLPK entries include files
around 100 GB, so whole-region download is inappropriate for the first milestone.
[Service](https://www.gis.nrw.de/geobasis/3D_mesh/SceneServer?f=pjson),
[download index](https://www.opengeodata.nrw.de/produkte/geobasis/lusat/akt/3dmesh/3dmesh/),
[geographic data catalog](https://www.opengeodata.nrw.de/produkte/geobasis/).

Freeze one recording-era roadway. A newer surrounding mesh is permitted in the
proposed workflow only after checking changes, removing transient objects and
marking its different date. Do not drape historical cars on present-day lanes.
Missing elevation cannot be disguised with a flat road and called measured.
Publication year and service update time are not acquisition dates. The public
exiD recording schema includes weekday/start time, but those fields alone do not
establish an exact calendar date; request it explicitly.
[Public format description](https://levelxdata.com/wp-content/uploads/2023/10/exiD-Format.pdf).

Google Map Tiles policy limits content to visualization and restricts extraction
and offline use. It is not our offline USD source. Prefer independently licensed
survey/map inputs for geometry and sensors.
[Google policy](https://developers.google.com/maps/documentation/tile/policies?hl=en).

## Work packages, dependencies and acceptance

| Order | Work and concrete output | Acceptance before dependent work |
|---|---|---|
| 1. Access and site freeze | Obtain licensed package; identify Aachen-Brand direction, location/recording IDs, dates, map version, source hashes and allowed uses | Package includes usable matching assets; licenses cover intended local research/conversion; exact footprint documented |
| 2. Factual qualification | Map observed entry/exit lines and queue visibility; inventory independent recording blocks/regimes; reserve A/B/C before fitting | Physical merge opportunities and accepted/rejected decisions observable or defensibly bounded; common requested-demand cohort and external waiting supported; no random-timestep holdout |
| 3. Geographic foundation | One local coordinate transform; lane graph, pavement ribbons, grade/crossfall where supported, shoulders, taper, barriers and bridge decks; source/error report | Independent control points and cross sections validate horizontal/vertical alignment, width, lane count, connectivity and taper; unresolved features visibly classified |
| 4. Existing-viewer integration | Layered USD environment, reusable vehicles, recorded replay, free/follow/overview cameras, source/date/debug overlays | Overlay against capture-era evidence; wheel/road contact and motion/lifecycle checks; no asset resolution errors or saved-source mutation; actual render inspected |
| 5. Traffic baseline and accounting | Import lane graph into SUMO, check converted connections/routes; extend existing requested/waiting/active/completed/removed ledger | Counts conserved through fixed horizon and separate drain; removals/censoring never become successful arrivals; human and ramp outcomes retained |
| 6. Human model and controls | Fit on A; admit/develop on B; choose three supported settings of one fixed gap model; competitive human/local/communicating scripts | Freeze observable tolerances before admission; no invented model ambiguity; no C access for tuning |
| 7. Driver/Isaac response | Frozen controller and explicit command adapter; held-out dynamics/following/merge response where needed; response surrogate for paired traffic reruns | Source/assumed vehicle parameters identified; single pose owner; supported speed/curvature/braking envelope and held-out error limits; replay does not count as physical validation |
| 8. Communication study | Equal local/communicating shared-PPO training family and centralized-critic opportunity; explicit messages; fixed fleet share, dispersed/clustered arrivals | Protocol, budget, training involvement and independent seeds agreed first; freeze safety logic, action authority and data privilege |
| 9. Final evidence | Untouched factual audit, paired policy matrix, subgroup/terminal guardrails, uncertainty and independent regeneration | Report all failed/missing runs; claim only supported single-site/model/envelope results; rights-safe release |

These are dependency gates, not nine completed tasks. Initial reconstruction may
proceed on admitted development geometry; expensive polishing and confirmatory
training wait for factual qualification. The full approved claim requires every
relevant gate. One short attractive replay cannot close them.

### Geometry and coordinate contract

- Use the supplied trajectory/map origin and CRS, not the recording's approximate
  latitude/longitude as an exact origin. Retain projected double precision until
  translating to a local metre, Z-up USD frame. Record axis, yaw and anchor rules.
- Verify vertical references separately; EPSG:25832 does not determine heights.
  Use the applicable geoid/datum transform and record grids/versions. Do not add a
  guessed vertical offset to conceal inconsistent height systems.
- Distinguish trajectory centres from the current replay's front-bumper anchor;
  apply vehicle dimensions and orientation explicitly. Round-trip and hand-check
  poses, lane membership and start/end timestamps.
- Validate against points/cross sections withheld from alignment, distributed
  across the ramp, mainline and elevation changes. Report errors in metres and
  topology mismatches, not only pictures. Two anchors alone are insufficient for
  an independent road-shape validation. Choose tolerances from source uncertainty
  and lane/contact/sensor needs before evaluating the withheld checks.
- Keep road topology, road contact meshes, appearance meshes and semantic/sensor
  labels separate. A ground-only terrain model can omit bridges; a photographic
  mesh can contain cars, vegetation and holes. Neither is automatically a collider.
- OpenDRIVE-to-SUMO and FBX/OSGB-to-USD are conversion work to verify, not already
  working integrations. Test one small licensed asset first. I3S is not assumed
  to import directly into Isaac; validate a permitted subset conversion or retain
  it in the GIS reference viewer and reconstruct the necessary surface separately.

Use `Environment`, `Vehicles`, `Lighting`, `Cameras`, `Physics` and `Debug` per
[scene conventions](../usd/SCENE_STRUCTURE.md). Keep heavy context selectively
loadable; collision/sensor-critical surfaces must remain loaded. Mark geometry as
measured, inferred or visual-only in the source ledger and inspection overlays.
Photorealistic textures establish neither road friction nor LiDAR reflectivity.

### Research and Hudson integration

Hudson's forwarded report describes 42 simulator-state observations, assisted
target-speed braking and two actuator actions; it does not establish a LiDAR-based
accepted full-scene policy. Following and merging still need validation.
[Driver contract](hudson-driver-research-contract.md).

His base-driver work is compatible with this geographic foundation. Geography
and accounting can progress while he qualifies the driver. For communication
comparisons, hold the base driver, sensors, safety rules, demand and training
opportunity fixed so the effect of messages can be interpreted. Germany-specific
driving rules/behavior and the recorded speed envelope must be modeled; this case
cannot establish California behavior or general real-world V2V benefit.

The approved research continues to use SUMO for traffic and Isaac for a bounded
physical response check feeding back into traffic. A fully physical trained fleet
or a perception-trained policy is a separate expansion requiring capacity and
scientific scope decisions. Do not accidentally make those prerequisites for the
first geography/replay milestone.

The remaining scientific risk is field of view. Vehicles entering a recording
are observed entries, not necessarily requested departures. A queue beyond the
image is not zero. Extending a visual road does not extend traffic evidence.
Annotate rejected as well as accepted opportunities with their observation
windows and uncertainty; accepted-gap distributions alone do not identify a
decision model. A published exiD merging-analysis implementation is a useful
method reference, not validation of our extraction:
[Li et al. code and method description](https://github.com/iyangli/A-reproducible-approach-to-merging-behavior-analysis-based-on-High-Definition-Map).

If access, geometry or boundary/partition support fails, report the failed check
and assess the reserve candidate against the same contract. Do not silently
replace the study with synthetic demand, another question or a cosmetic twin.
Any narrower empirical claim needs an explicit scope amendment. P01–P05 and P07
remain open; selecting a preferred interchange does not close them.

## Resources and immediate handoff

Needed: individual dataset access for each user as required by the grant; exact
road/traffic editions and rights; GIS/coordinate tooling; a verified asset-to-USD
conversion path; existing SUMO and isolated Isaac runtimes; adequate storage for
a clipped site; CPU preprocessing time and a bounded GPU window; an independent
geometry/traffic reviewer; measured training capacity before campaign sizing.
ArcGIS entitlement, CityEngine availability, storage requirements and people’s
availability are not established. No purchase or new person assignment is made.

The next concrete task is obtaining and qualifying the exiD package. A local
[access-request draft](exid-access-request-draft.md) is ready for Sam's university
details. It has not been sent. Request only the development package/metadata
needed initially; define and protect an untouched factual audit partition before
bulk analytical inspection. No new training has been launched.

## This session's evidence and limits

Local evidence directory: `outputs/i80-geospatial-rebuild-20261022-a/`.
`selection-sources/retrieval.json` records URLs, retrieval outcomes and SHA-256;
`manifest.json` inventories this session. No restricted exiD input was acquired.
The public NRW service response was gzip-encoded and decoded separately with the
raw response retained. The NRW guide timed out; I-24's data website failed TLS.

Before Sam broadened the site decision, a bounded I-80 geospatial trial recovered
4,024,202 clipped LiDAR points from 155 EPT nodes in the Alameda QL1 collection.
The initial alternative collection yielded no points in the clip; a deeper query
hit its node budget; the NAIP metadata/export request succeeded but image fetch
returned HTTP 502. These failures are retained. Source elevations/classes include
noise and have not been accepted as a road surface. No new road mesh, validated
height field, imagery texture or realistic twin was built from that trial.

This milestone changes documentation and the recommended next target. It does
not change simulator behavior, close admission gates, prove new rendering or
controller performance, or replace earlier measured test results. The shared
master/Sam refresher were read this session; shared-document writes remain on hold.
