# I-110 MLK–Vernon geometry and tire-contact refinement

27 October 2026. Codex for Sam. Sam explicitly authorized imagery refinement
and bounded tire-contact testing, and reports PeMS approval. No credentials,
application submission, policy training or shared Google Doc write occurred.

## What changed

The first build's OSM centre nodes were insufficient descriptions of physical
ramp joins. Added a reviewable, manually interpreted pavement trace from dated
NAIP imagery, reconstructed semantic lane boundaries/tapers, a robust lidar
cross-section fit and one continuous support mesh. The visual road now uses
that same mesh with a 15 mm display-only offset. There are no separate,
overlapping mainline/ramp colliders. Event opinions remain in their separate
layer and default Off.

The active viewer is `outputs/i110-refinement-20261027/scene-v07/world.usda`.
Use `scripts/open_i110_twin.cmd` or the existing workspace I110 launcher.
The earlier scene and all failed development attempts are preserved.

## Evidence and accuracy boundaries

- Source: [USDA FPAC-BC GEO NAIP image service](https://apps.geo.fpac.usda.gov/geo-imagery/rest/services/naip/conus_naip/ImageServer),
  exact names `m_3411862_se_11_060_20240506` and
  `m_3311806_ne_11_060_20240506`, acquired 6 May 2024, native 0.6 m pixels.
  Exported 0.5 m pixels do not increase native resolution. Source record IDs,
  request, extent and image hash are in the local imagery receipt. USDA asks
  for credit and describes the service as illustrative, not suitable for
  surveying/engineering. These are simulation-development interpretations.
- The 11 May 2022 USGS NAIP image was inspected as an additional dated view.
  Elevated expressway displacement/occlusion differs between images; the deck
  edge must not be mistaken for a ground-level freeway lane boundary.
- [Image interpretation](../scenarios/i110/mlk-vernon-image-trace.json) stores
  explicit pavement edges, hidden bridge spans, approximate divider spacing
  and auxiliary-lane taper assumptions. Mainline, entrance and exit lengths
  are approximately 815.1, 317.9 and 316.1 m. Shoulders and lanes are distinct.
  Individual painted stripes partly hidden by traffic are reconstructed, not
  claimed as independently measured. The roughly 1 m tracing sensitivity
  assumption is **not** a measured absolute error bound.
- Lidar is the previously acquired USGS CA_LosAngeles_1_B23 package, with
  2,460,015 clipped points. EPSG:6340 local origin is
  (381687.546870016, 3764108.929151129); Z retains source values minus 44 m.
  No new vertical-datum conversion or current-epoch accuracy claim is made.
- Local cross-section planes retain grade/crossfall, with bounded interpolation
  across missing spans and 9-station smoothing. Final maximum smoothing changes
  are 0.0105 m mainline, 0.0666 m entrance and 0.0513 m exit. These fitting
  diagnostics are not independent accuracy measurements.
- The collision mesh has 92,438 unique XY vertices and 179,490 upward triangles
  on a conservative 0.5 m grid. The original ribbon overlap discrepancy was at
  most 0.0646 m; overlapping samples are reconciled into a single shared Z.
  The resulting surface represents smooth pavement, not measured tire-scale
  roughness. Maximum triangle slope is approximately 14.62%; maximum centreline
  grades are about 5.86% mainline, 6.12% entrance and 8.93% exit.

Local figures: `refined-envelope-overlay.png`, `road-profiles.png` under the
refinement output directory. They were visually inspected; they are not an
independent survey audit. The annotated geometry still needs independent review
and higher-resolution/as-built evidence before a precise site-fidelity claim.

## Physical test contract

`scripts/probe_i110_contact.py` imports the existing full-size PhysX Factory
surrogate car: 1,800 kg, 3.2 m wheelbase, 1.6 m track, 0.35 m tire radius,
raycast suspension and the existing tire/torque configuration. It is not the
supplied visual car's measured dynamics and is not a deformable-tire model.

PhysX is the sole pose authority after spawn. A privileged-map, scripted
pure-pursuit controller writes steering and wheel torques, not poses/velocities.
Physics runs at 120 Hz, independently of rendering/SUMO. Each case has a fresh
composed fixture. The factory infinite ground plane is inactive. The exact road
support asset is referenced and assigned to the existing tarmac query/filter
resources. Tire hit collider identities, jounce, hit positions, poses, velocities
and chassis contact reports are retained at every tick.

The matrix includes a no-road negative control, all four through lanes with
60 mph targets, MLK/mainline and mainline/Vernon routes with 30 mph targets,
and five stationary seam/bridge checks. Ramps are not accepted for 60 mph by
these tests. Pass criteria: at least 99.5% all-four-wheel support after settling,
only the intended road as tire-hit source, upright Z >= 0.95, route completion
and no chassis contact reports. The no-road case must fall with no tire hits.
Checks test smooth-road support and bounded traversal; they do not validate
human following/merging, a learned driver, emergency manoeuvres or traffic flow.

Final outcomes and exact run references are recorded in
[the compact evidence](evidence/i110-refinement-20261027.json). Raw configuration,
traces, lifecycle logs, source hashes and supervisor outcomes remain local.
The contact matrix uses scene-v04's support asset. Scene-v07 changes only the
rendered surface/markings; the support file is byte-identical and verified.

## Preserved failures and corrections

1. The initial expanded ribbon fit refused an unsupported shoulder endpoint.
   Replaced independent edge fits with robust cross-section planes; edge
   extrapolation within the road width is now an explicit modelling assumption.
2. `geometry-v02.json` exposed an artificial exit-ramp height jump: classified
   returns near an overpass were incorrectly fitted as the lower road.
   Image-obscured spans were excluded and explicitly interpolated. The original
   ~3 m smoothing correction is retained in that rejected geometry; it is not
   silently reported as the accepted surface.
3. `contact-a` traversed its short route with all tires supported, but failed
   the chassis-contact gate. Native reports had >=0.287 m separation and zero
   impulse: the large mesh's automatically scaled speculative contact margin
   was inappropriate. Set road contact offset explicitly to 0.02 m, rest offset
   0. This corrects the configuration; it does not exempt real chassis contacts.
4. `contact-b` stopped on an incorrect test guard: it treated a 15 m descent
   from the initial road elevation as a fall. The saved last sample had four
   supported wheels and upright Z >0.999. The guard now compares with the
   local road elevation. That run is failed, despite the Isaac process returning
   exit 0. Acceptance always requires the result JSON, not just process exit.

## Rebuild and next boundary

Use isolated geo requirements for `scripts.acquire_i110_imagery` and
`scripts.refine_i110_geometry`; the latter takes previous geometry, clipped
lidar points and the committed annotation JSON. Use CPU OpenUSD for
`scripts.build_i110_twin` and `scripts.check_i110_refinement`. Use the configured
Isaac runtime only for the bounded physical probe/viewer. Do not install geo or
traffic packages into Isaac. Generated imagery, lidar, scenes and supplied cars
remain local; no asset redistribution rights are implied.

PeMS approval removes the account step, not the detector-quality gate. Next data
work is station metadata/health and a small, dated District 7 export. Sam signs
in directly or supplies exports; passwords never belong in chat, source, logs
or configuration. PeMS stays an optional offline calibration input. Independent
geometry review, ordinary-driver acceptance, traffic calibration and event
physics/traffic integration remain separate work. Preserve P01–P05/P07 and the
independent factual audit; this engineering milestone does not admit the site
for confirmatory research or authorize unattended training.
