# I-110 MLK–Vernon: first source-based road build

27 October 2026 · Sam-selected site and authorized construction.

Sam selected I-110 near USC and requested starting the MLK–Vernon road, separate
bottleneck layer and a PeMS walkthrough that does not hold up development. This
supersedes the earlier recommendation-only geographic status. It does not admit
an empirical study, establish team-wide adoption or authorize new training.

## Open the result

From the research workspace, double-click **Open I110 USC Twin.cmd**. The portable
repo launcher is `scripts/open_i110_twin.cmd`; it opens the existing configured
Isaac runtime and `outputs/i110-mlk-vernon-v01/scene-v02/world.usda`.

The viewer starts paused with **Off** selected for bottlenecks. Choose
Construction, Disabled vehicle or Police traffic break, then Play or scrub the
timeline. Events activate at 10 seconds and clear at 65 seconds. Follow looks
ahead along the road; Overview and Free camera remain available. The probe car
is a scripted kinematic inspection aid, not a trained or physically simulated
car. Its path changes with the selected event and resets deterministically.

## Geometry delivered and its limits

- **814.53 m mainline**, from the mapped MLK crossing reference at
  34.0109392, -118.2812851 to Vernon at 34.0035972, -118.2812689.
- **299.97 m MLK entrance** and **278.03 m Vernon exit** source-aligned ribbons.
- Separate measured terrain and bridge-deck context, including the elevated
  central facility and three intermediate/downstream transverse deck patches.
- Local metre coordinates, Z up, UTM grid east/north using EPSG:6340. Origin
  (381687.546870016, 3764108.929151129); source Z offset 44 m, with no claimed
  vertical-datum transformation. Route headings account for projected geometry.

Acquired 123 EPT nodes, 4,485,059 source points before clipping; retained
**2,460,015 points** in the bounded 170-by-985 m acquisition area. Included
1,334,404 ground and 309,642 bridge-deck points. The selector allowed depth 16;
the intersecting source nodes reached depth 12. Source: public USGS
`CA_LosAngeles_1_B23`, project acquisition era 2023–24. Current road epoch and
independent survey accuracy remain unqualified. A sampled EPT LAS header's GPS
time flag is inconsistent with treating its large time values as week-seconds;
individual flight dates were not inferred from that header.

Road elevation/crossfall use robust local fits to class-2 points, excluding
nearby raised surfaces. Deck class 17 is reconstructed separately; the road is
not draped over an upper bridge or flattened onto a slab. Road meshes contain
3,264 / 1,200 / 1,116 triangles for mainline/entrance/exit respectively. Separate
support collision meshes use the fitted coordinates; context terrain/decks are
display-only. The viewer does not step PhysX, so contact/driving acceptance is
still pending.

**Horizontal road envelopes remain provisional.** OSM supplies alignments and
topology; widths and the five-to-four-lane taper use explicit nominal assumptions.
No surveyed lane boundary or painted-marking accuracy is claimed. Markings have
not been invented as if measured. The optional debug centerline is hidden by
default. Independent current imagery/as-built checks must refine the envelopes.

Occluded pavement is flagged vertex by vertex in `geometry.json`. Mainline has
207 interpolated vertices out of 1,911; entrance 1/707; exit 53/658. The original
35 m gap ceiling rejected edge spans; the declared preliminary ceiling is now
60 m, with no endpoint extrapolation. This is a construction assumption, not
verified geometry beneath an obstruction. Both support counts and interpolation
masks survive in the geometry package. Mainline maximum fitted centerline grade
is about 6.09%; this is a source-derived model diagnostic, not an independent
accuracy certificate. The exit's approximately 9.16% peak deserves review before
physical driving. Ramp overlap/contact transitions also need native validation.

## Implemented separate event layer

Composition (strongest first): session overrides, `motion.usda`, `events.usda`,
`layout.usda`, `physics.usda`, `environment.usda`. Cones are reusable referenced
components. Incident vehicles reference the existing local car asset. Permanent
roads and source files are not edited by event changes or playback.

`traffic/highway_events.py` validates schedules and produces deterministic,
spatially localized restrictions with explicit synthetic provenance. Overlapping
events keep distinct identities; an event ending does not clear another event.
`usd/highway_event_layer.py` consumes those states and updates visibility, pose
and collision-enabled attributes in the session layer. Off disables all event
colliders as well as visuals. A police-break actor follows the grade with a
bounded lateral sweep; a disabled actor occupies the right-side inspection path;
a construction preset supplies a cone taper and work-zone restriction.

The accompanying single-car preview consumes current restrictions, stops ahead
of stationary events and follows a moving break. It is a declared kinematic
demonstration with an analytic speed bound and no trained policy. Lane names
(`sb_right`, etc.) are logical preview identifiers, not validated SUMO lane IDs.
This implementation does **not** yet insert events into a live calibrated SUMO
network or validate moving colliders in PhysX. Full traffic/physical adapters
remain next-stage integration work; no general crash dynamics are simulated.

## Verification and evidence

Local package: `outputs/i110-mlk-vernon-v01/`. Raw source receipts, source points,
geometry/interpolation masks, two scene versions, original inspection images,
tests and native runtime records are retained. Test and artifact checks are
summarized in the [compact evidence](evidence/i110-mlk-vernon-build-20261027.json).
The initial native capture revealed that the old fixed-direction follow camera
looked away from forward events; the revised camera follows route orientation.
The first capture remains historical evidence.

The existing supplied car's redistribution rights remain unqualified. Source,
tests and documentation are separate from generated scenes, screenshots, raw
lidar and private vehicle assets; those outputs remain local. No physical
vehicle validation, policy training, traffic calibration or empirical admission
is established by a rendered scene or passing unit tests.

## Rebuild

Use the existing isolated geo environment for acquisition/preparation and the
existing CPU OpenUSD environment for scene authoring. Do not install geo or
traffic packages into Isaac's Python.
The tested optional geospatial versions are pinned in
[requirements-geo.txt](../requirements-geo.txt); no new packages were installed
during this build.

```powershell
runtimes/geo-env/Scripts/python.exe scripts/acquire_i110_lidar.py --output outputs/i110-mlk-vernon-v01/source
runtimes/geo-env/Scripts/python.exe scripts/prepare_i110_geometry.py --source outputs/i110-mlk-vernon-v01/source --roads outputs/la-corridor-review-20261027-a/road-extract.json --output outputs/i110-mlk-vernon-v01/geometry.json
```

Then, in a Python environment with NumPy and OpenUSD, from the repo root:

```text
python -m scripts.build_i110_twin --geometry outputs/i110-mlk-vernon-v01/geometry.json --vehicle vehicles/sim_ready/sam-collected/v01/vehicle.usda --output outputs/i110-mlk-vernon-v01/scene-new
```

The author rejects an existing output scene directory. Preserve source receipts
and choose a new scene version when changing geometry. The OSM extract and
private car are explicit local inputs, not guaranteed public clone assets.

## Next accuracy work

Verify current lane/pavement edges and taper lengths, reconcile datum/epoch and
the steep exit section, and obtain independent geometry checks. Validate native
tire contact, ramp seams and vehicle response before accepting physical driving.
Build a boundary-complete traffic network and connect the event restrictions
to its lane/obstacle semantics. Keep background demand explicitly synthetic until
qualified observations are available. The [PeMS walkthrough and offline-data
contract](pems-access-and-offline-workflow.md) lets that work proceed independently.
