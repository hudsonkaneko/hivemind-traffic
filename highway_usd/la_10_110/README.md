# I-10 / I-110 Interchange: OpenUSD training environment

This is a medium-fidelity 1:1 replica of the Santa Monica Fwy (I-10) / Harbor Fwy (I-110) interchange in downtown Los Angeles, built for autonomous-vehicle simulation in NVIDIA Isaac Sim.

The model focuses on the interchange itself. It covers about 1.4 × 1.4 km of freeway centred on the point where the two mainlines cross (34.038114, -118.274017):
- every connector and loop ramp
- the collector roads
- the approaches out to about 700 m

Low-fidelity city context extends to about 1.6 km.

## Open it
- **Layered (editable):** open `la_10_110.usda` in Isaac Sim (File > Open).
- **Single file:** `la_10_110_flat.usdc`. It has the same content flattened into one file, for drag and drop, referencing, or `add_reference_to_stage`.

Stage conventions: Z-up, metres (`metersPerUnit = 1`), defaultPrim `/World`.

The local frame is an ENU tangent plane: +X is east and +Y is north. Its origin is stored in `customLayerData` and in `/World` attributes (`geo:originLatitude` / `geo:originLongitude`).

## What's inside
| Prim | Contents |
|---|---|
| `/World/Freeway/<road>_<roadwayId>` | One Xform per roadway (an OSM way split at junctions). Attributes: `osm:*`, `roadway:predecessors/successors`, `lanes`, `deck:minZ/maxZ`. Children: `Deck` (one seamless strip of shoulders, lanes and paved median/gore), `Structure` (bridge sides/soffit), `Barrier` (Jersey and shared median barriers), `Piers` (columns and pier caps), `PaintWhite`, `PaintYellow`, `LightPoles`. |
| `/World/Streets/Tile_*` | Surface streets merged per 800 m tile. These are visual context, but they keep their true overpass heights. |
| `/World/Context` | `Ground` (with cut-outs where the freeway is depressed), `Buildings` (OSM footprints extruded to OSM heights), `Parks`. |
| `/World/LaneGraph/L_<way>_<k>` | One `BasisCurves` per lane, using `purpose = guide` so it isn't rendered. Attributes: `lane:successors`, `lane:predecessors`, `lane:speedLimit` (m/s), `lane:roadName`, `lane:index` (0 = leftmost), `lane:count`. |
| `/World/SpawnPoints/SP_*` | Poses at every lane that enters the network. +X is the travel direction, and each pose sits 0.5 m above the deck. |
| `/World/Cameras` | `TopDown`, `TopDown_WithLoops`, `Overview_FromSouth`, `Interchange_FromSE`, `Interchange_FromNW`, `Stack_LowAngle`. |
| `/World/Signs/Gantry_*` | Overhead sign gantries: `Structure`, plus `Panel_*` with a textured `Face` and `Back`, and an optional `TabFace` (exit number or SIM LOOP). These are defined in `signs.usda`, next to the root file. |
| `/World/DespawnPoints/DP_*` | Lane ends where vehicles leave the network (street off-ramps). |
| `/World/Looks` | UsdPreviewSurface materials, plus physics materials `PhysAsphalt` (μs 0.9 / μd 0.8) and `PhysConcrete`. |
| `/World/PhysicsScene` | Gravity of 9.81 m/s² along -Z. |

**Physics.** Decks, bridge structure, barriers, piers, street decks, ground and buildings are static triangle-mesh colliders (`PhysicsCollisionAPI` with `physics:approximation = "none"`).

**Semantics.** Every mesh carries both the `SemanticsLabelsAPI` (`semantics:labels:class`, used by Isaac Sim 5/6) and the legacy `SemanticsAPI` attributes. The classes are `road`, `bridge`, `barrier`, `pillar`, `lane_marking`, `ground`, `building` and `vegetation`. They work with Replicator segmentation and lidar ground truth.

`lanegraph.json` holds the same lane graph for code that doesn't use USD. `isaac/lanegraph_api.py` uses it for routing and waypoints.

## Simulation loops (an endless freeway)
This part deliberately departs from reality. Every freeway carriageway that leaves the modelled area is routed around the outside and fed back into one that enters it:

| Loop | From (leaves the model) | Into (enters the model) | Bridge level |
|---|---|---|---|
| NW | 110 North, north edge | I-10 East, west edge (mainline + express) | ~14 m |
| SE | 110 South, south edge (mainline + San Pedro C-D) | I-10 West, east edge | ~14 m |
| NE | I-10 East, east edge (mainline + express) | 110 South, north edge (mainline + C-D) | ~7 m |
| SW | I-10 West, west edge (mainline + express) | 110 North, south edge | ~7 m |

- **Geometry.**
  - Each loop is a 5-lane elevated ring road with 180 m radius curves, about 2.8–4.3 km long.
  - A loop is never wider than the carriageways at either of its ends, so its deck can't spread over the opposite carriageway at the map edge.
  - Where several carriageways leave or enter together, short lead roads merge them onto the loop or split them off it. Extra lanes drop or merge at that lead junction.
  - Each loop leaves the map edge on a 60 m straight. A 160 m S-curve then moves it 18 m away from the opposite loop before it turns. This keeps the decks apart and puts the one unavoidable crossing far enough out that both loops are already at full bridge height.
  - Wherever two loops cross, the NW and SE loops bridge over the NE and SW loops.
- **Traffic in and out.** Ramps to and from surface streets are not looped. The 10 street on-ramp entries are where vehicles enter, and the 9 off-ramp exits are where they leave. Everything else circulates.
- **Lane graph.**
  - 296 of 341 lanes form one strongly connected circulating network, counting lane changes.
  - Every other lane leads into that network or to an exit, and no lane is a dead end.
  - `tools/check_circulation.py` verifies this on every build.
- **Spawning.** `/World/SpawnPoints` has `spawn:type = entry` poses at the street on-ramps, and `loop` poses seeded on every lane of every loop, so an episode can start with full traffic. `/World/DespawnPoints` marks the off-ramp lane ends.
- **Loop data.** Loop prims are named `Sim_Loop_<corner>_*`. Their lanes carry `lane:simLoop = "NW"` and so on, and `lanegraph.json["loops"]` describes each loop.
- **Realistic version.** Build with `--no-loops` to get the open network without the loops.

## Overhead guide signs
Each sign reflects the real signed lanes, taken from OSM `destination:lanes`, `destination:ref:lanes`, `destination`, `destination:ref` and `junction:ref`.
- **Placement.** Every real split gets an overhead gantry about 150 m upstream. Each panel hangs over exactly the lanes that feed its branch, taken from the lane graph.
- **Panel contents.**
  - Interstate or California route shields with the cardinal direction.
  - The destinations. An option lane (one lane feeding both branches) only shows names that the other branch doesn't own.
  - An exit-number tab, from `junction:ref`.
  - A yellow EXIT ONLY plate when the branch's lanes aren't shared with the through road.
  - One down arrow per lane.
- **Example.** The NB 110 approach shows *CA 110 NORTH — Pasadena / Hollywood* over lanes 1–3. Over lanes 4–5 it shows *EXIT 21, I-10 WEST / I-10 EAST — Santa Monica / San Bernardino, EXIT ONLY*.
- **Loop signs.** The simulation loops carry their own panels with an orange **SIM LOOP** tab naming the freeway they feed.
- **Files.** Sign faces are PNG textures in `textures/signs/`, rendered by `tools/signs.py` with Pillow. The flattened `.usdc` keeps relative texture paths.
- **Ground-truth data.** Every panel carries its content as attributes: `sign:text`, `sign:refs`, `sign:destinations`, `sign:exit`, `sign:exitOnly` and `sign:lanes`. Panels are labelled `traffic_sign` and gantries `sign_structure`. All signs are also listed in `lanegraph.json["signs"]`.
- **Source correction.** One typo in the OSM source data ("Conversation Center") is corrected to Convention Center.

## Lane graph for agents
- Each lane has:
  - `succ` / `pred` for driving on, and `left` / `right` lane-change neighbours.
  - Its real signed `destination`, `destinationRef` and `turn` (from OSM `turn:lanes`).
  - `speed` and `simLoop`.
- `isaac/lanegraph_api.py`:
  - `route(start, goal= | target_road= | target_destination=)` finds a route using successors plus lane changes.
  - `waypoints(route)` returns the points to follow.
  - `loop_lanes(corner)` lists the lanes on a loop.
  - `signs_for_lane(lane, signs)` returns the sign panels over a lane.
- Running `lanegraph_api.py` prints a full closed lap of about 17 km through two loops and the interchange connectors.

## How the roadways are made seamless
`tools/roadway.py` builds every roadway lane-first:
- **Junctions are real junctions.** OSM ways are split wherever another freeway way joins, so every diverge, merge and lane drop is an end-to-start connection.
- **Branches sit on their parent's lanes.**
  - Branches are ordered left to right by where they actually are, not by which way they point.
  - At a split, each branch starts exactly on the parent lanes it takes over: the leftmost branch takes the left lanes, the rightmost takes the right lanes.
  - The branches stay side by side until their surveyed centrelines separate, then follow the surveyed alignment, with about 25 m of smoothing.
  - Merges work the same way in reverse. A merge offset also carries back through short feeder segments.
- **Option lanes.** Where one lane feeds both branches, the branches fork apart over a 30–90 m gore taper. Inside the shared, widening lane no paint is drawn, so the two edge lines meet at the gore tip as a V.
- **Shared surface height.** Side-by-side branches stay at one height while their decks touch, so there's no step. They only diverge vertically once they're physically apart, and a barrier then separates them.
- **Lanes are snapped.** Every successor lane starts where its predecessor ends (the build checks for gaps of 1 cm or less). Dropped lanes taper over 150 m.
- **One deck per roadway.** Each deck is a single strip: `shoulder | lanes | shoulder`. Shoulders taper to zero inside gores, so there are no flat slab ends or overlapping decks.
- **Medians and gores.**
  - At grade, gaps up to 12 m between roadways at the same height are paved, with one shared median barrier. Opposing carriageways always get a continuous median barrier.
  - Any height step against a neighbouring deck gets a Jersey barrier.
  - On structures, only gore noses (up to 2 m) are paved. Separate bridges keep their real air gap and get a Jersey barrier on each edge.
- **Barriers, piers and lights.** Barriers run along open edges, drop-offs and mainline edges, but never inside a gore. Piers stand clear of any road below. Light poles are spaced every 60 m along mainline outer barriers.
- **No intersecting geometry in traffic.** `roadway.build_clear` and `Roadway.intrudes` make sure nothing a vehicle could hit ends up in a lane:
  - **Full-width deck clearance.** The network enforces clearance at centreline crossings only. After that, every lane centre and deck edge is tested against every unrelated deck across its full width. Wherever two decks are less than 6.2 m apart vertically, the solver adds height bounds and solves again: the upper deck goes up and the lower one is capped. Side-by-side decks with a 0.15–0.5 m curb-like ledge between them are levelled the same way.
  - **Shoulder trimming.** Side-by-side decks at different heights have their shoulders pulled back, so neither deck slab reaches over or under the other's lanes. The edge then gets a barrier.
  - **Vehicle envelopes.** Each lane carries an envelope 2.4 m wide and up to 4.4 m above the road surface. Piers, pier cap beams, light poles, barriers, structure walls and sign gantries are left out wherever they would enter another roadway's envelope. A gantry is first moved along its road, and is dropped only if no position is clear.
  - **Buildings.** Any building whose footprint, plus a 4 m buffer, touches a freeway deck is removed.

## How elevations were made
OSM gives 2D centrelines plus relative stacking (`bridge=yes`, `layer=N`). `tools/network.py` turns that into real 3D profiles:
1. **Nominal heights.** Each bridge level sits 7 m above the one below it, so the 10/110 stack reaches about 14 m.
2. **Transition zones.** These are chain-aware. A structure starts climbing only where it actually begins from, or ends into, a lower road. It reaches full height after `Δlevel × 7 m / grade`, with a design grade of 5% on mainlines and 6.5% on ramps.
3. **Grade limits.** Free samples come from a grade-limited envelope (two multi-source Dijkstra passes), followed by vertical-curve smoothing.
4. **Clearance at crossings.** Every 2D crossing between roads on different layers is checked. If the deck-to-deck separation is under 6.4 m (about 4.9 m of vertical clearance under the structure), the solver adds a bound and solves again. Depressed sections (OSM `layer<0`) are lowered instead of the crossing road being raised.

Results for the current build (the build also prints these):
- 113 freeway crossings, minimum separation 6.38 m
- freeway decks from grade to +14 m
- mainline and ramp grades of 9% or less. The one exception is an 11.7% grade on a 3 m connector stub.

## Rebuild / customise
Run the build with any Python 3.9+, for example the one bundled with Isaac:
```
kit\python\python.exe projects\la_10_110\build_all.py            # uses cached data/*.json
kit\python\python.exe projects\la_10_110\build_all.py --refresh  # re-download OSM first
```
It writes `dist/la_10_110/`, `dist/la_10_110.zip`, and the top-down previews `dist/preview_*.png`. It takes about 30 s.

To fix a specific road, add `overrides.json` next to `build_all.py`, keyed by OSM way id. Values override OSM tags, and `drop` removes a way:
```json
{ "123561756": {"lanes": "5"}, "148356702": {"layer": "2", "bridge": "yes"}, "999": {"drop": true} }
```

## Checks
- `kit\python\python.exe tools\check_lanegraph.py` checks lane continuity, grade, turn radius and reachability. Current results:
  - 341 lanes, all reachable
  - every successor joins within 1 cm
- `tools/check_overlaps.py` runs automatically during the build and prints `QA:` lines. It counts deck-to-deck clearance violations (under 6.2 m), lane centres inside a foreign deck, and unprotected height steps between decks. Current results:
  - 0 clearance violations
  - 0 lane-centre overlaps
  - 1 unprotected 0.4 m step, on a shoulder about 0.6 m outside the nearest lane at the north map edge
  - Expected option-lane widening points are reported separately.
- `tools/check_obstacles.py` also runs during the build, on the finished USD stage. It sweeps every lane in `lanegraph.json` with a vehicle envelope 2.4 m wide, from 0.12 to 4.4 m above the road, against every triangle of every mesh: piers, barriers, poles, gantries, sign panels, structure, other decks (including ledges of 0.15 m or more), streets, ground and buildings. Current result: 0 hits.
- Run `python.bat la_10_110\isaac\validate_in_isaac.py` from the Isaac Sim folder. It drops a rigid body on every spawn point and checks that it rests on the deck.

## Fidelity notes
- **Road geometry.** Centrelines, lane counts, ramp topology and bridge stacking come from OpenStreetMap, and the horizontal alignment is survey-grade.
- **Heights.** Bridge heights are engineered approximations, not surveyed values.
- **Simplified surroundings.** Terrain is flat (the area is within a few metres of level), and buildings are simple extruded blocks.
- **Lane-level layout.** Lane widths are 3.66 m (Caltrans standard). Paint is simplified to edge lines and dashed 3 m / 9 m dividers. Gore areas come from overlap tests.
- **Licence.** The map data is © OpenStreetMap contributors, used under the ODbL. Keep this attribution if you share the environment.
