# Sam's highway twin development plan

22 October 2026. Prepared by Codex for Sam. This is a scoped development plan
under Sam's approved research direction, not a replacement for the team's hybrid
roadmap or a new assignment to Hudson or Lorenzo.

## Starting point and scope

The live research master and both Sam refresher sections were read on 22 October
2026. Its latest repository entry is 20 October; readiness prose still describes
16 October. The local review and reproduced baseline supply newer engineering
evidence: commit `edb7125`, 1,784 tests passed, and deterministic scripted SUMO
trajectories. Both controlled cars finished at 54 seconds; three background cars
remained. This is not a traffic-wide outcome or a calibrated merge.

Sam now authorizes Isaac setup, usable 3D twin development and appropriate GitHub
synchronization. Detailed visualization is a main deliverable. The scientific
core remains one empirically supported open merge, Esri geography, human gap
uncertainty, factual calibration/development/untouched-audit partitions, matched
local/communicating shared PPO with CTDE in both arms, competitive scripts,
complete trip accounting, and a bounded physics-owned response experiment.
Training still requires an agreed protocol and explicit user involvement.

The repository's full hybrid physical fleet remains a separate broader roadmap.
Reuse its infrastructure without treating that broader roadmap as Sam's study.
SUMO owns replay poses; Isaac owns ego poses only in the separate response test.

## Deliverables and exit gates

22 October follow-up: Sam authorized real-merge qualification and complete trip
accounting. The new [evidence note](merge-accounting-2026-10-22.md) verifies the
accounting fixture and records I-80/Powell's unresolved admission gaps. Hudson's
forwarded report and commit `1fb5d1d` clarify that his current driver uses
simulator-state inputs, not LiDAR, and has no accepted full-scene learned policy.
The [integration contract](hudson-driver-research-contract.md) defines proposed
driver, command-interface and coordination gates. This updates the technical
handoff without adopting a full physical-fleet architecture or starting training.

| Stage | Concrete work and deliverable | Dependencies and acceptance | Main risk / parallel work |
|---|---|---|---|
| A: runtime | Isolated pinned Isaac Python, install log, package inventory, launch scripts | Verify GPU/driver/storage; import, app-ready, render to PNG, clean bounded shutdown | Registry/path/driver issues; asset inspection and plan can proceed during downloads |
| B: first usable 3D milestone | One supplied car, layered synthetic road replay, overview/follow cameras, play/pause/reset/scrub, selected-car speed/position/time | Reuse baseline recording; check explicit scale, axis, front-bumper anchor, stable IDs/lifecycle, source immutability; inspect real render and controls | Missing textures: visibly labeled fallback materials. Source is private until rights established. No Esri/calibration claim |
| C: geographic merge | Versioned Esri/CityEngine USD package and matching lane/merge surface | Site/data admission (P01–02), item IDs/rights/epochs/CRS, local-origin transform; two anchors, lane widths/connectivity and driving-surface check | Historical traffic vs modern context mismatch; source/rights requests can overlap A–B |
| D: accounting baseline | Requested/waiting/active/completed/removed/unfinished ledger, human/ramp splits, fixed horizon and drain report | Hand-computed delayed-entry fixture; no early-agent-end cutoff; teleports never count as completions; repeated seeds | Existing agent-focused termination is insufficient; independent of visual polish |
| E: matched controls | Route-aware observations, identical longitudinal actions/shields, shared centralized critic in L and C, explicit actor messages; tuned local/remote scripts | P03–13 freezes, actor/critic leakage tests, expiry/mask checks, truncation bootstrap tests, measured pilot budget | Current learner has local value input and different action semantics; do not relabel it CTDE |
| F: bounded Isaac response | One documented physics car; identification commands and held-out response; closed-loop following; validated surrogate fed into paired SUMO reruns | P17–18 envelope/physics/actuator provenance, independent validation inputs and error limits; single pose owner | Supplied visual geometry has no physics. A separate proxy needs explicit provenance; no real-car validation claim |
| G: frozen evidence | Locked models/policies/scripts/manifests, untouched factual audit, independent train/traffic uncertainty, figures and regeneration | P14–16, P19–20 decisions; capacity/precision demonstrated before matrix sizing; rights-safe artifact release | Unqualified data or weak policies can yield an honest interim result; no invented completion dates |

Each stage records passed, failed and pending checks with immutable run IDs,
source hashes, runtime versions and exact commands. Rendering performance is
measured on the actual scene after warm-up; GPU specifications are not throughput.
No fixed frame-rate promise, person-hour estimate or training capacity is assumed.
The master's December dates remain conditional research checkpoints.

## First milestone contract

Use `outputs/baseline-20261022-a/seed-2001-a/recording.json` and its network.
Original recordings and baseline layers remain untouched. New scene outputs get
their own directory. Metres, Z-up, X east/Y north, yaw = 90 − SUMO heading;
vehicle pose is front bumper. Reusable visuals extend behind that anchor.
The source car is identified by its supplied archive hash and exact USD prim,
not conflated with an existing prepared Americano or public physical sample.

The source collection has no authored unit metadata and lacks textures. A
centimetre-to-metre wrapper is an explicit engineering interpretation checked
against geometry/wheel positions; it is not measured vehicle dimensions.
Fallback paint/glass/rubber materials support inspection while preserving the
unchanged source. Wheel motion, sensors and dynamics are not inferred from meshes.

## Collaboration and release

Use the existing checkout on `codex/sam-visual-twin`; origin was fetched and
remains `edb7125`. Commit source/tests/docs; push a feature branch and open a
reviewable PR if credentials permit. Never merge over partner work. Private
vehicle geometry, machine paths, runtime packages and large raw evidence stay
local. Shared commands accept paths as arguments. The master Google Doc write
hold remains; prepare a local attributed update instead.

Recorded September focus areas connect this work with Hudson's traffic/repository
and Lorenzo's highway work, but do not establish exclusive ownership or current
availability. A practical handoff is the pose/asset contract to the highway
generator and the trip-ledger requirements to the traffic environment.

## Runtime reference

Choose Isaac Sim 6.0.1 for compatibility with the existing replay scripts rather
than migrating the repository to its experimental 7.x path. NVIDIA's
[Python setup](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/install_python.html)
requires a separate Python 3.12 runtime; the
[requirements](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/requirements.html)
list Windows 11 and RTX hardware. Actual launch/render checks determine local
readiness. This plan itself does not assert that installation has passed.
