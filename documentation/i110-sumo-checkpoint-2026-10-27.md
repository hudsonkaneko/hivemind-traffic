# I-110 SUMO setup and Hudson checkpoint reproduction

27 October 2026 · Codex for Sam · Authorized repository audit and network setup.

**A runnable I-110 ACC engineering baseline now exists. Hudson's trained weights
are not available in the inspected repository, so no learned-versus-ACC comparison
or reproduction of his reported 20/20 result has been performed.**

## Repository-only checkpoint audit

Fetched all published upstream heads without switching/merging branches. Inspected
32 upstream remote/tag refs and reachable Git object names; no `.pt`, `.pth`,
`.onnx`, `.ckpt`, `.safetensors` or ZIP candidate was found. The public release list
is empty. The Actions listing contains 66 `windows-py311-cpu-sumo` artifacts; the
pinned MAPPO workflow uploads `outputs/ci/*.xml` test reports. This is not a claim
about Hudson's unpublished workstation files, deleted/unreachable history, or
uninspected private storage. CI archives were not downloaded.

The committed evidence identifies:

- Report: `d12c168c72c3a7441fb274af3f483de1609dac19`.
- Training source: `c06e45eaa79698eda85d01a65fa6ae5a6811b6d7`.
- Run: `train-20261026T190003Z-9d133a8c`.
- Missing actor: `outputs/sumo_mappo/v01/train-20261026T190003Z-9d133a8c/policy.pt`.
- Actor SHA256: `6ec2167aa2c769ec41c9221b77382fe74658430ce852703ce32568d089e591fd`.

All **14 recorded source-file hashes** can be reproduced from Git. Two files
(`network.net.xml` and the v02 configuration) require restoring the recorded CRLF
line endings; the recovered bytes then exactly match their recorded SHA256s.
The normalized configuration digest also matches. Both initial and corrected
audit records are retained under `outputs/i110-sumo-20261027/`.

For inference, the minimum missing binary is the exact frozen TorchScript
`policy.pt` actor. Optimizer/critic training state is unnecessary for inference;
`best.pt`/`latest.pt` is needed for training resume. Original per-episode metrics,
action journals and the resolved runtime manifest would strengthen an independent
numerical audit. Existing committed aggregate metrics are reports, not a local rerun.

Since Sam can obtain files only through the repo, the practical handoff is for
the maintainer to publish that actor as a versioned repository release asset with
the above hash and matching manifest. No message/request has been sent to Hudson.
Re-running training would create a new policy, not recover the missing weights.

### Exact original inference contract

The actor has 27 inputs, two 128-unit hidden layers and one deterministic tanh
output. Physical acceleration is `-4.5 + (action + 1) * 3.25` m/s². The simulation
advances at 0.1 s, with a 60 s horizon, three fixed lanes, a 5 km straight road,
25 m/s cruise target and 26.8224 m/s ceiling. Observations are exact simulator
state, scaled in code; no separate learned observation-normalizer file is needed.
The local guard and SUMO safety (`speedMode=31`, `laneChangeMode=512`) are part of
the tested system. The actor alone is not the entire driver.

Observation order is speed, acceleration, lane index, remaining-route fraction,
speed limit, previous requested acceleration; then own/left/right lane ×
front/rear × gap/relative-speed/presence; then left/right existence and elapsed
fraction. Exact scaling is preserved in the recovered environment source.

The source runner has `check`, `preflight` and `train` modes, not a standalone
exported-actor evaluation CLI. Once the actor is published, prepare a bounded
inference-only entrypoint against the pinned fixture, use the recorded test seeds
410001–410020, and compare raw metrics before attempting corridor transfer.
No new training is needed or authorized by this setup.

### Corridor transfer is a separate gate

The current actor cannot be directly attached to this lane graph:

- `lane_index / 2` and lane-existence masks assume three lanes. I-110 has four
  through lanes and a fifth auxiliary lane.
- Neighbor calculations compare edge-local lane positions on one straight road.
  Those positions reset at I-110 edge boundaries and do not identify connected
  lanes or vehicles on ramps. A graph-aware longitudinal coordinate is required.
- A 5 km remaining-route fraction and a 60 s episode clock differ from this
  approximately 815 m crop; silently renormalizing them changes learned inputs.
- The acceleration-only actor cannot select a lane or decide a merge/exit.

Do not clip lane indices, invent absent-neighbor observations or relabel a
three-lane subset as the full corridor. Preserve the exact original fixture first;
then document a transfer interface and evaluate its limitations. A retrained
four/five-lane policy would be a separate approved experiment.

## Network and baseline delivered

The portable [profile input](../scenarios/i110/sumo-road-profiles.json) is a compact
derivative of the refined NAIP/lidar road package, with its source hash and local
EPSG:6340 origin. The [builder](../scripts/build_i110_sumo.py) authors plain XML and
uses netconvert 1.27.1 to compile the network. No raw lidar, imagery, PeMS login,
Isaac installation or generated USD scene is needed to rebuild this SUMO graph.

Four southbound through lanes, a provisional auxiliary lane, the MLK entrance
and Vernon exit are connected. Mainline station cuts are 0/55/280/455/650/750/815 m.
Lane zero is rightmost; through-lane identities survive the addition/drop. The
auxiliary lane ends with no direct sideways connector: drivers must change lanes
upstream. Ramp ports are blended over 30 m. Mainline cap is 60 mph, ramp cap is
30 mph, and ACC desired speed is about 55.9 mph. These are engineering settings,
not independently verified posted speeds or observed demand.

The 650 m lane-drop point is a provisional graph choice within the interpreted
590–750 m taper. SUMO uses constant lane widths; the compiled network does not
reproduce fractional pavement taper width. Heights use the road center profiles,
not tire contact or full lane crossfall. Virtual seam polygons occupy 2 m
longitudinal bands to keep connector geometry moving south. No ramp meter,
arterial signal or wider upstream/downstream network has been invented.

An early compiled network passed traffic checks but reversed some Vernon
connectors by up to 1.59 m. This was corrected; the builder and regression test
now reject northward geometry in every compiled lane, including internal lanes.
Earlier outputs are preserved. The accepted build is `network-c`, not `network-a`
or `network-b`.

The [baseline runner](../traffic/i110_baseline.py) uses native SUMO ACC for four
fixed-through-lane cars, IDM human proxies, explicit seeded requests, and the
existing complete-trip ledger. Every route, seed, version, source/input hash,
trajectory and native tripinfo file is saved in a fresh output directory.
Requested trips waiting outside the network remain counted. Runs continue to
H=60 s, then drain the same cohort separately within a 90 s budget.

The braking-leader schedule is separate from the network: 15→5 m/s during
8–12 s, hold to 17 s, recover to 15 m/s by 22 s, then release native control.
Requested event commands are recorded separately. The existing crash,
construction and police-break USD previews are not yet connected to SUMO.

## Verification and practical limits

Twenty targeted tests passed. Two existing backend timeout-marker warnings remain;
final scenario runs had independent 120 s process supervisors and exited naturally.
No new dependencies were installed. Runtime: Python 3.11.16, SUMO/TraCI 1.27.1.
The available Torch wheel is 2.14.0+cu132 rather than Hudson's reported +cpu wheel;
no actor was loaded and no GPU training or Isaac run occurred.

Six final runs cover ordinary and braking-leader cases at seeds 730101/730102,
with a repeat of 730101 for each case. Both repeats match their entire trace
SHA256 exactly; each different seed changes the trace. All runs have zero
collisions, teleports, removed trips and reported emergency stops. All controlled
cars retain their physical through lane, and native tripinfo agrees with the ledger.

| Case / seed | Requested | Completed at 60 s | Unfinished at 60 s | Completed after drain | Last completion |
|---|---:|---:|---:|---:|---:|
| Ordinary / 730101 | 44 | 24 | 20 | 44 | 101.0 s |
| Ordinary / 730102 | 44 | 23 | 21 | 44 | 101.8 s |
| Braking leader / 730101 | 45 | 24 | 21 | 45 | 101.0 s |
| Braking leader / 730102 | 45 | 23 | 22 | 45 | 101.8 s |

These are connectivity/accounting smoke passes. Ordinary runs put the ACC cars
ahead of the following demand and do not exercise leader response; gap/TTC are
therefore null, not infinite or zero. The braking case does exercise following:
minimum measured ACC bumper gap is 17.31 m and closing TTC 6.73 s in these cases.
Across the ACC fleet, peak finite-difference jerk reaches **65 m/s³**, so smooth,
comfortable ordinary driving is **not accepted**. Native ACC internal safety
override counts are not exposed here; they are explicitly unavailable rather
than assumed zero. Comfort/response calibration and harder cases remain next work.

The demand is synthetic, boundary-cropped and uncalibrated. No traffic efficiency
advantage, realism validation, learned-policy acceptance, empirical admission or
communication benefit follows. P01–P05/P07 and independent A/B/C requirements
remain open. PeMS exports can later calibrate demand/speed/occupancy offline;
credentials and a live PeMS connection are not required to run this setup.

## Rebuild and rerun

From the checkout root, using the configured traffic Python (not Isaac Python):

```text
python -m scripts.audit_hudson_checkpoint --output outputs/hudson-audit-new
python -m scripts.build_i110_sumo --output outputs/i110-network-new
python -m traffic.i110_baseline --network outputs/i110-network-new/network.net.xml --output outputs/i110-acc-new --seed 730101
python -m traffic.i110_baseline --network outputs/i110-network-new/network.net.xml --output outputs/i110-braking-new --case braking_leader --seed 730101
python -m pytest tests/test_i110_sumo.py tests/test_trip_accounting.py tests/test_sumo_backend.py -q
```

Use fresh output names; existing runs are never overwritten. A future actor can
be checked with `--checkpoint PATH` on the audit command; it verifies bytes only,
does not deserialize the model or run it. The audit needs the pinned commits
locally; fetch upstream branches first if necessary. The network and ACC commands
use only files on Sam's branch and installed traffic dependencies.

Evidence: [compact results](evidence/i110-sumo-20261027.json),
[development plan](../scenarios/i110/comparison-plan.json), and local
`outputs/i110-sumo-20261027/` (raw traces, audit, summary CSV, figure and supervisors).
The shared master was read; its latest checkpoint is still October 22. Shared
writes remain held, with a local document-ready update prepared.

Primary technical references: [SUMO plain XML/netconvert](https://eclipse.dev/sumo/docs/Networks/PlainXML.html)
and [SUMO ACC implementation and limitations](https://eclipse.dev/sumo/docs/Car-Following-Models/ACC.html).
