# Highway driving curriculum v01, including I-110

Follow-up: the approved pilot has now completed. See [training results and
revisions](highway-curriculum-training-v01.md). The preparation record below
preserves the original preflight status and selection rule.

November 2, 2026. The user requested Sam's I-110 merge road for MAPPO, then clarified
that training should cover highway speed, ordinary traffic, passing and different
lane counts. This is a new SUMO policy experiment. Segmentation preparation remains
separate and disabled; no learning run has started in this milestone.

## Imported road and separate policy contract

Selected road builder, profile, graph observations, trip accounting and graph tests
come from Sam's PR 16 at `b4f19e2b450ea585d1b80fa7472a091f928976de`. The exact imported
file hashes are in [source provenance](evidence/i110-merge-training-v01/source-import.json).
This is a selective import, not a merge of that PR or a claim that the corridor is
empirically calibrated. The original prepared vehicle and physics/sensor demos are
unaffected.

The accepted v04r2 actor was trained on a straight three-lane fixture. Its weights
are **not loaded** into this new task. We reuse the MAPPO network implementation,
but declare the distinct `highway_curriculum_graph_messages_v1` ABI. Equal tensor
width does not make the policies interchangeable. Fresh initialization is required.

Each actor receives 81 fields: 33 local features plus six masked eight-field peer
messages. Local features include normalized speed/acceleration, current lane,
remaining route, actual allowed speed, previous acceleration request, graph-based
front/rear neighbors in own/adjacent lanes, adjacent-lane availability, lane count,
distance to a required merge, merge requirement, cooldown and right/stay/left masks.
Unknown/internal junction lane changes are masked. Neighbor distances traverse
actual lane connections, including incoming rear traffic on another route.

Messages are snapshots from controlled peers only: southbound position difference,
sender merge requirement, relative speed, acceleration, previous acceleration/lane
request, age and presence. They use a 150 m Euclidean radio range, 10 Hz transmission,
one 0.1 s step delay and 0.5 s expiry, with seeded configurable packet loss. Position
difference is explicitly the corridor's southbound map axis, not a guarantee of
lane-path distance or a front/rear relation. Actors never receive the critic's joint
state. All sensing is exact current SUMO state/map; cameras and LiDAR are not inputs.

Output is normalized acceleration mapped to −4.5..2 m/s² plus a masked lane request
−1/0/+1. Speed follows the current lane limit under the 60 mph ceiling. Native SUMO
safety remains active; lane-gap checks require 1.5 s plus a stopping-distance floor.
Requested, filtered and realized accelerations are retained in compressed per-step
action journals. This does not
train physical steering or establish safe short headways.

## Curriculum and adaptation tests

| Partition | Layouts and tasks |
| --- | --- |
| Training | Generated 2, 3, 4 and 5 through-lane roads, plus I-110; four tasks each = 20 balanced cases |
| Tasks | Light-traffic cruising, dense following, passing a slower leader, ramp/auxiliary-lane merging |
| Validation | Separate 2/4/5-lane layouts, opposite bend and 55 mph limits; 12 task/layout cases |
| Held-out test | Separate 3/5/6-lane layouts, another bend and 45/60 mph conditions; 24 seeds across 12 cases |

An auxiliary lane adds one lane where present. The six-through-lane layout is not
used for training or validation. Training uses 50/60 mph mainline limits and 30 mph
ramps. Initial gaps, speeds and background routes vary by seed. Following and
passing include two vehicles ahead in every through lane from the start, as well
as the explicit slower leader and later incoming traffic. The case order is
a balanced cycle, not an automatic difficulty schedule. Four controlled actors
start on the mainline for cruising/following/passing, or two mainline/two ramp for
merging. Background traffic stays lightweight SUMO IDM.

These layouts test a defined class of southbound highways. They do not cover arbitrary
intersections, left-side driving, all road shapes, unseen obstacle classes or real
drivers. Per-task and per-layout held-out results are required before claiming
adaptation. A high aggregate reward can conceal a failed task.

## Proposed pilot configuration

[`experiments/configs/i110_merge_v1.json`](../experiments/configs/i110_merge_v1.json)
is the final configuration entry point despite its initial merge-only filename.

| Setting | Proposed value |
| --- | --- |
| Algorithm | Shared actor MAPPO, separate centralized critic, fresh graph weights |
| Actor / critic widths | 128/128 and 256/256; 81 local fields, 328 joint-state fields |
| Runtime | CPU, one worker, one Torch thread; no rendering |
| Episode | 0.1 s steps, maximum 80 s of control; fixed full-cohort accounting horizon |
| Actions | Acceleration plus legal lane change; five-second change cooldown |
| Budget | 500,000 active-agent transitions or two hours, whichever first |
| PPO | Actor LR 3e-5, critic LR 3e-4; four epochs, batch 256, clip 0.2, target KL 0.02 |
| Exploration | Entropy coefficient 0.001 |
| Rewards | Progress; one-time merge/pass/arrival; collision, intervention, jerk and lane-change penalties |
| Validation | Every 10 training episodes, all 12 fixed validation seeds |
| Selection | Safety, merges, unfinished cohort, intervention fraction, then reward; test excluded |

This is a pilot budget, not a promise of broad competence. The control-loop budget
is checked between bounded steps; final accounting, an update or checkpoint may
finish after the deadline. Checkpoints retain model, optimizer, Torch RNG, next seed,
transition count, validation rank and elapsed budget. `last.pt` is for resume;
`best.pt` and `policy.pt` reflect validation selection. The export includes an explicit
contract/hash sidecar. A run can finish without producing an accepted policy.

All requested trips are retained, including waiting, unfinished and removed trips.
The primary accounting horizon is fixed even when controlled cars finish early.
A separate 60 s native drain follows; drain completion is not credited as a learned
merge. Native-baseline intervention zero means no policy command to override and
is not comparable to learned/scripted intervention fractions.

## Commands and evidence

The ordinary command prints configuration and its approval hash; it cannot train:

```powershell
./.venv-traffic/Scripts/python.exe -m experiments.run_i110_merge
```

The explicit `--mode preflight` runs API/seed tests, scripted/random/native SUMO
episodes and an untrained forward/collector check with zero optimizer steps.
Full learning additionally requires `training_authorized=true` and the exact
reviewed `--approved-config-sha256`. That flag remains false.

Frozen scoring is available with `--mode evaluate --checkpoint <best.pt> --split
validation` or `test`, using the matching config. Test scoring rejects a checkpoint
that was not selected on validation. It never performs an optimizer update.

Runtime artifacts are under `outputs/i110-merge/v01/`; generated curriculum roads
are under `outputs/highway-curriculum/v01/`. These ignored directories are local,
not included in a Git clone. Builders preserve existing output directories. A new
machine must rebuild roads, inspect them and bind their actual hashes in a new
reviewed config; absolute paths/generated XML headers can change byte hashes.
No unreviewed geometry is silently accepted by the training runner.

Earlier merge-only checks are preserved separately. The first failed on a floating
point departure-time representation; numeric departures were normalized to the
millisecond grid without weakening ledger checks. Later merge-only scripted cases
completed both ramp merges with zero collisions but 36.8–52.4% interventions after
initial-condition randomization. These are development diagnostics, not accepted
learned performance, and they motivated the broader curriculum rather than replacing
its final evidence.

## Completed no-learning verification

Final preflight `preflight-20261103T002100Z-596c57ad` passed in 367.844 s with
**zero optimizer steps**. It ran 20 scripted task/layout cases, two random cases,
two native reference cases, an exact scripted repeat, PettingZoo API/seed tests,
an untrained model forward and a finite 128-transition collector/GAE check.
All runtime source hashes remained unchanged. Final config readiness/target
metadata was added after the run; model, environment and optimizer settings did
not change. **129 focused CPU tests pass**, including existing MAPPO regression tests.

| Scripted reference result | Measured range / total |
| --- | --- |
| Collision / teleport events | 0 / 0 across all 20 cases |
| Merge cases | Both ramp cars merge in all five layouts (10/10) |
| Passing cases | 2, 3, 4, 4, 4 passes across the five layouts |
| Cruise mean fraction of applicable speed limit | 0.945–0.994 |
| Following intervention fraction | 0.606–0.779 |
| Passing intervention fraction | 0.404–0.671 |
| Merge intervention fraction | 0.046–0.137 |

High dense-traffic intervention rates are a real limitation, not an accepted
controller result. Random actions remain very slow. These checks establish that
the environments and collector execute safely under SUMO's protections; they do
not establish a good learned policy. Training should be judged by separate task
scores, intervention burden and matched complete-cohort baselines.

[Compact results](evidence/i110-merge-training-v01/curriculum-preflight-summary.json),
[runtime/source provenance](evidence/i110-merge-training-v01/curriculum-preflight-provenance.json)
and [exact observation contract](evidence/i110-merge-training-v01/curriculum-preflight-contract.json)
are retained. Per-episode full trip ledgers and compressed action journals remain
under the local run directory. Earlier merge-only and lighter-front-traffic checks
remain preserved; no validation or held-out traffic seeds have been evaluated.

The same 129 tests also pass in a clean archive of commit `571b4d5`, without
copying generated roads or runtime packages into the archived source tree.
