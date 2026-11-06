# Highway environment v05 preparation and bounded pilot

November 5, 2026, America/Los_Angeles. **Environment preparation and final
no-learning preflight verified; authorized launch stopped during initial validation before learning, and a corrected resume is being prepared.** The inherited phase-0 actor remains unaccepted. V05
changes native lane motion, observed neighboring-lane occupancy, population
coverage and merge scoring before learning. It is a new environment treatment,
so its results must not be presented as a matched improvement over the old
environment or as physical/sensor-driven driving validation.

The living plan remains [sensor-driven training](sensor-driven-training-roadmap.md),
under the [hybrid roadmap](hybrid-roadmap.md). The resolved pilot configuration is
[highway_environment_v05.json](../experiments/configs/highway_environment_v05.json).
[Compact evidence](evidence/highway-environment-v05/index.json) preserves source
paths and hashes; full local outputs remain under
`outputs/highway-environment/v05/`. No weights or trajectories are copied into
this documentation package.

## Why the environment changed

The previous [merge recording](i110-merge-visual-v01.md) exposed abrupt native
lane movement and a success metric that counted first entry without requiring
stable post-merge behavior. V05 gives SUMO a **3.0-second lane-change duration**,
holds one known target until native lateral motion finishes, and measures the
actual lateral position and speed. Settled completion requires the target lane
identity, lateral offset below 0.05 m and lateral speed below 0.05 m/s. Requests
are not visual interpolation or learned steering. The measured fixture takes
3.1 seconds from command to settled completion because sampling is at 0.1 s.
No safe maneuver-abort controller is implemented.

The first continuous-motion fixture exposed a second problem. SUMO changes a
vehicle's primary lane ID near the maneuver midpoint, while the vehicle still
occupies part of the neighboring lane. Looking only at its primary lane omitted
that vehicle from the actor/teacher/guard's relevant leader context. The saved
diagnostic reports requested-versus-native acceleration differences rather than
asserting an unexplained SUMO cause.

The correction projects **current measured lateral occupancy** into both affected
lane corridors until the car centers. It handles occupancy before and after the
lane-ID switch and retains both corridors for an off-center stopped car. Neighbor
distances follow connected lanes with fractional parallel-lane projection; they
are not Euclidean leader gaps. Actor observations, scripted teacher, reward
target and safety guard share this context. No future maneuver intention or
privileged native control decision enters the actor.

## Preserved failed fixture and corrected repeat

The [paired evidence](evidence/highway-environment-v05/occupancy-correction.json)
retains the original failure and corrected runs. The complete physical population
assignment is identical: four controlled cars among 16 total, easy initial
traffic, scripted control, and 800 simulator steps. These are environment and
teacher checks, not learned-policy results.

| Measurement | Before occupancy correction | Corrected fixture |
| --- | ---: | ---: |
| Post-merge intervention decisions | 14 / 81 (17.28%) | 0 / 81 |
| Post-merge policy-filter changes | 0 | 0 |
| Post-merge native realization differences | 14 | 0 |
| Stable merges | 0 / 1 | 1 / 1 |
| Minimum post-merge headway | 1.7346 s | 1.7567 s |
| Post-merge distance | 138.3423 m | 138.0460 m |
| Post-merge allowed-distance progress ratio | 0.9607 | 0.9587 |

The corrected full run has **0 / 3,200 interventions**, zero collisions,
teleports, overspeed samples and measured emergency-braking samples. Its saved
native log has zero emergency events. A second corrected run has byte-identical
saved trace data, SHA-256
`7954d7555d1688e7fae6d910924df71611660da5d8aa4f364c5c669856e21492`.
The finite TTC minimum is absent in the corrected post-window because there is
no closing-leader TTC sample; infinite/nonclosing TTC is distinguished from
missing evidence. **No gate was relaxed** to make this fixture pass.

Raw failure: `outputs/highway-environment/v05/environment-smoke/20261106T020330Z/merge-repeat-a/`.
Diagnosis and measured-overlap evidence: sibling directories
`20261106T020555Z-diagnosis/` and `20261106T020644Z-overlap/`.
Correction: `20261106T020955Z-occupancy-fix/`.
Final repeat: `20261106T021058Z-final-repeat/`.

## Actor and control contract

The actor now receives **99 fields**: the original 45 local field positions,
six added motion fields, then six received-message slots of eight fields each.
The old local positions retain their meanings, but the neighbor values now
reflect measured corridor occupancy. Width compatibility therefore does not
mean identical old-environment semantics.

| New local index | Motion field |
| --- | --- |
| 45 | Native lateral position divided by lane width |
| 46 | Native lateral speed divided by 3 m/s |
| 47 | Committed lane direction |
| 48 | Maneuver in progress |
| 49 | Remaining maneuver distance fraction |
| 50 | Time since completion divided by five seconds |

The masked centralized critic has **24 stable slots and 2,400 inputs**
(`24 Ã— (99 + presence)`). Absent slots are excluded from learning. The actor
still requests acceleration in `[-4.5, 2] m/sÂ²` and masked lane intent `-1/0/+1`.
The 26.8224 m/s ceiling and existing 1.5 s following/stopping guards remain.
Messages use the six nearest fresh delivered snapshots within 150 m, a 0.1 s
delivery delay and 0.5 s expiry. Exact current SUMO/map fields remain structured
observations; camera/LiDAR policy input is still future work.

Warm-start migration copies the previous 45 local columns and 17 pooled-message
features, inserting six zero-weight columns for motion. The 24-slot critic and
optimizer start fresh. The configured parent is the existing phase-0 checkpoint
`outputs/highway-stage1/v03/train-20261103T230506Z-04ba1761/selected-00036-d97376b9/best.pt`,
SHA-256 `b77f55ae7262178ce0fa15729fe844e18693cd168905eea8b1bbd579c6913ab1`.
Its actor artifact hash is
`8526f6134b7eae90d3370bd12cc0487926a6ec4356c6c6ea2dd231e8aa8b1bb8`.
This migration preserves predictions for matching old feature values; it does
not establish competence under the enriched context. The parent's intervention
failures remain recorded in [population v04](population-near-limit-v04.md).

## Stable merge and full-route scoring

First through-lane entry, settled lateral completion, and stable completion are
separate events. After settled completion, stable success requires a **complete
eight-second observed window** with zero collision, emergency and overspeed
samples; headway at least 1.5 s and closing TTC at least 3 s when a leader exists;
interventions at most 5%; and progress at least 25% of the integrated allowed
distance. Missing observations or early arrival/truncation cannot manufacture a
completed window. Reports distinguish pending, failed and succeeded outcomes.

The environment retains per-car post-window and full-route metrics, requested,
filtered and realized acceleration, policy-filter changes and native realization
differences. Native emergency warnings remain a separate flushed-log measure.
Neighbor braking is reported as contemporaneous observation, without causal
attribution. Stable success adds a bounded **2.0 reward once**. An evidence-backed
unnecessary reversal adds a **0.5 penalty**; route/safety/passing exceptions and
unclassified reversals are not treated as proven oscillation. Necessary passing
is not prohibited. Full-episode acceptance still fails safety or intervention
problems outside an otherwise successful merge window.

## Demand and curriculum coverage

Demand construction draws physical positions, speeds, driver profiles and
departure times independently of controller assignment. Initial totals are
**8, 16 and 24**, independently paired with requested AV fractions
**0.125, 0.25, 0.5, 0.75 and 1.0**. Fixed membership persists through each episode;
there is no live controller handoff. Following and passing include a native slow
leader, so those tasks exclude full AV assignment. Full-AV cases are initial-only;
sustained mixed cases schedule later native arrivals, which change the actual
on-road fraction. Scheduled demand is not realized throughput or density.

Difficulty 0/1/2 changes gap slack, speed spread and native arrival intervals
(4.0/2.8/2.0 seconds). Three native driver profiles vary headway, acceleration and
speed factor without weakening the AV guard. Reset verifies all requested initial
IDs actually inserted. The pilot requests zero native pre-control warmup;
the eight-second merge window is a separate measurement, not hidden warmup.

The [six-road bank](evidence/highway-environment-v05/network-bank.json) adds
120/240 m upstream ramp placements in four training layouts and 180 m placements
in two validation layouts. These are placement parameters, not compiled ramp
lengths. Existing 2â€“5-lane networks and imported I-110 remain unchanged. I-110
validation uses disjoint demand seeds on a shared engineering road and is not
held-out geometry. The combined configuration has 36 training cases and 24 fixed
validation rows. The existing held-out test bank remains unused.

The [static sweep](evidence/highway-environment-v05/static-demand-summary.json)
passes **504 / 504 authored-demand cases** across seven roads, four tasks, three
difficulties, three totals and two seeds. It checks high/full participation and
graph clearance, not every fractional curriculum case or native traffic flow.
Native smokes separately exercise 24/24 controlled cruise for 100 steps and
zero-AV sustained inflow for 100 steps; the latter observes up to ten vehicles
from an initial cohort of eight. Neither short probe proves full-horizon task
competence for those populations.

Curriculum grades use the most recent 32 **complete learned-training reports**,
at least two per relevant task, and a 90% report-pass threshold. Cruise/following
gates permit grade 1; all four task gates permit grade 2; recent failures can
reduce the grade. Ordinary rows apply that grade as actual demand difficulty.
Cruise/following/passing/merge weights are `4/4/1/1`, then `3/3/3/2`, then
`2/2/3/5`; every grade retains all tasks. Ordinary speed blocks cover
12, 18, 22.352 and 26.8224 m/s. Communication varies independently between none,
local, and local with 30% packet loss.

At most 20% of issued rollouts retry failed **training** scenarios. A retry keeps
its original seed and difficulty but requires a fresh simulator rollout and a
new decision charge. No old PPO samples or validation trajectories enter the
retry bank. Partial reports, critic warmup and zero-step optimizer results do not
advance grades. JSON curriculum state records deterministic permutation indices,
issued scenario hashes, recent evidence, failure scenarios and grade history.

## Verified final preflight

[Final compact evidence](evidence/highway-environment-v05/preflight-final.json)
comes from `outputs/highway-environment/v05/preflight-20261106T021925Z-df9e42a7/`.
It passes in **64.750 seconds**, with **zero optimizer steps**, unchanged model
weights and no source drift. API/seed checks, exact teacher repeat, finite frozen
collection, actor export, population capacity and native inflow checks pass.
[The targeted CPU log](evidence/highway-environment-v05/targeted-tests.txt) records
**256 tests passed**, six retained TorchScript deprecation warnings, in 4.45 s.

| Full teacher fixture | Interventions / decisions | Task result |
| --- | ---: | --- |
| Synthetic cruise | 0 / 3,200 | Mean speed ratio 0.9847 |
| Synthetic following | 3 / 3,200 | Mean speed ratio 0.9097 |
| Synthetic passing | 5 / 3,200 | Four passes |
| Synthetic merge | 0 / 3,200 | One of one stable merge |
| I-110 merge | 75 / 1,404 (5.3419%) | One of one stable merge; full-route intervention gate fails |

All five fixtures have zero collisions, teleports, overspeed samples, measured
emergency samples and native emergency warnings. **Readiness is not driving
acceptance:** I-110 still exceeds the unchanged 5% intervention threshold. The
preflight remains a bounded plumbing/readiness check, and its own
`nominal_driving_acceptance` is false. The earlier 58.937-second preflight is
[preserved as history](evidence/highway-environment-v05/preflight-initial-history.json);
it is superseded by the final source/config-matched run. The subsequent
[65.875-second passing preflight](evidence/highway-environment-v05/preflight-final-before-whitespace.json)
is also retained: a whitespace-only source correction required a fresh matching
proof before launch.

## Resolved pilot and pending results

The [pretraining settings snapshot](evidence/highway-environment-v05/reviewed-pretraining-settings.json)
specifies one CPU worker/thread, MAPPO seed **9507**, training scenario seeds
**6800000â€“6800999**, and validation seeds **6900000â€“6900023**. The attempt is
bounded by **250,000 active-agent decisions or 3,600 aggregate seconds**, whichever
comes first, with 600 seconds reserved for final evaluation. Initial/final
evaluation share the wall budget; critic warmup, PPO collection and interrupted
reservations share the decision budget. Critic warmup is at most 4,096 decisions
and may stop below that ceiling to avoid changing an issued population row.

Actor/critic learning rates are `0.0001/0.0003`, with four epochs, minibatches of
256, PPO/value clipping 0.2, entropy coefficient 0.001, value coefficient 0.5,
gradient norm cap 0.5 and target KL 0.02. The separate imitation coefficient is
0.04 with lane-change weighting 10. The raw near-limit acceleration coefficient
remains 0.1 over the 1 m/s near-limit band. Gradient diagnostics remain enabled.
Initial and final deterministic evaluations use the same 24 rows; there is no
validation-driven checkpoint cherry-picking or held-out promotion.

**Training results: pending.** No updated actor, learning improvement, 24-case
acceptance result or rendered v05 policy behavior is claimed by this preparation
record. The frozen recorder is implemented to retain actual SUMO XYZ/heading
through the full 80-second accounting horizon and to suggest a maximum-300-frame
clip around the first actual merge. A missing merge is labeled and never rerolled.
Prepared-car rendering is presentation of SUMO replay, not physical transfer.

From the project root, preparation and no-learning verification use:

```powershell
& .\.venv-traffic\Scripts\python.exe -m experiments.run_highway_environment_v05 --mode config
& .\.venv-traffic\Scripts\python.exe -m experiments.run_highway_environment_v05 --mode preflight
```

Training requires the runner's explicit authorization flag, current readiness
proof and exact reviewed configuration digest. A future frozen replay supplies
`--policy`, `--expected-policy-sha256`, `--config`, `--case-index` and a fresh
project `--output` to `scripts/record_highway_environment_v05.py`; its selected
row, full report, sources, policy and network hashes remain with the recording.

Shared roadmap, change-log and connected-guide updates are handled with the
overall milestone. This technical note does not claim that the Google mirror
has already synchronized.


## First launch: preserved pre-learning failure

Run `train-20261106T022222Z-0b8b611a` stopped during fixed validation row 11, before any collected training decisions or optimizer steps. Its 125.344 seconds remain charged to the 3,600-second budget. The original checkpoint and failed run remain untouched. [Recovery evidence](evidence/highway-environment-v05/training-recovery.json) preserves the traceback and subsequent frozen checks.

The generator stored departure `8.200348149208052` while SUMO represented the same request at `8.2` seconds. Strict trip accounting correctly rejected an apparently early departure. The fix writes the same native millisecond time to both route XML and manifest, preserving the raw random schedule and all other vehicle fields. The original 504-case static sweep covered initial-only demand and could not expose this sustained-arrival precision mismatch. New regression tests exercise the strict ledger and sustained departures.

The exact failed seed 6900011 then completed 80 seconds in 23.641 seconds with unchanged frozen weights, zero collisions/teleports/native warnings and correct requested/departed timestamps. Its 34.62% intervention rate remains a policy failure. A separate ending-lane warning in row 10 exposed interrupted-maneuver cleanup. The corrected environment records native interruptions once and releases the held target; it never counts them as completion or a controller-implemented safe abort. An actor-visible map/speed mask requires enough connected target corridor for the three-second maneuver, maximum permitted acceleration, one control tick and the existing standstill clearance. Connected same-identity lane seams remain eligible; insufficient runway and unsafe gaps retain both diagnostic causes.

The same frozen row 10 then completed 80 seconds in 22.921 seconds with zero collisions, teleports or native warnings. Its 29.535% intervention rate remains a behavior failure. 265 targeted tests pass after both repairs. A strict, reviewed zero-update checkpoint migration retains all model/optimizer/RNG state, counters and the 125.344 seconds already charged; only the two corrected source hashes and fresh readiness proof may change. [Source-matched recovery preflight](evidence/highway-environment-v05/recovery-preflight.json) passed in 64.453 seconds with zero optimizer steps. [Checkpoint migration](evidence/highway-environment-v05/checkpoint-migration.json) verified unchanged state and retained elapsed budget. The corrected resume is authorized under the same limits.


## Second launch: native insertion feasibility

The resumed attempt `train-20261106T023346Z-75467609` stopped at initial validation row 21, before learning. Total charged runtime across both attempts is 512.048 seconds, with zero training decisions/optimizer updates. [Second failure evidence](evidence/highway-environment-v05/training-recovery-02.json) preserves the checkpoint and traceback.

All 24 requested initial vehicles eventually inserted, but SUMO ACC delayed a closing follower behind the slower leader. A later inflow vehicle entered before the cohort was complete, so strict reset verification correctly rejected the resulting population. The original clearance check was collision-conservative but did not reproduce native ACC insertion feasibility. [The source-backed spacing correction](https://raw.githubusercontent.com/eclipse-sumo/sumo/v1_27_1/src/microsim/cfmodels/MSCFModel_ACC.cpp) now passes [132/132 reset cases](evidence/highway-environment-v05/native-reset-sweep.json): all 24 validation rows and all 36 training road/task cases at each of three grades, with maximum eligible controlled counts. Every initial vehicle departs at zero and is present at 0.1 seconds; zero collisions, teleports or native warnings. The sweep takes 84.313 seconds. 269 targeted tests pass. Counts, speeds, assignment, native guards and evaluation seeds remain fixed; starting positions/initial spatial density change where native insertion requires more room. The failed row’s mean nearest bumper gap changes from 50.61 to 59.29 m; the minimum remains 38.93 m. [Position-only changes](evidence/highway-environment-v05/spawn-spacing-comparison.json) are retained explicitly. No learned results or completed resume are claimed at this checkpoint.


Fresh source-matched preflight passed in 65.453 seconds. [Second migration](evidence/highway-environment-v05/checkpoint-migration-02.json) verifies all starting state and 512.048 charged seconds remain unchanged; only demand source/readiness change. The same 250k/3,600-second total limits apply to the resumed launch.
