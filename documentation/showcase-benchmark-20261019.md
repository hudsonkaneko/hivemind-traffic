# Showcase display matrix — October 19, 2026

All **11 individual native trials passed their physical/scene/process gates**.
The two four-background cases passed the unchanged strict display thresholds;
all nine 80-background cases failed them. The original evaluator rejected
**all eight comparisons** with `Physical scene/assets/fleet contract differs`,
including the two same-configuration repeat pairs. That original failed result
is preserved. A narrowly corrected evaluator now accepts **all eight comparisons**
in a separate reassessment of the same recordings; no native trial was rerun.
The timing differences below are individual observations, not statistical or
causal performance guarantees. Dense strict-display qualification remains failed.

This report records individual outcomes from the
[declared matrix](showcase-validation.md), following the
[nine-run functional checkpoint](showcase-validation-20261019.md). The
[compact evidence JSON](evidence/showcase-benchmark-20261019.json) retains exact
measurements, source profiles and hashes. Earlier endpoint failures and repeat
evidence remain in their original report; they are not replaced by this matrix.

## Frozen source and protocol

Matrix `20261019T172621Z-63aa7c17` ran from commit
`3a3185ad2ae0c5aa4a93987b00b10385e37e19ab`. Its manifest captured a
49-file runtime/evaluator source union. The v09 run manifests contain 46 runtime
sources; the v07 editor/minimal pair contains 42. Their hashes match the recorded
matrix union. In particular, the bounded live controller retained SHA256
`7b3778e9635422f7773f63cde5ecee308d5fb4f48a49312363913d245d0d4042`.
The original benchmark evaluator is recorded as
`ae58a85c28d493200a0dddb1d01d732c10c9b0d7e70101487495793a09c4001e`.
Manifests retain working-tree state; this does not assert a pristine checkout.

Each trial used one public procedural PhysX main car, scripted nonreactive
kinematic backgrounds, privileged simulator state and the known circular map.
There was no training, sensed control or SUMO traffic. Runtime was Isaac Sim
`6.0.1-rc.7+release.42383.32955d8d.gl`, Python 3.11.9, Windows build 26200 and
GeForce RTX 5070 with 12,227 MiB, driver 610.88.

All trials used seed 101, 120 Hz physics, 60 Hz control, 10 Hz planning, eight CPU
workers, full 120 Hz traffic statistics and 90 driving seconds, plus two settling
and ten final-brake seconds. Statistics-label refresh remained 5 Hz. Frame
generation was disabled. The cases vary only the declared panel, display,
capture, evidence or editor settings; density is not a matched comparison.
The editor/minimal comparison uses v07 on both sides, not v07 versus v09.

The light cases have four backgrounds; all remaining cases have 80. Requested
display is 30 Hz except `dense-20hz`. `dense-captures` enables viewport capture;
`dense-full` uses full evidence. `minimal-cost` uses the minimal showcase UI.
The other cases use presentation evidence and the regular editor. These are
fixed-population trials without the earlier interactive endpoint schedule.

## Individual measured results

Every v09 run passed 32/32 run gates; each v07 run passed 31/31. All exited zero,
verified cleanup, recorded zero contacts and reported no runtime source changes.
The two light runs each recorded one pass, no lane change and minimum observed
clearance 1.896022 m. Every dense run recorded 16 passes, eight lane changes and
minimum clearance 1.779629 m. These are per-run observations. The separate
corrected comparison also verifies the declared cross-run equivalence checks.

| Case | Version / backgrounds / requested Hz | Active wall s | RTF | Lag p95 / max, s | Strict display |
| --- | --- | ---: | ---: | ---: | --- |
| `light-visible` | v09 / 4 / 30 | 102.000290 | 0.999997 | 0.000506 / 0.001208 | Passed |
| `light-hidden` | v09 / 4 / 30 | 102.000395 | 0.999996 | 0.000511 / 0.014820 | Passed |
| `dense-visible` | v09 / 80 / 30 | 114.308491 | 0.892322 | 13.529411 / 13.851056 | Failed |
| `dense-hidden` | v09 / 80 / 30 | 114.498922 | 0.890838 | 13.740176 / 14.092138 | Failed |
| `dense-20hz` | v09 / 80 / 20 | 102.000128 | 0.999999 | 0.202518 / 0.264480 | Failed |
| `dense-captures` | v09 / 80 / 30 | 114.252398 | 0.892760 | 13.551578 / 13.884175 | Failed |
| `dense-full` | v09 / 80 / 30 | 141.480089 | 0.720950 | 39.196332 / 39.480089 | Failed |
| `editor-cost` | v07 / 80 / 30 | 111.660309 | 0.913485 | 11.274404 / 11.534555 | Failed |
| `minimal-cost` | v07 / 80 / 30 | 102.462406 | 0.995487 | 2.548458 / 2.606414 | Failed |
| `dense-repeat-1` | v09 / 80 / 30 | 114.592150 | 0.890113 | 13.815437 / 14.166930 | Failed |
| `dense-repeat-2` | v09 / 80 / 30 | 115.981534 | 0.879450 | 14.897049 / 15.152176 | Failed |

Strict acceptance requires RTF 0.99–1.01, lag p95 at most 0.1 s and maximum lag
at most 0.5 s. Preview-loop interval p95 must be at most 1.5/requested-Hz seconds
and its maximum at most 200 ms. Native notification delivery must reach at least 99% of requested
rate, with interval p95 at most 1.1/requested-Hz seconds and maximum interval
100 ms. Application event-service interval p95 must be at most 50 ms and maximum
100 ms. All trials have empty display-qualification exclusion lists; the dense
failures are actual timing-gate failures, not exclusions for interactive pauses.

`dense-20hz` reaches near-one RTF but still fails the 0.1 s lag-p95 limit, native
maximum interval and application-service thresholds. `minimal-cost` also has
near-one RTF, while lag and maximum interval fail. `editor-cost` records a
2,214.218 ms maximum native notification interval. These observations do not
identify the cause of a stall. The evidence JSON preserves every failed threshold
and native/application interval value.

The two light cases qualify this bounded, requested-30-Hz measurement under the
declared gates. Kit frame notifications are **not monitor scanouts**, and their
rate does not establish 60 FPS, end-to-end latency or a one-to-one relation
between submitted and completed frames. No still image establishes smoothness.

## Original failure and corrected reassessment

The rejected comparisons are `light-panel`, `dense-panel`, `display-cadence`,
`capture-cost`, `evidence-cost`, `editor-cost`, `dense-repeat-1` and
`dense-repeat-2`. The failure occurs at the physical scene/assets/fleet contract
check. The original summary has `passed: false`, all trials completed and no
matrix stop reason. It remains immutable alongside the successful reassessment.

Independent diagnosis found the only rejected physical-contract field was
`vehicle.scene_composition.assembly_directory`: each fresh process had its own
absolute output directory. All 11 values were verified against that recording's
canonical `run/scene` location. The benchmark-local correction validates that exact
own-run path before normalizing only this packaging field to `scene/`. It does
not discard asset hashes, factory contracts, prim paths, physical properties,
reference policies or other composition data. Foreign, relative, traversal and
malformed assembly paths are rejected; a path missing on one side remains a
contract mismatch. Source and test review found no
remaining blocking finding.

Reassessment `20261019T175418Z-a5a6a122`, under
`outputs/showcase_benchmark_reassessment/`, used corrected evaluator SHA256
`9a388b68ebbac79e9575f45bb4dd1b0d7fcfea3652b142bdc2d36b5e7e135f1d`.
All eight pairs are valid, with **zero maximum position and scalar-speed
difference** and exact per-tick controls. Source, runtime, asset, physical and
load checks remain enforced. The original inputs were unchanged and no simulation
was repeated. The independent reassessment review verified nine output hashes
and six captured sources.

The complete corrected-source CPU suite passed **1,767 tests**, with **181
optional skips and 116 subtests**, in **86.23 s**. Metadata and JUnit evidence are
retained under `outputs/showcase_validation/cpu-comparator-20261019T175324Z/`.
The manifest records commit `3a3185a` plus the captured evaluator/test changes;
these are source-specific local results, not a current-head hosted-CI claim.

## Observed paired wall times

Direction is explicit: percent change is `100 × (second wall / first wall − 1)`.
Each row is one observed comparison under the recorded local desktop load.
Validity confirms matched physical behavior; it does not grant strict display
qualification or a causal speedup.

| Comparison | First → second | Observed active-wall change | Both strict display passed |
| --- | --- | ---: | --- |
| Light panel | hidden → visible | −0.000102% | Yes |
| Dense panel | hidden → visible | −0.166316% | No |
| Display cadence | dense hidden 30 Hz → 20 Hz | −10.916080% | No |
| Capture | dense hidden off → on | −0.215307% | No |
| Evidence | dense hidden presentation → full | +23.564560% | No |
| Editor | v07 minimal → regular editor | +8.976857% | No |
| Dense repeat 1 | dense visible baseline → repeat 1 | +0.248151% | No |
| Dense repeat 2 | dense visible baseline → repeat 2 | +1.463621% | No |

The dense panel and capture differences are smaller than both observed baseline
repeat changes. They do not establish that showing the panel or enabling capture
improves performance. Full recording, display cadence and editor variants show
larger observed differences, but each still has one trial per variant and the
dense timing failures remain. No default, physics cadence or safety/statistics
sampling requirement is changed by these observations.

The pre/post external Kit inventory was empty for every trial, and the recorded
desktop memory-load observations were available. The original comparison's
`load_qualified` field remained false after the earlier contract exception; it
must not be interpreted as evidence that another Kit process was observed.
The corrected assessment reaches this check and marks every pair load-qualified.
The actual inventory checks still do not rule out transient or non-Kit desktop
activity. GPU peak memory is a whole-desktop observation, not isolated simulator
memory.

## Timing investigation: late stalls and sustained lag

An independent reduction recomputed all 11 timing qualifications from saved
preview and native/application event timestamps; every result matched the
recorded strict qualification. Five dense v09 30 Hz presentation cases — visible,
hidden, captures and both repeats — have preview intervals over 200 ms at the
same **28.4333 s and 67.2667 s** simulation times. Their largest are about
225–228 ms, while their first frames are about 44–50 ms. Driving-segment RTF is
roughly 0.85–0.87, with sustained growing lag. A startup-only explanation is
inconsistent with these late repeated spikes and sustained shortfall.

The 20 Hz case has late long intervals at 40.25 and 85.25 s; near-one total RTF
does not undo its 0.202518 s lag p95 or native/service failures. The v07 editor
case has a 2.2124 s preview outlier at 89.3 s and a separate aggregate render
maximum of 2.1893 s. The minimal v07 case accumulates lag during driving and
catches up during final braking; its 0.995487 total RTF conceals 2.548458 s lag
p95. Full evidence shows sustained lag rather than only an initial delay.

These timings identify where to measure next, not which subsystem caused the
stalls. Aggregate publication maxima lack tick/subcall identity and cannot be
assigned to a particular long frame. The next bounded diagnostic should record
timestamped native/Sdf publication subcalls and CPU/allocation/GC events around
28.4333 and 67.2667 s, and inspect the separate editor render bracket. Preserve
the existing physics, control, safety and statistics cadences and unchanged
qualification thresholds. No speculative optimization is presented as verified.

## Independent audit and retained evidence

A read-only audit passed **15 checks per trial** and matched all **1,063 recorded
output hashes**. It checked captured runtime inventories against the matrix
snapshot, resolved configurations against the declared cases, physical acceptance,
completeness, cleanup, source-change reports, quiet pre/post Kit inventory and
strict display thresholds recomputed from the recorded measurements. It did not
launch a simulator, test suite or comparison evaluator.

The original matrix manifest, index, summary and resolved configuration stayed
unchanged. Their hashes are published in the evidence JSON, along with selected
per-run output hashes and source inventories deduplicated by profile. The local
audit is `outputs/showcase_validation/oct19-review/benchmark-individual-review.json`;
its reproducible reader is `reduce-benchmark-individuals.py` in that directory.
The corrected assessment, comparator and reassessment reviews, full CPU result,
and `matrix-frame-timing-analysis.json` are separately hash-referenced in the
published evidence JSON.
Raw telemetry remains ignored; the compact JSON cannot replace unavailable raw
files for independent recomputation.

| Case | Immutable native run ID |
| --- | --- |
| `light-visible` | `20261019T172621Z-6d756fdc` |
| `light-hidden` | `20261019T172819Z-662b17c9` |
| `dense-visible` | `20261019T173018Z-20ab40d6` |
| `dense-hidden` | `20261019T173229Z-824f0e72` |
| `dense-20hz` | `20261019T173440Z-3a6e8a63` |
| `dense-captures` | `20261019T173638Z-3107db6f` |
| `dense-full` | `20261019T173849Z-a1b25fdd` |
| `editor-cost` | `20261019T174129Z-4208ceeb` |
| `minimal-cost` | `20261019T174338Z-dae89ff9` |
| `dense-repeat-1` | `20261019T174532Z-f479e013` |
| `dense-repeat-2` | `20261019T174743Z-9a8fde34` |

From the repository root and recorded environment, the declared entry points are:

```text
python -m experiments.showcase_benchmark --plan
python -m experiments.showcase_benchmark --run
python -m experiments.showcase_benchmark --compare outputs/showcase_benchmark/20261019T172621Z-63aa7c17/index.json
```

`--plan` only describes the matrix. `--run` launches new serial trials.
Comparison under changed evaluator source is a new assessment of retained runs;
record it separately and do not overwrite the original result. The matrix and
corrected reassessment are complete. Dense strict-display acceptance remains
unmet; the next step is the bounded timing diagnostic described above.
