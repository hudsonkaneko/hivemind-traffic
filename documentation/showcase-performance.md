# Showcase performance — v06

Question: can the same 80-car scripted PhysX showcase deliver a smoother live
view by removing duplicate state reads, display edits and optional audit work,
without changing physical motion, control, safety thresholds or source assets?

## Predeclared comparison (2026-10-10)

- Preserve v04/v05 and all prior results. v06 is an opt-in version, not training.
- Seed 101, identical v05 fleet/vehicle/road, 120 Hz physics, 60 Hz control,
  10 Hz planner, same 35 mph target, same collision and command-expiry checks.
- Instrument v05 first. Compare v06 at the same 10 Hz display rate, then try
  20/30 Hz. GUI and headless are separate; headless still renders in this demo.
- Primary metrics: real-time factor, p95/max frame interval, named exclusive
  stage wall times; report startup separately. Keep every failed trial.
- Presentation mode samples native target-agreement and visual-agreement audits
  at 1 Hz (also first sample/frame). Every tick still reads measured body state,
  validates it, checks contact/clearance/road/attitude, and records a safety row.
  The controller still rejects invalid/stale object frames at its existing rate.
- Full evidence retains all original audit cadences and durable per-second
  chunks. Presentation retains every trajectory row but writes bulk records at
  exit, samples full object records at 1 Hz, and explicitly cannot claim durable
  recovery from a native process crash. No missing audit is counted as measured.
- Optimized native readback is valid only between commanded single-owner steps;
  never reuse a target as a measured observation. All displayed car body poses
  still update. Only decorative wheels farther than 150 m may skip spinning.
- Same-version repeat criterion: existing comparator, <=2 cm position and
  <=0.02 m/s speed difference, same passing/lane-change outcomes. (Corrected the
  initial transcription of its unchanged 0.02 tolerances before final trials.)
  Cross-version trajectory checks use <=1 cm and <=0.01 m/s, explicitly allowing
  version/display/audit fields to differ; they are not same-config qualification.
- Physical gates: existing zero-contact/overlap, >=0.6 m clearance, supported
  upright wheels, road containment, braking/hold, unchanged source/layer checks;
  >=6 passes and >=4 changes for 90-second driving runs.
- Final presentation timing gate: RTF 0.99–1.01, p95 accumulated lag <=0.1 s,
  maximum lag <=0.5 s, plus p95 frame interval <=1.5/display-Hz and max <=0.2 s.
  Test a 240-second GUI drive and a fresh matched repeat before qualifying it.
- Run GPU tests serially; no tests, source edits or document sync during measured
  windows. No driver changes, dependency installation or blind extension removal.

## Status

Implemented and physically verified on the recorded workstation. Two full GUI
drives repeated exactly. **Strict smooth real-time qualification remains open**:
both finished at approximately 1x but exceeded the predeclared lag/frame gates.
No acceptance threshold was relaxed. v04 remains the original lighter default.

## Measured development sequence

All entries below are GUI, seed 101, 80 backgrounds, unchanged source assets,
120 Hz physics, 60 Hz control, 10 Hz planning, and unpaced. Durations include
two settling and ten braking/hold seconds. They are development evidence, not
the final paced qualification. All physical/process gates passed.

| Run ID | Profile | Sim / active wall seconds | RTF | p95 / max display interval |
| --- | --- | --- | --- | --- |
| `20261011T002443Z-a0f8f55b` | Instrumented v05, full, 10 Hz | 102 / 125.7683 | 0.8110 | 161.1 / 316.5 ms |
| `20261011T002733Z-3587df67` | Initial v06 smoke, presentation, 10 Hz | 27 / 24.1913 | 1.1161 | 106.0 / 120.2 ms |
| `20261011T002818Z-3328188f` | Initial v06, presentation, 10 Hz | 102 / 101.5281 | 1.0046 | 118.3 / 280.9 ms |
| `20261011T003024Z-e9e68d7c` | Initial v06, presentation, 30 Hz | 27 / 43.2455 | 0.6243 | 67.3 / 85.7 ms |
| `20261011T003558Z-8f9e5990` | Exact-math v06, presentation, 20 Hz; external-load uncertainty | 27 / 35.6024 | 0.7584 | 83.4 / 238.4 ms |
| `20261011T003657Z-2f2285a4` | Exact-math v06, presentation, 10 Hz; competing GPU app | 102 / 126.0251 | 0.8094 | 135.7 / 354.7 ms |

The matched 90-second-drive pair reduced active wall time by 19.27%, with
16 passes/eight lane changes and 1.5693 m minimum clearance in both. This first
iteration was **not real-time qualified**: even its nearly 1x final RTF hides
2.33 seconds of accumulated lag earlier in the run. The 30 Hz trial could not
sustain real time. No slow trial is reclassified as a timing pass.

The baseline profiler attributed 34.05% to the render/app-update call, 16.77%
to physics, 10.21% to the controller and 9.18% to safety/metrics. Render includes
Kit/UI/scene synchronization, not exclusively GPU drawing. Whole-desktop GPU
sampling does not identify the individual render bottleneck. Initialization
was 13.91 seconds, separate from the live-loop timing.

The next exact-math optimization removes full route projection construction
when only circular station is needed, reuses command-local object geometry,
and hoists candidate-envelope constants without dropping any object or future
sample. The old controller remains the default unless v06 explicitly opts in.
2,000+ scalar projection checks, randomized candidate checks, complete synthetic
command sequences and 1,020 saved real object frames matched the legacy
calculation exactly. Saved object frames are 10 Hz, so that replay alone is not
a full 60 Hz controller or live physical qualification.

The final 90-second-drive trace also matched the baseline exactly at all 12,240
physics ticks, including applied controls/events. Its timing is **excluded from
isolated performance comparison**: desktop GPU memory grew to 10,347 MiB during
the run, and afterward the GPU remained at 92%/9,068 MiB with a game running.
The user closed the competing application; a fresh check showed 3% usage before
the long qualification below. No other application was closed by the agent.
The 20 Hz trial overlaps the period of changing desktop load; it is retained as
an exploratory timing failure, not a clean measurement of that rate's limit.

## Implementation boundaries

- `traffic/showcase_performance.py`: exclusive stage timing and explicit audit
  schedules. Final calibration uses the actual sequential-mark path; early runs
  above used an explicitly approximate empty-scope calibration. No corrected
  calibration has been backfilled into old results.
- `usd/showcase_fleet.py`: one immutable, validated measured fleet snapshot per
  completed physics tick, shared by consumers; invalidated before the next step.
  All 80 collision bodies and all displayed body transforms remain present.
- `visualization/loop_showcase_view.py`: one final camera/path update and an
  opt-in path toggle. Only distant decorative wheel spin is reduced; no collider
  or visible body is culled. References/layers and original vehicle assets remain.
- `scripts/physics_loop_showcase.py`: full-rate safety and trajectory sampling,
  separate sampled audit evidence, timing qualification and process cleanup.
- `experiments/loop_showcase_repeatability.py`: strict same-configuration/source
  repeats, explicitly distinguishing sampled from every-render visual audits.

Presentation stores complete 120 Hz ego/safety rows in memory until exit, 1 Hz
full object records, and 1 Hz optional native/visual agreement audits plus their
first samples. A native crash can lose unflushed presentation evidence. Full
mode preserves durable per-second checkpoints, 10 Hz objects, 120 Hz native
audits and checks every displayed frame. Neither profile disables contact,
clearance, road, wheel-support, attitude or command-expiry checks.

`passed` means physical/scene/process acceptance; `smooth_display_passed` is a
separate pacing/frame-interval gate. Sampled audit scope is included explicitly
in the result and repeatability report. `--headless` still renders here, and is
not a non-rendering physics benchmark. No LiDAR, SUMO, RL or V2V is added.

## Final GUI evidence and remaining timing gate

| Run ID | Outcome | Timing |
| --- | --- | --- |
| `20261011T004030Z-5e517202` | Paced presentation; 240 s drive + settle/stop; physical/scene/process pass | 252 / 252.0004373 s; p95 lag 1.6518 s, max lag 1.7569 s; p95/max frame 108.1/305.2 ms |
| `20261011T004825Z-591ff392` | Fresh identical repeat; physical/scene/process pass | 252 / 252.0005015 s; p95 lag 0.1941 s, max lag 0.3194 s; p95/max frame 104.9/311.1 ms |
| `20261011T004609Z-827df6f2` | Full-evidence 90 s drive with screenshots; all native/visual audits and durable chunks passed | 102 / 107.1692 s; 0.9518x unpaced, not a matched timing comparison because captures/full logging differ |

Both long runs completed **46 passes, 23 lane changes, 3,537.17 m, zero contacts
or overlap**, minimum clearance 1.5693 m, safe braking/hold and clean shutdown.
Repeat report `outputs/loop_showcase_repeatability/20261011T005532Z-1d7514f6`
validated source/output hashes, exact resolved settings, all 30,240 aligned
physical samples and 253 scheduled visual audits of 80 cars per run. Position
and speed differences were exactly zero; the two trajectory file hashes also
match. This is same-machine physical and **sampled** visual repeatability, not
every-render visual qualification or cross-platform determinism.

Full-evidence mode independently checked native target agreement on all 12,240
ticks, 1,020 displayed frames, and preserved 102 durable chunks. The 35-second
follow screenshot was inspected: prepared ego, surrounding block cars, lane
markings and adaptive cyan reference remain visible and aligned. Screenshots
remain local under that run; no source asset or screenshot was published.

The first long profile spent 105.17 s inside render/app-update, 53.39 s in
physics, 25.90 s in control and 19.26 s in safety/metrics. Reducing diagnostic
work alone does not eliminate render/UI synchronization cost. Initial v05/v06
native readbacks fell from 39,785 to one per completed frame plus initialization
(12,241 for a 102-second run); every safety sample still uses measured state.

### Failure controls

| Run ID | Fixture | Result |
| --- | --- | --- |
| `20261011T005305Z-8cf43413` | 80-car command loss, 15 s drive | Passed expiry-triggered full braking and stopped fleet/ego hold |
| `20261011T005343Z-05160e73` | Four-across blocked, 15 s drive | Retained failure: no collision/change, but the last-ten-second approach window exceeded the <=7.4 m/s following criterion |
| `20261011T005415Z-ff57de4e` | Four-across blocked, 90 s drive | Passed following-speed gate; no passes/changes, 6.9928 m minimum clearance |
| `20261011T005502Z-8b6826f7` | Deliberate contact positive control | Passed contact detection and clean shutdown; intentionally ends early, not collision-avoidance evidence |

### Reproduce

Run at repository root in the configured traffic Python environment. Isaac Sim
and the separately supplied Americano v07 are local prerequisites.

```text
python scripts/demo_loop_showcase.py --version v06
python scripts/demo_loop_showcase.py --version v06 --evidence full --drive-seconds 90 --unpaced --capture
python scripts/demo_loop_showcase.py --version v06 --mode blocked --drive-seconds 90 --unpaced
python -m experiments.loop_showcase_repeatability --first outputs/loop_showcase/20261011T004030Z-5e517202 --second outputs/loop_showcase/20261011T004825Z-591ff392
```

The IDs in the comparison command refer to local ignored evidence, not files
provided by a fresh clone. Supply two new run folders on another machine.
Use `--render-hz 20` or `30` only as experimental overrides; they are not qualified
defaults. Full evidence is intentionally more expensive. Avoid concurrent GPU
applications when benchmarking.

### Next measured optimization

Keep the remaining work focused on tracing render/app-update sub-stages before
further changes. Investigate UI/scene synchronization and immutable visual
instancing separately; preserve wheel access, colliders, references and physical
motion. Do not reduce physics/safety cadence, claim sensor performance from
this sensor-free demo, or change persistent application settings to pass a test.

This measurement-first approach follows NVIDIA's
[Isaac Sim performance handbook](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/reference_material/sim_performance_optimization_handbook.html)
and [rendering performance guidance](https://nvidia-omniverse.github.io/omniverse-performance/reference/rendering-performance.html).
The existing minimal-render profile already avoids high-quality path tracing;
the installed standalone experience disables its main-loop limiter by default.
That configuration alone is not proof of the precise render stall cause.

## Regression and documentation checkpoint

Final general suite: **1,483 passed, 140 skipped, 18 subtests passed**. Separate
USD fleet/view suite: **76 passed**. Targeted cross-profile/repeat evaluator
suite: 170 passed. The viewer and reproducible-experiment workflows kept scene
ownership, explicit evidence cadence and measured acceptance separate; no result
was promoted solely on an attractive frame or aggregate RTF.

Google guide tab **35 Showcase performance**, `t.dwsxfv73kedy`, mirrors the
changes, definitions, commands and open timing gate. Native readback verified
exact content, headings, four shaded Consolas command blocks and preservation
of all 35 older tabs. No Google page-layout/pixel audit is claimed. The original
source assets and raw evidence remain local; Git contains code/tests/notes only.
