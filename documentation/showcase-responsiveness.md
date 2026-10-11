# v07 responsiveness study

Status: implemented; physical/repeat and 30-Hz average real-time checks pass.
Strict hitch-free display and 60 FPS remain open, October 10, 2026 (Pacific).
Preserve v04-v06, all historical results and the supplied vehicle source. This
is a sensor-free scripted showcase, not training or a research traffic result.

## Predeclared scope

Keep the same 80 kinematic backgrounds, dynamic PhysX ego, seed 101, 35 mph
target, 120 Hz physics, 60 Hz control, 10 Hz planning and full-rate safety gates.
Separate optional display/statistics work from these physical responsibilities.
Use process-local settings only; no runtime, driver or dependency installation.

1. Add opt-in v07 and a minimal project-owned Kit showcase experience. Retain
   the existing editor experience as an explicit comparison option.
2. Cache immutable camera/path metadata and update statistics at 5 Hz. Preserve
   scene composition, ownership repair, measured transforms and all colliders.
3. Measure application-event service intervals separately from simulation/frame
   time. An event-loop service probe is not measured OS mouse-to-photon latency.
4. Smoke test 30 Hz, then 60 Hz only after physical/scene/process checks pass.
   Compare fresh matched profiles, holding settings fixed apart from the named
   display/runtime intervention. Keep failures and uncertain results.
5. Run two fresh 240-second GUI drives at the best candidate setting and verify
   source/configuration hashes and physical repeatability. Run full-evidence,
   command-loss, blocked and contact-positive controls separately.

## Acceptance (before simulator trials)

- Existing physical/scene/process thresholds remain unchanged. No extra hidden
  physics ticks, altered source assets, missing collision bodies or lost IDs.
- Candidate 30/60 Hz: RTF 0.99-1.01, p95 frame interval <=1.1/requested Hz,
  max frame interval <=0.1 s, p95 accumulated lag <=0.1 s, max lag <=0.5 s.
  Completed native application frames, not DLSS-generated frames, are counted.
- Event-loop service probe p95 <=50 ms and max <=100 ms. End-to-end hover/click
  qualification requires actual interaction and is reported separately.
  NEW_FRAME counts are native GPU-completion notifications: the installed
  RenderingManager permits zero/one/multiple deliveries per app update. They
  are not independently verified unique scanout frames. This study's display
  gate concerns app pacing/notification delivery, not monitor/input latency.
- Same-configuration repeats: existing <=0.02 m and <=0.02 m/s tolerances and
  identical pass/lane-change outcomes; no threshold relaxed to pass a trial.
- Inspect actual rendered captures, dynamic/static layer boundaries and clean
  shutdown. Headless throughput does not qualify interactive GUI performance.

Potential follow-ups (not automatic): trace renderer internals, measured worker
count comparison, immutable geometry instancing, or isolated Fabric-delegate
repair. Do not blindly enable the previously crashing delegate or asynchronous
rendering, or reduce sensing/physics validity to improve a display metric.

## Development trials

- First launch `20261011T011520Z-fd51d8c5` failed dependency resolution before
  physics; retained. Added runtime-resolved installed extension search paths.
- Minimal app `20261011T011548Z-615c13bb` passed physical/scene/process checks,
  but 27 simulated seconds took 40.529 s (0.666x); native frame deliveries were
  20 Hz, not requested 30. Hydra-order bracket p50 was 37.08 ms; UI bracket
  0.56 ms. This disproves editor panels as the dominant cost by themselves.
- Next controlled development change: disable gatherRenderResults for the
  single-viewport showcase only, as the installed base app documents this as a
  potential performance choice with multi-viewport artifact risk. Keep async
  rendering disabled and verify visuals/native clock; do not generalize to
  sensors or multiple viewports. Earlier run retains the strict setting.
- The no-gather trial `20261011T011738Z-c530f7d3` passed physical checks, still
  below real time (0.718x). It also captured screenshots, so this is not an
  isolated timing comparison. Next: compare 8/32 process worker counts, then
  opt-in immutable wheel/body geometry sharing without changing collisions.
- The 32-worker trial `20261011T011938Z-a1e6576c` passed physical/scene/process
  checks but was slower: 27 simulated seconds / 43.994 wall seconds (0.614x).
  Retain the 8-worker default. Its source-integrity check passed: it finished
  before the subsequent geometry edit, despite its result being polled later.
- V07 now uses explicit references and native instances only for immutable
  visual subtrees: 400 instance roots share five prototypes across 80 cars.
  All 1,280 composed render primitives remain. Editable spin pivots, IDs, all
  80 colliders and original entry/physics/source-geometry bytes are preserved.
  Wheel geometry gains a documented `/Geometry` child, never a physics path
  migration. Seventy-nine focused USD tests passed before runtime trials.
- Added opt-in bounded native CPU/GPU profiling to distinguish scene update
  work from actual drawing or waiting. Profiling trials and user-paused runs
  are excluded from smooth-display qualification; event intervals do not
  establish actual click-to-photon latency.
- Instanced native-profile trial `20261011T012513Z-fa125aae` passed all
  physical/scene/process gates, with 27 simulated seconds in 24.772 wall
  seconds. This instrumented, unpaced trial is **not** FPS qualification.
  Its 60-update diagnostic successfully captured CPU/GPU zones and restored
  prior settings. One late sample showed 12.95 ms in the inclusive Hydra render
  bracket versus 0.88 ms in the GPU RTX root zone; these API snapshots are not
  guaranteed to refer to the same frame. The whole-run Hydra event bracket
  median was 11.22 ms. This supports scene synchronization/work as the next
  bottleneck, not an assertion that GPU utilization alone diagnoses the cause.
  A short CPU-test overlap occurred at diagnostic startup; clean timing trials
  must run without concurrent tests/profiling.

## Regression checkpoint

Final general suite: **1,575 passed, 179 skipped, 18 subtests passed**. USD fleet/view
suite: **115 passed** using the USD environment's built-in unittest runner (that
environment does not install pytest). No dependencies were installed. A parallel
evaluator test initially hit a shared pytest temporary-directory collision;
the completed general suite includes every evaluator test and passed.
After read-only review, fixed profiling during pause, excluded headless runs
from GUI qualification, and kept sampled v07 repeatability labels explicit.
The complete targeted timing/profiling/repeat/comparison suite then passed
**262 tests**. No physical or controller equations changed in those repairs.

## Uninstrumented GUI trials

| Run | Setting | Physical/scene/process | RTF | Native delivery | Display gate |
| --- | --- | --- | --- | --- | --- |
| `20261011T012749Z-655ffebd` | 30 Hz, 8 workers, 15 s drive | Pass | 0.999995x | 30.04 FPS | Pass, short smoke only |
| `20261011T012834Z-ecd5c85b` | 60 Hz, 8 workers, 15 s drive | Pass | 0.83409x | 50.08 FPS | Fail |
| `20261011T012953Z-fe714ccc` | 60 Hz, 4 workers, 15 s drive | Pass | 0.85072x | About 51 FPS | Fail |
| `20261011T013100Z-97850f75` | 30 Hz, 8 workers, 240 s drive, development checkpoint | Pass | 0.999998x | 30.004 notifications/s | Fail: 141.40 ms worst notification interval |
| `20261011T013722Z-f3410868` | Final-source 30 Hz, 8 workers, 240 s drive | Pass | 0.999998x | 30.004 notifications/s | Fail: 126.00 ms worst notification interval |
| `20261011T014156Z-5131a70f` | Same-source repeat, 30 Hz, 240 s drive | Pass | 0.999999x | 30.004 notifications/s | Fail: 127.97 ms worst notification interval |

All three retain the same 1.58363 m minimum clearance, one completed lane change,
zero contacts and clean source/process checks. The 30 Hz smoke had p95 native
delivery interval 34.65 ms, maximum 97.66 ms, and p95 application-service interval
34.61 ms. Sixty requested updates per simulated second is not sixty delivered
frames per wall second: 60 Hz slowed the experiment, so it remains experimental.
The 8-worker 30 Hz default is retained for longer matched trials.
The development full drive achieved 46 passes, 23 lane changes and 1.56927 m
minimum clearance. Its p95 notification interval was 34.78 ms, but the rare
maximum exceeded the unchanged 100 ms gate. A 108.45 ms visual-publication
outlier was recorded; do not infer its precise cause (GC/OS/USD) without a trace.
Final same-source repeats follow the review fixes above; this earlier run is
retained, never rewritten to look like a final-source result.
The first final-source run also completed 46 passes and 23 changes with zero
contacts and 1.56927 m minimum clearance. P95 notification interval was 34.72 ms,
application-service p95 34.66 ms, and peak whole-desktop GPU memory 4,025 MiB.
Startup to live clock took 7.34 s; this is a single start, not a startup SLA.
End-of-process RSS was about 4.30 GiB; peak about 4.50 GiB. Those two samples do
not establish long-term memory-growth stability. All raw outputs remain local.

Strict repeat assessment `outputs/loop_showcase_repeatability/20261011T014658Z-4ae80450`
passed: **30,240 aligned physics samples, zero position/speed difference**,
identical 46 passes/23 changes and 253 scheduled visual agreement audits per run.
It verifies this same-source/configuration/runtime pair, not cross-platform
determinism. The second run's p95 notification interval was 34.56 ms and peak
desktop GPU memory 4,028 MiB. The 126–128 ms maximum intervals in both runs keep
the strict 100 ms smoothness gate open; the threshold was not relaxed.

The bounded cross-profile evaluator also passed against the preserved v06 run
`20261011T004825Z-591ff392`: 30,240 aligned samples, **zero position/speed
difference and exact control decisions**. It permits only declared display/app
and immutable-visual changes, requires the original fleet entry/physics/geometry
hashes unchanged, and lists every allowed source/asset difference. This is a
matched physical comparison, not a claim that v06/v07 have identical sources.

## Commands and boundaries

### Full evidence and controls

| Run | Test | Outcome |
| --- | --- | --- |
| `20261011T014828Z-168a9ad4` | Full evidence, 90 s drive, GUI/unpaced/captures | Pass: 12,240 native audits, 3,060 display audits, 16 passes/8 changes, zero contacts, durable chunks and clean shutdown |
| `20261011T015050Z-d4f69476` | Driver-command expiry, 15 s drive, headless/unpaced | Pass: expiry fallback applies full braking and holds; no collision |
| `20261011T015120Z-31a91cc9` | Four-across blocked traffic, 90 s drive, headless/unpaced | Pass: no passes/changes, following-speed gate and 6.99277 m minimum clearance |
| `20261011T015204Z-f9ac4d2a` | Intentional contact positive control, headless/unpaced | Pass: real contact detected and clean shutdown; deliberately ends early, not avoidance evidence |
| `20261011T015214Z-b864e267` | Optional editor app, 15 s drive, GUI/unpaced | Physical/scene/process pass; 27 simulated seconds in 24.440 wall seconds, not paced GUI qualification |

The full-evidence run took 108.738 wall seconds for 102 simulated seconds and
is not a real-time qualification. Its 55-second follow capture was visually
inspected: ego, colored block cars, road markings and cyan adaptive path remain
present. Source assets and screenshots stay local. Headless control tests are
excluded from GUI qualification. Driver-command expiry is not a V2V-network or
LiDAR-failure test. The installed close-time global-time warning was retained
in logs; no runtime Error/Fatal or USD-reference leak warnings were present.

### Reproduce

Use the configured traffic Python from the canonical repository root; Isaac is
discovered by the supervisor. The separately supplied Americano asset remains
a local prerequisite, not something included by cloning GitHub.

```text
python scripts/demo_loop_showcase.py --version v07
python scripts/demo_loop_showcase.py --version v07 --ui-profile editor
python scripts/demo_loop_showcase.py --version v07 --evidence full --drive-seconds 90
python scripts/demo_loop_showcase.py --version v07 --render-hz 60 --drive-seconds 15
python scripts/demo_loop_showcase.py --version v07 --profile-render --drive-seconds 15
python -m experiments.loop_showcase_repeatability --first outputs/loop_showcase/20261011T013722Z-f3410868 --second outputs/loop_showcase/20261011T014156Z-5131a70f
```

The last command names local evidence, not files supplied by a fresh clone.
Sixty Hz is experimental; profiling is diagnostic-only. A GUI run, a headless
run, an editor run and a sensor-enabled experiment are not interchangeable
performance evidence. The minimal app's no-gather setting is guarded to one
viewport; no async rendering or previously crashing Fabric delegate is enabled.
User-paused and headless runs cannot qualify GUI timing.

Next narrow performance work: trace the rare visual-publication outlier and
remaining Hydra synchronization before another renderer change. Do not suppress
GC, alter physical rates, drop collision bodies, or enable Fabric blindly to
make the counter look better. No sensor/memory-soak or actual click-latency
qualification follows from this sensor-free presentation study.

The viewer and reproducible-experiment workflows kept clean composition,
unchanged motion authority, source snapshots, failure controls and explicit
measurement scope. NVIDIA's [performance handbook](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/reference_material/sim_performance_optimization_handbook.html)
and [rendering guidance](https://nvidia-omniverse.github.io/omniverse-performance/reference/rendering-performance.html)
informed the bounded, measured trials; the installed API/tests supplied the
exact extension and notification behavior for this runtime.

## Documentation and Git checkpoint

Google engineering guide tab **36 Responsive showcase**, `t.jndashuyzqn1`,
contains the purpose, technology/definitions, measured limits, run IDs and five
shaded Consolas launch commands. Native API readback verified the expected text,
headings and command styling, with all **36 older tabs unchanged**. This is
native structure/style verification, not a Google page-layout pixel audit.
See the [learning guide](https://docs.google.com/document/d/1qeYucQnP4nznmLNJrlUz1hCIcY7KOigRPuhBVH-Zo0c/edit?tab=t.jndashuyzqn1)
and [draft PR #10](https://github.com/hudsonkaneko/hivemind-traffic/pull/10).
Only this milestone's code/tests/documentation are staged; unrelated existing
vehicle/road notes are left in the working tree. No source assets, screenshots,
installed dependencies or raw runtime evidence are published.
