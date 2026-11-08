# Showcase validation milestone — October 16, 2026

October 19 continuation: three native startup smokes and the legacy live-control
regression passed. After a remote-admission repair, a separately reviewed
[bounded gap-creation controller](showcase-gap-creation.md) passed the original
endpoint scenario twice, including all four targets and subsequent passing.
The [11-case performance matrix](showcase-benchmark-20261019.md) is complete:
all physical checks passed, and eight comparisons are valid after a separately
reviewed packaging-path correction. The two light cases pass strict display
timing; all nine dense cases fail. Original comparison failures are preserved.
See the [measured results and original screenshots](showcase-validation-20261019.md).
The October 16 implementation checkpoint and its evidence remain below.
Branch: `codex/showcase-validation`.
This presentation milestone preserves the order in [the active roadmap](hybrid-roadmap.md).

## October 19 engineering-guide synchronization

The dated continuation is synchronized to the existing Google guide's
[40 Showcase validation tab](https://docs.google.com/document/d/1qeYucQnP4nznmLNJrlUz1hCIcY7KOigRPuhBVH-Zo0c/edit?tab=t.lz1t1b5s17js).
Final native readback at **2026-10-19 18:14:15 UTC** passed all 207 checks.
All 41 tab identities, titles and order remain unchanged; the other 40 tab
objects and all 45 preexisting target-tab elements retain exact native equality.
The append contains six native tables, two command blocks with six shaded
command paragraphs, and 15 links including the six original screenshots.

The guide links implementation checkpoint
`7940a9757356fa2dcd6790cc8c1769e0d0ba1fbc`, whose two Windows CPU/SUMO
clean-clone jobs passed: [run 39299088357](https://github.com/hudsonkaneko/hivemind-traffic/actions/runs/39299088357)
and [run 39299079995](https://github.com/hudsonkaneko/hivemind-traffic/actions/runs/39299079995).
The [compact sync record](evidence/showcase-guide-sync-20261019.json) references
the local before/after readbacks and verifier by hash. Native text, inherited
effective styles, links and preservation were checked; PDF pagination and
rendered page fit were not inspected. Raw guide snapshots remain ignored locally.
The guide preserves the failed dense timing qualification and original rejected
comparisons alongside their separate corrected reassessment.

## Purpose and implementation

The four requested areas are bounded window/sidebar readiness, native live-control
validation, measured descriptive traffic statistics, and matched display costs.
One public procedural car is PhysX-owned; background cars are scripted kinematic
bodies. Inputs remain privileged simulator state and a known map. There is no
LiDAR, SUMO integration, learned policy, new training, or asset migration here.

- [Startup readiness](showcase-startup.md): record native and viewport dimensions,
  require stable layout and visible-sidebar docking within a bounded wait, and
  retain renderer failures. Explicit process-local sizing is a mitigation, not a
  demonstrated cause of the historical 120-by-0 swapchain error.
- [Live controls](showcase-control-validation.md): opt-in native-widget endpoint,
  rapid-request, pause/resume, transition, and subsequent passing checks.
- [Statistics](showcase-statistics.md): measured speed/gaps, acceleration/jerk,
  near-miss proxies, per-lifetime time loss and exposure-qualified fairness.
  Closed-loop laps are not completed trips.

## Predeclared native checks

Use the public vehicle throughout. The prepared vehicle has different dynamics
and cannot substitute into a matched pair. Seed 101; 120 Hz physics, 60 Hz
controls, 10 Hz planner; eight CPU workers; default viewport resolution and
camera. Preserve all contact, clearance, road, wheel-support, command-freshness,
scene-composition, source-integrity, and shutdown gates.

First run three fresh 15-second driving smokes with viewport captures, checking
each image and stopping on failure. These are separate from the performance
matrix so capture work does not contaminate its baseline. After they pass, run
the 180-second endpoint scenario and a same-configuration repeat; retain either
failure. Each run has
the existing two-second settle and ten-second final braking phase. Pause checks
require unchanged native physics clock, pose, traffic state and active settings
while the UI pumps; pending changes apply after resume. Scenario transition
deadlines are declared in its module. Protected vehicles are never removed just
to reach a density target. Passing after completed transitions is a separate gate.

## Predeclared performance matrix

Every matrix run uses 90 seconds driving, 2 seconds settle and 10 seconds braking,
paced GUI, statistics enabled, public vehicle and seed 101. No user control
changes, pauses, native profiling, CPU test overlap or other simulation jobs are
allowed in a qualified timing comparison. GPU memory remains below the existing
90% stop guard. Desktop GPU samples, competing Kit process counts and CPU/RAM
counters before and after each trial are retained. This observes desktop load;
it cannot rule out transient or non-Kit work. One sample per changed variant
supports descriptive differences, not a statistical or causal improvement claim.

| Case | Version / app | Active background cars | Panel | Display | Recording |
| --- | --- | ---: | --- | ---: | --- |
| light-visible | v09 editor | 4 | visible | 30 Hz | presentation |
| light-hidden | v09 editor | 4 | hidden | 30 Hz | presentation |
| dense-visible | v09 editor | 80 | visible | 30 Hz | presentation |
| dense-hidden | v09 editor | 80 | hidden | 30 Hz | presentation |
| dense-20hz | v09 editor | 80 | hidden | 20 Hz | presentation |
| dense-captures | v09 editor | 80 | hidden | 30 Hz | presentation + captures |
| dense-full | v09 editor | 80 | hidden | 30 Hz | full evidence |
| editor-cost | v07 editor | 80 | legacy controls | 30 Hz | presentation |
| minimal-cost | v07 minimal app | 80 | legacy controls | 30 Hz | presentation |
| dense-repeat-1 | v09 editor | 80 | visible | 30 Hz | presentation |
| dense-repeat-2 | v09 editor | 80 | visible | 30 Hz | presentation |

Pair only cases differing in the declared display/panel/recording factor; compare
the v07 app pair independently. Density tiers are separate physical scenarios,
not a matched intervention. V09 retains all 80 native pool slots at low density.
The three dense-visible runs also exercise repeated process/window startup.

Matched comparisons require identical captured source hashes, runtime identity,
physical settings/assets and per-tick control decisions, position/speed differences
at most 0.02 m / 0.02 m/s, no live requests and all physical/scene/process gates.
Timing success remains separate: RTF 0.99–1.01, p95 lag <=0.10 s, max lag <=0.50 s,
native-delivery p95 <=1.1/requested-Hz, maximum <=100 ms, event-service p95 <=50 ms
and max <=100 ms. NEW_FRAME notifications are not verified monitor scanouts or
mouse-to-photon latency. Failure does not authorize relaxing these thresholds.
No 60 Hz claim or trial is part of this matrix.

## Running the checks

Use the configured traffic Python from the repository root. The public launcher
selects the separate Isaac runtime; do not install traffic packages into it.

```text
python scripts/demo_loop_showcase.py --version v09 --vehicle public --drive-seconds 15 --capture
python scripts/demo_loop_showcase.py --version v09 --vehicle public --control-validation endpoints --drive-seconds 180
python -m experiments.showcase_benchmark --plan
python -m experiments.showcase_benchmark --run
python -m experiments.showcase_benchmark --compare outputs/showcase_benchmark/<run-id>/index.json
```

The first two commands create one run each; repeat as declared above. The
benchmark's default and `--plan` modes only print the matrix. `--run` executes
all eleven cases serially, writes the index after each state change, and refuses
to launch while other Kit processes are present or their inventory is unavailable.
It never closes another application. Physical, source, resource or cleanup
failures stop subsequent cases; a strict timing failure alone does not.

The evaluator checks the benchmark index hash, per-run source/output hashes,
declared configurations, physical gates, control/trajectory equivalence and
raw timing samples. Its overall `passed` means all eight comparisons are valid;
inspect `both_strict_display_qualified` separately for timing success. Missing,
stopped or altered runs cannot turn an incomplete matrix into a pass.

Independent agent review found no remaining actionable defects after the
integration fixes. CPU fixtures cover startup fault injection, control validation,
metric calculations and invalid benchmark evidence; native execution remains
the required next level of evidence.

## October 16 evidence and synchronization

Run results and source snapshots belong under unique ignored `outputs/` paths.
Initial workstation inspection found three other project viewers and a separate
GPU validation job active, with 10,366 / 12,227 MiB GPU memory and 81% sampled
utilization. Native checks wait for that competing work to clear; this observation
is not a Highway Sim performance measurement.

First integrated CPU regression: **1,660 passed, 180 optional skips, 96 subtests**
in 84.91 seconds; report `outputs/showcase_validation/cpu-20261016T180610Z/tests.xml`.
Final frozen-source regression: **1,712 passed, 180 optional skips, 98 subtests**
in 98.70 seconds; report
`outputs/showcase_validation/cpu-final-20261016T181237Z/tests.xml` and its
`cpu-checks.json` source-hash/runtime metadata. The benchmark's 50 CPU tests
are included. Its CLI plan prints the declared eleven cases without launching
Isaac; the retained plan and read-only runtime prerequisites are in that folder.
`git diff --check` passed. These ignored raw reports are local, not GitHub assets.
The endpoint command's filesystem-only preflight also passed; it establishes
neither native imports nor runtime behavior. Native execution is still pending.

Failed runs remain part of the record. Existing private changelog edits remain
local and unstaged. Implementation commit `3a8cea4` is published in
[draft PR 12](https://github.com/hudsonkaneko/hivemind-traffic/pull/12). Independent
review verified all 21 committed files, 62 recorded source/test hashes and the
test-report hash; its local record is `independent-review.md` beside that report.
GitHub clean-clone CI was in progress at this documentation checkpoint.

The Google engineering guide's **40 Showcase validation** tab is
[synced](https://docs.google.com/document/d/1qeYucQnP4nznmLNJrlUz1hCIcY7KOigRPuhBVH-Zo0c/edit?tab=t.lz1t1b5s17js)
and native-readback verified. All 40 older tabs, including order, body and styles,
remain exactly unchanged. The new tab contains 43 paragraphs, seven headings,
six shaded command blocks and verified source/PR links. The guide records CPU
success and native work blocked, without granting new runtime qualification.
Verification is retained in `outputs/doc-sync/showcase-validation/verification.json`;
PDF pagination was not inspected.
