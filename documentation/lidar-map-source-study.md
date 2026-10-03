# Independent render-product identity diagnostic

Predeclared before capture: compare the existing lifecycle sensor-map baseline
with a separate 64x64 camera render product's StableIdMap and StableIdMapDeltas.
Question: is the missing late-visible target mapping specific to the sensor
render-product path? This is exploratory diagnosis, not a production fix.

Run order: lifecycle/sensor, lifecycle/camera, lifecycle/camera repeat,
always-visible/camera control. All four results are retained. The fixture,
visibility schedule, scan timing, ROI, phase guards and acceptance thresholds
remain unchanged. No runtime installation, policy, exporter or recording changes.
The new switch defaults to sensor, preserving the previous capture path.

The camera map is read after each app update; callbacks use only mappings already
received, never future mappings. Both the original sensor labels and selected
labels are saved. IDs must match all 128 bits exactly; no high-bit masking,
geometry-based relabeling or unknown-point deletion is permitted. Camera map
updates and the scene are retained in each unique run package. A camera's presence
may change renderer scheduling; this experiment alone cannot prove a vendor bug.

Primary gate: the existing 99% target identity agreement in both steady visible
phases, alongside existing geometry, coverage, hidden-return and cadence gates.
If the candidate passes twice and the control passes, investigate the full curved
replay using its original gates. A small-fixture pass is not full validation.

Technology reference: the installed Replicator `test_stableIdMap` demonstrates
attaching a StableIdMap annotator to a camera render product. NVIDIA's
[RTX annotator documentation](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/sensors/isaacsim_sensors_rtx_annotators.html)
and local parser require full 128-bit IDs and allow unknown mappings. A warning
about unknown procedural IDs does not identify the cause of this lifecycle issue.

## Follow-up declared after the first two camera captures

Both camera captures still lacked the late-visible target. Next test 12 explicit
`rep.orchestrator.step(delta_time=0.0, pause_timeline=True)` calls while the target
has a temporary visible session opinion, before attaching the capture writer.
This differs from the earlier rejected paused `app.update()` preflight: the
orchestrator explicitly requests rendered frames. Restore visibility and time
zero, verify the authored visibility schedule, then capture the unchanged test.
Do not include initialization scans, and retain the renderer-supplied preflight
map with its provenance. Test lifecycle twice if the first passes. This is a
known-scene registration workaround candidate, not a general solution for
objects first created during a running simulation.

## Results: neither candidate is an accepted fix

The generated `lidar-map-source-results-complete.json` contains six hash-verified runs.
All completed with 48 scans, valid cadence and no callback errors. The camera
source failed twice: both steady visible phases had 0% identity agreement, while
distance, visibility and coverage checks passed. The fresh sensor baseline also
failed at 0%. The target path never appeared in either map for those runs.
Both always-visible camera controls passed with 34,398 and 34,390 hits and 100%
identities. The second control was inadvertently launched while the first shell's
completion was uncertain; both completed captures are retained, not cherry-picked.
Checked maximum range error in these five captures was below 1.86 cm.

Explicit rendered preflight registered the exact target ID and produced 100%
visible-phase agreement, **but failed the hidden-target gates**: 4,001 and 3,992
valid interior hits appeared in phases where the target should be invisible.
The authored USD schedule and timeline reset checks passed, so these do not
establish renderer state equivalence. This preflight is rejected and remains
opt-in only to reproduce the failure; it is not installed in the normal viewer.
No repeat or full-route promotion followed because its acceptance gate failed.

| Capture | Immutable run ID | Accepted? |
| --- | --- | --- |
| Sensor lifecycle | `20260924T163218Z-lifecycle-sensor-efd302` | No |
| Camera lifecycle | `20260924T163355Z-lifecycle-camera-d35145` | No |
| Camera repeat | `20260924T163434Z-lifecycle-camera-678f98` | No |
| Camera always-visible | `20260924T163455Z-always-visible-camera-f95271` | Yes, limited fixture only |
| Extra camera control | `20260924T163528Z-always-visible-camera-97bc2e` | Yes, limited fixture only |
| Rendered preflight | `20260924T163626Z-lifecycle-camera-ec1d1e` | No, hidden-target false returns |

This narrows the failure beyond the sensor writer alone, but does not prove a
specific vendor defect. A map appearing after initialization is insufficient if
visibility behavior changes. Full curved-replay geometry/identity failures remain
unresolved; no lidar-derived driving observations or further PPO training began.

## Reproduction and learning

Activate the traffic environment in the repository root and configure isaac_root
in hivemind.local.json (as in documentation/portable-demos.md). Run each full line
separately. Change `--map-source camera` to `--map-source sensor` for the baseline,
repeat the camera lifecycle once, and change the variant to `always-visible` for
the control. These are headless validation captures, not GUI demos.

```text
python -c "import subprocess,sys; from hivemind.launcher import ROOT,load_config,isaac_runtime; sys.exit(subprocess.call([str(isaac_runtime(ROOT,load_config(ROOT))),*sys.argv[1:]],cwd=ROOT))" scripts/validate_lidar_lifecycle.py --variant lifecycle --map-source camera
python scripts/analyze_lidar_lifecycle.py outputs/lidar_lifecycle/YOUR_RUN_ID
python -m json.tool documentation/lidar-map-source-results-complete.json
python -m pytest -q
```

Replace YOUR_RUN_ID with the capture's printed directory name. To reproduce the
rejected preflight specifically, append `--render-preflight` to the first command.
Do not use it as an initialization recommendation. Capture failure exits nonzero;
successful offline analysis is not sensor acceptance. Raw scans remain ignored.

The offline analyzer now reconstructs labels from the full raw 128-bit ID and
saved renderer maps, rejecting altered labels even when geometry looks correct.
It also verifies saved sensor-only labels alongside the selected camera labels.
All 66 automated tests passed, including a new test rejecting missing keys,
wrong paths and low-bit-only matches. All six raw packages passed hash and
offline-summary checks. Existing historical packages remain unmodified.

Next safe investigation: distinguish USD-to-renderer visibility update behavior
from map registration using an explicit per-frame visibility control, without
changing sensor gates. An alternate runtime comparison would need a separately
chosen compatible installation; this work did not upgrade external dependencies
or submit an upstream report.

Interview explanation: “I tested independent renderer maps and initialization
as competing hypotheses. One workaround appeared to repair labels but created
returns from hidden objects, so I rejected it using unchanged negative controls.”
