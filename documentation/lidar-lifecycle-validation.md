# Two-car lidar lifecycle isolation

Protocol declared before capture: compare always-visible and lifecycle variants,
then repeat lifecycle once. Each run has 288 frames at 60 Hz. A stationary ego
body sits behind a sensor at (0,0,1); a stationary 4x2x2 m target is centered at
(12,0,1). Z is up, units are metres, and +X points toward the target. No SUMO,
PPO, traffic exporter, vehicle instancing, or physical actuation is used.

The lifecycle target is hidden during [0,1), visible during [1,2), hidden during
[2,3), and visible from 3 seconds. The control remains visible throughout.
Both use the same Example_Rotary preset and stable-ID setting.

The primary metric is exact target-ID agreement on valid rays within +/-2 degrees
azimuth/elevation, an a priori interior-face ROI. Whole scans within 0.2 seconds
of visibility transitions/start/end are excluded in both variants. This isolates
steady identity behavior; it does NOT test transition latency or silhouette rays.
Each phase needs at least four scans; visible phases need 100 hits, 99% correct
IDs, <=10 cm scan p95 and <=20 cm maximum range error. Hidden phases need zero
valid ROI hits. Scan cadence must be 0.1 seconds within 100 ns. Gates do not
replace or relax the existing full-traffic protocol.

The hypothesis is that returns can correctly hit a newly visible target while
the renderer map lacks its ID even without our replay adapter. All variants and
the repeat are retained, whether they pass or fail. Sensor noise is not claimed
bitwise deterministic. Raw scans include exact object IDs, emitter/channel/echo
IDs, timestamps, flags and mapped labels. A geometry-derived label is never used
to fill missing renderer IDs. No checkpoint or training is involved.

Run from the repository root with the external Isaac installation:

```bat
C:\isaacsim\python.bat scripts\validate_lidar_lifecycle.py --variant always-visible
C:\isaacsim\python.bat scripts\validate_lidar_lifecycle.py --variant lifecycle
```

Each run creates its own `outputs/lidar_lifecycle/` package with source snapshots,
scene, configuration, runtime manifest and artifact hashes. Results are diagnostic,
not evidence that the original curved traffic lidar has passed acceptance.

## Observed results

`lidar-lifecycle-results.json` is generated from hash-verified raw captures by
`scripts/analyze_lidar_lifecycle.py`; offline summaries exactly match capture
summaries. The always-visible control passed with 34,402 checked hits and 100%
identity agreement. Its worst checked range error was 0.018565 m.

Both lifecycle runs passed geometry, hidden-target checks, scan cadence, and phase
coverage, but failed identity: 0% correct target labels in both visible phases.
They checked 14,393 and 14,420 visible-phase hits respectively. In both, the target
path was absent from the captured full/delta maps. The same nonzero raw ID
`37384897492231046530439738658` appeared on those unlabeled returns. Maximum checked
range error remained below 0.018565 m. All checked returns in all runs had echo ID
0. All runs had 48 scans and no callback errors.

This reproduces the labeling failure without the traffic exporter, instancing,
SUMO or PPO. It narrows the issue to the installed RTX sensor/annotator mapping
path or our minimal use of that API; it is not proof of a specific vendor defect.
It does not establish that every curved-replay range outlier is a labeling issue,
nor rule out multi-return effects elsewhere. The ROI and transition exclusions
were fixed before runs, and must not be transplanted into the full-traffic gate
to manufacture a pass. Timing here checks scan cadence, not dynamic pose timing.

Run IDs:

- Control: `20260923T153148Z-always-visible-7a6ed7`
- Lifecycle: `20260923T153233Z-lifecycle-443e9a`
- Repeat: `20260923T153253Z-lifecycle-389883`

Recompute without Isaac (substitute a run ID above):

```bat
.venv\Scripts\python.exe scripts\analyze_lidar_lifecycle.py outputs\lidar_lifecycle\RUN_ID
```

The analyzer accepts multiple run folders and `--output NEW_REPORT.json`, refuses
overwrites, and fails on artifact tampering or a mismatch with the captured
summary. Its exit success means analysis completed; inspect `summary.passed` for
sensor acceptance. Forty-two automated tests passed, including five new checks
for phase guards, reference geometry, unknown identities, empty runs, valid
controls and hidden-target false returns.

Next: use this small fixture to test documented map-refresh/initialization paths
or a separately approved runtime version. Keep the original full-traffic failures
visible and repeat its complete acceptance protocol after a candidate fix. No
runtime upgrade, external bug report, policy change, or sensor-control integration
was performed here.
