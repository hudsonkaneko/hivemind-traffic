# Live lidar validation results — 2026-09-28

Ten completed 25-second driving runs met their individual acceptance gates. All raw
scans and manifests were re-audited. The native noisy-profile suite nevertheless
**fails its stricter 1 mm repeated-clearance gate**; that result is preserved.
The separate ideal-sensor diagnostic passes without changing the tolerance.

## Measured runs

| Run ID | Mode / profile | Speed / initial gap | Minimum gap (m) | Max distance error (m) | Wall seconds | Driving gate |
|---|---|---|---:|---:|---:|---|
| 20260928T164635Z-stop-015d41 | stop / native | 4 m/s / 20 m | 5.220011 | 0.02130661 | 113.3 | PASS |
| 20260928T164834Z-stop-29694a | stop / native | 8 m/s / 40 m | 5.225281 | 0.03708954 | 112.4 | PASS |
| 20260928T165032Z-stop-9d1c0c | stop / native | 12 m/s / 65 m | 5.226433 | 0.05476074 | 114.2 | PASS |
| 20260928T165231Z-follow-932f68 | follow / native | 6 m/s / 30 m | 5.228514 | 0.02978973 | 109.9 | PASS |
| 20260928T165426Z-follow-f8819f | follow / native | 6 m/s / 30 m | 5.228499 | 0.02979164 | 106.3 | PASS |
| 20260928T165618Z-stop-6245a9 | stop / native | 8 m/s / 40 m | 5.225281 | 0.03708954 | 106.7 | PASS |
| 20260928T165810Z-stop-20b37e | stop / native + stale injection + GUI | 8 m/s / 40 m | 5.225280 | 0.03708954 | 141.4 | PASS |
| 20260928T170056Z-stop-999c83 | stop / native | 8 m/s / 40 m | 5.225281 | 0.03708954 | 111.6 | PASS |
| 20260928T170253Z-follow-48dec9 | follow / ideal | 6 m/s / 30 m | 5.219781 | 0.00000610 | 96.0 | PASS |
| 20260928T170432Z-follow-33d787 | follow / ideal + GUI | 6 m/s / 30 m | 5.219781 | 0.00000610 | 129.3 | PASS |

Every run ended at zero speed, had no recorded collisions, retained a gap above
5.2 m, and had zero ego speed overrides above the 0.15 m/s threshold. Total driving
coverage: 2,500 traffic steps (250 traffic seconds). This is a small deterministic
fixture study, not a safety certification or statistical generalization claim.

## Repeatability finding

| Largest per-step difference | Native profile | Ideal diagnostic |
|---|---:|---:|
| Ego position (m) | 0.000043614705404593224 | 0 |
| Speed (m/s) | 0.0004336039225263377 | 0 |
| Lidar clearance (m) | 0.00146484375 | 0 |
| Gap (m) | 0.000043614705404593224 | 0 |

The native profile includes angular and range error settings. Removing those
settings in a separate diagnostic produced identical recorded control/state traces
across one headless and one GUI run. This supports the sensor-noise explanation,
but does not guarantee cross-machine or cross-version determinism. The native
profile remains the default; its original repeat-gate failure has not been erased.
The separate ordinary-stop repeat also stayed within the original limits: maximum
clearance difference 0.000606536865234375 m, position difference
0.0000013663904709915187 m, speed difference 0.0000031789143879468185 m/s.

## Fault handling and automated checks

51 automated tests pass. Coverage includes real-SUMO step timing and deterministic
trajectories, lifecycle cleanup, sensor filtering, stale/missing data braking,
control bounds, artifact tampering, offline metric recomputation, and report gates.

At injected step 40, the GUI stop run logged `sensor-failsafe` and requested
4.38 m/s from 4.68 m/s, realized by SUMO. The car was already braking there;
this checks fault-path execution, not a difference from its ordinary action.
A separate unit test verifies that stale data brakes even when a healthy clear-road
scan would otherwise maintain cruise speed. Acquisition timeout freezes traffic
and fails the run rather than stepping with an old scan.

## Evidence and reproducibility

- Full machine-readable audits: [live-lidar-results.json](live-lidar-results.json).
- Commands, architecture and limitations: [live-lidar-demo.md](live-lidar-demo.md).
- Original gates and diagnostic amendment: [live-lidar-plan.md](live-lidar-plan.md).
- Native suite: `outputs/live_lidar/suite-20260928T164635Z.json`.
- Diagnostic suite: `outputs/live_lidar/suite-20260928T170056Z.json`.
- Each run has raw scans, telemetry, source snapshots, hashes, versions, and dirty-tree status.
- Run manifests record SUMO 1.27.1, Isaac Python 3.12.13, RTX 5070 and driver 610.74.
- GUI captures inspected: stop `20260928T165810Z-stop-20b37e/preview.png`,
  following `20260928T170432Z-follow-33d787/preview.png`, under `outputs/live_lidar/`.
- Raw runs and logs are ignored by Git; concise reports and source are committed.

## Retained failures and scope

The initial noisy repeat audit exits nonzero, intentionally. An earlier 3-second
exploratory run in the previous folder failed the final-stop gate because it ended
before stopping; it was not counted among these ten runs. An old-folder batch was
interrupted when the current canonical-workspace instructions were applied. Its
files were left intact; all ten reported runs were freshly generated in `highwaysim`.

This milestone demonstrates live lidar-based speed control with SUMO road-following,
not PPO training, lidar steering, wheel physics, two independent autonomous agents,
or wall-clock real time. Rendering uses sample-and-hold acquisition with distinct
sensor and traffic clocks. The lead is scripted. Known label/curved-replay issues
are not claimed fixed by this fixture. Existing waypoint/Isaac migration files were
left unchanged. Google Docs was not edited under the current local-only documentation
scope. The work is published on `codex/live-lidar-baseline`, not merged into `main`.
