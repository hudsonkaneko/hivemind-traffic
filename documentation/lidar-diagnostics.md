# Curved lidar failure diagnostics

## Outcome and scope

This is a diagnostic milestone, **not a sensor acceptance pass or a driving-policy
change**. Five exploratory 600-frame captures investigate the strict failures from
`replay-lidar-validation.md`. Every capture uses the same seed-2001 recording,
network, checkpoint, sensor preset, and acceptance thresholds. Each produces 100
scans, with the declared first three excluded. Short runs deliberately fail the
470-evaluated-scan coverage gate. Their identity percentages must not be compared
as improvements over the earlier full 48.3-second runs.

SUMO still owns recorded vehicle motion. Isaac renders kinematic playback. Lidar
does not feed the PPO policy. No external runtime, normal viewer, portable exporter,
training configuration, or original recording was changed.

## Hypotheses and experiments

Each experiment was selected before its capture; the sequence is exploratory,
not a pre-registered statistical comparison. One capture per variant is insufficient
to estimate random sensor variation or claim a causal improvement.

- **Baseline:** current capture with all diagnostic switches off.
- **Identity deltas:** request `StableIdMapDeltas` alongside the full map and retain
  exact renderer-provided ID/path updates even between accumulated scans. The map
  did not acquire the later background vehicles in this test.
- **Zero azimuth noise:** override only `azimuthErrorStd` to zero in the session
  layer. This is not a completely noiseless sensor: range accuracy, resolution,
  geometry, motion and other preset settings remain. Large errors persisted.
- **Identity preflight:** temporarily author all known vehicles visible, execute
  12 paused updates, clear those opinions, and verify visibility against every
  recorded timestamp before capture. This did not fix the map. No paused scans
  were allowed into the dataset. It is retained solely as a reproducible rejected
  diagnostic, not a recommended startup procedure.
- **Uninstanced vehicles plus deltas:** turn off `instanceable` only in the session
  layer while retaining the same referenced geometry. Compared with the deltas
  experiment, this tests the instancing hypothesis. It did not fix the map.

The new switches default off. The strict 10 cm per-scan p95, 20 cm maximum range
error, 99% identity agreement, timing and coverage requirements are unchanged.
Unmapped IDs remain unknown: no nearest-vehicle relabeling, high-bit masking,
point deletion, or acceptance-threshold relaxation was introduced.

## Diagnostic technology

`scripts/audit_replay_lidar.py` first verifies capture artifact hashes, then
recomputes ideal intersections from raw scans and the recorded trajectory. It
reports an expected-versus-observed vehicle confusion matrix and the ten largest
range errors. The unknown/nonvehicle category combines missing IDs and known road
or ground hits; it must not be interpreted as a missing-ID count by itself.

For each return over 20 cm error, the audit measures the ideal intersection's
distance to the closest box edge. In vehicle-local coordinates, the smallest
distance to a face is approximately zero; the second-smallest distance is the
distance along that face to an edge. This is a diagnostic of boundary proximity,
not proof of the physical cause and not a filter used by acceptance.

The generated `lidar-diagnostic-results.json` contains all five summaries, resolved
configs, capture commands, manifest hashes, audit-source hashes, confusion matrices
and outlier examples. Raw scans remain local under `outputs/replay_lidar_validation/`.
Each capture has frozen source snapshots, runtime details, input hashes, and output
hashes. The changing exploratory implementation is recorded rather than presented
as five captures from one clean commit.

In the zero-azimuth-noise capture, 52 returns exceeded 20 cm error; only 24 of
those ideal intersections were within 2 cm of a box edge. The maximum range error
was 8.337 m and the largest edge distance among outliers was 0.462 m. Thus neither
“all outliers are angular noise” nor “all are silhouette hits” is supported. These
descriptive values come from the generated report, not an acceptance exception.

## Run IDs and reproduction

From Command Prompt in the repository root, use the external Isaac installation:

```bat
C:\isaacsim\python.bat scripts\validate_replay_lidar.py --frames 600
C:\isaacsim\python.bat scripts\validate_replay_lidar.py --frames 600 --identity-deltas
C:\isaacsim\python.bat scripts\validate_replay_lidar.py --frames 600 --zero-azimuth-noise
C:\isaacsim\python.bat scripts\validate_replay_lidar.py --frames 600 --identity-preflight
C:\isaacsim\python.bat scripts\validate_replay_lidar.py --frames 600 --identity-deltas --uninstance-vehicles
```

Use your own Isaac installation path if different. Execute captures serially.
Nonzero exits here represent failed strict gates; inspect the summary and callback
errors to distinguish a completed failed check from a runtime crash.

| Variant | Run ID |
| --- | --- |
| Baseline | `20260922T214334Z-c40102` |
| Identity deltas | `20260922T213716Z-cf8b12` |
| Zero azimuth noise | `20260922T213823Z-8f762b` |
| Identity preflight | `20260922T213956Z-e569ac` |
| Uninstanced plus deltas | `20260922T214205Z-5bd21a` |

Recompute a read-only audit, without launching Isaac:

```bat
.venv\Scripts\python.exe scripts\audit_replay_lidar.py outputs\replay_lidar_validation\20260922T213823Z-8f762b
.venv\Scripts\python.exe -m pytest -q
```

The audit also accepts multiple run directories and `--output NEW_REPORT.json`.
It refuses to overwrite an existing report. Audit success means the analysis ran,
not that the captured sensor passed; read each `capture_summary.passed` value.

## Verification and next experiment

All 37 automated tests passed. New tests check face/edge distances, the replay
coordinate calculation, unknown-label preservation, read-only analysis and hash
tampering rejection. All five sensor captures finished without callback errors,
passed timing, and preserved the four shared USD layers byte-for-byte; all still
failed strict geometry/identity acceptance and intentionally failed short coverage.

Next: isolate a two-vehicle appear/disappear case to distinguish renderer map
lifecycle behavior from our replay adapter, and capture emitter/echo metadata to
check the nearest-surface assumption. Investigate non-edge outliers as well as
silhouette hits. Do not describe azimuth noise or silhouettes as a complete cause.
After a fix, rerun the full route twice plus negative controls with unchanged
acceptance gates. Runtime upgrades and upstream issue submission are not part of
this change. Sensor-based policy observations remain gated on validation.

## Learning explanation

An ablation changes a suspected cause and asks whether the failure remains. A
failed attempted fix is useful evidence when its inputs and outputs are preserved.
Here, an offline audit and controlled diagnostic switches narrow the problem
without making the test easier. An accurate interview explanation is: “I built
reproducible sensor diagnostics, preserved unknown identities and large errors,
and separated code correctness from sensor acceptance instead of claiming that a
plausible point cloud proved autonomous driving.”

Official API context: [NVIDIA RTX sensor annotators](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/sensors/isaacsim_sensors_rtx_annotators.html)
documents that some returned IDs lack a path entry. This caveat alone does not
establish the cause of our dynamic-map failure.
