# Initial visibility diagnostic

Predeclared study: run an instrumented lifecycle baseline, then two
initially-visible captures. Only initial visibility changes: the target remains
visible through 2 seconds, hides from 2 to 3 seconds, then reappears. No preflight,
camera product, runtime upgrade, policy change, or acceptance relaxation.

Hypothesis: initial visibility determines stable-ID registration; hiding and
reappearing afterward preserves correct labels and hidden-target behavior.
Use the existing range, identity, phase-coverage, cadence and negative-control
thresholds. Both candidate repeats must pass before claiming a fixture solution;
this is not validation of late-created objects or the full traffic replay.

Instrumentation saves every received sensor map buffer, including callbacks
without populated scans, cumulative per-scan maps, effective stable-ID settings
at startup/attachment, and composed descendant visibility at callback time.
USD visibility is not asserted to be the renderer's effective visibility.
Raw events remain hashed in each unique run package. No checkpoint applies.

## Results

All three captures completed with 48 scans and no callback errors. Offline
hash verification, full 128-bit label reconstruction and summary recomputation
passed for every package; generated results are in
`lidar-initial-visibility-results.json`.

| Run | Checked visible returns | ID agreement | Hidden-phase hits | Accepted |
| --- | ---: | ---: | ---: | --- |
| `20260927T193657Z-lifecycle-sensor-1b46bb` | 14,403 | 0% | 0 | No |
| `20260927T193814Z-initially-visible-sensor-805f07` | 18,409 | 100% | 0 | Yes, fixture only |
| `20260927T193847Z-initially-visible-sensor-d61faf` | 18,396 | 100% | 0 | Yes, fixture only |

The baseline's effective stable-ID setting was true at startup and attachment.
Its 96 recorded map buffers contained no target path. Maximum checked range
error across these runs stayed below 1.86 cm. All 67 automated tests passed.

Interpretation: initial visibility, not hiding and reappearing in general, is
the distinguishing factor in this fixture. This supports an initialization or
map-registration hypothesis but does not locate the fault inside the renderer
versus our API usage. It is not a production fix: making a car appear earlier
changes the intended traffic scenario. The normal viewer remains unchanged.

## Technology and design choices

USD time samples describe visibility; RTX generates returns and object IDs;
StableIdMap connects exact IDs to scene paths. Python/NumPy preserve raw buffers
and check an independent analytic box reference. Cumulative map snapshots close
the prior evidence gap when maps arrive without populated scans. Composed USD
visibility is recorded for comparison, not treated as proof of RTX state.

Only the initial visibility changed. Reproducibility guidance required retaining
the failing baseline and both repeats, with unchanged thresholds. The Isaac
validation guidance keeps this a sensor fixture, not a claim about vehicle physics.

## Portable reproduction

Activate the repository's Python environment and configure the external Isaac
runtime as described in `portable-demos.md`. Run each command on one line.

```text
python -c "import subprocess,sys; from hivemind.launcher import ROOT,load_config,isaac_runtime; sys.exit(subprocess.call([str(isaac_runtime(ROOT,load_config(ROOT))),*sys.argv[1:]],cwd=ROOT))" scripts/validate_lidar_lifecycle.py --variant initially-visible
python scripts/analyze_lidar_lifecycle.py outputs/lidar_lifecycle/YOUR_RUN_ID
python -m pytest -q
```

Repeat the first command once. Replace `initially-visible` with `lifecycle` for
the baseline, which is expected to exit nonzero on this runtime. These captures
are headless diagnostics, not GUI demos. Raw scans and logs are not in GitHub.

Next experiment: compare explicit visibility updates with authored time samples
for the initially-hidden case; then test a fresh map attachment after appearance.
Full curved-replay geometry and identity acceptance remains outstanding.

Interview explanation: "I isolated initial visibility as the variable that
changes label registration, reproduced the result twice, and checked that the
candidate still stopped detecting the target while it was hidden. I did not
present a changed scenario as a general fix."

## Google Docs sync

Synced tab: **23 Initial visibility and lidar registration**. It includes this entry's
purpose, technology, protocol, results, commands and limitations. Associate it
with the commit titled `Investigate initial visibility in lidar ID registration`.
The native document was reachable, but its prescribed trusted-read helper rejected
the absolute Windows workspace path (`workspaceRoot must be an absolute path`).
The later explicit-visibility task used direct native connector reads and writes
to sync this entry without changing existing tabs. The original helper issue
remains; it does not mean the document connector is unavailable.
