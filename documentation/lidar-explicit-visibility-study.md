# Explicit visibility diagnostic

Predeclared protocol: one fresh time-sampled lifecycle baseline, two explicit
lifecycle runs, and one explicit initially-visible control. Keep the sensor map,
geometry, timing, 0.2-second phase guards, 99% ID requirement, range limits and
hidden-return negative controls unchanged. No preflight or runtime upgrade.

Hypothesis: writing visibility defaults directly on appearance/disappearance
triggers registration updates that authored time-sample animation does not.
Poll the actual timeline before each application update and author only changes.
This can differ by one update at transition boundaries; record all write times
and do not evaluate transition-adjacent scans. No time samples are authored in
explicit mode. Two passes are needed to accept the candidate for this fixture.
Retain both repeats even on failure. No training or production viewer changes.

## Results

The fresh time-sampled baseline and both explicit lifecycle runs failed with
0% target-ID agreement in both checked visible phases. Geometry, hidden-target,
coverage and cadence gates passed. The explicit initially-visible control passed
with 100% visible-phase identity agreement and zero hidden-phase hits.
All four captures had 48 scans and no callback errors; raw artifact hashes,
full-ID labels and summaries were independently recomputed offline.

Run IDs, in declared order:

- `20260928T052103Z-lifecycle-sensor-65ca49`
- `20260928T052206Z-lifecycle-sensor-3e0c5a`
- `20260928T052237Z-lifecycle-sensor-295f1f`
- `20260928T052325Z-initially-visible-sensor-4ebb94`

Generated evidence: `lidar-explicit-visibility-results.json`. The two explicit
lifecycle runs wrote visibility changes at 0, 1.0000, 2.0167 and 3.0167 seconds.
Floating-point timeline accumulation delayed the later transitions by one
update, within the predeclared 0.2-second guards. No transition samples were
used to manufacture a pass. All 75 automated tests passed.

## Interpretation and technology

Changing from USD time samples to direct default-value writes does not repair
late-visible identity registration in this fixture. This weakens animation-only
change-notification as an explanation, but does not prove a vendor defect.
The control confirms that direct writes can hide and restore an already-mapped
target without false hidden returns. There is no accepted production fix.

The new `--visibility-mode explicit` is opt-in, checks the live timeline every
update, writes only actual visibility transitions, and preserves those events
in each hashed run. The original time-sampled default remains unchanged.
Preflight cannot be combined with this mode, keeping competing changes separate.
Python unit tests cover exact schedule boundaries; RTX captures test actual
sensor behavior. Neither alone substitutes for the other.

## Portable reproduction

Activate the traffic environment in the repository root and configure the
external runtime as described in `portable-demos.md`. Paste each command on
one line. Run serially with other Isaac applications closed.

```text
python -c "import subprocess,sys; from hivemind.launcher import ROOT,load_config,isaac_runtime; sys.exit(subprocess.call([str(isaac_runtime(ROOT,load_config(ROOT))),*sys.argv[1:]],cwd=ROOT))" scripts/validate_lidar_lifecycle.py --variant lifecycle --visibility-mode explicit
python scripts/analyze_lidar_lifecycle.py outputs/lidar_lifecycle/YOUR_RUN_ID
python -m json.tool documentation/lidar-explicit-visibility-results.json
```

Repeat the capture once; omit `--visibility-mode explicit` for the baseline;
use `--variant initially-visible` with explicit mode for the control. Replace
YOUR_RUN_ID with the printed folder. These are headless tests, not GUI demos.
Validation failure deliberately exits nonzero. Raw scans are local, not GitHub.

Next separate diagnostic: fresh map attachment after appearance, without
changing geometry or visibility. The normal viewer, PPO policy and external
runtime remain untouched; full curved-replay validation remains outstanding.

Interview explanation: "I tested whether animation versus explicit updates
caused missing labels. Both failed only for initially-hidden objects, while
an initially-visible control passed. This eliminated a plausible workaround
without weakening range or hidden-object checks."

Google Docs: tab 24 accompanies this change; tab 23 also syncs the prior commit
e293acf. The trusted-read helper remained incompatible with Windows paths;
direct native connector read/write and readback were used instead.
