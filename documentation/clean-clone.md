# Clean-clone readiness

October 15, 2026. Goal: prove named entry points from tracked files and a fresh
traffic environment, not promise that every historical experiment runs on every PC.

## Supported profiles

| Profile | Requirements | Validation boundary |
| --- | --- | --- |
| Automated CPU/SUMO | Windows x64, Python 3.11, requirements-ci.txt (SUMO/TraCI 1.27.1) | Fresh installation, full CPU tests, real SUMO reset/step/close/completion/repeat tests |
| Public loop showcase | Separate standalone Isaac Sim, supported NVIDIA GPU/driver; tested runtime 6.0.1-rc.7 | Procedural block main car, PhysX movement, scripted background cars; bounded native checks recorded below |
| Prepared main car | Above plus separately supplied local prepared vehicle package | Existing default; NOT included in Git |
| Trained/replay/ovrtx/Lab | Trusted compatible checkpoint, recordings and/or separately configured runtimes | Optional research paths, not certified by the CPU workflow |

Linux/macOS, other Python versions, other Isaac versions and GPU configurations
are not yet supported by this validation matrix. An RC runtime may not be readily
available to new users; qualify an obtainable stable release separately rather
than assuming identical APIs. Portable syntax is not universal compatibility.

## Install and check (Windows, from your clone root)

Install Git and Python 3.11 x64 first. Use a new environment; never install these
requirements into Isaac's Python. No activation or PowerShell policy change is required.

```text
py -3.11 -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements-ci.txt
.venv\Scripts\python.exe -m pip check
.venv\Scripts\python.exe -m pytest -q
.venv\Scripts\python.exe -m hivemind demo sumo --headless --check
.venv\Scripts\python.exe -m hivemind demo sumo
```

The SUMO wheel supplies executables in the environment. If discovery needs an
explicit path, set `sumo_binary` / `sumo_gui_binary` in ignored `hivemind.local.json`.
`requirements.txt` remains the existing pinned traffic stack; the CI profile adds
the matching SUMO binary package. These pins are not a complete hash-locked
transitive environment; retain `pip freeze`/installation reports with validation.
An inherited `PYTHONPATH` can override the installed TraCI package. Clear legacy
Python/SUMO overrides when diagnosing a fresh environment and inspect
`traci.__file__`; do not assume the interpreter alone establishes isolation.

## Public Isaac showcase

Install Isaac separately through NVIDIA's distribution and configure `isaac_root`
in `hivemind.local.json` (copy `hivemind.example.json`, fill only needed fields).
It must contain `python.bat` and `VERSION`. `ISAAC_SIM_PATH` is also supported.
No dependencies are installed into Isaac and no assets are downloaded automatically.

```text
python scripts/demo_loop_showcase.py --version v09 --vehicle public --traffic-count 12 --check
python scripts/demo_loop_showcase.py --version v09 --vehicle public --traffic-count 12 --drive-seconds 90
```

The public car reuses the installed NVIDIA PhysX Factory sample through the
existing composed scene builder. Repository code generates references, geometry
and physics layers into a unique per-run output package. No vendor source or
private Americano asset is redistributed in Git. Road source remains tracked;
the main car's model, dimensions, source hash and motion owner are recorded.
This simpler car has different dynamics from the prepared vehicle: comparisons
must match vehicle profiles. Neither profile establishes a calibrated real car.

`--check` is deliberately **filesystem preflight only**: missing required inputs
return status 2, but a pass does not prove imports, composition, driver support,
sensor timing or physical behavior. The native run performs those applicable
driving/scene/lifecycle gates. This showcase has no LiDAR or learning enabled.
The prepared profile remains the default; use `--vehicle prepared` explicitly
when presenting the separately supplied car. Restart requests preserve that choice.

## Continuous integration and separate GPU qualification

The **Clean clone** GitHub Actions workflow runs on pushes to main/codex branches
and pull requests. It installs the pinned profile on a fresh hosted Windows
runner, verifies dependency consistency and real SUMO availability, runs SUMO
integration tests (not mocks), then runs the full CPU suite. Optional native
USD/Isaac tests remain reported skips. XML reports are retained for 14 days even
on failure. Actions are pinned to commit SHAs; token permissions are read-only.

A green CPU job does **not** certify the Isaac viewer. GPU checks stay manual,
bounded and serial on this workstation. No self-hosted runner was installed;
do not run arbitrary pull-request code on a personal GPU runner. GUI response,
60 FPS, long-session memory growth, large fleets, RTX sensing and Lab training
have separate acceptance gates. Keep previous evidence intact.

## Evidence and remaining limits

Validated source: `8b01a93585dc5ad318466e1a68fb446ffbcf616b`,
[PR 11](https://github.com/hudsonkaneko/hivemind-traffic/pull/11), pending review
and merge. This is a verified Windows CPU/SUMO and bounded public-car readiness
checkpoint, not certification of all optional experiments.

| Check | Actual result |
| --- | --- |
| Fresh GitHub clone + new venv | Installation and pip check passed; no private vehicle directory or local configuration present |
| Fresh-clone full suite | 1,605 passed, 180 optional skips, 34 subtests, 101.38 s |
| Independent hosted Windows job | 1,605 passed, 180 skipped, 34 subtests, 121.82 s; four separately invoked real SUMO tests passed |
| Existing USD-only regression suite | 134 passed, 8.319 s |
| Public headless smoke | 27 simulated seconds; all 30 driving/scene gates and clean process exit passed |
| Two public GUI/control-smoke runs from the fresh clone | 102 simulated seconds each; all 32 driving/scene/sidebar gates, cleanup and source immutability passed |
| Repeatability | Exact equality of all 12,240 trajectory rows and traffic lifecycle records |
| Relocation | Generated scene reopened in a different directory; 19 layers, no missing dependencies, every resolved dependency inside copied package; structure validator passed |
| Missing private asset | Preflight rejected it with the public-profile remedy, without starting Isaac |

The [hosted CI evidence](https://github.com/hudsonkaneko/hivemind-traffic/actions/runs/38765009932)
is independent of this PC. Local clean-clone tests explicitly cleared inherited
`PYTHONPATH`, `SUMO_HOME` and `SUMO_BINARY`; readback confirmed Python, TraCI,
sumolib and SUMO came from the new environment, not the installed system SUMO.
Python was 3.11.9; executable/client versions were SUMO/TraCI 1.27.1. The first
resolved install included sumo-data 1.28.0 as allowed by eclipse-sumo's dependency;
the retained pip report records the full resolution. Transitive pins are not frozen.

The two GUI tests used the regular editor, 120 Hz physics / 60 Hz controls /
30 Hz requested display cadence, seed 101, with captures enabled. Native control
models requested 80 -> 12 -> 40 -> 12 background cars and speed/gap changes.
The trace stayed collision-free, in-road, upright and supported on all wheels;
minimum clearance was 1.81118 m and final speed approximately 6.51e-8 m/s.
Maximum achieved speed was 12.41955 m/s, not a sustained 35 mph qualification.
No passes or lane changes occurred in these control-smoke runs; do not cite them
as public-car overtaking validation or mouse-interaction latency tests.

Active-time ratios were 0.978916x and 0.999999x real time. Both **failed** strict
soft-real-time lag criteria (p95 lag 5.990 s and 1.142 s; worst displayed-frame
intervals 0.231 s and 0.225 s). The first run overlapped CPU installation/testing;
captures also affect timing. No performance improvement or 60 FPS claim is made.
Native checks reused this workstation's Isaac 6.0.1-rc.7, RTX 5070 / 12 GiB and
driver 610.88; they are not evidence of a fresh Isaac installation on another PC.

### Local evidence inventory (ignored; not shipped with Git)

Under `outputs/clean-clone-20261015-a/`:

- `checkout/`: isolated branch clone at the validated commit with a new `.venv`.
- `install.log`, `install-report.json`, `tests.xml`: install/resolution/test evidence.
- `relocated-scene/`: reopened composed public smoke stage, not a flattened export.
- `checkout/outputs/loop_showcase/20261015T160100Z-5eef61d2/` and
  `20261015T160403Z-1216adfc/`: repeated GUI runs with source hashes, resolved config,
  trajectories, lifecycle/control events, captures and process/GPU measurements.

Both trajectory files have SHA-256
`1d7da1db65cccbaf1aaa9077a877216bce31c3ee32f71b5b3add0f0e0bff136f`.
The earlier canonical smoke is
`outputs/loop_showcase/20261015T155740Z-cfada461/`.

Retained failures: `outputs/clean-clone-development/tests.xml` records the first
CPU run's temporary-folder assumption failure; that fixture now supplies its own
mock checkout rather than depending on pytest's output location. Native attempt
`outputs/loop_showcase/20261015T155604Z-5a4a7f0f/` failed before driving because the
behavior isolation guard required a real vehicle-source inspection. The public
Factory source is now inspected on an unattached stage before isolation and the
complete scene is inspected again before stepping; the guard was not weakened.

### Documentation synchronization

Google engineering guide tab **39 Clean-clone readiness** was added with purpose,
technologies, portable setup/demo commands, definitions, test outcomes and limits.
Native readback verified exact inserted text, seven title/heading paragraphs,
ten shaded monospace command paragraphs, and unchanged contents/titles/order of
all 39 previous tabs. Rendered pagination was not inspected. No training,
runtime upgrade, private asset publication or recovery-archive change occurred.

Sources: [GitHub Python CI](https://docs.github.com/en/actions/tutorials/build-and-test-code/python),
[official SUMO distributions](https://sumo.dlr.de/docs/Downloads.php).
