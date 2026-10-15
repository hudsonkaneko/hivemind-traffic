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

Implementation checkpoint: validation in progress. Record actual clean-clone,
native and remote-CI results here before calling this milestone verified.

Sources: [GitHub Python CI](https://docs.github.com/en/actions/tutorials/build-and-test-code/python),
[official SUMO distributions](https://sumo.dlr.de/docs/Downloads.php).
