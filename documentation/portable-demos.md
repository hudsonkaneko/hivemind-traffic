# Portable demonstration commands

Open a terminal **in your clone's root** (the directory containing README.md).
Use Python 3.11+ for the dependency-free launcher. `python` below means that
interpreter; use `python3` if your operating system gives it that name.
No absolute username or installation path belongs in these shared commands.

If `python` is not on PATH (as in the local verification terminal), activate the
existing traffic environment once per terminal. In Windows Command Prompt:

```text
.venv\Scripts\activate.bat
```

In Linux bash: `source .venv/bin/activate`. In PowerShell:
`.\.venv\Scripts\Activate.ps1` (subject to your execution policy). Activation
is optional: `.venv\Scripts\python.exe -m hivemind ...` on Windows or
`.venv/bin/python -m hivemind ...` on Linux also works without changing PATH.
For first-clone environment creation you still need an installed bootstrap
Python: use `py -3.11` on Windows when available or `python3` on Linux.

```text
python -m hivemind demo sumo
python -m hivemind demo random
python -m hivemind demo trained
python -m hivemind demo lidar
python -m hivemind demo replay
python -m hivemind demo ovrtx
```

| Demo | What it does | Required locally |
| --- | --- | --- |
| sumo / random | Live curved-road PettingZoo rollout in SUMO GUI; no training | Traffic Python packages and SUMO GUI |
| trained | Same GUI using a selected shared-PPO checkpoint; inference only | Above plus a trusted compatible 16-input checkpoint |
| lidar | Static three-box Isaac RTX lidar smoke test | Isaac installation and supported RTX hardware/runtime |
| replay | Existing traffic recording played in Isaac with lidar | Isaac plus recording.json and network.net.xml |
| ovrtx | Standalone still RGB render at 2 seconds, not a live viewer or lidar | ovrtx environment plus those replay inputs |

Append `--check` to any demo to check paths and Python module availability without
running the simulation, loading a checkpoint, or initializing the GPU. This is
not a GPU/driver, model-compatibility, or sensor-accuracy test.

## First-clone setup

Cloning Git does not install dependencies or include ignored results. Install
SUMO separately using its supported installer/package manager. Follow README.md
for this project's pinned Windows setup; do not assume all runtime packages
support every operating system or Python version.

Create the traffic environment with `python -m venv .venv`, then install:

Windows (Command Prompt or PowerShell):
```text
.venv\Scripts\python.exe -m pip install -r requirements.txt
```

Linux (where the pinned packages are supported):
```text
.venv/bin/python -m pip install -r requirements.txt
```

The launcher chooses `.venv/Scripts/python.exe` on Windows and `.venv/bin/python`
on POSIX systems. For ovrtx it uses `.venv-ovrtx` instead. If that environment
does not exist it checks the Python running the launcher (useful for an already
activated environment). To create the separate ovrtx environment, use
`python -m venv .venv-ovrtx`, then run its Python with
`-m pip install -r visualization/requirements-ovrtx.txt`.
Do not install traffic dependencies into Isaac's bundled Python.

Install Isaac separately and configure its root below. Its external Python
launcher is `python.bat` on Windows or `python.sh` on Linux. We do not install,
upgrade or download proprietary/GPU runtimes automatically. Runtime support
and GPU requirements still apply; portable command syntax is not universal
hardware compatibility.

## Local configuration and discovery

Copy `hivemind.example.json` to `hivemind.local.json` with your editor or file
manager, then fill only the fields you need. The local file is Git-ignored.
Use forward slashes in JSON paths (or escape backslashes). Values are paths,
not shell commands; do not embed extra quotes or arguments. Relative paths are
resolved against the repository root, not the current terminal directory.

- `traffic_python`: optional full path to a traffic Python executable.
- `ovrtx_python`: optional full path to a separate ovrtx Python executable.
- `isaac_root`: installation directory containing Isaac's Python launcher.
- `checkpoint`: trusted, compatible local `best.pt` for trained/record demos.
- `sumo_binary`, `sumo_gui_binary`: optional headless and GUI executables.

Each field also accepts an environment variable named `HIVEMIND_` plus the
uppercase field, e.g. `HIVEMIND_ISAAC_ROOT`. Environment values override JSON.
Isaac also accepts `ISAAC_SIM_PATH`. SUMO discovery tries the configured
GUI/headless field, then `SUMO_BINARY`, PATH, SUMO_HOME/bin, and conventional
Windows Program Files locations. If you set SUMO_BINARY explicitly, it must
match the selected GUI/headless demo; unset it to allow normal discovery.
Invalid explicit paths fail clearly rather than silently choosing another tool.

## Checkpoints and replay inputs

There is no published model download bundled in this feature. Obtain a trusted
checkpoint produced by the current project or train one using the existing
`experiments/README.md` workflow. Training is a separate, potentially long task.
Old 9-input checkpoints do not work with the current 16-input environment.
The existing loader uses `torch.load(..., weights_only=False)`: **never load
untrusted checkpoint files**. The setup check only verifies existence.

Select a checkpoint once in local configuration, or supply it for one run:

```text
python -m hivemind demo trained --checkpoint results/training/YOUR_RUN/checkpoints/best.pt
python -m hivemind demo record --checkpoint results/training/YOUR_RUN/checkpoints/best.pt
```

Replace YOUR_RUN with your actual run directory. `record` runs a headless trained
rollout and generates `outputs/sumo_replay/recording.json` and `network.net.xml`.
It refuses to replace either existing input unless you explicitly add
`--overwrite`; back up recordings you want to retain first. No recording is
silently regenerated by `replay` or `ovrtx`. Their missing-input errors point to
the record command. Do not run recording and rendering simultaneously.

## Controls, outputs and limits

```text
python -m hivemind demo sumo --seed 42 --delay 200
python -m hivemind demo trained --headless --horizon 400
python -m hivemind demo lidar --frames 1800
python -m hivemind demo replay --frames 2900
python -m hivemind demo ovrtx --time 15
```

Seed/horizon/delay apply to traffic (record uses seed, but its existing recorder
has its own horizon). Frames applies to Isaac demos. Time applies to ovrtx.
Default seeds are 42 for scripted/random and 2001 for trained/record. Run one
demo at a time and let it finish; Ctrl+C interrupts a stuck run. SUMO GUI closes
at episode end; Isaac closes at its frame budget. Initialization can be slow.

Rollouts write new folders under results/rollouts; lidar smoke tests write new
outputs/lidar_smoke folders. Replay refreshes generated USD layers, scan samples,
preview and replay_report under outputs/sumo_replay. ovrtx refreshes shared
generated layers and its render, PNG and manifest under outputs/ovrtx. Open
`outputs/ovrtx/traffic_2s.png` (or traffic_15s.png) with your usual image viewer.
The launcher does not automatically open the PNG. Source and weights are unchanged.

This is a standard-library dispatch layer, not a new simulator, policy, sensor
fix or packaging of Isaac. It uses argument arrays and a fixed repository working
directory, keeps runtime environments separate, and returns the child exit code.
Files remain in their original domain folders. A visible point cloud does not
mean the strict lidar acceptance tests pass; lidar still does not control PPO.

## Verification and learning exercise

Local verification for this change: all 65 automated tests passed. Setup checks
passed for sumo, trained, lidar, replay and ovrtx. After CMD activation, the exact
`python -m hivemind` entry point completed scripted and trained headless rollouts
with a four-step horizon (seed 42 and 2001 respectively). These short runs test
dispatch and checkpoint loading, not driving performance. The new evidence is
under results/rollouts; no GUI or GPU demo was rerun for this launcher change.
The local configuration was confirmed ignored by Git. A bare `python` was not
on PATH before activation, which is why the instructions include activation.

Launcher tests cover runtime layouts, local settings, spaces in paths, dependency
failures, missing artifacts, overwrite protection, headless/GUI dispatch and exit
codes without requiring an RTX GPU. Windows is the local verification platform;
POSIX path selection is unit-tested, not a claim of end-to-end Linux or macOS
runtime validation. See the change's test record in the learning guide.

Explain why one command can stay portable while the executable it selects changes
between machines. Then deliberately configure a missing interpreter and run
`--check`: it should report setup guidance without launching or modifying a scene.
