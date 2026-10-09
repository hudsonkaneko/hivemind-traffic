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

For the current presentation-ready entry points and their distinct controller
boundaries, see [demo readiness](demo-readiness.md). After activating your
configured traffic Python, these additional bounded launchers preserve old runs:

```text
python scripts/demo_obstacle_bypass.py
python scripts/demo_obstacle_bypass.py --profile low-speed
python scripts/demo_obstacle_bypass.py --real-time
python scripts/demo_waypoint.py --gui --capture
python -m experiments.demo_sumo_readiness --headless
```

The obstacle bypass is a scripted full-size physical car with RTX LiDAR, not
RL. The [35 mph profile note](physical-35mph.md) records the faster fixture and
its validation status; `--profile low-speed` retains the original 3 m/s demo.
Its optional `--real-time` mode uses lighter graphics, 20-Hz rendering and
measured 1x-target pacing; the GUI is **not yet consistently real time**.
See [preview commands, timing gates and actual results](realtime-preview.md).
The waypoint demo evaluates the existing RC-scale Leatherback PPO, not a
full-size highway policy. It needs a compatible Isaac Lab installation (set
`ISAACLAB_PATH` when it is not in your Documents/IsaacLab folder) and a trusted
local `outputs/training_v2/policy.zip`, or an explicit `--checkpoint` override.
Checkpoints are not included in Git. Add `--check` to either script for a
non-launching setup check. On this workstation the configured traffic Python is
`.venv-traffic/Scripts/python.exe`; environment directory names may differ on a
new clone. Do not install traffic packages into Isaac's Python.

For the newer, separate **physics-driven car** with curved road, overview/follow
cameras, planned path and LiDAR emergency braking, see the
[physical LiDAR view](physics-lidar-view.md). It does not require SUMO:

```text
python scripts/demo_physics_lidar.py --check
python scripts/demo_physics_lidar.py --points
```

The command is portable runtime discovery, not a bundled simulator: a compatible
Isaac installation and NVIDIA RTX hardware are still required. This new path is
validated on the recorded Windows/Isaac 6 installation, not every operating system
or Isaac release. Its current simple scripted car is not a learned policy.

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

## Mentor branch: continuous physical highway loop

Separate known-map scripted PhysX fixture, with no SUMO, LiDAR or RL:

```text
python scripts/demo_highway_loop.py --check
python scripts/demo_highway_loop.py --profile seam3
python scripts/demo_highway_loop.py --profile seam35
python scripts/demo_highway_loop.py --profile lap35
```

Use the configured traffic Python to launch; the supervisor discovers Isaac.
The GUI defaults to light graphics and best-effort real-time pacing. It has
follow/overview and pause controls. Runs are bounded and close on completion.
See [acceptance results and limitations](continuous-highway-loop.md) before
equating a short preview with full-lap, multi-car or learned-control validation.

## Prepared-car loop traffic showcase

Use the configured traffic environment from the repository root. The supervisor
discovers the installed Isaac runtime; it does not install it or require SUMO.
This demonstration also requires the separately supplied local prepared vehicle
at `vehicles/sim_ready/americano-i7-ev/v07/world.usda`. That asset is **not in
GitHub** and is not downloaded automatically; copying the repository alone does
not supply its unverified redistribution rights.

```text
python scripts/demo_loop_showcase.py --check
python scripts/demo_loop_showcase.py --version v04
```

The v04 default is a bounded four-minute driving preview plus settling and
braking. It closes on completion. Follow, traffic overhead, whole-loop overview,
pause and path-visibility controls are in the showcase window. Main-car motion
is PhysX, slower traffic is scripted kinematic, and object observations come
from simulator state—not LiDAR or RL. Pacing never skips physics ticks.

Shorter presentation and repeatable headless test:

```text
python scripts/demo_loop_showcase.py --version v04 --drive-seconds 90
python scripts/demo_loop_showcase.py --version v04 --drive-seconds 90 --headless --unpaced --capture
```

Start with a closer traffic-overhead camera by adding `--camera traffic`.
`--capture` saves actual viewport images in the fresh local output directory;
`--headless` hides the window but this showcase still renders, so it still
requires RTX hardware. The displayed path is the controller's reference.

Separate fault checks (not the presentation):

```text
python scripts/demo_loop_showcase.py --version v04 --mode blocked --drive-seconds 45 --headless --unpaced
python scripts/demo_loop_showcase.py --version v04 --mode dropout --drive-seconds 45 --headless --unpaced
```

The development-only `--mode contact-check --drive-seconds 15` deliberately
hits a background collider to prove contact reporting is active. Do not show
that fixture as successful obstacle avoidance.

Earlier milestone profiles remain selectable with `--version v01`, `v02`, or
`v03`, using the current corrected implementation. Their historical source is
preserved by Git; version folders record the original evidence and limitations.
See the [showcase evidence](loop-traffic-showcase.md) for the exact tested runs,
camera captures, physical versus real-time status, and retained failures. Close
other simulation runs before starting; memory measurements include the desktop.
