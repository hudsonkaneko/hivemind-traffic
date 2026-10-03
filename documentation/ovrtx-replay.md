# Standalone ovstage + ovrtx traffic rendering

The shared exporter is `scripts/replay_export.py`. It needs OpenUSD and NumPy,
not Isaac Sim. It creates road, referenced vehicle, motion, and root layers and
checks all vehicles' first/last positions and headings. The Isaac viewer uses
this same exporter and authors its lidar/camera opinions into its session layer.
The portable `world.usda` therefore contains no Isaac lidar or camera setup.

The separate renderer, `scripts/render_ovrtx.py`, loads the shared scene through
ovstage at a requested replay time in **seconds**. OpenUSD validation uses time
codes (60 per second); the ovstage loader performs that conversion internally.
Its camera and render-product definitions are in a separate render layer.
The current milestone renders fixed timestamps; it is not yet an interactive
ovrtx viewer or an ovrtx lidar implementation.

## Setup and run (Command Prompt, repository root)

```cmd
.venv\Scripts\python.exe -m venv .venv-ovrtx
.venv-ovrtx\Scripts\python.exe -m pip install -r visualization\requirements-ovrtx.txt
.venv-ovrtx\Scripts\python.exe scripts\render_ovrtx.py --minimal
.venv-ovrtx\Scripts\python.exe scripts\render_ovrtx.py --time 2
.venv-ovrtx\Scripts\python.exe scripts\render_ovrtx.py --time 15
```

The traffic commands require `outputs/sumo_replay/recording.json` and
`network.net.xml`. To regenerate those with an existing checkpoint:

```cmd
.venv\Scripts\python.exe scripts\record_sumo_replay.py --source-repo . --checkpoint results\training\YOUR_RUN\checkpoints\best.pt
```

The checkpoint is a local artifact; choose a compatible trained checkpoint from
your machine. Scans and recorded results are not downloaded with a Git clone.

Images and version/timestamp manifests are written to `outputs/ovrtx/`.
Every traffic render checks that all four shared USD layers retain their hashes
during rendering. First startup may compile shaders and take several minutes.
The NVIDIA minimal example uses an online scene; the traffic scene uses local
road and vehicle geometry. Python packages/runtime caches are separate from the
SUMO environment and the installed Isaac Sim runtime.

Official API reference used:
https://github.com/NVIDIA-Omniverse/ovrtx/tree/main/examples/python/minimal

## Verified milestone

On the RTX 5070 with driver 610.74 and Python 3.11, the minimal NVIDIA scene
rendered at 1920x1080. The traffic replay rendered at 960x540 for 2 s and 15 s;
both images were visually inspected. The 2 s traffic render took approximately
3.9 seconds including renderer initialization after caches were warm; this is a
small smoke-test measurement, not a throughput benchmark. The exporter passed
10 first/last position-and-heading checks across five vehicles.

Runtime-specific sensor and camera edits in Isaac are session-only. Lidar is
still provided by the Isaac adapter; RGB success does not establish ovrtx lidar
support for our sensor configuration or scan equivalence between runtimes.

Isaac may add render products to its in-memory root layer internally. The viewer
does not save that runtime stage over the portable replay files on disk.
The final Isaac regression ran 120 frames successfully with 20 moving lidar
callbacks. Reopening the portable stage afterward confirmed zero composition
errors and no Isaac lidar or render-product prims on disk.
