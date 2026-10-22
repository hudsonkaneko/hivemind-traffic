# Supplied-car visual twin milestone

This viewer composes a selected car from a user-supplied USD collection with a
saved SUMO trajectory. It supplies play/pause, reset, scrub, playback rate,
vehicle selection, overview/follow cameras and source-sample telemetry.
The synthetic road comes from SUMO. It is not yet the Esri-supported research
merge, a trained communicating controller, or a vehicle physics experiment.

## Reproduce locally

Use the existing traffic Python with `usd-core` to prepare the scene. Inputs
are local files; they are deliberately not shipped in Git. The reviewed scripted
baseline stores `recording.json` and `network.net.xml` under
`outputs/baseline-20261022-a/seed-2001-a/`.

```text
python scripts/build_visual_twin.py --source <supplied-vehicles.usd> --recording <recording.json> --network <network.net.xml> --output outputs/my-twin/scene
scripts\open_visual_twin.cmd outputs\my-twin\scene\world.usda
```

The default selected prim is `/Root/vehicles/sm_AmericanoI7EV_a1_05` from Sam's
collection. Preparation intentionally refuses to overwrite an existing version;
use `--asset vehicles/sim_ready/sam-collected/v02` and a new output for revisions.
The unchanged source is copied once beside versioned wrappers. Asset hashes
and geometry-derived dimensions are saved in `provenance.json`.

The native Isaac window stays open until closed. The custom panel controls
playback; its scrub slider pauses at the selected time. Vehicle selection follows
that car and updates sampled speed/position. The status line shows current replay
time and source sample time separately. After a car disappears, its last pose
remains available for the camera, but the telemetry explicitly says absent.
Use the standard Stage and Layers panels to inspect composition; camera views
are under `/World/Cameras`. Viewport render products are runtime-owned exceptions.

## Separate Isaac environment

Isaac 6.0.1 uses Python 3.12; do not put the traffic requirements in it. A local
environment at `runtimes/isaac-env` is ignored. A reproducible installation is:

```text
uv venv --python 3.12 --seed runtimes/isaac-env
runtimes\isaac-env\Scripts\python.exe -m pip install "isaacsim[all,extscache]==6.0.1.0" --extra-index-url https://pypi.nvidia.com
```

Handle NVIDIA's license prompt as the user; the viewer does not embed acceptance.
Follow NVIDIA's versioned installation instructions for CUDA-enabled Torch when
needed. Keep installation logs and the final freeze locally, and run `pip check`.

Bounded renderer/control verification:

```text
runtimes\isaac-env\Scripts\python.exe scripts/view_visual_twin.py --scene outputs/my-twin/scene/world.usda --headless --frames 360 --check-controls --evidence outputs/my-twin/check01
```

This produces actual viewport PNGs, callback-check results and static-layer
hash comparisons. It does not establish manual GUI interaction or steady-state
rendering performance. Choose a fresh evidence directory for every attempt.

## Asset and composition limits

The supplied archive hash is
`df10b018f70beba44b8c4608070dec7a8b99460533047c6e9d0faeb377ceb698`;
its USD hash is
`4c4c40682c0461b3f1d566ff0137a0c31ad1a2beaf755a6894aa3f2646fa3dd5`.
This is the supplied geometry, not the repository's separately prepared physics
Americano. Its wheel centroids identify local −X as forward. A 0.01 scale and
180-degree yaw put forward at +X; the wrapper moves the front to x=0 and the
bottom to z=0. Bounds are approximately 5.00 × 2.21 × 1.84 m. These dimensions
are a geometry-based interpretation, not a manufacturer's measurement.

The car's source textures are missing. The wrapper disables its source Looks
and binds explicit fallback paint/glass/rubber. Original source bytes remain
unchanged. A whole-layer dependency scan still sees missing textures elsewhere
in the preserved collection, including inactive materials; distinguish that
archive completeness failure from active composed material dependencies.
No texture download, rights clearance, calibrated collision shape, tire model,
wheel animation or dynamics is implied. Geometry and screenshots stay local
until redistribution rights are established.

All five traffic IDs currently use this same visual car. Its geometry dimensions
are not rescaled to each SUMO vehicle's collision envelope. Telemetry is the
authoritative traffic measurement; rendered clearances are not safety metrics.
Motion is linearly interpolated between recorded samples; lifecycle is sampled.
The original baseline ends when controlled agents finish, leaving three active
background trips. See the dedicated plan for the accounting repair before any
traffic-wide comparison.

Layers: `world.usda` composes `motion.usda` over `environment.usda`. Stable vehicle
roots carry motion; their immutable Model subtrees instance the referenced
versioned car. Cameras/UI operate in the session layer. Source, environment and
saved motion must remain unchanged during playback.
