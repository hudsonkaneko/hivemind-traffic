---
name: build-omniverse-traffic-viewer
description: Build an Omniverse traffic viewer from an OpenUSD scene using either an interactive Kit app or an ovstage plus ovrtx headless renderer and sensor service. Use for traffic playback, controls, telemetry, RTX visualization, or synthetic sensor output; do not use it as the traffic solver or RL trainer.
---

# Build Omniverse Traffic Viewer

## Highway Sim repository scope

Read the repository's `AGENTS.md` and `documentation/project-skills.md` before
applying this workflow. The active roadmap determines the selected backend:
SUMO authority statements below apply to SUMO-backed experiments, not to native
PhysX-controlled vehicles. Preserve one motion owner per vehicle. Repo-local
skills do not authorize training, dependency installation, or publication of
local-only vehicle assets; follow the current task and project authorization.

Choose one runtime for the first milestone. Do not require both:

- Choose **Omniverse Kit** for an interactive desktop application with a viewport, timeline, scenario controls, agent selection, overlays, and telemetry.
- Choose **ovstage + ovrtx** for a lightweight Python/C application, headless rendering, a custom service, or camera/LiDAR/radar output.

If both are eventually required, make them consume the same USD scene and simulation-state contract instead of coupling them directly.

## Preserve system boundaries

- SUMO/TraCI owns traffic evolution; a trained policy supplies decisions; OpenUSD carries visual state.
- The viewer consumes live snapshots or saved replay layers. UI commands such as run, pause, reset, and scenario selection go through an explicit controller interface.
- Train headlessly with rendering disabled unless pixels are policy observations. Use rendering for evaluation, sensors, replay, and demonstrations.
- Decouple simulation, USD updates, rendering, and UI. Use timestamps, a bounded queue, and interpolation; discard stale display frames instead of slowing training.

## Shared input contract

Define a snapshot containing simulation time, vehicle ID, position, orientation, optional velocity, traffic-signal state, and optional agent telemetry. Verify units, axes, prim paths, and asset resolution before building UI or sensors.

## Kit route

Keep extensions small and separately testable. A useful split is:

```text
edu.hivemind.traffic     # source connection and lifecycle
edu.hivemind.usd         # snapshot-to-stage synchronization
edu.hivemind.control     # run, pause, reset, scenario and replay controls
edu.hivemind.telemetry   # selection, metrics and communication overlays
```

- Author live transforms to a session/dynamic layer, not the static asset layers.
- Schedule stage and UI work through Kit's application lifecycle; never block the UI thread on SUMO or model inference.
- Provide live and replay modes using the same stage schema.
- Make debug overlays switchable so normal RTX rendering remains readable.

## ovstage + ovrtx route

- Start from NVIDIA's minimal Python example and confirm a single PNG render before integrating traffic.
- Let `ovstage` own runtime scene state; connect `ovrtx` for visualization or sensors. Renderer-owned stage APIs are deprecated in current `ovrtx` releases.
- Pin tested `ovstage`, `ovrtx`, Python, NVIDIA driver, and GPU requirements.
- For every sensor output, record sensor pose, calibration/configuration, simulation timestamp, frame number, and source scenario.
- Keep sensor cadence independent of SUMO's physics timestep when appropriate, and document interpolation.

## Deliverables

- One selected runtime with a documented launch command.
- USD/snapshot adapter and a replay fixture usable without SUMO.
- Interactive controls and telemetry for Kit, or timestamped render/sensor output for `ovrtx`.
- Environment/version manifest and a short performance report.

## Acceptance tests

Common tests:

- A recorded scenario loads without running SUMO and reproduces vehicle and signal motion at the expected timestamps.
- Missing assets or malformed snapshots produce actionable errors rather than a frozen viewer.
- The simulator continues at its configured rate when the viewer is disconnected or rendering more slowly.
- Static USD layers are not modified by playback.

Kit-specific tests:

- Run, pause, reset, scenario selection, timeline scrub, and agent selection work without blocking the viewport.
- Live and replay modes use the same prim paths and visual conventions.

`ovrtx`-specific tests:

- The minimal example renders successfully on the target machine before traffic integration.
- A fixed replay produces the requested RGB or sensor frames with monotonic timestamps and documented calibration.

## Official references

- [Getting started with Kit extensions](https://docs.omniverse.nvidia.com/kit/docs/kit-manual/latest/guide/extensions_basic.html)
- [NVIDIA ovrtx repository and setup](https://github.com/NVIDIA-Omniverse/ovrtx)
- [ovrtx examples](https://github.com/NVIDIA-Omniverse/ovrtx/blob/main/examples/README.md)
- [ovrtx documentation](https://nvidia-omniverse.github.io/ovrtx/)
