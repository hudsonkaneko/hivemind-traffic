# SUMO replay with moving RTX lidar

The recorded shared PPO run (seed 2001) contains both agents and background
vehicles. The source recording has 244 snapshots at 0.2-second intervals.
Run artifacts are under `outputs/sumo_replay/`.

## Watch the replay

From Command Prompt in the project folder:

```cmd
C:\isaacsim\python.bat scripts\traffic_replay_3d.py --gui --frames 2900
```

From PowerShell, prefix the command with `&`. The camera follows agent_0 and
green points visualize lidar returns. The window closes after the frame budget.
Use the installed Isaac Sim runtime's python.bat if it is located elsewhere.
The script rebuilds the generated layers and replaces the previous preview and
sensor report in this output folder. The original recording is unchanged.

`world.usda` can also be opened directly in Isaac Sim and scrubbed without SUMO.
Run the Python viewer to create the live sensor reader and point-cloud display.

## Contract and validation

- SUMO owns every vehicle pose. The replay adds no physics controller.
- Metres, Z up, X east, Y north; the existing SUMO network offset is already
  reflected in the recorded coordinates and is not applied a second time.
- Vehicle origins are front bumpers, with body geometry extending along local
  negative X. Yaw is 90 degrees minus SUMO's clockwise heading from north.
- All layers use 60 time codes per second. Source times are shifted so the first
  frame is at time zero. Positions and headings interpolate linearly.
- Hex-encoded vehicle IDs give reversible, stable USD paths. Original IDs are
  also stored as metadata. Visibility tracks departures and arrivals.
- `road.usda` is static geometry from the SUMO lane shapes; `motion.usda` holds
  sampled vehicle transforms; all cars reference `vehicle.usda`.
- Lidar is mounted on agent_0 at local (-2.5, 0, 1.8) metres. Motion BVH is enabled
  for the rotating RTX sensor. Example_Rotary is the sensor configuration.
- The exporter checks every vehicle's first and last composed position and
  rejects USD composition errors. The sensor test requires moving sensor poses
  and nonzero in-range returns. The report records sensor timestamps and mounts.
- `scan_*.npz` contains azimuth/elevation in degrees and ranges in metres, with a
  sensor timestamp. Those arrays are not Cartesian XYZ.

This is a simple block-model traffic and sensing prototype. It does not establish
lidar perception accuracy, sensor-based driving, or physical vehicle dynamics.
Lidar testing covers the rendered interval; the USD stage stores the full run.
The report records the checkpoint and source-network SHA-256 hashes.

Verified: the 1,800-render-frame headless run exited successfully, produced 300
sensor callbacks with a moving sensor and valid returns, and passed all 10
first/last vehicle position checks. The preview was visually inspected. The
motion-BVH warnings from the initial prototype are absent in this run.
See `outputs/sumo_replay/replay_report.json` and `logs/traffic-replay-3d.log`.

GUI startup fix: enable `isaacsim.sensors.rtx.nodes` before constructing the lidar
sensor, because sensor instances capture writer aliases at construction time.
Enabling it afterward left `draw-point-cloud` unavailable to that instance.
The corrected GUI path completed 240 frames, produced 40 sensor callbacks, and
exited with code 0. Log: `logs/traffic-replay-gui-check.log`. The same ordering
fix was applied to the standalone lidar smoke test.
