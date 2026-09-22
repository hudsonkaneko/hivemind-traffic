# RTX lidar starting point

The standalone test is `scripts/smoke_rtx_lidar.py`, using the installed Isaac Sim
6.0.1 experimental RTX API and Example_Rotary configuration. It creates a ground
plane, three 2 m boxes centered 5 m ahead/left/right, and a sensor 1 m above ground.
All objects are static; no vehicle controller or physics authority is involved.

Run from the project root:

```powershell
& C:\isaacsim\python.bat scripts\smoke_rtx_lidar.py
# Optional visible point cloud (closes after the frame budget):
& C:\isaacsim\python.bat scripts\smoke_rtx_lidar.py --gui --frames 1800
```

Each invocation saves a scene, raw sensor sample and JSON report in a new dated
`outputs/lidar_smoke/` directory. Raw GMO columns are azimuth and elevation in
degrees and range in metres, not Cartesian XYZ. The pass check requires at least
10 sensor callbacks and returns near the known front box face, 4 m away.
The report includes sensor timestamps and timeline times.

Hardware: RTX 5070, 12 GB, driver 610.74. Runtime:
6.0.1-rc.7+release.42383.32955d8d.gl. This tests static lidar generation only.
The next milestone is recorded SUMO vehicle motion in an OpenUSD stage, followed
by mounting this sensor on a replayed vehicle. SUMO will own those vehicle poses.

Verified run: `outputs/lidar_smoke/20260922T050408Z/summary.json` passed with
30 sensor callbacks and 24,010 returns near the front box's expected range.
The process exited successfully. Its saved `scene.usda` and `raw_gmo_xyz.npy`
provide a reusable static scene and sensor sample. Startup/runtime output is in
`logs/lidar-smoke-geometry.log`.
