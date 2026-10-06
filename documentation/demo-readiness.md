# Demo readiness and handoff

Updated October 6, 2026. These are three separate demonstrations with different
motion, sensing, and policy boundaries. Run them one at a time. Their portable
commands still depend on the local SUMO/Isaac setup and trusted local weights;
“portable” describes the launcher interface, not universal runtime support.

## 1. SUMO: scripted and frozen PPO

From the project root, launch either controller in the two-car curved-road SUMO
scenario:

```text
python -m hivemind demo sumo --seed 2001 --horizon 400
python -m hivemind demo trained --seed 2001 --horizon 400
```

The first is the scripted baseline; the second loads the configured frozen
shared-PPO checkpoint and performs inference only. Both retain the project's
state-based safety shield; it does not provide LiDAR perception. The
October readiness comparison passed its declared gates on three selected seeds
plus a repeat, with zero collisions; see the [protocol and results](demo-sumo-results.md).
Use the [portable setup guide](portable-demos.md) for environment and checkpoint
discovery.

## 2. Full-size physical car: scripted RTX obstacle bypass

```text
python scripts/demo_obstacle_bypass.py
```

This runs one simple full-size PhysX vehicle around a static obstacle using RTX
LiDAR and a known-map route. The default view follows the car; `--camera overview`
selects the overview. It runs without SUMO or RL. This is not a fleet or highway
traffic demo. Its recent GUI and headless measurements, limits, and PNG previews
are in the [physics demo performance report](demo-physics-performance.md); setup
and controller details are in [obstacle bypass](obstacle-bypass.md).

Optional lighter, measured 1x-target preview:

```text
python scripts/demo_obstacle_bypass.py --real-time
```

The GUI still accumulates lag during driving; this is not a verified real-time
mode. The HUD shows playback rate/lag, and the timing gate is separate from
driving correctness. See [current preview results](realtime-preview.md).

## 3. Isaac Lab: existing waypoint PPO

```text
python scripts/demo_waypoint.py --gui --capture
```

This is genuine inference from the existing Isaac Lab PPO checkpoint, controlling
the Leatherback with simulator-state waypoint observations. It uses neither
LiDAR nor V2V and does not model a full-size highway vehicle. The launcher uses
the current overview camera, saves one PNG, and closes at the end of its bounded
step budget. On the tested setup, the screenshot shows the RC-scale car small in
the viewport; zoom in during the live demo. The PNG is a still image, not a video.

The checkpoint is [outputs/training_v2/policy.zip](../outputs/training_v2/policy.zip),
468,327 bytes, SHA-256
`f67f1fa0cb45866cb247f99ac3daf80f6d8ecf4b24f86b96c358501e5cdeb427`. All three
validated runs used seed 43, one environment, and 1,800 control steps:

| Run | Result | Evidence |
| --- | --- | --- |
| Headless `20261006T083935Z-7e8f09a7` | 20 successes, 0 failures, 0 timeouts; 50.03 s process time | [summary](../outputs/waypoint_demo/20261006T083935Z-7e8f09a7/summary.json), [manifest](../outputs/waypoint_demo/20261006T083935Z-7e8f09a7/manifest.json) |
| GUI + capture `20261006T084240Z-dbe28c5d` | 20 successes, 0 failures, 0 timeouts; 101.55 s process time | [summary](../outputs/waypoint_demo/20261006T084240Z-dbe28c5d/summary.json), [manifest](../outputs/waypoint_demo/20261006T084240Z-dbe28c5d/manifest.json), [PNG](../outputs/waypoint_demo/20261006T084240Z-dbe28c5d/waypoint-overview.png) |
| Hardened-launcher headless repeat `20261006T085125Z-1f22039f` | 20 successes, 0 failures, 0 timeouts; 44.02 s process time | [summary](../outputs/waypoint_demo/20261006T085125Z-1f22039f/summary.json), [manifest](../outputs/waypoint_demo/20261006T085125Z-1f22039f/manifest.json) |

All three starts completed all 1,800 requested steps and left the checkpoint unchanged.
The recorded runtime was Isaac Sim `6.0.1-rc.7+release.42383.32955d8d.gl` with
Isaac Lab commit `0603cb1710dcda13087665f63abf9ac483b63c05`. The source snapshots
and hashes are in each manifest; for example, `scripts/demo_waypoint.py` hashes
to `84bd35d18d775c22a3da7dad923de4bd243f911e5118a4cebe170fb8f1111d79`, and
`scripts/waypoint_env.py` to
`afd4c5e2db14539ec2ccac3e9c86251cbedac9e500f0a6d2e647a6927626c4ba`. The
manifest hashes the Leatherback asset URI, not the remote USD bytes.

The final launcher also snapshots `hivemind/launcher.py`, verifies the saved
source copies at shutdown, records interruptions as failures, rejects malformed
summary field types, and bounds process-tree cleanup. Private local configuration
is not copied or hashed; resolved runtime paths and commands are recorded.
The final CPU suite passed **870 tests, with 10 unavailable-runtime skips**.

The default Isaac Lab launcher location is `%USERPROFILE%\Documents\IsaacLab`;
set `ISAACLAB_PATH` to another Isaac Lab root when needed. Check paths before a
live run, without initializing the simulator:

```text
python -m hivemind demo sumo --check
python -m hivemind demo trained --check
python scripts/demo_obstacle_bypass.py --check
python scripts/demo_waypoint.py --check
```

Each attempt writes a new evidence folder. Direct SUMO rollouts go under
`results/rollouts/<policy>/seed-<seed>-<timestamp>/` with a manifest, summary,
and step metrics; the multi-seed readiness comparison is under
`outputs/demo_sumo_readiness/`. The physical obstacle demo writes under
`outputs/vehicle_obstacle_bypass/`, and the waypoint launcher writes under
`outputs/waypoint_demo/`. On early exit, start with the terminal error and the
run's summary/manifest; inspect `runtime.log` where that demo records one.
Setup or GPU-guard failures preserve their evidence. Fix the reported path or
free GPU memory, then start a fresh run rather than replacing an old folder.
For SUMO environment discovery see [portable demos](portable-demos.md); for
measured RTX costs see [physics demo performance](demo-physics-performance.md).

These demonstrations do not combine the SUMO cars and full-size physical car into
a shared driving/cloning loop. The separately prepared static highway asset is
visual content, not that integration. Any new Isaac Lab policy training also
needs agreement on observations, actions, rewards, training budget, and
evaluation before it starts.
