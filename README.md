# Hivemind Traffic

Multi-agent highway traffic simulation combining SUMO/TraCI, PettingZoo, PPO communication policies, and OpenUSD/Isaac Sim visualization.

## Repository boundaries

| Directory | Responsibility |
|---|---|
| `scenarios/` | SUMO networks, routes, demand, and configurations |
| `traffic/` | TraCI runners and vehicle control |
| `environments/` | PettingZoo environments and wrappers |
| `policies/` | PPO networks and communication models |
| `usd/` | OpenUSD assets, stages, and exporters |
| `visualization/` | Kit extensions and ovrtx application code |
| `experiments/` | Training and evaluation configurations |
| `tests/` | Automated tests |
| `results/` | Evaluation summaries and publication-ready artifacts |

The existing single-car waypoint work below is the first prototype retained during migration into this layout.

## Single-car waypoint sandbox

Project location: `C:\Users\hudso\Documents\highwaysim`.

An Isaac Sim / Isaac Lab starting point for the highway project. A physics-driven
NVIDIA Leatherback car learns to reach a target on an open ground plane using PPO.
Each training environment contains one car; multiple independent environments
collect experience for the same policy.

## Installed setup

- Isaac Sim: `C:\isaacsim`, version `6.0.1-rc.7+release.42383.32955d8d.gl`.
- Isaac Lab: `C:\Users\hudso\Documents\IsaacLab`, branch
  `release/3.0.0-beta2`, commit `0603cb1710dcda13087665f63abf9ac483b63c05`.
- The Lab `_isaac_sim` directory junction points to `C:\isaacsim`.
- Python 3.12.13, PyTorch 2.10.0 + CUDA 12.8, Stable-Baselines3 PPO.
- Hardware checked: RTX 5070, approximately 12 GB VRAM, driver 610.74.

Isaac Lab was installed with its official `--install "rl[sb3],visualizer[kit]"`
command using the simulator's bundled Python. This release does not support
combining a standalone binary installation with a separate virtual environment.
The Windows Lab wrapper warns about a missing `setup_conda_env.bat`; the bundled
Python path still works and has been exercised by the tests below.

## Run from this folder in PowerShell

From Command Prompt, run `demo.cmd` in this folder, or double-click it in File
Explorer. It launches the PowerShell demo with the correct project paths.

```powershell
# Watch the trained car for about one minute
.\demo.ps1

# Close Isaac Sim yourself when you are done (instead of auto-closing)
.\run.ps1 -Mode evaluate -Steps 1800 -NumEnvs 1 -Gui -HoldOpen -Checkpoint outputs\training_v2\policy.zip

# Basic gravity/ground check
.\run.ps1 -Mode smoke

# Scripted forward / turn / brake test
.\run.ps1 -Mode vehicle -Gui

# Ground-truth controller following the middle lane of the circular highway
.\run.ps1 -Mode lane -Steps 4500 -Gui

# Leave the highway demo open after its evaluation finishes
.\run.ps1 -Mode lane -Steps 4500 -Gui -HoldOpen

# Train a fresh policy (total transitions across all environments)
.\run.ps1 -Mode train -Steps 131072 -NumEnvs 16

# Watch the saved policy (steps here means simulation/control steps)
.\run.ps1 -Mode evaluate -Steps 1800 -NumEnvs 1 -Gui -Checkpoint outputs\training_v2\policy.zip
```

The GUI closes when the requested steps finish unless `-HoldOpen` is supplied.
With `-HoldOpen`, it stays available until you close the Isaac Sim window or press
Ctrl+C in the terminal. Targets are green spheres.
Reaching a target ends the episode and resets the car with a new heading and
target; this version does not yet implement a continuous route between targets.
The car is small (approximately 42 cm long), so the demo camera is close to it.

For explicit evaluation seeds or harder targets, use the Python entry point:

```powershell
& "$env:USERPROFILE\Documents\IsaacLab\isaaclab.bat" -p scripts\train_waypoint.py --mode evaluate --checkpoint outputs\training_v2\policy.zip --num_envs 16 --steps 1200 --seed 43 --output outputs\evaluation_v2
```

## Task definition

- Ground-truth observations: target coordinates in the car frame, planar speed,
  yaw rate, actual steering angles, and previous commands (9 values).
- Actions: desired forward speed from 0–1 m/s and steering from −0.5–0.5 rad.
- PhysX timestep: 1/120 s; control timestep: 1/30 s.
- Targets: 2–5 m away, within ±0.65 rad of the initial forward direction.
- Success: within 0.4 m; timeout: 20 simulated seconds.
- Reward: progress toward the target, success bonus, small time/action-change
  penalties, and failure penalty for tipping or leaving the allowed area.
- Initial yaw is randomized over the full circle. This is not yet training for
  targets anywhere around the car; widen `--target_angle` for a later curriculum.

The shipped asset has a measured wheelbase of 0.32 m, track width of approximately
0.2416 m, and tire radius of 0.052 m. Those dimensions differ from the larger
dimensions in the NVIDIA controller snippet. The code uses the measured values.
Vehicle motion during an episode comes from wheel drives and contact physics.
Pose writes are used only at reset.

The stock asset's unsupported collision-group replication is disabled in the
task's local stage. Articulation self-collisions are also disabled. Suspension
joint reset positions are inside their actual limits. Isaac Lab 3 uses xyzw
quaternions; the standalone Isaac Sim test uses its wxyz API convention.

## Files and results

- `scripts/waypoint_env.py`: task, vehicle configuration, observations and rewards.
- `scripts/train_waypoint.py`: PPO training, checkpoint loading, evaluation and
  optional `--capture path.png` for GUI viewport capture.
- `scripts/smoke_sim.py`, `scripts/smoke_vehicle.py`: simulator and driving checks.
- `scripts/follow_circular_lane.py`: pure-pursuit baseline on the circular highway.
- `scenes/highway/v1/highway_v1.usda`: collision-enabled three-lane highway.
- `outputs/*/policy.zip`: trained policy; `checkpoints/`: intermediate policies.
- `outputs/*/summary.json`: completed-episode results and runtime.
- `outputs/*/config.json`: command arguments used for that run.
- `outputs/*/trace.json`: sampled positions, targets and commands for vehicle 0
  during evaluations.
- `logs/`: full startup, installation and training logs (ignored by Git).

Assets load from NVIDIA's asset server and are cached by the simulator. Internet
access is required for uncached assets. No camera or lidar observations are used.

These are initial functional checks, not a statistically validated traffic study.
Evaluation success rates count completed episodes; unfinished episodes at the
step limit are excluded. Training seeds and evaluation seeds are separate, but
one evaluation seed alone does not establish generalization.

## Verified results (September 11, 2026)

- The gravity check settled a 0.5 m cube at z ≈ 0.25 m.
- The scripted car test traveled 2.36 m forward, turned, and braked to about
  0.0044 m/s planar speed.
- CUDA tensor operations ran successfully on the RTX 5070.
- The graphical single-car check reached three targets and saved
  `outputs/waypoint_demo.png`; the car and green target were visually inspected.
- A 16-environment PPO run produced saved, reloadable policies. The longer run
  was resumed from a checkpoint after an interruption.
- On seed 43 over 1,200 control steps per environment:

| Controller | Targets reached | Timeouts | Failures | Completed-episode success |
|---|---:|---:|---:|---:|
| Random actions | 8 | 25 | 0 | 24.2% |
| First PPO policy | 17 | 24 | 0 | 41.5% |
| Longer-trained PPO policy | 187 | 0 | 0 | 100% |

Use `outputs/training_v2/policy.zip` for the longer-trained policy. The results
apply to the easy target distribution above, not arbitrary navigation or traffic.
Faster policies complete more episodes during the same simulation budget;
unfinished episodes are excluded from the percentage.

## References

- [Isaac Lab release](https://github.com/isaac-sim/IsaacLab/tree/release/3.0.0-beta2)
- [NVIDIA vehicle controller example](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/robot_simulation/mobile_robot_controllers.html)
