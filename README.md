# Hivemind Traffic

Read the complete [Engineering & Learning Guide](documentation/engineering-guide.md)
directly on GitHub. All Google guide tabs are mirrored in one searchable Markdown
file, including tables and copyable commands. Guide updates from the project chat
refresh both copies through the [documented sync workflow](documentation/learning-guide-sync.md#chat-managed-markdown-mirror).

## Current work: a multi-car highway merge policy on the LA I-10/I-110 roads

The main line of work is a shared driving policy for highway merging, trained with
multi-agent PPO (MAPPO) in SUMO through TraCI. Up to 24 controlled cars share one
policy network and drive among SUMO's own background traffic.
**How the sim cars work, end to end:** [sim-car explainer](documentation/how-the-sim-cars-work.md).
It covers what controls what, what each car sees, how it decides, what limits it, and
how it is rewarded and graded.

**Where it stands (November 19, 2026):**
- **One controller per car (v14).** The policy, plus the environment's own speed,
  following-gap, lane-end and comfort limits, is the only thing driving a controlled
  car. Up to v13, SUMO's own safety and right-of-way checks ran on top of the policy,
  and held merging cars at the LA yield lines whatever the policy asked for.
  [v14 plan](documentation/highway-merge-training-plan-v14.md).
- **Primary road.** The user's LA I-10/I-110 3D model
  (`highway_usd/la_10_110/`, with its ODbL attribution). Two SUMO merge corridors
  are cut from its lane graph exactly as the model has them: no acceleration lane,
  a ramp that yields where it joins the right lane, and 65/45 mph limits. The 3D
  model itself is not modified.
- **Merge target.** The car gets a target speed as input, and a ±1.8 m/s band reward
  (v12). At yield lines, the v14 target checks the gaps from the moment the car would
  cross, and rises no faster than the car can accelerate after a stop.
- **Results.** 8 of 14 development gates on the model-exact roads, for the round-16
  policy and for round 21 (the round-16 policy gets 9 of 14 with SUMO's checks off).
  All LA merge cases still fail. There has been no held-out evaluation.
- **Round 20 (v14 with the drive limit) trained without improving overall:** 8 → 7
  gates.
  - It made the southbound LA merge physically smoother: under PhysX, peak jerk
    14.1 → 5.5 m/s³.
  - It made other merges worse, and some of its cars braked to a stop on open road.
- **Round 21 (the stop fix) kept 8 gates and cut interventions.** It added a cost on
  braking against the merge target, plus a band tail:
  - pooled interventions 0.86% → 0.46%; stable merges 9 → 10 of 12;
  - no free-road stops in validation (round 20 ended with 180).
  - Its checkpoints now rank above the round-16 policy on development validation.
    Nothing is promoted, and 12,298 study decisions remain.
- **Physics drive (videos and checks; training stays in SUMO).** The policy can drive
  a PhysX car (the Americano) on the LA model's own deck colliders. A low-level
  controller turns each command into torque and steering, and the car's pose is
  mirrored back into SUMO. On the southbound merge it reaches the mainline without a
  collision. But it can't make the acceleration the policy asks for up the ramp's
  grade, so v14's intervention count rises from 1.4% to 33.6%. See the v14 plan's
  physics-drive section and `scripts/record_highway_flow_v14_physics.py`.
- **Drive limit (on in rounds 20 and 21).** v14 can cap each controlled
  car's acceleration at what the Americano's drive gives on the road model's grade
  (`traffic/road_grade.py`; checked against PhysX within 0.02 ± 0.04 m/s²). SUMO training
  then sees the same limit as the physics car.

**Version history of the merge line** (each plan records its rounds, failures and
evidence):

| Version | What it changed | Plan |
|---|---|---|
| v01–v03 | Teacher-free MAPPO on the I-110 merge (117-field observation); ramp visibility and driving rewards; speed recovery and smoothness rewards | [v01](documentation/highway-flow-rl-v01.md), [v02](documentation/highway-flow-rl-v02.md), [v03](documentation/highway-flow-rl-v03.md) |
| v04 | Merge gap preparation from current observations; complete-episode training | [v04](documentation/highway-merge-rewards-v04.md) |
| v05–v06 | Continuous acceleration with a comfort rate limit (v05); merge speed tracked against the gate's own target, faded merge inputs (v06) | [v06](documentation/highway-merge-training-plan-v06.md) |
| v07–v08 | Cut-in exposure and merge recovery (Codex branches) | [cut-in](documentation/highway-merge-cutin-2026-11-15.md), [recovery](documentation/highway-merge-recovery-2026-11-17.md) |
| v09–v10 | Configurations and rewards for training rounds 11 and 12 | [v09](documentation/highway-merge-training-plan-v09.md), [v10](documentation/highway-merge-training-plan-v10.md) |
| v11 | Merge-speed precondition on lane changes; shield projection (7 of 8 gates) | [v11](documentation/highway-merge-training-plan-v11.md) |
| v12 | Target-speed inputs and band reward; LA I-10/I-110 model as the primary road | [v12](documentation/highway-merge-training-plan-v12.md) |
| v13 | Target followed SUMO's yield decision (superseded by v14, never trained) | [v13](documentation/highway-merge-training-plan-v13.md) |
| v14 | Policy is the only controller; yield-line crossing check; reachable target | [v14](documentation/highway-merge-training-plan-v14.md) |

The curated [CHANGELOG](documentation/CHANGELOG.md) has dated entries for each step.
Videos, checkpoints and run outputs stay out of Git.

## Start from a fresh clone

See [clean-clone setup and validation](documentation/clean-clone.md). The supported
automated profile starts with **Windows x64 / Python 3.11 / SUMO 1.27.1**.
Cloning does not include Isaac Sim, private car assets, recordings or checkpoints.

```text
git clone https://github.com/hudsonkaneko/hivemind-traffic.git highwaysim
cd highwaysim
py -3.11 -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements-ci.txt
.venv\Scripts\python.exe -m pytest -q
.venv\Scripts\python.exe -m hivemind demo sumo
```

With a compatible external Isaac installation configured, the loop showcase has
an explicit public block-car option; no Americano download or checkpoint is needed:

```text
python scripts/demo_loop_showcase.py --version v09 --vehicle public --traffic-count 12 --check
python scripts/demo_loop_showcase.py --version v09 --vehicle public --traffic-count 12
```

`python` here is a normal Python 3.11+ interpreter, not Isaac's bundled Python.
The launcher starts the configured Isaac runtime in a separate process. This is
scripted PhysX driving with kinematic background cars, not trained autonomy or
LiDAR-based perception. The prepared vehicle remains the default for existing commands.

## One active workspace

The maintainer's canonical local checkout is `C:\Users\hudso\Documents\highwaysim`.
Its GitHub repository is **hudsonkaneko/hivemind-traffic**; the different local
folder name does not mean a second project. Older working folders are archived
under `C:\Users\hudso\Documents\expired_highwaysim` and are not active workspaces.

This checkout consolidates the original SUMO/PettingZoo/PPO and replay work with
the newer live LiDAR, obstacle-avoidance, real-time, and two-car V2V experiments.
See [consolidation and verification](documentation/consolidation-2026-10-02.md)
for preserved versions, environment separation, evidence, and limitations.

From the repository root, using a configured Python environment:

```text
python -m hivemind demo sumo --check
python -m hivemind demo trained --check
python scripts/demo_cooperative_lidar.py --check
```

Remove `--check` to launch the corresponding GUI demo. Configure Isaac/SUMO and
the traffic Python in the ignored `hivemind.local.json` as described in
[portable demos](documentation/portable-demos.md). On this machine the preserved
traffic environment is `.venv-traffic`, the newer CPU-test environment is `.venv`,
and rendering uses its own runtime. These environments must not be mixed into
Isaac's bundled Python.

Vehicle asset iterations belong in `vehicles/sim_ready/<vehicle-name>/v01`, etc.
Archived experiment results remain historical evidence, not fresh validation.

## Active development: hybrid vehicle physics

The [current 20-step roadmap](documentation/hybrid-roadmap.md) assigns evaluated
cars to Isaac/PhysX and lightweight background traffic to SUMO. This is a staged
migration: the existing traffic demos remain SUMO-driven. The separate one-car
[physical LiDAR view](documentation/physics-lidar-view.md) adds a marked curved
road, overview/follow cameras, planned-path overlays and scripted LiDAR braking.
It needs no SUMO; a two-way traffic bridge and physics-based training remain future work.

```text
python scripts/demo_physics_lidar.py --points
```

This opens the bounded physics-car demonstration using the configured external
Isaac runtime. It is scripted control, not a trained driving policy.
The separate [adaptive obstacle bypass](documentation/obstacle-bypass.md) makes
one physical car pass a static barrier and return, with the cyan line showing its
LiDAR-triggered planned route:

```text
python scripts/demo_obstacle_bypass.py
```

The [35 mph profile](documentation/physical-35mph.md) extends the road and
LiDAR-triggered maneuver for a 15.6464 m/s cruise target. Use `--profile low-speed`
to retain the original 3 m/s fixture. See the profile note for validation status;
vehicle speed is distinct from the measured real-time playback rate.

See the [documentation index](documentation/README.md) and
[development workflow](documentation/workflow.md) for changes, evidence and review.

## Scaling foundations and workstation measurements

The October 3 capacity study separates cheap SUMO background traffic from
expensive independent RTX sensors. See the [implementation walkthrough](documentation/scaling-foundations.md),
[measured capacity report](documentation/capacity-results.md), and
[shared state contract](documentation/state-contract-v1.md).
The current integrated cooperative demo still has two cars; the larger-fleet
probes are isolated capacity tests, not proof of large-fleet autonomy.

Multi-agent highway traffic simulation combining SUMO/TraCI, PettingZoo, PPO communication policies, and OpenUSD/Isaac Sim visualization.

## Repository boundaries

Eight project-domain agent skills are versioned in [`.agents/skills`](.agents/skills).
See [skill usage and scope](documentation/project-skills.md) for discovery,
current architecture boundaries, provenance and intentionally excluded local data.

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
