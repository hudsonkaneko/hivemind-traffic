# Parameter reference

Snapshot date: 2026-09-11. Values below describe the current code and the saved
`training_v2/policy.zip` metadata. Future run manifests store the resolved task
and PPO settings used for that particular execution.

## Scene and task

| Parameter | Value | Location |
|---|---:|---|
| Training environments | 16 | Runner `--num_envs` |
| GUI demo environments | 1 | `demo.ps1` |
| Environment spacing | 120 m | `WaypointCfg.scene` |
| Shared ground size | 3,000 × 3,000 m | `_setup_scene()` |
| Physics timestep | 1/120 s | `WaypointCfg.sim.dt` |
| Control decimation | 4 physics steps | `WaypointCfg.decimation` |
| Control timestep | 1/30 s | Derived |
| Nominal episode limit | 20 s / 600 control steps | `episode_length_s`; done check uses `max_episode_length - 1` |
| Initial root height | 0.1 m | `robot_cfg.init_state` |
| Initial heading | Uniform over [0, 2π) | `_reset_idx()` |
| Goal distance | Uniform over 2–5 m | `target_min_distance`, `target_max_distance` |
| Goal bearing from initial heading | Uniform over ±0.65 rad | `--target_angle` / `WaypointCfg.target_angle` |
| Goal success radius | 0.4 m, planar | `success_radius` |
| Maximum requested speed | 1 m/s, forward only | `max_speed` |
| Maximum center steering magnitude | 0.5 rad (28.65°) | `max_steering` |
| Action / observation dimensions | 2 / 9 | `action_space`, `observation_space` |
| Boundary | Local x/y within ±48 m | `_get_dones()` |
| Allowed local root z | −1 to +3 m | `_get_dones()` |
| Minimum upright component | 0.5 | `_get_dones()` |
| Position / velocity solver iterations | 8 / 2 | `articulation_props` |
| Articulation self-collisions | Disabled | `enabled_self_collisions` |
| Physics replication / Fabric cloning | Both disabled | Scene config |
| Steering / wheel actuator gains | Inherited from asset (`None` overrides) | `ImplicitActuatorCfg` |
| Rear / front shock reset positions | −0.03 / +0.03 m | `joint_pos` |
| Dome light intensity | 1,500 (API intensity value) | `_setup_scene()` |
| Target marker radius | 0.15 m | GUI-only marker |

Only `num_envs`, `seed`, `target_angle`, device, and visualizer settings have direct
runner switches. Change the corresponding source configuration for other values,
and create an iteration record explaining why.

## Reward coefficients

| Term | Coefficient |
|---|---:|
| Reduction in planar distance (m) | +4 |
| Per-control-step time penalty | −0.01 |
| Squared change in clipped actions | −0.01 |
| Target reached | +20 |
| Geometric failure | −20 |

Timeout has no separate terminal penalty. The exact expression is in
[`_get_rewards()`](../scripts/waypoint_env.py).

## PPO configuration

| Parameter | Value |
|---|---|
| Library | Stable-Baselines3 2.9.0 |
| Algorithm / policy | PPO / MlpPolicy |
| Hidden layers | [128, 128] |
| Learning rate | 0.0003 |
| Rollout length per environment | 256 |
| Transitions per 16-env rollout | 4,096 |
| Minibatch size | 256 |
| Epochs per rollout | 5 |
| Discount factor (gamma) | 0.99 |
| GAE lambda | 0.95 |
| Entropy coefficient | 0.01 |
| PPO clip range | 0.2 (observed in training log) |
| Value-loss coefficient | 0.5 (saved model metadata) |
| Maximum gradient norm | 0.5 (saved model metadata) |
| Advantage normalization | Enabled (saved model metadata) |
| Target KL stopping threshold | None (saved model metadata) |
| Training seed | 42 |
| Evaluation seed used in initial comparison | 43 |
| Policy training / inference device | CPU |
| Simulation device | cuda:0 |
| PyTorch CPU threads | 4 |
| Checkpoint cadence at 16 environments | Every 256 vector steps / 4,096 transitions |

The default fresh run requests 32,768 transitions. Evaluation `--steps` instead
counts control-loop steps **per environment**. Thus `--steps 1200 --num_envs 16`
means 19,200 transitions and 40 simulated seconds per environment. Timings in old
`summary.json` exclude most app/scene startup; they are not whole-command runtime.

## Installed environment

| Component | Recorded value |
|---|---|
| Isaac Sim | 6.0.1-rc.7+release.42383.32955d8d.gl at `C:\isaacsim` |
| Isaac Lab branch | `release/3.0.0-beta2` |
| Isaac Lab source commit | `0603cb1710dcda13087665f63abf9ac483b63c05` |
| Python | 3.12.13, bundled with Isaac Sim |
| PyTorch | 2.10.0+cu128 |
| Gymnasium | 1.2.1 |
| GPU / VRAM | RTX 5070 / approximately 12 GB |
| NVIDIA driver | 610.74 |

New manifests record installed Python package versions at execution time. Those
versions should take precedence over this dated table when investigating a later run.
