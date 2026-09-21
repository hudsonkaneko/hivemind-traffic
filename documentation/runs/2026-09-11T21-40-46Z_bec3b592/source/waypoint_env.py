"""Single-vehicle waypoint task, replicated for Isaac Lab PPO training.

Units are meters, seconds, and radians. The policy commands speed and steering;
wheel contact and vehicle motion are simulated by PhysX, never teleported during
an episode. Root poses are written only when resetting an episode.
"""
import math

import torch
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import quat_apply_inverse
from isaaclab_physx.physics import PhysxCfg

ASSET_URL = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/Robots/NVIDIA/Leatherback/leatherback.usd"
STEERING = ["Knuckle__Upright__Front_Left", "Knuckle__Upright__Front_Right"]
WHEELS = ["Wheel__Knuckle__Front_Left", "Wheel__Knuckle__Front_Right",
          "Wheel__Upright__Rear_Left", "Wheel__Upright__Rear_Right"]


@configclass
class WaypointCfg(DirectRLEnvCfg):
    decimation = 4
    episode_length_s = 20.0
    action_space = 2
    observation_space = 9
    state_space = 0
    sim = SimulationCfg(dt=1 / 120, render_interval=4, physics=PhysxCfg())
    scene = InteractiveSceneCfg(num_envs=16, env_spacing=120.0, replicate_physics=False, clone_in_fabric=False)
    target_angle = 0.65
    target_min_distance = 2.0
    target_max_distance = 5.0
    success_radius = 0.4
    max_speed = 1.0
    max_steering = 0.5
    show_targets = False
    robot_cfg = ArticulationCfg(
        prim_path="/World/envs/env_.*/Car",
        spawn=sim_utils.UsdFileCfg(
            usd_path=ASSET_URL,
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False, solver_position_iteration_count=8,
                solver_velocity_iteration_count=2,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(pos=(0, 0, 0.1), joint_pos={
            "(?!Shock__Rear_|Shock__Front_).*": 0.0,
            "Shock__Rear_.*": -0.03,
            "Shock__Front_.*": 0.03,
        }),
        actuators={
            "steering": ImplicitActuatorCfg(joint_names_expr=STEERING,
                stiffness=None, damping=None),
            "wheels": ImplicitActuatorCfg(joint_names_expr=WHEELS,
                stiffness=None, damping=None),
            "suspension": ImplicitActuatorCfg(joint_names_expr=["(?!Wheel__|Knuckle__).+"],
                stiffness=None, damping=None),
        },
    )


class WaypointEnv(DirectRLEnv):
    cfg: WaypointCfg

    def __init__(self, cfg, **kwargs):
        super().__init__(cfg, **kwargs)
        self.steer_ids, _ = self.car.find_joints(STEERING, preserve_order=True)
        self.wheel_ids, _ = self.car.find_joints(WHEELS, preserve_order=True)
        self.targets = torch.zeros((self.num_envs, 3), device=self.device)
        self.actions = torch.zeros((self.num_envs, 2), device=self.device)
        self.previous_actions = torch.zeros_like(self.actions)
        self.previous_distance = torch.zeros(self.num_envs, device=self.device)
        self.reached = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.failed = torch.zeros_like(self.reached)
        self.success_count = 0
        self.failure_count = 0
        self.timeout_count = 0
        self.markers = None
        if cfg.show_targets:
            from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
            self.markers = VisualizationMarkers(VisualizationMarkersCfg(
                prim_path="/World/Targets",
                markers={"goal": sim_utils.SphereCfg(radius=0.15,
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.1, 0.95, 0.3)))},
            ))

    def _setup_scene(self):
        self.car = Articulation(self.cfg.robot_cfg)
        # This asset's collision group cannot be replicated by PhysX. Self
        # collisions are already disabled on the articulation, so the group is
        # unnecessary for the isolated one-car task.
        group = self.sim.stage.GetPrimAtPath("/World/envs/env_0/Car/CollisionGroup")
        if group.IsValid():
            group.SetActive(False)
        ground = sim_utils.GroundPlaneCfg(size=(3000, 3000), color=(0.18, 0.20, 0.23))
        ground.func("/World/Ground", ground)
        self.scene.clone_environments(copy_from_source=False)
        self.scene.articulations["car"] = self.car
        if self.device == "cpu":
            self.scene.filter_collisions(global_prim_paths=["/World/Ground"])
        light = sim_utils.DomeLightCfg(intensity=1500.0)
        light.func("/World/Light", light)

    def _pre_physics_step(self, actions):
        self.previous_actions.copy_(self.actions)
        self.actions = actions.clamp(-1, 1).clone()
        self.previous_distance = self._distance()

    def _apply_action(self):
        # Ackermann geometry without a division by zero at straight steering.
        speed = (self.actions[:, 0] + 1) * 0.5 * self.cfg.max_speed
        tangent = torch.tan(self.actions[:, 1] * self.cfg.max_steering)
        curvature = tangent / 0.32
        left = 1 - 0.1208 * curvature
        right = 1 + 0.1208 * curvature
        angles = torch.stack((torch.atan(tangent / left), torch.atan(tangent / right)), dim=1)
        wheel_speed = speed / 0.052
        velocities = torch.stack((wheel_speed * torch.sqrt(left**2 + tangent**2),
                                  wheel_speed * torch.sqrt(right**2 + tangent**2),
                                  wheel_speed * left, wheel_speed * right), dim=1)
        self.car.set_joint_position_target_index(target=angles, joint_ids=self.steer_ids)
        self.car.set_joint_velocity_target_index(target=velocities, joint_ids=self.wheel_ids)

    def _distance(self):
        return torch.linalg.vector_norm(self.targets[:, :2] - self.car.data.root_pos_w.torch[:, :2], dim=1)

    def _get_observations(self):
        if self.markers is not None:
            points = self.targets.clone()
            points[:, 2] = 0.15
            self.markers.visualize(translations=points)
        delta = self.targets - self.car.data.root_pos_w.torch
        local = quat_apply_inverse(self.car.data.root_quat_w.torch, delta)
        obs = torch.cat((local[:, :2] / 5,
                         self.car.data.root_lin_vel_b.torch[:, :2] / self.cfg.max_speed,
                         self.car.data.root_ang_vel_b.torch[:, 2:3],
                         self.car.data.joint_pos.torch[:, self.steer_ids] / self.cfg.max_steering,
                         self.actions), dim=1)
        return {"policy": obs}

    def _get_dones(self):
        self.reached = self._distance() < self.cfg.success_radius
        pos = self.car.data.root_pos_w.torch - self.scene.env_origins
        q = self.car.data.root_quat_w.torch
        upright = 1 - 2 * (q[:, 0] ** 2 + q[:, 1] ** 2)
        self.failed = (upright < 0.5) | (pos[:, 2] < -1) | (pos[:, 2] > 3) | (pos[:, :2].abs().max(dim=1).values > 48)
        timeout = self.episode_length_buf >= self.max_episode_length - 1
        return self.reached | self.failed, timeout

    def _get_rewards(self):
        progress = self.previous_distance - self._distance()
        change = (self.actions - self.previous_actions).square().sum(dim=1)
        return 4 * progress - 0.01 - 0.01 * change + 20 * self.reached.float() - 20 * self.failed.float()

    def _reset_idx(self, env_ids):
        if env_ids is None:
            env_ids = self.car._ALL_INDICES
        self.success_count += int(self.reached[env_ids].sum().item())
        self.failure_count += int(self.failed[env_ids].sum().item())
        self.timeout_count += int((self.reset_time_outs[env_ids] & ~self.reached[env_ids] & ~self.failed[env_ids]).sum().item())
        super()._reset_idx(env_ids)
        n = len(env_ids)
        pose = self.car.data.default_root_pose.torch[env_ids].clone()
        pose[:, :3] += self.scene.env_origins[env_ids]
        yaw = torch.rand(n, device=self.device) * 2 * math.pi
        # Isaac Lab 3 uses xyzw; standalone Isaac Sim uses wxyz.
        pose[:, 3:5] = 0
        pose[:, 5] = torch.sin(yaw / 2)
        pose[:, 6] = torch.cos(yaw / 2)
        vel = torch.zeros((n, 6), device=self.device)
        self.car.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
        self.car.write_root_velocity_to_sim_index(root_velocity=vel, env_ids=env_ids)
        self.car.write_joint_position_to_sim_index(position=self.car.data.default_joint_pos.torch[env_ids].clone(), env_ids=env_ids)
        self.car.write_joint_velocity_to_sim_index(velocity=self.car.data.default_joint_vel.torch[env_ids].clone(), env_ids=env_ids)
        angle = yaw + (torch.rand(n, device=self.device) * 2 - 1) * self.cfg.target_angle
        distance = self.cfg.target_min_distance + torch.rand(n, device=self.device) * (self.cfg.target_max_distance - self.cfg.target_min_distance)
        self.targets[env_ids] = pose[:, :3]
        self.targets[env_ids, 0] += distance * torch.cos(angle)
        self.targets[env_ids, 1] += distance * torch.sin(angle)
        self.actions[env_ids] = 0
        self.previous_actions[env_ids] = 0
        self.reached[env_ids] = False
        self.failed[env_ids] = False
