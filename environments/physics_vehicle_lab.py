"""One native PhysX vehicle in a real Isaac Lab DirectRLEnv.

Import after AppLauncher starts Kit. This deliberately rejects replication:
native vehicle cloning, multi-agent operation and RTX sensing are separate gates.
The chassis is a Lab RigidObject for tensor read/reset, NOT an Articulation.
"""
import gymnasium as gym
import numpy as np
import torch
from isaaclab.assets import RigidObject, RigidObjectCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from isaaclab_physx.physics import PhysxCfg
from pxr import UsdGeom

from environments.physics_vehicle_contract import (
    ACTION_HIGH, ACTION_LOW, OBSERVATION_NAMES, bounded_action, observation_values,
)
from traffic.driver_control import AppliedControl, DriverCommand, DriverControlGate
from traffic.physx_vehicle import PhysxVehicle


@configclass
class PhysicsVehicleLabCfg(DirectRLEnvCfg):
    decimation = 2
    episode_length_s = 8.0
    seed = 101
    action_space = gym.spaces.Box(np.array(ACTION_LOW, dtype=np.float32),
                                  np.array(ACTION_HIGH, dtype=np.float32))
    observation_space = len(OBSERVATION_NAMES)
    state_space = 0
    sim = SimulationCfg(dt=1 / 120, render_interval=2, device='cpu', use_fabric=False,
                        physics_prim_path='/World/PhysicsScene',
                        enable_scene_query_support=True, physics=PhysxCfg())
    scene = InteractiveSceneCfg(num_envs=1, env_spacing=100.0,
                                replicate_physics=False, clone_in_fabric=False)
    drive_torque_per_front_wheel_nm = 700.0
    brake_torque_per_wheel_nm = 1500.0
    command_ttl_ticks = 12


class PhysicsVehicleLabEnv(DirectRLEnv):
    cfg: PhysicsVehicleLabCfg

    def __init__(self, cfg, **kwargs):
        if cfg.scene.num_envs != 1 or cfg.sim.device != 'cpu' or cfg.sim.use_fabric:
            raise ValueError('Native vehicle compatibility task requires one env, CPU physics, use_fabric=False')
        if cfg.decimation < 1 or not float(cfg.decimation).is_integer():
            raise ValueError('decimation must be a positive integer')
        self.episode_serial = 0
        self.episode_tick = 0
        self.sequence = 0
        self.reset_count = 0
        self.gate = DriverControlGate('lab-0', 'ego')
        self.applied = AppliedControl(0.0, 0.0, 1.0, 'initial', None, True)
        self.pending_command = None
        self.last_control = None
        self.last_state = None
        super().__init__(cfg, **kwargs)
        self.render_enabled = False
        if self._physics_handles_decimation:
            raise RuntimeError('Task requires one _apply_action call per physics tick')

    def _setup_scene(self):
        self.vehicle = PhysxVehicle(self.sim.stage, round(1 / self.cfg.sim.dt))
        transform = UsdGeom.Xformable(self.sim.stage.GetPrimAtPath(self.vehicle.path)).ComputeLocalToWorldTransform(0)
        self.initial_position = tuple(transform.ExtractTranslation())
        quat = transform.ExtractRotationQuat()
        self.initial_quaternion = (*tuple(quat.GetImaginary()), quat.GetReal())
        self.chassis = RigidObject(RigidObjectCfg(
            prim_path=self.vehicle.path, spawn=None,
            init_state=RigidObjectCfg.InitialStateCfg(pos=self.initial_position,
                                                       rot=self.initial_quaternion)))
        self.scene.rigid_objects['chassis'] = self.chassis

    def _pre_physics_step(self, actions):
        if tuple(actions.shape) != (1, 3):
            raise ValueError('Expected a (1, 3) steering/throttle/brake action tensor')
        values = bounded_action(actions[0].detach().cpu().tolist())
        self.actions = torch.tensor([values], dtype=torch.float32, device=self.device)
        self.pending_command = DriverCommand(
            f'lab-{self.episode_serial}', 'ego', self.sequence, self.episode_tick,
            self.episode_tick + self.cfg.command_ttl_ticks, *values)
        self.sequence += 1

    def _apply_action(self):
        self.applied = self.gate.step(tick=self.episode_tick, dt_s=self.physics_dt,
                                      command=self.pending_command)
        self.pending_command = None
        self.last_control = self.vehicle.apply(self.applied,
            self.cfg.drive_torque_per_front_wheel_nm, self.cfg.brake_torque_per_wheel_nm)
        self.episode_tick += 1

    def _get_observations(self):
        self.last_state = self.vehicle.state()
        values = observation_values(self.last_state, self.applied)
        return {'policy': torch.tensor([values], dtype=torch.float32, device=self.device)}

    def _get_dones(self):
        state = self.vehicle.state()
        self.extras['physics_state_before_reset'] = state
        self.extras['control_before_reset'] = self.last_control
        self.extras['episode_tick_before_reset'] = self.episode_tick
        self.extras['episode_serial_before_reset'] = self.episode_serial
        failure = (state['upright_z'] < 0.8 or state['position_m'][2] < 0.0
                   or state['position_m'][2] > 2.0
                   or max(abs(v) for v in state['position_m'][:2]) > 100.0)
        return (torch.tensor([failure], dtype=torch.bool, device=self.device),
                self.episode_length_buf >= self.max_episode_length)

    def _get_rewards(self):
        # Intentionally no learning claim: a training task needs an explicit objective.
        return torch.zeros(1, dtype=torch.float32, device=self.device)

    def _reset_idx(self, env_ids):
        if env_ids is not None and len(env_ids) == 0:
            return
        if env_ids is not None and list(env_ids) != [0]:
            raise ValueError('Only environment 0 exists in this compatibility probe')
        super()._reset_idx(env_ids)
        # Lab tensor writes are confined to this reset method. Vehicle rest-state
        # resets suspension/wheel dynamics too, unlike chassis-only teleportation.
        self.vehicle.physx.set_vehicle_to_rest_state(self.vehicle.path)
        for path in self.vehicle.wheel_paths:
            self.vehicle.physx.set_wheel_rotation_angle(path, 0.0)
        pose = torch.tensor([[*self.initial_position, *self.initial_quaternion]],
                            dtype=torch.float32, device=self.device)
        self.chassis.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
        self.chassis.write_root_velocity_to_sim_index(
            root_velocity=torch.zeros((1, 6), dtype=torch.float32, device=self.device), env_ids=env_ids)
        self.episode_serial += 1
        self.episode_tick = self.sequence = 0
        self.gate.reset(episode_id=f'lab-{self.episode_serial}', vehicle_id='ego')
        self.pending_command = None
        self.applied = AppliedControl(0.0, 0.0, 1.0, 'reset', None, True)
        self.last_control = self.vehicle.apply(self.applied,
            self.cfg.drive_torque_per_front_wheel_nm, self.cfg.brake_torque_per_wheel_nm)
        self.actions.zero_()
        self.reset_count += 1
