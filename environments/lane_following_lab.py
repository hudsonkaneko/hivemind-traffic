"""One composed native PhysX lane task in Isaac Lab; import after AppLauncher.

This is an environment/scripted-validation milestone, not permission to train.
The ordinary observation after a done is the reset observation, as in Lab.
Detached pre-reset observations, rewards and all-step records live in extras.
"""
from copy import deepcopy
from dataclasses import asdict
import math
from pathlib import Path

import gymnasium as gym
import numpy as np
import torch
from isaaclab.assets import RigidObject, RigidObjectCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from isaaclab_physx.physics import PhysxCfg

from environments.lane_following_contract import (
    OBSERVATION_NAMES, LaneFollowingTask, LaneTaskConfig, make_reset_spec,
)
from environments.physics_vehicle_contract import ACTION_HIGH, ACTION_LOW, bounded_action
from environments.lane_following_scene import (
    author_lane_view, bind_composed_vehicle, save_lane_scene,
    update_lane_view, validate_lane_scene,
)
from traffic.driver_control import AppliedControl, DriverCommand, DriverControlGate
from traffic.lane_geometry import load_routes
from traffic.path_following import FollowerConfig, PathFollower, PathReference, VehicleState
from traffic.vehicle_contacts import VehicleContactMonitor
from usd.physical_scene import PATHS, asset_hashes

ROOT = Path(__file__).resolve().parents[1]
CONTACT_PROCESSING_SETTING = '/physics/disableContactProcessing'


@configclass
class LaneFollowingLabCfg(DirectRLEnvCfg):
    decimation = 2
    episode_length_s = 35.0
    seed = 101
    action_space = gym.spaces.Box(np.array(ACTION_LOW, dtype=np.float32),
                                  np.array(ACTION_HIGH, dtype=np.float32))
    observation_space = len(OBSERVATION_NAMES)
    state_space = 0
    sim = SimulationCfg(dt=1 / 120, render_interval=4, device='cpu', use_fabric=False,
                        physics_prim_path=PATHS.physics, enable_scene_query_support=True,
                        physics=PhysxCfg())
    scene = InteractiveSceneCfg(num_envs=1, env_spacing=100.0,
                                replicate_physics=False, clone_in_fabric=False)
    # Lab's configclass recursively assigns nested dataclass fields. Keep its
    # serialized values here; construct the immutable task contract at runtime.
    task = asdict(LaneTaskConfig())
    route_id = 'straight100'
    routes_file = str(ROOT / 'scenarios/physics-road/routes.json')
    scene_directory = None
    reset_perturb = False
    render_during_step = False
    diagnostic_contact_barrier = False
    drive_torque_per_front_wheel_nm = 700.0
    brake_torque_per_wheel_nm = 1500.0
    command_ttl_ticks = 12


class LaneFollowingLabEnv(DirectRLEnv):
    cfg: LaneFollowingLabCfg

    def __init__(self, cfg, **kwargs):
        if (cfg.scene.num_envs != 1 or cfg.scene.replicate_physics or cfg.scene.clone_in_fabric
                or cfg.sim.device != 'cpu' or cfg.sim.use_fabric):
            raise ValueError('Lane task requires one CPU PhysX environment without cloning/Fabric')
        if not isinstance(cfg.sim.physics, PhysxCfg) or cfg.sim.physics_prim_path != PATHS.physics:
            raise ValueError('Lane task requires the declared native PhysX scene')
        if cfg.decimation != 2 or not math.isclose(cfg.sim.dt, 1 / 120, abs_tol=1e-12):
            raise ValueError('Lane task is fixed to 120 Hz physics and 60 Hz control')
        if not isinstance(cfg.task, dict) or set(cfg.task) != set(asdict(LaneTaskConfig())):
            raise ValueError('Lab task config must contain the complete serialized lane contract')
        task_config = LaneTaskConfig(**cfg.task)
        if cfg.episode_length_s != task_config.episode_length_s:
            raise ValueError('Lab episode length must equal the lane task contract')
        if cfg.events or cfg.action_noise_model or cfg.observation_noise_model:
            raise ValueError('Events and action/observation noise are outside this task contract')
        if any(type(v) is not bool for v in (cfg.reset_perturb, cfg.render_during_step, cfg.diagnostic_contact_barrier)):
            raise ValueError('Reset perturbation and rendering flags must be boolean')
        if type(cfg.seed) is not int or not 0 <= cfg.seed <= 2**32 - 1:
            raise ValueError('An unsigned 32-bit integer seed is required')
        if (cfg.drive_torque_per_front_wheel_nm != 700.0 or
                cfg.brake_torque_per_wheel_nm != 1500.0 or cfg.command_ttl_ticks != 12):
            raise ValueError('This milestone retains the validated actuator/TTL contract')
        if cfg.scene_directory is None:
            raise ValueError('Supply a fresh scene_directory within this checkout outputs')
        directory = Path(cfg.scene_directory).resolve()
        if not directory.is_relative_to(ROOT / 'outputs') or directory.exists():
            raise ValueError('scene_directory must be a fresh path under project outputs')
        self.scene_directory = directory
        self.route = load_routes(cfg.routes_file)[cfg.route_id]
        self.task = LaneFollowingTask(self.route, task_config)
        self.episode_serial = 0
        self.episode_tick = 0
        self.sequence = 0
        self.reset_count = 0
        self._total_physics_ticks = 0
        self._sampled_tick = -1
        self._unsupported_ticks = 0
        self._reset_seed = cfg.seed
        self._next_reset_index = 0
        self._reset_options = {}
        self._initialized_episode = False
        self._transition = None
        self._failure = None
        self._step_samples = []
        self._step_contacts = []
        self.last_state = None
        self.last_control = None
        self.last_reset = None
        self.pending_command = None
        self._contact_settings_owner = None
        self.contact_reporting = None
        self.gate = DriverControlGate('lane-lab-0', 'ego')
        self.applied = AppliedControl(0.0, 0.0, 1.0, 'initial', None, True)
        try:
            super().__init__(cfg, **kwargs)
            if self._physics_handles_decimation:
                self.close()
                raise RuntimeError('Lane task requires one action callback per native physics tick')
            self.vehicle.bind_native_state(self.sim.physics_sim_view)
            self._contact_reporting_snapshot('native_ready')
            self.vehicle.metadata['contact_reporting'] = deepcopy(self.contact_reporting)
            self.render_enabled = cfg.render_during_step
        except Exception:
            self._restore_contact_processing()
            raise

    def _enable_contact_processing(self):
        # Installed Lab PhysX initialization disables contact processing. Its
        # ContactSensor explicitly re-enables this same process-local setting;
        # our native report monitor needs that lifecycle step too.
        previous = self.sim.get_setting(CONTACT_PROCESSING_SETTING)
        if type(previous) is not bool:
            raise RuntimeError('Contact-processing setting must have a boolean native readback')
        self._contact_settings_owner = self.sim
        self.contact_reporting = {'setting': CONTACT_PROCESSING_SETTING,
            'previous_disable_contact_processing': previous,
            'requested_disable_contact_processing': False,
            'enable_phase': 'setup_scene_after_lab_settings_before_vehicle_initialization',
            'readback_checks': 0, 'snapshots': [], 'cleanup': None}
        self.sim.set_setting(CONTACT_PROCESSING_SETTING, False)
        self._require_contact_processing()

    def _require_contact_processing(self):
        if self._contact_settings_owner is None or self.contact_reporting is None:
            raise RuntimeError('Native contact reporting was not initialized')
        current = self._contact_settings_owner.get_setting(CONTACT_PROCESSING_SETTING)
        self.contact_reporting['last_disable_contact_processing'] = current
        self.contact_reporting['readback_checks'] += 1
        if current is not False:
            raise RuntimeError('Native contact processing became disabled or unavailable')

    def _contact_reporting_snapshot(self, phase):
        from pxr import PhysxSchema, UsdUtils
        self._require_contact_processing()
        cached_id = UsdUtils.StageCache.Get().GetId(self.sim.stage).ToLongInt()
        prim = self.sim.stage.GetPrimAtPath(self.vehicle.path)
        snapshot = {'phase': phase, 'stage_cache_id': cached_id,
            'monitor_stage_id': self.contacts.stage_id,
            'vehicle_path': self.contacts.vehicle_path,
            'contact_report_api': prim.HasAPI(PhysxSchema.PhysxContactReportAPI),
            'threshold': prim.GetAttribute('physxContactReport:threshold').Get()}
        self.contact_reporting['snapshots'].append(snapshot)
        if (cached_id <= 0 or cached_id != self.contacts.stage_id
                or self.contacts.vehicle_path != self.vehicle.path
                or not snapshot['contact_report_api'] or snapshot['threshold'] != 0.0):
            raise RuntimeError('Native contact monitor stage/schema identity mismatch')
        return snapshot

    def _restore_contact_processing(self):
        owner = self._contact_settings_owner
        if owner is None:
            return
        previous = self.contact_reporting['previous_disable_contact_processing']
        try:
            owner.set_setting(CONTACT_PROCESSING_SETTING, previous)
            current = owner.get_setting(CONTACT_PROCESSING_SETTING)
            self.contact_reporting['cleanup'] = {'restored_value': current, 'passed': current is previous}
        except Exception as exc:
            self.contact_reporting['cleanup'] = {'passed': False, 'error': f'{type(exc).__name__}: {exc}'}
        finally:
            self._contact_settings_owner = None

    def _setup_scene(self):
        import omni.physx
        from pxr import UsdGeom
        self._enable_contact_processing()
        self.vehicle = bind_composed_vehicle(self.sim.stage, 120, self.scene_directory)
        transform = UsdGeom.Xformable(self.sim.stage.GetPrimAtPath(self.vehicle.path)).ComputeLocalToWorldTransform(0)
        self.initial_height_m = float(transform.ExtractTranslation()[2])
        self.road_view, self.runtime_layer = author_lane_view(self.sim.stage, self.route, self.scene_directory,
            diagnostic_contact_barrier=self.cfg.diagnostic_contact_barrier)
        self.vehicle.metadata['diagnostic_contact_barrier'] = self.cfg.diagnostic_contact_barrier
        self.contacts = VehicleContactMonitor(self.sim.stage, self.vehicle.path)
        self._contact_reporting_snapshot('monitor_authored')
        self._physx_sim = omni.physx.get_physx_simulation_interface()
        self.chassis = RigidObject(RigidObjectCfg(
            prim_path=self.vehicle.path, spawn=None,
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, self.initial_height_m),
                                                       rot=(0.0, 0.0, 0.0, 1.0))))
        self.scene.rigid_objects['chassis'] = self.chassis
        initial_state = {'position_m': [0.0, 0.0, self.initial_height_m], 'yaw_rad': 0.0}
        for mode in ('follow', 'overview'):
            update_lane_view(self.sim.stage, self.road_view, self.runtime_layer, initial_state, mode=mode)
        self.initial_scene_contract = validate_lane_scene(self.sim.stage)
        self.scene_export = save_lane_scene(self.sim.stage, self.scene_directory, self.road_view, self.runtime_layer)
        self._static_hashes = dict(self.scene_export['static_hashes'])

    def reset(self, seed=None, options=None):
        if seed is not None:
            if type(seed) is not int or not 0 <= seed <= 2**32 - 1:
                raise ValueError('Reset seed must be an unsigned 32-bit integer')
            self._reset_seed = seed
            self._next_reset_index = 0
        options = {} if options is None else dict(options)
        if set(options) - {'perturb', 'lateral_offset_m', 'yaw_error_rad'}:
            raise ValueError('Unsupported reset option')
        # Validate bounded explicit diagnostic resets before touching physics.
        make_reset_spec(self.route, seed=self._reset_seed, episode_index=self._next_reset_index,
                        perturb=options.get('perturb', self.cfg.reset_perturb),
                        lateral_offset_m=options.get('lateral_offset_m'),
                        yaw_error_rad=options.get('yaw_error_rad'))
        self._reset_options = options
        self.extras = {}
        return super().reset(seed=seed, options=options)

    def _pre_physics_step(self, actions):
        if not self._initialized_episode:
            raise RuntimeError('Reset the lane task before stepping')
        self._require_contact_processing()
        if tuple(actions.shape) != (1, 3):
            raise ValueError('Expected [1,3] physical steering/throttle/brake action')
        values = bounded_action(actions[0].detach().cpu().tolist())
        self.actions = torch.tensor([values], dtype=torch.float32, device=self.device)
        self.requested_action = values
        self.extras = {}
        self._transition = None
        self._step_samples = []
        self._step_contacts = []
        if self.episode_tick < round(self.task.config.settle_s * 120) or self._failure:
            values = (0.0, 0.0, 1.0)
        self.pending_command = self._command(values)

    def _command(self, values):
        command = DriverCommand(f'lane-lab-{self.episode_serial}', 'ego', self.sequence,
                                self.episode_tick, self.episode_tick + self.cfg.command_ttl_ticks,
                                *values)
        self.sequence += 1
        return command

    def _observe_completed_tick(self):
        if self._sampled_tick == self.episode_tick:
            return
        state = self.vehicle.state(refresh_usd=False)
        events = self.contacts.sample(self._physx_sim, self._total_physics_ticks) if self.episode_tick else []
        self._unsupported_ticks = (self._unsupported_ticks + 1
            if self.episode_tick > round(self.task.config.settle_s * 120)
            and not all(state['wheel_on_ground']) else 0)
        reason = self.task.physics_guard(state, tick=self.episode_tick,
                                         unsupported_ticks=self._unsupported_ticks,
                                         contact_count=len(events), command_fallback=self.applied.is_fallback)
        if reason is not None and self._failure is None:
            self._failure = {'tick': self.episode_tick, 'reason': reason, 'physics_state': deepcopy(state)}
        self._step_contacts.extend(deepcopy(events))
        self._step_samples.append({'tick': self.episode_tick, 'physics_state': deepcopy(state),
                                   'contact_count': len(events), 'contacts': deepcopy(events),
                                   'unsupported_ticks': self._unsupported_ticks, 'guard_reason': reason})
        self.last_state = state
        self._sampled_tick = self.episode_tick

    def _apply_action(self):
        # Observe the preceding substep before PhysX can overwrite contact data.
        self._observe_completed_tick()
        if self._failure:
            self.pending_command = self._command((0.0, 0.0, 1.0))
        self.applied = self.gate.step(tick=self.episode_tick, dt_s=self.physics_dt,
                                      command=self.pending_command)
        self.pending_command = None
        self.last_control = self.vehicle.apply(self.applied,
            self.cfg.drive_torque_per_front_wheel_nm, self.cfg.brake_torque_per_wheel_nm)
        self.episode_tick += 1
        self._total_physics_ticks += 1

    def _get_dones(self):
        self._observe_completed_tick()
        self._transition = self.task.advance(self.last_state, self.applied, tick=self.episode_tick,
            contact_count=len(self._step_contacts), command_fallback=self.applied.is_fallback,
            failure_reason=None if self._failure is None else self._failure['reason'])
        tr = self._transition
        self._observation = tuple(tr.observation)
        terminated = torch.tensor([tr.terminated], dtype=torch.bool, device=self.device)
        truncated = torch.tensor([tr.truncated], dtype=torch.bool, device=self.device)
        final_obs = torch.tensor([tr.observation], dtype=torch.float32, device=self.device)
        record = tr.as_dict()
        record.update(episode_serial=self.episode_serial, episode_tick=self.episode_tick,
                      physics_state=deepcopy(self.last_state), control=deepcopy(self.last_control),
                      requested_action=list(self.requested_action),
                      contacts=deepcopy(self._step_contacts), physics_samples=deepcopy(self._step_samples),
                      first_guard_failure=deepcopy(self._failure))
        self.extras.update({
            'lane_transition': deepcopy(record),
            'final_observation': {'policy': final_obs.detach().clone()},
            '_final_observation': (terminated | truncated).detach().clone(),
            'final_reward': torch.tensor([tr.reward], dtype=torch.float32, device=self.device),
            'final_terminated': terminated.detach().clone(),
            'final_truncated': truncated.detach().clone(),
            'final_info': deepcopy(record),
        })
        return terminated, truncated

    def _get_rewards(self):
        if self._transition is None:
            raise RuntimeError('Transition must be evaluated before reward')
        return torch.tensor([self._transition.reward], dtype=torch.float32, device=self.device)

    def _get_observations(self):
        if not self._initialized_episode:
            raise RuntimeError('Reset the lane task before observations')
        return {'policy': torch.tensor([self._observation], dtype=torch.float32, device=self.device)}

    def _reset_idx(self, env_ids):
        ids = [0] if env_ids is None else [int(x) for x in env_ids]
        if not ids:
            return
        if ids != [0]:
            raise ValueError('Only environment zero is supported')
        super()._reset_idx(env_ids)
        options = self._reset_options
        self._reset_options = {}
        spec = make_reset_spec(self.route, seed=self._reset_seed, episode_index=self._next_reset_index,
            perturb=options.get('perturb', self.cfg.reset_perturb),
            lateral_offset_m=options.get('lateral_offset_m'), yaw_error_rad=options.get('yaw_error_rad'))
        self._next_reset_index += 1
        self.vehicle.physx.set_vehicle_to_rest_state(self.vehicle.path)
        for path in self.vehicle.wheel_paths:
            self.vehicle.physx.set_wheel_rotation_angle(path, 0.0)
        quaternion = (0.0, 0.0, math.sin(spec.yaw_rad / 2), math.cos(spec.yaw_rad / 2))
        pose = torch.tensor([[spec.x_m, spec.y_m, self.initial_height_m, *quaternion]],
                            dtype=torch.float32, device=self.device)
        self.chassis.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
        self.chassis.write_root_velocity_to_sim_index(
            root_velocity=torch.zeros((1, 6), dtype=torch.float32, device=self.device), env_ids=env_ids)
        self.episode_serial += 1
        self.episode_tick = self.sequence = 0
        self._sampled_tick = 0
        self._unsupported_ticks = 0
        self._failure = None
        self.pending_command = None
        self.gate.reset(episode_id=f'lane-lab-{self.episode_serial}', vehicle_id='ego')
        self.applied = AppliedControl(0.0, 0.0, 1.0, 'reset', None, True)
        self.last_control = self.vehicle.apply(self.applied,
            self.cfg.drive_torque_per_front_wheel_nm, self.cfg.brake_torque_per_wheel_nm)
        self.actions.zero_()
        self.last_state = self.vehicle.state(refresh_usd=False)
        self._observation = self.task.initialize(self.last_state, self.applied, tick=0)
        self._initialized_episode = True
        self.reset_count += 1
        self.follower = PathFollower({self.route.route_id: self.route},
            f'lane-lab-{self.episode_serial}', 'ego', FollowerConfig())
        from omni.physx.bindings._physx import VEHICLE_WHEEL_STATE_ROTATION_SPEED
        self.last_reset = {'reset_spec': asdict(spec), 'episode_serial': self.episode_serial,
            'episode_tick': 0, 'reset_count': self.reset_count,
            'physics_state': deepcopy(self.last_state), 'observation': list(self._observation),
            'wheel_rotation_speed_rad_s': [float(self.vehicle.physx.get_wheel_state(p)[VEHICLE_WHEEL_STATE_ROTATION_SPEED])
                                           for p in self.vehicle.wheel_paths]}
        # Preserve pre-reset extras through automatic reset. Manual reset clears
        # extras in reset(); this additional record describes the next episode.
        self.extras['reset'] = deepcopy(self.last_reset)

    def scripted_action(self):
        """One call per control tick; same physical action interface as an actor."""
        if self.episode_tick < round(self.task.config.settle_s * 120):
            return torch.tensor([[0.0, 0.0, 1.0]], dtype=torch.float32, device=self.device)
        identity = f'lane-lab-{self.episode_serial}'
        state = VehicleState.from_physics(self.last_state, episode_id=identity,
                                         vehicle_id='ego', tick=self.episode_tick)
        reference = PathReference(identity, 'ego', self.route.route_id, self.episode_tick,
                                  self.episode_tick + 24, self.task.config.target_speed_m_s,
                                  self.task.config.stop_s_m)
        command = self.follower.command(state, self.episode_tick, reference)
        return torch.tensor([[command.steering_rad, command.throttle, command.brake]],
                            dtype=torch.float32, device=self.device)

    def render_snapshot(self, mode='follow', *, render=True):
        """Prepare a named camera and optionally render; never advance physics."""
        if not self._initialized_episode:
            raise RuntimeError('Reset before preparing a screenshot')
        before = self.episode_tick
        self.vehicle.physx.update_transformations(False, True, True, False)
        camera_path = update_lane_view(self.sim.stage, self.road_view, self.runtime_layer,
                                       self.last_state, mode=mode)
        if render:
            self.sim.render()
        if self.episode_tick != before:
            raise RuntimeError('Rendering advanced the lane episode clock')
        return {'camera_path': camera_path, 'episode_serial': self.episode_serial,
                'episode_tick': self.episode_tick, 'physics_state': deepcopy(self.last_state)}

    def scene_contract(self):
        self._contact_reporting_snapshot('scene_contract')
        report = validate_lane_scene(self.sim.stage)
        current = asset_hashes(self.scene_directory)
        report.update(static_hashes=current, static_hashes_unchanged=current == self._static_hashes,
                      entry_path=self.scene_export['entry_path'], contact_reporting=deepcopy(self.contact_reporting))
        if current != self._static_hashes:
            raise RuntimeError('Published lane scene files changed during the episode')
        return report

    def close(self):
        try:
            self.viewport_cleanup = {'attempted': False, 'passed': True}
            try:
                from omni.kit.viewport.utility import get_active_viewport
                viewport = get_active_viewport()
                if viewport and str(viewport.camera_path) in (PATHS.follow_camera, PATHS.overview_camera):
                    self.viewport_cleanup['attempted'] = True
                    viewport.camera_path = '/OmniverseKit_Persp'
                    self.sim.render()
            except ImportError:
                pass  # A headless app need not load a viewport extension.
            except Exception as exc:
                self.viewport_cleanup.update(passed=False, error=f'{type(exc).__name__}: {exc}')
            super().close()
        finally:
            # Restore only after Lab has stopped native physics. This setting
            # is process-local and no installed/persistent config is edited.
            self._restore_contact_processing()
            # Release handles after Lab tears down callbacks/native tensors.
            for name in ('vehicle', 'chassis', 'road_view', 'runtime_layer', 'contacts', '_physx_sim'):
                if hasattr(self, name):
                    setattr(self, name, None)
