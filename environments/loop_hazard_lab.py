"""Versioned one-car native hazard task, retaining the validated lane callbacks.

Import only after AppLauncher. The ego is never pose-written outside reset.
Kinematic obstacle targets have a separate, explicit owner at each physics tick.
"""
from dataclasses import asdict
from copy import deepcopy

import torch
from isaaclab.utils import configclass

from environments.lane_following_lab import LaneFollowingLabCfg, LaneFollowingLabEnv, ROOT
from environments.loop_hazard_contract import HazardLaneTask
from traffic.road_hazards import MeasuredObject, make_hazard_schedule
from usd.loop_hazard_scene import author_hazards


@configclass
class LoopHazardLabCfg(LaneFollowingLabCfg):
    observation_space = 42
    route_id = 'loop_r500'
    routes_file = str(ROOT / 'scenarios/physics-road/loop-hazard-routes-v1.json')
    hazard_stage = 'clear'
    hazard_side = None


class LoopHazardLabEnv(LaneFollowingLabEnv):
    @torch.inference_mode()
    def reset(self, seed=None, options=None):
        # RSL allocates task tensors inside inference_mode. Native reset writes
        # those tensors in place, so evaluation must use that same context.
        return super().reset(seed=seed, options=options)

    @torch.inference_mode()
    def step(self, actions):
        return super().step(actions)

    def __init__(self, cfg, **kwargs):
        self.hazards = None
        self.schedule = None
        self.hazard_frames = ()
        super().__init__(cfg, **kwargs)
        self.hazards.bind(self.sim.physics_sim_view)

    def _make_schedule(self, index):
        options = {} if self.cfg.hazard_side is None else {'side': self.cfg.hazard_side}
        return make_hazard_schedule(self.cfg.hazard_stage, self._reset_seed,
                                    index, route=self.route, **options)

    def _poses(self, elapsed_s):
        frames = self.schedule.sample(self.route, elapsed_s)
        if frames:
            return [asdict(frames[0])]
        return []  # Scene owner parks the inactive pool outside the road.

    def _setup_scene(self):
        super()._setup_scene()
        self.task = HazardLaneTask(self.route, stage=self.cfg.hazard_stage)
        self.schedule = self._make_schedule(0)
        self.hazards = author_hazards(self.sim.stage, self.scene_directory, self._poses(0.0))
        self.hazard_scene_export = self.hazards.save_entry()

    def _refresh_objects(self):
        raw = self.hazards.measured_frames(tick=self.episode_tick)
        self.hazard_frames = (() if self.cfg.hazard_stage == 'clear'
                              else tuple(MeasuredObject(**frame) for frame in raw))
        self.task.set_objects(self.hazard_frames, now_s=self.episode_tick / 120.0)

    def _reset_idx(self, env_ids):
        if env_ids is not None and len(env_ids) == 0:
            return
        self.schedule = self._make_schedule(self._next_reset_index)
        self.hazards.reset_poses(self._poses(0.0), episode_id=f'hazard-{self.episode_serial + 1}')
        self.episode_tick = 0
        self._refresh_objects()
        super()._reset_idx(env_ids)
        self.last_reset['hazard_schedule'] = self.schedule.as_dict()
        self.last_reset['hazard_frames'] = [asdict(frame) for frame in self.hazard_frames]
        self.extras['reset'] = deepcopy(self.last_reset)

    def _observe_completed_tick(self):
        fresh = self._sampled_tick != self.episode_tick
        if fresh:
            self._refresh_objects()
        super()._observe_completed_tick()
        if fresh:
            self._step_samples[-1]['hazard_frames'] = [asdict(frame) for frame in self.hazard_frames]

    def _apply_action(self):
        self._observe_completed_tick()
        self.hazards.set_targets(self._poses((self.episode_tick + 1) / 120.0),
                                 tick=self.episode_tick + 1)
        super()._apply_action()

    def _get_dones(self):
        result = super()._get_dones()
        record = self.extras['lane_transition']
        record['hazard_stage'] = self.cfg.hazard_stage
        record['hazard_frames'] = [asdict(frame) for frame in self.hazard_frames]
        record['hazard_schedule'] = self.schedule.as_dict()
        record['observation_source'] = 'privileged_current_native_state_and_known_map'
        if hasattr(self.task, 'last_hazard_report'):
            record['hazard_report'] = deepcopy(self.task.last_hazard_report)
        self.extras['final_info'] = deepcopy(record)
        return result

    def scripted_action(self):
        if self.episode_tick < round(self.task.config.settle_s * 120):
            return torch.tensor([[0.0, 0.0, 1.0]], device=self.device)
        from traffic.path_following import PathReference, VehicleState
        identity = f'lane-lab-{self.episode_serial}'
        state = VehicleState.from_physics(self.last_state, episode_id=identity,
                                         vehicle_id='ego', tick=self.episode_tick)
        reference = PathReference(identity, 'ego', self.route.route_id, self.episode_tick,
                                  self.episode_tick + 24, 3.0, 100.0)
        command = self.follower.command(state, self.episode_tick, reference)
        action = torch.tensor([[command.steering_rad, command.throttle, command.brake]], device=self.device)
        target = self.task.current_safe_speed(self.last_state, self.episode_tick)
        error = target - self.last_state['speed_m_s']
        if target <= 0:
            throttle, brake = 0.0, 1.0
        elif error < -0.05:
            throttle, brake = 0.0, min(1.0, -error * 0.35)
        elif error > 0.05:
            throttle, brake = min(1.0, error * 0.7), 0.0
        else:
            throttle, brake = 0.0, 0.0
        action[0, 1:] = torch.tensor([throttle, brake], device=self.device)
        return action

    def render_snapshot(self, mode='follow', *, render=True):
        self.hazards.publish_visuals()
        return super().render_snapshot(mode, render=render)

    def scene_contract(self):
        self._contact_reporting_snapshot('hazard_scene_contract')
        report = self.hazards.validate()
        from usd.physical_scene import asset_hashes
        current = asset_hashes(self.scene_directory)
        if current != self.hazard_scene_export['static_hashes']:
            raise RuntimeError('Published hazard scene sources changed')
        report.update(contact_reporting=deepcopy(self.contact_reporting),
                      static_hashes_unchanged=True, static_hashes=current,
                      entry_path=self.hazard_scene_export['entry_path'])
        return report

    def close(self):
        if self.hazards is not None:
            self.hazards.release()
        super().close()
