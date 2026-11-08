"""One-reset continuous full-road adapter. Import only after AppLauncher.

The inherited Lab35s value is the PER-WINDOW budget. The new task enforces
three windows and a105s absolute ceiling via its explicit done callback.
No intermediate or final automatic reset writes the car or hazard pose.
"""
from copy import deepcopy
from dataclasses import asdict

from environments.full_loop_hazard_lab import FullLoopHazardLabEnv,FullLoopHazardLabCfg
from environments.continuous_loop_hazard_contract import ContinuousFullLoopHazardTask
from environments.lane_following_contract import _state


class ContinuousFullLoopHazardLabEnv(FullLoopHazardLabEnv):
    def __init__(self,cfg,**kwargs):
        if cfg.hazard_stage not in ('clear','crossing'):
            raise ValueError('Continuous mode supports clear/crossing, not the static stop task')
        self.native_window_boundaries=[]
        super().__init__(cfg,**kwargs)

    def _setup_scene(self):
        super()._setup_scene()
        self.original_hazard_route=self.route
        self.task=ContinuousFullLoopHazardTask(self.route,stage=self.cfg.hazard_stage)

    def _poses(self,elapsed_s):
        # A rolling ego route must NEVER translate the hazard's world path.
        route=getattr(self,'original_hazard_route',self.route)
        frames=self.schedule.sample(route,elapsed_s)
        return [asdict(frame) for frame in frames]

    def reset(self,seed=None,options=None):
        if self.reset_count:
            raise RuntimeError('Continuous deployment permits exactly one initial reset')
        return super().reset(seed=seed,options=options)

    def _reset_idx(self,env_ids):
        if env_ids is not None and len(env_ids)==0:
            return
        if self._initialized_episode:
            if self._transition is None or not (self._transition.terminated or self._transition.truncated):
                raise RuntimeError('Unexpected mid-run reset request')
            # DirectRLEnv asks to reset done environments. Keep the terminal
            # native state and returned observation instead of teleporting.
            self.extras['autoreset_suppressed']=dict(episode_tick=self.episode_tick,
                physical_reset_count=self.reset_count,terminal_state_retained=True)
            return
        super()._reset_idx(env_ids)

    def _get_dones(self):
        result=super()._get_dones()
        context=deepcopy(self.task.last_continuation)
        roll=context['window_roll']
        if roll is not None:
            # This is a second actual native read without a physics update.
            after=_state(self.vehicle.state(refresh_usd=False))
            unchanged=after==_state(roll['physics_state_before'])
            if not unchanged or self.reset_count!=1:
                raise RuntimeError('Route boundary changed native state or reset count')
            evidence=dict(tick=self.episode_tick,physical_reset_count=self.reset_count,
                before=deepcopy(roll['physics_state_before']),after=deepcopy(after),
                equal=True,hazard_schedule_index=self.schedule.episode_index)
            self.native_window_boundaries.append(evidence)
            context['native_boundary']=evidence
            self.route=self.task.route
            # Preserve follower state and control sequence; only map geometry
            # advances. Learned inference does not use the scripted follower.
            self.follower.routes[self.route.route_id]=self.route
        record=self.extras['lane_transition']
        record['continuation']=context
        record['full_sim_scope']='one continuous physical ego; three60m windows; twelve kinematic backgrounds'
        self.extras['final_info']=deepcopy(record)
        return result

    def scene_contract(self):
        report=super().scene_contract()
        report.update(continuous_windows=3,total_goal_m=180,window_seconds=35,
            absolute_seconds_limit=105,physical_reset_count=self.reset_count,
            hazard_reseed=False,native_window_boundaries=deepcopy(self.native_window_boundaries))
        if self.reset_count not in (0,1):
            raise RuntimeError('Unexpected physical reset in continuous deployment')
        return report
