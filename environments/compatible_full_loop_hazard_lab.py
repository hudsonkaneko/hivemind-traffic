"""Full-scene reset preflight; no actor feature or motion-authority changes."""
from copy import deepcopy
import json

from environments.full_loop_hazard_lab import FullLoopHazardLabCfg,FullLoopHazardLabEnv
from environments.full_loop_hazard_contract import background_settings,background_specs
from experiments.full_scene_hazard_preflight import background_plan,choose_schedule
from traffic.live_showcase import LiveTraffic


class CompatibleFullLoopHazardLabEnv(FullLoopHazardLabEnv):
    def __init__(self,cfg,**kwargs):
        self.pending_hazard_background_preflight=None
        self.preplay_parking_initialization=None
        super().__init__(cfg,**kwargs)

    def _save_full_scene(self):
        """Complete dormant parking poses before export and physics initialization.

        The reused LiveKinematicFleet changes dormant translations at authoring
        time but retains their base road orientations until the first target.
        Complete those initial orientations here so the very first measured
        all-pool pose check is meaningful. No active car attribute is written.
        """
        if (self._background_bound or self.background._view is not None
                or self.live_traffic.time_s != 0. or self.background._target_count != 0):
            raise RuntimeError('Dormant parking initialization requires pre-physics authoring')
        from pxr import Gf, Usd
        paths=[]
        with Usd.EditContext(self.sim.stage,self.background.dynamic_layer):
            for spec in self.background.specs:
                if self.background.is_active(spec):continue
                x,y,z,w=self.live_traffic.pose(spec.vehicle_id)['quaternion_xyzw']
                for path,quaternion in ((spec.chassis_path,Gf.Quatf(w,x,y,z)),
                                        (spec.render_path,Gf.Quatd(w,x,y,z))):
                    attribute=self.sim.stage.GetPrimAtPath(path).GetAttribute('xformOp:orient')
                    if not attribute or not attribute.Set(quaternion):
                        raise RuntimeError('Cannot author declared dormant orientation: '+path)
                    paths.append(path)
        self.preplay_parking_initialization=dict(dormant_slots=len(paths)//2,
            orientation_attributes=paths,active_pose_writes=0,native_pose_writes=0,
            physics_steps=0,phase='before full scene export and physics initialization')
        return super()._save_full_scene()

    def _make_schedule(self,index):
        # Initial authoring happens before FullLoop has built its fleet. That
        # exact same fixed plan is used here; later resets read the live phase.
        live=getattr(self,'live_traffic',None)
        initial=live is None
        if initial:live=LiveTraffic(background_specs(),background_settings())
        plan=background_plan(live)
        native=None
        if getattr(self,'_background_bound',False):
            native=self.background.pose_check(live.time_s)
            if native.get('passed') is not True:
                detail=dict(native_pose_check=native,background_plan=plan,
                    global_physics_tick=getattr(self,'_total_physics_ticks',None),
                    preplay_parking_initialization=getattr(self,'preplay_parking_initialization',None))
                raise RuntimeError('Current native background differs from compatibility plan: '+
                    json.dumps(detail,allow_nan=False,separators=(',',':')))
        side=self.cfg.hazard_side if self.cfg.hazard_stage=='crossing' else None
        schedule,report=choose_schedule(self.cfg.hazard_stage,self._reset_seed,index,self.route,plan,side=side)
        report.update(phase='pre_play_authoring' if initial else 'before_physical_reset',
            native_current_background=deepcopy(native),
            candidate_acceptance='scheduled complete35s clearance; no policy actions or outcomes consulted')
        self.pending_hazard_background_preflight=report
        return schedule

    def scene_contract(self):
        report=super().scene_contract()
        report['preplay_parking_initialization']=deepcopy(self.preplay_parking_initialization)
        return report

    def _reset_idx(self,env_ids):
        if env_ids is not None and len(env_ids)==0:return
        # Parent asks _make_schedule BEFORE set_transforms on the hazard/ego.
        super()._reset_idx(env_ids)
        self.last_reset['hazard_background_preflight']=deepcopy(self.pending_hazard_background_preflight)
        self.extras['reset']=deepcopy(self.last_reset)
