import math
import tempfile
import unittest
from types import SimpleNamespace
import numpy as np

from experiments.loop_showcase_config import make_config, fleet_specs
from experiments.showcase_controls import defaults
from traffic.live_showcase import LiveTraffic
from usd.live_showcase_fleet import LiveKinematicFleet
from usd.physical_scene import PATHS
try:
    from pxr import Usd,UsdGeom,UsdPhysics
except ImportError:Usd=None


class Bodies:
    def __init__(self,live):
        self.keys=list(reversed(live.specs))
        self.prim_paths=[live.specs[k].chassis_path for k in self.keys]
        self.transforms=np.asarray([(*live.pose(k)['position_m'],*live.pose(k)['quaternion_xyzw']) for k in self.keys],dtype=np.float32)
        self.velocities=np.zeros((80,6),dtype=np.float32)
        self.teleports=[]
    def get_transforms(self):return self.transforms
    def get_velocities(self):return self.velocities
    def set_transforms(self,data,indices):
        self.transforms[indices]=data[indices];self.teleports.extend(indices.tolist())
    def set_velocities(self,data,indices):raise AssertionError('Cannot command kinematic velocity directly')
    def set_kinematic_targets(self,data,indices):self.targets=data.copy()
    def step(self):
        self.velocities[:,:3]=(self.targets[:,:3]-self.transforms[:,:3])*120
        self.transforms=self.targets.copy()


@unittest.skipIf(Usd is None,'OpenUSD environment required')
class LiveFleetTests(unittest.TestCase):
    def test_pool_native_visibility_and_identity_survive_entry_exit(self):
        cfg=make_config('v09',controls={**defaults(),'traffic_count':12})
        live=LiveTraffic(fleet_specs(cfg),cfg['controls'])
        with tempfile.TemporaryDirectory() as directory:
            stage=Usd.Stage.CreateInMemory();UsdGeom.Xform.Define(stage,'/World')
            UsdPhysics.Scene.Define(stage,PATHS.physics)
            fleet=LiveKinematicFleet(stage,directory,fleet_specs(cfg),live=live,
                separate_visuals=True,batch_visual_updates=True,instance_visuals=True,cache_readback=True)
            bodies=Bodies(live)
            self.assertTrue(fleet.bind(SimpleNamespace(create_rigid_body_view=lambda _:bodies))['passed'])
            self.assertEqual(len(fleet.tracks(0)),12)
            self.assertTrue(fleet.publish_visuals()['passed'])
            native_paths=list(bodies.prim_paths)
            static={l.identifier:l.ExportToString() for l in stage.GetUsedLayers() if not l.anonymous}
            live.configure({**live.settings,'traffic_count':14,'traffic_mph':20.})
            for tick in range(61):
                fleet.reconcile(tick,(500,0))
                fleet.update((tick+1)/120);bodies.step()
                self.assertTrue(fleet.pose_check()['passed'])
                self.assertTrue(fleet.publish_visuals()['passed'])
            self.assertEqual(len(fleet.tracks(61)),14)
            live.configure({**live.settings,'traffic_count':12})
            for tick in range(61,122):
                fleet.reconcile(tick,(500,0))
                fleet.update((tick+1)/120);bodies.step()
            self.assertEqual(len(fleet.tracks(122)),12)
            self.assertTrue(fleet.publish_visuals()['passed'])
            self.assertEqual(len(bodies.teleports),4)
            self.assertEqual(bodies.prim_paths,native_paths)
            for i,key in enumerate(bodies.keys):
                visible=UsdGeom.Imageable(stage.GetPrimAtPath(live.specs[key].render_path)).ComputeVisibility()!='invisible'
                self.assertEqual(visible,key in live.active)
                if not visible:self.assertGreater(math.hypot(*bodies.transforms[i,:2]),10000)
            self.assertEqual(static,{l.identifier:l.ExportToString() for l in stage.GetUsedLayers() if not l.anonymous})
            self.assertTrue(fleet.static_assets_unchanged());fleet.release()


    def test_remote_gap_entry_preserves_pool_and_first_native_target_continuity(self):
        cfg=make_config('v09',controls={**defaults(),'traffic_count':79})
        specs=fleet_specs(cfg);live=LiveTraffic(specs,cfg['controls'])
        dormant=next(key for key in live.specs if key not in live.active)
        original_phase=live.specs[dormant].station_m
        ego=(500*math.cos(original_phase/500),500*math.sin(original_phase/500))
        with tempfile.TemporaryDirectory() as directory:
            stage=Usd.Stage.CreateInMemory();UsdGeom.Xform.Define(stage,'/World')
            UsdPhysics.Scene.Define(stage,PATHS.physics)
            fleet=LiveKinematicFleet(stage,directory,specs,live=live,
                separate_visuals=True,batch_visual_updates=True,instance_visuals=True,cache_readback=True)
            bodies=Bodies(live)
            self.assertTrue(fleet.bind(SimpleNamespace(create_rigid_body_view=lambda _:bodies))['passed'])
            index=bodies.keys.index(dormant)
            paths=list(bodies.prim_paths)
            static={l.identifier:l.ExportToString() for l in stage.GetUsedLayers() if not l.anonymous}
            initial_specs=dict(live.specs)
            live.configure({**live.settings,'traffic_count':80})
            for tick in range(61):
                if tick==30:
                    live.configure({**live.settings,'traffic_count':79})
                    opposite=live.station(dormant)/500+math.pi
                    current_ego=(500*math.cos(opposite),500*math.sin(opposite))
                else:
                    current_ego=ego
                if tick==60:live.configure({**live.settings,'traffic_count':80})
                before=bodies.transforms.copy();prior_events=len(live.events)
                fleet.reconcile(tick,current_ego)
                if len(live.events)>prior_events:
                    event=live.events[-1]
                    self.assertEqual(event['slot'],dormant)
                    np.testing.assert_array_equal(np.delete(bodies.transforms,index,axis=0),
                                                  np.delete(before,index,axis=0))
                    self.assertEqual(bodies.teleports[-1],index)
                    self.assertGreaterEqual(event['actual_arc_gap_m'],160.)
                    if tick==0:
                        self.assertEqual(event['placement_method'],'remote_gap_midpoint')
                        self.assertNotEqual(event['new_phase_offset_m'],0.)
                        first_identity=event['vehicle_id']
                    if tick==30:self.assertEqual(event['vehicle_id'],first_identity)
                    if tick==60:self.assertNotEqual(event['vehicle_id'],first_identity)
                entry_position=bodies.transforms[index,:3].copy()
                fleet.update((tick+1)/120)
                if tick in (0,60):
                    # A remote set_transforms already installed the entry pose;
                    # the first kinematic target is ordinary one-tick movement.
                    displacement=math.dist(entry_position,bodies.targets[index,:3])
                    self.assertTrue(.01<displacement<.1)
                bodies.step()
                self.assertTrue(fleet.pose_check()['passed'])
                self.assertTrue(fleet.publish_visuals()['passed'])
                self.assertEqual(len(fleet.tracks(tick+1)),79 if 30<=tick<60 else 80)
                visible=UsdGeom.Imageable(stage.GetPrimAtPath(live.specs[dormant].render_path)).ComputeVisibility()
                self.assertEqual(visible!='invisible',dormant in live.active)
            self.assertEqual([e['tick'] for e in live.events],[0,30,60])
            self.assertEqual([e['type'] for e in live.events],['enter','exit','enter'])
            self.assertEqual(bodies.teleports,[index,index,index])
            self.assertEqual(live.generations[dormant],2)
            self.assertEqual(live.specs,initial_specs)
            self.assertEqual(bodies.prim_paths,paths)
            self.assertEqual(static,{l.identifier:l.ExportToString() for l in stage.GetUsedLayers() if not l.anonymous})
            self.assertTrue(fleet.static_assets_unchanged())
            fleet.release()


if __name__=='__main__':unittest.main()
