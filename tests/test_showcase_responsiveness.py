import math
import unittest
from pathlib import Path
import tomllib

from experiments.loop_showcase_config import make_config,validate_config,fleet_specs
from visualization.showcase_responsiveness import assess_responsiveness,distribution_ms,prepared_runtime_inventory
from visualization.showcase_responsiveness import display_qualification_exclusions


class ResponsiveTests(unittest.TestCase):
    def test_regular_editor_default_keeps_optimizations_and_legacy_option(self):
        default=make_config('v07')
        minimal=make_config('v07',ui_profile='showcase')
        self.assertEqual(default,make_config('v07',ui_profile='editor'))
        self.assertEqual(default['ui_profile'],'editor')
        self.assertEqual({k for k in default if default[k]!=minimal[k]},{'ui_profile'})
        self.assertEqual(fleet_specs(default),fleet_specs(minimal))
        for key,value in dict(instance_visuals=True,evidence_profile='presentation',
                physics_hz=120,control_hz=60,planner_hz=10,render_hz=30,
                statistics_hz=5,cpu_threads=8,frame_generation=False).items():
            self.assertEqual(default[key],value)
        # Saved minimal-app configurations remain valid after changing the default.
        validate_config(default)
        validate_config(minimal)
        self.assertEqual(make_config()['version'],'v04')

    def test_gui_qualification_excludes_diagnostics_pauses_and_headless(self):
        self.assertEqual(display_qualification_exclusions(make_config('v07'),0),[])
        self.assertEqual(display_qualification_exclusions(make_config('v07',gui=False,profile_render=True),1),
            ['native_profiler_enabled','user_paused','headless'])

    def test_paused_and_active_render_share_bounded_probe(self):
        from scripts.physics_loop_showcase import render_frame
        events=[]
        class Session:
            def render(self):events.append('clock_checked_render')
        class Probe:
            def after_render(self):events.append('bounded_probe')
        render_frame(Session(),Probe())
        self.assertEqual(events,['clock_checked_render','bounded_probe'])
        render_frame(Session())
        self.assertEqual(events[-1],'clock_checked_render')

    def test_config_preserves_physical_contract(self):
        old=make_config('v06');new=make_config('v07')
        self.assertEqual({k for k in old if old[k]!=new[k]},{'version','render_hz'})
        self.assertEqual(fleet_specs(old),fleet_specs(new))
        self.assertEqual(new['statistics_hz'],5)
        self.assertFalse(new['frame_generation'])
        for hz in (10,20,30,60):
            for ui in ('showcase','editor'):
                for mode in ('presentation','full'):
                    validate_config(make_config('v07',render_hz=hz,ui_profile=ui,evidence=mode))

    def test_rejects_undeclared_profiles(self):
        for version in ('v01','v04','v05','v06'):
            with self.assertRaises(ValueError):make_config(version,ui_profile='showcase')
            with self.assertRaises(ValueError):make_config(version,profile_render=True)
            with self.assertRaises(ValueError):make_config(version,cpu_threads=8)
        for kwargs in ({'render_hz':True},{'render_hz':60.},{'render_hz':120},{'ui_profile':'hidden'}):
            with self.assertRaises(ValueError):make_config('v07',**kwargs)
        for key,value in (('frame_generation',True),('physics_hz',60),('statistics_hz',60)):
            cfg=make_config('v07');cfg[key]=value
            with self.assertRaises(ValueError):validate_config(cfg)
        for count in (4,8,16,32):validate_config(make_config('v07',cpu_threads=count))
        for count in (True,8.,0,64):
            with self.assertRaises(ValueError):make_config('v07',cpu_threads=count)
        validate_config(make_config('v07',profile_render=True))

    def test_frame_gate_uses_delivered_not_submitted_frames(self):
        frames=[dict(pre=i/60,post=i/60+.001) for i in range(600)]
        good=assess_responsiveness(frames,[i/60+.005 for i in range(600)],
            wall_s=10,simulation_s=10,render_hz=60,complete=True)
        self.assertTrue(good['passed']);self.assertFalse(good['actual_mouse_latency_qualified'])
        self.assertFalse(good['unique_frame_ids_verified'])
        missing=assess_responsiveness(frames,[i/30+.005 for i in range(300)],
            wall_s=10,simulation_s=10,render_hz=60,complete=True)
        self.assertFalse(missing['passed']);self.assertFalse(missing['gates']['native_fps'])

    def test_stalls_incomplete_and_nonfinite_fail(self):
        for values,complete in (([0,.02,.3],True),([0,.016,.032],False),([0,float('nan'),1],True)):
            result=assess_responsiveness([dict(pre=t) for t in values],values,
                wall_s=1,simulation_s=1,render_hz=60,complete=complete)
            self.assertFalse(result['passed'])
        self.assertEqual(distribution_ms([])['count'],0)

    def test_prepared_runtime_does_not_enable_authoring(self):
        class Manager:
            def is_extension_enabled(self,name):return name=='omni.physx'
            def get_extensions(self):return [{'id':'omni.physx-1','enabled':True},{'id':'unused','enabled':False}]
        result=prepared_runtime_inventory(Manager())
        self.assertEqual(result['enabled_extensions'],['omni.physx-1'])
        self.assertFalse(result['authoring_tools_enabled'])

    def test_experience_keeps_physics_no_persistent_or_async_changes(self):
        config=tomllib.loads((Path(__file__).resolve().parents[1]/'visualization/highwaysim.showcase.kit').read_text())
        settings=config['settings'];deps=config['dependencies']
        self.assertIn('isaacsim.core.simulation_manager',deps)
        self.assertIn('omni.hydra.scene_delegate',deps)
        for key in ('omni.physx.vehicle','omni.kit.window.stage','omni.kit.window.property'):
            self.assertNotIn(key,deps)
        self.assertFalse(settings['app']['settings']['persistent'])
        self.assertFalse(settings['app']['asyncRendering'])
        self.assertFalse(settings['rtx-transient']['dlssg']['enabled'])


if __name__=='__main__':unittest.main()
