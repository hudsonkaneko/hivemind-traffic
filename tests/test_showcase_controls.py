import copy
import math
import unittest

from experiments.showcase_controls import (defaults, validate_settings, SidebarState,
    RESTART_CONTROLS, LIVE_CONTROLS, FUTURE_CONTROLS, restart_assessment_config)
from experiments.loop_showcase_config import make_config, validate_config, fleet_specs
from scripts.demo_loop_showcase import next_run_config, can_restart


class SidebarTests(unittest.TestCase):
    def test_default_fleet_preserves_v07(self):
        old=make_config('v07');new=make_config('v08')
        self.assertEqual(fleet_specs(old),fleet_specs(new))
        for key in ('target_speed_m_s','physics_hz','control_hz','planner_hz','render_hz',
                    'min_clearance_m','drive_torque_nm','brake_torque_nm','instance_visuals'):
            self.assertEqual(old[key],new[key])
        self.assertEqual(make_config()['version'],'v04')
        validate_config(new)

    def test_every_count_has_exact_unique_safe_initial_placements(self):
        for count in range(4,81):
            cfg=make_config('v08',controls={**defaults(),'traffic_count':count})
            rows=fleet_specs(cfg)
            self.assertEqual(len(rows),count)
            self.assertEqual(len({r['vehicle_id'] for r in rows}),count)
            for lane in range(4):
                stations=sorted(r['station_m'] for r in rows if r['lane']==lane)
                if len(stations)>1:
                    gaps=[(b-a)%(math.tau*500) for a,b in zip(stations,stations[1:]+stations[:1])]
                    self.assertGreater(min(gaps),20)

    def test_common_angular_speed_at_limits(self):
        for speed in (5.,15.,25.):
            rows=fleet_specs(make_config('v08',controls={**defaults(),'traffic_mph':speed}))
            self.assertTrue(all(math.isclose(r['speed_m_s']/(500+3.7*r['lane']),speed*.44704/500) for r in rows))

    def test_rejects_unknown_future_and_bad_values(self):
        for values in ({}, {**defaults(),'av_ratio':50}, {**defaults(),'traffic_count':0},
                       {**defaults(),'traffic_count':80.0}, {**defaults(),'ego_mph':36},
                       {**defaults(),'following_s':1.}, {**defaults(),'render_hz':25},
                       {**defaults(),'resolution_percent':99}, {**defaults(),'ego_mph':10.}):
            with self.assertRaises(ValueError):validate_settings(values)
        for key in defaults():
            for value in (True,False,None,'20',float('nan'),float('inf')):
                with self.assertRaises(ValueError):validate_settings({**defaults(),key:value})

    def test_live_mailbox_coalesces_and_logs_only_applied(self):
        s=SidebarState(defaults())
        for i in range(100):s.edit('camera_distance_m',8+i%50)
        self.assertEqual(len(s.live_requests),1)
        self.assertEqual(s.active['camera_distance_m'],20)
        s.consume_live(12)
        self.assertEqual(s.active['camera_distance_m'],57)
        self.assertEqual(len(s.events),1)
        s.consume_live(13);self.assertEqual(len(s.events),1)
        s.edit('camera_distance_m',57);s.consume_live(14)
        self.assertEqual(len(s.events),1)

    def test_pending_never_changes_active_physics(self):
        s=SidebarState(defaults());s.edit('ego_mph',25.)
        self.assertTrue(s.dirty);s.consume_live(12)
        self.assertEqual(s.active['ego_mph'],35.)
        self.assertEqual(s.events,[])
        s.request_restart();s.edit('ego_mph',10.)
        self.assertEqual(s.restart_request['ego_mph'],25.)

    def test_future_controls_are_excluded(self):
        s=SidebarState(defaults())
        for group in FUTURE_CONTROLS.values():
            for title,*_ in group:
                with self.assertRaises(ValueError):s.edit(title,0)
        self.assertGreater(sum(map(len,FUTURE_CONTROLS.values())),20)

    def test_new_config_is_fresh_valid_and_smoke_does_not_loop(self):
        cfg=make_config('v08',control_smoke=True)
        new=next_run_config(cfg,{**defaults(),'traffic_count':12,'ego_mph':25.})
        validate_config(new)
        self.assertFalse(new['control_smoke'])
        self.assertEqual(len(fleet_specs(new)),12)
        self.assertEqual(cfg['controls']['traffic_count'],80)

    def test_restart_requires_safe_shutdown_and_no_errors(self):
        good=dict(restart_ready=True,run_status='stopped_for_requested_restart',exit_code=0,cleanup_verified=True,
            gates={'braking':True,'stationary_hold':True},gpu_peak_mib=1000,
            runtime_error_lines=[],source_changed_during_run=[],gpu_monitor_errors=[])
        self.assertTrue(can_restart(good))
        for change in ({'exit_code':-1},{'exit_code':1},{'cleanup_verified':False},
                       {'gates':{'braking':False}}, {'error':'failed'},
                       {'session_close_error':'leak'}, {'stop_reason':'deadline'},
                       {'runtime_error_lines':['error']},{'source_changed_during_run':['code']},
                       {'usd_reference_warning_count':1},{'gpu_peak_mib':None},
                       {'run_status':'finished'},{'gpu_monitor_errors':['missing']}):
            self.assertFalse(can_restart({**good,**change}))

    def test_stop_assessment_preserves_source_config(self):
        cfg=make_config('v08');before=copy.deepcopy(cfg)
        stop=restart_assessment_config(cfg,15)
        self.assertEqual(stop['duration_s'],27);self.assertEqual(cfg,before)
        for invalid in (0,14,241,15.,True):
            with self.assertRaises(ValueError):restart_assessment_config(cfg,invalid)

    def test_sidebar_only_flags_and_corruption(self):
        for kwargs in ({'controls':defaults()},{'panel_hidden':True},{'control_smoke':True}):
            with self.assertRaises(ValueError):make_config('v07',**kwargs)
        for kwargs in ({'mode':'blocked'},{'ui_profile':'showcase'}, {'profile_render':True},
                       {'gui':False,'control_smoke':True},{'panel_hidden':True,'control_smoke':True}):
            with self.assertRaises(ValueError):make_config('v08',**kwargs)
        cfg=make_config('v08');cfg['target_speed_m_s']=99
        with self.assertRaises(ValueError):validate_config(cfg)


if __name__=='__main__':unittest.main()
