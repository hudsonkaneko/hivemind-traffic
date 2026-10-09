"""Predeclared showcase acceptance gates must reject corrupted/unsafe evidence."""
from copy import deepcopy
import math
import unittest

from experiments.loop_showcase_config import assess, fleet_specs, make_config, validate_config


def valid_rows(config):
    """Synthetic trace with exact clock, XY travel, finite safe state and a stop."""
    hz = config['physics_hz']
    drive_end = (config['settle_s'] + config['drive_s']) * hz
    distance = 0.
    speed = 0.
    rows = []
    for index in range(config['duration_s'] * hz):
        previous_speed = speed
        if index < config['settle_s'] * hz:
            phase = 'settle'
            speed = 0.
        elif index < drive_end:
            phase = 'drive'
            speed = config['target_speed_m_s']
        else:
            phase = 'brake'
            speed = max(0., previous_speed - 4. / hz)
        distance += (previous_speed + speed) * .5 / hz
        expiry = index + config['command_ttl_ticks'] if phase != 'brake' else drive_end - 1 + config['command_ttl_ticks']
        rows.append(dict(tick=index+1, applied_tick=index, sim_time_s=(index+1)/hz,
                         phase=phase, speed_m_s=speed, traveled_distance_m=distance,
                         position_m=[distance, 0., 1.], clearance_m=3., overlap=False,
                         footprint_in_road=True, wheel_on_ground=[True]*4, upright_z=1.,
                         background_pose_error_m=0., command_expires_tick=expiry,
                         fallback=config['mode']=='dropout' and index>=expiry,
                         throttle=0. if phase=='brake' else .2,
                         brake=1. if phase=='brake' else 0.))
    return rows


class ShowcaseConfigurationTests(unittest.TestCase):
    def test_version_profiles_are_explicit_and_portable(self):
        for version, duration in (('v01',30),('v02',90),('v03',240),('v04',240)):
            cfg=make_config(version)
            self.assertEqual(cfg['drive_s'],duration)
            self.assertEqual(cfg['duration_s'],cfg['settle_s']+duration+cfg['brake_s'])
            self.assertTrue(cfg['no_sumo'] and cfg['no_lidar'] and cfg['no_training'])
            self.assertNotIn(':',cfg['vehicle_directory'])
            self.assertEqual(cfg['physics_hz'] % cfg['control_hz'],0)
            self.assertEqual(cfg['physics_hz'] % cfg['planner_hz'],0)
            self.assertEqual(cfg['physics_hz'] % cfg['render_hz'],0)
            validate_config(cfg)

    def test_invalid_names_rejected(self):
        for kwargs in ({'version':'v99'},{'mode':'training'},{'camera':'bad'}):
            with self.assertRaises(ValueError):make_config(**kwargs)

    def test_flags_require_actual_bools(self):
        for key in ('gui','paced','capture'):
            for value in (0,1,None,'true'):
                with self.assertRaises(ValueError):make_config(**{key:value})

    def test_bounded_duration_and_no_bool_integer_shortcut(self):
        for duration in (14,1801,-1,0,True,False,15.,float('nan'),'90'):
            with self.assertRaises(ValueError):make_config(drive_s=duration)
        for duration in (15,90,1800):
            self.assertEqual(make_config(drive_s=duration)['drive_s'],duration)

    def test_short_runs_are_smoke_not_overtaking_qualification(self):
        self.assertEqual(make_config('v03',drive_s=89)['minimum_passes'],0)
        self.assertGreaterEqual(make_config('v03',drive_s=90)['minimum_passes'],3)
        self.assertGreaterEqual(make_config('v03',drive_s=90)['minimum_lane_changes'],2)
        for mode in ('blocked','dropout','contact-check'):
            self.assertEqual(make_config(mode=mode)['minimum_passes'],0)

    def test_resolved_contract_rejects_silent_changes(self):
        cfg=make_config()
        for key,value in (('physics_hz',60),('target_speed_m_s',30.),('min_clearance_m',0.),
                          ('minimum_passes',0),('unexpected_option',True)):
            altered=dict(cfg,**{key:value})
            with self.assertRaises(ValueError):validate_config(altered)

    def test_fleet_modes_have_stable_unique_ids_and_finite_motion(self):
        for mode in ('showcase','blocked','dropout','contact-check'):
            cfg=make_config(mode=mode)
            fleet=fleet_specs(cfg)
            self.assertEqual(fleet,fleet_specs(cfg))
            self.assertEqual(len({v['vehicle_id'] for v in fleet}),len(fleet))
            self.assertTrue(all(v['lane'] in range(4) and v['station_m']>=0
                                and math.isfinite(v['speed_m_s']) and v['speed_m_s']>=0 for v in fleet))
        blocked=fleet_specs(make_config(mode='blocked'))
        self.assertEqual(sorted(v['lane'] for v in blocked),list(range(4)))
        self.assertEqual(len({v['station_m'] for v in blocked}),1)


class ShowcaseAssessmentTests(unittest.TestCase):
    def setUp(self):
        self.cfg=make_config('v02',gui=False,paced=False)
        self.rows=valid_rows(self.cfg)
        self.passes={'background_000','background_001','background_002'}

    def assess(self,rows=None,cfg=None,contacts=0,passes=None,changes=2):
        return assess(self.rows if rows is None else rows,self.cfg if cfg is None else cfg,
                      contacts,self.passes if passes is None else passes,changes)

    def test_clean_synthetic_evidence_passes(self):
        result=self.assess()
        self.assertTrue(result['passed'],result)
        self.assertGreater(result['braking_distance_m'],0)
        self.assertGreaterEqual(result['stationary_hold_s'],self.cfg['min_hold_s'])

    def test_incomplete_or_noncontiguous_trace_fails(self):
        self.assertFalse(self.assess(rows=self.rows[:-1])['passed'])
        for field,value in (('tick',1000),('applied_tick',1000)):
            rows=deepcopy(self.rows);rows[400][field]=value
            self.assertFalse(self.assess(rows=rows)['passed'])

    def test_contacts_overlap_clearance_and_road_departure_fail(self):
        self.assertFalse(self.assess(contacts=1)['passed'])
        for field,value in (('overlap',True),('clearance_m',self.cfg['min_clearance_m']-.001),
                            ('footprint_in_road',False),('wheel_on_ground',[True,False,True,True]),
                            ('upright_z',.9),('speed_m_s',self.cfg['max_speed_m_s']+.1),
                            ('background_pose_error_m',.1)):
            rows=deepcopy(self.rows);rows[400][field]=value
            self.assertFalse(self.assess(rows=rows)['passed'],field)

    def test_passes_and_lane_changes_both_required(self):
        self.assertFalse(self.assess(passes=set())['passed'])
        self.assertFalse(self.assess(changes=0)['passed'])

    def test_braking_cannot_be_skipped(self):
        rows=deepcopy(self.rows)
        for row in rows:
            if row['phase']=='brake':row['speed_m_s']=5.
        self.assertFalse(self.assess(rows=rows)['passed'])

    def test_stationary_hold_rejects_reacceleration_and_drift(self):
        for field,value in (('speed_m_s',.2),('position_m',[self.rows[-1]['position_m'][0],1.,1.])):
            rows=deepcopy(self.rows);rows[-1][field]=value
            self.assertFalse(self.assess(rows=rows)['passed'])

    def test_traveled_distance_cannot_underreport_actual_motion(self):
        rows=deepcopy(self.rows)
        for row in rows:row['traveled_distance_m']=0.
        self.assertFalse(self.assess(rows=rows)['passed'])

    def test_clock_must_match_physics_ticks(self):
        rows=deepcopy(self.rows);rows[400]['sim_time_s']+=.2
        self.assertFalse(self.assess(rows=rows)['passed'])

    def test_nonfinite_rows_rejected_even_when_not_first(self):
        for field in ('upright_z','background_pose_error_m','clearance_m'):
            rows=deepcopy(self.rows);rows[400][field]=float('nan')
            self.assertFalse(self.assess(rows=rows)['passed'],field)

    def test_missing_wheel_samples_cannot_pass_vacuously(self):
        rows=deepcopy(self.rows);rows[400]['wheel_on_ground']=[]
        self.assertFalse(self.assess(rows=rows)['passed'])

    def test_missing_fields_and_empty_trace_fail_without_crashing(self):
        self.assertFalse(self.assess(rows=[])['passed'])
        for field in ('phase','position_m','command_expires_tick','wheel_on_ground','clearance_m'):
            rows=deepcopy(self.rows);del rows[400][field]
            self.assertFalse(self.assess(rows=rows)['passed'],field)

    def test_position_and_boolean_fields_have_strict_types(self):
        for field,value in (('position_m',[0.,float('nan'),1.]),('position_m',[0.,0.]),
                            ('wheel_on_ground',[1,1,1,1]),('fallback',1),('tick',True)):
            rows=deepcopy(self.rows);rows[400][field]=value
            self.assertFalse(self.assess(rows=rows)['passed'],field)

    def test_wrong_phase_and_duplicate_pass_ids_fail(self):
        rows=deepcopy(self.rows);rows[400]['phase']='settle'
        self.assertFalse(self.assess(rows=rows)['passed'])
        self.assertFalse(self.assess(passes=['same_car']*3)['passed'])

    def test_invalid_summary_counts_fail(self):
        self.assertFalse(self.assess(contacts=-1)['passed'])
        self.assertFalse(self.assess(changes=True)['passed'])

    def test_blocked_requires_following_not_lane_change(self):
        cfg=make_config(mode='blocked',drive_s=15)
        cfg['target_speed_m_s']=6.7 # Synthetic assessor fixture; not launch configuration.
        rows=valid_rows(cfg)
        self.assertTrue(self.assess(rows=rows,cfg=cfg,changes=0)['passed'])
        self.assertFalse(self.assess(rows=rows,cfg=cfg,changes=1)['passed'])
        rows[17*cfg['physics_hz']-2]['speed_m_s']=8.
        self.assertFalse(self.assess(rows=rows,cfg=cfg,changes=0)['passed'])

    def test_dropout_requires_fresh_fallback_after_exact_expiry(self):
        cfg=make_config(mode='dropout',drive_s=15)
        rows=valid_rows(cfg)
        self.assertTrue(self.assess(rows=rows,cfg=cfg)['passed'])
        expired=next(r for r in rows if r['phase']=='brake' and r['applied_tick']>=r['command_expires_tick'])
        expired['fallback']=False
        self.assertFalse(self.assess(rows=rows,cfg=cfg)['passed'])

    def test_contact_positive_control_needs_actual_callback(self):
        cfg=make_config(mode='contact-check',drive_s=15)
        rows=valid_rows(cfg)[:250]
        self.assertFalse(self.assess(rows=rows,cfg=cfg,contacts=0)['passed'])
        self.assertTrue(self.assess(rows=rows,cfg=cfg,contacts=1)['passed'])


if __name__=='__main__':
    unittest.main()
