"""Presentation cadence never changes the physical/control experiment."""
import unittest

from experiments.loop_showcase_config import make_config,validate_config,fleet_specs
from traffic.showcase_performance import SequentialProfiler,audit_schedule


class TimingTests(unittest.TestCase):
    def test_exclusive_sequential_phases_and_tail(self):
        clock=iter([0,10,35,40,50,70,80])
        profile=SequentialProfiler(clock=lambda:next(clock))
        profile.begin_iteration();profile.mark('physics');profile.mark('render');profile.finish_iteration()
        profile.begin_iteration();profile.mark('physics');profile.finish_iteration()
        report=profile.summary(simulation_time_s=2)
        self.assertEqual(report['iterations'],2)
        self.assertAlmostEqual(report['loop_wall_s'],70e-9)
        self.assertAlmostEqual(report['stages']['physics']['total_s'],30e-9)
        self.assertAlmostEqual(report['stages']['render']['total_s'],25e-9)
        self.assertAlmostEqual(report['stages']['loop_other']['total_s'],15e-9)
        self.assertAlmostEqual(report['stage_coverage_percent'],100.)

    def test_invalid_phase_lifecycle(self):
        profile=SequentialProfiler()
        with self.assertRaises(RuntimeError):profile.mark('state')
        profile.begin_iteration()
        for stage in ('','loop_other'):
            with self.assertRaises(ValueError):profile.mark(stage)
        with self.assertRaises(RuntimeError):profile.measure('existing_scope')
        profile.finish_iteration()

    def test_calibration_uses_actual_marks_and_keeps_run_empty(self):
        profile=SequentialProfiler()
        report=profile.calibrate_overhead(5)
        self.assertIn('sequential',report['method'])
        self.assertGreater(report['calibration_overhead_per_scope_ns'],0)
        self.assertEqual(profile.iterations,0)
        self.assertEqual(profile.samples_ns,{})
        for invalid in (0,True,1.5):
            with self.assertRaises(ValueError):profile.calibrate_overhead(invalid)


class PresentationContractTests(unittest.TestCase):
    def test_v06_matches_all_physical_settings_and_fleet(self):
        old=make_config('v05');new=make_config('v06')
        differences={key for key in old if old[key]!=new[key]}
        self.assertEqual(differences,{'version'})
        self.assertEqual(fleet_specs(old),fleet_specs(new))
        self.assertEqual(new['render_hz'],10)
        self.assertEqual(new['evidence_profile'],'presentation')
        self.assertEqual(new['decorative_wheel_distance_m'],150.)
        validate_config(new)

    def test_old_presets_and_default_remain_unchanged(self):
        self.assertEqual(make_config()['version'],'v04')
        for version in ('v04','v05'):
            cfg=make_config(version)
            self.assertNotIn('evidence_profile',cfg)
            self.assertNotIn('decorative_wheel_distance_m',cfg)
            for extra in ({'evidence':'full'},{'render_hz':20}):
                with self.assertRaises(ValueError):make_config(version,**extra)

    def test_optional_rates_are_explicit_and_validated(self):
        for mode in ('full','presentation'):
            for hz in (10,20,30):validate_config(make_config('v06',evidence=mode,render_hz=hz))
        for extra in ({'evidence':'none'},{'render_hz':60},{'render_hz':True},{'render_hz':30.}):
            with self.assertRaises(ValueError):make_config('v06',**extra)
        cfg=make_config('v06');cfg['decorative_wheel_distance_m']=10
        with self.assertRaises(ValueError):validate_config(cfg)

    def test_full_audits_and_crash_safe_chunks(self):
        for version in ('v05','v06'):
            cfg=make_config(version,**({'evidence':'full'} if version=='v06' else {}))
            samples=[audit_schedule(cfg,tick) for tick in range(1,121)]
            self.assertEqual(sum(row['native'] for row in samples),120)
            self.assertEqual(sum(row['visual'] for row in samples),cfg['render_hz'])
            self.assertEqual(sum(row['objects'] for row in samples),10)
            self.assertEqual(sum(row['checkpoint'] for row in samples),1)

    def test_presentation_sampling_is_anchored_to_simulation_ticks(self):
        cfg=make_config('v06',render_hz=30)
        samples=[audit_schedule(cfg,t) for t in range(1,361)]
        self.assertEqual([i+1 for i,s in enumerate(samples) if s['native']],[1,120,240,360])
        self.assertEqual([i+1 for i,s in enumerate(samples) if s['visual']],[4,120,240,360])
        self.assertEqual(sum(s['objects'] for s in samples),3)
        self.assertFalse(any(s['checkpoint'] for s in samples))
        for tick in (0,-1,True,1.5):
            with self.assertRaises(ValueError):audit_schedule(cfg,tick)


if __name__=='__main__':unittest.main()
