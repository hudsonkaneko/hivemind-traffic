import pytest

from traffic.runtime_profile import RuntimeProfiler


class FakeClock:
    def __init__(self):
        self.now = 0

    def __call__(self):
        self.now += 10
        return self.now

    def advance(self, nanoseconds):
        self.now += nanoseconds


def test_stage_samples_and_iteration_residual_are_non_overlapping():
    clock = FakeClock()
    profile = RuntimeProfiler(clock=clock)
    profile.begin_iteration()
    with profile.measure('scan'):
        clock.advance(30)
    with profile.measure('physics'):
        clock.advance(50)
    profile.finish_iteration()

    profile.begin_iteration()
    with profile.measure('scan'):
        clock.advance(10)
    profile.finish_iteration()

    report = profile.summary(simulation_time_s=2 / 120)
    assert report['iterations'] == 2
    assert report['non_overlapping_stages'] is True
    assert report['loop_wall_s'] == pytest.approx(170e-9)
    assert report['iteration_timing'] == dict(
        count=2, total_s=pytest.approx(170e-9), p50_ms=pytest.approx(40e-6),
        p95_ms=pytest.approx(130e-6), max_ms=pytest.approx(130e-6))
    assert report['wall_to_sim_ratio'] == pytest.approx(report['loop_wall_s'] / (2 / 120))
    assert report['stage_coverage_percent'] == pytest.approx(100)
    assert report['stages']['scan']['count'] == 2
    assert report['stages']['scan']['total_s'] == pytest.approx(60e-9)
    assert report['stages']['scan']['p50_ms'] == pytest.approx(20e-6)
    assert report['stages']['scan']['p95_ms'] == pytest.approx(40e-6)
    assert report['stages']['physics']['count'] == 1
    assert report['stages']['loop_other']['count'] == 2


def test_profiler_rejects_unbalanced_iterations_and_out_of_scope_timing():
    profile = RuntimeProfiler(clock=FakeClock())
    with pytest.raises(RuntimeError):
        profile.finish_iteration()
    with pytest.raises(ValueError):
        with profile.measure('loop_other'):
            pass
    with pytest.raises(RuntimeError):
        with profile.measure('scan'):
            pass
    profile.begin_iteration()
    with pytest.raises(RuntimeError):
        profile.begin_iteration()
    profile.finish_iteration()


def test_stage_scopes_reject_nesting():
    profile = RuntimeProfiler(clock=FakeClock())
    profile.begin_iteration()
    with pytest.raises(RuntimeError, match='cannot be nested'):
        with profile.measure('outer'):
            with profile.measure('inner'):
                pass
    profile.finish_iteration()
    assert profile.summary()['stages']['outer']['count'] == 1
    assert 'inner' not in profile.summary()['stages']


def test_exception_cleans_up_stage_and_keeps_iteration_measurable():
    clock = FakeClock()
    profile = RuntimeProfiler(clock=clock)
    profile.begin_iteration()
    with pytest.raises(ValueError, match='fixture'):
        with profile.measure('control'):
            clock.advance(25)
            raise ValueError('fixture')
    assert profile._active_stage is None
    profile.finish_iteration()
    assert profile.summary()['stages']['control']['count'] == 1


def test_empty_scope_calibration_uses_separate_profiler():
    profile = RuntimeProfiler(clock=FakeClock())
    calibration = profile.calibrate_overhead(samples=8)
    assert calibration['estimated_loop_scope_count'] == 0
    profile.begin_iteration()
    with profile.measure('physics'):
        pass
    profile.finish_iteration()
    adjusted = profile.estimate_overhead_for_run(calibration)
    assert adjusted['calibration_samples'] == 8
    assert adjusted['calibration_scope_overhead_ns'] >= 0
    assert adjusted['calibration_overhead_per_scope_ns'] >= 0
    assert adjusted['estimated_loop_scope_count'] == 1
    assert adjusted['estimated_loop_scope_overhead_ns'] >= 0
