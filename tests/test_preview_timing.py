import pytest

from visualization.preview_timing import PreviewClock, rendering_profile


class FakeTime:
    def __init__(self):
        self.now = 100.
        self.sleeps = []

    def clock(self):
        return self.now

    def sleep(self, delay):
        self.sleeps.append(delay)
        self.now += delay


def test_graphics_profile_changes_only_rgb_settings():
    assert rendering_profile() == dict(renderer='RealTimePathTracing', minimal_shading_mode=0,
                                      width=1280, height=800, anti_aliasing=3)
    profile = rendering_profile(True)
    assert profile.pop('extra_args') == ['--/app/vsync=false']
    assert profile == dict(renderer='MinimalRendering', minimal_shading_mode=2,
                           width=960, height=600, anti_aliasing=2, limit_cpu_threads=8)
    with pytest.raises(ValueError):
        rendering_profile(1)


def test_pacing_waits_to_1x_without_skipping_simulation():
    fake = FakeTime()
    pacer = PreviewClock(paced=True, clock=fake.clock, sleep=fake.sleep)
    for tick in range(1, 1501):
        fake.now += .01  # Represents completed work, not a simulated timestep.
        row = pacer.finish_frame(tick/30)
        assert row['rtf'] == pytest.approx(1.)
    assert len(fake.sleeps) == 1500
    assert pacer.summary()['soft_realtime_passed']
    assert pacer.summary()['simulation_time_s'] == 50.
    assert pacer.summary()['p95_frame_interval_s'] == pytest.approx(1/30)


def test_slow_render_reports_lag_without_sleeping_or_resetting_origin():
    fake = FakeTime()
    pacer = PreviewClock(paced=True, clock=fake.clock, sleep=fake.sleep)
    fake.now += 2.
    first = pacer.finish_frame(1.)
    assert first['lag_s'] == 1.
    fake.now += 1.
    second = pacer.finish_frame(2.)
    assert second['lag_s'] == 1.
    assert not fake.sleeps
    assert not pacer.summary()['soft_realtime_passed']


def test_explicit_pause_is_excluded_not_counted_as_slow_simulation():
    fake = FakeTime()
    pacer = PreviewClock(paced=True, clock=fake.clock, sleep=fake.sleep)
    fake.now += 4.1
    row = pacer.finish_frame(.2, paused_wall_s=4.)
    assert row['active_wall_s'] == pytest.approx(.2)
    assert fake.sleeps == pytest.approx([.1])
    assert pacer.summary()['soft_realtime_passed']


def test_unpaced_work_is_measured_but_cannot_pass_1x_gate():
    fake = FakeTime()
    pacer = PreviewClock(paced=False, clock=fake.clock, sleep=fake.sleep)
    fake.now += .5
    assert pacer.finish_frame(1.)['rtf'] == 2.
    assert not fake.sleeps
    assert not pacer.summary()['soft_realtime_passed']


def test_one_long_stall_cannot_be_hidden_by_catching_up():
    fake = FakeTime()
    pacer = PreviewClock(paced=True, clock=fake.clock, sleep=fake.sleep)
    fake.now += .7
    pacer.finish_frame(1/30)
    for tick in range(2, 1501):
        pacer.finish_frame(tick/30)
    report = pacer.summary()
    assert report['rtf'] == pytest.approx(1.)
    assert report['max_lag_s'] > .5
    assert not report['soft_realtime_passed']


def test_empty_clock_is_not_verified():
    assert not PreviewClock(paced=True).summary()['soft_realtime_passed']


def test_aborted_prefix_cannot_pass_expected_episode_gate():
    fake = FakeTime()
    pacer = PreviewClock(paced=True, clock=fake.clock, sleep=fake.sleep)
    pacer.finish_frame(.05)
    report = pacer.summary(expected_simulation_time_s=50., expected_frames=1000)
    assert not report['expected_window_complete']
    assert not report['soft_realtime_passed']
    for tick in range(2,1001):
        pacer.finish_frame(tick/20)
    report = pacer.summary(expected_simulation_time_s=50., expected_frames=1000)
    assert report['expected_window_complete']
    assert report['soft_realtime_passed']
    assert not pacer.summary(expected_simulation_time_s=50., expected_frames=1001)['soft_realtime_passed']
