"""CPU counter/lifecycle tests; RTX clock alignment still requires a real probe."""
from types import SimpleNamespace
import math

import pytest

from traffic.rendered_physics_session import (
    RenderedPhysicsSession, _positive_rate, verify_clock_transition,
)


@pytest.mark.parametrize('value', [True, False, 0, -1, 30.0, '30', math.nan])
def test_rate_validation_before_loading_isaac(value):
    with pytest.raises(ValueError):
        _positive_rate(value, 'rate')


def test_render_rate_must_divide_physics_rate_before_loading_isaac():
    with pytest.raises(ValueError, match='divide'):
        RenderedPhysicsSession(physics_hz=120, render_hz=31)


def test_clock_accepts_one_step_and_zero_step_render():
    verify_clock_transition((2, 2 / 120), (3, 3 / 120), steps=1, dt=1 / 120)
    verify_clock_transition((3, 3 / 120), (3, 3 / 120), steps=0, dt=1 / 120)


@pytest.mark.parametrize('before, after, expected_steps', [
    ((2, 0), (4, 2 / 120), 1),
    ((2, 0), (3, 1 / 120), 0),
    ((2, 0), (2, 1 / 120), 0),
    ((2, 0), (3, 0), 1),
    ((2, 0), (3, math.nan), 1),
    ((2, math.inf), (3, math.inf), 1),
])
def test_extra_missing_or_nonfinite_time_is_rejected(before, after, expected_steps):
    with pytest.raises(RuntimeError):
        verify_clock_transition(before, after, steps=expected_steps, dt=1 / 120)


@pytest.fixture
def running_session():
    """No Isaac imports: exercise public operations with a counted fake manager."""
    session = RenderedPhysicsSession.__new__(RenderedPhysicsSession)
    native = SimpleNamespace(count=2, time_s=2 / 120)
    session.stage_id = 42
    session.context = SimpleNamespace(get_stage_id=lambda: 42)
    session.timeline = SimpleNamespace(is_playing=lambda: True)
    session.closed = False
    session.started = True
    session.physics_hz = 120
    session.render_hz = 30
    session.render_every_steps = 4
    session.dt = 1 / 120
    session.physics_steps = 0
    session.render_count = 0
    session._origin = (2, 2 / 120)
    session._last_clock = session._origin

    def manager_step(*, steps, update_fabric):
        assert steps == 1 and update_fabric is False
        native.count += 1
        native.time_s += session.dt

    session.manager = SimpleNamespace(
        get_num_physics_steps=lambda: native.count,
        get_simulation_time=lambda: native.time_s,
        step=manager_step,
    )
    usd_updates = []
    session.physx = SimpleNamespace(update_transformations=lambda *args: usd_updates.append(args))
    settings = {'/app/player/playSimulations': True}
    session.settings = SimpleNamespace(
        get_as_bool=lambda name: settings[name],
        set_bool=lambda name, value: settings.__setitem__(name, value),
    )
    session.renderer = SimpleNamespace(render=lambda: None)
    return session, native, settings, usd_updates


def test_one_step_and_render_have_one_motion_authority(running_session):
    session, native, settings, usd_updates = running_session
    for _ in range(4):
        session.step()
    result = session.render()
    assert result['native_physics_steps'] == 6
    assert result['physics_steps'] == 4
    assert result['time_s'] == pytest.approx(4 / 120)
    assert result['absolute_time_s'] == pytest.approx(6 / 120)
    assert result['origin_time_s'] == pytest.approx(2 / 120)
    assert result['render_count'] == 1
    assert len(usd_updates) == 4
    assert all(args == (False, True, True, False) for args in usd_updates)
    assert settings['/app/player/playSimulations'] is True


def test_render_that_secretly_steps_is_rejected(running_session):
    session, native, *_ = running_session
    session.renderer.render = lambda: session.manager.step(steps=1, update_fabric=False)
    with pytest.raises(RuntimeError, match='Unexpected physics advancement'):
        session.render()
    assert session.render_count == 0


def test_foreign_app_update_between_owned_operations_is_rejected(running_session):
    session, native, *_ = running_session
    native.count += 1
    native.time_s += session.dt
    with pytest.raises(RuntimeError, match='Unexpected physics advancement'):
        session.step()


def test_render_exception_restores_play_setting(running_session):
    session, _, settings, _ = running_session

    def broken_render():
        settings['/app/player/playSimulations'] = False
        raise RuntimeError('render failure')

    session.renderer.render = broken_render
    with pytest.raises(RuntimeError, match='render failure'):
        session.render()
    assert settings['/app/player/playSimulations'] is True
    assert session.render_count == 0


def test_application_pause_renders_do_not_manufacture_sensor_time(running_session):
    session, *_ = running_session
    for _ in range(8):
        session.render()
    assert session.time_s == 0
    assert session.physics_steps == 0
    assert session.render_count == 8


def test_timeline_stop_or_replaced_stage_fail_closed(running_session):
    session, *_ = running_session
    session.timeline.is_playing = lambda: False
    with pytest.raises(RuntimeError, match='started/playing'):
        session.step()
    session.timeline.is_playing = lambda: True
    session.context.get_stage_id = lambda: 99
    with pytest.raises(RuntimeError, match='no longer owns'):
        session.render()
