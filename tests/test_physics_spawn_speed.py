"""Decision 22 (plan v15, November 27, 2026): the physics server spawns the body at SUMO's speed as its horizontal
forward speed, the speed read() reports and the gate sees. No Isaac Sim: the pure pieces and the source."""
import inspect
import math

import pytest

from scripts import physics_drive_server as server

DT = .1


def horizontal_forward_speed(velocity, forward):
    """read()'s speed, as written there (checked below)."""
    return (velocity[0] * forward[0] + velocity[1] * forward[1]) / max(math.hypot(forward[0], forward[1]), 1e-9)


@pytest.mark.parametrize('yaw_deg, pitch_deg', [(0., 2.7), (37., 2.6), (-120., -3.1), (200., 5.)])
def test_a_pitched_spawn_reads_sumos_speed(yaw_deg, pitch_deg):
    speed, pitch = 12.24, math.radians(pitch_deg)
    forward = server.rotate(server.quaternion(math.radians(yaw_deg), pitch), (1., 0., 0.))
    velocity, along = server.spawn_velocity(speed, forward)
    assert horizontal_forward_speed(velocity, forward) == pytest.approx(speed, abs=1e-12)
    assert along == pytest.approx(speed / math.cos(pitch), abs=1e-12)              # along the road, a little faster
    assert math.dist(velocity, (0., 0., 0.)) == pytest.approx(along, abs=1e-12)    # along the body's heading
    assert velocity[2] == pytest.approx(along * math.sin(pitch), abs=1e-12)
    old = tuple(speed * f for f in forward)                                        # the convention before
    step = (horizontal_forward_speed(old, forward) - speed) / DT
    assert step == pytest.approx(-speed * (1 - math.cos(pitch)) / DT, abs=1e-12)   # decision 20's spawn step


def test_the_adams_spawn_step_is_gone():
    speed, pitch = 12.24, math.radians(2.7)
    forward = server.rotate(server.quaternion(.4, pitch), (1., 0., 0.))
    old = tuple(speed * f for f in forward)
    assert (horizontal_forward_speed(old, forward) - speed) / DT == pytest.approx(-.1358, abs=1e-3)
    velocity, _ = server.spawn_velocity(speed, forward)
    assert (horizontal_forward_speed(velocity, forward) - speed) / DT == pytest.approx(0., abs=1e-9)


def test_a_flat_spawn_is_unchanged():
    forward = server.rotate(server.quaternion(1.1, 0.), (1., 0., 0.))
    velocity, along = server.spawn_velocity(20., forward)
    assert along == pytest.approx(20., abs=1e-12)
    assert velocity == pytest.approx(tuple(20. * f for f in forward), abs=1e-12)


def test_spawn_uses_it_for_the_body_and_the_wheels_and_read_reports_the_horizontal_speed():
    spawn = inspect.getsource(server.Server.spawn)
    assert "velocity, along = spawn_velocity(pose['speed'], forward)" in spawn
    assert 'set_wheel_rotation_speed(wheel, along / spec.wheel_radius_m)' in spawn
    assert "speed_convention='horizontal (decision 22)'" in spawn
    assert 'speed * f for f in forward' not in spawn
    assert "speed=(velocity[0] * forward[0] + velocity[1] * forward[1]) / max(horizontal, 1e-9)" in \
        inspect.getsource(server.Server.read)
