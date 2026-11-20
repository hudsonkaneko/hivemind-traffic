"""Reference paths and the SUMO shadow of a physics-driven car (no Isaac Sim; SUMO's netconvert for one test)."""
import math
import subprocess
from types import SimpleNamespace

import pytest

from traffic.physical_drive import LaneNetwork, project, reference_path, smoothstep5


# --- pure geometry ---------------------------------------------------------------------
def test_smoothstep_is_flat_at_both_ends():
    assert smoothstep5(-1.) == 0. and smoothstep5(0.) == 0. and smoothstep5(1.) == 1. and smoothstep5(2.) == 1.
    assert smoothstep5(.5) == pytest.approx(.5)
    assert smoothstep5(.001) < 1e-7 and 1. - smoothstep5(.999) < 1e-7


def test_project_gives_arc_length_and_left_offset():
    shape = [(0., 0.), (10., 0.), (10., 10.)]
    assert project(shape, 4., 1.) == pytest.approx((4., 1.))
    assert project(shape, 11., 5.) == pytest.approx((15., -1.))   # right of the northbound leg


class FakeLane:
    def __init__(self, lane_id, shape, outgoing=()):
        self.lane_id, self.shape, self.outgoing = lane_id, shape, list(outgoing)

    def getLength(self):
        return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(self.shape, self.shape[1:]))


class FakeNetwork(LaneNetwork):
    """Two parallel straight lanes 3.6 m apart, 300 m long, on one edge that ends the route."""

    def __init__(self):
        self.lanes = {'e_0': FakeLane('e_0', [(0., 0.), (300., 0.)]), 'e_1': FakeLane('e_1', [(0., 3.6), (300., 3.6)])}

    def lane(self, lane_id):
        return self.lanes[lane_id]

    def shape(self, lane_id):
        return list(self.lanes[lane_id].shape)

    def width(self, lane_id):
        return 3.6

    def lanes_ahead(self, lane_id, route, minimum_m):
        return [lane_id]


def test_path_without_a_maneuver_is_the_lane_centerline_from_the_bumper():
    path = reference_path(FakeNetwork(), ['e'], 'e_0', 50., .3, 20.)
    assert path[0] == pytest.approx((50., 0.))
    assert all(y == pytest.approx(0.) for _, y in path)
    assert path[-1][0] - path[0][0] == pytest.approx(80.)      # max(40 m, 4 s * 20 m/s)


def test_path_continues_straight_past_the_end_of_the_corridor():
    path = reference_path(FakeNetwork(), ['e'], 'e_0', 299.5, 0., 20.)
    assert path[0] == pytest.approx((299.5, 0.)) and path[-1] == pytest.approx((379.5, 0.))


def test_lane_change_path_starts_on_the_source_and_reaches_the_target_when_time_is_up():
    maneuver = dict(source_lane='e_0', target_lane='e_1', started_at=10., duration_s=3.)
    start = reference_path(FakeNetwork(), ['e'], 'e_0', 50., 0., 20., maneuver=maneuver, now=10.)
    assert start[0][1] == pytest.approx(0.) and start[-1][1] == pytest.approx(3.6)
    for (x, y), (_, later) in zip(start, start[1:]):
        assert later >= y - 1e-9                                  # moves monotonically toward the target
    halfway = reference_path(FakeNetwork(), ['e'], 'e_0', 80., 1.8, 20., maneuver=maneuver, now=11.5)
    assert halfway[0][1] == pytest.approx(1.8)                    # smoothstep5(0.5) = 0.5 of the 3.6 m spacing


def test_lanes_ahead_follow_the_route_through_the_junction_lane(tmp_path):
    sumolib = pytest.importorskip('sumolib')
    nodes = tmp_path / 'n.nod.xml'
    edges = tmp_path / 'e.edg.xml'
    nodes.write_text('<nodes><node id="a" x="0" y="0"/><node id="b" x="100" y="0"/><node id="c" x="200" y="20"/>'
                     '<node id="d" x="300" y="20"/></nodes>')
    edges.write_text('<edges><edge id="ab" from="a" to="b" numLanes="1"/><edge id="bc" from="b" to="c" numLanes="1"/>'
                     '<edge id="cd" from="c" to="d" numLanes="1"/></edges>')
    net = tmp_path / 'net.net.xml'
    subprocess.run([sumolib.checkBinary('netconvert'), '-n', str(nodes), '-e', str(edges), '-o', str(net),
                    '--no-turnarounds', 'true'], check=True, capture_output=True)
    lanes = LaneNetwork(net)
    ahead = lanes.lanes_ahead('ab_0', ['ab', 'bc', 'cd'], 10_000.)
    assert ahead[0] == 'ab_0' and ahead[-1] == 'cd_0'
    assert any(lane.startswith(':b') for lane in ahead) and 'bc_0' in ahead
    path = reference_path(lanes, ['ab', 'bc', 'cd'], 'ab_0', 95., 0., 25.)
    assert max(y for _, y in path) > 5.                           # the path climbs with the second edge


# --- the shadow ------------------------------------------------------------------------
class FakeDomain:
    def __init__(self):
        self.calls, self.lane, self.lateral = [], 'e_0', 0.

    def setAcceleration(self, *args):
        self.calls.append(('setAcceleration', args))

    def changeLane(self, *args):
        self.calls.append(('changeLane', args))

    def getLateralSpeed(self, vid):
        return 0.

    def getLaneID(self, vid):
        return self.lane

    def getLateralLanePosition(self, vid):
        return self.lateral


def shadow_env():
    from environments.highway_flow_rl_v14_physical_env import _ShadowVehicle
    env = SimpleNamespace(physical_agents=frozenset({'agent_0'}), physical_released=False, physical_commands={},
                          physical_maneuvers={}, physical_lateral_speed={}, physical_lateral_previous={}, time=5.,
                          lane_change_duration=3., states={'agent_0': dict(lane_id='e_0')},
                          graph=SimpleNamespace(lanes={'e_0': dict(edge='e', index=0), 'e_1': dict(edge='e', index=1)}),
                          lanes=FakeNetwork())
    domain = FakeDomain()
    return env, domain, _ShadowVehicle(env, domain)


def test_commands_for_the_physics_car_go_to_the_driver_and_others_to_sumo():
    env, domain, vehicle = shadow_env()
    vehicle.setAcceleration('agent_0', 1.5, .1)
    vehicle.changeLane('agent_0', 1, 3.1)
    vehicle.setAcceleration('main_1', -1., .1)
    assert env.physical_commands == {'agent_0': dict(acceleration=1.5)}
    assert env.physical_maneuvers['agent_0'] == dict(source_lane='e_0', target_lane='e_1', started_at=5., duration_s=3.)
    assert domain.calls == [('setAcceleration', ('main_1', -1., .1))]
    with pytest.raises(RuntimeError):
        vehicle.changeLaneRelative('agent_0', 1, 3.)
    env.physical_released = True                                  # the drain: SUMO drives again
    vehicle.setAcceleration('agent_0', .5, .1)
    assert domain.calls[-1] == ('setAcceleration', ('agent_0', .5, .1))


def test_lateral_speed_is_measured_across_a_lane_line():
    env, domain, vehicle = shadow_env()
    domain.lateral = 1.7
    assert vehicle.getLateralSpeed('agent_0') == 0.               # first sample
    env.time, domain.lane, domain.lateral = 5.1, 'e_1', -1.8      # crossed the line: 0.1 m further left
    assert vehicle.getLateralSpeed('agent_0') == pytest.approx(1.)
    assert vehicle.getLateralSpeed('agent_0') == pytest.approx(1.)   # repeated reads in one step agree
