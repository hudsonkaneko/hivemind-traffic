"""CPU-only checks for real native speed-ceiling configuration and coherence."""
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest

from environments.highway_population_env import (
    HighwayPopulationEnv, episode_speed_ceiling, population_leader_speed, write_population_demand,
)
from environments.highway_stage1_env import HighwayStage1Env
from tests.test_highway_merge_context import graph, vehicle
from tests.test_highway_population_env import population_bare


class LaneAPI:
    def __init__(self, speeds):
        self.speeds = dict(speeds)
        self.writes = []
    def getIDList(self):
        return tuple(self.speeds)
    def getMaxSpeed(self, lane):
        return self.speeds[lane]
    def setMaxSpeed(self, lane, speed):
        self.writes.append((lane, speed))
        self.speeds[lane] = speed


def bare(speeds, ceiling=12.):
    env = HighwayPopulationEnv.__new__(HighwayPopulationEnv)
    env.speed_limit_m_s = ceiling
    env._speed_limits_applied = False
    env.native_speed_limit_manifest = []
    env.graph = SimpleNamespace(speed_limits=dict(speeds))
    env.conn = SimpleNamespace(lane=LaneAPI(speeds))
    return env


@pytest.mark.parametrize('value', [True, False, 0., -1., 5.9, 26.8225, float('nan'), float('inf'), '12'])
def test_speed_ceiling_rejects_invalid_or_unsupported_values(value):
    with pytest.raises(ValueError, match='Episode speed ceiling'):
        episode_speed_ceiling(value)


@pytest.mark.parametrize('value', [6., 12., 18., 22.352, 26.8224])
def test_speed_ceiling_accepts_declared_highway_domain(value):
    assert episode_speed_ceiling(value) == value


def test_default_does_not_access_or_mutate_native_connection():
    env = bare({'through_0': 22.352}, None)
    env.conn = None
    env._apply_episode_speed_limit()
    assert not env._speed_limits_applied
    assert env.graph.speed_limits == {'through_0': 22.352}


def test_every_lane_including_internal_is_lowered_without_raising_curve_limit():
    original = {'through_0': 26.8224, 'ramp_0': 9., ':junction_0_0': 18.}
    env = bare(original)
    env._apply_episode_speed_limit()
    assert env.conn.lane.speeds == {'through_0': 12., 'ramp_0': 9., ':junction_0_0': 12.}
    assert env.graph.speed_limits == env.conn.lane.speeds
    assert {row['lane_id'] for row in env.native_speed_limit_manifest} == set(original)
    assert all(row['applied_speed_m_s'] == row['readback_speed_m_s'] for row in env.native_speed_limit_manifest)
    assert env._speed_limits_applied
    env._apply_episode_speed_limit()
    assert len(env.conn.lane.writes) == len(original)


def test_high_requested_ceiling_never_raises_lower_native_limit():
    env = bare({'through_0': 22.352}, 26.8224)
    env._apply_episode_speed_limit()
    assert env.conn.lane.speeds['through_0'] == 22.352


def test_inventory_or_readback_failure_cannot_mark_configuration_complete():
    env = bare({'through_0': 22.352})
    env.graph.speed_limits['missing'] = 20.
    with pytest.raises(ValueError, match='inventory'):
        env._apply_episode_speed_limit()
    assert not env.conn.lane.writes
    env = bare({'through_0': 22.352})
    env.conn.lane.setMaxSpeed = lambda lane, speed: None
    with pytest.raises(ValueError, match='readback'):
        env._apply_episode_speed_limit()
    assert not env._speed_limits_applied
    assert env.graph.speed_limits == {'through_0': 22.352}


def test_native_limit_precedes_first_step_and_leader_request_is_corrected_before_next(monkeypatch):
    env = bare({'through_0': 26.8224})
    env.task = 'following'
    env._controlled_agents = lambda: ('agent_0',)
    env.population_stats = dict(road_samples=0, total_on_road_sum=0, controlled_on_road_sum=0,
                               on_road_av_fraction_sum=0., maximum_on_road=0)
    calls = []
    env.conn.vehicle = SimpleNamespace(setSpeed=lambda vehicle, value: calls.append((vehicle, value)))
    def step(self):
        assert self.conn.lane.speeds['through_0'] == 12.
        assert self.graph.speed_limits['through_0'] == 12.
        calls.append('native_step')
        self.states = {'agent_0': {}, 'slow_leader': {}}
        self.conn.vehicle.setSpeed('slow_leader', 16.)
    monkeypatch.setattr(HighwayStage1Env, '_advance', step)
    env._advance()
    assert calls == ['native_step', ('slow_leader', 16.), ('slow_leader', pytest.approx(9.6))]
    assert env.population_stats['road_samples'] == 1


@pytest.mark.parametrize('task', ['cruise', 'following', 'passing', 'merge'])
def test_low_speed_departures_remain_feasible_and_leader_remains_slow(graph, tmp_path, task):
    path = tmp_path/'demand.xml'
    rows = write_population_demand(path, 11, graph, task, 4, 16, speed_limit_m_s=12.)
    assert len(rows) == 16
    assert all(row['speed'] <= 9.6 + 1e-8 for row in rows)
    if task in ('following', 'passing'):
        leader = next(row for row in rows if row['slot'] == 'slow_leader')
        assert leader['speed'] == pytest.approx(population_leader_speed(task, 12.))
        assert leader['speed'] < 12.
    values = ET.parse(path).findall('vType')
    assert all(v.get('maxSpeed') == '26.8224' and v.get('tau') == '1.5' for v in values)


def test_default_demand_is_byte_identical_when_option_is_absent(graph, tmp_path):
    first, second = tmp_path/'first.xml', tmp_path/'second.xml'
    a = write_population_demand(first, 3, graph, 'passing', 4, 16)
    b = write_population_demand(second, 3, graph, 'passing', 4, 16, speed_limit_m_s=None)
    assert a == b and first.read_bytes() == second.read_bytes()
    assert next(r['speed'] for r in a if r['slot'] == 'slow_leader') == 10.


def test_graph_merge_limit_and_current_native_allowed_speed_feed_actor_fields(graph):
    env = population_bare(graph, 1)
    env.speed_limit_m_s = 12.
    env._speed_limits_applied = False
    env.native_speed_limit_manifest = []
    env.conn = SimpleNamespace(lane=LaneAPI(graph.speed_limits))
    env._apply_episode_speed_limit()
    ego = vehicle('agent_0', 'sb2_0', 10., ('mlk_on', 'sb2', 'sb3', 'sb4'))
    ego['allowed_speed'] = env.conn.lane.getMaxSpeed('sb2_0')
    ego['speed'] = 10.
    env.states = {'agent_0': ego}
    observation = env._observations()['agent_0']
    assert observation[4] == pytest.approx(12./26.8224)
    assert observation[44] == pytest.approx(12./26.8224)


def test_reset_reapplies_ceiling_for_each_new_connection(monkeypatch):
    env = bare({'through_0': 26.8224})
    env.total_vehicles = 8
    def reset(self, seed=None, options=None):
        assert not self._speed_limits_applied
        self.conn = SimpleNamespace(lane=LaneAPI({'through_0': 26.8224}))
        self._apply_episode_speed_limit()
        self.states = {str(i): {} for i in range(8)}
        return {}, {}
    monkeypatch.setattr(HighwayStage1Env, 'reset', reset)
    for seed in (1, 2):
        assert env.reset(seed=seed) == ({}, {})
        assert env.conn.lane.writes == [('through_0', 12.)]
    with pytest.raises(ValueError, match='configured at construction'):
        env.reset(options={'speed_limit_m_s': 18.})
