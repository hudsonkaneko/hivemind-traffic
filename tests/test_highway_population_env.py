import xml.etree.ElementTree as ET

import numpy as np
import pytest

from environments.highway_population_env import HighwayPopulationEnv, write_population_demand, _main_start
from environments.i110_merge_env import AGENTS, I110MergeEnv
from tests.test_highway_merge_context import graph, vehicle
from tests.test_highway_stage1_env import bare_env


def population_bare(graph, count=8):
    env = bare_env(graph, vehicle('agent_0', 'sb1_1', 100.))
    env.__class__ = HighwayPopulationEnv
    env.possible_agents = [f'agent_{i}' for i in range(8)]
    env.controlled_count, env.total_vehicles = count, 16
    env.merge_agents = ('agent_0',)
    env.population_stats = dict(peer_observations=0, selected_peer_sum=0,
                               maximum_selected_peers=0, overflow_peer_observations=0,
                               zero_peer_observations=0,
                               selected_peer_histogram={str(i): 0 for i in range(7)})
    return env


@pytest.mark.parametrize('count', [0, 1, 2, 4, 6, 8])
@pytest.mark.parametrize('total', [8, 16, 24])
def test_requested_population_is_exact_and_includes_all_roles(graph, tmp_path, count, total):
    path = tmp_path / 'demand.xml'
    rows = write_population_demand(path, 25, graph, 'merge', count, total)
    vehicles = ET.parse(path).getroot().findall('vehicle')
    assert len(vehicles) == total == len(rows)
    assert sum(v.attrib['type'] == 'controlled' for v in vehicles) == count
    assert {v.attrib['id'] for v in vehicles if v.attrib['type'] == 'controlled'} == {f'agent_{i}' for i in range(count)}
    assert sum(v['route'] == 'ramp' for v in rows) == 2
    assert all(v.attrib['depart'] == '0' for v in vehicles)
    if count:
        assert any(v['controlled'] and v['route'] == 'ramp' for v in rows)


def test_assignment_changes_without_changing_physical_cohort(graph, tmp_path):
    def cohort(count):
        return write_population_demand(tmp_path / f'{count}.xml', 911, graph, 'merge', count, 24)
    low, high = cohort(1), cohort(8)
    physical = lambda rows: [{k: v for k, v in row.items() if k not in ('vehicle_id', 'controlled')} for row in rows]
    assert physical(low) == physical(high)
    assert {v['slot'] for v in low if v['controlled']}.issubset({v['slot'] for v in high if v['controlled']})
    repeated = cohort(8)
    assert high == repeated
    other = write_population_demand(tmp_path / 'other.xml', 912, graph, 'merge', 8, 24)
    assert [v['slot'] for v in high if v['controlled']] != [v['slot'] for v in other if v['controlled']]


@pytest.mark.parametrize('task', ['following', 'passing'])
def test_leader_count_is_included_and_full_av_task_is_rejected(graph, tmp_path, task):
    rows = write_population_demand(tmp_path / 'demand.xml', 2, graph, task, 6, 8)
    assert len(rows) == 8
    assert sum(v['controlled'] for v in rows) == 6
    leader = next(v for v in rows if v['slot'] == 'slow_leader')
    assert not leader['controlled'] and leader['vehicle_id'] == 'slow_leader'
    assert all(11. <= v['speed'] < 13. for v in rows if v['slot'].startswith('main_'))
    with pytest.raises(ValueError, match='native slow leader'):
        write_population_demand(tmp_path / 'bad.xml', 2, graph, task, 8, 8)


def test_population_state_masks_empty_slots_and_preserves_local_width(graph):
    env = population_bare(graph, 1)
    state = env.state()
    assert state.shape == (752,)
    assert state[93] == 1 and np.all(state[94:] == 0)
    env.agents, env.obs = [], {}
    assert np.all(env.state() == 0)


def test_nearest_six_fresh_messages_and_missing_slots(graph):
    env = population_bare(graph)
    env.communication, env.time = 'local', 1.
    for i in range(1, 8):
        sender = f'agent_{i}'
        env.states[sender] = vehicle(sender, 'sb1_1', 100. + 10 * i)
        env.inbox['agent_0'][sender] = dict(time=.9, x=0., y=-100. - 10 * i,
            speed=20., acceleration=0., request=0., lane_intent=0., requires_merge=0.)
    selected = env._selected_messages('agent_0')
    assert [sender for sender, _ in selected] == [f'agent_{i}' for i in range(1, 7)]
    obs = env._observations()['agent_0']
    assert obs.shape == (93,) and obs[45:].reshape(6, 8)[:, 7].sum() == 6
    assert env.population_stats['overflow_peer_observations'] == 1
    assert env.population_stats['selected_peer_histogram']['6'] == 1
    env.inbox['agent_0']['agent_1']['time'] = .4
    env.states.pop('agent_2')
    selected = env._selected_messages('agent_0')
    assert [sender for sender, _ in selected] == [f'agent_{i}' for i in range(3, 8)]
    obs = env._observations()['agent_0']
    assert obs[45:].reshape(6, 8)[:, 7].sum() == 5
    assert np.all(obs[-8:] == 0)
    env.communication = 'none'
    assert np.all(env._observations()['agent_0'][45:] == 0)


def test_population_merge_roles_and_team_normalization_use_episode_count(graph):
    env = population_bare(graph, 1)
    env.task = 'merge'
    env.states['agent_0'] = vehicle('agent_0', 'sb2_0', 10., ('mlk_on', 'sb2', 'sb3', 'sb4'))
    def advance():
        env.time += .1
        env.states = {'agent_0': vehicle('agent_0', 'sb2_1', 12., ('mlk_on', 'sb2', 'sb3', 'sb4'))}
    env._advance = advance
    _, _, _, _, info = env.step({'agent_0': np.array([0., 0.], np.float32)})
    assert info['agent_0']['merge_completed']
    assert info['agent_0']['reward_terms']['cooperation'] == pytest.approx(.1)


def test_legacy_default_hooks_preserve_four_agent_contract():
    env = I110MergeEnv.__new__(I110MergeEnv)
    assert env._controlled_agents() == AGENTS
    assert env._merge_agents() == ('agent_2', 'agent_3')


def test_generic_parallel_api_reset_options_do_not_change_population(monkeypatch):
    from environments.highway_stage1_env import HighwayStage1Env
    env = HighwayPopulationEnv.__new__(HighwayPopulationEnv)
    env.total_vehicles = 8
    def parent_reset(self, seed=None, options=None):
        self.states = {str(i): {} for i in range(8)}
        return {}, {}
    monkeypatch.setattr(HighwayStage1Env, 'reset', parent_reset)
    assert env.reset(seed=0, options={'options': 1}) == ({}, {})
    with pytest.raises(ValueError, match='configured at construction'):
        env.reset(options={'controlled_count': 4})


def test_spawn_grid_near_road_seam_has_explicit_safe_interior_position(graph):
    assert _main_start(graph, 199., 1) == (1, 1, 5.)
    assert _main_start(graph, 201., 1) == (1, 1, 5.)
    assert _main_start(graph, 241., 1) == (1, 1, 41.)
