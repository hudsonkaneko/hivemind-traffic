import xml.etree.ElementTree as ET

import pytest

pytest.importorskip('torch')

from environments.highway_flow_rl_v05_env import HighwayFlowRLV05Env


class _Sim:
    def __init__(self, arrived):
        self.arrived = arrived

    def getArrivedIDList(self):
        return list(self.arrived)


class _Conn:
    def __init__(self, arrived=()):
        self.simulation = _Sim(arrived)


def env(arrived=()):
    instance = HighwayFlowRLV05Env.__new__(HighwayFlowRLV05Env)  # logic only, no SUMO
    instance.v05_variant = 'startup'
    instance._reset_v05()
    instance.states, instance.time, instance.conn = {'agent_0': {}}, 33.0, _Conn(arrived)
    return instance


def before(instance, leader):
    front = {'id': leader} if leader else None
    instance._flow_before_context = {'agent_0': {'corridors': {'own': {'front': front}}}}


def test_only_decision_after_leader_exit_is_exempt():
    instance = env(arrived={'slow_leader'})
    before(instance, 'slow_leader')            # leader present before, gone after: exit
    instance._road_end_sample('agent_0', {'realized_jerk_m_s3': -0.1})
    instance.time, instance.conn = 33.1, _Conn()
    before(instance, None)
    instance._road_end_sample('agent_0', {'realized_jerk_m_s3': 5.3})   # exempt reaction
    instance.time = 33.2
    instance._road_end_sample('agent_0', {'realized_jerk_m_s3': 3.5})   # counts again
    car = instance.road_end['per_car']['agent_0']
    assert car['peak_abs_jerk_m_s3'] == 3.5
    assert car['exempt'] == [dict(time_s=33.1, jerk_m_s3=5.3)]
    assert instance.road_end['leader_exits'] == 1


def test_leader_lost_without_arrival_is_not_exempt():
    instance = env(arrived=())                 # leader vanished for another reason
    instance.states = {'agent_0': {}}
    before(instance, 'other')
    instance._road_end_sample('agent_0', {'realized_jerk_m_s3': 0.})
    instance.time = 33.1
    instance._road_end_sample('agent_0', {'realized_jerk_m_s3': 6.0})
    assert instance.road_end['per_car']['agent_0']['peak_abs_jerk_m_s3'] == 6.0
    assert instance.road_end['exempt_decisions'] == 0


def test_leader_still_present_is_not_exit():
    instance = env(arrived={'lead'})
    instance.states = {'agent_0': {}, 'lead': {}}
    before(instance, 'lead')
    instance._road_end_sample('agent_0', {'realized_jerk_m_s3': 0.})
    assert instance.road_end['leader_exits'] == 0


def test_invalid_follow_model_rejected():
    with pytest.raises(ValueError):
        HighwayFlowRLV05Env('net', 'dir', controlled_car_follow_model='IDM')


@pytest.mark.parametrize('model', ['ACC', 'Krauss'])
def test_demand_writes_requested_controlled_model(tmp_path, model):
    from traffic.highway_v05_demand import write_demand
    from traffic.i110_policy_interface import LaneGraph
    net = tmp_path / 'net.xml'
    lanes = ''.join(f'<edge id="sb{i}" from="n{i-1}" to="n{i}"><lane id="sb{i}_0" index="0" length="400" speed="22"/>'
                    f'<lane id="sb{i}_1" index="1" length="400" speed="22"/></edge>' for i in range(1, 6))
    conns = ''.join(f'<connection from="sb{i}" to="sb{i+1}" fromLane="{l}" toLane="{l}"/>' for i in range(1, 5) for l in (0, 1))
    net.write_text(f'<net>{lanes}{conns}</net>')
    graph = LaneGraph(net, through_lanes=2)
    graph.speed_limits = {lid: 22. for lid in graph.lanes}
    write_demand(tmp_path / 'r.rou.xml', 1, graph, 'cruise', 1, 8, controlled_car_follow_model=model)
    vtype = ET.parse(tmp_path / 'r.rou.xml').find("vType[@id='controlled']")
    assert vtype.get('carFollowModel') == model


def test_demand_rejects_unknown_model(tmp_path):
    from traffic.highway_v05_demand import write_demand
    with pytest.raises(ValueError):
        write_demand(tmp_path / 'r.rou.xml', 1, None, 'cruise', 1, 8, controlled_car_follow_model='IDM')


def test_ruled_gates_substitute_only_with_rule_and_keep_original():
    from experiments.run_highway_flow_rl_v05 import ruled_gates
    car = dict(assigned_controlled=True, route_completed=True,
               all=dict(decisions=10, intervention_decisions=0, jerk_m_s3=dict(realized=dict(peak_abs=5.3))))
    report = dict(task='following', smooth_control=dict(per_car=dict(agent_0=car)),
                  road_end=dict(per_car=dict(agent_0=dict(peak_abs_jerk_m_s3=3.5,
                                                          exempt=[dict(time_s=33.1, jerk_m_s3=5.3)]))))
    assert ruled_gates(report)['per_car']['agent_0']['peak_abs_jerk_m_s3'] == 5.3   # rule off
    report['road_end_leader_exit_rule'] = True
    result = ruled_gates(report)
    assert result['per_car']['agent_0']['peak_abs_jerk_m_s3'] == 3.5
    assert result['per_car']['agent_0']['checks']['peak_jerk'] is True
    assert result['evaluation_rules']['exempt'] == [dict(agent='agent_0', time_s=33.1, jerk_m_s3=5.3)]
    assert car['all']['jerk_m_s3']['realized']['peak_abs'] == 5.3                     # evidence untouched
