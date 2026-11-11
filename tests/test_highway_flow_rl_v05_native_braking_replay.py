import pytest

pytest.importorskip('traci')

from scripts.replay_highway_flow_rl_v05_native_braking import junction_state


class _Vehicle:
    def getNextLinks(self, agent):
        # TraCI order: lane, hasPrio, isOpen, hasFoe, viaLane, state, direction, length
        return [('sb2_0', False, True, False, ':n1_3_0', 'm', 's', 15.65)]

    def getJunctionFoes(self, agent, distance):
        return []

    def getLeader(self, agent, distance):
        return None


class _Conn:
    vehicle = _Vehicle()


def test_next_link_fields_follow_traci_order():
    state = junction_state(_Conn(), 'agent_0')
    link = state['next_links'][0]
    assert link == dict(to_lane='sb2_0', via=':n1_3_0', has_priority=False, is_open=True,
                        has_approaching_foe=False, state='m', direction='s', link_length_m=15.65)
    assert state['junction_foes'] == [] and state['leader'] is None
