import hashlib
import json
from pathlib import Path

import numpy as np
import pytest
import torch

from traffic.mappo_physical_policy import (
    CAP_M_S, EXPECTED_POLICY_SHA256, FrozenPhysicalActor, LOCAL_FEATURES,
    MESSAGE_FEATURES, StraightRoadPolicyBridge,
)

ROOT = Path(__file__).resolve().parents[1]
POLICY = ROOT / 'policies/versions/highway-stage1-v03-phase0/policy.pt'


def vehicle(identity='ego', x=20., y=0., speed=12., **overrides):
    return dict(id=identity, x_m=x, y_m=y, yaw_rad=0., speed_m_s=speed,
                acceleration_m_s2=.25, length_m=4.8, width_m=1.8, **overrides)


def observation(bridge=None, ego=None, traffic=(), **kwargs):
    return (bridge or StraightRoadPolicyBridge()).observe(ego or vehicle(), traffic,
        **dict(dict(time_s=10., distance_travelled_m=18.), **kwargs))


@pytest.fixture
def straight_graph(tmp_path):
    from traffic.highway_merge_context import Stage1LaneGraph
    network = tmp_path / 'straight.net.xml'
    network.write_text('<net><edge id="sb1">' + ''.join(
        f'<lane id="sb1_{i}" index="{i}" length="2000" speed="15.6464"/>'
        for i in range(3)) + '</edge></net>', encoding='utf-8')
    return Stage1LaneGraph(network, through_lanes=3)


def original_observation(graph, ego, traffic, *, previous=0., time=10., last_change=-5., distance=18.):
    from environments.highway_stage1_env import HighwayStage1Env
    env = HighwayStage1Env.__new__(HighwayStage1Env)
    env.graph = graph
    env.agents = ['ego']
    env.states = {}
    for source in [ego, *traffic]:
        lane = int(round(source['y_m'] / 3.6))
        env.states[source['id']] = dict(id=source['id'], x=source['x_m'], y=source['y_m'],
            lane_id=f'sb1_{lane}', lane_position=source['x_m'] + source['length_m'] / 2,
            route=('sb1',), speed=source['speed_m_s'], acceleration=source['acceleration_m_s2'],
            distance=distance, allowed_speed=15.6464, length=source['length_m'])
    env.previous = {'ego': (previous, 0)}
    env.last_change = {'ego': last_change}
    env.inbox = {'ego': {}}
    env.time, env.start_time, env.horizon = time, 0., 800
    env.communication = 'none'
    return env._observations()['ego']


@pytest.mark.parametrize('lane,speed,leader_gap,relative,last_change', [
    (0, 0., None, 0., -5.), (1, 12., 40., -4., -5.),
    (2, 15.6464, 120., 0., 8.), (0, 16., 3., -8., -5.),
    (1, 8., 121., 0., -5.),
])
def test_all_93_fields_match_original_straight_observation(straight_graph,
        lane, speed, leader_gap, relative, last_change):
    ego = vehicle(y=3.6 * lane, speed=speed)
    traffic = [vehicle('rear_left', x=5., y=3.6 * min(2, lane + 1), speed=18.)]
    if leader_gap is not None:
        traffic.append(vehicle('leader', x=20. + 4.8 + leader_gap,
                               y=3.6 * lane, speed=max(0., speed + relative)))
    observed = observation(ego=ego, traffic=traffic, previous_request_m_s2=-.7,
                           last_lane_change_s=last_change)
    original = original_observation(straight_graph, ego, traffic, previous=-.7,
                                    last_change=last_change)
    np.testing.assert_array_equal(np.asarray(observed['observation'], np.float32), original)


def test_contract_names_and_valid_absences_are_exact():
    contract = json.loads(POLICY.with_name('contract.json').read_text())
    assert list(LOCAL_FEATURES) == contract['local_features']
    assert list(MESSAGE_FEATURES) == contract['message_features']
    observed = observation()
    values = observed['observation']
    assert values[27:29] == [1., 0.]  # No ending merge, not fabricated zero distance.
    assert values[33:43] == [0.] * 10  # Unknown/unneeded merge context.
    assert values[43] == pytest.approx(2. / 4.5)
    assert values[44:] == [0.] * 49
    assert observed['delivered_message_count'] == 0
    assert 'single autonomous' in observed['messages_reason']


def test_actual_bumper_gaps_and_straddling_vehicle_masks():
    ego = vehicle()
    other = vehicle('overlap', x=35., y=1.8, speed=8.)
    observed = observation(ego=ego, traffic=[other])
    assert observed['neighbors']['own']['front']['bumper_gap_m'] == pytest.approx(10.2)
    assert observed['neighbors']['left']['front']['bumper_gap_m'] == pytest.approx(10.2)
    assert observed['mask'] == [0., 1., 0.]


def test_physical_lane_change_mask_and_occupied_lane_guard_are_separate():
    ego = vehicle(y=2.)
    traffic = [vehicle('origin_leader', x=35., y=0., speed=8.)]
    observed = observation(ego=ego, traffic=traffic, lane_change_active=True)
    assert observed['lane_index'] == 1
    assert observed['mask'] == [0., 1., 0.]
    assert observed['neighbors']['own']['front'] is None
    assert observed['acceleration_ceiling_m_s2'] == 2.
    assert observed['physical_acceleration_bounds']['physical_occupied_lane_0_following'] < -4.5
    result = StraightRoadPolicyBridge.filter_action(
        dict(requested_acceleration_m_s2=2., lane_delta=1), observed)
    assert result['filtered_acceleration_m_s2'] == -4.5
    assert result['filtered_lane_delta'] == 0
    assert 'lane_mask' in result['causes']
    assert 'physical_occupied_lane_0_following' in result['causes']


@pytest.mark.parametrize('ego,traffic,kwargs', [
    ({'id': 'ego'}, [], {}),
    (vehicle(), [vehicle()], {}),
    (dict(vehicle(), speed_m_s=float('nan')), [], {}),
    (dict(vehicle(), yaw_rad=3.14), [], {}),
    (vehicle(y=-4.), [], {}),
    (vehicle(x=2001.), [], {}),
    (vehicle(), [], {'ego_lane_index': 2}),
    (vehicle(), [], {'time_s': -1.}),
    (vehicle(), [], {'last_lane_change_s': 11.}),
    (vehicle(), [], {'distance_travelled_m': -1.}),
    (vehicle(), [], {'lane_change_active': 1}),
])
def test_missing_nonfinite_or_inconsistent_state_rejected(ego, traffic, kwargs):
    with pytest.raises(ValueError):
        observation(ego=ego, traffic=traffic, **kwargs)


@pytest.mark.parametrize('kwargs', [dict(lane_count=8), dict(lane_count=True),
    dict(road_length_m=0), dict(lane_width_m=0), dict(speed_limit_m_s=CAP_M_S+.1)])
def test_unsupported_map_rejected(kwargs):
    with pytest.raises(ValueError):
        StraightRoadPolicyBridge(**kwargs)


def test_frozen_actor_deterministic_matches_direct_export_and_keeps_weights():
    initial_hash = hashlib.sha256(POLICY.read_bytes()).hexdigest()
    assert initial_hash == EXPECTED_POLICY_SHA256
    frozen = FrozenPhysicalActor(POLICY)
    original = torch.jit.load(str(POLICY), map_location='cpu').eval()
    before = {key: value.clone() for key, value in frozen.actor.state_dict().items()}
    for lane in range(3):
        obs = observation(ego=vehicle(y=lane * 3.6))['observation']
        result = frozen.act(obs)
        repeat = frozen.act(obs)
        with torch.inference_mode():
            direct = original(torch.tensor([obs], dtype=torch.float32))[0].tolist()
        assert result == repeat
        assert result['normalized_acceleration'] == direct[0]
        assert result['requested_acceleration_m_s2'] == -4.5 + (direct[0] + 1.) * 3.25
        assert result['lane_delta'] == int(direct[1])
    assert all(torch.equal(before[key], value) for key, value in frozen.actor.state_dict().items())
    assert all(not parameter.requires_grad and parameter.grad is None for parameter in frozen.actor.parameters())
    assert hashlib.sha256(POLICY.read_bytes()).hexdigest() == initial_hash


def test_frozen_actor_rejects_wrong_checksum_contract_and_observation(tmp_path):
    wrong = tmp_path / 'wrong.pt'
    wrong.write_bytes(b'not the frozen actor')
    with pytest.raises(ValueError, match='checksum'):
        FrozenPhysicalActor(wrong)
    contract = json.loads(POLICY.with_name('contract.json').read_text())
    contract['normalization'] = 'different'
    changed = tmp_path / 'contract.json'
    changed.write_text(json.dumps(contract))
    with pytest.raises(ValueError, match='contract'):
        FrozenPhysicalActor(POLICY, changed)
    frozen = FrozenPhysicalActor(POLICY)
    for obs in ([0.] * 81, [float('nan')] * 93, [0.] * 93):
        with pytest.raises(ValueError):
            frozen.act(obs)
