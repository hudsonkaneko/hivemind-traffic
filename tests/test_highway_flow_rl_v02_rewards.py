"""Offline v02 reward and fake-native invariance checks; never launch SUMO."""
from copy import deepcopy
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from environments.highway_environment_v05 import HighwayEnvironmentV05Env, ContinuousMotionLaneGraph
from environments.highway_flow_rl_env import HighwayFlowRLEnv
from environments.highway_flow_rl_v02_env import HighwayFlowRLV02Env
from traffic.highway_flow_rl_v02_rewards import (DEFAULT_REWARD_WEIGHTS, MASK_VERSION,
    cruise_speed_component, map_preview, optional_detour_evidence, validate_weights)
from tests.test_highway_flow_rl_env import rl_bare
from tests.test_highway_merge_context import graph
from tests.test_highway_smooth_control import fake_motion


def evidence():
    state = dict(speed=16., acceleration=0., allowed_speed=18., length=4.8,
                 lane_identity={'kind': 'through'}, lateral_position=0., lateral_speed=0.)
    own = dict(lane_id='through', known=True, legal_speed_m_s=18., front=None,
               route=dict(known=True, reachable=True, distance_m=1000.),
               native_response_estimate_m_s2=.8,
               front_lateral_speed_over3=0., front_lateral_offset_overwidth=0.)
    target = dict(own, lane_id='ending', route=dict(known=True, reachable=False, distance_m=500.))
    context = dict(corridors=dict(own=own, right=target, left=deepcopy(own)),
                   entry=dict(known=False, entry_active=False), occupied_worst_acceleration_m_s2=.8)
    preview = dict(known=True, minimum_legal_speed_m_s=18., horizon_m=120., yielding_link_ahead=False)
    return state, context, preview


def v02_bare(graph):
    env = rl_bare(graph, jerk_reward_weight=.03)
    env.__class__ = HighwayFlowRLV02Env
    env.v02_reward_weights = dict(DEFAULT_REWARD_WEIGHTS)
    env._reset_v02()
    return env


@pytest.mark.parametrize('speed,expected', [(0., 0.), (9., .025), (18., .1), (18.1, 0.)])
def test_bounded_cruise_bonus_uses_executed_local_limit(speed, expected):
    state, context, preview = evidence()
    state['speed'] = speed
    value, audit = cruise_speed_component(state, context, preview)
    assert value == pytest.approx(expected)
    assert audit['legal_speed_m_s'] == 18.


@pytest.mark.parametrize('condition', ['leader', 'braking', 'following_response', 'yield',
    'entry', 'lower_limit', 'route_end', 'required_merge', 'unknown_map', 'cut_in', 'maneuver'])
def test_required_slowdowns_receive_no_added_penalty_or_speed_bonus(condition):
    state, context, preview = evidence()
    if condition == 'leader':
        context['corridors']['own']['front'] = dict(bumper_gap_m=25., relative_speed_m_s=-4.)
    elif condition == 'braking': state['acceleration'] = -4.5
    elif condition == 'following_response': context['occupied_worst_acceleration_m_s2'] = -1.
    elif condition == 'yield': preview['yielding_link_ahead'] = True
    elif condition == 'entry': context['entry']['entry_active'] = True
    elif condition == 'lower_limit': preview['minimum_legal_speed_m_s'] = 13.41
    elif condition == 'route_end': context['corridors']['own']['route']['distance_m'] = 30.
    elif condition == 'required_merge': context['corridors']['own']['route']['reachable'] = False
    elif condition == 'unknown_map': preview['known'] = False
    elif condition == 'cut_in': context['corridors']['right']['front_lateral_speed_over3'] = -.1
    value, audit = cruise_speed_component(state, context, preview, maneuver_in_progress=condition == 'maneuver')
    assert value == 0. and not audit['eligible'] and audit['exclusion_reasons']


@pytest.mark.parametrize('headway,relative,response,eligible', [
    (3., 1., .2, True), (2.999, 1., .2, False), (3., .999, .2, False),
    (3., 1., .1999, False), (4., -1., 1., False), (4., 2., -1., False)])
def test_opening_leader_exception_has_conservative_boundaries(headway, relative, response, eligible):
    state, context, preview = evidence()
    context['corridors']['own'].update(
        front=dict(bumper_gap_m=headway * state['speed'], relative_speed_m_s=relative),
        native_response_estimate_m_s2=response)
    value, audit = cruise_speed_component(state, context, preview)
    assert audit['eligible'] == eligible
    assert (value > 0.) == eligible


def test_optional_detour_and_required_route_counterexamples():
    state, context, _ = evidence()
    kinds = {'through': 'through', 'ending': 'auxiliary'}
    assert optional_detour_evidence(state, context, -1, kinds)['optional_detour']
    for change in ('required_merge', 'required_exit', 'through_lane', 'unknown_route', 'entry'):
        modified, changed_kinds = deepcopy(context), dict(kinds)
        if change == 'required_merge': modified['corridors']['own']['route']['reachable'] = False
        elif change == 'required_exit': modified['corridors']['right']['route']['reachable'] = True
        elif change == 'through_lane': changed_kinds['ending'] = 'through'
        elif change == 'unknown_route': modified['corridors']['right']['route']['known'] = False
        else: modified['entry']['entry_active'] = True
        assert not optional_detour_evidence(state, modified, -1, changed_kinds)['optional_detour'], change
    assert not optional_detour_evidence(state, context, 0, kinds)['optional_detour']


def test_useful_passing_exception_requires_observed_benefit_and_return_runway():
    state, context, _ = evidence()
    own, target = context['corridors']['own'], context['corridors']['right']
    own.update(front=dict(bumper_gap_m=35., relative_speed_m_s=-4.), native_response_estimate_m_s2=-1.)
    target['native_response_estimate_m_s2'] = 1.
    kinds = {'through': 'through', 'ending': 'auxiliary'}
    result = optional_detour_evidence(state, context, -1, kinds)
    assert result['passing_benefit'] and not result['optional_detour']
    target['route']['distance_m'] = result['return_runway_required_m'] - .01
    assert optional_detour_evidence(state, context, -1, kinds)['optional_detour']
    target['route']['distance_m'] = 500.
    own['front']['relative_speed_m_s'] = 2.
    own['native_response_estimate_m_s2'] = target['native_response_estimate_m_s2']
    assert optional_detour_evidence(state, context, -1, kinds)['optional_detour']


def test_penalty_event_waits_for_actual_motion_and_consumes_once(graph):
    env = v02_bare(graph)
    event = dict(started_at=1., evidence=dict(optional_detour=True))
    env._v02_pending['agent_0'] = event
    env.maneuvers['agent_0'] = dict(started_at=1., moved=False)
    assert env._consume_detour_event('agent_0') is None
    env.maneuvers['agent_0']['moved'] = True
    assert env._consume_detour_event('agent_0')['evidence']['optional_detour']
    assert env._consume_detour_event('agent_0') is None
    env._v02_pending['agent_0'] = event
    env.maneuvers.clear()
    assert env._consume_detour_event('agent_0') is None  # Native refusal is no detour.
    assert not env._v02_pending


def test_v02_changes_only_added_rewards_not_motion_masks_or_actor_inputs(graph):
    baseline, env = rl_bare(graph, jerk_reward_weight=.03), v02_bare(graph)
    for item in (baseline, env):
        fake_motion(item, [.2, -4.5, .1])
    for _ in range(3):
        action = {'agent_0': np.array([1., 0.], np.float32)}
        a, b = baseline.step(action), env.step(action)
        info = b[4]['agent_0']
        np.testing.assert_array_equal(a[0]['agent_0'], b[0]['agent_0'])
        assert b[0]['agent_0'].shape == (117,) and env.state().shape == (2832,)
        for key in ('requested_acceleration', 'filtered_acceleration', 'realized_acceleration',
                    'override', 'override_causes', 'lane_rejected', 'lane_applied_delta'):
            assert a[4]['agent_0'][key] == info[key]
        assert b[1]['agent_0'] == pytest.approx(sum(info['reward_terms'].values()))
        assert b[1]['agent_0'] - a[1]['agent_0'] == pytest.approx(
            info['reward_terms']['cruise_speed'] + info['reward_terms']['route_detour'])
        assert len(info['v02_reward_audit']['pre_action']['observation117']) == 117
        if info['realized_acceleration'] < -.05:
            assert info['reward_terms']['cruise_speed'] == 0.
    assert env.stats['overrides'] == baseline.stats['overrides']
    assert env.stats['agent_steps'] == baseline.stats['agent_steps']
    assert env._mask.__func__ is HighwayEnvironmentV05Env._mask
    assert env._lane_rejection_causes.__func__ is HighwayEnvironmentV05Env._lane_rejection_causes
    assert env.conn.vehicle.setAcceleration.call_args_list == baseline.conn.vehicle.setAcceleration.call_args_list
    assert env.conn.vehicle.changeLane.call_args_list == baseline.conn.vehicle.changeLane.call_args_list
    assert env.v02_mask_audit['decision_samples'] == 3
    assert env.v02_mask_audit['additional_blocked_actions'] == 0
    journal = [json.loads(line) for line in env.journal.getvalue().splitlines()]
    assert sum(sum(r['agents']['agent_0']['reward_terms'].values()) for r in journal) == pytest.approx(sum(env.stats['reward_terms'].values()))


@pytest.mark.parametrize('weights', [dict(cruise_speed=.1), dict(cruise_speed=True, route_detour=.5),
    dict(cruise_speed=-1., route_detour=.5), dict(cruise_speed=.1, route_detour=float('nan'))])
def test_invalid_weights_fail_before_runtime(weights, tmp_path):
    with pytest.raises(ValueError, match='v02_reward_weights'):
        HighwayFlowRLV02Env(tmp_path / 'missing.xml', tmp_path, v02_reward_weights=weights)


def test_report_keeps_weights_and_mask_denominators_separate(graph, monkeypatch):
    monkeypatch.setattr(HighwayFlowRLEnv, 'report', lambda self: dict(override_fraction=.25, reward_weights=dict(self.reward_weights)))
    env = v02_bare(graph)
    result = env.report()
    assert result['override_fraction'] == .25
    assert result['v02_mask_audit']['version'] == MASK_VERSION
    assert not result['v02_mask_audit']['changed']
    assert result['v02_reward_weights'] == dict(cruise_speed=.1, route_detour=.5)
    assert 'cruise_speed' not in result['reward_weights']


def saved_trace_audit(recording_dir, output_dir):
    """Inspect known historical examples without claiming an exact ABI replay.

    V01 did not log observation117. Use saved pre-action context and matching
    native frame plus the executed capped map; do not reconstruct/claim a
    historical policy observation or a learned effect of these new rewards.
    """
    recording_dir, output_dir = Path(recording_dir), Path(output_dir)
    root = Path(__file__).resolve().parents[1]
    if not output_dir.resolve().is_relative_to(root / 'outputs/highway-flow-rl/v02'):
        raise ValueError('Reward audit outputs must stay under project outputs/highway-flow-rl/v02')
    if output_dir.exists():
        raise ValueError('Use a fresh reward-audit output directory')
    recording_path = recording_dir / 'recording.json'
    action_path = next(recording_dir.glob('episode/*/actions.jsonl.gz'))
    recording = json.loads(recording_path.read_text())
    config_path = recording_dir / 'resolved-config.json'
    config = json.loads(config_path.read_text())
    matching = [r for r in config['validation_rows'] if r['seed'] == recording['seed']]
    if len(matching) != 1:
        raise ValueError('Recorded seed must identify one executed development row')
    ceiling = matching[0]['speed_limit_m_s']
    frames = {round(row['time'], 6): {s['id']: s for s in row['vehicles']} for row in recording['frames']}
    graph = ContinuousMotionLaneGraph(recording_dir / 'network.net.xml')
    graph.speed_limits = {lane: min(ceiling, speed) for lane, speed in graph.speed_limits.items()}
    rows, exclusions, pending, moved = [], {}, {}, set()
    with gzip.open(action_path, 'rt') as stream:
        for line in stream:
            decision = json.loads(line)
            info = decision['agents'].get('agent_0')
            frame = frames.get(round(decision['time'] - .1, 6), {}).get('agent_0')
            if not info or not frame:
                continue
            context = info['flow_context']
            cap = context['corridors']['own']['legal_speed_m_s']
            state = dict(frame, allowed_speed=cap, lane_position=frame['lane_position_m'])
            bonus, cruise = cruise_speed_component(state, context, map_preview(graph, state),
                maneuver_in_progress=frame['maneuver_in_progress'])
            veto = info['realized_acceleration'] is None or info['realized_acceleration'] < -.05
            if veto: bonus = 0.
            for reason in cruise['exclusion_reasons']:
                exclusions[reason] = exclusions.get(reason, 0) + 1
            kinds = {r['lane_id']: graph.identity(r['lane_id']).get('kind')
                     for r in context['corridors'].values() if r['lane_id']}
            detour = optional_detour_evidence(state, context, info['lane_request'], kinds)
            maneuver = info['maneuver'].get('committed_target')
            first_move = False
            if maneuver:
                key = (maneuver['started_at'], maneuver['source_lane'], maneuver['target_lane'])
                pending.setdefault(key, detour)
                first_move = bool(maneuver['moved'] and key not in moved)
                if first_move:
                    moved.add(key)
                    detour = pending[key]
            rows.append(dict(time_s=decision['time'], speed_m_s=frame['speed'], cruise_speed_reward=bonus,
                cruise_eligible=cruise['eligible'] and not veto,
                exclusion_reasons=cruise['exclusion_reasons'], measured_braking_veto=veto,
                lane_request=info['lane_request'], detour_evidence=detour,
                first_observed_motion=first_move,
                route_detour_reward=-.5 if first_move and detour['optional_detour'] else 0.))
    result = dict(schema='highway_flow_rl_v02_offline_reward_audit_v1', training=False, simulator_runs=0,
        scope='Focal agent_0 saved-context inspection, not complete observation replay or new policy evidence',
        missing_observation117=True, weights=DEFAULT_REWARD_WEIGHTS, samples=len(rows),
        cruise_formula='.1 * (pre_action_speed / executed_legal_limit)^2 when eligible, else zero',
        opening_leader_gate=dict(minimum_headway_s=3., minimum_opening_speed_m_s=1., minimum_response_m_s2=.2),
        cruise_eligible_decisions=sum(r['cruise_eligible'] for r in rows),
        cruise_speed_reward=sum(r['cruise_speed_reward'] for r in rows), exclusions=exclusions,
        detour_events=[r for r in rows if r['route_detour_reward']],
        inputs={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
                (recording_path, action_path, config_path, recording_dir / 'network.net.xml')},
        sources={str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in
                 (root / 'traffic/highway_flow_rl_v02_rewards.py', Path(__file__).resolve())},
        rows=rows)
    output_dir.mkdir(parents=True)
    (output_dir / 'reward-audit.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    return {k: v for k, v in result.items() if k != 'rows'}


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--recording', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(saved_trace_audit(args.recording, args.output), indent=2))
