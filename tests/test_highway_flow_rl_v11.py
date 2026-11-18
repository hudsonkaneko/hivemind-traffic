"""V11: merge-speed precondition rule, environment mask hook, contract and runner wiring (no SUMO, no training)."""
import json
from copy import deepcopy
from pathlib import Path

import pytest

from traffic import highway_flow_rl_v10_rewards as v10
from traffic import highway_flow_rl_v11_contract as contract
from traffic import highway_flow_rl_v11_rewards as v11

ROOT = Path(__file__).resolve().parents[1]
RULE = v11.SETTINGS_BY_ROUND[13]


def config_path(round_index):
    return ROOT / f'configs/training/highway-flow-rl-v11-round{round_index}.json'


# --- rule ---------------------------------------------------------------------
def test_matched_run_counts_only_the_trailing_matched_decisions():
    samples = [dict(speed_match=m) for m in (True, True, False, True, True, True)]
    assert v11.matched_run(samples) == 3
    assert v11.matched_run(samples + [dict(speed_match=False)]) == 0
    assert v11.matched_run([]) == v11.matched_run(None) == 0


def test_merge_waits_for_matched_speed_and_releases_near_the_lane_end():
    wait = v11.merge_speed_precondition(39, 600., 20., RULE)
    assert not wait['permitted'] and not wait['matched'] and not wait['released']
    assert v11.merge_speed_precondition(40, 600., 20., RULE)['permitted']
    # release distance is 10 s of travel, at least 150 m: the rule can only delay a merge
    assert v11.merge_speed_precondition(0, 200., 20., RULE) == dict(permitted=True, matched=False, released=True,
        matched_run=0, required_run=40, remaining_m=200., release_distance_m=200.)
    assert not v11.merge_speed_precondition(0, 201., 20., RULE)['permitted']
    assert v11.merge_speed_precondition(0, 150., 3., RULE)['released']
    assert not v11.merge_speed_precondition(0, 151., 3., RULE)['permitted']
    for bad in ((-1, 10., 10.), (1.5, 10., 10.), (1, float('nan'), 10.), (1, 10., -1.)):
        with pytest.raises(ValueError):
            v11.merge_speed_precondition(*bad, RULE)


def test_dwell_leaves_room_under_the_quality_gate():
    # 161 window samples, at most 16 may miss; 50 precede a 3 s lane change request.
    assert 50 - RULE['merge_dwell_decisions'] <= 16
    assert v11.validate_settings(RULE) == RULE
    for bad in ({}, dict(RULE, merge_dwell_decisions=1), dict(RULE, extra=1)):
        with pytest.raises(ValueError):
            v11.validate_settings(bad)
    assert v11.V10_SETTINGS == v10.SETTINGS


# --- environment hook (logic only) ---------------------------------------------
def _env(monkeypatch, *, reachable=False, entry_known=True, lane_delta=1, distance=600., earlier=(0., 1., 1.)):
    pytest.importorskip('torch')
    from environments import highway_flow_rl_v11_env as module
    env = object.__new__(module.HighwayFlowRLV11Env)
    env.v11_settings = dict(RULE)
    env.states, env.graph, env.time = dict(agent_0=dict(speed=20.)), None, 3.2
    env.quality_history = {}
    env._merge_gate, env._merge_gate_counted = {}, set()
    env.v11_metrics = dict(blocked_decisions=0, per_car={})
    flow = dict(corridors=dict(own=dict(route=dict(known=True, reachable=reachable, distance_m=distance))),
                entry=dict(known=entry_known, lane_delta=lane_delta))
    monkeypatch.setattr(module, 'flow_context', lambda graph, state, states: deepcopy(flow))
    monkeypatch.setattr(module.HighwayFlowRLV10Env, '_mask', lambda self, agent, context: list(earlier))
    return env, flow


def test_mask_blocks_the_merge_until_speed_is_matched(monkeypatch):
    env, _ = _env(monkeypatch)
    env.quality_history['agent_0'] = [dict(speed_match=True)] * 39
    assert env._mask('agent_0', {}) == [0., 1., 0.]
    assert env._mask('agent_0', {}) == [0., 1., 0.]          # built twice in one decision: counted once
    assert env.v11_metrics['blocked_decisions'] == 1
    assert env.v11_metrics['per_car']['agent_0']['first_blocked_time_s'] == 3.2
    env.quality_history['agent_0'].append(dict(speed_match=True))
    assert env._mask('agent_0', {}) == [0., 1., 1.]          # 40 matched decisions: the earlier mask applies
    assert env._merge_gate['agent_0']['matched'] and env.v11_metrics['blocked_decisions'] == 1


def test_mask_only_removes_permissions_and_only_toward_the_merge_lane(monkeypatch):
    env, _ = _env(monkeypatch, earlier=(1., 1., 0.))           # gap rules already forbid the merge side
    assert env._mask('agent_0', {}) == [1., 1., 0.]            # the other side is untouched
    assert env.v11_metrics['blocked_decisions'] == 0           # nothing was removed by this rule
    env, _ = _env(monkeypatch, lane_delta=-1, earlier=(1., 1., 1.))
    assert env._mask('agent_0', {}) == [0., 1., 1.]


@pytest.mark.parametrize('kwargs', [dict(reachable=True), dict(entry_known=False), dict(lane_delta=0)])
def test_rule_does_not_apply_to_cars_that_need_no_merge(monkeypatch, kwargs):
    env, _ = _env(monkeypatch, **kwargs)
    assert env._mask('agent_0', {}) == [0., 1., 1.] and env._merge_gate['agent_0'] is None


def test_rule_releases_near_the_lane_end_and_records_how_the_merge_started(monkeypatch):
    env, _ = _env(monkeypatch, distance=200.)                  # 10 s at 20 m/s
    assert env._mask('agent_0', {}) == [0., 1., 1.]
    from environments import highway_flow_rl_v11_env as module
    monkeypatch.setattr(module.HighwayFlowRLV10Env, '_begin_maneuver', lambda self, agent, delta: None)
    env._begin_maneuver('agent_0', 1)
    car = env.v11_metrics['per_car']['agent_0']
    assert car['started_by'] == 'lane_end_release' and car['matched_run_at_start'] == 0
    assert car['merge_started_time_s'] == 3.2 and car['speed_at_start_m_s'] == 20.


def test_environment_refuses_unreviewed_settings():
    pytest.importorskip('torch')
    from environments.highway_flow_rl_v11_env import HighwayFlowRLV11Env
    with pytest.raises(ValueError, match='Unreviewed v11 settings'):
        HighwayFlowRLV11Env(None, None, v11_settings=dict(RULE, merge_dwell_decisions=5))


# --- contract and configuration -------------------------------------------------
@pytest.fixture(params=sorted(contract.ROUNDS))
def cfg(request):
    return json.loads(config_path(request.param).read_text(encoding='utf-8'))


def test_committed_config_is_what_the_prepare_script_builds(cfg):
    from scripts import prepare_highway_flow_rl_v11 as prepare
    assert prepare.build(cfg['round_index'], check_inputs=False) == cfg
    assert contract.validate_config(deepcopy(cfg)) == cfg
    assert cfg['held_out_authorized'] is False and cfg['training_authorized'] is contract.ROUNDS[cfg['round_index']]['training']
    assert 'use some autonomy' in cfg['authorization']


def test_only_declared_fields_differ_from_round_9(cfg):
    round9 = json.loads((ROOT / 'configs/training/highway-flow-rl-v06-round9.json').read_bytes().replace(b'\r\n', b'\n'))
    changed = {k for k in set(cfg) | set(round9) if cfg.get(k) != round9.get(k)}
    assert changed <= {'schema', 'round_index', 'training_authorized', 'learning_profile', 'reward_version',
                       'v10_reward_settings', 'v11_settings', 'v11_mask_version', 'warm_start', 'parent_config_sha256',
                       'budget', 'stop_rules', 'provenance', 'publication_branch', 'execution', 'authorization',
                       'missed_gap_seed_check', 'held_out_authorized'}
    for key in ('training_rows', 'validation_rows', 'held_out_rows', 'safety', 'acceptance', 'optimizer',
                'action_contract', 'rate_limit', 'mask_version', 'observation_size'):
        assert cfg[key] == round9[key]


@pytest.mark.parametrize('change', [
    dict(round_index=99), dict(learning_profile='other'), dict(held_out_authorized=True),
    dict(v11_settings=dict(RULE, merge_dwell_decisions=10)), dict(v11_mask_version='other'),
    dict(warm_start=dict(path='x', sha256='0' * 64)), dict(publication_branch='main'),
    dict(rate_limit='startup'), dict(schema='highway_flow_rl_v10')])
def test_contract_rejects_undeclared_changes(cfg, change):
    with pytest.raises(ValueError):
        contract.validate_config(dict(deepcopy(cfg), **change))


def test_contract_rejects_budget_stop_rule_and_authorization_drift(cfg):
    for mutate in (lambda c: c['budget'].update(total_decisions=c['budget']['total_decisions'] + 1),
                   lambda c: c['stop_rules'].update(merge_speed_match_floor=0.),
                   lambda c: c['stop_rules'].update(any_collision=False),
                   lambda c: c.update(training_authorized=not c['training_authorized']),
                   lambda c: c['optimizer'].update(actor_lr=1.)):
        changed = deepcopy(cfg)
        mutate(changed)
        with pytest.raises(ValueError):
            contract.validate_config(changed)


def test_study_budget_is_the_bounded_extension_declared_for_the_autonomy_decision():
    assert contract.STUDY_BUDGET['total_decisions'] == 5230000 + 1000000
    assert contract.STUDY_BUDGET['total_wall_seconds'] == 46800 + 4 * 3600
    assert contract.STUDY_BUDGET['max_diagnostic_episodes'] == 128 + 40
    assert contract.ROUNDS[13]['training'] is False and contract.ROUNDS[13]['budget']['per_round_decisions'] == 0


def test_install_points_shared_loop_at_v11_and_keeps_the_canonical_ledger():
    pytest.importorskip('torch')
    from experiments import run_highway_flow_rl_v11 as runner
    saved = {k: getattr(runner.base, k) for k in ('OUTPUT_ROOT', 'CHECKPOINT_SCHEMA', 'NEW_SOURCES', 'validate_config',
                                                  'validate_preflight', 'env_for', 'build_model', 'STUDY')}
    try:
        runner.install()
        assert runner.base.env_for is runner.env_for and runner.base.validate_config is contract.validate_config
        assert runner.base.OUTPUT_ROOT == ROOT / 'outputs/highway-flow-rl/v11'
        assert runner.base.STUDY == runner.shared_study_directory()
        assert set(runner.SOURCES) <= set(runner.base.NEW_SOURCES)
        assert all((ROOT / name).is_file() for name in runner.SOURCES)
    finally:
        for key, value in saved.items():
            setattr(runner.base, key, value)
