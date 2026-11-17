"""V10 proposal: post-merge override cost, environment hook, contract and runner wiring (no SUMO, no training)."""
import hashlib
import json
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import pytest

from traffic import highway_flow_rl_v06_rewards as v06
from traffic import highway_flow_rl_v09_rewards as v09
from traffic import highway_flow_rl_v10_rewards as v10

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'configs/training/highway-flow-rl-v10-round12.json'
ROUND9 = ROOT / 'configs/training/highway-flow-rl-v06-round9.json'
TARGET = dict(eligible=True, target_m_s=16.)


# --- reward term ------------------------------------------------------------
def test_post_merge_override_cost_per_gate_counted_decision():
    assert v10.post_merge_override(0) == 0.
    assert v10.post_merge_override(1) == pytest.approx(-1.)
    total = 5 * (v10.post_merge_override(1) - v06.WEIGHTS.get('intervention', .2))
    assert -total > v09.SETTINGS['graded_quality_weight']  # failing the gate costs more than the best payment
    for bad in (-1, 1.5, True):
        with pytest.raises(ValueError):
            v10.post_merge_override(bad)


def test_v10_settings_extend_v09_without_changing_it():
    assert {k: v10.SETTINGS[k] for k in v09.SETTINGS} == v09.SETTINGS == v10.V09_SETTINGS
    assert set(v10.SETTINGS) - set(v09.SETTINGS) == {'post_merge_override_weight'}


# --- environment hook (logic only) -------------------------------------------
def _env():
    pytest.importorskip('torch')
    from environments.highway_flow_rl_v10_env import HighwayFlowRLV10Env
    env = object.__new__(HighwayFlowRLV10Env)
    env.v06_weights, env.v09_settings, env.v10_settings = dict(v06.WEIGHTS), dict(v09.SETTINGS), dict(v10.SETTINGS)
    env._v05_seen, env.graph = set(), SimpleNamespace(lanes={})
    env.quality_results, env.quality_history, env.approach_stops = {}, {}, {}
    env.merge_trackers = {}
    env._reset_v06()
    return env


def test_environment_charges_exactly_the_trackers_post_merge_interventions():
    env = _env()
    tracker = SimpleNamespace(post_window=dict(intervention_decisions=0))
    env.merge_trackers['agent_0'] = tracker
    snapshot = dict(ego_state=dict(speed=12., lane_id='x'), context={}, target=TARGET, gap_plan={}, time_s=0.)
    counts = [0, 0, 1, 1, 3, 3, 3]  # cumulative window count after each decision (two at once on step 4)
    charged = []
    for step, count in enumerate(counts):
        tracker.post_window['intervention_decisions'] = count
        info = dict(realized_jerk_m_s3=1., v04_reward_audit={})
        terms = env._extra_rewards('agent_0', info, snapshot, dict(raw_requested_jerk=1., requested=0.), dict(speed=12.), True)
        env._v05_seen.add('agent_0')
        charged.append(terms['post_merge_override'])
        assert terms['merge_tracking'] == pytest.approx(-.5)  # v09 terms are unchanged
        assert set(terms) == {'jerk_excess', 'merge_tracking', 'avoidable_braking', 'quality_merge_graded',
                              'post_merge_override'}
    assert charged == [0., 0., -1., 0., -2., 0., 0.]
    assert env.v06_metrics['reward_terms']['post_merge_override'] == pytest.approx(-3.)
    assert env._extra_rewards('agent_1', dict(realized_jerk_m_s3=1., v04_reward_audit={}), snapshot,
                              dict(raw_requested_jerk=1., requested=0.), dict(speed=12.), True)['post_merge_override'] == 0.
    env._reset_v06()  # a new episode starts its own count
    assert env._window_interventions == {}


def test_environment_rejects_unreviewed_v10_settings():
    pytest.importorskip('torch')
    from environments.highway_flow_rl_v10_env import HighwayFlowRLV10Env
    env = object.__new__(HighwayFlowRLV10Env)
    with pytest.raises(ValueError):
        HighwayFlowRLV10Env.__init__(env, v10_settings=dict(v10.SETTINGS, post_merge_override_weight=3.))


# --- contract ---------------------------------------------------------------
@pytest.fixture
def cfg():
    return json.loads(CONFIG.read_text(encoding='utf-8'))


def test_proposed_config_validates_and_is_not_authorized(cfg):
    from traffic import highway_flow_rl_v10_contract as contract
    contract.validate_config(cfg)
    assert cfg['training_authorized'] is False and cfg['held_out_authorized'] is False
    with pytest.raises(ValueError):
        contract.validate_config(cfg, require_training=True)
    assert cfg['round_index'] == 12 and cfg['budget']['per_round_decisions'] == 200000
    assert cfg['budget']['total_decisions'] == 5230000  # no new decision ceiling
    assert cfg['budget']['max_diagnostic_episodes'] == 127
    assert 5230000 - 5022403 >= cfg['budget']['per_round_decisions']  # ledger after round 11
    assert cfg['stop_rules']['merge_speed_match_floor'] == .4343 and cfg['stop_rules']['merge_floor_action'] == 'stop'
    assert cfg['missed_gap_seed_check']['built'] == 15 and all(r['blocker'] for r in cfg['missed_gap_seed_check']['rows'])


def test_only_declared_fields_differ_from_round_9(cfg):
    from traffic import highway_flow_rl_v10_contract as contract
    raw = ROUND9.read_bytes().replace(b'\r\n', b'\n')
    assert hashlib.sha256(raw).hexdigest() == contract.PARENT_CONFIG_SHA256
    parent = json.loads(raw.decode('utf-8'))
    view = contract.round9_view(cfg)
    changed = {k for k in set(view) | set(parent) if view.get(k) != parent.get(k)}
    assert changed == {'stop_rules', 'provenance', 'missed_gap_seed_check', 'authorization',
                       'training_authorized', 'publication_branch'}
    for key in ('training_rows', 'validation_rows', 'held_out_rows', 'optimizer', 'acceptance', 'safety',
                'v06_reward_weights', 'reward_weights', 'raw_request_scope', 'rate_limit', 'critic_warmup_decisions'):
        assert cfg[key] == parent[key]


def test_round12_training_seeds_avoid_evaluation_seeds_and_missed_gap_check_covers_the_round(cfg):
    held = {r['seed'] for r in cfg['validation_rows'] + cfg['held_out_rows']}
    rows = cfg['training_rows']
    for cycle in range(40):
        for row in rows:
            for critic in (0, 50000):
                assert row['seed'] + 1000 * cycle + 10000 * 11 + critic not in held
    reachable = -(-cfg['budget']['per_round_decisions'] // sum(r['rollout_decisions'] for r in rows))
    built = {(c['row'], c['seed']) for c in cfg['missed_gap_seed_check']['rows']}
    for i, row in enumerate(rows):
        if row.get('missed_gap'):
            assert all((i, row['seed'] + 1000 * c + 110000) in built for c in range(reachable))
            assert (i, row['seed'] + 110000 + 50000) in built


@pytest.mark.parametrize('change', [
    lambda c: c.update(v10_reward_settings=dict(c['v10_reward_settings'], post_merge_override_weight=2.)),
    lambda c: c.update(v10_reward_settings=dict(c['v10_reward_settings'], tracking_cap=1.)),
    lambda c: c.update(v09_reward_settings=dict(v09.SETTINGS)),
    lambda c: c.update(warm_start=dict(path='outputs/highway-flow-rl/v09/round-11/last.pt', sha256='0' * 64)),
    lambda c: c.update(budget=dict(c['budget'], per_round_decisions=300000)),
    lambda c: c.update(budget=dict(c['budget'], total_decisions=5530000)),
    lambda c: c['stop_rules'].update(merge_floor_action='report'),
    lambda c: c.update(optimizer=dict(c['optimizer'], target_kl=.2)),
    lambda c: c.update(round_index=11),
    lambda c: c.update(reward_version=v09.REWARD_VERSION),
    lambda c: c['training_rows'][0].update(seed=c['validation_rows'][0]['seed']),
    lambda c: c.update(execution=dict(c['execution'], output_root='outputs/highway-flow-rl/v09')),
    lambda c: c.update(schema='highway_flow_rl_v09'),
])
def test_contract_rejects_undeclared_changes(cfg, change):
    from traffic import highway_flow_rl_v10_contract as contract
    changed = deepcopy(cfg)
    change(changed)
    with pytest.raises(ValueError):
        contract.validate_config(changed)


# --- runner wiring ----------------------------------------------------------
NAMES = ('OUTPUT_ROOT', 'CHECKPOINT_SCHEMA', 'NEW_SOURCES', 'validate_config', 'validate_preflight', 'env_for',
         'episode', 'evaluate', 'build_model', 'ruled_gates', 'MappoTrainer', 'STUDY')


@pytest.fixture
def runner(monkeypatch):
    pytest.importorskip('torch')
    from experiments import run_highway_flow_rl_v10 as module
    for name in NAMES:  # restore the shared loop's globals after the test
        monkeypatch.setattr(module.base, name, getattr(module.base, name))
    module.install()
    return module


def test_install_points_shared_loop_at_v10_with_the_v09_loader_and_canonical_ledger(runner):
    from experiments import run_highway_flow_rl_v06 as previous
    from experiments import run_highway_flow_rl_v09 as v09_runner
    from traffic import highway_flow_rl_v10_contract as contract
    base = runner.base
    assert base.env_for is runner.env_for and base.build_model is v09_runner.build_model
    assert base.validate_config is contract.validate_config and base.CHECKPOINT_SCHEMA.endswith('v10')
    assert base.MappoTrainer is previous.BufferedMappoTrainer and base.ruled_gates is v06.v06_gates
    assert base.evaluate is previous.evaluate and base.episode is previous.episode
    assert set(runner.SOURCES) <= set(base.NEW_SOURCES) and base.OUTPUT_ROOT == ROOT / 'outputs/highway-flow-rl/v10'
    assert base.STUDY == runner.shared_study_directory() and base.STUDY.as_posix().endswith('outputs/highway-flow-rl/v02')
    assert all((ROOT / name).is_file() for name in base.NEW_SOURCES)


def test_staging_reuses_the_v09_checks_with_the_v10_contract():
    from scripts import stage_highway_flow_rl_v09_inputs as old
    from scripts import stage_highway_flow_rl_v10_inputs as new
    assert new.stage is old.stage and new.declared_inputs is old.declared_inputs
    assert new.contract.SCHEMA == 'highway_flow_rl_v10'
