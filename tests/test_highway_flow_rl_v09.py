"""V09 proposal: reward terms, environment hook, contract and runner wiring (no SUMO, no training)."""
import hashlib
import json
from copy import deepcopy
from pathlib import Path

import pytest

from traffic import highway_flow_rl_v06_rewards as v06
from traffic import highway_flow_rl_v09_rewards as v09

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'configs/training/highway-flow-rl-v09-round11.json'
ROUND9 = ROOT / 'configs/training/highway-flow-rl-v06-round9.json'
TARGET = dict(eligible=True, target_m_s=16.)


# --- reward terms -----------------------------------------------------------
def test_tracking_matches_v06_inside_tolerance_and_keeps_rising_to_8_m_s():
    for speed in (16., 15.2, 14.5, 17.9):
        assert v09.merge_tracking(TARGET, speed, True, .25) == pytest.approx(v06.merge_tracking(TARGET, speed, True, .25))
    assert v06.merge_tracking(TARGET, 13., True, .25) == pytest.approx(-.25)  # v06 is flat past 2 m/s
    assert v09.merge_tracking(TARGET, 13., True, .25) == pytest.approx(-.375)
    assert v09.merge_tracking(TARGET, 11., True, .25) < v09.merge_tracking(TARGET, 12., True, .25)
    assert v09.merge_tracking(TARGET, 4., True, .25) == pytest.approx(-1.)  # capped at 8 m/s
    assert v09.merge_tracking(TARGET, 13., False, .25) == 0.
    assert v09.merge_tracking(dict(eligible=False), 13., True, .25) == 0.
    assert v09.merge_tracking(TARGET, None, True, .25) == 0.


def test_jerk_excess_skips_only_the_raw_request_during_insertion_lag():
    for before in range(5):
        assert v09.jerk_excess(4., 30., 1.5, before) == 0.  # rate-limited insertion: request vs 0 m/s2
        assert v09.jerk_excess(8., 30., 1.5, before) == pytest.approx(v06.jerk_excess(8., 0., 1.5))
    assert v09.jerk_excess(4., 30., 1.5, 5) == pytest.approx(v06.jerk_excess(4., 30., 1.5)) == pytest.approx(-1.5)
    assert v09.jerk_excess(None, 3., 1.5, 7) == 0.


def _window(n=100, match=60, override=0, jerk=2., stopped=0, braking=0):
    return [dict(stopped=i < stopped, jerk=jerk, override=i < override,
                 neighbor_acceleration=-4.5 if i < braking else -1., speed_match=i < match) for i in range(n)]


def test_graded_quality_pays_match_fraction_scaled_by_calm_neighbors():
    payment, details = v09.graded_quality(_window(), 0.)
    assert payment == pytest.approx(5 * .6) and details['eligible']
    payment, details = v09.graded_quality(_window(braking=5), 0.)
    assert payment == pytest.approx(5 * .6 * .95) and details['neighbor_calm_fraction'] == pytest.approx(.95)
    assert v09.graded_quality(_window(match=100), 0.)[0] == pytest.approx(5.)


@pytest.mark.parametrize('window,approach', [(_window(jerk=5.5), 0.), (_window(override=6), 0.),
                                             (_window(stopped=11), 0.), (_window(), 1.2), ([], 0.)])
def test_graded_quality_requires_the_cars_own_gate_conditions(window, approach):
    assert v09.graded_quality(window, approach)[0] == 0.


# --- environment hook (logic only) -------------------------------------------
def _env():
    pytest.importorskip('torch')
    from types import SimpleNamespace
    from environments.highway_flow_rl_v09_env import HighwayFlowRLV09Env
    env = object.__new__(HighwayFlowRLV09Env)
    env.v06_weights, env.v09_settings, env._v05_seen = dict(v06.WEIGHTS), dict(v09.SETTINGS), set()
    env.graph = SimpleNamespace(lanes={})
    env.quality_results, env.quality_history, env.approach_stops = {}, {}, {}
    env._reset_v06()
    return env


def test_environment_terms_lag_aware_jerk_unsaturated_tracking_and_single_graded_payment():
    env = _env()
    snapshot = dict(ego_state=dict(speed=12., lane_id='x'), context={}, target=TARGET, gap_plan={}, time_s=0.)
    state = dict(speed=12.)
    totals = []
    for step in range(8):
        if step == 6:  # the stable-merge decision sets the gate's result and window
            env.quality_results['agent_0'] = dict(qualified=False)
            env.quality_history['agent_0'] = _window(match=50)
        info = dict(realized_jerk_m_s3=4., v04_reward_audit={})
        terms = env._extra_rewards('agent_0', info, snapshot, dict(raw_requested_jerk=20., requested=1.), state, True)
        env._v05_seen.add('agent_0')
        totals.append(terms)
        assert terms['merge_tracking'] == pytest.approx(-.5)  # 4 m/s slow: v06 would give -0.25
        assert terms['jerk_excess'] == (0. if step < 5 else pytest.approx(-1.5))
        assert terms['quality_merge_graded'] == (pytest.approx(2.5) if step == 6 else 0.)
        assert ('v09_quality_payment' in info) == (step == 6)
    metrics = env.v06_metrics
    assert metrics['reward_terms']['quality_merge_graded'] == pytest.approx(2.5)
    assert metrics['per_car']['agent_0']['decisions'] == 8
    assert env._graded_paid['agent_0']['payment'] == pytest.approx(2.5)


def test_environment_rejects_unreviewed_v09_settings():
    pytest.importorskip('torch')
    from environments.highway_flow_rl_v09_env import HighwayFlowRLV09Env
    env = object.__new__(HighwayFlowRLV09Env)
    with pytest.raises(ValueError):
        HighwayFlowRLV09Env.__init__(env, v09_settings=dict(v09.SETTINGS, tracking_cap=1.))


# --- contract ---------------------------------------------------------------
@pytest.fixture
def cfg():
    return json.loads(CONFIG.read_text(encoding='utf-8'))


def test_proposed_config_validates_and_is_not_authorized(cfg):
    from traffic import highway_flow_rl_v09_contract as contract
    contract.validate_config(cfg)
    assert cfg['training_authorized'] is False
    with pytest.raises(ValueError):
        contract.validate_config(cfg, require_training=True)
    assert cfg['round_index'] == 11 and cfg['budget']['total_decisions'] == 5230000
    assert cfg['stop_rules']['merge_speed_match_floor'] == .4343 and cfg['stop_rules']['merge_floor_action'] == 'stop'
    assert cfg['missed_gap_seed_check']['built'] == 15


def test_only_declared_fields_differ_from_round_9(cfg):
    from traffic import highway_flow_rl_v09_contract as contract
    parent = json.loads(ROUND9.read_bytes().replace(b'\r\n', b'\n').decode('utf-8'))
    assert hashlib.sha256(ROUND9.read_bytes().replace(b'\r\n', b'\n')).hexdigest() == contract.PARENT_CONFIG_SHA256
    view = contract.round9_view(cfg)
    changed = {k for k in set(view) | set(parent) if view.get(k) != parent.get(k)}
    assert changed == {'stop_rules', 'provenance', 'missed_gap_seed_check', 'authorization',
                       'training_authorized', 'publication_branch'}
    for key in ('training_rows', 'validation_rows', 'held_out_rows', 'optimizer', 'acceptance', 'safety',
                'v06_reward_weights', 'reward_weights', 'raw_request_scope', 'rate_limit', 'critic_warmup_decisions'):
        assert cfg[key] == parent[key]


@pytest.mark.parametrize('change', [
    lambda c: c.update(v09_reward_settings=dict(c['v09_reward_settings'], tracking_cap=2.)),
    lambda c: c.update(warm_start=dict(c['warm_start'], sha256='0' * 64)),
    lambda c: c.update(budget=dict(c['budget'], total_decisions=6000000)),
    lambda c: c['stop_rules'].update(merge_floor_action='report'),
    lambda c: c['stop_rules'].update(merge_speed_match_floor=.40),
    lambda c: c.update(optimizer=dict(c['optimizer'], target_kl=.2)),
    lambda c: c.update(round_index=10),
    lambda c: c.update(rate_limit='startup'),
    lambda c: c['training_rows'][0].update(seed=c['validation_rows'][0]['seed']),
    lambda c: c.update(execution=dict(c['execution'], worktree='.')),
    lambda c: c.update(schema='highway_flow_rl_v06'),
])
def test_contract_rejects_undeclared_changes(cfg, change):
    from traffic import highway_flow_rl_v09_contract as contract
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
    from experiments import run_highway_flow_rl_v09 as module
    for name in NAMES:  # restore the shared loop's globals after the test
        monkeypatch.setattr(module.base, name, getattr(module.base, name))
    module.install()
    return module


def test_install_points_shared_loop_at_v09_and_the_canonical_ledger(runner):
    from experiments import run_highway_flow_rl_v06 as previous
    from traffic import highway_flow_rl_v09_contract as contract
    base = runner.base
    assert base.env_for is runner.env_for and base.build_model is runner.build_model
    assert base.validate_config is contract.validate_config and base.CHECKPOINT_SCHEMA.endswith('v09')
    assert base.MappoTrainer is previous.BufferedMappoTrainer and base.ruled_gates is v06.v06_gates
    assert base.evaluate is previous.evaluate and base.episode is previous.episode
    assert set(runner.SOURCES) <= set(base.NEW_SOURCES) and base.OUTPUT_ROOT == ROOT / 'outputs/highway-flow-rl/v09'
    assert base.STUDY == runner.shared_study_directory() and base.STUDY.as_posix().endswith('outputs/highway-flow-rl/v02')
    if (ROOT / '.git').is_file():  # a linked worktree shares the canonical checkout's ledger, not its own
        assert not base.STUDY.resolve().is_relative_to(ROOT.resolve())


def test_warm_start_loader_requires_the_pinned_round9_checkpoint(runner, cfg, tmp_path):
    import torch
    from traffic.highway_flow_contract import file_digest
    directory = ROOT / 'outputs' / f'test-v09-warm-start-{tmp_path.name}'
    directory.mkdir(parents=True)
    try:
        path = directory / 'last.pt'
        good = dict(schema='highway_flow_rl_checkpoint_v06', model=runner.base.FlowModel().state_dict(),
                    optimizer={}, rng={}, state={}, config_sha256=cfg['parent_config_sha256'])
        torch.save(good, path)
        pinned = dict(cfg, warm_start=dict(path=path.relative_to(ROOT).as_posix(), sha256=file_digest(path)))
        model, trainer, saved = runner.build_model(pinned)
        assert saved['schema'] == 'highway_flow_rl_checkpoint_v06' and trainer.episodes_per_update == 4
        for bad in (dict(good, schema='highway_flow_rl_checkpoint_v04'), dict(good, config_sha256='0' * 64)):
            torch.save(bad, path)
            with pytest.raises(ValueError):
                runner.build_model(dict(cfg, warm_start=dict(pinned['warm_start'], sha256=file_digest(path))))
        with pytest.raises(ValueError):  # checksum pin
            runner.build_model(pinned)
    finally:
        for item in directory.iterdir():
            item.unlink()
        directory.rmdir()


# --- input staging ------------------------------------------------------------
def test_staging_copies_declared_inputs_with_hash_checks(tmp_path):
    from scripts import stage_highway_flow_rl_v09_inputs as stage
    source, target = tmp_path / 'canonical', tmp_path / 'worktree'
    files = {'outputs/net/a.net.xml': b'a', 'outputs/net/b.net.xml': b'b', 'outputs/ckpt/last.pt': b'w'}
    for relative, data in files.items():
        (source / relative).parent.mkdir(parents=True, exist_ok=True)
        (source / relative).write_bytes(data)
    sha = {k: hashlib.sha256(v).hexdigest() for k, v in files.items()}
    row = lambda net: dict(case=dict(network=net, network_sha256=sha[net]))  # noqa: E731
    cfg = dict(warm_start=dict(path='outputs/ckpt/last.pt', sha256=sha['outputs/ckpt/last.pt']),
               training_rows=[row('outputs/net/a.net.xml'), row('outputs/net/b.net.xml')],
               validation_rows=[row('outputs/net/a.net.xml')], held_out_rows=[dict(case=dict(network='x', network_sha256='y'))])
    result = stage.stage(cfg, source, target)
    assert {r['path'] for r in result} == set(files) and all(r['action'] == 'copied' for r in result)
    assert all(r['action'] == 'already_present' for r in stage.stage(cfg, source, target))
    (target / 'outputs/net/a.net.xml').write_bytes(b'changed')
    with pytest.raises(ValueError):
        stage.stage(cfg, source, target)
