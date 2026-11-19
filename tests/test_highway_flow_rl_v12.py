"""V12: target-speed inputs, the target_band reward, the 117-to-121 migration, the anchored
trainer, the contract and the runner wiring (no SUMO, no training)."""
import json
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from traffic import highway_flow_rl_v12_contract as contract
from traffic import highway_flow_rl_v12_rewards as v12

ROOT = Path(__file__).resolve().parents[1]
S = v12.SETTINGS
ACTIVE = dict(eligible=True, target_m_s=18.)


# --- reward and inputs ---------------------------------------------------------
def test_band_reward_is_zero_inside_the_tolerance_and_linear_outside_it():
    assert S['band_tolerance_m_s'] == 1.8 < 2.0          # inside the gate's own 2 m/s match window
    for speed in (16.25, 17., 18., 19.75):
        assert v12.target_band(ACTIVE, speed, True) == 0.
    assert v12.target_band(ACTIVE, 18. + 3.6, True) == pytest.approx(-1.)
    assert v12.target_band(ACTIVE, 18. - 5.4, True) == pytest.approx(-2.)
    assert v12.target_band(ACTIVE, 40., True) == pytest.approx(-4.)   # capped
    assert v12.target_band(ACTIVE, 0., True) == pytest.approx(-4.)


def test_band_reward_and_inputs_apply_only_to_pending_merges_with_an_eligible_target():
    for target, pending in ((ACTIVE, False), (dict(eligible=False, target_m_s=18.), True), (None, True),
                            (dict(eligible=True, target_m_s=float('nan')), True)):
        assert v12.target_band(target, 10., pending) == 0.
        assert v12.target_features(target, 10., pending).tolist() == [0., 0., 0., 0.]
    assert v12.target_band(ACTIVE, None, True) == 0.


def test_target_inputs_carry_the_target_the_error_and_the_band():
    from traffic.highway_merge_context import CAP
    far = v12.target_features(ACTIVE, 10., True)
    assert far.tolist() == pytest.approx([1., 18. / CAP, 8. / 9., 0.])
    near = v12.target_features(ACTIVE, 19., True)
    assert near.tolist() == pytest.approx([1., 18. / CAP, -1. / 9., 1.])
    assert v12.target_features(ACTIVE, -20., True)[2] == 1.               # clipped
    assert v12.target_features(dict(eligible=True, target_m_s=40.), 30., True)[1] == 1.


def test_target_inputs_fade_in_and_insert_before_the_message_slots():
    raw = np.array([1., .7, .5, 1.], np.float32)
    faded, steps = None, 0
    while faded is None or not np.array_equal(faded, raw):
        faded = v12.fade(faded if steps else np.zeros(4, np.float32), raw)
        steps += 1
    assert steps == S['fade_steps']                                       # a 0-to-1 flip takes 10 decisions
    back = v12.fade(raw, np.zeros(4))
    for _ in range(S['fade_steps'] - 1):
        back = v12.fade(back, np.zeros(4))
    assert back.tolist() == [0., 0., 0., 0.]                              # lands exactly on zero
    observation = np.arange(117, dtype=np.float32) / 117.
    wide = v12.insert(observation, raw)
    assert wide.shape == (121,) and np.array_equal(wide[:69], observation[:69])
    assert np.array_equal(wide[69:73], raw) and np.array_equal(wide[73:], observation[69:])
    with pytest.raises(ValueError):
        v12.insert(np.zeros(121), raw)
    for bad in ({}, dict(S, band_weight=2.), dict(S, extra=1)):
        with pytest.raises(ValueError):
            v12.validate_settings(bad)


# --- environment (logic only) --------------------------------------------------
def _env(monkeypatch, pending=True):
    pytest.importorskip('torch')
    from environments import highway_flow_rl_v12_env as module
    env = object.__new__(module.HighwayFlowRLV12Env)
    env.v12_settings = dict(S)
    env.possible_agents = [f'agent_{i}' for i in range(24)]
    env.agents = ['agent_0', 'agent_1']
    env.graph = None
    env.states = dict(agent_0=dict(speed=10.), agent_1=dict(speed=19.), agent_5=dict(speed=12.))
    env.flow_contexts = dict(agent_0={}, agent_1={}, agent_5={})
    env.merge_trackers = dict(agent_0=SimpleNamespace(status='pending' if pending else 'stable'),
                              agent_1=SimpleNamespace(status='stable'), agent_5=SimpleNamespace(status='pending'))
    env._target_inputs, env._augmented = {}, {}
    monkeypatch.setattr(module, 'map_evidence', lambda graph, state: None)
    monkeypatch.setattr(module, 'merge_speed_target', lambda state, context, preview: (dict(ACTIVE), {}))
    return env, module


def test_environment_adds_faded_inputs_for_live_cars_and_raw_ones_for_departed_cars(monkeypatch):
    env, module = _env(monkeypatch)
    observations = {a: np.full(117, .25, np.float32) for a in ('agent_0', 'agent_1', 'agent_5')}
    first = env._augment(observations)
    assert all(value.shape == (121,) for value in first.values())
    assert first['agent_0'][69:73].tolist() == pytest.approx([1., .18 / .268224, 8. / 9., 0.], abs=1e-6)  # first decision
    assert first['agent_1'][69:73].tolist() == [0., 0., 0., 0.]           # merge not pending
    assert first['agent_5'][69] == 1. and 'agent_5' not in env._target_inputs
    env.merge_trackers['agent_0'].status = 'stable'
    second = env._augment(observations)
    assert second['agent_0'][69] == pytest.approx(.9)                      # fades out
    state = env.state()
    assert state.shape == (module.STATE_SIZE,) == (2928,)
    rows = state.reshape(24, 122)
    assert np.array_equal(rows[0, :121], second['agent_0']) and rows[0, 121] == 1.
    assert rows[5].tolist() == [0.] * 122                                  # not live: absent from the critic state
    assert env.observation_space('agent_0').shape == (121,)


def test_environment_charges_the_band_on_the_pre_decision_target(monkeypatch):
    env, module = _env(monkeypatch)
    monkeypatch.setattr(module.HighwayFlowRLV11Env, '_extra_rewards', lambda *args: dict(merge_tracking=-.1))
    env.v06_metrics = dict(reward_terms={})
    env.v12_metrics = dict(decisions=0, band_decisions=0, in_band_decisions=0, band_reward=0., per_car={})
    snapshot = dict(target=dict(ACTIVE), time_s=4.2)
    info = {}
    terms = env._extra_rewards('agent_0', info, snapshot, {}, dict(speed=18. + 5.4), True)
    assert terms == dict(merge_tracking=-.1, target_band=pytest.approx(-2.))
    terms = env._extra_rewards('agent_0', info, snapshot, {}, dict(speed=19.), True)
    assert terms['target_band'] == 0.
    car = env.v12_metrics['per_car']['agent_0']
    assert (car['band_decisions'], car['in_band_decisions'], car['first_time_s']) == (2, 1, 4.2)
    assert env._extra_rewards('agent_1', info, snapshot, {}, dict(speed=0.), False)['target_band'] == 0.
    assert env.v06_metrics['reward_terms']['target_band'] == pytest.approx(-2.)


def test_environment_refuses_unreviewed_settings():
    pytest.importorskip('torch')
    from environments.highway_flow_rl_v12_env import HighwayFlowRLV12Env
    with pytest.raises(ValueError):
        HighwayFlowRLV12Env('missing.net.xml', ROOT / 'outputs', v12_settings=dict(S, band_cap=10.))


# --- model migration and trainer ---------------------------------------------------
def _parent_and_migrated():
    torch = pytest.importorskip('torch')
    from policies import highway_flow, highway_flow_v12 as flow12
    torch.manual_seed(5)
    parent = highway_flow.FlowModel()
    with torch.no_grad():
        for parameter in parent.parameters():
            parameter.add_(torch.randn_like(parameter) * .05)
    model, info = flow12.migrate_from_flow117(parent.state_dict())
    return torch, parent, model, info, flow12


def _observations(torch, count=32):
    observations = torch.rand(count, 117) * 2. - 1.
    observations[:, 30:33] = 1.
    return observations


def test_migration_keeps_every_actor_and_critic_output_whatever_the_new_inputs_hold():
    torch, parent, model, info, flow12 = _parent_and_migrated()
    assert (model.observation_size, model.state_size) == (121, 2928)
    assert info['inserted_local_indices'] == [69, 70, 71, 72] and info['inserted_weights'] == 'zero'
    narrow = _observations(torch)
    wide = torch.cat((narrow[:, :69], torch.rand(32, 4), narrow[:, 69:]), 1)
    with torch.no_grad():
        assert torch.allclose(parent.actor.mean_net(narrow), model.actor.mean_net(wide), atol=1e-6)
        assert torch.equal(parent.act(narrow, deterministic=True)[0], model.act(wide, deterministic=True)[0])
        rows = torch.cat((narrow, torch.ones(32, 1)), 1)[:24].reshape(1, -1)
        wide_rows = torch.cat((wide, torch.ones(32, 1)), 1)[:24].reshape(1, -1)
        indices = torch.arange(24)
        assert torch.allclose(parent.values(rows.repeat(24, 1), indices),
                              model.values(wide_rows.repeat(24, 1), indices), atol=1e-5)
    assert torch.equal(parent.log_std, model.log_std)
    with pytest.raises(ValueError):
        flow12.migrate_from_flow117(model.state_dict())                  # already 121 wide


def test_exported_v12_actor_matches_the_model(tmp_path):
    torch, _, model, _, _ = _parent_and_migrated()
    from policies.mappo import export_actor
    path = export_actor(model, tmp_path / 'actor.pt')
    actor = torch.jit.load(str(path))
    wide = torch.cat((_observations(torch)[:, :69], torch.rand(32, 4), _observations(torch)[:, 69:]), 1)
    with torch.no_grad():
        assert torch.equal(model.act(wide, deterministic=True)[0], actor(wide))


def _part(torch, wide, active_rows):
    wide = wide.clone()
    wide[:, 69] = 0.
    wide[active_rows, 69] = 1.
    return dict(local_obs=wide)


def test_anchor_holds_only_decisions_without_an_active_target(monkeypatch):
    torch, _, model, _, flow12 = _parent_and_migrated()
    from experiments import run_highway_flow_rl_v12 as runner
    from policies.mappo import MappoTrainer
    monkeypatch.setattr(MappoTrainer, '_loss_terms', lambda self, part, ratio, entropy: (
        self.model.actor.mean_net(part['local_obs']).sum() * 0., {}, {}))
    anchor = flow12.frozen_copy(model)
    trainer = runner.AnchoredShieldTrainer(model, anchor=anchor, anchor_coef=10., anchor_lane_coef=1.,
                                           shield_coef=0., actor_lr=1e-3, critic_lr=1e-3)
    wide = torch.cat((_observations(torch)[:, :69], torch.zeros(32, 4), _observations(torch)[:, 69:]), 1)
    part = _part(torch, wide, slice(0, 8))
    loss, weighted, stats = trainer._loss_terms(part, None, None)
    assert float(loss) == 0. and stats['anchor_held_fraction'] == .75 and stats['target_active_samples'] == 8
    with torch.no_grad():
        for parameter in model.actor.parameters():
            parameter.add_(torch.randn_like(parameter) * .05)
    loss, weighted, stats = trainer._loss_terms(part, None, None)
    assert float(weighted['anchor']) > 0. and float(weighted['anchor_lane']) > 0.
    # the loss depends only on the held rows: changing the active rows' other inputs changes nothing
    moved = part['local_obs'].clone()
    moved[:8, :69] = torch.rand(8, 69)
    again = trainer._loss_terms(dict(local_obs=moved), None, None)[0]
    assert torch.allclose(loss, again)
    with pytest.raises(ValueError):
        runner.AnchoredShieldTrainer(model, anchor=None, anchor_coef=1., actor_lr=1e-3, critic_lr=1e-3)


def test_shield_with_the_comfort_ceiling_matches_the_v11_band(monkeypatch):
    torch, _, model, _, flow12 = _parent_and_migrated()
    from experiments import run_highway_flow_rl_v11 as v11runner
    from experiments import run_highway_flow_rl_v12 as runner
    from policies.mappo import MappoTrainer
    monkeypatch.setattr(MappoTrainer, '_loss_terms', lambda self, part, ratio, entropy: (
        self.model.actor.mean_net(part['local_obs']).sum() * 0., {}, {}))
    trainer = runner.AnchoredShieldTrainer(model, shield_coef=50., comfort_speed_ceiling=True,
                                           actor_lr=1e-3, critic_lr=1e-3)
    wide = torch.cat((_observations(torch)[:, :69], torch.zeros(32, 4), _observations(torch)[:, 69:]), 1)
    _, weighted, stats = trainer._loss_terms(dict(local_obs=wide), None, None)
    request = runner.mean_request(model.actor.mean_net(wide))
    expected = v11runner.shield_consistency(request, wide[:, 1] * 4.5, wide[:, 43] * 4.5, ceiling_margin=.05,
        rate_bound=.35, speed_margin=(wide[:, 4] - wide[:, 0]) * 26.8224).mean()
    assert torch.allclose(weighted['shield'], 50. * expected)


# --- contract --------------------------------------------------------------------
@pytest.fixture(scope='module')
def cfg():
    return json.loads((ROOT / 'configs/training/highway-flow-rl-v12-round18.json').read_text())


def test_committed_config_is_what_the_contract_builds(cfg):
    assert cfg == contract.build_config(18)
    assert contract.validate_config(deepcopy(cfg), require_training=True) == cfg


@pytest.mark.parametrize('change', [
    lambda c: c.update(v12_settings=dict(c['v12_settings'], band_weight=2.)),
    lambda c: c['optimizer'].update(actor_lr=1e-4),
    lambda c: c['v12_training'].update(anchor_coef=0.),
    lambda c: c['budget'].update(per_round_decisions=600000),
    lambda c: c['stop_rules'].update(any_collision=False),
    lambda c: c.update(held_out_authorized=True),
    lambda c: c['training_rows'][0].update(seed=1),
    lambda c: c['validation_rows'][0].update(seed=1),
    lambda c: c['training_rows'][0]['case'].update(network_sha256='0' * 64),
    lambda c: c.update(extra=1),
    lambda c: c['acceptance'].update(max_peak_jerk_m_s3=6.),
])
def test_contract_rejects_undeclared_changes(cfg, change):
    changed = deepcopy(cfg)
    change(changed)
    with pytest.raises(ValueError):
        contract.validate_config(changed)


def test_round_keeps_round16_gates_rewards_and_validation_cases(cfg):
    reference = contract.reference_config()
    for key in ('acceptance', 'target_settings', 'safety', 'action_contract', 'reward_weights', 'v06_reward_weights',
                'v06_settings', 'v10_reward_settings', 'v11_settings', 'v11_mask_version', 'mask_version', 'gae',
                'rate_limit', 'controlled_car_follow_model', 'road_end_leader_exit_rule', 'raw_request_scope',
                'held_out_rows', 'seed'):
        assert cfg[key] == reference[key], key
    assert cfg['validation_rows'][:8] == reference['validation_rows']
    assert 'v11_projection' not in cfg and cfg['warm_start'] == contract.WARM_START
    assert cfg['parent_config_sha256'] == contract.REFERENCE_CONFIG['sha256']


def test_la_corridors_are_the_primary_rows_with_distinct_seeds(cfg):
    la_train = [r for r in cfg['training_rows'] if r['case'].get('road_model') == 'highway_usd/la_10_110']
    la_val = [r for r in cfg['validation_rows'] if r['case'].get('road_model') == 'highway_usd/la_10_110']
    assert (len(la_train), len(cfg['training_rows']), len(la_val)) == (12, 20, 6)
    assert {r['case']['corridor'] for r in la_train} == {r['case']['corridor'] for r in la_val} == set(contract.LA_CORRIDORS)
    assert sum(r['case']['task'] == 'merge' for r in la_train) == 8
    reference = contract.reference_config()
    old = {r['seed'] for r in reference['training_rows'] + reference['validation_rows'] + reference['held_out_rows']}
    new = [r['seed'] for r in la_train + la_val]
    assert len(set(new)) == len(new) and not set(new) & old
    ids = [r['case']['id'] for r in cfg['training_rows']]
    assert len(set(ids)) == len(ids)
    for row in la_train:
        assert row['rollout_decisions'] == row['horizon_steps'] * row['controlled_count']
        assert row['controlled_count'] <= row['total_vehicles'] <= 24
        assert row['speed_limit_m_s'] == cfg['safety']['global_speed_ceiling_m_s']


def test_round_fits_the_remaining_study_budget(cfg):
    spent_decisions, spent_seconds = 5627084, 48250.8        # shared ledger before v12
    assert cfg['budget']['per_round_decisions'] <= cfg['budget']['total_decisions'] - spent_decisions
    assert cfg['budget']['total_decisions'] == 6230000 and cfg['budget']['total_wall_seconds'] == 61200
    assert cfg['budget']['per_round_wall_seconds'] <= cfg['budget']['total_wall_seconds'] - spent_seconds
    assert cfg['critic_warmup_decisions'] < cfg['evaluation_interval_decisions'] < cfg['budget']['per_round_decisions']


# --- runner wiring -----------------------------------------------------------------
def test_install_points_the_shared_loop_at_v12_and_keeps_the_canonical_ledger():
    pytest.importorskip('torch')
    from experiments import run_highway_flow_rl_v06 as v06runner
    from experiments import run_highway_flow_rl_v12 as runner
    from policies import highway_flow_v12 as flow12
    base = runner.base
    saved = {name: getattr(base, name) for name in ('OUTPUT_ROOT', 'CHECKPOINT_SCHEMA', 'NEW_SOURCES',
             'validate_config', 'validate_preflight', 'env_for', 'build_model', 'evaluate', 'episode', 'FlowModel',
             'critic_update', 'finish_preflight', 'MappoTrainer', 'STUDY', 'ruled_gates')}
    saved_episode = v06runner._episode
    try:
        runner.install()
        assert base.OUTPUT_ROOT == runner.ROOT / 'outputs/highway-flow-rl/v12'
        assert base.CHECKPOINT_SCHEMA == 'highway_flow_rl_checkpoint_v12'
        assert base.FlowModel is flow12.FlowModelV12 and base.critic_update is flow12.critic_update
        assert base.env_for is runner.env_for and base.evaluate is runner.evaluate
        assert base.finish_preflight is runner.finish_preflight and base.build_model is runner.build_model
        assert base.episode is v06runner.episode and v06runner._episode is runner.episode
        assert base.validate_config is contract.validate_config
        assert base.STUDY.as_posix().endswith('outputs/highway-flow-rl/v02')
        assert set(runner.SOURCES) <= set(base.NEW_SOURCES)
        assert all((runner.ROOT / name).is_file() for name in base.NEW_SOURCES)
    finally:
        for name, value in saved.items():
            setattr(base, name, value)
        v06runner._episode = saved_episode


def test_relative_merge_floor_stops_only_periodic_validations_that_collapse(tmp_path):
    pytest.importorskip('torch')
    from experiments import run_highway_flow_rl_v06 as v06runner
    from experiments import run_highway_flow_rl_v12 as runner
    cfg = dict(stop_rules=dict(relative_merge_floor=dict(drop=.25, minimum=.1)))
    reports = [dict(merge_target_ids=['a'], quality_merges=dict(a=dict(speed_match_fraction=.8)))]
    (tmp_path / 'initial-validation.json').write_text(json.dumps(dict(reports=reports)))
    result = dict(summary=dict(merge_speed_match=.56))
    runner.merge_floor(cfg, tmp_path, 'periodic-validation-00010', result)
    assert result['summary']['relative_merge_floor'] == pytest.approx(.55)
    with pytest.raises(v06runner.StopRuleTriggered):
        runner.merge_floor(cfg, tmp_path, 'periodic-validation-00020', dict(summary=dict(merge_speed_match=.5)))
    runner.merge_floor(cfg, tmp_path, 'final-validation', dict(summary=dict(merge_speed_match=.0)))
