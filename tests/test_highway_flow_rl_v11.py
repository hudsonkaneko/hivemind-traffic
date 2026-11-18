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
                       'v11_training', 'v11_projection',
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
                   lambda c: c.update(v11_training=dict(shield_coef=1.)),
                   lambda c: c.update(v11_projection=dict(iterations=9)),
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


# --- shield-consistency loss ------------------------------------------------------
def test_shield_consistency_is_zero_inside_the_filters_band_and_grows_outside():
    torch = pytest.importorskip('torch')
    from experiments.run_highway_flow_rl_v11 import shield_consistency

    def distance(request, measured, ceiling):
        return float(shield_consistency(torch.tensor(request), torch.tensor(measured), torch.tensor(ceiling),
                                        ceiling_margin=.05, rate_bound=.35))
    assert distance(.2, 0., 2.) == 0. and distance(-.3, 0., 2.) == 0.           # inside the rate band
    assert distance(.62, -.06, 2.) == pytest.approx((.62 - .29) ** 2)           # leader left: step of 0.68 m/s2
    assert distance(.118, 0., 0.) == pytest.approx((.118 + .05) ** 2)           # at the speed limit: ceiling 0
    assert distance(-.06, 0., 0.) == 0.                                         # just under the ceiling margin
    assert distance(-3., -.5, -3.) == pytest.approx(.05 ** 2, abs=1e-6)         # ceiling demands hard braking: follow it
    assert distance(-3.05, -.5, -3.) == pytest.approx(0., abs=1e-9)             # the rate band never forbids that
    assert distance(-1.5, 0., 2.) == pytest.approx((1.5 - .35) ** 2)            # unforced hard braking step


def test_trainer_adds_the_loss_only_when_the_round_declares_it():
    torch = pytest.importorskip('torch')
    from experiments import run_highway_flow_rl_v11 as runner
    from policies.highway_flow import FlowModel
    torch.manual_seed(0)
    model = FlowModel()
    observations = torch.zeros(6, 117)
    observations[:, 30:33] = torch.tensor([0., 1., 0.])
    observations[:, 43] = torch.tensor([2., 2., 0., 0., -3., 2.]) / 4.5
    observations[:, 1] = torch.tensor([0., 1., 0., 0., -.5, -2.]) / 4.5
    with torch.no_grad():
        actions, log_probs, entropy, latent = model.act(observations)
    part = dict(local_obs=observations, global_states=torch.zeros(6, 2832), agent_indices=torch.arange(6),
                actions=actions, pre_tanh_actions=latent, old_log_probs=log_probs, old_values=torch.zeros(6),
                advantages=torch.zeros(6), returns=torch.zeros(6))
    ratio = torch.ones(6)
    cfg = json.loads(config_path(15).read_text(encoding='utf-8'))
    plain = runner.trainer_for(dict())(model, **cfg['optimizer'])
    loss0, weighted0, stats0 = plain._loss_terms(part, ratio, entropy)
    assert 'shield' not in weighted0 and 'shield_loss_m2_s4' not in stats0
    trainer = runner.trainer_for(cfg)(model, **cfg['optimizer'])
    assert (trainer.shield_coef, trainer.ceiling_margin, trainer.rate_bound) == (50., .05, .35)
    assert trainer.episodes_per_update == cfg['optimizer']['episodes_per_update']
    loss, weighted, stats = trainer._loss_terms(part, ratio, entropy)
    # a fresh actor requests about -1.25 m/s2 (the middle of [-4.5, 2]): outside the band on most rows
    assert stats['shield_outside_fraction'] > 0
    assert float(weighted['shield'].detach()) == pytest.approx(50. * stats['shield_loss_m2_s4'], rel=1e-5)
    assert float(loss.detach()) == pytest.approx(float(loss0.detach()) + float(weighted['shield'].detach()), rel=1e-5)
    weighted['shield'].backward()
    assert sum(float(p.grad.abs().sum()) for p in model.actor.parameters() if p.grad is not None) > 0
    assert all(p.grad is None or float(p.grad.abs().sum()) == 0. for p in model.critic.parameters())
    with pytest.raises(ValueError):
        runner.ShieldConsistentTrainer(model, shield_coef=-1., **cfg['optimizer'])


# --- shield projection (round 16) ---------------------------------------------------
def test_projection_fit_moves_requests_into_the_band_and_keeps_the_rest():
    torch = pytest.importorskip('torch')
    from experiments import run_highway_flow_rl_v11_projection as projection
    from policies.highway_flow import FlowModel
    torch.manual_seed(1)
    model, reference = FlowModel(), FlowModel()
    reference.load_state_dict(model.state_dict())
    with torch.no_grad():   # give the actor a clearly positive request so some rows start outside the band
        model.actor.mean_net[1][-1].bias[0] = reference.actor.mean_net[1][-1].bias[0] = 1.
    settings = dict(json.loads(config_path(16).read_text(encoding='utf-8'))['v11_projection'], epochs=60, batch_size=64)
    observations = torch.zeros(256, 117)
    observations[:, 30:33] = torch.tensor([0., 1., 0.])
    observations[:, 43] = 2. / 4.5
    observations[:128, 1] = 1.1 / 4.5         # measured 1.1 m/s2: a request near 0.98 is inside the band
    observations[128:, 1] = 0.                # just inserted: the same request is 0.63 above measured + 0.35
    critic = {k: v.clone() for k, v in model.state_dict().items() if not k.startswith('actor.')}
    stats = projection.fit(model, reference, observations, settings, torch.Generator().manual_seed(0))
    assert stats['before']['outside_fraction'] == pytest.approx(.5) and stats['before']['preserve_m2_s4'] == 0.
    assert stats['after']['shield_m2_s4'] < .01 * stats['before']['shield_m2_s4']
    request, _ = projection.actor_outputs(model, observations)
    kept, _ = projection.actor_outputs(reference, observations)
    assert float((request[:128] - kept[:128]).abs().max().detach()) < .1      # rows inside the band stay with the reference
    assert float(request[128:].max()) < .35 + .1                     # rows outside moved to the band edge
    assert all(torch.equal(v, critic[k]) for k, v in model.state_dict().items() if not k.startswith('actor.'))


def test_projection_round_is_declared_without_ppo_settings():
    cfg = json.loads(config_path(16).read_text(encoding='utf-8'))
    assert 'v11_training' not in cfg and cfg['v11_projection']['iterations'] == 2
    rows = cfg['training_rows']
    assert cfg['v11_projection']['episodes_per_iteration'] == len(rows)
    assert cfg['budget']['per_round_decisions'] == 2 * sum(r['horizon_steps'] * r['controlled_count'] for r in rows)


# --- guided projection (round 17) ----------------------------------------------------
def test_comfort_speed_ceiling_lets_a_car_ramp_down_before_the_speed_limit():
    torch = pytest.importorskip('torch')
    from experiments.run_highway_flow_rl_v11 import comfort_speed_ceiling, shield_band, shield_consistency
    ceiling = lambda margin: float(comfort_speed_ceiling(torch.tensor(margin), .35))  # noqa: E731
    assert ceiling(0.) == 0. and ceiling(-1.) == 0.
    assert ceiling(.06) == pytest.approx(.473, abs=.002)       # the measured case: 0.6 m/s2 was too much there
    assert ceiling(.15) == pytest.approx(.850, abs=.002)
    for margin in (.02, .06, .15, .5):                         # ramping down at 0.35 per decision stays under the limit
        a, gained = ceiling(margin), 0.
        while a > 0:
            gained += a * .1
            a -= .35
        assert gained <= margin + 1e-6
    lower, upper = shield_band(torch.tensor(.6), torch.tensor(.6), ceiling_margin=.05, rate_bound=.35,
                               speed_margin=torch.tensor(.06))
    assert float(upper) == pytest.approx(.473, abs=.002) and float(lower) == pytest.approx(.25)
    plain = shield_consistency(torch.tensor(.55), torch.tensor(.6), torch.tensor(.64), ceiling_margin=.05, rate_bound=.35)
    ahead = shield_consistency(torch.tensor(.55), torch.tensor(.6), torch.tensor(.64), ceiling_margin=.05, rate_bound=.35,
                               speed_margin=torch.tensor(.064))
    assert float(plain) == 0. and float(ahead) > 0.           # round 15/16 band unchanged without the margin


def _guided_env(speed=15., maneuver=False, status='pending', merged=None, eligible=True, target=16.88):
    from types import SimpleNamespace
    tracker = SimpleNamespace(status=status, merge_complete_time_s=merged)
    return SimpleNamespace(merge_trackers=dict(agent_0=tracker), states=dict(agent_0=dict(speed=speed)),
        flow_contexts=dict(agent_0={}), graph=None, maneuvers=dict(agent_0={}) if maneuver else {},
        agents=['agent_0']), dict(eligible=eligible, target_m_s=target)


def test_guidance_label_tracks_the_gate_target_only_while_the_merge_is_pending(monkeypatch):
    torch = pytest.importorskip('torch')
    from experiments import run_highway_flow_rl_v11_projection as projection
    settings = json.loads(config_path(17).read_text(encoding='utf-8'))['v11_projection']
    observation = torch.zeros(117)
    observation[43] = 2. / 4.5
    observation[0], observation[4] = 15. / projection.CAP, 18. / projection.CAP

    def label(**kwargs):
        env, target = _guided_env(**kwargs)
        monkeypatch.setattr(projection, 'merge_speed_target', lambda state, context, preview: (target, {}))
        monkeypatch.setattr(projection, 'map_evidence', lambda graph, state: None)
        return projection.guidance_label(env, 'agent_0', observation, settings)
    assert label() == pytest.approx(.35)                      # 0.6 * (16.88 - 0.7 - 15) = 0.71, held to measured + 0.35
    observation[1] = .6 / 4.5
    assert label() == pytest.approx(.6 * (16.88 - .7 - 15.))  # waiting: aims 0.7 m/s under the target
    assert label(maneuver=True) == pytest.approx(.95)         # lane change started: aims at the target (rate-bounded)
    assert label(speed=17.5) == pytest.approx(.6 - .35)       # above the waiting goal: eases off at the rate bound
    assert label(status='succeeded') is None and label(merged=19.5) is None and label(eligible=False) is None


def test_labeling_actor_records_labels_and_drives_them_only_in_the_expert_pass(monkeypatch):
    torch = pytest.importorskip('torch')
    from experiments import run_highway_flow_rl_v11_projection as projection
    from policies.highway_flow import FlowModel
    settings = json.loads(config_path(17).read_text(encoding='utf-8'))['v11_projection']
    monkeypatch.setattr(projection, 'guidance_label', lambda env, agent, observation, s: .5 if agent == 'agent_0' else None)
    torch.manual_seed(0)
    model = FlowModel()
    observations = torch.zeros(24, 117)
    observations[:, 30:33] = torch.tensor([0., 1., 0.])
    env, _ = _guided_env()
    with torch.no_grad():
        plain = model.act(observations, deterministic=True)[0]
        learner = projection.LabelingActor(model, settings, expert=False)
        learner.env = env
        kept = learner.act(observations, deterministic=True)[0]
        expert = projection.LabelingActor(model, settings, expert=True)
        expert.env = env
        driven = expert.act(observations, deterministic=True)[0]
    assert torch.equal(kept, plain)
    assert float(driven[0, 0]) == pytest.approx((.5 + 4.5) / 3.25 - 1.) and torch.equal(driven[1:], plain[1:])
    for actor in (learner, expert):
        assert len(actor.labels) == 1 and float(actor.labels[0][0]) == .5 and bool(torch.isnan(actor.labels[0][1:]).all())


def test_guided_fit_follows_labels_where_given_and_the_reference_elsewhere():
    torch = pytest.importorskip('torch')
    from experiments import run_highway_flow_rl_v11_projection as projection
    from policies.highway_flow import FlowModel
    torch.manual_seed(2)
    model, reference = FlowModel(), FlowModel()
    reference.load_state_dict(model.state_dict())
    settings = dict(json.loads(config_path(17).read_text(encoding='utf-8'))['v11_projection'], epochs=80, batch_size=64)
    observations = torch.zeros(256, 117)
    observations[:, 30:33] = torch.tensor([0., 1., 0.])
    observations[:, 43] = 2. / 4.5
    observations[:, 0], observations[:, 4] = 10. / projection.CAP, 18. / projection.CAP
    observations[:, 1] = -1.25 / 4.5                           # a fresh actor requests about -1.25 m/s2: inside the band
    observations[128:, 28] = 1.                                # the guided rows differ in one input
    labels = torch.full((256,), float('nan'))
    labels[128:] = -1.0
    stats = projection.fit(model, reference, observations, settings, torch.Generator().manual_seed(0), labels)
    assert stats['before']['guided_samples'] == 128 and stats['after']['guidance_rmse_m_s2'] < .05
    request, _ = projection.actor_outputs(model, observations)
    kept, _ = projection.actor_outputs(reference, observations)
    assert float((request[:128] - kept[:128]).abs().max()) < .05 and float((request[128:] + 1.).abs().max()) < .1


def test_round17_collection_adds_i110_merge_rows_with_training_seeds_only():
    from experiments.run_highway_flow_rl_v11_projection import collection_rows
    cfg = json.loads(config_path(17).read_text(encoding='utf-8'))
    settings = cfg['v11_projection']
    forbidden = {row['seed'] for row in cfg['validation_rows'] + cfg['held_out_rows']}
    seen, decisions = set(), 0
    for iteration in range(settings['iterations']):
        rows = collection_rows(cfg, iteration, settings)
        assert len(rows) == settings['episodes_per_iteration'] == 30
        assert [row['case']['id'] for row in rows[:20]] == [row['case']['id'] for row in cfg['training_rows']]
        assert sum(row['case']['id'] == 'i110-merge' for row in rows) == 6
        assert sum(row['case']['id'] == 'i110-merge-focused-8' for row in rows) == 6
        for row in rows:
            key = (row['case']['id'], row['seed'])
            assert key not in seen and row['seed'] not in forbidden and not (row.get('missed_gap') and row in rows[20:])
            seen.add(key)
            decisions += row['horizon_steps'] * row['controlled_count']
    assert decisions == cfg['budget']['per_round_decisions']
    warm = cfg['warm_start']
    assert warm['path'].endswith('round-16/candidates/checkpoint-02.pt') and len(warm['sha256']) == 64
