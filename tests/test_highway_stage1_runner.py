from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import pytest
import torch

from experiments import run_highway_stage1 as runner


def config():
    return runner.load_config(runner.DEFAULT)


def write_config(tmp_path, cfg):
    path = tmp_path / 'config.json'; path.write_text(json.dumps(cfg)); return path


def test_config_disabled_and_total_allocation_explicit():
    cfg = config()
    assert not cfg['training_authorized']
    assert cfg['behavior_cloning']['max_agent_transitions'] == 50000
    assert sum(p['agent_transitions'] for p in cfg['phases']) == 450000
    assert cfg['phases'][-2]['merge_density'] == 'light'
    assert cfg['phases'][-1]['merge_density'] == 'dense'


@pytest.mark.parametrize('change', ['overspend', 'overlap', 'legacy', 'phase_sum'])
def test_bad_configuration_rejected(tmp_path, change):
    cfg = config()
    if change == 'overspend': cfg['training']['max_wall_seconds'] = 7201
    elif change == 'overlap': cfg['behavior_cloning']['seed_start'] = cfg['training']['seed_start']
    elif change == 'legacy': cfg['model']['observation_size'] = 81
    else: cfg['phases'][-1]['agent_transitions'] += 1
    with pytest.raises(ValueError): runner.load_config(write_config(tmp_path, cfg))


def test_training_stops_before_simulation_without_authorization(monkeypatch):
    monkeypatch.chdir(runner.ROOT)
    with pytest.raises(ValueError, match='explicit authorization'):
        runner.main(['--mode', 'train'])


def test_failed_episode_reservation_not_refunded():
    cfg = config(); state = runner.new_state(cfg)
    seed, reserved = runner.reserve_episode(state, cfg, 0)
    assert state['ppo_spent'] == reserved == 3200
    assert state['next_seed'] == seed + 1
    assert state['ppo_actual'] == 0
    # An interrupted episode remains charged; a later reservation cannot reuse it.
    runner.reserve_episode(state, cfg, 0)
    assert state['ppo_spent'] == 6400


def test_settlement_tracks_actual_and_refunds_only_completed_unused_capacity():
    cfg = config(); state = runner.new_state(cfg)
    runner.reserve_episode(state, cfg, 0)
    runner.settle_episode(state, 3198)
    assert state['ppo_spent'] == state['ppo_actual'] == state['phase_spent'][0] == 3198
    assert state['pending'] is None


def test_partial_joint_step_does_not_start_another_phase_episode():
    cfg = config()
    assert runner.phase_index(cfg, [99998, 100000, 125000, 125000]) is None


def test_curriculum_retains_earlier_tasks_and_never_selects_test():
    cfg = config()
    for p in cfg['phases']:
        tasks = {runner.case_for(cfg, 6100000 + i, phase=p, episode_index=i)['task'] for i in range(len(p['task_cycle']))}
        assert {'cruise', 'following'} <= tasks
    with pytest.raises(ValueError, match='cannot select evaluation'):
        runner.case_for(cfg, 5310000, 'test', cfg['phases'][0])


def report(task='cruise', **changes):
    result = dict(case_id='case', seed=1, task=task, collisions=0, teleports=0, overspeed_steps=0,
        intervention_fraction=0., mean_fraction_of_speed_limit=.95, passes=2, successful_merges=2)
    result.update(changes); return result


def test_nominal_quality_checks_each_case_not_aggregate():
    cfg = config()
    result = runner.quality([report(), report('following', intervention_fraction=.2)], cfg)
    assert not result['passed'] and result['failures'][0]['causes'] == ['overrides']
    assert not runner.quality([report('merge', successful_merges=1)], cfg)['passed']


def test_readiness_rejects_teacher_only_evidence(tmp_path, monkeypatch):
    monkeypatch.setattr(runner, 'ROOT', tmp_path)
    out = tmp_path / 'outputs'; out.mkdir()
    summary = out / 'summary.json'
    summary.write_text(json.dumps(dict(passed=True, teacher_ready=True, optimizer_steps=0)))
    cfg = config()
    cfg['readiness'].update(preflight_passed=True, teacher_ready=True,
        summary_path='outputs/summary.json', summary_sha256=runner.digest(summary))
    with pytest.raises(ValueError, match='complete no-learning preflight'):
        runner.readiness_check(cfg)


def test_preparation_fingerprint_changes_with_reward_and_geometry():
    cfg = config(); changed = deepcopy(cfg)
    changed['reward_weights']['progress'] += .1
    assert runner.preparation_digest(cfg) != runner.preparation_digest(changed)


@pytest.mark.parametrize('change', ['seed', 'phase_density', 'preflight_density', 'geometry'])
def test_preparation_fingerprint_binds_exact_preflight_scope(change):
    cfg = config(); changed = deepcopy(cfg)
    if change == 'seed': changed['evaluation']['preflight_seeds'][0] += 10000
    elif change == 'phase_density': changed['phases'][-1]['merge_density'] = 'light'
    elif change == 'preflight_density': changed['preflight']['merge_density_coverage'] = ['light']
    else: changed['geometry']['bank_manifest_sha256'] = 'changed'
    assert runner.preparation_digest(cfg) != runner.preparation_digest(changed)


def test_fully_accepted_actor_outranks_more_passes_with_failed_overrides():
    cfg = config()
    accepted = report('passing', accounting={'totals': {'unfinished': 40}}, overrides=1, agent_steps=100, reward=1.)
    shield_dependent = dict(accepted, passes=30, intervention_fraction=.06, overrides=6, reward=100.)
    assert runner.selection_rank([accepted], cfg) > runner.selection_rank([shield_dependent], cfg)


def test_unaccepted_moving_actor_still_outranks_parked_actor():
    cfg = config()
    moving = report(intervention_fraction=.2, accounting={'totals': {'unfinished': 40}},
                    overrides=20, agent_steps=100, reward=1.)
    parked = dict(moving, mean_fraction_of_speed_limit=.07, intervention_fraction=0., overrides=0)
    assert runner.selection_rank([moving], cfg) > runner.selection_rank([parked], cfg)


class FakeEnv:
    case_id = 'synthetic-no-simulator'
    directory = runner.ROOT / 'outputs' / 'synthetic-no-simulator'
    control_mode = 'policy'

    def observation(self, speed):
        x = np.zeros(93, np.float32); x[0] = speed; x[4] = 1.; x[31] = 1.; x[43] = 2 / 4.5
        return x

    def reset(self, seed):
        self.agents = list(runner.AGENTS); self.closed = False
        self.time = self.start_time = 0.; self.horizon = 1
        self.obs = {a: self.observation(.2) for a in self.agents}
        return self.obs, {}

    def state(self):
        return np.concatenate([np.r_[self.obs[a], 1.] if a in self.agents else np.zeros(94) for a in runner.AGENTS]).astype(np.float32)

    def step(self, actions):
        current = self.agents[:]
        final = {a: self.observation(.4) for a in current}
        self.time = .1
        self.agents = []
        return final, dict.fromkeys(current, 1.), dict.fromkeys(current, False), dict.fromkeys(current, True), {}

    def report(self): return report()
    def close(self): self.closed = True


class ProbeModel:
    def act(self, local, deterministic=False):
        actions = torch.zeros(4, 2)
        return actions, torch.zeros(4), torch.zeros(4), actions.clone()
    def values(self, state, indices): return state[:, 0]


def test_collector_bootstraps_real_truncation_state_and_closes():
    env = FakeEnv()
    result, batch, demos = runner.episode(env, 1, kind='learned', model=ProbeModel(), collect=True)
    assert env.closed and result['agent_transitions'] == 4 and demos is None
    torch.testing.assert_close(batch['next_values'], torch.full((1, 4), .4))
    assert batch['local_obs'].shape == (1, 4, 93)
    assert batch['global_states'].shape == (1, 4, 376)
    assert batch['teacher_actions'].shape == (1, 4, 2)


def test_demonstrations_are_separate_from_ppo_samples():
    env = FakeEnv()
    result, batch, demos = runner.episode(env, 1, demonstrations=True)
    assert batch is None and demos['observations'].shape == (4, 93)
    with pytest.raises(ValueError, match='separate'):
        runner.episode(env, 1, kind='learned', model=ProbeModel(), collect=True, demonstrations=True)


def test_primary_accounting_finishes_only_declared_horizon():
    class AccountingEnv:
        time = 0.; start_time = 0.; horizon = 3
        def report(self):
            return dict(collisions=0, teleports=0, accounting={'requested': 42, 'observed_time': self.time})
        def _advance(self): self.time += .1
    env = AccountingEnv()
    result = runner.primary_accounting(env, partial=False)
    assert result['primary_horizon_complete'] and result['accounting_comparison_valid']
    assert result['post_control_native_steps'] == 3
    assert result['accounting']['requested'] == 42
    assert env.time == pytest.approx(.3)


def test_partial_accounting_keeps_requested_cohort_but_is_not_a_fair_horizon():
    class AccountingEnv:
        time = .1; start_time = 0.; horizon = 800
        def report(self): return dict(collisions=0, teleports=0, accounting={'requested': 42})
        def _advance(self): raise AssertionError('Partial collector must not advance the simulator')
    result = runner.primary_accounting(AccountingEnv(), partial=True)
    assert not result['primary_horizon_complete'] and not result['accounting_comparison_valid']
    assert result['accounting']['requested'] == 42


def test_primary_accounting_respects_expired_deadline():
    class AccountingEnv:
        time = .1; start_time = 0.; horizon = 800
        def report(self): return dict(collisions=0, teleports=0, accounting={'requested': 42})
        def _advance(self): raise AssertionError('Expired deadline must precede advancing')
    with pytest.raises(runner.PilotBudgetExhausted):
        runner.primary_accounting(AccountingEnv(), partial=False, deadline=0.)


def test_full_time_does_not_override_invalid_trip_ledger():
    class AccountingEnv:
        time = 80.; start_time = 0.; horizon = 800
        def report(self):
            return dict(collisions=0, teleports=0, accounting={'requested': 42, 'comparison_valid': False})
    result = runner.primary_accounting(AccountingEnv(), partial=False)
    assert result['primary_horizon_complete'] and not result['accounting_comparison_valid']


def synthetic_training_pipeline(monkeypatch):
    """Tiny synthetic data only: no SUMO, teacher capture or driving checkpoint."""
    cfg = config()
    cfg['behavior_cloning'].update(max_agent_transitions=4, epochs=1, minibatch_size=4)
    cfg['training']['max_agent_transitions'] = 20
    for phase in cfg['phases']: phase['agent_transitions'] = 4
    cfg['optimizer'].update(epochs=1, minibatch_size=4)
    monkeypatch.setattr(runner, 'model_for', lambda c: runner.MappoActorCritic(**c['model'],
        message_encoder=True, hybrid_actions=True, actor_hidden_sizes=(8,), critic_hidden_sizes=(8,)))
    monkeypatch.setattr(runner, 'env_for', lambda *a, **k: None)
    def fake_episode(env, seed, model=None, collect=False, demonstrations=False, **kwargs):
        obs = torch.zeros(4, 93); obs[:, 31] = 1.; obs[:, 43] = 2 / 4.5
        r = report(seed=seed, agent_transitions=4, partial_control_horizon=False,
            accounting={'totals': {'unfinished': 0}}, overrides=0, agent_steps=4, reward=1.)
        if demonstrations: return r, None, dict(observations=obs, actions=torch.zeros(4, 2))
        action, logp, _, latent = model.act(obs)
        state = torch.zeros(4, 376); indices = torch.arange(4)
        batch = dict(local_obs=obs, global_states=state, agent_indices=indices, actions=action,
            pre_tanh_actions=latent, old_log_probs=logp, old_values=model.values(state, indices),
            advantages=torch.ones(4), returns=torch.ones(4), active=torch.ones(4, dtype=torch.bool),
            teacher_actions=torch.zeros(4, 2))
        return r, batch, None
    monkeypatch.setattr(runner, 'episode', fake_episode)
    monkeypatch.setattr(runner, 'evaluate', lambda *a, **k: [report(
        accounting={'totals': {'unfinished': 0}}, overrides=0, agent_steps=4, reward=1.)])
    return cfg


def test_tiny_synthetic_pipeline_counts_demos_and_all_phases(tmp_path, monkeypatch):
    cfg = synthetic_training_pipeline(monkeypatch)
    result = runner.train(cfg, tmp_path, runner.config_digest(cfg), {}, runner.time.monotonic())
    assert result['passed'], result
    assert result['actual_agent_transitions'] == result['charged_agent_transitions'] == 20
    assert result['demo_actual'] == 4 and result['ppo_actual'] == 16
    saved = torch.load(tmp_path / 'last.pt', weights_only=False)
    assert saved['state']['evaluated_boundaries'] == [0, 1, 2, 3]
    assert saved['state']['bc_complete'] and saved['state']['run_closed']
    assert (tmp_path / saved['demonstrations_file']).is_file()


def test_synthetic_interrupted_update_resumes_fresh_batch_without_double_counting(tmp_path, monkeypatch):
    cfg = synthetic_training_pipeline(monkeypatch)
    original = runner.MappoTrainer.update
    failed = False
    def fail_once(self, batch, **kwargs):
        nonlocal failed
        if not failed:
            failed = True
            raise RuntimeError('Synthetic optimizer interruption')
        return original(self, batch, **kwargs)
    monkeypatch.setattr(runner.MappoTrainer, 'update', fail_once)
    first = tmp_path / 'first'; first.mkdir()
    result = runner.train(cfg, first, runner.config_digest(cfg), {}, runner.time.monotonic())
    assert not result['passed'] and 'Synthetic optimizer interruption' in result['error']
    before = torch.load(first / 'last.pt', weights_only=False)
    assert before['state']['pending']['kind'] == 'ppo_update'
    assert before['state']['ppo_actual'] == 4
    second = tmp_path / 'resume'; second.mkdir()
    resumed = runner.train(cfg, second, runner.config_digest(cfg), {}, runner.time.monotonic(), first / 'last.pt')
    assert resumed['passed'], resumed
    assert resumed['actual_agent_transitions'] == resumed['charged_agent_transitions'] == 20
    assert resumed['bc_optimizer_steps'] == before['state']['bc_optimizer_steps']
    assert resumed['updates'] == 4


def test_selected_export_failure_preserves_resumable_atomic_generation(tmp_path, monkeypatch):
    cfg = synthetic_training_pipeline(monkeypatch)
    original = runner.export_actor
    failed = False
    def fail_once(model, path):
        nonlocal failed
        if not failed:
            failed = True
            raise OSError('Synthetic export interruption')
        return original(model, path)
    monkeypatch.setattr(runner, 'export_actor', fail_once)
    first = tmp_path / 'first'; first.mkdir()
    result = runner.train(cfg, first, runner.config_digest(cfg), {}, runner.time.monotonic())
    assert not result['passed'] and 'Synthetic export interruption' in result['error']
    stable = torch.load(first / 'last.pt', weights_only=False)
    assert stable['state']['selected_generation'] is None
    assert stable['state']['bc_complete'] and not stable['state']['after_bc_evaluated']
    second = tmp_path / 'resumed'; second.mkdir()
    resumed = runner.train(cfg, second, runner.config_digest(cfg), {}, runner.time.monotonic(), first / 'last.pt')
    assert resumed['passed'], resumed
    selected = resumed['selected_generation']
    assert (second / selected / 'best.pt').is_file()
    assert (second / selected / 'policy.pt').is_file()
    assert json.loads((second / 'selection.json').read_text())['generation'] == selected
    assert resumed['actual_agent_transitions'] == 20


def geometry_fixture(tmp_path, monkeypatch, *, passed=True):
    monkeypatch.setattr(runner, 'ROOT', tmp_path)
    output = tmp_path / 'outputs'; output.mkdir()
    layouts = []
    for i in range(10):
        network = output / f'network-{i}.xml'; network.write_text(f'fixture {i}')
        layouts.append(dict(id=f'layout-{i}', network=network.relative_to(tmp_path).as_posix(),
            network_sha256=runner.digest(network), through_lanes=2,
            split='train' if i < 4 else 'validation' if i < 7 else 'test', speed_limit_mph=55))
    bank = output / 'bank.json'
    bank.write_text(json.dumps(dict(geometry_version=runner.GEOMETRY_VERSION, layouts=layouts)))
    preserved = output / 'i110.xml'; preserved.write_text('preserved original fixture')
    original = dict(network='outputs/i110.xml', network_sha256=runner.digest(preserved), geometry_version='i110_provisional_v1')
    cfg = dict(geometry=dict(version=runner.GEOMETRY_VERSION, bank_manifest='outputs/bank.json',
        bank_manifest_sha256=runner.digest(bank), preserved_i110=original),
        curriculum=layouts + [dict(id='i110-cruise', split='train', through_lanes=4, speed_limit_mph=60, **original)])
    calls = []
    def validate(path, through_lanes):
        calls.append(str(path))
        return dict(passed=passed, connector_crossings=[] if passed else ['crossing'])
    monkeypatch.setitem(sys.modules, 'scripts.build_highway_curriculum_v2',
        SimpleNamespace(GEOMETRY_VERSION=runner.GEOMETRY_VERSION, validate_geometry=validate))
    return cfg, bank, calls


def test_geometry_bank_verifies_all_layouts_without_running_held_out_traffic(tmp_path, monkeypatch):
    cfg, bank, calls = geometry_fixture(tmp_path, monkeypatch)
    result = runner.geometry_check(cfg)
    assert result['passed'] and len(calls) == 10
    assert 'no held-out traffic' in result['held_out_scope']


def test_geometry_bank_rejects_changed_manifest(tmp_path, monkeypatch):
    cfg, bank, _ = geometry_fixture(tmp_path, monkeypatch)
    bank.write_text(bank.read_text() + '\n')
    with pytest.raises(ValueError, match='manifest path/hash'):
        runner.geometry_check(cfg)


def test_geometry_bank_rejects_crossing_connector(tmp_path, monkeypatch):
    cfg, _, _ = geometry_fixture(tmp_path, monkeypatch, passed=False)
    with pytest.raises(ValueError, match='structural checks failed'):
        runner.geometry_check(cfg)


def test_geometry_bank_rejects_curriculum_fixture_mismatch(tmp_path, monkeypatch):
    cfg, _, _ = geometry_fixture(tmp_path, monkeypatch)
    cfg['curriculum'][0] = dict(cfg['curriculum'][0], through_lanes=3)
    with pytest.raises(ValueError, match='does not match'):
        runner.geometry_check(cfg)
