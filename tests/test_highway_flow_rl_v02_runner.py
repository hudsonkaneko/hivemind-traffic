"""CPU-only ledger, gating, checkpoint and curriculum regression tests; no SUMO."""
from copy import deepcopy
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import torch

from experiments import run_highway_flow_rl_v02 as runner
from traffic.highway_flow_rl_v02_contract import PREFLIGHT_CHECKS


@pytest.fixture(autouse=True)
def no_learning(monkeypatch):
    monkeypatch.setattr(torch.optim.Adam, 'step', Mock(side_effect=AssertionError('Unit tests cannot train')))
    torch.set_num_threads(1)


@pytest.fixture
def cfg():
    return json.loads((runner.ROOT / 'configs/training/highway-flow-rl-v02.json').read_text())


@pytest.fixture
def sandbox(monkeypatch):
    outputs = runner.ROOT / 'outputs'
    outputs.mkdir(exist_ok=True)
    with TemporaryDirectory(prefix='v02-runner-tests-', dir=outputs) as directory:
        root = Path(directory)
        monkeypatch.setattr(runner, 'ROOT', root)
        monkeypatch.setattr(runner, 'STUDY', root / 'outputs/highway-flow-rl/v02')
        yield root


def segment(category='training', seconds=100., decisions=123, **extra):
    return dict(category=category, command='train' if category == 'training' else 'evaluate',
        round_index=1, output='outputs/example', status='complete', charged_seconds=seconds,
        decisions=decisions, episodes=1, **extra)


def ledger(*segments):
    return dict(schema=runner.LEDGER_SCHEMA, inherited_seconds=0., segments=list(segments))


def test_imported_network_screen_and_failed_round_are_both_charged(cfg):
    screen = dict(command='network_screen', round_index=0, status='complete', charged_seconds=120., decisions=0)
    failed = segment(seconds=250.)
    failed['status'] = 'failed'
    totals = runner.ledger_totals(ledger(screen, failed), 1)
    assert totals['total'] == 370.
    assert totals['preparation'] == 120.
    assert totals['round_seconds'] == 250.
    assert totals['round_decisions'] == 123
    assert runner.available_seconds(cfg, totals, 'training') == 650.
    assert runner.available_seconds(cfg, totals, 'preparation') == 180.
    assert runner.available_seconds(cfg, totals, 'final_evaluation') == 600.


@pytest.mark.parametrize('change', [dict(status='running'), dict(charged_seconds=-1),
    dict(charged_seconds=math.nan), dict(decisions=-1), dict(decisions=True), dict(category='ignored')])
def test_invalid_or_unclosed_history_fails_closed(change):
    row = segment()
    row.update(change)
    with pytest.raises(ValueError):
        runner.ledger_totals(ledger(row), 1)


def test_phase_and_total_budgets_cannot_reset_between_rounds(cfg):
    totals = runner.ledger_totals(ledger(segment(seconds=900.),
        dict(segment(seconds=800.), round_index=2), segment('preparation', 180., decisions=0),
        segment('final_evaluation', 500., decisions=0)), 2)
    assert runner.available_seconds(cfg, totals, 'training') == 100.
    assert runner.available_seconds(cfg, totals, 'preparation') == 0.
    assert runner.available_seconds(cfg, totals, 'final_evaluation') == 100.


def test_session_releases_lock_and_charges_setup_failures(cfg, sandbox, monkeypatch):
    clock = SimpleNamespace(now=10.)
    monkeypatch.setattr(runner.time, 'monotonic', lambda: clock.now)
    output = runner.STUDY / 'failed-setup'
    with pytest.raises(RuntimeError, match='setup failed'):
        with runner.LedgerSession(cfg, 'preflight', output, 'config', started=10., episodes=1):
            clock.now = 13.5
            raise RuntimeError('setup failed')
    assert not (runner.STUDY / 'active.lock').exists()
    saved = json.loads((runner.STUDY / 'cost-ledger.json').read_text())
    assert saved['segments'][0]['status'] == 'failed'
    assert saved['segments'][0]['charged_seconds'] == 3.5
    assert saved['segments'][0]['episodes'] == 1


def test_diagnostics_cannot_exceed_eight_episodes(cfg, sandbox):
    runner.save(runner.STUDY / 'cost-ledger.json', ledger(dict(segment('preparation', 20., 0), episodes=8)))
    with pytest.raises(ValueError, match='eight'):
        with runner.LedgerSession(cfg, 'preflight', runner.STUDY / 'ninth', 'config',
                                  started=runner.time.monotonic(), episodes=1):
            pytest.fail('Ninth diagnostic was admitted')
    assert not (runner.STUDY / 'active.lock').exists()


def test_disabled_training_rejected_at_public_train_entry(cfg):
    cfg['training_authorized'] = False
    with pytest.raises(ValueError, match='disabled'):
        runner.train(cfg, None, None, None, None, None, 'config', {})


def test_exact_resume_rejects_source_drift_before_constructing_model(cfg, sandbox, monkeypatch):
    saved = dict(schema=runner.CHECKPOINT_SCHEMA, config_sha256='same', source_hashes={'runner': 'old'},
        model={}, optimizer={}, rng={}, state={})
    monkeypatch.setattr(runner.torch, 'load', lambda *args, **kwargs: saved)
    constructor = Mock(side_effect=AssertionError('Do not construct before provenance check'))
    monkeypatch.setattr(runner, 'FlowModel', constructor)
    with pytest.raises(ValueError, match='Exact resume'):
        runner.build_model(cfg, checkpoint='outputs/last.pt', resume=True, fingerprint='same', sources={'runner': 'new'})
    constructor.assert_not_called()


def test_missing_full_checkpoint_cannot_fall_back_to_old_initializer(cfg, sandbox, monkeypatch):
    monkeypatch.setattr(runner.torch, 'load', lambda *args, **kwargs: dict(schema='highway_flow_rl_checkpoint_v01', model={}))
    monkeypatch.setattr(runner.flow, 'initialize_candidate', Mock(side_effect=AssertionError('No historical migration')))
    with pytest.raises(ValueError, match='Incomplete'):
        runner.build_model(cfg)


def test_interrupted_rollout_is_explicitly_discarded_without_refunding_decisions():
    state = runner.new_state()
    state.update(decisions=2048, pending=dict(kind='episode', reserved=2048, critic=False, output='outputs/rollout'))
    runner.recover_pending(state)
    assert state['decisions'] == 2048
    assert state['ppo_episodes'] == 1
    assert state['pending'] is None
    assert state['recoveries'][0]['kind'] == 'discarded_interrupted_rollout'


def test_interrupted_update_preserves_on_policy_batch_for_rollback_resume():
    batch = {'local_obs': torch.ones(2, 117)}
    state = runner.new_state()
    state['pending'] = dict(kind='updating', batch=batch)
    runner.recover_pending(state)
    assert state['pending']['kind'] == 'update_ready'
    assert state['pending']['batch'] == batch
    assert state['recoveries'][0]['kind'] == 'rolled_back_uncommitted_update'


class FakeOptimizer:
    def __init__(self):
        self.value = 0
    def state_dict(self):
        return {'value': self.value}
    def load_state_dict(self, values):
        self.value = values['value']


def test_failed_partial_update_restores_weights_optimizer_rng_and_retains_batch(sandbox):
    model = torch.nn.Linear(1, 1)
    original = deepcopy(model.state_dict())
    optimizer = FakeOptimizer()
    def interrupted(batch, deadline):
        with torch.no_grad():
            model.weight.add_(3.)
        optimizer.value = 9
        raise RuntimeError('partial update')
    trainer = SimpleNamespace(optimizer=optimizer, update=interrupted)
    state = runner.new_state()
    batch = {'marker': 1}
    state['pending'] = dict(kind='update_ready', critic=False, batch=batch)
    checkpoints = []
    with pytest.raises(RuntimeError, match='partial update'):
        runner.update_pending(state, model, trainer, lambda: checkpoints.append(deepcopy(state)), sandbox, None)
    assert all(torch.equal(original[key], value) for key, value in model.state_dict().items())
    assert optimizer.value == 0
    assert state['pending']['kind'] == 'update_ready'
    assert state['pending']['batch'] == batch
    assert len(checkpoints) == 2
    assert state['recoveries'][-1]['unknown_attempted_steps'] is True


def report(case_id='case', seed=1, decisions=100):
    return dict(case_id=case_id, seed=seed, task='cruise', agent_transitions=decisions, agent_steps=decisions,
        overrides=0, peak_abs_jerk=1., collisions=0, teleports=0, overspeed_steps=0,
        emergency_braking_observed=0, primary_horizon_complete=True, partial_control_horizon=False,
        accounting_comparison_valid=True, native_emergencies=dict(available=True, unique_events=0),
        v02_reward_summary=dict(detour_motion_events=0),
        per_car_full_route={'agent_0': dict(leader_samples=0, missing_headway_samples=0,
            missing_ttc_samples=0, minimum_headway_s=None, minimum_ttc_s=None)},
        smooth_control=dict(per_car={'agent_0': dict(assigned_controlled=True, route_completed=True,
            all=dict(decisions=decisions, intervention_decisions=0, jerk_m_s3=dict(realized=dict(peak_abs=1.)),
                legal_speed_utilization=dict(mean=.95)))}))


def test_gates_reject_one_bad_car_even_when_fleet_average_is_good():
    measured = report()
    measured['smooth_control']['per_car']['agent_1'] = deepcopy(measured['smooth_control']['per_car']['agent_0'])
    measured['smooth_control']['per_car']['agent_1']['all']['jerk_m_s3']['realized']['peak_abs'] = 6.
    gates = runner.development_gates(measured)
    assert gates['per_car']['agent_0']['passed'] is True
    assert gates['per_car']['agent_1']['passed'] is False
    assert gates['passed'] is False


def test_missing_or_incomplete_route_evidence_cannot_pass():
    measured = report()
    assert runner.development_gates(measured)['passed'] is True
    measured['smooth_control']['per_car']['agent_0']['route_completed'] = False
    assert runner.development_gates(measured)['passed'] is False


def test_optional_detour_cannot_pass_clean_safety_and_fleet_metrics():
    measured = report()
    measured['v02_reward_summary']['detour_motion_events'] = 1
    assert 'optional_ending_lane_detour_or_missing_audit' in runner.development_gates(measured)['failures']
    measured.pop('v02_reward_summary')
    assert runner.development_gates(measured)['passed'] is False


@pytest.mark.parametrize('field,value', [('minimum_headway_s', 1.49), ('minimum_ttc_s', 2.99),
    ('missing_headway_samples', 1), ('missing_ttc_samples', 1)])
def test_full_route_headway_ttc_or_missing_coverage_fails_per_car_gate(field, value):
    measured = report()
    measured['per_car_full_route']['agent_0'][field] = value
    gates = runner.development_gates(measured)
    assert gates['passed'] is False
    assert gates['per_car']['agent_0'][field] == value
    measured['smooth_control']['per_car'] = {}
    assert runner.development_gates(measured)['passed'] is False


def test_partial_diagnostic_keeps_declared_horizon_and_marks_limit(cfg):
    args = SimpleNamespace(final=False, case_index=0, limit_steps=12)
    rows = runner.selected_rows(cfg, args)
    assert rows[0]['horizon_steps'] == cfg['validation_rows'][0]['horizon_steps']
    assert rows[0]['limit_steps'] == 12
    assert 'limit_steps' not in cfg['validation_rows'][0]


def test_mock_training_caps_rollouts_and_exposes_all_twelve_cases(cfg, sandbox, monkeypatch):
    """Exercise orchestration with fake rollouts/updates, never a simulator or optimizer."""
    cfg['training_authorized'] = True
    output = runner.STUDY / 'mock-round'
    output.mkdir(parents=True)
    source = {'runner': 'sha'}
    proof = dict(schema='highway_flow_rl_preflight_v02', passed=True, training=False, optimizer_steps=0,
        config_sha256='config', source_hashes=source, checks=dict.fromkeys(PREFLIGHT_CHECKS, True))
    monkeypatch.setattr(runner, 'hashes', lambda: source)
    monkeypatch.setattr(runner.time, 'monotonic', lambda: 1.)
    monkeypatch.setattr(runner, 'export_actor', lambda *args: None)
    model = torch.nn.Linear(1, 1)
    trainer = SimpleNamespace(optimizer=FakeOptimizer(), update=lambda batch, deadline: dict(optimizer_steps=1, approx_kl=.01))
    captured = []
    def fake_episode(config, row, directory, current_model, **options):
        captured.append((row['case']['id'], options['limit'], row['seed']))
        return report(row['case']['id'], row['seed'], options['limit']), {'fake_batch': True}
    def fake_evaluate(config, directory, label, current_model, deadline, *, progress=None, completed=None, **options):
        reports = [report()]
        if progress:
            progress(reports)
        return dict(summary=runner.summarize(reports), reports=reports)
    monkeypatch.setattr(runner, 'episode', fake_episode)
    monkeypatch.setattr(runner, 'evaluate', fake_evaluate)
    session = SimpleNamespace(deadline=901., segment={}, totals=dict(round_decisions=0, total_decisions=0),
        elapsed=lambda: 0., checkpoint=lambda decisions: None)
    result = runner.train(cfg, output, model, trainer, None, session, 'config', source, preflight=proof)
    assert result['state']['decisions'] == 30000
    limits = {row['case']['id']: row.get('rollout_decisions', 2048) for row in cfg['training_rows']}
    assert all(limit <= limits[case] for case, limit, _ in captured)
    assert {case for case, _, _ in captured[:12]} == {row['case']['id'] for row in cfg['training_rows']}
    assert result['state']['pending'] is None
    assert result['state']['phase'] == 'complete'
    assert result['state']['optimizer_steps'] == len(captured)
