"""Pure CPU contracts. Native rollouts and every optimizer step are mocked."""
from copy import deepcopy
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import Mock
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from experiments import run_highway_smooth_control as runner
from traffic import highway_smooth_contract as contract
from traffic import highway_smooth_schedule as schedule


@pytest.fixture
def cfg():
    return runner.build_config()


@pytest.fixture(autouse=True)
def prohibit_optimizer_steps(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError('Tests may not execute any optimizer step')
    monkeypatch.setattr(torch.optim.Adam, 'step', forbidden)
    torch.set_num_threads(1)
    (runner.ROOT / 'outputs').mkdir(exist_ok=True)


@pytest.fixture
def synthetic_initialization(cfg, monkeypatch):
    """Real, untrained checkpoint/actor pair independent of local run artifacts."""
    with TemporaryDirectory(dir=runner.ROOT / 'outputs', prefix='smooth-init-test-') as directory:
        folder = Path(directory).resolve()
        assert folder.is_relative_to(runner.ROOT / 'outputs')
        with torch.random.fork_rng(devices=[]):
            torch.manual_seed(1234)
            model = runner.old.model_for(cfg['base_config'])
            saved = deepcopy(model.state_dict())
            torch.save({'model': saved}, folder / 'checkpoint.pt')
            runner.export_actor(model, folder / 'actor.pt')
        expected = {}
        for key, filename in (('actor', 'actor.pt'), ('checkpoint', 'checkpoint.pt')):
            path = folder / filename
            expected[key] = dict(path=path.relative_to(runner.ROOT).as_posix(),
                                 sha256=contract.file_digest(path))
            cfg['initialization'][key] = deepcopy(expected[key])
        # Replace fixture identities only; validation, hashing, real loading and
        # checkpoint/export parity checks remain active.
        monkeypatch.setattr(contract, 'FROZEN_INITIALIZATION', expected)
        yield cfg, saved


def test_disabled_config_matches_checked_in_file(cfg):
    assert cfg == runner.load_config()
    assert cfg['training_authorized'] is False and cfg['readiness'] is None
    with pytest.raises(ValueError, match='authorization'):
        contract.require_training(cfg, schedule.digest(cfg), {})


@pytest.mark.parametrize('field,value', [('total_decisions', 100001), ('per_arm_decisions', 50001),
    ('wall_seconds', 3601), ('critic_warmup_per_arm', 2049), ('wall_seconds', float('nan')),
    ('wall_seconds', True), ('final_evaluation_reserve_seconds', 899)])
def test_budget_rejects_changed_ceiling(cfg, field, value):
    cfg['budget'][field] = value
    with pytest.raises(ValueError):
        contract.validate_config(cfg)


def test_immutable_case_shares_and_separate_decision_exposure(cfg):
    rows = schedule.build_schedule(cfg)
    assert rows == schedule.build_schedule(deepcopy(cfg))
    first = rows[:20]
    assert {task: sum(r['case']['task'] == task for r in first) for task in ('cruise', 'following', 'passing', 'merge')} == dict(cruise=4, following=6, passing=4, merge=6)
    assert [sum(r['difficulty'] == d for r in first) for d in range(3)] == [8, 6, 6]
    assert all(r['total_vehicles'] == 24 and r['difficulty'] == 2 for r in rows if r['matched_schedule']['dense_following'])
    assert set(r['seed'] for r in rows).isdisjoint(r['seed'] for r in cfg['base_config']['validation_cases'])
    assert all(r['case']['split'] == 'train' for r in rows)
    assert any(r['controlled_count'] == r['total_vehicles'] == 24 for r in rows)
    assert all(r['traffic_mode'] == 'initial' and r['case']['task'] not in ('following', 'passing') for r in rows if r['controlled_count'] == r['total_vehicles'])
    state = contract.new_arm_state()
    row = schedule.next_case(rows, state)
    row['difficulty'] = 99
    assert rows[0]['difficulty'] != 99
    assert state['schedule_index'] == 1
    report = schedule.coverage_report(state)
    assert report['tasks']['cruise']['planned_case_fraction'] == .2
    assert report['tasks']['cruise']['decision_target_shortfall'] == .2


def test_corrected_network_all_validation_rows_and_contract_changes_rejected(cfg):
    assert len(cfg['base_config']['validation_cases']) == 24
    case = next(r['case'] for r in cfg['base_config']['validation_cases'] if 'i110' in r['case']['id'])
    assert case['network_sha256'] == runner.NETWORK_SHA
    case['network_sha256'] = '0' * 64
    with pytest.raises(ValueError, match='corrected'):
        contract.validate_config(cfg)


@pytest.mark.parametrize('mutation', ['actor', 'checkpoint', 'validation_population', 'validation_channel', 'parent'])
def test_exact_frozen_inputs_and_entire_validation_bank_are_pinned(cfg, mutation):
    if mutation in ('actor', 'checkpoint'):
        cfg['initialization'][mutation]['sha256'] = '0' * 64
    elif mutation == 'validation_population':
        cfg['base_config']['validation_cases'][19]['controlled_count'] = 1
    elif mutation == 'validation_channel':
        cfg['base_config']['validation_cases'][19]['packet_loss'] = .1
    else:
        cfg['provenance']['parent_config_sha256'] = '0' * 64
    with pytest.raises(ValueError):
        contract.validate_config(cfg)


def test_reservation_refunds_only_success_and_never_rewrites_row(cfg):
    row = schedule.build_schedule(cfg)[0]
    frozen = deepcopy(row)
    state = contract.new_arm_state()
    reserve = contract.reserve_episode(cfg, state, row, critic_only=True)
    assert reserve <= 2048
    contract.complete_episode(state, row['controlled_count'])
    assert state['spent'] == state['actual'] == row['controlled_count']
    assert row == frozen
    state['pending']['kind'] = 'update_ready'
    contract.recover_pending(state)
    assert state['pending']['kind'] == 'update_ready'
    state['pending']['kind'] = 'updating'
    contract.recover_pending(state)
    assert state['pending'] is None and state['spent'] == row['controlled_count']
    assert state['failures'][-1]['uncommitted_optimizer_steps_unknown']
    state = contract.new_arm_state()
    reserve = contract.reserve_episode(cfg, state, row, critic_only=False)
    contract.recover_pending(state)
    assert state['spent'] == reserve and state['actual'] == 0


def test_resume_charges_unclean_time_without_duplicate_preparation():
    assert contract.elapsed_on_resume(dict(elapsed_seconds=125., run_closed=True), 1000, 2000) == 125.
    assert contract.elapsed_on_resume(dict(elapsed_seconds=125., run_closed=False), 1000, 1020) == 145.


def write_proof(cfg, tmp_path, monkeypatch, status='completed'):
    parent = tmp_path / runner.PARENT_CONFIG
    parent.parent.mkdir(parents=True)
    parent.write_bytes((runner.ROOT / runner.PARENT_CONFIG).read_bytes())
    monkeypatch.setattr(contract, 'ROOT', tmp_path)
    ledger_path = tmp_path / cfg['preparation_ledger']
    ledger_path.parent.mkdir(parents=True)
    ledger_path.write_text(json.dumps(dict(schema='smooth_preparation_ledger_v01', attempts=[
        dict(status='failed', charged_wall_seconds=12.5, reserved_wall_seconds=180),
        dict(status=status, charged_wall_seconds=10., reserved_wall_seconds=180)])))
    proof = dict(schema='highway_smooth_preflight_v01', passed=True, training=False, optimizer_steps=0,
        source_hashes={'example.py': 'a'}, preparation_sha256=contract.preparation_digest(cfg),
        checks={key: True for key in contract.CHECKS}, preparation_wall_seconds=22.5,
        preparation_ledger_sha256=contract.file_digest(ledger_path))
    proof_path = tmp_path / 'outputs/proof.json'
    proof_path.write_text(json.dumps(proof))
    cfg['readiness'] = dict(summary_path='outputs/proof.json', summary_sha256=contract.file_digest(proof_path))
    return proof, proof_path


def test_readiness_charges_failures_and_requires_exact_sources(cfg, tmp_path, monkeypatch):
    write_proof(cfg, tmp_path, monkeypatch)
    assert contract.readiness_check(cfg, {'example.py': 'a'})['charged_preparation_wall_seconds'] == 22.5
    with pytest.raises(ValueError, match='stale'):
        contract.readiness_check(cfg, {'example.py': 'changed'})
    cfg['training_authorized'] = True
    assert contract.require_training(cfg, schedule.digest(cfg), {'example.py': 'a'})
    with pytest.raises(ValueError, match='authorization'):
        contract.require_training(cfg, 'stale', {'example.py': 'a'})


def test_running_preparation_refuses_launch(cfg, tmp_path, monkeypatch):
    write_proof(cfg, tmp_path, monkeypatch, status='running')
    with pytest.raises(ValueError, match='unfinished'):
        contract.readiness_check(cfg, {'example.py': 'a'})


@pytest.mark.parametrize('mutation', ['missing_comfort', 'learned', 'wrong_cost', 'exhausted'])
def test_invalid_proof_fails_closed(cfg, tmp_path, monkeypatch, mutation):
    proof, path = write_proof(cfg, tmp_path, monkeypatch)
    if mutation == 'missing_comfort':
        proof['checks'].pop('comfort')
    elif mutation == 'learned':
        proof['optimizer_steps'] = 1
    elif mutation == 'wrong_cost':
        proof['preparation_wall_seconds'] = 10.
    else:
        cfg['budget']['wall_seconds'] = 910
        proof['preparation_sha256'] = contract.preparation_digest(cfg)
    path.write_text(json.dumps(proof))
    cfg['readiness']['summary_sha256'] = contract.file_digest(path)
    with pytest.raises(ValueError):
        contract.readiness_check(cfg, {'example.py': 'a'})


def synthetic_batch(model):
    local = torch.zeros(24, 99)
    local[:, 30:33] = 1.
    local[:, 0] = .5
    local[:, 4] = .7
    global_rows = torch.cat((local, torch.ones(24, 1)), dim=1).reshape(1, 2400).repeat(24, 1)
    indices = torch.arange(24)
    with torch.no_grad():
        actions, logs, _, pre = model.act(local, deterministic=True)
        values = model.values(global_rows, indices)
    return dict(local_obs=local, global_states=global_rows, agent_indices=indices,
        actions=actions, pre_tanh_actions=pre, old_log_probs=logs, old_values=values,
        advantages=torch.linspace(-1., 1., 24), returns=values + .2,
        active=torch.ones(24, dtype=torch.bool), teacher_actions=torch.zeros(24, 2))


def test_identical_frozen_actor_fresh_critic_rng_and_no_update_diagnostics(synthetic_initialization):
    cfg, saved = synthetic_initialization
    before = runner.capture_rng()
    arms = runner.initialize_arms(cfg)
    assert runner.tree_equal(before, runner.capture_rng())
    left, right = (arms[name] for name in contract.ARMS)
    assert runner.tree_equal(left['model'].state_dict(), right['model'].state_dict())
    loaded = left['model'].state_dict()
    assert all(torch.equal(value, loaded[name]) for name, value in saved.items()
               if name.startswith('actor.') or name == 'log_std')
    assert any(not torch.equal(value, loaded[name]) for name, value in saved.items()
               if name.startswith('critic.'))
    assert runner.tree_equal(left['rng'], right['rng'])
    assert not left['trainer'].optimizer.state_dict()['state']
    batch = synthetic_batch(left['model'])
    result = runner.frozen_loss_check(cfg, {name: batch for name in contract.ARMS}, arms)
    assert result['passed'] and result['optimizer_steps'] == 0 and result['state_unchanged']
    assert result['arms'][contract.ARMS[0]]['active_samples'] == 24
    with runner.arm_rng(left):
        first = torch.rand(3)
    with runner.arm_rng(right):
        assert torch.equal(first, torch.rand(3))
    assert runner.tree_equal(before, runner.capture_rng())


@pytest.mark.parametrize('artifact', ['actor', 'checkpoint'])
def test_initialization_rejects_corrupted_fixture_artifact(synthetic_initialization, artifact):
    cfg, _ = synthetic_initialization
    path = runner.ROOT / cfg['initialization'][artifact]['path']
    path.write_bytes(path.read_bytes() + b'corrupt')
    with pytest.raises(ValueError, match='Frozen ' + artifact + ' checksum mismatch'):
        runner.initialize_arms(cfg)


def test_train_api_refuses_disabled_before_creating_output(cfg, tmp_path, monkeypatch):
    monkeypatch.setattr(runner, 'sources', lambda: {})
    monkeypatch.setattr(runner, 'initialize_arms', Mock(side_effect=AssertionError('must not initialize')))
    with pytest.raises(ValueError, match='authorization'):
        runner.train(cfg, tmp_path / 'no-run', approved_config_sha256=schedule.digest(cfg))
    assert not (tmp_path / 'no-run').exists()


def endpoint_reports(cfg, p95=2., mean=.5, speed=.95, peak=4.):
    reports = []
    for row in cfg['base_config']['validation_cases']:
        merge = row['case']['task'] == 'merge'
        report = dict(case_id=row['case']['id'], seed=row['seed'], task=row['case']['task'],
            agent_steps=100, overrides=1, collisions=0, teleports=0, overspeed_steps=0,
            emergency_braking_observed=0, passes=1, successful_merges=int(merge), controlled_count=row['controlled_count'],
            intervention_fraction=.01, mean_fraction_of_speed_limit=speed,
            native_emergencies=dict(available=True, unique_events=0), primary_horizon_complete=True,
            accounting_comparison_valid=True, partial_control_horizon=False,
            merge_target_count=int(merge), merge_target_ids=['agent_0'] if merge else [],
            stable_merge=dict(stable=int(merge)), completed_trips=8,
            smooth_control=dict(controlled=dict(jerk_m_s3=dict(realized=dict(samples=100, p95_abs=p95, mean_abs=mean))),
                per_car={'agent_0': dict(assigned_controlled=True,
                    nominal_merge_stop=dict(coverage_complete=True, longest_consecutive_stop_s=0.,
                        threshold_speed_m_s=.1, start_time_s=None, end_time_s=None, stable_completion_time_s=18.1,
                        observed_start_time_s=.1, observed_end_time_s=18.1),
                    phases=dict(approach=dict(jerk_m_s3=dict(realized=dict(samples=10, peak_abs=peak)))))}))
        reports.append(report)
    return reports


def test_endpoint_gate_uses_all_24_rows_and_fixed_nominal_windows(cfg):
    control = endpoint_reports(cfg)
    treatment = endpoint_reports(cfg, p95=1.7)
    result = contract.compare_endpoints(cfg, control, treatment)
    assert result['proposal_gates_passed'] and not result['accepted']
    assert len(result['paired_cases']) == 24
    assert {r['index'] for r in result['nominal_approach']} == {3, 7, 19}
    assert result['median_paired_p95_relative_improvement'] == pytest.approx(.15)
    with pytest.raises(ValueError, match='24'):
        contract.compare_endpoints(cfg, control[:-1], treatment)
    treatment[0]['seed'] += 1
    with pytest.raises(ValueError, match='mismatch'):
        contract.compare_endpoints(cfg, control, treatment)


@pytest.mark.parametrize('arm', ['control', 'treatment'])
@pytest.mark.parametrize('field,value', [('primary_horizon_complete', False),
    ('accounting_comparison_valid', False), ('partial_control_horizon', True),
    ('primary_horizon_complete', None), ('accounting_comparison_valid', None),
    ('partial_control_horizon', None)])
def test_paired_comparison_requires_full_accounting_for_both_arms(cfg, arm, field, value):
    control, treatment = endpoint_reports(cfg), endpoint_reports(cfg, p95=1.7)
    target = control if arm == 'control' else treatment
    if value is None:
        target[23].pop(field)
    else:
        target[23][field] = value
    result = contract.compare_endpoints(cfg, control, treatment)
    assert not result['checks']['paired_full_horizon_accounting']
    assert not result['proposal_gates_passed']


def test_complete_control_safety_failure_is_reported_without_masking_candidate(cfg):
    control, treatment = endpoint_reports(cfg), endpoint_reports(cfg, p95=1.7)
    control[0]['collisions'] = 1
    result = contract.compare_endpoints(cfg, control, treatment)
    assert result['checks']['paired_full_horizon_accounting']
    assert not result['ordinary']['control']['all_gates_passed']
    assert result['ordinary']['treatment']['all_gates_passed']
    assert result['proposal_gates_passed']


@pytest.mark.parametrize('failure', ['peak', 'mean', 'speed', 'trips', 'passes', 'stable', 'safety', 'median'])
def test_endpoint_each_gate_cannot_hide_regression(cfg, failure):
    control, treatment = endpoint_reports(cfg), endpoint_reports(cfg, p95=1.7)
    expected = dict(peak='nominal_approach_peak', mean='task_mean_jerk', speed='task_speed',
        trips='total_task_outcomes', passes='total_task_outcomes', stable='total_task_outcomes',
        safety='ordinary_safety_and_task_gates', median='median_paired_p95')[failure]
    if failure == 'peak':
        treatment[19]['smooth_control']['per_car']['agent_0']['phases']['approach']['jerk_m_s3']['realized']['peak_abs'] = 5.01
    elif failure == 'mean':
        treatment[0]['smooth_control']['controlled']['jerk_m_s3']['realized']['mean_abs'] = 1.
    elif failure == 'speed':
        treatment[0]['mean_fraction_of_speed_limit'] = .7
    elif failure in ('trips', 'passes'):
        treatment[0]['completed_trips' if failure == 'trips' else 'passes'] -= 1
    elif failure == 'stable':
        treatment[3]['stable_merge']['stable'] = 0
    elif failure == 'safety':
        treatment[0]['collisions'] = 1
    else:
        treatment = endpoint_reports(cfg, p95=1.9)
    result = contract.compare_endpoints(cfg, control, treatment)
    assert not result['proposal_gates_passed'] and not result['checks'][expected]


@pytest.mark.parametrize('seconds,passed', [(0., True), (1.9, True), (2., False), (16.8, False)])
def test_nominal_merge_stop_boundary_includes_exact_two_seconds(cfg, seconds, passed):
    control, treatment = endpoint_reports(cfg), endpoint_reports(cfg, p95=1.7)
    treatment[19]['smooth_control']['per_car']['agent_0']['nominal_merge_stop']['longest_consecutive_stop_s'] = seconds
    result = contract.compare_endpoints(cfg, control, treatment)
    assert result['checks']['nominal_merge_no_prolonged_stop'] is passed
    assert result['proposal_gates_passed'] is passed
    assert {r['index'] for r in result['nominal_merge_stops']} == {3, 7, 19}


def test_pending_merge_stop_window_cannot_pass(cfg):
    control, treatment = endpoint_reports(cfg), endpoint_reports(cfg, p95=1.7)
    stop = treatment[3]['smooth_control']['per_car']['agent_0']['nominal_merge_stop']
    stop.update(coverage_complete=False, stable_completion_time_s=None)
    result = contract.compare_endpoints(cfg, control, treatment)
    assert not result['checks']['nominal_merge_no_prolonged_stop']
    assert not result['proposal_gates_passed']


@pytest.mark.parametrize('mutation', ['threshold', 'missing_duration', 'bad_time', 'missing'])
def test_nominal_merge_stop_evidence_cannot_be_relabeled_or_omitted(cfg, mutation):
    control, treatment = endpoint_reports(cfg), endpoint_reports(cfg, p95=1.7)
    car = treatment[7]['smooth_control']['per_car']['agent_0']
    if mutation == 'threshold':
        car['nominal_merge_stop']['threshold_speed_m_s'] = .01
    elif mutation == 'missing_duration':
        car['nominal_merge_stop']['longest_consecutive_stop_s'] = None
    elif mutation == 'bad_time':
        car['nominal_merge_stop']['observed_end_time_s'] = 10.
    else:
        car.pop('nominal_merge_stop')
    with pytest.raises((ValueError, KeyError)):
        contract.compare_endpoints(cfg, control, treatment)


def test_real_stop_metric_with_no_stop_has_null_event_bounds_and_passes(cfg):
    from traffic.highway_smooth_metrics import nominal_merge_stop
    evidence = nominal_merge_stop([dict(time_s=i*.1, speed_m_s=18.) for i in range(21)], 2.)
    assert evidence['coverage_complete'] and evidence['start_time_s'] is None and evidence['end_time_s'] is None
    control, treatment = endpoint_reports(cfg), endpoint_reports(cfg, p95=1.7)
    for rows in (control, treatment):
        for index in (3, 7, 19):
            rows[index]['smooth_control']['per_car']['agent_0']['nominal_merge_stop'] = deepcopy(evidence)
    result = contract.compare_endpoints(cfg, control, treatment)
    assert result['checks']['nominal_merge_no_prolonged_stop'] and result['proposal_gates_passed']


@pytest.mark.parametrize('mutation', ['reference', 'formula', 'gamma', 'environment', 'optimizer'])
def test_config_rejects_misrepresented_runtime(cfg, mutation):
    if mutation == 'reference':
        cfg['reference_parameters']['entry_preview_time_s'] = 3.
    elif mutation == 'formula':
        cfg['reward_bundle']['jerk_argument'] = 'clipped jerk'
    elif mutation == 'gamma':
        cfg['gae']['gamma'] = .9
    elif mutation == 'environment':
        cfg['base_config']['environment']['native_warmup_seconds'] = 2.
    else:
        cfg['base_config']['optimizer']['entropy_coef'] = .2
        cfg['optimizer']['entropy_coef'] = .2
    with pytest.raises(ValueError):
        contract.validate_config(cfg)


def fake_arms():
    result = {}
    for name in contract.ARMS:
        weights = {'fake': torch.tensor(1.)}
        model = SimpleNamespace(state_dict=lambda w=weights: w, load_state_dict=lambda value: None)
        optimizer = SimpleNamespace(state_dict=lambda: {}, load_state_dict=lambda value: None)
        trainer = SimpleNamespace(optimizer=optimizer, update=Mock(return_value=dict(optimizer_steps=0)))
        result[name] = dict(model=model, trainer=trainer, rng=runner.capture_rng())
    return result


def mock_launch(cfg, monkeypatch):
    cfg['budget'].update(per_arm_decisions=16, total_decisions=32, critic_warmup_per_arm=0)
    monkeypatch.setattr(runner, 'sources', lambda: {})
    monkeypatch.setattr(runner, 'require_training', lambda *args: dict(charged_preparation_wall_seconds=123.))
    arms = fake_arms()
    monkeypatch.setattr(runner, 'initialize_arms', lambda cfg: arms)
    monkeypatch.setattr(runner, 'env_for', lambda cfg, output, row, name: SimpleNamespace(row=row, name=name))
    monkeypatch.setattr(runner, 'compare_endpoints', lambda *args: dict(proposal_gates_passed=False))
    monkeypatch.setattr(runner.old, 'summarize', lambda reports: dict(cases=len(reports)))
    monkeypatch.setattr(runner, 'export_actor', lambda model, path: path.write_bytes(b'test-only-no-policy'))
    calls = []
    def episode(env, row, **kwargs):
        calls.append((env.name, row['seed'], kwargs.get('collect', False)))
        if kwargs.get('collect'):
            return dict(agent_transitions=min(kwargs['limit'], row['controlled_count'])), {'fake': torch.ones(1)}
        return dict(case_id=row['case']['id'], seed=row['seed']), None
    monkeypatch.setattr(runner.old, 'episode', episode)
    return arms, calls


def test_mocked_paired_run_reuses_shared_initial_and_all_final_rows(cfg, monkeypatch):
    arms, calls = mock_launch(cfg, monkeypatch)
    with TemporaryDirectory(dir=runner.ROOT / 'outputs', prefix='smooth-contract-test-') as directory:
        run = Path(directory) / 'run'
        result = runner.train(cfg, run, approved_config_sha256='test-mock')
        assert result['completed'] and result['elapsed_seconds'] >= 123.
        assert sum(not collect for _, _, collect in calls) == 72
        learned = {name: [seed for arm, seed, collect in calls if arm == name and collect] for name in contract.ARMS}
        assert learned[contract.ARMS[0]] == learned[contract.ARMS[1]]
        assert all(value['spent'] <= 16 for value in result['counters'].values())
        assert sum(value['spent'] for value in result['counters'].values()) <= 32
        assert all(value['optimizer_steps'] == 0 for value in result['counters'].values())
        assert all(arm['trainer'].update.called for arm in arms.values())


def test_mocked_failed_update_is_not_replayed_on_resume(cfg, monkeypatch):
    arms, calls = mock_launch(cfg, monkeypatch)
    arms[contract.ARMS[0]]['trainer'].update.side_effect = RuntimeError('mock failure before any real step')
    with TemporaryDirectory(dir=runner.ROOT / 'outputs', prefix='smooth-contract-test-') as directory:
        run = Path(directory) / 'failed'
        with pytest.raises(RuntimeError, match='mock failure'):
            runner.train(cfg, run, approved_config_sha256='test-mock')
        saved = torch.load(run / 'last.pt', weights_only=False)
        counter = saved['state']['arms'][contract.ARMS[0]]
        assert counter['pending'] is None and counter['failures'][-1]['kind'] == 'updating'
        assert saved['state']['elapsed_seconds'] >= 123.
        assert 'mock failure' in json.loads((run / 'failure.json').read_text())['traceback']
        first_seed = next(seed for _, seed, collect in calls if collect)
        calls.clear()
        arms[contract.ARMS[0]]['trainer'].update.side_effect = None
        runner.train(cfg, Path(directory)/'resumed', approved_config_sha256='test-mock', resume=run/'last.pt')
        assert not any(arm == contract.ARMS[0] and seed == first_seed and collect for arm, seed, collect in calls)
        assert sum(not collect for _, _, collect in calls) == 48


def test_setup_failure_has_durable_budget_and_no_weights(cfg, monkeypatch):
    mock_launch(cfg, monkeypatch)
    monkeypatch.setattr(runner, 'initialize_arms', Mock(side_effect=RuntimeError('mock initialization failure')))
    with TemporaryDirectory(dir=runner.ROOT / 'outputs', prefix='smooth-contract-test-') as directory:
        run = Path(directory)/'failed'
        with pytest.raises(RuntimeError, match='initialization'):
            runner.train(cfg, run, approved_config_sha256='test-mock')
        saved = torch.load(run/'last.pt', weights_only=False)
        assert saved['arms'] is None and saved['state']['run_closed']
        assert saved['state']['elapsed_seconds'] >= 123.
        assert (run/'failure.json').exists()
