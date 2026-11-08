"""Pure CPU contracts. Every optimizer step forbidden; native calls mocked."""
from copy import deepcopy
import json
import os
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
import pytest
import torch
from experiments import run_highway_flow as runner
from traffic import highway_flow_contract as contract
from traffic import highway_flow_schedule as schedule
from policies.highway_flow import FlowModel,FlowTrainer,migrate_actor_99


@pytest.fixture(autouse=True)
def prohibit_steps(monkeypatch):
    monkeypatch.setattr(torch.optim.Adam,'step',Mock(side_effect=AssertionError('No optimizer steps authorized')))
    torch.set_num_threads(1)
    (runner.ROOT/'outputs').mkdir(exist_ok=True)


@pytest.fixture
def cfg():
    # Pure contracts use committed metadata only. Native bank/checkpoint artifacts
    # are deliberately absent in an ordinary clone and never synthesized here.
    return runner.build_config(None)


def require_local_artifacts(paths):
    missing=[str(path) for path in paths if not Path(path).is_file()]
    if os.environ.get('HIGHWAY_FLOW_PORTABLE_TESTS')=='1' or missing:
        pytest.skip('Local flow artifact check unavailable: '+(', '.join(missing) if missing else
            'HIGHWAY_FLOW_PORTABLE_TESTS=1 explicitly exercises the clean-clone path'))


def test_disabled_resolved_config():
    published=json.loads(runner.DEFAULT.read_text(encoding='utf-8'))
    manifest=runner.ROOT/published['network_bank']['manifest_path']
    require_local_artifacts([manifest])
    cfg=runner.build_config(published['network_bank']['manifest_path'])
    resolved=runner.load_config()
    expected=deepcopy(cfg)
    expected['initialization']['selected_id']=resolved['initialization']['selected_id']
    expected['readiness']=resolved['readiness']
    assert expected==resolved
    assert resolved['training_authorized'] is False


def test_pure_config_stays_disabled_without_any_generated_bank(cfg):
    assert cfg['network_bank'] is None
    assert cfg['training_authorized'] is False and cfg['readiness'] is None
    assert cfg['initialization']['selected_id'] is None
    with pytest.raises(ValueError,match='authorization'):
        contract.require_training(cfg,schedule.digest(cfg),{})


@pytest.mark.parametrize('field,value',[('wall_seconds',5401),('per_arm_decisions',30001),('total_decisions',60001),
    ('critic_warmup_per_arm',2049),('preparation_wall_seconds',1801),('final_evaluation_reserve_seconds',899)])
def test_budget_cannot_expand(cfg,field,value):
    cfg['budget'][field]=value
    with pytest.raises(ValueError): contract.validate_config(cfg)


@pytest.mark.parametrize('mutation',['actor','optimizer','model','case','reference','reward','schedule','parent'])
def test_misrepresented_runtime_rejected(cfg,mutation):
    if mutation=='actor': cfg['initialization']['candidates']['v05_final']['sha256']='0'*64
    elif mutation=='optimizer': cfg['optimizer']['actor_lr']=.01
    elif mutation=='model': cfg['base_config']['model']['observation_size']=99
    elif mutation=='case': cfg['base_config']['validation_cases'][19]['controlled_count']=1
    elif mutation=='reference': cfg['reference_parameters']['PLANNING_HORIZON_S']=7.
    elif mutation=='reward': cfg['base_config']['reward_weights']['jerk']=.03
    elif mutation=='schedule': cfg['schedule']['seed_start']=6900000
    else: cfg['provenance']['parent_config_sha256']='0'*64
    with pytest.raises(ValueError): contract.validate_config(cfg)


def test_schedule_all18_cases_difficulties_and_separate_coverage(cfg):
    rows=schedule.build_schedule(cfg)
    assert rows==schedule.build_schedule(deepcopy(cfg))
    assert len({r['case']['id'] for r in rows[:18]})==18
    assert all(r['controlled_count']==1 and r['requested_av_fraction']==1./r['total_vehicles'] for r in rows[:18])
    assert sum(r['controlled_count'] for r in rows[:18])==18
    assert sum(800*r['controlled_count'] for r in rows[:18])==14400
    assert 14400+cfg['budget']['critic_warmup_per_arm']<cfg['budget']['per_arm_decisions']
    assert {r['total_vehicles'] for r in rows[:18]}=={8,16,24}
    assert any(r['controlled_count']>1 for r in rows[18:])
    assert {r['difficulty'] for r in rows[:18]}=={0,1,2}
    assert {r['case']['task'] for r in rows}=={'cruise','following'}
    assert {r['speed_limit_m_s'] for r in rows}=={12.,18.,22.352,26.8224}
    assert {r['total_vehicles'] for r in rows}=={8,16,24}
    assert any(r['controlled_count']==r['total_vehicles']==24 for r in rows)
    assert all(r['traffic_mode']=='initial' and r['case']['task']=='cruise' for r in rows if r['controlled_count']==r['total_vehicles'])
    assert set(r['seed'] for r in rows).isdisjoint(r['seed'] for r in cfg['base_config']['validation_cases'])
    state=contract.new_arm_state(); row=schedule.next_case(rows,state)
    row['difficulty']=99
    assert rows[0]['difficulty']!=99
    row=rows[0]
    schedule.add_coverage(state,row,100,critic_only=True,report=dict(primary_horizon_complete=True,accounting_comparison_valid=True,partial_control_horizon=False))
    assert not schedule.coverage_report(state,cfg)['full_case_coverage']
    assert len(schedule.coverage_report(state,cfg)['missing_complete_case_ids'])==18


def test_warmup_has_independent_cursor_and_cannot_consume_ppo_coverage(cfg):
    rows=schedule.build_schedule(cfg);state=contract.new_arm_state()
    warmup=[schedule.next_case(rows,state,critic_only=True) for _ in range(3)]
    assert state['critic_schedule_index']==3 and state['schedule_index']==0
    ppo=[schedule.next_case(rows,state) for _ in range(18)]
    assert ppo==rows[:18] and ppo[0]==warmup[0]
    assert all(r['controlled_count']==1 for r in ppo)


def test_reservations_failure_and_unclean_resume(cfg):
    row=schedule.build_schedule(cfg)[0]; state=contract.new_arm_state()
    reserved=contract.reserve_episode(cfg,state,row,critic_only=True)
    assert reserved<=2048
    contract.complete_episode(state,row['controlled_count'])
    assert state['actual']==state['spent']==row['controlled_count']
    state['pending']['kind']='updating'
    contract.recover_pending(state)
    assert state['pending'] is None and state['failures'][-1]['uncommitted_optimizer_steps_unknown']
    state=contract.new_arm_state(); reserved=contract.reserve_episode(cfg,state,row,critic_only=False)
    contract.recover_pending(state)
    assert state['spent']==reserved and state['actual']==0
    assert contract.elapsed_on_resume(dict(elapsed_seconds=120.,run_closed=False),1000,1020)==140.
    assert contract.elapsed_on_resume(dict(elapsed_seconds=120.,run_closed=True),1000,1020)==120.


def synthetic_batch(model):
    local=torch.zeros(24,117); local[:,0]=.65; local[:,4]=.67; local[:,30:33]=1.
    state=torch.cat((local,torch.ones(24,1)),dim=-1).flatten().repeat(24,1)
    indices=torch.arange(24)
    with torch.no_grad():
        actions,logs,_,pre=model.act(local,deterministic=True)
        values=model.values(state,indices)
    return dict(local_obs=local,global_states=state,agent_indices=indices,actions=actions,pre_tanh_actions=pre,
        old_log_probs=logs,old_values=values,advantages=torch.linspace(-1.,1.,24),returns=values+.2,
        active=torch.ones(24,dtype=torch.bool),teacher_actions=torch.zeros(24,2),
        acceleration_supervision_eligible=torch.ones(24,dtype=torch.bool),lane_supervision_eligible=torch.ones(24,dtype=torch.bool))


def test_frozen_candidates_migrate_without_rng_or_critic_transfer(cfg):
    require_local_artifacts([runner.ROOT/spec['path'] for spec in cfg['initialization']['candidates'].values()]
        +[runner.ROOT/spec['actor_path'] for spec in cfg['initialization']['candidates'].values() if spec.get('actor_path')])
    before=runner.capture_rng(); models=runner.initialize_models(cfg)
    assert runner.tree_equal(before,runner.capture_rng())
    assert runner.tree_equal(models['v05_final'].critic.state_dict(),models['v05_initializer'].critic.state_dict())
    for model in models.values():
        assert model.observation_size==117 and model.state_size==2832
        assert torch.count_nonzero(model.actor.state_dict()['mean_net.1.0.weight'][:,51:69])==0
        x=torch.zeros(2,117); x[:,30:33]=1.
        assert torch.isfinite(model.actor(x)).all()
    with pytest.raises(ValueError,match='unresolved'): runner.initialize_arms(cfg)


def test_only_multiplier_differs_and_frozen_gradient_state_unchanged(cfg):
    spec=cfg['initialization']['candidates']['v05_final']
    require_local_artifacts([runner.ROOT/spec['path'],runner.ROOT/spec['actor_path']])
    arms=runner.initialize_arms(cfg,'v05_final'); a,b=(arms[k] for k in contract.ARMS)
    assert runner.tree_equal(a['model'].state_dict(),b['model'].state_dict())
    assert runner.tree_equal(a['rng'],b['rng'])
    assert not a['trainer'].optimizer.state_dict()['state']
    batch=synthetic_batch(a['model'])
    result=runner.frozen_loss_check(cfg,{k:batch for k in contract.ARMS},arms)
    assert result['passed'] and result['optimizer_steps']==0 and result['state_unchanged']
    parts=[]
    for arm in (a,b):
        part,_=arm['trainer']._prepare_batch(batch)
        log,ent=arm['model'].evaluate_actions(part['local_obs'],part['actions'],part['pre_tanh_actions'])
        _,weighted,_=arm['trainer']._loss_terms(part,(log-part['old_log_probs']).exp(),ent)
        parts.append(weighted)
    for name in parts[0]:
        assert float(parts[1][name].detach())==pytest.approx(float(parts[0][name].detach())*(5 if name=='acceleration' else 1))


def test_unqualified_labels_have_zero_auxiliary_gradient_but_keep_ppo(cfg):
    model=FlowModel(); trainer=FlowTrainer(model,**cfg['optimizer']); batch=synthetic_batch(model)
    batch['acceleration_supervision_eligible'].zero_(); batch['lane_supervision_eligible'].zero_()
    part,count=trainer._prepare_batch(batch)
    assert count==24
    log,ent=model.evaluate_actions(part['local_obs'],part['actions'],part['pre_tanh_actions'])
    _,parts,stats=trainer._loss_terms(part,(log-part['old_log_probs']).exp(),ent)
    assert all(float(parts[k].detach())==0. for k in ('acceleration','lane','near_limit'))
    assert stats['acceleration_supervision_samples']==stats['near_limit_samples']==0
    batch.pop('lane_supervision_eligible')
    with pytest.raises(ValueError,match='eligible'): trainer._prepare_batch(batch)


def endpoint_reports(cfg):
    reports=[]
    for row in cfg['base_config']['validation_cases']:
        merge=row['case']['task']=='merge'
        reports.append(dict(case_id=row['case']['id'],seed=row['seed'],task=row['case']['task'],agent_steps=100,
            overrides=1,collisions=0,teleports=0,overspeed_steps=0,emergency_braking_observed=0,passes=1,
            successful_merges=int(merge),controlled_count=row['controlled_count'],intervention_fraction=.01,
            mean_fraction_of_speed_limit=.95,native_emergencies=dict(available=True,unique_events=0),
            primary_horizon_complete=True,accounting_comparison_valid=True,partial_control_horizon=False,
            merge_target_count=int(merge),stable_merge=dict(stable=int(merge)),completed_trips=8,requested_trips=10,
            accounting=dict(comparison_valid=True,time=80.,totals=dict(requested=10,completed=8,unfinished=2,
                waiting=0,active=2,teleporting=0,removed=0,boundary_time_s=500.))))
    return reports


def test_selection_progress_precedes_overrides_and_tie_prefers_final(cfg):
    final,early=endpoint_reports(cfg),endpoint_reports(cfg)
    assert contract.rank_selection(cfg,dict(v05_final=final,v05_initializer=early))['selected_id']=='v05_final'
    for r in early:
        r['overrides']=0; r['intervention_fraction']=0.; r['accounting']['totals']['boundary_time_s']=510.
    assert contract.rank_selection(cfg,dict(v05_final=final,v05_initializer=early))['selected_id']=='v05_final'
    early[0]['completed_trips']=9; early[0]['accounting']['totals'].update(completed=9,active=1,unfinished=1)
    result=contract.rank_selection(cfg,dict(v05_final=final,v05_initializer=early))
    assert result['selected_id']=='v05_initializer' and not result['accepted']


@pytest.mark.parametrize('which',[0,1,2])
@pytest.mark.parametrize('field,value',[('primary_horizon_complete',False),('accounting_comparison_valid',False),('partial_control_horizon',True)])
def test_both_arms_and_baseline_need_complete24(cfg,which,field,value):
    rows=[endpoint_reports(cfg) for _ in range(3)]; rows[which][23][field]=value
    with pytest.raises(ValueError,match='complete'): contract.compare_endpoints(cfg,*rows)


def test_foundation_retention_is_distinct_from_global_acceptance(cfg):
    rows=endpoint_reports(cfg)
    for r in rows:
        if r['task']=='merge': r['stable_merge']['stable']=0
    result=contract.compare_endpoints(cfg,rows,deepcopy(rows),deepcopy(rows))
    for arm in result['arms'].values():
        assert arm['foundation_stage_gates_passed'] and not arm['full_task_gates']['all_gates_passed']
    assert not result['accepted']
    later=deepcopy(rows); later[18]['passes']=0
    assert not contract.compare_endpoints(cfg,rows,rows,later)['arms'][contract.ARMS[1]]['retention_passed']


def test_train_refuses_before_output_or_initialization(cfg,monkeypatch,tmp_path):
    monkeypatch.setattr(runner,'sources',lambda:{})
    monkeypatch.setattr(runner,'initialize_arms',Mock(side_effect=AssertionError('must not initialize')))
    with pytest.raises(ValueError,match='authorization'):
        runner.train(cfg,tmp_path/'no-run',approved_config_sha256=schedule.digest(cfg))
    assert not (tmp_path/'no-run').exists()


def test_readiness_requires_selected_actor_all_checks_and_charges_failures(cfg,monkeypatch):
    with TemporaryDirectory(dir=runner.ROOT/'outputs',prefix='flow-proof-test-') as directory:
        base=Path(directory); ledger=base/'ledger.json'; proof=base/'proof.json'
        ledger.write_text(json.dumps(dict(schema='highway_flow_preparation_ledger_v01',attempts=[
            dict(status='failed',charged_wall_seconds=12.),dict(status='completed',charged_wall_seconds=10.)])))
        cfg['preparation_ledger']=str(ledger); cfg['initialization']['selected_id']='v05_final'
        value=dict(schema='highway_flow_preflight_v01',passed=True,training=False,optimizer_steps=0,
            source_hashes={'source':'hash'},preparation_sha256=contract.preparation_digest(cfg),
            checks={k:True for k in contract.CHECKS},preparation_wall_seconds=22.,
            preparation_ledger_sha256=contract.file_digest(ledger),selected_initializer_id='v05_final')
        proof.write_text(json.dumps(value)); cfg['readiness']=dict(summary_path=str(proof),summary_sha256=contract.file_digest(proof))
        assert contract.readiness_check(cfg,{'source':'hash'})['charged_preparation_wall_seconds']==22.
        with pytest.raises(ValueError,match='stale'): contract.readiness_check(cfg,{'source':'changed'})
        value['checks']['reference']=False; proof.write_text(json.dumps(value));cfg['readiness']['summary_sha256']=contract.file_digest(proof)
        with pytest.raises(ValueError,match='failed'): contract.readiness_check(cfg,{'source':'hash'})


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
    monkeypatch.setattr(runner, '_claim_run', lambda *args: None)
    monkeypatch.setattr(runner, 'compare_endpoints', lambda *args: dict(arms={name:dict(foundation_stage_gates_passed=False) for name in contract.ARMS}))
    monkeypatch.setattr(runner.old, 'summarize', lambda reports: dict(cases=len(reports)))
    monkeypatch.setattr(runner, 'export_actor', lambda model, path: path.write_bytes(b'test-only-no-policy'))
    calls = []
    def episode(env, row, **kwargs):
        calls.append((env.name, row['seed'], kwargs.get('collect', False)))
        if kwargs.get('collect'):
            return dict(agent_transitions=min(kwargs['limit'], row['controlled_count'])), {'fake': torch.ones(1)}
        return dict(case_id=row['case']['id'], seed=row['seed']), None
    monkeypatch.setattr(runner, 'episode', episode)
    return arms, calls


def test_mocked_paired_run_reuses_shared_initial_and_all_final_rows(cfg, monkeypatch):
    arms, calls = mock_launch(cfg, monkeypatch)
    with TemporaryDirectory(dir=runner.ROOT / 'outputs', prefix='flow-contract-test-') as directory:
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
    with TemporaryDirectory(dir=runner.ROOT / 'outputs', prefix='flow-contract-test-') as directory:
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
    with TemporaryDirectory(dir=runner.ROOT / 'outputs', prefix='flow-contract-test-') as directory:
        run = Path(directory)/'failed'
        with pytest.raises(RuntimeError, match='initialization'):
            runner.train(cfg, run, approved_config_sha256='test-mock')
        saved = torch.load(run/'last.pt', weights_only=False)
        assert saved['arms'] is None and saved['state']['run_closed']
        assert saved['state']['elapsed_seconds'] >= 123.
        assert (run/'failure.json').exists()


def test_single_study_launch_chain_forbids_fresh_retry_and_old_fork(cfg):
    with TemporaryDirectory(dir=runner.ROOT/'outputs',prefix='flow-chain-test-') as directory:
        root=Path(directory)
        cfg['output_directory']=root.relative_to(runner.ROOT).as_posix()
        first,second=root/'first',root/'second'
        runner._claim_run(cfg,first,None,'same-config')
        with pytest.raises(ValueError,match='already launched'):
            runner._claim_run(cfg,second,None,'same-config')
        runner._claim_run(cfg,second,first/'last.pt','same-config')
        with pytest.raises(ValueError,match='already launched'):
            runner._claim_run(cfg,root/'fork',first/'last.pt','same-config')
        assert not (root/'training-launch.lock').exists()


def test_gradient_capture_is_predeclared_train_only_and_checks_eligibility(cfg):
    from scripts.check_highway_flow_gradients import gradient_row,verify_batches
    row=gradient_row(cfg)
    assert row['case']['split']=='train' and row['case']['task']=='following' and row['difficulty']==2
    assert row['seed'] not in {r['seed'] for r in cfg['base_config']['validation_cases']}
    batch=synthetic_batch(FlowModel()); other=deepcopy(batch)
    assert all(verify_batches(batch,other).values())
    other['acceleration_supervision_eligible'][0]=False
    with pytest.raises(ValueError,match='qualified labels'): verify_batches(batch,other)
