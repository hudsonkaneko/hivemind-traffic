"""Launch/provenance and shared-budget regression checks without learning."""
from copy import deepcopy
import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from experiments import run_highway_flow_rl_v03 as runner
from traffic.highway_flow_rl_v03_contract import validate_config, validate_preflight, PREFLIGHT_CHECKS
from tests.test_highway_flow_rl_v02_runner import report as old_report


@pytest.fixture
def cfg():
    return json.loads((runner.ROOT/'configs/training/highway-flow-rl-v03.json').read_text())


def test_review_config_and_disabled_training_fail_before_work(cfg,monkeypatch,tmp_path):
    cfg['training_authorized']=False
    assert validate_config(cfg) is cfg
    with pytest.raises(ValueError,match='disabled'):
        runner.train(cfg,None,None,None,None,None,'config',{})
    no_model=Mock(side_effect=AssertionError('Must reject before loading model'))
    monkeypatch.setattr(runner,'build_model',no_model)
    (tmp_path/'disabled.json').write_text(json.dumps(cfg))
    monkeypatch.setattr(runner,'project_path',lambda value:tmp_path/'disabled.json')
    args=SimpleNamespace(config='disabled.json',command='train')
    with pytest.raises(ValueError,match='disabled'): runner.run(args)
    no_model.assert_not_called()


@pytest.mark.parametrize('field,value', [('reward_weights',{}),('target_settings',{}),('v03_reward_weights',{}),
    ('shared_cost_ledger','outputs/new-ledger.json'),('observation_size',118),('safety',{})])
def test_changed_contract_fails(cfg,field,value):
    cfg[field]=value
    with pytest.raises(ValueError): validate_config(cfg)


def test_native_preflight_requires_current_version_sources_and_config():
    proof=dict(schema='highway_flow_rl_preflight_v03',passed=True,training=False,optimizer_steps=0,
        config_sha256='cfg',source_hashes={'a':'sha'},checks=dict.fromkeys(PREFLIGHT_CHECKS,True))
    assert validate_preflight(proof,'cfg',{'a':'sha'}) is proof
    for change in (dict(schema='highway_flow_rl_preflight_v02'),dict(source_hashes={}),
                   dict(config_sha256='stale'),dict(optimizer_steps=1),dict(checks={})):
        with pytest.raises(ValueError):validate_preflight(dict(proof,**change),'cfg',{'a':'sha'})


def test_ledger_path_schema_and_previous_costs_preserved(cfg):
    assert runner.STUDY==runner.ROOT/'outputs/highway-flow-rl/v02'
    assert runner.OUTPUT_ROOT==runner.ROOT/'outputs/highway-flow-rl/v03'
    ledger=dict(schema='highway_flow_rl_cost_ledger_v02',inherited_seconds=0.,segments=[
        dict(category='preparation',command='offline_reward_audit',status='failed',charged_seconds=115.734,
             decisions=0,episodes=0,round_index=1)])
    totals=runner.ledger_totals(ledger,1)
    assert totals['total']==115.734
    assert runner.available_seconds(cfg,totals,'preparation')==pytest.approx(64.266)


def test_missing_cruise_coverage_or_avoidable_braking_cannot_pass():
    report=old_report();report['v03_reward_summary']=report.pop('v02_reward_summary')
    assert not runner.development_gates(report)['passed']
    report['avoidable_braking']={'agent_0':dict(clear_cruise_decisions=20,
        clear_cruise_speed_fraction_sum=19.,events=0)}
    assert runner.development_gates(report)['passed']
    report['avoidable_braking']['agent_0']['events']=1
    assert not runner.development_gates(report)['passed']


def test_exact_resume_rejects_changed_source_before_model(cfg,monkeypatch):
    saved=dict(schema=runner.CHECKPOINT_SCHEMA,config_sha256='same',source_hashes={'a':'old'},
        model={},optimizer={},rng={},state={})
    monkeypatch.setattr(runner.torch,'load',lambda *a,**kw:saved)
    constructor=Mock(side_effect=AssertionError('No model construction'))
    monkeypatch.setattr(runner,'FlowModel',constructor)
    with pytest.raises(ValueError,match='Exact resume'):
        runner.build_model(cfg,checkpoint='outputs/last.pt',resume=True,fingerprint='same',sources={'a':'new'})
    constructor.assert_not_called()


def test_all_runner_sources_exist():
    assert all((runner.ROOT/name).is_file() for name in runner.NEW_SOURCES)


def test_missing_shared_ledger_cannot_silently_reset_budget(cfg,tmp_path,monkeypatch):
    monkeypatch.setattr(runner,'STUDY',tmp_path/'missing-study')
    with pytest.raises(ValueError,match='never reset'):
        with runner.LedgerSession(cfg,'preflight',tmp_path/'out','cfg',started=runner.time.monotonic()):
            pytest.fail('Missing ledger admitted')


def long_cfg():
    return json.loads((runner.ROOT/'configs/training/highway-flow-rl-v03-long.json').read_text())


def test_extended_profile_only_expands_learning_and_retains_costs(cfg):
    long=long_cfg()
    validate_config(long,require_training=True)
    for key in ('reward_weights','v03_reward_weights','safety','target_settings','training_rows','validation_rows','held_out_rows'):
        assert long[key]==cfg[key]
    assert long['optimizer']==dict(cfg['optimizer'],epochs=8)
    totals=dict(total=418.687,training=294.859,preparation=123.828,final_evaluation=0.,
        diagnostic_seconds=89.125,round_seconds=0.,round_decisions=0,total_decisions=29999)
    assert runner.available_seconds(long,totals,'training')==5400.
    assert long['budget']['total_decisions']-totals['total_decisions']==300001
    assert not long['held_out_authorized']


@pytest.mark.parametrize('change',[dict(round_index=1),dict(learning_profile='anything')])
def test_extended_profile_rejects_unreviewed_rounds(change):
    cfg=long_cfg();cfg.update(change)
    with pytest.raises(ValueError):validate_config(cfg)


@pytest.mark.parametrize('key,value',[('epochs',9),('target_kl',.2)])
def test_extended_profile_keeps_optimizer_boundaries(key,value):
    cfg=long_cfg();cfg['optimizer'][key]=value
    with pytest.raises(ValueError):validate_config(cfg)


def test_extended_profile_cannot_restart_from_other_weights():
    cfg=long_cfg();cfg['warm_start']['sha256']='0'*64
    with pytest.raises(ValueError,match='round-one weights'):validate_config(cfg)


@pytest.mark.parametrize('field,value',[('seed',123),('controlled_count',24)])
def test_spawn_recovery_preflight_cannot_change_the_recorded_case(field,value):
    cfg=long_cfg();cfg['recovery_preflight_row'][field]=value
    with pytest.raises(ValueError,match='recorded failed training scenario'):
        validate_config(cfg)
