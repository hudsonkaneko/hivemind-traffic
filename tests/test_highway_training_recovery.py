import json
import hashlib
import pytest
import torch
from experiments.run_i110_merge import restore_selected_checkpoint, SCHEMA, PilotBudgetExhausted
from experiments.run_i110_merge import rank, RANKING_VERSION


def export_fixture(directory):
    actor=b'frozen actor fixture'
    (directory/'policy.pt').write_bytes(actor)
    (directory/'policy-contract.json').write_text(json.dumps({'policy_sha256':hashlib.sha256(actor).hexdigest()}))


def test_resume_preserves_selected_actor_without_new_improvement(tmp_path):
    old=tmp_path/'old';old.mkdir();new=tmp_path/'new';new.mkdir()
    torch.save(dict(schema=SCHEMA,config_sha256='fixed',selection_status='validation_selected',
                    best_rank=[0,-1,0,0,.5,-40,-.2,1.],ranking_version=RANKING_VERSION),old/'best.pt')
    export_fixture(old)
    score=restore_selected_checkpoint(old/'last.pt',new,'fixed')
    assert (new/'policy.pt').read_bytes()==b'frozen actor fixture'
    assert torch.load(new/'best.pt',weights_only=False)['best_rank']==list(score)
    assert json.loads((new/'policy-contract.json').read_text())['validation_rank']==list(score)


def test_resume_recomputes_old_rank_without_changing_source(tmp_path):
    old=tmp_path/'old';old.mkdir();new=tmp_path/'new';new.mkdir()
    torch.save(dict(schema=SCHEMA,config_sha256='fixed',selection_status='validation_selected',
                    best_rank=[0,0,-40,-.2,1.],update=10),old/'best.pt')
    export_fixture(old)
    report=dict(task='cruise',mean_fraction_of_speed_limit=.5,passes=0,collisions=0,
        teleports=0,overspeed_steps=0,successful_merges=0,accounting={'totals':{'unfinished':40}},
        overrides=2,agent_steps=10,reward=1.)
    (old/'validation-00010.json').write_text(json.dumps([report]))
    original=(old/'best.pt').read_bytes()
    assert restore_selected_checkpoint(old/'last.pt',new,'fixed')==rank([report])
    assert (old/'best.pt').read_bytes()==original
    assert torch.load(new/'best.pt',weights_only=False)['ranking_version']==RANKING_VERSION


def test_resume_rejects_unrelated_selected_checkpoint(tmp_path):
    torch.save(dict(schema=SCHEMA,config_sha256='other',selection_status='validation_selected'),tmp_path/'best.pt')
    with pytest.raises(ValueError,match='contract mismatch'):
        restore_selected_checkpoint(tmp_path/'last.pt',tmp_path,'fixed')


@pytest.mark.parametrize('broken',['missing','checksum'])
def test_resume_rejects_incomplete_or_corrupt_actor_export(tmp_path,broken):
    old=tmp_path/'old';old.mkdir();new=tmp_path/'new';new.mkdir()
    torch.save(dict(schema=SCHEMA,config_sha256='fixed',selection_status='validation_selected',
                    best_rank=[0],ranking_version=RANKING_VERSION),old/'best.pt')
    if broken=='checksum':
        export_fixture(old)
        (old/'policy.pt').write_bytes(b'changed')
    with pytest.raises(ValueError,match='missing|checksum'):
        restore_selected_checkpoint(old/'last.pt',new,'fixed')
    assert not list(new.iterdir())


def test_budget_exception_is_distinct_from_runtime_timeout():
    assert isinstance(PilotBudgetExhausted(),TimeoutError)
    assert not isinstance(TimeoutError(),PilotBudgetExhausted)


def test_stationary_policy_cannot_win_on_fewer_interventions():
    stopped=dict(task='cruise',mean_fraction_of_speed_limit=.07,passes=0,
        collisions=0,teleports=0,overspeed_steps=0,successful_merges=0,
        accounting={'totals':{'unfinished':40}},overrides=0,agent_steps=3200,reward=0.)
    moving=dict(stopped,mean_fraction_of_speed_limit=.96,overrides=1600,reward=2000.)
    assert rank([moving])>rank([stopped])


def test_successful_passing_precedes_intervention_minimization():
    no_pass=dict(task='passing',mean_fraction_of_speed_limit=.5,passes=0,
        collisions=0,teleports=0,overspeed_steps=0,successful_merges=0,
        accounting={'totals':{'unfinished':40}},overrides=0,agent_steps=3200,reward=0.)
    passing=dict(no_pass,passes=1,overrides=1000)
    assert rank([passing])>rank([no_pass])
