import json
from pathlib import Path

import pytest
import torch

from scripts import evaluate_hudson_actor as harness
from scripts.hudson_actor_worker import CheckedActor


def allow_only_git(monkeypatch):
    original=harness.subprocess.Popen
    def guarded(command,*args,**kwargs):
        if command[0]=='git':return original(command,*args,**kwargs)
        pytest.fail('Evaluation child must not launch')
    monkeypatch.setattr(harness.subprocess,'Popen',guarded)


def test_missing_actor_preserves_pinned_v02_bundle_without_child(tmp_path,monkeypatch):
    allow_only_git(monkeypatch)
    result=harness.run(tmp_path/'run',checkpoint=tmp_path/'absent.pt',mode='evaluate')
    assert result['status']=='blocked_missing_or_wrong_checkpoint'
    plan=json.loads((tmp_path/'run/plan.json').read_text())
    config=json.loads((tmp_path/'run/source/experiments/configs/sumo_mappo_v2.json').read_text())
    assert config['study']=='sumo-mappo-v02-cruise-margin'
    assert plan['seeds']==dict(smoke=[520001],held_out=list(range(410001,410021)))
    assert len(plan['source_sha256'])==14
    assert all(harness.sha(tmp_path/'run/source'/name)==digest for name,digest in plan['source_sha256'].items())


def test_wrong_actor_never_loads_or_copies(tmp_path,monkeypatch):
    policy=tmp_path/'wrong.pt';policy.write_bytes(b'not the recorded checkpoint')
    allow_only_git(monkeypatch)
    result=harness.run(tmp_path/'run',checkpoint=policy,mode='evaluate')
    assert result['status']=='blocked_missing_or_wrong_checkpoint'
    assert not (tmp_path/'run/policy.pt').exists()
    assert result['actor_loaded'] is False


def test_native_cannot_consume_held_out_and_output_cannot_overwrite(tmp_path):
    with pytest.raises(ValueError,match='held-out'):
        harness.run(tmp_path/'run',mode='native-check',suite='held-out')
    assert not (tmp_path/'run').exists()
    (tmp_path/'existing').mkdir()
    with pytest.raises(FileExistsError):
        harness.prepare(tmp_path/'existing')


def test_actor_contract_preserves_valid_action_values_without_gradients():
    actor=CheckedActor(lambda obs:obs[:,:1]*.25)
    result=actor(torch.ones((4,27),dtype=torch.float32,requires_grad=True))
    assert torch.equal(result,torch.full((4,1),.25)) and not result.requires_grad


@pytest.mark.parametrize('observations',[torch.zeros((4,30)),torch.zeros((4,27),dtype=torch.float64),torch.full((4,27),float('nan'))])
def test_actor_rejects_incompatible_observations(observations):
    with pytest.raises(ValueError):CheckedActor(lambda obs:torch.zeros((4,1)))(observations)


@pytest.mark.parametrize('output',[torch.zeros((4,)),torch.full((4,1),1.01),torch.full((4,1),float('nan'))])
def test_actor_rejects_invalid_actions(output):
    with pytest.raises(ValueError):CheckedActor(lambda obs:output)(torch.zeros((4,27)))
