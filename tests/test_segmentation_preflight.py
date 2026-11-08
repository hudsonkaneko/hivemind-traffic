import json
from pathlib import Path
import pytest
from experiments.preflight_segmentation import validate_config,state_digest

ROOT=Path(__file__).resolve().parents[1]


def config():
    return json.loads((ROOT/'configs/perception/segmentation_refinement_training_v02.proposed.json').read_text())


def test_preflight_contract_does_not_authorize_learning():
    validate_config(config())


@pytest.mark.parametrize('field', ['launch_enabled','training_authorized'])
def test_preflight_refuses_training_enabled_config(field):
    cfg=config();cfg[field]=True
    with pytest.raises(ValueError):validate_config(cfg)


def test_bounded_reviewed_batch_contract():
    cfg=config();cfg['budget']['microbatch_size']=64
    with pytest.raises(ValueError):validate_config(cfg)
    cfg=config();cfg['budget']['microbatch_size']=2;cfg['budget']['gradient_accumulation_steps']=8
    validate_config(cfg)


def test_hash_distinguishes_parameter_and_buffer_mutation():
    import torch
    model=torch.nn.BatchNorm2d(2)
    params=state_digest(model,True);full=state_digest(model)
    model.running_mean.add_(1)
    assert state_digest(model,True)==params and state_digest(model)!=full
    with torch.no_grad():model.weight.add_(1)
    assert state_digest(model,True)!=params


def test_dataset_preflight_selects_train_rows_and_rejects_tampering(tmp_path,monkeypatch):
    import hashlib
    import experiments.preflight_segmentation as preflight
    monkeypatch.setattr(preflight,'ROOT',tmp_path)
    folder=tmp_path/'outputs/data';folder.mkdir(parents=True)
    frames=[]
    for index in range(4):
        row={'split':'train'}
        for kind in ('rgb','semantic'):
            name=f'{index}-{kind}.bin';content=f'{kind}-{index}'.encode();(folder/name).write_bytes(content)
            row[kind+'_path']=name;row[kind+'_sha256']=hashlib.sha256(content).hexdigest()
        frames.append(row)
    # Deliberately unusable held-out paths must never be opened by preflight.
    frames.insert(0,{'split':'test','rgb_path':'missing','semantic_path':'missing'})
    manifest={'schema_version':'segmentation-dataset-v02','training_ready':True,'frames':frames,'class_counts_by_split':{'train':[1]*6}}
    (folder/'dataset.json').write_text(json.dumps(manifest))
    cfg=config();cfg['dataset']['manifest']='outputs/data/dataset.json'
    pairs,hashes,counts,info=preflight.load_inputs(cfg,'dataset')
    assert len(pairs)==4 and counts==[1]*6 and info['training_ready']
    (folder/'0-rgb.bin').write_bytes(b'changed')
    with pytest.raises(ValueError,match='hash mismatch'):preflight.load_inputs(cfg,'dataset')


def test_gpu_guard_rejects_isaac_inventory_without_stopping_process(monkeypatch):
    import experiments.preflight_segmentation as preflight
    monkeypatch.setattr(preflight.os,'name','nt')
    calls=[]
    def inventory(command,**kwargs):
        calls.append(command)
        return json.dumps({'ProcessId':999999,'Name':'python.exe','ExecutablePath':'C:\\isaacsim\\python.exe','CommandLine':'arbitrary.py'})
    monkeypatch.setattr(preflight.subprocess,'check_output',inventory)
    with pytest.raises(RuntimeError,match='must run serially'):preflight.ensure_gpu_exclusive()
    assert 'ExecutablePath' in calls[0][-1] and 'capture_semantic_lidar_fusion' in calls[0][-1]
