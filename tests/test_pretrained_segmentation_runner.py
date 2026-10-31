import copy
import hashlib
import json
from pathlib import Path

import pytest
from experiments import preview_pretrained_segmentation as runner


@pytest.fixture
def config(tmp_path,monkeypatch):
    cfg=json.loads(runner.CONFIG.read_text())
    monkeypatch.setattr(runner,'ROOT',tmp_path)
    (tmp_path/cfg['source_capture_run']).mkdir(parents=True)
    return cfg


@pytest.mark.parametrize('key', ['training','new_simulator_runs'])
def test_preview_rejects_training_and_new_simulation(config,key):
    config[key]=True
    with pytest.raises(ValueError,match='inference-only'):runner.validate_config(config)


def test_preview_keeps_cpu_and_frozen_head(config):
    runner.validate_config(config)
    config['inference']['device']='cuda'
    with pytest.raises(ValueError,match='CPU'):runner.validate_config(config)
    config['inference']['device']='cpu';config['model']['trained_head_replaced']=True
    with pytest.raises(ValueError,match='trained head'):runner.validate_config(config)


def test_source_input_hashes_and_path_boundaries(config):
    source=runner.validate_config(config);config['cases']=['static-101'];config['frames_per_case']=1
    case=source/'static-101';case.mkdir()
    frame={'stem':'0000','rgb_path':'0000-rgb.png','semantic_path':'0000-semantic.npy'}
    hashes={}
    for key in ('rgb_path','semantic_path'):
        p=case/frame[key];p.write_bytes(b'saved evidence')
        hashes[str(p.relative_to(source))]=hashlib.sha256(p.read_bytes()).hexdigest()
    (source/'manifest.json').write_text(json.dumps({'status':'passed','output_hashes':hashes}))
    report=case/'capture-result.json'
    report.write_text(json.dumps({'passed':True,'frames':[frame]}))
    items,found=runner.capture_inputs(config,source)
    assert len(items)==1 and len(found)==2
    (case/frame['rgb_path']).write_bytes(b'changed')
    with pytest.raises(ValueError,match='hash mismatch'):runner.capture_inputs(config,source)
    frame['rgb_path']='../outside.png';report.write_text(json.dumps({'passed':True,'frames':[frame]}))
    with pytest.raises(ValueError,match='Escaping'):runner.capture_inputs(config,source)
