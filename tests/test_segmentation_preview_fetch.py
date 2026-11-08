import copy
import hashlib
import io
import json

import pytest

from scripts import fetch_segmentation_preview_model as fetch


@pytest.fixture
def setup(tmp_path, monkeypatch):
    monkeypatch.setattr(fetch, 'ROOT', tmp_path)
    weights = b'unit-test-weights-never-executed'
    monkeypatch.setattr(fetch, 'WEIGHTS_BYTES', len(weights))
    monkeypatch.setattr(fetch, 'WEIGHTS_SHA256', hashlib.sha256(weights).hexdigest())
    model = dict(model_type='segformer', architectures=['SegformerForSemanticSegmentation'],
                 id2label={str(index):name for index,name in enumerate(fetch.LABELS)})
    processor = dict(do_normalize=True, do_resize=True, size=512, resample=2,
                     reduce_labels=False, image_mean=[.485,.456,.406], image_std=[.229,.224,.225])
    payloads = {'config.json':json.dumps(model).encode(),
                'preprocessor_config.json':json.dumps(processor).encode(), 'pytorch_model.bin':weights}
    requests = []

    def urlopen(url, timeout):
        requests.append(url)
        assert timeout == 60
        assert '/resolve/'+fetch.REVISION+'/' in url
        return io.BytesIO(payloads[url.rsplit('/',1)[1]])

    monkeypatch.setattr(fetch, 'urlopen', urlopen)
    config = dict(version='pretrained-segmentation-preview-v01', training=False,
        new_simulator_runs=False, model=dict(id=fetch.MODEL_ID, revision=fetch.REVISION,
        weights_sha256=fetch.WEIGHTS_SHA256, weights_bytes=fetch.WEIGHTS_BYTES))
    return config, payloads, requests


def test_pinned_three_file_download_then_offline_verified_reuse(setup):
    config, payloads, requests = setup
    directory, manifest = fetch.fetch_model(config)
    assert directory.is_relative_to(fetch.ROOT/'runtimes')
    assert len(requests) == 3
    assert set(manifest['files']) == set(payloads)
    assert manifest['training'] is False and manifest['weights_executed'] is False
    assert manifest['files']['pytorch_model.bin']['sha256'] == fetch.WEIGHTS_SHA256
    before = {p.name:p.read_bytes() for p in directory.iterdir()}
    again, same = fetch.fetch_model(config)
    assert again == directory and same == manifest
    assert len(requests) == 3
    assert before == {p.name:p.read_bytes() for p in directory.iterdir()}


@pytest.mark.parametrize('field,value', [('training',True), ('new_simulator_runs',True), ('version','unknown')])
def test_refuses_training_and_unreviewed_configuration_without_network(setup,field,value):
    config, _, requests = setup
    config[field] = value
    with pytest.raises(ValueError,match='inference-only'):
        fetch.fetch_model(config)
    assert requests == []


@pytest.mark.parametrize('field,value', [('id','untrusted/model'), ('revision','main'),
    ('weights_sha256','0'*64), ('weights_bytes',1)])
def test_refuses_changed_model_pins(setup,field,value):
    config, _, requests = setup
    config['model'][field] = value
    with pytest.raises(ValueError,match='pins'):
        fetch.fetch_model(config)
    assert requests == []


def test_corrupt_cached_file_is_preserved_without_network(setup):
    config, _, requests = setup
    directory, _ = fetch.fetch_model(config)
    (directory/'pytorch_model.bin').write_bytes(b'corrupt')
    with pytest.raises(ValueError,match='weight'):
        fetch.fetch_model(config)
    assert (directory/'pytorch_model.bin').read_bytes() == b'corrupt'
    assert len(requests) == 3


def test_partial_existing_cache_is_preserved_without_network(setup):
    config, _, requests = setup
    directory=fetch._cache_directory();directory.mkdir(parents=True)
    (directory/'pytorch_model.bin').write_bytes(b'partial')
    with pytest.raises(ValueError,match='no download manifest'):
        fetch.fetch_model(config)
    assert (directory/'pytorch_model.bin').read_bytes()==b'partial'
    assert requests == []


def test_bad_download_keeps_failure_evidence_and_no_published_cache(setup):
    config, payloads, _ = setup
    payloads['pytorch_model.bin']=b'invalid'
    with pytest.raises(ValueError,match='weight'):
        fetch.fetch_model(config)
    assert not fetch._cache_directory().exists()
    failures=list(fetch._cache_directory().parent.glob('.download-*/download-failure.json'))
    assert len(failures)==1
    assert json.loads(failures[0].read_text())['training'] is False


def test_rejects_wrong_native_class_inventory(setup):
    config, payloads, _ = setup
    data=json.loads(payloads['config.json']);data['id2label']['0']='background'
    payloads['config.json']=json.dumps(data).encode()
    with pytest.raises(ValueError,match='19-class'):
        fetch.fetch_model(config)
    assert not fetch._cache_directory().exists()


def test_check_cli_never_downloads(setup,monkeypatch,capsys):
    config, _, requests = setup
    path=fetch.ROOT/'config.json';path.write_text(json.dumps(config))
    monkeypatch.chdir(fetch.ROOT)
    assert fetch.main(['--config',str(path),'--check'])==0
    plan=json.loads(capsys.readouterr().out)
    assert plan['revision']==fetch.REVISION
    assert len(plan['files'])==3 and requests==[]
    assert not fetch._cache_directory().exists()
