import importlib.util
import json
from pathlib import Path

import pytest

PATH=Path(__file__).resolve().parents[1]/'skills/organize-av-evidence/scripts/build_catalog.py'
spec=importlib.util.spec_from_file_location('evidence_catalog',PATH)
catalog=importlib.util.module_from_spec(spec);spec.loader.exec_module(catalog)


def activity():
    return dict(id='a',title='A test question',domain='Test fixture',question='Did the change help?',
        why_now='An earlier bounded test failed.',trigger=dict(text='Explicit fixture request',sources=['repo:note.md']),
        method='Matched synthetic test',status='development',result='Not a scientific result',limits='Test data only',
        next_action='Review fixture',references=['repo:note.md'],prefixes=['repo:outputs/run'],artifacts=[dict(
            path='repo:outputs/run/figure.svg',generator='repo:plot.py',inputs=['repo:outputs/run/metrics.json'],
            explanation='A <script> label remains text.',claim='Fixture only',limits='Not evidence of traffic effects',lineage_basis='recorded')])


@pytest.fixture
def fixture(tmp_path):
    repo=tmp_path/'repo';project=tmp_path/'project';repo.mkdir();project.mkdir()
    (repo/'outputs/run').mkdir(parents=True)
    (repo/'note.md').write_text('Question and trigger')
    (repo/'plot.py').write_text('# fixture')
    (repo/'outputs/run/figure.svg').write_text('<svg/>')
    (repo/'outputs/run/metrics.json').write_text('{"pass":true}')
    (repo/'orphan.png').write_bytes(b'fixture, not an actual image')
    registry=dict(schema_version=1,scope=dict(roots={'repo':['.'],'project':[]}),activities=[activity()])
    return repo,project,registry


def test_catalog_links_derivation_and_exposes_unassigned_files(fixture,tmp_path):
    repo,project,registry=fixture
    path=repo/'registry.json';path.write_text(json.dumps(registry))
    coverage=catalog.build(project,repo,tmp_path/'catalog',path)
    rows=json.loads((tmp_path/'catalog/inventory.json').read_text())
    assert next(r for r in rows if r['path']=='outputs/run/figure.svg')['provenance']=='explicit_lineage'
    assert next(r for r in rows if r['path']=='orphan.png')['provenance']=='needs_backfill'
    assert not coverage['historical_backfill_complete']
    document=(tmp_path/'catalog/index.html').read_text()
    encoded=document.split('<script id="data" type="application/json">')[1].split('</script>')[0]
    assert '<script>' not in encoded
    decoded=json.loads(encoded)
    assert next(r for r in decoded if r['path']=='outputs/run/figure.svg')['explanation']=='A <script> label remains text.'
    with pytest.raises(FileExistsError):catalog.build(project,repo,tmp_path/'catalog',path)


def test_missing_inputs_reported_and_escaping_reference_rejected(fixture):
    repo,project,registry=fixture;roots={'repo':repo,'project':project}
    registry['activities'][0]['artifacts'][0]['inputs']=['repo:missing.csv']
    assert catalog.validate(registry,roots)[0]['error']=='missing_artifact_input'
    registry['activities'][0]['artifacts'][0]['inputs']=['repo:../outside.csv']
    with pytest.raises(ValueError):catalog.validate(registry,roots)


def test_raw_sources_are_metadata_only_and_manifest_mismatch_is_visible(fixture,tmp_path):
    repo,project,registry=fixture
    (repo/'outputs/run/source.laz').write_bytes(b'private source')
    (repo/'outputs/run/manifest.json').write_text(json.dumps({'outputs':{'metrics.json':'0'*64}}))
    records,issues,_=catalog.scan(registry,{'repo':repo,'project':project},tmp_path/'out')
    raw=next(r for r in records if r['path'].endswith('.laz'))
    assert raw['sha256'] is None and raw['verification']=='metadata_only'
    assert any(i['error']=='manifest_hash_mismatch' for i in issues)


def test_duplicate_artifact_ids_are_rejected(fixture):
    repo,project,registry=fixture
    registry['activities'][0]['artifacts']*=2
    with pytest.raises(ValueError):catalog.validate(registry,{'repo':repo,'project':project})
