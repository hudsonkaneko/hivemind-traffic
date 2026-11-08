import json
import numpy as np
import pytest
from traffic.segmentation_dataset_v02 import ROOT,scene_plan,canonical_mask,HUMANS

def config():return json.loads((ROOT/'configs/perception/segmentation_dataset_v02.json').read_text())

def test_plan_reproducible_group_split_and_negatives():
    a=scene_plan(config());assert a==scene_plan(config());assert len(a)==180
    assert sum(x['frames'] for x in a)==3600
    assert len(set(x['seed'] for x in a))==180
    assert len(set(HUMANS.values()))==3
    for split,n in [('train',144),('validation',18),('test',18)]:
        rows=[x for x in a if x['split']==split];assert len(rows)==n
        assert any(x['empty_negative'] for x in rows) and any(not x['empty_negative'] for x in rows)
    assert len({x['road_family'] for x in a})==3

def test_smoke_is_not_training_dataset():
    rows=scene_plan(config(),True);assert len(rows)==6 and sum(x['frames'] for x in rows)==12

def test_canonical_semantics_aliases_and_ambiguity():
    raw=np.array([[0,1,2,3]])
    assert canonical_mask(raw,{'0':{'class':'BACKGROUND'},'1':{'class':'car'},'2':{'class':'drivable'},'3':{'class':'pedestrian'}}).tolist()==[[0,3,1,4]]
    with pytest.raises(ValueError):canonical_mask(raw,{'1':{'class':'car,pedestrian'}})

def test_no_training_contract():
    c=config();c['training']=True
    with pytest.raises(ValueError):scene_plan(c)

def test_manifest_refuses_tamper_and_escape(tmp_path):
    from PIL import Image
    from traffic.segmentation_dataset_v02 import validate_manifest,sha256
    Image.fromarray(np.zeros((360,640,3),dtype=np.uint8)).save(tmp_path/'rgb.png')
    np.save(tmp_path/'semantic.npy',np.zeros((360,640),dtype=np.uint8))
    np.save(tmp_path/'instance.npy',np.zeros((360,640),dtype=np.uint32))
    np.save(tmp_path/'depth.npy',np.ones((360,640),dtype=np.float32))
    row=dict(split='train',scene_id='train-000',frame_index=0,human_identity=HUMANS['train'],road_family='constant_curvature',obstacle_family='box_and_cone',perceptual_hash='0'*64)
    for key,suffix in [('rgb','png'),('semantic','npy'),('instance','npy'),('depth','npy')]:row[key+'_path']=key+'.'+suffix;row[key+'_sha256']=sha256(tmp_path/row[key+'_path'])
    result=validate_manifest({'frames':[row]},tmp_path)
    assert not result['training_ready'] and 'missing_class:train' in result['errors']
    row['rgb_sha256']='0'*64
    with pytest.raises(ValueError,match='Hash mismatch'):validate_manifest({'frames':[row]},tmp_path)
    row['rgb_path']='../escaped.png'
    with pytest.raises(ValueError,match='Missing/escaping'):validate_manifest({'frames':[row]},tmp_path)
