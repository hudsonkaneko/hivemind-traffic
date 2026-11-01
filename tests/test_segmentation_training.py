"""CPU-only loss, geometry, manifest and resumability tests; no model training."""
import copy
import hashlib
import json
from pathlib import Path
import random
import numpy as np
import pytest
import torch
from PIL import Image

from traffic.segmentation_training import (CLASSES,prepare_batch,restore_logits,segmentation_loss,
    class_weights_from_counts,six_class_metrics,aggregate_metrics,atomic_checkpoint,rng_state,
    restore_rng,config_fingerprint,lr_multiplier,read_manifest,build_optimizer)
from experiments.train_segmentation import validate_config


def test_exact_normalized_padding_and_joint_flip():
    rgb=np.zeros((360,640,3),np.uint8);rgb[:,320:]=255
    gt=np.zeros((360,640),np.uint8);gt[:,320:]=5
    x,y=prepare_batch([rgb],[gt])
    assert x.shape==(1,3,384,640) and y.shape==(1,360,640)
    assert torch.count_nonzero(x[:,:,:12])==0 and torch.count_nonzero(x[:,:,372:])==0
    assert y[0,0,400]==5 and x[0,0,12,400]>0 and x[0,0,12,0]<0
    class Flip:
        def uniform(self,*args):return 1.
        def random(self):return 0.
    fx,fy=prepare_batch([rgb],[gt],augment=True,rng=Flip())
    assert torch.equal(fy,torch.flip(y,[-1])) and torch.equal(fx,torch.flip(x,[-1]))


def test_restore_removes_padding_before_loss():
    z=torch.zeros(1,6,384,640);z[:,:,12:372]=3;z[:,:,:12]=100;z[:,:,372:]=100
    out=restore_logits(z)
    assert out.shape==(1,6,360,640) and torch.all(out==3)


def test_loss_finite_gradients_and_ignore_independence():
    torch.manual_seed(1)
    target=torch.tensor([[[0,0,1,1,1],[0,2,2,3,3],[4,4,5,5,5],[255,255,5,5,5]]],dtype=torch.long)
    z=torch.randn(1,6,4,5,requires_grad=True)
    loss,parts=segmentation_loss(z,target,np.ones(6))
    loss.backward();assert torch.isfinite(loss) and torch.isfinite(z.grad).all()
    assert torch.all(z.grad[0,:,3,:2]==0)
    altered=z.detach().clone();altered[0,:,3,:2]=1000
    changed,_=segmentation_loss(altered,target,np.ones(6))
    torch.testing.assert_close(changed,loss.detach())
    assert set(parts)=={'ce','dice','boundary','total'}


def test_perfect_logits_beat_wrong_boundaries():
    target=torch.zeros((1,10,10),dtype=torch.long);target[:,3:7,3:7]=5
    good=torch.nn.functional.one_hot(target,6).permute(0,3,1,2).float()*15
    wrong=torch.roll(good,3,dims=3)
    g,gt=segmentation_loss(good,target,np.ones(6));w,wt=segmentation_loss(wrong,target,np.ones(6))
    assert g<w and gt['boundary']<wt['boundary']


def test_no_foreground_dice_zero_and_all_ignore_rejected():
    z=torch.zeros(1,6,5,5,requires_grad=True);target=torch.zeros(1,5,5,dtype=torch.long)
    _,terms=segmentation_loss(z,target,np.ones(6));assert terms['dice']==0 and terms['boundary']==0
    with pytest.raises(ValueError):segmentation_loss(z,target+255,np.ones(6))


def test_class_weights_derived_and_missing_class_rejected():
    weights=class_weights_from_counts([100000,100000,1000,1000,100,100])
    assert weights[4]>weights[2]>weights[0] and min(weights)>=.25 and max(weights)<=4
    with pytest.raises(ValueError):class_weights_from_counts([10,10,10,10,10,0])


def test_six_class_metrics_counts_obstacle_and_lane():
    gt=np.tile(np.arange(6,dtype=np.uint8),(12,2))
    same=aggregate_metrics([six_class_metrics(gt,gt)])
    assert same['mean_iou']==1 and same['selection_score']==1
    bad=gt.copy();bad[gt==5]=1
    scored=aggregate_metrics([six_class_metrics(bad,gt)])
    assert scored['per_class']['obstacle']['recall']==0 and scored['false_drivable_rate']>0
    absent=np.zeros((5,5),np.uint8);a=aggregate_metrics([six_class_metrics(absent,absent)])
    assert a['per_class']['pedestrian']['recall'] is None and a['selection_score'] is None


def test_lane_is_part_of_both_road_boundaries():
    gt=np.zeros((20,20),np.uint8);gt[4:16,4:16]=1;gt[8:10,4:16]=2
    pred=gt.copy();pred[gt==2]=1
    metric=aggregate_metrics([six_class_metrics(pred,gt)])
    assert metric['boundary']['road']['f1']==1


def test_ignore_does_not_create_class_or_boundary_errors():
    gt=np.ones((10,10),np.uint8);gt[:2]=255;pred=np.ones_like(gt);pred[:2]=5
    scored=six_class_metrics(pred,gt)
    assert np.asarray(scored['confusion']).sum()==80 and scored['boundary']['road']['truth']==0


def test_checkpoint_roundtrip_restores_model_optimizer_scheduler_rng(tmp_path):
    # Tiny synthetic fixture only; no pretrained model or task data.
    torch.manual_seed(3);random.seed(3);rng=np.random.default_rng(3)
    model=torch.nn.Linear(2,1);opt=torch.optim.AdamW(model.parameters(),lr=.01);sched=torch.optim.lr_scheduler.LambdaLR(opt,lambda i:1/(1+i))
    def step():
        value=torch.randn(2,2)+float(rng.random())+random.random()
        opt.zero_grad();loss=model(value).square().sum();loss.backward();opt.step();sched.step();return loss.detach()
    step();path=tmp_path/'last.pt'
    atomic_checkpoint(path,dict(model=model.state_dict(),optimizer=opt.state_dict(),scheduler=sched.state_dict(),rng=rng_state(rng),state={'epoch':2,'next_index':16,'order':[2,1,0]}))
    expected=step();expected_state=copy.deepcopy(model.state_dict())
    saved=torch.load(path,weights_only=True);model.load_state_dict(saved['model']);opt.load_state_dict(saved['optimizer']);sched.load_state_dict(saved['scheduler']);restore_rng(saved['rng'],rng)
    actual=step();torch.testing.assert_close(actual,expected)
    for key,value in expected_state.items():torch.testing.assert_close(model.state_dict()[key],value)
    assert not path.with_suffix('.pt.tmp').exists() and saved['state']['next_index']==16


def test_scheduler_endpoints():
    assert lr_multiplier(0)==.1 and lr_multiplier(180)==1
    assert lr_multiplier(3600)==.1 and lr_multiplier(4000)==.1


def test_config_guard_and_fingerprint():
    cfg=json.loads(Path('configs/perception/segmentation_refinement_training_v02.proposed.json').read_text())
    validate_config(cfg)
    with pytest.raises(ValueError,match='disabled'):validate_config(cfg,launch=True)
    changed=copy.deepcopy(cfg);changed['launch_enabled']=True;changed['training_authorized']=True
    assert config_fingerprint(cfg)==config_fingerprint(changed)
    changed['budget']['microbatch_size']=8
    with pytest.raises(ValueError):validate_config(changed)
    assert config_fingerprint(cfg)!=config_fingerprint(changed)


def make_manifest(tmp_path):
    frames=[]
    for i,split in enumerate(('train','validation','test')):
        rgb=np.full((360,640,3),i*50,np.uint8);mask=np.tile(np.arange(640)%6,(360,1)).astype(np.uint8)
        rp=tmp_path/f'{i}.png';mp=tmp_path/f'{i}.npy';Image.fromarray(rgb).save(rp);np.save(mp,mask)
        frames.append(dict(split=split,scene_id=f'scene{i}',rgb_path=rp.name,semantic_path=mp.name,rgb_sha256=hashlib.sha256(rp.read_bytes()).hexdigest(),semantic_sha256=hashlib.sha256(mp.read_bytes()).hexdigest()))
    path=tmp_path/'manifest.json';data=dict(schema_version='segmentation-dataset-v02',classes=CLASSES,frames=frames,training_ready=True)
    path.write_text(json.dumps(data));return path,data


def test_manifest_hash_and_scene_isolation(tmp_path):
    path,data=make_manifest(tmp_path);_,splits,counts=read_manifest(path)
    assert len(splits['train'])==1 and counts['train'].sum()==360*640
    data['frames'][1]['scene_id']='scene0';path.write_text(json.dumps(data))
    with pytest.raises(ValueError,match='crosses'):read_manifest(path)


def test_manifest_rejects_modified_file(tmp_path):
    path,data=make_manifest(tmp_path)
    (tmp_path/data['frames'][0]['rgb_path']).write_bytes(b'changed')
    with pytest.raises(ValueError,match='hash'):read_manifest(path)


def test_manifest_rejects_escape(tmp_path):
    path,data=make_manifest(tmp_path);data['frames'][0]['rgb_path']='../outside.png';path.write_text(json.dumps(data))
    with pytest.raises(ValueError,match='escaping'):read_manifest(path)


def test_optimizer_encoder_decoder_and_no_decay():
    class Tiny(torch.nn.Module):
        def __init__(self):
            super().__init__();self.segformer=torch.nn.Linear(3,3);self.decode_head=torch.nn.Linear(3,6)
    cfg=dict(encoder_learning_rate=3e-5,decoder_and_classifier_learning_rate=3e-4,weight_decay=.01,betas=[.9,.999],epsilon=1e-8)
    model=Tiny();optimizer=build_optimizer(model,cfg)
    assert len(optimizer.param_groups)==4 and {g['lr'] for g in optimizer.param_groups}=={3e-5,3e-4}
    for group in optimizer.param_groups:
        assert all(p.ndim>1 for p in group['params']) if group['weight_decay'] else all(p.ndim==1 for p in group['params'])


def test_gpu_guard_allows_desktop_but_rejects_compute(monkeypatch):
    import experiments.train_segmentation as runner
    monkeypatch.setattr(runner.subprocess,'check_output',lambda command,**kwargs:'' if command[0]=='powershell' else '101, C:/Windows/explorer.exe\n102, C:/Apps/chrome.exe\n')
    assert len(runner.gpu_exclusive_check())==2
    monkeypatch.setattr(runner.subprocess,'check_output',lambda command,**kwargs:'' if command[0]=='powershell' else '101, C:/Runtime/python.exe\n')
    with pytest.raises(RuntimeError,match='compute'):runner.gpu_exclusive_check()


def test_atomic_save_failure_preserves_previous_checkpoint(tmp_path,monkeypatch):
    path=tmp_path/'last.pt';atomic_checkpoint(path,{'complete_update':10})
    original=path.read_bytes()
    def fail(*args,**kwargs):raise OSError('disk full')
    monkeypatch.setattr(torch,'save',fail)
    with pytest.raises(OSError):atomic_checkpoint(path,{'complete_update':11})
    assert path.read_bytes()==original


def test_test_evaluation_rejects_resume_only_checkpoint_before_model(tmp_path,monkeypatch):
    import argparse
    import experiments.train_segmentation as runner
    cfg=json.loads(Path('configs/perception/segmentation_refinement_training_v02.proposed.json').read_text())
    out=tmp_path/'outputs';out.mkdir();data=out/'dataset.json';data.write_text('{}')
    checkpoint=out/'last.pt'
    payload=dict(config_fingerprint=config_fingerprint(cfg),dataset_manifest_sha256=hashlib.sha256(data.read_bytes()).hexdigest(),classes=CLASSES,selection_status='resume_only')
    atomic_checkpoint(checkpoint,payload);monkeypatch.setattr(runner,'ROOT',tmp_path)
    args=argparse.Namespace(evaluate=checkpoint,split='test',device='cpu')
    with pytest.raises(ValueError,match='validation-selected'):runner.evaluate_checkpoint(args,cfg,tmp_path/'config.json',data,{})


def test_configuration_cannot_silently_change_loss_or_augmentation():
    cfg=json.loads(Path('configs/perception/segmentation_refinement_training_v02.proposed.json').read_text())
    cfg['loss']['formula']='CE only'
    with pytest.raises(ValueError,match='Loss'):validate_config(cfg)
    cfg=json.loads(Path('configs/perception/segmentation_refinement_training_v02.proposed.json').read_text())
    cfg['preprocessing']['augmentation']['horizontal_flip_probability']=.9
    with pytest.raises(ValueError,match='Augmentation'):validate_config(cfg)


@pytest.mark.parametrize('corruption',['classes','schema','duplicate_rgb','missing_class'])
def test_manifest_refuses_forged_readiness(tmp_path,corruption):
    path,data=make_manifest(tmp_path)
    if corruption=='classes':data['classes']={'road':0,'background':1}
    elif corruption=='schema':data['schema_version']='untrusted-other-contract'
    elif corruption=='duplicate_rgb':
        row=data['frames'][1];row['rgb_path']=data['frames'][0]['rgb_path'];row['rgb_sha256']=data['frames'][0]['rgb_sha256']
    else:
        row=data['frames'][2];f=tmp_path/row['semantic_path'];np.save(f,np.zeros((360,640),np.uint8));row['semantic_sha256']=hashlib.sha256(f.read_bytes()).hexdigest()
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):read_manifest(path)


@pytest.mark.parametrize('authorized',[False,True])
def test_launch_rejected_before_model_for_disabled_or_unready(tmp_path,monkeypatch,authorized):
    import experiments.train_segmentation as runner
    import traffic.segmentation_training as core
    cfg=json.loads(Path('configs/perception/segmentation_refinement_training_v02.proposed.json').read_text())
    cfg['launch_enabled']=authorized;cfg['training_authorized']=authorized
    config_path=tmp_path/'config.json';config_path.write_text(json.dumps(cfg))
    calls=[]
    monkeypatch.setattr(core,'build_model',lambda *a,**k:calls.append(True))
    monkeypatch.setattr(runner,'ROOT',tmp_path);monkeypatch.chdir(tmp_path)
    with pytest.raises(ValueError,match='ready' if authorized else 'disabled'):
        runner.main(['--config',str(config_path)])
    assert calls==[]


def test_manifest_binding_rejects_changed_reviewed_bytes(tmp_path,monkeypatch):
    import experiments.train_segmentation as runner
    monkeypatch.setattr(runner,'ROOT',tmp_path)
    folder=tmp_path/'outputs';folder.mkdir();manifest=folder/'reviewed.json';manifest.write_text('{"reviewed":true}')
    cfg={'dataset':{'manifest':'outputs/reviewed.json','ready':True,'manifest_sha256':hashlib.sha256(manifest.read_bytes()).hexdigest()}}
    assert runner.resolve_dataset_manifest(cfg)==manifest
    manifest.write_text('{"reviewed":true,"changed":true}')
    with pytest.raises(ValueError,match='differs from reviewed'):runner.resolve_dataset_manifest(cfg)


def test_check_refuses_missing_ready_manifest_without_creating_model(tmp_path,monkeypatch):
    import experiments.train_segmentation as runner
    import traffic.segmentation_training as core
    cfg=json.loads(Path('configs/perception/segmentation_refinement_training_v02.proposed.json').read_text())
    cfg['dataset'].update(ready=True,manifest='outputs/missing.json',manifest_sha256='0'*64)
    config=tmp_path/'config.json';config.write_text(json.dumps(cfg));calls=[]
    monkeypatch.setattr(runner,'ROOT',tmp_path);monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(core,'build_model',lambda *a,**k:calls.append(True))
    with pytest.raises(ValueError,match='missing'):runner.main(['--config',str(config),'--check'])
    assert not calls


def test_ready_dataset_requires_manifest_digest(tmp_path,monkeypatch):
    import experiments.train_segmentation as runner
    monkeypatch.setattr(runner,'ROOT',tmp_path);folder=tmp_path/'outputs';folder.mkdir();(folder/'manifest.json').write_text('{}')
    with pytest.raises(ValueError,match='SHA-256'):runner.resolve_dataset_manifest({'dataset':{'manifest':'outputs/manifest.json','ready':True}})
