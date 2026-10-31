"""Offline spatial-transform and boundary-score tests; no learned model needed."""
import numpy as np
import pytest
from PIL import Image

from traffic.segmentation_refinement import (prepare_rgb, restore_logits,
    boundary_metrics, aggregate_boundary_metrics, PADDING_RGB)


@pytest.mark.parametrize('variant,canvas,resized,padding',[
    ('square512',(512,512),(512,512),(0,0,0,0)),
    ('letterbox512',(512,512),(288,512),(112,0,112,0)),
    ('letterbox1024',(576,1024),(576,1024),(0,0,0,0)),
])
def test_preprocessing_geometry_and_source_preservation(variant,canvas,resized,padding):
    source=np.full((360,640,3),70,dtype=np.uint8); before=source.copy()
    output,t=prepare_rgb(source,variant)
    assert output.shape==(*canvas,3)
    assert tuple(t['resized_hw'])==resized
    assert tuple(t['padding_tlbr'])==padding
    np.testing.assert_array_equal(source,before)
    if variant=='letterbox512':
        np.testing.assert_array_equal(output[0,0],PADDING_RGB)
        np.testing.assert_array_equal(output[112,0],source[0,0])


def test_square_resize_matches_processor_pil_bilinear():
    rgb=np.random.default_rng(5).integers(0,256,(31,53,3),dtype=np.uint8)
    output,_=prepare_rgb(rgb,'square512')
    np.testing.assert_array_equal(output,np.asarray(Image.fromarray(rgb).resize((512,512),Image.Resampling.BILINEAR)))


def test_high_resolution_odd_shape_has_divisible_canvas_and_bounded_aspect_error():
    _,t=prepare_rgb(np.zeros((359,641,3),dtype=np.uint8),'letterbox1024')
    assert all(v%32==0 for v in t['canvas_hw'])
    assert max(t['resized_hw'])==1024
    assert abs(t['resized_hw'][0]-359*1024/641)<=.5


def test_restore_removes_padding_before_resize_and_argmax():
    torch=pytest.importorskip('torch')
    _,t=prepare_rgb(np.zeros((360,640,3),dtype=np.uint8),'letterbox512')
    logits=torch.zeros((1,2,512,512));logits[:,0]=5
    logits[:,1,112:400]=9
    restored=restore_logits(logits,t)
    assert restored.shape==(1,2,360,640)
    assert (restored.argmax(1)==1).all()


def test_baseline_logit_restore_is_exact_single_interpolation():
    torch=pytest.importorskip('torch')
    _,t=prepare_rgb(np.zeros((360,640,3),dtype=np.uint8),'square512')
    logits=torch.randn((1,3,128,128),generator=torch.Generator().manual_seed(1))
    expected=torch.nn.functional.interpolate(logits,size=(360,640),mode='bilinear',align_corners=False)
    assert torch.equal(restore_logits(logits,t),expected)


def test_square512_processor_and_inverse_match_original_pipeline_without_weights():
    torch=pytest.importorskip('torch')
    transformers=pytest.importorskip('transformers')
    # Explicit pinned processor settings; no cache, network, weights or model.
    processor=transformers.SegformerImageProcessor(do_resize=True,
        size={'height':512,'width':512},resample=Image.Resampling.BILINEAR,
        do_rescale=True,rescale_factor=1/255,do_normalize=True,
        image_mean=[.485,.456,.406],image_std=[.229,.224,.225],
        do_reduce_labels=False)
    rgb=np.random.default_rng(41).integers(0,256,(360,640,3),dtype=np.uint8)
    original=processor(images=rgb,return_tensors='pt')['pixel_values']
    prepared,transform=prepare_rgb(rgb,'square512')
    candidate=processor(images=prepared,do_resize=False,return_tensors='pt')['pixel_values']
    assert torch.equal(original,candidate)
    logits=torch.randn((1,19,128,128),generator=torch.Generator().manual_seed(42))
    old=torch.nn.functional.interpolate(logits,size=rgb.shape[:2],mode='bilinear',align_corners=False)
    new=restore_logits(logits,transform)
    assert torch.equal(old,new)
    assert torch.equal(old.argmax(1),new.argmax(1))


def test_identity_boundaries_and_empty_classes_are_honest():
    mask=np.zeros((20,20),dtype=np.uint8);mask[5:15,5:15]=3
    row=boundary_metrics(mask,mask)
    assert row['per_class']['vehicle']['f1']==1
    assert row['per_class']['road']['f1'] is None
    assert row['false_drivable']['false_drivable_fraction_of_evaluated_prediction'] is None


def test_boundary_tolerance_detects_shift_without_counting_distant_match():
    gt=np.zeros((40,40),dtype=np.uint8);gt[10:20,10:20]=3
    pred=np.zeros_like(gt);pred[10:20,12:22]=3
    assert boundary_metrics(pred,gt,2)['per_class']['vehicle']['f1']==1
    assert boundary_metrics(pred,gt,0)['per_class']['vehicle']['f1']<1
    pred[:]=0;pred[25:35,25:35]=3
    assert boundary_metrics(pred,gt,2)['per_class']['vehicle']['f1']==0


def test_road_spill_includes_obstacles_excludes_lane_and_reports_denominators():
    gt=np.array([[1,2,5,0,3]],dtype=np.uint8);pred=np.ones_like(gt)
    scores=boundary_metrics(pred,gt)['false_drivable']
    assert scores['false_drivable_pixels']==3
    assert scores['predicted_road_pixels_excluding_gt_lane']==4
    assert scores['false_drivable_fraction_of_evaluated_prediction']==.75
    assert scores['obstacle_pixels_predicted_road_fraction']==1


def test_lane_stripe_does_not_create_road_extent_boundary():
    gt=np.zeros((20,20),dtype=np.uint8);gt[3:17,3:17]=1;gt[5:15,9:11]=2
    pred=gt.copy();pred[pred==2]=1
    assert boundary_metrics(pred,gt,0)['per_class']['road']['f1']==1


def test_aggregation_pools_counts():
    gt=np.zeros((20,20),dtype=np.uint8);gt[5:15,5:15]=3
    good=boundary_metrics(gt,gt);bad=boundary_metrics(np.zeros_like(gt),gt)
    result=aggregate_boundary_metrics([good,bad])
    assert result['frames']==2
    assert result['per_class']['vehicle']['precision']==1
    assert result['per_class']['vehicle']['recall']==.5
    assert result['per_class']['vehicle']['f1']==pytest.approx(2/3)


def test_reject_invalid_inputs_and_incompatible_aggregate():
    with pytest.raises(ValueError):prepare_rgb(np.zeros((10,10,3)),'square512')
    with pytest.raises(ValueError):prepare_rgb(np.zeros((10,10,3),dtype=np.uint8),'unknown')
    mask=np.zeros((3,3),dtype=np.uint8)
    with pytest.raises(ValueError):boundary_metrics(mask,mask,-1)
    with pytest.raises(ValueError):aggregate_boundary_metrics([])
    with pytest.raises(ValueError):aggregate_boundary_metrics([boundary_metrics(mask,mask,1),boundary_metrics(mask,mask,2)])
