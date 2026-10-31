"""Synthetic metric/display checks; no model, simulator or training invocation."""
import numpy as np
import pytest
from PIL import Image

from traffic.segmentation_preview import (CITYSCAPES_LABELS, CITYSCAPES_TO_PROJECT,
    map_cityscapes, metrics, aggregate_metrics, error_rgb, render_preview)


def test_explicit_cityscapes_mapping_preserves_pretrained_head_contract():
    native=np.arange(19,dtype=np.uint8).reshape(1,19)
    assert len(CITYSCAPES_LABELS)==19
    assert map_cityscapes(native).tolist()==[list(CITYSCAPES_TO_PROJECT)]
    assert map_cityscapes(native)[0,0]==1
    assert map_cityscapes(native)[0,13]==3
    assert map_cityscapes(native)[0,11]==4
    with pytest.raises(ValueError):map_cityscapes(np.array([[255]],dtype=np.uint8))
    with pytest.raises(ValueError):map_cityscapes(np.array([[1.0]]))


def test_unsupported_ground_truth_never_counts_as_recognized():
    gt=np.array([[1,3,2,5]],dtype=np.uint8)
    pred=np.array([[1,0,0,0]],dtype=np.uint8)
    result=metrics(pred,gt)
    assert result['total_pixels']==4 and result['evaluated_pixels']==2
    assert result['unsupported_gt_pixels']==2
    assert result['accuracy']==.5
    assert result['per_class']['vehicle']['recall']==0
    assert result['per_class']['obstacle']['iou'] is None
    assert result['per_class']['lane']['supported'] is False
    assert result['per_class']['pedestrian']['iou'] is None
    # False background prediction is part of shared-class IoU's union.
    assert result['mean_iou_present_union']==pytest.approx(1/3)


def test_all_unsupported_has_no_score_not_perfect_accuracy():
    result=metrics(np.zeros((2,2),dtype=np.uint8),np.full((2,2),5,dtype=np.uint8))
    assert result['accuracy'] is None and result['mean_iou_present_union'] is None
    assert result['evaluated_pixels']==0 and result['unsupported_gt_pixels']==4


def test_aggregation_uses_pixel_counts_instead_of_frame_average():
    good=metrics(np.ones((1,1),dtype=np.uint8),np.ones((1,1),dtype=np.uint8))
    bad=metrics(np.zeros((1,9),dtype=np.uint8),np.ones((1,9),dtype=np.uint8))
    result=aggregate_metrics([good,bad])
    assert result['accuracy']==.1
    assert result['per_class']['drivable']['recall']==.1
    assert result['frames']==2


def test_error_image_distinguishes_wrong_from_unsupported():
    gt=np.array([[1,3,2,5]],dtype=np.uint8);pred=np.array([[1,0,0,0]],dtype=np.uint8)
    image=error_rgb(pred,gt)
    assert image[0,0,1]>image[0,0,0]  # Green correct.
    assert image[0,1,0]>image[0,1,1]  # Red error.
    assert not np.array_equal(image[0,1],image[0,2])
    assert not np.array_equal(image[0,1],image[0,3])


@pytest.mark.parametrize('pred,gt',[
    (np.array([[2]],dtype=np.uint8),np.array([[2]],dtype=np.uint8)),
    (np.array([[0]],dtype=np.uint8),np.array([[6]],dtype=np.uint8)),
    (np.array([[0]],dtype=np.uint8),np.zeros((2,2),dtype=np.uint8)),
])
def test_rejects_unmapped_predictions_or_bad_dimensions(pred,gt):
    with pytest.raises(ValueError):metrics(pred,gt)


def test_render_four_panels_preserves_sources_and_refuses_misleading_metrics(tmp_path):
    rgb=np.full((32,64,3),90,dtype=np.uint8)
    gt=np.ones((32,64),dtype=np.uint8);gt[:,48:]=5
    pred=np.ones_like(gt);before=rgb.copy();scores=metrics(pred,gt)
    path=tmp_path/'preview.png'
    rendered=render_preview(path,rgb,pred,gt,scores,'Synthetic display contract test')
    assert rendered['width']==4*64
    assert Image.open(path).size==(256,312)
    np.testing.assert_array_equal(rgb,before)
    with pytest.raises(FileExistsError):render_preview(path,rgb,pred,gt,scores,'Duplicate')
    wrong=dict(scores,accuracy=0.)
    with pytest.raises(ValueError,match='metrics'):render_preview(tmp_path/'bad.png',rgb,pred,gt,wrong,'Bad')
