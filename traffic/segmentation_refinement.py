"""Inference-only image transforms and boundary diagnostics on saved captures.

Image preparation and logit restoration never accept ground truth. Metric
functions are evaluation-only. No simulator, learner or model download occurs.
"""
from __future__ import annotations

import math
import numpy as np

VARIANTS = ('square512', 'letterbox512', 'letterbox1024')
PADDING_RGB = (124, 116, 104)  # Rounded ImageNet mean * 255.


def prepare_rgb(rgb, variant):
    """Return prepared uint8 RGB and invertible spatial-transform metadata.

    square512 reproduces the original stretched 512-square processor input.
    letterbox512 preserves aspect in a centered 512-square canvas.
    letterbox1024 scales the longest side to 1024, then centers in the smallest
    canvas with both dimensions divisible by 32. PIL bilinear matches the
    pretrained processor's RGB resize. Normalization remains the caller's job.
    """
    from PIL import Image
    array = np.asarray(rgb)
    if array.dtype != np.uint8 or array.ndim != 3 or array.shape[2] != 3 or not array.size:
        raise ValueError('RGB must be nonempty uint8 H x W x 3')
    if variant not in VARIANTS:
        raise ValueError('Unknown preprocessing variant')
    h, w = array.shape[:2]
    if variant == 'square512':
        rh = rw = ch = cw = 512
    else:
        side = 512 if variant == 'letterbox512' else 1024
        scale = side / max(h, w)
        rh, rw = max(1, round(h * scale)), max(1, round(w * scale))
        ch, cw = (512, 512) if side == 512 else (math.ceil(rh / 32) * 32, math.ceil(rw / 32) * 32)
    top, left = (ch-rh)//2, (cw-rw)//2
    resized = np.asarray(Image.fromarray(array).resize((rw, rh), Image.Resampling.BILINEAR))
    canvas = np.empty((ch, cw, 3), dtype=np.uint8)
    canvas[:] = PADDING_RGB
    canvas[top:top+rh, left:left+rw] = resized
    transform = dict(variant=variant, original_hw=[h,w], resized_hw=[rh,rw],
        canvas_hw=[ch,cw], padding_tlbr=[top,left,ch-rh-top,cw-rw-left],
        padding_rgb=list(PADDING_RGB), resize='PIL bilinear',
        inverse='bilinear logits to canvas; remove padding; bilinear logits to original; caller argmax')
    return canvas, transform


def restore_logits(logits, transform):
    """Restore logits before argmax, removing padding in input-canvas space."""
    import torch
    import torch.nn.functional as functional
    if not isinstance(logits, torch.Tensor) or logits.ndim != 4 or logits.shape[0] != 1 or not logits.is_floating_point():
        raise ValueError('Expected single-image floating [1,C,H,W] logits')
    if any(n < 1 for n in logits.shape) or not torch.isfinite(logits).all():
        raise ValueError('Logits must be nonempty and finite')
    if transform.get('variant') not in VARIANTS:
        raise ValueError('Unknown transform variant')
    h,w = transform['original_hw']; rh,rw = transform['resized_hw']
    ch,cw = transform['canvas_hw']; top,left,bottom,right = transform['padding_tlbr']
    values = [h,w,rh,rw,ch,cw,top,left,bottom,right]
    if any(type(v) is not int for v in values) or min(h,w,rh,rw,ch,cw)<1 or min(top,left,bottom,right)<0:
        raise ValueError('Invalid transform dimensions')
    if top+rh+bottom != ch or left+rw+right != cw:
        raise ValueError('Padding and dimensions disagree')
    # Preserve exact v01 baseline: avoid an extra interpolation through 512.
    if top == left == bottom == right == 0:
        return functional.interpolate(logits, size=(h,w), mode='bilinear', align_corners=False)
    canvas = functional.interpolate(logits, size=(ch,cw), mode='bilinear', align_corners=False)
    unpadded = canvas[:,:,top:top+rh,left:left+rw]
    return functional.interpolate(unpadded, size=(h,w), mode='bilinear', align_corners=False)


def _boundary(mask):
    """Foreground-side 4-neighbor boundary; image border alone is not an edge."""
    result = np.zeros_like(mask, dtype=bool)
    result[1:] |= mask[1:] & ~mask[:-1]
    result[:-1] |= mask[:-1] & ~mask[1:]
    result[:,1:] |= mask[:,1:] & ~mask[:,:-1]
    result[:,:-1] |= mask[:,:-1] & ~mask[:,1:]
    return result


def _dilate_disk(mask, radius):
    h,w = mask.shape; padded = np.pad(mask, radius); result = np.zeros_like(mask)
    for dy in range(-radius,radius+1):
        for dx in range(-radius,radius+1):
            if dx*dx+dy*dy <= radius*radius:
                result |= padded[radius+dy:radius+dy+h,radius+dx:radius+dx+w]
    return result


def _ratio(a,b):
    return float(a/b) if b else None


def _boundary_scores(counts):
    result = dict(counts)
    precision = _ratio(counts['matched_prediction_boundary_pixels'],counts['prediction_boundary_pixels'])
    recall = _ratio(counts['matched_truth_boundary_pixels'],counts['truth_boundary_pixels'])
    result.update(precision=precision, recall=recall,
        f1=(2*precision*recall/(precision+recall) if precision+recall else 0.)
        if precision is not None and recall is not None else None)
    return result


def _road_scores(counts):
    result = dict(counts)
    result.update(false_drivable_fraction_of_evaluated_prediction=_ratio(counts['false_drivable_pixels'],counts['predicted_road_pixels_excluding_gt_lane']),
        false_drivable_rate_on_nonroad=_ratio(counts['false_drivable_pixels'],counts['nonroad_gt_pixels']),
        obstacle_pixels_predicted_road_fraction=_ratio(counts['obstacle_pixels_predicted_road'],counts['obstacle_gt_pixels']))
    return result


def boundary_metrics(pred_project, gt, tolerance_px=2):
    """Boundary and road-spill metrics including unsupported obstacle errors.

    GT road extent combines road + lane markings, so markings inside a road
    don't create artificial road-edge errors. Vehicle evaluation includes all
    image classes as negatives. Tolerance is Euclidean distance in original
    pixels. Each boundary is matched independently (not one-to-one). Scores
    with an absent boundary denominator are null, never perfect by default.
    """
    from traffic.segmentation_preview import metrics
    metrics(pred_project, gt)  # Validate the existing explicit taxonomy.
    if type(tolerance_px) is not int or not 0 <= tolerance_px <= 16:
        raise ValueError('Boundary tolerance must be an integer in [0,16]')
    pred,truth = np.asarray(pred_project),np.asarray(gt)
    per_class = {}
    for name,p_mask,g_mask in [('road',pred==1,np.isin(truth,(1,2))),('vehicle',pred==3,truth==3)]:
        pb,gb = _boundary(p_mask),_boundary(g_mask)
        per_class[name] = _boundary_scores(dict(prediction_boundary_pixels=int(pb.sum()),
            truth_boundary_pixels=int(gb.sum()),
            matched_prediction_boundary_pixels=int((pb&_dilate_disk(gb,tolerance_px)).sum()),
            matched_truth_boundary_pixels=int((gb&_dilate_disk(pb,tolerance_px)).sum())))
    nonroad = ~np.isin(truth,(1,2))
    road = _road_scores(dict(false_drivable_pixels=int(((pred==1)&nonroad).sum()),
        predicted_road_pixels_excluding_gt_lane=int(((pred==1)&(truth!=2)).sum()),
        nonroad_gt_pixels=int(nonroad.sum()), obstacle_gt_pixels=int((truth==5).sum()),
        obstacle_pixels_predicted_road=int(((truth==5)&(pred==1)).sum())))
    return dict(tolerance_px=tolerance_px, per_class=per_class, false_drivable=road,
        scope='Original-resolution foreground-side internal 4-neighbor boundaries; Euclidean tolerance; GT road includes lane; image border alone excluded; obstacle errors included in road spill')


def aggregate_boundary_metrics(rows):
    """Pool raw boundary/area counts across frames, not per-frame scores."""
    if not rows or len({row['tolerance_px'] for row in rows}) != 1:
        raise ValueError('Nonempty rows with a common tolerance required')
    boundary_keys = ('prediction_boundary_pixels','truth_boundary_pixels',
        'matched_prediction_boundary_pixels','matched_truth_boundary_pixels')
    road_keys = ('false_drivable_pixels','predicted_road_pixels_excluding_gt_lane',
        'nonroad_gt_pixels','obstacle_gt_pixels','obstacle_pixels_predicted_road')
    return dict(frames=len(rows), tolerance_px=rows[0]['tolerance_px'], scope=rows[0]['scope'],
        per_class={name:_boundary_scores({key:sum(row['per_class'][name][key] for row in rows)
            for key in boundary_keys}) for name in ('road','vehicle')},
        false_drivable=_road_scores({key:sum(row['false_drivable'][key] for row in rows) for key in road_keys}))
