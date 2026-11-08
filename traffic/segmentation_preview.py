"""Offline class mapping, metrics and honest pretrained segmentation previews.

This module imports neither a learner nor a simulator. The pretrained model's
Cityscapes classes overlap only part of the simulator taxonomy. Ground-truth
lane and generic-obstacle pixels are shown explicitly and excluded from the
shared-class score; that exclusion is not evidence the model recognized them.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np


PROJECT_CLASSES = ('background', 'drivable', 'lane', 'vehicle', 'pedestrian', 'obstacle')
CITYSCAPES_LABELS = (
    'road', 'sidewalk', 'building', 'wall', 'fence', 'pole', 'traffic light',
    'traffic sign', 'vegetation', 'terrain', 'sky', 'person', 'rider', 'car',
    'truck', 'bus', 'train', 'motorcycle', 'bicycle',
)
CITYSCAPES_TO_PROJECT = (1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 3, 3, 3, 3, 3, 3)
SUPPORTED_PROJECT_IDS = (0, 1, 3, 4)
UNSUPPORTED_GT_IDS = (2, 5)
PALETTE = np.asarray(((35, 40, 50), (70, 180, 90), (250, 230, 60),
    (60, 140, 250), (230, 80, 180), (245, 125, 40)), dtype=np.uint8)
MATCH_COLOR = (42, 145, 85)
ERROR_COLOR = (225, 55, 65)
UNSUPPORTED_COLOR = (245, 180, 45)
METRIC_SCOPE = ('Pixel metrics on GT background/drivable/vehicle/pedestrian only; '
    'GT lane and generic-obstacle pixels are unsupported and excluded. '
    'Background is a coarse aggregation of other Cityscapes categories, including rider. '
    'This is saved-frame domain-transfer preview, not driving acceptance.')


def _mask(value, name, allowed):
    array = np.asarray(value)
    if array.ndim != 2 or not array.size or array.dtype.kind not in 'iu':
        raise ValueError(name + ' must be a nonempty 2D integer mask')
    if not np.isin(array, allowed).all():
        raise ValueError(name + ' contains IDs outside its declared taxonomy')
    return array


def map_cityscapes(native_mask):
    """Map original Cityscapes train IDs; never guess unknown/void categories."""
    native = _mask(native_mask, 'Cityscapes prediction', range(len(CITYSCAPES_LABELS)))
    return np.asarray(CITYSCAPES_TO_PROJECT, dtype=np.uint8)[native]


def _ratio(numerator, denominator):
    return float(numerator / denominator) if denominator else None


def _summarize(confusion, total_pixels, unsupported_counts):
    confusion = np.asarray(confusion, dtype=np.int64)
    evaluated = int(confusion.sum())
    per_class = {}
    for class_id, name in enumerate(PROJECT_CLASSES):
        if class_id in UNSUPPORTED_GT_IDS:
            per_class[name] = dict(supported=False, gt_pixels=int(unsupported_counts.get(name, 0)),
                predicted_pixels=None, true_positive=None, false_positive=None,
                false_negative=None, iou=None, recall=None, precision=None)
            continue
        truth = int(confusion[class_id, :].sum())
        predicted = int(confusion[:, class_id].sum())
        correct = int(confusion[class_id, class_id])
        union = truth + predicted - correct
        per_class[name] = dict(supported=True, gt_pixels=truth, predicted_pixels=predicted,
            true_positive=correct, false_positive=predicted-correct,
            false_negative=truth-correct, iou=_ratio(correct, union),
            recall=_ratio(correct, truth), precision=_ratio(correct, predicted))
    ious = [row['iou'] for row in per_class.values() if row['iou'] is not None]
    return dict(total_pixels=int(total_pixels), evaluated_pixels=evaluated,
        unsupported_gt_pixels=int(sum(unsupported_counts.values())),
        unsupported_gt_by_class={name:int(unsupported_counts.get(name,0))
                                 for name in ('lane','obstacle')},
        evaluated_fraction=_ratio(evaluated,total_pixels),
        correct_pixels=int(np.trace(confusion)),
        accuracy=_ratio(int(np.trace(confusion)),evaluated),
        mean_iou_present_union=float(np.mean(ious)) if ious else None,
        mean_iou_class_count=len(ious), confusion_matrix=confusion.tolist(),
        confusion_order=list(PROJECT_CLASSES), per_class=per_class,
        metric_scope=METRIC_SCOPE)


def metrics(pred_project, gt):
    """Measure shared taxonomy without silently counting unsupported GT as correct."""
    predicted = _mask(pred_project, 'Mapped prediction', SUPPORTED_PROJECT_IDS)
    truth = _mask(gt, 'Ground truth', range(len(PROJECT_CLASSES)))
    if predicted.shape != truth.shape:
        raise ValueError('Prediction and ground truth dimensions disagree')
    evaluated = np.isin(truth, SUPPORTED_PROJECT_IDS)
    confusion = np.zeros((len(PROJECT_CLASSES),len(PROJECT_CLASSES)),dtype=np.int64)
    np.add.at(confusion,(truth[evaluated],predicted[evaluated]),1)
    unsupported = {PROJECT_CLASSES[i]:int(np.count_nonzero(truth==i)) for i in UNSUPPORTED_GT_IDS}
    result = _summarize(confusion,truth.size,unsupported)
    result['prediction_pixels_all_image'] = {PROJECT_CLASSES[i]:int(np.count_nonzero(predicted==i))
                                            for i in SUPPORTED_PROJECT_IDS}
    return result


def aggregate_metrics(frame_metrics):
    """Aggregate pixel counts, not unweighted averages of per-frame scores."""
    if not frame_metrics:
        raise ValueError('At least one frame metric is required')
    matrices = [np.asarray(row['confusion_matrix'],dtype=np.int64) for row in frame_metrics]
    if any(matrix.shape!=(6,6) or np.any(matrix<0) for matrix in matrices):
        raise ValueError('Invalid frame confusion matrix')
    unsupported={name:sum(row['unsupported_gt_by_class'][name] for row in frame_metrics)
                 for name in ('lane','obstacle')}
    result=_summarize(sum(matrices),sum(row['total_pixels'] for row in frame_metrics),unsupported)
    result['frames']=len(frame_metrics)
    result['prediction_pixels_all_image']={name:sum(row['prediction_pixels_all_image'][name]
                                                  for row in frame_metrics)
                                          for name in ('background','drivable','vehicle','pedestrian')}
    return result


def error_rgb(pred_project, gt):
    """Green correct, red mismatch, amber unsupported GT with visible hatching."""
    metrics(pred_project,gt)  # Validate exactly the scoring contract.
    predicted,truth=np.asarray(pred_project),np.asarray(gt)
    unsupported=np.isin(truth,UNSUPPORTED_GT_IDS)
    rgb=np.empty((*truth.shape,3),dtype=np.uint8)
    rgb[:]=MATCH_COLOR
    rgb[(predicted!=truth)&~unsupported]=ERROR_COLOR
    rgb[unsupported]=UNSUPPORTED_COLOR
    y,x=np.indices(truth.shape)
    rgb[unsupported&((x+y)%12<3)]=(115,85,25)
    return rgb


def _font(size):
    from PIL import ImageFont
    try:
        return ImageFont.load_default(size=size)
    except TypeError:  # Compatibility with older Pillow; no external font download.
        for name in ('DejaVuSans.ttf','arial.ttf'):
            try:
                return ImageFont.truetype(name,size)
            except OSError:
                pass
        return ImageFont.load_default()


def render_preview(path, rgb, pred_project, gt, frame_metrics, title, *,
                   model_label='SegFormer B0 / Cityscapes19'):
    """Save four horizontal native-size panels; never alter the source RGB/masks."""
    from PIL import Image, ImageDraw
    truth=np.asarray(gt);predicted=np.asarray(pred_project);rgb=np.asarray(rgb)
    measured=metrics(predicted,truth)
    if frame_metrics != measured:
        raise ValueError('Displayed metrics do not match these prediction/GT masks')
    if rgb.dtype!=np.uint8 or rgb.shape!=(*truth.shape,3):
        raise ValueError('RGB must be uint8 H x W x 3 matching the masks')
    if not isinstance(title,str) or not title:
        raise ValueError('Preview title is required')
    if not isinstance(model_label,str) or not model_label:
        raise ValueError('Model label is required')
    path=Path(path)
    if path.exists():
        raise FileExistsError('Preserve existing preview: '+str(path))
    h,w=truth.shape
    header=120;footer=160
    canvas=Image.new('RGB',(4*w,h+header+footer),(20,24,32))
    draw=ImageDraw.Draw(canvas)
    font=_font(20);small=_font(17);title_font=_font(24)
    draw.text((14,12),title,fill='white',font=title_font)
    draw.text((14,40),model_label+' | native Cityscapes19 -> shared project classes; frozen model',
              fill=(185,195,210),font=small)
    titles=('RGB | saved capture','Predicted mask | pretrained','Ground truth | simulator','Errors | shared classes only')
    subtitles=('Frozen input; no new simulator run','Mapped Cityscapes classes; no training',
               'Includes lane and generic obstacle','Amber hatch = unsupported GT, excluded')
    panels=(rgb,PALETTE[predicted],PALETTE[truth],error_rgb(predicted,truth))
    for i,(name,subtitle,panel) in enumerate(zip(titles,subtitles,panels)):
        x=i*w
        draw.text((x+14,65),name,fill='white',font=font)
        draw.text((x+14,91),subtitle,fill=(185,195,210),font=small)
        canvas.paste(Image.fromarray(panel),(x,header))
        if i:draw.line((x,50,x,h+header),fill=(90,100,110),width=2)
    y=h+header+12
    def percentage(value):return 'not evaluated' if value is None else f'{100*value:.2f}%'
    draw.text((14,y),f"Shared-class accuracy {percentage(measured['accuracy'])} | mean IoU {percentage(measured['mean_iou_present_union'])} | evaluated pixels {measured['evaluated_pixels']:,}/{measured['total_pixels']:,}",fill='white',font=font)
    draw.text((14,y+32),'Mask colors: dark other/background / green drivable / yellow lane / blue vehicle / pink pedestrian / orange obstacle',fill=(205,215,225),font=small)
    draw.text((14,y+60),'Error colors: green correct / red wrong / amber hatch unsupported lane or obstacle (excluded from scores)',fill=(205,215,225),font=small)
    draw.text((14,y+88),'Background groups several Cityscapes classes. Unsupported classes and absent objects are not evidence of recognition.',fill=(205,215,225),font=small)
    draw.text((14,y+116),'Inference-only saved-frame preview; no training, new simulation or driving-policy change.',fill=(205,215,225),font=small)
    canvas.save(path)
    return dict(path=str(path),width=canvas.width,height=canvas.height)
