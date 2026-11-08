"""Calibrated camera-mask projection onto timed WORLD LiDAR returns.

This module uses only NumPy and does not start a simulator or learner. The
camera-to-world transform uses USD's row-vector convention; the camera looks
down local -Z with +Y up. Input XYZ is already WORLD and is never transformed
through the LiDAR mount a second time. Both timestamps must use the same clock.

Fusion does not accept scene IDs, simulator object IDs or rendered depth. A
separate evaluator may compare its labels against those independent oracles.
The sparse LiDAR z-buffer rejects sampled foreground occlusion only: it cannot
prove visibility where the camera sees a foreground surface absent from LiDAR.
Timestamp rejection is not motion compensation or moving-object deskewing.
"""
from __future__ import annotations

import math
from numbers import Integral, Real
from typing import Mapping

import numpy as np


REASONS = (
    'accepted', 'invalid_return', 'nonfinite_point', 'invalid_time',
    'future_return', 'time_skew', 'outside_depth_range', 'outside_image', 'sparse_occlusion',
    'semantic_boundary', 'unknown_label', 'low_confidence',
)
REASON_IDS = {name: index for index, name in enumerate(REASONS)}
FUTURE_TOLERANCE_S = 1e-5


def _number(value, name, *, minimum=None, maximum=None):
    if (not isinstance(value, Real) or isinstance(value, (bool, np.bool_))
            or not math.isfinite(float(value))
            or minimum is not None and value < minimum
            or maximum is not None and value > maximum):
        raise ValueError('Invalid ' + name)
    return float(value)


def _integer(value, name, *, minimum=0, maximum=None):
    if (not isinstance(value, Integral) or isinstance(value, (bool, np.bool_))
            or value < minimum or maximum is not None and value > maximum):
        raise ValueError('Invalid ' + name)
    return int(value)


def pinhole_calibration(width, height, focal_length, horizontal_aperture,
                        vertical_aperture, *, horizontal_aperture_offset=0.,
                        vertical_aperture_offset=0., near_m=.1, far_m=250.,
                        projection='perspective', distortion_model='none'):
    """Build raster-edge intrinsics from live USD camera attributes.

    Focal length and apertures must use the same units. Coordinates (u,v) are
    measured from the image's top-left edge; floor selects the containing pixel.
    Lens distortion is intentionally unsupported, including fisheye projection.
    """
    if projection != 'perspective' or distortion_model != 'none':
        raise ValueError('Only undistorted perspective cameras are supported')
    width = _integer(width, 'width', minimum=1)
    height = _integer(height, 'height', minimum=1)
    focal_length = _number(focal_length, 'focal length', minimum=np.finfo(float).tiny)
    horizontal_aperture = _number(horizontal_aperture, 'horizontal aperture', minimum=np.finfo(float).tiny)
    vertical_aperture = _number(vertical_aperture, 'vertical aperture', minimum=np.finfo(float).tiny)
    horizontal_aperture_offset = _number(horizontal_aperture_offset, 'horizontal aperture offset')
    vertical_aperture_offset = _number(vertical_aperture_offset, 'vertical aperture offset')
    result = dict(model='pinhole', width=width, height=height,
        fx=width*focal_length/horizontal_aperture,
        fy=height*focal_length/vertical_aperture,
        cx=width*(.5-horizontal_aperture_offset/horizontal_aperture),
        cy=height*(.5+vertical_aperture_offset/vertical_aperture),
        near_m=near_m, far_m=far_m, distortion_model='none',
        pixel_convention='top-left raster edge; floor selects containing pixel')
    _calibration(result, (height, width))
    return result


def _calibration(calibration: Mapping, shape):
    if not isinstance(calibration, Mapping) or calibration.get('model') != 'pinhole':
        raise ValueError('Explicit pinhole calibration is required')
    if calibration.get('distortion_model', 'none') != 'none':
        raise ValueError('Distorted camera projection is unsupported')
    width = _integer(calibration.get('width'), 'width', minimum=1)
    height = _integer(calibration.get('height'), 'height', minimum=1)
    if shape != (height, width):
        raise ValueError('Semantic mask and calibration dimensions disagree')
    values = {key: _number(calibration.get(key), key) for key in ('fx', 'fy', 'cx', 'cy')}
    if values['fx'] <= 0 or values['fy'] <= 0:
        raise ValueError('Focal lengths must be positive')
    values['near_m'] = _number(calibration.get('near_m', .1), 'near plane', minimum=0.)
    values['far_m'] = _number(calibration.get('far_m', 250.), 'far plane')
    if values['far_m'] <= values['near_m']:
        raise ValueError('Depth range must have positive extent')
    return dict(width=width, height=height, **values)


def _camera_transform(value):
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('Camera transform must be a finite 4x4 USD row matrix')
    if not np.allclose(matrix[:, 3], [0, 0, 0, 1], rtol=0, atol=1e-7):
        raise ValueError('Camera transform must use affine USD row-vector convention')
    rotation = matrix[:3, :3]
    if (not np.allclose(rotation @ rotation.T, np.eye(3), rtol=0, atol=1e-5)
            or not np.isclose(np.linalg.det(rotation), 1., rtol=0, atol=1e-5)):
        raise ValueError('Camera transform must be rigid without scale or reflection')
    return matrix


def _semantic_interior(mask, margin):
    if margin == 0:
        return np.ones(mask.shape, dtype=bool)
    # Sentinel cannot occur in an accepted mask. Image edges are conservative
    # boundaries too, since pixels immediately outside the frame are unknown.
    padded = np.pad(mask.astype(np.int64), margin, constant_values=-2)
    height, width = mask.shape
    interior = np.ones(mask.shape, dtype=bool)
    for dy in range(2*margin+1):
        for dx in range(2*margin+1):
            interior &= padded[dy:dy+height, dx:dx+width] == mask
    return interior


def fuse_semantic_lidar(xyz_world, semantic_mask, camera_to_world, calibration,
                        *, point_times_s, image_time_s, flags=None, valid_returns=None,
                        max_time_delta_s=.05, occlusion_cell_px=2,
                        occlusion_tolerance_m=.15, boundary_margin_px=1,
                        min_confidence=0., confidence=None):
    """Return per-input-point class labels plus explicit rejection reasons.

    ``flags`` uses RTX GMO's successful-hit bit 64. Alternatively callers may
    provide an explicit boolean ``valid_returns`` mask. Supplying neither means
    that every row is a declared valid return; zero XYZ is not silently a hit.
    Provide at most one of these arguments. WORLD NONCOMPENSATED coordinates
    represent each ray's acquisition time; this function only projects them
    into the image-time camera pose and gates temporal skew, without pretending
    to correct movement of the hit object between those times.
    Returns acquired after the image are rejected, apart from a fixed 10us
    numeric tolerance; the permitted age does not grant future observations.

    Unknown/rejected labels are -1, distinct from valid class 0 (background).
    Confidence is the supplied camera class confidence, not an independently
    calibrated fusion probability. When absent, assigned mask labels carry 1.
    An occlusion cell uses the nearest timely LiDAR depth as its sparse z-buffer.
    """
    xyz = np.asarray(xyz_world, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError('WORLD points must have shape (N,3)')
    count = len(xyz)
    mask = np.asarray(semantic_mask)
    if (mask.ndim != 2 or mask.dtype.kind not in 'iu' or mask.size == 0
            or np.any(mask < -1) or np.any(mask > np.iinfo(np.int32).max)):
        raise ValueError('Semantic mask must contain integer class IDs or -1 unknown')
    intrinsic = _calibration(calibration, mask.shape)
    matrix = _camera_transform(camera_to_world)
    image_time_s = _number(image_time_s, 'image time', minimum=0.)
    max_time_delta_s = _number(max_time_delta_s, 'maximum time delta', minimum=0.)
    occlusion_tolerance_m = _number(occlusion_tolerance_m, 'occlusion tolerance', minimum=0.)
    occlusion_cell_px = _integer(occlusion_cell_px, 'occlusion cell', minimum=1, maximum=64)
    boundary_margin_px = _integer(boundary_margin_px, 'boundary margin', minimum=0, maximum=16)
    min_confidence = _number(min_confidence, 'minimum confidence', minimum=0., maximum=1.)
    times = np.asarray(point_times_s, dtype=np.float64)
    if times.shape != (count,):
        raise ValueError('Point times must have shape (N,) in the image clock')
    if flags is not None and valid_returns is not None:
        raise ValueError('Supply flags or explicit valid returns, not both')
    if flags is not None:
        flags = np.asarray(flags)
        if (flags.shape != (count,) or flags.dtype.kind not in 'iu'
                or np.any(flags < 0) or np.any(flags > np.iinfo(np.uint32).max)):
            raise ValueError('RTX flags must be uint32-compatible and match points')
        hits = (flags.astype(np.uint32) & 64) != 0
    elif valid_returns is not None:
        hits = np.asarray(valid_returns)
        if hits.shape != (count,) or hits.dtype.kind != 'b':
            raise ValueError('Explicit valid returns must be a boolean vector')
    else:
        hits = np.ones(count, dtype=bool)
    if confidence is None:
        confidence_image = None
    else:
        confidence_image = np.asarray(confidence, dtype=np.float64)
        if (confidence_image.shape != mask.shape or not np.isfinite(confidence_image).all()
                or np.any(confidence_image < 0) or np.any(confidence_image > 1)):
            raise ValueError('Camera confidence must be finite [0,1] and match the mask')

    # Row convention: world = camera @ R + t; inverse rotation is R.T.
    with np.errstate(invalid='ignore', divide='ignore', over='ignore'):
        camera_xyz = (xyz-matrix[3, :3]) @ matrix[:3, :3].T
        depth = -camera_xyz[:, 2]
        uv = np.column_stack((intrinsic['fx']*camera_xyz[:, 0]/depth+intrinsic['cx'],
                              -intrinsic['fy']*camera_xyz[:, 1]/depth+intrinsic['cy']))
        time_delta = times-image_time_s
    finite_xyz = np.isfinite(xyz).all(axis=1) & np.isfinite(camera_xyz).all(axis=1)
    finite_time = np.isfinite(times) & (times >= 0)
    causal = finite_time & (time_delta <= FUTURE_TOLERANCE_S)
    timely = causal & (np.abs(time_delta) <= max_time_delta_s+1e-9)
    depth_valid = np.isfinite(depth) & (depth > 0) & (depth >= intrinsic['near_m']) & (depth <= intrinsic['far_m'])
    inside = (np.isfinite(uv).all(axis=1) & (uv[:, 0] >= 0) & (uv[:, 0] < intrinsic['width'])
              & (uv[:, 1] >= 0) & (uv[:, 1] < intrinsic['height']))
    projectable = hits & finite_xyz & depth_valid & inside
    eligible = projectable & timely
    reason = np.zeros(count, dtype=np.uint8)

    def reject(condition, name):
        reason[(reason == 0) & condition] = REASON_IDS[name]

    reject(~hits, 'invalid_return')
    reject(~finite_xyz, 'nonfinite_point')
    reject(~finite_time, 'invalid_time')
    reject(~causal, 'future_return')
    reject(~timely, 'time_skew')
    reject(~depth_valid, 'outside_depth_range')
    reject(~inside, 'outside_image')
    indices = np.flatnonzero(eligible)
    pixels = np.floor(uv[indices]).astype(np.int64)
    cell_width = (intrinsic['width']+occlusion_cell_px-1)//occlusion_cell_px
    cell_height = (intrinsic['height']+occlusion_cell_px-1)//occlusion_cell_px
    cells = pixels[:, 1]//occlusion_cell_px*cell_width + pixels[:, 0]//occlusion_cell_px
    nearest = np.full(cell_width*cell_height, np.inf)
    np.minimum.at(nearest, cells, depth[indices])
    obscured = depth[indices] > nearest[cells]+occlusion_tolerance_m
    reason[indices[obscured]] = REASON_IDS['sparse_occlusion']
    interior = _semantic_interior(mask, boundary_margin_px)
    candidate_labels = mask[pixels[:, 1], pixels[:, 0]].astype(np.int32)
    at_boundary = ~interior[pixels[:, 1], pixels[:, 0]]
    active = reason[indices] == 0
    reason[indices[active & at_boundary]] = REASON_IDS['semantic_boundary']
    active = reason[indices] == 0
    reason[indices[active & (candidate_labels < 0)]] = REASON_IDS['unknown_label']
    candidate_confidence = (np.ones(len(indices)) if confidence_image is None
                            else confidence_image[pixels[:, 1], pixels[:, 0]])
    active = reason[indices] == 0
    reason[indices[active & (candidate_confidence < min_confidence)]] = REASON_IDS['low_confidence']
    selected = reason[indices] == 0
    labels = np.full(count, -1, dtype=np.int32)
    label_confidence = np.zeros(count, dtype=np.float32)
    labels[indices[selected]] = candidate_labels[selected]
    label_confidence[indices[selected]] = candidate_confidence[selected]
    accepted = reason == REASON_IDS['accepted']
    counts = np.bincount(reason, minlength=len(REASONS))
    class_values, class_counts = np.unique(labels[accepted], return_counts=True)
    stats = dict(input_points=count, successful_returns=int(hits.sum()),
        geometrically_projectable_points=int(projectable.sum()),
        projectable_timely_points=int(eligible.sum()), accepted_points=int(accepted.sum()),
        accepted_fraction_of_projectable=float(accepted.sum()/projectable.sum()) if projectable.any() else 0.,
        accepted_fraction_of_timely_projectable=float(accepted.sum()/eligible.sum()) if eligible.any() else 0.,
        reason_counts={name: int(counts[index]) for index, name in enumerate(REASONS)},
        class_counts={str(int(value)): int(n) for value, n in zip(class_values, class_counts)},
        maximum_accepted_time_delta_s=float(np.abs(time_delta[accepted]).max()) if accepted.any() else None,
        settings=dict(max_time_delta_s=max_time_delta_s, occlusion_cell_px=occlusion_cell_px,
                      occlusion_tolerance_m=occlusion_tolerance_m, boundary_margin_px=boundary_margin_px,
                      min_confidence=min_confidence, future_tolerance_s=FUTURE_TOLERANCE_S),
        input_frame='WORLD NONCOMPENSATED', camera_transform='USD row-vector camera-to-world',
        pixel_convention='top-left raster edge; floor selects containing pixel',
        object_ids_used=False, rendered_depth_used=False, moving_object_compensation=False,
        visibility_limit='Sparse LiDAR occlusion only; unseen foreground and object motion may remain ambiguous')
    return dict(labels=labels, uv=uv, camera_depth_m=depth, accepted=accepted,
                projectable=projectable, timely_projectable=eligible,
                reason=reason, confidence=label_confidence, time_delta_s=time_delta, stats=stats)
