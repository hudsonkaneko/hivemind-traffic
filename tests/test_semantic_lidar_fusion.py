"""Known geometry tests for fusion; no simulator, oracle labels or training."""
import numpy as np
import pytest

from traffic.semantic_lidar_fusion import (
    REASON_IDS, fuse_semantic_lidar, pinhole_calibration,
)


def calibration():
    return pinhole_calibration(100, 60, 10, 20, 12)


def fuse(points, mask=None, *, times=None, **kwargs):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if mask is None:
        mask = np.full((60, 100), 3, dtype=np.uint8)
    if times is None:
        times = np.full(len(points), 1.)
    return fuse_semantic_lidar(points, mask, np.eye(4), calibration(),
        point_times_s=times, image_time_s=1., **kwargs)


def test_known_pixel_geometry_and_raster_class_sampling():
    mask = np.full((60, 100), 1, dtype=np.uint8)
    mask[:, 55:] = 3
    result = fuse([[0, 0, -10], [2, 1, -10]], mask)
    np.testing.assert_allclose(result['uv'], [[50, 30], [60, 25]])
    np.testing.assert_array_equal(result['labels'], [1, 3])
    np.testing.assert_array_equal(result['camera_depth_m'], [10, 10])
    assert result['stats']['accepted_points'] == 2
    assert result['stats']['class_counts'] == {'1': 1, '3': 1}


def test_world_returns_use_rigid_usd_row_transform_once():
    # Camera is translated and rotated 90 degrees. Forward local -Z becomes
    # world +X. Retaining world XYZ avoids applying the LiDAR mount twice.
    matrix = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [4, 2, 3, 1]], dtype=float)
    local = np.array([[0, 0, -10], [2, 1, -10]], dtype=float)
    world = local @ matrix[:3, :3] + matrix[3, :3]
    result = fuse_semantic_lidar(world, np.full((60, 100), 3, np.uint8), matrix,
        calibration(), point_times_s=np.ones(2), image_time_s=1.)
    np.testing.assert_allclose(result['uv'], [[50, 30], [60, 25]])
    assert result['accepted'].all()


def test_aperture_offsets_and_distortion_rejection():
    c = pinhole_calibration(100, 60, 10, 20, 12,
        horizontal_aperture_offset=1, vertical_aperture_offset=1)
    assert c['fx'] == c['fy'] == 50
    assert c['cx'] == 45 and c['cy'] == 35
    for kwargs in ({'projection': 'orthographic'}, {'distortion_model': 'fisheye'}):
        with pytest.raises(ValueError, match='undistorted perspective'):
            pinhole_calibration(100, 60, 10, 20, 12, **kwargs)


def test_missing_hits_nonfinite_points_depth_and_field_of_view_are_rejected():
    points = [[0, 0, -10], [np.nan, 0, -10], [0, 0, 10], [0, 0, -.01],
              [0, 0, -300], [30, 0, -10], [0, 0, -10]]
    result = fuse(points, flags=np.array([0, 64, 64, 64, 64, 64, 64], dtype=np.uint32))
    expected = ['invalid_return', 'nonfinite_point', 'outside_depth_range',
                'outside_depth_range', 'outside_depth_range', 'outside_image', 'accepted']
    assert result['reason'].tolist() == [REASON_IDS[name] for name in expected]
    assert result['stats']['reason_counts']['accepted'] == 1
    assert sum(result['stats']['reason_counts'].values()) == len(points)
    assert result['projectable'].sum() == 1


def test_timestamps_reject_stale_invalid_and_noncausal_long_skew():
    result = fuse([[0, 0, -10]]*5, times=[.95, .949, np.nan, -1, 1.1],
                  max_time_delta_s=.05)
    expected = ['accepted', 'time_skew', 'invalid_time', 'invalid_time', 'future_return']
    assert result['reason'].tolist() == [REASON_IDS[name] for name in expected]
    assert result['stats']['geometrically_projectable_points'] == 5
    assert result['stats']['projectable_timely_points'] == 1
    assert result['stats']['maximum_accepted_time_delta_s'] == pytest.approx(.05)


def test_small_future_skew_cannot_use_the_past_age_budget():
    result = fuse([[0, 0, -10]]*3, times=[1.000001, 1.0001, 1.04], max_time_delta_s=.05)
    assert result['reason'].tolist() == [REASON_IDS['accepted'], REASON_IDS['future_return'], REASON_IDS['future_return']]
    assert result['time_delta_s'][2] == pytest.approx(.04)


def test_sparse_occlusion_uses_nearest_lidar_depth_not_oracle_depth():
    # First/second hit the same pixel; the further point is hidden. The third
    # differs by only 5cm, within the declared tolerance, and remains eligible.
    result = fuse([[0, 0, -5], [0, 0, -10], [0, 0, -5.05]])
    assert result['accepted'].tolist() == [True, False, True]
    assert result['reason'][1] == REASON_IDS['sparse_occlusion']
    assert result['stats']['rendered_depth_used'] is False
    assert result['stats']['object_ids_used'] is False


def test_stale_foreground_does_not_occlude_current_return():
    result = fuse([[0, 0, -5], [0, 0, -10]], times=[.5, 1.])
    assert result['accepted'].tolist() == [False, True]
    assert result['reason'][0] == REASON_IDS['time_skew']


def test_boundary_unknown_class_and_confidence_have_distinct_reasons():
    mask = np.full((60, 100), 3, np.int32)
    mask[:, 50:] = 5
    at_boundary = fuse([[0, 0, -10]], mask)
    assert at_boundary['reason'][0] == REASON_IDS['semantic_boundary']
    unknown = fuse([[0, 0, -10]], np.full((60, 100), -1, np.int32))
    assert unknown['reason'][0] == REASON_IDS['unknown_label']
    low = fuse([[0, 0, -10]], confidence=np.full((60, 100), .3), min_confidence=.5)
    assert low['reason'][0] == REASON_IDS['low_confidence']
    background = fuse([[0, 0, -10]], np.zeros((60, 100), np.uint8))
    assert background['labels'][0] == 0 and background['accepted'][0]


def test_output_keeps_identity_and_does_not_mutate_inputs():
    points = np.array([[0, 0, -10], [2, 1, -10], [30, 0, -10]], dtype=float)
    mask = np.full((60, 100), 5, np.uint8)
    before_points, before_mask = points.copy(), mask.copy()
    first, second = fuse(points, mask), fuse(points, mask)
    np.testing.assert_array_equal(points, before_points)
    np.testing.assert_array_equal(mask, before_mask)
    for key in ('labels', 'uv', 'camera_depth_m', 'reason', 'accepted'):
        np.testing.assert_array_equal(first[key], second[key])
    assert first['labels'].tolist() == [5, 5, -1]


def test_empty_cloud_has_complete_zero_count_result():
    result = fuse([])
    assert result['labels'].shape == (0,)
    assert result['uv'].shape == (0, 2)
    assert result['stats']['accepted_points'] == 0
    assert sum(result['stats']['reason_counts'].values()) == 0


@pytest.mark.parametrize('bad', [np.ones((3, 3)), np.diag([2, 1, 1, 1]),
                                 np.diag([-1, 1, 1, 1]),
                                 np.eye(4).T+np.array([[0,0,0,1],[0,0,0,0],[0,0,0,0],[0,0,0,0]])])
def test_invalid_transform_fails_closed(bad):
    with pytest.raises(ValueError, match='Camera transform'):
        fuse_semantic_lidar([[0, 0, -10]], np.zeros((60, 100), np.uint8), bad,
            calibration(), point_times_s=[1.], image_time_s=1.)


@pytest.mark.parametrize('kwargs', [dict(flags=[1.5]), dict(flags=[-1]),
    dict(valid_returns=[1]), dict(flags=[64], valid_returns=[True]),
    dict(max_time_delta_s=-1), dict(boundary_margin_px=-1), dict(occlusion_cell_px=0),
    dict(confidence=np.full((60, 100), np.nan)), dict(min_confidence=2)])
def test_malformed_configuration_or_return_contract_fails_closed(kwargs):
    with pytest.raises(ValueError):
        fuse([[0, 0, -10]], **kwargs)


def test_mask_dimension_and_integer_class_contract_are_checked():
    for mask in (np.zeros((10, 10), np.uint8), np.zeros((60, 100), float),
                 np.full((60, 100), -2, np.int32)):
        with pytest.raises(ValueError):
            fuse([[0, 0, -10]], mask)
