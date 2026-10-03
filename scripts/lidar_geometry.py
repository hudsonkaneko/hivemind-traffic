"""Pure NumPy ground truth for the controlled lidar validation fixture."""
import numpy as np


def directions(azimuth_deg, elevation_deg):
    az, el = np.radians(azimuth_deg), np.radians(elevation_deg)
    return np.column_stack((np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)))


def ray_box_ranges(origins, rays, centers, size):
    """Slab intersection; return infinity for misses, including parallel rays."""
    lo, hi = centers - np.asarray(size) / 2, centers + np.asarray(size) / 2
    parallel = np.abs(rays) < 1e-12
    outside = np.any(parallel & ((origins < lo) | (origins > hi)), axis=1)
    safe = np.where(parallel, 1.0, rays)
    a, b = (lo - origins) / safe, (hi - origins) / safe
    near = np.max(np.where(parallel, -np.inf, np.minimum(a, b)), axis=1)
    far = np.min(np.where(parallel, np.inf, np.maximum(a, b)), axis=1)
    hit = (~outside) & (far >= np.maximum(near, 0))
    return np.where(hit, np.where(near >= 0, near, far), np.inf)


def fixture_positions(times, config, moving=True):
    times = np.asarray(times)
    origins = np.tile(config['sensor_mount_m'], (times.size, 1)).astype(float)
    centers = np.tile(config['target_initial_center_m'], (times.size, 1)).astype(float)
    if moving:
        origins[:, 0] += times * config['ego_velocity_mps']
        centers[:, 0] += times * config['target_velocity_mps']
    return origins, centers


def analyze_scan(scan, config, moving=True):
    """Select rays by angle and expected geometry, NEVER by measured range error."""
    az, el = scan['azimuth_deg'], scan['elevation_deg']
    times = (scan['timestamp_ns'] + scan['offset_ns'].astype(np.float64)) * 1e-9 - config.get('sensor_to_usd_offset_s', 0)
    origins, centers = fixture_positions(times, config, moving)
    rays = directions(az, el)
    expected = ray_box_ranges(origins, rays, centers, config['target_size_m'])
    roi = (np.abs(az) < config['roi_azimuth_deg']) & (np.abs(el) < config['roi_elevation_deg'])
    selected = roi & np.isfinite(expected)
    valid = np.isfinite(scan['range_m']) & (scan['range_m'] > 0) & ((scan['flags'] & 64) != 0)
    observed = selected & valid
    errors = np.abs(scan['range_m'][observed] - expected[observed])
    return dict(expected_hits=int(selected.sum()), detected_hits=int(observed.sum()),
                detection_fraction=float(observed.sum() / max(1, selected.sum())),
                direction_consistency_fraction=float(observed.sum() / max(1, (roi & valid).sum())),
                range_p95_m=float(np.percentile(errors, 95)) if errors.size else None,
                range_max_m=float(errors.max()) if errors.size else None), errors


def summarize(records, config, moving=True, errors=()):
    """Frozen acceptance gates; first configured scans are startup warmup."""
    usable = records[config['warmup_scans']:]
    enough = len(usable) >= config['min_scans']
    geometry = enough and all(
        r['expected_hits'] >= config['min_expected_hits_per_scan'] and
        r['detection_fraction'] >= config['min_detection_fraction'] and
        r['direction_consistency_fraction'] >= config['min_detection_fraction'] and
        r['range_p95_m'] is not None and r['range_p95_m'] <= config['range_p95_tolerance_m'] and
        r['range_max_m'] <= config['range_max_tolerance_m'] for r in usable)
    timestamps = np.array([r['timestamp_ns'] for r in usable], dtype=float) * 1e-9
    cadence_errors = np.abs(np.diff(timestamps) - config['scan_period_s'])
    pose_errors = []
    for r in usable:
        for end in ['start', 'end']:
            t = r[f'frame_{end}_ns'] * 1e-9 - config['sensor_to_usd_offset_s']
            expected, _ = fixture_positions(np.array([t]), config, moving)
            pose_errors.append(float(np.linalg.norm(np.array(r[f'frame_{end}_position']) - expected[0])))
    timing = enough and bool(np.all(np.diff(timestamps) > 0)) and bool(np.all(cadence_errors <= 1e-7)) and all(
        r['offset_min_ns'] >= 0 and r['offset_max_ns'] <= config['scan_period_s'] * 1e9 and
        r['coordinate_type'] == 'CoordsType.SPHERICAL' and r['reference'] == 'FrameOfReference.SENSOR' and
        r['motion_compensation'] == 'MotionCompensationState.NONCOMPENSATED' and
        abs(r['frame_end_ns'] * 1e-9 - (r['timestamp_ns'] * 1e-9 + config['scan_period_s'])) <= 1e-7
        for r in usable) and max(pose_errors, default=float('inf')) <= config['pose_tolerance_m']
    return dict(passed=bool(geometry and timing and not errors), geometry_passed=bool(geometry),
                timing_validated=bool(timing), scans=len(records), evaluated_scans=len(usable), errors=list(errors),
                worst_scan_p95_m=max((r['range_p95_m'] for r in usable if r['range_p95_m'] is not None), default=None),
                worst_range_error_m=max((r['range_max_m'] for r in usable if r['range_max_m'] is not None), default=None),
                max_sensor_pose_error_m=max(pose_errors, default=None),
                max_cadence_error_s=float(cadence_errors.max()) if cadence_errors.size else None,
                max_callback_frame_end_skew_s=max((abs(r['callback_time_s'] - r['frame_end_ns'] * 1e-9) for r in usable), default=None),
                scope='Controlled two-box translation fixture; per-return ray/box range and direction consistency',
                limitation='Returned rays only: not full firing-pattern recall, full traffic replay, or physics validation.')
