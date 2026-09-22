"""Independent recorded-trajectory reference for the block-car USD replay."""
import numpy as np
try:
    from .lidar_geometry import directions, ray_box_ranges
except ImportError:
    from lidar_geometry import directions, ray_box_ranges


def rotate(vectors, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.column_stack((c*vectors[:, 0]-s*vectors[:, 1],
                            s*vectors[:, 0]+c*vectors[:, 1], vectors[:, 2]))


class ReplayReference:
    def __init__(self, recording, config):
        self.config = config
        frames = recording['frames']
        self.times = np.array([f['time']-frames[0]['time'] for f in frames])
        self.ids = sorted({v['id'] for f in frames for v in f['vehicles']})
        self.tracks = {}
        for vehicle_id in self.ids:
            samples = [(i, v) for i, f in enumerate(frames) for v in f['vehicles'] if v['id'] == vehicle_id]
            indices = [i for i, _ in samples]
            visible = np.zeros(len(frames), dtype=bool)
            visible[indices] = True
            self.tracks[vehicle_id] = dict(times=self.times[indices],
                x=np.array([v['x'] for _, v in samples]), y=np.array([v['y'] for _, v in samples]),
                # Match the existing exporter's scalar rotateZ interpolation, including its limitations.
                yaw=np.radians([90-v['heading'] for _, v in samples]), visible=visible,
                length=samples[0][1]['length'], width=samples[0][1]['width'])

    def pose(self, vehicle_id, times):
        times = np.asarray(times)
        track = self.tracks[vehicle_id]
        position = np.column_stack((np.interp(times, track['times'], track['x']),
                                    np.interp(times, track['times'], track['y']), np.zeros(times.size)))
        yaw = np.interp(times, track['times'], track['yaw'])
        index = np.clip(np.searchsorted(self.times, times, side='right')-1, 0, len(self.times)-1)
        return position, yaw, track['visible'][index]

    def sensor_pose(self, times):
        position, yaw, visible = self.pose(self.config['ego_id'], times)
        mount = np.tile(self.config['sensor_mount_m'], (len(times), 1))
        return position + rotate(mount, yaw), yaw, visible

    def expected(self, scan, time_shift=0.0, reverse_angles=False):
        times = (float(scan['timestamp_ns']) + scan['offset_ns'].astype(float))*1e-9
        times += time_shift - self.config['sensor_to_usd_offset_s']
        origin, yaw, ego_visible = self.sensor_pose(times)
        az = -scan['azimuth_deg'] if reverse_angles else scan['azimuth_deg']
        rays = rotate(directions(az, scan['elevation_deg']), yaw)
        nearest = np.full(len(times), np.inf)
        labels = np.full(len(times), -1, dtype=np.int16)
        for label, vehicle_id in enumerate(self.ids):
            pos, angle, visible = self.pose(vehicle_id, times)
            track = self.tracks[vehicle_id]
            local_origin = rotate(origin-pos, -angle)
            local_ray = rotate(rays, -angle)
            center = np.tile([-track['length']/2, 0, self.config['body_center_height_m']], (len(times), 1))
            distance = ray_box_ranges(local_origin, local_ray, center,
                                       [track['length'], track['width'], self.config['body_height_m']])
            hit = visible & ego_visible & (distance < nearest)
            nearest[hit], labels[hit] = distance[hit], label
        return nearest, labels

    def analyze(self, scan, time_shift=0.0, reverse_angles=False):
        expected, labels = self.expected(scan, time_shift, reverse_angles)
        valid = np.isfinite(scan['range_m']) & (scan['range_m'] > 0) & ((scan['flags'] & 64) != 0)
        selected = np.isfinite(expected)
        observed = selected & valid
        errors = np.abs(scan['range_m'][observed]-expected[observed])
        union = (selected | (scan['object_label'] >= 0)) & valid
        agreement = union & (labels == scan['object_label'])
        matched = observed & (labels == scan['object_label'])
        matched_errors = np.abs(scan['range_m'][matched]-expected[matched])
        return dict(expected_hits=int(selected.sum()), observed_hits=int(observed.sum()),
                    identity_checks=int(union.sum()), identity_matches=int(agreement.sum()),
                    p95_m=float(np.percentile(errors, 95)) if errors.size else None,
                    max_m=float(errors.max()) if errors.size else None,
                    outliers_over_20cm=int((errors>.2).sum()),
                    matched_identity_p95_m=float(np.percentile(matched_errors,95)) if matched_errors.size else None,
                    vehicle_hits={name:int((observed & (labels == i)).sum()) for i,name in enumerate(self.ids)}), errors


def summarize(records, config, callback_errors=()):
    rows = records[config['warmup_scans']:]
    coverage = len(rows) >= config['min_evaluated_scans']
    count = sum(r['observed_hits'] for r in rows)
    identities = sum(r['identity_checks'] for r in rows)
    agreement = sum(r['identity_matches'] for r in rows) / max(1, identities)
    vehicles = sorted({v for r in rows for v,n in r['vehicle_hits'].items() if n and v != config['ego_id']})
    geometry = count >= config['min_vehicle_rays'] and len(vehicles) >= config['min_other_vehicles'] and agreement >= config['min_identity_agreement'] and all(
        r['p95_m'] <= config['range_p95_tolerance_m'] and r['max_m'] <= config['range_max_tolerance_m']
        for r in rows if r['p95_m'] is not None)
    ts = np.array([r['timestamp_ns'] for r in rows])*1e-9
    cadence = np.max(np.abs(np.diff(ts)-config['scan_period_s'])) if len(ts)>1 else float('inf')
    pose_error = max((r['pose_error_m'] for r in rows), default=float('inf'))
    timing = cadence <= 1e-7 and pose_error <= config['pose_tolerance_m'] and all(r['format_and_time_valid'] for r in rows)
    return dict(passed=bool(coverage and geometry and timing and not callback_errors),
                geometry_passed=bool(geometry), timing_passed=bool(timing), coverage_passed=bool(coverage),
                scans=len(records), evaluated_scans=len(rows), vehicle_rays=count, other_vehicles=vehicles,
                identity_agreement=agreement, max_pose_error_m=pose_error if np.isfinite(pose_error) else None,
                max_cadence_error_s=float(cadence) if np.isfinite(cadence) else None,
                worst_scan_p95_m=max((r['p95_m'] for r in rows if r['p95_m'] is not None), default=None),
                worst_range_error_m=max((r['max_m'] for r in rows if r['max_m'] is not None), default=None),
                scans_without_vehicle_intersections=sum(r['expected_hits']==0 for r in rows),
                outliers_over_20cm=sum(r.get('outliers_over_20cm',0) for r in rows),
                callback_errors=list(callback_errors))
