"""CPU-only, fail-safe LiDAR emergency stop using a short forward corridor.

This is a geometric point-cloud filter, not recognition, localization, planning,
or a learned policy. It receives no obstacle labels, object IDs, or simulator
obstacle positions. Points must already be expressed relative to the vehicle's
chassis in metres: +X forward, +Y left, +Z up. Frame conversion and timestamp
acquisition are the caller's responsibility, not inferred here.

At 3 m/s, the fresh-scan default stopping corridor is 4.6 m beyond the front
bumper: v²/(2*1.5) + v*0.2 + 1.0. Older samples extend this by speed times the
age of the oldest point. The planning deceleration is deliberately conservative
relative to the prior ~0.67 m 3 m/s braking fixture, but is not a guarantee of
stopping distance on different tires, roads, hardware or sensor settings.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real

import numpy as np

VEHICLE_FRAME = 'vehicle_local_chassis_x_forward_y_left_z_up_m'


def _finite(value):
    try:
        return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)
    except (TypeError, ValueError, OverflowError):
        return False


def _tick(value):
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def _identity(value):
    return isinstance(value, str) and bool(value.strip())


@dataclass(frozen=True)
class LidarScan:
    """Untrusted scan delivery; times are episode-local physics ticks.

    ``points`` must be a nonempty finite N×3 sequence (lists or array rows).
    Acquisition bounds describe the oldest/newest samples in this cloud;
    ``delivery_tick`` does not renew their age. Coordinates are chassis-relative
    at ``coordinate_reference_tick`` (defaults to acquisition_end_tick), never
    world coordinates or unconverted sensor-local points. A caller transforming
    WORLD returns using current odometry must set this reference tick explicitly.
    The frozen wrapper does not freeze an underlying array: callers must not
    mutate a scan while it is being evaluated.
    """
    episode_id: str
    vehicle_id: str
    acquisition_start_tick: int
    acquisition_end_tick: int
    delivery_tick: int
    points: object
    frame: str = VEHICLE_FRAME
    coordinate_reference_tick: int | None = None


@dataclass(frozen=True)
class LidarBrakeConfig:
    physics_hz: int = 120
    max_speed_m_s: float = 3.0
    front_bumper_offset_m: float = 2.4
    half_width_m: float = 0.9
    lateral_margin_m: float = 0.25
    # Relative to the chassis origin (~1 m above the ground), not the sensor.
    # This excludes the flat ground; low curbs and overhanging hazards are not
    # validated by this simple vertical slab and require a broader perception task.
    min_height_m: float = -0.6
    max_height_m: float = 0.6
    planning_deceleration_m_s2: float = 1.5
    reaction_margin_s: float = 0.2
    stopping_margin_m: float = 1.0
    max_scan_age_ticks: int = 24
    max_scan_span_ticks: int = 24
    max_points: int = 500_000

    def __post_init__(self):
        if not _tick(self.physics_hz) or not 1 <= self.physics_hz <= 1000:
            raise ValueError('physics_hz must be an integer in [1,1000]')
        for name in ('max_scan_age_ticks', 'max_scan_span_ticks', 'max_points'):
            value = getattr(self, name)
            if not _tick(value) or value < 1:
                raise ValueError(f'{name} must be a positive integer')
        for name in ('max_speed_m_s', 'front_bumper_offset_m', 'half_width_m',
                     'planning_deceleration_m_s2', 'stopping_margin_m'):
            if not _finite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f'{name} must be finite and positive')
        for name in ('lateral_margin_m', 'reaction_margin_s'):
            if not _finite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f'{name} must be finite and nonnegative')
        if (not _finite(self.min_height_m) or not _finite(self.max_height_m)
                or self.min_height_m >= self.max_height_m):
            raise ValueError('Vertical filter bounds must be finite and increasing')
        if self.max_speed_m_s > 3:
            raise ValueError('This uncalibrated short-corridor stop fixture is limited to 3 m/s')


@dataclass(frozen=True)
class LidarBrakeDecision:
    status: str  # clear | obstacle | stale_invalid
    reason: str
    target_speed_m_s: float
    brake_override: float
    nearest_gap_m: float | None
    stopping_distance_m: float | None
    oldest_sample_age_s: float | None
    stop_latched: bool
    latched_obstacle_gap_m: float | None
    points_in_forward_corridor: int
    frame: str = VEHICLE_FRAME
    observation_source: str = 'lidar_point_cloud'


class LidarEmergencyBrake:
    """Stop on nearby cloud returns; hold obstacle stops until explicit reset.

    Invalid/missing/empty/future/stale scans fail safe (zero target/full brake).
    A later valid clear scan may recover from an input fault, but NEVER releases
    an obstacle latch. Reset belongs to a new physical episode or an explicitly
    authorized restart, not routine scan delivery. This straight corridor does
    not follow steering curvature; success on a gentle-curve fixture does not
    establish general curved-road collision avoidance.
    """
    def __init__(self, episode_id, vehicle_id, config=None):
        self.config = LidarBrakeConfig() if config is None else config
        if not isinstance(self.config, LidarBrakeConfig):
            raise ValueError('config must be LidarBrakeConfig')
        self.reset(episode_id=episode_id, vehicle_id=vehicle_id)

    def reset(self, *, episode_id, vehicle_id):
        if not _identity(episode_id) or not _identity(vehicle_id):
            raise ValueError('episode_id and vehicle_id must be nonempty strings')
        self.episode_id, self.vehicle_id = episode_id, vehicle_id
        self._last_tick = None
        self._last_acquisition_end = None
        self._stop_latched = False
        self._latched_gap = None

    def _invalid(self, reason):
        return LidarBrakeDecision('stale_invalid', reason, 0.0, 1.0, None, None,
                                  None, self._stop_latched, self._latched_gap, 0)

    def evaluate(self, scan, *, tick, speed_m_s, requested_target_speed_m_s=3.0):
        """Evaluate once per advancing control tick; scans may be held while fresh."""
        if not _tick(tick) or (self._last_tick is not None and tick <= self._last_tick):
            return self._invalid('invalid_local_clock')
        self._last_tick = tick
        cfg = self.config
        if not _finite(speed_m_s) or not 0 <= speed_m_s <= cfg.max_speed_m_s:
            return self._invalid('invalid_or_excessive_speed')
        if (not _finite(requested_target_speed_m_s)
                or not 0 <= requested_target_speed_m_s <= cfg.max_speed_m_s):
            return self._invalid('invalid_target_speed')
        if not isinstance(scan, LidarScan):
            return self._invalid('missing_or_invalid_scan')
        if scan.episode_id != self.episode_id or scan.vehicle_id != self.vehicle_id:
            return self._invalid('identity_mismatch')
        if scan.frame != VEHICLE_FRAME:
            return self._invalid('wrong_coordinate_frame')
        start, end, delivery = scan.acquisition_start_tick, scan.acquisition_end_tick, scan.delivery_tick
        if not all(_tick(v) for v in (start, end, delivery)):
            return self._invalid('invalid_scan_timestamp')
        if not start <= end <= delivery <= tick:
            return self._invalid('future_or_misordered_scan')
        reference_tick = end if scan.coordinate_reference_tick is None else scan.coordinate_reference_tick
        if not _tick(reference_tick) or not end <= reference_tick <= tick:
            return self._invalid('invalid_coordinate_reference_tick')
        if end-start > cfg.max_scan_span_ticks:
            return self._invalid('excessive_acquisition_span')
        # Bound the oldest point, not merely the newest return. Otherwise a
        # full acquisition window could silently double the configured age.
        if tick-start > cfg.max_scan_age_ticks:
            return self._invalid('stale_scan')
        if self._last_acquisition_end is not None and end < self._last_acquisition_end:
            return self._invalid('replayed_scan')
        points = scan.points
        if isinstance(points, (str, bytes, dict)):
            return self._invalid('invalid_point_cloud_shape')
        try:
            count = len(points)
        except (TypeError, ValueError):
            return self._invalid('invalid_point_cloud_shape')
        if count == 0:
            return self._invalid('empty_point_cloud')
        if count > cfg.max_points:
            return self._invalid('excessive_point_count')
        nearest, corridor_count = None, 0
        try:
            # RTX packets arrive as float arrays. Preserve the scalar validation
            # path for arbitrary inputs (including mixed booleans/objects).
            if (type(points) is np.ndarray and points.ndim == 2
                    and points.shape[1] == 3
                    and points.dtype in (np.dtype('float32'), np.dtype('float64'))):
                if not np.isfinite(points).all():
                    return self._invalid('nonfinite_or_invalid_point')
                cloud = points.astype(np.float64, copy=False)
                mask = ((cloud[:, 0] > cfg.front_bumper_offset_m)
                        & (np.abs(cloud[:, 1]) <= cfg.half_width_m+cfg.lateral_margin_m)
                        & (cloud[:, 2] >= cfg.min_height_m)
                        & (cloud[:, 2] <= cfg.max_height_m))
                corridor_count = int(np.count_nonzero(mask))
                if corridor_count:
                    nearest = float(np.min(cloud[mask, 0])-cfg.front_bumper_offset_m)
                scalar_points = ()
            else:
                scalar_points = points
            for point in scalar_points:
                if isinstance(point, (str, bytes, dict)) or len(point) != 3:
                    return self._invalid('invalid_point_cloud_shape')
                if not all(_finite(v) for v in point):
                    return self._invalid('nonfinite_or_invalid_point')
                x, y, z = (float(v) for v in point)
                # Exclude own-body returns, but do NOT introduce an additional
                # blind gap beyond the bumper that could hide a close obstacle.
                if (x > cfg.front_bumper_offset_m
                        and abs(y) <= cfg.half_width_m+cfg.lateral_margin_m
                        and cfg.min_height_m <= z <= cfg.max_height_m):
                    gap = x-cfg.front_bumper_offset_m
                    nearest = gap if nearest is None else min(nearest, gap)
                    corridor_count += 1
        except (TypeError, ValueError, OverflowError):
            return self._invalid('invalid_point_cloud_shape')
        self._last_acquisition_end = end
        # Retain the oldest-return age conservatively even when caller odometry
        # has transformed historical WORLD returns into the current ego frame.
        oldest_age = (tick-start)/cfg.physics_hz
        stopping_distance = (float(speed_m_s)**2/(2*cfg.planning_deceleration_m_s2)
                             + float(speed_m_s)*(cfg.reaction_margin_s+oldest_age)
                             + cfg.stopping_margin_m)
        detected = nearest is not None and nearest <= stopping_distance
        if detected and not self._stop_latched:
            self._stop_latched = True
            self._latched_gap = nearest
        stopped = self._stop_latched
        return LidarBrakeDecision(
            'obstacle' if stopped else 'clear',
            'obstacle_in_stopping_corridor' if detected else ('latched_obstacle_stop' if stopped else 'corridor_clear'),
            0.0 if stopped else float(requested_target_speed_m_s), 1.0 if stopped else 0.0,
            nearest, stopping_distance, oldest_age, stopped, self._latched_gap, corridor_count)
