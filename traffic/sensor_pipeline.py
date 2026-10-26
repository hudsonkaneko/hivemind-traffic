"""Experimental CPU perception using RGB, chassis point returns and noisy odometry.

This is a low-speed geometric prototype, not semantic perception. Camera lane
evidence is measured in image pixels, not calibrated road metres. All simulation
truth belongs upstream of SeededSensorModel; inference accepts measurements only.
Chassis axes are X forward, Y left, Z up; estimated map axes are arbitrary local XY.
"""
from dataclasses import dataclass
import math
import numpy as np
from traffic.lidar_tracking import LidarTracker


@dataclass(frozen=True)
class SensorHeader:
    sensor_id: str
    frame_id: str
    acquisition_s: float
    delivery_s: float
    calibration_id: str
    valid: bool = True
    episode_id: str = 'sensor-v01'

    def fresh(self, now_s, max_age_s=.2):
        try:
            times_valid = (all(math.isfinite(v) for v in (self.acquisition_s, self.delivery_s, now_s))
                and 0 <= self.acquisition_s <= self.delivery_s <= now_s
                and now_s - self.acquisition_s <= max_age_s)
        except (TypeError, ValueError):
            times_valid = False
        return (self.valid is True and all(isinstance(v, str) and bool(v.strip())
                for v in (self.sensor_id, self.frame_id, self.calibration_id, self.episode_id))
                and times_valid)


@dataclass(frozen=True)
class CameraFrame:
    header: SensorHeader
    rgb: object


@dataclass(frozen=True)
class LidarFrame:
    header: SensorHeader
    points_chassis: object


@dataclass(frozen=True)
class EgoMeasurement:
    header: SensorHeader
    wheel_speed_m_s: float
    yaw_rate_rad_s: float


@dataclass(frozen=True)
class LocalizationFix:
    header: SensorHeader
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class EgoEstimate:
    timestamp_s: float
    x_m: float
    y_m: float
    yaw_rad: float
    speed_m_s: float
    position_uncertainty_m: float


class SeededSensorModel:
    """Explicit truth-to-measurement boundary; noise values are prototype settings.

    Wheel speed sigma .03 m/s; gyro sigma .002 rad/s plus fixed episode bias
    .001 rad/s; initial localization sigma .15 m and yaw sigma .01 rad.
    This does not model GNSS multipath, wheel slip or a complete 6-axis IMU.
    """
    def __init__(self, seed=0, episode_id='sensor-v01'):
        self.rng = np.random.default_rng(seed)
        self.gyro_bias = float(self.rng.normal(0, .001))
        self.episode_id = episode_id

    def localization(self, x_m, y_m, yaw_rad, acquisition_s=0., delivery_s=None):
        if not np.isfinite([x_m, y_m, yaw_rad]).all():
            raise ValueError('Nonfinite localization source')
        return LocalizationFix(SensorHeader('localization', 'estimated_map', acquisition_s,
            acquisition_s if delivery_s is None else delivery_s, 'localization-v1',episode_id=self.episode_id),
            float(x_m+self.rng.normal(0,.15)), float(y_m+self.rng.normal(0,.15)),
            float(yaw_rad+self.rng.normal(0,.01)))

    def telemetry(self, speed_m_s, yaw_rate_rad_s, acquisition_s, delivery_s=None):
        if not np.isfinite([speed_m_s, yaw_rate_rad_s]).all():
            raise ValueError('Nonfinite telemetry source')
        return EgoMeasurement(SensorHeader('wheel_gyro', 'chassis', acquisition_s,
            acquisition_s if delivery_s is None else delivery_s, 'wheel-gyro-v1',episode_id=self.episode_id),
            float(speed_m_s+self.rng.normal(0,.03)),
            float(yaw_rate_rad_s+self.gyro_bias+self.rng.normal(0,.002)))


class EgoEstimator:
    def __init__(self, localization_calibration_id='localization-v1', telemetry_calibration_id='wheel-gyro-v1', episode_id='sensor-v01'):
        self.estimate = None
        self.localization_calibration_id = localization_calibration_id
        self.telemetry_calibration_id = telemetry_calibration_id
        self.episode_id = episode_id

    def initialize(self, fix, now_s=None):
        now = fix.header.delivery_s if now_s is None else now_s
        if (not fix.header.fresh(now) or fix.header.frame_id != 'estimated_map'
                or fix.header.calibration_id != self.localization_calibration_id
                or fix.header.episode_id != self.episode_id
                or not np.isfinite([fix.x_m, fix.y_m, fix.yaw_rad]).all()):
            raise ValueError('Invalid localization fix')
        self.estimate = EgoEstimate(fix.header.acquisition_s, fix.x_m, fix.y_m,
                                    fix.yaw_rad, 0., .15)
        return self.estimate

    def update(self, measurement, now_s=None):
        now = measurement.header.delivery_s if now_s is None else now_s
        if (self.estimate is None or not measurement.header.fresh(now)
                or measurement.header.frame_id != 'chassis'
                or measurement.header.calibration_id != self.telemetry_calibration_id
                or measurement.header.episode_id != self.episode_id
                or not np.isfinite([measurement.wheel_speed_m_s, measurement.yaw_rate_rad_s]).all()):
            raise ValueError('Invalid telemetry or uninitialized estimator')
        old = self.estimate
        dt = measurement.header.acquisition_s-old.timestamp_s
        if not 0 < dt <= .2:
            raise ValueError('Telemetry must increase in time without gaps over .2 s')
        yaw_delta = measurement.yaw_rate_rad_s*dt
        mid_yaw = old.yaw_rad+yaw_delta/2
        distance = measurement.wheel_speed_m_s*dt
        self.estimate = EgoEstimate(measurement.header.acquisition_s,
            old.x_m+distance*math.cos(mid_yaw), old.y_m+distance*math.sin(mid_yaw),
            (old.yaw_rad+yaw_delta+math.pi)%(2*math.pi)-math.pi,
            measurement.wheel_speed_m_s, old.position_uncertainty_m+dt*.03+abs(distance)*.01)
        return self.estimate


@dataclass(frozen=True)
class LaneEvidence:
    valid: bool
    center_offset_pixels: float | None
    confidence: float
    reason: str


def detect_lane_rgb(rgb):
    """Dark road/bright boundary heuristic on lower RGB. No labels or depth.

    Searches outward from a dark central road strip for the nearest neutral bright
    boundary on each side. Bright shoulders outside those boundaries are allowed.
    This is image geometry, not recognition of legal lanes or drivable surfaces.
    Rejects bland and centrally bright frames. Shadows, worn paint, glare,
    junctions and white vehicles can defeat it; confidence is not calibrated.
    """
    rgb = np.asarray(rgb)
    if rgb.ndim != 3 or rgb.shape[2] not in (3,4) or rgb.dtype != np.uint8 or min(rgb.shape[:2]) < 32:
        return LaneEvidence(False,None,0.,'invalid_rgb')
    h,w = rgb.shape[:2]
    color = rgb[int(h*.55):int(h*.9),:,:3].astype(float)
    mask = (color.min(axis=2)>170) & (np.ptp(color,axis=2)<45)
    darkness = color.max(axis=2)<140
    left,right = [],[]
    center = w//2
    central_half_width = max(2,int(w*.02))
    for row,dark in zip(mask,darkness):
        if np.mean(dark[center-central_half_width:center+central_half_width]) < .9:
            continue
        l = np.flatnonzero(row[:w//2]); r = np.flatnonzero(row[w//2:])+w//2
        if len(l) and len(r):
            nearest_left,nearest_right = int(l[-1]),int(r[0])
            span = nearest_right-nearest_left
            # A broad, mostly dark interior is required; isolated bright blobs
            # against a bright/unknown background must not imply a clear lane.
            if (.1*w <= span <= .95*w
                    and np.mean(dark[nearest_left+1:nearest_right]) >= .85):
                left.append(float(nearest_left)); right.append(float(nearest_right))
    fraction = len(left)/max(len(mask),1)
    if fraction < .3:
        return LaneEvidence(False,None,0.,'insufficient_two_boundary_evidence')
    centers = (np.asarray(left)+right)/2
    if np.std(centers)>w*.08:
        return LaneEvidence(False,None,0.,'inconsistent_boundaries')
    return LaneEvidence(True,float(np.median(centers)-w/2),float(fraction),'rgb_dark_road_bright_boundary_heuristic')


@dataclass(frozen=True)
class PerceptionResult:
    timestamp_s: float
    valid: bool
    stop_required: bool
    reasons: tuple
    lane: LaneEvidence
    tracks: tuple
    nearest_forward_gap_m: float | None
    ego: EgoEstimate


class SensorPipeline:
    """Low-speed forward corridor gate; no claim of all-around free space.

    Fresh empty scans are unknown, not clear. Tracker IDs are generated locally.
    Each LiDAR packet is processed once; retries fail closed rather than renew age.
    Ego pose must match acquisition within .05 s; no scan motion compensation.
    """
    def __init__(self, max_age_s=.2, camera_calibration_id='front-camera-v1',
                 lidar_calibration_id='front-lidar-v1', ego_half_length_m=2.6,
                 corridor_half_width_m=1.2, episode_id='sensor-v01'):
        if not math.isfinite(max_age_s) or not 0 < max_age_s <= .2:
            raise ValueError('max_age_s must be in (0,.2]')
        self.max_age_s = max_age_s
        self.camera_calibration_id = camera_calibration_id
        self.lidar_calibration_id = lidar_calibration_id
        if not all(math.isfinite(v) and v > 0 for v in (ego_half_length_m,corridor_half_width_m)):
            raise ValueError('Vehicle corridor dimensions must be finite and positive')
        self.ego_half_length_m = ego_half_length_m
        self.corridor_half_width_m = corridor_half_width_m
        self.tracker = LidarTracker()
        self._last_lidar_s = -1.
        self._obstacle_stop_release_gap = None
        self._last_camera_s = -1.
        self.episode_id = episode_id

    def process(self, camera, lidar, ego, now_s):
        reasons = []
        lane = LaneEvidence(False,None,0.,'unavailable')
        if (not camera.header.fresh(now_s,self.max_age_s)
                or camera.header.frame_id != 'camera_optical'
                or camera.header.calibration_id != self.camera_calibration_id
                or camera.header.episode_id != self.episode_id
                or camera.header.acquisition_s <= self._last_camera_s):
            reasons.append('camera_invalid_or_stale')
        else:
            self._last_camera_s = camera.header.acquisition_s
            lane = detect_lane_rgb(camera.rgb)
            if not lane.valid:
                reasons.append('lane_unknown')
        if (not np.isfinite([ego.timestamp_s,ego.x_m,ego.y_m,ego.yaw_rad,ego.speed_m_s,ego.position_uncertainty_m]).all()
                or ego.position_uncertainty_m < 0
                or not 0 <= now_s-ego.timestamp_s <= self.max_age_s):
            reasons.append('ego_invalid_or_stale')
        try:
            points = np.asarray(lidar.points_chassis,dtype=float)
        except (TypeError, ValueError):
            points = np.empty((0,3))
        good_lidar = (lidar.header.fresh(now_s,self.max_age_s)
            and lidar.header.frame_id == 'chassis'
            and lidar.header.calibration_id == self.lidar_calibration_id
            and lidar.header.episode_id == self.episode_id
            and points.ndim == 2 and points.shape[1] == 3
            and 20 <= len(points) <= 200000 and np.isfinite(points).all()
            and lidar.header.acquisition_s > self._last_lidar_s
            and abs(lidar.header.acquisition_s-ego.timestamp_s) <= .05)
        tracks, gap = (),None
        if not good_lidar:
            reasons.append('lidar_invalid_stale_or_unsynchronized')
        elif 'ego_invalid_or_stale' not in reasons:
            obstacles = points[(points[:,2]>-.6)&(points[:,2]<.6)&(points[:,0]>self.ego_half_length_m)]
            corridor = obstacles[np.abs(obstacles[:,1])<self.corridor_half_width_m]
            if len(corridor):
                gap = max(0.,float(corridor[:,0].min()-self.ego_half_length_m))
            c,s = math.cos(ego.yaw_rad),math.sin(ego.yaw_rad)
            world = obstacles[:,:2] @ np.array([[c,s],[-s,c]]) + [ego.x_m,ego.y_m]
            tracks = tuple(self.tracker.update(world,lidar.header.acquisition_s))
            self._last_lidar_s = lidar.header.acquisition_s
        valid = not reasons
        speed = max(0.,ego.speed_m_s)
        scan_age = max(0.,now_s-lidar.header.acquisition_s) if good_lidar else self.max_age_s
        stop_distance = speed*speed/3 + speed*(.2+scan_age)+1.
        blocked = gap is not None and gap <= stop_distance
        # Do not accelerate toward the same obstacle merely because braking
        # reduced the speed-dependent stopping distance. Release only on fresh
        # measured clearance greater than the initial stop gap plus one metre.
        if blocked and self._obstacle_stop_release_gap is None:
            self._obstacle_stop_release_gap = gap+1.
        if self._obstacle_stop_release_gap is not None:
            if good_lidar and gap is not None and gap > self._obstacle_stop_release_gap:
                self._obstacle_stop_release_gap = None
            else:
                blocked = True
        return PerceptionResult(now_s,valid,not valid or blocked,tuple(reasons),lane,tracks,gap,ego)
