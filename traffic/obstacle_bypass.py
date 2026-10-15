"""Bounded static-obstacle fixture: sensed bounds, known road, extent priors.

CPU only. Ego odometry is privileged; obstacle coordinates come only from
chassis-frame LiDAR. This is not general autonomy or moving-object tracking.
The first confirmed detour stays immutable for the rest of the episode.
"""
from dataclasses import dataclass
import math

from traffic.lane_geometry import LaneRoute, RouteSegment
from traffic.lidar_braking import LidarEmergencyBrake, LidarBrakeConfig
from traffic.speed_profiles import speed_limit
from traffic.path_following import VehicleState, FRAME, SOURCE, _finite, _tick

PLANNER_SOURCE = 'lidar_sensed_bounds_known_road_and_explicit_extent_prior'


@dataclass(frozen=True)
class BypassConfig:
    passing_y_m: float = 3.8
    road_min_y_m: float = -1.8
    road_max_y_m: float = 5.4
    shift_x_m: float = 20.0
    detection_range_m: float = 30.0
    max_obstacle_length_m: float = 4.0
    max_obstacle_width_m: float = 3.0
    longitudinal_clearance_m: float = 4.0
    half_width_m: float = 0.9
    half_length_m: float = 2.4
    lateral_margin_m: float = 0.25
    stop_x_m: float = 90.0
    target_speed_m_s: float = 3.0
    min_hits: int = 3
    confirmation_scans: int = 2
    max_confirmation_ticks: int = 120
    speed_profile: str = 'low-speed'

    def __post_init__(self):
        values = {k:v for k,v in vars(self).items() if k != 'speed_profile'}
        if not all(_finite(v) for v in values.values()):
            raise ValueError('Bypass configuration must be finite')
        if (not all(_tick(v) and v > 0 for v in (self.min_hits, self.confirmation_scans, self.max_confirmation_ticks))
                or self.confirmation_scans < 2 or self.min_hits < 3
                or not 0 < self.target_speed_m_s <= speed_limit(self.speed_profile)
                or self.road_min_y_m >= 0 or self.road_max_y_m <= self.passing_y_m
                or any(values[k] <= 0 for k in ('passing_y_m', 'shift_x_m',
                    'detection_range_m', 'max_obstacle_length_m', 'max_obstacle_width_m',
                    'longitudinal_clearance_m', 'half_width_m', 'half_length_m', 'stop_x_m'))
                or self.lateral_margin_m < 0):
            raise ValueError('Invalid bounded bypass configuration')


@dataclass(frozen=True)
class BypassDecision:
    status: str
    reason: str
    route: LaneRoute
    target_speed_m_s: float
    stop_s_m: float
    sensed_bounds: tuple | None
    extent_prior_m: tuple
    source: str = PLANNER_SOURCE
    sensed_acquisition_ticks: tuple | None = None


def road_footprint_bounds(route, config):
    """Conservative lateral extrema of the rectangle following the reference.

    Sample spacing is at most 5 cm. Between samples, centre Y has derivative
    bounded by 1 and rectangle half-extent by |curvature|*(half width+length).
    Inflate sampled extrema by this Lipschitz bound times half the spacing,
    plus a numerical margin. This certifies the ideal reference footprint,
    not tracking error or suspension/body roll in the physical simulation.
    """
    count = max(1, math.ceil(route.length_m/.05))
    spacing = route.length_m/count
    max_curvature = max(abs(segment.curvature_rad_m) for segment in route.segments)
    margin = (1+max_curvature*(config.half_width_m+config.half_length_m))*spacing/2 + 1e-8
    lower, upper = math.inf, -math.inf
    for index in range(count+1):
        point = route.evaluate(index*spacing)
        extent = (config.half_width_m*abs(math.cos(point.yaw_rad))
                  + config.half_length_m*abs(math.sin(point.yaw_rad)))
        lower = min(lower, point.position_xy[1]-extent)
        upper = max(upper, point.position_xy[1]+extent)
    return lower-margin, upper+margin


class ObstacleBypassPlanner:
    def __init__(self, episode_id, vehicle_id, nominal_route, config=None):
        self.config = config or BypassConfig()
        if not isinstance(self.config, BypassConfig):
            raise ValueError('Expected BypassConfig')
        if (not isinstance(nominal_route, LaneRoute)
                or nominal_route.origin_xy_m != (0.0, 0.0)
                or nominal_route.initial_yaw_rad != 0
                or any(s.curvature_rad_m != 0 for s in nominal_route.segments)
                or nominal_route.length_m < self.config.stop_x_m):
            raise ValueError('Bypass requires a straight +X nominal route at Y=0')
        self.episode_id, self.vehicle_id = episode_id, vehicle_id
        self.route = nominal_route
        self._validator = LidarEmergencyBrake(episode_id, vehicle_id,
            LidarBrakeConfig(speed_profile=self.config.speed_profile,
                             max_speed_m_s=speed_limit(self.config.speed_profile)))
        self._candidate = None
        self._confirmations = 0
        self._last_scan_end = None
        self._bounds = None
        self._frozen = False
        self._stop_reason = None
        self._candidate_start_tick = None
        self._bounds_ticks = None

    def _decision(self, status, reason):
        cfg = self.config
        return BypassDecision(status, reason, self.route,
            0.0 if status in ('stop', 'stale_invalid') else cfg.target_speed_m_s,
            self.route.project(cfg.stop_x_m, 0).s_m, self._bounds,
            (cfg.max_obstacle_length_m, cfg.max_obstacle_width_m),
            sensed_acquisition_ticks=self._bounds_ticks)

    def _waiting(self, tick, ego_x):
        cfg = self.config
        if self._candidate is not None and (
                self._candidate[0]-cfg.longitudinal_clearance_m-cfg.shift_x_m-2 <= ego_x+1
                or tick-self._candidate_start_tick >= cfg.max_confirmation_ticks):
            self._stop_reason = 'no_feasible_bounded_detour'
            return self._decision('stop', self._stop_reason)
        return self._decision('confirming', 'awaiting_bounded_obstacle_coverage')

    def _build(self, bounds, ego_x, *, check_lateral_coverage=True):
        cfg = self.config
        front, _, low_y, high_y = bounds
        start = front - cfg.longitudinal_clearance_m - cfg.shift_x_m - 2.0
        return_start = front + cfg.max_obstacle_length_m + cfg.longitudinal_clearance_m
        theta = 2 * math.atan2(cfg.passing_y_m, cfg.shift_x_m)
        radius = (cfg.shift_x_m**2 + cfg.passing_y_m**2)/(4*cfg.passing_y_m)
        if (start <= ego_x + 1.0 or start <= 0
                or return_start + cfg.shift_x_m >= cfg.stop_x_m
                or high_y-low_y > cfg.max_obstacle_width_m
                or (check_lateral_coverage and cfg.passing_y_m - (low_y + cfg.max_obstacle_width_m)
                    < cfg.half_width_m + cfg.lateral_margin_m)):
            return None
        arc = radius*theta
        segments = (RouteSegment(start), RouteSegment(arc, 1/radius),
            RouteSegment(arc, -1/radius),
            RouteSegment(return_start-start-cfg.shift_x_m),
            RouteSegment(arc, -1/radius), RouteSegment(arc, 1/radius),
            RouteSegment(cfg.stop_x_m-return_start-cfg.shift_x_m))
        route = LaneRoute(self.route.route_id + '_lidar_bypass', self.route.width_m, segments)
        lower, upper = road_footprint_bounds(route, cfg)
        if lower < cfg.road_min_y_m or upper > cfg.road_max_y_m:
            return None
        return route

    def update(self, scan, state, tick):
        if (not isinstance(state, VehicleState) or not _tick(tick)
                or state.episode_id != self.episode_id or state.vehicle_id != self.vehicle_id
                or not _tick(state.tick) or state.tick != tick
                or state.frame != FRAME or state.source != SOURCE
                or not all(_finite(v) for v in (state.x_m, state.y_m, state.yaw_rad, state.speed_m_s))
                or not 0 <= state.speed_m_s <= speed_limit(self.config.speed_profile)):
            return self._decision('stale_invalid', 'invalid_ego_odometry')
        validated = self._validator.evaluate(scan, tick=tick, speed_m_s=state.speed_m_s)
        if validated.status == 'stale_invalid':
            return self._decision('stale_invalid', validated.reason)
        reference = scan.acquisition_end_tick if scan.coordinate_reference_tick is None else scan.coordinate_reference_tick
        if reference != tick:
            return self._decision('stale_invalid', 'scan_not_in_current_chassis_frame')
        if self._stop_reason:
            return self._decision('stop', self._stop_reason)
        if self._frozen:
            return self._decision('detour', 'confirmed_static_detour_frozen')
        cfg = self.config
        c, s = math.cos(state.yaw_rad), math.sin(state.yaw_rad)
        hits = []
        for x, y, z in scan.points:
            if not -.6 <= z <= .6 or (abs(x) <= cfg.half_length_m and abs(y) <= cfg.half_width_m):
                continue
            wx, wy = state.x_m+c*x-s*y, state.y_m+s*x+c*y
            if (0 < wx-state.x_m <= cfg.detection_range_m
                    and cfg.road_min_y_m <= wy <= cfg.road_max_y_m
                    and abs(wy) <= cfg.half_width_m+cfg.lateral_margin_m):
                hits.append((wx, wy))
        if len(hits) < cfg.min_hits:
            if self._candidate is not None:
                return self._waiting(tick, state.x_m)
            return self._decision('nominal', 'no_confirmed_nominal_lane_obstacle')
        front = min(p[0] for p in hits)
        cluster = [p for p in hits if p[0] <= front+1.0]
        if len(cluster) < cfg.min_hits:
            return self._waiting(tick, state.x_m)
        bounds = (front, max(p[0] for p in cluster), min(p[1] for p in cluster), max(p[1] for p in cluster))
        if self._candidate is None or abs(front-self._candidate[0]) > 1.0:
            self._candidate, self._confirmations = bounds, 0
            self._candidate_start_tick = tick
            self._bounds_ticks = (scan.acquisition_start_tick, scan.acquisition_end_tick)
        if scan.acquisition_end_tick != self._last_scan_end:
            self._confirmations += 1
            self._last_scan_end = scan.acquisition_end_tick
            previous = self._candidate
            self._candidate = (min(previous[0], bounds[0]), max(previous[1], bounds[1]),
                min(previous[2], bounds[2]), max(previous[3], bounds[3]))
            self._bounds_ticks = (min(self._bounds_ticks[0], scan.acquisition_start_tick),
                                  scan.acquisition_end_tick)
        self._bounds = self._candidate
        if self._confirmations < cfg.confirmation_scans:
            return self._waiting(tick, state.x_m)
        route = self._build(self._bounds, state.x_m)
        if route is None:
            # Sparse face coverage may improve on a subsequent distinct scan.
            # Test the fixed geometry independently before waiting for it.
            if (self._bounds[3]-self._bounds[2] <= cfg.max_obstacle_width_m
                    and self._build(self._bounds, state.x_m, check_lateral_coverage=False) is not None):
                return self._waiting(tick, state.x_m)
            self._stop_reason = 'no_feasible_bounded_detour'
            return self._decision('stop', self._stop_reason)
        self.route, self._frozen = route, True
        return self._decision('detour', 'confirmed_static_obstacle_bypass')
