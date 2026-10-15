"""Immutable analytic lane geometry, independent of SUMO, Isaac and road artwork.

World coordinates are right-handed XY metres, with +Z up. Yaw is radians,
counterclockwise from +X; positive lateral displacement/curvature turns left.
``s_m`` is centreline arc length, not the vehicle's front-bumper position.
The caller selects a vehicle reference point (the driver uses its rear axle).

Endpoints have infinite tangent-ray extensions: evaluate/project can return
negative s or s beyond route length. That avoids pinning a vehicle's progress
to zero before entry or to the last sample after exit. A route is not a traffic
network: projection is nearest geometry, with the smallest s winning ties;
it does not disambiguate overlapping routes using vehicle history.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from numbers import Real
from pathlib import Path


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    return float(value)


def _xy(value, name='position_xy'):
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        raise ValueError(f'{name} must contain two finite metre coordinates')
    return tuple(_number(v, name) for v in value)


def wrap_yaw(angle):
    """Normalize a finite angle to [-pi, pi)."""
    angle = _number(angle, 'yaw_rad')
    return (angle + math.pi) % math.tau - math.pi


@dataclass(frozen=True, slots=True)
class RigidTransform2D:
    """Rigid local-to-world frame: rotate by yaw, then translate; no unit scaling."""

    origin_xy_m: tuple[float, float] = (0.0, 0.0)
    yaw_rad: float = 0.0

    def __post_init__(self):
        object.__setattr__(self, 'origin_xy_m', _xy(self.origin_xy_m, 'origin_xy_m'))
        object.__setattr__(self, 'yaw_rad', wrap_yaw(self.yaw_rad))

    def to_world(self, position_xy):
        x, y = _xy(position_xy)
        c, s = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        return self.origin_xy_m[0] + c*x - s*y, self.origin_xy_m[1] + s*x + c*y

    def to_local(self, position_xy):
        x, y = _xy(position_xy)
        x, y = x - self.origin_xy_m[0], y - self.origin_xy_m[1]
        c, s = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        return c*x + s*y, -s*x + c*y

    def heading_to_world(self, local_yaw_rad):
        return wrap_yaw(_number(local_yaw_rad, 'local_yaw_rad') + self.yaw_rad)

    def heading_to_local(self, world_yaw_rad):
        return wrap_yaw(_number(world_yaw_rad, 'world_yaw_rad') - self.yaw_rad)


@dataclass(frozen=True, slots=True)
class RouteSegment:
    length_m: float
    curvature_rad_m: float = 0.0

    def __post_init__(self):
        length = _number(self.length_m, 'length_m')
        curvature = _number(self.curvature_rad_m, 'curvature_rad_m')
        if length <= 0:
            raise ValueError('Segment length must be positive')
        if abs(length * curvature) >= math.pi:
            raise ValueError('Each arc must sweep less than pi radians; split longer arcs explicitly')
        object.__setattr__(self, 'length_m', length)
        object.__setattr__(self, 'curvature_rad_m', curvature)


@dataclass(frozen=True, slots=True)
class RoutePoint:
    s_m: float
    position_xy: tuple[float, float]
    yaw_rad: float
    curvature_rad_m: float
    lateral_offset_m: float = 0.0


@dataclass(frozen=True, slots=True)
class LaneProjection:
    s_m: float
    lateral_error_m: float
    distance_m: float
    point: RoutePoint


@dataclass(frozen=True, slots=True)
class _Piece:
    start_s_m: float
    segment: RouteSegment
    position_xy: tuple[float, float]
    yaw_rad: float

    def evaluate(self, local_s):
        delta = self.segment.curvature_rad_m * local_s
        half = delta / 2
        # Stable straight limit without subtracting almost-identical sines.
        chord = local_s * (math.sin(half) / half if half else 1.0)
        middle_yaw = self.yaw_rad + half
        return RoutePoint(self.start_s_m + local_s,
                          (self.position_xy[0] + chord * math.cos(middle_yaw),
                           self.position_xy[1] + chord * math.sin(middle_yaw)),
                          wrap_yaw(self.yaw_rad + delta), self.segment.curvature_rad_m)


@dataclass(frozen=True, slots=True)
class LaneRoute:
    route_id: str
    width_m: float
    segments: tuple[RouteSegment, ...]
    origin_xy_m: tuple[float, float] = (0.0, 0.0)
    initial_yaw_rad: float = 0.0
    _pieces: tuple[_Piece, ...] = field(init=False, repr=False)
    length_m: float = field(init=False)

    def __post_init__(self):
        if not isinstance(self.route_id, str) or not self.route_id.strip():
            raise ValueError('route_id must be a nonempty string')
        width = _number(self.width_m, 'width_m')
        if width <= 0:
            raise ValueError('Lane width must be positive')
        if not isinstance(self.segments, (tuple, list)) or not self.segments:
            raise ValueError('At least one RouteSegment is required')
        segments = tuple(self.segments)
        if any(not isinstance(segment, RouteSegment) for segment in segments):
            raise ValueError('segments must contain RouteSegment objects')
        if any(abs(segment.curvature_rad_m) * width / 2 >= 1 for segment in segments):
            raise ValueError('Lane half-width must be smaller than every arc radius')
        origin = _xy(self.origin_xy_m, 'origin_xy_m')
        yaw = wrap_yaw(self.initial_yaw_rad)
        object.__setattr__(self, 'width_m', width)
        object.__setattr__(self, 'segments', segments)
        object.__setattr__(self, 'origin_xy_m', origin)
        object.__setattr__(self, 'initial_yaw_rad', yaw)
        pieces, station, position = [], 0.0, origin
        for segment in segments:
            piece = _Piece(station, segment, position, yaw)
            endpoint = piece.evaluate(segment.length_m)
            pieces.append(piece)
            station, position, yaw = endpoint.s_m, endpoint.position_xy, endpoint.yaw_rad
            if not all(math.isfinite(v) for v in (station, *position)):
                raise ValueError('Route extent exceeds finite coordinate representation')
        object.__setattr__(self, '_pieces', tuple(pieces))
        object.__setattr__(self, 'length_m', station)

    @property
    def total_length_m(self):
        return self.length_m

    @classmethod
    def from_dict(cls, definition):
        """Load explicit unit-bearing fields, rejecting silent misspellings."""
        keys = {'route_id', 'width_m', 'origin_xy_m', 'initial_yaw_rad', 'segments'}
        if not isinstance(definition, dict) or set(definition) != keys:
            raise ValueError(f'Route definition requires exactly {sorted(keys)}')
        if not isinstance(definition['segments'], list):
            raise ValueError('segments must be a list')
        segments = []
        for item in definition['segments']:
            if not isinstance(item, dict) or set(item) != {'length_m', 'curvature_rad_m'}:
                raise ValueError('Each segment requires length_m and curvature_rad_m')
            segments.append(RouteSegment(**item))
        return cls(**{**definition, 'segments': tuple(segments)})

    def evaluate(self, s_m):
        """Exact centreline point; curvature is right-continuous at joins.

        Before entry and after exit, the endpoint tangent continues linearly
        with zero curvature. Endpoint s=length uses the last segment curvature.
        """
        s_m = _number(s_m, 's_m')
        if s_m < 0:
            return RoutePoint(s_m,
                (self.origin_xy_m[0] + s_m * math.cos(self.initial_yaw_rad),
                 self.origin_xy_m[1] + s_m * math.sin(self.initial_yaw_rad)),
                self.initial_yaw_rad, 0.0)
        if s_m > self.length_m:
            end = self._pieces[-1].evaluate(self._pieces[-1].segment.length_m)
            beyond = s_m - self.length_m
            return RoutePoint(s_m, (end.position_xy[0] + beyond * math.cos(end.yaw_rad),
                                    end.position_xy[1] + beyond * math.sin(end.yaw_rad)), end.yaw_rad, 0.0)
        for piece in reversed(self._pieces):
            if s_m >= piece.start_s_m:
                return piece.evaluate(s_m - piece.start_s_m)
        raise AssertionError('Nonnegative station must belong to a route piece')

    def project(self, x_m, y_m):
        """Nearest analytic centreline/tangent-ray projection; positive error left.

        Uses exact segment and circular-arc candidates, not a sampled-polyline
        approximation. Ties within floating-point distance tolerance choose the
        smallest station. At a circle centre, its earliest arc endpoint wins.
        """
        x, y = _number(x_m, 'x_m'), _number(y_m, 'y_m')
        candidates = []
        for piece in self._pieces:
            length, k = piece.segment.length_m, piece.segment.curvature_rad_m
            if k == 0:
                along = ((x - piece.position_xy[0]) * math.cos(piece.yaw_rad)
                         + (y - piece.position_xy[1]) * math.sin(piece.yaw_rad))
                candidates.append(piece.start_s_m + min(length, max(0.0, along)))
            else:
                candidates.extend((piece.start_s_m, piece.start_s_m + length))
                dx, dy = x - piece.position_xy[0], y - piece.position_xy[1]
                local_x = dx * math.cos(piece.yaw_rad) + dy * math.sin(piece.yaw_rad)
                local_y = -dx * math.sin(piece.yaw_rad) + dy * math.cos(piece.yaw_rad)
                # Local-frame form avoids subtracting a huge 1/k circle centre
                # or wrapping a tiny angle through pi when curvature is small.
                sine_term, cosine_term = k * local_x, 1 - k * local_y
                if sine_term != 0 or cosine_term != 0:
                    delta = math.atan2(sine_term, cosine_term)
                    for turns in (-1, 0, 1):
                        along = (delta + turns * math.tau) / k
                        if 0 <= along <= length:
                            candidates.append(piece.start_s_m + along)
        start_along = ((x - self.origin_xy_m[0]) * math.cos(self.initial_yaw_rad)
                       + (y - self.origin_xy_m[1]) * math.sin(self.initial_yaw_rad))
        if start_along < 0:
            candidates.append(start_along)
        end = self.evaluate(self.length_m)
        end_along = (x - end.position_xy[0]) * math.cos(end.yaw_rad) + (y - end.position_xy[1]) * math.sin(end.yaw_rad)
        if end_along > 0:
            candidates.append(self.length_m + end_along)
        best_point, best_distance = None, math.inf
        for station in sorted(candidates):
            point = self.evaluate(station)
            distance = math.hypot(x - point.position_xy[0], y - point.position_xy[1])
            if best_point is None or distance < best_distance - 1e-12:
                best_point, best_distance = point, distance
        dx, dy = x - best_point.position_xy[0], y - best_point.position_xy[1]
        lateral = -math.sin(best_point.yaw_rad) * dx + math.cos(best_point.yaw_rad) * dy
        return LaneProjection(best_point.s_m, lateral, best_distance, best_point)

    def sample(self, max_spacing_m=0.5, lateral_offset_m=0.0):
        """Immutable shared centreline/boundary samples, including exact joins.

        Offset is positive left. Sample s remains the *centreline* station,
        while position/curvature describe the parallel offset curve. The step
        bound applies to that curve's arc distance, not just chord length.
        This is a geometry export, not a SUMO network or Isaac stage exporter.
        """
        spacing = _number(max_spacing_m, 'max_spacing_m')
        offset = _number(lateral_offset_m, 'lateral_offset_m')
        if spacing <= 0:
            raise ValueError('max_spacing_m must be positive')
        if any(1 - p.segment.curvature_rad_m * offset <= 0 for p in self._pieces):
            raise ValueError('Offset must not reach or cross an arc centre')
        stations = []
        for piece in self._pieces:
            factor = 1 - piece.segment.curvature_rad_m * offset
            count = max(1, math.ceil(piece.segment.length_m * factor / spacing))
            if count > 1_000_000:
                raise ValueError('Requested sampling density exceeds the bounded exporter')
            stations.extend(piece.start_s_m + piece.segment.length_m * i / count for i in range(count))
        stations.append(self.length_m)
        points = []
        for station in stations:
            point = self.evaluate(station)
            points.append(RoutePoint(station,
                (point.position_xy[0] - offset * math.sin(point.yaw_rad),
                 point.position_xy[1] + offset * math.cos(point.yaw_rad)),
                point.yaw_rad, point.curvature_rad_m / (1 - point.curvature_rad_m * offset), offset))
        return tuple(points)


def load_routes(path):
    """Load the versioned shared fixture; no simulator initialization or writes."""
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data, dict) or data.get('schema_version') != 1:
        raise ValueError('Unsupported lane route schema')
    if data.get('frame') != 'xy_m_z_up_yaw_ccw_rad':
        raise ValueError('Route frame must be explicit XY metres / Z up / CCW radians')
    definitions = data.get('routes')
    if not isinstance(definitions, list) or not definitions:
        raise ValueError('Expected a nonempty routes list')
    routes = [LaneRoute.from_dict(item) for item in definitions]
    if len({route.route_id for route in routes}) != len(routes):
        raise ValueError('Route IDs must be unique')
    return {route.route_id: route for route in routes}
