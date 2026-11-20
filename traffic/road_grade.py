"""Road grade under a SUMO car from the 3D road model, and the acceleration the car's drive can reach there.

User decision (November 19, 2026): "Yes, build it" (a grade-aware acceleration limit). SUMO's
cars ignore grade: a commanded acceleration is carried out whatever the slope. The physics
drive (scripts/record_highway_flow_v14_physics.py) showed that the prepared Americano cannot:
up the southbound LA ramp it reached about 1.37 m/s^2 where the policy asked for 1.6-1.7.
This module gives the v14 environment the same limit the physics car has:

- grade: the road model's own height under the car's two axles. The heights come from the
  model lane graph lanes that the corridor manifest names (the same hash-checked source
  as scripts/render_i110_merge_replay.py --road-surface), sampled along every SUMO lane of
  the corridor network once, so a lookup during an episode is an interpolation;
- drive ceiling: the longitudinal force balance of the physics car at full drive torque,
  with the vehicle package's own mass, wheel radius, wheel inertia and wheel damping:

      a_max = (n_driven T / r - n_wheels c v / r^2 - m g sin(theta)) / (m + n_wheels I / r^2)

  T is the drive torque limit per driven wheel, c the PhysX wheel damping rate (damping
  torque = c * wheel speed), I the wheel moment of inertia, theta the road slope.

Only cases with a road model get a grade; synthetic networks are flat by construction, and
a network without a road model gets grade 0 with that recorded as its source.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

GRAVITY_M_S2 = 9.81            # physics:gravityMagnitude of the vehicle package's physics scene
SAMPLE_M = .5                  # lane profile spacing
BEFORE_M, AFTER_M = 6., 6.     # profiles continue straight past both lane ends (axles behind the bumper)
MAX_ROAD_DISTANCE_M = 4.       # a lane sample farther than this from every model lane is an error

# The prepared Americano as the physics drive runs it.
AMERICANO_V14 = dict(
    vehicle='vehicles/sim_ready/americano-i7-ev/v14',
    mass_kg=1800.,                    # assets/vehicle-physics.usda physics:mass
    wheels=4,
    wheel_radius_m=.401728,           # physxVehicleWheel:radius
    wheel_moi_kg_m2=1.225,            # physxVehicleWheel:moi (each wheel)
    wheel_damping_n_m_s=.25,          # physxVehicleWheel:dampingRate (each wheel)
    driven_wheels=2,                  # front-wheel drive (wheels 0 and 1)
    max_drive_nm_per_wheel=700.,      # scripts/physics_drive_server.py --max-drive-nm default
    front_axle_behind_bumper_m=.9085,  # bumper 2.4085 m ahead of the wheelbase midpoint, axles at +-1.5 m
    rear_axle_behind_bumper_m=3.9085,
)


# The limit the environment applies. Checked against PhysX (outputs/test-runs/physics-stage1-06-rendered
# and outputs/highway-flow-rl/v14/physics-01): over 137 decisions spent wholly at the drive-torque
# limit with all wheels down, measured minus the force balance at the steepest grade within 0.2 s
# ahead was -0.017 +- 0.042 m/s^2. The headroom keeps the car's speed controller off its torque
# limit, so it can still correct; with it, 1 of those 137 decisions fell more than v14's 0.05 m/s^2
# realization tolerance short (the sag where the southbound ramp steepens into the junction).
DRIVE_LIMIT = dict(vehicle=AMERICANO_V14, headroom_m_s2=.1, lookahead_s=(0., .1, .2))


def validate_spec(spec):
    if dict(spec) != AMERICANO_V14:
        raise ValueError('Unreviewed vehicle limit; only the Americano v14 values are declared')
    return dict(spec)


def validate_drive_limit(settings):
    settings = dict(settings)
    if {**settings, 'lookahead_s': tuple(settings.get('lookahead_s', ()))} != DRIVE_LIMIT:
        raise ValueError('Unreviewed drive limit settings')
    return dict(DRIVE_LIMIT)


def drive_ceiling(spec, speed_m_s, grade):
    """Largest acceleration (m/s^2) the drive can produce at this speed on this grade (rise/run)."""
    r, wheels = spec['wheel_radius_m'], spec['wheels']
    drive = spec['driven_wheels'] * spec['max_drive_nm_per_wheel'] / r
    damping = wheels * spec['wheel_damping_n_m_s'] * max(0., speed_m_s) / r ** 2
    slope = spec['mass_kg'] * GRAVITY_M_S2 * grade / math.sqrt(1. + grade * grade)
    return (drive - damping - slope) / (spec['mass_kg'] + wheels * spec['wheel_moi_kg_m2'] / r ** 2)


class ModelSurface:
    """scripts/render_i110_merge_replay.surface_elevation for many points at once (numpy)."""

    def __init__(self, surface):
        self.segments = []
        for lane in surface.values():
            p = np.asarray(lane['points'], dtype=float)
            if p.ndim != 2 or p.shape[0] < 2 or p.shape[1] != 3:
                raise ValueError('Model lanes need two or more (x, y, z) points')
            self.segments.append((p[:-1], p[1:] - p[:-1]))

    def heights(self, xy, limit=MAX_ROAD_DISTANCE_M, check=None):
        """Heights at an (N, 2) array of points: the two nearest lanes, inverse-square weighted.

        Points selected by the boolean mask `check` (all by default) must lie within `limit`
        metres of a model lane; the others take the nearest heights, which past the end of
        the model's lanes is the height at their ends.
        """
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        distance, height = [], []
        for a, d in self.segments:
            n = np.maximum(d[:, 0] ** 2 + d[:, 1] ** 2, 1e-12)
            dx, dy = xy[:, :1] - a[None, :, 0], xy[:, 1:] - a[None, :, 1]
            u = np.clip((dx * d[None, :, 0] + dy * d[None, :, 1]) / n[None, :], 0., 1.)
            dist2 = (dx - u * d[None, :, 0]) ** 2 + (dy - u * d[None, :, 1]) ** 2
            best = np.argmin(dist2, axis=1)
            rows = np.arange(xy.shape[0])
            distance.append(np.sqrt(dist2[rows, best]))
            height.append(a[best, 2] + u[rows, best] * d[best, 2])
        distance, height = np.stack(distance, axis=1), np.stack(height, axis=1)
        order = np.argsort(distance, axis=1)[:, :2]
        rows = np.arange(xy.shape[0])[:, None]
        near, z = distance[rows, order], height[rows, order]
        checked = near[:, 0] if check is None else np.where(np.asarray(check, dtype=bool), near[:, 0], 0.)
        if limit is not None and float(checked.max(initial=0.)) > limit:
            worst = int(np.argmax(checked))
            raise ValueError(f'({xy[worst, 0]:.1f}, {xy[worst, 1]:.1f}) is {near[worst, 0]:.2f} m from the model road')
        if near.shape[1] == 1:
            return z[:, 0]
        w = 1. / np.maximum(near, .01) ** 2
        return (w * z).sum(axis=1) / w.sum(axis=1)


def _walk(shape, s):
    """Points at arc lengths s along a polyline, continuing straight past both ends."""
    p = np.asarray(shape, dtype=float)[:, :2]
    seg = p[1:] - p[:-1]
    length = np.hypot(seg[:, 0], seg[:, 1])
    keep = length > 1e-9
    p0, seg, length = p[:-1][keep], seg[keep], length[keep]
    if not len(length):
        raise ValueError('A lane shape needs two distinct points')
    start = np.concatenate([[0.], np.cumsum(length)[:-1]])
    index = np.clip(np.searchsorted(start, s, side='right') - 1, 0, len(length) - 1)
    u = (s - start[index]) / length[index]          # < 0 before the start, > 1 past the end (straight on)
    return p0[index] + u[:, None] * seg[index]


class RoadGrade:
    """Grade under a car on any lane of a corridor network, from the road model's surface."""

    def __init__(self, network_path, manifest_path=None, *, spec=AMERICANO_V14):
        import sumolib
        self.spec = validate_spec(spec)
        self.network_path = Path(network_path)
        self.profiles = {}
        if manifest_path is None:
            self.info = dict(source='flat: no road model for this network (grade 0)')
            return
        from scripts.render_i110_merge_replay import read_model_surface
        surface, info = read_model_surface(manifest_path, network_path)
        model = ModelSurface(surface)
        net = sumolib.net.readNet(str(network_path), withInternal=True)
        for edge in net.getEdges(withInternal=True):
            for lane in edge.getLanes():
                shape, length = lane.getShape(), float(lane.getLength())
                s = np.arange(-BEFORE_M, length + AFTER_M + 1e-9, SAMPLE_M)
                xy = _walk(shape, s * _shape_scale(shape, length))
                on_lane = (s >= 0.) & (s <= length)      # the extensions past a corridor's ends leave the model's lanes
                self.profiles[lane.getID()] = (s, model.heights(xy, check=on_lane))
        self.info = dict(info, source='model lane graph (corridor manifest)', lanes=len(self.profiles),
                         sample_m=SAMPLE_M)

    def height(self, lane_id, position_m):
        profile = self.profiles.get(lane_id)
        if profile is None:
            return None
        s, z = profile
        return float(np.interp(position_m, s, z))

    def grade(self, lane_id, bumper_position_m):
        """Rise over run between the car's axles; 0 on a flat network, None for an unknown lane."""
        if not self.profiles:
            return 0.
        front = self.height(lane_id, bumper_position_m - self.spec['front_axle_behind_bumper_m'])
        rear = self.height(lane_id, bumper_position_m - self.spec['rear_axle_behind_bumper_m'])
        if front is None or rear is None:
            return None
        return (front - rear) / (self.spec['rear_axle_behind_bumper_m'] - self.spec['front_axle_behind_bumper_m'])

    def ceiling(self, lane_id, bumper_position_m, speed_m_s, *, headroom_m_s2=0., lookahead_s=(0.,)):
        """The drive limit at the steepest grade the car reaches within the look-ahead, less the headroom.

        Look-ahead positions continue along the car's lane profile, which runs straight on for
        AFTER_M past the lane end (enough for 0.2 s at 27 m/s).
        """
        grades = [self.grade(lane_id, bumper_position_m + max(0., speed_m_s) * t) for t in lookahead_s]
        if any(g is None for g in grades):
            raise ValueError(f'No road grade for lane {lane_id}')
        grade = max(grades)
        return dict(grade=grades[0], limiting_grade=grade,
                    ceiling_m_s2=drive_ceiling(self.spec, speed_m_s, grade) - headroom_m_s2)


def _shape_scale(shape, length):
    """SUMO lane positions run over the lane's declared length, which can differ from its drawn shape."""
    p = np.asarray(shape, dtype=float)[:, :2]
    drawn = float(np.hypot(*(p[1:] - p[:-1]).T).sum())
    return drawn / length if length > 1e-9 and drawn > 1e-9 else 1.
