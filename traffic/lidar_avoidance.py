"""Conservative two-lane, STATIC-obstacle avoidance from lidar and ego odometry.

No obstacle coordinates, labels, IDs, or other-vehicle truth enter the planner.
Lane centres come from the known road map. SUMO executes lateral motion.
"""
from dataclasses import dataclass
import math
import numpy as np
from traffic.lidar_control import Clearance, target_speed

LANES = (-4.8, -1.6)


def road_points(azimuth, elevation, ranges, flags, ego):
    arrays = [np.asarray(a) for a in (azimuth, elevation, ranges, flags)]
    if len({a.shape for a in arrays}) != 1 or arrays[0].ndim != 1:
        raise ValueError('Aligned one-dimensional lidar arrays required')
    az, el, r, flags = arrays
    valid = np.isfinite(az) & np.isfinite(el) & np.isfinite(r) & (r > 0) & ((flags & 64) != 0)
    healthy = len(r) >= 1000 and int(valid.sum()) >= 20
    a, e = np.radians(az), np.radians(el)
    x = r*np.cos(e)*np.cos(a); y = r*np.cos(e)*np.sin(a); z = 1+r*np.sin(e)
    self_hit = (x >= -5.2) & (x <= .2) & (np.abs(y) <= 1.2)
    keep = valid & ~self_hit & (z > .3) & (z < 1.5) & (r < 80)
    yaw = math.radians(90-ego['angle'])
    wx = ego['x'] + x[keep]*math.cos(yaw)-y[keep]*math.sin(yaw)
    wy = ego['y'] + x[keep]*math.sin(yaw)+y[keep]*math.cos(yaw)
    return np.column_stack((wx, wy)), healthy


@dataclass(frozen=True)
class Decision:
    speed: float
    lane_request: int | None
    phase: str
    reason: str
    clearance: float
    adjacent_clear: bool
    healthy: bool


class AvoidancePlanner:
    def __init__(self, cruise=6.):
        self.cruise = min(cruise, 6.)
        self.phase = 'approach'
        self.cells = set()
        self.saw_obstacle = False

    def step(self, points, healthy, ego, *, fresh=True):
        if not healthy or not fresh:
            speed, reason = target_speed(Clearance(0, False, 0), ego['speed'])
            return Decision(speed, None, self.phase, reason, 0., False, healthy)
        # Persistent 25 cm occupancy cells are valid only for this static fixture.
        self.cells.update(map(tuple, np.rint(points/.25).astype(int)))
        world = np.array(sorted(self.cells), dtype=float).reshape(-1, 2)*.25
        dx = world[:, 0]-ego['x']
        def lane_points(lane):
            return np.abs(world[:, 1]-LANES[lane]) < 1.2
        def clear(lane):
            return not np.any(lane_points(lane) & (dx > -30) & (dx < 45))
        def forward(lanes):
            in_lane = np.zeros(len(dx), dtype=bool)
            for lane in lanes: in_lane |= lane_points(lane)
            ahead = dx[in_lane & (dx > 0)]
            return float(ahead.min()) if len(ahead) else 80.
        request = None
        if self.phase == 'approach' and forward([0]) < 45:
            self.saw_obstacle = True
            if clear(1):
                self.phase = 'outbound'; request = 1
        elif self.phase == 'outbound' and abs(ego['y']-LANES[1]) < .02:
            self.phase = 'passing'
        elif self.phase == 'passing':
            obstacle = dx[lane_points(0)]
            # Require measured obstacle memory behind the entire ego, plus margin.
            behind = self.saw_obstacle and len(obstacle) and float(obstacle.max()) < -12
            return_clear = not np.any(lane_points(0) & (dx > -12) & (dx < 45))
            if behind and return_clear:
                self.phase = 'returning'; request = 0
        elif self.phase == 'returning' and abs(ego['y']-LANES[0]) < .02:
            self.phase = 'complete'
        lanes = [0, 1] if self.phase in ('outbound', 'returning') else ([1] if self.phase == 'passing' else [0])
        distance = forward(lanes)
        speed, _ = target_speed(Clearance(distance, True, len(world)), ego['speed'], cruise=self.cruise)
        return Decision(speed, request, self.phase, 'static-lidar-'+self.phase, distance, clear(1), True)


def body_bounds(ego):
    """Conservative world AABB of the oriented 5x2 m proxy, for evaluation only."""
    yaw = math.radians(90-ego['angle'])
    points = [(ego['x']+x*math.cos(yaw)-y*math.sin(yaw),
               ego['y']+x*math.sin(yaw)+y*math.cos(yaw)) for x in (-5, 0) for y in (-1, 1)]
    return (min(p[0] for p in points), max(p[0] for p in points),
            min(p[1] for p in points), max(p[1] for p in points))


def rectangle_gap(a, b):
    dx = max(a[0]-b[1], b[0]-a[1], 0)
    dy = max(a[2]-b[3], b[2]-a[3], 0)
    return math.hypot(dx, dy)


def summarize(rows, blocked=False):
    if not rows: return dict(passed=False, steps=0)
    minimum = min(r['obstacle_gap'] for r in rows)
    max_lateral_step = max(abs(r['ego_after']['y']-r['ego']['y']) for r in rows)
    interventions = sum(abs(r['realized_speed']-r['requested_speed']) > .15 for r in rows)
    completed = any(r['phase']=='complete' for r in rows)
    stopped = rows[-1]['realized_speed'] < .2
    fallback = stopped and all(abs(r['ego_after']['y']-LANES[0]) < .02 for r in rows)
    on_road = all(body_bounds(r['ego_after'])[2] >= -6.4 and body_bounds(r['ego_after'])[3] <= 0 for r in rows)
    safe = minimum > .3 and max_lateral_step <= .15 and interventions == 0 and on_road and all(r['healthy'] and not r['collisions'] for r in rows)
    passed = safe and (fallback if blocked else completed and rows[-1]['ego_after']['x'] > rows[-1]['obstacle_end_x']+10)
    return dict(passed=bool(passed), steps=len(rows), completed=completed, blocked_fallback=bool(fallback),
                min_obstacle_gap_m=minimum, max_lateral_step_m=max_lateral_step,
                speed_interventions=interventions, final_speed=rows[-1]['realized_speed'],
                phase_sequence=list(dict.fromkeys(r['phase'] for r in rows)))
