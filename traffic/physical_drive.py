"""Reference paths for a physics-driven car that follows the policy's lane decisions (no simulator).

The merge policy decides a longitudinal acceleration and a lane (stay, left, right) every 0.1 s.
On a physics car those decisions are carried out by a low-level controller that steers along a
reference path. This module builds that path from the SUMO network the policy sees:

- the lane sequence ahead follows the vehicle's route through the network's connections,
  including junction-internal lanes (for example the LA ramp's `:n1_0_0` into `sb2_0`);
- with no lane change the path is the current lane's centerline (continued straight past the
  end of the SUMO corridor, so a car at its end still has a path ahead);
- during a lane change the path starts on the source lane's centerline and moves onto the
  target lane's with a quintic smoothstep over the maneuver duration (SUMO's
  --lanechange.duration, 3 s in these environments), timed by the car's current speed.

Pure Python and sumolib only, so it is testable without Isaac. Lateral offsets follow SUMO's
convention: positive is to the left of the lane's direction of travel.
"""
from __future__ import annotations

import math


def smoothstep5(t):
    """Quintic smoothstep: 0 at t <= 0, 1 at t >= 1, zero slope and curvature at both ends."""
    t = min(1., max(0., t))
    return t * t * t * (10. - 15. * t + 6. * t * t)


def _polyline_point(shape, s):
    """Point, unit direction and left normal at arc length s along a polyline (clamped to its ends)."""
    done = 0.
    for a, b in zip(shape, shape[1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if seg < 1e-9:
            continue
        if done + seg >= s or b is shape[-1]:
            u = min(1., max(0., (s - done) / seg))
            dx, dy = (b[0] - a[0]) / seg, (b[1] - a[1]) / seg
            return (a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1])), (dx, dy), (-dy, dx)
        done += seg
    raise ValueError('Degenerate polyline')


def project(shape, x, y):
    """Arc length along a polyline of the point nearest (x, y), and the signed left offset."""
    best, done = (math.inf, 0., 0.), 0.
    for a, b in zip(shape, shape[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        seg = math.hypot(dx, dy)
        if seg < 1e-9:
            continue
        u = max(0., min(1., ((x - a[0]) * dx + (y - a[1]) * dy) / (seg * seg)))
        px, py = a[0] + u * dx, a[1] + u * dy
        d = math.hypot(x - px, y - py)
        if d < best[0]:
            side = math.copysign(d, dx * (y - a[1]) - dy * (x - a[0]))
            best = (d, done + u * seg, side)
        done += seg
    return best[1], best[2]


class LaneNetwork:
    """The lane geometry and connections of one SUMO network, read once with sumolib."""

    def __init__(self, network_path):
        import sumolib
        self.net = sumolib.net.readNet(str(network_path), withInternal=True)

    def lane(self, lane_id):
        return self.net.getLane(lane_id)

    def shape(self, lane_id):
        return [tuple(p[:2]) for p in self.net.getLane(lane_id).getShape()]

    def width(self, lane_id):
        return float(self.net.getLane(lane_id).getWidth())

    def lanes_ahead(self, lane_id, route, minimum_m):
        """Lane ids from lane_id onward along the route, through internal lanes, covering minimum_m."""
        lanes, total = [lane_id], self.net.getLane(lane_id).getLength()
        while total < minimum_m:
            lane = self.net.getLane(lanes[-1])
            edge = lane.getEdge().getID()
            options = lane.getOutgoing()
            if edge.startswith(':'):
                nxt = options[0].getToLane().getID() if options else None
            else:
                later = route[route.index(edge) + 1:] if edge in route else []
                matches = [c for c in options if later and c.getToLane().getEdge().getID() == later[0]]
                nxt = (matches[0].getViaLaneID() or matches[0].getToLane().getID()) if matches else None
            if nxt is None:
                break                      # the lane ends here (route end or a lane drop)
            lanes.append(nxt)
            total += self.net.getLane(nxt).getLength()
        return lanes

    def joined_shape(self, lanes):
        points = []
        for lane_id in lanes:
            shape = self.shape(lane_id)
            points.extend(shape[1:] if points and math.hypot(shape[0][0] - points[-1][0],
                                                              shape[0][1] - points[-1][1]) < .05 else shape)
        return points


def _shape_covering(network, lane_id, route, x, y, ahead_m):
    """Joined lane shape from lane_id along the route reaching ahead_m past the car's projection.

    During a lane change the car may already be one or more edges past the lane the maneuver
    was named on (its target lane is named on the edge where it started), so the reach is
    grown until it covers the car's position plus the lookahead, or the lanes end.
    """
    minimum = network.lane(lane_id).getLength() + ahead_m + 60.
    while True:
        lanes = network.lanes_ahead(lane_id, route, minimum)
        shape = network.joined_shape(lanes)
        total = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(shape, shape[1:]))
        if total >= project(shape, x, y)[0] + ahead_m + 10. or total < minimum - 1e-6 or minimum > 1e5:
            return shape                  # covers the car, or the lanes end before the minimum
        minimum *= 2.


def _extended(shape, x, y, ahead_m):
    """The shape, continued straight past its end when it ends less than ahead_m beyond (x, y).

    The SUMO corridor (route end or a dropped lane) ends where the modelled road goes on; a car
    near that end still needs a path ahead to steer by. Its environment stops it or removes it.
    """
    s0, _ = project(shape, x, y)
    total = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(shape, shape[1:]))
    short = s0 + ahead_m + 10. - total
    if short <= 0.:
        return shape
    (ex, ey), (dx, dy), _ = _polyline_point(shape, total)
    return list(shape) + [(ex + dx * short, ey + dy * short)]


def reference_path(network, route, lane_id, x, y, speed, maneuver=None, now=0., ahead_m=None, step_m=2.):
    """Points (x, y) the car should follow from its front bumper position (x, y) onward.

    maneuver: None, or dict(source_lane, target_lane, started_at, duration_s) for a lane change
    in progress. The path is then built on the target lane's centerline, offset toward the
    source lane by (1 - smoothstep5(elapsed / duration)) of the lane spacing, so the car reaches
    the target centerline when the maneuver's time is up.
    """
    speed = max(float(speed), 1.)
    ahead_m = ahead_m if ahead_m is not None else max(40., 4. * speed)
    base_lane = maneuver['target_lane'] if maneuver else lane_id
    shape = _extended(_shape_covering(network, base_lane, route, x, y, ahead_m), x, y, ahead_m)
    s0, _ = project(shape, x, y)
    spacing = 0.
    if maneuver:
        source = _extended(_shape_covering(network, maneuver['source_lane'], route, x, y, ahead_m), x, y, ahead_m)
        sx, sy = _polyline_point(source, project(source, x, y)[0])[0]
        spacing = project(shape, sx, sy)[1]                 # signed offset of the source lane from the target
    points, s = [], 0.
    while s <= ahead_m + 1e-9:
        (px, py), _, (nx, ny) = _polyline_point(shape, s0 + s)
        offset = 0.
        if maneuver:
            elapsed = now - maneuver['started_at'] + s / speed
            offset = spacing * (1. - smoothstep5(elapsed / maneuver['duration_s']))
        points.append((px + nx * offset, py + ny * offset))
        s += step_m
    return points
