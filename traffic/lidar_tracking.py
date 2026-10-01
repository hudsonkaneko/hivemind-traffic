"""Bounded, deterministic world-XY lidar tracking without simulator identities.

Tracks are geometric estimates, not vehicle identity or guaranteed velocity.
Partial visible surfaces shift box centres; ``motion_uncertainty`` (metres) makes
that limitation explicit and grows while a track is unobserved.
"""
from dataclasses import dataclass, replace
import math

import numpy as np


@dataclass(frozen=True)
class Track:
    id: int
    x: float
    y: float
    vx: float
    vy: float
    xmin: float
    xmax: float
    ymin: float
    ymax: float
    last_seen: float
    observed_count: int
    age: float
    motion_uncertainty: float
    state_time: float


class LidarTracker:
    """Cluster occupied grid cells and associate boxes to predicted tracks.

    ``update`` returns observed and extrapolated tracks, sorted by local ID.
    ``state_time`` timestamps the returned position and bounding box, including
    predictions; ``last_seen`` timestamps the last actual lidar observation.
    Empty (0, 2) frames advance time. Invalid frames and non-increasing times
    raise ValueError before changing state. IDs are never ground-truth IDs.
    """

    def __init__(self, *, cluster_cell_size=0.75, min_points=3,
                 association_distance=4.0, max_missed_seconds=0.5,
                 velocity_smoothing=0.5, max_tracks=128, max_points=200000):
        for name, value in (("cluster_cell_size", cluster_cell_size),
                            ("association_distance", association_distance),
                            ("max_missed_seconds", max_missed_seconds)):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if not math.isfinite(velocity_smoothing) or not 0 < velocity_smoothing <= 1:
            raise ValueError("velocity_smoothing must be in (0, 1]")
        for name, value in (("min_points", min_points), ("max_tracks", max_tracks),
                            ("max_points", max_points)):
            if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or value < 1:
                raise ValueError(f"{name} must be a positive integer")
        self.cluster_cell_size = float(cluster_cell_size)
        self.min_points = int(min_points)
        self.association_distance = float(association_distance)
        self.max_missed_seconds = float(max_missed_seconds)
        self.velocity_smoothing = float(velocity_smoothing)
        self.max_tracks = int(max_tracks)
        self.max_points = int(max_points)
        self._tracks = {}
        self._births = {}
        self._timestamp = None
        self._next_id = 1

    def _clusters(self, points):
        cells = {}
        for index, point in enumerate(points):
            key = tuple(math.floor(float(v) / self.cluster_cell_size) for v in point)
            cells.setdefault(key, []).append(index)
        remaining = set(cells)
        boxes = []
        for seed in sorted(cells):
            if seed not in remaining:
                continue
            remaining.remove(seed)
            pending, indices = [seed], []
            while pending:
                cx, cy = pending.pop()
                indices.extend(cells[(cx, cy)])
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        neighbour = (cx + dx, cy + dy)
                        if neighbour in remaining:
                            remaining.remove(neighbour)
                            pending.append(neighbour)
            if len(indices) >= self.min_points:
                cluster = points[indices]
                low, high = cluster.min(axis=0), cluster.max(axis=0)
                boxes.append((float(low[0]), float(high[0]), float(low[1]), float(high[1])))
        return sorted(boxes)

    @staticmethod
    def _predict(track, dt):
        dx, dy = track.vx * dt, track.vy * dt
        return replace(track, x=track.x + dx, y=track.y + dy,
                       xmin=track.xmin + dx, xmax=track.xmax + dx,
                       ymin=track.ymin + dy, ymax=track.ymax + dy,
                       motion_uncertainty=track.motion_uncertainty + dt * (1 + math.hypot(track.vx, track.vy) * .2))

    def update(self, points, timestamp):
        try:
            timestamp = float(timestamp)
            points = np.asarray(points, dtype=np.float64)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("expected finite timestamp and numeric Nx2 points") from exc
        if not math.isfinite(timestamp) or (self._timestamp is not None and timestamp <= self._timestamp):
            raise ValueError("timestamp must be finite and strictly increasing")
        if points.ndim != 2 or points.shape[1] != 2 or len(points) > self.max_points or not np.isfinite(points).all():
            raise ValueError("points must be finite Nx2 within max_points")
        # Guard arithmetic overflow before any mutation, including grid conversion.
        if len(points) and np.max(np.abs(points)) > 1e12:
            raise ValueError("world coordinates exceed supported range")
        boxes = self._clusters(points)
        dt = 0 if self._timestamp is None else timestamp - self._timestamp
        prior = {i: t for i, t in self._tracks.items()
                 if timestamp - t.last_seen <= self.max_missed_seconds}
        predicted = {i: self._predict(t, dt) for i, t in prior.items()}
        candidates = []
        for i, track in predicted.items():
            for j, (xmin, xmax, ymin, ymax) in enumerate(boxes):
                gap_x = max(track.xmin - xmax, xmin - track.xmax, 0)
                gap_y = max(track.ymin - ymax, ymin - track.ymax, 0)
                gap = math.hypot(gap_x, gap_y)
                distance = math.hypot((xmin + xmax) / 2 - track.x, (ymin + ymax) / 2 - track.y)
                if gap <= self.association_distance and distance <= self.association_distance + max(track.xmax-track.xmin, track.ymax-track.ymin):
                    candidates.append((distance, i, j))
        matched_ids, matched_boxes = set(), set()
        result = dict(predicted)
        for _, i, j in sorted(candidates):
            if i in matched_ids or j in matched_boxes:
                continue
            matched_ids.add(i)
            matched_boxes.add(j)
            xmin, xmax, ymin, ymax = boxes[j]
            x, y = (xmin + xmax) / 2, (ymin + ymax) / 2
            old, prediction = prior[i], predicted[i]
            # Stored old position is at previous frame time (possibly predicted).
            alpha = self.velocity_smoothing
            vx = (1-alpha)*old.vx + alpha*(x-old.x)/dt
            vy = (1-alpha)*old.vy + alpha*(y-old.y)/dt
            uncertainty = self.cluster_cell_size + math.hypot(x-prediction.x, y-prediction.y)
            result[i] = Track(i, x, y, vx, vy, xmin, xmax, ymin, ymax,
                              timestamp, old.observed_count + 1, timestamp-self._births[i], uncertainty,
                              timestamp)
        births = {i: self._births[i] for i in result}
        next_id = self._next_id
        for j, (xmin, xmax, ymin, ymax) in enumerate(boxes):
            if j in matched_boxes or len(result) >= self.max_tracks:
                continue
            result[next_id] = Track(next_id, (xmin+xmax)/2, (ymin+ymax)/2, 0., 0.,
                                    xmin, xmax, ymin, ymax, timestamp, 1, 0., self.cluster_cell_size,
                                    timestamp)
            births[next_id] = timestamp
            next_id += 1
        result = {i: replace(t, age=timestamp-births[i], state_time=timestamp)
                  for i, t in result.items()}
        for track in result.values():
            if not all(math.isfinite(value) for value in (
                    track.x, track.y, track.vx, track.vy, track.xmin,
                    track.xmax, track.ymin, track.ymax, track.age,
                    track.motion_uncertainty)):
                raise ValueError("frame timing or coordinates overflow motion estimates")
        self._tracks, self._births = result, births
        self._timestamp, self._next_id = timestamp, next_id
        return [result[i] for i in sorted(result)]
