import unittest
from dataclasses import FrozenInstanceError

import numpy as np

from traffic.lidar_tracking import LidarTracker


def box(x=0., y=0.):
    return np.array([[x+a, y+b] for a in np.linspace(0, 4, 17)
                     for b in (0, .5, 1)], dtype=float)


class LidarTrackingTests(unittest.TestCase):
    def test_translation_and_immutable_snapshot(self):
        tracker = LidarTracker(velocity_smoothing=1)
        first = tracker.update(box(), 0)[0]
        second = tracker.update(box(1), .25)[0]
        self.assertEqual(first.id, second.id)
        self.assertAlmostEqual(second.vx, 4)
        self.assertAlmostEqual(second.vy, 0)
        self.assertEqual(second.observed_count, 2)
        self.assertAlmostEqual(second.age, .25)
        self.assertEqual(second.state_time, .25)
        self.assertGreater(second.motion_uncertainty, 0)
        self.assertEqual(first.x, 2)
        with self.assertRaises(FrozenInstanceError):
            first.x = 99

    def test_missed_prediction_and_expiry(self):
        tracker = LidarTracker(velocity_smoothing=1)
        tracker.update(box(), 0)
        seen = tracker.update(box(1), .25)[0]
        missed = tracker.update(np.empty((0, 2)), .5)[0]
        self.assertAlmostEqual(missed.x, seen.x+1)
        self.assertEqual(missed.last_seen, .25)
        self.assertEqual(missed.state_time, .5)
        self.assertEqual(missed.xmin, seen.xmin + 1)
        self.assertEqual(missed.observed_count, 2)
        self.assertGreater(missed.motion_uncertainty, seen.motion_uncertainty)
        self.assertEqual(tracker.update(np.empty((0, 2)), .76), [])
        self.assertNotEqual(tracker.update(box(), 1)[0].id, seen.id)

    def test_two_objects_and_partial_surface(self):
        tracker = LidarTracker()
        tracks = tracker.update(np.concatenate([box(), box(0, 3.2)]), 0)
        self.assertEqual(len(tracks), 2)
        partial = np.concatenate([box(.2)[::3], box(.2, 3.2)])
        updated = tracker.update(partial, .1)
        self.assertEqual([t.id for t in updated], [t.id for t in tracks])
        self.assertTrue(all(t.observed_count == 2 for t in updated))

    def test_invalid_frame_does_not_mutate(self):
        tracker = LidarTracker(max_points=60)
        before = tracker.update(box(), 1)
        bad = [(box(), 1), (box(), .9), (box(), float('nan')),
               (np.array([[np.nan, 0]]), 2), (np.array([1, 2]), 2),
               (np.zeros((61, 2)), 2), (np.array([[1e300, 0]]), 2)]
        for points, timestamp in bad:
            with self.assertRaises(ValueError):
                tracker.update(points, timestamp)
            self.assertEqual(list(tracker._tracks.values()), before)
            self.assertEqual(tracker._timestamp, 1)
        self.assertEqual(tracker.update(box(.1), 1.1)[0].observed_count, 2)

    def test_bounded_tracks_and_deterministic_order(self):
        points = np.concatenate([box(20), box(), box(10)])
        tracker = LidarTracker(max_tracks=2)
        result = tracker.update(points[::-1], 0)
        other = LidarTracker(max_tracks=2).update(points, 0)
        self.assertEqual(result, other)
        self.assertEqual(len(result), 2)


if __name__ == '__main__':
    unittest.main()
