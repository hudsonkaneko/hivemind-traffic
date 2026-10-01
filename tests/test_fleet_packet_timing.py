"""Scan completion uses acquisition metadata even with narrow valid-hit spans."""
import unittest

import numpy as np

from traffic.realtime_lidar import packet_points


EGO = dict(x=0., y=0., speed=6., angle=90.)


def packet():
    count = 1000
    return dict(xyz=np.tile([10., 0., 1.], (count, 1)),
                flags=np.full(count, 64, dtype=np.uint8),
                offset=np.linspace(35_000_000, 100_000_000, count),
                timestamp=1_000_000_000, scan_complete=1)


class FleetPacketTimingTests(unittest.TestCase):
    def test_complete_scan_accepts_65ms_valid_hit_span(self):
        points, healthy, age = packet_points(packet(), EGO, 1.1,
                                             require_scan_complete=True)
        self.assertTrue(healthy)
        self.assertEqual(points.shape, (1000, 2))
        self.assertAlmostEqual(age, .1)

    def test_completion_flag_zero_or_missing_rejected(self):
        for present in (True, False):
            data = packet()
            if present:
                data['scan_complete'] = 0
            else:
                del data['scan_complete']
            _, healthy, _ = packet_points(data, EGO, 1.1,
                                          require_scan_complete=True)
            self.assertFalse(healthy)

    def test_legacy_default_rejects_narrow_hit_span(self):
        _, healthy, _ = packet_points(packet(), EGO, 1.1)
        self.assertFalse(healthy)

    def test_stale_scan_origin_rejected_despite_recent_returns(self):
        # Earliest returned ray is only 35ms old, while scan origin is 100ms
        # old: origin freshness controls the complete-scan contract.
        data = packet()
        data['offset'] = np.linspace(65_000_000, 100_000_000, 1000)
        self.assertLess(1.1-(data['timestamp']+data['offset'].min())*1e-9, .05)
        _, healthy, age = packet_points(data, EGO, 1.1, max_age=.05,
                                       require_scan_complete=True)
        self.assertFalse(healthy)
        self.assertAlmostEqual(age, .1)

    def test_negative_oversized_and_nonfinite_offsets_rejected(self):
        for invalid in (-1., 110_000_001., float('nan'), float('inf')):
            data = packet()
            data['offset'][0] = invalid
            _, healthy, _ = packet_points(data, EGO, 1.1,
                                          require_scan_complete=True)
            self.assertFalse(healthy)

    def test_future_ray_or_future_scan_rejected(self):
        for origin, now in ((1_000_000_000, 1.05), (1_200_000_000, 1.1)):
            data = packet()
            data['timestamp'] = origin
            _, healthy, _ = packet_points(data, EGO, now,
                                          require_scan_complete=True)
            self.assertFalse(healthy)


if __name__ == '__main__':
    unittest.main()
