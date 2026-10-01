"""CPU safety contracts for receiver-local cooperative lidar decisions."""
from dataclasses import replace
import unittest

from traffic.cooperative_control import CooperativeController
from traffic.lidar_avoidance import LANES
from traffic.lidar_tracking import Track
from traffic.v2v import IntentPayload, V2VMessage


def ego(lane=0, x=40., speed=6.):
    return dict(x=x, y=LANES[lane], speed=speed)


def track(x, lane, *, vx=0., count=3, uncertainty=.75,
          last_seen=1., state_time=1., local_id=1):
    y = LANES[lane]
    return Track(local_id, x, y, vx, 0., x-2, x+2, y-.5, y+.5,
                 last_seen, count, .4, uncertainty, state_time)


def intent(*, expires_at=2., sent_at=.8):
    return V2VMessage(1, 'test', 'ego', 0, sent_at, expires_at,
                      IntentPayload(40., LANES[0], 6., 1, 'requesting'))


class CooperativeControlTests(unittest.TestCase):
    def test_stale_sensor_brakes_without_lane_command(self):
        controller = CooperativeController('ego', 0)
        decision = controller.step(ego(), [track(65, 0)], [], 1., fresh=False)
        self.assertLess(decision.speed, 6.)
        self.assertIsNone(decision.lane_request)
        self.assertIsNone(decision.intended_lane)
        self.assertFalse(decision.target_clear)
        self.assertEqual(decision.reason, 'sensor-failsafe')

    def test_moving_target_lane_blocks_merge(self):
        controller = CooperativeController('ego', 0)
        decision = controller.step(ego(), [track(65, 0), track(35, 1, vx=6., local_id=2)],
                                   [], 1., fresh=True)
        self.assertEqual(decision.phase, 'requesting')
        self.assertEqual(decision.intended_lane, 1)
        self.assertIsNone(decision.lane_request)
        self.assertFalse(decision.target_clear)

    def test_clear_target_lane_permits_merge(self):
        decision = CooperativeController('ego', 0).step(
            ego(), [track(65, 0)], [], 1., fresh=True)
        self.assertEqual(decision.lane_request, 1)
        self.assertTrue(decision.target_clear)
        self.assertEqual(decision.phase, 'outbound')

    def test_fast_rear_car_crossing_between_sampled_horizons_blocks_merge(self):
        # At 0 seconds the car is behind; by 2 seconds it is ahead. Its
        # swept relative box crosses the ego at about 1 second.
        decision = CooperativeController('ego', 0).step(
            ego(), [track(65, 0), track(20, 1, vx=26., local_id=2)],
            [], 1., fresh=True)
        self.assertFalse(decision.target_clear)
        self.assertIsNone(decision.lane_request)
        self.assertEqual(decision.phase, 'requesting')

    def test_lateral_track_entering_target_lane_blocks_merge(self):
        # Initially outside the target lane, then moving toward it while
        # maintaining the ego's longitudinal speed and position.
        crossing = replace(track(40, 1, vx=6., local_id=2),
                           y=1.6, ymin=1.1, ymax=2.1, vy=-2.)
        decision = CooperativeController('ego', 0).step(
            ego(), [track(65, 0), crossing], [], 1., fresh=True)
        self.assertFalse(decision.target_clear)
        self.assertIsNone(decision.lane_request)
        self.assertEqual(decision.phase, 'requesting')

    def test_controllers_use_only_their_supplied_tracks(self):
        blocked = CooperativeController('ego', 0)
        independent = CooperativeController('other', 0)
        obstacle = track(65, 0)
        first = blocked.step(ego(), [obstacle, track(35, 1, vx=6., local_id=2)],
                             [], 1., fresh=True)
        second = independent.step(ego(), [obstacle], [], 1., fresh=True)
        self.assertIsNone(first.lane_request)
        self.assertEqual(second.lane_request, 1)
        self.assertEqual(first.phase, 'requesting')
        self.assertEqual(blocked.phase, 'requesting')

    def test_message_yields_without_granting_clearance(self):
        peer = CooperativeController('peer', 1)
        yielding = peer.step(ego(1, x=34.), [], [intent()], 1., fresh=True)
        self.assertEqual(yielding.reason, 'yield-to-intent')
        self.assertLess(yielding.speed, 6.)
        self.assertIsNone(yielding.lane_request)
        requester = CooperativeController('ego', 0)
        blocked = requester.step(ego(), [track(65, 0), track(35, 1, vx=6., local_id=2)],
                                 [intent()], 1., fresh=True)
        self.assertFalse(blocked.target_clear)
        self.assertIsNone(blocked.lane_request)

    def test_expired_and_future_messages_do_not_yield(self):
        for message in (intent(expires_at=1.), intent(sent_at=1.1)):
            result = CooperativeController('peer', 1).step(
                ego(1, x=34.), [], [message], 1., fresh=True)
            self.assertEqual(result.phase, 'cruise')
            self.assertEqual(result.speed, 6.)

    def test_prediction_uses_state_time_without_double_counting(self):
        moving = track(24, 0, vx=20., last_seen=.7, state_time=1.)
        result = CooperativeController('ego', 0).step(
            ego(x=0.), [moving], [], 1., fresh=True)
        self.assertAlmostEqual(result.clearance, 18.75)
        # A later control tick adds only time since the predicted state.
        later = CooperativeController('ego', 0).step(
            ego(x=0.), [moving], [], 1.1, fresh=True)
        self.assertAlmostEqual(later.clearance, 20.75)

    def test_uncertain_target_lane_track_blocks_merge(self):
        for uncertain in (track(70, 1, vx=30., count=1, local_id=2),
                          track(70, 1, vx=30., uncertainty=3., local_id=2)):
            result = CooperativeController('ego', 0).step(
                ego(), [track(65, 0), uncertain], [], 1., fresh=True)
            self.assertFalse(result.target_clear)
            self.assertIsNone(result.lane_request)

    def test_expired_tracks_do_not_grant_obstacle_observation(self):
        result = CooperativeController('ego', 0).step(
            ego(), [replace(track(65, 0), last_seen=.49)], [], 1., fresh=True)
        self.assertEqual(result.phase, 'approach')
        self.assertIsNone(result.lane_request)
        self.assertIsNone(result.observed_obstacle)


if __name__ == '__main__':
    unittest.main()
