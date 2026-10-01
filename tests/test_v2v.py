import random
import unittest
from dataclasses import replace

from traffic.v2v import IntentPayload, ObservedObstacle, V2VBus


class V2VTests(unittest.TestCase):
    def payload(self):
        return IntentPayload(40, 0, 6, 1, "requesting", ObservedObstacle(99, 101, -1, 1))

    def test_modes_and_receiver_isolation(self):
        for mode, expected in (("none", 0), ("ideal", 1)):
            bus = V2VBus("episode", ["ego", "peer"], mode=mode)
            message = bus.publish("ego", 0, self.payload())
            self.assertEqual(len(bus.receive("peer", 0)), expected)
            self.assertEqual(bus.receive("ego", 0), ())
            self.assertEqual(bus.receive("peer", 0), ())
            self.assertEqual(bus.telemetry["sent_messages"], expected)
            self.assertEqual(bus.telemetry["sent_bytes"], message.byte_size() * expected)

    def test_duplicate_old_sequence_wrong_episode_version_and_stale(self):
        bus = V2VBus("episode", ["ego", "peer"])
        original = bus.publish("ego", 0, self.payload())
        self.assertEqual(bus.receive("peer", 0), (original,))
        bus.inject("peer", original, 0)
        self.assertEqual(bus.receive("peer", 0), ())
        newer = bus.publish("ego", .1, self.payload())
        self.assertEqual(bus.receive("peer", .1), (newer,))
        bus.inject("peer", original, .1)
        bus.inject("peer", replace(newer, episode_id="other"), .1)
        bus.inject("peer", replace(newer, version=2), .1)
        self.assertEqual(bus.receive("peer", .1), ())
        bus.inject("peer", replace(newer, sequence=9), 2)
        self.assertEqual(bus.receive("peer", 2), ())
        self.assertEqual(bus.telemetry["duplicate_messages"], 1)
        self.assertEqual(bus.telemetry["out_of_order_messages"], 1)
        self.assertEqual(bus.telemetry["invalid_messages"], 2)
        self.assertEqual(bus.telemetry["expired_messages"], 1)

    def test_exact_seeded_drop_and_delay(self):
        seed, chance = 17, .35
        bus = V2VBus("episode", ["ego", "peer"], mode="degraded", seed=seed,
                     delay_s=.2, jitter_s=.4, drop_probability=chance, ttl_s=100)
        rng = random.Random(seed)
        expected, dropped = [], 0
        for index in range(10):
            now = float(index)
            message = bus.publish("ego", now, self.payload())
            if rng.random() < chance:
                dropped += 1
                self.assertEqual(bus.receive("peer", now + .9), ())
            else:
                arrival = now + (.2 + rng.uniform(0, .4))
                self.assertEqual(bus.receive("peer", arrival - 1e-8), ())
                self.assertEqual(bus.receive("peer", arrival), (message,))
                expected.append(message)
        self.assertEqual(bus.telemetry["dropped_messages"], dropped)
        self.assertEqual(bus.telemetry["delivered_messages"], len(expected))

    def test_bounded_queue_and_inbox_expiry(self):
        bus = V2VBus("episode", ["ego", "peer"], max_pending=1)
        bus.publish("ego", 0, self.payload())
        bus.publish("ego", 0, self.payload())
        self.assertEqual(bus.telemetry["dropped_messages"], 1)
        self.assertEqual(bus.receive("peer", 1), ())
        self.assertEqual(bus.telemetry["expired_messages"], 1)

    def test_validation_and_sender_envelope(self):
        bus = V2VBus("episode", ["ego", "peer"])
        with self.assertRaises(ValueError):
            bus.publish("imposter", 0, self.payload())
        with self.assertRaises(ValueError):
            bus.publish("ego", float("nan"), self.payload())
        for kwargs in ({"speed": -1}, {"x": float("inf")}, {"target_lane": True}, {"phase": ""}):
            with self.assertRaises(ValueError):
                replace(self.payload(), **kwargs)
        bus.receive("peer", 2)
        with self.assertRaises(ValueError):
            bus.receive("peer", 1)


if __name__ == "__main__":
    unittest.main()
