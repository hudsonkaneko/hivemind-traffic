import heapq

import pytest

from traffic.mappo_communication import V2VChannel, V2VMessage


def states():
    return {name: dict(position_m=position, lane=0, speed_m_s=20.0,
                       acceleration_m_s2=-1.0, intended_acceleration_m_s2=-0.5)
            for name, position in (("a", 0.0), ("b", 40.0), ("c", 1000.0))}


def test_delayed_immutable_payload_and_byte_accounting():
    channel = V2VChannel()
    data = states()
    channel.broadcast(0.0, data)
    data["b"]["speed_m_s"] = 999.0
    assert channel.receive("a", 0.0, 0.0) == []
    messages = channel.receive("a", 0.1, 0.0)
    assert len(messages) == 1
    assert messages[0].speed_m_s == 20.0
    assert messages[0].timestamp_s == 0
    stats = channel.telemetry()
    assert stats["broadcast_messages"] == 3
    assert stats["recipient_attempts"] == 6
    assert stats["delivered_messages"] == 2
    assert stats["dropped_range"] == 4
    assert stats["delivered_bytes"] == (len(messages[0].serialize()) +
                                        len(channel.receive("b", 0.1, 40)[0].serialize()))
    assert stats["mean_delivery_delay_s"] == pytest.approx(0.1)


def test_rate_expiration_and_reset():
    channel = V2VChannel()
    channel.broadcast(0, states())
    channel.broadcast(0.05, states())
    assert channel.telemetry()["broadcast_messages"] == 3
    assert channel.receive("a", 0.1, 0)
    assert channel.receive("a", 0.6, 0) == []
    assert channel.telemetry()["expired_inbox"] == 1
    channel.reset(12, "new")
    assert channel.receive("a", 0, 0) == []
    assert channel.telemetry()["broadcast_messages"] == 0


def test_seeded_loss_delay_independent_of_mapping_order():
    left = V2VChannel(dict(latency_s=[0, 0.3], packet_loss=0.2))
    right = V2VChannel(dict(latency_s=[0, 0.3], packet_loss=0.2))
    left.reset(31)
    right.reset(31)
    for step in range(20):
        now = step / 10
        left.broadcast(now, states())
        right.broadcast(now, dict(reversed(list(states().items()))))
        assert left.receive("a", now, 0) == right.receive("a", now, 0)
    assert left.telemetry() == right.telemetry()


def test_no_communication_loss_and_outage():
    for config, counter in ((dict(mode="none"), "broadcast_messages"),
                            (dict(packet_loss=1), "delivered_messages"),
                            (dict(outages_s=[[0, 2]]), "delivered_messages")):
        channel = V2VChannel(config)
        channel.broadcast(0, states())
        assert channel.receive("a", 0.1, 0) == []
        assert channel.telemetry()[counter] == 0


def test_full_range_and_nearest_limit():
    channel = V2VChannel(dict(mode="full", max_neighbors=1))
    channel.broadcast(0, states())
    assert channel.receive("a", 0.1, 0)[0].sender_id == "b"
    assert channel.receive("c", 0.1, 1000)[0].sender_id == "b"
    assert channel.telemetry()["delivered_messages"] == 6


def enqueue(channel, sequence, timestamp, due, episode="episode"):
    message = V2VMessage("b", episode, sequence, timestamp, 40, 0, 20, 0, 0)
    channel._order += 1
    heapq.heappush(channel._queue, (due, channel._order, "a", message.serialize()))


def test_duplicate_out_of_order_and_wrong_episode_rejected():
    channel = V2VChannel()
    enqueue(channel, 2, 0.1, 0.1)
    enqueue(channel, 1, 0, 0.2)
    enqueue(channel, 2, 0.1, 0.2)
    enqueue(channel, 3, 0.1, 0.2, "old")
    assert channel.receive("a", 0.2, 0)[0].sequence == 2
    assert channel.telemetry()["dropped_out_of_order"] == 2
    assert channel.telemetry()["dropped_episode"] == 1


def test_delayed_expired_payload_never_enters_inbox():
    channel = V2VChannel(dict(latency_s=[0.6, 0.6]))
    channel.broadcast(0, states())
    assert channel.receive("a", 0.6, 0) == []
    assert channel.telemetry()["dropped_stale"] == 2


def test_outage_drops_pending_delivery():
    channel = V2VChannel(dict(outages_s=[[0.05, 1.05]]))
    channel.broadcast(0, states())
    assert channel.receive("a", 0.1, 0) == []
    assert channel.telemetry()["dropped_outage"] == 2


@pytest.mark.parametrize("config", [dict(latency_s=[-1, 0]), dict(packet_loss=2),
                                   dict(expiry_s=0), dict(max_neighbors=0),
                                   dict(outages_s=[[2, 1]]), dict(mode="magic")])
def test_invalid_channel_settings(config):
    with pytest.raises(ValueError):
        V2VChannel(config)


def test_time_cannot_rewind_or_payload_contain_nan():
    channel = V2VChannel()
    channel.receive("a", 1, 0)
    with pytest.raises(ValueError):
        channel.broadcast(0, states())
    data = states()
    data["a"]["speed_m_s"] = float("nan")
    with pytest.raises(ValueError):
        channel.broadcast(1, data)
