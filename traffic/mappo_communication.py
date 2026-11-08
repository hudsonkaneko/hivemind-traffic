"""Deterministic simulated V2V transport; actors see delivered payloads only.

Positions are longitudinal metres on this fixed-lane highway. Payload byte counts
exclude radio/transport headers. Range is tested at transmission; receive ranking
uses the transmitted (possibly stale) position, never live sender state.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import heapq
import json
import math
import random
from typing import Mapping


@dataclass(frozen=True)
class V2VMessage:
    sender_id: str
    episode_id: str
    sequence: int
    timestamp_s: float
    position_m: float
    lane: int
    speed_m_s: float
    acceleration_m_s2: float
    intended_acceleration_m_s2: float
    intended_lane_delta: int | None = None

    def serialize(self) -> bytes:
        payload=asdict(self)
        if self.intended_lane_delta is None:payload.pop("intended_lane_delta")
        elif self.intended_lane_delta not in (-1,0,1):raise ValueError("Lane intent must be -1, 0 or 1")
        return json.dumps(payload, sort_keys=True, separators=(",", ":"),
                          allow_nan=False).encode("utf-8")


class V2VChannel:
    """One broadcast per sender per update interval, with per-recipient loss.

    ``latency_s`` is a two-element uniform range. Outages are half-open episode
    time intervals and suppress both transmission and delivery. ``none`` exposes
    no messages; ``full`` bypasses range only, preserving timing and loss.
    """
    DEFAULTS = dict(mode="local", range_m=150.0, max_neighbors=6, update_hz=10.0,
                    latency_s=[0.1, 0.1], packet_loss=0.0, expiry_s=0.5,
                    outages_s=[])

    def __init__(self, config: Mapping | None = None):
        self.config = {**self.DEFAULTS, **dict(config or {})}
        unknown = set(self.config) - set(self.DEFAULTS)
        if unknown:
            raise ValueError(f"Unknown communication settings: {sorted(unknown)}")
        c = self.config
        if c["mode"] not in ("none", "local", "full"):
            raise ValueError("Communication mode must be none, local or full")
        for key in ("range_m", "update_hz", "expiry_s"):
            if not math.isfinite(c[key]) or c[key] <= 0:
                raise ValueError(f"{key} must be finite and positive")
        if not isinstance(c["max_neighbors"], int) or c["max_neighbors"] < 1:
            raise ValueError("max_neighbors must be a positive integer")
        if not 0 <= c["packet_loss"] <= 1:
            raise ValueError("packet_loss must be in [0, 1]")
        lo, hi = c["latency_s"]
        if not all(math.isfinite(x) for x in (lo, hi)) or not 0 <= lo <= hi:
            raise ValueError("latency_s must be finite, ordered and nonnegative")
        for start, end in c["outages_s"]:
            if not all(math.isfinite(x) for x in (start, end)) or not 0 <= start < end:
                raise ValueError("outages_s must contain nonnegative ordered intervals")
        self.reset(0)

    def reset(self, seed: int, episode_id: str = "episode") -> None:
        self.episode_id = str(episode_id)
        self._rng = random.Random(int(seed))
        self._queue = []
        self._order = 0
        self._sequence = {}
        self._last_sent = {}
        self._inbox = {}
        self._highest_sequence = {}
        self._time = 0.0
        self._stats = dict(broadcast_messages=0, broadcast_bytes=0,
                           recipient_attempts=0, delivered_messages=0,
                           delivered_bytes=0, dropped_loss=0, dropped_outage=0,
                           dropped_range=0, dropped_stale=0,
                           dropped_out_of_order=0, dropped_episode=0,
                           expired_inbox=0, delivery_delay_sum_s=0.0,
                           max_delivery_delay_s=0.0)

    def _check_time(self, now_s: float) -> None:
        if not math.isfinite(now_s) or now_s < self._time - 1e-9:
            raise ValueError("Communication time must be finite and nondecreasing")
        self._time = max(self._time, float(now_s))

    def _outage(self, time_s: float) -> bool:
        return any(start <= time_s < end for start, end in self.config["outages_s"])

    def broadcast(self, now_s: float, states: Mapping[str, Mapping]) -> None:
        self._check_time(now_s)
        if self.config["mode"] == "none":
            return
        for sender in sorted(states):
            if now_s - self._last_sent.get(sender, -math.inf) < 1 / self.config["update_hz"] - 1e-9:
                continue
            state = states[sender]
            sequence = self._sequence.get(sender, 0)
            msg = V2VMessage(sender, self.episode_id, sequence, float(now_s),
                             float(state["position_m"]), int(state["lane"]),
                             float(state["speed_m_s"]), float(state["acceleration_m_s2"]),
                             float(state["intended_acceleration_m_s2"]),state.get("intended_lane_delta"))
            payload = msg.serialize()  # immutable snapshot, validates finite values
            self._last_sent[sender] = now_s
            self._sequence[sender] = sequence + 1
            self._stats["broadcast_messages"] += 1
            self._stats["broadcast_bytes"] += len(payload)
            for receiver in sorted(states):
                if receiver == sender:
                    continue
                self._stats["recipient_attempts"] += 1
                if (self.config["mode"] == "local" and
                        abs(float(states[receiver]["position_m"]) - msg.position_m) > self.config["range_m"]):
                    self._stats["dropped_range"] += 1
                    continue
                if self._outage(now_s):
                    self._stats["dropped_outage"] += 1
                    continue
                if self._rng.random() < self.config["packet_loss"]:
                    self._stats["dropped_loss"] += 1
                    continue
                delay = self._rng.uniform(*self.config["latency_s"])
                self._order += 1
                heapq.heappush(self._queue, (now_s + delay, self._order, receiver, payload))

    def _deliver(self, now_s: float) -> None:
        while self._queue and self._queue[0][0] <= now_s + 1e-9:
            due, _, receiver, payload = heapq.heappop(self._queue)
            msg = V2VMessage(**json.loads(payload))
            if msg.episode_id != self.episode_id:
                self._stats["dropped_episode"] += 1
                continue
            if self._outage(due) or self._outage(now_s):
                self._stats["dropped_outage"] += 1
                continue
            if now_s - msg.timestamp_s > self.config["expiry_s"] + 1e-9:
                self._stats["dropped_stale"] += 1
                continue
            key = (receiver, msg.sender_id)
            if msg.sequence <= self._highest_sequence.get(key, -1):
                self._stats["dropped_out_of_order"] += 1
                continue
            self._highest_sequence[key] = msg.sequence
            self._inbox.setdefault(receiver, {})[msg.sender_id] = msg
            delay = now_s - msg.timestamp_s
            self._stats["delivered_messages"] += 1
            self._stats["delivered_bytes"] += len(payload)
            self._stats["delivery_delay_sum_s"] += delay
            self._stats["max_delivery_delay_s"] = max(self._stats["max_delivery_delay_s"], delay)

    def receive(self, receiver_id: str, now_s: float, position_m: float) -> list[V2VMessage]:
        self._check_time(now_s)
        if not math.isfinite(position_m):
            raise ValueError("Receiver position must be finite")
        self._deliver(now_s)
        inbox = self._inbox.get(receiver_id, {})
        for sender in list(inbox):
            if now_s - inbox[sender].timestamp_s > self.config["expiry_s"] + 1e-9:
                del inbox[sender]
                self._stats["expired_inbox"] += 1
        if self._outage(now_s):
            return []
        return sorted(inbox.values(), key=lambda m: (abs(m.position_m - position_m), m.sender_id))[:self.config["max_neighbors"]]

    def telemetry(self) -> dict:
        result = dict(self._stats)
        result["pending_deliveries"] = len(self._queue)
        result["mean_delivery_delay_s"] = result["delivery_delay_sum_s"] / max(1, result["delivered_messages"])
        return result
