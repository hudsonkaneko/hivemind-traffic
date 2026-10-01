"""Small explicit V2V intent protocol; transport never plans vehicle actions.

Positions and obstacle bounds are sender observations, not world truth. Consumers
must apply their own sensor checks before acting on a fresh intent.
"""
from dataclasses import asdict, dataclass
import heapq
import json
import math
import random

PROTOCOL_VERSION = 1


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be finite")


@dataclass(frozen=True)
class ObservedObstacle:
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def __post_init__(self):
        for name, value in asdict(self).items():
            _finite(value, name)
        if self.x_min > self.x_max or self.y_min > self.y_max:
            raise ValueError("obstacle bounds must be ordered")


@dataclass(frozen=True)
class IntentPayload:
    x: float
    y: float
    speed: float
    target_lane: int | None
    phase: str
    observed_obstacle: ObservedObstacle | None = None

    def __post_init__(self):
        for name in ("x", "y", "speed"):
            _finite(getattr(self, name), name)
        if self.speed < 0:
            raise ValueError("speed must be nonnegative")
        if self.target_lane is not None and (type(self.target_lane) is not int or self.target_lane not in (0, 1)):
            raise ValueError("target_lane must be None, 0, or 1")
        if not isinstance(self.phase, str) or not self.phase or len(self.phase) > 64:
            raise ValueError("phase must be a nonempty string of at most 64 characters")
        if self.observed_obstacle is not None and not isinstance(self.observed_obstacle, ObservedObstacle):
            raise ValueError("observed_obstacle must be observed bounds")


@dataclass(frozen=True)
class V2VMessage:
    version: int
    episode_id: str
    sender_id: str
    sequence: int
    sent_at: float
    expires_at: float
    payload: IntentPayload

    def __post_init__(self):
        if type(self.version) is not int or type(self.sequence) is not int or self.sequence < 0:
            raise ValueError("version and sequence must be integer fields")
        if not isinstance(self.episode_id, str) or not self.episode_id or not isinstance(self.sender_id, str) or not self.sender_id:
            raise ValueError("episode and sender IDs must be nonempty strings")
        _finite(self.sent_at, "sent_at")
        _finite(self.expires_at, "expires_at")
        if self.expires_at <= self.sent_at or not isinstance(self.payload, IntentPayload):
            raise ValueError("message needs a positive lifetime and valid payload")

    def byte_size(self):
        return len(json.dumps(asdict(self), sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8"))


class V2VBus:
    """Deterministic bounded simulated broadcast with receiver-local ordering.

    publish accepts a registered sender and a payload, never a supplied sender
    envelope. Identity is simulation registration, not cryptographic security.
    receive drains the named receiver's inbox. All calls use nondecreasing
    simulation time. Capacity is shared across pending deliveries and inboxes.
    """

    def __init__(self, episode_id, vehicle_ids, mode="ideal", seed=0,
                 delay_s=0.0, jitter_s=0.0, drop_probability=0.0,
                 ttl_s=1.0, max_pending=4096):
        ids = tuple(vehicle_ids)
        if not isinstance(episode_id, str) or not episode_id or not ids or any(not isinstance(v, str) or not v for v in ids) or len(set(ids)) != len(ids):
            raise ValueError("episode and unique vehicle IDs are required")
        if mode not in ("none", "ideal", "degraded"):
            raise ValueError("unknown communication mode")
        for name, value in (("delay_s", delay_s), ("jitter_s", jitter_s), ("drop_probability", drop_probability), ("ttl_s", ttl_s)):
            _finite(value, name)
        if delay_s < 0 or jitter_s < 0 or not 0 <= drop_probability <= 1 or ttl_s <= 0 or type(max_pending) is not int or max_pending < 1:
            raise ValueError("invalid transport parameters")
        self.episode_id, self.vehicle_ids, self.mode = episode_id, tuple(sorted(ids)), mode
        self.delay_s, self.jitter_s, self.drop_probability, self.ttl_s = delay_s, jitter_s, drop_probability, ttl_s
        self.max_pending = max_pending
        self._rng, self._pending, self._ordinal, self._time = random.Random(seed), [], 0, -math.inf
        self._sequences = {v: 0 for v in ids}
        self._inboxes = {v: [] for v in ids}
        self._last = {v: {} for v in ids}
        self.telemetry = {f"{kind}_{unit}": 0 for kind in ("sent", "delivered", "dropped", "expired") for unit in ("messages", "bytes")}
        self.telemetry.update(invalid_messages=0, duplicate_messages=0, out_of_order_messages=0)

    def _count(self, kind, message):
        self.telemetry[f"{kind}_messages"] += 1
        self.telemetry[f"{kind}_bytes"] += message.byte_size()

    def _advance(self, now):
        _finite(now, "now")
        if now < self._time:
            raise ValueError("simulation time cannot go backwards")
        self._time = now
        for receiver, inbox in self._inboxes.items():
            fresh = []
            for message in inbox:
                if now >= message.expires_at:
                    self._count("expired", message)
                else:
                    fresh.append(message)
            self._inboxes[receiver] = fresh
        while self._pending and self._pending[0][0] <= now:
            _, _, receiver, message = heapq.heappop(self._pending)
            if message.version != PROTOCOL_VERSION or message.episode_id != self.episode_id or message.sender_id not in self._sequences or message.sender_id == receiver or message.sent_at > now:
                self.telemetry["invalid_messages"] += 1
                self._count("dropped", message)
            elif now >= message.expires_at:
                self._count("expired", message)
            else:
                previous = self._last[receiver].get(message.sender_id, -1)
                if message.sequence <= previous:
                    self.telemetry["duplicate_messages" if message.sequence == previous else "out_of_order_messages"] += 1
                    self._count("dropped", message)
                else:
                    self._last[receiver][message.sender_id] = message.sequence
                    self._inboxes[receiver].append(message)
                    self._count("delivered", message)

    def _queue(self, receiver, message, deliver_at):
        if len(self._pending) + sum(map(len, self._inboxes.values())) >= self.max_pending:
            self._count("dropped", message)
            return
        self._ordinal += 1
        heapq.heappush(self._pending, (deliver_at, self._ordinal, receiver, message))

    def publish(self, sender_id, now, payload):
        if sender_id not in self._sequences:
            raise ValueError("sender is not registered")
        if not isinstance(payload, IntentPayload):
            raise ValueError("publish requires an IntentPayload")
        self._advance(now)
        message = V2VMessage(PROTOCOL_VERSION, self.episode_id, sender_id, self._sequences[sender_id], now, now + self.ttl_s, payload)
        self._sequences[sender_id] += 1
        if self.mode == "none":
            return message
        for receiver in self.vehicle_ids:
            if receiver == sender_id:
                continue
            self._count("sent", message)
            if self.mode == "degraded" and self._rng.random() < self.drop_probability:
                self._count("dropped", message)
                continue
            delay = self.delay_s + self._rng.uniform(0, self.jitter_s) if self.mode == "degraded" else 0.0
            self._queue(receiver, message, now + delay)
        return message

    def inject(self, receiver_id, message, deliver_at):
        """Explicit untrusted replay hook; production callers use publish."""
        if receiver_id not in self._inboxes or not isinstance(message, V2VMessage):
            raise ValueError("invalid receiver or envelope")
        _finite(deliver_at, "deliver_at")
        if deliver_at < self._time:
            raise ValueError("cannot inject into the past")
        if self.mode != "none":
            self._queue(receiver_id, message, deliver_at)

    def receive(self, receiver_id, now):
        if receiver_id not in self._inboxes:
            raise ValueError("receiver is not registered")
        self._advance(now)
        result = tuple(self._inboxes[receiver_id])
        self._inboxes[receiver_id].clear()
        return result
