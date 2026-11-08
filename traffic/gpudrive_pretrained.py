"""Frozen CPU inference for the author's released GPUDrive 10k-scene policy.

This is a SUMO-independent feature encoder and inference port, not GPUDrive's
simulator. The checkpoint-era source is pinned deliberately: later upstream
versions changed vehicle-size normalization. No training or downloads occur.

Architecture adapted from Emerge-Lab/gpudrive (MIT), commit
70e7fadd5d938caebceeb8d42f87ba5f081b6c7d, networks/late_fusion.py.
Copyright (c) 2024 Saman Kazemkhani, Aarav Pandya, Daphne Cornelisse,
Brennan Shacklett, Eugene Vinitsky.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

from __future__ import annotations

import hashlib
import json
import math
from enum import IntEnum
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np
import torch
from torch import nn


MODEL_REPOSITORY = "daphne-cornelisse/policy_S10_000_02_27"
MODEL_REVISION = "1532950cad84dafc6e9d976a2bcc524ee481a1a1"
SOURCE_REVISION = "70e7fadd5d938caebceeb8d42f87ba5f081b6c7d"
WEIGHTS_SHA256 = "f3f26475def35f375c6c72d8f8f20b2b091f77175010345dc3fa968a860521b7"
CONFIG_SHA256 = "e3f4cb2599f8b36aa8da4eb5c8fcb520a8e9b799ced35bfa6806fb80e0e98829"
OBSERVATION_DIM = 2984
MAX_PARTNERS = 63
MAX_ROAD_SEGMENTS = 200
OBSERVATION_RADIUS_M = 50.0

EGO_FEATURES = (
    "speed", "length", "width", "local_goal_x", "local_goal_y", "is_collided"
)
PARTNER_FEATURES = (
    "speed", "local_x", "local_y", "relative_yaw", "length", "width"
)
ROAD_FEATURES = (
    "local_midpoint_x", "local_midpoint_y", "half_length", "half_width",
    "half_height", "relative_yaw", "none", "road_edge", "road_line",
    "road_lane", "crosswalk", "speed_bump", "stop_sign",
)


class RoadType(IntEnum):
    NONE = 0
    EDGE = 1
    LINE = 2
    LANE = 3
    CROSSWALK = 4
    SPEED_BUMP = 5
    STOP_SIGN = 6


def _normalize_position(values: np.ndarray) -> np.ndarray:
    # Preserve the author's float32 operation order, including its rounding.
    return 2 * ((values - np.float32(-1000)) / np.float32(2000)) - 1


def _relative_yaw(yaw: float, ego_yaw: float) -> float:
    return math.atan2(math.sin(yaw - ego_yaw), math.cos(yaw - ego_yaw))


def encode_observation(
    ego: Mapping[str, float],
    partners: Sequence[Mapping[str, float]],
    roads: Sequence[Mapping],
    goal: Sequence[float],
) -> np.ndarray:
    """Encode world-space vehicle centers and road segments as 2984 float32s.

    Vehicles require x, y, yaw (radians CCW from +x), speed (m/s), full length
    and full width (m). Ego may include is_collided, default false. Partners
    exclude ego. Road mappings require start/end XY points and type (RoadType
    or 0..6); half_width/half_height default to the author's 0.1 m. Their
    midpoint, HALF length and heading describe the oriented bounding box.
    Crosswalks/signs need correctly supplied geometry; no types are inferred.

    Forward is local +x, left is +y. Partners and segment midpoints outside
    50 m are omitted. Preserve input order, taking first 63 partners and first
    200 roads, matching the released linear road selection (not nearest-K).
    Extra SUMO vehicles beyond 63 are a transfer limitation of the caller's
    selected population. Missing partner slots are zero; missing roads have
    zero geometry and the None one-hot bit. The network pools these slots,
    with NO extra attention/visibility mask. No feature clipping is applied.

    Full dimensions are used here: do not pre-multiply by the simulator's
    0.7 collision-box scale, which was absent from checkpoint-era features.
    """
    ex, ey, yaw = float(ego["x"]), float(ego["y"]), float(ego["yaw"])
    c, s = math.cos(yaw), math.sin(yaw)

    def local(x: float, y: float) -> tuple[float, float]:
        dx, dy = float(x) - ex, float(y) - ey
        return c * dx + s * dy, -s * dx + c * dy

    goal_x, goal_y = local(*goal)
    ego_array = np.asarray([
        ego["speed"], ego["length"], ego["width"], goal_x, goal_y,
        float(ego.get("is_collided", False)),
    ], dtype=np.float32)
    ego_array[:3] /= np.asarray([100, 30, 15], dtype=np.float32)
    ego_array[3:5] = _normalize_position(ego_array[3:5])

    partner_array = np.zeros((MAX_PARTNERS, 6), dtype=np.float32)
    count = 0
    for vehicle in partners:
        x, y = local(vehicle["x"], vehicle["y"])
        if math.hypot(x, y) > OBSERVATION_RADIUS_M:
            continue
        partner_array[count] = [
            vehicle["speed"], x, y, _relative_yaw(vehicle["yaw"], yaw),
            vehicle["length"], vehicle["width"],
        ]
        count += 1
        if count == MAX_PARTNERS:
            break
    partner_array[:, 0] /= 100
    partner_array[:, 1:3] = _normalize_position(partner_array[:, 1:3])
    partner_array[:, 3:6] /= np.asarray([2 * math.pi, 30, 15], dtype=np.float32)

    road_array = np.zeros((MAX_ROAD_SEGMENTS, 13), dtype=np.float32)
    road_array[:, 6] = 1  # Padded road is the None category, not 13 zeros.
    count = 0
    for road in roads:
        x1, y1 = road["start"]
        x2, y2 = road["end"]
        x, y = local((x1 + x2) / 2, (y1 + y2) / 2)
        if math.hypot(x, y) > OBSERVATION_RADIUS_M:
            continue
        road_type = RoadType(road["type"])
        road_array[count, :6] = [
            x, y, math.hypot(x2 - x1, y2 - y1) / 2,
            road.get("half_width", 0.1), road.get("half_height", 0.1),
            _relative_yaw(math.atan2(y2 - y1, x2 - x1), yaw),
        ]
        road_array[count, 6] = 0
        road_array[count, 6 + road_type] = 1
        count += 1
        if count == MAX_ROAD_SEGMENTS:
            break
    road_array[:, :2] = _normalize_position(road_array[:, :2])
    road_array[:, 2:6] /= np.asarray([100, 100, 100, 2 * math.pi], dtype=np.float32)
    result = np.concatenate((ego_array, partner_array.ravel(), road_array.ravel()))
    if not np.isfinite(result).all():
        raise ValueError("GPUDrive observations must contain only finite values")
    return result


def action_values(index: int) -> tuple[float, float]:
    """Author's accel-major 7 x 13 product, rounded to three decimals.

    Steering covers [-pi, pi], not a narrower conventional steering range.
    Preserve these values when integrating the classic tan(steering) model.
    """
    if not isinstance(index, (int, np.integer)) or not 0 <= index < 91:
        raise ValueError("GPUDrive action index must be an integer in [0, 90]")
    accel = torch.round(torch.linspace(-4.0, 4.0, 7), decimals=3)
    steer = torch.round(torch.linspace(-torch.pi, torch.pi, 13), decimals=3)
    return float(accel[index // 13]), float(steer[index % 13])


class _LateFusionNetwork(nn.Module):
    """Exact released layer names, dimensions and unmasked max pooling."""

    def __init__(self) -> None:
        super().__init__()

        def embed(features: int) -> nn.Sequential:
            return nn.Sequential(
                nn.Linear(features, 64), nn.LayerNorm(64), nn.Tanh(),
                nn.Dropout(0.0), nn.Linear(64, 64),
            )

        self.ego_embed = embed(6)
        self.partner_embed = embed(6)
        self.road_map_embed = embed(13)
        self.shared_embed = nn.Sequential(nn.Linear(192, 128), nn.Dropout(0.0))
        self.actor = nn.Linear(128, 91)
        self.critic = nn.Linear(128, 1)

    def forward(self, observation: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        ego = self.ego_embed(observation[:, :6])
        partners = self.partner_embed(observation[:, 6:384].reshape(-1, 63, 6)).max(1).values
        roads = self.road_map_embed(observation[:, 384:].reshape(-1, 200, 13)).max(1).values
        shared = self.shared_embed(torch.cat((ego, partners, roads), dim=1))
        return self.actor(shared), self.critic(shared)


def _read_pinned_safetensors(path: Path) -> dict[str, torch.Tensor]:
    """Read only the checksum-pinned F32 archive; no pickle or remote code.

    This small reader avoids installing safetensors just for a 207 kB file.
    It deliberately supports only the exact reviewed checkpoint format.
    """
    raw = path.read_bytes()
    if hashlib.sha256(raw).hexdigest() != WEIGHTS_SHA256:
        raise ValueError("GPUDrive checkpoint SHA256 does not match the pinned release")
    header_size = int.from_bytes(raw[:8], "little")
    header = json.loads(raw[8:8 + header_size])
    data = raw[8 + header_size:]
    state = {}
    for name, spec in header.items():
        if name == "__metadata__":
            continue
        start, end = spec["data_offsets"]
        shape = spec["shape"]
        if spec["dtype"] != "F32" or not (0 <= start < end <= len(data)):
            raise ValueError("Unsupported checkpoint tensor format")
        if end - start != math.prod(shape) * 4:
            raise ValueError("Checkpoint tensor shape/size mismatch")
        state[name] = torch.frombuffer(bytearray(data[start:end]), dtype=torch.float32).reshape(shape).clone()
    return state


class FrozenGPUDrivePolicy:
    """CPU-only deterministic inference; all parameters have gradients disabled."""

    def __init__(self, model_directory: str | Path) -> None:
        self.model_directory = Path(model_directory)
        config = (self.model_directory / "config.json").read_bytes()
        if hashlib.sha256(config).hexdigest() != CONFIG_SHA256:
            raise ValueError("GPUDrive config SHA256 does not match the pinned release")
        self.config = json.loads(config)
        # Initial random weights are immediately replaced, without changing the
        # caller's RNG state. This evaluation never initializes an optimizer.
        with torch.random.fork_rng(devices=[]):
            self.network = _LateFusionNetwork()
        self.network.load_state_dict(
            _read_pinned_safetensors(self.model_directory / "model.safetensors"), strict=True
        )
        self.network.eval().requires_grad_(False)

    @classmethod
    def from_pretrained(cls, model_directory: str | Path) -> "FrozenGPUDrivePolicy":
        """Load the local pinned directory. This method never downloads files."""
        return cls(model_directory)

    def logits(self, observations: np.ndarray | torch.Tensor) -> np.ndarray:
        values = torch.as_tensor(observations, dtype=torch.float32, device="cpu")
        if values.ndim == 1:
            values = values.unsqueeze(0)
        if values.ndim != 2 or values.shape[1] != OBSERVATION_DIM:
            raise ValueError("Expected one or more 2984-feature observations")
        if not bool(torch.isfinite(values).all()):
            raise ValueError("GPUDrive observations must contain only finite values")
        with torch.inference_mode():
            logits, _ = self.network(values)
        return logits.numpy()

    def infer(self, observation: np.ndarray | torch.Tensor) -> tuple[float, float, int]:
        """Return acceleration (m/s²), steering (rad), and deterministic index."""
        logits = self.logits(observation)
        if len(logits) != 1:
            raise ValueError("infer expects exactly one ego observation")
        index = int(logits[0].argmax())
        acceleration, steering = action_values(index)
        return acceleration, steering, index
