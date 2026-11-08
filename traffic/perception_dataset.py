"""Portable, checksummed camera dataset contract; no simulator or training imports.

Semantic ground truth and evaluation labels are supervision, never actor inputs.
Scene/seed splits are fixed before frames are collected.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import struct

import numpy as np

SCHEMA_VERSION = "perception-dataset-v1"
CLASSES = {"background": 0, "drivable": 1, "lane": 2, "vehicle": 3,
           "pedestrian": 4, "obstacle": 5}
SPLITS = ("train", "val", "test")
PATH_FIELDS = ("rgb_path", "semantic_path", "lidar_path", "labels_path")


def deterministic_split(scenario_id: str, seed: int) -> str:
    """Stable 80/10/10 grouping; tiny smoke sets should assign splits explicitly."""
    value = int(hashlib.sha256(f"{scenario_id}:{seed}".encode()).hexdigest()[:8], 16) % 100
    return "train" if value < 80 else "val" if value < 90 else "test"


def _file(root: Path, relative: str) -> Path:
    if not isinstance(relative, str) or not relative or Path(relative).is_absolute():
        raise ValueError("Dataset paths must be nonempty and relative")
    path = (root / relative).resolve()
    if not path.is_relative_to(root.resolve()) or not path.is_file():
        raise ValueError(f"Missing or escaping dataset file: {relative}")
    return path


def _sha(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def validate_manifest(root: str | Path, manifest: dict) -> dict:
    """Validate files, SHA256, dimensions, classes and scene split isolation.

    Returns counts suitable for a small evidence report; raises ValueError on
    corruption or leakage. PNG headers are inspected without a Pillow dependency.
    This is structural validation, not proof of label accuracy or visual diversity.
    """
    root = Path(root)
    if manifest.get("schema_version") != SCHEMA_VERSION or manifest.get("classes") != CLASSES:
        raise ValueError("Unsupported dataset schema or semantic classes")
    if manifest.get("supervision_role") != "training_and_evaluation_only":
        raise ValueError("Simulator labels must be supervision only")
    frames = manifest.get("frames", [])
    if not frames:
        raise ValueError("Dataset has no frames")
    groups, scenes, identities, content = {}, {}, set(), {}
    counts = dict.fromkeys(SPLITS, 0)
    pixels = dict.fromkeys(CLASSES, 0)
    for frame in frames:
        for key in ("scene_id", "scenario_id", "camera_id"):
            if not isinstance(frame.get(key), str) or not frame[key]:
                raise ValueError(f"Missing frame identity: {key}")
        seed = frame.get("seed")
        if type(seed) is not int:
            raise ValueError("Seed must be an integer")
        split = frame.get("split")
        if split not in SPLITS:
            raise ValueError("Unknown split")
        stamp = frame.get("timestamp_s")
        if not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or stamp < 0:
            raise ValueError("Invalid frame timestamp")
        key = (frame["scene_id"], frame["camera_id"], stamp)
        if key in identities:
            raise ValueError("Duplicate frame identity")
        identities.add(key)
        group = (frame["scenario_id"], seed)
        if groups.setdefault(group, split) != split:
            raise ValueError("Scenario/seed group leaks across splits")
        scene_contract = (group, split)
        if scenes.setdefault(frame["scene_id"], scene_contract) != scene_contract:
            raise ValueError("Scene identity leaks or changes scenario/seed")
        for required in ("rgb_path", "semantic_path"):
            if required not in frame:
                raise ValueError(f"Missing {required}")
        for field in PATH_FIELDS:
            if frame.get(field) is not None:
                path = _file(root, frame[field])
                if frame.get("sha256", {}).get(field) != _sha(path):
                    raise ValueError(f"Checksum mismatch: {field}")
        rgb = _file(root, frame["rgb_path"])
        with rgb.open("rb") as stream:
            header = stream.read(24)
        if len(header) != 24 or header[:8] != b"\x89PNG\r\n\x1a\n" or header[12:16] != b"IHDR":
            raise ValueError("RGB file must have a PNG IHDR")
        width, height = struct.unpack(">II", header[16:24])
        labels = np.load(_file(root, frame["semantic_path"]), allow_pickle=False)
        if labels.ndim != 2 or labels.shape != (height, width) or not np.issubdtype(labels.dtype, np.integer):
            raise ValueError("Semantic mask must be integer HxW matching RGB")
        if labels.size == 0 or not set(np.unique(labels)).issubset(CLASSES.values()):
            raise ValueError("Unknown semantic class or empty frame")
        for name, class_id in CLASSES.items():
            pixels[name] += int(np.count_nonzero(labels == class_id))
        signature = (frame["sha256"]["rgb_path"], frame["sha256"]["semantic_path"])
        if content.setdefault(signature, split) != split:
            raise ValueError("Identical RGB/semantic pair leaks across splits")
        counts[split] += 1
    requested = manifest.get("metadata", {}).get("requested_classes", list(CLASSES))
    if not set(requested).issubset(CLASSES):
        raise ValueError("Unknown requested class")
    return {"frame_count": len(frames), "scene_count": len(scenes), "split_frames": counts,
            "class_pixel_counts": pixels, "missing_requested_classes": [name for name in requested if pixels[name] == 0],
            "structurally_valid": True, "visual_label_review_required": True}


def write_manifest(root: str | Path, frames: list[dict], metadata: dict) -> dict:
    """Hash existing files, validate, then write root/manifest.json; return manifest.

    Required frame keys: scene_id, scenario_id, seed, split, timestamp_s,
    camera_id, rgb_path, semantic_path. Optional lidar_path and labels_path are
    checksummed only; their calibration/schema belongs in metadata.
    """
    root = Path(root)
    enriched = []
    for supplied in frames:
        frame = dict(supplied)
        frame["sha256"] = {field: _sha(_file(root, frame[field])) for field in PATH_FIELDS if frame.get(field) is not None}
        enriched.append(frame)
    manifest = {"schema_version": SCHEMA_VERSION, "classes": CLASSES,
                "supervision_role": "training_and_evaluation_only", "metadata": metadata, "frames": enriched}
    manifest["validation"] = validate_manifest(root, manifest)
    destination = root / "manifest.json"
    if destination.exists():
        raise FileExistsError("Preserve existing dataset manifests; use a fresh output directory")
    destination.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest
