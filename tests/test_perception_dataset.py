"""Dataset failures that would silently corrupt offline evaluation."""
import copy
import json
from pathlib import Path
import struct
import tempfile
import unittest
import zlib

import numpy as np

from traffic.perception_dataset import deterministic_split, validate_manifest, write_manifest


def png(path, value):
    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", 4, 3, 8, 2, 0, 0, 0))
                     + chunk(b"IDAT", zlib.compress((b"\x00" + bytes([value] * 12)) * 3)) + chunk(b"IEND", b""))


class PerceptionDatasetTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.frames = []
        for i, split in enumerate(("train", "val", "test")):
            png(self.root / f"{i}.png", i * 20)
            np.save(self.root / f"{i}.npy", np.full((3, 4), i, dtype=np.uint8))
            self.frames.append(dict(scene_id=f"scene{i}", scenario_id="obstacle", seed=i,
                                    split=split, timestamp_s=0.1, camera_id="CameraFront",
                                    rgb_path=f"{i}.png", semantic_path=f"{i}.npy"))

    def manifest(self):
        return write_manifest(self.root, self.frames, {"requested_classes": ["lane", "pedestrian"]})

    def test_roundtrip_counts_and_missing_class(self):
        manifest = self.manifest()
        self.assertEqual(manifest, json.loads((self.root / "manifest.json").read_text()))
        report = validate_manifest(self.root, manifest)
        self.assertEqual(report["split_frames"], {"train": 1, "val": 1, "test": 1})
        self.assertEqual(report["missing_requested_classes"], ["pedestrian"])

    def test_corrupt_file_rejected(self):
        manifest = self.manifest()
        (self.root / "0.png").write_bytes(b"changed")
        with self.assertRaisesRegex(ValueError, "Checksum"):
            validate_manifest(self.root, manifest)

    def test_seed_split_leak_rejected(self):
        self.frames[1]["seed"] = 0
        with self.assertRaisesRegex(ValueError, "group leaks"):
            self.manifest()

    def test_scene_split_leak_rejected(self):
        self.frames[1]["scene_id"] = "scene0"
        self.frames[1]["timestamp_s"] = 0.2
        with self.assertRaisesRegex(ValueError, "Scene identity"):
            self.manifest()

    def test_duplicate_frame_rejected(self):
        self.frames.append(copy.deepcopy(self.frames[0]))
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            self.manifest()

    def test_identical_content_across_splits_rejected(self):
        self.frames[1]["rgb_path"] = "0.png"
        self.frames[1]["semantic_path"] = "0.npy"
        with self.assertRaisesRegex(ValueError, "Identical"):
            self.manifest()

    def test_invalid_class_rejected(self):
        np.save(self.root / "0.npy", np.full((3, 4), 99, dtype=np.uint8))
        with self.assertRaisesRegex(ValueError, "Unknown semantic"):
            self.manifest()

    def test_wrong_dimensions_rejected(self):
        np.save(self.root / "0.npy", np.zeros((4, 3), dtype=np.uint8))
        with self.assertRaisesRegex(ValueError, "matching RGB"):
            self.manifest()

    def test_paths_cannot_escape(self):
        self.frames[0]["rgb_path"] = "../outside.png"
        with self.assertRaisesRegex(ValueError, "escaping"):
            self.manifest()

    def test_preserve_existing_manifest(self):
        self.manifest()
        with self.assertRaises(FileExistsError):
            self.manifest()

    def test_invalid_timestamp_rejected(self):
        self.frames[0]["timestamp_s"] = float("nan")
        with self.assertRaisesRegex(ValueError, "timestamp"):
            self.manifest()

    def test_stable_split(self):
        self.assertEqual(deterministic_split("obstacle", 42), deterministic_split("obstacle", 42))
        self.assertEqual(set(deterministic_split("obstacle", i) for i in range(100)), {"train", "val", "test"})


if __name__ == "__main__":
    unittest.main()
