"""Reject unsupported native settings before creating outputs or starting Kit."""
import copy
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from experiments import prepare_perception_dataset as capture


class CaptureConfigurationTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((capture.ROOT / "configs/perception/dataset_smoke_v01.json").read_text())

    def invoke(self, config):
        # A sentinel stops accepted configurations before run files or Kit exist.
        with patch("sys.argv", ["prepare_perception_dataset"]), \
             patch.object(Path, "cwd", return_value=capture.ROOT), \
             patch.object(Path, "read_text", return_value=json.dumps(config)), \
             patch.object(capture.subprocess, "check_output", return_value=""), \
             patch.object(capture, "run_package", side_effect=RuntimeError("accepted-config-sentinel")) as create_run, \
             patch.object(capture, "isaac_runtime") as runtime:
            try:
                capture.main()
            finally:
                runtime.assert_not_called()
                self.created_run = create_run.call_count

    def test_current_settings_accepted(self):
        with self.assertRaisesRegex(RuntimeError, "accepted-config-sentinel"):
            self.invoke(self.config)
        self.assertEqual(self.created_run, 1)

    def test_unsupported_settings_fail_before_run_creation(self):
        alternatives = {"training_enabled": True, "camera": "CameraFrontTele",
                        "resolution_wh": [1280, 720], "capture_hz": 20,
                        "target_frames_per_scene": 100}
        for key, value in alternatives.items():
            with self.subTest(setting=key):
                config = copy.deepcopy(self.config)
                config[key] = value
                with self.assertRaisesRegex(ValueError, "Unsupported smoke settings"):
                    self.invoke(config)
                self.assertEqual(self.created_run, 0)

    def test_unsupported_total_budget_rejected(self):
        self.config["limits"]["max_total_frames"] = 20000
        with self.assertRaisesRegex(ValueError, "Unsupported smoke settings"):
            self.invoke(self.config)
        self.assertEqual(self.created_run, 0)

    def test_missing_native_setting_rejected(self):
        del self.config["resolution_wh"]
        with self.assertRaisesRegex(ValueError, "Unsupported smoke settings"):
            self.invoke(self.config)
        self.assertEqual(self.created_run, 0)


if __name__ == "__main__":
    unittest.main()
