"""Consolidation must not couple lightweight sensor helpers to SUMO imports."""
import subprocess
import sys
from pathlib import Path


def test_sensor_helpers_without_traci():
    root = Path(__file__).resolve().parents[1]
    code = """
import sys
sys.modules['traci'] = None
import traffic.live_runtime
import traffic.v2v
"""
    subprocess.run([sys.executable, '-c', code], cwd=root, check=True)


def test_legacy_public_backend_exports():
    from traffic import SumoBackend, VehicleCommand
    from traffic.sumo_backend import SumoBackend as DirectBackend
    assert SumoBackend is DirectBackend
    assert VehicleCommand(target_speed=2).target_speed == 2
