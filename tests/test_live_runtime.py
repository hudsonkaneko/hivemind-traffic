import pytest
from traffic.live_runtime import find_isaac, find_sumo


def test_sumo_override(monkeypatch, tmp_path):
    fake = tmp_path/'sumo'
    fake.touch()
    monkeypatch.setenv('SUMO_BINARY', str(fake))
    assert find_sumo() == fake


def test_isaac_override(monkeypatch, tmp_path):
    import os
    launcher = tmp_path/('python.bat' if os.name == 'nt' else 'python.sh')
    launcher.touch()
    monkeypatch.setenv('ISAAC_SIM_PATH', str(tmp_path))
    assert find_isaac() == launcher
