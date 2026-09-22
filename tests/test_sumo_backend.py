from pathlib import Path

import pytest

from traffic.scripted_baseline import run_episode
from traffic.sumo_backend import SumoBackend, VehicleCommand


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "scenarios" / "single_vehicle" / "scenario.sumocfg"


def test_reset_step_and_idempotent_close():
    backend = SumoBackend(CONFIG)
    initial = backend.reset(seed=42)
    assert initial.simulation_time == pytest.approx(0.2)
    assert backend.active_vehicle_ids == ("agent_0",)

    result = backend.step({"agent_0": VehicleCommand(target_speed=12.0)})
    assert result.simulation_time - initial.simulation_time == pytest.approx(0.2)
    assert result.requested_actions["agent_0"]["target_speed"] == 12.0
    assert result.states["agent_0"]["speed"] >= 8.0

    backend.close()
    backend.close()
    assert not backend.is_open


def test_rejects_action_for_inactive_vehicle():
    with SumoBackend(CONFIG) as backend:
        backend.reset(seed=42)
        with pytest.raises(KeyError):
            backend.step({"missing": VehicleCommand(target_speed=10.0)})


@pytest.mark.timeout(30)
def test_scripted_run_completes_without_collision():
    rows, summary = run_episode(CONFIG, seed=42, max_steps=600)
    assert rows
    assert summary["completed"]
    assert summary["collision_vehicle_ids"] == []
    assert any(row["lane_index"] == 1 for row in rows if row["lane_index"] is not None)


@pytest.mark.timeout(45)
def test_same_seed_replays_identically():
    first_rows, first_summary = run_episode(CONFIG, seed=42, max_steps=600)
    second_rows, second_summary = run_episode(CONFIG, seed=42, max_steps=600)
    assert first_summary == second_summary
    assert first_rows == second_rows

