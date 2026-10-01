"""CPU checks for atomic fleet command validation and one SUMO step per batch."""
import sys
from types import SimpleNamespace

import pytest

from traffic.live_runtime import ROOT, find_sumo

BINARY = find_sumo()
sys.path.append(str(BINARY.parent.parent / "tools"))
from traffic import fleet_sumo


class FakeConnection:
    def __init__(self):
        self.calls = []
        self.time = 0.0
        self.active = ["ego", "peer"]
        self.vehicle = SimpleNamespace(
            setSpeed=lambda *args: self.calls.append(("speed", *args)),
            changeLane=lambda *args: self.calls.append(("lane", *args)),
            getIDList=lambda: self.active,
            getSubscriptionResults=self.subscription,
        )
        self.simulation = SimpleNamespace(
            getTime=lambda: self.time,
            getCollidingVehiclesIDList=lambda: [],
        )

    def subscription(self, vehicle_id):
        constants = fleet_sumo.traci.constants
        return {
            constants.VAR_POSITION: (40, -4.8) if vehicle_id == "ego" else (34, -1.6),
            constants.VAR_SPEED: 6 if vehicle_id == "ego" else 2,
            constants.VAR_ANGLE: 90,
        }

    def simulationStep(self):
        self.calls.append(("step",))
        self.time += .1


def fake_fleet():
    fleet = fleet_sumo.FleetTraffic.__new__(fleet_sumo.FleetTraffic)
    fleet.connection = FakeConnection()
    return fleet


@pytest.mark.parametrize("actions", [
    {"ego": (6, None)},
    {"ego": (6, None), "peer": (2, None), "other": (1, None)},
    {"ego": (6, 1), "peer": (-1, None)},
    {"ego": (6, 1), "peer": (float("nan"), None)},
    {"ego": (6, 1), "peer": (float("inf"), None)},
    {"ego": (6, 1), "peer": (2, 2)},
])
def test_invalid_batch_has_no_partial_mutation(actions):
    fleet = fake_fleet()
    before = fleet.snapshot()
    with pytest.raises(ValueError):
        fleet.step(actions)
    assert fleet.connection.calls == []
    assert fleet.snapshot() == before


def test_whole_batch_precedes_exactly_one_advancement():
    fleet = fake_fleet()
    state = fleet.step({"ego": (6, 1), "peer": (2, None)})
    assert fleet.connection.calls == [
        ("lane", "ego", 1, 5.),
        ("speed", "ego", 6.),
        ("speed", "peer", 2.),
        ("step",),
    ]
    assert state["time"] == pytest.approx(.1)
    assert set(state["vehicles"]) == {"ego", "peer"}
    assert state["vehicles"]["ego"] != state["vehicles"]["peer"]
    assert state["vehicles"]["ego"] is not state["vehicles"]["peer"]
    fleet.step({"peer": (3, 0), "ego": (5, None)})
    assert fleet.connection.calls.count(("step",)) == 2


def test_snapshot_fails_when_controlled_vehicle_missing():
    fleet = fake_fleet()
    fleet.connection.active = ["ego"]
    with pytest.raises(RuntimeError, match="left fixture"):
        fleet.snapshot()


def test_real_sumo_distinct_vehicles_and_single_tick():
    fleet = fleet_sumo.FleetTraffic(
        BINARY, ROOT / "scenarios/live_lidar/scenario.sumocfg", seed=42,
    )
    try:
        initial = fleet.snapshot()
        assert set(initial["vehicles"]) == {"ego", "peer"}
        assert initial["vehicles"]["ego"]["x"] == pytest.approx(40)
        assert initial["vehicles"]["peer"]["x"] == pytest.approx(34)
        assert initial["vehicles"]["ego"]["y"] == pytest.approx(-4.8)
        assert initial["vehicles"]["peer"]["y"] == pytest.approx(-1.6)
        state = fleet.step({"ego": (6, None), "peer": (2, None)})
        assert state["time"] - initial["time"] == pytest.approx(.1)
        assert not state["collisions"]
        assert state["vehicles"]["ego"]["x"] > initial["vehicles"]["ego"]["x"]
        assert state["vehicles"]["peer"]["speed"] < state["vehicles"]["ego"]["speed"]
        for vehicle_id in fleet.vehicle_ids:
            assert fleet.connection.vehicle.getLaneChangeMode(vehicle_id) == 512
            assert fleet.connection.vehicle.getSpeedMode(vehicle_id) == 31
    finally:
        fleet.close()
        fleet.close()
        assert fleet.connection is None
