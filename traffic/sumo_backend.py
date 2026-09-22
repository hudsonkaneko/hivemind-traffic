"""Lifecycle-safe SUMO/TraCI backend for deterministic traffic control."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import os
from pathlib import Path
import shutil
from typing import Mapping
from uuid import uuid4

import traci
from traci import constants as tc


@dataclass(frozen=True)
class VehicleCommand:
    """Tactical command requested for one active SUMO vehicle."""

    target_speed: float | None = None
    lane_delta: int = 0


@dataclass(frozen=True)
class StepResult:
    simulation_time: float
    states: dict[str, dict[str, float | int | str]]
    departed: tuple[str, ...]
    arrived: tuple[str, ...]
    collisions: tuple[str, ...]
    requested_actions: dict[str, dict[str, float | int | None]]
    executed_actions: dict[str, dict[str, float | int | bool | None]]


def find_sumo_binary(*, gui: bool = False) -> Path:
    """Resolve SUMO without tying the repository to one user's file system."""

    program = "sumo-gui" if gui else "sumo"
    executable = f"{program}.exe" if os.name == "nt" else program
    candidates: list[Path] = []
    if value := os.environ.get("SUMO_BINARY"):
        candidates.append(Path(value))
    if value := shutil.which(program):
        candidates.append(Path(value))
    if value := os.environ.get("SUMO_HOME"):
        candidates.append(Path(value) / "bin" / executable)
    if os.name == "nt":
        for variable in ("ProgramFiles(x86)", "ProgramFiles"):
            if root := os.environ.get(variable):
                candidates.append(Path(root) / "Eclipse" / "Sumo" / "bin" / executable)

    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise FileNotFoundError(
        "SUMO was not found. Set SUMO_BINARY or SUMO_HOME, or add SUMO to PATH."
    )


class SumoBackend:
    """Own one TraCI connection and advance SUMO exactly once per step call."""

    def __init__(
        self,
        config_path: str | Path,
        *,
        step_length: float = 0.2,
        sumo_binary: str | Path | None = None,
        gui: bool = False,
        gui_delay_ms: int = 50,
    ) -> None:
        self.config_path = Path(config_path).resolve()
        self.step_length = float(step_length)
        self.gui = bool(gui)
        self.gui_delay_ms = int(gui_delay_ms)
        self.sumo_binary = (
            Path(sumo_binary).resolve() if sumo_binary else find_sumo_binary(gui=self.gui)
        )
        self._connection = None
        self._label: str | None = None
        self._active: set[str] = set()

    @property
    def active_vehicle_ids(self) -> tuple[str, ...]:
        return tuple(sorted(self._active))

    @property
    def is_open(self) -> bool:
        return self._connection is not None

    def reset(self, seed: int = 42) -> StepResult:
        if not self.config_path.is_file():
            raise FileNotFoundError(f"SUMO configuration does not exist: {self.config_path}")
        self.close()
        self._label = f"hivemind-{uuid4().hex}"
        command = [
            str(self.sumo_binary),
            "-c",
            str(self.config_path),
            "--seed",
            str(seed),
            "--step-length",
            str(self.step_length),
            "--no-step-log",
            "true",
            "--xml-validation",
            "never",
            "--quit-on-end",
            "true",
        ]
        if self.gui:
            command.extend(["--delay", str(self.gui_delay_ms), "--start"])
        try:
            traci.start(command, label=self._label)
            self._connection = traci.getConnection(self._label)
            return self._advance({})
        except Exception:
            self.close()
            raise

    def step(self, actions: Mapping[str, VehicleCommand]) -> StepResult:
        connection = self._require_connection()
        unknown = set(actions) - self._active
        if unknown:
            raise KeyError(f"Actions referenced inactive vehicles: {sorted(unknown)}")

        requested: dict[str, dict[str, float | int | None]] = {}
        executed: dict[str, dict[str, float | int | bool | None]] = {}
        for vehicle_id in sorted(actions):
            command = actions[vehicle_id]
            requested[vehicle_id] = asdict(command)
            current_speed = float(connection.vehicle.getSpeed(vehicle_id))
            allowed_speed = float(connection.vehicle.getAllowedSpeed(vehicle_id))
            target_speed = current_speed
            speed_clamped = False
            if command.target_speed is not None:
                target_speed = max(0.0, min(float(command.target_speed), allowed_speed))
                speed_clamped = target_speed != float(command.target_speed)
                connection.vehicle.setSpeed(vehicle_id, target_speed)

            lane_delta = max(-1, min(1, int(command.lane_delta)))
            lane_clamped = lane_delta != int(command.lane_delta)
            if lane_delta:
                connection.vehicle.changeLaneRelative(vehicle_id, lane_delta, self.step_length)
            executed[vehicle_id] = {
                "target_speed": target_speed,
                "speed_clamped": speed_clamped,
                "lane_delta": lane_delta,
                "lane_clamped": lane_clamped,
            }
        return self._advance(requested, executed)

    def close(self) -> None:
        connection, self._connection = self._connection, None
        self._active.clear()
        self._label = None
        if connection is not None:
            try:
                connection.close()
            except traci.exceptions.FatalTraCIError:
                pass

    def _advance(
        self,
        requested: dict[str, dict[str, float | int | None]],
        executed: dict[str, dict[str, float | int | bool | None]] | None = None,
    ) -> StepResult:
        connection = self._require_connection()
        connection.simulationStep()
        departed = tuple(sorted(connection.simulation.getDepartedIDList()))
        arrived = tuple(sorted(connection.simulation.getArrivedIDList()))
        collisions = tuple(sorted(connection.simulation.getCollidingVehiclesIDList()))
        self._active.update(departed)
        self._active.difference_update(arrived)
        for vehicle_id in departed:
            connection.vehicle.subscribe(
                vehicle_id,
                (
                    tc.VAR_SPEED,
                    tc.VAR_ACCELERATION,
                    tc.VAR_POSITION,
                    tc.VAR_LANE_INDEX,
                    tc.VAR_LANE_ID,
                    tc.VAR_ROAD_ID,
                    tc.VAR_DISTANCE,
                    tc.VAR_WAITING_TIME,
                    tc.VAR_TIMELOSS,
                ),
            )

        states: dict[str, dict[str, float | int | str]] = {}
        for vehicle_id in sorted(self._active):
            values = connection.vehicle.getSubscriptionResults(vehicle_id) or {}
            x, y = values.get(tc.VAR_POSITION, (0.0, 0.0))
            states[vehicle_id] = {
                "x": float(x),
                "y": float(y),
                "speed": float(values.get(tc.VAR_SPEED, 0.0)),
                "acceleration": float(values.get(tc.VAR_ACCELERATION, 0.0)),
                "lane_index": int(values.get(tc.VAR_LANE_INDEX, -1)),
                "lane_id": str(values.get(tc.VAR_LANE_ID, "")),
                "road_id": str(values.get(tc.VAR_ROAD_ID, "")),
                "distance": float(values.get(tc.VAR_DISTANCE, 0.0)),
                "waiting_time": float(values.get(tc.VAR_WAITING_TIME, 0.0)),
                "time_loss": float(values.get(tc.VAR_TIMELOSS, 0.0)),
            }
        return StepResult(
            simulation_time=float(connection.simulation.getTime()),
            states=states,
            departed=departed,
            arrived=arrived,
            collisions=collisions,
            requested_actions=requested,
            executed_actions=executed or {},
        )

    def _require_connection(self):
        if self._connection is None:
            raise RuntimeError("SUMO backend is not open; call reset() first.")
        return self._connection

    def __enter__(self) -> "SumoBackend":
        return self

    def __exit__(self, *_args) -> None:
        self.close()
