"""Headless SUMO motion owner for the separate MAPPO v01 highway fixture."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import shutil
import subprocess
from typing import Mapping
from uuid import uuid4
import xml.etree.ElementTree as ET

import numpy as np
import traci
from traci import constants as tc

from traffic.sumo_backend import find_sumo_binary

ROOT = Path(__file__).resolve().parents[1]
SCENARIO = ROOT / "scenarios" / "mappo_highway"
AGENT_IDS = tuple(f"agent_{index}" for index in range(4))
MAX_SPAWN_STEPS = 100


@dataclass(frozen=True)
class MappoStepResult:
    simulation_time: float
    states: dict[str, dict]
    departed: tuple[str, ...] = ()
    arrived: tuple[str, ...] = ()
    collisions: tuple[str, ...] = ()
    teleported: tuple[str, ...] = ()


def project_output_directory(value: str | Path | None) -> Path:
    """Keep generated scenarios and evidence inside this checkout."""
    directory = Path(value) if value is not None else ROOT / "outputs" / "sumo_mappo" / uuid4().hex
    directory = directory.resolve()
    if not directory.is_relative_to(ROOT / "outputs"):
        raise ValueError("SUMO MAPPO run_directory must be under this checkout's outputs/")
    directory.mkdir(parents=True, exist_ok=True)
    return directory


def write_episode(config: dict, seed: int, directory: Path) -> dict:
    """Create immutable seeded demand for one reset, without running simulation."""
    directory.mkdir(parents=True, exist_ok=False)
    rng = np.random.default_rng(seed)
    cap = float(config["speed_limit_m_s"])
    speed_low, speed_high = config["initial_speed_range_m_s"]
    demand = ET.Element("routes")
    ET.SubElement(demand, "vType", id="car", length="4.8", width="1.9", minGap="2.5",
                  accel="2.0", decel="4.5", emergencyDecel="9.0", tau="1.5", sigma="0.0",
                  maxSpeed=str(cap), speedFactor="1.0", speedDev="0.0", carFollowModel="Krauss")
    ET.SubElement(demand, "route", id="straight", edges="highway")
    vehicles = []
    for agent, lane, position in zip(AGENT_IDS, (0, 0, 1, 2), (220, 130, 175, 190)):
        vehicles.append(dict(id=agent, lane=lane, position=position + float(rng.uniform(-10, 10)),
                             speed=float(rng.uniform(speed_low, speed_high)), depart=0.0))
    if config.get("driving"):
        for vehicle,position in zip(vehicles,(220.,165.,110.,55.)):
            vehicle.update(lane=0,position=position,speed=20.)
    if config.get("wave"):
        for vehicle, position in zip(vehicles, (230., 175., 120., 65.)):
            vehicle.update(lane=0, position=position, speed=20.)
    for lane in range(3):
        for index, position in enumerate((65, 310, 460)):
            vehicles.append(dict(id=f"background_initial_{lane}_{index}", lane=lane,
                                 position=position + float(rng.uniform(-10, 10)),
                                 speed=float(rng.uniform(speed_low, speed_high)), depart=0.0))
    if config.get("wave"):
        # The designated leader is ahead of the controlled platoon; other initial cars cannot overlap it.
        for vehicle in vehicles:
            if vehicle["id"].startswith("background_initial_0_"):
                index = int(vehicle["id"].rsplit("_", 1)[1])
                vehicle.update(position=(15., 300., 440.)[index], speed=20.)
    if config.get("driving"):
        for vehicle in vehicles:
            if vehicle["id"]=="background_initial_0_0":vehicle.update(position=15.,speed=17.)
            if vehicle["id"]=="background_initial_0_1":vehicle.update(position=310.,speed=config["driving"]["slow_leader_speed_m_s"])
    # Ordinary vehicles keep entering. Departures may be delayed by SUMO's safe insertion.
    interval_low, interval_high = config["background_depart_interval_range_s"]
    for lane in range(3):
        depart = float(rng.uniform(interval_low, interval_high))
        index = 0
        while depart < config["dt"] * config["horizon"] + 1.0:
            vehicles.append(dict(id=f"background_flow_{lane}_{index}", lane=lane, position=0.0,
                                 speed=float(rng.uniform(speed_low, speed_high)), depart=depart))
            depart += float(rng.uniform(interval_low, interval_high))
            index += 1
    vehicles.sort(key=lambda item: (item["depart"], -item["position"], item["id"]))
    for vehicle in vehicles:
        ET.SubElement(demand, "vehicle", id=vehicle["id"], type="car", route="straight",
                      depart=f'{vehicle["depart"]:.6f}', departLane=str(vehicle["lane"]),
                      departPos=f'{vehicle["position"]:.6f}', departSpeed=f'{vehicle["speed"]:.6f}')
    ET.ElementTree(demand).write(directory / "demand.rou.xml", encoding="utf-8", xml_declaration=True)
    shutil.copy2(SCENARIO / "network.net.xml", directory / "network.net.xml")
    configuration = ET.Element("configuration")
    inputs = ET.SubElement(configuration, "input")
    ET.SubElement(inputs, "net-file", value="network.net.xml")
    ET.SubElement(inputs, "route-files", value="demand.rou.xml")
    ET.ElementTree(configuration).write(directory / "scenario.sumocfg", encoding="utf-8", xml_declaration=True)
    manifest = {"seed": seed, "config": config, "vehicles": vehicles, "controlled_ids": AGENT_IDS,
                "speed_mode": 31, "lane_change_mode": 512, "motion_owner": "SUMO",
                "generator_optimizer_steps": 0, "note": "Scenario generation does not run a learning optimizer."}
    (directory / "scenario.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


class MappoSumoBackend:
    """One private TraCI connection, one simulationStep per joint control batch."""

    def __init__(self, config: dict, run_directory=None, sumo_binary=None, *, gui=False):
        self.config = dict(config)
        self.run_directory = project_output_directory(run_directory)
        self.sumo_binary = Path(sumo_binary) if sumo_binary else find_sumo_binary(gui=gui)
        self.gui = gui
        self.connection = None
        self.simulation_steps = 0
        self.episode_directory = None
        self.version = None
        self._log_handle = None

    @property
    def active_vehicle_ids(self):
        return tuple(sorted(self.connection.vehicle.getIDList())) if self.connection else ()

    def reset(self, seed=42):
        self.close()
        self.episode_directory = self.run_directory / f"episode-{seed}-{uuid4().hex[:8]}"
        write_episode(self.config, seed, self.episode_directory)
        self._pulse_start_s = None
        if self.config.get("wave"):
            self._pulse_start_s = float(np.random.default_rng(seed + 73421).uniform(*self.config["wave"]["pulse_start_s_range"]))
        label = f"mappo-{uuid4().hex}"
        log_directory = ROOT / "logs" / "sumo_mappo"
        log_directory.mkdir(parents=True, exist_ok=True)
        log_path = log_directory / f"{label}.log"
        error_log_path = log_directory / f"{label}.error.log"
        command = [str(self.sumo_binary), "-c", str(self.episode_directory / "scenario.sumocfg"),
                   "--seed", str(seed), "--step-length", str(self.config["dt"]),
                   "--no-step-log", "true", "--duration-log.disable", "true",
                   "--xml-validation", "never", "--time-to-teleport", "-1",
                   "--collision.action", "warn", "--collision.check-junctions", "true",
                   "--quit-on-end", "true", "--error-log", str(error_log_path)]
        if self.gui:
            command.extend(["--start", "--window-size", "1280,560"])
        try:
            self._log_handle = log_path.open("w", encoding="utf-8")
            traci.start(command, label=label, stdout=self._log_handle, verbose=False)
            self.connection = traci.getConnection(label)
            if self.gui:
                # Bound GUI/rendering stalls independently of headless training.
                self.connection._socket.settimeout(15)
            self.version = self.connection.getVersion()
            self.simulation_steps = 0
            # SUMO may safely delay insertion when sampled departure speeds need
            # more braking distance. Let native traffic open the gap; do not
            # disable insertion checks or silently discard the sampled seed.
            for spawn_steps in range(1, MAX_SPAWN_STEPS + 1):
                result = self._advance()
                if result.collisions or result.teleported or set(AGENT_IDS).intersection(result.arrived):
                    raise RuntimeError("Invalid traffic event during initial spawn warm-up")
                if set(AGENT_IDS).issubset(result.states):
                    break
            else:
                missing = sorted(set(AGENT_IDS) - set(result.states))
                raise RuntimeError(f"Controlled vehicles failed to spawn after {MAX_SPAWN_STEPS} steps: {missing}")
            (self.episode_directory / "runtime.json").write_text(json.dumps({
                "sumo_version": self.version, "traci_version": traci.__version__,
                "command": command, "log": str(log_path), "error_log": str(error_log_path),
                "initial_spawn_steps": spawn_steps,
                "spawn_warmup_seconds": result.simulation_time,
            }, indent=2) + "\n", encoding="utf-8")
            self._episode_start_time = result.simulation_time
            return result
        except BaseException:
            self.close()
            raise

    def step(self, accelerations: Mapping[str, float], *, lane_changes=None, headways=None):
        if self.connection is None:
            raise RuntimeError("Backend requires reset()")
        active = set(self.active_vehicle_ids)
        for vehicle, acceleration in accelerations.items():
            if vehicle not in active or vehicle not in AGENT_IDS:
                raise KeyError(f"Cannot control inactive or non-policy vehicle {vehicle}")
            if not math.isfinite(acceleration) or not -4.5 <= acceleration <= 2.0:
                raise ValueError(f"Invalid physical acceleration {acceleration}")
        lane_changes=lane_changes or {};headways=headways or {}
        if (lane_changes or headways) and not self.config.get("driving"):
            raise ValueError("Maneuvers require v04 driving")
        for vehicle,delta in lane_changes.items():
            if vehicle not in active or vehicle not in AGENT_IDS or delta not in (-1,0,1):
                raise ValueError("Invalid lane change request")
            if not 0<=self.connection.vehicle.getLaneIndex(vehicle)+delta<3:
                raise ValueError("Lane request crosses road boundary")
        for vehicle,headway in headways.items():
            if vehicle not in active or vehicle not in AGENT_IDS or not math.isfinite(headway) or headway<.8:
                raise ValueError("Invalid native headway")
        for vehicle,headway in sorted(headways.items()):self.connection.vehicle.setTau(vehicle,float(headway))
        for vehicle,delta in sorted(lane_changes.items()):
            self.connection.vehicle.changeLane(vehicle,self.connection.vehicle.getLaneIndex(vehicle)+delta,self.config["dt"])
        wave = self.config.get("wave")
        if wave:
            elapsed = self.connection.simulation.getTime() - self._episode_start_time
            if self._pulse_start_s <= elapsed < self._pulse_start_s + wave["pulse_duration_s"]:
                leader = "background_initial_0_1"
                if leader in active:
                    self.connection.vehicle.setAcceleration(leader, wave["pulse_acceleration_m_s2"], self.config["dt"])
        for vehicle in sorted(accelerations):
            self.connection.vehicle.setAcceleration(vehicle, float(accelerations[vehicle]), self.config["dt"])
        return self._advance()

    def _advance(self):
        connection = self.connection
        connection.simulationStep()
        self.simulation_steps += 1
        departed = tuple(sorted(connection.simulation.getDepartedIDList()))
        active = set(connection.vehicle.getIDList())
        for vehicle in departed:
            if vehicle not in active:
                continue
            # Both policy and ordinary vehicles retain SUMO's safety checks, with fixed lanes.
            connection.vehicle.setSpeedMode(vehicle, 31)
            connection.vehicle.setLaneChangeMode(vehicle, 512)
            if self.config.get("driving") and vehicle=="background_initial_0_1":
                connection.vehicle.setMaxSpeed(vehicle,self.config["driving"]["slow_leader_speed_m_s"])
            connection.vehicle.subscribe(vehicle, [tc.VAR_SPEED, tc.VAR_ACCELERATION, tc.VAR_POSITION,
                tc.VAR_LANE_INDEX, tc.VAR_LANEPOSITION, tc.VAR_DISTANCE, tc.VAR_LENGTH,
                tc.VAR_ALLOWED_SPEED, tc.VAR_WAITING_TIME, tc.VAR_TIMELOSS])
        subscribed = connection.vehicle.getAllSubscriptionResults()
        states = {}
        for vehicle in sorted(active):
            values = subscribed[vehicle]
            x, y = values[tc.VAR_POSITION]
            states[vehicle] = dict(speed=float(values[tc.VAR_SPEED]),
                acceleration=float(values[tc.VAR_ACCELERATION]), lane_index=int(values[tc.VAR_LANE_INDEX]),
                position=float(values[tc.VAR_LANEPOSITION]), distance=float(values[tc.VAR_DISTANCE]),
                length=float(values[tc.VAR_LENGTH]), speed_limit=float(values[tc.VAR_ALLOWED_SPEED]),
                waiting_time=float(values[tc.VAR_WAITING_TIME]), time_loss=float(values[tc.VAR_TIMELOSS]),
                x=float(x), y=float(y))
        return MappoStepResult(float(connection.simulation.getTime()), states, departed,
            tuple(sorted(connection.simulation.getArrivedIDList())),
            tuple(sorted(connection.simulation.getCollidingVehiclesIDList())),
            tuple(sorted(connection.simulation.getStartingTeleportIDList())))

    def close(self):
        connection, self.connection = self.connection, None
        try:
            if connection is not None:
                try:
                    connection.close(wait=not self.gui)
                except traci.exceptions.FatalTraCIError:
                    # A crashed/previously disconnected SUMO process must not prevent local cleanup.
                    pass
                finally:
                    if self.gui and connection._process is not None:
                        process = connection._process
                        try:
                            process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait(timeout=5)
        finally:
            if self._log_handle is not None:
                self._log_handle.close()
                self._log_handle = None
