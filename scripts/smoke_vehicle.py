"""Exercise the stock Leatherback steering, propulsion, and brakes."""
import argparse
import json
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true")
args = parser.parse_args()

from isaacsim import SimulationApp

app = SimulationApp({"headless": not args.gui})
import numpy as np
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.objects import GroundPlane, DomeLight
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot.experimental.wheeled_robots.controllers import AckermannController
from isaacsim.storage.native import get_assets_root_path

GroundPlane("/World/Ground", templates=None, colors=[0.2, 0.22, 0.25])
DomeLight("/World/Light").set_intensities(1000)
asset = get_assets_root_path() + "/Isaac/Robots/NVIDIA/Leatherback/leatherback.usd"
print("Loading vehicle:", asset, flush=True)
stage_utils.add_reference_to_stage(usd_path=asset, path="/World/Car")
car = Articulation("/World/Car")
steering = car.get_dof_indices(["Knuckle__Upright__Front_Left", "Knuckle__Upright__Front_Right"])
wheels = car.get_dof_indices([
    "Wheel__Knuckle__Front_Left", "Wheel__Knuckle__Front_Right",
    "Wheel__Upright__Rear_Left", "Wheel__Upright__Rear_Right",
])
controller = AckermannController(wheel_base=0.32, track_width=0.2416,
                                front_wheel_radius=0.052, back_wheel_radius=0.052)
SimulationManager.setup_simulation(dt=1 / 60, device="cpu")
app_utils.play()
for _ in range(60):
    app.update()
p, q = car.get_world_poses()
trajectory = [{"phase": "initial", "position": p.numpy().tolist(), "orientation": q.numpy().tolist()}]
print("JOINT_NAMES=" + str(car.dof_names), flush=True)
for name, angle, speed, steps in [("forward", 0, 0.6, 240), ("turn", 0.3, 0.6, 240), ("brake", 0, 0, 120)]:
    positions, velocities = controller.forward([angle, 0, speed, 0, 1 / 60])
    for _ in range(steps):
        car.set_dof_position_targets(positions, dof_indices=steering)
        car.set_dof_velocity_targets(velocities, dof_indices=wheels)
        app.update()
    p, q = car.get_world_poses()
    linear, angular = car.get_velocities()
    trajectory.append({"phase": name, "position": p.numpy().tolist(), "orientation": q.numpy().tolist(), "linear_velocity": linear.numpy().tolist()})
    print("VEHICLE_PHASE=" + json.dumps(trajectory[-1]), flush=True)
out = Path(__file__).resolve().parents[1] / "outputs"
out.mkdir(exist_ok=True)
(out / "vehicle_smoke.json").write_text(json.dumps(trajectory, indent=2))
stage_utils.get_current_stage(backend="usd").Export(str(out / "vehicle_scene.usda"))
travel = np.linalg.norm(np.array(trajectory[1]["position"])[0, :2] - np.array(trajectory[0]["position"])[0, :2])
stopped_speed = float(np.linalg.norm(trajectory[-1]["linear_velocity"][0][:2]))
turned = abs(float(trajectory[2]["orientation"][0][3])) > 0.2
passed = bool(travel > 1.5 and stopped_speed < 0.05 and turned)
print("VEHICLE_RESULT=" + json.dumps({"forward_distance_m": float(travel), "stopped_speed": stopped_speed, "turned": turned, "passed": passed}), flush=True)
app_utils.stop()
app.close(exit_code=0 if passed else 1)
