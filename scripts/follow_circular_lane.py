"""Drive the Leatherback clockwise around a circular lane using pure pursuit.

This is the ground-truth baseline for future camera-based lane perception. The
controller reads the simulated car pose, not pixels or painted-line geometry.
"""

import argparse
import json
import math
from pathlib import Path


parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true", help="Show the Isaac Sim window")
parser.add_argument("--hold-open", action="store_true", help="Keep the GUI open after the run")
parser.add_argument("--steps", type=int, default=4500, help="Simulation steps at 60 Hz")
parser.add_argument("--lane", type=int, choices=(1, 2, 3), default=2)
parser.add_argument("--speed", type=float, default=3.0, help="Target speed in m/s")
parser.add_argument("--lookahead", type=float, default=3.0, help="Pure-pursuit lookahead in m")
args = parser.parse_args()

from isaacsim import SimulationApp

app = SimulationApp({"headless": not args.gui})

import numpy as np
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot.experimental.wheeled_robots.controllers import AckermannController
from isaacsim.storage.native import get_assets_root_path


PROJECT = Path(__file__).resolve().parents[1]
SCENE = PROJECT / "scenes" / "highway" / "v1" / "highway_v1.usda"
OUTPUT = PROJECT / "outputs" / "lane_follow_v1"
DT = 1.0 / 60.0
WHEELBASE = 0.32
WHEEL_RADIUS = 0.052
LANE_WIDTH = 3.7
CENTER_RADIUS = 30.0
LANE_RADIUS = CENTER_RADIUS + (args.lane - 2) * LANE_WIDTH
ROAD_INNER_RADIUS = CENTER_RADIUS - 1.5 * LANE_WIDTH
ROAD_OUTER_RADIUS = CENTER_RADIUS + 1.5 * LANE_WIDTH


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def yaw_from_wxyz(quaternion: np.ndarray) -> float:
    w, x, y, z = quaternion
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


opened, _ = stage_utils.open_stage(str(SCENE))
if not opened:
    raise RuntimeError(f"Could not open scene: {SCENE}")

asset = get_assets_root_path() + "/Isaac/Robots/NVIDIA/Leatherback/leatherback.usd"
stage_utils.add_reference_to_stage(usd_path=asset, path="/World/Car")
car = Articulation("/World/Car")
steering = car.get_dof_indices(["Knuckle__Upright__Front_Left", "Knuckle__Upright__Front_Right"])
wheels = car.get_dof_indices([
    "Wheel__Knuckle__Front_Left", "Wheel__Knuckle__Front_Right",
    "Wheel__Upright__Rear_Left", "Wheel__Upright__Rear_Right",
])
controller = AckermannController(
    wheel_base=WHEELBASE,
    track_width=0.2416,
    front_wheel_radius=WHEEL_RADIUS,
    back_wheel_radius=WHEEL_RADIUS,
)

# This stock collision group is unnecessary for a single car and is not
# supported by the replicated PhysX setup used elsewhere in this project.
collision_group = stage_utils.get_current_stage(backend="usd").GetPrimAtPath("/World/Car/CollisionGroup")
if collision_group.IsValid():
    collision_group.SetActive(False)

# Author the initial pose before PhysX creates the articulation. Moving the
# already-simulating root of this highly constrained vehicle can destabilize
# its suspension joints in some Isaac Sim builds.
start_yaw = -math.pi / 2
car.set_world_poses(
    positions=np.array([[LANE_RADIUS, 0.0, 0.115]], dtype=np.float32),
    orientations=np.array([[math.cos(start_yaw / 2), 0.0, 0.0, math.sin(start_yaw / 2)]], dtype=np.float32),
)

SimulationManager.setup_simulation(dt=DT, device="cpu")
app_utils.play()
for _ in range(60):
    app.update()

trace = []
lane_errors = []
heading_errors = []
previous_angle = 0.0
clockwise_progress = 0.0
left_road = False

for step in range(args.steps):
    positions, orientations = car.get_world_poses()
    position = positions.numpy()[0]
    quaternion = orientations.numpy()[0]
    x, y = float(position[0]), float(position[1])
    radius = math.hypot(x, y)
    angle = math.atan2(y, x)
    yaw = yaw_from_wxyz(quaternion)

    if step:
        # Clockwise motion decreases the polar angle.
        clockwise_progress += -wrap(angle - previous_angle)
    previous_angle = angle

    lane_error = radius - LANE_RADIUS
    desired_yaw = angle - math.pi / 2
    heading_error = wrap(desired_yaw - yaw)
    lane_errors.append(lane_error)
    heading_errors.append(heading_error)
    left_road = left_road or radius < ROAD_INNER_RADIUS or radius > ROAD_OUTER_RADIUS

    # Pick a target on the same lane, lookahead metres clockwise around the arc.
    target_angle = angle - args.lookahead / LANE_RADIUS
    target_x = LANE_RADIUS * math.cos(target_angle)
    target_y = LANE_RADIUS * math.sin(target_angle)
    dx, dy = target_x - x, target_y - y
    target_bearing = math.atan2(dy, dx)
    alpha = wrap(target_bearing - yaw)
    steering_angle = math.atan2(2 * WHEELBASE * math.sin(alpha), args.lookahead)
    steering_angle = float(np.clip(steering_angle, -0.5, 0.5))

    joint_positions, joint_velocities = controller.forward([steering_angle, 0, args.speed, 0, DT])
    car.set_dof_position_targets(joint_positions, dof_indices=steering)
    car.set_dof_velocity_targets(joint_velocities, dof_indices=wheels)
    app.update()

    if step % 30 == 0:
        linear, _ = car.get_velocities()
        trace.append({
            "time_s": step * DT,
            "position": [x, y, float(position[2])],
            "speed_mps": float(np.linalg.norm(linear.numpy()[0, :2])),
            "lane_error_m": lane_error,
            "heading_error_rad": heading_error,
            "steering_rad": steering_angle,
            "clockwise_laps": clockwise_progress / math.tau,
        })

lane_errors_array = np.asarray(lane_errors)
heading_errors_array = np.asarray(heading_errors)
elapsed = args.steps * DT
actual_distance = max(0.0, clockwise_progress * LANE_RADIUS)
expected_distance = args.speed * elapsed
summary = {
    "scene": str(SCENE.relative_to(PROJECT)),
    "lane": args.lane,
    "lane_radius_m": LANE_RADIUS,
    "target_speed_mps": args.speed,
    "elapsed_simulation_s": elapsed,
    "clockwise_laps": clockwise_progress / math.tau,
    "distance_around_lane_m": actual_distance,
    "rms_lane_error_m": float(np.sqrt(np.mean(lane_errors_array**2))),
    "max_abs_lane_error_m": float(np.max(np.abs(lane_errors_array))),
    "rms_heading_error_deg": float(np.degrees(np.sqrt(np.mean(heading_errors_array**2)))),
    "left_road": left_road,
}
summary["passed"] = bool(
    not left_road
    and summary["rms_lane_error_m"] < 0.75
    and actual_distance > 0.65 * expected_distance
)

OUTPUT.mkdir(parents=True, exist_ok=True)
(OUTPUT / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
(OUTPUT / "trace.json").write_text(json.dumps(trace, indent=2), encoding="utf-8")
print("LANE_FOLLOW_RESULT=" + json.dumps(summary), flush=True)

if args.gui and args.hold_open:
    print("Simulation complete; close Isaac Sim or press Ctrl+C to exit.", flush=True)
    while app.is_running():
        app.update()

app_utils.stop()
app.close(exit_code=0 if summary["passed"] else 1)
