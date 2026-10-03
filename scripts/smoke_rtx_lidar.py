"""Standalone RTX lidar smoke test for the installed Isaac Sim 6.0.1 API."""
import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true")
parser.add_argument("--frames", type=int, default=180)
args, _ = parser.parse_known_args()
root = Path(__file__).resolve().parents[1]
output = root / "outputs" / "lidar_smoke" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
output.mkdir(parents=True)

from isaacsim import SimulationApp
app = SimulationApp({"headless": not args.gui, "width": 640, "height": 480})
import numpy as np
import omni.usd
import omni.timeline
import omni.replicator.core as rep
from pxr import UsdGeom, UsdLux
from isaacsim.core.experimental.objects import Cube, GroundPlane
from isaacsim.core.experimental.utils.app import enable_extension
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor, parse_generic_model_output_data

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
GroundPlane("/World/Ground")
for name, position in (("Front", [5, 0, 1]), ("Left", [0, 5, 1]), ("Right", [0, -5, 1])):
    Cube("/World/" + name, positions=np.array(position), sizes=2.0)
UsdLux.DomeLight.Define(stage, "/World/Light").CreateIntensityAttr(800)
if args.gui:
    enable_extension("isaacsim.sensors.rtx.nodes")
lidar = Lidar.create("/World/Lidar", config="Example_Rotary", translations=np.array([0, 0, 1.0]))
sensor = LidarSensor(lidar, annotators=[])
records = []

class SmokeWriter(rep.Writer):
    def __init__(self):
        self.data_structure = "renderProduct"
        self.annotators = [rep.annotators.get("GenericModelOutput")]

    def write(self, data):
        for product in data.get("renderProducts", {}).values():
            raw = product.get("GenericModelOutput")
            if isinstance(raw, dict):
                raw = raw.get("data")
            if raw is None:
                continue
            gmo = parse_generic_model_output_data(raw)
            if not gmo.numElements:
                continue
            xyz = np.column_stack([np.asarray(gmo.x), np.asarray(gmo.y), np.asarray(gmo.z)])
            finite = np.isfinite(xyz).all(axis=1)
            front_rays = finite & (np.abs(xyz[:, 0]) < 2) & (np.abs(xyz[:, 1]) < 2)
            front_hits = front_rays & (np.abs(xyz[:, 2] - 4.0) < 0.15)
            records.append({"elements": int(gmo.numElements), "finite_elements": int(finite.sum()),
                            "front_box_hits": int(front_hits.sum()), "timestamp_ns": int(gmo.timestampNs),
                            "simulation_time": omni.timeline.get_timeline_interface().get_current_time()})
            if len(records) == 10:
                np.save(output / "raw_gmo_xyz.npy", xyz)

rep.WriterRegistry.register(SmokeWriter)
sensor.attach_writer("SmokeWriter")
if args.gui:
    sensor.attach_writer("draw-point-cloud", size=0.04, color=[0, 1, 0.5, 1])
stage.Export(str(output / "scene.usda"))
timeline = omni.timeline.get_timeline_interface()
try:
    timeline.play()
    for _ in range(args.frames):
        app.update()
    passed = len(records) >= 10 and sum(r["front_box_hits"] for r in records) > 0
    summary = {"passed": passed, "config": "Example_Rotary", "sensor_position_m": [0, 0, 1],
               "frames_requested": args.frames, "sensor_frames": records,
               "raw_columns": ["azimuth_degrees", "elevation_degrees", "range_m"],
               "scope": "Static RTX sensor test with front box face at 4 m; no traffic or vehicle physics validation."}
    (output / "summary.json").write_text(json.dumps(summary, indent=2))
    print("LIDAR_SMOKE=" + json.dumps({"passed": passed, "output": str(output), "sensor_frames": len(records)}), flush=True)
finally:
    timeline.stop()
    app.close()
if not passed:
    raise SystemExit(1)
