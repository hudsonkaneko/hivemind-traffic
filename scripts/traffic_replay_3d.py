"""Build a layered SUMO replay and run moving RTX lidar in Isaac Sim 6.0.1."""
import argparse
import json
from pathlib import Path
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument('--gui', action='store_true')
parser.add_argument('--frames', type=int, default=240)
args, _ = parser.parse_known_args()
root = Path(__file__).resolve().parents[1]
output = root / 'outputs' / 'sumo_replay'
data = json.loads((output / 'recording.json').read_text())
from isaacsim import SimulationApp
app = SimulationApp({'headless': not args.gui, 'width': 960, 'height': 540, 'enable_motion_bvh': True})
import omni.usd
import omni.timeline
import omni.replicator.core as rep
from pxr import UsdGeom
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor, parse_generic_model_output_data
from isaacsim.core.experimental.utils.app import enable_extension
from isaacsim.core.utils.viewports import set_camera_view

from replay_export import export_replay, vehicle_path
export_info = export_replay(output)
ids, anchors, checks = export_info['ids'], export_info['anchors'], export_info['checks']
end_seconds = export_info['end_seconds']
frames = data['frames']
# Isaac sensor and camera opinions belong to the session layer only.
omni.usd.get_context().open_stage(str(output / 'world.usda'))
app.update()
stage = omni.usd.get_context().get_stage()
stage.SetEditTarget(stage.GetSessionLayer())
ego_path = vehicle_path('agent_0')
if args.gui:
    # Sensor instances capture writer aliases at construction time.
    enable_extension('isaacsim.sensors.rtx.nodes')
lidar = Lidar.create(ego_path+'/Lidar', config='Example_Rotary', translations=np.array([-2.5,0,1.8]))
sensor = LidarSensor(lidar, annotators=[])
records = []
class ReplayWriter(rep.Writer):
    def __init__(self):
        self.data_structure = 'renderProduct'
        self.annotators = [rep.annotators.get('GenericModelOutput')]
    def write(self, data):
        for product in data.get('renderProducts', {}).values():
            raw = product.get('GenericModelOutput')
            if isinstance(raw, dict): raw = raw.get('data')
            if raw is None: continue
            gmo = parse_generic_model_output_data(raw)
            if not gmo.numElements: continue
            az, el, distance = np.asarray(gmo.x), np.asarray(gmo.y), np.asarray(gmo.z)
            valid = np.isfinite(distance) & (distance>0) & (distance<100)
            time = omni.timeline.get_timeline_interface().get_current_time()
            position = UsdGeom.Xformable(stage.GetPrimAtPath(ego_path+'/Lidar')).ComputeLocalToWorldTransform(time*60).ExtractTranslation()
            records.append(dict(timestamp_ns=int(gmo.timestampNs), playback_time=time,
                                sensor_position_m=list(position), returns=int(valid.sum())))
            if len(records) in (5, 15, 25):
                np.savez_compressed(output/f'scan_{len(records):03}.npz', azimuth_deg=az[valid],
                                    elevation_deg=el[valid], range_m=distance[valid], timestamp_ns=int(gmo.timestampNs))
rep.WriterRegistry.register(ReplayWriter)
sensor.attach_writer('ReplayWriter')
if args.gui:
    sensor.attach_writer('draw-point-cloud', size=.08, color=[0,1,.5,1])
first_ego = next(v for v in frames[0]['vehicles'] if v['id']=='agent_0')
set_camera_view(eye=np.array([first_ego['x']-18,first_ego['y']-22,18]),
                target=np.array([first_ego['x']+12,first_ego['y'],0]))
# Runtime render-product creation may author into the root despite the session
# edit target. Never save that runtime stage over the portable replay on disk.
timeline = omni.timeline.get_timeline_interface()
timeline.set_end_time(end_seconds)
timeline.set_looping(False)
timeline.play()
try:
    for index in range(args.frames):
        app.update()
        t = timeline.get_current_time()
        ego_position = UsdGeom.Xformable(stage.GetPrimAtPath(ego_path)).ComputeLocalToWorldTransform(t*60).ExtractTranslation()
        set_camera_view(eye=np.array(ego_position)+[-18,-22,18],
                        target=np.array(ego_position)+[8,0,0])
        if index == args.frames//2:
            from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
            capture = capture_viewport_to_file(get_active_viewport(), str(output/'preview.png'))
    moved = len(records)>1 and np.linalg.norm(np.array(records[-1]['sensor_position_m'])-records[0]['sensor_position_m'])>1
    passed = moved and sum(r['returns'] for r in records)>100
    report = dict(passed=bool(passed), vehicle_ids=ids, endpoint_checks=checks,
                  coordinate_map='SUMO x/y -> USD x/y; Z up, metres; yaw=90-heading; front-bumper origin',
                  road_anchors=anchors[:2], source_timestep=.2, time_codes_per_second=60,
                  interpolation='linear positions and yaw between source samples',
                  seed=data['seed'], checkpoint_sha256=data['checkpoint_sha256'],
                  network_sha256=data['network_sha256'], sensor_mount_m=[-2.5,0,1.8],
                  sensor_frames=records, limitation='Kinematic replay and sensor generation, not physics or lidar-based driving')
    (output/'replay_report.json').write_text(json.dumps(report, indent=2))
    print('REPLAY_RESULT='+json.dumps(dict(passed=bool(passed), sensor_frames=len(records), output=str(output))), flush=True)
finally:
    timeline.stop()
    sensor.detach_writer('ReplayWriter')
    app.close()
if not passed: raise SystemExit(1)
