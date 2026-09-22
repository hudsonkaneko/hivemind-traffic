"""Build a layered SUMO replay and run moving RTX lidar in Isaac Sim 6.0.1."""
import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
import sys
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
from pxr import Usd, UsdGeom, UsdLux, Gf, Sdf
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor, parse_generic_model_output_data
from isaacsim.core.experimental.utils.app import enable_extension
from isaacsim.core.utils.viewports import set_camera_view

def vehicle_path(vehicle_id):
    return '/World/Vehicles/v_' + vehicle_id.encode('utf-8').hex()

def pose(vehicle):
    return (vehicle['x'], vehicle['y'], 0.0), 90.0 - vehicle['heading']

def stage_file(name):
    stage = Usd.Stage.CreateNew(str(output / name))
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    stage.SetTimeCodesPerSecond(60)
    stage.SetFramesPerSecond(60)
    return stage

def cube(stage, path, position, scale, color):
    shape = UsdGeom.Cube.Define(stage, path)
    shape.CreateSizeAttr(1)
    shape.AddTranslateOp().Set(Gf.Vec3d(*position))
    shape.AddScaleOp().Set(Gf.Vec3f(*scale))
    shape.CreateDisplayColorAttr([Gf.Vec3f(*color)])

# Vehicle origin is the SUMO front bumper, +X forward. Body extends backward.
asset = stage_file('vehicle.usda')
asset.SetDefaultPrim(UsdGeom.Xform.Define(asset, '/Vehicle').GetPrim())
cube(asset, '/Vehicle/Body', (-0.5, 0, 0.65), (1, 1, 1.1), (0.12, 0.45, 0.85))
asset.GetRootLayer().Save()
static = stage_file('road.usda')
UsdGeom.Xform.Define(static, '/World')
cube(static, '/World/Ground', (450, 110, -0.2), (1100, 500, 0.2), (0.12, 0.2, 0.1))
anchors = []
for index, lane in enumerate(ET.parse(output / 'network.net.xml').findall('.//lane')):
    points = [tuple(map(float, p.split(','))) for p in lane.attrib['shape'].split()]
    width = float(lane.attrib.get('width', 3.2))
    for segment, (a, b) in enumerate(zip(points, points[1:])):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length = math.hypot(dx, dy)
        if length < 0.001:
            continue
        anchors.extend([a, b])
        nx, ny = -dy/length*width/2, dx/length*width/2
        mesh = UsdGeom.Mesh.Define(static, f'/World/Road/l{index}s{segment}')
        mesh.CreatePointsAttr([(a[0]+nx,a[1]+ny,0), (a[0]-nx,a[1]-ny,0),
                              (b[0]-nx,b[1]-ny,0), (b[0]+nx,b[1]+ny,0)])
        mesh.CreateFaceVertexCountsAttr([4])
        mesh.CreateFaceVertexIndicesAttr([0,1,2,3])
        mesh.CreateSubdivisionSchemeAttr('none')
        mesh.CreateDisplayColorAttr([Gf.Vec3f(.17,.18,.20)])
UsdLux.DomeLight.Define(static, '/World/Light').CreateIntensityAttr(1000)
static.GetRootLayer().Save()
dynamic = stage_file('motion.usda')
frames = data['frames']
t0 = frames[0]['time']
ids = sorted({v['id'] for f in frames for v in f['vehicles']})
ops = {}
for vehicle_id in ids:
    first = next(v for f in frames for v in f['vehicles'] if v['id']==vehicle_id)
    path = vehicle_path(vehicle_id)
    transform = UsdGeom.Xform.Define(dynamic, path)
    transform.GetPrim().SetCustomDataByKey('sumo_id', vehicle_id)
    ops[vehicle_id] = (transform.AddTranslateOp(), transform.AddRotateZOp(), transform.CreateVisibilityAttr())
    body = UsdGeom.Xform.Define(dynamic, path+'/Model')
    body.GetPrim().GetReferences().AddReference('./vehicle.usda')
    body.AddScaleOp().Set(Gf.Vec3f(first['length'], first['width'], 1))
    body.GetPrim().SetInstanceable(True)
for frame in frames:
    tc = (frame['time']-t0)*60
    present = {v['id']: v for v in frame['vehicles']}
    for vehicle_id in ids:
        trans, rot, vis = ops[vehicle_id]
        vis.Set('inherited' if vehicle_id in present else 'invisible', tc)
        if vehicle_id in present:
            position, yaw = pose(present[vehicle_id])
            trans.Set(Gf.Vec3d(*position), tc)
            rot.Set(yaw, tc)
dynamic.GetRootLayer().Save()
world = stage_file('world.usda')
world.GetRootLayer().subLayerPaths = ['./motion.usda', './road.usda']
world.SetStartTimeCode(0)
world.SetEndTimeCode((frames[-1]['time']-t0)*60)
world.SetTimeCodesPerSecond(60)
world.SetFramesPerSecond(60)
world.GetRootLayer().Save()
# Check the composed first/last transforms for each ID and the coordinate contract.
assert pose(dict(x=10,y=20,heading=90))[0] == (10,20,0)
assert pose(dict(x=0,y=0,heading=0))[1] == 90
assert anchors[0] != anchors[1]
checks = 0
for vehicle_id in ids:
    samples = [(f, v) for f in frames for v in f['vehicles'] if v['id']==vehicle_id]
    for frame, vehicle in (samples[0], samples[-1]):
        tc = (frame['time']-t0)*60
        matrix = UsdGeom.Xformable(world.GetPrimAtPath(vehicle_path(vehicle_id))).ComputeLocalToWorldTransform(tc)
        assert np.allclose(matrix.ExtractTranslation(), pose(vehicle)[0], atol=1e-6)
        checks += 1
assert not world.GetCompositionErrors()
end_seconds = world.GetEndTimeCode()/60
del world, dynamic, static, asset, ops, transform, body
omni.usd.get_context().open_stage(str(output / 'world.usda'))
app.update()
stage = omni.usd.get_context().get_stage()
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
stage.GetRootLayer().Save()
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
