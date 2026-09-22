"""Portable SUMO recording to OpenUSD export; no Isaac Sim imports."""
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from pxr import Usd, UsdGeom, UsdLux, Gf

def vehicle_path(vehicle_id):
    return '/World/Vehicles/v_' + vehicle_id.encode('utf-8').hex()

def pose(vehicle):
    return (vehicle['x'], vehicle['y'], 0.0), 90.0 - vehicle['heading']

def stage_file(output, name):
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


def export_replay(output):
    output = Path(output)
    data = json.loads((output / 'recording.json').read_text())
    # Vehicle origin is the SUMO front bumper, +X forward. Body extends backward.
    asset = stage_file(output, 'vehicle.usda')
    asset.SetDefaultPrim(UsdGeom.Xform.Define(asset, '/Vehicle').GetPrim())
    cube(asset, '/Vehicle/Body', (-0.5, 0, 0.65), (1, 1, 1.1), (0.12, 0.45, 0.85))
    asset.GetRootLayer().Save()
    static = stage_file(output, 'road.usda')
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
    dynamic = stage_file(output, 'motion.usda')
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
    world = stage_file(output, 'world.usda')
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
            yaw = math.radians(pose(vehicle)[1])
            assert np.allclose(matrix.TransformDir(Gf.Vec3d(1,0,0)),
                               (math.cos(yaw), math.sin(yaw), 0), atol=1e-6)
            checks += 1
    assert not world.GetCompositionErrors()
    end_seconds = world.GetEndTimeCode()/60
    return dict(ids=ids, anchors=anchors, checks=checks, end_seconds=end_seconds)
