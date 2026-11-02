"""Public geometry fixtures verify replay contracts without distributing private cars."""
import json

import pytest

pytest.importorskip('pxr')
from pxr import Gf, Usd, UsdGeom
from scripts.build_visual_twin import build_scene, new_stage, vehicle_path


def test_pose_heading_lifecycle_and_immutable_inputs(tmp_path):
    asset = new_stage(tmp_path/'car.usda', '/Vehicle', 'component')
    UsdGeom.Cube.Define(asset, '/Vehicle/Body')
    asset.GetRootLayer().Save()
    network=tmp_path/'network.xml'
    network.write_text('<net><edge><lane shape="0,0 100,0" width="3.2"/></edge></net>')
    recording=tmp_path/'recording.json'
    def v(x,heading):
        return dict(id='car / alpha',x=x,y=20,heading=heading,speed=4)
    recording.write_text(json.dumps(dict(frames=[dict(time=10,vehicles=[]),dict(time=11,vehicles=[v(10,90)]),dict(time=12,vehicles=[v(14,0)]),dict(time=13,vehicles=[])])))
    before={p:p.read_bytes() for p in [recording,network,tmp_path/'car.usda']}
    output=tmp_path/'scene'
    result=build_scene(recording,network,tmp_path/'car.usda',output)
    stage=Usd.Stage.Open(str(output/'world.usda'))
    prim=stage.GetPrimAtPath(vehicle_path('car / alpha'))
    assert stage.GetDefaultPrim().GetPath()=='/World'
    assert result['duration_s']==3
    assert not stage.GetCompositionErrors()
    assert UsdGeom.Imageable(prim).ComputeVisibility(0)=='invisible'
    assert UsdGeom.Imageable(prim).ComputeVisibility(180)=='invisible'
    a=UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(60)
    b=UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(120)
    assert (a.ExtractTranslation()-Gf.Vec3d(10,20,.02)).GetLength()<1e-6
    assert (b.TransformDir(Gf.Vec3d(1,0,0))-Gf.Vec3d(0,1,0)).GetLength()<1e-6
    assert all(p.read_bytes()==content for p,content in before.items())
    with pytest.raises(FileExistsError):
        build_scene(recording,network,tmp_path/'car.usda',output)


def test_invalid_time_sequence_rejected(tmp_path):
    recording=tmp_path/'bad.json'
    recording.write_text(json.dumps(dict(frames=[dict(time=2,vehicles=[]),dict(time=1,vehicles=[])])))
    with pytest.raises(ValueError,match='increasing'):
        build_scene(recording,tmp_path/'network.xml',tmp_path/'car.usda',tmp_path/'scene')


def test_heading_crosses_north_without_full_spin(tmp_path):
    asset=new_stage(tmp_path/'car.usda','/Vehicle','component')
    UsdGeom.Cube.Define(asset,'/Vehicle/Body'); asset.GetRootLayer().Save()
    network=tmp_path/'network.xml'; network.write_text('<net><edge><lane shape="0,0 0,100"/></edge></net>')
    recording=tmp_path/'recording.json'
    recording.write_text(json.dumps(dict(frames=[dict(time=i,vehicles=[dict(id='car',x=0,y=i,heading=h)]) for i,h in enumerate((359,1))])))
    build_scene(recording,network,tmp_path/'car.usda',tmp_path/'scene')
    stage=Usd.Stage.Open(str(tmp_path/'scene/world.usda'))
    direction=UsdGeom.Xformable(stage.GetPrimAtPath(vehicle_path('car'))).ComputeLocalToWorldTransform(30).TransformDir(Gf.Vec3d(1,0,0))
    assert (direction-Gf.Vec3d(0,1,0)).GetLength()<1e-6
