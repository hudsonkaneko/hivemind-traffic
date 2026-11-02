import math
import pytest
from pxr import Usd, UsdGeom
from scripts.build_i110_comparison import LaneAttitude, car


def test_lane_attitude_uses_lane_distance_and_grade(tmp_path):
    p=tmp_path/'net.xml'
    p.write_text('<net><edge id="e"><lane id="e_0" length="20" '
                 'shape="0,0,0 0,10,1 10,10,2"/></edge></net>')
    reader=LaneAttitude(p)
    assert reader.rotation('e_0',5)==pytest.approx((0,-math.degrees(math.atan(.1)),90))
    assert reader.rotation('e_0',15)==pytest.approx((0,-math.degrees(math.atan(.1)),0))


def test_generated_car_is_self_contained_with_front_bumper_anchor(tmp_path):
    p=tmp_path/'car.usda';car(p,(.1,.3,.8))
    stage=Usd.Stage.Open(str(p))
    assert stage.GetDefaultPrim().GetPath()=='/Vehicle'
    assert not stage.GetCompositionErrors() and not stage.GetRootLayer().GetExternalReferences()
    body=UsdGeom.BBoxCache(0,['default']).ComputeWorldBound(stage.GetPrimAtPath('/Vehicle/Body')).ComputeAlignedRange()
    assert body.GetMax()[0]==pytest.approx(0,abs=1e-6)
    assert body.GetMin()[0]==pytest.approx(-4.8,abs=1e-6)
