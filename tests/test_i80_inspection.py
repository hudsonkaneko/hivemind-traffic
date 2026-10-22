import json
import math

import pytest

pytest.importorskip('pxr')
from pxr import Gf, Usd, UsdGeom
from scripts.build_i80_inspection import (ORIGIN_FT, SURVEY_FOOT_M,
    build_inspection, clip_segment, local_metres, observation_recording)
from scripts.build_visual_twin import new_stage, vehicle_path


def row(t=1000, x=6042500, y=2132800):
    return dict(location='i-80',vehicle_id='1',global_time=str(t),
                global_x=str(x),global_y=str(y),v_vel='10')


def test_coordinate_axes_and_clipping():
    assert local_metres(ORIGIN_FT) == (0,0)
    assert local_metres((ORIGIN_FT[0]+100,ORIGIN_FT[1]+200)) == pytest.approx(
        (100*SURVEY_FOOT_M,200*SURVEY_FOOT_M))
    assert clip_segment((-10,5),(20,5),(0,0,10,10)) == [(0,5),(10,5)]
    assert clip_segment((-1,0),(-1,10),(0,0,10,10)) is None
    with pytest.raises(ValueError):
        local_metres((math.nan,0))


def test_observations_do_not_invent_trips_or_bridge_gaps():
    data = observation_recording([row(),row(1100,x=6042501)])
    assert 'accounting' not in data['frames'][0]
    assert data['frames'][0]['vehicles'][0]['heading'] == pytest.approx(90)
    assert data['frames'][1]['time'] == .1
    with pytest.raises(ValueError,match='Duplicate'):
        observation_recording([row(),row()])
    with pytest.raises(ValueError,match='gap'):
        observation_recording([row(),row(1200)])


def test_inspection_composes_original_viewer_contract_without_sumo_claim(tmp_path):
    asset=new_stage(tmp_path/'car.usda','/Vehicle','component')
    UsdGeom.Cube.Define(asset,'/Vehicle/Body');asset.GetRootLayer().Save()
    sample=tmp_path/'sample.json'
    sample.write_text(json.dumps([row(),row(1100,x=6042501)]))
    gis=tmp_path/'gis.json'
    gis.write_text(json.dumps({'emeryville':[{'record_index':0,
        'attributes':{'TYPE':'Highway'},'paths':[[ORIGIN_FT,(6042501,2132800)]]}],
        'camera-coverage':[]}))
    esri=tmp_path/'esri.json'
    esri.write_text(json.dumps({'spatialReference':{'wkid':102643,'latestWkid':2227},'features':[]}))
    inputs={p:p.read_bytes() for p in (sample,gis,esri,tmp_path/'car.usda')}
    output=tmp_path/'scene'
    manifest=build_inspection(sample,gis,esri,tmp_path/'car.usda',output)
    stage=Usd.Stage.Open(str(output/'world.usda'))
    assert not stage.GetCompositionErrors()
    assert list(stage.GetRootLayer().subLayerPaths) == ['./motion.usda','./environment.usda']
    prim=stage.GetPrimAtPath(vehicle_path('ngsim_1'))
    assert not prim.GetCustomDataByKey('sumo_id')
    assert prim.GetCustomDataByKey('external_id') == 'ngsim_1'
    assert 'NGSIM' in prim.GetCustomDataByKey('motion_authority')
    transform=UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(6)
    assert tuple(transform.ExtractTranslation()) == pytest.approx((SURVEY_FOOT_M,0,.02))
    assert tuple(transform.TransformDir(Gf.Vec3d(1,0,0))) == pytest.approx((1,0,0))
    assert 'NOT trip completion' in manifest['viewer_footer']
    assert 'SITE NOT ADMITTED' in manifest['viewer_heading']
    assert all(p.read_bytes()==b for p,b in inputs.items())
    with pytest.raises(FileExistsError):
        build_inspection(sample,gis,esri,tmp_path/'car.usda',output)
