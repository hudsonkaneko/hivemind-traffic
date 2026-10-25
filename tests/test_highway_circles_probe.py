import math
import json
from pathlib import Path
import pytest
from traffic.circles_control import acceleration_to_pedals
from traffic.highway_circles_probe import CEILING,lane_route,lane_only_observation,circles_inputs,guard,validate_protocol
from traffic.driver_control import AppliedControl


def state(radius=500.,angle=0.,speed=3.):
    yaw=angle+math.pi/2
    return dict(position_m=[radius*math.cos(angle),radius*math.sin(angle),.8],
        velocity_m_s=[speed*math.cos(yaw),speed*math.sin(yaw),0.],speed_m_s=speed,
        yaw_rad=yaw,quaternion_xyzw=[0.,0.,math.sin(yaw/2),math.cos(yaw/2)],
        upright_z=1.,wheel_on_ground=[True]*4)


def test_ceiling_is_60mph_and_does_not_change_legacy_default():
    assert CEILING==pytest.approx(26.8224)
    assert acceleration_to_pedals(10,1,speed_ceiling=CEILING)==pytest.approx((.42,0,10.6))
    assert acceleration_to_pedals(CEILING,1,speed_ceiling=CEILING)==(0,0,CEILING)
    assert acceleration_to_pedals(3,1)==(0,0,3)


@pytest.mark.parametrize('name,value', [('training',True),('physics_hz',60),
    ('traffic_target_speed_m_s',26.8224),('no_leader_input',{}),('duration_s',3600)])
def test_protocol_rejects_unimplemented_or_unbounded_configuration(name,value):
    config=json.loads((Path(__file__).resolve().parents[1]/'experiments/highway-circles-60mph-v1.json').read_text())
    validate_protocol(config)
    config[name]=value
    with pytest.raises(ValueError):validate_protocol(config)


def test_adjacent_lane_is_not_a_stationary_leader():
    route=lane_route('ego',500,0,1000)
    values,meta=circles_inputs([state(),state(503.7,.04),state(507.4,-.04)],route)
    assert values==pytest.approx([3,CEILING,1000])
    assert meta['leader_index'] is None
    values,meta=circles_inputs([state(),state(500,.04,2)],route)
    assert values[2]==pytest.approx(15.2)
    assert meta['leader_index']==1


def test_lane_only_actor_contract_rolls_reference_without_resetting_speed():
    applied=AppliedControl(.01,.2,0,'accepted',1,False)
    vector,meta=lane_only_observation(state(503.7,.04+65/503.7),applied,
                                     radius=503.7,start_angle=.04,tick=3000)
    assert len(vector)==42 and vector[22:]==[0.]*20
    assert vector[0]==pytest.approx(5) and vector[5]==pytest.approx(3)
    assert vector[14]==3 and vector[20]==0 and meta['window']==1


def test_full_footprint_and_high_speed_guards():
    route=lane_route('ego',500,0,1000)
    assert guard(state(speed=20),route,[],0,300)['reason'] is None
    assert guard(state(speed=27),route,[],0,300)['reason']=='overspeed'
    assert guard(state(498.9),route,[],0,300)['reason']=='lane_departure'
    assert guard(state(),route,[{}],0,300)['reason']=='rigid_contact'


def test_export_does_not_recompose_live_stage(tmp_path):
    pytest.importorskip('pxr.Usd')
    from pxr import Usd,UsdGeom
    from usd.highway_circles_scene import export_initial_scene
    layer=Usd.Stage.CreateNew(str(tmp_path/'asset.usda'))
    UsdGeom.Xform.Define(layer,'/World')
    UsdGeom.Cube.Define(layer,'/World/Car')
    layer.GetRootLayer().Save()
    stage=Usd.Stage.CreateInMemory()
    stage.GetRootLayer().subLayerPaths=[str(tmp_path/'asset.usda')]
    handle=stage.GetPrimAtPath('/World/Car')
    before=stage.GetRootLayer().ExportToString()
    saved=export_initial_scene(stage,tmp_path)
    assert handle and handle.GetTypeName()=='Cube'
    assert stage.GetRootLayer().ExportToString()==before
    reopened=Usd.Stage.Open(str(saved))
    assert reopened.GetPrimAtPath('/World/Car')
