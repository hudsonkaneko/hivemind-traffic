import math
import pytest
from traffic.highway_events import HighwayEvent,active_restrictions,development_events,preview_trajectory,restrictive_speed_cap


def test_event_time_boundaries_and_overlap():
    a=HighwayEvent('a','construction',10,20,50,80,('r',))
    b=HighwayEvent('b','disabled_vehicle',15,25,70,70,('r',))
    assert not active_restrictions([a,b],9.9)
    assert len(active_restrictions([a,b],15))==2
    assert [s['event_id'] for s in active_restrictions([a,b],20)]==['b']
    assert not active_restrictions([a,b],25)
    assert not active_restrictions([a,b],17,enabled=False)
    with pytest.raises(ValueError):active_restrictions([a,a],17)


def test_police_break_repeat_seek_and_bounds():
    events=development_events()['Police traffic break'];e=events[0]
    samples=[e.state(t/10) for t in range(801)]
    assert samples==[e.state(t/10) for t in range(801)]
    assert all(180<=s['station_m']<=530 and abs(s['lateral_m'])<=4 for s in samples)
    assert e.state(10)['lateral_speed_m_s']>0
    assert e.state(12)['lateral_speed_m_s']==pytest.approx(0,abs=1e-8)
    assert e.state(20)==e.state(20)
    for bad in (math.nan,math.inf,-1):
        with pytest.raises(ValueError):e.state(bad)


def test_local_restrictions_do_not_stop_downstream_or_other_lanes():
    states=active_restrictions(development_events()['Construction'],30)
    assert restrictive_speed_cap(states,'sb_right',465,8)==0
    assert restrictive_speed_cap(states,'sb_right',540,8)==8
    assert restrictive_speed_cap(states,'sb_left',465,8)==8


def test_inspection_probe_stops_and_releases_without_pems():
    modes=development_events();clear=preview_trajectory(modes['Off'])
    stopped=preview_trajectory(modes['Construction'])
    assert max(p['station_m'] for p in stopped if p['time']<65)<=462.01
    assert stopped[-1]['station_m']>stopped[650]['station_m']
    assert clear[-1]['station_m']>stopped[-1]['station_m']
    assert stopped==preview_trajectory(modes['Construction'])


def test_invalid_schedules_rejected():
    with pytest.raises(ValueError):HighwayEvent('bad','construction',20,10,0,1,('r',))
    with pytest.raises(ValueError):HighwayEvent('bad','construction',0,10,0,1,('r',),speed_m_s=1)


def test_layer_toggles_collision_and_preserves_saved_assets(tmp_path):
    pytest.importorskip('pxr')
    from pxr import Usd,UsdGeom,UsdPhysics
    from scripts.build_visual_twin import new_stage
    from usd.highway_event_layer import EventLayerController
    s=new_stage(tmp_path/'world.usda')
    paths=[f'/World/Environment/Props/Events/Cone_{i:02}' for i in range(13)]+['/World/Vehicles/vehicle_disabled_demo','/World/Vehicles/vehicle_police_demo']
    for path in paths:
        x=UsdGeom.Xform.Define(s,path);x.AddTranslateOp();x.AddRotateXYZOp()
        cube=UsdGeom.Cube.Define(s,path+'/Collider');UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(False)
    s.GetRootLayer().Save();before=(tmp_path/'world.usda').read_bytes()
    route=dict(station_m=[0,900],center_xyz=[[0,0,0],[0,-900,0]],widths_m=[18,18],
               points=[[x,y,0] for y in (0,-900) for x in (-9,-6,-3,0,3,6,9)])
    s.SetEditTarget(s.GetSessionLayer());ctl=EventLayerController(s,route)
    ctl.update(20,'Construction')
    cone=s.GetPrimAtPath(paths[0]);assert UsdGeom.Imageable(cone).GetVisibilityAttr().Get()=='inherited'
    assert UsdPhysics.CollisionAPI(s.GetPrimAtPath(paths[0]+'/Collider')).GetCollisionEnabledAttr().Get()
    ctl.update(20,'Off')
    assert UsdGeom.Imageable(cone).GetVisibilityAttr().Get()=='invisible'
    assert not UsdPhysics.CollisionAPI(s.GetPrimAtPath(paths[0]+'/Collider')).GetCollisionEnabledAttr().Get()
    assert (tmp_path/'world.usda').read_bytes()==before
    s.SetEditTarget(s.GetRootLayer())
    with pytest.raises(RuntimeError):ctl.update(20)
