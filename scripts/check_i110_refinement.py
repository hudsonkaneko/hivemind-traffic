"""Verify the refined stage, actual support triangles and optional event isolation."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics
from usd.highway_event_layer import EventLayerController


def check(scene,geometry):
    stage=Usd.Stage.Open(str(scene/'world.usda'))
    assert stage and not stage.GetCompositionErrors()
    assert UsdGeom.GetStageUpAxis(stage)=='Z' and UsdGeom.GetStageMetersPerUnit(stage)==1
    assert stage.GetTimeCodesPerSecond()==60
    record=json.loads(geometry.read_text());support=record['collision_mesh']
    path='/World/Physics/RoadSupport/Surface';prim=stage.GetPrimAtPath(path)
    assert UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
    v=np.array(UsdGeom.Mesh(prim).GetPointsAttr().Get());expected=np.array(support['points'])
    assert np.max(abs(v-expected))<.0001
    f=np.array(UsdGeom.Mesh(prim).GetFaceVertexIndicesAttr().Get()).reshape(-1,3)
    assert np.array_equal(f,np.array(support['faces']))
    tri=v[f];normals=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])
    assert np.all(normals[:,2]>0)
    assert len(np.unique(v[:,:2],axis=0))==len(v)
    assert abs(prim.GetAttribute('physxCollision:contactOffset').Get()-.02)<1e-8
    road_colliders=[str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)
                    and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()]
    assert road_colliders==[path]
    visual=np.array(UsdGeom.Mesh(stage.GetPrimAtPath('/World/Environment/Highway/RoadSurface')).GetPointsAttr().Get())
    assert np.array_equal(visual,v)
    hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in scene.glob('*.usda')}
    manifest=json.loads((scene/'manifest.json').read_text())
    route=json.loads((scene/'event-route.json').read_text())
    stage.SetEditTarget(stage.GetSessionLayer());events=EventLayerController(stage,route,manifest['probe_path'])
    for mode in events.presets:
        for t in (0,10,30,50,65,80):events.update(t,mode)
    events.update(0,'Off')
    after=[str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)
           and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()]
    assert after==road_colliders
    assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    return dict(passed=True,composition_errors=0,vertices=len(v),triangles=len(f),
                distinct_road_colliders=road_colliders,positive_triangle_normals=True,
                shared_vertex_identity=True,visual_and_collision_vertices_identical=True,
                representation_error_m=float(np.max(abs(v-expected))),
                maximum_triangle_slope=float(np.max(np.linalg.norm(normals[:,:2],axis=1)/normals[:,2])),
                event_modes=len(events.presets),static_layers_unchanged=True,
                warning='Representation precision and geometric checks do not establish survey accuracy')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',type=Path,required=True);p.add_argument('--geometry',type=Path,required=True)
    a=p.parse_args();print(json.dumps(check(a.scene,a.geometry),indent=2))
