"""Read-only artifact checks for a prepared LA scene, independent of its builder."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics
from usd.highway_event_layer import EventLayerController,route_pose,right_lane_offset


def check(scene,geometry):
    stage=Usd.Stage.Open(str(scene));data=json.loads(geometry.read_text())
    assert stage and not stage.GetCompositionErrors()
    assert stage.GetDefaultPrim().GetPath()=='/World'
    assert UsdGeom.GetStageMetersPerUnit(stage)==1 and UsdGeom.GetStageUpAxis(stage)=='Z'
    assert stage.GetTimeCodesPerSecond()==60
    summary={}
    for name in ('Southbound','MlkEntrance','VernonExit'):
        source=data['meshes'][name];v=np.array(source['points']);faces=np.array(source['faces'])
        prim=stage.GetPrimAtPath('/World/Physics/'+name+'Support')
        actual=np.array(UsdGeom.Mesh(prim).GetPointsAttr().Get())
        error=float(np.max(abs(actual-v)));assert error<.0001
        assert UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
        tri=v[faces];assert (np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])[:,2]>0).all()
        center=np.array(source['center_xyz']);s=np.array(source['station_m'])
        mask=np.array(source['interpolated']);spans=[]
        for j in range(mask.shape[1]):
            known=~mask[:,j]
            for i in np.flatnonzero(mask[:,j]):
                left=np.flatnonzero(known & (s<s[i]))[-1];right=np.flatnonzero(known & (s>s[i]))[0]
                spans.append(float(s[right]-s[left]))
        assert max(spans,default=0)<=60
        summary[name]=dict(length_m=float(s[-1]),source_to_collider_max_abs_error_m=error,
                           interpolated_vertices=int(mask.sum()),total_vertices=len(v),
                           max_bracketed_interpolation_m=max(spans,default=0),
                           max_center_grade=float(np.max(abs(np.diff(center[:,2])/np.diff(s)))))
    files=list(scene.parent.glob('*.usda'))
    before={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    stage.SetEditTarget(stage.GetSessionLayer());route=data['meshes']['Southbound']
    manifest=json.loads((scene.parent/'manifest.json').read_text())
    control=EventLayerController(stage,route,manifest['probe_path'])
    checks={}
    for mode in control.presets:
        for t in (0,10,20,50,64.9,65,80):
            states=control.update(t,mode)
            assert bool(states)==(mode!='Off' and 10<=t<65)
        checks[mode]=True
    control.update(30,'Construction');snapshot=stage.GetSessionLayer().ExportToString()
    control.update(70,'Police traffic break');control.update(30,'Construction')
    assert stage.GetSessionLayer().ExportToString()==snapshot
    control.update(30,'Off')
    for paths in control.actor_paths.values():
        for path in paths:
            prim=stage.GetPrimAtPath(path)
            assert UsdGeom.Imageable(prim).GetVisibilityAttr().Get()=='invisible'
            for child in Usd.PrimRange(prim):
                if child.HasAPI(UsdPhysics.CollisionAPI):assert not UsdPhysics.CollisionAPI(child).GetCollisionEnabledAttr().Get()
    after={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in files};assert before==after
    # Reopen the source stage: event preview changes must never persist there.
    fresh=Usd.Stage.Open(str(scene))
    assert fresh.GetSessionLayer().empty
    return dict(composition_errors=0,metres_z_up_60fps=True,surfaces=summary,
                event_modes=checks,seek_repeat_exact=True,event_off_collisions_disabled=True,
                source_layers_unchanged=True,source_layer_hashes=before,
                limitation='Representation/source-fit checks, not independent survey or native physics driving')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('scene','geometry','report'):p.add_argument('--'+key,type=Path,required=True)
    a=p.parse_args();r=check(a.scene,a.geometry);a.report.write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
