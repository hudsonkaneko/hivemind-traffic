"""Check composed replay state and active materials against the source recording."""
import argparse
import json
import math
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

from build_visual_twin import vehicle_path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene',type=Path)
    parser.add_argument('--report',type=Path,required=True)
    args=parser.parse_args()
    if args.report.exists():
        parser.error('Choose a fresh report path')
    stage=Usd.Stage.Open(str(args.scene.resolve()))
    frames=json.loads((args.scene.parent/'recording.json').read_text())['frames']
    ids=sorted({v['id'] for f in frames for v in f['vehicles']})
    checks=0
    max_error=0.
    for frame in frames:
        tc=(frame['time']-frames[0]['time'])*60
        present={v['id']:v for v in frame['vehicles']}
        for identifier in ids:
            prim=stage.GetPrimAtPath(vehicle_path(identifier))
            v=present.get(identifier)
            assert (UsdGeom.Imageable(prim).ComputeVisibility(tc)!='invisible')==bool(v)
            if v:
                transform=UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(tc)
                error=(transform.ExtractTranslation()-Gf.Vec3d(v['x'],v['y'],.02)).GetLength()
                max_error=max(max_error,error)
                assert error<1e-6
                yaw=math.radians(90-v['heading'])
                assert (transform.TransformDir(Gf.Vec3d(1,0,0))-Gf.Vec3d(math.cos(yaw),math.sin(yaw),0)).GetLength()<1e-6
            checks+=1
    unresolved=[]
    meshes=0
    for prim in Usd.PrimRange.Stage(stage,Usd.TraverseInstanceProxies()):
        for attr in prim.GetAttributes():
            if attr.GetTypeName()==Sdf.ValueTypeNames.Asset:
                value=attr.Get()
                if value and value.path and not value.resolvedPath:
                    unresolved.append(str(attr.GetPath()))
        if prim.IsA(UsdGeom.Mesh) and '/Vehicles/' in str(prim.GetPath()):
            bound,_=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
            assert bound
            meshes+=1
    assert not stage.GetCompositionErrors()
    assert not unresolved
    assert stage.GetDefaultPrim().GetPath()=='/World'
    assert UsdGeom.GetStageMetersPerUnit(stage)==1
    assert UsdGeom.GetStageUpAxis(stage)=='Z'
    result=dict(passed=True,lifecycle_pose_heading_checks=checks,max_position_error_m=max_error,
                vehicle_mesh_instances=meshes,active_unresolved_assets=unresolved,
                limitation='Active composition check; retained original library may contain inactive missing textures. Not rendering or physical validation.')
    args.report.write_text(json.dumps(result,indent=2))
    print(json.dumps(result))


if __name__=='__main__':
    main()
