"""Reopen physical-adapter scenes in plain OpenUSD from another working directory."""
import argparse
import hashlib
import json
from pathlib import Path

from pxr import Usd, UsdGeom, UsdUtils

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    args = parser.parse_args()
    run = args.run.resolve()
    if not run.is_relative_to(ROOT/'outputs') or not Path.cwd().resolve().is_relative_to(ROOT):
        raise ValueError('Use the canonical checkout and an existing output run')
    scenes = sorted(run.glob('*/scene/world.usda'))
    if not scenes:
        raise ValueError('No composed scenes found')
    result = dict(passed=True, usd_version=Usd.GetVersion(), working_directory=str(Path.cwd()), scenes=[])
    for path in scenes:
        before = {str(p.relative_to(path.parent)): hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in path.parent.rglob('*.usda')}
        stage = Usd.Stage.Open(str(path))
        errors = [str(e) for e in stage.GetCompositionErrors()]
        if stage.GetDefaultPrim().GetPath().pathString != '/World':
            errors.append('defaultPrim is not World')
        if UsdGeom.GetStageMetersPerUnit(stage) != 1 or UsdGeom.GetStageUpAxis(stage) != 'Z':
            errors.append('Invalid units/up axis')
        required = ['/World/Environment/Highway', '/World/Environment/Ground',
                    '/World/Vehicles/vehicle_ego/Chassis', '/World/Physics/Scene', '/World/Cameras/Follow']
        required += ['/World/Vehicles/vehicle_ego/Chassis/'+wheel for wheel in
                     ('FrontLeftWheel', 'FrontRightWheel', 'RearLeftWheel', 'RearRightWheel')]
        for target in required:
            prim = stage.GetPrimAtPath(target)
            if not prim or not prim.IsLoaded() or not prim.IsActive():
                errors.append('Missing/unloaded '+target)
        layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(path))
        errors.extend('Unresolved '+str(p) for p in unresolved)
        prims = []
        for prim in stage.Traverse():
            for relationship in prim.GetRelationships():
                for target in relationship.GetTargets():
                    if target.IsAbsolutePath() and not stage.GetPrimAtPath(target.GetPrimPath()):
                        errors.append('Unresolved relationship '+str(relationship.GetPath())+' -> '+str(target))
            refs = prim.GetMetadata('references')
            if refs:
                for ref in refs.GetAddedOrExplicitItems():
                    if ref.assetPath and Path(ref.assetPath).is_absolute():
                        errors.append('Nonportable reference '+ref.assetPath)
            prims.append(dict(path=str(prim.GetPath()), type=prim.GetTypeName(),
                              kind=Usd.ModelAPI(prim).GetKind(), reference=prim.HasAuthoredReferences()))
        after = {str(p.relative_to(path.parent)): hashlib.sha256(p.read_bytes()).hexdigest()
                 for p in path.parent.rglob('*.usda')}
        if before != after:
            errors.append('Opening the scene changed an authored file')
        record = dict(scene=str(path.relative_to(run)), passed=not errors, errors=errors,
                      prims=prims, layers=[Path(layer.identifier).name for layer in layers],
                      asset_count=len(assets), source_sha256=before)
        result['scenes'].append(record)
        result['passed'] &= record['passed']
    target = run/'reopen-validation.json'
    if target.exists():
        raise ValueError('Preserve previous reopen evidence; target already exists')
    target.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(dict(passed=result['passed'], scenes=len(scenes), output=str(target),
                         errors=[r['errors'] for r in result['scenes']]), indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
