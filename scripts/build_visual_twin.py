"""Compose a private supplied visual asset with a saved SUMO replay (no simulation)."""
import argparse
import hashlib
import json
import math
import os
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def new_stage(path, root='/World', kind='assembly'):
    stage = Usd.Stage.CreateNew(str(path))
    prim = UsdGeom.Xform.Define(stage, root).GetPrim()
    stage.SetDefaultPrim(prim)
    Usd.ModelAPI(prim).SetKind(kind)
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, 'Z')
    stage.SetTimeCodesPerSecond(60)
    stage.SetFramesPerSecond(60)
    return stage


def material(stage, path, color, metallic=0, roughness=.5):
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path+'/Surface')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(metallic)
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(roughness)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    return mat


def prepare_car(source, prim_path, destination, scale=.01, yaw=180):
    """Reference unchanged collection; normalize one car and override incomplete looks."""
    destination.mkdir(parents=True, exist_ok=False)
    source_dir = destination.parent/'source'
    source_dir.mkdir(exist_ok=True)
    copied = source_dir/'vehicles.usd'
    if copied.exists() and sha(copied) != sha(source):
        raise ValueError('Existing private source has different bytes; choose a new asset name')
    if not copied.exists():
        shutil.copyfile(source, copied)
    stage = new_stage(destination/'vehicle.usda', '/Vehicle', 'component')
    visual = UsdGeom.Xform.Define(stage, '/Vehicle/Visuals')
    visual.GetPrim().GetReferences().AddReference('../source/vehicles.usd', prim_path)
    visual.GetPrim().SetInstanceable(False)
    if not visual.GetPrim().GetChildren():
        raise ValueError(f'No geometry at source prim {prim_path}')
    visual.SetXformOpOrder([])
    matrix = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw))
    matrix = Gf.Matrix4d().SetScale(scale) * matrix
    op = visual.AddTransformOp(opSuffix='normalization')
    op.Set(matrix)
    bbox = UsdGeom.BBoxCache(0, ['default', 'render']).ComputeWorldBound(visual.GetPrim()).ComputeAlignedRange()
    lo, hi = bbox.GetMin(), bbox.GetMax()
    matrix.SetTranslateOnly(Gf.Vec3d(-hi[0], -(hi[1]+lo[1])/2, -lo[2]))
    op.Set(matrix)
    # This selected subtree has local Looks. Deactivation preserves the source and
    # prevents incomplete MDL texture opinions from becoming runtime dependencies.
    stage.OverridePrim('/Vehicle/Visuals/Looks').SetActive(False)
    paint = material(stage, '/Vehicle/Looks/Paint', (.055, .32, .63), .6, .25)
    glass = material(stage, '/Vehicle/Looks/Glass', (.025, .055, .08), .5, .12)
    rubber = material(stage, '/Vehicle/Looks/Rubber', (.025, .03, .035), 0, .8)
    count = 0
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            name = prim.GetName().lower()
            mat = glass if 'glass' in name else rubber if 'wheel' in name else paint
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
            count += 1
    stage.GetRootLayer().Save()
    result = dict(source_sha256=sha(source), source_prim=prim_path,
                  scale_interpretation=scale, yaw_degrees=yaw, meshes=count,
                  dimensions_m=list(hi-lo), source_units_authored=False,
                  anchor='front bumper +X; bottom at Z=0',
                  materials='fallback paint, dark glass and rubber; source textures absent',
                  physics=False, redistribution='unqualified; local only')
    (destination/'provenance.json').write_text(json.dumps(result, indent=2))
    return result


def vehicle_path(identifier):
    return '/World/Vehicles/v_'+identifier.encode('utf-8').hex()


def build_scene(recording_path, network_path, vehicle, output):
    output.mkdir(parents=True, exist_ok=False)
    data = json.loads(recording_path.read_text())
    frames = data['frames']
    if not frames or any(b['time'] <= a['time'] for a, b in zip(frames, frames[1:])):
        raise ValueError('Recording must have nonempty, strictly increasing times')
    shutil.copyfile(recording_path, output/'recording.json')
    shutil.copyfile(network_path, output/'network.net.xml')
    asset_path = Path(os.path.relpath(vehicle, output)).as_posix()
    static = new_stage(output/'environment.usda')
    for path, kind in [('/World/Environment','group'),('/World/Environment/Highway','component')]:
        Usd.ModelAPI(UsdGeom.Xform.Define(static, path).GetPrim()).SetKind(kind)
    anchors = []
    for index, lane in enumerate(ET.parse(network_path).findall('.//lane')):
        points = [tuple(map(float, p.split(',')))[:2] for p in lane.attrib['shape'].split()]
        width = float(lane.attrib.get('width', 3.2))
        for segment, (a, b) in enumerate(zip(points, points[1:])):
            dx, dy = b[0]-a[0], b[1]-a[1]
            length = math.hypot(dx, dy)
            if length < .001:
                continue
            anchors.extend([a,b])
            nx, ny = -dy/length*width/2, dx/length*width/2
            mesh = UsdGeom.Mesh.Define(static, f'/World/Environment/Highway/Lane_{index}/Surface_{segment}')
            mesh.CreatePointsAttr([(a[0]+nx,a[1]+ny,0),(a[0]-nx,a[1]-ny,0),(b[0]-nx,b[1]-ny,0),(b[0]+nx,b[1]+ny,0)])
            mesh.CreateFaceVertexCountsAttr([4]); mesh.CreateFaceVertexIndicesAttr([0,1,2,3])
            mesh.CreateSubdivisionSchemeAttr('none'); mesh.CreateDoubleSidedAttr(True)
            mesh.CreateDisplayColorAttr([Gf.Vec3f(.11,.13,.16)])
            # Narrow edge stripes make the geometry and direction readable.
            for side in (-1,1):
                stripe = UsdGeom.Mesh.Define(static, f'/World/Environment/Highway/Lane_{index}/Edge_{segment}_{"L" if side==1 else "R"}')
                sx, sy = nx*side, ny*side
                ix, iy = sx*.96, sy*.96
                stripe.CreatePointsAttr([(a[0]+sx,a[1]+sy,.012),(a[0]+ix,a[1]+iy,.012),(b[0]+ix,b[1]+iy,.012),(b[0]+sx,b[1]+sy,.012)])
                stripe.CreateFaceVertexCountsAttr([4]); stripe.CreateFaceVertexIndicesAttr([0,1,2,3])
                stripe.CreateSubdivisionSchemeAttr('none'); stripe.CreateDoubleSidedAttr(True)
                stripe.CreateDisplayColorAttr([Gf.Vec3f(.8,.82,.76)])
    if not anchors:
        raise ValueError('Network has no nonzero lane segments')
    xmin,ymin=min(p[0] for p in anchors),min(p[1] for p in anchors)
    xmax,ymax=max(p[0] for p in anchors),max(p[1] for p in anchors)
    ground = UsdGeom.Cube.Define(static, '/World/Environment/Ground')
    ground.CreateSizeAttr(1); ground.AddTranslateOp().Set(((xmin+xmax)/2,(ymin+ymax)/2,-.16))
    ground.AddScaleOp().Set((xmax-xmin+200,ymax-ymin+200,.2)); ground.CreateDisplayColorAttr([(.14,.20,.16)])
    UsdGeom.Scope.Define(static, '/World/Lighting')
    UsdLux.DomeLight.Define(static, '/World/Lighting/Sky').CreateIntensityAttr(700)
    sun = UsdLux.DistantLight.Define(static, '/World/Lighting/Sun')
    sun.CreateIntensityAttr(2500); sun.AddRotateXYZOp().Set((-35,-25,20))
    UsdGeom.Scope.Define(static, '/World/Cameras')
    for name in ('Overview','Follow'):
        camera = UsdGeom.Camera.Define(static, '/World/Cameras/'+name)
        camera.CreateFocalLengthAttr(22 if name=='Overview' else 35)
        camera.CreateHorizontalApertureAttr(36)
        camera.CreateVerticalApertureAttr(22.5)
        camera.CreateClippingRangeAttr((.1,5000))
    static.GetRootLayer().Save()
    motion = new_stage(output/'motion.usda')
    Usd.ModelAPI(UsdGeom.Xform.Define(motion, '/World/Vehicles').GetPrim()).SetKind('group')
    ids = sorted({v['id'] for f in frames for v in f['vehicles']})
    ops = {}
    for identifier in ids:
        xf = UsdGeom.Xform.Define(motion, vehicle_path(identifier))
        Usd.ModelAPI(xf.GetPrim()).SetKind('component')
        xf.GetPrim().SetCustomDataByKey('sumo_id',identifier)
        xf.GetPrim().SetCustomDataByKey('motion_authority','recorded SUMO; visual only')
        ops[identifier] = (xf.AddTranslateOp(),xf.AddRotateZOp(),xf.CreateVisibilityAttr())
        model = UsdGeom.Xform.Define(motion, vehicle_path(identifier)+'/Model')
        model.GetPrim().GetReferences().AddReference(asset_path)
        # Model-kind metadata belongs to the placement boundary above.
        model.GetPrim().ClearMetadata('kind')
        model.GetPrim().SetMetadata('kind','subcomponent')
        model.GetPrim().SetInstanceable(True)
    previous_yaw = {}
    for frame in frames:
        tc = (frame['time']-frames[0]['time'])*60
        present = {v['id']:v for v in frame['vehicles']}
        for identifier, (trans,rot,vis) in ops.items():
            v = present.get(identifier)
            vis.Set('inherited' if v else 'invisible',tc)
            if v:
                trans.Set((v['x'],v['y'],.02),tc)
                yaw = 90-v['heading']
                if identifier in previous_yaw:
                    yaw = previous_yaw[identifier] + (yaw-previous_yaw[identifier]+180)%360-180
                previous_yaw[identifier] = yaw
                rot.Set(yaw,tc)
    motion.GetRootLayer().Save()
    world = new_stage(output/'world.usda')
    world.GetRootLayer().subLayerPaths = ['./motion.usda','./environment.usda']
    world.SetStartTimeCode(0); world.SetEndTimeCode((frames[-1]['time']-frames[0]['time'])*60)
    world.SetInterpolationType(Usd.InterpolationTypeLinear)
    world.GetRootLayer().Save()
    if world.GetCompositionErrors():
        raise RuntimeError(str(world.GetCompositionErrors()))
    manifest = dict(recording_sha256=sha(recording_path), network_sha256=sha(network_path),
                    ids=ids, source_time_origin=frames[0]['time'], duration_s=world.GetEndTimeCode()/60,
                    units='metres',up_axis='Z',yaw='90 - SUMO heading',anchor='front bumper',
                    anchors=anchors[:2],geometry='synthetic SUMO lane shapes; not Esri',
                    road_bounds_m=[xmin,ymin,xmax,ymax],
                    limitation='SUMO visual replay, no physics, no learned policy; fallback materials; visual dimensions differ from SUMO envelopes')
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2))
    return manifest


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--prim',default='/Root/vehicles/sm_AmericanoI7EV_a1_05')
    parser.add_argument('--recording',type=Path,required=True)
    parser.add_argument('--network',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--asset',type=Path,default=ROOT/'vehicles/sim_ready/sam-collected/v01')
    args=parser.parse_args()
    for target in (args.output,args.asset):
        if not target.resolve().is_relative_to(ROOT):
            parser.error('Prepared assets and outputs must stay in this checkout')
    asset=prepare_car(args.source,args.prim,args.asset)
    scene=build_scene(args.recording,args.network,args.asset/'vehicle.usda',args.output)
    print(json.dumps(dict(asset=asset,scene=scene),indent=2))


if __name__=='__main__':
    main()
