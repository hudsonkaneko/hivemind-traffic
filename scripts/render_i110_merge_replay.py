"""Render a bounded frozen-policy SUMO merge recording; no physics or training.

Run with the configured Isaac Python. The input is recording.json plus the
exact network.net.xml; every vehicle pose comes from that recording. Road
surfaces and illustrative markings are reconstructed from SUMO lane geometry.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SIMREADY_MAIN = ROOT/'vehicles/sim_ready/americano-i7-ev/v14/assets/vehicle.usda'


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prepared_main_spec(path):
    """Resolve the prepared metre/Z-up/+X-forward chassis package contract.

    This is intentionally not an arbitrary USD normalizer. The model metadata
    describes the same ground-relative chassis frame used by the v02 replay.
    """
    entry = Path(path).resolve()
    if entry.is_dir():
        entry = entry/'assets/vehicle.usda'
    package = entry.parent.parent
    model_path = package/'model.json'
    if not entry.is_file() or not model_path.is_file():
        raise ValueError('SimReady main car requires a prepared assets/vehicle.usda and package model.json')
    model = json.loads(model_path.read_text(encoding='utf-8'))
    for name in ('body_collision_dimensions_m', 'body_collision_center_m', 'chassis_origin_in_v05_m'):
        values = model.get(name)
        if (not isinstance(values, list) or len(values) != 3 or
                any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values)):
            raise ValueError('Invalid prepared coordinate metadata: '+name)
    dimensions = model['body_collision_dimensions_m']
    if any(value <= 0 for value in dimensions):
        raise ValueError('Prepared body dimensions must be positive')
    front = model['body_collision_center_m'][0]+dimensions[0]/2.
    origin = model['chassis_origin_in_v05_m']
    centers = model.get('wheel_centers_m', {})
    if (front <= 0 or abs(origin[0]+front) > 1e-4 or abs(origin[1]) > 1e-6 or origin[2] <= 0 or
            any(not isinstance(centers.get(name), list) or len(centers[name]) != 3 or
                any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
                    for value in centers[name]) for name in ('FL', 'FR', 'RL', 'RR'))):
        raise ValueError('Unsupported prepared front-bumper/chassis coordinate contract')
    if not all(centers[name][0] > 0 for name in ('FL', 'FR')) or not all(centers[name][0] < 0 for name in ('RL', 'RR')):
        raise ValueError('Prepared car must use +X forward with named front/rear wheels')
    return dict(source_entry=str(entry), source_package=str(package), model_path=str(model_path),
        version=model.get('version', 'unspecified'), front_bumper_to_chassis_m=front,
        source_chassis_height_m=origin[2], body_dimensions_m=dimensions,
        texture_status=model.get('texture_status', 'unspecified'), license=model.get('license', 'unspecified'),
        local_only=True, scale_applied=[1., 1., 1.], forward_axis='+X', up_axis='Z', meters_per_unit=1.,
        pose_reference='SUMO front bumper on road; prepared chassis offset along local -X',
        sensor_capture=False, physics_control=False)


def author_prepared_main(output, source):
    """Reference immutable prepared assets and disable dynamics in a replay layer."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdUtils

    spec = prepared_main_spec(source)
    package, entry = Path(spec['source_package']), Path(spec['source_entry'])
    original = Usd.Stage.Open(str(entry))
    if (not original or original.GetCompositionErrors() or str(original.GetDefaultPrim().GetPath()) != '/Vehicle'
            or UsdGeom.GetStageUpAxis(original) != 'Z' or UsdGeom.GetStageMetersPerUnit(original) != 1.):
        raise ValueError('Prepared asset must compose a metre-scale Z-up /Vehicle entry')
    chassis = original.GetPrimAtPath('/Vehicle/Chassis')
    if not chassis:
        raise ValueError('Prepared asset is missing /Vehicle/Chassis')
    root_transform = UsdGeom.Xformable(original.GetDefaultPrim()).GetLocalTransformation()
    if root_transform != Gf.Matrix4d(1.):
        raise ValueError('Prepared /Vehicle root must not introduce an extra transform or scale')
    expected = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0., 0., spec['source_chassis_height_m']))
    actual = UsdGeom.Xformable(chassis).GetLocalTransformation()
    if any(abs(actual[i][j]-expected[i][j]) > 1e-6 for i in range(4) for j in range(4)):
        raise ValueError('Prepared chassis axes/scale differ from the declared replay frame')
    layers, assets, missing = UsdUtils.ComputeAllDependencies(Sdf.AssetPath(str(entry)))
    if missing:
        raise ValueError('Prepared asset has unresolved dependencies: '+repr(list(missing)))
    dependencies = {Path(layer.realPath).resolve() for layer in layers if layer.realPath}
    dependencies.update(Path(asset).resolve() for asset in assets)
    dependencies.add(Path(spec['model_path']))
    if any(not path.is_relative_to(package) for path in dependencies):
        raise ValueError('Prepared dependency escapes its declared package')
    hashes = {path.relative_to(package).as_posix(): sha256(path) for path in sorted(dependencies)}
    layer = Usd.Stage.CreateNew(str(output/'prepared-main.usda'))
    model = UsdGeom.Xform.Define(layer, '/Vehicle')
    layer.SetDefaultPrim(model.GetPrim())
    Usd.ModelAPI(model.GetPrim()).SetKind('component')
    UsdGeom.SetStageMetersPerUnit(layer, 1.)
    UsdGeom.SetStageUpAxis(layer, 'Z')
    layer.SetTimeCodesPerSecond(10.)
    relative = Path(os.path.relpath(entry, output)).as_posix()
    model.GetPrim().GetReferences().AddReference(relative, '/Vehicle')
    # Source Chassis already supplies its one-metre ground-relative height.
    # No SUMO length/width scaling or second vertical offset is applied.
    model.AddTranslateOp(opSuffix='frontBumper').Set(Gf.Vec3d(-spec['front_bumper_to_chassis_m'], 0., 0.))
    disabled = []
    for prim in layer.Traverse():
        attrs = []
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            attrs.append(UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False))
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            attrs.append(UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False))
        if prim.IsA(UsdPhysics.Joint):
            attrs.append(UsdPhysics.Joint(prim).CreateJointEnabledAttr(False))
        vehicle_enabled = prim.GetAttribute('physxVehicle:vehicleEnabled')
        if vehicle_enabled:
            vehicle_enabled.Set(False)
            attrs.append(vehicle_enabled)
        disabled.extend(str(attr.GetPath()) for attr in attrs)
    if any(prim.IsA(UsdPhysics.Scene) for prim in layer.Traverse()):
        raise ValueError('Prepared entry unexpectedly includes a physics scene')
    spec.update(reference=relative, source_sha256=hashes, disabled_physics_attributes=disabled,
                camera_prims=[str(prim.GetPath()) for prim in layer.Traverse() if prim.IsA(UsdGeom.Camera)],
                sensor_mount_prims=[str(prim.GetPath()) for prim in layer.Traverse()
                                    if prim.GetName().startswith('Lidar') and prim.IsA(UsdGeom.Xform)])
    layer.GetRootLayer().Save()
    if layer.GetCompositionErrors() or any(sha256(package/name) != digest for name, digest in hashes.items()):
        raise RuntimeError('Prepared replay composition or source immutability check failed')
    spec['source_unchanged'] = True
    (output/'prepared-main-manifest.json').write_text(json.dumps(spec, indent=2), encoding='utf-8')
    return spec


def vehicle_path(identity):
    return '/World/Vehicles/vehicle_' + identity.encode('utf-8').hex()


def read_lanes(path):
    lanes = {}
    for element in ET.parse(path).findall('.//lane'):
        points = [tuple(map(float, item.split(','))) for item in element.get('shape', '').split()]
        points = [point if len(point) == 3 else (*point, 0.) for point in points]
        if len(points) >= 2:
            lanes[element.get('id')] = dict(points=points, width=float(element.get('width', 3.2)))
    if not lanes:
        raise ValueError('Network contains no lane polylines')
    return lanes


def elevation(vehicle, lanes):
    if 'z' in vehicle:
        return float(vehicle['z'])
    lane = lanes.get(vehicle.get('lane_id', vehicle.get('lane')))
    candidates = [lane] if lane else lanes.values()
    nearest = (math.inf, 0.)
    for candidate in candidates:
        for a, b in zip(candidate['points'], candidate['points'][1:]):
            dx, dy = b[0]-a[0], b[1]-a[1]
            u = max(0., min(1., ((vehicle['x']-a[0])*dx+(vehicle['y']-a[1])*dy)/max(dx*dx+dy*dy, 1e-12)))
            distance = (vehicle['x']-a[0]-u*dx)**2+(vehicle['y']-a[1]-u*dy)**2
            nearest = min(nearest, (distance, a[2]+u*(b[2]-a[2])))
    return nearest[1]


def select_frames(data, start=None, end=None, maximum=300):
    start = data.get('clip_start_time', data['frames'][0]['time']) if start is None else start
    end = data.get('clip_end_time', data['frames'][-1]['time']) if end is None else end
    frames = [f for f in data['frames'] if start-1e-6 <= f['time'] <= end+1e-6]
    if not frames or len(frames) > maximum:
        raise ValueError(f'Expected 1..{maximum} frames, got {len(frames)}; select a shorter merge window')
    if any(b['time'] <= a['time'] for a, b in zip(frames, frames[1:])):
        raise ValueError('Replay timestamps must be strictly increasing')
    return frames


def screenshot_samples(frames, events):
    samples = [('before', 0)]
    inside = [event for event in events if frames[0]['time']-1e-6 <= event['time'] <= frames[-1]['time']+1e-6]
    for number, event in enumerate(inside):
        index = min(range(len(frames)), key=lambda i: abs(frames[i]['time']-event['time']))
        samples.append(('merge' if number == 0 else 'merge-'+str(number+1), index))
    if not inside:
        samples.append(('midpoint', len(frames)//2))
    return [*samples, ('after', len(frames)-1)]


def strip(a, b, width, offset=0., height=0.):
    dx, dy = b[0]-a[0], b[1]-a[1]
    length = math.hypot(dx, dy)
    if length < 1e-8:
        return None
    nx, ny = -dy/length, dx/length
    return [(p[0]+nx*d, p[1]+ny*d, p[2]+height)
            for p, d in ((a, offset+width/2), (a, offset-width/2),
                         (b, offset-width/2), (b, offset+width/2))]


def road_quads(lane, offset, width, dashed=False):
    result, station = [], 0.
    for a, b in zip(lane['points'], lane['points'][1:]):
        length = math.hypot(b[0]-a[0], b[1]-a[1])
        if not dashed:
            polygon = strip(a, b, width, offset, .025 if width < .3 else 0.)
            if polygon:
                result.append(polygon)
        elif length > 1e-8:
            s = station
            while s < station+length-1e-8:
                boundary = min(station+length, (math.floor(s/9.)+1)*9.)
                painted_end = min(boundary, math.floor(s/9.)*9.+3.)
                if painted_end > s+1e-8:
                    p = [a[k]+(b[k]-a[k])*(s-station)/length for k in range(3)]
                    q = [a[k]+(b[k]-a[k])*(painted_end-station)/length for k in range(3)]
                    result.append(strip(p, q, width, offset, .025))
                s = boundary
        station += length
    return result


def author_scene(output, frames, lanes, focal, controlled, highlighted=None, *, simready_main=None):
    from pxr import Usd, UsdGeom, UsdLux, Gf

    def stage(name, root='/World', kind='assembly'):
        result = Usd.Stage.CreateNew(str(output/name))
        result.SetDefaultPrim(UsdGeom.Xform.Define(result, root).GetPrim())
        Usd.ModelAPI(result.GetDefaultPrim()).SetKind(kind)
        UsdGeom.SetStageMetersPerUnit(result, 1.)
        UsdGeom.SetStageUpAxis(result, UsdGeom.Tokens.z)
        result.SetTimeCodesPerSecond(10.)
        result.SetFramesPerSecond(10.)
        return result

    def group(scene, path):
        Usd.ModelAPI(UsdGeom.Xform.Define(scene, path).GetPrim()).SetKind('group')

    def cube(scene, path, position, scale, color):
        item = UsdGeom.Cube.Define(scene, path)
        item.CreateSizeAttr(1.)
        item.AddTranslateOp().Set(Gf.Vec3d(*position))
        item.AddScaleOp().Set(Gf.Vec3f(*scale))
        item.CreateDisplayColorAttr([Gf.Vec3f(*color)])

    def mesh(scene, path, polygons, color):
        item = UsdGeom.Mesh.Define(scene, path)
        item.CreatePointsAttr([point for polygon in polygons for point in polygon])
        item.CreateFaceVertexCountsAttr([4]*len(polygons))
        item.CreateFaceVertexIndicesAttr(list(range(4*len(polygons))))
        item.CreateSubdivisionSchemeAttr('none')
        item.CreateDoubleSidedAttr(True)
        item.CreateDisplayColorAttr([Gf.Vec3f(*color)])

    if simready_main is not None:
        if not any(vehicle['id'] == focal for frame in frames for vehicle in frame['vehicles']):
            raise ValueError('Prepared focal vehicle absent from selected frames')
        author_prepared_main(output, simready_main)
    asset = stage('vehicle.usda', '/Vehicle', 'component')
    cube(asset, '/Vehicle/Body', (-2.3, 0., .62), (4.6, 1.8, .9), (.1, .64, .8))
    cube(asset, '/Vehicle/Cabin', (-2.5, 0., 1.23), (2.45, 1.52, .55), (.14, .19, .23))
    for name, x, y in [('FrontLeft', -.85, .9), ('FrontRight', -.85, -.9),
                       ('RearLeft', -3.55, .9), ('RearRight', -3.55, -.9)]:
        wheel = UsdGeom.Cylinder.Define(asset, '/Vehicle/'+name+'Wheel')
        wheel.CreateAxisAttr('Y'); wheel.CreateRadiusAttr(.33); wheel.CreateHeightAttr(.22)
        wheel.AddTranslateOp().Set(Gf.Vec3d(x, y, .33))
        wheel.CreateDisplayColorAttr([Gf.Vec3f(.025, .03, .035)])
    for side, y in [('Left', .6), ('Right', -.6)]:
        cube(asset, '/Vehicle/'+side+'Headlight', (-.005, y, .77), (.03, .35, .15), (.95, .95, .8))
        cube(asset, '/Vehicle/'+side+'Taillight', (-4.605, y, .77), (.03, .35, .15), (.7, .025, .015))
    asset.GetRootLayer().Save()

    road = stage('environment.usda')
    group(road, '/World/Environment'); group(road, '/World/Environment/Highway')
    points = [p for lane in lanes.values() for p in lane['points']]
    bounds = [(min(p[i] for p in points), max(p[i] for p in points)) for i in range(3)]
    cube(road, '/World/Environment/Ground', ((sum(bounds[0])/2), (sum(bounds[1])/2), bounds[2][0]-1.),
         (bounds[0][1]-bounds[0][0]+120., bounds[1][1]-bounds[1][0]+120., 1.), (.12, .17, .13))
    for identity, lane in lanes.items():
        path = '/World/Environment/Highway/Lane_'+identity.encode().hex()
        group(road, path)
        road.GetPrimAtPath(path).SetCustomDataByKey('sumo_lane_id', identity)
        mesh(road, path+'/RoadSurface', road_quads(lane, 0., lane['width']), (.16, .18, .2))
        if not identity.startswith(':'):
            mesh(road, path+'/LeftBoundary', road_quads(lane, lane['width']/2, .10, True), (.8, .82, .77))
            if identity.endswith('_0'):
                mesh(road, path+'/RightEdge', road_quads(lane, -lane['width']/2, .12), (.86, .87, .82))
    road.GetRootLayer().Save()

    motion = stage('motion.usda'); group(motion, '/World/Vehicles')
    identities = sorted({v['id'] for f in frames for v in f['vehicles']})
    for identity in identities:
        vehicle = next(v for f in frames for v in f['vehicles'] if v['id'] == identity)
        path = vehicle_path(identity)
        group(motion, path)
        root = UsdGeom.Xform(motion.GetPrimAtPath(path))
        root.GetPrim().SetCustomDataByKey('sumo_id', identity)
        root.GetPrim().SetCustomDataByKey('motion_authority', 'recorded_SUMO')
        position, heading, visibility = root.AddTranslateOp(), root.AddRotateZOp(), root.CreateVisibilityAttr()
        model = UsdGeom.Xform.Define(motion, path+'/Model')
        prepared = simready_main is not None and identity == focal
        model.GetPrim().GetReferences().AddReference('./prepared-main.usda' if prepared else './vehicle.usda')
        if prepared:
            root.GetPrim().SetCustomDataByKey('presentation_asset', 'prepared_main_unscaled')
        else:
            model.AddScaleOp().Set(Gf.Vec3f(vehicle.get('length', 4.6)/4.6, vehicle.get('width', 1.8)/1.8, 1.))
            color = (1., .36, .035) if identity in (highlighted or {focal}) else ((.02, .68, .86) if identity in controlled else (.46, .5, .53))
            UsdGeom.Gprim(motion.OverridePrim(path+'/Model/Body')).CreateDisplayColorAttr([Gf.Vec3f(*color)])
        for frame in frames:
            vehicle = next((v for v in frame['vehicles'] if v['id'] == identity), None)
            tc = round((frame['time']-frames[0]['time'])*10., 8)
            visibility.Set('inherited' if vehicle else 'invisible', tc)
            if vehicle:
                position.Set(Gf.Vec3d(vehicle['x'], vehicle['y'], elevation(vehicle, lanes)), tc)
                heading.Set(90.-vehicle['heading'], tc)
    motion.GetRootLayer().Save()

    render = stage('render.usda')
    UsdGeom.Scope.Define(render, '/World/Cameras')
    camera = UsdGeom.Camera.Define(render, '/World/Cameras/MergeFollow')
    camera.CreateFocalLengthAttr(24.); camera.CreateHorizontalApertureAttr(36.)
    camera.CreateClippingRangeAttr(Gf.Vec2f(.1, 3000.)); camera.AddTransformOp()
    UsdGeom.Scope.Define(render, '/World/Lighting')
    UsdLux.DomeLight.Define(render, '/World/Lighting/Sky').CreateIntensityAttr(800.)
    sun = UsdLux.DistantLight.Define(render, '/World/Lighting/Sun')
    sun.CreateIntensityAttr(1800.); sun.AddRotateXYZOp().Set(Gf.Vec3f(-35., -20., 35.))
    render.GetRootLayer().Save()
    world = stage('world.usda')
    world.GetRootLayer().subLayerPaths = ['./render.usda', './motion.usda', './environment.usda']
    world.SetStartTimeCode(0.); world.SetEndTimeCode((frames[-1]['time']-frames[0]['time'])*10.)
    world.GetRootLayer().Save()
    if world.GetCompositionErrors():
        raise RuntimeError(str(world.GetCompositionErrors()))
    return identities


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recording', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--start-time', type=float)
    parser.add_argument('--end-time', type=float)
    parser.add_argument('--focal-agent', default='agent_2')
    parser.add_argument('--simready-main', type=Path, nargs='?', const=DEFAULT_SIMREADY_MAIN,
                        help='Prepared asset entry or package for the focal car; bare option selects local Americano v14')
    parser.add_argument('--highlight-agents', nargs='+', default=['agent_2', 'agent_3'])
    parser.add_argument('--camera-mode', choices=['merge-fixed', 'follow'], default='merge-fixed')
    parser.add_argument('--merge-center', type=float, nargs=2, default=[-10., -280.], metavar=('X', 'Y'))
    parser.add_argument('--fps', type=int, default=10)
    parser.add_argument('--playback-speed', type=float, default=1.)
    parser.add_argument('--ffmpeg', type=Path, required=True)
    parser.add_argument('--author-only', action='store_true')
    args = parser.parse_args()
    recording, output = args.recording.resolve(), args.output.resolve()
    if not Path.cwd().resolve().is_relative_to(ROOT) or not output.is_relative_to(ROOT/'outputs'):
        parser.error('Run inside the checkout; new outputs belong under outputs/')
    if not 1 <= args.fps <= 30 or not .25 <= args.playback_speed <= 2.:
        parser.error('FPS must be 1..30 and playback speed .25..2')
    data = json.loads((recording/'recording.json').read_text(encoding='utf-8'))
    frames = select_frames(data, args.start_time, args.end_time)
    if not any(v['id'] == args.focal_agent for f in frames for v in f['vehicles']):
        raise ValueError('Focal vehicle absent from selected frames')
    if len(frames) > 1 and any(abs(b['time']-a['time']-1./args.fps) > 1e-5 for a, b in zip(frames, frames[1:])):
        raise ValueError('FPS must exactly match recorded frame cadence; no motion resampling is performed')
    if not args.author_only and not args.ffmpeg.is_file():
        raise FileNotFoundError('Provide the existing ffmpeg executable')
    output.mkdir(parents=True, exist_ok=False)
    (output/'source').mkdir()
    shutil.copy2(Path(__file__), output/'source'/Path(__file__).name)
    shutil.copy2(recording/'network.net.xml', output/'network.net.xml')
    lanes = read_lanes(output/'network.net.xml')
    controlled = set(data.get('controlled_ids', ['agent_0', 'agent_1', 'agent_2', 'agent_3']))
    report = dict(passed=False, training=False, motion_owner='SUMO recorded frozen-policy inference',
                  limitation='Kinematic visualization, not PhysX control validation or sensor capture. Road reconstructed from recorded SUMO network polylines; markings illustrative.',
                  source_recording=str(recording), source_recording_sha256=sha256(recording/'recording.json'),
                  renderer_source_sha256=sha256(__file__), runtime_python=sys.executable,
                  runtime_python_version=sys.version, runtime_python_sha256=sha256(sys.executable),
                  checkpoint_sha256=data.get('checkpoint_sha256'), network_sha256=sha256(output/'network.net.xml'),
                  focal_agent=args.focal_agent, highlighted_agents=args.highlight_agents, camera_mode=args.camera_mode,
                  merge_center=args.merge_center, frames=len(frames), start_time=frames[0]['time'],
                  end_time=frames[-1]['time'], fps=args.fps, playback_speed=args.playback_speed,
                  source_pose_interpolation=False, colors={'ramp_controlled':'orange', 'mainline_controlled':'cyan', 'native':'gray'})
    if args.simready_main is not None:
        report['prepared_main'] = prepared_main_spec(args.simready_main)
        report['colors']['prepared_focal'] = 'original prepared materials; no tint or rescaling'
    report['merge_events'] = data.get('merge_events', [])
    report['expected_video_duration_s'] = len(frames)/(args.fps*args.playback_speed)
    report['source_timestamp_span_s'] = frames[-1]['time']-frames[0]['time']
    (output/'render-request.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    app = None
    try:
        if not args.author_only:
            from isaacsim import SimulationApp
            app = SimulationApp({'headless': True, 'width': 1280, 'height': 720, 'limit_cpu_threads': 8,
                                 'extra_args': ['--/app/useFabricSceneDelegate=false',
                                                '--/persistent/app/viewport/displayOptions=0']})
            import carb
            carb.settings.get_settings().set('/persistent/app/viewport/displayOptions', 0)
            versions = {}
            for package in ('isaacsim', 'isaacsim-core', 'usd-core', 'numpy'):
                try:
                    versions[package] = importlib.metadata.version(package)
                except importlib.metadata.PackageNotFoundError:
                    pass
            report['runtime_package_versions'] = versions
            version_files = [parent/'VERSION' for parent in Path(sys.executable).parents if (parent/'VERSION').is_file()]
            report['runtime_version_files'] = {str(path):path.read_text(encoding='utf-8').strip() for path in version_files}
        identities = author_scene(output, frames, lanes, args.focal_agent, controlled, set(args.highlight_agents),
                                  simready_main=args.simready_main)
        names = ['world.usda', 'environment.usda', 'motion.usda', 'vehicle.usda', 'render.usda']
        if args.simready_main is not None:
            names += ['prepared-main.usda', 'prepared-main-manifest.json']
            report['prepared_main'] = json.loads((output/'prepared-main-manifest.json').read_text(encoding='utf-8'))
        hashes = {name: sha256(output/name) for name in names}
        report.update(static_layer_sha256=hashes, vehicle_count=len(identities), source_prim_map={v:vehicle_path(v) for v in identities})
        if args.author_only:
            report.update(passed=True, author_only=True)
            return 0
        import omni.usd
        from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
        from pxr import UsdGeom, Gf
        omni.usd.get_context().open_stage(str(output/'world.usda'))
        for _ in range(30): app.update()
        stage = omni.usd.get_context().get_stage()
        composition_errors = [str(error) for error in stage.GetCompositionErrors()]
        if composition_errors:
            raise RuntimeError('Composed scene errors: '+str(composition_errors))
        report['composition_errors'] = composition_errors
        report['composed_root_children'] = [str(prim.GetPath()) for prim in stage.GetDefaultPrim().GetChildren()]
        report['composed_sublayers'] = list(stage.GetRootLayer().subLayerPaths)
        stage.SetEditTarget(stage.GetSessionLayer())
        viewport = get_active_viewport()
        viewport.camera_path = '/World/Cameras/MergeFollow'
        viewport.resolution = (1280, 720)
        camera_op = UsdGeom.Xformable(stage.GetPrimAtPath(viewport.camera_path)).GetOrderedXformOps()[0]
        frame_dir = output/'frames'; frame_dir.mkdir()
        focal_position = None
        first_focal = next(v for f in frames for v in f['vehicles'] if v['id'] == args.focal_agent)
        camera_position = (first_focal['x'], first_focal['y'], elevation(first_focal, lanes))
        if args.camera_mode == 'merge-fixed':
            center_vehicle = dict(x=args.merge_center[0], y=args.merge_center[1])
            camera_position = (*args.merge_center, elevation(center_vehicle, lanes))
        capture_handles, timings, maximum_pose_error, camera_poses = [], [], 0., []
        for index, frame in enumerate(frames):
            started = time.perf_counter()
            present = {v['id']: v for v in frame['vehicles']}
            for identity in identities:
                prim = stage.GetPrimAtPath(vehicle_path(identity))
                UsdGeom.Imageable(prim).GetVisibilityAttr().Set('inherited' if identity in present else 'invisible')
                if identity in present:
                    vehicle = present[identity]
                    expected = (vehicle['x'], vehicle['y'], elevation(vehicle, lanes))
                    for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
                        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate: op.Set(Gf.Vec3d(*expected))
                        elif op.GetOpType() == UsdGeom.XformOp.TypeRotateZ: op.Set(90.-vehicle['heading'])
                    actual = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0.).ExtractTranslation()
                    maximum_pose_error = max(maximum_pose_error, max(abs(actual[i]-expected[i]) for i in range(3)))
            if args.camera_mode == 'follow' and args.focal_agent in present:
                focal = present[args.focal_agent]
                focal_position = (focal['x'], focal['y'], elevation(focal, lanes))
                camera_position = tuple(.65*a+.35*b for a, b in zip(camera_position, focal_position))
            target = Gf.Vec3d(camera_position[0], camera_position[1]-8., camera_position[2])
            eye = target+Gf.Vec3d(62., 46., 78.)
            matrix = Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0., 0., 1.)).GetInverse()
            camera_op.Set(matrix)
            camera_poses.append(dict(time=frame['time'], eye=list(eye), target=list(target)))
            for _ in range(3 if index else 20): app.update()
            target_file = frame_dir/f'{index:05d}.png'
            capture_handles.append(capture_viewport_to_file(viewport, str(target_file)))
            deadline = time.perf_counter()+15.
            while not target_file.exists() and time.perf_counter() < deadline:
                app.update()
            if not target_file.is_file():
                raise RuntimeError(f'Viewport capture timed out: {target_file.name}')
            timings.append(time.perf_counter()-started)
        for _ in range(5): app.update()
        changed = [name for name, digest in hashes.items() if sha256(output/name) != digest]
        if args.simready_main is not None:
            spec = report['prepared_main']
            source_changes = [name for name, digest in spec['source_sha256'].items()
                              if sha256(Path(spec['source_package'])/name) != digest]
            report['prepared_main']['source_unchanged'] = not source_changes
            changed.extend('prepared source: '+name for name in source_changes)
        if changed or maximum_pose_error > 1e-6:
            raise RuntimeError(f'Replay verification failed: changed={changed}, pose_error={maximum_pose_error}')
        video = output/'i110-merge.mp4'
        encode = [str(args.ffmpeg.resolve()), '-hide_banner', '-n', '-framerate', str(args.fps*args.playback_speed),
                  '-i', str(frame_dir/'%05d.png'), '-c:v', 'libx264', '-preset', 'fast', '-crf', '18',
                  '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(video)]
        encoded = subprocess.run(encode, capture_output=True, text=True, check=True, timeout=180)
        (output/'encoding.log').write_text(encoded.stderr, encoding='utf-8')
        decoded = subprocess.run([str(args.ffmpeg.resolve()), '-v', 'error', '-i', str(video), '-progress', 'pipe:1', '-f', 'null', '-'],
                                 capture_output=True, text=True, check=True, timeout=180)
        counts = [int(line.split('=')[1]) for line in decoded.stdout.splitlines() if line.startswith('frame=')]
        if not counts or counts[-1] != len(frames):
            raise RuntimeError('Decoded frame count does not match the recorded sequence')
        screenshots = []
        for label, index in screenshot_samples(frames, data.get('merge_events', [])):
            destination = output/f'{label}.png'; shutil.copy2(frame_dir/f'{index:05d}.png', destination)
            screenshots.append(dict(file=destination.name, source_frame=index, simulation_time=frames[index]['time']))
        (output/'camera-poses.json').write_text(json.dumps(camera_poses, indent=2), encoding='utf-8')
        report.update(passed=True, static_layers_unchanged=True, max_position_error_m=maximum_pose_error,
                      capture_wall_s=sum(timings), capture_mean_s=sum(timings)/len(timings),
                      runtime='Isaac Sim SimulationApp', runtime_namespace_exceptions=['/Render', '/OmniverseKit_Persp'],
                      video=video.name, video_sha256=sha256(video), ffmpeg_sha256=sha256(args.ffmpeg),
                      video_decode_verified=True, decoded_frames=counts[-1], screenshots=screenshots,
                      viewport_display_options=0)
        return 0
    except Exception as error:
        report['error'] = repr(error)
        raise
    finally:
        # Isaac fastShutdown may terminate Python inside close(). Persist evidence
        # and finish CPU encoding first; nothing required may follow this call.
        spec = report.get('prepared_main', {})
        if 'source_sha256' in spec:
            spec['source_unchanged'] = all((Path(spec['source_package'])/name).is_file() and
                sha256(Path(spec['source_package'])/name) == digest for name, digest in spec['source_sha256'].items())
            if not spec['source_unchanged']:
                report['passed'] = False
        (output/'render-report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        if app is not None: app.close()


if __name__ == '__main__':
    raise SystemExit(main())
