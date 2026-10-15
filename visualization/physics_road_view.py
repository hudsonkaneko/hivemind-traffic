"""Decorative road and driver-reference view; never writes vehicle physics state.

Static road artwork and live debug overlays occupy separate USD layers. The
optional clean layout references a reusable, file-backed road asset; its debug
layer remains anonymous during live updates. All geometry uses the existing
analytic lane map. No collision, mass,
rigid-body, sensor or actuator API is authored here. Debug prims use default
render purpose so they are visible without enabling every physics guide mesh.
They are NOT sensor-excluded and can enter raw RTX LiDAR scans. Their surfaces
stay at or below 0.10 m, for the runner's separately validated ground-height
filter; they are not viewport-only graphics.

The pure geometry/camera functions import neither Isaac nor USD. USD imports
are deferred until ``author_physics_road`` is called inside a running viewer.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real
from pathlib import Path


LEGACY_ROAD_ROOT = '/World/PhysicsRoad'
CLEAN_ROAD_ROOT = '/World/Environment/Highway'
CLEAN_DEBUG_ROOT = '/World/Debug/Route'


@dataclass(frozen=True, slots=True)
class CameraPose:
    eye: tuple[float, float, float]
    target: tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class MeshData:
    points: tuple[tuple[float, float, float], ...]
    face_counts: tuple[int, ...]
    face_indices: tuple[int, ...]


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f'{name} must be finite')
    return float(value)


def _position(value, count, name):
    if not isinstance(value, (tuple, list)) or len(value) != count:
        raise ValueError(f'{name} must contain {count} coordinates')
    return tuple(_number(v, name) for v in value)


def route_ribbon(route, start_s_m, end_s_m, *, width_m, lateral_offset_m=0.0,
                 z_m=0.025, max_spacing_m=0.5):
    """Quad ribbon sampled from analytic route tangents, including exact joins."""
    begin, end = _number(start_s_m, 'start_s_m'), _number(end_s_m, 'end_s_m')
    width, offset = _number(width_m, 'width_m'), _number(lateral_offset_m, 'lateral_offset_m')
    z, spacing = _number(z_m, 'z_m'), _number(max_spacing_m, 'max_spacing_m')
    if end <= begin or width <= 0 or spacing <= 0:
        raise ValueError('Ribbon requires increasing stations, positive width and spacing')
    maximum_lateral = abs(offset) + width / 2
    curvature = max(abs(segment.curvature_rad_m) for segment in route.segments)
    if curvature * maximum_lateral >= 1:
        raise ValueError('Ribbon must not reach a curve centre')
    # Bound outer-edge spacing as well as centreline spacing.
    count = math.ceil((end - begin) * (1 + curvature * maximum_lateral) / spacing)
    if count > 100_000:
        raise ValueError('Requested decorative ribbon is too dense')
    stations = {begin + (end - begin) * i / count for i in range(count + 1)}
    station = 0.0
    for segment in route.segments:
        if begin < station < end:
            stations.add(station)
        station += segment.length_m
    if begin < route.length_m < end:
        stations.add(route.length_m)
    points = []
    for station in sorted(stations):
        point = route.evaluate(station)
        nx, ny = -math.sin(point.yaw_rad), math.cos(point.yaw_rad)
        for lateral in (offset + width / 2, offset - width / 2):
            points.append((point.position_xy[0] + lateral * nx,
                           point.position_xy[1] + lateral * ny, z))
    faces = len(points) // 2 - 1
    indices = tuple(index for i in range(faces) for index in (2*i, 2*i+1, 2*i+3, 2*i+2))
    return MeshData(tuple(points), (4,) * faces, indices)


def target_ring(target_xy, *, radius_m=0.38, width_m=0.09, z_m=0.085, segments=32):
    """Ground-level pursuit-target ring, never a vertical obstacle-shaped marker."""
    x, y = _position(target_xy, 2, 'target_xy')
    radius, width, z = _number(radius_m, 'radius_m'), _number(width_m, 'width_m'), _number(z_m, 'z_m')
    if radius <= 0 or not 0 < width < 2 * radius or not 0 <= z <= 0.1:
        raise ValueError('Ring needs positive radius/width and ground-level z in [0, .1]')
    if isinstance(segments, bool) or not isinstance(segments, int) or not 8 <= segments <= 256:
        raise ValueError('Ring segments must be in [8, 256]')
    points = []
    for index in range(segments):
        angle = math.tau * index / segments
        for r in (radius + width / 2, radius - width / 2):
            points.append((x + r * math.cos(angle), y + r * math.sin(angle), z))
    indices = tuple(v for i in range(segments) for v in (2*i, 2*((i+1) % segments),
                                                       2*((i+1) % segments)+1, 2*i+1))
    return MeshData(tuple(points), (4,) * segments, indices)


def build_road_geometry(route, *, endpoint_padding_m=5.0):
    """Return the three static road meshes; the collision plane stays untouched."""
    padding = _number(endpoint_padding_m, 'endpoint_padding_m')
    if padding < 0:
        raise ValueError('Endpoint padding must be nonnegative')
    edge_width = min(0.12, route.width_m / 8)
    offset = route.width_m / 2 - edge_width / 2
    bounds = (-padding, route.length_m + padding)
    return {
        'Asphalt': route_ribbon(route, *bounds, width_m=route.width_m, z_m=0.025),
        'LeftEdge': route_ribbon(route, *bounds, width_m=edge_width, lateral_offset_m=offset, z_m=0.035),
        'RightEdge': route_ribbon(route, *bounds, width_m=edge_width, lateral_offset_m=-offset, z_m=0.035),
    }


def lane_divider_geometry(route, lateral_offset_m):
    """Three-metre white dashes with three-metre gaps on the static road."""
    offset = _number(lateral_offset_m, 'lane_divider_m')
    if abs(offset) + 0.06 >= route.width_m / 2:
        raise ValueError('Lane divider must lie inside the road edges')
    points, counts, indices = [], [], []
    for start in range(0, math.ceil(route.length_m), 6):
        dash = route_ribbon(route, start, min(start + 3.0, route.length_m),
                            width_m=0.12, lateral_offset_m=offset, z_m=0.04)
        indices.extend(index + len(points) for index in dash.face_indices)
        points.extend(dash.points)
        counts.extend(dash.face_counts)
    return MeshData(tuple(points), tuple(counts), tuple(indices))


def camera_pose_overview(route):
    """Oblique camera framing the entire padded route in a typical 16:9 view."""
    points = build_road_geometry(route)['Asphalt'].points
    minimum = tuple(min(p[axis] for p in points) for axis in (0, 1))
    maximum = tuple(max(p[axis] for p in points) for axis in (0, 1))
    center = ((minimum[0] + maximum[0]) / 2, (minimum[1] + maximum[1]) / 2, 0.0)
    span = max(maximum[0] - minimum[0], maximum[1] - minimum[1], 20.0)
    # Position relative to the route's initial forward axis for rotated fixtures.
    yaw = route.initial_yaw_rad
    behind, right = -0.20 * span, -0.65 * span
    return CameraPose((center[0] + behind * math.cos(yaw) - right * math.sin(yaw),
                       center[1] + behind * math.sin(yaw) + right * math.cos(yaw),
                       max(25.0, 1.10 * span)), center)


def camera_pose_follow(position_m, yaw_rad):
    """Chase view: 12 m behind, 9 m right, 8 m above; target 10 m ahead."""
    x, y, z = _position(position_m, 3, 'position_m')
    yaw = _number(yaw_rad, 'yaw_rad')
    c, s = math.cos(yaw), math.sin(yaw)
    return CameraPose((x - 12*c + 9*s, y - 12*s - 9*c, z + 8),
                      (x + 10*c, y + 10*s, z))


def _author_mesh(stage, path, geometry, color, material_path, *, purpose='default', emissive=False):
    from pxr import Gf, Sdf, UsdGeom, UsdShade
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    mesh.CreatePurposeAttr(purpose)
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, material_path + '/Shader')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(0.9)
    if emissive:
        shader.CreateInput('emissiveColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    _update_mesh(mesh, geometry)
    return mesh


def _update_mesh(mesh, geometry):
    from pxr import Gf, Vt
    # Short Python tuples can be inferred as Gf vectors rather than USD arrays.
    # Explicit array types also keep shrinking previews valid near route ends.
    mesh.CreatePointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*p) for p in geometry.points]))
    mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray(geometry.face_counts))
    mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray(geometry.face_indices))
    minimum = [min(p[i] for p in geometry.points) for i in range(3)]
    maximum = [max(p[i] for p in geometry.points) for i in range(3)]
    mesh.CreateExtentAttr().Set([Gf.Vec3f(*minimum), Gf.Vec3f(*maximum)])


class PhysicsRoadView:
    """Updates view-only debug opinions; has no vehicle/control/physics handle."""

    def __init__(self, stage, route, root_path, static_layer, dynamic_layer, *,
                 debug_root_path=None, asset_path=None, assembly_path=None):
        self.stage, self.route, self.root_path = stage, route, root_path
        self.debug_root_path = debug_root_path or root_path + '/Debug'
        self.asset_path = asset_path
        self.assembly_path = assembly_path
        self.clean_layout = asset_path is not None
        # Routes are immutable. Cache only this view's road framing, not vehicle
        # poses or LiDAR/control state; overview previously rebuilt road meshes.
        self._overview_route = route
        self._overview_camera = camera_pose_overview(route)
        self._displayed_full_route = None
        self._target_translation = None
        self.static_layer, self.dynamic_layer = static_layer, dynamic_layer
        self.metadata = dict(route_id=route.route_id, route_length_m=route.length_m,
                             lane_width_m=route.width_m, movement_authority='unchanged_isaac_physx',
                             collision_authored=False, debug_purpose='default',
                             debug_sensor_exclusion_verified=False, debug_z_max_m=0.085,
                             debug_sensor_visibility='Renderable geometry; may enter raw RTX LiDAR scans',
                             debug_roi_handling='Ground-level surfaces; braking-height exclusion must be checked by the runner',
                             endpoint_padding_m=5.0, clean_layout=self.clean_layout,
                             road_root_path=root_path, debug_root_path=self.debug_root_path,
                             static_layer=static_layer.identifier, dynamic_layer=dynamic_layer.identifier)
        if self.clean_layout:
            self.metadata.update(road_asset_path=str(asset_path),
                                 road_assembly_path=str(assembly_path),
                                 road_asset_default_prim='/Highway',
                                 road_asset_reference=asset_path.relative_to(assembly_path.parent).as_posix())

    def update(self, state, target_xy=None, show_reference=True, planned_route=None,
               preview_distance_m=25.0, full_plan=False):
        """Consume a plain observed state and optional planner target; never move it."""
        from pxr import Gf, Usd, UsdGeom
        position = _position(state['position_m'], 3, 'position_m')
        route = self.route if planned_route is None else planned_route
        distance = _number(preview_distance_m, 'preview_distance_m')
        if distance <= 0:
            raise ValueError('Preview distance must be positive')
        station = 0. if full_plan else route.project(*position[:2]).s_m
        begin = max(0.0, min(route.length_m, station))
        end = route.length_m if full_plan else min(route.length_m, begin + distance)
        debug_root = self.debug_root_path
        with Usd.EditContext(self.stage, self.dynamic_layer):
            root = UsdGeom.Imageable(self.stage.GetPrimAtPath(debug_root))
            root.CreateVisibilityAttr().Set('inherited' if show_reference else 'invisible')
            path_mesh = UsdGeom.Mesh(self.stage.GetPrimAtPath(debug_root + '/UpcomingPath'))
            path_mesh.CreateVisibilityAttr().Set('inherited' if end > begin else 'invisible')
            if end > begin and (not full_plan or self._displayed_full_route != route):
                _update_mesh(path_mesh, route_ribbon(route, begin, end, width_m=0.16, z_m=0.07))
            self._displayed_full_route = route if full_plan else None
            target_mesh = UsdGeom.Mesh(self.stage.GetPrimAtPath(debug_root + '/PursuitTarget'))
            target_mesh.CreateVisibilityAttr().Set('inherited' if target_xy is not None else 'invisible')
            if target_xy is not None:
                if full_plan:
                    if self._target_translation is None:
                        _update_mesh(target_mesh, target_ring((0.,0.)))
                        self._target_translation = target_mesh.AddTranslateOp()
                    xy = _position(target_xy, 2, 'target_xy')
                    self._target_translation.Set(Gf.Vec3d(*xy,0.))
                else:
                    if self._target_translation is not None:
                        self._target_translation.Set(Gf.Vec3d(0.,0.,0.))
                        self._target_translation = None
                        target_mesh.ClearXformOpOrder()
                    _update_mesh(target_mesh, target_ring(target_xy))

    def camera_pose(self, mode, state=None):
        if mode == 'overview':
            if self.route != self._overview_route:
                self._overview_route = self.route
                self._overview_camera = camera_pose_overview(self.route)
            return self._overview_camera
        if mode == 'follow' and state is not None:
            return camera_pose_follow(state['position_m'], state['yaw_rad'])
        raise ValueError('Camera mode must be overview, or follow with an observed state')


def _author_clean_road(stage, route, dividers, show_reference_centerline, asset_directory):
    """Publish an immutable decorative road asset and compose its placement."""
    from pxr import Kind, Sdf, Usd, UsdGeom
    asset_path = asset_directory / 'road.usda'
    assembly_path = asset_directory.parent / 'road-layout.usda'
    for output in (asset_path, assembly_path):
        if output.exists():
            raise FileExistsError(f'Road export already exists; choose a new scene directory: {output}')
    if stage.GetPrimAtPath(CLEAN_DEBUG_ROOT):
        raise ValueError('Debug route root already exists; do not overwrite existing scene opinions')

    # Author at the asset boundary first so every material target is internal
    # and USD can remap it when the asset is placed at another namespace path.
    asset = Usd.Stage.CreateInMemory('road-asset.usda')
    UsdGeom.SetStageUpAxis(asset, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(asset, 1.0)
    asset.SetTimeCodesPerSecond(stage.GetTimeCodesPerSecond())
    root = UsdGeom.Xform.Define(asset, '/Highway').GetPrim()
    asset.SetDefaultPrim(root)
    Usd.ModelAPI(root).SetKind(Kind.Tokens.component)
    root.SetAssetInfoByKey('name', 'Highway')
    root.SetAssetInfoByKey('identifier', Sdf.AssetPath('road.usda'))
    root.SetAssetInfoByKey('version', '1')
    root.SetCustomDataByKey('route_id', route.route_id)
    root.SetCustomDataByKey('decorative_only', True)
    UsdGeom.Scope.Define(asset, '/Highway/Geometry')
    UsdGeom.Scope.Define(asset, '/Highway/Looks')
    geometry = build_road_geometry(route)
    for source_name, name, color in [('Asphalt', 'RoadSurface', (0.035, 0.047, 0.060)),
                                     ('LeftEdge', 'LeftEdge', (0.92, 0.92, 0.85)),
                                     ('RightEdge', 'RightEdge', (0.92, 0.92, 0.85))]:
        _author_mesh(asset, '/Highway/Geometry/' + name, geometry[source_name], color,
                     '/Highway/Looks/' + name)
    for index, divider in enumerate(dividers):
        _author_mesh(asset, f'/Highway/Geometry/LaneDivider_{index + 1:03d}', divider,
                     (0.92, 0.92, 0.92), '/Highway/Looks/LaneDivider')

    old_sublayers = list(stage.GetRootLayer().subLayerPaths)
    created_outputs = []
    try:
        # A relative reference must belong to a file-backed layer; an anonymous
        # layer would resolve it against a process-dependent working directory.
        if not asset.GetRootLayer().Export(str(asset_path)):
            raise RuntimeError(f'Could not export road asset: {asset_path}')
        created_outputs.append(asset_path)
        static = Sdf.Layer.CreateNew(str(assembly_path))
        if static is None:
            raise RuntimeError(f'Could not create road assembly layer: {assembly_path}')
        created_outputs.append(assembly_path)
        assembly = Usd.Stage.Open(static)
        UsdGeom.SetStageUpAxis(assembly, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(assembly, 1.0)
        assembly.SetTimeCodesPerSecond(stage.GetTimeCodesPerSecond())
        world = UsdGeom.Xform.Define(assembly, '/World').GetPrim()
        assembly.SetDefaultPrim(world)
        Usd.ModelAPI(world).SetKind(Kind.Tokens.assembly)
        environment = UsdGeom.Xform.Define(assembly, '/World/Environment').GetPrim()
        Usd.ModelAPI(environment).SetKind(Kind.Tokens.group)
        highway = UsdGeom.Xform.Define(assembly, CLEAN_ROAD_ROOT).GetPrim()
        highway.GetReferences().AddReference(asset_path.relative_to(assembly_path.parent).as_posix())
        if assembly.GetCompositionErrors():
            raise RuntimeError(f'Road asset composition failed: {assembly.GetCompositionErrors()}')
        if not static.Save():
            raise RuntimeError(f'Could not save road assembly layer: {assembly_path}')

        dynamic = Sdf.Layer.CreateAnonymous('physics-road-debug.usda')
        stage.GetRootLayer().subLayerPaths = [dynamic.identifier, static.identifier] + old_sublayers
        with Usd.EditContext(stage, dynamic):
            UsdGeom.Scope.Define(stage, '/World/Debug')
            debug = UsdGeom.Xform.Define(stage, CLEAN_DEBUG_ROOT)
            debug.CreatePurposeAttr(UsdGeom.Tokens.default_)
            UsdGeom.Scope.Define(stage, CLEAN_DEBUG_ROOT + '/Looks')
            reference = _author_mesh(stage, CLEAN_DEBUG_ROOT + '/ReferenceCenterline',
                         route_ribbon(route, 0, route.length_m, width_m=0.06, z_m=0.05),
                         (0.0, 0.36, 0.45), CLEAN_DEBUG_ROOT + '/Looks/Reference', emissive=True)
            reference.CreateVisibilityAttr().Set('inherited' if show_reference_centerline else 'invisible')
            _author_mesh(stage, CLEAN_DEBUG_ROOT + '/UpcomingPath',
                         route_ribbon(route, 0, min(25.0, route.length_m), width_m=0.16, z_m=0.07),
                         (0.0, 0.95, 1.0), CLEAN_DEBUG_ROOT + '/Looks/Upcoming', emissive=True)
            target = _author_mesh(stage, CLEAN_DEBUG_ROOT + '/PursuitTarget',
                                 target_ring(route.evaluate(0).position_xy),
                                 (1.0, 0.12, 0.65), CLEAN_DEBUG_ROOT + '/Looks/Target', emissive=True)
            target.CreateVisibilityAttr().Set('invisible')
        return PhysicsRoadView(stage, route, CLEAN_ROAD_ROOT, static, dynamic,
                               debug_root_path=CLEAN_DEBUG_ROOT, asset_path=asset_path,
                               assembly_path=assembly_path)
    except Exception:
        stage.GetRootLayer().subLayerPaths = old_sublayers
        # Only new files belonging to this unsuccessful export are disposable.
        for output in reversed(created_outputs):
            output.unlink(missing_ok=True)
        raise


def author_physics_road(stage, route, root_path=LEGACY_ROAD_ROOT,
                        lane_dividers_m=(), show_reference_centerline=True, *,
                        clean_layout=False, asset_directory=None):
    """Attach independent artwork/debug layers, preserving the edit target.

    Does not save existing files, change the stage unit/up-axis contract, add
    lights or create collision geometry. Legacy mode retains anonymous layers
    and the original namespace. ``clean_layout=True`` requires an existing,
    fresh ``asset_directory`` and publishes ``road.usda`` plus its sibling
    ``../road-layout.usda``. It places the road at ``/World/Environment/Highway``
    and all debug content at ``/World/Debug/Route``. Existing prims or output
    files are rejected. The caller owns final root/dynamic layer packaging.
    """
    if clean_layout:
        if root_path not in (LEGACY_ROAD_ROOT, CLEAN_ROAD_ROOT):
            raise ValueError('clean_layout uses /World/Environment/Highway as its road root')
        if asset_directory is None:
            raise ValueError('clean_layout requires an existing asset_directory')
        asset_directory = Path(asset_directory).resolve()
        if not asset_directory.is_dir():
            raise ValueError('asset_directory must be an existing directory')
        root_path = CLEAN_ROAD_ROOT
    elif asset_directory is not None:
        raise ValueError('asset_directory is only supported with clean_layout=True')
    from pxr import Sdf, Usd, UsdGeom
    path = Sdf.Path(root_path)
    if not path.IsAbsolutePath() or not path.IsPrimPath() or path == Sdf.Path.absoluteRootPath:
        raise ValueError('root_path must be an absolute USD prim path')
    if stage.GetPrimAtPath(root_path):
        raise ValueError('View root already exists; do not overwrite existing scene opinions')
    if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z or not math.isclose(UsdGeom.GetStageMetersPerUnit(stage), 1.0):
        raise ValueError('The physical-road view requires an existing Z-up, metre stage')
    dividers = [lane_divider_geometry(route, offset) for offset in lane_dividers_m]
    if clean_layout:
        return _author_clean_road(stage, route, dividers, show_reference_centerline, asset_directory)
    static = Sdf.Layer.CreateAnonymous('physics-road-static.usda')
    dynamic = Sdf.Layer.CreateAnonymous('physics-road-debug.usda')
    old_sublayers = list(stage.GetRootLayer().subLayerPaths)
    stage.GetRootLayer().subLayerPaths = [dynamic.identifier, static.identifier] + old_sublayers
    try:
        with Usd.EditContext(stage, static):
            root = UsdGeom.Xform.Define(stage, root_path)
            root.GetPrim().SetCustomDataByKey('route_id', route.route_id)
            root.GetPrim().SetCustomDataByKey('decorative_only', True)
            geometry = build_road_geometry(route)
            for name, color in [('Asphalt', (0.035, 0.047, 0.060)),
                                ('LeftEdge', (0.92, 0.92, 0.85)), ('RightEdge', (0.92, 0.92, 0.85))]:
                _author_mesh(stage, root_path + '/Static/' + name, geometry[name], color,
                             root_path + '/Materials/' + name)
            for index, divider in enumerate(dividers):
                _author_mesh(stage, root_path + f'/Static/LaneDivider{index}', divider,
                             (0.92, 0.92, 0.92), root_path + '/Materials/LaneDivider')
            debug = UsdGeom.Xform.Define(stage, root_path + '/Debug')
            debug.CreatePurposeAttr(UsdGeom.Tokens.default_)
            reference = _author_mesh(stage, root_path + '/Debug/ReferenceCenterline',
                         route_ribbon(route, 0, route.length_m, width_m=0.06, z_m=0.05),
                         (0.0, 0.36, 0.45), root_path + '/Materials/Reference', emissive=True)
            reference.CreateVisibilityAttr().Set('inherited' if show_reference_centerline else 'invisible')
        with Usd.EditContext(stage, dynamic):
            _author_mesh(stage, root_path + '/Debug/UpcomingPath',
                         route_ribbon(route, 0, min(25.0, route.length_m), width_m=0.16, z_m=0.07),
                         (0.0, 0.95, 1.0), root_path + '/Materials/Upcoming', emissive=True)
            target = _author_mesh(stage, root_path + '/Debug/PursuitTarget',
                                 target_ring(route.evaluate(0).position_xy),
                                 (1.0, 0.12, 0.65), root_path + '/Materials/Target', emissive=True)
            target.CreateVisibilityAttr().Set('invisible')
        return PhysicsRoadView(stage, route, root_path, static, dynamic)
    except Exception:
        # New layers are disposable; pre-existing root opinions remain intact.
        stage.GetRootLayer().subLayerPaths = old_sublayers
        raise
