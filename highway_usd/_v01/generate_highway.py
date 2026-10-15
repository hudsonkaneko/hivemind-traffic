"""Deterministically author the static four-leaf highway draft; see README.md."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import shapely
from shapely.geometry import LineString, Polygon
from shapely.geometry.polygon import orient
from shapely.ops import unary_union
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

TAU = math.tau
SIDES = {"East": 0, "North": 90, "West": 180, "South": 270}
ROOT = "/World"
ROAD = ROOT + "/RoadNetwork"
NAV = ROOT + "/Navigation"


@dataclass(frozen=True)
class Parameters:
    inner_radius_m: float = 180.0
    lane_width_m: float = 3.7
    main_lanes: int = 4
    leaf_extension_m: float = 180.0
    leaf_half_angle_deg: float = 32.0
    circle_samples: int = 1440
    ramp_samples: int = 1024
    road_top_m: float = 0.0
    road_thickness_m: float = 0.3
    ground_margin_m: float = 35.0
    main_speed_hint_m_s: float = 15.0
    auxiliary_speed_hint_m_s: float = 10.0
    ramp_speed_hint_m_s: float = 8.0

    @property
    def auxiliary_radius(self):
        return self.inner_radius_m + (self.main_lanes + 0.5) * self.lane_width_m

    @property
    def outer_radius(self):
        return self.inner_radius_m + (self.main_lanes + 1) * self.lane_width_m

    def validate(self):
        assert self.main_lanes == 4, "This version's contract is four main lanes."
        assert self.inner_radius_m > 20 * self.lane_width_m > 0
        assert 0 < self.leaf_half_angle_deg < 40
        assert self.leaf_extension_m > 10 * self.lane_width_m
        assert self.circle_samples >= 720 and self.circle_samples % 360 == 0
        assert self.ramp_samples >= 512
        assert self.road_thickness_m > 0
        assert self.road_top_m == 0, "V1 waypoint and ground datum is Z=0."


def circle(radius, count, z=0.0):
    return [(radius * math.cos(TAU*i/count), radius * math.sin(TAU*i/count), z)
            for i in range(count)]


def disk(radius, p):
    return Polygon([(x, y) for x, y, _ in circle(radius, p.circle_samples)])


def petal(degrees, p):
    """Polar sin^4 lobe. Position, tangent and curvature match the circle at ends.

    Strictly increasing polar angle also prevents self-crossings. The ramp and
    the corresponding auxiliary arc form the return loop; it is not a flyover.
    """
    points = []
    for i in range(p.ramp_samples + 1):
        t = i / p.ramp_samples
        angle = math.radians(degrees - p.leaf_half_angle_deg + 2*p.leaf_half_angle_deg*t)
        radius = p.auxiliary_radius + p.leaf_extension_m * math.sin(math.pi*t)**4
        points.append((radius*math.cos(angle), radius*math.sin(angle), p.road_top_m))
    return points


def polygons(geometry):
    if geometry.is_empty:
        return []
    return [geometry] if geometry.geom_type == "Polygon" else list(geometry.geoms)


def triangles(geometry):
    result = []
    for poly in polygons(geometry):
        for tri in shapely.constrained_delaunay_triangles(poly).geoms:
            coords = list(orient(tri, sign=1).exterior.coords)[:3]
            result.append(coords)
    area = sum(Polygon(t).area for t in result)
    assert abs(area - geometry.area) < max(1e-6, geometry.area * 1e-10)
    return result


def attr(prim, name, type_name, value):
    prim.CreateAttribute(name, type_name, custom=True).Set(value)


def relation(prim, name, targets):
    prim.CreateRelationship(name, custom=True).SetTargets([Sdf.Path(t) for t in targets])


def material(stage, name, color, physics=False):
    mat = UsdShade.Material.Define(stage, ROOT + "/Looks/" + name)
    shader = UsdShade.Shader.Define(stage, str(mat.GetPath()) + "/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.92)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    if physics:
        api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        api.CreateStaticFrictionAttr(0.9)
        api.CreateDynamicFrictionAttr(0.8)
        api.CreateRestitutionAttr(0.0)
    return mat


def mesh(stage, path, shape, z, mat=None, solid_depth=0.0, collision=False):
    """Constrained triangulation preserves holes; welded solid for collision."""
    # USD Mesh points are float32. Triangulate those exact coordinates so tiny
    # slivers in boolean/paint geometry cannot collapse during USD serialization.
    shape = shapely.set_precision(shape, .001)
    shape = shapely.transform(shape, lambda xy: xy.astype("float32").astype("float64"))
    assert shape.is_valid, f"Invalid snapped footprint: {path}"
    points, faces, lookup = [], [], {}

    def vertex(x, y, height):
        key = (round(x, 9), round(y, 9), round(height, 9))
        if key not in lookup:
            lookup[key] = len(points)
            points.append(key)
        return lookup[key]

    def face(coords, height, reverse=False):
        ids = [vertex(x, y, height) for x, y in coords]
        faces.append(ids[::-1] if reverse else ids)

    for triangle in triangles(shape):
        face(triangle, z)
        if solid_depth:
            face(triangle, z-solid_depth, reverse=True)
    if solid_depth:
        for poly in polygons(shape):
            poly = orient(poly, sign=1)
            for boundary in [poly.exterior, *poly.interiors]:
                coords = list(boundary.coords)
                for (x1, y1), (x2, y2) in zip(coords, coords[1:]):
                    faces.append([vertex(x1, y1, z), vertex(x1, y1, z-solid_depth),
                                  vertex(x2, y2, z-solid_depth), vertex(x2, y2, z)])
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray(points))
    m.CreateFaceVertexCountsAttr([len(f) for f in faces])
    m.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    m.CreateSubdivisionSchemeAttr("none")
    m.CreateOrientationAttr("rightHanded")
    m.CreateDoubleSidedAttr(False)
    m.CreateExtentAttr(UsdGeom.PointBased.ComputeExtent(m.GetPointsAttr().Get()))
    if mat:
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
    if collision:
        api = UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        api.CreateCollisionEnabledAttr(True)
        api.CreateSimulationOwnerRel().SetTargets([ROOT + "/Physics/Scene"])
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("none")
        m.CreatePurposeAttr("guide")
    return m


def curve(stage, path, points, closed, color=(0.1, 0.7, 0.9)):
    c = UsdGeom.BasisCurves.Define(stage, path)
    c.CreateTypeAttr("linear")
    c.CreateWrapAttr("periodic" if closed else "nonperiodic")
    c.CreateCurveVertexCountsAttr([len(points)])
    c.CreatePointsAttr(Vt.Vec3fArray(points))
    c.CreateWidthsAttr([0.12])
    c.SetWidthsInterpolation("constant")
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    c.CreatePurposeAttr("guide")
    c.CreateExtentAttr(UsdGeom.PointBased.ComputeExtent(c.GetPointsAttr().Get()))
    return c.GetPrim()


def path_length(points, closed=False):
    pairs = list(zip(points, points[1:]))
    if closed:
        pairs.append((points[-1], points[0]))
    return sum(math.dist(a, b) for a, b in pairs)


def build_network(p):
    lanes, edges = {}, {}
    for i in range(5):
        name = f"MainLane{i+1}" if i < 4 else "AuxiliaryLane"
        pts = circle(p.inner_radius_m + (i+.5)*p.lane_width_m, p.circle_samples)
        lanes[name] = dict(id=name, kind="main" if i < 4 else "auxiliary", index=i+1,
                           usd_path=f"{NAV}/Lanes/{name}", width_m=p.lane_width_m,
                           closed=True, points=pts,
                           speed_hint_m_s=p.main_speed_hint_m_s if i < 4 else p.auxiliary_speed_hint_m_s,
                           left_neighbor=f"MainLane{i}" if i > 0 else None,
                           right_neighbor=(f"MainLane{i+2}" if i < 3 else "AuxiliaryLane") if i < 4 else None)
    aux = lanes["AuxiliaryLane"]["points"]
    events = []
    for name, degrees in SIDES.items():
        for kind, angle in [("Diverge", degrees-p.leaf_half_angle_deg), ("Merge", degrees+p.leaf_half_angle_deg)]:
            sample = round((angle % 360) / 360*p.circle_samples) % p.circle_samples
            assert abs(sample/p.circle_samples*360 - angle % 360) < 1e-8
            events.append((sample, name, kind))
    events.sort()
    nodes = {f"{name}{kind}": dict(auxiliary_sample=i, point=aux[i]) for i, name, kind in events}
    for j, (start, name, kind) in enumerate(events):
        stop, next_name, next_kind = events[(j+1) % len(events)]
        indices = [(start+k) % len(aux) for k in range((stop-start) % len(aux)+1)]
        key = f"Aux_{name}{kind}_to_{next_name}{next_kind}"
        edges[key] = dict(id=key, lane="AuxiliaryLane", kind="auxiliary", closed=False,
                          start_node=f"{name}{kind}", end_node=f"{next_name}{next_kind}",
                          points=[aux[i] for i in indices], usd_path=f"{NAV}/Edges/{key}")
    for name, degrees in SIDES.items():
        points = petal(degrees, p)
        # Canonical shared values prevent tiny endpoint mismatch in sidecars.
        points[0] = nodes[f"{name}Diverge"]["point"]
        points[-1] = nodes[f"{name}Merge"]["point"]
        key = f"Ramp{name}"
        lanes[key] = dict(id=key, kind="ramp", index=1, closed=False, points=points,
                          usd_path=f"{NAV}/Lanes/{key}", width_m=p.lane_width_m,
                          speed_hint_m_s=p.ramp_speed_hint_m_s, left_neighbor=None, right_neighbor=None)
        edges[key] = dict(id=key, kind="ramp", lane=key, closed=False,
                          start_node=f"{name}Diverge", end_node=f"{name}Merge",
                          points=points, usd_path=f"{NAV}/Edges/{key}")
    for i in range(1, 5):
        key = f"MainLane{i}"
        edges[key] = dict(id=key, kind="main", lane=key, closed=True,
                          start_node=None, end_node=None, points=lanes[key]["points"],
                          usd_path=f"{NAV}/Edges/{key}")
    for key, edge in edges.items():
        edge["successors"] = [key] if edge["closed"] else [k for k, e in edges.items() if e["start_node"] == edge["end_node"]]
        edge["length_m"] = path_length(edge["points"], edge["closed"])
    return dict(schema_version=1, units="meters", up_axis="Z", forward_axis="+X",
                east_axis="+X", north_axis="+Y", traffic_direction="counterclockwise",
                point_reference="road surface centerline; apply vehicle-specific height offset",
                waypoint_order="travel direction", parameters=asdict(p), lanes=lanes, nodes=nodes, edges=edges)


def build_shapes(network, p):
    main = disk(p.inner_radius_m+4*p.lane_width_m, p).difference(disk(p.inner_radius_m, p))
    aux = disk(p.outer_radius, p).difference(disk(p.inner_radius_m+4*p.lane_width_m, p))
    strips = {name: LineString(network["lanes"][f"Ramp{name}"]["points"]).buffer(p.lane_width_m/2, cap_style="flat", join_style="round") for name in SIDES}
    pavement = unary_union([main, aux, *strips.values()])
    assert pavement.geom_type == "Polygon" and pavement.is_valid
    return main, aux, strips, pavement


def add_arrows(stage, network, white):
    shapes = []
    for lane in network["lanes"].values():
        pts = lane["points"]
        indices = range(30, len(pts)-30, 120) if lane["closed"] else [256, 512, 768]
        for i in indices:
            a, b = pts[i], pts[(i+1) % len(pts)]
            dx, dy = b[0]-a[0], b[1]-a[1]
            size = math.hypot(dx, dy)
            fx, fy = dx/size, dy/size
            coordinates = [(-1.4,-.13), (.3,-.13), (.3,-.48), (1.4,0), (.3,.48), (.3,.13), (-1.4,.13)]
            shapes.append(Polygon([(a[0]+u*fx-v*fy, a[1]+u*fy+v*fx) for u, v in coordinates]))
    mesh(stage, ROAD+"/Markings/DirectionArrows", unary_union(shapes), .014, white)


def author(output, network, p):
    stage = Usd.Stage.CreateNew(str(output / "highway_v01.usda"))
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, ROOT).GetPrim())
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    stage.SetTimeCodesPerSecond(60)
    stage.SetFramesPerSecond(60)
    stage.GetRootLayer().customLayerData = {"version": "_v01", "generator": "generate_highway.py", "navigation": "navigation.json", "motionAuthority": "external controller; static environment only"}
    for path in [ROAD, ROAD+"/CircularHighway", ROAD+"/AuxiliaryLane", ROAD+"/Ramps", NAV, NAV+"/Lanes", NAV+"/Edges", ROOT+"/SpawnPoints", ROOT+"/MergeZones", ROOT+"/DivergeZones", ROOT+"/Debug", ROOT+"/Physics", ROOT+"/Looks"]:
        UsdGeom.Xform.Define(stage, path)
    attr(stage.GetPrimAtPath(ROAD), "road:direction", Sdf.ValueTypeNames.Token, "counterclockwise")
    attr(stage.GetPrimAtPath(ROAD), "road:mainLaneCount", Sdf.ValueTypeNames.Int, 4)
    attr(stage.GetPrimAtPath(ROAD), "road:auxiliaryLaneCount", Sdf.ValueTypeNames.Int, 1)
    scene = UsdPhysics.Scene.Define(stage, ROOT+"/Physics/Scene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    asphalt = material(stage, "Asphalt", (.07, .085, .10), True)
    auxiliary = material(stage, "AuxiliaryAsphalt", (.095, .115, .13))
    earth = material(stage, "Ground", (.16, .205, .15), True)
    white = material(stage, "WhitePaint", (.85, .87, .86))
    yellow = material(stage, "YellowPaint", (.95, .65, .12))
    main, aux, strips, pavement = build_shapes(network, p)
    mesh(stage, ROAD+"/CircularHighway/Surface", main, 0, asphalt)
    mesh(stage, ROAD+"/AuxiliaryLane/Surface", aux, 0, auxiliary)
    for name, strip in strips.items():
        path = ROAD+"/Ramps/"+name
        parent = UsdGeom.Xform.Define(stage, path).GetPrim()
        relation(parent, "road:navigation", [network["lanes"][f"Ramp{name}"]["usd_path"]])
        relation(parent, "road:exitFrom", [ROAD+"/AuxiliaryLane"])
        relation(parent, "road:mergeInto", [ROAD+"/AuxiliaryLane"])
        mesh(stage, path+"/Surface", strip.difference(disk(p.outer_radius, p)), 0, asphalt)
    collider = mesh(stage, ROOT+"/Physics/RoadCollider", pavement, 0, None, p.road_thickness_m, True)
    UsdShade.MaterialBindingAPI.Apply(collider.GetPrim()).Bind(asphalt, materialPurpose="physics")
    relation(stage.GetPrimAtPath(ROAD), "road:collision", [str(collider.GetPath())])
    extent = p.auxiliary_radius+p.leaf_extension_m+p.ground_margin_m
    # Analytic box avoids an 823 m triangle mesh and PhysX cooking warnings.
    ground = UsdGeom.Cube.Define(stage, ROOT+"/Ground")
    ground.CreateSizeAttr(2.0)
    ground.AddTranslateOp().Set(Gf.Vec3d(0, 0, -p.road_thickness_m-.5))
    ground.AddScaleOp().Set(Gf.Vec3f(extent, extent, .5))
    UsdShade.MaterialBindingAPI.Apply(ground.GetPrim()).Bind(earth)
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim()).CreateSimulationOwnerRel().SetTargets([scene.GetPath()])
    UsdShade.MaterialBindingAPI.Apply(ground.GetPrim()).Bind(earth, materialPurpose="physics")
    UsdGeom.Xform.Define(stage, ROAD+"/Markings")
    white_lines = []
    for poly in polygons(pavement):
        for ring in [poly.exterior, *poly.interiors]:
            white_lines.append(LineString(ring.coords).buffer(.065))
    mesh(stage, ROAD+"/Markings/PavementEdges", unary_union(white_lines), .01, white)
    inner_line = LineString([(x,y) for x,y,_ in circle(p.inner_radius_m+.12, p.circle_samples)] + [(p.inner_radius_m+.12,0)])
    mesh(stage, ROAD+"/Markings/InnerYellow", inner_line.buffer(.08), .012, yellow)
    dashes = []
    for divider in range(1,5):
        radius = p.inner_radius_m + divider*p.lane_width_m
        n = round(TAU*radius/9)
        for i in range(n):
            theta = TAU*i/n
            pts = [(radius*math.cos(theta+j*.75/radius), radius*math.sin(theta+j*.75/radius)) for j in range(5)]
            dashes.append(LineString(pts).buffer(.065, cap_style="flat"))
    mesh(stage, ROAD+"/Markings/LaneDividers", unary_union(dashes), .012, white)
    add_arrows(stage, network, white)
    for name, lane in network["lanes"].items():
        prim = curve(stage, lane["usd_path"], lane["points"], lane["closed"])
        for key, typ, value in [("id", Sdf.ValueTypeNames.String, name), ("kind", Sdf.ValueTypeNames.Token, lane["kind"]), ("width", Sdf.ValueTypeNames.Float, lane["width_m"]), ("speedHint", Sdf.ValueTypeNames.Float, lane["speed_hint_m_s"]), ("closed", Sdf.ValueTypeNames.Bool, lane["closed"])]:
            attr(prim, "lane:"+key, typ, value)
        for side in ["left", "right"]:
            neighbor = lane[side+"_neighbor"]
            if neighbor:
                relation(prim, "lane:"+side+"Neighbor", [network["lanes"][neighbor]["usd_path"]])
        relation(prim, "lane:edges", [e["usd_path"] for e in network["edges"].values() if e["lane"] == name])
    relation(stage.GetPrimAtPath(ROAD+"/CircularHighway"), "road:lanes", [network["lanes"][f"MainLane{i}"]["usd_path"] for i in range(1,5)])
    relation(stage.GetPrimAtPath(ROAD+"/AuxiliaryLane"), "road:lanes", [network["lanes"]["AuxiliaryLane"]["usd_path"]])
    for name, edge in network["edges"].items():
        prim = curve(stage, edge["usd_path"], edge["points"], edge["closed"])
        UsdGeom.Imageable(prim).CreateVisibilityAttr("invisible")
        relation(prim, "navigation:successors", [network["edges"][s]["usd_path"] for s in edge["successors"]])
        relation(prim, "navigation:lane", [network["lanes"][edge["lane"]]["usd_path"]])
        attr(prim, "navigation:length", Sdf.ValueTypeNames.Double, edge["length_m"])
    for name, node in network["nodes"].items():
        kind = "Diverge" if name.endswith("Diverge") else "Merge"
        side = name.removesuffix(kind)
        path = f"{ROOT}/{kind}Zones/{side}"
        prim = UsdGeom.Xform.Define(stage, path).GetPrim()
        UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(*node["point"]))
        attr(prim, "zone:auxiliarySample", Sdf.ValueTypeNames.Int, node["auxiliary_sample"])
        attr(prim, "zone:longitudinalHalfLength", Sdf.ValueTypeNames.Float, 15.0)
        attr(prim, "zone:policy", Sdf.ValueTypeNames.String, "metadata only; controller must implement yielding and routing")
        relation(prim, "zone:lane", [network["lanes"]["AuxiliaryLane"]["usd_path"]])
        relation(prim, "zone:ramp", [network["lanes"][f"Ramp{side}"]["usd_path"]])
        marker = UsdGeom.Sphere.Define(stage, ROOT+f"/Debug/{side}{kind}")
        marker.CreateRadiusAttr(1.2)
        marker.AddTranslateOp().Set(Gf.Vec3d(node["point"][0], node["point"][1], 1.0))
        marker.CreatePurposeAttr("guide")
        marker.CreateDisplayColorAttr([Gf.Vec3f(*( (.95,.45,.10) if kind == "Diverge" else (.15,.85,.45) ))])
    spawns = []
    for lane in list(network["lanes"].values())[:5]:
        for degrees in [45,135,225,315]:
            i = round(degrees/360*p.circle_samples)
            point = lane["points"][i]
            prim = UsdGeom.Xform.Define(stage, f"{ROOT}/SpawnPoints/{lane['id']}_{degrees}").GetPrim()
            xform = UsdGeom.Xformable(prim)
            xform.AddTranslateOp().Set(Gf.Vec3d(*point))
            xform.AddRotateZOp().Set(degrees+90)
            relation(prim, "spawn:lane", [lane["usd_path"]])
            attr(prim, "spawn:sampleIndex", Sdf.ValueTypeNames.Int, i)
            attr(prim, "spawn:heightOffsetRequired", Sdf.ValueTypeNames.Bool, True)
            spawns.append(dict(usd_path=str(prim.GetPath()), lane=lane["id"], point=point, yaw_deg=degrees+90, sample_index=i))
    network["spawn_points"] = spawns
    sun = UsdLux.DistantLight.Define(stage, ROOT+"/Lighting/Sun")
    sun.CreateIntensityAttr(2200)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(25, -30, 0))
    dome = UsdLux.DomeLight.Define(stage, ROOT+"/Lighting/Sky")
    dome.CreateIntensityAttr(450)
    camera = UsdGeom.Camera.Define(stage, ROOT+"/Cameras/Top")
    camera.CreateProjectionAttr("orthographic")
    camera.CreateHorizontalApertureAttr(2*extent*10)
    camera.CreateVerticalApertureAttr(2*extent*10)
    camera.CreateClippingRangeAttr(Gf.Vec2f(.1, 2000))
    camera.AddTranslateOp().Set(Gf.Vec3d(0, 0, 900))
    stage.GetRootLayer().Save()
    return stage, pavement


def preview(output, network, pavement, p):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as Patch
    fig, ax = plt.subplots(figsize=(11,11), facecolor="#eef2ed")
    ax.set_facecolor("#eef2ed")
    poly = orient(pavement, sign=1)
    ax.add_patch(Patch(list(poly.exterior.coords), color="#343e46"))
    for hole in poly.interiors:
        ax.add_patch(Patch(list(hole.coords), color="#eef2ed"))
    for lane in network["lanes"].values():
        pts = lane["points"] + ([lane["points"][0]] if lane["closed"] else [])
        ax.plot([a[0] for a in pts], [a[1] for a in pts], color="#73c6d1" if lane["kind"] != "main" else "#f3f2e9", lw=.65, alpha=.95)
        indices = [0,360,720,1080] if lane["closed"] else [256,512,768]
        for i in indices:
            a,b = pts[i],pts[i+5]
            dx,dy=b[0]-a[0],b[1]-a[1]
            norm=math.hypot(dx,dy)
            ax.arrow(a[0],a[1],8*dx/norm,8*dy/norm,head_width=2.4,head_length=3,color="#73c6d1" if lane["kind"] != "main" else "white",length_includes_head=True)
    for name, node in network["nodes"].items():
        x,y,_=node["point"]
        ax.scatter([x],[y],s=32,c="#df8129" if name.endswith("Diverge") else "#259560",zorder=5)
    for name,degrees in SIDES.items():
        r=p.auxiliary_radius+p.leaf_extension_m+18
        ax.text(r*math.cos(math.radians(degrees)),r*math.sin(math.radians(degrees)),name.upper(),ha="center",va="center",fontsize=10,fontweight="bold",color="#263b34")
    ax.text(0,20,"FOUR-LEAF HIGHWAY",ha="center",fontsize=16,fontweight="bold",color="#263b34")
    ax.text(0,-5,"4 main lanes + continuous outer auxiliary",ha="center",fontsize=11,color="#4b6258")
    ax.text(0,-25,"Counterclockwise · 3.7 m lanes",ha="center",fontsize=10,color="#4b6258")
    ax.text(0,-48,"Orange: diverge    Green: merge",ha="center",fontsize=9,color="#4b6258")
    ax.set_aspect("equal")
    ax.set_xlabel("East / X (m)")
    ax.set_ylabel("North / Y (m)")
    ax.set_title("_v01  |  Navigation and paved footprint",loc="left",pad=16,fontsize=13)
    ax.spines[["top","right"]].set_visible(False)
    fig.tight_layout()
    fig.savefig(output/"overview.png",dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument("--parameters", type=Path, help="Optional JSON overrides; use a new _vNN output folder for later revisions.")
    args = parser.parse_args()
    p = Parameters(**(json.loads(args.parameters.read_text()) if args.parameters else {}))
    p.validate()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    network = build_network(p)
    _, pavement = author(output, network, p)
    (output/"navigation.json").write_text(json.dumps(network, separators=(",",":"), allow_nan=False)+"\n", encoding="utf-8", newline="\n")
    (output/"parameters.json").write_text(json.dumps(asdict(p), indent=2)+"\n", encoding="utf-8", newline="\n")
    preview(output, network, pavement, p)
    manifest = dict(version="_v01", usd_version=".".join(map(str,Usd.GetVersion())), shapely_version=shapely.__version__, parameters=asdict(p), files={})
    for name in ["highway_v01.usda", "navigation.json", "parameters.json", "generate_highway.py"]:
        source = output/name if (output/name).exists() else Path(__file__).with_name(name)
        manifest["files"][name] = hashlib.sha256(source.read_bytes()).hexdigest()
    (output/"manifest.json").write_text(json.dumps(manifest, indent=2)+"\n", encoding="utf-8", newline="\n")
    print(json.dumps(dict(stage=str(output/"highway_v01.usda"), lanes=len(network["lanes"]), edges=len(network["edges"]), paved_area_m2=pavement.area)))


if __name__ == "__main__":
    main()
