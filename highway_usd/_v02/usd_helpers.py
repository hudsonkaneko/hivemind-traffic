"""Version-local USD mesh/curve helpers, retained from v01 with no v01 edits."""
import math
import shapely
from shapely.geometry import Polygon
from shapely.geometry.polygon import orient
from pxr import Gf,Sdf,UsdGeom,UsdPhysics,UsdShade,Vt
ROOT="/World"
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
