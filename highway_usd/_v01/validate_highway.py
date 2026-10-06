"""Validate the saved stage and directed route graph, independently of generation."""
from __future__ import annotations
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import numpy as np
import shapely
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union
from pxr import Usd, UsdGeom, UsdPhysics, UsdValidation

ROOT = "/World"


def check(condition, message):
    if not condition:
        raise AssertionError(message)


def vector_angle(a, b):
    cosine = np.dot(a,b)/(np.linalg.norm(a)*np.linalg.norm(b))
    return math.degrees(math.acos(float(np.clip(cosine,-1,1))))


def mesh_faces(mesh):
    pts = np.array(mesh.GetPointsAttr().Get(), dtype=float)
    counts = mesh.GetFaceVertexCountsAttr().Get()
    indices = mesh.GetFaceVertexIndicesAttr().Get()
    valid, reason = UsdGeom.Mesh.ValidateTopology(indices, counts, len(pts))
    check(valid, f"Invalid mesh {mesh.GetPath()}: {reason}")
    cursor = 0
    faces = []
    for count in counts:
        faces.append(list(indices[cursor:cursor+count]))
        cursor += count
    return pts, faces


def validate(directory):
    network = json.loads((directory/"navigation.json").read_text())
    p = network["parameters"]
    stage = Usd.Stage.Open(str(directory/"highway_v01.usda"))
    check(stage is not None, "Stage failed to open")
    check(not stage.GetCompositionErrors(), "Composition errors")
    check(str(stage.GetDefaultPrim().GetPath()) == ROOT, "defaultPrim")
    check(UsdGeom.GetStageUpAxis(stage) == "Z", "upAxis")
    check(UsdGeom.GetStageMetersPerUnit(stage) == 1.0, "metersPerUnit")
    expected = ["RoadNetwork", "RoadNetwork/CircularHighway", "RoadNetwork/AuxiliaryLane", "Navigation", "SpawnPoints", "MergeZones", "DivergeZones", "Debug", "Physics"]
    expected += ["RoadNetwork/Ramps/"+s for s in ["North","East","South","West"]]
    for name in expected:
        check(bool(stage.GetPrimAtPath(ROOT+"/"+name)), "Missing hierarchy: "+name)
    rel_count = 0
    for prim in stage.Traverse():
        for rel in prim.GetRelationships():
            for target in rel.GetTargets():
                check(bool(stage.GetObjectAtPath(target)), f"Dangling relationship {rel.GetPath()}: {target}")
                rel_count += 1
        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            pts, faces = mesh_faces(mesh)
            check(np.isfinite(pts).all(), "Nonfinite mesh points")
            check(mesh.GetSubdivisionSchemeAttr().Get() == "none", "Subdivision would change road")
            for face in faces:
                a,b,c = pts[face[:3]]
                check(np.linalg.norm(np.cross(b-a,c-a)) > 1e-12, f"Degenerate face {prim.GetPath()}: {face}")
        check(not prim.HasAPI(UsdPhysics.RigidBodyAPI), "Draft should be static")
    lanes = network["lanes"]
    edges = network["edges"]
    check(len(lanes) == 9 and len(edges) == 16, "Lane/edge count")
    check(len(network["spawn_points"]) == 20, "Spawn count")
    float_error = 0.0
    max_step = 0.0
    for item in [*lanes.values(), *edges.values()]:
        prim = stage.GetPrimAtPath(item["usd_path"])
        c = UsdGeom.BasisCurves(prim)
        pts = np.array(c.GetPointsAttr().Get(), dtype=float)
        reference = np.array(item["points"])
        check(pts.shape == reference.shape, "Curve point count differs from sidecar")
        float_error = max(float_error, float(np.max(np.abs(pts-reference))))
        check(c.GetTypeAttr().Get() == "linear", "Waypoint curves must interpolate the exported path")
        check(c.GetWrapAttr().Get() == ("periodic" if item["closed"] else "nonperiodic"), "Curve closure")
        check(sum(c.GetCurveVertexCountsAttr().Get()) == len(pts), "Curve count")
        sample_steps = np.linalg.norm(np.diff(reference,axis=0),axis=1)
        check(float(sample_steps.min()) > 0, "Duplicate waypoint")
        max_step = max(max_step, float(sample_steps.max()))
    check(float_error < 5e-5, "USD float coordinates differ too far from JSON doubles")
    check(max_step < 2, "Navigation sample gap exceeds 2 m")
    for i in range(5):
        key = f"MainLane{i+1}" if i < 4 else "AuxiliaryLane"
        pts = np.array(lanes[key]["points"])
        radius = p["inner_radius_m"]+(i+.5)*p["lane_width_m"]
        check(np.max(np.abs(np.linalg.norm(pts[:,:2],axis=1)-radius)) < 1e-9, "Non-circular primary lane")
        check(np.cross(pts[0],pts[1])[2] > 0, "Wrong travel direction")
        heading = pts[1]-pts[0]
        right = np.array([heading[1],-heading[0],0])
        check(np.dot(right,pts[0]) > 0, "Outside must be driver's right")
    endpoint_error = 0.0
    tangent_errors = []
    for key, edge in edges.items():
        check(len(edge["successors"]) > 0, "Dead-end edge "+key)
        expected_successors = [str(edges[s]["usd_path"]) for s in edge["successors"]]
        authored = [str(s) for s in stage.GetPrimAtPath(edge["usd_path"]).GetRelationship("navigation:successors").GetTargets()]
        check(authored == expected_successors, "Successor relationship disagrees with graph")
        for successor in edge["successors"]:
            check(successor in edges, "Missing successor")
            if edge["closed"]:
                check(successor == key, "Main lane must repeat")
                continue
            gap = math.dist(edge["points"][-1],edges[successor]["points"][0])
            endpoint_error = max(endpoint_error,gap)
            check(gap < 1e-8, f"Open join {key} to {successor}")
    aux_ramps = {k for k,e in edges.items() if e["kind"] != "main"}
    for start in aux_ramps:
        seen, todo = set(), [start]
        while todo:
            node = todo.pop()
            if node in seen:
                continue
            seen.add(node)
            todo.extend(edges[node]["successors"])
        check(seen == aux_ramps, f"Auxiliary/ramp network is not strongly connected from {start}")
    ramp_metrics = {}
    east = np.array(lanes["RampEast"]["points"])
    for side,angle in [("East",0),("North",90),("West",180),("South",270)]:
        pts = np.array(lanes["Ramp"+side]["points"])
        check(LineString(pts).is_simple, "Ramp self-intersects")
        a = math.radians(angle)
        rotated = east @ np.array([[math.cos(a),math.sin(a),0],[-math.sin(a),math.cos(a),0],[0,0,1]])
        check(np.max(np.abs(rotated-pts)) < 1e-9, "Ramp rotational symmetry")
        for index,direction in [(0,pts[1]-pts[0]),(-1,pts[-1]-pts[-2])]:
            tangent = np.array([-pts[index,1],pts[index,0],0])
            error = vector_angle(tangent,direction)
            tangent_errors.append(error)
            check(error < .1, "Merge/diverge tangent error > 0.1 degrees")
        # Circumcircle radius of three consecutive samples diagnoses tight turns.
        a_len = np.linalg.norm(pts[1:-1]-pts[:-2],axis=1)
        b_len = np.linalg.norm(pts[2:]-pts[1:-1],axis=1)
        c_len = np.linalg.norm(pts[2:]-pts[:-2],axis=1)
        twice_area = np.linalg.norm(np.cross(pts[1:-1]-pts[:-2],pts[2:]-pts[:-2]),axis=1)
        radius = a_len*b_len*c_len/(2*np.maximum(twice_area,1e-18))
        ramp_metrics[side] = dict(length_m=edges["Ramp"+side]["length_m"], min_sampled_radius_m=float(radius.min()), exit_gap_m=math.dist(pts[0],network["nodes"][side+"Diverge"]["point"]), merge_gap_m=math.dist(pts[-1],network["nodes"][side+"Merge"]["point"]))
    collider = UsdGeom.Mesh(stage.GetPrimAtPath(ROOT+"/Physics/RoadCollider"))
    check(collider.GetPrim().HasAPI(UsdPhysics.CollisionAPI), "No road collider")
    check(UsdPhysics.MeshCollisionAPI(collider).GetApproximationAttr().Get() == "none", "Road would become a convex hull")
    check(stage.GetPrimAtPath(ROOT+"/Ground").HasAPI(UsdPhysics.CollisionAPI), "No ground collider")
    pts, faces = mesh_faces(collider)
    undirected, directed = Counter(), Counter()
    top = []
    for face in faces:
        for a,b in zip(face,face[1:]+face[:1]):
            undirected[tuple(sorted((a,b)))] += 1
            directed[(a,b)] += 1
        if np.allclose(pts[face,2],0,atol=1e-9):
            a,b,c = pts[face[:3]]
            check(np.cross(b-a,c-a)[2] > 0, "Downward road face")
            top.append(Polygon(pts[face,:2]))
    check(set(undirected.values()) == {2}, "Road collider is not watertight")
    check(all(directed[(a,b)] == directed[(b,a)] for a,b in directed), "Inconsistent collider winding")
    surface = unary_union(top)
    check(surface.geom_type == "Polygon" and surface.is_valid, "Disconnected collision surface")
    check(len(surface.interiors) == 5, "Expected center island and four petal interiors")
    check(abs(sum(t.area for t in top)-surface.area) < 1e-4, "Overlapping road triangles")
    for key,lane in lanes.items():
        check(surface.buffer(5e-5).covers(LineString(lane["points"])), "Centerline leaves road: "+key)
    # Render surface equals the physics top without overlapping coplanar joins.
    render_polygons = []
    for path in [ROOT+"/RoadNetwork/CircularHighway/Surface",ROOT+"/RoadNetwork/AuxiliaryLane/Surface"]+[ROOT+"/RoadNetwork/Ramps/"+s+"/Surface" for s in ramp_metrics]:
        vp,vf = mesh_faces(UsdGeom.Mesh(stage.GetPrimAtPath(path)))
        render_polygons.extend(Polygon(vp[f,:2]) for f in vf)
    rendered = unary_union(render_polygons)
    check(abs(sum(t.area for t in render_polygons)-rendered.area) < .01, "Coplanar visual overlaps")
    check(surface.symmetric_difference(rendered).area < .05, "Render and collision footprints disagree")
    for spawn in network["spawn_points"]:
        check(surface.contains(Point(spawn["point"][:2])), "Spawn off pavement")
        check(math.dist(spawn["point"],lanes[spawn["lane"]]["points"][spawn["sample_index"]]) == 0, "Spawn off lane")
    validators = UsdValidation.ValidationRegistry().GetOrLoadAllValidators()
    issues = UsdValidation.ValidationContext(validators).Validate(stage)
    compliance_errors = [e.GetMessage() for e in issues if e.GetType() == UsdValidation.ValidationErrorType.Error]
    compliance_failed = []
    compliance_warnings = [e.GetMessage() for e in issues if e.GetType() != UsdValidation.ValidationErrorType.Error]
    check(not compliance_errors and not compliance_failed, f"USD compliance failures: {compliance_errors+compliance_failed}")
    manifest = json.loads((directory/"manifest.json").read_text())
    for name,expected_hash in manifest["files"].items():
        path = directory/name
        if not path.exists() and name == "generate_highway.py":
            path = Path(__file__).with_name(name)
        check(hashlib.sha256(path.read_bytes()).hexdigest() == expected_hash, "Manifest hash mismatch: "+name)
    return dict(passed=True, stage_sha256=manifest["files"]["highway_v01.usda"], usd_version=".".join(map(str,Usd.GetVersion())), prim_count=len(list(stage.Traverse())), lane_count=len(lanes), route_edge_count=len(edges), spawn_count=20, relationship_target_count=rel_count, all_routes_continue=True, auxiliary_and_ramps_strongly_connected=True, ramp_metrics=ramp_metrics, maximum_endpoint_gap_m=endpoint_error, maximum_join_tangent_error_deg=max(tangent_errors), maximum_waypoint_spacing_m=max_step, maximum_usd_float_error_m=float_error, collision_mesh_watertight=True, road_triangle_overlap_area_m2=sum(t.area for t in top)-surface.area, render_collision_difference_area_m2=surface.symmetric_difference(rendered).area, road_area_m2=surface.area, collider_face_count=len(faces), usd_validator_count=len(validators), usd_compliance_errors=compliance_errors, usd_compliance_failed_checks=compliance_failed, usd_compliance_warnings=compliance_warnings, scope="Static USD/schema, sampled geometry and route graph. No vehicle/controller acceptance implied.")


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory",type=Path,default=Path(__file__).resolve().parent)
    args=parser.parse_args()
    result=validate(args.directory.resolve())
    (args.directory/"validation_report.json").write_text(json.dumps(result,indent=2)+"\n")
    print(json.dumps(result,indent=2))


if __name__ == "__main__":
    main()
