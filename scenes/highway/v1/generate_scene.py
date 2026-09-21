"""Generate the version 1 circular highway USD scene.

The file is intentionally dependency-free so road dimensions can be changed
without requiring Isaac Sim's Python environment.
"""

from __future__ import annotations

import math
from pathlib import Path


SEGMENTS = 192
LANES = 3
LANE_WIDTH = 3.7
CENTER_RADIUS = 30.0
ROAD_THICKNESS = 0.12


def fmt(value: float) -> str:
    return f"{value:.6f}".rstrip("0").rstrip(".")


def vec(point: tuple[float, float, float]) -> str:
    return f"({fmt(point[0])}, {fmt(point[1])}, {fmt(point[2])})"


def ring_mesh(inner: float, outer: float, z_top: float, thickness: float) -> tuple[list, list]:
    points = []
    for z in (z_top, z_top - thickness):
        for radius in (inner, outer):
            points.extend(
                (radius * math.cos(i * math.tau / SEGMENTS), radius * math.sin(i * math.tau / SEGMENTS), z)
                for i in range(SEGMENTS)
            )

    faces = []
    ti, to, bi, bo = 0, SEGMENTS, SEGMENTS * 2, SEGMENTS * 3
    for i in range(SEGMENTS):
        j = (i + 1) % SEGMENTS
        faces.extend(
            [
                (ti + i, ti + j, to + j, to + i),
                (bi + i, bo + i, bo + j, bi + j),
                (ti + i, bi + i, bi + j, ti + j),
                (to + i, to + j, bo + j, bo + i),
            ]
        )
    return points, faces


def flat_ring(inner: float, outer: float, z: float, segments: int = SEGMENTS) -> tuple[list, list]:
    points = []
    for radius in (inner, outer):
        points.extend(
            (radius * math.cos(i * math.tau / segments), radius * math.sin(i * math.tau / segments), z)
            for i in range(segments)
        )
    faces = [(i, (i + 1) % segments, segments + (i + 1) % segments, segments + i) for i in range(segments)]
    return points, faces


def dashed_ring(radius: float, width: float, z: float, dash_count: int = 48, duty: float = 0.56) -> tuple[list, list]:
    points, faces = [], []
    half = width / 2
    for i in range(dash_count):
        a0 = math.tau * (i / dash_count)
        a1 = math.tau * ((i + duty) / dash_count)
        base = len(points)
        points.extend(
            [
                ((radius - half) * math.cos(a0), (radius - half) * math.sin(a0), z),
                ((radius - half) * math.cos(a1), (radius - half) * math.sin(a1), z),
                ((radius + half) * math.cos(a1), (radius + half) * math.sin(a1), z),
                ((radius + half) * math.cos(a0), (radius + half) * math.sin(a0), z),
            ]
        )
        faces.append((base, base + 1, base + 2, base + 3))
    return points, faces


def arrows(z: float) -> tuple[list, list]:
    # Arrow points in local coordinates: x is clockwise travel, y is lane lateral.
    shape = [(-1.25, -0.16), (0.25, -0.16), (0.25, -0.48), (1.25, 0.0), (0.25, 0.48), (0.25, 0.16), (-1.25, 0.16)]
    points, faces = [], []
    for lane in range(LANES):
        radius = CENTER_RADIUS + (lane - (LANES - 1) / 2) * LANE_WIDTH
        for marker in range(8):
            angle = math.tau * (marker / 8 + lane / 24)
            center = (radius * math.cos(angle), radius * math.sin(angle))
            travel = (math.sin(angle), -math.cos(angle))  # clockwise tangent
            lateral = (math.cos(angle), math.sin(angle))
            base = len(points)
            for x, y in shape:
                points.append((center[0] + travel[0] * x + lateral[0] * y, center[1] + travel[1] * x + lateral[1] * y, z))
            faces.append(tuple(base + i for i in range(len(shape))))
    return points, faces


def mesh(name: str, points: list, faces: list, material: str, collision: bool = False) -> str:
    api = ' prepend apiSchemas = ["PhysicsCollisionAPI"]' if collision else ""
    counts = ", ".join(str(len(face)) for face in faces)
    indices = ", ".join(str(index) for face in faces for index in face)
    return f'''    def Mesh "{name}" ({api.strip()})
    {{
        uniform token subdivisionScheme = "none"
        bool doubleSided = true
        point3f[] points = [{", ".join(vec(p) for p in points)}]
        int[] faceVertexCounts = [{counts}]
        int[] faceVertexIndices = [{indices}]
        rel material:binding = </World/Looks/{material}>
    }}
'''


def material(name: str, color: tuple[float, float, float], roughness: float) -> str:
    return f'''        def Material "{name}"
        {{
            token outputs:surface.connect = </World/Looks/{name}/Shader.outputs:surface>
            def Shader "Shader"
            {{
                uniform token info:id = "UsdPreviewSurface"
                color3f inputs:diffuseColor = ({fmt(color[0])}, {fmt(color[1])}, {fmt(color[2])})
                float inputs:roughness = {fmt(roughness)}
                token outputs:surface
            }}
        }}
'''


def generate() -> str:
    road_half_width = LANES * LANE_WIDTH / 2
    inner, outer = CENTER_RADIUS - road_half_width, CENTER_RADIUS + road_half_width
    road_points, road_faces = ring_mesh(inner, outer, 0.08, ROAD_THICKNESS)

    parts = [
        '#usda 1.0\n(\n    defaultPrim = "World"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n',
        'def Xform "World"\n{\n',
        '    def Scope "Looks"\n    {\n',
        material("Asphalt", (0.055, 0.06, 0.065), 0.88),
        material("WhitePaint", (0.92, 0.92, 0.84), 0.72),
        material("Grass", (0.09, 0.22, 0.075), 0.95),
        '    }\n',
        '''    def Cube "Ground" (prepend apiSchemas = ["PhysicsCollisionAPI"])
    {
        double size = 2
        double3 xformOp:scale = (60, 60, 0.05)
        double3 xformOp:translate = (0, 0, 0.02)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
        rel material:binding = </World/Looks/Grass>
    }
''',
        # Tire contact uses the flat ground collider directly beneath the road.
        # Keeping collision off the closed annular mesh avoids unstable contact
        # normals at its densely tessellated edges.
        mesh("Road", road_points, road_faces, "Asphalt"),
    ]

    # Solid shoulder boundaries.
    for name, radius in (("InnerEdgeLine", inner + 0.18), ("OuterEdgeLine", outer - 0.18)):
        points, faces = flat_ring(radius - 0.08, radius + 0.08, 0.145)
        parts.append(mesh(name, points, faces, "WhitePaint"))

    # Dashed separators between each lane.
    for index in range(1, LANES):
        radius = inner + index * LANE_WIDTH
        points, faces = dashed_ring(radius, 0.14, 0.15)
        parts.append(mesh(f"LaneDivider{index}", points, faces, "WhitePaint"))

    arrow_points, arrow_faces = arrows(0.155)
    parts.append(mesh("ClockwiseDirectionArrows", arrow_points, arrow_faces, "WhitePaint"))
    parts.append('''    def DistantLight "Sun"
    {
        float inputs:angle = 0.53
        float inputs:intensity = 3000
        color3f inputs:color = (1, 0.95, 0.86)
        quatf xformOp:orient = (0.822, 0.22, -0.44, -0.29)
        uniform token[] xformOpOrder = ["xformOp:orient"]
    }
}
''')
    return "".join(parts)


if __name__ == "__main__":
    destination = Path(__file__).with_name("highway_v1.usda")
    destination.write_text(generate(), encoding="utf-8", newline="\n")
    print(f"Wrote {destination}")
