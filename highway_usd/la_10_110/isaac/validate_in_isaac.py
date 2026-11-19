"""Physics smoke test inside Isaac Sim (headless).

Drops a 1 m rigid cube 1 m above every spawn point, simulates 4 s, and checks
each cube came to rest on the freeway deck (i.e. the road colliders work and
nothing falls through). Run from the Isaac Sim install folder:

    python.bat <path>/la_10_110/isaac/validate_in_isaac.py [--gui]
"""
import argparse
import os
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--gui", action="store_true")
ap.add_argument("--usd", default=os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "la_10_110.usda"))
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": not args.gui})

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, UsdGeom, UsdPhysics  # noqa: E402

ctx = omni.usd.get_context()
ctx.open_stage(args.usd)
for _ in range(30):
    app.update()
stage = ctx.get_stage()
print("opened", args.usd)

spawns = [p for p in stage.GetPrimAtPath("/World/SpawnPoints").GetChildren()]
probes = []
for i, sp in enumerate(spawns):
    t = sp.GetAttribute("xformOp:translate").Get()
    path = f"/World/Probes/Cube_{i}"
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(t[0], t[1], t[2] + 1.0))
    UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(500.0)
    probes.append((cube, t[2] - 0.5, sp.GetName()))

tl = omni.timeline.get_timeline_interface()
tl.play()
for _ in range(240):
    app.update()
tl.pause()

fails = 0
for cube, deck_z, name in probes:
    z = omni.usd.get_world_transform_matrix(cube.GetPrim()).ExtractTranslation()[2]
    ok = abs(z - (deck_z + 0.5)) < 0.6
    fails += not ok
    if not ok:
        print(f"FAIL {name}: cube z={z:.2f}, deck z={deck_z:.2f}")
print(f"{len(probes) - fails}/{len(probes)} spawn points hold a rigid body on the deck")
if not args.gui:
    app.close()
    sys.exit(1 if fails else 0)
while app.is_running():
    app.update()
app.close()
