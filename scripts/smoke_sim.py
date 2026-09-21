"""Check standalone startup and gravity using the installed Isaac Sim."""
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
import json
import omni.usd
import isaacsim.core.experimental.utils.app as app_utils
from isaacsim.core.experimental.objects import Cube, GroundPlane
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim

GroundPlane("/World/Ground")
Cube("/World/Cube", positions=[0, 0, 2], sizes=0.5)
body = RigidPrim("/World/Cube")
GeomPrim("/World/Cube", apply_collision_apis=True)
app_utils.play()
for _ in range(180):
    app.update()
position, _ = body.get_world_poses()
z = float(position.numpy()[0, 2])
print("SMOKE_RESULT=" + json.dumps({"cube_z": z, "passed": 0.15 < z < 0.4}), flush=True)
app.close(exit_code=0 if 0.15 < z < 0.4 else 1)
