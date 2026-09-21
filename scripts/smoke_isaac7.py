"""Check that standalone Isaac Sim 7 physics moves a body under gravity."""
import json
from pathlib import Path
import isaacsim.physics_engines.ovphysx
from isaacsim.ovsim.api import make_client


def main():
    client = make_client("in-process")
    initialized = False
    try:
        if not client.control.authoring.create_stage():
            raise RuntimeError("Stage creation failed")
        for kind in ("Cube", "ColliderBody", "RigidBody"):
            client.control.authoring.define_prim("/World/Cube", kind)
        client.data.write("/World/Cube", "size", 1.0)
        client.data.write("/World/Cube", "position", [0.0, 0.0, 2.0])
        client.control.authoring.define_prim("/World/Ground", "GroundPlane")
        client.control.authoring.define_prim("/World/Physics", "PhysicsScene")
        client.data.write("/World/Physics", "physics:gravityDirection", [0.0, 0.0, -1.0])
        client.data.write("/World/Physics", "physics:gravityMagnitude", 9.81)
        client.control.simulation.set_parameter("physics", "physics-engine", "ovphysx")
        client.control.simulation.initialize()
        initialized = True
        before = float(client.data.read("/World/Cube", "position").numpy()[0, 2])
        for _ in range(10):
            client.control.simulation.step()
        after = float(client.data.read("/World/Cube", "position").numpy()[0, 2])
        if not 0 < after < before:
            raise RuntimeError(f"Gravity check failed: {before} -> {after}")
        result = {"status": "passed", "initial_height": before, "final_height": after}
        output = Path(__file__).resolve().parents[1] / "outputs/migration/physics-smoke.json"
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2))
        print(json.dumps(result), flush=True)
    finally:
        if initialized:
            client.control.simulation.invalidate()
        client.control.authoring.close_stage()


if __name__ == "__main__":
    main()
