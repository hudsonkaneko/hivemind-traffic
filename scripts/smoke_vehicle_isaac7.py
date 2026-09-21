"""Validate the existing Leatherback asset against standalone Isaac Sim 7."""
import json
import argparse
from pathlib import Path
import numpy as np
import warp as wp
import isaacsim.physics_engines.ovphysx
from isaacsim.foundation.objects import Stage, PhysicsScene
from isaacsim.foundation.prims import GroundPlane
from isaacsim.physics.entities import ArticulationEntity
from isaacsim.physics.manager import PhysicsManager
from pxr import Sdf

ROOT = Path(__file__).resolve().parents[1]
ASSET = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/Robots/NVIDIA/Leatherback/leatherback.usd"
WHEELS = ["Wheel__Knuckle__Front_Left", "Wheel__Knuckle__Front_Right",
          "Wheel__Upright__Rear_Left", "Wheel__Upright__Rear_Right"]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dt", type=float, default=1 / 120)
    args = parser.parse_args()
    author = Stage("openusd").create_stage()
    try:
        author.define_prim("/World")
        GroundPlane("/World/Ground", sizes=100.0)
        PhysicsScene("/World/Physics").set_gravities([0.0, 0.0, -9.81])
        layer = Sdf.Layer.CreateAnonymous("vehicle.usda")
        layer.ImportFromString(author.export_stage_to_string())
        car = Sdf.CreatePrimInLayer(layer, "/World/Car")
        car.specifier = Sdf.SpecifierDef
        car.typeName = "Xform"
        car.referenceList.prependedItems = [Sdf.Reference(ASSET)]
        Sdf.AttributeSpec(car, "physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).default = False
        Sdf.AttributeSpec(car, "physxArticulation:solverPositionIterationCount", Sdf.ValueTypeNames.Int).default = 8
        Sdf.AttributeSpec(car, "physxArticulation:solverVelocityIterationCount", Sdf.ValueTypeNames.Int).default = 2
        Sdf.CreatePrimInLayer(layer, "/World/Car/CollisionGroup").active = False
        text = layer.ExportToString()
    finally:
        author.close_stage()
    output = ROOT / "outputs/migration/vehicle"
    output.mkdir(parents=True, exist_ok=True)
    (output / "scene.usda").write_text(text)
    stage = Stage("ovstage").import_stage_from_string(text, make_default=False)
    manager = PhysicsManager.get_instance()
    try:
        if not manager.switch_physics_engine("ovphysx"):
            raise RuntimeError("OvPhysX unavailable")
        manager.setup(dt=args.dt)
        if not manager.initialize(stage.get_stage_ptr(), stage.get_stage_id()):
            raise RuntimeError("Vehicle physics initialization failed")
        car = ArticulationEntity("ovphysx", "/World/Car")
        print("JOINTS=" + repr(car.dof_names), flush=True)
        print("INITIAL_POSE=" + repr(car.get_world_poses()[0].numpy()), flush=True)
        print("INITIAL_JOINTS=" + repr(car.get_dof_positions().numpy()), flush=True)
        health = {"masses": car.get_link_masses().numpy().tolist(),
                  "inertias": car.get_link_inertias().numpy().tolist(),
                  "gains": [a.numpy().tolist() for a in car.get_dof_gains()],
                  "link_names": list(car.link_names)}
        (output / "initial-health.json").write_text(json.dumps(health, indent=2))
        # Preserve authored joint poses for this importer validation; changing
        # suspension joints independently can violate the asset's closed loops.
        for step in range(round(1 / args.dt)):
            manager.step()
            if not np.isfinite(car.get_world_poses()[0].numpy()).all():
                raise RuntimeError(f"Vehicle became non-finite at settling step {step + 1}, dt={args.dt}")
        start = car.get_world_poses()[0].numpy()[0].copy()
        targets = np.zeros((1, len(car.dof_names)), dtype=np.float32)
        for name in WHEELS:
            targets[0, list(car.dof_names).index(name)] = 1 / 0.052
        car.set_dof_velocity_targets(wp.array(targets, dtype=wp.float32, device="cpu"))
        manager.step(steps=round(2 / args.dt))
        end = car.get_world_poses()[0].numpy()[0].copy()
        distance = float(np.linalg.norm(end[:2] - start[:2]))
        result = {"start": start.tolist(), "end": end.tolist(), "distance": distance,
                  "joint_names": list(car.dof_names), "status": "passed" if distance > 0.25 and abs(end[2]) < 1 else "failed"}
        (output / "result.json").write_text(json.dumps(result, indent=2))
        print(json.dumps(result), flush=True)
        if result["status"] != "passed":
            raise RuntimeError("Vehicle did not drive stably")
    finally:
        if manager.is_initialized():
            manager.invalidate()
        stage.close_stage()


if __name__ == "__main__":
    main()
