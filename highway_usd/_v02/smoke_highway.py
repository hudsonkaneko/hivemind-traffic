"""Bounded Isaac Sim road-contact probe; dynamic spheres exist only in memory.

Run with ordinary Python. The supervisor launches the existing Isaac runtime,
retains logs/results in a new --output directory, and enforces a wall deadline.
"""
from __future__ import annotations
import argparse
import gc
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def child(output):
    app = session = None
    result = dict(passed=False, stage_sha256=digest(HERE/"highway_v02.usda"))
    started = time.monotonic()
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({"headless":True,"width":640,"height":480,"disable_viewport_updates":True})
        import omni.physx
        from pxr import Gf, UsdGeom, UsdPhysics, Usd
        sys.path.insert(0,str(PROJECT))
        from traffic.physics_session import PhysicsSession
        session = PhysicsSession(120)
        stage = session.stage
        stage.GetRootLayer().subLayerPaths = [str(HERE/"highway_v02.usda")]
        network = json.loads((HERE/"navigation.json").read_text())
        locations = [(f"main_{i}", network["lanes"][f"MainLane{i}"]["points"][200], .3) for i in range(1,5)]
        for name in ["AuxiliaryLane","CollectorLane"]:
            locations.append((name,network["lanes"][name]["points"][200],.3))
        for name,lane in network["lanes"].items():
            if lane["kind"] in ["exit","return"]:
                for i in [160,320,480]:
                    locations.append((f"{name}_{i}",lane["points"][i],.3))
        locations += [(name,node["point"],.3) for name,node in network["nodes"].items()]
        locations.append(("ground_center",[0,0,0],0.0))
        bodies = []
        for name,point,expected_z in locations:
            path = "/World/SmokeBodies/"+name
            sphere = UsdGeom.Sphere.Define(stage,path)
            sphere.CreateRadiusAttr(.3)
            sphere.AddTranslateOp().Set(Gf.Vec3d(point[0],point[1],1.5))
            prim = sphere.GetPrim()
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.RigidBodyAPI.Apply(prim)
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(20.0)
            bodies.append((path,point,expected_z))
        # No USD handles are retained across session close.
        sphere = prim = None
        session.attach()
        physx = omni.physx.get_physx_interface()
        for tick in range(360):
            session.step(1/120,tick/120)
        contacts = []
        for path,point,expected_z in bodies:
            state = physx.get_rigidbody_transformation(path)
            if not state["ret_val"]:
                raise RuntimeError("Missing rigid-body state: "+path)
            position = list(state["position"])
            error = abs(position[2]-expected_z)
            xy_error = math.hypot(position[0]-point[0],position[1]-point[1])
            contacts.append(dict(path=path,position_m=position,expected_z_m=expected_z,height_error_m=error,xy_drift_m=xy_error,passed=all(math.isfinite(v) for v in position) and error < .04 and xy_error < .05))
        result.update(passed=all(c["passed"] for c in contacts),contacts=contacts,physics_hz=120,steps=360,simulated_seconds=3.0,usd_version=".".join(map(str,Usd.GetVersion())),scope="47 stationary sphere drop contacts at main/aux lanes, ramps, all joins and ground; no vehicle driving, tire, yielding or traffic validation")
        stage = None
        gc.collect()
        session.close()
        result["lifecycle"] = session.lifecycle
        session = None
    except Exception:
        result["passed"] = False
        result["exception"] = traceback.format_exc()
    finally:
        result["wall_seconds"] = time.monotonic()-started
        (output/"contact_result.json").write_text(json.dumps(result,indent=2)+"\n")
        print("HIGHWAY_CONTACT_RESULT="+json.dumps(result),flush=True)
        if app:
            app.close()
    return 0 if result["passed"] else 1


def supervise(args):
    output = args.output.resolve()
    if not output.is_relative_to(PROJECT/"outputs"):
        raise ValueError("Smoke evidence must be inside the project's outputs folder")
    output.mkdir(parents=True,exist_ok=False)
    config = dict(stage_sha256=digest(HERE/"highway_v02.usda"),script_sha256=digest(Path(__file__)),runtime=str(args.runtime),timeout_seconds=args.timeout,steps=360,physics_hz=120)
    (output/"resolved_config.json").write_text(json.dumps(config,indent=2)+"\n")
    command = [str(args.runtime),str(Path(__file__)),"--child","--output",str(output)]
    before=time.monotonic()
    timed_out=False
    with (output/"runtime.log").open("w",encoding="utf-8") as log:
        process=subprocess.Popen(command,cwd=str(PROJECT),stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
        try:
            returncode=process.wait(timeout=args.timeout)
        except subprocess.TimeoutExpired:
            timed_out=True
            if os.name == "nt":
                subprocess.run(["taskkill","/PID",str(process.pid),"/T","/F"],capture_output=True,check=False)
            else:
                process.kill()
            returncode=process.wait(timeout=20)
    log=(output/"runtime.log").read_text(encoding="utf-8",errors="replace")
    contacts=json.loads((output/"contact_result.json").read_text()) if (output/"contact_result.json").exists() else {}
    errors=[line for line in log.splitlines() if "[Error]" in line or "Unexpected reference count" in line]
    result=dict(passed=returncode == 0 and contacts.get("passed",False) and not timed_out and not errors,contact_checks_passed=contacts.get("passed",False),returncode=returncode,timed_out=timed_out,runtime_error_lines=errors,runtime_warning_count=sum("[Warning]" in line for line in log.splitlines()),stage_unchanged=digest(HERE/"highway_v02.usda") == config["stage_sha256"],elapsed_seconds=time.monotonic()-before,stage_sha256=config["stage_sha256"],evidence=str(output.relative_to(PROJECT)))
    result["passed"] = result["passed"] and result["stage_unchanged"]
    (output/"smoke_result.json").write_text(json.dumps(result,indent=2)+"\n")
    print(json.dumps(result,indent=2))
    return 0 if result["passed"] else 1


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--runtime",type=Path,default=Path("C:/isaacsim/python.bat"))
    parser.add_argument("--timeout",type=float,default=180)
    parser.add_argument("--child",action="store_true",help=argparse.SUPPRESS)
    args=parser.parse_args()
    return child(args.output.resolve()) if args.child else supervise(args)


if __name__ == "__main__":
    sys.exit(main())
