"""Capture actual USD/RTX views for this version, without starting simulation.

Use the project's .venv-ovrtx Python. --scene and --output support later _vNN
folders. Keep the screenshots, camera layer and manifest with each version.
"""
from __future__ import annotations
import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import time
import numpy as np
from PIL import Image
import ovrtx
import ovstage
from pxr import Gf, Usd, UsdGeom, UsdRender


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene",type=Path,default=Path(__file__).with_name("highway_v02.usda"))
    parser.add_argument("--output",type=Path,default=Path(__file__).with_name("screenshots"))
    parser.add_argument("--frames",type=int,default=24)
    args=parser.parse_args()
    source=args.scene.resolve()
    output=args.output.resolve()
    output.mkdir(parents=True,exist_ok=True)
    project=Path(__file__).resolve().parents[2]
    log_dir=project/"logs"/"highway_usd"
    log_dir.mkdir(parents=True,exist_ok=True)
    network=json.loads(source.with_name("navigation.json").read_text())
    p=network["parameters"]
    reach=p["collector_radius_m"]+p["lane_width_m"]/2
    # Long gentle tapers share pavement near the graph's decision point.
    # Frame the physical fork farther along the exit, rather than an unbranched section.
    exit_point=np.array(network["lanes"]["EastExit"]["points"][256])
    views={
        "top":dict(eye=[0,0,reach*2.4],target=[0,0,0],up=[0,1,0],resolution=[1800,1800],ortho_width_m=reach*2.15),
        "angled":dict(eye=[reach*1.4,-reach*1.75,reach*1.9],target=[0,0,0],up=[0,0,1],resolution=[1800,1400],focal_length_mm=36),
        "east_diverge":dict(eye=(exit_point+np.array([75,-65,115])).tolist(),target=(exit_point+np.array([4,14,0])).tolist(),up=[0,0,1],resolution=[1800,1200],focal_length_mm=45),
    }
    camera_path=output/"capture_views.usda"
    usd=Usd.Stage.CreateNew(str(camera_path))
    usd.GetRootLayer().subLayerPaths=[os.path.relpath(source,output).replace("\\","/")]
    UsdGeom.SetStageUpAxis(usd,"Z")
    UsdGeom.SetStageMetersPerUnit(usd,1)
    # Guides and collision inspection surfaces are excluded only in this view layer.
    for path in ["/World/Navigation","/World/Debug","/World/Physics/RoadCollider"]:
        UsdGeom.Imageable(usd.OverridePrim(path)).CreateVisibilityAttr("invisible")
    for name,view in views.items():
        root="/Render/"+name
        camera=UsdGeom.Camera.Define(usd,root+"/Camera")
        camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*view["eye"]),Gf.Vec3d(*view["target"]),Gf.Vec3d(*view["up"])).GetInverse())
        camera.CreateClippingRangeAttr(Gf.Vec2f(.1,5000))
        if "ortho_width_m" in view:
            camera.CreateProjectionAttr("orthographic")
            camera.CreateHorizontalApertureAttr(view["ortho_width_m"]*10)
            camera.CreateVerticalApertureAttr(view["ortho_width_m"]*10*view["resolution"][1]/view["resolution"][0])
        else:
            camera.CreateHorizontalApertureAttr(36)
            camera.CreateVerticalApertureAttr(36*view["resolution"][1]/view["resolution"][0])
            camera.CreateFocalLengthAttr(view["focal_length_mm"])
        product=UsdRender.Product.Define(usd,root+"/Product")
        product.CreateCameraRel().SetTargets([camera.GetPath()])
        product.CreateResolutionAttr(Gf.Vec2i(*view["resolution"]))
        var=UsdRender.Var.Define(usd,root+"/Color")
        var.CreateSourceNameAttr("LdrColor")
        var.CreateDataTypeAttr("color4f")
        product.CreateOrderedVarsRel().SetTargets([var.GetPath()])
    usd.GetRootLayer().Save()
    usd=None
    stage_hash=hashlib.sha256(source.read_bytes()).hexdigest()
    started=time.monotonic()
    renderer=ovrtx.Renderer(ovrtx.RendererConfig(log_file_path=str(log_dir/(source.parent.name+"-screenshots.log")),log_level="warning"))
    stage=ovstage.Stage("highway.documentation")
    renderer.attach_ovstage(stage)
    captures={}
    try:
        ovstage.population.open_usd(stage,str(camera_path),ordinal=1,time_code=0)
        product_paths={"/Render/"+name+"/Product" for name in views}
        for ordinal in range(1,args.frames+1):
            stage.advance_write_floor(ordinal,ovstage.Scope.ALL).wait()
            products=renderer.step(render_products=product_paths,delta_time=1/60,ordinal=ordinal)
            for product_path, product in products.items():
                name=str(product_path).split("/")[2]
                for frame in product.frames:
                    mapped=frame.render_vars["/Render/"+name+"/Color"].map(device=ovrtx.Device.CPU)
                    data=np.from_dlpack(mapped)
                    captures[name]=data.copy()
                    del data
                    mapped.unmap()
                    del mapped
            del products
        for name in views:
            pixels=captures[name]
            if pixels[...,:3].std() < 1:
                raise RuntimeError("Blank capture: "+name)
            Image.fromarray(pixels).save(output/(name+".png"))
            views[name]["image"]=name+".png"
            views[name]["image_sha256"]=hashlib.sha256((output/(name+".png")).read_bytes()).hexdigest()
        assert hashlib.sha256(source.read_bytes()).hexdigest() == stage_hash
        report=dict(version=source.parent.name,source_scene=source.name,stage_sha256=stage_hash,source_unchanged=True,capture_method="ovstage + ovrtx, actual USD render; static scene, no physics",frames=args.frames,views=views,runtime={name:importlib.metadata.version(name) for name in ["ovstage","ovrtx","usd-core","numpy","pillow"]},elapsed_seconds=time.monotonic()-started)
        (output/"screenshots.json").write_text(json.dumps(report,indent=2)+"\n")
        print(json.dumps(report,indent=2),flush=True)
    finally:
        renderer.detach_ovstage()
        stage.destroy()
        renderer.destroy()


if __name__ == "__main__":
    main()
