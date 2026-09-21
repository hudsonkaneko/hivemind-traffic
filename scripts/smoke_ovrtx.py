"""Validate standalone OVRTX using NVIDIA's reference USD render product.

This tests the renderer only; it does not claim vehicle or physics migration.
"""
import json
import faulthandler
import argparse
from importlib.metadata import version
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "outputs" / "migration" / "ovrtx-smoke"
URL = "https://omniverse-content-production.s3.us-west-2.amazonaws.com/Samples/Robot-OVRTX/robot-ovrtx.usda"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--with-physics", action="store_true")
    args = parser.parse_args()
    if args.with_physics:
        import isaacsim.physics_engines.ovphysx
    import ovrtx
    import ovstage
    faulthandler.dump_traceback_later(120, repeat=True)
    OUTPUT.mkdir(parents=True, exist_ok=True)
    print("Creating OVRTX renderer", flush=True)
    renderer = ovrtx.Renderer()
    print("Creating and attaching scene", flush=True)
    stage = ovstage.Stage("highwaysim.migration.smoke")
    renderer.attach_ovstage(stage)
    try:
        print("Loading reference USD", flush=True)
        ovstage.population.open_usd(stage, URL, ordinal=1)
        stage.advance_write_floor(1, ovstage.Scope.ALL).wait()
        print("Rendering camera", flush=True)
        products = renderer.step(render_products={"/Render/Camera"}, delta_time=1 / 60, ordinal=1)
        saved = 0
        for product in products.values():
            for frame in product.frames:
                print("RENDER_VARS=" + repr(list(frame.render_vars)), flush=True)
                key = "/Render/Camera/LdrColor" if "/Render/Camera/LdrColor" in frame.render_vars else "LdrColor"
                mapped = frame.render_vars[key].map(device=ovrtx.Device.CPU)
                view = np.from_dlpack(mapped)
                pixels = view.copy()
                del view
                mapped.unmap()
                del mapped
                if pixels.size == 0 or not np.isfinite(pixels).all():
                    raise RuntimeError("Invalid camera pixels")
                Image.fromarray(pixels).save(OUTPUT / f"frame-{saved}.png")
                saved += 1
        if not saved:
            raise RuntimeError("Renderer produced no camera frames")
        del frame, product, products
        result = {"status": "passed", "frames": saved, "scene": URL, "physics_loaded": args.with_physics,
                  "ovrtx": version("ovrtx"), "ovstage_distribution": version("ovstage"),
                  "loaded_ovstage_module": ovstage.__file__}
        (OUTPUT / "result.json").write_text(json.dumps(result, indent=2))
        print(json.dumps(result), flush=True)
    finally:
        renderer.detach_ovstage()
        stage.destroy()
        renderer.destroy()


if __name__ == "__main__":
    main()
