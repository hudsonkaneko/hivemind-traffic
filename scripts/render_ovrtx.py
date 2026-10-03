"""Render a timestamp from the portable traffic replay using ovstage + ovrtx."""
import argparse
import hashlib
import importlib.metadata
import json
from time import perf_counter
from pathlib import Path
import numpy as np
from PIL import Image
import ovrtx
import ovstage
from pxr import Usd, UsdGeom, UsdRender, Gf
from replay_export import export_replay, vehicle_path

parser = argparse.ArgumentParser()
parser.add_argument('--minimal', action='store_true', help='Render NVIDIA minimal example scene first')
parser.add_argument('--time', type=float, default=2.0, help='Replay seconds')
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
output = root / 'outputs' / 'ovrtx'
output.mkdir(parents=True, exist_ok=True)
(root / 'logs').mkdir(exist_ok=True)
replay = root / 'outputs' / 'sumo_replay'
manifest = {'runtime': {p: importlib.metadata.version(p) for p in ('ovrtx','ovstage','usd-core','numpy','pillow')},
            'time_seconds': args.time, 'minimal': args.minimal}
if args.minimal:
    source = 'https://omniverse-content-production.s3.us-west-2.amazonaws.com/Samples/Robot-OVRTX/robot-ovrtx.usda'
else:
    info = export_replay(replay)
    if not 0 <= args.time <= info['end_seconds']:
        raise ValueError('Requested time is outside the recorded replay')
    shared = Usd.Stage.Open(str(replay / 'world.usda'))
    tc = args.time * shared.GetTimeCodesPerSecond()
    ego = UsdGeom.Xformable(shared.GetPrimAtPath(vehicle_path('agent_0'))).ComputeLocalToWorldTransform(tc).ExtractTranslation()
    camera_layer = Usd.Stage.CreateNew(str(output / 'render.usda'))
    camera_layer.GetRootLayer().subLayerPaths = ['../sumo_replay/world.usda']
    camera_layer.SetTimeCodesPerSecond(60)
    camera = UsdGeom.Camera.Define(camera_layer, '/Render/View')
    camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(ego+Gf.Vec3d(-18,-22,18), ego+Gf.Vec3d(8,0,0), Gf.Vec3d(0,0,1)).GetInverse())
    camera.CreateClippingRangeAttr(Gf.Vec2f(.1, 3000))
    camera.CreateFocalLengthAttr(24)
    camera.CreateHorizontalApertureAttr(36)
    product = UsdRender.Product.Define(camera_layer, '/Render/Camera')
    product.CreateCameraRel().SetTargets(['/Render/View'])
    product.CreateResolutionAttr(Gf.Vec2i(960,540))
    var = UsdRender.Var.Define(camera_layer, '/Render/Camera/LdrColor')
    var.CreateSourceNameAttr('LdrColor')
    var.CreateDataTypeAttr('color4f')
    product.CreateOrderedVarsRel().SetTargets([var.GetPath()])
    camera_layer.GetRootLayer().Save()
    source = str(output / 'render.usda')
    manifest['export_checks'] = info
    manifest['ego_position_m'] = list(ego)
    recording = json.loads((replay / 'recording.json').read_text())
    manifest['source'] = {key: recording[key] for key in ('seed', 'checkpoint_sha256', 'network_sha256')}
    files = [replay / name for name in ('world.usda','road.usda','motion.usda','vehicle.usda')]
    before = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in files}

print('Starting ovrtx renderer', flush=True)
started = perf_counter()
renderer = ovrtx.Renderer(ovrtx.RendererConfig(
    log_file_path=str(root / 'logs' / 'ovrtx-runtime.log'), log_level='info'))
stage = ovstage.Stage('hivemind.replay')
renderer.attach_ovstage(stage)
try:
    ovstage.population.open_usd(stage, source, ordinal=1, time_code=args.time)
    stage.advance_write_floor(1, ovstage.Scope.ALL).wait()
    pixels = None
    for ordinal in range(1, 9):
        stage.advance_write_floor(ordinal, ovstage.Scope.ALL).wait()
        products = renderer.step(render_products={'/Render/Camera'}, delta_time=1/60, ordinal=ordinal)
        for product in products.values():
            for frame in product.frames:
                mapped = frame.render_vars['/Render/Camera/LdrColor'].map(device=ovrtx.Device.CPU)
                view = np.from_dlpack(mapped)
                pixels = view.copy()
                del view
                mapped.unmap()
                del mapped
        del products
    if pixels is None or pixels[..., :3].std() < 1:
        raise RuntimeError('Renderer returned no usable image')
    filename = 'minimal.png' if args.minimal else f'traffic_{args.time:g}s.png'
    Image.fromarray(pixels).save(output / filename)
    if not args.minimal:
        assert before == {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
        manifest['shared_layer_sha256'] = before
    manifest.update(passed=True, image=filename, shape=list(pixels.shape),
                    wall_time_seconds=perf_counter()-started)
    (output / (filename+'.json')).write_text(json.dumps(manifest, indent=2))
    print(json.dumps(manifest), flush=True)
finally:
    renderer.detach_ovstage()
    stage.destroy()
    renderer.destroy()
