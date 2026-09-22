"""Controlled kinematic RTX lidar geometry/timing test; run with Isaac Python."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
import time
import uuid

import numpy as np
from lidar_geometry import analyze_scan, fixture_positions, summarize

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def command(args):
    result = subprocess.run(args, cwd=ROOT, capture_output=True, text=True)
    return result.stdout.strip() if result.returncode == 0 else result.stderr.strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--variant', choices=['static', 'moving'], default='moving')
    parser.add_argument('--config', type=Path, default=ROOT / 'experiments/configs/lidar_validation.json')
    parser.add_argument('--gui', action='store_true')
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    moving = args.variant == 'moving'
    run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + args.variant + '-' + uuid.uuid4().hex[:6]
    output = ROOT / 'outputs/lidar_validation' / run_id
    output.mkdir(parents=True, exist_ok=False)
    (output / 'resolved-config.json').write_text(json.dumps(config, indent=2))
    diff = command(['git', 'diff', 'HEAD'])
    (output / 'working-tree.patch').write_text(diff)
    source_dir = output / 'source'
    source_dir.mkdir()
    for source in [Path(__file__), ROOT/'scripts/lidar_geometry.py']:
        (source_dir / source.name).write_bytes(source.read_bytes())
    manifest = dict(run_id=run_id, status='running', start_utc=datetime.now(timezone.utc).isoformat(),
                    command=[sys.executable, *sys.argv], git_commit=command(['git', 'rev-parse', 'HEAD']),
                    git_status=command(['git', 'status', '--porcelain']), python=sys.version,
                    os=platform.platform(), cpu=platform.processor(), numpy=np.__version__,
                    gpu=command(['nvidia-smi', '--query-gpu=name,driver_version', '--format=csv,noheader']),
                    variant=args.variant, seed=None, checkpoint=None,
                    reason='Deterministic analytic fixture; no training or random placement.',
                    authority='USD kinematic fixture owns both poses; no physics actuation or live SUMO.',
                    inputs={str(p.relative_to(ROOT)): digest(p) for p in
                            [Path(__file__), ROOT/'scripts/lidar_geometry.py', args.config.resolve()]})
    manifest_path = output / 'manifest.json'
    manifest_path.write_text(json.dumps(manifest, indent=2))
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': not args.gui, 'width': 640, 'height': 480, 'enable_motion_bvh': True})
    records, failures = [], []
    start = time.monotonic()
    try:
        import omni.usd
        import omni.timeline
        import omni.replicator.core as rep
        from pxr import Gf, Usd, UsdGeom, UsdLux
        from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor, parse_generic_model_output_data
        from isaacsim.sensors.experimental.rtx.generic_model_output import ElementFlags
        assert ElementFlags.VALID.value == 64, 'Unsupported GMO validity mask'
        stage = omni.usd.get_context().get_stage()
        manifest['usd_version'] = Usd.GetVersion()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1)
        stage.SetTimeCodesPerSecond(config['fps'])
        stage.SetFramesPerSecond(config['fps'])
        stage.SetStartTimeCode(0)
        stage.SetEndTimeCode(config['frames'] + 60)
        ego = UsdGeom.Xform.Define(stage, '/World/Ego')
        target = UsdGeom.Xform.Define(stage, '/World/Target')
        ego_op, target_op = ego.AddTranslateOp(), target.AddTranslateOp()
        for tick in range(config['frames'] + 61):
            origins, centers = fixture_positions(np.array([tick / config['fps']]), config, moving)
            ego_op.Set(Gf.Vec3d(*(origins[0] - config['sensor_mount_m']).tolist()), tick)
            target_op.Set(Gf.Vec3d(*centers[0].tolist()), tick)
        for path, scale, translation in [('/World/Ego/Body', [4, 2, .6], [-2, 0, .3]),
                                          ('/World/Target/Body', config['target_size_m'], [0, 0, 0])]:
            cube = UsdGeom.Cube.Define(stage, path)
            cube.CreateSizeAttr(1)
            cube.AddTranslateOp().Set(Gf.Vec3d(*translation))
            cube.AddScaleOp().Set(Gf.Vec3f(*scale))
        UsdLux.DomeLight.Define(stage, '/World/Light').CreateIntensityAttr(800)
        if args.gui:
            from isaacsim.core.experimental.utils.app import enable_extension
            enable_extension('isaacsim.sensors.rtx.nodes')
        lidar = Lidar.create('/World/Ego/Lidar', config=config['sensor_config'],
                             translations=np.array(config['sensor_mount_m']))
        sensor = LidarSensor(lidar, annotators=[])
        prim = stage.GetPrimAtPath('/World/Ego/Lidar')
        manifest['sensor_attributes'] = {a.GetName(): str(a.Get()) for a in prim.GetAttributes() if a.GetName().startswith('omni:sensor')}
        version_path = Path(sys.executable).parent.parent / 'VERSION'
        manifest['isaac_version'] = version_path.read_text().strip() if version_path.exists() else 'See runtime log'
        timeline = omni.timeline.get_timeline_interface()
        timeline.set_target_framerate(config['fps'])
        timeline.set_end_time((config['frames'] + 60) / config['fps'])
        timeline.set_looping(False)

        class ValidationWriter(rep.Writer):
            def __init__(self):
                self.data_structure = 'renderProduct'
                self.annotators = [rep.annotators.get('GenericModelOutput')]

            def write(self, data):
                try:
                    for product in data.get('renderProducts', {}).values():
                        raw = product.get('GenericModelOutput')
                        if isinstance(raw, dict):
                            raw = raw.get('data')
                        if raw is None:
                            continue
                        gmo = parse_generic_model_output_data(raw)
                        if not gmo.numElements:
                            continue
                        scan = dict(azimuth_deg=np.array(gmo.x), elevation_deg=np.array(gmo.y),
                                    range_m=np.array(gmo.z), offset_ns=np.array(gmo.timeOffsetNs),
                                    flags=np.array(gmo.flags), timestamp_ns=int(gmo.timestampNs))
                        number = len(records)
                        np.savez_compressed(output / f'scan_{number:04d}.npz', **scan)
                        metrics, _ = analyze_scan(scan, config, moving)
                        records.append(dict(index=number, **metrics, timestamp_ns=int(gmo.timestampNs),
                                            callback_time_s=timeline.get_current_time(), frame_id=int(gmo.frameId),
                                            frame_start_ns=int(gmo.frameStart.timestampNs), frame_end_ns=int(gmo.frameEnd.timestampNs),
                                            frame_start_position=list(gmo.frameStart.posM), frame_end_position=list(gmo.frameEnd.posM),
                                            offset_min_ns=int(scan['offset_ns'].min()), offset_max_ns=int(scan['offset_ns'].max()),
                                            coordinate_type=str(gmo.elementsCoordsType), reference=str(gmo.frameOfReference),
                                            motion_compensation=str(gmo.motionCompensationState)))
                except Exception as error:
                    failures.append(repr(error))

        rep.WriterRegistry.register(ValidationWriter)
        sensor.attach_writer('ValidationWriter')
        if args.gui:
            sensor.attach_writer('draw-point-cloud', size=.05, color=[0, 1, .5, 1])
            from isaacsim.core.utils.viewports import set_camera_view
            set_camera_view(eye=np.array([-8, -18, 12]), target=np.array([10, 1, 0]))
        stage.Export(str(output / 'scene.usda'))
        timeline.play()
        for _ in range(config['frames']):
            app.update()
        timeline.stop()
        sensor.detach_writer('ValidationWriter')
        (output / 'scan-metrics.json').write_text(json.dumps(records, indent=2))
        summary = summarize(records, config, moving, failures)
        (output / 'summary.json').write_text(json.dumps(summary, indent=2))
        manifest['status'] = 'passed' if summary['passed'] else 'failed'
        print('LIDAR_VALIDATION=' + json.dumps(dict(output=str(output), **summary)), flush=True)
    except BaseException as error:
        manifest['status'] = 'failed'
        manifest['error'] = repr(error)
        import traceback
        traceback.print_exc()
        raise
    finally:
        manifest['end_utc'] = datetime.now(timezone.utc).isoformat()
        manifest['wall_seconds'] = time.monotonic() - start
        manifest['output_hashes'] = {str(p.relative_to(output)): digest(p) for p in output.rglob('*') if p.is_file() and p != manifest_path}
        manifest_path.write_text(json.dumps(manifest, indent=2))
        app.close(skip_cleanup=False, exit_code=0 if manifest['status'] == 'passed' else 1)
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
