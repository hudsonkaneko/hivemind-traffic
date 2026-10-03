"""Capture full curved traffic replay with per-ray geometry/identity/timing checks."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import sys
import uuid
import platform
import numpy as np
from replay_lidar_geometry import ReplayReference, summarize
from validate_moving_lidar import digest, command

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--replay-dir', type=Path, default=ROOT/'outputs/sumo_replay')
    parser.add_argument('--config', type=Path, default=ROOT/'experiments/configs/replay_lidar_validation.json')
    parser.add_argument('--frames', type=int, help='Diagnostic short run; normal coverage gates still apply')
    parser.add_argument('--identity-deltas', action='store_true', help='Exploratory capture of renderer identity updates')
    parser.add_argument('--zero-azimuth-noise', action='store_true', help='Controlled azimuth-noise ablation, not production validation')
    parser.add_argument('--identity-preflight', action='store_true', help='Diagnostic: register all known replay vehicles before capture')
    parser.add_argument('--uninstance-vehicles', action='store_true', help='Diagnostic session-only renderer instancing ablation')
    args = parser.parse_args()
    if args.frames is not None and args.frames <= 0:
        parser.error('--frames must be positive')
    config = json.loads(args.config.read_text())
    config['identity_deltas'] = args.identity_deltas
    config['zero_azimuth_noise'] = args.zero_azimuth_noise
    config['identity_preflight'] = args.identity_preflight
    config['uninstance_vehicles'] = args.uninstance_vehicles
    config['requested_frames'] = args.frames or round(config['duration_s']*config['fps'])
    data = json.loads((args.replay_dir/'recording.json').read_text())
    reference = ReplayReference(data, config)
    run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:6]
    output = ROOT/'outputs/replay_lidar_validation'/run_id
    output.mkdir(parents=True, exist_ok=False)
    for name in ['recording.json', 'network.net.xml']:
        shutil.copyfile(args.replay_dir/name, output/name)
    if digest(output/'network.net.xml') != data['network_sha256']:
        raise ValueError('Recording/network hash mismatch')
    (output/'resolved-config.json').write_text(json.dumps(config, indent=2))
    (output/'source').mkdir()
    for name in ['validate_replay_lidar.py','replay_lidar_geometry.py','replay_export.py','lidar_geometry.py','validate_moving_lidar.py']:
        shutil.copyfile(ROOT/'scripts'/name, output/'source'/name)
    (output/'working-tree.patch').write_text(command(['git','diff','HEAD']))
    manifest = dict(run_id=run_id,status='running',start_utc=datetime.now(timezone.utc).isoformat(),
                    git_commit=command(['git','rev-parse','HEAD']),git_status=command(['git','status','--porcelain']),
                    command=[sys.executable,*sys.argv],python=sys.version,numpy=np.__version__,os=platform.platform(),
                    gpu=command(['nvidia-smi','--query-gpu=name,driver_version','--format=csv,noheader']),
                    seed=data['seed'],checkpoint_sha256=data['checkpoint_sha256'],recording_sha256=digest(output/'recording.json'),
                    authority='Recorded SUMO poses, rendered kinematically. No physics actuation or policy changes.')
    manifest_path = output/'manifest.json'
    manifest_path.write_text(json.dumps(manifest,indent=2))
    from isaacsim import SimulationApp
    app = SimulationApp({'headless':True,'width':640,'height':480,'enable_motion_bvh':True,
                         'extra_args':['--/rtx-transient/stableIds/enabled=true']})
    records, errors = [], []
    try:
        import omni.usd
        import omni.timeline
        import omni.replicator.core as rep
        from pxr import Usd, UsdGeom
        from isaacsim.sensors.experimental.rtx import (Lidar,LidarSensor,parse_generic_model_output_data,
                                                      parse_stable_id_map_data,parse_object_ids)
        from replay_export import export_replay, vehicle_path
        export_info = export_replay(output)
        manifest['endpoint_checks'] = export_info['checks']
        manifest['usd_version'] = Usd.GetVersion()
        version_file = next((p/'VERSION' for p in Path(sys.executable).parents if (p/'VERSION').is_file()),None)
        manifest['isaac_version'] = version_file.read_text().strip() if version_file else 'unavailable'
        shared = {n:digest(output/n) for n in ['world.usda','motion.usda','road.usda','vehicle.usda']}
        omni.usd.get_context().open_stage(str(output/'world.usda'))
        app.update()
        stage = omni.usd.get_context().get_stage()
        stage.SetEditTarget(stage.GetSessionLayer())
        if config['uninstance_vehicles']:
            for vehicle_id in reference.ids:
                stage.GetPrimAtPath(vehicle_path(vehicle_id)+'/Model').SetInstanceable(False)
        if config['identity_preflight']:
            # Temporary initialization only; never edit the portable asset layers.
            for vehicle_id in reference.ids:
                UsdGeom.Imageable(stage.GetPrimAtPath(vehicle_path(vehicle_id))).GetVisibilityAttr().Set('inherited')
        lidar = Lidar.create(vehicle_path(config['ego_id'])+'/Lidar',config=config['sensor_config'],
                             translations=np.array(config['sensor_mount_m']),aux_output_level='FULL')
        if config['zero_azimuth_noise']:
            attribute = lidar.prims[0].GetAttribute('omni:sensor:Core:azimuthErrorStd')
            if not attribute or not attribute.Set(0.0) or attribute.Get() != 0.0:
                raise RuntimeError('Could not disable azimuth noise in the session layer')
        sensor = LidarSensor(lidar,annotators=[])
        manifest['sensor_attributes'] = {a.GetName():str(a.Get()) for a in lidar.prims[0].GetAttributes() if a.GetName().startswith('omni:sensor')}
        timeline = omni.timeline.get_timeline_interface()
        timeline.set_target_framerate(config['fps'])
        timeline.set_end_time(config['duration_s']+.1)
        timeline.set_looping(False)
        paths = [vehicle_path(v) for v in reference.ids]

        class ReplayValidationWriter(rep.Writer):
            def __init__(self):
                self.data_structure='renderProduct'
                self.annotators=[rep.annotators.get('GenericModelOutput'),rep.annotators.get('StableIdMap')]
                self.identity_map = {}
                self.map_updates = []
                if config['identity_deltas']:
                    self.annotators.append(rep.annotators.get('StableIdMapDeltas'))
            def write(self,payload):
                try:
                    for product in payload.get('renderProducts',{}).values():
                        # Identity updates can arrive between accumulated lidar scans.
                        for map_name in ['StableIdMap', 'StableIdMapDeltas'] if config['identity_deltas'] else ['StableIdMap']:
                            map_raw = product.get(map_name)
                            if isinstance(map_raw, dict): map_raw = map_raw.get('data')
                            if map_raw is None or not np.asarray(map_raw).size: continue
                            entries = parse_stable_id_map_data(map_raw)
                            changed = {k:v for k,v in entries.items() if self.identity_map.get(k) != v}
                            if changed:
                                update_index = len(self.map_updates)
                                np.save(output/f'identity_update_{update_index:04d}.npy', np.asarray(map_raw))
                                self.map_updates.append(dict(source=map_name,callback_time_s=timeline.get_current_time(),
                                    entries={str(k):str(v) for k,v in changed.items()}))
                                self.identity_map.update(changed)
                        raw=product.get('GenericModelOutput')
                        if isinstance(raw,dict): raw=raw.get('data')
                        if raw is None: continue
                        gmo=parse_generic_model_output_data(raw)
                        if not gmo.numElements: continue
                        sid_map = dict(self.identity_map)
                        oid=parse_object_ids(gmo.objId)
                        label_map={key:next((i for i,path in enumerate(paths) if str(value).startswith(path+'/')), -1) for key,value in sid_map.items()}
                        scan=dict(azimuth_deg=np.array(gmo.x),elevation_deg=np.array(gmo.y),range_m=np.array(gmo.z),
                                  flags=np.array(gmo.flags),offset_ns=np.array(gmo.timeOffsetNs),timestamp_ns=int(gmo.timestampNs),
                                  object_id_raw=np.array(gmo.objId),
                                  object_label=np.array([label_map.get(key,-1) for key in oid],dtype=np.int16))
                        number=len(records)
                        np.savez_compressed(output/f'scan_{number:04d}.npz',**scan)
                        metrics,_=reference.analyze(scan)
                        pose_errors=[]
                        for frame in [gmo.frameStart,gmo.frameEnd]:
                            expected,_,_=reference.sensor_pose(np.array([frame.timestampNs*1e-9-config['sensor_to_usd_offset_s']]))
                            pose_errors.append(float(np.linalg.norm(expected[0]-np.array(frame.posM))))
                        format_valid=(str(gmo.elementsCoordsType)=='CoordsType.SPHERICAL' and str(gmo.frameOfReference)=='FrameOfReference.SENSOR'
                                      and str(gmo.motionCompensationState)=='MotionCompensationState.NONCOMPENSATED')
                        time_valid=(scan['offset_ns'].min()>=0 and scan['offset_ns'].max()<=config['scan_period_s']*1e9
                                    and abs(gmo.frameEnd.timestampNs-gmo.timestampNs-config['scan_period_s']*1e9)<=100)
                        records.append(dict(index=number,**metrics,timestamp_ns=int(gmo.timestampNs),
                            pose_error_m=max(pose_errors),format_and_time_valid=bool(format_valid and time_valid),
                            callback_time_s=timeline.get_current_time(),frame_start_ns=int(gmo.frameStart.timestampNs),
                            frame_end_ns=int(gmo.frameEnd.timestampNs),frame_start_position=list(gmo.frameStart.posM),
                            frame_end_position=list(gmo.frameEnd.posM),frame_start_orientation=list(gmo.frameStart.orientation),
                            frame_end_orientation=list(gmo.frameEnd.orientation),stable_id_map={str(k):str(v) for k,v in sid_map.items()}))
                        (output/'identity-updates.json').write_text(json.dumps(self.map_updates,indent=2))
                except Exception as error:
                    errors.append(repr(error))
        rep.WriterRegistry.register(ReplayValidationWriter)
        sensor.attach_writer('ReplayValidationWriter')
        if config['identity_preflight']:
            for _ in range(12):
                app.update()
            for vehicle_id in reference.ids:
                attr = UsdGeom.Imageable(stage.GetPrimAtPath(vehicle_path(vehicle_id))).GetVisibilityAttr()
                attr.Clear()
                for time in reference.times:
                    expected = reference.pose(vehicle_id, np.array([time]))[2][0]
                    if (attr.Get(time*60) != 'invisible') != bool(expected):
                        raise RuntimeError('Preflight did not restore recorded visibility')
            if records:
                raise RuntimeError('Preflight unexpectedly produced scans while paused')
        timeline.play()
        for _ in range(config['requested_frames']):
            app.update()
        timeline.stop()
        sensor.detach_writer('ReplayValidationWriter')
        (output/'scan-metrics.json').write_text(json.dumps(records,indent=2))
        summary=summarize(records,config,errors)
        summary['shared_layers_unchanged']=all(digest(output/n)==h for n,h in shared.items())
        summary['passed']=summary['passed'] and summary['shared_layers_unchanged']
        (output/'summary.json').write_text(json.dumps(summary,indent=2))
        manifest['status']='passed' if summary['passed'] else 'failed'
        print('REPLAY_LIDAR_VALIDATION='+json.dumps(dict(output=str(output),**summary)),flush=True)
    except BaseException as error:
        import traceback
        traceback.print_exc()
        manifest['status']='failed'
        manifest['error']=repr(error)
    finally:
        manifest['end_utc']=datetime.now(timezone.utc).isoformat()
        manifest['output_hashes']={str(p.relative_to(output)):digest(p) for p in output.rglob('*') if p.is_file() and p!=manifest_path}
        manifest_path.write_text(json.dumps(manifest,indent=2))
        app.close(exit_code=0 if manifest['status']=='passed' else 1)
    return 0 if manifest['status']=='passed' else 1


if __name__=='__main__':
    raise SystemExit(main())
