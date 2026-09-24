"""Two stationary block cars: isolate RTX identity mapping across visibility changes."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import platform
import shutil
import sys
import uuid
import numpy as np
from lidar_lifecycle_analysis import analyze, summarize
from validate_moving_lidar import command, digest

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--variant',choices=['always-visible','lifecycle'],default='lifecycle')
    parser.add_argument('--map-source',choices=['sensor','camera'],default='sensor',
                        help='Diagnostic: compare the sensor map with a separate camera render product')
    parser.add_argument('--render-preflight',action='store_true',
                        help='Rejected diagnostic: frozen-time registration; can produce false hidden-target returns')
    args=parser.parse_args()
    if args.render_preflight and args.map_source!='camera':
        parser.error('--render-preflight requires --map-source camera')
    config=json.loads((ROOT/'experiments/configs/lidar_lifecycle.json').read_text())
    config['map_source']=args.map_source
    config['render_preflight']=args.render_preflight
    run_id=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+args.variant+'-'+args.map_source+'-'+uuid.uuid4().hex[:6]
    output=ROOT/'outputs/lidar_lifecycle'/run_id
    output.mkdir(parents=True,exist_ok=False)
    (output/'source').mkdir()
    for name in ['validate_lidar_lifecycle.py','lidar_lifecycle_analysis.py','lidar_geometry.py','validate_moving_lidar.py']:
        shutil.copyfile(ROOT/'scripts'/name,output/'source'/name)
    (output/'resolved-config.json').write_text(json.dumps(config,indent=2))
    (output/'working-tree.patch').write_text(command(['git','diff','HEAD']))
    manifest=dict(run_id=run_id,status='running',variant=args.variant,command=[sys.executable,*sys.argv],
        start_utc=datetime.now(timezone.utc).isoformat(),git_commit=command(['git','rev-parse','HEAD']),
        git_status=command(['git','status','--porcelain']),python=sys.version,numpy=np.__version__,os=platform.platform(),
        gpu=command(['nvidia-smi','--query-gpu=name,driver_version','--format=csv,noheader']),
        checkpoint=None,seed=None,reason='Analytic fixed geometry; no training or random placement. Renderer noise is not claimed bitwise deterministic.',
        authority='Standalone kinematic USD fixture; no SUMO, PPO, replay exporter, physics, or vehicle control.')
    manifest_path=output/'manifest.json'
    manifest_path.write_text(json.dumps(manifest,indent=2))
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'width':640,'height':480,'enable_motion_bvh':True,
                       'extra_args':['--/rtx-transient/stableIds/enabled=true']})
    rows,errors=[],[]
    try:
        import omni.usd
        import omni.timeline
        import omni.replicator.core as rep
        from pxr import Usd, UsdGeom, Gf
        from isaacsim.sensors.experimental.rtx import Lidar,LidarSensor,parse_generic_model_output_data,parse_stable_id_map_data,parse_object_ids
        stage=omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage,1)
        stage.SetTimeCodesPerSecond(config['fps'])
        stage.SetFramesPerSecond(config['fps'])
        for name,center,size in [('Ego',[-2,0,.3],[4,2,.6]),('Target',config['target_center_m'],config['target_size_m'])]:
            UsdGeom.Xform.Define(stage,'/World/'+name)
            body=UsdGeom.Cube.Define(stage,'/World/'+name+'/Body')
            body.CreateSizeAttr(1)
            body.AddTranslateOp().Set(Gf.Vec3d(*center))
            body.AddScaleOp().Set(Gf.Vec3f(*size))
        vis=UsdGeom.Imageable(stage.GetPrimAtPath('/World/Target')).CreateVisibilityAttr()
        for time,visible in [(0,args.variant=='always-visible'),(1,True),(2,args.variant=='always-visible'),(3,True)]:
            vis.Set('inherited' if visible else 'invisible',time*config['fps'])
        lidar=Lidar.create('/World/Ego/Lidar',config=config['sensor_config'],translations=np.array(config['sensor_mount_m']),aux_output_level='FULL')
        sensor=LidarSensor(lidar,annotators=[])
        # Keep both independent renderer maps. Never infer labels from geometry.
        camera_mapping={}
        camera_updates=[]
        camera_annotators=[]
        if args.map_source=='camera':
            camera=rep.create.camera(position=(0,0,5),look_at=(12,0,1))
            camera_product=rep.create.render_product(camera,(64,64))
            for name in ['StableIdMap','StableIdMapDeltas']:
                annotator=rep.annotators.get(name)
                annotator.attach(camera_product)
                camera_annotators.append((name,annotator))
        manifest['sensor_attributes']={a.GetName():str(a.Get()) for a in lidar.prims[0].GetAttributes() if a.GetName().startswith('omni:sensor')}
        manifest['usd_version']=Usd.GetVersion()
        version=next((p/'VERSION' for p in Path(sys.executable).parents if (p/'VERSION').is_file()),None)
        manifest['isaac_version']=version.read_text().strip() if version else 'unavailable'
        timeline=omni.timeline.get_timeline_interface()
        timeline.set_target_framerate(config['fps'])
        timeline.set_end_time(config['frames']/config['fps']+.1)
        timeline.set_looping(False)
        if args.render_preflight:
            # Author a temporary stronger visibility opinion, then remove it.
            with Usd.EditContext(stage,stage.GetSessionLayer()):
                override=UsdGeom.Imageable(stage.GetPrimAtPath('/World/Target')).GetVisibilityAttr()
                override.Set('inherited')
                for frame in range(12):
                    rep.orchestrator.step(delta_time=0.0,pause_timeline=True)
                    for name,annotator in camera_annotators:
                        raw=annotator.get_data()
                        if isinstance(raw,dict): raw=raw.get('data')
                        if raw is not None and np.asarray(raw).size:
                            entries=parse_stable_id_map_data(raw)
                            camera_mapping.update(entries)
                            camera_updates.append(dict(preflight_frame=frame,source=name,
                                time_s=timeline.get_current_time(),entries={str(k):str(v) for k,v in entries.items()}))
                override.Clear()
            timeline.stop()
            timeline.set_current_time(0.)
            if abs(timeline.get_current_time())>1e-9:
                raise RuntimeError('Preflight failed to restore initial time')
            for time,visible in [(0,args.variant=='always-visible'),(1,True),(2,args.variant=='always-visible'),(3,True)]:
                if (vis.Get(time*config['fps'])!='invisible')!=visible:
                    raise RuntimeError('Preflight failed to restore visibility')
            manifest['preflight_target_registered']='/World/Target/Body' in camera_mapping.values()

        class LifecycleWriter(rep.Writer):
            def __init__(self):
                self.data_structure='renderProduct'
                self.annotators=[rep.annotators.get(n) for n in ['GenericModelOutput','StableIdMap','StableIdMapDeltas']]
                self.mapping={}

            def write(self,payload):
                try:
                    for product in payload.get('renderProducts',{}).values():
                        maps={}
                        for name in ['StableIdMap','StableIdMapDeltas']:
                            raw=product.get(name)
                            if isinstance(raw,dict): raw=raw.get('data')
                            if raw is not None and np.asarray(raw).size:
                                entries=parse_stable_id_map_data(raw)
                                self.mapping.update(entries)
                                maps[name]={str(k):str(v) for k,v in entries.items()}
                        raw=product.get('GenericModelOutput')
                        if isinstance(raw,dict): raw=raw.get('data')
                        if raw is None: continue
                        gmo=parse_generic_model_output_data(raw)
                        if not gmo.numElements: continue
                        ids=parse_object_ids(gmo.objId)
                        selected_map=camera_mapping if args.map_source=='camera' else self.mapping
                        scan=dict(timestamp_ns=int(gmo.timestampNs),azimuth_deg=np.array(gmo.x),elevation_deg=np.array(gmo.y),
                            range_m=np.array(gmo.z),flags=np.array(gmo.flags),offset_ns=np.array(gmo.timeOffsetNs),
                            object_id_raw=np.array(gmo.objId),target_label=np.array([selected_map.get(k)=='/World/Target/Body' for k in ids]),
                            sensor_target_label=np.array([self.mapping.get(k)=='/World/Target/Body' for k in ids]),
                            emitter_id=np.array(gmo.emitterId),echo_id=np.array(gmo.echoId),channel_id=np.array(gmo.channelId))
                        index=len(rows)
                        np.savez_compressed(output/f'scan_{index:04d}.npz',**scan)
                        if args.map_source=='camera':
                            maps['CameraStableIdMap']={str(k):str(v) for k,v in camera_mapping.items()}
                        rows.append(dict(index=index,**analyze(scan,config,args.variant),maps=maps,
                            callback_time_s=timeline.get_current_time(),frame_start_ns=int(gmo.frameStart.timestampNs),
                            frame_end_ns=int(gmo.frameEnd.timestampNs),frame_start_position=list(gmo.frameStart.posM),
                            frame_end_position=list(gmo.frameEnd.posM),coordinate_type=str(gmo.elementsCoordsType),
                            reference=str(gmo.frameOfReference),motion_compensation=str(gmo.motionCompensationState)))
                except Exception as error:
                    errors.append(repr(error))
        rep.WriterRegistry.register(LifecycleWriter)
        sensor.attach_writer('LifecycleWriter')
        stage.Export(str(output/'scene.usda'))
        timeline.play()
        for frame in range(config['frames']):
            app.update()
            for name,annotator in camera_annotators:
                raw=annotator.get_data()
                if isinstance(raw,dict): raw=raw.get('data')
                if raw is not None and np.asarray(raw).size:
                    entries=parse_stable_id_map_data(raw)
                    changed={k:v for k,v in entries.items() if camera_mapping.get(k)!=v}
                    if changed:
                        camera_updates.append(dict(frame=frame,source=name,time_s=timeline.get_current_time(),
                            entries={str(k):str(v) for k,v in changed.items()}))
                        camera_mapping.update(changed)
        timeline.stop()
        sensor.detach_writer('LifecycleWriter')
        for _,annotator in camera_annotators:
            annotator.detach(camera_product)
        (output/'camera-map-updates.json').write_text(json.dumps(camera_updates,indent=2))
        result=summarize(rows,config,args.variant,errors)
        (output/'scan-metrics.json').write_text(json.dumps(rows,indent=2))
        (output/'summary.json').write_text(json.dumps(result,indent=2))
        manifest['status']='passed' if result['passed'] else 'failed'
        print('LIDAR_LIFECYCLE='+json.dumps(dict(output=str(output),**result)),flush=True)
    except BaseException as error:
        import traceback
        traceback.print_exc()
        manifest.update(status='failed',error=repr(error))
    finally:
        manifest['end_utc']=datetime.now(timezone.utc).isoformat()
        manifest['output_hashes']={str(p.relative_to(output)):digest(p) for p in output.rglob('*') if p.is_file() and p!=manifest_path}
        manifest_path.write_text(json.dumps(manifest,indent=2))
        app.close(exit_code=0 if manifest['status']=='passed' else 1)


if __name__=='__main__':
    main()
