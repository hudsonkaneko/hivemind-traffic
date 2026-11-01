"""One-process native RTX dataset capture; no learner or policy imports."""
import argparse,json,sys,time,traceback
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args();output=args.output.resolve()
    if Path.cwd().resolve()!=ROOT or not output.is_relative_to(ROOT/'outputs'):raise ValueError('Use project outputs')
    cfg=json.loads((output/'resolved-config.json').read_text());scenes=cfg['scenes'];app=None
    report=dict(passed=False,training=False,frames=[],scene_checks=[])
    try:
        import numpy as np
        from PIL import Image,ImageDraw
        from isaacsim import SimulationApp
        app=SimulationApp(dict(headless=True,renderer='RayTracedLighting',width=640,height=360,create_new_stage=True,limit_cpu_threads=8))
        import omni.usd
        import omni.replicator.core as rep
        from usd.segmentation_dataset_scene import author_assets,setup_stage,set_scene,save_scene
        from traffic.segmentation_dataset_v02 import canonical_mask,sha256
        from pxr import UsdGeom
        stage=omni.usd.get_context().get_stage();assets=author_assets(output/'assets',cfg,scenes,ROOT);layer=setup_stage(stage,output)
        render=rep.create.render_product('/World/Cameras/Front',(640,360))
        annotators={name:rep.AnnotatorRegistry.get_annotator(name) for name in ['rgb','semantic_segmentation','instance_segmentation','distance_to_image_plane']}
        for ann in annotators.values():ann.attach(render)
        palette=np.array([[30,35,45],[70,180,90],[250,230,60],[60,130,240],[230,80,180],[240,120,40]],dtype=np.uint8)
        for scene in scenes:
            folder=output/'frames'/scene['scene_id'];folder.mkdir(parents=True)
            for index in range(scene['frames']):
                metadata=set_scene(stage,layer,assets,scene,index)
                rep.orchestrator.step(rt_subframes=4 if index==0 else 2,delta_time=0.,pause_timeline=True)
                rgb=np.asarray(annotators['rgb'].get_data())[:,:,:3].copy()
                semantic=annotators['semantic_segmentation'].get_data();mask=canonical_mask(semantic['data'],semantic['info']['idToLabels'])
                instance=annotators['instance_segmentation'].get_data();ids=np.asarray(instance['data']).squeeze().copy()
                depth=np.asarray(annotators['distance_to_image_plane'].get_data()).squeeze().copy()
                if rgb.shape!=(360,640,3) or mask.shape!=(360,640) or ids.shape!=(360,640) or depth.shape!=(360,640):raise RuntimeError('Native annotation shape failure')
                stem=f'{index:04d}';paths={}
                for kind,suffix in [('rgb','png'),('semantic','npy'),('instance','npy'),('depth','npy')]:paths[kind]=folder/(stem+'-'+kind+'.'+suffix)
                Image.fromarray(rgb).save(paths['rgb']);np.save(paths['semantic'],mask,allow_pickle=False);np.save(paths['instance'],ids,allow_pickle=False);np.save(paths['depth'],depth,allow_pickle=False)
                camera=UsdGeom.Camera(stage.GetPrimAtPath('/World/Cameras/Front'));focal=float(camera.GetFocalLengthAttr().Get());ha=float(camera.GetHorizontalApertureAttr().Get());va=float(camera.GetVerticalApertureAttr().Get())
                instance_objects={}
                for key,path in instance.get('info',{}).get('idToLabels',{}).items():
                    components=str(path).split('/')
                    instance_objects[key]='/'.join(components[:5]) if '/People/' in str(path) or '/Props/' in str(path) else '/'.join(components[:4]) if '/Vehicles/' in str(path) else str(path)
                metadata.update(scene=scene,frame_index=index,instance_info=instance.get('info',{}),instance_object_roots=instance_objects,semantic_info=semantic.get('info',{}),calibration=dict(width=640,height=360,fx=640*focal/ha,fy=360*focal/va,cx=320,cy=180,focal_length=focal,horizontal_aperture=ha,vertical_aperture=va),data_source='native RTX render products',kinematic_visual_capture=True)
                metadata_path=folder/(stem+'-metadata.json');metadata_path.write_text(json.dumps(metadata,indent=2,default=str))
                row=dict(split=scene['split'],scene_id=scene['scene_id'],group_id=scene['scene_id'],frame_index=index,seed=scene['seed'],road_family=scene['road_family'],obstacle_family=scene['obstacle_family'],human_identity=scene['human_identity'],empty_negative=scene['empty_negative'],class_counts=np.bincount(mask.ravel(),minlength=6).tolist())
                for key,path in paths.items():row[key+'_path']=path.relative_to(output).as_posix();row[key+'_sha256']=sha256(path)
                row['metadata_path']=metadata_path.relative_to(output).as_posix();row['metadata_sha256']=sha256(metadata_path)
                gray=np.asarray(Image.fromarray(rgb).convert('L').resize((8,8)));row['perceptual_hash']=''.join('1' if v>gray.mean() else '0' for v in gray.ravel())
                report['frames'].append(row)
                if index==0:
                    shot=Image.new('RGB',(1280,420),(20,24,32));shot.paste(Image.fromarray(rgb),(0,60));shot.paste(Image.fromarray(palette[mask]),(640,60));draw=ImageDraw.Draw(shot);draw.text((12,12),scene['scene_id']+' / actual RGB + simulator semantic GT / no training',fill='white');draw.text((12,32),'Road green / lane yellow / vehicle blue / real human pink / obstacle orange',fill='white');shot.save(folder/'capture.png')
                    report['scene_checks'].append(dict(scene_id=scene['scene_id'],**save_scene(stage,layer,output,scene['scene_id'])))
            print('CAPTURED '+scene['scene_id']+' '+str(len(report['frames'])),flush=True)
            (output/'capture-progress.json').write_text(json.dumps(dict(completed_frames=len(report['frames']),last_scene=scene['scene_id'])))
        report['passed']=all(not c['composition_errors'] and not c['missing_dependencies'] for c in report['scene_checks'])
        for ann in annotators.values():ann.detach(render)
    except Exception as e:report.update(error=repr(e),traceback=traceback.format_exc());traceback.print_exc()
    finally:
        (output/'capture-result.json').write_text(json.dumps(report,indent=2,default=str))
        if app:app.close()
    return 0 if report['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
