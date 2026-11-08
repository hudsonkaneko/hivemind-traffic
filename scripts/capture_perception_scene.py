"""Capture one bounded labeled scene. No learning or model imports."""
import argparse
import json
from pathlib import Path
import sys
import traceback
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    output=parser.parse_args().output.resolve()
    if Path.cwd().resolve()!=ROOT or not output.is_relative_to(ROOT/'outputs') or (output/'capture-result.json').exists():
        parser.error('Use fresh project output')
    cfg=json.loads((output/'resolved-config.json').read_text())
    scene_cfg=cfg['scene'];source=ROOT/cfg['source_vehicle']
    from usd.showcase_vehicle import package_hashes
    hashes=package_hashes(source)
    app=session=camera=vehicle=view=None;lidars=[];frames=[]
    result=dict(passed=False,training=False)
    try:
        import numpy as np
        from isaacsim import SimulationApp
        app=SimulationApp(dict(headless=True,renderer='RealTimePathTracing',width=640,height=360,
            enable_motion_bvh=True,disable_viewport_updates=False,create_new_stage=False,limit_cpu_threads=8))
        from pxr import Gf,Sdf,Usd,UsdGeom,UsdLux,UsdSemantics
        from PIL import Image,ImageDraw
        from traffic.physics_session import prepare_vehicle_runtime
        from traffic.rendered_physics_session import RenderedPhysicsSession
        from traffic.native_sensor_capture import NativeCamera
        from traffic.physical_lidar import PhysicalLidar
        from traffic.driver_control import AppliedControl
        from traffic.lane_geometry import LaneRoute,RouteSegment
        from traffic.bypass_validation import OrientedBox
        from usd.showcase_vehicle import compose_prepared_vehicle
        from usd.physical_scene import PATHS,author_scene_details
        from visualization.physics_road_view import author_physics_road
        from traffic.perception_dataset import CLASSES
        result['runtime']=prepare_vehicle_runtime()
        rng=np.random.default_rng(scene_cfg['seed'])
        session=RenderedPhysicsSession(120,20,physics_scene_path=PATHS.physics);stage=session.stage
        scene=output/'scene';vehicle=compose_prepared_vehicle(stage,scene,source,120)
        curvature={101:0.,102:.004,103:-.004,104:.006,201:-.007,301:.009}[scene_cfg['seed']]
        view=author_physics_road(stage,LaneRoute('dataset-road',3.6,(RouteSegment(80.,curvature),)),
            clean_layout=True,show_reference_centerline=False,asset_directory=scene/'assets')
        distance=float(rng.uniform(10.,18.))
        author_scene_details(stage,scene,OrientedBox(distance,float(rng.uniform(-.5,.5)),0.,1.,1.))
        UsdGeom.Imageable(stage.GetPrimAtPath('/World/Debug')).CreateVisibilityAttr().Set('invisible')
        def label(path,name):
            UsdSemantics.LabelsAPI.Apply(stage.GetPrimAtPath(path),'class').CreateLabelsAttr().Set([name])
        label('/World/Environment/Highway/Geometry/RoadSurface','drivable')
        for name in ('LeftEdge','RightEdge'):label('/World/Environment/Highway/Geometry/'+name,'lane')
        label(PATHS.barrier,'obstacle')
        moving=None
        scenario=scene_cfg['scenario_id']
        if scenario in ('clear_lane','lead_vehicle'):
            UsdGeom.Imageable(stage.GetPrimAtPath(PATHS.barrier)).CreateVisibilityAttr().Set('invisible')
        if scenario=='lead_vehicle':
            with Usd.EditContext(stage,Sdf.Layer.FindOrOpen(str(scene/'layout.usda'))):
                lead=UsdGeom.Xform.Define(stage,'/World/Vehicles/vehicle_lead')
                lead.GetPrim().GetReferences().AddReference('./assets/vehicle-geometry.usda')
                lead.AddTranslateOp().Set(Gf.Vec3d(distance,0,0))
                label('/World/Vehicles/vehicle_lead','vehicle')
        if scenario=='crossing_target':
            # A box is an obstacle proxy, never a pedestrian label.
            moving=UsdGeom.Xformable(stage.GetPrimAtPath(PATHS.barrier)).GetOrderedXformOps()[0]
        intensity=float(rng.uniform(600,1400))
        UsdLux.DomeLight(stage.GetPrimAtPath('/World/Lighting/Dome')).GetIntensityAttr().Set(intensity)
        Sdf.Layer.FindOrOpen(str(scene/'layout.usda')).Save()
        view.dynamic_layer.Export(str(scene/'debug.usda'));stage.GetRootLayer().Export(str(scene/'world.usda'))
        saved=Sdf.Layer.FindOrOpen(str(scene/'world.usda'));saved.subLayerPaths=['./debug.usda' if p==view.dynamic_layer.identifier else './'+Path(p).name for p in saved.subLayerPaths];saved.Save();saved=None
        from isaacsim.core.utils.viewports import set_camera_view
        path=vehicle.path+'/Sensors/CameraFront'
        # Initialize the viewport using evaluation camera, without changing source camera.
        set_camera_view(eye=np.array([7.,-9.,7.]),target=np.array([0.,0.,1.]),camera_prim_path=PATHS.overview_camera)
        from isaacsim.core.rendering_manager import ViewportManager
        if not ViewportManager.wait_for_viewport(max_frames=120,sleep_time=.02)[0]:raise RuntimeError('Viewport not ready')
        camera=NativeCamera(path,resolution=(640,360),episode_id=scene_cfg['scene_id'],
            calibration_id=vehicle.metadata['multi_camera_rig']['version']+'/CameraFront')
        camera.sensor.attach_annotators(['semantic_segmentation'])
        sensor_specs=[dict(name='LidarRoof',runtime_profile=vehicle.metadata['roof_lidar']['runtime_profile'])]+vehicle.metadata['perimeter_lidars']
        for spec in sensor_specs:
            lidars.append(PhysicalLidar(vehicle.path,episode_id=scene_cfg['scene_id'],vehicle_id=spec['name'],
                tick_source=lambda:session.physics_steps,sensor_path=vehicle.path+'/Sensors/'+spec['name']+'/Scan',
                mount_translation_m=(0,0,0),scan_profile=spec['runtime_profile']))
        vehicle.apply(AppliedControl(0,0,1,'warmup',None,True),700,1500)
        session.start();palette=np.array([[30,35,45],[70,180,90],[250,230,60],[60,130,240],[230,80,180],[240,120,40]],dtype=np.uint8)
        for tick in range(1,241):
            session.step()
            if moving:moving.Set(Gf.Vec3d(distance,-1.5+session.time_s, .75))
            if tick%6==0:session.render()
            if tick%12 or tick<132:continue
            frame,depth=camera.read(session.time_s)
            raw,info=camera.sensor.get_data('semantic_segmentation')
            if raw is None:raise RuntimeError('Missing semantic image')
            raw=raw.numpy().squeeze();mask=np.zeros(raw.shape,dtype=np.uint8)
            unknown=[]
            for key,labels in info['idToLabels'].items():
                names=set(labels.get('class','').split(','));names.discard('')
                if 'car' in names:names.remove('car');names.add('vehicle')
                known=names.intersection(CLASSES)
                if len(known)>1:raise RuntimeError('Ambiguous semantic class '+str(names))
                if known:mask[raw==int(key)]=CLASSES[known.pop()]
                elif names-{'BACKGROUND','UNLABELLED'}:unknown.append(sorted(names))
            if unknown:raise RuntimeError('Unknown labels '+str(unknown))
            stem=f'{len(frames):04d}';rgb_path=stem+'-rgb.png';semantic_path=stem+'-semantic.npy';lidar_path=stem+'-lidar.npz';labels_path=stem+'-labels.json'
            Image.fromarray(frame.rgb).save(output/rgb_path);np.save(output/semantic_path,mask)
            arrays={};timing={}
            for index,lidar in enumerate(lidars):
                packet=lidar.latest
                if lidar.errors or packet is None or not packet['scan_complete']:raise RuntimeError('Invalid LiDAR packet')
                if packet['episode_id']!=scene_cfg['scene_id']:raise RuntimeError('Episode leakage')
                arrays[f'sensor{index}_xyz_world']=packet['xyz'];arrays[f'sensor{index}_flags']=packet['flags'];arrays[f'sensor{index}_offset_ns']=packet['offset_ns']
                timing[lidar.path]=dict(timestamp_ns=packet['timestamp_ns'],delivery_tick=packet['delivery_tick'],
                    frame_end_ns=packet['frame_end_ns'],sensor_position_world_m=np.asarray(packet['sensor_position_world_m']).tolist())
            np.savez_compressed(output/lidar_path,**arrays)
            poses={name:np.asarray(UsdGeom.Xformable(stage.GetPrimAtPath(prim_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).tolist()
                for name,prim_path in [('camera',path),('chassis',vehicle.path)]}
            (output/labels_path).write_text(json.dumps(dict(role='labels_and_evaluation_only',id_to_labels=info['idToLabels'],lidar_timing=timing,
                transforms_at_rgb_capture=poses,transform_convention='USD row-vector local-to-world matrices; camera looks down local -Z with +Y up; world meters Z-up',
                alignment_limit='WORLD NONCOMPENSATED scans; transforms support projection but do not deskew moving objects or guarantee simultaneous acquisition',
                simulation_time_s=session.time_s,epoch_s=session.origin_time_s if hasattr(session,'origin_time_s') else session.snapshot()['origin_time_s']),indent=2))
            frames.append(dict(**scene_cfg,timestamp_s=session.time_s,camera_id='CameraFront',rgb_path=rgb_path,semantic_path=semantic_path,lidar_path=lidar_path,labels_path=labels_path))
            if len(frames)==1:
                panel=Image.new('RGB',(1280,390));panel.paste(Image.fromarray(frame.rgb),(0,30));panel.paste(Image.fromarray(palette[mask]),(640,30));draw=ImageDraw.Draw(panel);draw.text((10,8),scene_cfg['scene_id']+' | RGB',fill='white');draw.text((650,8),'Ground-truth labels (not predictions)',fill='white');panel.save(output/'sample.png')
            vehicle.apply(AppliedControl(0,.12,0,'dataset_scripted_motion',None,False),700,1500)
        result.update(passed=len(frames)==10,frames=frames,source_hashes=hashes,camera_calibration=vehicle.metadata['multi_camera_rig']['cameras'][0],
            lidar_metadata=[d.metadata for d in lidars],scene_parameters=dict(curvature=curvature,obstacle_distance=distance,dome_intensity=intensity),clock=session.snapshot())
    except Exception as error:
        result.update(error=repr(error),traceback=traceback.format_exc());traceback.print_exc()
    finally:
        result['source_unchanged']=package_hashes(source)==hashes
        result['passed']=bool(result.get('passed') and result['source_unchanged'])
        for lidar in lidars:
            try:lidar.close()
            except Exception:result['passed']=False
        if camera:
            try:camera.sensor.detach_annotators(['semantic_segmentation']);camera.close()
            except Exception:result['passed']=False
        lidars.clear();lidar=camera=vehicle=view=stage=lead=moving=None
        if session:
            try:session.close()
            except Exception:result['passed']=False
        (output/'capture-result.json').write_text(json.dumps(result,indent=2))
        if app:app.close()
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
