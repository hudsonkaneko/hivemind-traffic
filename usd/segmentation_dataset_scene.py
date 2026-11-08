"""Composed image-dataset scenes: reusable assets, layout and transient opinions."""
from pathlib import Path
import math,shutil,os

def author_assets(directory,config,scenes,root):
    from pxr import Usd,UsdGeom,UsdShade,Gf,Sdf,UsdSemantics
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    shutil.copytree(root/config['vehicle_visual'].rsplit('/',1)[0],directory/'vehicle',dirs_exist_ok=False)
    humans=directory/'humans';humans.mkdir()
    for name in sorted(set(s['human_identity'] for s in scenes)):
        source=root/config['human_asset_root']/name;target=humans/name;target.mkdir()
        shutil.copyfile(source/(name+'.usd'),target/(name+'.usd'));shutil.copytree(source/'textures',target/'textures')
    def stage_file(name):
        stage=Usd.Stage.CreateNew(str(directory/name));UsdGeom.SetStageMetersPerUnit(stage,1);UsdGeom.SetStageUpAxis(stage,'Z')
        prim=UsdGeom.Xform.Define(stage,'/Asset').GetPrim();stage.SetDefaultPrim(prim);Usd.ModelAPI(prim).SetKind('component');return stage
    def label(prim,name):UsdSemantics.LabelsAPI.Apply(prim,'class').CreateLabelsAttr().Set([name])
    def material(stage,name,color):
        mat=UsdShade.Material.Define(stage,'/Asset/Looks/'+name);shader=UsdShade.Shader.Define(stage,'/Asset/Looks/'+name+'/Surface')
        shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color));shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.85)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface');return mat
    def strip(stage,path,points,color,semantic):
        mesh=UsdGeom.Mesh.Define(stage,path);mesh.CreatePointsAttr(points);mesh.CreateFaceVertexCountsAttr([4]*(len(points)//4));mesh.CreateFaceVertexIndicesAttr(list(range(len(points))))
        mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDoubleSidedAttr(True);label(mesh.GetPrim(),semantic);UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material(stage,path.rsplit('/',1)[1],color))
    for scene in scenes:
        stage=stage_file('road-'+scene['scene_id']+'.usda');w=scene['road_width'];road=[];left=[];right=[];middle=[]
        def center(x):
            if scene['road_family']=='constant_curvature':return scene['curvature']*x*x/2
            return (2.5 if scene['road_family']=='slow_s_bend' else 4.0)*math.sin(x/(32 if scene['road_family']=='slow_s_bend' else 21))
        for x in range(-10,100):
            a,b=center(x),center(x+1)
            road.extend([(x,a-w/2,.01),(x+1,b-w/2,.01),(x+1,b+w/2,.01),(x,a+w/2,.01)])
            for out,offset in [(left,-w/2+.12),(right,w/2-.12)]:out.extend([(x,a+offset-.08,.02),(x+1,b+offset-.08,.02),(x+1,b+offset+.08,.02),(x,a+offset+.08,.02)])
            if x%6<3:middle.extend([(x,a-.06,.023),(x+1,b-.06,.023),(x+1,b+.06,.023),(x,a+.06,.023)])
        strip(stage,'/Asset/RoadSurface',road,scene['road_color'],'road')
        for name,points in [('LeftEdge',left),('RightEdge',right),('CenterDashes',middle)]:strip(stage,'/Asset/'+name,points,(.8,.77,.55),'lane')
        strip(stage,'/Asset/Ground',[(-20,-90,-.01),(110,-90,-.01),(110,90,-.01),(-20,90,-.01)],scene['ground_color'],'background');stage.GetRootLayer().Save()
    for name,kind in [('box','cube'),('cone','cone'),('cylinder','cylinder'),('barrier','barrier'),('pyramid','pyramid'),('lowcylinder','lowcylinder')]:
        stage=stage_file('obstacle-'+name+'.usda')
        if kind in ('cube','barrier'):
            mesh=UsdGeom.Cube.Define(stage,'/Asset/Geometry');mesh.CreateSizeAttr(1);mesh.AddTranslateOp().Set(Gf.Vec3d(0,0,.6));mesh.AddScaleOp().Set(Gf.Vec3f(1.2,1.2 if kind=='cube' else 2.5,1.2))
        elif kind in ('cone','cylinder','lowcylinder'):
            cls=UsdGeom.Cone if kind=='cone' else UsdGeom.Cylinder;mesh=cls.Define(stage,'/Asset/Geometry');mesh.CreateRadiusAttr(.4 if kind=='cone' else .6);mesh.CreateHeightAttr(1.2 if kind!='lowcylinder' else .35);mesh.AddTranslateOp().Set(Gf.Vec3d(0,0,.6 if kind!='lowcylinder' else .175))
        else:
            mesh=UsdGeom.Mesh.Define(stage,'/Asset/Geometry');mesh.CreatePointsAttr([(-.6,-.6,0),(.6,-.6,0),(.6,.6,0),(-.6,.6,0),(0,0,1.5)]);mesh.CreateFaceVertexCountsAttr([3,3,3,3,4]);mesh.CreateFaceVertexIndicesAttr([0,1,4,1,2,4,2,3,4,3,0,4,0,3,2,1]);mesh.CreateSubdivisionSchemeAttr('none')
        label(mesh.GetPrim(),'obstacle');UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material(stage,'Paint',(.9,.25,.04)));stage.GetRootLayer().Save()
    return directory

def setup_stage(stage,directory):
    from pxr import Usd,UsdGeom,UsdLux,Sdf,Gf
    directory=Path(directory);stage.SetDefaultPrim(UsdGeom.Xform.Define(stage,'/World').GetPrim());UsdGeom.SetStageMetersPerUnit(stage,1);UsdGeom.SetStageUpAxis(stage,'Z')
    Usd.ModelAPI(stage.GetDefaultPrim()).SetKind('assembly')
    for path in ['/World/Environment','/World/Environment/Props','/World/Environment/People','/World/Vehicles']:
        Usd.ModelAPI(UsdGeom.Xform.Define(stage,path).GetPrim()).SetKind('group')
    for path in ['/World/Cameras','/World/Lighting']:UsdGeom.Scope.Define(stage,path)
    camera=UsdGeom.Camera.Define(stage,'/World/Cameras/Front');camera.CreateFocalLengthAttr(10.39230442);camera.CreateHorizontalApertureAttr(36);camera.CreateVerticalApertureAttr(20.25);camera.CreateClippingRangeAttr(Gf.Vec2f(.1,250))
    UsdLux.DomeLight.Define(stage,'/World/Lighting/Dome').CreateIntensityAttr(500)
    sun=UsdLux.DistantLight.Define(stage,'/World/Lighting/Sun');sun.CreateIntensityAttr(1500);sun.CreateAngleAttr(.5)
    layer=Sdf.Layer.CreateAnonymous('dataset-transient');stage.GetRootLayer().subLayerPaths.insert(0,layer.identifier)
    return layer

def set_scene(stage,layer,assets,scene,frame_index):
    from pxr import Usd,UsdGeom,UsdLux,Sdf,Gf,UsdSemantics
    assets=Path(assets);rng=__import__('numpy').random.default_rng(scene['seed']);x=.09*frame_index
    with Usd.EditContext(stage,layer):
        # Namespace stable, references and poses remain runtime-owned opinions.
        road=UsdGeom.Xform.Define(stage,'/World/Environment/Highway');road.GetPrim().GetReferences().SetReferences([Sdf.Reference(str(assets/('road-'+scene['scene_id']+'.usda')))])
        camera=UsdGeom.Xformable(stage.GetPrimAtPath('/World/Cameras/Front'));eye=Gf.Vec3d(x,scene['camera_lateral'],scene['camera_height']);pitch=math.radians(scene['camera_pitch']);target=eye+Gf.Vec3d(20,0,-20*math.tan(pitch));camera.MakeMatrixXform().Set(Gf.Matrix4d().SetLookAt(eye,target,Gf.Vec3d(0,0,1)).GetInverse())
        sun=UsdLux.DistantLight(stage.GetPrimAtPath('/World/Lighting/Sun'));sun.GetIntensityAttr().Set(scene['sunlight']);UsdGeom.Xformable(sun.GetPrim()).MakeMatrixXform().Set(Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0,1,0),-scene['sun_elevation'])))
        def place(path,ref,position,yaw=0,semantic=None):
            obj=UsdGeom.Xform.Define(stage,path);obj.GetPrim().GetReferences().SetReferences([Sdf.Reference(str(ref))]);Usd.ModelAPI(obj.GetPrim()).SetKind('component')
            transform=Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0,0,1),yaw));transform.SetTranslateOnly(Gf.Vec3d(*position));obj.MakeMatrixXform().Set(transform)
            UsdGeom.Imageable(obj).CreateVisibilityAttr().Set('invisible' if scene['empty_negative'] else 'inherited')
            if semantic:UsdSemantics.LabelsAPI.Apply(obj.GetPrim(),'class').CreateLabelsAttr().Set([semantic])
        place('/World/Vehicles/vehicle_lead',assets/'vehicle/vehicle-geometry.usda',(scene['lead_distance']+.04*frame_index,-1.6,0),0,'vehicle')
        place('/World/Vehicles/vehicle_adjacent',assets/'vehicle/vehicle-geometry.usda',(scene['lead_distance']+9-.025*frame_index,2,0),5,'vehicle')
        names=scene['obstacle_family'].split('_and_')
        for i,name in enumerate(names):place('/World/Environment/Props/Obstacle_'+str(i+1),assets/('obstacle-'+name+'.usda'),(scene['obstacle_distance']+i*8,3.2 if i==0 else -3.6,0),float(rng.uniform(-30,30)),'obstacle')
        for source in (assets/'humans').iterdir():
            name=source.name;path='/World/Environment/People/Pedestrian_'+name
            place(path,source/(name+'.usd'),(scene['pedestrian_distance'],-2.8+.015*frame_index,.16),-90,'pedestrian')
            human=stage.GetPrimAtPath(path)
            UsdGeom.Imageable(human).GetVisibilityAttr().Set('inherited' if name==scene['human_identity'] and not scene['empty_negative'] else 'invisible')
            for prim in Usd.PrimRange(human):
                if prim.IsA(UsdGeom.Mesh):UsdSemantics.LabelsAPI.Apply(prim,'class').CreateLabelsAttr().Set(['pedestrian'])
    return dict(camera_to_world=[list(row) for row in camera.GetLocalTransformation()],motion_authority='dataset_scripted_visual_only',physics=False)

def save_scene(stage,layer,output,scene_id):
    from pxr import Usd,Sdf,UsdUtils,UsdGeom
    output=Path(output);directory=output/'scenes'/scene_id;directory.mkdir(parents=True,exist_ok=True)
    target=directory/'layout.usda';layer.Export(str(target))
    saved=Sdf.Layer.FindOrOpen(str(target));text=saved.ExportToString();text=text.replace((output/'assets').as_posix(),os.path.relpath(output/'assets',directory).replace('\\','/'));text=text.replace(str(output/'assets').replace('\\','/'),os.path.relpath(output/'assets',directory).replace('\\','/'));saved.ImportFromString(text);saved.Save()
    world=directory/'world.usda';stage.GetRootLayer().Export(str(world));root=Sdf.Layer.FindOrOpen(str(world));root.subLayerPaths=['./layout.usda'];root.Save()
    reopened=Usd.Stage.Open(str(world));layers,assets,missing=UsdUtils.ComputeAllDependencies(str(world));
    # OmniPBR.mdl is supplied by Isaac's material resolver, not a missing dataset texture.
    missing=[str(x) for x in missing if str(x)!='OmniPBR.mdl']
    return dict(default_prim=str(reopened.GetDefaultPrim().GetPath()),units=UsdGeom.GetStageMetersPerUnit(reopened),up_axis=str(UsdGeom.GetStageUpAxis(reopened)),composition_errors=[str(x) for x in reopened.GetCompositionErrors()],missing_dependencies=missing,layer_count=len(layers),
                prim_paths=[str(p.GetPath()) for p in reopened.Traverse() if p.GetPath().pathElementCount<=3],runtime_namespace_exceptions=['/Render','/Replicator'],physics=False)
