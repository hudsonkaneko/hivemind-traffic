"""Versioned roof LiDAR source migration; historical source paths stay intact."""
import json
from pathlib import Path
import shutil

ROOF_HEIGHT_M = 1.45


def author_roof_lidar(stage):
    from pxr import Gf, Sdf, UsdGeom
    layer = stage.GetRootLayer()
    root = '/Vehicle/Chassis/Sensors/'
    edits = Sdf.BatchNamespaceEdit()
    for old,new in [('LidarFront','LidarRoof'),('LidarHousing','LidarRoofHousing'),('LidarSupport','LidarRoofSupport')]:
        edits.Add(root+old,root+new)
    if not layer.CanApply(edits) or not layer.Apply(edits):
        raise ValueError('Expected an unmigrated source sensor layer')
    mount = UsdGeom.Xformable(stage.GetPrimAtPath(root+'LidarRoof'))
    mount.GetOrderedXformOps()[0].Set(Gf.Vec3d(0,0,ROOF_HEIGHT_M))
    mount.GetPrim().SetCustomDataByKey('sensorRole','roof_surround_lidar')
    mount.GetPrim().SetCustomDataByKey('runtimeSensorChild','Scan')
    for name,z in [('LidarRoofHousing',1.30),('LidarRoofSupport',1.01)]:
        xform=UsdGeom.Xformable(stage.GetPrimAtPath(root+name))
        xform.GetOrderedXformOps()[0].Set(Gf.Vec3d(0,0,z))
        if name.endswith('Support'):
            xform.GetOrderedXformOps()[1].Set(Gf.Vec3d(.08,.08,.40))


def prepare(source,target):
    from pxr import Usd
    from usd.showcase_vehicle import _inspect_source,package_hashes
    from usd.vehicle_sensor_rig import sensor_paths
    root=Path(__file__).resolve().parents[1]
    source,target=Path(source).resolve(),Path(target).resolve()
    if not source.is_relative_to(root) or not target.is_relative_to(root) or target.exists():
        raise ValueError('Use a new version inside the project')
    model,before=_inspect_source(source)
    shutil.copytree(source,target)
    stage=Usd.Stage.Open(str(target/'assets/vehicle-sensors.usda'))
    author_roof_lidar(stage);stage.GetRootLayer().Save()
    entry=Usd.Stage.Open(str(target/'assets/vehicle.usda'))
    entry.GetDefaultPrim().SetAssetInfoByKey('version',target.name);entry.GetRootLayer().Save()
    model.update(version=target.name,parent_version=source.name)
    model['sensor_rig'].update(version='roof-lidar-v01',paths=sensor_paths(model['chassis_path'],lidar_name='LidarRoof'),
        lidar_translation_m=[0,0,ROOF_HEIGHT_M],visual_supports='Camera supports and LidarRoofSupport; visual only')
    model['roof_lidar']=dict(version='roof-lidar-v01',mount_translation_m=[0,0,ROOF_HEIGHT_M],
        runtime_profile='Example_Rotary / low-speed',source_authored_mount=True,
        runtime_sensor_child='Scan',dynamics_modified=False,
        limitations='Surround coverage must be measured; no claim of full near-field visibility')
    (target/'model.json').write_text(json.dumps(model,indent=2)+'\n',encoding='utf-8')
    _inspect_source(target)
    if package_hashes(source)!=before: raise RuntimeError('Parent source changed')
    return dict(parent_hashes=before,source_hashes=package_hashes(target))
