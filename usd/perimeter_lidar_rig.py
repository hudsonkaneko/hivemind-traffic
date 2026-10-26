"""Add two body-attached near-field LiDAR mounts to a new vehicle version."""
import json
from pathlib import Path
import shutil
from usd.near_field_sensor_rig import PROFILE, SENSORS_PATH, inspect_sensor_layout


def lidar_specs():
    return [dict(name=name, translation_m=[0., sign*1.18, .25],
                 housing_center_m=[0., sign*1.11, .25], runtime_profile=PROFILE,
                 runtime_sensor_child='Scan', mount_style='compact body housing')
            for name, sign in [('LidarLeftNear', 1), ('LidarRightNear', -1)]]


def author_perimeter_lidars(stage):
    from pxr import Gf, UsdGeom
    if not stage.GetPrimAtPath(SENSORS_PATH):
        raise ValueError('Missing chassis Sensors hierarchy')
    specs = lidar_specs()
    if any(stage.GetPrimAtPath(SENSORS_PATH+'/'+spec['name']) for spec in specs):
        raise ValueError('Perimeter LiDAR mounts already exist')
    for spec in specs:
        mount = UsdGeom.Xform.Define(stage, SENSORS_PATH+'/'+spec['name'])
        mount.AddTranslateOp().Set(Gf.Vec3d(*spec['translation_m']))
        mount.GetPrim().SetCustomDataByKey('runtimeProfile', PROFILE)
        mount.GetPrim().SetCustomDataByKey('runtimeSensorChild', 'Scan')
        mount.GetPrim().SetCustomDataByKey('sensorRole', 'body_near_field_lidar')
        mount.GetPrim().SetCustomDataByKey('profileConfigurationOwner', 'traffic.physical_lidar.PhysicalLidar')
        housing = UsdGeom.Cube.Define(stage, SENSORS_PATH+'/'+spec['name']+'Housing')
        housing.CreateSizeAttr().Set(1.)
        housing.AddTranslateOp().Set(Gf.Vec3d(*spec['housing_center_m']))
        housing.AddScaleOp().Set(Gf.Vec3d(.12, .12, .10))
        housing.CreateDisplayColorAttr().Set([Gf.Vec3f(.035, .045, .055)])
        housing.GetPrim().SetCustomDataByKey('visualOnlyNoMassOrCollision', True)
    return specs


def prepare(source, target):
    from pxr import Usd
    from usd.showcase_vehicle import _inspect_source, package_hashes
    root = Path(__file__).resolve().parents[1]
    source, target = Path(source).resolve(), Path(target).resolve()
    if (not source.is_relative_to(root) or not target.is_relative_to(root)
            or target.exists() or target.is_relative_to(source) or source.is_relative_to(target)):
        raise ValueError('Use a separate new version inside the project')
    model, before = _inspect_source(source)
    original = inspect_sensor_layout(Usd.Stage.Open(str(source/'assets/vehicle.usda')))
    shutil.copytree(source, target)
    sensor_stage = Usd.Stage.Open(str(target/'assets/vehicle-sensors.usda'))
    specs = author_perimeter_lidars(sensor_stage)
    sensor_stage.GetRootLayer().Save()
    entry = Usd.Stage.Open(str(target/'assets/vehicle.usda'))
    entry.GetDefaultPrim().SetAssetInfoByKey('version', target.name)
    entry.GetRootLayer().Save()
    revised = inspect_sensor_layout(entry)
    if {name: revised.get(name) for name in original} != original:
        raise RuntimeError('Existing sensor geometry or optics changed')
    model.update(version=target.name, parent_version=source.name, perimeter_lidars=specs)
    model['perimeter_lidar_rig'] = dict(version='perimeter-lidar-v01', dynamics_modified=False,
        purpose='Compensate measured roof-only side occlusion',
        limitations='Generic simulated sensors; near-field coverage requires native probe evidence')
    (target/'model.json').write_text(json.dumps(model, indent=2)+'\n', encoding='utf-8')
    _inspect_source(target)
    after = package_hashes(target)
    changed = sorted(name for name in before.keys() | after.keys() if before.get(name) != after.get(name))
    if not set(changed).issubset({'model.json', 'assets/vehicle.usda', 'assets/vehicle-sensors.usda'}):
        raise RuntimeError('Unexpected package changes: '+repr(changed))
    if package_hashes(source) != before:
        raise RuntimeError('Parent source changed')
    return dict(parent_version=source.name, version=target.name,
        entry=(target/'assets/vehicle.usda').relative_to(root).as_posix(),
        parent_hashes=before, source_hashes=after, changed_files=changed,
        perimeter_lidars=specs, existing_sensors_preserved=True, runtime_validation='pending')
