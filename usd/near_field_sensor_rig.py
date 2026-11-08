"""Version a body-camera vehicle with an explicit runtime LiDAR profile contract."""
import json
from pathlib import Path
import shutil

PROFILE = 'roof-near-field-v1'
SENSORS_PATH = '/Vehicle/Chassis/Sensors'


def inspect_sensor_layout(stage):
    """Reject stale/stalk-mounted sources; capture geometry for preservation checks."""
    from pxr import UsdGeom
    from usd.seven_camera_rig import camera_specs
    if stage is None or stage.GetCompositionErrors():
        raise ValueError('Invalid vehicle entry composition')
    sensors = stage.GetPrimAtPath(SENSORS_PATH)
    if not sensors:
        raise ValueError('Missing chassis sensor hierarchy')
    expected = {spec['name'] for spec in camera_specs()}
    cameras = {p.GetName() for p in sensors.GetChildren() if p.IsA(UsdGeom.Camera)}
    if cameras != expected:
        raise ValueError('Expected exactly seven named cameras')
    if any(p.GetName().startswith('Camera') and p.GetName().endswith('Support')
           for p in sensors.GetChildren()):
        raise ValueError('Camera stalks are not permitted in the body-mounted source')
    for name in expected:
        housing = stage.GetPrimAtPath(SENSORS_PATH+'/'+name+'Housing')
        if not housing or housing.GetCustomDataByKey('bodySurfaceAnchor') is None:
            raise ValueError('Missing body-attached camera housing: '+name)
    mount = stage.GetPrimAtPath(SENSORS_PATH+'/LidarRoof')
    if not mount or stage.GetPrimAtPath(SENSORS_PATH+'/LidarFront'):
        raise ValueError('Expected canonical roof LiDAR mount')
    if stage.GetPrimAtPath(SENSORS_PATH+'/LidarRoof/Scan'):
        raise ValueError('Runtime LiDAR must not be published into source')
    return {p.GetName(): {a.GetName(): str(a.Get()) for a in p.GetAttributes()}
            for p in sensors.GetChildren()}


def author_profile_contract(stage):
    """Metadata only: the runtime owns Scan creation, not the published source."""
    mount = stage.GetPrimAtPath(SENSORS_PATH+'/LidarRoof')
    if not mount:
        raise ValueError('Missing roof LiDAR mount')
    mount.SetCustomDataByKey('runtimeProfile', PROFILE)
    mount.SetCustomDataByKey('runtimeSensorChild', 'Scan')
    mount.SetCustomDataByKey('profileConfigurationOwner', 'traffic.physical_lidar.PhysicalLidar')
    mount.SetCustomDataByKey('profileSpecification', '64 uniformly spaced elevation beams -65..10 degrees; 20 Hz scan; 7200 Hz pattern firing')
    mount.SetCustomDataByKey('coverageValidation', 'Requires native probe evidence; not guaranteed by source metadata')


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
    sensors = Usd.Stage.Open(str(target/'assets/vehicle-sensors.usda'))
    author_profile_contract(sensors)
    sensors.GetRootLayer().Save()
    entry = Usd.Stage.Open(str(target/'assets/vehicle.usda'))
    entry.GetDefaultPrim().SetAssetInfoByKey('version', target.name)
    entry.GetRootLayer().Save()
    if inspect_sensor_layout(entry) != original:
        raise RuntimeError('Profile change altered sensor geometry or camera configuration')
    model.update(version=target.name, parent_version=source.name)
    model['roof_lidar'].update(version='near-field-v01', runtime_profile=PROFILE,
        runtime_configuration_owner='traffic.physical_lidar.PhysicalLidar',
        scan_configuration=dict(emitters=64, elevation_min_deg=-65., elevation_max_deg=10.,
            elevation_spacing='uniform inclusive', azimuth_offset_deg=0., scan_hz=20.,
            pattern_firing_hz=7200., hardware_model='generic simulated optics'),
        geometry_modified=False, runtime_sensor_child='Scan',
        limitations='Roof occlusion still requires measured near-field coverage; profile metadata is not validation')
    (target/'model.json').write_text(json.dumps(model, indent=2)+'\n', encoding='utf-8')
    _inspect_source(target)
    after = package_hashes(target)
    allowed = {'model.json', 'assets/vehicle.usda', 'assets/vehicle-sensors.usda'}
    changed = sorted(name for name in before.keys() | after.keys() if before.get(name) != after.get(name))
    if not set(changed).issubset(allowed):
        raise RuntimeError('Unexpected package changes: '+repr(changed))
    if package_hashes(source) != before:
        raise RuntimeError('Parent source changed')
    return dict(parent_version=source.name, version=target.name,
        entry=str((target/'assets/vehicle.usda').relative_to(root)).replace('\\', '/'),
        runtime_profile=PROFILE, parent_hashes=before, source_hashes=after,
        changed_files=changed, cameras_preserved=True, geometry_preserved=True,
        runtime_validation='pending')
