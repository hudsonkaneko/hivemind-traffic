"""Source-authored sensor interface; no renderer or physics process is started.

Camera optics and mount dimensions are provisional engineering values. Housings
are visual-only: this milestone does not claim changed vehicle mass/inertia.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import shutil

RIG_VERSION = 'sensor-rig-v01'
CAMERA_TRANSLATION = (.8, 0., 1.12)
LIDAR_TRANSLATION = (0., 0., 1.25)
CAMERA_PITCH_DEGREES = 15.
_C, _S = math.cos(math.radians(CAMERA_PITCH_DEGREES)/2), math.sin(math.radians(CAMERA_PITCH_DEGREES)/2)
CAMERA_ORIENTATION_WXYZ = (.5*(_C+_S), .5*(_C-_S), .5*(_S-_C), -.5*(_C+_S))
RESOLUTION = (1280, 720)


def sensor_paths(chassis_path):
    root = str(chassis_path).rstrip('/') + '/Sensors'
    return dict(root=root, camera=root + '/CameraFront',
                lidar_mount=root + '/LidarFront', lidar=root + '/LidarFront/Scan')


def author_sensor_rig(stage, chassis_path):
    """Author under an existing moving chassis in the current edit target."""
    from pxr import Gf, Sdf, UsdGeom
    if not stage.GetPrimAtPath(chassis_path):
        raise ValueError('Sensor rig requires an existing chassis')
    paths = sensor_paths(chassis_path)
    if stage.GetPrimAtPath(paths['root']):
        raise ValueError('Sensor rig already exists')
    root = UsdGeom.Xform.Define(stage, paths['root'])
    root.GetPrim().SetCustomDataByKey('sensorRigVersion', RIG_VERSION)
    root.GetPrim().SetCustomDataByKey('sensorMountAssumptions', 'Provisional roof mounts; visual-only housings')
    camera = UsdGeom.Camera.Define(stage, paths['camera'])
    # Preserve the NVIDIA schema token even in a plain-USD authoring runtime.
    camera.GetPrim().AddAppliedSchema('OmniSensorAPI')
    camera.GetPrim().CreateAttribute('omni:sensor:tickRate', Sdf.ValueTypeNames.Float, custom=False).Set(10.)
    camera.AddTranslateOp().Set(Gf.Vec3d(*CAMERA_TRANSLATION))
    camera.AddOrientOp().Set(Gf.Quatf(CAMERA_ORIENTATION_WXYZ[0], Gf.Vec3f(*CAMERA_ORIENTATION_WXYZ[1:])))
    camera.CreateProjectionAttr().Set('perspective')
    camera.CreateFocalLengthAttr().Set(18.)
    camera.CreateHorizontalApertureAttr().Set(36.)
    camera.CreateVerticalApertureAttr().Set(20.25)
    camera.CreateClippingRangeAttr().Set(Gf.Vec2f(.1, 250.))
    camera.GetPrim().CreateAttribute('sensor:resolution', Sdf.ValueTypeNames.Int2, custom=True).Set(Gf.Vec2i(*RESOLUTION))
    camera.GetPrim().CreateAttribute('sensor:frequencyHz', Sdf.ValueTypeNames.Float, custom=True).Set(10.)
    lidar = UsdGeom.Xform.Define(stage, paths['lidar_mount'])
    lidar.AddTranslateOp().Set(Gf.Vec3d(*LIDAR_TRANSLATION))
    lidar.GetPrim().SetCustomDataByKey('runtimeSensorChild', 'Scan')
    for name, position, scale in (
        ('CameraHousing', (.71, 0., 1.12), (.12, .16, .10)),
        ('LidarHousing', (0., 0., 1.10), (.24, .24, .20)),
        ('CameraSupport', (.71, 0., .94), (.04, .04, .26)),
        ('LidarSupport', (0., 0., .91), (.08, .08, .20)),
    ):
        housing = UsdGeom.Cube.Define(stage, paths['root'] + '/' + name)
        housing.CreateSizeAttr().Set(1.)
        housing.AddTranslateOp().Set(Gf.Vec3d(*position))
        housing.AddScaleOp().Set(Gf.Vec3d(*scale))
        housing.CreateDisplayColorAttr().Set([Gf.Vec3f(.06, .08, .10)])
        housing.GetPrim().SetCustomDataByKey('visualOnlyNoMassOrCollision', True)
    return paths


def prepare_sensor_vehicle(source_directory, target_directory, backup_directory):
    """Create a self-contained new prepared version, preserving exact originals.

    This changes the reusable asset entry, not a simulation/session overlay.
    All destinations must be new directories inside the active checkout.
    """
    from pxr import Sdf, Usd, UsdGeom
    from usd.showcase_vehicle import _inspect_source, package_hashes
    root = Path(__file__).resolve().parents[1]
    source, target, backup = [Path(p).resolve() for p in (source_directory, target_directory, backup_directory)]
    for path in (source, target, backup):
        if not path.is_relative_to(root):
            raise ValueError('Vehicle preparation paths must stay inside checkout')
    if target.exists() or backup.exists():
        raise FileExistsError('New source version and backup must not exist')
    if any(a == b or a.is_relative_to(b) for a, b in ((target, source), (backup, source), (target, backup), (backup, target))):
        raise ValueError('Source, version and backup must be separate')
    model, before = _inspect_source(source)
    shutil.copytree(source, backup)
    shutil.copytree(source, target)
    rig_file = target / 'assets' / 'vehicle-sensors.usda'
    rig = Usd.Stage.CreateNew(str(rig_file))
    vehicle = UsdGeom.Xform.Define(rig, '/Vehicle')
    rig.SetDefaultPrim(vehicle.GetPrim())
    UsdGeom.SetStageMetersPerUnit(rig, 1.)
    UsdGeom.SetStageUpAxis(rig, 'Z')
    rig.SetTimeCodesPerSecond(120.)
    rig.OverridePrim('/Vehicle/Chassis')
    author_sensor_rig(rig, '/Vehicle/Chassis')
    rig.GetRootLayer().Save()
    entry = Sdf.Layer.FindOrOpen(str(target / 'assets' / 'vehicle.usda'))
    entry.subLayerPaths.insert(0, './vehicle-sensors.usda')
    spec = entry.GetPrimAtPath('/Vehicle')
    info = dict(spec.assetInfo)
    info['version'] = target.name
    spec.assetInfo = info
    entry.Save()
    model.update(version=target.name, parent_version=source.name,
                 sensor_rig=dict(version=RIG_VERSION, layer='assets/vehicle-sensors.usda',
                                 paths=sensor_paths(model['chassis_path']),
                                 camera_translation_m=CAMERA_TRANSLATION,
                                 lidar_translation_m=LIDAR_TRANSLATION,
                                 camera_orientation_wxyz=CAMERA_ORIENTATION_WXYZ,
                                 camera_downward_pitch_degrees=CAMERA_PITCH_DEGREES,
                                 camera_resolution=RESOLUTION, camera_frequency_hz=10,
                                 focal_length_mm=18, horizontal_aperture_mm=36,
                                 source_authored=True, dynamics_modified=False,
                                 limitations='Provisional mounts; visual-only housings; runtime sensing must be validated'))
    (target / 'model.json').write_text(json.dumps(model, indent=2) + '\n', encoding='utf-8')
    _inspect_source(target)
    if package_hashes(source) != before or package_hashes(backup) != before:
        raise RuntimeError('Original or backup changed during sensor preparation')
    result = dict(source=str(source), target=str(target), backup=str(backup),
                  source_hashes=before, target_hashes=package_hashes(target),
                  paths=sensor_paths(model['chassis_path']), parent_unchanged=True,
                  runtime_sensing_validated=False)
    return result
