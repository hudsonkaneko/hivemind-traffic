"""Reusable road asset composition; USD checks need pxr, never a renderer."""
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

from traffic.lane_geometry import load_routes
from visualization.physics_road_view import (
    CLEAN_DEBUG_ROOT, CLEAN_ROAD_ROOT, PhysicsRoadView, author_physics_road,
)


@pytest.fixture
def route():
    return load_routes(Path(__file__).parents[1] / 'scenarios/physics-road/routes.json')['left_r60']


@pytest.fixture
def assets(tmp_path):
    directory = tmp_path / 'scene' / 'assets'
    directory.mkdir(parents=True)
    return directory


def _stage():
    pytest.importorskip('pxr.Usd')
    from pxr import Kind, Usd, UsdGeom
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    stage.SetTimeCodesPerSecond(60)
    world = UsdGeom.Xform.Define(stage, '/World').GetPrim()
    Usd.ModelAPI(world).SetKind(Kind.Tokens.assembly)
    stage.SetDefaultPrim(world)
    return stage


def test_clean_options_fail_before_requiring_usd(assets):
    with pytest.raises(ValueError, match='existing asset_directory'):
        author_physics_road(None, None, clean_layout=True)
    with pytest.raises(ValueError, match='existing directory'):
        author_physics_road(None, None, clean_layout=True, asset_directory=assets / 'missing')
    with pytest.raises(ValueError, match='road root'):
        author_physics_road(None, None, root_path='/Custom', clean_layout=True, asset_directory=assets)
    with pytest.raises(ValueError, match='only supported'):
        author_physics_road(None, None, asset_directory=assets)
    assert list(assets.iterdir()) == []


def test_view_keeps_legacy_properties_and_exposes_clean_paths_without_usd(route, assets):
    layer = SimpleNamespace(identifier='test-layer')
    legacy = PhysicsRoadView(None, route, '/World/PhysicsRoad', layer, layer)
    assert legacy.root_path == '/World/PhysicsRoad'
    assert legacy.debug_root_path == '/World/PhysicsRoad/Debug'
    assert not legacy.clean_layout
    clean = PhysicsRoadView(None, route, CLEAN_ROAD_ROOT, layer, layer,
                            debug_root_path=CLEAN_DEBUG_ROOT, asset_path=assets / 'road.usda',
                            assembly_path=assets.parent / 'road-layout.usda')
    assert clean.static_layer is layer
    assert clean.dynamic_layer is layer
    assert clean.clean_layout
    assert clean.metadata['road_asset_reference'] == 'assets/road.usda'
    assert clean.metadata['debug_root_path'] == CLEAN_DEBUG_ROOT
    assert clean.camera_pose('overview') == legacy.camera_pose('overview')


def test_asset_is_self_contained_and_referenced_with_clean_hierarchy(route, assets):
    stage = _stage()
    from pxr import Kind, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
    original_target = stage.GetEditTarget().GetLayer()
    view = author_physics_road(stage, route, clean_layout=True, asset_directory=assets,
                               lane_dividers_m=(0.0,), show_reference_centerline=False)
    assert view.root_path == CLEAN_ROAD_ROOT
    assert view.debug_root_path == CLEAN_DEBUG_ROOT
    assert not view.static_layer.anonymous
    assert view.dynamic_layer.anonymous
    assert stage.GetEditTarget().GetLayer() == original_target
    assert not stage.GetCompositionErrors()
    assert not stage.GetPrimAtPath('/World/PhysicsRoad')
    assert not stage.GetPrimAtPath(CLEAN_ROAD_ROOT + '/Debug')
    assert Usd.ModelAPI(stage.GetPrimAtPath('/World/Environment')).GetKind() == Kind.Tokens.group
    assert Usd.ModelAPI(stage.GetPrimAtPath(CLEAN_ROAD_ROOT)).GetKind() == Kind.Tokens.component
    assert stage.GetPrimAtPath(CLEAN_ROAD_ROOT).IsModel()
    reference = view.static_layer.GetPrimAtPath(CLEAN_ROAD_ROOT).referenceList.prependedItems
    assert len(reference) == 1
    assert reference[0].assetPath == 'assets/road.usda'
    assert reference[0].primPath == Sdf.Path.emptyPath
    assert view.static_layer.GetPrimAtPath(CLEAN_ROAD_ROOT + '/Geometry') is None
    assert view.static_layer.GetPrimAtPath(CLEAN_DEBUG_ROOT) is None

    asset = Usd.Stage.Open(str(view.asset_path))
    assert str(asset.GetDefaultPrim().GetPath()) == '/Highway'
    assert [str(prim.GetPath()) for prim in asset.GetPseudoRoot().GetChildren()] == ['/Highway']
    assert UsdGeom.GetStageMetersPerUnit(asset) == 1.0
    assert UsdGeom.GetStageUpAxis(asset) == UsdGeom.Tokens.z
    assert asset.GetTimeCodesPerSecond() == 60
    assert asset.GetDefaultPrim().GetAssetInfo()['identifier'].path == 'road.usda'
    assert not asset.GetCompositionErrors()
    assert asset.GetPrimAtPath('/Highway/Geometry/LaneDivider_001')
    for prim in stage.Traverse():
        assert not prim.HasAPI(UsdPhysics.CollisionAPI)
        assert not prim.HasAPI(UsdPhysics.RigidBodyAPI)
        assert not prim.HasAPI(UsdPhysics.MassAPI)
        if prim.IsA(UsdGeom.Mesh):
            targets = UsdShade.MaterialBindingAPI(prim).GetDirectBindingRel().GetTargets()
            owner = CLEAN_ROAD_ROOT if prim.GetPath().HasPrefix(CLEAN_ROAD_ROOT) else CLEAN_DEBUG_ROOT
            assert len(targets) == 1
            assert targets[0].HasPrefix(owner + '/Looks')
            assert stage.GetPrimAtPath(targets[0])
    for name in ('ReferenceCenterline', 'UpcomingPath', 'PursuitTarget'):
        path = CLEAN_DEBUG_ROOT + '/' + name
        assert view.dynamic_layer.GetPrimAtPath(path)
        mesh = UsdGeom.Mesh(stage.GetPrimAtPath(path))
        assert mesh.GetPurposeAttr().Get() == 'default'
        assert max(point[2] for point in mesh.GetPointsAttr().Get()) <= .1
    reference_mesh = UsdGeom.Imageable(stage.GetPrimAtPath(CLEAN_DEBUG_ROOT + '/ReferenceCenterline'))
    assert reference_mesh.GetVisibilityAttr().Get() == 'invisible'


def test_updates_touch_only_debug_and_keep_published_files_unchanged(route, assets):
    stage = _stage()
    from pxr import Gf, UsdGeom
    view = author_physics_road(stage, route, clean_layout=True, asset_directory=assets)
    asset_before = view.asset_path.read_bytes()
    assembly_before = view.assembly_path.read_bytes()
    static_before = view.static_layer.ExportToString()
    root_before = stage.GetRootLayer().ExportToString()
    debug_before = view.dynamic_layer.ExportToString()
    view.update(dict(position_m=[10, 0, 1]), target_xy=(15, 2), full_plan=True)
    view.update(dict(position_m=[20, 0, 1]), target_xy=(25, 3), full_plan=True)
    target = UsdGeom.Xformable(stage.GetPrimAtPath(CLEAN_DEBUG_ROOT + '/PursuitTarget'))
    assert target.ComputeLocalToWorldTransform(0).ExtractTranslation() == Gf.Vec3d(25, 3, 0)
    view.update(dict(position_m=[20, 0, 1]), show_reference=False)
    assert UsdGeom.Imageable(stage.GetPrimAtPath(CLEAN_DEBUG_ROOT)).GetVisibilityAttr().Get() == 'invisible'
    assert view.dynamic_layer.ExportToString() != debug_before
    assert view.static_layer.ExportToString() == static_before
    assert stage.GetRootLayer().ExportToString() == root_before
    assert view.asset_path.read_bytes() == asset_before
    assert view.assembly_path.read_bytes() == assembly_before


def test_saved_composition_reopens_in_fresh_process_from_another_directory(route, assets, tmp_path):
    stage = _stage()
    from pxr import Sdf, Usd, UsdGeom
    view = author_physics_road(stage, route, clean_layout=True, asset_directory=assets)
    debug_file = assets.parent / 'road-debug.usda'
    assert view.dynamic_layer.Export(str(debug_file))
    # The packaging caller exports the anonymous debug layer and rebinds its
    # root sublayer paths, preserving the separate road reference/asset files.
    packaged_root = Sdf.Layer.CreateAnonymous('scene.usda')
    packaged_root.TransferContent(stage.GetRootLayer())
    packaged_root.subLayerPaths = ['road-debug.usda', 'road-layout.usda']
    scene_path = assets.parent / 'scene.usda'
    assert packaged_root.Export(str(scene_path))
    reopened = Usd.Stage.Open(str(scene_path))
    assert not reopened.GetCompositionErrors()
    assert reopened.GetPrimAtPath(CLEAN_ROAD_ROOT).HasAuthoredReferences()
    assert reopened.GetPrimAtPath(CLEAN_DEBUG_ROOT + '/UpcomingPath')
    mesh = UsdGeom.Mesh(reopened.GetPrimAtPath(CLEAN_ROAD_ROOT + '/Geometry/RoadSurface'))
    assert mesh.GetPointsAttr().Get()
    unrelated = tmp_path / 'other-working-directory'
    unrelated.mkdir()
    script = """
import sys
from pxr import Usd, UsdGeom, UsdShade
stage = Usd.Stage.Open(sys.argv[1])
assert stage and not stage.GetCompositionErrors()
assert str(stage.GetDefaultPrim().GetPath()) == '/World'
road = stage.GetPrimAtPath('/World/Environment/Highway')
assert road.HasAuthoredReferences() and road.IsModel()
mesh = UsdGeom.Mesh(stage.GetPrimAtPath('/World/Environment/Highway/Geometry/RoadSurface'))
assert mesh.GetPointsAttr().Get()
material, relationship = UsdShade.MaterialBindingAPI(mesh).ComputeBoundMaterial()
assert material and str(material.GetPath()) == '/World/Environment/Highway/Looks/RoadSurface'
assert stage.GetPrimAtPath('/World/Debug/Route/UpcomingPath')
assert not any(layer.anonymous for layer in stage.GetUsedLayers() if layer != stage.GetSessionLayer())
"""
    result = subprocess.run([sys.executable, '-c', script, str(scene_path)], cwd=unrelated,
                            text=True, capture_output=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    assert not result.stderr.strip(), result.stderr


@pytest.mark.parametrize('occupied', [CLEAN_ROAD_ROOT, CLEAN_DEBUG_ROOT])
def test_existing_scene_namespaces_are_not_overwritten(route, assets, occupied):
    stage = _stage()
    from pxr import UsdGeom
    UsdGeom.Xform.Define(stage, occupied)
    before = stage.GetRootLayer().ExportToString()
    with pytest.raises(ValueError, match='already exists'):
        author_physics_road(stage, route, clean_layout=True, asset_directory=assets)
    assert stage.GetRootLayer().ExportToString() == before
    assert list(assets.iterdir()) == []
    assert not (assets.parent / 'road-layout.usda').exists()


def test_existing_export_is_not_replaced(route, assets):
    first = author_physics_road(_stage(), route, clean_layout=True, asset_directory=assets)
    before = first.asset_path.read_bytes(), first.assembly_path.read_bytes()
    stage = _stage()
    root_before = stage.GetRootLayer().ExportToString()
    with pytest.raises(FileExistsError, match='already exists'):
        author_physics_road(stage, route, clean_layout=True, asset_directory=assets)
    assert (first.asset_path.read_bytes(), first.assembly_path.read_bytes()) == before
    assert stage.GetRootLayer().ExportToString() == root_before


def test_failed_debug_authoring_rolls_back_new_files_and_stage(route, assets, monkeypatch):
    stage = _stage()
    import visualization.physics_road_view as module
    before = stage.GetRootLayer().ExportToString()
    original = module._author_mesh
    def fail_debug(stage, path, *args, **kwargs):
        if path.startswith(CLEAN_DEBUG_ROOT):
            raise RuntimeError('test debug failure')
        return original(stage, path, *args, **kwargs)
    monkeypatch.setattr(module, '_author_mesh', fail_debug)
    with pytest.raises(RuntimeError, match='test debug failure'):
        module.author_physics_road(stage, route, clean_layout=True, asset_directory=assets)
    assert stage.GetRootLayer().ExportToString() == before
    assert not stage.GetPrimAtPath(CLEAN_ROAD_ROOT)
    assert not stage.GetPrimAtPath(CLEAN_DEBUG_ROOT)
    assert list(assets.iterdir()) == []
    assert not (assets.parent / 'road-layout.usda').exists()
