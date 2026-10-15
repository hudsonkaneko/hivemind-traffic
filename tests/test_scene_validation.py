"""USD-only preservation checks; no Kit app, renderer or physics stepping."""
import pytest

from usd.scene_validation import compare_factory_contract


@pytest.fixture
def contract():
    pytest.importorskip('pxr.Usd')
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
    source = Usd.Stage.CreateInMemory()
    world = UsdGeom.Xform.Define(source, '/World').GetPrim()
    source.SetDefaultPrim(world)
    body = UsdGeom.Xform.Define(source, '/World/Car').GetPrim()
    body.SetCustomDataByKey('physxVehicle:referenceFrameIsCenterOfMass', False)
    UsdPhysics.RigidBodyAPI.Apply(body)
    UsdPhysics.MassAPI.Apply(body).CreateMassAttr(1800.)
    collider = UsdGeom.Cube.Define(source, '/World/Car/Collider').GetPrim()
    UsdPhysics.CollisionAPI.Apply(collider)
    relation = collider.CreateRelationship('material:binding:physics', custom=False)
    relation.SetTargets(['/World/Material'])
    relation.SetMetadata('bindMaterialAs', 'weakerThanDescendants')
    material = source.DefinePrim('/World/Material', 'Material')
    output = material.CreateAttribute('outputs:value', Sdf.ValueTypeNames.Float, custom=False)
    output.Set(.75)
    friction = body.CreateAttribute('test:friction', Sdf.ValueTypeNames.Float, custom=False)
    friction.SetConnections(['/World/Material.outputs:value'])
    sampled = body.CreateAttribute('test:sampled', Sdf.ValueTypeNames.Float3, custom=False)
    sampled.Set(Gf.Vec3f(1, 2, 3), 1.)
    sampled.Set(Gf.Vec3f(4, 5, 6), 2.)
    composed = Usd.Stage.CreateInMemory()
    assert Sdf.CopySpec(source.GetRootLayer(), '/World', composed.GetRootLayer(), '/Assembly')
    mapped = lambda path: path.ReplacePrefix(Sdf.Path('/World'), Sdf.Path('/Assembly'))
    return source, composed, mapped


def test_normalized_contract_passes_and_counts_values_and_bindings(contract):
    report = compare_factory_contract(*contract)
    assert report['passed']
    assert report['prims'] == 3
    assert report['relationships'] == report['relationship_targets'] == 1
    assert report['connections'] == 1
    assert report['values'] > report['attributes']


@pytest.mark.parametrize('mutation,match', [
    ('mass', 'default value'), ('collision_schema', 'applied schemas'),
    ('missing_property', 'authored properties'), ('extra_property', 'authored properties'),
    ('relationship', 'targets'), ('connection', 'connections'),
    ('sample', 'value at 2.0'), ('sample_removed', 'time samples'),
    ('binding_strength', 'metadata'), ('custom_data', 'customData'),
    ('type', ' type'), ('inactive', 'inactive'),
])
def test_rejects_loss_or_change_even_when_target_prims_still_exist(contract, mutation, match):
    from pxr import Gf, Sdf, UsdPhysics
    source, composed, mapped = contract
    body = composed.GetPrimAtPath('/Assembly/Car')
    collider = composed.GetPrimAtPath('/Assembly/Car/Collider')
    if mutation == 'mass':
        body.GetAttribute('physics:mass').Set(1700.)
    elif mutation == 'collision_schema':
        collider.RemoveAPI(UsdPhysics.CollisionAPI)
    elif mutation == 'missing_property':
        body.RemoveProperty('physics:mass')
    elif mutation == 'extra_property':
        body.CreateAttribute('test:extra', Sdf.ValueTypeNames.Float).Set(1.)
    elif mutation == 'relationship':
        collider.GetRelationship('material:binding:physics').SetTargets(['/Assembly/Car'])
    elif mutation == 'connection':
        body.GetAttribute('test:friction').SetConnections(['/Assembly/Car.physics:mass'])
    elif mutation == 'sample':
        body.GetAttribute('test:sampled').Set(Gf.Vec3f(4, 5, 7), 2.)
    elif mutation == 'sample_removed':
        body.GetAttribute('test:sampled').ClearAtTime(2.)
    elif mutation == 'binding_strength':
        collider.GetRelationship('material:binding:physics').ClearMetadata('bindMaterialAs')
    elif mutation == 'custom_data':
        body.SetCustomDataByKey('physxVehicle:referenceFrameIsCenterOfMass', True)
    elif mutation == 'type':
        collider.SetTypeName('Sphere')
    else:
        collider.SetActive(False)
    with pytest.raises(ValueError, match=match):
        compare_factory_contract(source, composed, mapped)


def test_composition_can_split_properties_across_layers(contract):
    from pxr import Sdf, Usd
    source, composed, mapped = contract
    physics = Sdf.Layer.CreateAnonymous('physics.usda')
    composed.GetRootLayer().subLayerPaths.append(physics.identifier)
    path = '/Assembly/Car.physics:mass'
    physics_stage = Usd.Stage.Open(physics)
    physics_stage.OverridePrim('/Assembly/Car')
    assert Sdf.CopySpec(composed.GetRootLayer(), path, physics, path)
    composed.GetPrimAtPath('/Assembly/Car').RemoveProperty('physics:mass')
    assert compare_factory_contract(source, composed, mapped)['passed']


def test_source_must_have_a_nonempty_world():
    pytest.importorskip('pxr.Usd')
    from pxr import Usd, UsdGeom
    source, composed = Usd.Stage.CreateInMemory(), Usd.Stage.CreateInMemory()
    with pytest.raises(ValueError, match='missing /World'):
        compare_factory_contract(source, composed, lambda path:path)
    UsdGeom.Xform.Define(source, '/World')
    with pytest.raises(ValueError, match='no prims below'):
        compare_factory_contract(source, composed, lambda path:path)
