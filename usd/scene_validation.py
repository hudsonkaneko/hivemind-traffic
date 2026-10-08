"""Exact pre-simulation comparison of a Factory source and its composed scene."""


def compare_factory_contract(source, composed_stage, map_path_callable):
    """Require mapped source prims to retain their authored physical contract.

    Call after source geometry normalization and scene composition, before any
    runtime APIs, sensor state or actuator commands add opinions. Assembly
    wrappers and model metadata are intentionally outside this comparison.
    Property metadata, including physics material binding strength, is retained.
    No tolerance, schema fallback substitution or target-order relaxation is used.
    """
    from pxr import Sdf, Usd

    world = source.GetPrimAtPath('/World')
    if not world:
        raise ValueError('Factory contract source is missing /World')
    errors = []
    counts = dict(prims=0, attributes=0, relationships=0, values=0,
                  connections=0, relationship_targets=0)
    destinations = set()

    def check(actual, expected, context):
        if actual != expected:
            errors.append(f'{context}: expected {expected!r}, got {actual!r}')

    def targets(paths, owner):
        return [map_path_callable(path if path.IsAbsolutePath()
                                  else path.MakeAbsolutePath(owner)) for path in paths]

    def metadata(prop):
        # These structural/value fields are compared through composed USD APIs
        # below: their raw list-op representation can change during extraction.
        skip = {'default', 'timeSamples', 'connectionPaths', 'targetPaths',
                'typeName', 'custom', 'variability'}
        return {key: value for key, value in prop.GetAllAuthoredMetadata().items()
                if key not in skip}

    for original in Usd.PrimRange(world):
        if original == world:
            continue
        original_path = original.GetPath()
        destination = Sdf.Path(map_path_callable(original_path))
        if destination in destinations:
            errors.append(f'Nonunique mapped prim: {original_path} -> {destination}')
        destinations.add(destination)
        current = composed_stage.GetPrimAtPath(destination)
        counts['prims'] += 1
        if not current or not current.IsActive() or not current.IsLoaded():
            errors.append(f'Missing/inactive/unloaded mapped prim: {original_path} -> {destination}')
            continue
        context = str(destination)
        check(current.GetTypeName(), original.GetTypeName(), context + ' type')
        check(current.GetAppliedSchemas(), original.GetAppliedSchemas(), context + ' applied schemas')
        # PhysX stores its center-of-mass reference-frame convention here.
        check(current.GetCustomData(), original.GetCustomData(), context + ' customData')
        expected = {prop.GetName(): prop for prop in original.GetAuthoredProperties()}
        actual = {prop.GetName(): prop for prop in current.GetAuthoredProperties()}
        check(sorted(actual), sorted(expected), context + ' authored properties')
        for name, original_prop in expected.items():
            if name not in actual:
                continue
            current_prop = actual[name]
            location = context + '.' + name
            is_relationship = isinstance(original_prop, Usd.Relationship)
            if isinstance(current_prop, Usd.Relationship) != is_relationship:
                errors.append(location + ': property kind changed')
                continue
            check(current_prop.IsCustom(), original_prop.IsCustom(), location + ' custom')
            check(metadata(current_prop), metadata(original_prop), location + ' metadata')
            if is_relationship:
                counts['relationships'] += 1
                mapped = targets(original_prop.GetTargets(), original_path)
                counts['relationship_targets'] += len(mapped)
                check(current_prop.GetTargets(), mapped, location + ' targets')
                continue
            counts['attributes'] += 1
            check(current_prop.GetTypeName(), original_prop.GetTypeName(), location + ' value type')
            check(current_prop.GetVariability(), original_prop.GetVariability(), location + ' variability')
            check(current_prop.HasAuthoredValueOpinion(), original_prop.HasAuthoredValueOpinion(),
                  location + ' authored value opinion')
            check(current_prop.Get(), original_prop.Get(), location + ' default value')
            counts['values'] += 1
            original_times = original_prop.GetTimeSamples()
            check(current_prop.GetTimeSamples(), original_times, location + ' time samples')
            for time in original_times:
                check(current_prop.Get(time), original_prop.Get(time), f'{location} value at {time}')
                counts['values'] += 1
            mapped = targets(original_prop.GetConnections(), original_path)
            counts['connections'] += len(mapped)
            check(current_prop.GetConnections(), mapped, location + ' connections')
    if not counts['prims']:
        errors.append('Factory contract source has no prims below /World')
    if errors:
        details = '; '.join(errors[:20])
        if len(errors) > 20:
            details += f'; ... {len(errors) - 20} additional discrepancies'
        raise ValueError('Factory contract mismatch: ' + details)
    return dict(passed=True, comparison='exact authored properties and applied schemas', **counts)
