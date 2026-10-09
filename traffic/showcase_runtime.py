"""Process-local isolation of unused embedded-USD behavior scripting.

The installed Kit behavior extension retained a USDRT stage handle and crashed
in its stage-closed callback while releasing that handle. This showcase uses
explicit Python controllers, not scripts embedded on USD prims. Reject embedded
behavior consumers first, then disable only that optional extension through the
public extension manager before creating the owned scene. No installed files or
global user preferences are changed; this is not a blanket warning suppression.

Public API: https://docs.omniverse.nvidia.com/kit/docs/kit-manual/110.1.2/
omni.ext/omni.ext.ExtensionManager.html
"""

BEHAVIOR_EXTENSION = 'omni.behavior.scripting.core'
REQUIRED_RUNTIME_EXTENSIONS = (
    'omni.physx',
    'isaacsim.core.simulation_manager',
    'isaacsim.core.rendering_manager',
)


def inspect_behavior_free_stage(stage):
    """Read a fully loaded stage, including inactive authored prims; never edit it."""
    if stage is None:
        raise ValueError('An open USD stage is required for the behavior usage guard')
    errors = list(stage.GetCompositionErrors())
    if errors:
        raise ValueError('Cannot inspect behavior usage in an unresolved stage: ' + repr(errors))
    consumers = []
    count = 0
    for prim in stage.TraverseAll():
        count += 1
        schemas = [name for name in prim.GetAppliedSchemas()
                   if 'script' in name.lower() or 'behavior' in name.lower()]
        properties = [prop.GetName() for prop in prim.GetProperties()
                      if prop.GetName().lower().startswith(('omni:scripting', 'omni:behavior'))]
        if schemas or properties:
            consumers.append(dict(path=str(prim.GetPath()), schemas=schemas, properties=properties))
    if count == 0:
        raise ValueError('Cannot qualify behavior usage from an empty stage')
    if consumers:
        raise ValueError('Showcase cannot disable embedded behavior required by this stage: ' + repr(consumers))
    return dict(passed=True, prim_count=count, behavior_consumers=[],
                scope='Loaded composed stage including inactive authored prims; no authoring performed')


def _enabled_ids(manager):
    return {entry['id'] for entry in manager.get_extensions()
            if entry.get('enabled') and isinstance(entry.get('id'), str)}


def assert_behavior_scripting_disabled(manager):
    """Fail if later extension setup silently re-enabled the isolated consumer."""
    if manager.is_extension_enabled(BEHAVIOR_EXTENSION):
        raise RuntimeError('Unused behavior scripting unexpectedly remains enabled')
    return dict(extension_id=BEHAVIOR_EXTENSION, enabled=False, passed=True)


def disable_unused_behavior_scripting(manager, source_stage_checks):
    """Disable through the public manager only after explicit input inspections.

    The required source checks cover the supplied vehicle and road separately.
    The caller checks the final composed scene again before starting physics.
    Enabled/disabled runtime IDs are retained to disclose any dependency effect.
    """
    if (not isinstance(source_stage_checks, dict)
            or set(source_stage_checks) != {'vehicle', 'highway'}
            or any(not isinstance(check, dict) or check.get('passed') is not True
                   or type(check.get('prim_count')) is not int or check['prim_count'] <= 0
                   or check.get('behavior_consumers') != []
                   for check in source_stage_checks.values())):
        raise ValueError('Passed vehicle and highway behavior-usage checks are required')
    before = _enabled_ids(manager)
    enabled_before = bool(manager.is_extension_enabled(BEHAVIOR_EXTENSION))
    required_before = {name: bool(manager.is_extension_enabled(name))
                       for name in REQUIRED_RUNTIME_EXTENSIONS}
    if enabled_before and not manager.set_extension_enabled_immediate(BEHAVIOR_EXTENSION, False):
        raise RuntimeError('The extension manager refused process-local behavior isolation')
    status = assert_behavior_scripting_disabled(manager)
    after = _enabled_ids(manager)
    required_after = {name: bool(manager.is_extension_enabled(name))
                      for name in REQUIRED_RUNTIME_EXTENSIONS}
    if any(was_enabled and not required_after[name] for name, was_enabled in required_before.items()):
        raise RuntimeError('Behavior isolation unexpectedly disabled a required physics/rendering runtime')
    return dict(passed=True, extension_id=BEHAVIOR_EXTENSION,
                enabled_before=enabled_before, enabled_after=status['enabled'],
                disabled_extension_ids=sorted(before - after),
                newly_enabled_extension_ids=sorted(after - before),
                required_runtime_before=required_before, required_runtime_after=required_after,
                source_stage_checks=source_stage_checks,
                scope='This child process only; no vendor files or persistent preference edits',
                reason='No embedded scene behaviors are used; isolate the observed USDRT stage-close consumer')
