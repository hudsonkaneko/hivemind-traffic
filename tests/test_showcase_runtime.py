"""No Kit/GPU required: behavior guards and public extension API isolation."""
from types import SimpleNamespace
import unittest

from traffic.showcase_runtime import (
    BEHAVIOR_EXTENSION, REQUIRED_RUNTIME_EXTENSIONS,
    assert_behavior_scripting_disabled, disable_unused_behavior_scripting,
    inspect_behavior_free_stage,
)


class FakePrim:
    def __init__(self, path='/World', schemas=(), properties=()):
        self.path, self.schemas, self.properties = path, schemas, properties

    def GetPath(self): return self.path
    def GetAppliedSchemas(self): return self.schemas
    def GetProperties(self):
        return [SimpleNamespace(GetName=lambda name=name: name) for name in self.properties]


class FakeStage:
    def __init__(self, prims=None, errors=()):
        self.prims = [FakePrim()] if prims is None else prims
        self.errors = errors

    def GetCompositionErrors(self): return self.errors
    def TraverseAll(self): return iter(self.prims)


class FakeManager:
    def __init__(self, enabled=True, refuse=False, sticky=False, disable_required=False):
        self.enabled = set(REQUIRED_RUNTIME_EXTENSIONS)
        if enabled: self.enabled.add(BEHAVIOR_EXTENSION)
        self.refuse, self.sticky, self.disable_required = refuse, sticky, disable_required
        self.calls = []

    def get_extensions(self):
        return [dict(id=name+'-1.0', enabled=True) for name in self.enabled]

    def is_extension_enabled(self, name): return name in self.enabled

    def set_extension_enabled_immediate(self, name, enabled):
        self.calls.append((name, enabled))
        if self.refuse: return False
        if not self.sticky: self.enabled.discard(name)
        if self.disable_required: self.enabled.discard(REQUIRED_RUNTIME_EXTENSIONS[0])
        return True


def source_checks():
    return {name: inspect_behavior_free_stage(FakeStage()) for name in ('vehicle', 'highway')}


class ShowcaseRuntimeTests(unittest.TestCase):
    def test_clean_composed_stage_reports_every_prim(self):
        report = inspect_behavior_free_stage(FakeStage([FakePrim(), FakePrim('/World/Vehicle', ['PhysicsRigidBodyAPI'])]))
        self.assertTrue(report['passed'])
        self.assertEqual(report['prim_count'], 2)
        self.assertEqual(report['behavior_consumers'], [])

    def test_behavior_or_scripting_schema_is_rejected_even_without_properties(self):
        for schema in ('OmniScriptingAPI', 'BehaviorScriptAPI', 'SomeBehaviorAPI'):
            with self.subTest(schema=schema), self.assertRaises(ValueError):
                inspect_behavior_free_stage(FakeStage([FakePrim(schemas=[schema])]))

    def test_embedded_script_properties_are_rejected_without_applied_schema(self):
        for name in ('omni:scripting:scripts', 'omni:behavior:file'):
            with self.subTest(name=name), self.assertRaises(ValueError):
                inspect_behavior_free_stage(FakeStage([FakePrim(properties=[name])]))

    def test_absent_empty_and_unresolved_stage_cannot_qualify(self):
        for stage in (None, FakeStage([]), FakeStage(errors=['unresolved reference'])):
            with self.subTest(stage=stage), self.assertRaises(ValueError):
                inspect_behavior_free_stage(stage)

    def test_disable_uses_only_public_api_and_records_actual_state(self):
        manager = FakeManager()
        report = disable_unused_behavior_scripting(manager, source_checks())
        self.assertEqual(manager.calls, [(BEHAVIOR_EXTENSION, False)])
        self.assertTrue(report['enabled_before'])
        self.assertFalse(report['enabled_after'])
        self.assertEqual(report['disabled_extension_ids'], [BEHAVIOR_EXTENSION+'-1.0'])
        self.assertEqual(report['required_runtime_before'], report['required_runtime_after'])

    def test_already_disabled_does_not_toggle_unrelated_extensions(self):
        manager = FakeManager(enabled=False)
        report = disable_unused_behavior_scripting(manager, source_checks())
        self.assertEqual(manager.calls, [])
        self.assertFalse(report['enabled_before'])
        self.assertEqual(report['disabled_extension_ids'], [])

    def test_missing_or_false_input_usage_checks_prevent_any_disable(self):
        for checks in ({}, {'vehicle': source_checks()['vehicle']},
                       dict(vehicle={'passed': False}, highway={'passed': True}),
                       dict(vehicle=dict(passed=True,prim_count=True,behavior_consumers=[]),highway=source_checks()['highway'])):
            manager = FakeManager()
            with self.subTest(checks=checks), self.assertRaises(ValueError):
                disable_unused_behavior_scripting(manager, checks)
            self.assertEqual(manager.calls, [])

    def test_manager_failure_or_persistent_enable_fails_closed(self):
        for manager in (FakeManager(refuse=True), FakeManager(sticky=True), FakeManager(disable_required=True)):
            with self.subTest(manager=manager), self.assertRaises(RuntimeError):
                disable_unused_behavior_scripting(manager, source_checks())

    def test_later_reactivation_is_detected(self):
        manager = FakeManager(enabled=False)
        self.assertTrue(assert_behavior_scripting_disabled(manager)['passed'])
        manager.enabled.add(BEHAVIOR_EXTENSION)
        with self.assertRaises(RuntimeError): assert_behavior_scripting_disabled(manager)


if __name__ == '__main__':
    unittest.main()
