"""Bounded, single-stage PhysX lifecycle helpers for standalone probes.

Call after SimulationApp starts. Vehicle authoring UI is optional; the PhysX
vehicle runtime stays enabled. Never mutate installed extension implementation.
The caller must drop all vehicle/prim/stage references before ``close()``.
"""
from __future__ import annotations

import gc
import json
import math
import time

_factory = None


def prepare_vehicle_runtime(disable_authoring_tools=True):
    """Import the installed sample, then optionally release its authoring UI.

    Retain only the Factory module object, not an extension/wizard instance.
    Disabling authoring is process-local and does not edit runtime settings.
    """
    global _factory
    import omni.kit.app

    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate('omni.physx.vehicle', True)
    from omni.physxvehicle.scripts.helpers import Factory
    _factory = Factory
    if disable_authoring_tools:
        manager.set_extension_enabled_immediate('omni.physx.vehicle', False)
        gc.collect()
    if not manager.is_extension_enabled('omni.physx'):
        raise RuntimeError('Core omni.physx must remain enabled')
    authoring = manager.is_extension_enabled('omni.physx.vehicle')
    if authoring == bool(disable_authoring_tools):
        raise RuntimeError('Requested vehicle authoring-extension state was not applied')
    return dict(core_physx_enabled=True, authoring_tools_enabled=authoring,
                factory_path=Factory.__file__)


def vehicle_factory():
    """Return the imported sample without re-enabling its authoring extension."""
    global _factory
    if _factory is None:
        # Legacy callers explicitly enable the authoring extension themselves.
        # Merely requesting their Factory must not change that runtime policy.
        from omni.physxvehicle.scripts.helpers import Factory
        _factory = Factory
    return _factory


class PhysicsSession:
    """One fresh Kit stage, manually advanced at a declared fixed physics rate.

    Only one session can own the default USD context at a time. ``step`` never
    updates Kit's timeline; application updates occur only before attach or
    after physics detach, while Kit's timeline is stopped.
    """
    _active = None

    def __init__(self, physics_hz=120):
        if isinstance(physics_hz, bool) or not isinstance(physics_hz, int) or physics_hz <= 0:
            raise ValueError('physics_hz must be a positive integer')
        if PhysicsSession._active is not None:
            raise RuntimeError('Another PhysicsSession owns the default USD context')
        import omni.kit.app
        import omni.physx
        import omni.timeline
        import omni.usd
        from pxr import UsdUtils

        self.physics_hz = physics_hz
        self.app = omni.kit.app.get_app()
        self.context = omni.usd.get_context()
        self.sim = omni.physx.get_physx_simulation_interface()
        self.timeline = omni.timeline.get_timeline_interface()
        self.cache = UsdUtils.StageCache.Get()
        self.stage = None
        self.attached = False
        self.closed = False
        self.lifecycle = []
        self._last_time = None
        if self.timeline.is_playing():
            raise RuntimeError('Stop the Kit timeline before manually stepping PhysX')
        # Startup creates an empty stage. Close it before creating this owned
        # scene, without holding a Python stage object across context.close_stage.
        if self.context.get_stage() is not None:
            previous_id = self.context.get_stage_id()
            self.sim.detach_stage()
            if not self.context.close_stage():
                raise RuntimeError('Could not close previous USD context stage')
            self.app.update()
            self._erase_if_cached(previous_id)
        if not self.context.new_stage():
            raise RuntimeError('Could not create fresh USD context stage')
        # Process the stage-open lifecycle before retaining project handles or
        # attaching physics. Direct simulate/fetch_results calls do not pump
        # queued Kit/extension events. This is a bounded setup update, never an
        # extra application update inside the fixed-tick simulation loop.
        self.app.update()
        if self.timeline.is_playing():
            raise RuntimeError('Timeline unexpectedly playing after new-stage update')
        self._mark('stage_open_events_drained', stage_id=self.context.get_stage_id(), updates=1)
        self.stage = self.context.get_stage()
        self.stage_id = self.cache.GetId(self.stage).ToLongInt()
        if not self.stage_id:
            raise RuntimeError('Fresh stage is absent from USD stage cache')
        PhysicsSession._active = self
        self._mark('created', stage_id=self.stage_id, cache_count=self._cache_count())

    def _cache_count(self):
        return len(self.cache.GetAllStages())

    def _erase_if_cached(self, stage_id):
        from pxr import Usd
        cache_id = Usd.StageCache.Id.FromLongInt(stage_id)
        present = bool(self.cache.Find(cache_id))
        if present:
            self.cache.Erase(cache_id)
        return dict(old_stage_cached_before_erase=present,
                    old_stage_cached_after_erase=bool(self.cache.Find(cache_id)))

    def _mark(self, event, **values):
        row = dict(event=event, wall_monotonic_s=time.perf_counter(), **values)
        self.lifecycle.append(row)
        print('PHYSICS_LIFECYCLE='+json.dumps(row), flush=True)

    def attach(self):
        if self.closed or self.attached:
            raise RuntimeError('Attach requires a fresh unattached session')
        if not self.sim.attach_stage(self.stage_id):
            raise RuntimeError('PhysX failed to attach the owned USD stage')
        self.attached = True
        self._mark('attached', stage_id=self.stage_id)

    def step(self, dt, t):
        if not self.attached or self.closed:
            raise RuntimeError('Cannot step a detached session')
        if not math.isfinite(dt) or not math.isclose(dt, 1/self.physics_hz, rel_tol=0, abs_tol=1e-12):
            raise ValueError('dt must match the declared fixed physics rate')
        if not math.isfinite(t) or t < 0 or (self._last_time is not None and t <= self._last_time):
            raise ValueError('Simulation time must be finite and strictly increasing')
        if self.timeline.is_playing():
            raise RuntimeError('Kit timeline must stay stopped during manual stepping')
        self.sim.simulate(dt, t)
        self.sim.fetch_results()
        self._last_time = t

    def close(self):
        if self.closed:
            return
        self._mark('closing', stage_id=self.stage_id, cache_count=self._cache_count())
        # Release PhysX first, then project handles, then context, then residual
        # cache entry. Erasing a context-owned cache entry first can hang Kit.
        self.sim.detach_stage()
        self.attached = False
        self.stage = None
        gc.collect()
        if not self.context.close_stage():
            raise RuntimeError('USD context refused to close the owned stage')
        if self.timeline.is_playing():
            raise RuntimeError('Timeline unexpectedly playing during close')
        self.app.update()
        gc.collect()
        cache_state = self._erase_if_cached(self.stage_id)
        self.closed = True
        PhysicsSession._active = None
        self._mark('closed', stage_id=self.stage_id, cache_count=self._cache_count(),
                   context_has_stage=self.context.get_stage() is not None, **cache_state)
        if cache_state['old_stage_cached_after_erase']:
            raise RuntimeError('Closed stage remains in stage cache')

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
